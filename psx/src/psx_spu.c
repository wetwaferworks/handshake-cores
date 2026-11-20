/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_spu.c - PS1 SPU, split across the handshake:
 *   Phase 0: CPU-visible register file, 512 KiB sound RAM uploads (manual
 *            FIFO + DMA4), and a time-stamped register-write event log in the
 *            BACK slot (sample index = frame cycle / 768).
 *   Phase 2: 24-voice synthesis over the const FRONT slot: 4-bit ADPCM
 *            (5 filters, shift, loop-start/end/repeat flags), 4-tap Gaussian
 *            or linear interpolation, ADSR envelopes (attack/decay/sustain/
 *            release, linear+exponential), noise, pitch modulation, voice +
 *            master volume, 44.1 kHz stereo int16; plus a deterministic
 *            735 -> 800 linear resampler for the 48 kHz hs_core ABI.
 * Not modelled: reverb, CD/external audio inputs, volume sweeps (their target
 * level is used), SPU IRQ, CPU-visible ENDX/ADSR readback (Phase 0 cannot see
 * Phase 2; reads return the last written value / 0).
 */
#include "psx_internal.h"
#include "psx_spu_gauss.h"

#include <string.h>

#define REG_MVOL_L 0xC0u
#define REG_MVOL_R 0xC1u
#define REG_KON_LO 0xC4u
#define REG_KON_HI 0xC5u
#define REG_KOFF_LO 0xC6u
#define REG_KOFF_HI 0xC7u
#define REG_PMON_LO 0xC8u
#define REG_NON_LO 0xCAu
#define REG_XFER_ADDR 0xD3u
#define REG_FIFO 0xD4u
#define REG_SPUCNT 0xD5u
#define REG_SPUSTAT 0xD7u

/* ======================= Phase 0 side ======================= */
static void ram_put16(hs_psx_t *s, uint32_t v)
{
    uint32_t a = s->spu_xfer_addr & (HS_PSX_SPU_RAM - 2u);
    s->spu_ram0[a] = (uint8_t)v;
    s->spu_ram0[a + 1u] = (uint8_t)(v >> 8);
    s->spu_stale[0][a / PSX_SPU_BLK] = 1;
    s->spu_stale[1][a / PSX_SPU_BLK] = 1;
    s->spu_xfer_addr = (a + 2u) & (HS_PSX_SPU_RAM - 1u);
}

static void log_event(hs_psx_t *s, uint32_t reg, uint32_t v)
{
    psx_slot_t *b = psx_back(s);
    uint32_t smp = s->frame_cyc / 768u;
    if (smp >= HS_PSX_SPU_PER_FRAME) smp = HS_PSX_SPU_PER_FRAME - 1u;
    if (b->n_ev >= PSX_SPU_EV_MAX) { b->ev_dropped++; return; }
    b->spu_ev[b->n_ev].sample = (uint16_t)smp;
    b->spu_ev[b->n_ev].reg = (uint16_t)reg;
    b->spu_ev[b->n_ev].val = (uint16_t)v;
    b->spu_ev[b->n_ev]._pad = 0;
    b->n_ev++;
}

void psx_spu_write(hs_psx_t *s, uint32_t off, uint32_t v)
{
    uint32_t r = off >> 1;
    v &= 0xFFFFu;
    if (r >= 256u) return;
    if (r == REG_FIFO) { ram_put16(s, v); return; }
    s->spu_reg[r] = (uint16_t)v;
    if (r == REG_XFER_ADDR) s->spu_xfer_addr = (v * 8u) & (HS_PSX_SPU_RAM - 1u);
    if (r == REG_SPUCNT) s->spu_reg[REG_SPUSTAT] = (uint16_t)((s->spu_reg[REG_SPUSTAT] & ~0x3Fu) | (v & 0x3Fu));
    log_event(s, r, v);
}

uint32_t psx_spu_read(hs_psx_t *s, uint32_t off)
{
    uint32_t r = off >> 1;
    if (r >= 256u) return 0;
    if (r == REG_SPUSTAT) return (uint32_t)(s->spu_reg[REG_SPUCNT] & 0x3Fu);
    if (r == 0xCEu || r == 0xCFu) return 0;          /* ENDX: not observable from Phase 0 */
    return s->spu_reg[r];
}

void psx_spu_dma_write(hs_psx_t *s, uint32_t w)
{
    ram_put16(s, w & 0xFFFFu);
    ram_put16(s, w >> 16);
}

uint32_t psx_spu_dma_read(hs_psx_t *s)
{
    uint32_t a = s->spu_xfer_addr & (HS_PSX_SPU_RAM - 4u);
    uint32_t v = (uint32_t)s->spu_ram0[a] | ((uint32_t)s->spu_ram0[a + 1] << 8) |
                 ((uint32_t)s->spu_ram0[a + 2] << 16) | ((uint32_t)s->spu_ram0[a + 3] << 24);
    s->spu_xfer_addr = (a + 4u) & (HS_PSX_SPU_RAM - 1u);
    return v;
}

void psx_spu_frame_begin(hs_psx_t *s)
{
    (void)s;                    /* the event log is emptied by commit_phase */
}

/* ======================= Phase 2 side ======================= */
void psx_spu_audio_reset(psx_audio_priv_t *a)
{
    memset(a, 0, sizeof(*a));
    a->noise_lvl = 1;
}

static const int32_t k_pos[5] = { 0, 60, 115, 98, 122 };
static const int32_t k_neg[5] = { 0, 0, -52, -55, -60 };

static void decode_block(psx_voice_t *v, const uint8_t *ram)
{
    const uint8_t *blk = ram + (v->addr & (HS_PSX_SPU_RAM - 16u));
    uint32_t shift = blk[0] & 0xFu, filt = (blk[0] >> 4) & 7u, i;
    if (shift > 12u) shift = 9u;
    if (filt > 4u) filt = 4u;
    v->flags = blk[1];
    for (i = 0; i < 28u; i++) {
        uint32_t nib = (blk[2u + (i >> 1)] >> ((i & 1u) * 4u)) & 0xFu;
        int32_t smp = (int32_t)(int16_t)(uint16_t)(nib << 12) >> shift;
        smp += (v->old1 * k_pos[filt] + v->old2 * k_neg[filt] + 32) >> 6;
        if (smp > 32767) smp = 32767;
        if (smp < -32768) smp = -32768;
        v->old2 = v->old1;
        v->old1 = (int16_t)smp;
        v->buf[i] = (int16_t)smp;
    }
    if (v->flags & 4u)
        v->repeat = v->addr & (HS_PSX_SPU_RAM - 16u);
}

static void key_on(psx_audio_priv_t *a, uint32_t n, const uint8_t *ram)
{
    psx_voice_t *v = &a->v[n];
    const uint16_t *r = &a->reg[n * 8u];
    v->addr = ((uint32_t)r[3] * 8u) & (HS_PSX_SPU_RAM - 16u);
    v->repeat = ((uint32_t)r[7] * 8u) & (HS_PSX_SPU_RAM - 16u);
    v->counter = 0;
    v->old1 = v->old2 = 0;
    v->hist[0] = v->hist[1] = v->hist[2] = v->hist[3] = 0;
    v->env = 0;
    v->env_phase = 1;
    v->env_cnt = 0;
    v->on = 1;
    a->endx &= ~(1u << n);
    decode_block(v, ram);
}

static void env_step(psx_voice_t *v, int ex, int dec, int32_t shift, int32_t step)
{
    uint32_t cycles = 1u << (shift > 11 ? shift - 11 : 0);
    int32_t adj = step * (1 << (shift < 11 ? 11 - shift : 0));
    if (ex && !dec && v->env > 0x6000) cycles *= 4u;
    if (ex && dec) adj = (adj * v->env) >> 15;
    if (++v->env_cnt < cycles) return;
    v->env_cnt = 0;
    v->env += adj;
    if (v->env < 0) v->env = 0;
    if (v->env > 0x7FFF) v->env = 0x7FFF;
}

static void env_tick(psx_voice_t *v, const uint16_t *r)
{
    uint32_t a1 = r[4], a2 = r[5];
    switch (v->env_phase) {
    case 1:
        env_step(v, (a1 >> 15) & 1u, 0, (int32_t)((a1 >> 10) & 0x1Fu), 7 - (int32_t)((a1 >> 8) & 3u));
        if (v->env >= 0x7FFF) { v->env_phase = 2; v->env_cnt = 0; }
        break;
    case 2: {
        int32_t sl = (int32_t)(((a1 & 0xFu) + 1u) * 0x800u);
        if (sl > 0x7FFF) sl = 0x7FFF;
        env_step(v, 1, 1, (int32_t)((a1 >> 4) & 0xFu), -8);
        if (v->env <= sl) { v->env_phase = 3; v->env_cnt = 0; }
        } break;
    case 3: {
        int dec = (a2 >> 14) & 1u;
        int32_t st = (int32_t)((a2 >> 6) & 3u);
        env_step(v, (a2 >> 15) & 1u, dec, (int32_t)((a2 >> 8) & 0x1Fu), dec ? (-8 + st) : (7 - st));
        } break;
    case 4:
        env_step(v, (a2 >> 5) & 1u, 1, (int32_t)(a2 & 0x1Fu), -8);
        if (v->env <= 0) { v->env = 0; v->on = 0; v->env_phase = 0; }
        break;
    default: break;
    }
}

static inline int32_t vol15(uint16_t r)
{
    if (r & 0x8000u) return 0x7FFF;            /* sweep mode: target level */
    return (int32_t)(int16_t)(uint16_t)(r << 1);
}

static inline int32_t voice_sample(const psx_voice_t *v, int32_t k)  /* k in -3..27 */
{
    return k < 0 ? v->hist[4 + k] : v->buf[k];
}

static void apply_event(psx_audio_priv_t *a, const psx_spu_ev_t *e, const uint8_t *ram)
{
    uint32_t r = e->reg, v = e->val, n;
    a->reg[r] = (uint16_t)v;
    if (r == REG_KON_LO || r == REG_KON_HI) {
        uint32_t base = (r == REG_KON_HI) ? 16u : 0u;
        for (n = 0; n < 16u && base + n < 24u; n++)
            if (v & (1u << n)) key_on(a, base + n, ram);
    } else if (r == REG_KOFF_LO || r == REG_KOFF_HI) {
        uint32_t base = (r == REG_KOFF_HI) ? 16u : 0u;
        for (n = 0; n < 16u && base + n < 24u; n++)
            if ((v & (1u << n)) && a->v[base + n].on) { a->v[base + n].env_phase = 4; a->v[base + n].env_cnt = 0; }
    }
}

void psx_spu_synth_frame(hs_psx_t *s)
{
    const psx_slot_t *f = psx_front(s);
    psx_audio_priv_t *a = &s->aud;
    const uint8_t *ram = f->spu_ram;
    uint32_t i, n, ev = 0;
    for (i = 0; i < HS_PSX_SPU_PER_FRAME; i++) {
        int32_t L = 0, Rr = 0, prev_out = 0;
        uint32_t pmon = (uint32_t)a->reg[REG_PMON_LO] | ((uint32_t)a->reg[REG_PMON_LO + 1u] << 16);
        uint32_t non = (uint32_t)a->reg[REG_NON_LO] | ((uint32_t)a->reg[REG_NON_LO + 1u] << 16);
        uint32_t cnt = a->reg[REG_SPUCNT];
        while (ev < f->n_ev && f->spu_ev[ev].sample <= i) { apply_event(a, &f->spu_ev[ev], ram); ev++; }
        /* noise generator (deterministic LFSR, rate from SPUCNT bits 8..13) */
        {
            uint32_t nshift = (cnt >> 10) & 0xFu, nstep = ((cnt >> 8) & 3u) + 4u;
            a->noise_cnt += nstep;
            if (a->noise_cnt >= (0x20000u >> nshift)) {
                uint32_t lv = a->noise_lvl & 0xFFFFu;
                uint32_t parity = ((lv >> 15) ^ (lv >> 12) ^ (lv >> 11) ^ (lv >> 10) ^ 1u) & 1u;
                a->noise_cnt = 0;
                a->noise_lvl = ((lv << 1) | parity) & 0xFFFFu;
            }
        }
        for (n = 0; n < 24u; n++) {
            psx_voice_t *v = &a->v[n];
            const uint16_t *r = &a->reg[n * 8u];
            int32_t smp, idx;
            uint32_t step, ph;
            if (!v->on) { v->last_out = 0; prev_out = 0; continue; }
            step = r[2];
            if (step > 0x3FFFu) step = 0x3FFFu;
            if (n > 0u && (pmon & (1u << n))) {
                int32_t f2 = 32768 + prev_out;
                step = (uint32_t)(((int64_t)step * f2) >> 15);
                if (step > 0x3FFFu) step = 0x3FFFu;
            }
            idx = (int32_t)(v->counter >> 12);
            ph = (v->counter >> 4) & 0xFFu;
            if (non & (1u << n)) {
                smp = (int32_t)(int16_t)(uint16_t)a->noise_lvl;
            } else {
                smp = (k_spu_gauss[ph][0] * voice_sample(v, idx - 3) + k_spu_gauss[ph][1] * voice_sample(v, idx - 2) +
                       k_spu_gauss[ph][2] * voice_sample(v, idx - 1) + k_spu_gauss[ph][3] * voice_sample(v, idx)) >> 15;
            }
            env_tick(v, r);
            smp = (smp * v->env) >> 15;
            v->last_out = smp;
            prev_out = smp;
            L += (smp * vol15(r[0])) >> 15;
            Rr += (smp * vol15(r[1])) >> 15;
            v->counter += step;
            if (v->counter >= (28u << 12)) {   /* at most one block per sample (step <= 0x3FFF) */
                v->counter -= 28u << 12;
                v->hist[0] = v->buf[24]; v->hist[1] = v->buf[25]; v->hist[2] = v->buf[26]; v->hist[3] = v->buf[27];
                if (v->flags & 1u) {
                    a->endx |= 1u << n;
                    if (v->flags & 2u) v->addr = v->repeat;
                    else { v->on = 0; v->env = 0; v->env_phase = 0; v->addr += 16u; }
                } else {
                    v->addr = (v->addr + 16u) & (HS_PSX_SPU_RAM - 16u);
                }
                if (v->on) decode_block(v, ram);
            }
        }
        if (!(cnt & 0x8000u) || !(cnt & 0x4000u)) { L = 0; Rr = 0; }
        L = (L * vol15(a->reg[REG_MVOL_L])) >> 15;
        Rr = (Rr * vol15(a->reg[REG_MVOL_R])) >> 15;
        if (L > 32767) L = 32767;
        if (L < -32768) L = -32768;
        if (Rr > 32767) Rr = 32767;
        if (Rr < -32768) Rr = -32768;
        a->native[2u * i] = (int16_t)L;
        a->native[2u * i + 1u] = (int16_t)Rr;
    }
    while (ev < f->n_ev) { apply_event(a, &f->spu_ev[ev], ram); ev++; }
    a->frames++;
}
