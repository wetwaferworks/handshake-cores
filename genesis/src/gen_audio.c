/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_audio.c - hs-genesis sound phase (Core 2 domain). Owns the Z80, its
 * 8 KB RAM, the YM2612 and the SN76489. Reads ONLY the committed FRONT
 * audio snapshot: the frame's timestamped 68k-side events (YM/PSG writes,
 * Z80 RAM uploads, BUSREQ/RESET edges, V-blank Z80 IRQ). The Z80 runs in
 * lockstep with the 48 kHz output clock, events are replayed at their exact
 * Z80-clock position, YM2612 runs at its native fM/144 rate and the PSG at
 * clock/16, both box-filtered down to 48 kHz with exact integer
 * accumulators. Deterministic, integer only.
 */
#include "gen_internal.h"
#include <string.h>

#define YM_DEN  (7u * 144u * 48000u)
#define PSG_DEN (15u * 16u * 48000u)

static uint8_t zrd(void *ctx, uint16_t a)
{
    gen_aud_domain_t *u = (gen_aud_domain_t *)ctx;
    uint32_t a68;
    if (a < 0x4000) return u->zram[a & 0x1FFF];
    if (a < 0x6000) return ym_read_status(&u->ym);
    if (a < 0x8000) return 0xFF;
    a68 = ((uint32_t)u->bank << 15) | (a & 0x7FFFu);
    if (a68 < 0x400000u && a68 < u->rom_size) return u->rom[a68];
    return 0xFF;
}

static void zwr(void *ctx, uint16_t a, uint8_t v)
{
    gen_aud_domain_t *u = (gen_aud_domain_t *)ctx;
    if (a < 0x4000) { u->zram[a & 0x1FFF] = v; return; }
    if (a < 0x6000) { ym_write(&u->ym, (uint8_t)(a & 3), v); return; }
    if (a < 0x6100) { u->bank = (uint16_t)(((u->bank >> 1) | ((v & 1u) << 8)) & 0x1FF); return; }
    if (a >= 0x7F10 && a < 0x7F18 && (a & 1)) { psg_write(&u->psg, v); return; }
}

static uint8_t zin(void *ctx, uint16_t p) { (void)ctx; (void)p; return 0xFF; }
static void zout(void *ctx, uint16_t p, uint8_t v) { (void)ctx; (void)p; (void)v; }

void gen_audio_reset(gen_aud_domain_t *a)
{
    uint64_t cyc = a->z80.cyc;
    z80_reset(&a->z80);
    a->z80.cyc = cyc;
    a->z80.irq_line = 0;
    ym_reset(&a->ym);
    psg_reset(&a->psg);
    memset(a->zram, 0, sizeof(a->zram));
    a->bank = 0;
    a->busreq = 0;
    a->reset = 1;
    a->zirq = 0;
    a->ym_acc = a->psg_acc = a->smp_acc = 0;
    a->ym_l = a->ym_r = 0;
    a->peak_ym = a->peak_psg = 0;
}

void gen_audio_init(gen_aud_domain_t *a)
{
    z80_bus_t b;
    const uint8_t *rom = a->rom;
    uint32_t rs = a->rom_size;
    memset(a, 0, sizeof(*a));
    a->rom = rom;
    a->rom_size = rs;
    b.read = zrd; b.write = zwr; b.in = zin; b.out = zout; b.ctx = a;
    z80_init(&a->z80, &b);
    gen_audio_reset(a);
}

static void run_z80(gen_aud_domain_t *a, uint64_t target)
{
    if (!a->busreq && !a->reset) z80_run_until(&a->z80, target);
    else if (a->z80.cyc < target) a->z80.cyc = target;
}

static void apply_event(gen_aud_domain_t *a, const gen_audio_ev_t *e)
{
    switch (e->kind) {
    case GEV_YM:     ym_write(&a->ym, e->a, (uint8_t)e->v); break;
    case GEV_PSG:    psg_write(&a->psg, (uint8_t)e->v); break;
    case GEV_ZRAM:   a->zram[e->v & 0x1FFF] = e->a; break;
    case GEV_BUSREQ: a->busreq = e->a; break;
    case GEV_RESET:
        if (e->a && !a->reset) {
            uint64_t cyc = a->z80.cyc;
            z80_reset(&a->z80);
            a->z80.cyc = cyc;
            ym_reset(&a->ym);
        }
        a->reset = e->a;
        break;
    case GEV_ZIRQ:   a->zirq = e->a; a->z80.irq_line = e->a; break;
    default: break;
    }
}

uint32_t gen_audio_frame(gen_aud_domain_t *a, const gen_audio_snap_t *s,
                         int16_t *pcm, uint32_t max_frames)
{
    uint32_t lines = s->pal ? GEN_LINES_PAL : GEN_LINES_NTSC;
    uint64_t clk = s->pal ? GEN_MASTER_PAL : GEN_MASTER_NTSC;
    uint64_t master_frame = (uint64_t)GEN_MASTER_LINE * lines;
    uint64_t num = a->smp_acc + 48000ull * master_frame;
    uint32_t n = (uint32_t)(num / clk), i, e = 0;
    uint64_t zf = s->frame_z80, base = a->z80_base;
    uint32_t nev = s->n_ev < GEN_MAX_EVENTS ? s->n_ev : GEN_MAX_EVENTS;
    a->smp_acc = num % clk;
    if (n == 0) n = 1;

    for (i = 0; i < n; i++) {
        uint64_t tgt = base + zf * (i + 1) / n;
        int32_t yl = 0, yr = 0, ps = 0, l, r;
        uint32_t ycnt = 0, pcnt = 0;
        while (e < nev) {
            uint64_t zt = base + (uint64_t)s->ev[e].ts * 7u / 15u;
            if (zt > tgt) break;
            run_z80(a, zt);
            apply_event(a, &s->ev[e]);
            e++;
        }
        run_z80(a, tgt);

        a->ym_acc += clk;
        while (a->ym_acc >= YM_DEN && ycnt < 4) {
            int32_t ll, rr;
            a->ym_acc -= YM_DEN;
            ym_clock(&a->ym, &ll, &rr);
            yl += ll; yr += rr; ycnt++;
        }
        if (ycnt) { a->ym_l = yl / (int32_t)ycnt; a->ym_r = yr / (int32_t)ycnt; }

        a->psg_acc += clk;
        while (a->psg_acc >= PSG_DEN && pcnt < 8) {
            a->psg_acc -= PSG_DEN;
            ps += psg_tick(&a->psg);
            pcnt++;
        }
        if (pcnt) ps /= (int32_t)pcnt;

        if ((a->ym_l < 0 ? -a->ym_l : a->ym_l) > a->peak_ym) a->peak_ym = a->ym_l < 0 ? -a->ym_l : a->ym_l;
        if ((ps < 0 ? -ps : ps) > a->peak_psg) a->peak_psg = ps < 0 ? -ps : ps;

        l = a->ym_l / 2 + ps;
        r = a->ym_r / 2 + ps;
        if (l > 32767) l = 32767;
        if (l < -32768) l = -32768;
        if (r > 32767) r = 32767;
        if (r < -32768) r = -32768;
        if (i < max_frames && pcm) {
            pcm[2 * i] = (int16_t)l;
            pcm[2 * i + 1] = (int16_t)r;
        }
    }
    while (e < nev) { apply_event(a, &s->ev[e]); e++; }
    a->z80_base = base + zf;
    if (a->z80.cyc < a->z80_base) a->z80.cyc = a->z80_base;
    a->frames++;
    a->samples_out += n;
    return n < max_frames ? n : max_frames;
}
