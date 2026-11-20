/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_apu.c - clean-room 4-channel Game Boy APU.
 *   CH1 pulse + sweep, CH2 pulse, CH3 32-step wave RAM, CH4 15/7-bit LFSR
 *   noise, 512 Hz frame sequencer (length / sweep / envelope), NR50/NR51
 *   stereo panning + master volume, integer DC-blocking high-pass filter.
 *
 * The same code runs in two places:
 *   - Core 0 (state only, out == NULL) so the CPU can read NR52 / lengths,
 *   - Core 2 (synth) replaying the frame's register-write events from the
 *     committed audio snapshot, producing 48 kHz stereo int16 PCM.
 * Both paths execute bit-identical integer arithmetic, so the synth state
 * after a frame equals the CPU-side state (checked by the test suite).
 */
#include "gbc_internal.h"

#include <string.h>

static const uint8_t duty_tbl[4] = { 0x01u, 0x81u, 0x87u, 0x7Eu };  /* 12.5/25/50/75 % */

static const uint8_t read_or[0x30] = {
    0x80, 0x3F, 0x00, 0xFF, 0xBF,  0xFF, 0x3F, 0x00, 0xFF, 0xBF,
    0x7F, 0xFF, 0x9F, 0xFF, 0xBF,  0xFF, 0xFF, 0x00, 0x00, 0xBF,
    0x00, 0x00, 0x70, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

void gbc_apu_reset(gbc_apu_t *a, int cgb)
{
    (void)cgb;
    memset(a, 0, sizeof(*a));
    a->fs_timer = 8192u;
    a->ch4.lfsr = 0x7FFFu;
    /* post-boot state: sound on, master volume max, all panned */
    a->power = 1;
    a->nr50 = 0x77; a->nr51 = 0xF3;
    a->regs[0x14] = 0x77; a->regs[0x15] = 0xF3; a->regs[0x16] = 0x80;
    a->regs[0x00] = 0x80; a->regs[0x01] = 0xBF; a->regs[0x02] = 0xF3;
    a->regs[0x04] = 0xBF; a->regs[0x06] = 0x3F; a->regs[0x09] = 0xBF;
    a->regs[0x0A] = 0x7F; a->regs[0x0B] = 0xFF; a->regs[0x0C] = 0x9F;
    a->regs[0x0E] = 0xBF; a->regs[0x10] = 0xFF; a->regs[0x13] = 0xBF;
    a->ch1.duty = 2;
    a->ch1.env_init = 0xF; a->ch1.env_period = 3; a->ch1.dac = 1;
}

uint8_t gbc_apu_read(const gbc_apu_t *a, uint32_t reg)
{
    if (reg >= 0x30u)
        return 0xFF;
    if (reg >= 0x20u)
        return a->ch3.ram[reg - 0x20u];
    if (reg == 0x16u) {
        return (uint8_t)((a->power ? 0x80u : 0u) | 0x70u |
                         (a->ch1.on ? 1u : 0u) | (a->ch2.on ? 2u : 0u) |
                         (a->ch3.on ? 4u : 0u) | (a->ch4.on ? 8u : 0u));
    }
    return (uint8_t)(a->regs[reg] | read_or[reg]);
}

static uint32_t noise_period(const gbc_noise_t *n)
{
    uint32_t d = n->div_code ? (uint32_t)n->div_code * 16u : 8u;
    return d << n->shift;
}

static uint16_t sweep_calc(gbc_square_t *c)
{
    uint16_t d = (uint16_t)(c->sw_shadow >> c->sw_shift);
    uint16_t nf;
    if (c->sw_neg) {
        nf = (uint16_t)(c->sw_shadow - d);
        c->sw_neg_used = 1;
    } else {
        nf = (uint16_t)(c->sw_shadow + d);
    }
    if (nf > 2047u)
        c->on = 0;
    return nf;
}

static void square_write(gbc_square_t *c, uint32_t r, uint8_t v, int has_sweep)
{
    switch (r) {
    case 0:
        if (has_sweep) {
            c->sw_period = (uint8_t)((v >> 4) & 7u);
            c->sw_shift = (uint8_t)(v & 7u);
            if (c->sw_neg && !(v & 0x08u) && c->sw_neg_used)
                c->on = 0;
            c->sw_neg = (uint8_t)((v >> 3) & 1u);
        }
        break;
    case 1:
        c->duty = (uint8_t)(v >> 6);
        c->len = (uint16_t)(64u - (v & 63u));
        break;
    case 2:
        c->env_init = (uint8_t)(v >> 4);
        c->env_dir = (uint8_t)((v >> 3) & 1u);
        c->env_period = (uint8_t)(v & 7u);
        c->dac = (uint8_t)((v & 0xF8u) != 0);
        if (!c->dac) c->on = 0;
        break;
    case 3:
        c->freq = (uint16_t)((c->freq & 0x700u) | v);
        break;
    case 4:
        c->freq = (uint16_t)((c->freq & 0xFFu) | ((uint16_t)(v & 7u) << 8));
        c->len_en = (uint8_t)((v >> 6) & 1u);
        if (v & 0x80u) {
            c->on = c->dac;
            if (c->len == 0) c->len = 64;
            c->timer = (int32_t)((2048u - c->freq) * 4u);
            c->vol = c->env_init;
            c->env_timer = c->env_period ? c->env_period : 8u;
            if (has_sweep) {
                c->sw_shadow = c->freq;
                c->sw_timer = c->sw_period ? c->sw_period : 8u;
                c->sw_en = (uint8_t)(c->sw_period || c->sw_shift);
                c->sw_neg_used = 0;
                if (c->sw_shift)
                    (void)sweep_calc(c);
            }
        }
        break;
    default:
        break;
    }
}

void gbc_apu_write(gbc_apu_t *a, uint32_t reg, uint8_t v)
{
    if (reg >= 0x30u)
        return;
    if (reg >= 0x20u) {
        a->ch3.ram[reg - 0x20u] = v;
        a->regs[reg] = v;
        return;
    }
    if (reg == 0x16u) {
        uint8_t on = (uint8_t)(v >> 7);
        if (a->power && !on) {
            uint8_t ram[16];
            memcpy(ram, a->ch3.ram, 16);
            memset(a->regs, 0, 0x20);
            memset(&a->ch1, 0, sizeof(a->ch1));
            memset(&a->ch2, 0, sizeof(a->ch2));
            memset(&a->ch3, 0, sizeof(a->ch3));
            memset(&a->ch4, 0, sizeof(a->ch4));
            memcpy(a->ch3.ram, ram, 16);
            a->ch4.lfsr = 0x7FFFu;
            a->nr50 = 0; a->nr51 = 0;
        } else if (!a->power && on) {
            a->fs_step = 0;
            a->fs_timer = 8192u;
        }
        a->power = on;
        a->regs[0x16] = (uint8_t)(v & 0x80u);
        return;
    }
    if (!a->power)
        return;
    a->regs[reg] = v;
    if (reg <= 0x04u) {
        square_write(&a->ch1, reg, v, 1);
    } else if (reg >= 0x05u && reg <= 0x09u) {
        if (reg != 0x05u) square_write(&a->ch2, reg - 0x05u, v, 0);
    } else if (reg >= 0x0Au && reg <= 0x0Eu) {
        gbc_wave_t *w = &a->ch3;
        switch (reg) {
        case 0x0A: w->dac = (uint8_t)(v >> 7); if (!w->dac) w->on = 0; break;
        case 0x0B: w->len = (uint16_t)(256u - v); break;
        case 0x0C: w->vol_code = (uint8_t)((v >> 5) & 3u); break;
        case 0x0D: w->freq = (uint16_t)((w->freq & 0x700u) | v); break;
        default:
            w->freq = (uint16_t)((w->freq & 0xFFu) | ((uint16_t)(v & 7u) << 8));
            w->len_en = (uint8_t)((v >> 6) & 1u);
            if (v & 0x80u) {
                w->on = w->dac;
                if (w->len == 0) w->len = 256;
                w->timer = (int32_t)((2048u - w->freq) * 2u);
                w->pos = 0;
            }
            break;
        }
    } else if (reg >= 0x10u && reg <= 0x13u) {
        gbc_noise_t *n = &a->ch4;
        switch (reg) {
        case 0x10: n->len = (uint16_t)(64u - (v & 63u)); break;
        case 0x11:
            n->env_init = (uint8_t)(v >> 4);
            n->env_dir = (uint8_t)((v >> 3) & 1u);
            n->env_period = (uint8_t)(v & 7u);
            n->dac = (uint8_t)((v & 0xF8u) != 0);
            if (!n->dac) n->on = 0;
            break;
        case 0x12:
            n->shift = (uint8_t)(v >> 4);
            n->width7 = (uint8_t)((v >> 3) & 1u);
            n->div_code = (uint8_t)(v & 7u);
            break;
        default:
            n->len_en = (uint8_t)((v >> 6) & 1u);
            if (v & 0x80u) {
                n->on = n->dac;
                if (n->len == 0) n->len = 64;
                n->timer = (int32_t)noise_period(n);
                n->lfsr = 0x7FFFu;
                n->vol = n->env_init;
                n->env_timer = n->env_period ? n->env_period : 8u;
            }
            break;
        }
    } else if (reg == 0x14u) {
        a->nr50 = v;
    } else if (reg == 0x15u) {
        a->nr51 = v;
    }
}

/* ---- frame sequencer -------------------------------------------------- */
static void len_clock(uint8_t *on, uint8_t len_en, uint16_t *len)
{
    if (len_en && *len) {
        (*len)--;
        if (*len == 0)
            *on = 0;
    }
}

static void env_clock(uint8_t *vol, uint8_t dir, uint8_t period, uint8_t *timer)
{
    if (!period)
        return;
    if (*timer) (*timer)--;
    if (*timer == 0) {
        *timer = period;
        if (dir && *vol < 15u) (*vol)++;
        else if (!dir && *vol > 0u) (*vol)--;
    }
}

static void fs_tick(gbc_apu_t *a)
{
    uint8_t s = a->fs_step;
    if ((s & 1u) == 0) {
        len_clock(&a->ch1.on, a->ch1.len_en, &a->ch1.len);
        len_clock(&a->ch2.on, a->ch2.len_en, &a->ch2.len);
        len_clock(&a->ch3.on, a->ch3.len_en, &a->ch3.len);
        len_clock(&a->ch4.on, a->ch4.len_en, &a->ch4.len);
    }
    if (s == 2u || s == 6u) {
        gbc_square_t *c = &a->ch1;
        if (c->sw_timer) c->sw_timer--;
        if (c->sw_timer == 0) {
            c->sw_timer = c->sw_period ? c->sw_period : 8u;
            if (c->sw_en && c->sw_period) {
                uint16_t nf = sweep_calc(c);
                if (nf <= 2047u && c->sw_shift) {
                    c->sw_shadow = nf;
                    c->freq = nf;
                    (void)sweep_calc(c);
                }
            }
        }
    }
    if (s == 7u) {
        env_clock(&a->ch1.vol, a->ch1.env_dir, a->ch1.env_period, &a->ch1.env_timer);
        env_clock(&a->ch2.vol, a->ch2.env_dir, a->ch2.env_period, &a->ch2.env_timer);
        env_clock(&a->ch4.vol, a->ch4.env_dir, a->ch4.env_period, &a->ch4.env_timer);
    }
    a->fs_step = (uint8_t)((s + 1u) & 7u);
}

/* ---- channel timers ---------------------------------------------------- */
static void square_adv(gbc_square_t *c, uint32_t n)
{
    int32_t period = (int32_t)((2048u - c->freq) * 4u);
    c->timer -= (int32_t)n;
    if (c->timer <= 0) {
        int32_t k = 1 + (-c->timer) / period;
        c->duty_pos = (uint8_t)((c->duty_pos + (uint32_t)k) & 7u);
        c->timer += k * period;
    }
}

static void wave_adv(gbc_wave_t *w, uint32_t n)
{
    int32_t period = (int32_t)((2048u - w->freq) * 2u);
    w->timer -= (int32_t)n;
    if (w->timer <= 0) {
        int32_t k = 1 + (-w->timer) / period;
        w->pos = (uint8_t)((w->pos + (uint32_t)k) & 31u);
        w->timer += k * period;
        w->sample = (uint8_t)((w->pos & 1u) ? (w->ram[w->pos >> 1] & 0x0Fu) : (w->ram[w->pos >> 1] >> 4));
    }
}

static void noise_adv(gbc_noise_t *ns, uint32_t n)
{
    int32_t period = (int32_t)noise_period(ns);
    int32_t k, i;
    ns->timer -= (int32_t)n;
    if (ns->timer > 0)
        return;
    k = 1 + (-ns->timer) / period;
    ns->timer += k * period;
    if (k > 64) k = 64 + (k & 1);   /* LFSR saturates at very high rates */
    for (i = 0; i < k; i++) {
        uint16_t x = (uint16_t)((ns->lfsr ^ (ns->lfsr >> 1)) & 1u);
        ns->lfsr = (uint16_t)((ns->lfsr >> 1) | (x << 14));
        if (ns->width7)
            ns->lfsr = (uint16_t)((ns->lfsr & ~0x40u) | (x << 6));
    }
}

/* portable arithmetic shift right for signed 64-bit */
static inline int64_t asr64(int64_t v, unsigned s)
{
    return v >= 0 ? (v >> s) : -((-v + ((int64_t)1 << s) - 1) >> s);
}

static void mix(gbc_apu_t *a, int16_t *out_lr)
{
    int32_t ch[4], l = 0, r = 0, i;
    int32_t ol, or_;
    const gbc_square_t *s1 = &a->ch1, *s2 = &a->ch2;
    const gbc_wave_t *w = &a->ch3;
    const gbc_noise_t *ns = &a->ch4;
    int32_t d;

    d = (s1->on && ((duty_tbl[s1->duty] >> s1->duty_pos) & 1u)) ? s1->vol : 0;
    ch[0] = s1->dac ? 2 * d - 15 : 0;
    d = (s2->on && ((duty_tbl[s2->duty] >> s2->duty_pos) & 1u)) ? s2->vol : 0;
    ch[1] = s2->dac ? 2 * d - 15 : 0;
    d = (w->on && w->vol_code) ? (w->sample >> (w->vol_code - 1u)) : 0;
    ch[2] = w->dac ? 2 * d - 15 : 0;
    d = (ns->on && !(ns->lfsr & 1u)) ? ns->vol : 0;
    ch[3] = ns->dac ? 2 * d - 15 : 0;

    for (i = 0; i < 4; i++) {
        if (a->nr51 & (0x10u << i)) l += ch[i];
        if (a->nr51 & (0x01u << i)) r += ch[i];
    }
    if (!a->power) { l = 0; r = 0; }
    l *= (int32_t)(((a->nr50 >> 4) & 7u) + 1u);
    r *= (int32_t)((a->nr50 & 7u) + 1u);
    /* integer DC-blocking high-pass, Q8 state, pole 65273/65536 (~0.996) */
    {
        int64_t xl = (int64_t)l * 256, xr = (int64_t)r * 256;
        int64_t yl = xl - a->hp_prev_l + asr64((int64_t)a->hp_l * 65273, 16);
        int64_t yr = xr - a->hp_prev_r + asr64((int64_t)a->hp_r * 65273, 16);
        if (yl > 0x3FFFFFFF) yl = 0x3FFFFFFF;
        if (yl < -0x3FFFFFFF) yl = -0x3FFFFFFF;
        if (yr > 0x3FFFFFFF) yr = 0x3FFFFFFF;
        if (yr < -0x3FFFFFFF) yr = -0x3FFFFFFF;
        a->hp_prev_l = (int32_t)xl; a->hp_prev_r = (int32_t)xr;
        a->hp_l = (int32_t)yl; a->hp_r = (int32_t)yr;
        ol = (int32_t)asr64(yl * 64, 8);
        or_ = (int32_t)asr64(yr * 64, 8);
    }
    if (ol > 32767) ol = 32767;
    if (ol < -32768) ol = -32768;
    if (or_ > 32767) or_ = 32767;
    if (or_ < -32768) or_ = -32768;
    out_lr[0] = (int16_t)ol;
    out_lr[1] = (int16_t)or_;
}

uint32_t gbc_apu_run(gbc_apu_t *a, uint32_t cycles, int16_t *out, uint32_t out_cap)
{
    uint32_t produced = 0;
    while (cycles) {
        uint32_t to_s = (GB_CLOCK_HZ - a->sample_acc + HS_AUDIO_RATE - 1u) / HS_AUDIO_RATE;
        uint32_t n = cycles;
        if (a->fs_timer < n) n = a->fs_timer;
        if (to_s < n) n = to_s;
        if (n == 0) n = 1;
        if (a->power) {
            if (a->ch1.on) square_adv(&a->ch1, n);
            if (a->ch2.on) square_adv(&a->ch2, n);
            if (a->ch3.on) wave_adv(&a->ch3, n);
            if (a->ch4.on) noise_adv(&a->ch4, n);
        }
        cycles -= n;
        a->fs_timer -= n;
        if (a->fs_timer == 0) {
            a->fs_timer = 8192u;
            if (a->power)
                fs_tick(a);
        }
        a->sample_acc += n * HS_AUDIO_RATE;
        if (a->sample_acc >= GB_CLOCK_HZ) {
            int16_t lr[2];
            a->sample_acc -= GB_CLOCK_HZ;
            mix(a, lr);
            if (out && produced < out_cap) {
                out[produced * 2u] = lr[0];
                out[produced * 2u + 1u] = lr[1];
            }
            produced++;
        }
    }
    return produced;
}
