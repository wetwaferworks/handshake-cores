/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_ym2612.c - clean-room Yamaha YM2612 (OPN2) FM synthesizer.
 *
 * 6 channels x 4 operators, 8 algorithms, op1 self-feedback, 20-bit phase
 * accumulators with detune/multiple, 4-stage ADSR envelope generator
 * (rate = 2*R + key-scale, ticked every 3 samples with 8-step increment
 * patterns), log-sine + exponent output stage (14-bit), LFO AM/PM,
 * channel-3 special (per-operator frequency) mode, timers A/B with status
 * flags, L/R panning and the channel-6 8-bit DAC. Integer only.
 */
#include "gen_sound.h"
#include "gen_ym_tables.h"
#include <string.h>

/* Detune offsets (phase increment units) per key code, DT 1..3. */
static const uint8_t k_dt[4][32] = {
    { 0 },
    { 0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7 },
    { 1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7,8,8,9,10,11,12,13,14,16,16,16,16 },
    { 2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7,8,8,9,10,11,12,13,14,16,17,19,20,22,22,22,22 }
};
/* EG increment fine patterns for rates below 48 (rate & 3). */
static const uint8_t k_eg_lo[4][8] = {
    { 0,1,0,1,0,1,0,1 }, { 0,1,0,1,1,1,0,1 }, { 0,1,1,1,0,1,1,1 }, { 0,1,1,1,1,1,1,1 }
};
/* Extra increments for rates 48..59 (rate & 3). */
static const uint8_t k_eg_hi[4][8] = {
    { 0,0,0,0,0,0,0,0 }, { 0,0,0,1,0,0,0,1 }, { 0,1,0,1,0,1,0,1 }, { 0,1,1,1,0,1,1,1 }
};
static const uint8_t k_lfo_period[8] = { 108, 77, 71, 67, 62, 44, 8, 5 };
static const uint16_t k_fms_mul[8] = { 0, 8, 16, 24, 34, 49, 98, 196 };
static const uint8_t k_ams_shift[4] = { 8, 3, 1, 0 };

/* register slot -> operator number (0..3 = OP1..OP4) and back */
static const uint8_t k_slot_to_opn[4] = { 0, 2, 1, 3 };
static const uint8_t k_opn_to_slot[4] = { 0, 2, 1, 3 };

void ym_reset(gen_ym_t *y)
{
    int c, s;
    memset(y, 0, sizeof(*y));
    for (c = 0; c < 6; c++) {
        y->ch[c].pan_l = y->ch[c].pan_r = 1;
        for (s = 0; s < 4; s++) { y->op[c][s].env = 1023; y->op[c][s].state = 3; y->op[c][s].mul = 0; }
    }
    y->dac = 0x80;
    y->ta_cnt = 1024;
    y->tb_cnt = 256;
}

uint8_t ym_read_status(const gen_ym_t *y) { return y->status & 3; }

static void key_on(gen_ym_t *y, int c, int s, int on)
{
    ym_op_t *o = &y->op[c][s];
    if (on && !o->key) {
        o->phase = 0;
        o->state = 0;
        if (2u * o->ar >= 62u) { o->env = 0; o->state = 1; }
    } else if (!on && o->key) {
        o->state = 3;
    }
    o->key = (uint8_t)on;
}

static void write_reg(gen_ym_t *y, int part, uint8_t r, uint8_t v)
{
    int c, s;
    if (part == 0 && r < 0x30) {
        switch (r) {
        case 0x22: y->lfo_en = (v >> 3) & 1; y->lfo_freq = v & 7; if (!y->lfo_en) { y->lfo_step = 0; y->lfo_cnt = 0; } break;
        case 0x24: y->timer_a = (uint16_t)((y->timer_a & 3) | (v << 2)); break;
        case 0x25: y->timer_a = (uint16_t)((y->timer_a & 0x3FC) | (v & 3)); break;
        case 0x26: y->timer_b = v; break;
        case 0x27:
            if ((v & 1) && !(y->mode27 & 1)) y->ta_cnt = (uint16_t)(1024 - y->timer_a);
            if ((v & 2) && !(y->mode27 & 2)) { y->tb_cnt = (uint16_t)(256 - y->timer_b); y->tb_sub = 0; }
            if (v & 0x10) y->status &= (uint8_t)~1;
            if (v & 0x20) y->status &= (uint8_t)~2;
            y->mode27 = v;
            break;
        case 0x28: {
            int ch = v & 3;
            if (ch == 3) break;
            if (v & 4) ch += 3;
            for (s = 0; s < 4; s++) key_on(y, ch, k_opn_to_slot[s], (v >> (4 + s)) & 1);
            break;
        }
        case 0x2A: y->dac = v; break;
        case 0x2B: y->dac_en = (v >> 7) & 1; break;
        default: break;
        }
        return;
    }
    if (r < 0x30) return;
    c = r & 3;
    if (c == 3) return;
    c += part * 3;
    if (r < 0xA0) {
        ym_op_t *o;
        s = (r >> 2) & 3;
        o = &y->op[c][s];
        switch (r & 0xF0) {
        case 0x30: o->dt = (v >> 4) & 7; o->mul = v & 15; break;
        case 0x40: o->tl = v & 0x7F; break;
        case 0x50: o->ks = v >> 6; o->ar = v & 0x1F; break;
        case 0x60: o->am = v >> 7; o->dr = v & 0x1F; break;
        case 0x70: o->sr = v & 0x1F; break;
        case 0x80: o->sl = v >> 4; o->rr = v & 15; break;
        default: o->ssg = v & 15; break;
        }
        return;
    }
    switch (r & 0xFC) {
    case 0xA0: y->ch[c].fnum = (uint16_t)(((y->ch[c].latch & 7) << 8) | v);
               y->ch[c].block = (y->ch[c].latch >> 3) & 7; break;
    case 0xA4: y->ch[c].latch = v & 0x3F; break;
    case 0xA8:
        if (part == 0) {
            int k = r & 3; /* 0xA8 -> OP3, 0xA9 -> OP1, 0xAA -> OP2 */
            y->ch3_fnum[k] = (uint16_t)(((y->ch3_latch & 7) << 8) | v);
            y->ch3_block[k] = (y->ch3_latch >> 3) & 7;
        }
        break;
    case 0xAC: if (part == 0) y->ch3_latch = v & 0x3F; break;
    case 0xB0: y->ch[c].fb = (v >> 3) & 7; y->ch[c].alg = v & 7; break;
    case 0xB4: y->ch[c].pan_l = (v >> 7) & 1; y->ch[c].pan_r = (v >> 6) & 1;
               y->ch[c].ams = (v >> 4) & 3; y->ch[c].fms = v & 7; break;
    default: break;
    }
}

void ym_write(gen_ym_t *y, uint8_t port, uint8_t v)
{
    if ((port & 1) == 0) { y->addr = v; y->part = (port >> 1) & 1; return; }
    write_reg(y, (port >> 1) & 1, y->addr, v);
}

static inline uint32_t keycode(uint32_t block, uint32_t fnum)
{
    uint32_t f11 = (fnum >> 10) & 1, f10 = (fnum >> 9) & 1, f9 = (fnum >> 8) & 1, f8 = (fnum >> 7) & 1;
    uint32_t n3 = (f11 & (f10 | f9 | f8)) | ((f11 ^ 1u) & f10 & f9 & f8);
    return (block << 2) | (f11 << 1) | n3;
}

static inline int32_t op_out(uint32_t idx10, uint32_t att10)
{
    uint32_t q = idx10 & 0xFF, total, mag;
    if (idx10 & 0x100) q = 255 - q;
    total = k_ym_logsin[q] + (att10 << 2);
    if (total >= (13u << 8)) mag = 0;
    else mag = (uint32_t)k_ym_pow2[total & 0xFF] >> (total >> 8);
    if (mag > 8191) mag = 8191;
    return (idx10 & 0x200) ? -(int32_t)mag : (int32_t)mag;
}

static void eg_step(ym_op_t *o, uint32_t kc, uint32_t egc)
{
    uint32_t rreg, rate, shift, inc, step;
    uint32_t sl = (o->sl == 15) ? 0x3E0u : ((uint32_t)o->sl << 5);
    switch (o->state) {
    case 0: rreg = o->ar; break;
    case 1: rreg = o->dr; break;
    case 2: rreg = o->sr; break;
    default: rreg = (uint32_t)o->rr * 2u + 1u; break;
    }
    if (rreg == 0) return;
    rate = 2u * rreg + (kc >> (3 - o->ks));
    if (o->state == 3) rate = (uint32_t)o->rr * 4u + 2u + (kc >> (3 - o->ks));
    if (rate > 63) rate = 63;
    shift = rate < 48 ? 11u - (rate >> 2) : 0u;
    if (egc & ((1u << shift) - 1u)) return;
    step = (egc >> shift) & 7;
    if (rate < 48) inc = k_eg_lo[rate & 3][step];
    else if (rate < 60) inc = (1u + k_eg_hi[rate & 3][step]) << ((rate >> 2) - 12);
    else inc = 8;
    if (inc == 0) return;
    if (o->state == 0) {
        int32_t e = o->env;
        if (rate >= 62) e = 0;
        else e -= ((e + 1) * (int32_t)inc + 15) / 16;   /* floor((~e * inc) / 16) */
        if (e <= 0) { e = 0; o->state = 1; }
        o->env = (uint16_t)e;
        return;
    }
    {
        uint32_t e = o->env + inc;
        if (e > 1023) e = 1023;
        o->env = (uint16_t)e;
        if (o->state == 1 && e >= sl) o->state = 2;
    }
}

void ym_clock(gen_ym_t *y, int32_t *outl, int32_t *outr)
{
    int c, s;
    int32_t suml = 0, sumr = 0;
    uint32_t am_val, lfo_tri;
    int32_t pm_val;

    /* LFO: 128 steps, period table in samples */
    if (y->lfo_en) {
        if (++y->lfo_cnt >= k_lfo_period[y->lfo_freq]) { y->lfo_cnt = 0; y->lfo_step = (uint8_t)((y->lfo_step + 1) & 127); }
    }
    lfo_tri = y->lfo_step < 64 ? (63u - y->lfo_step) : (uint32_t)(y->lfo_step - 64);
    am_val = lfo_tri * 2u;                                   /* 0..126 */
    pm_val = (y->lfo_step < 32) ? y->lfo_step : (y->lfo_step < 96 ? 63 - y->lfo_step : y->lfo_step - 127);

    /* envelope tick every 3 samples */
    if (++y->eg_div >= 3) {
        y->eg_div = 0;
        y->eg_cnt = (y->eg_cnt + 1) & 0xFFF;
        if (y->eg_cnt == 0) y->eg_cnt = 1;
    }

    for (c = 0; c < 6; c++) {
        ym_ch_t *ch = &y->ch[c];
        int32_t o[4], mod, sum;
        uint32_t att[4];
        for (s = 0; s < 4; s++) {
            ym_op_t *op = &y->op[c][s];
            int opn = k_slot_to_opn[s];
            uint32_t fnum = ch->fnum, block = ch->block, kc, base, dtv, inc;
            if (c == 2 && (y->mode27 & 0xC0) && opn != 3) {
                static const uint8_t map[3] = { 1, 2, 0 };    /* OP1->A9, OP2->AA, OP3->A8 */
                fnum = y->ch3_fnum[map[opn]];
                block = y->ch3_block[map[opn]];
            }
            if (y->lfo_en && ch->fms) {
                int32_t df = ((int32_t)(fnum >> 4) * (int32_t)k_fms_mul[ch->fms] * pm_val) / 8192;
                fnum = (uint32_t)((int32_t)fnum + df) & 0x7FF;
            }
            kc = keycode(block, fnum);
            base = (fnum << block) >> 1;
            dtv = k_dt[op->dt & 3][kc];
            if (op->dt & 4) base -= dtv; else base += dtv;
            base &= 0x1FFFF;
            inc = op->mul ? base * op->mul : base >> 1;
            op->phase = (op->phase + inc) & 0xFFFFF;
            if (y->eg_div == 0) eg_step(op, kc, y->eg_cnt);
            att[opn] = op->env + ((uint32_t)op->tl << 3);
            if (y->lfo_en && op->am && ch->ams) att[opn] += am_val >> k_ams_shift[ch->ams];
            if (att[opn] > 1023) att[opn] = 1023;
        }
        {
            /* operator phases by OPn index */
            uint32_t p1 = y->op[c][0].phase >> 10, p2 = y->op[c][2].phase >> 10;
            uint32_t p3 = y->op[c][1].phase >> 10, p4 = y->op[c][3].phase >> 10;
            int32_t fbm = ch->fb ? ((ch->fb1 + ch->fb2) >> (10 - ch->fb)) : 0;
            o[0] = op_out((p1 + (uint32_t)fbm) & 0x3FF, att[0]);
            ch->fb2 = ch->fb1; ch->fb1 = o[0];
            switch (ch->alg) {
            case 0:
                o[1] = op_out((p2 + (uint32_t)(o[0] >> 1)) & 0x3FF, att[1]);
                o[2] = op_out((p3 + (uint32_t)(o[1] >> 1)) & 0x3FF, att[2]);
                o[3] = op_out((p4 + (uint32_t)(o[2] >> 1)) & 0x3FF, att[3]);
                sum = o[3]; break;
            case 1:
                o[1] = op_out(p2, att[1]);
                o[2] = op_out((p3 + (uint32_t)((o[0] + o[1]) >> 1)) & 0x3FF, att[2]);
                o[3] = op_out((p4 + (uint32_t)(o[2] >> 1)) & 0x3FF, att[3]);
                sum = o[3]; break;
            case 2:
                o[1] = op_out(p2, att[1]);
                o[2] = op_out((p3 + (uint32_t)(o[1] >> 1)) & 0x3FF, att[2]);
                o[3] = op_out((p4 + (uint32_t)((o[0] + o[2]) >> 1)) & 0x3FF, att[3]);
                sum = o[3]; break;
            case 3:
                o[1] = op_out((p2 + (uint32_t)(o[0] >> 1)) & 0x3FF, att[1]);
                o[2] = op_out(p3, att[2]);
                o[3] = op_out((p4 + (uint32_t)((o[1] + o[2]) >> 1)) & 0x3FF, att[3]);
                sum = o[3]; break;
            case 4:
                o[1] = op_out((p2 + (uint32_t)(o[0] >> 1)) & 0x3FF, att[1]);
                o[2] = op_out(p3, att[2]);
                o[3] = op_out((p4 + (uint32_t)(o[2] >> 1)) & 0x3FF, att[3]);
                sum = o[1] + o[3]; break;
            case 5:
                mod = o[0] >> 1;
                o[1] = op_out((p2 + (uint32_t)mod) & 0x3FF, att[1]);
                o[2] = op_out((p3 + (uint32_t)mod) & 0x3FF, att[2]);
                o[3] = op_out((p4 + (uint32_t)mod) & 0x3FF, att[3]);
                sum = o[1] + o[2] + o[3]; break;
            case 6:
                o[1] = op_out((p2 + (uint32_t)(o[0] >> 1)) & 0x3FF, att[1]);
                o[2] = op_out(p3, att[2]);
                o[3] = op_out(p4, att[3]);
                sum = o[1] + o[2] + o[3]; break;
            default:
                o[1] = op_out(p2, att[1]);
                o[2] = op_out(p3, att[2]);
                o[3] = op_out(p4, att[3]);
                sum = o[0] + o[1] + o[2] + o[3]; break;
            }
        }
        if (sum > 8191) sum = 8191;
        if (sum < -8192) sum = -8192;
        if (c == 5 && y->dac_en) sum = ((int32_t)y->dac - 128) << 6;
        ch->out = sum;
        if (ch->pan_l) suml += sum;
        if (ch->pan_r) sumr += sum;
    }

    /* timers: A ticks every sample, B every 16 samples */
    if (y->mode27 & 1) {
        if (--y->ta_cnt == 0) {
            if (y->mode27 & 4) y->status |= 1;
            y->ta_cnt = (uint16_t)(1024 - y->timer_a);
        }
    }
    if (y->mode27 & 2) {
        if (++y->tb_sub >= 16) {
            y->tb_sub = 0;
            if (--y->tb_cnt == 0) {
                if (y->mode27 & 8) y->status |= 2;
                y->tb_cnt = (uint16_t)(256 - y->timer_b);
            }
        }
    }
    y->samples++;
    *outl = suml;
    *outr = sumr;
}
