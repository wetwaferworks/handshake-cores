/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_sound.h - clean-room Yamaha YM2612 (OPN2) and TI SN76489 PSG models.
 * Integer only, deterministic, no allocation, no global mutable state.
 */
#ifndef HS_GEN_SOUND_H
#define HS_GEN_SOUND_H

#include <stdint.h>

/* ---- YM2612 ---------------------------------------------------------- */
typedef struct ym_op {
    uint32_t phase;
    int32_t  out;        /* last 14-bit signed output */
    uint16_t env;        /* 10-bit attenuation, 0 = loudest */
    uint8_t  state;      /* 0 attack, 1 decay, 2 sustain, 3 release */
    uint8_t  key;
    uint8_t  dt, mul, tl, ks, ar, dr, sr, sl, rr, am, ssg;
    uint8_t  _pad;
} ym_op_t;

typedef struct ym_ch {
    uint16_t fnum;
    uint8_t  block, latch;
    uint8_t  alg, fb, pan_l, pan_r, ams, fms;
    uint8_t  _pad[2];
    int32_t  fb1, fb2;   /* op1 feedback history */
    int32_t  out;        /* last channel output (14-bit) */
} ym_ch_t;

typedef struct gen_ym {
    ym_ch_t  ch[6];
    ym_op_t  op[6][4];   /* register slot order: 0 = OP1, 1 = OP3, 2 = OP2, 3 = OP4 */
    uint16_t ch3_fnum[3];
    uint8_t  ch3_block[3], ch3_latch;
    uint8_t  addr, part;
    uint8_t  mode27, dac_en, dac;
    uint8_t  lfo_en, lfo_freq, lfo_step;
    uint8_t  status;
    uint8_t  timer_b;
    uint16_t timer_a;
    uint16_t ta_cnt;
    uint16_t tb_cnt;
    uint16_t tb_sub;
    uint16_t lfo_cnt;
    uint32_t eg_cnt;
    uint32_t eg_div;
    uint32_t samples;
} gen_ym_t;

void    ym_reset(gen_ym_t *y);
void    ym_write(gen_ym_t *y, uint8_t port, uint8_t v);   /* port 0..3 */
uint8_t ym_read_status(const gen_ym_t *y);
/* Generate one native (fM/144) sample. */
void    ym_clock(gen_ym_t *y, int32_t *l, int32_t *r);

/* ---- SN76489 --------------------------------------------------------- */
typedef struct gen_psg {
    uint16_t period[4];
    uint16_t counter[4];
    uint8_t  vol[4];
    uint8_t  out[4];      /* square/noise output bit */
    uint16_t lfsr;
    uint8_t  latch_ch, latch_vol;
    uint8_t  noise_reg;
    uint8_t  _pad[3];
} gen_psg_t;

void    psg_reset(gen_psg_t *p);
void    psg_write(gen_psg_t *p, uint8_t v);
/* One PSG tick (clock/16); returns signed mono sample. */
int32_t psg_tick(gen_psg_t *p);

#endif
