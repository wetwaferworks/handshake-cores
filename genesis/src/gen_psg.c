/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_psg.c - clean-room SN76489 (Sega VDP-integrated variant): three
 * 10-bit square tone channels, one noise channel with a 16-bit LFSR
 * (white noise taps bits 0 and 3), 4-bit attenuation in 2 dB steps.
 */
#include "gen_sound.h"
#include <string.h>

/* 2 dB per step attenuation, step 15 = off. */
static const int16_t k_psg_vol[16] = {
    2000, 1589, 1262, 1002, 796, 632, 502, 399, 317, 252, 200, 159, 126, 100, 80, 0
};

void psg_reset(gen_psg_t *p)
{
    int i;
    memset(p, 0, sizeof(*p));
    for (i = 0; i < 4; i++) { p->vol[i] = 15; p->counter[i] = 1; p->period[i] = 0; }
    p->lfsr = 0x8000;
}

void psg_write(gen_psg_t *p, uint8_t v)
{
    if (v & 0x80) {
        p->latch_ch = (v >> 5) & 3;
        p->latch_vol = (v >> 4) & 1;
        if (p->latch_vol) {
            p->vol[p->latch_ch] = v & 0xF;
        } else if (p->latch_ch == 3) {
            p->noise_reg = v & 7;
            p->lfsr = 0x8000;
        } else {
            p->period[p->latch_ch] = (uint16_t)((p->period[p->latch_ch] & 0x3F0) | (v & 0xF));
        }
        return;
    }
    if (p->latch_vol) {
        p->vol[p->latch_ch] = v & 0xF;
    } else if (p->latch_ch == 3) {
        p->noise_reg = v & 7;
        p->lfsr = 0x8000;
    } else {
        p->period[p->latch_ch] = (uint16_t)((p->period[p->latch_ch] & 0x00F) | ((v & 0x3F) << 4));
    }
}

int32_t psg_tick(gen_psg_t *p)
{
    int32_t mix = 0;
    int i;
    for (i = 0; i < 3; i++) {
        if (--p->counter[i] == 0) {
            p->counter[i] = p->period[i] ? p->period[i] : 1;
            p->out[i] ^= 1;
        }
        if (p->period[i] <= 1) mix += k_psg_vol[p->vol[i]];      /* DC-high */
        else mix += p->out[i] ? k_psg_vol[p->vol[i]] : -k_psg_vol[p->vol[i]];
    }
    if (--p->counter[3] == 0) {
        uint16_t per;
        switch (p->noise_reg & 3) {
        case 0: per = 0x10; break;
        case 1: per = 0x20; break;
        case 2: per = 0x40; break;
        default: per = p->period[2] ? p->period[2] : 1; break;
        }
        p->counter[3] = per;
        p->out[3] ^= 1;
        if (p->out[3]) {   /* shift on the rising edge */
            uint16_t fb = (p->noise_reg & 4) ? (uint16_t)((p->lfsr ^ (p->lfsr >> 3)) & 1) : (uint16_t)(p->lfsr & 1);
            p->lfsr = (uint16_t)((p->lfsr >> 1) | (fb << 15));
        }
    }
    mix += (p->lfsr & 1) ? k_psg_vol[p->vol[3]] : -k_psg_vol[p->vol[3]];
    return mix;
}
