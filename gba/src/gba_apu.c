/* SPDX-License-Identifier: MIT
 * gba_apu.c - clean-room GBA audio: Direct Sound FIFO A/B (samples captured by
 * the CPU phase as timestamped events) + the 4 legacy DMG PSG channels, replayed
 * and mixed deterministically to 48 kHz stereo int16 in the audio phase.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include "gba_internal.h"

#define SEQ_PERIOD 32768u /* 512 Hz frame sequencer at 2^24 Hz */

static const uint8_t k_duty[4][8] = {
  {0, 0, 0, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 1, 1, 1}, {0, 1, 1, 1, 1, 1, 1, 0}};
static const uint8_t k_noise_div[8] = {8, 16, 32, 48, 64, 80, 96, 112};

static inline uint16_t R(const gba_apu *a, uint32_t off) { return a->reg[(off - 0x60u) >> 1]; }

void gba_apu_reset(gba_apu *a) {
  memset(a, 0, sizeof *a);
  a->nz.lfsr = 0x7FFFu;
  a->reg[(0x88u - 0x60u) >> 1] = 0x200u;
}

static void clock_length(gba_apu *a) {
  for (int i = 0; i < 2; i++) {
    uint16_t ctl = R(a, i ? 0x6Cu : 0x64u);
    if ((ctl & 0x4000u) && a->sq[i].len > 0 && --a->sq[i].len == 0) a->sq[i].on = 0;
  }
  if ((R(a, 0x74u) & 0x4000u) && a->wv.len > 0 && --a->wv.len == 0) a->wv.on = 0;
  if ((R(a, 0x7Cu) & 0x4000u) && a->nz.len > 0 && --a->nz.len == 0) a->nz.on = 0;
}

static void env_step(int *vol, int *timer, int period, int up) {
  if (!period) return;
  if (--*timer > 0) return;
  *timer = period;
  if (up && *vol < 15) (*vol)++;
  else if (!up && *vol > 0) (*vol)--;
}

static void clock_sweep(gba_apu *a) {
  if (!a->sq[0].sweep_on || !a->sq[0].sweep_period) return;
  if (--a->sq[0].sweep_timer > 0) return;
  a->sq[0].sweep_timer = a->sq[0].sweep_period;
  uint32_t sh = a->sq[0].sweep_shadow, d = sh >> a->sq[0].sweep_shift;
  uint32_t nf = a->sq[0].sweep_neg ? sh - d : sh + d;
  if (nf > 2047u) { a->sq[0].on = 0; return; }
  if (a->sq[0].sweep_shift) { a->sq[0].sweep_shadow = nf; a->sq[0].freq = nf; }
}

static void advance(gba_apu *a, uint32_t cyc) {
  a->seq_timer += cyc;
  for (int guard = 0; a->seq_timer >= SEQ_PERIOD && guard < 64; guard++) {
    a->seq_timer -= SEQ_PERIOD;
    int st = a->seq_step;
    a->seq_step = (st + 1) & 7;
    if (!(st & 1)) clock_length(a);
    if (st == 2 || st == 6) clock_sweep(a);
    if (st == 7) {
      for (int i = 0; i < 2; i++) env_step(&a->sq[i].vol, &a->sq[i].env_timer, a->sq[i].env_period, a->sq[i].env_up);
      env_step(&a->nz.vol, &a->nz.env_timer, a->nz.env_period, a->nz.env_up);
    }
  }
  for (int i = 0; i < 2; i++) {
    if (!a->sq[i].on) continue;
    uint32_t period = (2048u - a->sq[i].freq) * 16u;
    a->sq[i].timer += cyc;
    a->sq[i].duty_pos = (int)((a->sq[i].duty_pos + a->sq[i].timer / period) & 7u);
    a->sq[i].timer %= period;
  }
  if (a->wv.on) {
    uint32_t period = (2048u - a->wv.freq) * 8u;
    uint32_t n = (R(a, 0x70u) & 0x20u) ? 64u : 32u;
    a->wv.timer += cyc;
    a->wv.pos = (int)((a->wv.pos + a->wv.timer / period) % n);
    a->wv.timer %= period;
  }
  if (a->nz.on) {
    uint16_t c = R(a, 0x7Cu);
    uint32_t period = ((uint32_t)k_noise_div[c & 7u] << ((c >> 4) & 15u)) * 4u;
    a->nz.timer += cyc;
    uint32_t steps = a->nz.timer / period;
    a->nz.timer %= period;
    if (steps > 4096u) steps = 4096u;
    for (uint32_t s = 0; s < steps; s++) {
      uint16_t l = a->nz.lfsr, x = (uint16_t)((l ^ (l >> 1)) & 1u);
      l = (uint16_t)((l >> 1) | (x << 14));
      if (c & 8u) l = (uint16_t)((l & ~0x40u) | (x << 6));
      a->nz.lfsr = l;
    }
  }
}

static void trigger_square(gba_apu *a, int i) {
  uint16_t env = R(a, i ? 0x68u : 0x62u);
  a->sq[i].vol = env >> 12;
  a->sq[i].env_up = (env >> 11) & 1u;
  a->sq[i].env_period = (env >> 8) & 7u;
  a->sq[i].env_timer = a->sq[i].env_period;
  a->sq[i].on = (env & 0xF800u) != 0;
  if (!a->sq[i].len) a->sq[i].len = 64;
  a->sq[i].timer = 0;
  if (i == 0) {
    uint16_t sw = R(a, 0x60u);
    a->sq[0].sweep_shadow = a->sq[0].freq;
    a->sq[0].sweep_period = (sw >> 4) & 7u;
    a->sq[0].sweep_timer = a->sq[0].sweep_period;
    a->sq[0].sweep_shift = sw & 7u;
    a->sq[0].sweep_neg = (sw >> 3) & 1u;
    a->sq[0].sweep_on = a->sq[0].sweep_period || a->sq[0].sweep_shift;
  }
}

static void apply(gba_apu *a, uint32_t reg, uint16_t v) {
  if (reg == 0xF0u || reg == 0xF1u) { a->fifo[reg - 0xF0u] = (int8_t)(uint8_t)v; return; }
  if (reg < 0x60u || reg >= 0xA0u) return;
  if (reg >= 0x90u) {
    int bank = !((R(a, 0x70u) >> 6) & 1u);
    a->wave[bank][reg - 0x90u] = (uint8_t)v;
    a->wave[bank][reg - 0x90u + 1u] = (uint8_t)(v >> 8);
    return;
  }
  a->reg[(reg - 0x60u) >> 1] = v;
  switch (reg) {
    case 0x62: a->sq[0].len = 64 - (v & 63u); break;
    case 0x68: a->sq[1].len = 64 - (v & 63u); break;
    case 0x64: case 0x6C: {
      int i = reg == 0x6Cu;
      a->sq[i].freq = v & 0x7FFu;
      if (v & 0x8000u) trigger_square(a, i);
      break;
    }
    case 0x70: if (!(v & 0x80u)) a->wv.on = 0; break;
    case 0x72: a->wv.len = 256 - (v & 0xFFu); break;
    case 0x74:
      a->wv.freq = v & 0x7FFu;
      if (v & 0x8000u) {
        a->wv.on = (R(a, 0x70u) & 0x80u) != 0;
        if (!a->wv.len) a->wv.len = 256;
        a->wv.pos = 0;
        a->wv.timer = 0;
      }
      break;
    case 0x78: a->nz.len = 64 - (v & 63u); break;
    case 0x7C:
      if (v & 0x8000u) {
        uint16_t env = R(a, 0x78u);
        a->nz.vol = env >> 12;
        a->nz.env_up = (env >> 11) & 1u;
        a->nz.env_period = (env >> 8) & 7u;
        a->nz.env_timer = a->nz.env_period;
        a->nz.on = (env & 0xF800u) != 0;
        if (!a->nz.len) a->nz.len = 64;
        a->nz.lfsr = 0x7FFFu;
        a->nz.timer = 0;
      }
      break;
    default: break;
  }
}

static void mix(const gba_apu *a, int16_t *out) {
  uint16_t cl = R(a, 0x80u), ch = R(a, 0x82u), cx = R(a, 0x84u);
  if (!(cx & 0x80u)) { out[0] = out[1] = 0; return; }
  int32_t s[4] = {0, 0, 0, 0};
  for (int i = 0; i < 2; i++) {
    if (!a->sq[i].on) continue;
    uint32_t duty = (R(a, i ? 0x68u : 0x62u) >> 6) & 3u;
    s[i] = k_duty[duty][a->sq[i].duty_pos] ? a->sq[i].vol : -a->sq[i].vol;
  }
  if (a->wv.on) {
    uint16_t w70 = R(a, 0x70u), w72 = R(a, 0x72u);
    int bank = (w70 >> 6) & 1u, pos = a->wv.pos;
    if (pos >= 32) { bank ^= 1; pos -= 32; }
    uint8_t b = a->wave[bank][pos >> 1];
    int32_t smp = (pos & 1) ? (b & 15) : (b >> 4);
    smp = smp * 2 - 15;
    if (w72 & 0x8000u) smp = smp * 3 / 4;
    else {
      switch ((w72 >> 13) & 3u) {
        case 0: smp = 0; break;
        case 2: smp /= 2; break;
        case 3: smp /= 4; break;
        default: break;
      }
    }
    s[2] = smp;
  }
  if (a->nz.on) s[3] = (a->nz.lfsr & 1u) ? -a->nz.vol : a->nz.vol;
  int32_t pr = 0, pl = 0;
  for (int i = 0; i < 4; i++) {
    if (cl & (0x100u << i)) pr += s[i];
    if (cl & (0x1000u << i)) pl += s[i];
  }
  pr *= (int32_t)(cl & 7u) + 1;
  pl *= (int32_t)((cl >> 4) & 7u) + 1;
  int sh = (ch & 3u) == 2u ? 0 : ((ch & 3u) == 1u ? 1 : 2);
  pr >>= sh;
  pl >>= sh;
  int32_t fa = a->fifo[0] * ((ch & 4u) ? 4 : 2), fb = a->fifo[1] * ((ch & 8u) ? 4 : 2);
  int32_t L = pl, Rr = pr;
  if (ch & 0x0100u) Rr += fa;
  if (ch & 0x0200u) L += fa;
  if (ch & 0x1000u) Rr += fb;
  if (ch & 0x2000u) L += fb;
  L *= 16;
  Rr *= 16;
  if (L > 32767) L = 32767;
  if (L < -32768) L = -32768;
  if (Rr > 32767) Rr = 32767;
  if (Rr < -32768) Rr = -32768;
  out[0] = (int16_t)L;
  out[1] = (int16_t)Rr;
}

int gba_apu_synth(gba_apu *a, const gba_snapshot *s, int16_t *out, int max_frames) {
  const uint64_t F = HS_GBA_CYCLES_PER_FRAME, RATE = HS_GBA_AUDIO_RATE;
  uint64_t f = s->frame_no;
  uint64_t k0 = (f * F * RATE + (1ull << 24) - 1u) >> 24;
  uint64_t k1 = ((f + 1u) * F * RATE + (1ull << 24) - 1u) >> 24;
  uint32_t n = (uint32_t)(k1 - k0), ei = 0, cur = 0;
  if (max_frames < 0) max_frames = 0;
  int written = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint64_t k = k0 + i;
    uint32_t t = (uint32_t)(((k << 24) / RATE) - f * F);
    for (; ei < s->n_ev && s->ev[ei].t <= t; ei++) {
      uint32_t et = s->ev[ei].t;
      if (et > cur) { advance(a, et - cur); cur = et; }
      apply(a, s->ev[ei].reg, s->ev[ei].val);
    }
    if (t > cur) { advance(a, t - cur); cur = t; }
    if (written < max_frames && out) { mix(a, &out[written * 2]); written++; }
  }
  for (; ei < s->n_ev; ei++) {
    uint32_t et = s->ev[ei].t;
    if (et > cur && et <= F) { advance(a, et - cur); cur = et; }
    apply(a, s->ev[ei].reg, s->ev[ei].val);
  }
  if (cur < F) advance(a, (uint32_t)F - cur);
  a->frames++;
  a->sample_index = k1;
  return written;
}
