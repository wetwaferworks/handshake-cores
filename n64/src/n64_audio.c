/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_audio.c - Phase 2 (Core 2): AI DMA FIFO playback for hs-n64.
 * Reads ONLY the FRONT audio snapshot (the AI buffers Phase 0 captured at
 * AI_LEN writes, at the DAC rate VI_CLOCK/(DACRATE+1)) and resamples them to
 * 48 kHz stereo int16 with exact 16.16 fixed-point linear interpolation.
 * Writes only n64_audio_t and the caller's PCM buffer.
 */
#include "n64_internal.h"

uint32_t n64_audio_synth(hs_n64_t *n, int16_t *pcm, uint32_t max_frames)
{
    const n64_snap_t *s = n64_front(n);
    n64_audio_t *a = &n->audio;
    uint32_t cnt = s->ai_n, out = 0, step, p, guard;
    if (cnt == 0u || s->ai_freq == 0u) return 0;
    step = (uint32_t)(((uint64_t)s->ai_freq << 16) / HS_AUDIO_RATE);
    p = a->phase;
    for (guard = 0; guard < 65536u && out < max_frames; guard++) {
        uint32_t idx = p >> 16, fr = p & 0xFFFFu;
        int32_t l0, r0, l1, r1;
        if (idx >= cnt) break;
        l0 = idx ? s->ai[(idx - 1u) * 2u] : a->last_l;
        r0 = idx ? s->ai[(idx - 1u) * 2u + 1u] : a->last_r;
        l1 = s->ai[idx * 2u]; r1 = s->ai[idx * 2u + 1u];
        pcm[out * 2u] = (int16_t)(l0 + (int32_t)(((int64_t)(l1 - l0) * fr) >> 16));
        pcm[out * 2u + 1u] = (int16_t)(r0 + (int32_t)(((int64_t)(r1 - r0) * fr) >> 16));
        out++;
        p += step;
    }
    if ((p >> 16) >= cnt) p -= cnt << 16; else p = 0;
    a->phase = p;
    a->last_l = s->ai[(cnt - 1u) * 2u];
    a->last_r = s->ai[(cnt - 1u) * 2u + 1u];
    a->frames_out += out;
    return out;
}
