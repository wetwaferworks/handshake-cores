/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * bench_hs_snes.c — single-thread phase-by-phase benchmark of hs-snes.
 * Measures CPU-phase, PPU+640x480-phase and APU-phase time separately so the
 * headroom of splitting phases across cores is visible.
 * usage: bench_hs_snes [frames] [lakesnes_bench.sfc]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../hs_snes.h"
#include "snes_testroms.h"

static uint64_t hs_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

static void bench(const char *name, const uint8_t *rom, size_t n, int frames)
{
    hs_snes *s = calloc(1, hs_snes_sizeof());
    hs_snes_slot *sl = calloc(1, sizeof *sl);
    hs_snes_video *v = calloc(1, sizeof *v);
    hs_snes_dsp *d = calloc(1, sizeof *d);
    uint32_t *fb = calloc(HS_SNES_OUT_W * HS_SNES_OUT_H, 4);
    static int16_t pcm[HS_SNES_AUDIO_MAX * 2];
    uint64_t tc = 0, tv = 0, ta = 0, h = 0xcbf29ce484222325ull, t0, t1, t2, t3;
    int f, np = 0;
    double cpu_ms, vid_ms, apu_ms, serial, ser_fps, p2, p3;
    if (hs_snes_init(s, rom, n) != 0) { printf("%s: init failed\n", name); return; }
    hs_snes_video_init(v);
    hs_snes_dsp_init(d);
    for (f = 0; f < frames; f++) {
        t0 = hs_now_ns();
        hs_snes_step_cpu_phase(s, sl);
        t1 = hs_now_ns();
        hs_snes_render_video_phase(sl, v, fb);
        t2 = hs_now_ns();
        np = hs_snes_synth_audio_phase(sl, d, pcm, HS_SNES_AUDIO_MAX);
        t3 = hs_now_ns();
        tc += t1 - t0; tv += t2 - t1; ta += t3 - t2;
        h = hs_snes_frame_hash(h, fb, pcm, np);
    }
    cpu_ms = (double)tc / 1e6 / frames;
    vid_ms = (double)tv / 1e6 / frames;
    apu_ms = (double)ta / 1e6 / frames;
    serial = cpu_ms + vid_ms + apu_ms;
    ser_fps = 1000.0 / serial;
    p2 = 1000.0 / (cpu_ms + apu_ms > vid_ms ? cpu_ms + apu_ms : vid_ms);
    p3 = cpu_ms;
    if (vid_ms > p3) p3 = vid_ms;
    if (apu_ms > p3) p3 = apu_ms;
    printf("%-14s frames=%d  cpu=%.3f ms  ppu+640x480=%.3f ms  apu=%.3f ms  serial=%.3f ms (%.1f fps)"
           "  ideal-2core=%.1f fps  ideal-3core=%.1f fps  ledger=%016llx\n",
           name, frames, cpu_ms, vid_ms, apu_ms, serial, ser_fps, p2, 1000.0 / p3, (unsigned long long)h);
    free(s); free(sl); free(v); free(d); free(fb);
}

int main(int argc, char **argv)
{
    int frames = argc > 1 ? atoi(argv[1]) : 600;
    static uint8_t rom1[0x8000], rom7[0x20000], ext[0x400000];
    tr_info i1, i7;
    tr_build_mode1_rom(rom1, sizeof rom1, &i1);
    tr_build_mode7_rom(rom7, sizeof rom7, &i7);
    if (argc > 2) {
        FILE *f = fopen(argv[2], "rb");
        if (f) {
            size_t n = fread(ext, 1, sizeof ext, f);
            fclose(f);
            bench("lakesnes-bench", ext, n, frames);
        }
    }
    bench("hs-mode1-demo", rom1, i1.size, frames);
    bench("hs-mode7-floor", rom7, i7.size, frames);
    return 0;
}
