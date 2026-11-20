/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * bench_mt_hs_snes.c — pthread-free, mutex-free multi-core phase benchmark.
 * Worker processes (fork) share one anonymous MAP_SHARED region holding a
 * ring of immutable hs_snes_slot frames.  The only cross-core coordination
 * is a monotonically increasing publish counter per phase written with
 * release stores and read with acquire loads (no CAS, no locks, no
 * futex): CPU publishes slot f, PPU/APU workers publish "consumed f".
 *
 * configs: 1 = serial, 2 = {CPU+APU | PPU}, 3 = {CPU | PPU | APU},
 *          4 = {CPU | PPU rows 0-239 | PPU rows 240-479 | APU}
 * usage: bench_mt_hs_snes [frames] [rom.sfc]
 */
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include "../hs_snes.h"
#include "snes_testroms.h"

#define RING 3

typedef struct shared {
    _Alignas(64) atomic_uint produced;
    _Alignas(64) atomic_uint done_v0;
    _Alignas(64) atomic_uint done_v1;
    _Alignas(64) atomic_uint done_a;
    _Alignas(64) uint64_t hash_v0, hash_v1, hash_a;
    hs_snes_slot slot[RING];
    uint32_t fb[HS_SNES_OUT_W * HS_SNES_OUT_H];
} shared;

static uint64_t hs_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

/* bounded acquire poll: yields the core between polls, never spins on a lock */
static void wait_ge(atomic_uint *w, unsigned target)
{
    unsigned spins = 0;
    while (atomic_load_explicit(w, memory_order_acquire) < target) {
        if (++spins > 64u) { sched_yield(); spins = 0; }
    }
}

static uint64_t hash_rows(uint64_t h, const uint32_t *fb, int y0, int y1)
{
    size_t i;
    for (i = (size_t)y0 * HS_SNES_OUT_W; i < (size_t)y1 * HS_SNES_OUT_W; i++) h = (h ^ (fb[i] & 0xFFFFFFu)) * 1099511628211ull;
    return h;
}
static uint64_t hash_pcm(uint64_t h, const int16_t *p, int n)
{
    int i;
    for (i = 0; i < 2 * n; i++) h = (h ^ (uint16_t)p[i]) * 1099511628211ull;
    return h;
}

/* video ledgers are kept per half (rows 0-239 / 240-479) so every config is comparable */
static void video_worker(shared *sh, int frames, int y0, int y1, atomic_uint *done)
{
    hs_snes_video *v = calloc(1, sizeof *v);
    uint64_t ht = 0xcbf29ce484222325ull, hb = 0xcbf29ce484222325ull;
    int f;
    hs_snes_video_init(v);
    for (f = 0; f < frames; f++) {
        wait_ge(&sh->produced, (unsigned)f + 1u);
        hs_snes_render_video_rows(&sh->slot[f % RING], v, sh->fb, y0, y1);
        if (y0 < 240) ht = hash_rows(ht, sh->fb, y0, y1 < 240 ? y1 : 240);
        if (y1 > 240) hb = hash_rows(hb, sh->fb, y0 > 240 ? y0 : 240, y1);
        atomic_store_explicit(done, (unsigned)f + 1u, memory_order_release);
    }
    if (y0 < 240) sh->hash_v0 = ht;
    if (y1 > 240) sh->hash_v1 = hb;
    free(v);
}
static void audio_worker(shared *sh, int frames, atomic_uint *done, uint64_t *hout)
{
    hs_snes_dsp *d = calloc(1, sizeof *d);
    static int16_t pcm[HS_SNES_AUDIO_MAX * 2];
    uint64_t h = 0xcbf29ce484222325ull;
    int f, n;
    hs_snes_dsp_init(d);
    for (f = 0; f < frames; f++) {
        wait_ge(&sh->produced, (unsigned)f + 1u);
        n = hs_snes_synth_audio_phase(&sh->slot[f % RING], d, pcm, HS_SNES_AUDIO_MAX);
        h = hash_pcm(h, pcm, n);
        atomic_store_explicit(done, (unsigned)f + 1u, memory_order_release);
    }
    *hout = h;
    free(d);
}

static double run(int cfg, const uint8_t *rom, size_t n, int frames, uint64_t out[3])
{
    shared *sh = mmap(0, sizeof(shared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    hs_snes *s = calloc(1, hs_snes_sizeof());
    hs_snes_dsp *d = 0;
    static int16_t pcm[HS_SNES_AUDIO_MAX * 2];
    pid_t kids[3];
    int nk = 0, f, k;
    uint64_t t0, t1, ha = 0xcbf29ce484222325ull;
    if (sh == MAP_FAILED) return -1;
    memset(sh, 0, sizeof *sh);
    atomic_init(&sh->produced, 0); atomic_init(&sh->done_v0, 0); atomic_init(&sh->done_v1, 0); atomic_init(&sh->done_a, 0);
    hs_snes_init(s, rom, n);
    t0 = hs_now_ns();
    if (cfg == 1) {
        hs_snes_video *v = calloc(1, sizeof *v);
        uint64_t hv = 0xcbf29ce484222325ull, hb = 0xcbf29ce484222325ull;
        d = calloc(1, sizeof *d);
        hs_snes_video_init(v); hs_snes_dsp_init(d);
        for (f = 0; f < frames; f++) {
            hs_snes_step_cpu_phase(s, &sh->slot[0]);
            hs_snes_render_video_phase(&sh->slot[0], v, sh->fb);
            hv = hash_rows(hv, sh->fb, 0, 240);
            hb = hash_rows(hb, sh->fb, 240, 480);
            k = hs_snes_synth_audio_phase(&sh->slot[0], d, pcm, HS_SNES_AUDIO_MAX);
            ha = hash_pcm(ha, pcm, k);
        }
        t1 = hs_now_ns();
        out[0] = hv; out[1] = hb; out[2] = ha;
        free(v); free(d); free(s);
        munmap(sh, sizeof *sh);
        return (double)frames * 1e9 / (double)(t1 - t0);
    }
    /* spawn workers */
    if (cfg == 4) {
        if ((kids[nk++] = fork()) == 0) { video_worker(sh, frames, 0, 240, &sh->done_v0); _exit(0); }
        if ((kids[nk++] = fork()) == 0) { video_worker(sh, frames, 240, 480, &sh->done_v1); _exit(0); }
    } else {
        if ((kids[nk++] = fork()) == 0) { video_worker(sh, frames, 0, 480, &sh->done_v0); _exit(0); }
        atomic_store_explicit(&sh->done_v1, (unsigned)frames, memory_order_release);
    }
    if (cfg >= 3) {
        if ((kids[nk++] = fork()) == 0) { audio_worker(sh, frames, &sh->done_a, &sh->hash_a); _exit(0); }
    } else {
        d = calloc(1, sizeof *d);
        hs_snes_dsp_init(d);
    }
    for (f = 0; f < frames; f++) {
        unsigned need = (f >= RING) ? (unsigned)(f - RING + 1) : 0u;
        wait_ge(&sh->done_v0, need);
        wait_ge(&sh->done_v1, need);
        if (cfg >= 3) wait_ge(&sh->done_a, need);
        hs_snes_step_cpu_phase(s, &sh->slot[f % RING]);
        atomic_store_explicit(&sh->produced, (unsigned)f + 1u, memory_order_release);
        if (cfg == 2) {   /* CPU core also runs the APU phase on the slot it just made */
            k = hs_snes_synth_audio_phase(&sh->slot[f % RING], d, pcm, HS_SNES_AUDIO_MAX);
            ha = hash_pcm(ha, pcm, k);
        }
    }
    wait_ge(&sh->done_v0, (unsigned)frames);
    wait_ge(&sh->done_v1, (unsigned)frames);
    if (cfg >= 3) wait_ge(&sh->done_a, (unsigned)frames);
    t1 = hs_now_ns();
    for (k = 0; k < nk; k++) waitpid(kids[k], 0, 0);
    out[0] = sh->hash_v0; out[1] = sh->hash_v1; out[2] = (cfg >= 3) ? sh->hash_a : ha;
    free(d); free(s);
    munmap(sh, sizeof *sh);
    return (double)frames * 1e9 / (double)(t1 - t0);
}

int main(int argc, char **argv)
{
    int frames = argc > 1 ? atoi(argv[1]) : 600, cfg;
    static uint8_t rom[0x400000];
    size_t n = 0;
    tr_info info;
    uint64_t ref[3] = {0, 0, 0}, h[3] = {0, 0, 0};
    double fps1 = 1;
    static const char *names[5] = {"", "{CPU+PPU+APU serial}", "{CPU+APU | PPU}", "{CPU | PPU | APU}", "{CPU | PPU-top | PPU-bot | APU}"};
    if (argc > 2) {
        FILE *f = fopen(argv[2], "rb");
        if (f) { n = fread(rom, 1, sizeof rom, f); fclose(f); }
    }
    if (!n) n = tr_build_mode1_rom(rom, sizeof rom, &info);
    printf("hs-snes multi-core phase bench: %d frames, %ld online cores, rom=%s\n", frames,
           sysconf(_SC_NPROCESSORS_ONLN), argc > 2 ? argv[2] : "built-in hs_mode1_demo");
    for (cfg = 1; cfg <= 4; cfg++) {
        double fps = run(cfg, rom, n, frames, h);
        int same;
        if (cfg == 1) { memcpy(ref, h, sizeof ref); fps1 = fps; }
        same = (h[0] == ref[0] && h[1] == ref[1] && h[2] == ref[2]);
        printf("  cfg %d  %-32s %7.1f fps  (%.2fx vs serial)  ledger %s (vtop=%016llx vbot=%016llx a=%016llx)\n", cfg,
               names[cfg], fps, fps / fps1, same ? "MATCH" : "MISMATCH",
               (unsigned long long)h[0], (unsigned long long)h[1], (unsigned long long)h[2]);
    }
    return 0;
}
