/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * bench_n64.c - per-phase timing bench for hs-n64 (pure C11, component TU).
 * Runs the demo ROM for F frames, then re-renders every recorded frame.
 * Prints min-of-R per-frame milliseconds for Phase 0 (CPU+RSP), Phase 1
 * (1x render, whole frame and as S strip-aligned slices), Phase 2, plus
 * hashes so a speed run doubles as an identity check.
 *   bench_n64 [--frames=F] [--reps=R] [--slices=S] [--ref]
 * --ref selects the reference CPU interpreter and the reference RDP path.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hs_test.h"
#include "n64_internal.h"

static uint64_t fnv(uint64_t h, const void *p, size_t len) { return hs_fnv1a64(h, (const uint8_t *)p, len); }

/* Busy (non-idle) MIPS loop: 22 instructions of ALU, shifts, loads and stores
 * over a 4 KiB RDRAM window, a taken backward branch and a delay slot. It
 * measures the interpreter itself; the idle-loop fast-forward never applies. */
#define BI(op, rs, rt, imm) ((uint32_t)(((uint32_t)(op) << 26) | ((uint32_t)(rs) << 21) | ((uint32_t)(rt) << 16) | ((uint32_t)(imm) & 0xFFFFu)))
#define BR(fn, rs, rt, rd, sa) ((uint32_t)(((uint32_t)(rs) << 21) | ((uint32_t)(rt) << 16) | ((uint32_t)(rd) << 11) | ((uint32_t)(sa) << 6) | (uint32_t)(fn)))
static uint64_t busy_run(hs_n64_t *n, uint32_t flags, uint32_t instrs, uint64_t *ns)
{
    static const uint32_t prog[] = {
        BI(15, 0, 16, 0x8000), BI(13, 16, 16, 0x4000),            /* s0 = 0x80004000 */
        BI(9, 0, 8, 0), BI(9, 0, 9, 1),                           /* t0 = 0, t1 = 1 */
        /* loop: */
        BI(35, 16, 10, 0),        BI(9, 10, 10, 3),       BR(0x00, 0, 10, 11, 2),  BR(0x26, 11, 9, 12, 0),
        BI(43, 16, 12, 4),        BI(36, 16, 13, 5),      BR(0x21, 13, 12, 14, 0), BR(0x2B, 14, 10, 15, 0),
        BI(41, 16, 14, 8),        BI(37, 16, 24, 8),      BR(0x25, 24, 15, 25, 0), BR(0x2D, 25, 8, 8, 0),
        BI(12, 8, 17, 0x0FFC),    BR(0x21, 17, 16, 18, 0), BI(43, 18, 8, 0x100),   BI(35, 18, 19, 0x100),
        BR(0x27, 19, 0, 20, 0),   BR(0x02, 0, 20, 21, 7), BI(10, 21, 22, 77),      BI(9, 9, 9, 1),
        BR(0x2A, 22, 9, 23, 0),   BI(5, 0, 9, -22),       BI(9, 23, 23, 1),        BI(4, 0, 0, -24),
        0u
    };
    uint32_t i;
    uint64_t t0;
    hs_core_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.console_id = HS_SYS_N64; cfg.flags = flags;
    hs_n64_core_ops.init(n, sizeof(*n), &cfg);
    n->cpu.cp0[C0_STATUS] = 0x34000000u;             /* CU0|CU1|FR, no BEV/ERL, interrupts off */
    for (i = 0; i < sizeof(prog) / 4u; i++) n64_wbe32(n->rdram + 0x1000u + i * 4u, prog[i]);
    n->cpu.pc = 0xFFFFFFFF80001000ull; n->cpu.npc = n->cpu.pc + 4u;
    t0 = hs_host_now_ns();
    n64_cpu_run(n, instrs);
    *ns = hs_host_now_ns() - t0;
    return n64_cpu_state_hash(n);
}

int main(int argc, char **argv)
{
    uint32_t frames = 90u, reps = 3u, slices = 3u, ref = 0u, f, r, s, i;
    size_t sz = (sizeof(hs_n64_t) + 63u) & ~(size_t)63u;
    hs_n64_t *n = (hs_n64_t *)aligned_alloc(64, sz);
    uint32_t *fb = (uint32_t *)aligned_alloc(64, HS_FB_W * HS_FB_H * 4u);
    const char *rom_path = "roms/hs_n64_demo.z64", *png = NULL;
    size_t rom_cap = 64u << 20;          /* a user-supplied cartridge dump may be up to 64 MiB */
    uint32_t xflags = 0u, user_rom = 0u;
    uint8_t *rom = (uint8_t *)malloc(rom_cap);
    int16_t pcm[2048 * 2];
    size_t romlen;
    uint64_t best0 = ~0ull, best1 = ~0ull, bests = ~0ull, best2 = ~0ull;
    uint64_t sl_best[HS_N64_MAX_SLICES];
    uint64_t hcpu = 0, hfb = 0, hfb_sl = 0;
    hs_core_config_t cfg;
    for (i = 1; i < (uint32_t)argc; i++) {
        if (strncmp(argv[i], "--frames=", 9) == 0) frames = (uint32_t)atoi(argv[i] + 9);
        else if (strncmp(argv[i], "--reps=", 7) == 0) reps = (uint32_t)atoi(argv[i] + 7);
        else if (strncmp(argv[i], "--slices=", 9) == 0) slices = (uint32_t)atoi(argv[i] + 9);
        else if (strcmp(argv[i], "--ref") == 0) ref = 1u;
        else if (strncmp(argv[i], "--rom=", 6) == 0) { rom_path = argv[i] + 6; user_rom = 1u; }   /* never bundled: the owner's own dump */
        else if (strncmp(argv[i], "--cfg=", 6) == 0) xflags = (uint32_t)strtoul(argv[i] + 6, NULL, 16);
        else if (strncmp(argv[i], "--png=", 6) == 0) png = argv[i] + 6;
    }
    if (slices < 1u) slices = 1u;
    if (slices > HS_N64_MAX_SLICES) slices = HS_N64_MAX_SLICES;
    if (!n || !fb || !rom) { printf("alloc failed\n"); return 2; }
    romlen = hs_read_file(rom_path, rom, rom_cap);
    if (romlen < 0x101000u) { printf("ROM missing or too small: %s\n", rom_path); return 2; }
    for (i = 0; i < HS_N64_MAX_SLICES; i++) sl_best[i] = ~0ull;
    memset(&cfg, 0, sizeof(cfg));
    cfg.console_id = HS_SYS_N64;
    cfg.flags = (ref ? (HS_N64_CFG_REF_INTERP | HS_N64_CFG_REF_RDP) : 0u) | xflags;
    for (r = 0; r < reps; r++) {
        uint64_t t0sum = 0, t1sum = 0, tssum = 0, t2sum = 0, slsum[HS_N64_MAX_SLICES];
        uint64_t h = HS_FNV64_OFFSET, hs = HS_FNV64_OFFSET;
        memset(slsum, 0, sizeof(slsum));
        memset(n, 0, sz);
        hs_n64_core_ops.init(n, sz, &cfg);
        if (hs_n64_core_ops.load_rom(n, rom, romlen) != 0) { printf("load failed\n"); return 2; }
        for (f = 0; f < frames; f++) {
            uint64_t a, b, c;
            a = hs_host_now_ns();
            hs_n64_core_ops.step_cpu_phase(n, 0, 0);
            b = hs_host_now_ns();
            hs_n64_core_ops.commit_phase(n);
            hs_n64_core_ops.render_video_phase(n, fb, HS_FB_W, HS_SSAA_1X);
            c = hs_host_now_ns();
            t0sum += b - a; t1sum += c - b;
            h = fnv(h, fb, HS_FB_W * HS_FB_H * 4u);
            memset(fb, 0, HS_FB_W * HS_FB_H * 4u);
            for (s = 0; s < slices; s++) {
                uint64_t d = hs_host_now_ns(), e;
                hs_n64_render_slice(n, fb, HS_FB_W, HS_SSAA_1X, s, slices);
                e = hs_host_now_ns();
                slsum[s] += e - d; tssum += e - d;
            }
            hs = fnv(hs, fb, HS_FB_W * HS_FB_H * 4u);
            a = hs_host_now_ns();
            (void)hs_n64_core_ops.synth_audio_phase(n, pcm, 2048);
            t2sum += hs_host_now_ns() - a;
        }
        if (t0sum < best0) best0 = t0sum;
        if (t1sum < best1) best1 = t1sum;
        if (tssum < bests) bests = tssum;
        if (t2sum < best2) best2 = t2sum;
        for (s = 0; s < slices; s++) if (slsum[s] < sl_best[s]) sl_best[s] = slsum[s];
        hcpu = n64_cpu_state_hash(n); hfb = h; hfb_sl = hs;
    }
    if (user_rom) {                       /* compatibility report for a real cartridge */
        static const char *ukind[6] = { "hs", "fast3d", "f3dex", "f3dex2", "s2dex", "unknown" };
        const n64_snap_t *fr = n64_front(n);
        char id[5];
        {
            uint32_t w38 = n64_rom_word(n, 0x38u), w3c = n64_rom_word(n, 0x3Cu);
            uint8_t c4[4];
            c4[0] = (uint8_t)w38; c4[1] = (uint8_t)(w3c >> 24); c4[2] = (uint8_t)(w3c >> 16); c4[3] = (uint8_t)(w3c >> 8);
            for (i = 0; i < 4u; i++) id[i] = (c4[i] >= 32u && c4[i] < 127u) ? (char)c4[i] : '?';
        }
        id[4] = 0;
        printf("COMPAT rom %s bytes %zu id %s cic %u%s crc_ok %u save_type %u rdram %u\n", rom_path, romlen, id, n->save.cic,
               n->save.cic_known ? "" : "(unknown IPL3, 6102 boot)", n->rom_crc_ok, n->save.type, n->rdram_size >> 20);
        printf("COMPAT ucode %s detects %u gfx_tasks_hle %u unknown_gfx_tasks %u unhandled_cmds %u audio_tasks_skipped %u lle_tasks %u lle_unimpl %u\n",
               ukind[n->ucinfo.kind < 6u ? n->ucinfo.kind : 5u], n->ucinfo.detects, n->rsp.hle_gfx, n->ucinfo.unknown_tasks,
               n->ucinfo.unhandled, n->ucinfo.audio_skipped, n->rsp.lle_tasks, n->rsp.lle_unimpl);
        printf("COMPAT last_frame tris %u rdp_words %u overflow %u vi_origin %08x vi_width %u cpu_pc %016llx reserved_instr %u save_writes %u\n",
               fr->ntris, fr->rdp_n, fr->overflow, fr->vi_origin, fr->vi_width, (unsigned long long)n->cpu.pc, n->cpu.halted_bad, n->save.writes);
        if (png) {
            hs_n64_core_ops.render_video_phase(n, fb, HS_FB_W, HS_SSAA_1X);
            printf("COMPAT png %s %s\n", png, hs_png_write_xrgb(png, fb, HS_FB_W, HS_FB_H, HS_FB_W) == 0 ? "written" : "FAILED");
        }
    }
    {
        uint64_t hb_f = 0, hb_r = 0, nsf = ~0ull, nsr = ~0ull, t;
        for (r = 0; r < reps; r++) {
            hb_f = busy_run(n, 0u, N64_INSTR_PER_FRAME, &t); if (t < nsf) nsf = t;
            hb_r = busy_run(n, HS_N64_CFG_REF_INTERP, N64_INSTR_PER_FRAME, &t); if (t < nsr) nsr = t;
        }
        printf("BENCH busy_cpu_781250_instr_ms fast %.3f reference %.3f state %016llx %016llx %s\n",
               (double)nsf / 1e6, (double)nsr / 1e6, (unsigned long long)hb_f, (unsigned long long)hb_r,
               hb_f == hb_r ? "FAST==REF" : "FAST!=REF");
        if (hb_f != hb_r) hfb_sl ^= 1u;
    }
    printf("BENCH frames=%u reps=%u slices=%u interp=%s\n", frames, reps, slices, ref ? "reference-cpu+rdp" : "fast-cpu+rdp");
    printf("BENCH phase0_cpu_rsp_ms %.3f\n", (double)best0 / frames / 1e6);
    printf("BENCH phase1_render_1x_ms %.3f\n", (double)best1 / frames / 1e6);
    printf("BENCH phase1_render_1x_sliced_total_ms %.3f\n", (double)bests / frames / 1e6);
    for (s = 0; s < slices; s++) printf("BENCH phase1_slice%u_ms %.3f\n", s, (double)sl_best[s] / frames / 1e6);
    printf("BENCH phase2_audio_ms %.3f\n", (double)best2 / frames / 1e6);
    printf("BENCH hash cpu_state %016llx fb_all_frames %016llx fb_sliced_all_frames %016llx %s\n",
           (unsigned long long)hcpu, (unsigned long long)hfb, (unsigned long long)hfb_sl,
           hfb == hfb_sl ? "SLICES==WHOLE" : "SLICES!=WHOLE");
    free(rom); free(fb); free(n);
    return hfb == hfb_sl ? 0 : 1;
}
