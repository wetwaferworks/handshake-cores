/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * test_hs_snes.c — real verification of the clean-room hs-snes core:
 *   1. 65C816 torture ROM (E/native, M/X widths, DP, long, MVN/MVP, BCD,
 *      mul/div, DMA, HDMA, NMI, V-IRQ, WAI, auto-joypad, stack-relative...)
 *   2. SPC700 IPL handshake + real port-protocol upload + DSP tone
 *   3. Mode 1 demo ROM (tiles, scrolling, sprites, HDMA gradient, color
 *      math, window, audio) and Mode 7 rotating perspective floor ROM:
 *      120 frames, ledger hashes, 640x480 edge fill, PNG previews
 *   4. split-PPU-worker equivalence + pipelined (double-buffered) phases
 * usage: test_hs_snes <outdir>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../hs_snes.h"
#include "snes_testroms.h"
#include "png_write.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct rig {
    hs_snes *s;
    hs_snes_slot *slot[2];
    hs_snes_video *vid;
    hs_snes_dsp *dsp;
    uint32_t *fb;
    int16_t pcm[HS_SNES_AUDIO_MAX * 2];
    int pcm_n;
} rig;

static rig *rig_new(const uint8_t *rom, size_t n)
{
    rig *r = calloc(1, sizeof *r);
    r->s = calloc(1, hs_snes_sizeof());
    r->slot[0] = calloc(1, sizeof(hs_snes_slot));
    r->slot[1] = calloc(1, sizeof(hs_snes_slot));
    r->vid = calloc(1, sizeof(hs_snes_video));
    r->dsp = calloc(1, sizeof(hs_snes_dsp));
    r->fb = calloc(HS_SNES_OUT_W * HS_SNES_OUT_H, 4);
    if (hs_snes_init(r->s, rom, n) != 0) { printf("  FAIL: init\n"); g_fail++; }
    hs_snes_video_init(r->vid);
    hs_snes_dsp_init(r->dsp);
    return r;
}
static void rig_free(rig *r)
{
    free(r->s); free(r->slot[0]); free(r->slot[1]); free(r->vid); free(r->dsp); free(r->fb); free(r);
}
/* one serial frame through all three phases */
static void rig_frame(rig *r, int k)
{
    hs_snes_slot *sl = r->slot[k & 1];
    hs_snes_step_cpu_phase(r->s, sl);
    hs_snes_render_video_phase(sl, r->vid, r->fb);
    r->pcm_n = hs_snes_synth_audio_phase(sl, r->dsp, r->pcm, HS_SNES_AUDIO_MAX);
}
static uint8_t res8(rig *r, unsigned off) { return hs_snes_peek(r->s, TR_RES + off); }
static uint16_t res16(rig *r, unsigned off) { return (uint16_t)(res8(r, off) | (res8(r, off + 1) << 8)); }

static uint64_t fnv_px(uint64_t h, const uint32_t *fb)
{
    size_t i;
    for (i = 0; i < (size_t)HS_SNES_OUT_W * HS_SNES_OUT_H; i++) h = (h ^ (fb[i] & 0xFFFFFFu)) * 1099511628211ull;
    return h;
}
static uint64_t fnv_pcm(uint64_t h, const int16_t *p, int n)
{
    int i;
    for (i = 0; i < n * 2; i++) h = (h ^ (uint16_t)p[i]) * 1099511628211ull;
    return h;
}

static int edge_fill_ok(const uint32_t *fb, int *zero_edge)
{
    int x, y, z = 0;
    for (x = 0; x < HS_SNES_OUT_W; x++) {
        if (!(fb[x] & 0xFFFFFFu)) z++;
        if (!(fb[(size_t)(HS_SNES_OUT_H - 1) * HS_SNES_OUT_W + (size_t)x] & 0xFFFFFFu)) z++;
    }
    for (y = 0; y < HS_SNES_OUT_H; y++) {
        if (!(fb[(size_t)y * HS_SNES_OUT_W] & 0xFFFFFFu)) z++;
        if (!(fb[(size_t)y * HS_SNES_OUT_W + HS_SNES_OUT_W - 1] & 0xFFFFFFu)) z++;
    }
    *zero_edge = z;
    return z == 0;
}
static int distinct_colors(const uint32_t *fb)
{
    static uint8_t seen[1 << 15];
    int n = 0;
    size_t i;
    memset(seen, 0, sizeof seen);
    for (i = 0; i < (size_t)HS_SNES_OUT_W * HS_SNES_OUT_H; i++) {
        uint32_t p = fb[i];
        uint32_t k = ((p >> 9) & 0x7C00u) | ((p >> 6) & 0x3E0u) | ((p >> 3) & 0x1Fu);
        if (!seen[k]) { seen[k] = 1; n++; }
    }
    return n;
}

static void test_cpu(const char *outdir)
{
    static uint8_t rom[0x10000];
    tr_info info;
    rig *r;
    int f;
    char path[512];
    FILE *fp;
    printf("[1] 65C816 CPU torture ROM\n");
    tr_build_cpu_rom(rom, sizeof rom, &info);
    snprintf(path, sizeof path, "%s/hs_cpu_torture.sfc", outdir);
    fp = fopen(path, "wb"); if (fp) { fwrite(rom, 1, info.size, fp); fclose(fp); }
    r = rig_new(rom, info.size);
    CHECK(hs_snes_mapper(r->s) == HS_SNES_MAP_LOROM, "LoROM detection");
    hs_snes_set_input(r->s, HS_SNES_BTN_A | HS_SNES_BTN_START, 0);
    for (f = 0; f < 12; f++) rig_frame(r, f);
    CHECK(res8(r, 0xFF) == 0xA5, "torture ROM completed (done=%02X)", res8(r, 0xFF));
    CHECK(res8(r, 0x00) == 0x37, "emulation ADC/P = %02X (exp 37)", res8(r, 0x00));
    CHECK(res8(r, 0x01) == 0xFF, "emulation S low = %02X (exp FF)", res8(r, 0x01));
    CHECK(res8(r, 0x02) == 0xB5, "XCE native P = %02X (exp B5)", res8(r, 0x02));
    CHECK(res16(r, 0x04) == 0x2345, "16-bit ADC = %04X", res16(r, 0x04));
    CHECK(res8(r, 0x06) == 0xAB && res8(r, 0x07) == 0x12, "M switch keeps B / XBA = %02X %02X", res8(r, 6), res8(r, 7));
    CHECK(res16(r, 0x08) == 0xBEEF, "direct page D=$0100 store = %04X", res16(r, 0x08));
    CHECK(res16(r, 0x0A) == 0x5678, "24-bit long,X = %04X", res16(r, 0x0A));
    CHECK(res16(r, 0x0C) == 0x000E, "MVN copy = %04X", res16(r, 0x0C));
    CHECK(res16(r, 0x0E) == 0x0002, "MVP copy = %04X", res16(r, 0x0E));
    CHECK(res16(r, 0x40) == 0x1FFF && res16(r, 0x42) == 0x3FFF, "MVP X/Y end = %04X/%04X", res16(r, 0x40), res16(r, 0x42));
    CHECK(res16(r, 0x10) == 0x1000, "BCD16 0999+1 = %04X", res16(r, 0x10));
    CHECK(res16(r, 0x12) == 0x0999, "BCD16 1000-1 = %04X", res16(r, 0x12));
    CHECK(res8(r, 0x14) == 0x00 && res8(r, 0x15) == 0x01, "BCD8 99+1 = %02X C=%d", res8(r, 0x14), res8(r, 0x15));
    CHECK(res8(r, 0x44) == 0x09, "BCD8 10-1 = %02X", res8(r, 0x44));
    CHECK(res16(r, 0x16) == 0x03A8, "mul $12*$34 = %04X", res16(r, 0x16));
    CHECK(res16(r, 0x18) == 0x0123 && res8(r, 0x1A) == 4, "div $1234/$10 = %04X r%d", res16(r, 0x18), res8(r, 0x1A));
    CHECK(res8(r, 0x1C) == 94, "DMA ROM->WRAM($2180) last = %d", res8(r, 0x1C));
    CHECK(res8(r, 0x1D) == 31 && res8(r, 0x1E) == 34, "DMA ROM->VRAM + $2139/$213A = %d %d", res8(r, 0x1D), res8(r, 0x1E));
    CHECK(res16(r, 0x30) == 0xABCD, "PEA + LDA sr,S = %04X", res16(r, 0x30));
    CHECK(res16(r, 0x32) == 0xBEEF, "LDA (sr,S),Y = %04X", res16(r, 0x32));
    CHECK(res16(r, 0x34) == 0x1111, "JSR/RTS = %04X", res16(r, 0x34));
    CHECK(res16(r, 0x36) == 0x2222, "JSL/RTL = %04X", res16(r, 0x36));
    CHECK(res16(r, 0x38) == 0x3333, "COP vector/RTI = %04X", res16(r, 0x38));
    CHECK(res16(r, 0x46) == info.per_addr, "PER = %04X (exp %04X)", res16(r, 0x46), info.per_addr);
    CHECK(res16(r, 0x3A) == 0x2345, "LDA (dp),Y = %04X", res16(r, 0x3A));
    CHECK(res16(r, 0x3C) == 0x2345, "LDA [dp],Y = %04X", res16(r, 0x3C));
    CHECK(res8(r, 0x20) == 4, "NMI count = %d (exp 4)", res8(r, 0x20));
    CHECK(res8(r, 0x21) >= 3, "V-IRQ count = %d", res8(r, 0x21));
    CHECK(res8(r, 0x22) == 100 && res8(r, 0x23) == 0, "V-IRQ line via $2137/$213D = %d", res8(r, 0x22) | (res8(r, 0x23) << 8));
    CHECK(res8(r, 0x26) == 0x80 && res8(r, 0x27) == 0x10, "auto-joypad $4218/9 = %02X %02X", res8(r, 0x26), res8(r, 0x27));
    CHECK(hs_snes_peek(r->s, 0x7E6000) == 0x11 && hs_snes_peek(r->s, 0x7E6001) == 0x22 &&
          hs_snes_peek(r->s, 0x7E6002) == 0x33 && hs_snes_peek(r->s, 0x7E6003) == 0x44 &&
          hs_snes_peek(r->s, 0x7E6004) == 0x55, "HDMA -> WRAM lines 1-4 = %02X %02X %02X %02X %02X",
          hs_snes_peek(r->s, 0x7E6000), hs_snes_peek(r->s, 0x7E6001), hs_snes_peek(r->s, 0x7E6002),
          hs_snes_peek(r->s, 0x7E6003), hs_snes_peek(r->s, 0x7E6004));
    {
        hs_snes_cpu_view cv;
        hs_snes_cpu_state(r->s, &cv);
        CHECK(cv.stopped == 1 && cv.e == 0, "STP reached in native mode");
    }
    rig_free(r);
}

typedef struct runres { uint64_t vhash, ahash, all; int edge_zero, colors; long pcm_total; int peak; } runres;

static void run_demo(const char *name, const uint8_t *rom, size_t n, int frames, const char *png, runres *out, tr_info *info, int spc_checks)
{
    rig *r = rig_new(rom, n);
    uint64_t vh = 0xcbf29ce484222325ull, ah = 0xcbf29ce484222325ull;
    int f, i, peak = 0;
    long total = 0;
    for (f = 0; f < frames; f++) {
        rig_frame(r, f);
        vh = fnv_px(vh, r->fb);
        ah = fnv_pcm(ah, r->pcm, r->pcm_n);
        total += r->pcm_n;
        for (i = 0; i < r->pcm_n * 2; i++) { int v = r->pcm[i] < 0 ? -r->pcm[i] : r->pcm[i]; if (v > peak) peak = v; }
    }
    out->vhash = vh; out->ahash = ah; out->all = hs_snes_frame_hash(vh ^ ah, r->fb, r->pcm, r->pcm_n);
    edge_fill_ok(r->fb, &out->edge_zero);
    out->colors = distinct_colors(r->fb);
    out->pcm_total = total; out->peak = peak;
    if (png && hs_png_write_xrgb(png, r->fb, HS_SNES_OUT_W, HS_SNES_OUT_H) == 0) printf("  wrote %s\n", png);
    if (spc_checks) {
        int ok = 1;
        unsigned k;
        for (k = 0; k < info->spc_blob_len; k++)
            if (hs_snes_aram_peek(r->s, (uint16_t)(info->spc_dest + k)) != rom[info->spc_blob_off + k]) { ok = 0; break; }
        printf("[2] SPC700 IPL + port protocol + S-DSP\n");
        CHECK(res8(r, 0x100) == 0xAA, "IPL handshake $AA/$BB seen by 65C816 (%02X)", res8(r, 0x100));
        CHECK(ok, "ARAM upload via $2140-$2143 matches %u-byte blob (first diff @%u)", info->spc_blob_len, k);
        CHECK(res8(r, 0x101) == 0x5A, "uploaded SPC program running, port0 ready = %02X", res8(r, 0x101));
        CHECK(res8(r, 0x102) != 0 && hs_snes_apu_port_out(r->s, 1) != 0, "SPC timer T0 ticks visible on port1 = %d", hs_snes_apu_port_out(r->s, 1));
        CHECK(peak > 1000, "non-zero DSP stereo waveform (peak %d)", peak);
        CHECK(total > frames * 790 && total < frames * 810, "48 kHz output: %ld stereo frames / %d video frames", total, frames);
        CHECK(r->slot[(frames - 1) & 1]->apu_log_dropped == 0, "APU log never overflowed");
        printf("  SPC cycles=%llu  port-sync log hash (last frame)=%016llx\n",
               (unsigned long long)hs_snes_spc_cycles(r->s), (unsigned long long)r->slot[(frames - 1) & 1]->port_log_hash);
    }
    printf("  %s: video=%016llx audio=%016llx edge_zero=%d colors=%d pcm=%ld peak=%d\n", name,
           (unsigned long long)vh, (unsigned long long)ah, out->edge_zero, out->colors, total, peak);
    rig_free(r);
}

/* split PPU workers + pipelined phases must match serial bit-for-bit */
static void test_split_pipeline(const uint8_t *rom, size_t n, uint64_t serial_v, uint64_t serial_a, int frames)
{
    rig *r = rig_new(rom, n);
    hs_snes_video *v2 = calloc(1, sizeof *v2);
    uint32_t *fb2 = calloc(HS_SNES_OUT_W * HS_SNES_OUT_H, 4);
    uint64_t vh = 0xcbf29ce484222325ull, ah = 0xcbf29ce484222325ull;
    int f, mism = 0;
    printf("[4] split PPU workers + pipelined double-buffered phases\n");
    hs_snes_video_init(v2);
    /* pipeline: CPU produces slot f while video/audio consume slot f-1 */
    hs_snes_step_cpu_phase(r->s, r->slot[0]);
    for (f = 1; f <= frames; f++) {
        const hs_snes_slot *prev = r->slot[(f - 1) & 1];
        if (f < frames + 1) hs_snes_step_cpu_phase(r->s, r->slot[f & 1]);
        hs_snes_render_video_rows(prev, r->vid, r->fb, 0, 240);   /* worker A */
        hs_snes_render_video_rows(prev, v2, r->fb, 240, 480);     /* worker B */
        hs_snes_render_video_phase(prev, v2, fb2);
        if (memcmp(r->fb, fb2, (size_t)HS_SNES_OUT_W * HS_SNES_OUT_H * 4)) mism++;
        r->pcm_n = hs_snes_synth_audio_phase(prev, r->dsp, r->pcm, HS_SNES_AUDIO_MAX);
        vh = fnv_px(vh, r->fb);
        ah = fnv_pcm(ah, r->pcm, r->pcm_n);
    }
    CHECK(mism == 0, "2-worker row split == full render on all %d frames (%d mismatches)", frames, mism);
    CHECK(vh == serial_v && ah == serial_a, "pipelined phases reproduce serial ledger (v %016llx a %016llx)",
          (unsigned long long)vh, (unsigned long long)ah);
    free(v2); free(fb2);
    rig_free(r);
}

int main(int argc, char **argv)
{
    const char *outdir = argc > 1 ? argv[1] : "out";
    static uint8_t rom1[0x8000], rom7[0x20000];
    tr_info i1, i7;
    runres m1, m1b, m7;
    char p1[512], p7[512];
    FILE *fp;

    printf("hs-snes verification (sizeof slot=%zu machine=%zu video=%zu dsp=%zu)\n",
           sizeof(hs_snes_slot), hs_snes_sizeof(), sizeof(hs_snes_video), sizeof(hs_snes_dsp));
    test_cpu(outdir);

    if (!tr_build_mode1_rom(rom1, sizeof rom1, &i1)) { printf("FAIL: mode1 rom build\n"); return 1; }
    if (!tr_build_mode7_rom(rom7, sizeof rom7, &i7)) { printf("FAIL: mode7 rom build\n"); return 1; }
    snprintf(p1, sizeof p1, "%s/hs_mode1_demo.sfc", outdir);
    fp = fopen(p1, "wb"); if (fp) { fwrite(rom1, 1, i1.size, fp); fclose(fp); }
    snprintf(p7, sizeof p7, "%s/hs_mode7_floor.sfc", outdir);
    fp = fopen(p7, "wb"); if (fp) { fwrite(rom7, 1, i7.size, fp); fclose(fp); }

    snprintf(p1, sizeof p1, "%s/snes_mode1_640x480.png", outdir);
    run_demo("mode1", rom1, i1.size, 120, p1, &m1, &i1, 1);
    printf("[3] Mode 1 + Mode 7 render (120 frames)\n");
    CHECK(m1.edge_zero == 0, "Mode 1 edge-to-edge 640x480 fill (zero-pixels on border: %d)", m1.edge_zero);
    CHECK(m1.colors > 40, "Mode 1 rich frame (%d distinct colors)", m1.colors);
    run_demo("mode1-rerun", rom1, i1.size, 120, 0, &m1b, &i1, 0);
    CHECK(m1.vhash == m1b.vhash && m1.ahash == m1b.ahash, "Mode 1 deterministic rerun");
    snprintf(p7, sizeof p7, "%s/snes_mode7_640x480.png", outdir);
    run_demo("mode7", rom7, i7.size, 120, p7, &m7, &i7, 0);
    CHECK(m7.edge_zero == 0, "Mode 7 edge-to-edge 640x480 fill (zero-pixels on border: %d)", m7.edge_zero);
    CHECK(m7.colors > 40, "Mode 7 rich frame (%d distinct colors)", m7.colors);
    test_split_pipeline(rom1, i1.size, m1.vhash, m1.ahash, 120);

    printf("HASH mode1 video=%016llx audio=%016llx\n", (unsigned long long)m1.vhash, (unsigned long long)m1.ahash);
    printf("HASH mode7 video=%016llx audio=%016llx\n", (unsigned long long)m7.vhash, (unsigned long long)m7.ahash);
    printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
