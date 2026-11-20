/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * test_hs_video.c - shared video library tests + SSAA/FXAA benchmark.
 *   - 640x480 stretcher for every target native size (sharp + nearest):
 *     full coverage, exactness, LUT properties, timing.
 *   - FXAA: flat invariance, edge-local changes, determinism.
 *   - SSAA 1x/2x/4x/8x strip-tiled downsampler: correctness, memory bound,
 *     quality vs 8x reference, timing, ssaa_compare.png.
 * Prints "HASH <name> <hex>" lines for cross-arch comparison.
 */
#include "hs_video.h"
#include "hs_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static hs_tstats_t T;
static const char *OUTDIR = "out";
static int QUICK = 0;
static uint32_t *SRC, *DST, *DST2, *ROWTMP, *STRIP;
static uint8_t *LUMA;

static uint64_t fnv(uint64_t h, const void *p, size_t n) { return hs_fnv1a64(h, p, n); }
static uint64_t fb_hash(const uint32_t *fb) { return fnv(HS_FNV64_OFFSET, fb, (size_t)HS_FB_W * HS_FB_H * 4u); }

static void save_png(const char *name, const uint32_t *px, uint32_t w, uint32_t h, size_t stride)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", OUTDIR, name);
    if (hs_png_write_xrgb(path, px, w, h, stride) == 0)
        printf("  wrote %s\n", path);
}

/* ======================================================== scaler */
static void test_scaler(void)
{
    static const struct { uint32_t w, h; const char *what; } R[] = {
        { 160, 144, "GB/GBC" }, { 240, 160, "GBA" }, { 256, 224, "SNES" },
        { 320, 224, "Genesis" }, { 320, 240, "PS1/N64" }, { 640, 480, "native" },
    };
    unsigned r;
    for (r = 0; r < sizeof(R) / sizeof(R[0]); r++) {
        hs_scaler_t sc, sn;
        uint32_t w = R[r].w, h = R[r].h, x, y, bad_cov = 0, bad_exact = 0, bad_near = 0, bad_lut = 0;
        uint32_t bx = 0, by = 0, s = 0x9E3779B9u;
        uint64_t hs, hn, t0, ts, tn;
        unsigned it, iters = QUICK ? 20 : 200;
        HS_CHECK(&T, hs_scaler_init(&sc, w, h, HS_SCALE_STRETCH_SHARP) == 0, "init sharp");
        HS_CHECK(&T, hs_scaler_init(&sn, w, h, HS_SCALE_STRETCH_NEAREST) == 0, "init nearest");
        for (y = 0; y < h; y++) for (x = 0; x < w; x++) { s = s * 1103515245u + 12345u; SRC[y * w + x] = (s >> 8) & 0xFFFFFFu; }
        /* LUT properties */
        if (sc.x0[0] != 0 || sc.x0[HS_FB_W - 1] != w - 1 || sc.y0[0] != 0 || sc.y0[HS_FB_H - 1] != h - 1) bad_lut++;
        for (x = 1; x < HS_FB_W; x++) if (sc.x0[x] < sc.x0[x - 1] || sc.x0[x] > sc.x0[x - 1] + 1) bad_lut++;
        for (y = 1; y < HS_FB_H; y++) if (sc.y0[y] < sc.y0[y - 1] || sc.y0[y] > sc.y0[y - 1] + 1) bad_lut++;
        for (x = 0; x < HS_FB_W; x++) if (sc.xw[x]) bx++;
        for (y = 0; y < HS_FB_H; y++) if (sc.yw[y]) by++;
        /* sentinel: 0xFFxxxxxx can never be produced from 0x00xxxxxx input */
        for (x = 0; x < HS_FB_W * HS_FB_H; x++) DST[x] = 0xFF00FF00u;
        hs_scaler_run(&sc, SRC, w, DST, HS_FB_W, ROWTMP);
        for (x = 0; x < HS_FB_W * HS_FB_H; x++) if (DST[x] & 0xFF000000u) bad_cov++;
        for (y = 0; y < HS_FB_H; y++) for (x = 0; x < HS_FB_W; x++)
            if (!sc.xw[x] && !sc.yw[y] && DST[y * HS_FB_W + x] != SRC[sc.y0[y] * w + sc.x0[x]]) bad_exact++;
        hs = fb_hash(DST);
        for (x = 0; x < HS_FB_W * HS_FB_H; x++) DST2[x] = 0xFF00FF00u;
        hs_scaler_run(&sn, SRC, w, DST2, HS_FB_W, ROWTMP);
        for (y = 0; y < HS_FB_H; y++) for (x = 0; x < HS_FB_W; x++)
            if (DST2[y * HS_FB_W + x] != SRC[sn.y0[y] * w + sn.x0[x]]) bad_near++;
        hn = fb_hash(DST2);
        /* solid colour stays exactly solid */
        for (x = 0; x < w * h; x++) SRC[x] = 0x00123456u;
        hs_scaler_run(&sc, SRC, w, DST, HS_FB_W, ROWTMP);
        {
            uint32_t bad_solid = 0;
            for (x = 0; x < HS_FB_W * HS_FB_H; x++) if (DST[x] != 0x00123456u) bad_solid++;
            HS_CHECK(&T, bad_solid == 0, "%ux%u solid colour not preserved (%u)", w, h, bad_solid);
        }
        t0 = hs_host_now_ns();
        for (it = 0; it < iters; it++) hs_scaler_run(&sc, SRC, w, DST, HS_FB_W, ROWTMP);
        ts = hs_host_now_ns() - t0;
        t0 = hs_host_now_ns();
        for (it = 0; it < iters; it++) hs_scaler_run(&sn, SRC, w, DST, HS_FB_W, ROWTMP);
        tn = hs_host_now_ns() - t0;
        HS_CHECK(&T, bad_lut == 0, "%ux%u LUT monotonic/coverage %u", w, h, bad_lut);
        HS_CHECK(&T, bad_cov == 0, "%ux%u unfilled pixels %u (black bars!)", w, h, bad_cov);
        HS_CHECK(&T, bad_exact == 0, "%ux%u sharp unblended pixels inexact %u", w, h, bad_exact);
        HS_CHECK(&T, bad_near == 0, "%ux%u nearest mismatch %u", w, h, bad_near);
        if (w == 160) HS_CHECK(&T, bx == 0, "160->640 is exact 4x: no blended columns (%u)", bx);
        printf("[scale] %3ux%-3u %-7s -> 640x480: 307200/307200 px filled, blended cols=%3u rows=%3u | sharp %.3f ms, nearest %.3f ms\n",
               w, h, R[r].what, bx, by, ts / 1e6 / iters, tn / 1e6 / iters);
        printf("HASH scale_%ux%u.sharp %016llx\nHASH scale_%ux%u.nearest %016llx\n", w, h, (unsigned long long)hs, w, h, (unsigned long long)hn);
    }
}

/* ======================================================== FXAA */
static void test_fxaa(void)
{
    uint32_t x, y, changed = 0, far = 0, bad_flat = 0;
    uint64_t h;
    for (x = 0; x < HS_FB_W * HS_FB_H; x++) SRC[x] = 0x00406080u;
    hs_fxaa(SRC, HS_FB_W, DST, HS_FB_W, HS_FB_W, HS_FB_H, LUMA);
    for (x = 0; x < HS_FB_W * HS_FB_H; x++) if (DST[x] != SRC[x]) bad_flat++;
    HS_CHECK(&T, bad_flat == 0, "FXAA altered a flat image (%u)", bad_flat);
    /* aliased diagonal edge (slope 1/3) black/white + a gradient region */
    for (y = 0; y < HS_FB_H; y++) for (x = 0; x < HS_FB_W; x++) {
        uint32_t v = (x * 3u < y * 4u + 100u) ? 0x00F0F0F0u : 0x00101010u;
        if (x > 500) v = ((x - 500u) * 255u / 140u) * 0x010101u;   /* smooth: must stay */
        SRC[y * HS_FB_W + x] = v;
    }
    hs_fxaa(SRC, HS_FB_W, DST, HS_FB_W, HS_FB_W, HS_FB_H, LUMA);
    for (y = 1; y + 1 < HS_FB_H; y++) for (x = 1; x + 1 < HS_FB_W; x++) {
        uint32_t i = y * HS_FB_W + x;
        if (DST[i] != SRC[i]) {
            int near_edge = (SRC[i] != SRC[i - 1]) || (SRC[i] != SRC[i + 1]) ||
                            (SRC[i] != SRC[i - HS_FB_W]) || (SRC[i] != SRC[i + HS_FB_W]);
            changed++;
            if (!near_edge || x > 501) far++;
        }
    }
    h = fb_hash(DST);
    HS_CHECK(&T, changed > 400, "FXAA smoothed only %u edge pixels", changed);
    HS_CHECK(&T, far == 0, "FXAA changed %u pixels away from edges", far);
    printf("[fxaa] flat image untouched; diagonal edge: %u pixels blended, %u off-edge changes\n", changed, far);
    printf("HASH fxaa_edge %016llx\n", (unsigned long long)h);
    save_png("fxaa_edge_test.png", DST, HS_FB_W, HS_FB_H, HS_FB_W);
}

/* ======================================================== SSAA scene */
static const int32_t SIN64[64] = {
    0, 1606, 3196, 4756, 6270, 7723, 9102, 10394, 11585, 12665, 13623, 14449, 15137, 15679, 16069, 16305,
    16384, 16305, 16069, 15679, 15137, 14449, 13623, 12665, 11585, 10394, 9102, 7723, 6270, 4756, 3196, 1606,
    0, -1606, -3196, -4756, -6270, -7723, -9102, -10394, -11585, -12665, -13623, -14449, -15137, -15679, -16069, -16305,
    -16384, -16305, -16069, -15679, -15137, -14449, -13623, -12665, -11585, -10394, -9102, -7723, -6270, -4756, -3196, -1606
};
#define ISIN(i) SIN64[(i) & 63]
#define ICOS(i) SIN64[((i) + 16) & 63]

typedef struct { int32_t x[3], y[3]; uint32_t color; } tri_t;   /* Q8 output-pixel coords */
#define MAX_TRIS 160
typedef struct { tri_t t[MAX_TRIS]; uint32_t n; } scene_t;

static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0, bit = 1ull << 62;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; } else r >>= 1;
        bit >>= 2;
    }
    return (uint32_t)r;
}

static void add_tri(scene_t *s, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t c)
{
    tri_t *t;
    int64_t area;
    if (s->n >= MAX_TRIS) return;
    t = &s->t[s->n++];
    area = (int64_t)(x1 - x0) * (y2 - y0) - (int64_t)(y1 - y0) * (x2 - x0);
    if (area < 0) { int32_t tx = x1, ty = y1; x1 = x2; y1 = y2; x2 = tx; y2 = ty; }
    t->x[0] = x0; t->y[0] = y0; t->x[1] = x1; t->y[1] = y1; t->x[2] = x2; t->y[2] = y2; t->color = c;
}

/* 1-pixel-wide line as a thin quad */
static void add_line(scene_t *s, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t c)
{
    int64_t dx = x1 - x0, dy = y1 - y0;
    uint32_t len = isqrt64((uint64_t)(dx * dx + dy * dy));
    int32_t nx, ny;
    if (!len) return;
    nx = (int32_t)(-dy * 128 / (int64_t)len);   /* half width = 0.5 px = 128 Q8 */
    ny = (int32_t)(dx * 128 / (int64_t)len);
    add_tri(s, x0 + nx, y0 + ny, x1 + nx, y1 + ny, x1 - nx, y1 - ny, c);
    add_tri(s, x0 + nx, y0 + ny, x1 - nx, y1 - ny, x0 - nx, y0 - ny, c);
}

static void build_scene(scene_t *s, uint32_t phase)
{
    int32_t cx = 320 * 256, cy = 240 * 256, i;
    s->n = 0;
    /* large diagonal triangles (rotating fan) */
    for (i = 0; i < 6; i++) {
        int32_t a = (int32_t)phase + i * 11, b = a + 7;
        int32_t r = (int32_t)(150 + i * 14) * 256;
        static const uint32_t col[6] = { 0xE04040u, 0x40C060u, 0x4060E0u, 0xE0C040u, 0xC040C0u, 0x40C0C0u };
        add_tri(s, cx, cy,
                cx + (int32_t)((int64_t)r * ICOS(a) / 16384), cy + (int32_t)((int64_t)r * ISIN(a) / 16384),
                cx + (int32_t)((int64_t)r * ICOS(b) / 16384), cy + (int32_t)((int64_t)r * ISIN(b) / 16384), col[i]);
    }
    /* slanted thin sliver triangles near horizontal / vertical (worst aliasing) */
    add_tri(s, 20 * 256, 40 * 256, 620 * 256, 52 * 256, 20 * 256, 46 * 256, 0xFFFFFFu);
    add_tri(s, 40 * 256, 60 * 256, 52 * 256, 460 * 256, 46 * 256, 60 * 256, 0xFFFF80u);
    /* wireframe: 32 rays + 16-gon outline */
    for (i = 0; i < 32; i++) {
        int32_t a = (int32_t)phase + i * 2;
        add_line(s, cx, cy, cx + (int32_t)((int64_t)230 * 256 * ICOS(a) / 16384),
                 cy + (int32_t)((int64_t)230 * 256 * ISIN(a) / 16384), 0xF8F8F8u);
    }
    for (i = 0; i < 16; i++) {
        int32_t a = (int32_t)phase + i * 4, b = a + 4;
        add_line(s, cx + (int32_t)((int64_t)200 * 256 * ICOS(a) / 16384), cy + (int32_t)((int64_t)200 * 256 * ISIN(a) / 16384),
                 cx + (int32_t)((int64_t)200 * 256 * ICOS(b) / 16384), cy + (int32_t)((int64_t)200 * 256 * ISIN(b) / 16384), 0xFFE060u);
    }
}

/* Rasterize the hi-res strip. Sample (xh,yh) sits at ((2xh+1)/(2ss)) px. */
static void raster_strip(void *user, uint32_t *strip, uint32_t sw, uint32_t y0h, uint32_t rows, uint32_t ss)
{
    const scene_t *s = (const scene_t *)user;
    int32_t half = 128 / (int32_t)ss;     /* Q8 half-sample step */
    uint32_t r, t;
    /* background: vertical gradient */
    for (r = 0; r < rows; r++) {
        uint32_t *row = strip + (size_t)r * sw;
        uint32_t yq = (2u * (y0h + r) + 1u) * (uint32_t)half;   /* Q8 */
        uint32_t g = 24u + (yq >> 8) * 64u / 480u, x;
        uint32_t c = (g / 3u << 16) | (g / 2u << 8) | g;
        for (x = 0; x < sw; x++) row[x] = c;
    }
    for (t = 0; t < s->n; t++) {
        const tri_t *tr = &s->t[t];
        int32_t minx = tr->x[0], maxx = minx, miny = tr->y[0], maxy = miny, k;
        int64_t ax[3], ay[3], c0[3];
        int32_t xs, xe, ys, ye, yy;
        for (k = 1; k < 3; k++) {
            if (tr->x[k] < minx) minx = tr->x[k];
            if (tr->x[k] > maxx) maxx = tr->x[k];
            if (tr->y[k] < miny) miny = tr->y[k];
            if (tr->y[k] > maxy) maxy = tr->y[k];
        }
        /* hi-res sample index range */
        xs = (minx / half - 1) / 2; xe = (maxx / half + 1) / 2 + 1;
        ys = (miny / half - 1) / 2; ye = (maxy / half + 1) / 2 + 1;
        if (xs < 0) xs = 0;
        if (xe > (int32_t)sw) xe = (int32_t)sw;
        if (ys < (int32_t)y0h) ys = (int32_t)y0h;
        if (ye > (int32_t)(y0h + rows)) ye = (int32_t)(y0h + rows);
        if (xs >= xe || ys >= ye) continue;
        for (k = 0; k < 3; k++) {
            int32_t k1 = (k + 1) % 3;
            ax[k] = tr->x[k1] - tr->x[k];  /* E = ax*(py-y0) - ay*(px-x0) */
            ay[k] = tr->y[k1] - tr->y[k];
            c0[k] = 0;
        }
        for (yy = ys; yy < ye; yy++) {
            int64_t py = (int64_t)(2 * yy + 1) * half;
            int64_t px0 = (int64_t)(2 * xs + 1) * half;
            int64_t e[3], step[3];
            uint32_t *row = strip + (size_t)(yy - (int32_t)y0h) * sw;
            int32_t xx;
            for (k = 0; k < 3; k++) {
                e[k] = ax[k] * (py - tr->y[k]) - ay[k] * (px0 - tr->x[k]) + c0[k];
                step[k] = -ay[k] * 2 * half;
            }
            for (xx = xs; xx < xe; xx++) {
                if (e[0] >= 0 && e[1] >= 0 && e[2] >= 0)
                    row[xx] = tr->color;
                e[0] += step[0]; e[1] += step[1]; e[2] += step[2];
            }
        }
    }
}

static uint64_t abs_err(const uint32_t *a, const uint32_t *b)
{
    uint64_t e = 0;
    uint32_t i;
    for (i = 0; i < HS_FB_W * HS_FB_H; i++) {
        int32_t dr = (int32_t)((a[i] >> 16) & 255u) - (int32_t)((b[i] >> 16) & 255u);
        int32_t dg = (int32_t)((a[i] >> 8) & 255u) - (int32_t)((b[i] >> 8) & 255u);
        int32_t db = (int32_t)(a[i] & 255u) - (int32_t)(b[i] & 255u);
        e += (uint64_t)((dr < 0 ? -dr : dr) + (dg < 0 ? -dg : dg) + (db < 0 ? -db : db));
    }
    return e;
}

static void test_ssaa(void)
{
    static scene_t scene;
    uint32_t *fr[5];
    static const char *names[5] = { "1x", "fxaa", "2x", "4x", "8x" };
    static const uint32_t factor[5] = { 1, 1, 2, 4, 8 };
    uint32_t iters[5];
    uint64_t err[5], hsh[5];
    double ms[5];
    unsigned m, it;
    uint32_t x, y;

    for (m = 0; m < 5; m++) fr[m] = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    for (m = 0; m < 4; m++) {
        uint32_t ss = 1u << (m == 0 ? 0 : m);
        size_t b = hs_ssaa_strip_bytes(ss);
        HS_CHECK(&T, b <= HS_SSAA_STRIP_BUDGET, "strip bytes %zu", b);
        printf("[ssaa] %ux: strip %u output rows = %ux%u hi-res samples, %zu KiB working set (vs %.1f MiB full target)\n",
               ss, hs_ssaa_rows_per_strip(ss), HS_FB_W * ss, hs_ssaa_rows_per_strip(ss) * ss, b / 1024,
               (double)HS_FB_W * HS_FB_H * ss * ss * 4.0 / (1024.0 * 1024.0));
    }
    /* downsample correctness */
    {
        uint32_t ss = 2, bad = 0;
        for (y = 0; y < 32; y++) for (x = 0; x < HS_FB_W * 2u; x++) STRIP[y * HS_FB_W * 2u + x] = ((x ^ y) & 1u) ? 0x00FFFFFFu : 0;
        hs_ssaa_downsample(STRIP, ss, 16, DST, HS_FB_W);
        for (x = 0; x < HS_FB_W * 16u; x++) if (DST[x] != 0x00808080u) bad++;
        HS_CHECK(&T, bad == 0, "2x2 checker -> 50%% grey (%u bad, got %08X)", bad, DST[0]);
        for (x = 0; x < HS_FB_W * 8u * 2u * 8u; x++) STRIP[x] = 0x00A0B0C0u;
        hs_ssaa_downsample(STRIP, 8, 2, DST, HS_FB_W);
        bad = 0;
        for (x = 0; x < HS_FB_W * 2u; x++) if (DST[x] != 0x00A0B0C0u) bad++;
        HS_CHECK(&T, bad == 0, "8x8 constant preserved (%u)", bad);
    }
    build_scene(&scene, 3);
    printf("[ssaa] scene: %u triangles (6 fan + 2 slivers + 48 wireframe lines as quads)\n", scene.n);
    for (m = 0; m < 5; m++) {
        uint64_t t0;
        iters[m] = QUICK ? (m == 4 ? 1u : 2u) : (m == 4 ? 3u : (m == 3 ? 6u : 20u));
        t0 = hs_host_now_ns();
        for (it = 0; it < iters[m]; it++) {
            if (m == 1) {
                hs_ssaa_render(1, raster_strip, &scene, STRIP, DST2, HS_FB_W);
                hs_fxaa(DST2, HS_FB_W, fr[m], HS_FB_W, HS_FB_W, HS_FB_H, LUMA);
            } else {
                hs_ssaa_render(factor[m], raster_strip, &scene, STRIP, fr[m], HS_FB_W);
            }
        }
        ms[m] = (double)(hs_host_now_ns() - t0) / 1e6 / iters[m];
        hsh[m] = fb_hash(fr[m]);
    }
    /* FXAA alone (post-pass cost) */
    {
        uint64_t t0 = hs_host_now_ns();
        unsigned n = QUICK ? 2 : 20;
        for (it = 0; it < n; it++) hs_fxaa(fr[0], HS_FB_W, DST2, HS_FB_W, HS_FB_W, HS_FB_H, LUMA);
        printf("[ssaa] FXAA post-pass alone: %.3f ms/frame\n", (double)(hs_host_now_ns() - t0) / 1e6 / n);
    }
    for (m = 0; m < 5; m++) err[m] = abs_err(fr[m], fr[4]);
    for (m = 0; m < 5; m++) {
        printf("[ssaa] %-4s frame %8.3f ms (%7.1f fps)  mean |err| vs 8x = %.4f  hash %016llx\n",
               names[m], ms[m], 1000.0 / ms[m], (double)err[m] / (HS_FB_W * HS_FB_H * 3.0), (unsigned long long)hsh[m]);
        printf("HASH ssaa_%s %016llx\n", names[m], (unsigned long long)hsh[m]);
    }
    HS_CHECK(&T, err[0] > err[2] && err[2] > err[3] && err[3] > 0, "SSAA error not monotonic 1x>2x>4x");
    HS_CHECK(&T, err[1] < err[0], "FXAA did not reduce aliasing vs 1x");
    /* determinism: re-render 4x, identical */
    hs_ssaa_render(4, raster_strip, &scene, STRIP, DST2, HS_FB_W);
    HS_CHECK(&T, fb_hash(DST2) == hsh[3], "4x re-render not bit-identical");

    /* ssaa_compare.png: 3x2 grid of 2x-zoomed 320x240 crops: [1x | FXAA | 2x] / [4x | 8x | full 8x] */
    {
        uint32_t W = 1920, H = 960;
        uint32_t *cmp = (uint32_t *)malloc((size_t)W * H * 4u);
        uint32_t panel;
        uint32_t cx0 = 330, cy0 = 120;    /* crop origin (edge-dense region) */
        for (panel = 0; panel < 6; panel++) {
            uint32_t px = (panel % 3) * 640, py = (panel / 3) * 480;
            const uint32_t *src = fr[panel < 5 ? (panel == 0 ? 0 : panel == 1 ? 1 : panel == 2 ? 2 : panel == 3 ? 3 : 4) : 4];
            for (y = 0; y < 480; y++) for (x = 0; x < 640; x++) {
                uint32_t v;
                if (panel == 5) v = src[y * HS_FB_W + x];
                else v = src[(cy0 + y / 2) * HS_FB_W + (cx0 + x / 2)];
                if (x == 0 || y == 0) v = 0x202020u;
                cmp[(py + y) * W + px + x] = v;
            }
        }
        save_png("ssaa_compare.png", cmp, W, H, W);
        free(cmp);
        save_png("ssaa_1x_640x480.png", fr[0], HS_FB_W, HS_FB_H, HS_FB_W);
        save_png("ssaa_8x_640x480.png", fr[4], HS_FB_W, HS_FB_H, HS_FB_W);
    }
    for (m = 0; m < 5; m++) free(fr[m]);
}

int main(int argc, char **argv)
{
    int i;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--quick")) QUICK = 1;
        else if (!strncmp(argv[i], "--out=", 6)) OUTDIR = argv[i] + 6;
    }
    SRC = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    DST = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    DST2 = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    ROWTMP = (uint32_t *)malloc(2u * HS_FB_W * 4u);
    STRIP = (uint32_t *)malloc(HS_SSAA_STRIP_BUDGET);
    LUMA = (uint8_t *)malloc((size_t)HS_FB_W * HS_FB_H);
    if (!SRC || !DST || !DST2 || !ROWTMP || !STRIP || !LUMA) return 2;
    test_scaler();
    test_fxaa();
    test_ssaa();
    printf("\nRESULT hs-video: %u passed, %u failed\n", T.pass, T.fail);
    return T.fail ? 1 : 0;
}
