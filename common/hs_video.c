/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_video.c - clean-room 640x480 stretcher, integer FXAA, strip SSAA.
 */
#include "hs_video.h"

#include <string.h>

/* ------------------------------------------------------------------------
 * LUT construction.
 *
 * Output pixel d covers the source interval [d*S/D, (d+1)*S/D). Because
 * S <= D (upscale) that interval touches at most two source pixels. The
 * sharp weight is the fraction of the output pixel that falls on the second
 * source pixel: crisp pixel interiors, one blended column/row at each
 * source boundary -> no uneven-width shimmer when 160 -> 640 etc.
 * ---------------------------------------------------------------------- */
static void build_axis(uint16_t *idx, uint8_t *wt, uint32_t src, uint32_t dst, int sharp)
{
    uint32_t d;
    for (d = 0; d < dst; d++) {
        uint32_t start = d * src;            /* in units of 1/dst source px */
        uint32_t end   = start + src;
        uint32_t s0    = start / dst;
        uint32_t bound = (s0 + 1u) * dst;
        uint32_t w     = 0;
        if (sharp) {
            if (end > bound) {
                w = ((end - bound) * 256u + src / 2u) / src;
                if (w > 255u)
                    w = 255u;
            }
        } else {
            /* nearest: sample at the output pixel centre */
            s0 = (2u * start + src) / (2u * dst);
        }
        if (s0 >= src)
            s0 = src - 1u;
        if (s0 + 1u >= src)
            w = 0;
        idx[d] = (uint16_t)s0;
        wt[d]  = (uint8_t)w;
    }
}

int hs_scaler_init(hs_scaler_t *sc, uint32_t src_w, uint32_t src_h, hs_video_mode_t mode)
{
    int sharp;
    if (!sc || src_w == 0 || src_h == 0 || src_w > HS_FB_W || src_h > HS_FB_H)
        return -1;
    memset(sc, 0, sizeof(*sc));
    sharp = (mode != HS_SCALE_STRETCH_NEAREST);
    build_axis(sc->x0, sc->xw, src_w, HS_FB_W, sharp);
    build_axis(sc->y0, sc->yw, src_h, HS_FB_H, sharp);
    sc->src_w = (uint16_t)src_w;
    sc->src_h = (uint16_t)src_h;
    sc->sharp = (uint8_t)sharp;
    return 0;
}

/* Blend two XRGB8888 pixels: (a*(256-w) + b*w) >> 8, two channels per mul. */
static inline uint32_t blend8(uint32_t a, uint32_t b, uint32_t w)
{
    uint32_t iw = 256u - w;
    uint32_t rb = ((a & 0x00FF00FFu) * iw + (b & 0x00FF00FFu) * w) >> 8;
    uint32_t g  = ((a & 0x0000FF00u) * iw + (b & 0x0000FF00u) * w) >> 8;
    return (rb & 0x00FF00FFu) | (g & 0x0000FF00u);
}

static void hscale_row(const hs_scaler_t *sc, const uint32_t *s, uint32_t *out)
{
    uint32_t x;
    for (x = 0; x < HS_FB_W; x++) {
        uint32_t i = sc->x0[x];
        uint32_t w = sc->xw[x];
        uint32_t a = s[i];
        out[x] = w ? blend8(a, s[i + 1u], w) : a;
    }
}

void hs_scaler_run(const hs_scaler_t *sc, const uint32_t *src, size_t src_stride,
                   uint32_t *dst, size_t dst_stride, uint32_t *row_tmp)
{
    /* Two cached horizontally-scaled source rows (ring of 2). */
    uint32_t *ra = row_tmp;
    uint32_t *rb = row_tmp + HS_FB_W;
    int32_t   ya = -1, yb = -1;
    uint32_t  y;

    for (y = 0; y < HS_FB_H; y++) {
        uint32_t  s0 = sc->y0[y];
        uint32_t  w  = sc->yw[y];
        uint32_t *d  = dst + (size_t)y * dst_stride;
        uint32_t *top, *bot;
        uint32_t  x;

        if ((int32_t)s0 == ya) {
            top = ra;
        } else if ((int32_t)s0 == yb) {
            top = rb;
        } else {
            /* evict the row that is not s0+1 */
            if (yb == (int32_t)s0 + 1) {
                hscale_row(sc, src + (size_t)s0 * src_stride, ra);
                ya = (int32_t)s0; top = ra;
            } else {
                hscale_row(sc, src + (size_t)s0 * src_stride, rb);
                yb = (int32_t)s0; top = rb;
            }
        }
        if (w == 0) {
            memcpy(d, top, HS_FB_W * sizeof(uint32_t));
            continue;
        }
        if ((int32_t)(s0 + 1u) == ya) {
            bot = ra;
        } else if ((int32_t)(s0 + 1u) == yb) {
            bot = rb;
        } else if (top == ra) {
            hscale_row(sc, src + (size_t)(s0 + 1u) * src_stride, rb);
            yb = (int32_t)(s0 + 1u); bot = rb;
        } else {
            hscale_row(sc, src + (size_t)(s0 + 1u) * src_stride, ra);
            ya = (int32_t)(s0 + 1u); bot = ra;
        }
        for (x = 0; x < HS_FB_W; x++)
            d[x] = blend8(top[x], bot[x], w);
    }
}

/* ------------------------------------------------------------------------
 * Integer FXAA (luma edge detect + directional blend + short end search).
 * All arithmetic is integer; output is bit-identical on every arch.
 * ---------------------------------------------------------------------- */
#define FXAA_EDGE_MIN   8u   /* ~0.031 */
#define FXAA_EDGE_SHIFT 3u   /* threshold = max/8 (0.125) */
#define FXAA_SEARCH     8

static inline uint32_t luma8(uint32_t c)
{
    return ((((c >> 16) & 255u) * 77u) + (((c >> 8) & 255u) * 150u) + ((c & 255u) * 29u)) >> 8;
}

static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

void hs_fxaa(const uint32_t *src, size_t src_stride, uint32_t *dst, size_t dst_stride,
             uint32_t w, uint32_t h, uint8_t *L)
{
    uint32_t x, y;
    if (w < 3 || h < 3) {
        for (y = 0; y < h; y++)
            memcpy(dst + (size_t)y * dst_stride, src + (size_t)y * src_stride, w * 4u);
        return;
    }
    for (y = 0; y < h; y++) {
        const uint32_t *s = src + (size_t)y * src_stride;
        uint8_t *l = L + (size_t)y * w;
        for (x = 0; x < w; x++)
            l[x] = (uint8_t)luma8(s[x]);
    }
    for (y = 0; y < h; y++) {
        uint32_t yn = y ? y - 1u : 0u;
        uint32_t ys = (y + 1u < h) ? y + 1u : h - 1u;
        const uint8_t *lr = L + (size_t)y * w;
        const uint8_t *ln = L + (size_t)yn * w;
        const uint8_t *ls = L + (size_t)ys * w;
        const uint32_t *srow = src + (size_t)y * src_stride;
        uint32_t *drow = dst + (size_t)y * dst_stride;
        for (x = 0; x < w; x++) {
            uint32_t xw = x ? x - 1u : 0u;
            uint32_t xe = (x + 1u < w) ? x + 1u : w - 1u;
            int32_t M = lr[x], N = ln[x], S = ls[x], W = lr[xw], E = lr[xe];
            int32_t mx = M, mn = M, range, thr;
            int32_t NW, NE, SW, SE, eh, ev, l1, l2, g1, g2, side_l, grad, avg_loc;
            int32_t sum, avg, sub, ss, sub_blend, edge_blend, blend;
            int horz, neg, i, d1, d2, done1, done2;
            int32_t e1 = 0, e2 = 0;
            uint32_t nb;

            if (N > mx) mx = N; if (S > mx) mx = S; if (W > mx) mx = W; if (E > mx) mx = E;
            if (N < mn) mn = N; if (S < mn) mn = S; if (W < mn) mn = W; if (E < mn) mn = E;
            range = mx - mn;
            thr = (int32_t)((uint32_t)mx >> FXAA_EDGE_SHIFT);
            if (thr < (int32_t)FXAA_EDGE_MIN)
                thr = (int32_t)FXAA_EDGE_MIN;
            if (range < thr) {
                drow[x] = srow[x];
                continue;
            }
            NW = ln[xw]; NE = ln[xe]; SW = ls[xw]; SE = ls[xe];
            eh = iabs32(NW + SW - 2 * W) + 2 * iabs32(N + S - 2 * M) + iabs32(NE + SE - 2 * E);
            ev = iabs32(NW + NE - 2 * N) + 2 * iabs32(W + E - 2 * M) + iabs32(SW + SE - 2 * S);
            horz = eh >= ev;              /* horizontal edge -> blend vertically */
            l1 = horz ? N : W;
            l2 = horz ? S : E;
            g1 = iabs32(l1 - M);
            g2 = iabs32(l2 - M);
            neg = g1 >= g2;
            side_l = neg ? l1 : l2;
            grad = (g1 > g2 ? g1 : g2) >> 2;
            avg_loc = (M + side_l) >> 1;

            /* sub-pixel aliasing estimate */
            sum = 2 * (N + S + E + W) + NW + NE + SW + SE;
            avg = (sum * 5461) >> 16;          /* /12 */
            sub = (iabs32(avg - M) * 256) / range;
            if (sub > 256) sub = 256;
            ss = (sub * sub * (768 - 2 * sub)) >> 16;      /* smoothstep, 0..256 */
            sub_blend = ((ss * ss) >> 8) * 3 >> 2;          /* *0.75 -> 0..192 */

            /* edge end search along the edge (bounded) */
            d1 = FXAA_SEARCH; d2 = FXAA_SEARCH; done1 = 0; done2 = 0;
            for (i = 1; i <= FXAA_SEARCH; i++) {
                int32_t ax, bx, ay, by, la, lb;
                if (horz) {
                    int32_t oy = neg ? (int32_t)yn : (int32_t)ys;
                    ax = (int32_t)x - i; bx = (int32_t)x + i;
                    if (ax < 0) ax = 0;
                    if (bx >= (int32_t)w) bx = (int32_t)w - 1;
                    la = (L[(size_t)y * w + (uint32_t)ax] + L[(size_t)oy * w + (uint32_t)ax]) >> 1;
                    lb = (L[(size_t)y * w + (uint32_t)bx] + L[(size_t)oy * w + (uint32_t)bx]) >> 1;
                } else {
                    int32_t ox = neg ? (int32_t)xw : (int32_t)xe;
                    ay = (int32_t)y - i; by = (int32_t)y + i;
                    if (ay < 0) ay = 0;
                    if (by >= (int32_t)h) by = (int32_t)h - 1;
                    la = (L[(size_t)ay * w + x] + L[(size_t)ay * w + (uint32_t)ox]) >> 1;
                    lb = (L[(size_t)by * w + x] + L[(size_t)by * w + (uint32_t)ox]) >> 1;
                }
                if (!done1) {
                    e1 = la - avg_loc;
                    if (iabs32(e1) >= grad) { d1 = i; done1 = 1; }
                }
                if (!done2) {
                    e2 = lb - avg_loc;
                    if (iabs32(e2) >= grad) { d2 = i; done2 = 1; }
                }
                if (done1 && done2)
                    break;
            }
            {
                int32_t closer_e = (d1 < d2) ? e1 : e2;
                int32_t dmin = (d1 < d2) ? d1 : d2;
                int good = ((M - avg_loc) < 0) != (closer_e < 0);
                edge_blend = good ? (128 - (dmin * 256) / (d1 + d2)) : 0;
                if (edge_blend < 0) edge_blend = 0;
            }
            blend = edge_blend > sub_blend ? edge_blend : sub_blend;
            if (horz)
                nb = neg ? src[(size_t)yn * src_stride + x] : src[(size_t)ys * src_stride + x];
            else
                nb = neg ? srow[xw] : srow[xe];
            drow[x] = blend8(srow[x], nb, (uint32_t)blend);
        }
    }
}

/* ------------------------------------------------------------------------
 * Strip-tiled supersampling.
 * ---------------------------------------------------------------------- */
uint32_t hs_ssaa_factor(hs_video_mode_t mode)
{
    switch (mode) {
    case HS_SSAA_2X: return 2u;
    case HS_SSAA_4X: return 4u;
    case HS_SSAA_8X: return 8u;
    default:         return 1u;
    }
}

uint32_t hs_ssaa_rows_per_strip(uint32_t ss)
{
    uint32_t row_bytes = HS_FB_W * ss * ss * 4u;
    uint32_t rows = HS_SSAA_STRIP_BUDGET / row_bytes;
    if (rows > 16u) rows = 16u;
    if (rows < 1u) rows = 1u;
    return rows;
}

size_t hs_ssaa_strip_bytes(uint32_t ss)
{
    return (size_t)hs_ssaa_rows_per_strip(ss) * HS_FB_W * ss * ss * 4u;
}

static uint32_t ilog2_small(uint32_t v)
{
    uint32_t r = 0;
    while (v > 1u && r < 8u) { v >>= 1; r++; }
    return r;
}

void hs_ssaa_downsample(const uint32_t *strip, uint32_t ss, uint32_t rows_out,
                        uint32_t *dst, size_t dst_stride)
{
    uint32_t sw = HS_FB_W * ss;
    uint32_t sh = 2u * ilog2_small(ss);          /* log2(ss*ss) */
    uint32_t half = sh ? (1u << (sh - 1u)) : 0u;
    uint32_t oy, ox, sy, sx;
    if (ss == 1u) {
        for (oy = 0; oy < rows_out; oy++)
            memcpy(dst + (size_t)oy * dst_stride, strip + (size_t)oy * sw, HS_FB_W * 4u);
        return;
    }
    for (oy = 0; oy < rows_out; oy++) {
        uint32_t *d = dst + (size_t)oy * dst_stride;
        const uint32_t *base = strip + (size_t)oy * ss * sw;
        for (ox = 0; ox < HS_FB_W; ox++) {
            uint32_t rb = 0, g = 0;
            const uint32_t *p = base + ox * ss;
            for (sy = 0; sy < ss; sy++) {
                const uint32_t *q = p + (size_t)sy * sw;
                for (sx = 0; sx < ss; sx++) {
                    rb += q[sx] & 0x00FF00FFu;
                    g  += q[sx] & 0x0000FF00u;
                }
            }
            rb = ((rb + (half | (half << 16))) >> sh) & 0x00FF00FFu;
            g  = ((g + (half << 8)) >> sh) & 0x0000FF00u;
            d[ox] = rb | g;
        }
    }
}

void hs_ssaa_render(uint32_t ss, hs_raster_strip_fn fn, void *user,
                    uint32_t *strip_buf, uint32_t *dst, size_t dst_stride)
{
    uint32_t rows = hs_ssaa_rows_per_strip(ss);
    uint32_t y;
    for (y = 0; y < HS_FB_H; y += rows) {
        uint32_t r = rows;
        if (y + r > HS_FB_H)
            r = HS_FB_H - y;
        fn(user, strip_buf, HS_FB_W * ss, y * ss, r * ss, ss);
        hs_ssaa_downsample(strip_buf, ss, r, dst + (size_t)y * dst_stride, dst_stride);
    }
}
