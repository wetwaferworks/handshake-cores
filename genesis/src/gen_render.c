/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_render.c - clean-room Genesis VDP rasterizer + 640x480 stretcher
 * (Core 1 domain). Reads ONLY the committed FRONT video snapshot (64 KB
 * VRAM + per-scanline registers/CRAM/VSRAM) and writes ONLY Core-1 scratch
 * and the caller's framebuffer.
 *
 *   - H32 (256) / H40 (320) widths, V28 (224) / V30 (240) heights
 *   - Plane A, Plane B, Window; full / 8-line / per-cell / per-line HScroll;
 *     full-screen and 2-cell VScroll; tile priority, H/V flip, 4x16 palettes
 *   - 80 (H40) / 64 (H32) sprites via the link list, 1x1..4x4 cells,
 *     20/16 sprites per line, x=0 masking, priority, H/V flip
 *   - shadow/highlight (planes low -> shadow, palette 3 colours 14/15 as
 *     highlight/shadow operators, high-priority sprites normal)
 *   - edge-to-edge 640x480: H40 is an exact 2x column double, H32/V28/V30
 *     use precomputed integer area-weighted LUTs (no floats, no per-pixel
 *     divides) or the nearest-neighbour LUT.
 */
#include "gen_internal.h"
#include <string.h>

static void build_lut(uint32_t src, uint32_t dst, uint16_t *x0, uint8_t *w, uint16_t *n0)
{
    uint32_t o;
    for (o = 0; o < dst; o++) {
        uint32_t a = o * src, end = (o + 1) * src;
        uint32_t i0 = a / dst, bnd = (i0 + 1) * dst, wt = 0;
        if (end > bnd && i0 + 1 < src) wt = ((end - bnd) * 256u + src / 2) / src;
        if (wt > 255) wt = 255;
        x0[o] = (uint16_t)i0;
        w[o] = (uint8_t)wt;
        n0[o] = (uint16_t)(((2 * o + 1) * src) / (2 * dst));
    }
}

void gen_render_init(gen_vid_domain_t *v)
{
    uint32_t i;
    memset(v, 0, sizeof(*v));
    build_lut(256, 640, v->xlut_x0[0], v->xlut_w[0], v->nlut_x0[0]);
    build_lut(320, 640, v->xlut_x0[1], v->xlut_w[1], v->nlut_x0[1]);
    build_lut(224, 480, v->ylut_y0[0], v->ylut_w[0], v->nlut_y0[0]);
    build_lut(240, 480, v->ylut_y0[1], v->ylut_w[1], v->nlut_y0[1]);
    for (i = 0; i < 512; i++) {
        uint32_t r = i & 7, g = (i >> 3) & 7, b = (i >> 6) & 7;
        v->color[0][i] = (r * 18u) << 16 | (g * 18u) << 8 | (b * 18u);
        v->color[1][i] = (r * 36u) << 16 | (g * 36u) << 8 | (b * 36u);
        v->color[2][i] = (126u + r * 18u) << 16 | (126u + g * 18u) << 8 | (126u + b * 18u);
    }
}

static inline uint32_t idx9(uint16_t c)
{
    return ((c >> 1) & 7u) | (((c >> 5) & 7u) << 3) | (((c >> 9) & 7u) << 6);
}

static inline uint16_t vram16(const uint8_t *vram, uint32_t a)
{
    a &= 0xFFFE;
    return (uint16_t)((vram[a] << 8) | vram[a + 1]);
}

static inline uint32_t cells(uint32_t code) { return code == 1 ? 64u : (code == 3 ? 128u : 32u); }

static inline uint8_t tile_px(const uint8_t *vram, uint16_t entry, uint32_t px, uint32_t py)
{
    uint32_t tile = entry & 0x7FFu, b;
    if (entry & 0x0800) px = 7 - px;
    if (entry & 0x1000) py = 7 - py;
    b = vram[(tile * 32u + py * 4u + (px >> 1)) & 0xFFFFu];
    b = (px & 1) ? (b & 0xF) : (b >> 4);
    return (uint8_t)((entry & 0x8000 ? 0x80 : 0) | (((entry >> 13) & 3) << 4) | b);
}

static void draw_plane(const gen_video_snap_t *s, const gen_line_snap_t *ls, uint32_t y, uint32_t w,
                       int plane_b, uint8_t *out, int h40)
{
    const uint8_t *vram = s->vram;
    const uint8_t *r = ls->reg;
    uint32_t pw = cells(r[16] & 3), ph = cells((r[16] >> 4) & 3);
    uint32_t base = plane_b ? ((uint32_t)(r[4] & 7) << 13) : ((uint32_t)(r[2] & 0x38) << 10);
    uint32_t hsb = (uint32_t)(r[13] & 0x3F) << 10, off, hs, x;
    uint32_t wmask = pw * 8 - 1, hmask = ph * 8 - 1;
    int vs2 = (r[11] & 4) != 0;
    (void)h40;
    if (pw == 128 && ph == 128) ph = 32;
    switch (r[11] & 3) {
    case 0: off = 0; break;
    case 1: off = (y & 7) * 4; break;
    case 2: off = (y & ~7u) * 4; break;
    default: off = y * 4; break;
    }
    hs = vram16(vram, hsb + off + (plane_b ? 2u : 0u)) & 0x3FF;
    for (x = 0; x < w; x++) {
        uint32_t vs, sx, sy, col = x >> 4;
        uint16_t e;
        if (vs2) vs = col < 20 ? ls->vsram[col * 2 + (plane_b ? 1 : 0)] : ls->vsram[plane_b ? 1 : 0];
        else vs = ls->vsram[plane_b ? 1 : 0];
        sx = (x - hs) & wmask;
        sy = (y + vs) & hmask;
        e = vram16(vram, base + ((sy >> 3) * pw + (sx >> 3)) * 2u);
        out[x] = tile_px(vram, e, sx & 7, sy & 7);
    }
}

static void draw_window(const gen_video_snap_t *s, const gen_line_snap_t *ls, uint32_t y,
                        uint32_t x_from, uint32_t x_to, uint8_t *out, int h40)
{
    const uint8_t *r = ls->reg;
    uint32_t base = (uint32_t)(r[3] & (h40 ? 0x3C : 0x3E)) << 10;
    uint32_t wc = h40 ? 64u : 32u, x;
    for (x = x_from; x < x_to; x++) {
        uint16_t e = vram16(s->vram, base + ((y >> 3) * wc + (x >> 3)) * 2u);
        out[x] = tile_px(s->vram, e, x & 7, y & 7);
    }
}

static void draw_sprites(const gen_video_snap_t *s, const gen_line_snap_t *ls, uint32_t y, uint32_t w,
                         uint8_t *out, int h40)
{
    const uint8_t *vram = s->vram;
    const uint8_t *r = ls->reg;
    uint32_t sat = (uint32_t)(r[5] & (h40 ? 0x7E : 0x7F)) << 9;
    uint32_t max_spr = h40 ? 80u : 64u, max_line = h40 ? 20u : 16u, n, idx = 0, on_line = 0;
    int masked = 0;
    memset(out, 0, w);
    for (n = 0; n < max_spr; n++) {
        uint32_t e = (sat + idx * 8u) & 0xFFFF;
        uint32_t sy = vram16(vram, e) & 0x3FF;
        uint8_t size = vram[(e + 2) & 0xFFFF];
        uint32_t link = vram[(e + 3) & 0xFFFF] & 0x7F;
        uint16_t attr = vram16(vram, e + 4);
        uint32_t sx = vram16(vram, e + 6) & 0x1FF;
        uint32_t hc = ((size >> 2) & 3) + 1u, vc = (size & 3) + 1u;
        int32_t row = (int32_t)y + 128 - (int32_t)sy;
        if (row >= 0 && row < (int32_t)(vc * 8)) {
            if (on_line >= max_line) break;
            on_line++;
            if (sx == 0 && on_line > 1) masked = 1;
            if (!masked) {
                uint32_t rr = (attr & 0x1000) ? vc * 8 - 1 - (uint32_t)row : (uint32_t)row;
                uint32_t px, pwid = hc * 8;
                uint8_t hi = (uint8_t)(((attr & 0x8000) ? 0x80 : 0) | (((attr >> 13) & 3) << 4));
                for (px = 0; px < pwid; px++) {
                    int32_t X = (int32_t)sx - 128 + (int32_t)px;
                    uint32_t c, tile, b;
                    if (X < 0 || X >= (int32_t)w) continue;
                    if (out[X] & 0x0F) continue;
                    c = (attr & 0x0800) ? pwid - 1 - px : px;
                    tile = ((attr & 0x7FFu) + (c >> 3) * vc + (rr >> 3)) & 0x7FFu;
                    b = vram[(tile * 32u + (rr & 7) * 4u + ((c & 7) >> 1)) & 0xFFFF];
                    b = (c & 1) ? (b & 0xF) : (b >> 4);
                    if (b) out[X] = (uint8_t)(hi | b);
                }
            }
        }
        idx = link;
        if (idx == 0 || idx >= max_spr) break;
    }
}

static inline uint32_t blend(uint32_t p, uint32_t q, uint32_t w)
{
    uint32_t iw = 256u - w;
    uint32_t rb = (((p & 0xFF00FFu) * iw + (q & 0xFF00FFu) * w) >> 8) & 0xFF00FFu;
    uint32_t g = (((p & 0x00FF00u) * iw + (q & 0x00FF00u) * w) >> 8) & 0x00FF00u;
    return rb | g;
}

static void render_line(gen_vid_domain_t *v, const gen_video_snap_t *s, uint32_t y, int nearest)
{
    const gen_line_snap_t *ls = &s->line[y];
    const uint8_t *r = ls->reg;
    int h40 = (r[12] & 0x81) != 0;
    uint32_t w = h40 ? 320u : 256u, x;
    uint32_t bg = ls->cram[r[7] & 0x3F];
    int sh = (r[12] & 0x08) != 0;
    uint32_t *row = v->rows[y];
    uint32_t *lp = v->line_px;

    if (!(r[1] & 0x40)) {
        uint32_t c = v->color[1][idx9((uint16_t)bg)];
        for (x = 0; x < 640; x++) row[x] = c;
        return;
    }

    draw_plane(s, ls, y, w, 1, v->pix_b, h40);
    draw_plane(s, ls, y, w, 0, v->pix_a, h40);
    {
        /* window replaces plane A where active */
        uint32_t vp = (uint32_t)(r[18] & 0x1F) * 8u;
        int vwin = (r[18] & 0x80) ? (y >= vp) : (y < vp);
        if (vwin) draw_window(s, ls, y, 0, w, v->pix_a, h40);
        else {
            uint32_t hp = (uint32_t)(r[17] & 0x1F) * 16u;
            if (hp > w) hp = w;
            if (r[17] & 0x80) draw_window(s, ls, y, hp, w, v->pix_a, h40);
            else if (hp) draw_window(s, ls, y, 0, hp, v->pix_a, h40);
        }
    }
    draw_sprites(s, ls, y, w, v->pix_s, h40);

    for (x = 0; x < w; x++) {
        uint8_t b = v->pix_b[x], a = v->pix_a[x], sp = v->pix_s[x];
        uint32_t col = r[7] & 0x3F, shade = 1;
        if ((b & 0x0F) && !(b & 0x80)) col = b & 0x3F;
        if ((a & 0x0F) && !(a & 0x80)) col = a & 0x3F;
        if (sh) {
            uint32_t pcol;
            if ((b & 0x0F) && (b & 0x80)) col = b & 0x3F;
            if ((a & 0x0F) && (a & 0x80)) col = a & 0x3F;
            pcol = col;
            shade = ((a | b) & 0x80) ? 1u : 0u;
            if (sp & 0x0F) {
                uint32_t sc = sp & 0x3F;
                if (sc == 0x3E) { col = pcol; shade = shade < 2 ? shade + 1 : 2; }
                else if (sc == 0x3F) { col = pcol; shade = 0; }
                else if ((sp & 0x80) || !(((b & 0x0F) && (b & 0x80)) || ((a & 0x0F) && (a & 0x80)))) {
                    col = sc;
                    if (sp & 0x80) shade = 1;
                }
            }
        } else {
            if ((sp & 0x0F) && !(sp & 0x80)) col = sp & 0x3F;
            if ((b & 0x0F) && (b & 0x80)) col = b & 0x3F;
            if ((a & 0x0F) && (a & 0x80)) col = a & 0x3F;
            if ((sp & 0x0F) && (sp & 0x80)) col = sp & 0x3F;
        }
        lp[x] = v->color[shade][idx9(ls->cram[col])];
    }

    if (h40) {
        for (x = 0; x < 320; x++) { row[2 * x] = lp[x]; row[2 * x + 1] = lp[x]; }
    } else if (nearest) {
        const uint16_t *n0 = v->nlut_x0[0];
        for (x = 0; x < 640; x++) row[x] = lp[n0[x]];
    } else {
        const uint16_t *x0 = v->xlut_x0[0];
        const uint8_t *xw = v->xlut_w[0];
        for (x = 0; x < 640; x++) {
            uint32_t p = lp[x0[x]];
            row[x] = xw[x] ? blend(p, lp[x0[x] + 1], xw[x]) : p;
        }
    }
}

void gen_render_frame(gen_vid_domain_t *v, const gen_video_snap_t *s,
                      uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    uint32_t y, h = s->active_h == 240 ? 240u : 224u, t = h == 240 ? 1u : 0u;
    int nearest = mode == HS_SCALE_STRETCH_NEAREST;
    for (y = 0; y < h; y++) render_line(v, s, y, nearest);
    for (y = 0; y < 480; y++) {
        uint32_t *dst = fb + (size_t)y * stride;
        if (nearest) {
            memcpy(dst, v->rows[v->nlut_y0[t][y]], 640 * sizeof(uint32_t));
        } else {
            uint32_t y0 = v->ylut_y0[t][y], w = v->ylut_w[t][y], x;
            if (!w) memcpy(dst, v->rows[y0], 640 * sizeof(uint32_t));
            else {
                const uint32_t *p = v->rows[y0], *q = v->rows[y0 + 1];
                for (x = 0; x < 640; x++) dst[x] = blend(p[x], q[x], w);
            }
        }
    }
    v->frames++;
}
