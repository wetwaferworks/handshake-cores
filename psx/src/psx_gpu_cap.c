/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_gpu_cap.c - Phase 0 GPU front end: GP0/GP1 decoding + GPUSTAT/GPUREAD.
 *
 * Phase 0 NEVER rasterizes polygons, lines or sprites. Every drawing packet
 * is decoded into a fixed-size psx_gpu_prim_t (absolute VRAM coordinates,
 * frozen drawing environment) appended to the BACK slot's command list.
 * Only the 2D transfer commands that define texture/CLUT state (CPU->VRAM
 * upload, fill, VRAM->VRAM copy) are applied to the Phase-0 VRAM, which is
 * delta-copied (dirty rows) into the BACK slot at the end of the frame.
 */
#include "psx_internal.h"

#include <string.h>

static inline int32_t sx11(uint32_t v) { return (int32_t)((v & 0x7FFu) ^ 0x400u) - 0x400; }

static void mark_rows(hs_psx_t *s, uint32_t y, uint32_t h)
{
    uint32_t i;
    if (h > PSX_VRAM_H) h = PSX_VRAM_H;
    for (i = 0; i < h; i++) {
        uint32_t r = (y + i) & (PSX_VRAM_H - 1u);
        s->vram_stale[0][r] = 1;
        s->vram_stale[1][r] = 1;
    }
}

void psx_gpu_reset(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    memset(g, 0, sizeof(*g));
    g->da_x1 = 0; g->da_y1 = 0;
    g->disp_mode = 0;
    g->disp_off = 1;
    g->h_range = 0x200u | (0xC00u << 12);
    g->v_range = 0x010u | (0x100u << 10);
}

/* ---- primitive emission ---- */
static psx_gpu_prim_t *new_prim(hs_psx_t *s, uint8_t type)
{
    psx_slot_t *b = psx_back(s);
    psx_gpu0_t *g = &s->gpu;
    psx_gpu_prim_t *p;
    if (b->nprims >= PSX_MAX_PRIMS) { b->dropped++; return NULL; }
    p = &b->prims[b->nprims++];
    memset(p, 0, sizeof(*p));
    p->type = type;
    p->clip_x0 = (int16_t)g->da_x0; p->clip_y0 = (int16_t)g->da_y0;
    p->clip_x1 = (int16_t)g->da_x1; p->clip_y1 = (int16_t)g->da_y1;
    p->tw_mask_x = (uint8_t)(g->texwin & 0x1Fu);
    p->tw_mask_y = (uint8_t)((g->texwin >> 5) & 0x1Fu);
    p->tw_off_x = (uint8_t)((g->texwin >> 10) & 0x1Fu);
    p->tw_off_y = (uint8_t)((g->texwin >> 15) & 0x1Fu);
    if (g->mask_bits & 1u) p->flags |= PSX_PRIM_MASK_SET;
    if (g->mask_bits & 2u) p->flags |= PSX_PRIM_MASK_CHK;
    if (type == PSX_PRIM_TRI || type == PSX_PRIM_RECT || type == PSX_PRIM_LINE) {
        if (!g->present_set) {
            g->present_set = 1;
            g->present_x = (uint16_t)g->da_x0;
            g->present_y = (uint16_t)g->da_y0;
        }
    }
    return p;
}

static void apply_texpage(psx_gpu_prim_t *p, uint32_t tp)
{
    p->tpage_x = (uint16_t)((tp & 0xFu) * 64u);
    p->tpage_y = (uint16_t)(((tp >> 4) & 1u) * 256u);
    p->abr = (uint8_t)((tp >> 5) & 3u);
    p->tex_depth = (uint8_t)((tp >> 7) & 3u);
    if (p->tex_depth == 3u) p->tex_depth = 2u;
}

static int too_big(const int32_t *x, const int32_t *y, int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++) {
            int32_t dx = x[i] - x[j], dy = y[i] - y[j];
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            if (dx > 1023 || dy > 511) return 1;
        }
    return 0;
}

static void emit_tri(hs_psx_t *s, uint32_t cmd, const int32_t *x, const int32_t *y,
                     const uint32_t *rgb, const uint8_t *u, const uint8_t *v,
                     const int *idx, uint32_t clut, uint32_t tp)
{
    psx_gpu0_t *g = &s->gpu;
    int32_t xx[3], yy[3];
    psx_gpu_prim_t *p;
    int k;
    for (k = 0; k < 3; k++) { xx[k] = x[idx[k]]; yy[k] = y[idx[k]]; }
    if (too_big(xx, yy, 3)) return;
    p = new_prim(s, PSX_PRIM_TRI);
    if (!p) return;
    for (k = 0; k < 3; k++) {
        p->x[k] = (int16_t)xx[k]; p->y[k] = (int16_t)yy[k];
        p->rgb[k] = rgb[idx[k]];
        p->u[k] = u[idx[k]]; p->v[k] = v[idx[k]];
    }
    if (cmd & 0x10u) p->flags |= PSX_PRIM_GOURAUD;
    if (cmd & 0x02u) p->flags |= PSX_PRIM_SEMI;
    if (cmd & 0x04u) {
        p->flags |= PSX_PRIM_TEXTURED;
        if (cmd & 0x01u) p->flags |= PSX_PRIM_RAW;
        apply_texpage(p, tp);
        p->clut_x = (uint16_t)((clut & 0x3Fu) * 16u);
        p->clut_y = (uint16_t)((clut >> 6) & 0x1FFu);
    } else {
        apply_texpage(p, g->texpage);
    }
    if ((g->texpage & 0x200u) && ((cmd & 0x10u) || ((cmd & 0x04u) && !(cmd & 0x01u))))
        p->flags |= PSX_PRIM_DITHER;
}

static void do_polygon(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t cmd = g->fifo[0] >> 24;
    int nv = (cmd & 0x08u) ? 4 : 3, tex = (cmd & 0x04u) != 0, gour = (cmd & 0x10u) != 0;
    int32_t x[4], y[4];
    uint32_t rgb[4];
    uint8_t u[4] = { 0, 0, 0, 0 }, v[4] = { 0, 0, 0, 0 };
    uint32_t clut = 0, tp = g->texpage, i = 0;
    int k;
    static const int t0[3] = { 0, 1, 2 }, t1[3] = { 1, 2, 3 };
    for (k = 0; k < nv; k++) {
        uint32_t xy, col;
        if (k == 0 || !gour) col = g->fifo[0] & 0xFFFFFFu;
        else col = g->fifo[i++] & 0xFFFFFFu;
        if (k == 0) i = 1;
        xy = g->fifo[i++];
        rgb[k] = col;
        x[k] = sx11(xy) + g->off_x;
        y[k] = sx11(xy >> 16) + g->off_y;
        if (tex) {
            uint32_t uv = g->fifo[i++];
            u[k] = (uint8_t)uv; v[k] = (uint8_t)(uv >> 8);
            if (k == 0) clut = uv >> 16;
            if (k == 1) tp = uv >> 16;
        }
    }
    if (tex) {   /* textured polygons update the global texpage (GPUSTAT 0..8, 11) */
        g->texpage = (g->texpage & ~0x9FFu) | (tp & 0x9FFu);
    }
    emit_tri(s, cmd, x, y, rgb, u, v, t0, clut, tp);
    if (nv == 4)
        emit_tri(s, cmd, x, y, rgb, u, v, t1, clut, tp);
}

static void emit_line(hs_psx_t *s, uint32_t cmd, uint32_t xy0, uint32_t c0, uint32_t xy1, uint32_t c1)
{
    psx_gpu0_t *g = &s->gpu;
    int32_t x[2], y[2];
    psx_gpu_prim_t *p;
    x[0] = sx11(xy0) + g->off_x; y[0] = sx11(xy0 >> 16) + g->off_y;
    x[1] = sx11(xy1) + g->off_x; y[1] = sx11(xy1 >> 16) + g->off_y;
    if (too_big(x, y, 2)) return;
    p = new_prim(s, PSX_PRIM_LINE);
    if (!p) return;
    p->x[0] = (int16_t)x[0]; p->y[0] = (int16_t)y[0];
    p->x[1] = (int16_t)x[1]; p->y[1] = (int16_t)y[1];
    p->rgb[0] = c0 & 0xFFFFFFu;
    p->rgb[1] = (cmd & 0x10u) ? (c1 & 0xFFFFFFu) : (c0 & 0xFFFFFFu);
    if (cmd & 0x10u) p->flags |= PSX_PRIM_GOURAUD;
    if (cmd & 0x02u) p->flags |= PSX_PRIM_SEMI;
    apply_texpage(p, g->texpage);
    if ((g->texpage & 0x200u) && (cmd & 0x10u)) p->flags |= PSX_PRIM_DITHER;
}

static void do_rect(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t cmd = g->fifo[0] >> 24, i = 2, w, h, sz = (cmd >> 3) & 3u, uv = 0;
    psx_gpu_prim_t *p;
    int32_t x = sx11(g->fifo[1]) + g->off_x, y = sx11(g->fifo[1] >> 16) + g->off_y;
    if (cmd & 0x04u) uv = g->fifo[i++];
    if (sz == 0) { w = g->fifo[i] & 0x3FFu; h = (g->fifo[i] >> 16) & 0x1FFu; }
    else if (sz == 1) { w = h = 1; }
    else if (sz == 2) { w = h = 8; }
    else { w = h = 16; }
    if (!w || !h) return;
    p = new_prim(s, PSX_PRIM_RECT);
    if (!p) return;
    p->x[0] = (int16_t)x; p->y[0] = (int16_t)y;
    p->w = (uint16_t)w; p->h = (uint16_t)h;
    p->rgb[0] = p->rgb[1] = p->rgb[2] = g->fifo[0] & 0xFFFFFFu;
    if (cmd & 0x02u) p->flags |= PSX_PRIM_SEMI;
    apply_texpage(p, g->texpage);
    if (cmd & 0x04u) {
        p->flags |= PSX_PRIM_TEXTURED;
        if (cmd & 0x01u) p->flags |= PSX_PRIM_RAW;
        p->u[0] = (uint8_t)uv; p->v[0] = (uint8_t)(uv >> 8);
        p->clut_x = (uint16_t)(((uv >> 16) & 0x3Fu) * 16u);
        p->clut_y = (uint16_t)((uv >> 22) & 0x1FFu);
    }
}

static inline uint16_t rgb24_to_15(uint32_t c)
{
    return (uint16_t)(((c >> 3) & 0x1Fu) | (((c >> 11) & 0x1Fu) << 5) | (((c >> 19) & 0x1Fu) << 10));
}

static void do_fill(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t x = g->fifo[1] & 0x3F0u, y = (g->fifo[1] >> 16) & 0x1FFu;
    uint32_t w = ((g->fifo[2] & 0x3FFu) + 0xFu) & ~0xFu, h = (g->fifo[2] >> 16) & 0x1FFu;
    uint16_t c = rgb24_to_15(g->fifo[0]);
    uint32_t i, j;
    psx_gpu_prim_t *p;
    for (j = 0; j < h; j++) {
        uint16_t *row = s->vram0 + ((y + j) & 511u) * PSX_VRAM_W;
        for (i = 0; i < w; i++) row[(x + i) & 1023u] = c;
    }
    mark_rows(s, y, h);
    p = new_prim(s, PSX_PRIM_FILL);
    if (!p) return;
    p->x[0] = (int16_t)x; p->y[0] = (int16_t)y; p->w = (uint16_t)w; p->h = (uint16_t)h;
    p->rgb[0] = g->fifo[0] & 0xFFFFFFu;
}

static void do_copy(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t sx = g->fifo[1] & 0x3FFu, sy = (g->fifo[1] >> 16) & 0x1FFu;
    uint32_t dx = g->fifo[2] & 0x3FFu, dy = (g->fifo[2] >> 16) & 0x1FFu;
    uint32_t w = ((g->fifo[3] - 1u) & 0x3FFu) + 1u, h = (((g->fifo[3] >> 16) - 1u) & 0x1FFu) + 1u;
    uint32_t i, j;
    psx_gpu_prim_t *p;
    for (j = 0; j < h; j++)
        for (i = 0; i < w; i++) {
            uint16_t v = s->vram0[((sy + j) & 511u) * PSX_VRAM_W + ((sx + i) & 1023u)];
            uint16_t *d = &s->vram0[((dy + j) & 511u) * PSX_VRAM_W + ((dx + i) & 1023u)];
            if ((g->mask_bits & 2u) && (*d & 0x8000u)) continue;
            *d = (uint16_t)(v | ((g->mask_bits & 1u) ? 0x8000u : 0u));
        }
    mark_rows(s, dy, h);
    p = new_prim(s, PSX_PRIM_COPY);
    if (!p) return;
    p->x[0] = (int16_t)sx; p->y[0] = (int16_t)sy; p->x[1] = (int16_t)dx; p->y[1] = (int16_t)dy;
    p->w = (uint16_t)w; p->h = (uint16_t)h;
}

static void vram_put(hs_psx_t *s, uint16_t v)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t x = (g->xfer_x + g->xfer_i % g->xfer_w) & 1023u;
    uint32_t y = (g->xfer_y + g->xfer_i / g->xfer_w) & 511u;
    uint16_t *d = &s->vram0[y * PSX_VRAM_W + x];
    if (!((g->mask_bits & 2u) && (*d & 0x8000u)))
        *d = (uint16_t)(v | ((g->mask_bits & 1u) ? 0x8000u : 0u));
    g->xfer_i++;
}

/* number of words for a command byte (0 = single-word / env) */
static uint32_t cmd_len(uint32_t c)
{
    if (c >= 0x20u && c < 0x40u) {
        uint32_t nv = (c & 0x08u) ? 4u : 3u, n = 1u + nv;
        if (c & 0x04u) n += nv;
        if (c & 0x10u) n += nv - 1u;
        return n;
    }
    if (c >= 0x40u && c < 0x60u) {
        if (c & 0x08u) return (c & 0x10u) ? 4u : 3u;   /* polyline: first two vertices */
        return (c & 0x10u) ? 4u : 3u;
    }
    if (c >= 0x60u && c < 0x80u) {
        uint32_t n = 2u;
        if (c & 0x04u) n++;
        if (((c >> 3) & 3u) == 0u) n++;
        return n;
    }
    if (c == 0x02u) return 3u;
    if (c >= 0x80u && c < 0xA0u) return 4u;
    if (c >= 0xA0u && c < 0xE0u) return 3u;
    return 1u;
}

static void exec_cmd(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t w0 = g->fifo[0], c = w0 >> 24;
    if (c >= 0x20u && c < 0x40u) { do_polygon(s); return; }
    if (c >= 0x40u && c < 0x60u) {
        if (c & 0x10u) emit_line(s, c, g->fifo[1], w0, g->fifo[3], g->fifo[2]);
        else           emit_line(s, c, g->fifo[1], w0, g->fifo[2], w0);
        if (c & 0x08u) {       /* polyline continues */
            g->mode = 2;
            g->pl_cmd = c;
            g->pl_last_xy = (c & 0x10u) ? g->fifo[3] : g->fifo[2];
            g->pl_last_rgb = (c & 0x10u) ? g->fifo[2] : w0;
            g->pl_n = 0;
            g->pl_rgb_next = 0;
        }
        return;
    }
    if (c >= 0x60u && c < 0x80u) { do_rect(s); return; }
    if (c == 0x02u) { do_fill(s); return; }
    if (c >= 0x80u && c < 0xA0u) { do_copy(s); return; }
    if (c >= 0xA0u && c < 0xC0u) {
        g->xfer_x = g->fifo[1] & 0x3FFu; g->xfer_y = (g->fifo[1] >> 16) & 0x1FFu;
        g->xfer_w = ((g->fifo[2] - 1u) & 0x3FFu) + 1u;
        g->xfer_h = (((g->fifo[2] >> 16) - 1u) & 0x1FFu) + 1u;
        g->xfer_i = 0; g->xfer_n = g->xfer_w * g->xfer_h;
        mark_rows(s, g->xfer_y, g->xfer_h);
        g->mode = 1;
        return;
    }
    if (c >= 0xC0u && c < 0xE0u) {
        g->rd_x = g->fifo[1] & 0x3FFu; g->rd_y = (g->fifo[1] >> 16) & 0x1FFu;
        g->rd_w = ((g->fifo[2] - 1u) & 0x3FFu) + 1u;
        g->rd_h = (((g->fifo[2] >> 16) - 1u) & 0x1FFu) + 1u;
        g->rd_i = 0; g->rd_n = g->rd_w * g->rd_h; g->reading = 1;
        return;
    }
    switch (c) {
    case 0xE1: g->texpage = w0 & 0x3FFFu; break;
    case 0xE2: g->texwin = w0 & 0xFFFFFu; break;
    case 0xE3: g->da_x0 = (int32_t)(w0 & 0x3FFu); g->da_y0 = (int32_t)((w0 >> 10) & 0x3FFu); break;
    case 0xE4: g->da_x1 = (int32_t)(w0 & 0x3FFu); g->da_y1 = (int32_t)((w0 >> 10) & 0x3FFu); break;
    case 0xE5: g->off_x = sx11(w0); g->off_y = sx11(w0 >> 11); break;
    case 0xE6: g->mask_bits = w0 & 3u; break;
    default: break;    /* NOP / cache clear / IRQ request */
    }
}

void psx_gp0(hs_psx_t *s, uint32_t w)
{
    psx_gpu0_t *g = &s->gpu;
    if (g->mode == 1u) {
        vram_put(s, (uint16_t)w);
        if (g->xfer_i < g->xfer_n) vram_put(s, (uint16_t)(w >> 16));
        if (g->xfer_i >= g->xfer_n) g->mode = 0;
        return;
    }
    if (g->mode == 2u) {
        uint32_t c = g->pl_cmd;
        if ((w & 0xF000F000u) == 0x50005000u && (g->pl_n > 0 || !(c & 0x10u) || g->pl_rgb_next == 0)) {
            g->mode = 0;
            return;
        }
        if (c & 0x10u) {
            if (!g->pl_rgb_next) { g->pl_rgb_next = 1; g->pl_n = w | 0x80000000u; return; }
            g->pl_rgb_next = 0;
            emit_line(s, c, g->pl_last_xy, g->pl_last_rgb, w, g->pl_n & 0xFFFFFFu);
            g->pl_last_xy = w; g->pl_last_rgb = g->pl_n & 0xFFFFFFu;
            g->pl_n = 1;
        } else {
            emit_line(s, c, g->pl_last_xy, g->pl_last_rgb, w, g->pl_last_rgb);
            g->pl_last_xy = w;
            g->pl_n = 1;
        }
        return;
    }
    if (g->nfifo == 0) {
        g->need = cmd_len(w >> 24);
    }
    g->fifo[g->nfifo++] = w;
    if (g->nfifo >= g->need) {
        exec_cmd(s);
        g->nfifo = 0;
    }
}

void psx_gp1(hs_psx_t *s, uint32_t w)
{
    psx_gpu0_t *g = &s->gpu;
    uint32_t c = (w >> 24) & 0x3Fu;
    switch (c) {
    case 0x00: {
        uint32_t present_set = g->present_set, px = g->present_x, py = g->present_y;
        psx_gpu_reset(s);
        g->present_set = (uint8_t)present_set; g->present_x = (uint16_t)px; g->present_y = (uint16_t)py;
        } break;
    case 0x01: g->nfifo = 0; g->mode = 0; break;
    case 0x02: break;
    case 0x03: g->disp_off = w & 1u; break;
    case 0x04: g->dma_dir = w & 3u; break;
    case 0x05: g->disp_x = w & 0x3FEu; g->disp_y = (w >> 10) & 0x1FFu; break;
    case 0x06: g->h_range = w & 0xFFFFFFu; break;
    case 0x07: g->v_range = w & 0xFFFFFu; break;
    case 0x08: g->disp_mode = w & 0x7Fu; break;
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        switch (w & 7u) {
        case 2: g->gpuread = g->texwin; break;
        case 3: g->gpuread = (uint32_t)g->da_x0 | ((uint32_t)g->da_y0 << 10); break;
        case 4: g->gpuread = (uint32_t)g->da_x1 | ((uint32_t)g->da_y1 << 10); break;
        case 5: g->gpuread = ((uint32_t)g->off_x & 0x7FFu) | (((uint32_t)g->off_y & 0x7FFu) << 11); break;
        case 7: g->gpuread = 2; break;
        default: break;
        }
        break;
    default: break;
    }
}

uint32_t psx_gpuread(hs_psx_t *s)
{
    psx_gpu0_t *g = &s->gpu;
    if (g->reading) {
        uint32_t v = 0, k;
        for (k = 0; k < 2u; k++) {
            uint32_t x = (g->rd_x + g->rd_i % g->rd_w) & 1023u, y = (g->rd_y + g->rd_i / g->rd_w) & 511u;
            v |= (uint32_t)s->vram0[y * PSX_VRAM_W + x] << (16u * k);
            g->rd_i++;
        }
        if (g->rd_i >= g->rd_n) g->reading = 0;
        g->gpuread = v;
    }
    return g->gpuread;
}

uint32_t psx_gpustat(const hs_psx_t *s)
{
    const psx_gpu0_t *g = &s->gpu;
    uint32_t m = g->disp_mode, st;
    st = (g->texpage & 0x7FFu);                       /* 0..10 */
    st |= (g->mask_bits & 1u) << 11;
    st |= ((g->mask_bits >> 1) & 1u) << 12;
    st |= ((m >> 6) & 1u) << 16;
    st |= (m & 3u) << 17;
    st |= ((m >> 2) & 1u) << 19;
    st |= ((m >> 3) & 1u) << 20;
    st |= ((m >> 4) & 1u) << 21;
    st |= ((m >> 5) & 1u) << 22;
    st |= (g->disp_off & 1u) << 23;
    st |= (1u << 26) | (1u << 27) | (1u << 28);
    st |= (g->dma_dir & 3u) << 29;
    if (g->dma_dir == 1u || g->dma_dir == 2u || g->dma_dir == 3u) st |= 1u << 25;
    st |= (s->frame_cyc / 2146u & 1u) << 31;          /* odd/even line, advances with time */
    return st;
}

void psx_gpu_frame_begin(hs_psx_t *s)
{
    s->gpu.present_set = 0;     /* the list itself is emptied by commit_phase */
}

void psx_gpu_frame_end(hs_psx_t *s)
{
    psx_slot_t *b = psx_back(s);
    psx_gpu0_t *g = &s->gpu;
    static const uint16_t hres[4] = { 256, 320, 512, 640 };
    uint32_t m = g->disp_mode;
    b->disp_w = (m & 0x40u) ? 368u : hres[m & 3u];
    b->disp_h = ((m & 4u) && (m & 0x20u)) ? 480u : 240u;
    b->disp_24bpp = (uint8_t)((m >> 4) & 1u);
    b->disp_enabled = (uint8_t)!(g->disp_off & 1u);
    if (g->present_set && !b->disp_24bpp) { b->present_x = g->present_x; b->present_y = g->present_y; }
    else { b->present_x = (uint16_t)g->disp_x; b->present_y = (uint16_t)g->disp_y; }
}
