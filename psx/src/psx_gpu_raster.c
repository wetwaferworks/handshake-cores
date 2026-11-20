/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_gpu_raster.c - Phase 1 software PS1 GPU rasterizer.
 *
 * Reads ONLY the const FRONT slot (primitive list + 2D-transfer VRAM) and
 * writes ONLY video-private scratch + the caller framebuffer.
 *
 * One rasterizer serves every mode. The present rectangle of VRAM
 * (present_x, present_y, disp_w, disp_h) is mapped onto a virtual target
 * of tw x th pixels:
 *   native 1x : tw x th = disp_w x disp_h, colours quantized to 15 bit with
 *               the PS1 4x4 dither matrix, then the shared sharp scaler
 *               stretches edge-to-edge to 640x480 (+ optional FXAA),
 *   SSAA Nx   : tw x th = 640N x 480N, rendered in horizontal strips via the
 *               shared hs_ssaa_render (<= 320 KiB strip, L2 resident) and
 *               N x N box-filtered straight into the 640x480 frame.
 * Geometry is set up per strip in 28.4 fixed point with a top-left fill
 * rule (no double blending on shared edges), affine UV + Gouraud through
 * exact per-row plane evaluation. Integer only: bit-identical everywhere.
 * Bit 24 of a target pixel carries the PS1 mask bit.
 */
#include "psx_internal.h"

#include <string.h>

typedef struct rctx {
    const psx_slot_t *f;
    const uint16_t   *vram;
    uint32_t *buf;              /* strip memory; row r at buf + r*stride */
    int64_t   stride;
    int64_t   tw, th;           /* virtual target size */
    int64_t   y0, rows;         /* strip rows in target space */
    int64_t   px, py, dw, dh;   /* present rect (VRAM) */
    int       native;
    const uint16_t *colmap;     /* target column -> VRAM x (absolute) */
} rctx_t;

static inline int64_t fdiv(int64_t a, int64_t b)   /* floor(a/b), b > 0 */
{
    int64_t q = a / b;
    if ((a % b) != 0 && a < 0) q--;
    return q;
}
static inline int64_t cdiv(int64_t a, int64_t b) { return -fdiv(-a, b); }

/* half-pixel VRAM coordinate -> 28.4 target coordinate */
static inline int64_t map_x(const rctx_t *c, int64_t h) { return fdiv((h - 2 * c->px) * c->tw * 8, c->dw); }
static inline int64_t map_y(const rctx_t *c, int64_t h) { return fdiv((h - 2 * c->py) * c->th * 8, c->dh); }
/* first target pixel whose sample centre (16i+8) is >= a */
static inline int64_t pix_first(int64_t a) { return cdiv(a - 8, 16); }

static const int8_t k_dither[4][4] = {
    { -4,  0, -3,  1 }, {  2, -2,  3, -1 }, { -3,  1, -4,  0 }, {  3, -1,  2, -2 }
};

static inline uint32_t c5to8(uint32_t c) { return (c << 3) | (c >> 2); }
static inline uint32_t v15_to_px(uint16_t v)
{
    return (c5to8(v & 31u) << 16) | (c5to8((v >> 5) & 31u) << 8) | c5to8((v >> 10) & 31u) |
           ((uint32_t)(v >> 15) << 24);
}

static inline uint16_t tex_fetch(const rctx_t *c, const psx_gpu_prim_t *p, uint32_t u, uint32_t v)
{
    const uint16_t *vr = c->vram;
    uint32_t mx = (uint32_t)p->tw_mask_x * 8u, my = (uint32_t)p->tw_mask_y * 8u;
    u &= 0xFFu; v &= 0xFFu;
    u = (u & ~mx) | (((uint32_t)p->tw_off_x * 8u) & mx);
    v = (v & ~my) | (((uint32_t)p->tw_off_y * 8u) & my);
    {
        uint32_t row = ((uint32_t)p->tpage_y + v) & 511u;
        if (p->tex_depth == 0u) {
            uint16_t w = vr[row * 1024u + (((uint32_t)p->tpage_x + (u >> 2)) & 1023u)];
            uint32_t idx = (w >> ((u & 3u) * 4u)) & 0xFu;
            return vr[((uint32_t)p->clut_y & 511u) * 1024u + (((uint32_t)p->clut_x + idx) & 1023u)];
        } else if (p->tex_depth == 1u) {
            uint16_t w = vr[row * 1024u + (((uint32_t)p->tpage_x + (u >> 1)) & 1023u)];
            uint32_t idx = (w >> ((u & 1u) * 8u)) & 0xFFu;
            return vr[((uint32_t)p->clut_y & 511u) * 1024u + (((uint32_t)p->clut_x + idx) & 1023u)];
        }
        return vr[row * 1024u + (((uint32_t)p->tpage_x + u) & 1023u)];
    }
}

static inline uint32_t clamp255(int32_t v) { return v < 0 ? 0u : (v > 255 ? 255u : (uint32_t)v); }

/* Shade one target pixel. r,g,b are 8-bit vertex colours; tex = texel or 0xFFFFFFFF. */
static inline void put_px(const rctx_t *c, const psx_gpu_prim_t *p, uint32_t *d, int64_t x, int64_t y,
                          uint32_t r, uint32_t g, uint32_t b, uint32_t texel, int textured)
{
    uint32_t dst = *d, semi = (p->flags & PSX_PRIM_SEMI) != 0, mbit = 0;
    if ((p->flags & PSX_PRIM_MASK_CHK) && (dst & 0x01000000u)) return;
    if (textured) {
        uint32_t tr, tg, tb;
        if (texel == 0u) return;                 /* fully transparent texel */
        semi = semi && (texel & 0x8000u);
        mbit = texel >> 15;
        tr = c5to8(texel & 31u); tg = c5to8((texel >> 5) & 31u); tb = c5to8((texel >> 10) & 31u);
        if (!(p->flags & PSX_PRIM_RAW)) {
            tr = (tr * r) >> 7; tg = (tg * g) >> 7; tb = (tb * b) >> 7;
            if (tr > 255u) tr = 255u;
            if (tg > 255u) tg = 255u;
            if (tb > 255u) tb = 255u;
        }
        r = tr; g = tg; b = tb;
    }
    if (semi) {
        int32_t br = (int32_t)((dst >> 16) & 0xFFu), bg = (int32_t)((dst >> 8) & 0xFFu), bb = (int32_t)(dst & 0xFFu);
        switch (p->abr) {
        case 0: r = (uint32_t)(br + (int32_t)r) >> 1; g = (uint32_t)(bg + (int32_t)g) >> 1; b = (uint32_t)(bb + (int32_t)b) >> 1; break;
        case 1: r = clamp255(br + (int32_t)r); g = clamp255(bg + (int32_t)g); b = clamp255(bb + (int32_t)b); break;
        case 2: r = clamp255(br - (int32_t)r); g = clamp255(bg - (int32_t)g); b = clamp255(bb - (int32_t)b); break;
        default: r = clamp255(br + (int32_t)(r >> 2)); g = clamp255(bg + (int32_t)(g >> 2)); b = clamp255(bb + (int32_t)(b >> 2)); break;
        }
    }
    if (c->native) {
        if (p->flags & PSX_PRIM_DITHER) {
            int32_t dv = k_dither[y & 3][x & 3];
            r = clamp255((int32_t)r + dv); g = clamp255((int32_t)g + dv); b = clamp255((int32_t)b + dv);
        }
        r = c5to8(r >> 3); g = c5to8(g >> 3); b = c5to8(b >> 3);
    }
    if (p->flags & PSX_PRIM_MASK_SET) mbit = 1u;
    *d = (r << 16) | (g << 8) | b | (mbit << 24);
}

/* target-space clip window for a prim: [cx0,cx1) x [cy0,cy1) */
static void clip_window(const rctx_t *c, const psx_gpu_prim_t *p, int64_t *cx0, int64_t *cx1,
                        int64_t *cy0, int64_t *cy1)
{
    int64_t a = pix_first(map_x(c, 2 * (int64_t)p->clip_x0));
    int64_t b = pix_first(map_x(c, 2 * ((int64_t)p->clip_x1 + 1)));
    int64_t e = pix_first(map_y(c, 2 * (int64_t)p->clip_y0));
    int64_t f = pix_first(map_y(c, 2 * ((int64_t)p->clip_y1 + 1)));
    if (a < 0) a = 0;
    if (b > c->tw) b = c->tw;
    if (e < c->y0) e = c->y0;
    if (f > c->y0 + c->rows) f = c->y0 + c->rows;
    if (f > c->th) f = c->th;
    *cx0 = a; *cx1 = b; *cy0 = e; *cy1 = f;
}

/* Rasterize a triangle given in half-pixel VRAM units. */
static void raster_tri(const rctx_t *c, const psx_gpu_prim_t *p, const int64_t *hx, const int64_t *hy,
                       const uint32_t *rgb, const uint8_t *uu, const uint8_t *vv, int64_t cx0, int64_t cx1,
                       int64_t cy0, int64_t cy1)
{
    int64_t X[3], Y[3], area, A[3], B[3], C[3], bias[3];
    int64_t ylo, yhi, xlo, xhi, y;
    int64_t attr[3][5], dadx[5];
    int textured = (p->flags & PSX_PRIM_TEXTURED) != 0;
    int gour = (p->flags & PSX_PRIM_GOURAUD) != 0;
    int na = textured ? 5 : 3, k, j;
    static const int e0[3] = { 1, 2, 0 }, e1[3] = { 2, 0, 1 };  /* edge k opposite vertex k */
    for (k = 0; k < 3; k++) { X[k] = map_x(c, hx[k]); Y[k] = map_y(c, hy[k]); }
    area = (X[1] - X[0]) * (Y[2] - Y[0]) - (Y[1] - Y[0]) * (X[2] - X[0]);
    if (area == 0) return;
    {   /* y range */
        int64_t mn = Y[0], mx = Y[0];
        for (k = 1; k < 3; k++) { if (Y[k] < mn) mn = Y[k]; if (Y[k] > mx) mx = Y[k]; }
        ylo = pix_first(mn); yhi = pix_first(mx + 1);
        if (ylo < cy0) ylo = cy0;
        if (yhi > cy1) yhi = cy1;
        if (ylo >= yhi) return;
        mn = X[0]; mx = X[0];
        for (k = 1; k < 3; k++) { if (X[k] < mn) mn = X[k]; if (X[k] > mx) mx = X[k]; }
        xlo = pix_first(mn); xhi = pix_first(mx + 1);
        if (xlo < cx0) xlo = cx0;
        if (xhi > cx1) xhi = cx1;
        if (xlo >= xhi) return;
    }
    /* edge k (from vertex e0[k] to e1[k]) weights vertex k; e = (xb-xa)(sy-ya) - (yb-ya)(sx-xa) */
    for (k = 0; k < 3; k++) {
        int64_t xa = X[e0[k]], ya = Y[e0[k]], xb = X[e1[k]], yb = Y[e1[k]];
        int64_t dx = xb - xa, dy = yb - ya;
        if (area < 0) { dx = -dx; dy = -dy; }
        /* e(sx,sy) = dx*(sy-ya) - dy*(sx-xa) ; sx = 16i+8 */
        B[k] = -dy * 16;                     /* per target pixel in x */
        A[k] = dy * xa - dy * 8 - dx * ya;   /* constant part; + dx*sy per row */
        C[k] = dx;
        bias[k] = (dy > 0 || (dy == 0 && dx < 0)) ? 0 : 1;
    }
    if (area < 0) area = -area;
    /* attributes: r g b u v */
    for (k = 0; k < 3; k++) {
        attr[k][0] = rgb[k] & 0xFFu; attr[k][1] = (rgb[k] >> 8) & 0xFFu; attr[k][2] = (rgb[k] >> 16) & 0xFFu;
        attr[k][3] = uu[k]; attr[k][4] = vv[k];
        if (!gour) { attr[k][0] = rgb[0] & 0xFFu; attr[k][1] = (rgb[0] >> 8) & 0xFFu; attr[k][2] = (rgb[0] >> 16) & 0xFFu; }
    }
    for (j = 0; j < na; j++)
        dadx[j] = ((B[0] * attr[0][j] + B[1] * attr[1][j] + B[2] * attr[2][j]) * 65536) / area;

    for (y = ylo; y < yhi; y++) {
        int64_t sy = 16 * y + 8, lo = xlo, hi = xhi - 1, e[3], x;
        uint32_t *row;
        int64_t a[5];
        for (k = 0; k < 3; k++) {
            int64_t e0v = A[k] + C[k] * sy;         /* e at i = 0 */
            int64_t t = e0v - bias[k];
            e[k] = e0v;
            if (B[k] > 0) { int64_t m = cdiv(-t, B[k]); if (m > lo) lo = m; }
            else if (B[k] < 0) { int64_t m = fdiv(t, -B[k]); if (m < hi) hi = m; }
            else if (t < 0) { lo = 1; hi = 0; }
        }
        if (lo > hi) continue;
        for (j = 0; j < na; j++) {
            int64_t num = 0;
            for (k = 0; k < 3; k++) num += (e[k] + B[k] * lo) * attr[k][j];
            a[j] = ((num * 256) / area) * 256;
        }
        row = c->buf + (y - c->y0) * c->stride;
        for (x = lo; x <= hi; x++) {
            uint32_t r = clamp255((int32_t)(a[0] >> 16)), g = clamp255((int32_t)(a[1] >> 16));
            uint32_t b = clamp255((int32_t)(a[2] >> 16)), tx = 0;
            if (textured)
                tx = tex_fetch(c, p, clamp255((int32_t)(a[3] >> 16)), clamp255((int32_t)(a[4] >> 16)));
            put_px(c, p, row + x, x, y, r, g, b, tx, textured);
            for (j = 0; j < na; j++) a[j] += dadx[j];
        }
    }
}

static void draw_tri(const rctx_t *c, const psx_gpu_prim_t *p)
{
    int64_t hx[3], hy[3], cx0, cx1, cy0, cy1;
    int k;
    clip_window(c, p, &cx0, &cx1, &cy0, &cy1);
    if (cx0 >= cx1 || cy0 >= cy1) return;
    for (k = 0; k < 3; k++) { hx[k] = 2 * (int64_t)p->x[k]; hy[k] = 2 * (int64_t)p->y[k]; }
    raster_tri(c, p, hx, hy, p->rgb, p->u, p->v, cx0, cx1, cy0, cy1);
}

static void draw_line(const rctx_t *c, const psx_gpu_prim_t *p)
{
    int64_t cx0, cx1, cy0, cy1, x0, y0, x1, y1, dx, dy;
    int64_t qx[4], qy[4], tx[3], ty[3];
    uint32_t qc[4], tc[3];
    uint8_t z[3] = { 0, 0, 0 };
    psx_gpu_prim_t q = *p;
    clip_window(c, p, &cx0, &cx1, &cy0, &cy1);
    if (cx0 >= cx1 || cy0 >= cy1) return;
    q.flags = (uint8_t)(q.flags & ~PSX_PRIM_TEXTURED);
    q.flags |= PSX_PRIM_GOURAUD;
    /* half-pixel units, centre of pixel (x,y) is 2x+1 */
    x0 = 2 * (int64_t)p->x[0] + 1; y0 = 2 * (int64_t)p->y[0] + 1;
    x1 = 2 * (int64_t)p->x[1] + 1; y1 = 2 * (int64_t)p->y[1] + 1;
    dx = x1 - x0; dy = y1 - y0;
    if ((dx < 0 ? -dx : dx) >= (dy < 0 ? -dy : dy)) {
        int64_t s = dx >= 0 ? 1 : -1;
        x0 -= s; x1 += s;
        qx[0] = x0; qy[0] = y0 - 1; qx[1] = x0; qy[1] = y0 + 1;
        qx[2] = x1; qy[2] = y1 - 1; qx[3] = x1; qy[3] = y1 + 1;
    } else {
        int64_t s = dy >= 0 ? 1 : -1;
        y0 -= s; y1 += s;
        qx[0] = x0 - 1; qy[0] = y0; qx[1] = x0 + 1; qy[1] = y0;
        qx[2] = x1 - 1; qy[2] = y1; qx[3] = x1 + 1; qy[3] = y1;
    }
    qc[0] = qc[1] = p->rgb[0]; qc[2] = qc[3] = p->rgb[1];
    tx[0] = qx[0]; ty[0] = qy[0]; tx[1] = qx[1]; ty[1] = qy[1]; tx[2] = qx[2]; ty[2] = qy[2];
    tc[0] = qc[0]; tc[1] = qc[1]; tc[2] = qc[2];
    raster_tri(c, &q, tx, ty, tc, z, z, cx0, cx1, cy0, cy1);
    tx[0] = qx[1]; ty[0] = qy[1]; tx[1] = qx[2]; ty[1] = qy[2]; tx[2] = qx[3]; ty[2] = qy[3];
    tc[0] = qc[1]; tc[1] = qc[2]; tc[2] = qc[3];
    raster_tri(c, &q, tx, ty, tc, z, z, cx0, cx1, cy0, cy1);
}

static void draw_rect(const rctx_t *c, const psx_gpu_prim_t *p)
{
    int64_t cx0, cx1, cy0, cy1, xa, xb, ya, yb, x, y;
    int64_t du, dv, u0, v0;
    int textured = (p->flags & PSX_PRIM_TEXTURED) != 0;
    uint32_t r = p->rgb[0] & 0xFFu, g = (p->rgb[0] >> 8) & 0xFFu, b = (p->rgb[0] >> 16) & 0xFFu;
    clip_window(c, p, &cx0, &cx1, &cy0, &cy1);
    xa = pix_first(map_x(c, 2 * (int64_t)p->x[0]));
    xb = pix_first(map_x(c, 2 * ((int64_t)p->x[0] + p->w)));
    ya = pix_first(map_y(c, 2 * (int64_t)p->y[0]));
    yb = pix_first(map_y(c, 2 * ((int64_t)p->y[0] + p->h)));
    if (xa < cx0) xa = cx0;
    if (xb > cx1) xb = cx1;
    if (ya < cy0) ya = cy0;
    if (yb > cy1) yb = cy1;
    if (xa >= xb || ya >= yb) return;
    /* texel coordinate of a sample centre, 16.16: vram = px + (16i+8)*dw/(16tw) */
    du = (c->dw * 65536) / c->tw;
    dv = (c->dh * 65536) / c->th;
    u0 = ((2 * c->px * c->tw + (2 * xa + 1) * c->dw) * 32768) / c->tw - (int64_t)p->x[0] * 65536;
    v0 = ((2 * c->py * c->th + (2 * ya + 1) * c->dh) * 32768) / c->th - (int64_t)p->y[0] * 65536;
    for (y = ya; y < yb; y++) {
        uint32_t *row = c->buf + (y - c->y0) * c->stride;
        int64_t vf = v0 + (y - ya) * dv, uf = u0;
        uint32_t tv = (uint32_t)(p->v[0] + (vf >> 16));
        for (x = xa; x < xb; x++) {
            uint32_t tx = 0;
            if (textured) tx = tex_fetch(c, p, (uint32_t)(p->u[0] + (uf >> 16)), tv);
            put_px(c, p, row + x, x, y, r, g, b, tx, textured);
            uf += du;
        }
    }
}

static void draw_fill(const rctx_t *c, const psx_gpu_prim_t *p)
{
    int64_t xa = pix_first(map_x(c, 2 * (int64_t)p->x[0]));
    int64_t xb = pix_first(map_x(c, 2 * ((int64_t)p->x[0] + p->w)));
    int64_t ya = pix_first(map_y(c, 2 * (int64_t)p->y[0]));
    int64_t yb = pix_first(map_y(c, 2 * ((int64_t)p->y[0] + p->h)));
    uint32_t col = p->rgb[0], r = col & 0xFFu, g = (col >> 8) & 0xFFu, b = (col >> 16) & 0xFFu, px, y, x;
    r = c5to8(r >> 3); g = c5to8(g >> 3); b = c5to8(b >> 3);   /* fills are 15-bit in VRAM */
    px = (r << 16) | (g << 8) | b;
    if (xa < 0) xa = 0;
    if (xb > c->tw) xb = c->tw;
    if (ya < c->y0) ya = c->y0;
    if (yb > c->y0 + c->rows) yb = c->y0 + c->rows;
    for (y = (uint32_t)ya; (int64_t)y < yb; y++) {
        uint32_t *row = c->buf + ((int64_t)y - c->y0) * c->stride;
        for (x = (uint32_t)xa; (int64_t)x < xb; x++) row[x] = px;
    }
}

static void draw_copy(const rctx_t *c, const psx_gpu_prim_t *p)
{
    /* destination already holds the copied texels in the 2D VRAM snapshot */
    int64_t xa = pix_first(map_x(c, 2 * (int64_t)p->x[1]));
    int64_t xb = pix_first(map_x(c, 2 * ((int64_t)p->x[1] + p->w)));
    int64_t ya = pix_first(map_y(c, 2 * (int64_t)p->y[1]));
    int64_t yb = pix_first(map_y(c, 2 * ((int64_t)p->y[1] + p->h)));
    int64_t x, y;
    if (xa < 0) xa = 0;
    if (xb > c->tw) xb = c->tw;
    if (ya < c->y0) ya = c->y0;
    if (yb > c->y0 + c->rows) yb = c->y0 + c->rows;
    for (y = ya; y < yb; y++) {
        uint32_t *row = c->buf + (y - c->y0) * c->stride;
        uint32_t vy = (uint32_t)(c->py + (y * c->dh) / c->th) & 511u;
        for (x = xa; x < xb; x++)
            row[x] = v15_to_px(c->vram[vy * 1024u + c->colmap[x]]);
    }
}

static void draw_base(const rctx_t *c)
{
    const psx_slot_t *f = c->f;
    int64_t y, x;
    for (y = 0; y < c->rows && c->y0 + y < c->th; y++) {
        uint32_t *row = c->buf + y * c->stride;
        uint32_t vy = (uint32_t)(c->py + ((c->y0 + y) * c->dh) / c->th) & 511u;
        const uint16_t *vr = c->vram + vy * 1024u;
        if (!f->disp_enabled) { for (x = 0; x < c->tw; x++) row[x] = 0; continue; }
        if (f->disp_24bpp) {
            for (x = 0; x < c->tw; x++) {
                uint32_t n = (uint32_t)((x * c->dw) / c->tw);                /* 24-bit pixel index */
                uint32_t bo = (uint32_t)c->px * 2u + n * 3u, k, rgbv = 0;
                for (k = 0; k < 3u; k++) {
                    uint32_t bb = bo + k, w = vr[(bb >> 1) & 1023u];
                    rgbv |= ((bb & 1u) ? (w >> 8) : (w & 0xFFu)) << (8u * k);
                }
                row[x] = ((rgbv & 0xFFu) << 16) | (rgbv & 0xFF00u) | ((rgbv >> 16) & 0xFFu);
            }
        } else {
            for (x = 0; x < c->tw; x++) row[x] = v15_to_px(vr[c->colmap[x]]);
        }
    }
}

static int fill_covers(const rctx_t *c, const psx_gpu_prim_t *p)
{
    return p->type == PSX_PRIM_FILL && p->x[0] <= c->px && p->y[0] <= c->py &&
           (int64_t)p->x[0] + p->w >= c->px + c->dw && (int64_t)p->y[0] + p->h >= c->py + c->dh;
}

static void draw_all(rctx_t *c)
{
    const psx_slot_t *f = c->f;
    uint32_t i, start = 0;
    if (f->disp_enabled && f->nprims && fill_covers(c, &f->prims[0]) && !f->disp_24bpp) {
        draw_fill(c, &f->prims[0]);
        start = 1;
    } else {
        draw_base(c);
    }
    if (!f->disp_enabled || f->disp_24bpp) return;
    for (i = start; i < f->nprims; i++) {
        const psx_gpu_prim_t *p = &f->prims[i];
        switch (p->type) {
        case PSX_PRIM_TRI:  draw_tri(c, p); break;
        case PSX_PRIM_RECT: draw_rect(c, p); break;
        case PSX_PRIM_LINE: draw_line(c, p); break;
        case PSX_PRIM_FILL: draw_fill(c, p); break;
        case PSX_PRIM_COPY: draw_copy(c, p); break;
        default: break;
        }
    }
}

static void strip_cb(void *user, uint32_t *strip, uint32_t strip_w, uint32_t y0_hi, uint32_t rows_hi, uint32_t ss)
{
    rctx_t *c = (rctx_t *)user;
    (void)ss;
    c->buf = strip;
    c->stride = strip_w;
    c->y0 = y0_hi;
    c->rows = rows_hi;
    draw_all(c);
}

static void ctx_init(hs_psx_t *s, rctx_t *c, int64_t tw, int64_t th)
{
    const psx_slot_t *f = psx_front(s);
    int64_t x;
    c->f = f;
    c->vram = f->vram;
    c->px = f->present_x & 1023u;
    c->py = f->present_y & 511u;
    c->dw = f->disp_w ? f->disp_w : 320;
    c->dh = f->disp_h ? f->disp_h : 240;
    c->tw = tw; c->th = th;
    for (x = 0; x < tw; x++)
        s->vid.colmap[x] = (uint16_t)((c->px + (x * c->dw) / tw) & 1023);
    c->colmap = s->vid.colmap;
}

void psx_raster_frame(hs_psx_t *s, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    psx_video_priv_t *v = &s->vid;
    rctx_t c;
    uint32_t ss = hs_ssaa_factor(mode);
    int fxaa = (mode == HS_AA_FXAA) || (ss > 1u && v->post_fxaa);
    uint32_t *dst = fxaa ? v->tmp640 : fb;
    size_t dst_stride = fxaa ? HS_FB_W : stride;
    memset(&c, 0, sizeof(c));
    if (ss > 1u) {
        ctx_init(s, &c, (int64_t)HS_FB_W * ss, (int64_t)HS_FB_H * ss);
        c.native = 0;
        hs_ssaa_render(ss, strip_cb, &c, v->strip, dst, dst_stride);
    } else {
        const psx_slot_t *f = psx_front(s);
        uint32_t dw = f->disp_w ? f->disp_w : 320u, dh = f->disp_h ? f->disp_h : 240u, i;
        ctx_init(s, &c, dw, dh);
        c.native = 1;
        c.buf = v->native;
        c.stride = dw;
        c.y0 = 0;
        c.rows = dh;
        draw_all(&c);
        for (i = 0; i < dw * dh; i++) v->native[i] &= 0x00FFFFFFu;
        hs_scaler_init(&v->scaler, dw, dh, mode == HS_SCALE_STRETCH_NEAREST ? HS_SCALE_STRETCH_NEAREST
                                                                              : HS_SCALE_STRETCH_SHARP);
        hs_scaler_run(&v->scaler, v->native, dw, dst, dst_stride, v->row_tmp);
    }
    if (fxaa)
        hs_fxaa(v->tmp640, HS_FB_W, fb, stride, HS_FB_W, HS_FB_H, v->luma);
}
