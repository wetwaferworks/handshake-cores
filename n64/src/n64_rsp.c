/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_rsp.c - clean-room RSP task dispatch for hs-n64 (Phase 0 only).
 *   - OSTask decode from DMEM 0xFC0 and dispatch:
 *       type 1 : HLE display-list engine (Fast3D / F3DEX-compatible command
 *                encodings: G_MTX, G_VTX, G_TRI1/2, G_DL, G_MOVEMEM viewport
 *                + lights, G_MOVEWORD segments/num lights/fog, geometry and
 *                other modes, G_TEXTURE, texture rectangles, RDP pass-through)
 *       type 2 : HLE audio engine (HandshakeOS "HS-ABI": order-2 predictive
 *                4-bit ADPCM, linear resampler, volume/pan, mixer, interleave)
 *       other  : scalar-unit LLE interpreter over IMEM/DMEM (no vector unit)
 *   - RDP front end: every RDP command (from the display list or from the
 *     DP_START/DP_END registers) is appended to a bounded command list in
 *     the BACK snapshot; texture loads are resolved out of RDRAM into a
 *     snapshot-local texture blob so Phase 1 never touches RDRAM.
 * All matrix / vertex / lighting math is integer fixed-point (s15.16).
 */
#include <string.h>
#include "n64_internal.h"

/* ------------------------------------------------------------ helpers */
static uint32_t seg_addr(const hs_n64_t *n, uint32_t a)
{
    return (n->gfx.seg[(a >> 24) & 15u] + (a & 0x00FFFFFFu)) & 0x00FFFFFFu;
}

static uint32_t rd32(const hs_n64_t *n, uint32_t a)
{
    a &= 0x00FFFFFCu;
    return (a + 4u <= n->rdram_size) ? n64_be32(n->rdram + a) : 0u;
}

static int16_t rd16s(const hs_n64_t *n, uint32_t a)
{
    a &= 0x00FFFFFEu;
    return (a + 2u <= n->rdram_size) ? (int16_t)n64_be16(n->rdram + a) : 0;
}

static uint8_t rd8(const hs_n64_t *n, uint32_t a)
{
    a &= 0x00FFFFFFu;
    return a < n->rdram_size ? n->rdram[a] : 0u;
}

static int32_t clamp32(int64_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : (int32_t)v);
}

uint32_t n64_isqrt64(uint64_t v)
{
    uint64_t r = 0, bit = 1ull << 62;
    uint32_t i;
    for (i = 0; i < 32u && bit > v; i++) bit >>= 2;
    for (i = 0; i < 32u && bit; i++) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* ----------------------------------------------------- matrix maths */
void n64_rsp_load_mtx(const uint8_t *src, int32_t *out)
{
    uint32_t i;
    for (i = 0; i < 16u; i++) {
        int32_t hi = (int16_t)n64_be16(src + i * 2u);
        uint32_t lo = n64_be16(src + 32u + i * 2u);
        out[i] = (int32_t)(((uint32_t)hi << 16) | lo);
    }
}

void n64_rsp_matmul(const int32_t *a, const int32_t *b, int32_t *out)
{
    int32_t t[16];
    uint32_t i, j, k;
    for (i = 0; i < 4u; i++)
        for (j = 0; j < 4u; j++) {
            int64_t s = 0;
            for (k = 0; k < 4u; k++) s += (int64_t)a[i * 4u + k] * b[k * 4u + j];
            t[i * 4u + j] = (int32_t)(s >> 16);
        }
    memcpy(out, t, sizeof(t));
}

/* row vector (x y z 1) * m, inputs integer model units, output s15.16 */
void n64_rsp_xform(const int32_t *m, int32_t x, int32_t y, int32_t z, int32_t *o)
{
    uint32_t j;
    for (j = 0; j < 4u; j++) {
        int64_t s = (int64_t)x * m[j] + (int64_t)y * m[4u + j] + (int64_t)z * m[8u + j] + m[12u + j];
        o[j] = clamp32(s, INT32_MIN, INT32_MAX);
    }
}

uint32_t n64_rsp_light(const n64_gfx_t *g, const int32_t *mv, int8_t nx, int8_t ny, int8_t nz)
{
    int64_t tx = ((int64_t)nx * mv[0] + (int64_t)ny * mv[4] + (int64_t)nz * mv[8]) >> 8;
    int64_t ty = ((int64_t)nx * mv[1] + (int64_t)ny * mv[5] + (int64_t)nz * mv[9]) >> 8;
    int64_t tz = ((int64_t)nx * mv[2] + (int64_t)ny * mv[6] + (int64_t)nz * mv[10]) >> 8;
    uint64_t l2 = (uint64_t)(tx * tx + ty * ty + tz * tz);
    int64_t len = (int64_t)n64_isqrt64(l2);
    int32_t acc[3];
    uint32_t i, k, nl = g->num_lights > 7u ? 7u : g->num_lights;
    for (k = 0; k < 3u; k++) acc[k] = (int32_t)g->light_col[nl][k] << 8;   /* ambient */
    if (len == 0) len = 1;
    for (i = 0; i < nl; i++) {
        int64_t d = tx * g->light_dir[i][0] + ty * g->light_dir[i][1] + tz * g->light_dir[i][2];
        int64_t ld = (int64_t)n64_isqrt64((uint64_t)((int32_t)g->light_dir[i][0] * g->light_dir[i][0] +
                                                     (int32_t)g->light_dir[i][1] * g->light_dir[i][1] +
                                                     (int32_t)g->light_dir[i][2] * g->light_dir[i][2]));
        int64_t f;
        if (d <= 0 || ld == 0) continue;
        f = (d << 8) / (len * ld);                  /* 0..256 */
        if (f > 256) f = 256;
        for (k = 0; k < 3u; k++) acc[k] += (int32_t)(g->light_col[i][k] * f);
    }
    for (k = 0; k < 3u; k++) acc[k] = clamp32(acc[k] >> 8, 0, 255);
    return ((uint32_t)acc[0] << 24) | ((uint32_t)acc[1] << 16) | ((uint32_t)acc[2] << 8);
}

/* -------------------------------------------------------- RDP front end */
static void emit(hs_n64_t *n, const uint32_t *w, uint32_t cnt)
{
    n64_snap_t *b = n64_back(n);
    if (b->rdp_n + cnt > N64_RDP_MAX_WORDS) { b->overflow++; return; }
    memcpy(&b->rdp[b->rdp_n], w, cnt * 4u);
    b->rdp_n += cnt;
}

static uint32_t tex_bytes(uint32_t texels, uint32_t siz) { return (texels * (4u << siz) + 7u) >> 3; }

static void emit_tex_load(hs_n64_t *n, const uint32_t *w, uint32_t row0, uint32_t start, uint32_t len)
{
    n64_snap_t *b = n64_back(n);
    n64_gfx_t *g = &n->gfx;
    uint32_t hdr[4], i, src = g->timg_addr + start;
    if (len > 4096u + 1024u) len = 4096u + 1024u;
    if (b->tex_n + len > N64_TEXBLOB_MAX) { b->overflow++; return; }
    for (i = 0; i < len; i++) b->tex[b->tex_n + i] = rd8(n, src + i);
    hdr[0] = (N64_OP_HS_TIMG << 24) | (g->timg_fmt << 21) | (g->timg_siz << 19) | ((g->timg_w - 1u) & 0x3FFu);
    hdr[1] = b->tex_n;
    hdr[2] = row0;
    hdr[3] = len;
    b->tex_n += (len + 7u) & ~7u;
    if (b->tex_n > N64_TEXBLOB_MAX) b->tex_n = N64_TEXBLOB_MAX;
    emit(n, hdr, 4);
    emit(n, w, 2);
}

void n64_rdp_ingest(hs_n64_t *n, const uint32_t *w, uint32_t nwords)
{
    n64_gfx_t *g = &n->gfx;
    n64_snap_t *b = n64_back(n);
    uint32_t op = (w[0] >> 24) & 0x3Fu;
    uint32_t tmp[2];
    switch (op) {
    case 0x3D:                                   /* Set Texture Image */
        g->timg_fmt = (w[0] >> 21) & 7u;
        g->timg_siz = (w[0] >> 19) & 3u;
        g->timg_w = (w[0] & 0x3FFu) + 1u;
        g->timg_addr = w[1] & 0x00FFFFFFu;
        return;
    case 0x33: {                                 /* Load Block */
        uint32_t sl = (w[0] >> 12) & 0xFFFu, tl = w[0] & 0xFFFu, sh = (w[1] >> 12) & 0xFFFu;
        uint32_t texels = (sh >= sl ? sh - sl : 0u) + 1u;
        emit_tex_load(n, w, tl, tex_bytes(tl * g->timg_w, g->timg_siz), tex_bytes(sl + texels, g->timg_siz));
        return;
    }
    case 0x30: {                                 /* Load TLUT */
        uint32_t tl = (w[0] & 0xFFFu) >> 2, sh = ((w[1] >> 12) & 0xFFFu) >> 2;
        emit_tex_load(n, w, tl, tex_bytes(tl * g->timg_w, g->timg_siz), tex_bytes(sh + 1u, g->timg_siz));
        return;
    }
    case 0x34: {                                 /* Load Tile */
        uint32_t tl = (w[0] & 0xFFFu) >> 2, th = (w[1] & 0xFFFu) >> 2;
        uint32_t rows = (th >= tl ? th - tl : 0u) + 1u;
        emit_tex_load(n, w, tl, tex_bytes(tl * g->timg_w, g->timg_siz), rows * tex_bytes(g->timg_w, g->timg_siz));
        return;
    }
    case 0x3F:                                   /* Set Color Image */
        g->cimg_w = (w[0] & 0xFFFu) + 1u;
        tmp[0] = w[0]; tmp[1] = w[1] & 0x00FFFFFFu;
        emit(n, tmp, 2);
        if (b->native_w == 0) b->native_w = g->cimg_w;
        return;
    case 0x3E:                                   /* Set Z Image */
        tmp[0] = w[0]; tmp[1] = w[1] & 0x00FFFFFFu;
        emit(n, tmp, 2);
        return;
    case 0x2D:                                   /* Set Scissor */
        g->sc_x1 = (int32_t)((w[1] >> 12) & 0xFFFu);
        g->sc_y1 = (int32_t)(w[1] & 0xFFFu);
        b->native_w = (uint32_t)g->sc_x1 >> 2;
        b->native_h = (uint32_t)g->sc_y1 >> 2;
        emit(n, w, 2);
        return;
    case 0x29:                                   /* Full Sync */
        n->dp_status &= ~0x70u;
        n64_mi_raise(n, MI_DP);
        return;
    case 0x26: case 0x27: case 0x28:             /* load/pipe/tile sync */
        return;
    case 0x24: case 0x25:                        /* texture rectangle (4 words) */
        if (nwords >= 4u) emit(n, w, 4);
        return;
    default:
        if (op >= 0x08u && op <= 0x0Fu) { b->overflow += 0; return; }  /* edge-coefficient triangles: not consumed */
        if (op < 0x08u) return;                  /* no-op */
        emit(n, w, 2);
        return;
    }
}

/* DP_START/DP_END direct command buffer pass-through (RDRAM source) */
void n64_dp_run(hs_n64_t *n)
{
    uint32_t a = n->dp_current, guard;
    for (guard = 0; guard < 65536u && a < n->dp_end; guard++) {
        uint32_t w[4], op, len;
        w[0] = rd32(n, a); w[1] = rd32(n, a + 4u);
        op = (w[0] >> 24) & 0x3Fu;
        len = 8u;
        if (op >= 0x08u && op <= 0x0Fu) {
            len = 32u + ((op & 4u) ? 64u : 0u) + ((op & 2u) ? 64u : 0u) + ((op & 1u) ? 16u : 0u);
        } else if (op == 0x24u || op == 0x25u) {
            len = 16u;
            w[2] = rd32(n, a + 8u); w[3] = rd32(n, a + 12u);
        }
        n64_rdp_ingest(n, w, len == 16u ? 4u : 2u);
        a += len;
    }
    n->dp_current = a;
    n->dp_status |= 0x80u;   /* cbuf ready */
}

/* -------------------------------------------------------- HLE gfx */
#define G_ZBUFFER        0x00000001u
#define G_SHADE          0x00000004u
#define G_SHADING_SMOOTH 0x00000200u
#define G_CULL_FRONT     0x00001000u
#define G_CULL_BACK      0x00002000u
#define G_FOG            0x00010000u
#define G_LIGHTING       0x00020000u

static void update_mvp(n64_gfx_t *g)
{
    if (!g->mvp_valid) { n64_rsp_matmul(g->mv[g->mv_top], g->proj, g->mvp); g->mvp_valid = 1; }
}

static void load_vertices(hs_n64_t *n, uint32_t addr, uint32_t v0, uint32_t cnt)
{
    n64_gfx_t *g = &n->gfx;
    uint32_t i;
    update_mvp(g);
    for (i = 0; i < cnt && v0 + i < 64u; i++) {
        uint32_t a = addr + i * 16u;
        n64_vtx_t *v = &g->vbuf[v0 + i];
        int32_t c[4];
        int16_t x = rd16s(n, a), y = rd16s(n, a + 2u), z = rd16s(n, a + 4u);
        int16_t s = rd16s(n, a + 8u), t = rd16s(n, a + 10u);
        uint8_t cr = rd8(n, a + 12u), cg = rd8(n, a + 13u), cb = rd8(n, a + 14u), ca = rd8(n, a + 15u);
        n64_rsp_xform(g->mvp, x, y, z, c);
        v->cx = c[0]; v->cy = c[1]; v->cz = c[2]; v->cw = c[3];
        v->clip = 0;
        if (c[0] < -c[3]) v->clip |= 1u;
        if (c[0] > c[3])  v->clip |= 2u;
        if (c[1] < -c[3]) v->clip |= 4u;
        if (c[1] > c[3])  v->clip |= 8u;
        if (c[2] < -c[3]) v->clip |= 16u;
        if (g->geom & G_LIGHTING) {
            uint32_t col = n64_rsp_light(g, g->mv[g->mv_top], (int8_t)cr, (int8_t)cg, (int8_t)cb);
            v->r = (uint8_t)(col >> 24); v->g = (uint8_t)(col >> 16); v->b = (uint8_t)(col >> 8);
        } else {
            v->r = cr; v->g = cg; v->b = cb;
        }
        v->a = ca;
        if ((g->geom & G_FOG) && c[3] > 0) {
            int64_t zn = ((int64_t)c[2] << 16) / c[3];
            v->a = (uint8_t)clamp32(((zn * g->fog_mul) >> 16) + g->fog_off, 0, 255);
        }
        v->s = (int32_t)(((int64_t)s * g->tex_sscale) >> 16) * 2048;   /* s10.5 -> s15.16 */
        v->t = (int32_t)(((int64_t)t * g->tex_tscale) >> 16) * 2048;
    }
}

static n64_vtx_t lerp_vtx(const n64_vtx_t *a, const n64_vtx_t *b, int64_t f /* 0..65536 */)
{
    n64_vtx_t o;
#define L32(fa, fb) (int32_t)((int64_t)(fa) + ((((int64_t)(fb) - (int64_t)(fa)) * f) >> 16))
    o.cx = L32(a->cx, b->cx); o.cy = L32(a->cy, b->cy); o.cz = L32(a->cz, b->cz); o.cw = L32(a->cw, b->cw);
    o.s = L32(a->s, b->s); o.t = L32(a->t, b->t);
    o.r = (uint8_t)L32(a->r, b->r); o.g = (uint8_t)L32(a->g, b->g);
    o.b = (uint8_t)L32(a->b, b->b); o.a = (uint8_t)L32(a->a, b->a);
#undef L32
    o.clip = 0;
    return o;
}

static void emit_screen_tri(hs_n64_t *n, const n64_vtx_t *const v[3])
{
    n64_gfx_t *g = &n->gfx;
    n64_snap_t *b = n64_back(n);
    uint32_t w[N64_TRI_WORDS], i, flags = 0;
    int64_t xs[3], ys[3], area;
    for (i = 0; i < 3u; i++) {
        int64_t cw = v[i]->cw < 64 ? 64 : v[i]->cw;
        int64_t nx = ((int64_t)v[i]->cx * 65536) / cw, ny = ((int64_t)v[i]->cy * 65536) / cw;
        int64_t nz = ((int64_t)v[i]->cz * 65536) / cw;
        int64_t dz;
        xs[i] = ((int64_t)g->vp_trans[0] << 14) + ((nx * g->vp_scale[0]) >> 2);
        ys[i] = ((int64_t)g->vp_trans[1] << 14) - ((ny * g->vp_scale[1]) >> 2);
        if (xs[i] > (32767LL << 16)) xs[i] = 32767LL << 16;
        if (xs[i] < -(32767LL << 16)) xs[i] = -(32767LL << 16);
        if (ys[i] > (32767LL << 16)) ys[i] = 32767LL << 16;
        if (ys[i] < -(32767LL << 16)) ys[i] = -(32767LL << 16);
        dz = ((nz * g->vp_scale[2]) + ((int64_t)g->vp_trans[2] << 16)) >> 10;
        w[1 + i * 7u + 0u] = (uint32_t)(int32_t)xs[i];
        w[1 + i * 7u + 1u] = (uint32_t)(int32_t)ys[i];
        w[1 + i * 7u + 2u] = (uint32_t)clamp32(dz, 0, 65535);
        w[1 + i * 7u + 3u] = (uint32_t)(int32_t)cw;
        w[1 + i * 7u + 4u] = (uint32_t)v[i]->s;
        w[1 + i * 7u + 5u] = (uint32_t)v[i]->t;
        if (g->geom & G_SHADING_SMOOTH)
            w[1 + i * 7u + 6u] = ((uint32_t)v[i]->r << 24) | ((uint32_t)v[i]->g << 16) | ((uint32_t)v[i]->b << 8) | v[i]->a;
        else
            w[1 + i * 7u + 6u] = ((uint32_t)v[0]->r << 24) | ((uint32_t)v[0]->g << 16) | ((uint32_t)v[0]->b << 8) | v[0]->a;
    }
    area = ((xs[1] - xs[0]) >> 4) * ((ys[2] - ys[0]) >> 4) - ((xs[2] - xs[0]) >> 4) * ((ys[1] - ys[0]) >> 4);
    if (area == 0) return;
    if ((g->geom & G_CULL_BACK) && area > 0) return;
    if ((g->geom & G_CULL_FRONT) && area < 0) return;
    if (g->geom & G_ZBUFFER) flags |= N64_TRI_Z;
    if (g->geom & G_SHADE) flags |= N64_TRI_SHADE;
    if (g->tex_on) flags |= N64_TRI_TEX;
    w[0] = (N64_OP_HS_TRI << 24) | (g->tex_tile << 8) | flags;
    emit(n, w, N64_TRI_WORDS);
    b->ntris++;
}

static void draw_tri(hs_n64_t *n, uint32_t i0, uint32_t i1, uint32_t i2)
{
    n64_gfx_t *g = &n->gfx;
    const n64_vtx_t *in[3];
    n64_vtx_t poly[4];
    uint32_t np = 0, i;
    if (i0 >= 64u || i1 >= 64u || i2 >= 64u) return;
    in[0] = &g->vbuf[i0]; in[1] = &g->vbuf[i1]; in[2] = &g->vbuf[i2];
    if (in[0]->clip & in[1]->clip & in[2]->clip) return;          /* trivially outside */
    if (!((in[0]->clip | in[1]->clip | in[2]->clip) & 16u)) {
        emit_screen_tri(n, in);
        return;
    }
    /* clip against the near plane z >= -w */
    for (i = 0; i < 3u; i++) {
        const n64_vtx_t *a = in[i], *b = in[(i + 1u) % 3u];
        int64_t da = (int64_t)a->cz + a->cw, db = (int64_t)b->cz + b->cw;
        if (da >= 0 && np < 4u) poly[np++] = *a;
        if ((da >= 0) != (db >= 0) && np < 4u) {
            int64_t f = (da * 65536) / (da - db);
            poly[np++] = lerp_vtx(a, b, f);
        }
    }
    if (np >= 3u) {
        const n64_vtx_t *t0[3];
        t0[0] = &poly[0]; t0[1] = &poly[1]; t0[2] = &poly[2];
        emit_screen_tri(n, t0);
        if (np == 4u) { t0[1] = &poly[2]; t0[2] = &poly[3]; emit_screen_tri(n, t0); }
    }
}

static void gfx_reset(n64_gfx_t *g)
{
    uint32_t i;
    memset(g, 0, sizeof(*g));
    for (i = 0; i < 16u; i++) { g->proj[i] = 0; g->mv[0][i] = 0; }
    g->proj[0] = g->proj[5] = g->proj[10] = g->proj[15] = 65536;
    g->mv[0][0] = g->mv[0][5] = g->mv[0][10] = g->mv[0][15] = 65536;
    g->vp_scale[0] = 640; g->vp_scale[1] = 480; g->vp_scale[2] = 511;
    g->vp_trans[0] = 640; g->vp_trans[1] = 480; g->vp_trans[2] = 511;
    g->timg_w = 1;
}

static void gfx_task(hs_n64_t *n, uint32_t dl, uint32_t flags)
{
    n64_gfx_t *g = &n->gfx;
    n64_snap_t *b = n64_back(n);
    uint32_t stack[18], sp = 0, pc = dl & 0x00FFFFF8u, k;
    uint32_t keep_om_hi = g->om_hi, keep_om_lo = g->om_lo;
    gfx_reset(g);
    g->om_hi = keep_om_hi; g->om_lo = keep_om_lo;
    g->f3d_mode = (flags & 0x8000u) ? 1u : 0u;
    b->gfx_tasks++;
    for (k = 0; k < N64_DL_MAX_CMDS; k++) {
        uint32_t w0 = rd32(n, pc), w1 = rd32(n, pc + 4u), op = w0 >> 24;
        uint32_t cmd[4];
        pc += 8u;
        switch (op) {
        case 0x00: break;
        case 0x01: {                                    /* G_MTX */
            uint32_t p = (w0 >> 16) & 0xFFu, a = seg_addr(n, w1);
            int32_t m[16];
            uint8_t raw[64];
            uint32_t i;
            for (i = 0; i < 64u; i++) raw[i] = rd8(n, a + i);
            n64_rsp_load_mtx(raw, m);
            if (p & 1u) {
                if (p & 2u) memcpy(g->proj, m, sizeof(m)); else n64_rsp_matmul(m, g->proj, g->proj);
            } else {
                if ((p & 4u) && g->mv_top < 9u) { memcpy(g->mv[g->mv_top + 1u], g->mv[g->mv_top], sizeof(m)); g->mv_top++; }
                if (p & 2u) memcpy(g->mv[g->mv_top], m, sizeof(m)); else n64_rsp_matmul(m, g->mv[g->mv_top], g->mv[g->mv_top]);
            }
            g->mvp_valid = 0;
            break;
        }
        case 0x03: {                                    /* G_MOVEMEM */
            uint32_t idx = (w0 >> 16) & 0xFFu, a = seg_addr(n, w1), i;
            if (idx == 0x80u) {
                for (i = 0; i < 4u; i++) { g->vp_scale[i] = rd16s(n, a + i * 2u); g->vp_trans[i] = rd16s(n, a + 8u + i * 2u); }
            } else if (idx >= 0x86u && idx <= 0x94u) {
                uint32_t l = (idx - 0x86u) >> 1;
                for (i = 0; i < 3u; i++) { g->light_col[l][i] = rd8(n, a + i); g->light_dir[l][i] = (int8_t)rd8(n, a + 8u + i); }
            }
            break;
        }
        case 0x04: {                                    /* G_VTX */
            uint32_t cnt, v0;
            if (g->f3d_mode) { cnt = ((w0 >> 20) & 15u) + 1u; v0 = (w0 >> 16) & 15u; }
            else { cnt = (w0 >> 10) & 0x3Fu; v0 = ((w0 >> 16) & 0xFFu) >> 1; }
            load_vertices(n, seg_addr(n, w1), v0, cnt);
            break;
        }
        case 0x06:                                      /* G_DL */
            if (((w0 >> 16) & 0xFFu) == 0u) { if (sp < 18u) stack[sp++] = pc; else break; }
            pc = seg_addr(n, w1) & ~7u;
            break;
        case 0xB1:                                      /* G_TRI2 (F3DEX) */
            draw_tri(n, ((w0 >> 16) & 0xFFu) >> 1, ((w0 >> 8) & 0xFFu) >> 1, (w0 & 0xFFu) >> 1);
            draw_tri(n, ((w1 >> 16) & 0xFFu) >> 1, ((w1 >> 8) & 0xFFu) >> 1, (w1 & 0xFFu) >> 1);
            break;
        case 0xBF: {                                    /* G_TRI1 */
            uint32_t d = g->f3d_mode ? 10u : 2u;
            draw_tri(n, ((w1 >> 16) & 0xFFu) / d, ((w1 >> 8) & 0xFFu) / d, (w1 & 0xFFu) / d);
            break;
        }
        case 0xB3: g->rdphalf2 = w1; break;
        case 0xB4: g->rdphalf1 = w1; break;
        case 0xB2: break;
        case 0xB6: g->geom &= ~w1; break;
        case 0xB7: g->geom |= w1; break;
        case 0xB8:                                      /* G_ENDDL */
            if (sp == 0u) { k = N64_DL_MAX_CMDS; break; }
            pc = stack[--sp];
            break;
        case 0xB9: case 0xBA: {                         /* SETOTHERMODE_L / _H */
            uint32_t sh = (w0 >> 8) & 0xFFu, len = w0 & 0xFFu;
            uint32_t m = (len >= 32u ? 0xFFFFFFFFu : ((1u << len) - 1u)) << (sh & 31u);
            if (op == 0xB9) g->om_lo = (g->om_lo & ~m) | (w1 & m);
            else g->om_hi = (g->om_hi & ~m) | (w1 & m);
            cmd[0] = (0xEFu << 24) | (g->om_hi & 0x00FFFFFFu); cmd[1] = g->om_lo;
            n64_rdp_ingest(n, cmd, 2);
            break;
        }
        case 0xBB:                                      /* G_TEXTURE */
            g->tex_on = w0 & 0xFFu ? 1u : 0u;
            g->tex_tile = (w0 >> 8) & 7u;
            g->tex_sscale = w1 >> 16;
            g->tex_tscale = w1 & 0xFFFFu;
            break;
        case 0xBC: {                                    /* G_MOVEWORD */
            uint32_t idx = w0 & 0xFFu, off = (w0 >> 8) & 0xFFFFu;
            if (idx == 0x02u) g->num_lights = (w1 & 0x80000000u) ? ((w1 & 0x7FFFFFFFu) >> 5) - 1u : w1 / 24u;
            else if (idx == 0x06u) g->seg[(off >> 2) & 15u] = w1 & 0x00FFFFFFu;
            else if (idx == 0x08u) { g->fog_mul = (int16_t)(w1 >> 16); g->fog_off = (int16_t)(w1 & 0xFFFFu); }
            if (g->num_lights > 7u) g->num_lights = 7u;
            break;
        }
        case 0xBD: if (g->mv_top) { g->mv_top--; g->mvp_valid = 0; } break;
        case 0xBE: break;                               /* G_CULLDL: conservative no-op */
        case 0xE4: case 0xE5: {                         /* texture rectangle + 2 RDPHALF words */
            uint32_t h1 = rd32(n, pc + 4u), h2 = rd32(n, pc + 12u);
            pc += 16u;
            cmd[0] = w0; cmd[1] = w1; cmd[2] = h1; cmd[3] = h2;
            n64_rdp_ingest(n, cmd, 4);
            break;
        }
        default:
            if (op >= 0xC0u) {
                cmd[0] = w0; cmd[1] = w1;
                if (op == 0xFDu || op == 0xFFu || op == 0xFEu) cmd[1] = seg_addr(n, w1);
                n64_rdp_ingest(n, cmd, 2);
            }
            break;
        }
    }
    n->rsp.hle_gfx++;
}

/* ------------------------------------------------- microcode identification
 * Commercial OSTasks carry ucode / ucode_data pointers. The microcode's data
 * segment contains a readable version banner; the HLE engine picks the command
 * encoding from it. Tasks with ucode_data == 0 (HandshakeOS demo / homebrew HS
 * tasks) keep the original HS behaviour, so existing ROMs are unaffected. */
static int has_str(const uint8_t *p, uint32_t len, const char *s)
{
    uint32_t i, k, sl = (uint32_t)strlen(s);
    for (i = 0; i + sl <= len; i++) {
        for (k = 0; k < sl && p[i + k] == (uint8_t)s[k]; k++) {}
        if (k == sl) return 1;
    }
    return 0;
}

uint32_t n64_ucode_classify(const uint8_t *data, uint32_t len)
{
    if (has_str(data, len, "F3DEX2") || has_str(data, len, "F3DZEX") || has_str(data, len, "F3DLX2") ||
        has_str(data, len, "F3DLP2") || has_str(data, len, "F3DLX.Rej2") || has_str(data, len, "F3DFLX2")) return N64_UC_F3DEX2;
    if (has_str(data, len, "S2DEX")) return N64_UC_S2DEX;
    if (has_str(data, len, "L3DEX") ) return N64_UC_UNKNOWN_GFX;
    if (has_str(data, len, "F3DEX") || has_str(data, len, "F3DLX") || has_str(data, len, "F3DLP")) return N64_UC_F3DEX;
    if (has_str(data, len, "RSP SW Version: 2.0") || has_str(data, len, "RSP Gfx ucode F3D") ) return N64_UC_FAST3D;
    return N64_UC_UNKNOWN_GFX;
}

static uint32_t ucode_kind(hs_n64_t *n, uint32_t ucode_data, uint32_t size)
{
    n64_ucinfo_t *u = &n->ucinfo;
    uint8_t buf[0x800];
    uint32_t i, len = size ? size : 0x800u;
    if (ucode_data == 0u) return N64_UC_HS;
    if (u->addr == ucode_data && u->valid) { u->hits++; return u->kind; }
    if (len > sizeof(buf)) len = sizeof(buf);
    for (i = 0; i < len; i++) buf[i] = rd8(n, ucode_data + i);
    u->addr = ucode_data; u->valid = 1u; u->kind = n64_ucode_classify(buf, len); u->detects++;
    return u->kind;
}

/* F3DEX2 geometry-mode bits -> the internal (Fast3D / F3DEX) layout */
static uint32_t geom_from_f3dex2(uint32_t m)
{
    uint32_t g = m & (G_ZBUFFER | G_SHADE | G_FOG | G_LIGHTING | 0x000C0000u);   /* same positions */
    if (m & 0x00000200u) g |= G_CULL_FRONT;
    if (m & 0x00000400u) g |= G_CULL_BACK;
    if (m & 0x00200000u) g |= G_SHADING_SMOOTH;
    return g;
}

static void do_mtx(hs_n64_t *n, uint32_t addr, int proj, int load, int push)
{
    n64_gfx_t *g = &n->gfx;
    int32_t m[16];
    uint8_t raw[64];
    uint32_t i;
    for (i = 0; i < 64u; i++) raw[i] = rd8(n, addr + i);
    n64_rsp_load_mtx(raw, m);
    if (proj) {
        if (load) memcpy(g->proj, m, sizeof(m)); else n64_rsp_matmul(m, g->proj, g->proj);
    } else {
        if (push && g->mv_top < 9u) { memcpy(g->mv[g->mv_top + 1u], g->mv[g->mv_top], sizeof(m)); g->mv_top++; }
        if (load) memcpy(g->mv[g->mv_top], m, sizeof(m)); else n64_rsp_matmul(m, g->mv[g->mv_top], g->mv[g->mv_top]);
    }
    g->mvp_valid = 0;
}

/* F3DEX2 / F3DZEX (Zelda) command decoder. Same vertex format, lighting,
 * clipping and RDP front end as the F3DEX path; only the encodings differ. */
static void gfx_task_f3dex2(hs_n64_t *n, uint32_t dl)
{
    n64_gfx_t *g = &n->gfx;
    n64_snap_t *b = n64_back(n);
    uint32_t stack[18], sp = 0, pc = dl & 0x00FFFFF8u, k, geom2 = 0;
    uint32_t keep_om_hi = g->om_hi, keep_om_lo = g->om_lo;
    gfx_reset(g);
    g->om_hi = keep_om_hi; g->om_lo = keep_om_lo;
    g->f3d_mode = 0;
    b->gfx_tasks++;
    for (k = 0; k < N64_DL_MAX_CMDS; k++) {
        uint32_t w0 = rd32(n, pc), w1 = rd32(n, pc + 4u), op = w0 >> 24;
        uint32_t cmd[4];
        pc += 8u;
        switch (op) {
        case 0x00: case 0xE0: case 0x03: case 0x08: case 0xD3: case 0xD4: case 0xD5: case 0xD6: break; /* NOOP, CULLDL (conservative), LINE3D, SPECIAL, DMA_IO */
        case 0x01: {                                    /* G_VTX: n in bits 12-19, end index*2 in bits 1-7 */
            uint32_t cnt = (w0 >> 12) & 0xFFu, end = (w0 >> 1) & 0x7Fu;
            if (cnt && cnt <= end) load_vertices(n, seg_addr(n, w1), end - cnt, cnt);
            break;
        }
        case 0x02: n->ucinfo.unhandled++; break;        /* G_MODIFYVTX: not yet */
        case 0x04:                                      /* G_BRANCH_Z: always take the near (high-detail) branch */
            pc = seg_addr(n, g->rdphalf1) & ~7u;
            break;
        case 0x05:
            draw_tri(n, ((w0 >> 16) & 0xFFu) >> 1, ((w0 >> 8) & 0xFFu) >> 1, (w0 & 0xFFu) >> 1);
            break;
        case 0x06: case 0x07:                           /* G_TRI2, G_QUAD */
            draw_tri(n, ((w0 >> 16) & 0xFFu) >> 1, ((w0 >> 8) & 0xFFu) >> 1, (w0 & 0xFFu) >> 1);
            draw_tri(n, ((w1 >> 16) & 0xFFu) >> 1, ((w1 >> 8) & 0xFFu) >> 1, (w1 & 0xFFu) >> 1);
            break;
        case 0xD7:                                      /* G_TEXTURE */
            g->tex_on = ((w0 >> 1) & 0x7Fu) ? 1u : 0u;
            g->tex_tile = (w0 >> 8) & 7u;
            g->tex_sscale = w1 >> 16;
            g->tex_tscale = w1 & 0xFFFFu;
            break;
        case 0xD8: {                                    /* G_POPMTX: w1 = 64 * count */
            uint32_t c = w1 >> 6;
            while (c-- && g->mv_top) g->mv_top--;
            g->mvp_valid = 0;
            break;
        }
        case 0xD9:                                      /* G_GEOMETRYMODE: and ~clear, or set */
            geom2 = (geom2 & (w0 | 0xFF000000u)) | w1;
            g->geom = geom_from_f3dex2(geom2);
            break;
        case 0xDA: {                                    /* G_MTX: params ^ PUSH(1); LOAD 2, PROJECTION 4 */
            uint32_t p = (w0 & 0xFFu) ^ 0x01u;
            do_mtx(n, seg_addr(n, w1), (p & 4u) != 0, (p & 2u) != 0, (p & 1u) != 0);
            break;
        }
        case 0xDB: {                                    /* G_MOVEWORD: index bits 16-23, offset 0-15 */
            uint32_t idx = (w0 >> 16) & 0xFFu, off = w0 & 0xFFFFu;
            if (idx == 0x02u) g->num_lights = w1 / 24u;
            else if (idx == 0x06u) g->seg[(off >> 2) & 15u] = w1 & 0x00FFFFFFu;
            else if (idx == 0x08u) { g->fog_mul = (int16_t)(w1 >> 16); g->fog_off = (int16_t)(w1 & 0xFFFFu); }
            else if (idx == 0x0Au) {                    /* G_MW_LIGHTCOL: light (off/24), col or copy */
                uint32_t l = off / 24u;
                if (l < 8u && (off % 24u) == 0u) { g->light_col[l][0] = (uint8_t)(w1 >> 24); g->light_col[l][1] = (uint8_t)(w1 >> 16); g->light_col[l][2] = (uint8_t)(w1 >> 8); }
            }
            if (g->num_lights > 7u) g->num_lights = 7u;
            break;
        }
        case 0xDC: {                                    /* G_MOVEMEM: index bits 0-7, offset*8 bits 8-15 */
            uint32_t idx = w0 & 0xFFu, off = ((w0 >> 8) & 0xFFu) * 8u, a = seg_addr(n, w1), i;
            if (idx == 8u) {                            /* G_MV_VIEWPORT */
                for (i = 0; i < 4u; i++) { g->vp_scale[i] = rd16s(n, a + i * 2u); g->vp_trans[i] = rd16s(n, a + 8u + i * 2u); }
            } else if (idx == 10u && off >= 48u) {      /* G_MV_LIGHT: light l at (l + 2) * 24 */
                uint32_t l = off / 24u - 2u;
                if (l < 8u) for (i = 0; i < 3u; i++) { g->light_col[l][i] = rd8(n, a + i); g->light_dir[l][i] = (int8_t)rd8(n, a + 8u + i); }
            } else if (idx == 14u) n->ucinfo.unhandled++;   /* G_MV_MATRIX (forced MVP): not yet */
            break;
        }
        case 0xDD: n->ucinfo.unhandled++; break;        /* G_LOAD_UCODE: not yet */
        case 0xDE:                                      /* G_DL (push unless bits 16-23 set) */
            if (((w0 >> 16) & 0xFFu) == 0u) { if (sp < 18u) stack[sp++] = pc; else break; }
            pc = seg_addr(n, w1) & ~7u;
            break;
        case 0xDF:                                      /* G_ENDDL */
            if (sp == 0u) { k = N64_DL_MAX_CMDS; break; }
            pc = stack[--sp];
            break;
        case 0xE1: g->rdphalf1 = w1; break;
        case 0xF1: g->rdphalf2 = w1; break;
        case 0xE2: case 0xE3: {                         /* SETOTHERMODE_L / _H: shift = 32 - s - len */
            uint32_t len = (w0 & 0xFFu) + 1u, s = (w0 >> 8) & 0xFFu, sh = (32u > s + len) ? 32u - s - len : 0u;
            uint32_t m = (len >= 32u ? 0xFFFFFFFFu : ((1u << len) - 1u)) << (sh & 31u);
            if (op == 0xE2) g->om_lo = (g->om_lo & ~m) | (w1 & m);
            else g->om_hi = (g->om_hi & ~m) | (w1 & m);
            cmd[0] = (0xEFu << 24) | (g->om_hi & 0x00FFFFFFu); cmd[1] = g->om_lo;
            n64_rdp_ingest(n, cmd, 2);
            break;
        }
        case 0xE4: case 0xE5: {                         /* texture rectangle + RDPHALF_1/2 commands */
            uint32_t h1 = rd32(n, pc + 4u), h2 = rd32(n, pc + 12u);
            pc += 16u;
            cmd[0] = w0; cmd[1] = w1; cmd[2] = h1; cmd[3] = h2;
            n64_rdp_ingest(n, cmd, 4);
            break;
        }
        default:
            if (op >= 0xE6u) {                          /* RDP pass-through (0xE6..0xFF) */
                cmd[0] = w0; cmd[1] = w1;
                if (op == 0xFDu || op == 0xFFu || op == 0xFEu) cmd[1] = seg_addr(n, w1);
                n64_rdp_ingest(n, cmd, 2);
            } else n->ucinfo.unhandled++;
            break;
        }
    }
    n->rsp.hle_gfx++;
}

/* ------------------------------------------------------- HLE audio */
static int16_t dm16(const hs_n64_t *n, uint32_t a) { a &= 0xFFEu; return (int16_t)n64_be16(n->rsp.dmem + a); }
static void dw16(hs_n64_t *n, uint32_t a, int32_t v) { a &= 0xFFEu; n64_wbe16(n->rsp.dmem + a, (uint16_t)(int16_t)clamp32(v, -32768, 32767)); }

int16_t n64_adpcm_sample(int16_t p1, int16_t p2, int32_t resid, const int16_t *book)
{
    int64_t acc = (int64_t)book[0] * p1 + (int64_t)book[1] * p2 + ((int64_t)resid * 2048);
    return (int16_t)clamp32(acc >> 11, -32768, 32767);
}

static void audio_task(hs_n64_t *n, uint32_t list, uint32_t size)
{
    n64_rsp_t *r = &n->rsp;
    uint32_t k, ncmd = size / 8u, i;
    if (ncmd > N64_AUDIO_MAX_CMDS) ncmd = N64_AUDIO_MAX_CMDS;
    n64_back(n)->audio_tasks++;
    for (k = 0; k < ncmd; k++) {
        uint32_t w0 = rd32(n, list + k * 8u), w1 = rd32(n, list + k * 8u + 4u);
        switch (w0 >> 24) {
        case 0x01: {                                   /* ADPCM */
            uint32_t st = w1 & 0x00FFFFFEu, in = r->a_in, out = r->a_out, f, j;
            int16_t p1 = 0, p2 = 0;
            if (!((w0 >> 16) & 1u)) { p1 = rd16s(n, st); p2 = rd16s(n, st + 2u); }
            for (f = 0; f < r->a_count / 16u && f < 256u; f++) {
                uint8_t h = r->dmem[(in + f * 9u) & 0xFFFu];
                uint32_t scale = h >> 4;
                for (j = 0; j < 16u; j++) {
                    uint8_t byte = r->dmem[(in + f * 9u + 1u + (j >> 1)) & 0xFFFu];
                    int32_t nib = (j & 1u) ? (byte & 15) : (byte >> 4);
                    int16_t s;
                    if (nib >= 8) nib -= 16;
                    s = n64_adpcm_sample(p1, p2, nib * (1 << scale), r->a_book);
                    p2 = p1; p1 = s;
                    dw16(n, out + (f * 16u + j) * 2u, s);
                }
            }
            if (st + 4u <= n->rdram_size) { n64_wbe16(n->rdram + st, (uint16_t)p1); n64_wbe16(n->rdram + st + 2u, (uint16_t)p2); }
            break;
        }
        case 0x02:                                     /* CLEARBUFF */
            for (i = 0; i < (w1 & 0xFFFu); i++) r->dmem[((w0 & 0xFFFu) + i) & 0xFFFu] = 0;
            break;
        case 0x03: {                                   /* PAN */
            uint32_t ol = (w1 >> 16) & 0xFFFu, orr = w1 & 0xFFFu;
            for (i = 0; i < r->a_count && i < 2048u; i++) {
                int32_t s = dm16(n, r->a_in + i * 2u);
                dw16(n, ol + i * 2u, (s * r->a_vol_l) >> 15);
                dw16(n, orr + i * 2u, (s * r->a_vol_r) >> 15);
            }
            break;
        }
        case 0x04: case 0x06: {                        /* LOADBUFF / SAVEBUFF */
            uint32_t dm = (w0 >> 12) & 0xFFFu, len = w0 & 0xFFFu, dr = w1 & 0x00FFFFFFu;
            for (i = 0; i < len; i++) {
                if (dr + i >= n->rdram_size) break;
                if ((w0 >> 24) == 0x04u) r->dmem[(dm + i) & 0xFFFu] = n->rdram[dr + i];
                else n->rdram[dr + i] = r->dmem[(dm + i) & 0xFFFu];
            }
            break;
        }
        case 0x05: {                                   /* RESAMPLE */
            uint32_t st = w1 & 0x00FFFFFCu, pitch = (w0 & 0xFFFFu) << 2, phase = 0, pos;
            int32_t last = 0;
            if (!((w0 >> 16) & 1u)) { phase = rd32(n, st); last = rd16s(n, st + 4u); }
            for (i = 0; i < r->a_count && i < 2048u; i++) {
                uint32_t idx = phase >> 16, fr = phase & 0xFFFFu;
                int32_t s0 = idx == 0u ? last : dm16(n, r->a_in + (idx - 1u) * 2u);
                int32_t s1 = dm16(n, r->a_in + idx * 2u);
                dw16(n, r->a_out + i * 2u, s0 + (int32_t)(((int64_t)(s1 - s0) * fr) >> 16));
                phase += pitch;
            }
            pos = phase >> 16;
            last = pos == 0u ? last : dm16(n, r->a_in + (pos - 1u) * 2u);
            phase &= 0xFFFFu;
            if (st + 8u <= n->rdram_size) { n64_wbe32(n->rdram + st, phase); n64_wbe16(n->rdram + st + 4u, (uint16_t)(int16_t)last); }
            break;
        }
        case 0x08: r->a_in = (w0 >> 12) & 0xFFFu; r->a_out = w0 & 0xFFFu; r->a_count = w1 & 0xFFFFu; break;
        case 0x09: r->a_vol_l = (int16_t)(w1 >> 16); r->a_vol_r = (int16_t)(w1 & 0xFFFFu); break;
        case 0x0B: r->a_book[0] = rd16s(n, w1); r->a_book[1] = rd16s(n, w1 + 2u); break;
        case 0x0C: {                                   /* MIXER */
            int32_t gain = (int16_t)(w0 & 0xFFFFu);
            uint32_t in = (w1 >> 16) & 0xFFFu, out = w1 & 0xFFFu;
            for (i = 0; i < r->a_count && i < 2048u; i++)
                dw16(n, out + i * 2u, dm16(n, out + i * 2u) + ((dm16(n, in + i * 2u) * gain) >> 15));
            break;
        }
        case 0x0D: {                                   /* INTERLEAVE */
            uint32_t out = w0 & 0xFFFu, l = (w1 >> 16) & 0xFFFu, rr = w1 & 0xFFFu;
            int16_t tmp[2048];
            uint32_t cnt = r->a_count > 1024u ? 1024u : r->a_count;
            for (i = 0; i < cnt; i++) { tmp[2u * i] = dm16(n, l + i * 2u); tmp[2u * i + 1u] = dm16(n, rr + i * 2u); }
            for (i = 0; i < cnt * 2u; i++) dw16(n, out + i * 2u, tmp[i]);
            break;
        }
        default: break;
        }
    }
    r->hle_audio++;
}

/* ------------------------------------------------- scalar-unit LLE */
static uint32_t su_c0_read(hs_n64_t *n, uint32_t rd)
{
    if (rd < 8u) return n64_phys_read32(n, 0x04040000u + rd * 4u);
    return n64_phys_read32(n, 0x04100000u + (rd & 7u) * 4u);
}

static void su_c0_write(hs_n64_t *n, uint32_t rd, uint32_t v)
{
    if (rd < 8u) {
        if (rd == 4u) {                 /* status writes from the RSP side never restart it */
            uint32_t keep = n->rsp.status & 1u;
            n->rsp.status |= 1u;
            n64_phys_write32(n, 0x04040010u, v & ~1u, 0xFFFFFFFFu);
            if (!keep && !(v & 2u)) n->rsp.status &= ~1u;
            return;
        }
        n64_phys_write32(n, 0x04040000u + rd * 4u, v, 0xFFFFFFFFu);
    } else {
        n64_phys_write32(n, 0x04100000u + (rd & 7u) * 4u, v, 0xFFFFFFFFu);
    }
}

static void lle_run(hs_n64_t *n)
{
    n64_rsp_t *r = &n->rsp;
    uint32_t *g = r->sgpr;
    uint32_t pc = r->pc & 0xFFCu, npc = (pc + 4u) & 0xFFCu, k;
    r->lle_tasks++;
    for (k = 0; k < N64_LLE_MAX_STEPS; k++) {
        uint32_t ins = n64_be32(r->imem + pc), op = ins >> 26;
        uint32_t rs = (ins >> 21) & 31u, rt = (ins >> 16) & 31u, rd = (ins >> 11) & 31u, sa = (ins >> 6) & 31u;
        uint32_t imm = (uint32_t)(int32_t)(int16_t)(uint16_t)ins;
        uint32_t target = 0, taken = 0, halt = 0, a;
        switch (op) {
        case 0x00:
            switch (ins & 63u) {
            case 0x00: g[rd] = g[rt] << sa; break;
            case 0x02: g[rd] = g[rt] >> sa; break;
            case 0x03: g[rd] = (uint32_t)((int32_t)g[rt] >> sa); break;
            case 0x04: g[rd] = g[rt] << (g[rs] & 31u); break;
            case 0x06: g[rd] = g[rt] >> (g[rs] & 31u); break;
            case 0x07: g[rd] = (uint32_t)((int32_t)g[rt] >> (g[rs] & 31u)); break;
            case 0x08: taken = 1; target = g[rs]; break;
            case 0x09: taken = 1; target = g[rs]; g[rd] = (pc + 8u) & 0xFFCu; break;
            case 0x0D: halt = 1; break;
            case 0x20: case 0x21: g[rd] = g[rs] + g[rt]; break;
            case 0x22: case 0x23: g[rd] = g[rs] - g[rt]; break;
            case 0x24: g[rd] = g[rs] & g[rt]; break;
            case 0x25: g[rd] = g[rs] | g[rt]; break;
            case 0x26: g[rd] = g[rs] ^ g[rt]; break;
            case 0x27: g[rd] = ~(g[rs] | g[rt]); break;
            case 0x2A: g[rd] = (int32_t)g[rs] < (int32_t)g[rt]; break;
            case 0x2B: g[rd] = g[rs] < g[rt]; break;
            default: r->lle_unimpl++; break;
            }
            break;
        case 0x01:
            if (rt & 0x10u) g[31] = (pc + 8u) & 0xFFCu;
            if (((rt & 1u) && (int32_t)g[rs] >= 0) || (!(rt & 1u) && (int32_t)g[rs] < 0)) { taken = 1; target = pc + 4u + (imm << 2); }
            break;
        case 0x02: case 0x03:
            if (op == 3u) g[31] = (pc + 8u) & 0xFFCu;
            taken = 1; target = (ins & 0x3FFu) << 2; break;
        case 0x04: if (g[rs] == g[rt]) { taken = 1; target = pc + 4u + (imm << 2); } break;
        case 0x05: if (g[rs] != g[rt]) { taken = 1; target = pc + 4u + (imm << 2); } break;
        case 0x06: if ((int32_t)g[rs] <= 0) { taken = 1; target = pc + 4u + (imm << 2); } break;
        case 0x07: if ((int32_t)g[rs] > 0) { taken = 1; target = pc + 4u + (imm << 2); } break;
        case 0x08: case 0x09: g[rt] = g[rs] + imm; break;
        case 0x0A: g[rt] = (int32_t)g[rs] < (int32_t)imm; break;
        case 0x0B: g[rt] = g[rs] < imm; break;
        case 0x0C: g[rt] = g[rs] & (ins & 0xFFFFu); break;
        case 0x0D: g[rt] = g[rs] | (ins & 0xFFFFu); break;
        case 0x0E: g[rt] = g[rs] ^ (ins & 0xFFFFu); break;
        case 0x0F: g[rt] = ins << 16; break;
        case 0x10:
            if (rs == 0u) g[rt] = su_c0_read(n, rd & 15u);
            else if (rs == 4u) su_c0_write(n, rd & 15u, g[rt]);
            break;
        case 0x20: a = g[rs] + imm; g[rt] = (uint32_t)(int32_t)(int8_t)r->dmem[a & 0xFFFu]; break;
        case 0x21: a = g[rs] + imm; g[rt] = (uint32_t)(int32_t)(int16_t)((r->dmem[a & 0xFFFu] << 8) | r->dmem[(a + 1u) & 0xFFFu]); break;
        case 0x23: a = g[rs] + imm;
                   g[rt] = ((uint32_t)r->dmem[a & 0xFFFu] << 24) | ((uint32_t)r->dmem[(a + 1u) & 0xFFFu] << 16) |
                           ((uint32_t)r->dmem[(a + 2u) & 0xFFFu] << 8) | r->dmem[(a + 3u) & 0xFFFu];
                   break;
        case 0x24: a = g[rs] + imm; g[rt] = r->dmem[a & 0xFFFu]; break;
        case 0x25: a = g[rs] + imm; g[rt] = ((uint32_t)r->dmem[a & 0xFFFu] << 8) | r->dmem[(a + 1u) & 0xFFFu]; break;
        case 0x28: a = g[rs] + imm; r->dmem[a & 0xFFFu] = (uint8_t)g[rt]; break;
        case 0x29: a = g[rs] + imm; r->dmem[a & 0xFFFu] = (uint8_t)(g[rt] >> 8); r->dmem[(a + 1u) & 0xFFFu] = (uint8_t)g[rt]; break;
        case 0x2B: a = g[rs] + imm;
                   r->dmem[a & 0xFFFu] = (uint8_t)(g[rt] >> 24); r->dmem[(a + 1u) & 0xFFFu] = (uint8_t)(g[rt] >> 16);
                   r->dmem[(a + 2u) & 0xFFFu] = (uint8_t)(g[rt] >> 8); r->dmem[(a + 3u) & 0xFFFu] = (uint8_t)g[rt];
                   break;
        default: r->lle_unimpl++; halt = 1; break;          /* COP2 / LWC2 / SWC2: vector unit not modelled */
        }
        g[0] = 0;
        r->lle_steps++;
        if (halt) { pc = (pc + 4u) & 0xFFCu; break; }
        pc = npc;
        npc = taken ? (target & 0xFFCu) : ((npc + 4u) & 0xFFCu);
    }
    r->pc = pc;
}

/* ----------------------------------------------------------- dispatch */
void n64_rsp_start(hs_n64_t *n)
{
    n64_rsp_t *r = &n->rsp;
    uint32_t type = n64_be32(r->dmem + 0xFC0u);
    uint32_t flags = n64_be32(r->dmem + 0xFC4u);
    uint32_t data = n64_be32(r->dmem + 0xFF0u) & 0x00FFFFFFu;
    uint32_t size = n64_be32(r->dmem + 0xFF4u);
    uint32_t ucd = n64_be32(r->dmem + 0xFD8u) & 0x00FFFFFFu, ucd_size = n64_be32(r->dmem + 0xFDCu);
    if (type == 1u) {
        uint32_t kind = ucode_kind(n, ucd, ucd_size);
        if (kind == N64_UC_F3DEX2) gfx_task_f3dex2(n, data);
        else if (kind == N64_UC_HS) gfx_task(n, data, flags);
        else if (kind == N64_UC_FAST3D || kind == N64_UC_F3DEX) gfx_task(n, data, kind == N64_UC_FAST3D ? 0x8000u : 0u);
        else n->ucinfo.unknown_tasks++;      /* S2DEX / unknown: skipped, counted */
        r->status |= 0x200u;                 /* SIG2: task done */
    } else if (type == 2u) {
        if (ucd == 0u) audio_task(n, data, size);
        else n->ucinfo.audio_skipped++;      /* Nintendo audio ABIs not implemented yet: silent, RDRAM untouched */
    } else {
        lle_run(n);
    }
    r->status |= 0x3u;                       /* halted + broke */
    if (r->status & 0x40u) n64_mi_raise(n, MI_SP);
}
