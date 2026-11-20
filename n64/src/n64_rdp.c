/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_rdp.c - clean-room RDP rasterizer for hs-n64 (Phase 1 only).
 * Replays the frozen FRONT command list into one strip of the virtual
 * (640*ss)x(480*ss) target at a time (hs_ssaa_render strip tiling):
 *   - edge-function triangles, watertight tie rule, sample-centre coverage
 *   - Gouraud shade, perspective-correct s/t, 16-bit Z compare/update
 *   - TMEM (4 KiB) with RGBA16/RGBA32/CI4/CI8/IA4/IA8/IA16/I4/I8 + TLUT,
 *     wrap / mirror / clamp, point, RDP 3-point and 4-tap bilinear filters
 *   - colour combiner (1-cycle / 2-cycle, all documented selector inputs)
 *   - blender (P*A + M*B)/(A+B) incl. fog, alpha compare
 *   - fill rectangles (colour and Z targets), texture rectangles, scissor
 * Attribute setup uses IEEE doubles once per primitive (no FMA contraction);
 * all per-sample stepping is exact int64 fixed point, so the output is
 * bit-identical on x86_64 and AArch64. Reads ONLY the published snapshot
 * and writes ONLY n64_video_t + the caller framebuffer.
 */
#include <string.h>
#include "n64_internal.h"

typedef struct rctx {
    n64_video_t *v;
    const n64_snap_t *s;
    uint32_t *strip;
    uint16_t *z;
    uint32_t sw, y0, rows, ss;
    int64_t kx, ky;              /* native 16.16 -> virtual 24.8: v8 = (x16 * k) >> 24 */
    uint32_t nw, nh;
    uint32_t bilinear4;
    uint32_t ref;                /* HS_N64_CFG_REF_RDP: reference per-pixel path */
    n64_rstate_t *rs;            /* private raster state of this context */
    uint64_t *samples;
    uint32_t *prims;
    uint32_t pal_valid;          /* fast path: pal[] mirrors the current TLUT */
    uint32_t pal[256];
} rctx_t;

void n64_rstate_reset(n64_rstate_t *rs)
{
    memset(rs, 0, sizeof(*rs));
    rs->sc_x1 = 320 << 2; rs->sc_y1 = 240 << 2;
    rs->timg_w = 1;
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ---------------------------------------------------------- state */
static void set_tile(n64_rstate_t *rs, uint32_t w0, uint32_t w1)
{
    uint32_t t = (w1 >> 24) & 7u;
    n64_tile_t *tl = &rs->tile[t];
    tl->fmt = (w0 >> 21) & 7u; tl->siz = (w0 >> 19) & 3u;
    tl->line = (w0 >> 9) & 0x1FFu; tl->tmem = w0 & 0x1FFu;
    tl->pal = (w1 >> 20) & 15u;
    tl->cmt = (w1 >> 18) & 3u; tl->maskt = (w1 >> 14) & 15u; tl->shiftt = (w1 >> 10) & 15u;
    tl->cms = (w1 >> 8) & 3u; tl->masks = (w1 >> 4) & 15u; tl->shifts = w1 & 15u;
    rs->tile_raw[t][0] = w0; rs->tile_raw[t][1] = w1;
}

static void set_tile_size(n64_rstate_t *rs, uint32_t w0, uint32_t w1)
{
    uint32_t t = (w1 >> 24) & 7u;
    n64_tile_t *tl = &rs->tile[t];
    tl->sl = (w0 >> 12) & 0xFFFu; tl->tl = w0 & 0xFFFu;
    tl->sh = (w1 >> 12) & 0xFFFu; tl->th = w1 & 0xFFFu;
    rs->tsize_raw[t][0] = w0; rs->tsize_raw[t][1] = w1;
}

static uint8_t blob_byte(const n64_snap_t *s, const n64_rstate_t *rs, uint32_t rel)
{
    if (rel >= rs->timg_len) return 0;
    if (rs->timg_off + rel >= N64_TEXBLOB_MAX) return 0;
    return s->tex[rs->timg_off + rel];
}

/* TMEM loads read the snapshot texture blob (resolved from RDRAM in Phase 0) */
static void do_load(const n64_snap_t *s, n64_rstate_t *rs, uint32_t op, uint32_t w0, uint32_t w1)
{
    uint32_t t = (w1 >> 24) & 7u;
    n64_tile_t *tl = &rs->tile[t];
    uint32_t bits = 4u << rs->timg_siz;
    uint32_t base = tl->tmem * 8u;
    uint32_t i, r;
    if (op == 0x33u) {                               /* Load Block */
        uint32_t sl = (w0 >> 12) & 0xFFFu, sh = (w1 >> 12) & 0xFFFu;
        uint32_t texels = (sh >= sl ? sh - sl : 0u) + 1u;
        uint32_t nb = (texels * bits + 7u) >> 3, src = (sl * bits) >> 3;
        if (nb > 4096u) nb = 4096u;
        for (i = 0; i < nb; i++) rs->tmem[(base + i) & 0xFFFu] = blob_byte(s, rs, src + i);
        tl->sl = sl << 2; tl->tl = (w0 & 0xFFFu) << 2; tl->sh = sh << 2; tl->th = tl->tl;
        return;
    }
    {                                                /* Load Tile / Load TLUT */
        uint32_t sl = ((w0 >> 12) & 0xFFFu) >> 2, tlr = (w0 & 0xFFFu) >> 2;
        uint32_t sh = ((w1 >> 12) & 0xFFFu) >> 2, th = (w1 & 0xFFFu) >> 2;
        uint32_t row_bytes = (rs->timg_w * bits + 7u) >> 3;
        uint32_t nb = ((sh >= sl ? sh - sl : 0u) + 1u) * bits / 8u;
        uint32_t line = op == 0x30u ? nb : tl->line * 8u;
        if (nb == 0) nb = 1;
        for (r = tlr; r <= th && r < tlr + 256u; r++) {
            uint32_t src = (r - rs->timg_row0) * row_bytes + ((sl * bits) >> 3);
            uint32_t dst = base + (r - tlr) * line;
            for (i = 0; i < nb && i < 4096u; i++) rs->tmem[(dst + i) & 0xFFFu] = blob_byte(s, rs, src + i);
        }
        if (op == 0x34u) {
            tl->sl = (w0 >> 12) & 0xFFFu; tl->tl = w0 & 0xFFFu;
            tl->sh = (w1 >> 12) & 0xFFFu; tl->th = w1 & 0xFFFu;
        }
    }
}

/* ---------------------------------------------------------- texels */
static uint32_t c5551(uint32_t c)
{
    uint32_t r = (c >> 11) & 31u, g = (c >> 6) & 31u, b = (c >> 1) & 31u;
    return ((r << 3 | r >> 2) << 24) | ((g << 3 | g >> 2) << 16) | ((b << 3 | b >> 2) << 8) | ((c & 1u) ? 255u : 0u);
}

static uint32_t tlut_entry(const n64_rstate_t *rs, uint32_t idx)
{
    uint32_t a = (0x800u + idx * 2u) & 0xFFEu;
    uint32_t c = ((uint32_t)rs->tmem[a] << 8) | rs->tmem[a + 1u];
    if (((rs->om_hi >> 14) & 3u) == 3u) {            /* IA16 TLUT */
        uint32_t i8 = c >> 8;
        return (i8 << 24) | (i8 << 16) | (i8 << 8) | (c & 0xFFu);
    }
    return c5551(c);
}

static uint32_t fetch_texel(const n64_rstate_t *rs, const n64_tile_t *tl, uint32_t s, uint32_t t)
{
    uint32_t base = tl->tmem * 8u + t * tl->line * 8u;
    const uint8_t *m = rs->tmem;
    uint32_t a, v, i;
    switch ((tl->fmt << 2) | tl->siz) {
    case (0u << 2) | 2u: a = (base + s * 2u) & 0xFFEu; return c5551(((uint32_t)m[a] << 8) | m[a + 1u]);
    case (0u << 2) | 3u: a = (base + s * 4u) & 0xFFCu;
        return ((uint32_t)m[a] << 24) | ((uint32_t)m[a + 1u] << 16) | ((uint32_t)m[a + 2u] << 8) | m[a + 3u];
    case (2u << 2) | 0u: a = (base + (s >> 1)) & 0xFFFu; v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4);
        return tlut_entry(rs, (tl->pal << 4) | v);
    case (2u << 2) | 1u: a = (base + s) & 0xFFFu; return tlut_entry(rs, m[a]);
    case (3u << 2) | 0u: a = (base + (s >> 1)) & 0xFFFu; v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4);
        i = (v >> 1) * 36u + ((v >> 1) >> 1); i = clampi((int32_t)i, 0, 255);
        return (i << 24) | (i << 16) | (i << 8) | ((v & 1u) ? 255u : 0u);
    case (3u << 2) | 1u: a = (base + s) & 0xFFFu; v = m[a];
        i = (v >> 4) * 17u;
        return (i << 24) | (i << 16) | (i << 8) | ((v & 15u) * 17u);
    case (3u << 2) | 2u: a = (base + s * 2u) & 0xFFEu;
        i = m[a];
        return (i << 24) | (i << 16) | (i << 8) | m[a + 1u];
    case (4u << 2) | 0u: a = (base + (s >> 1)) & 0xFFFu; v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4);
        i = v * 17u;
        return (i << 24) | (i << 16) | (i << 8) | i;
    case (4u << 2) | 1u: a = (base + s) & 0xFFFu; i = m[a];
        return (i << 24) | (i << 16) | (i << 8) | i;
    default:
        a = (base + s * 2u) & 0xFFEu; return c5551(((uint32_t)m[a] << 8) | m[a + 1u]);
    }
}

static uint32_t wrap_coord(int32_t c, uint32_t mask, uint32_t cm, int32_t clampmax)
{
    if ((cm & 2u) || mask == 0u) {
        if (c < 0) c = 0;
        if (c > clampmax) c = clampmax;
    }
    if (mask) {
        uint32_t u = (uint32_t)c;
        if ((cm & 1u) && ((u >> mask) & 1u)) u = ~u;
        return u & ((1u << mask) - 1u);
    }
    return (uint32_t)c;
}

static int32_t shift_coord(int32_t c, uint32_t sh)
{
    if (sh == 0u) return c;
    if (sh <= 10u) return c >> sh;
    return (int32_t)((uint32_t)c << (16u - sh));
}

static uint32_t lerp_c(uint32_t a, uint32_t b, uint32_t f /* 0..32 */)
{
    uint32_t o = 0, k;
    for (k = 0; k < 32u; k += 8u) {
        int32_t x = (int32_t)((a >> k) & 0xFFu), y = (int32_t)((b >> k) & 0xFFu);
        o |= (uint32_t)(x + (((y - x) * (int32_t)f + 16) >> 5)) << k;
    }
    return o;
}

/* s16, t16: texel coordinates s15.16 (already perspective divided) */
static uint32_t sample_tex(const rctx_t *c, const n64_rstate_t *rs, uint32_t tile, int32_t s16, int32_t t16, int filt)
{
    const n64_tile_t *tl = &rs->tile[tile & 7u];
    int32_t s = shift_coord(s16 >> 11, tl->shifts), t = shift_coord(t16 >> 11, tl->shiftt);  /* 10.5 */
    int32_t si, ti, fs, ft;
    int32_t cs = (int32_t)((tl->sh >> 2) - (tl->sl >> 2)), ct = (int32_t)((tl->th >> 2) - (tl->tl >> 2));
    uint32_t s0, s1, t0, t1, a00, a10, a01, a11;
    s -= (int32_t)tl->sl << 3;
    t -= (int32_t)tl->tl << 3;
    if (cs < 0) cs = 0;
    if (ct < 0) ct = 0;
    si = s >> 5; ti = t >> 5; fs = s & 31; ft = t & 31;
    s0 = wrap_coord(si, tl->masks, tl->cms, cs);
    t0 = wrap_coord(ti, tl->maskt, tl->cmt, ct);
    if (!filt) return fetch_texel(rs, tl, s0, t0);
    s1 = wrap_coord(si + 1, tl->masks, tl->cms, cs);
    t1 = wrap_coord(ti + 1, tl->maskt, tl->cmt, ct);
    a00 = fetch_texel(rs, tl, s0, t0); a10 = fetch_texel(rs, tl, s1, t0);
    a01 = fetch_texel(rs, tl, s0, t1); a11 = fetch_texel(rs, tl, s1, t1);
    if (c->bilinear4) return lerp_c(lerp_c(a00, a10, (uint32_t)fs), lerp_c(a01, a11, (uint32_t)fs), (uint32_t)ft);
    {                                                /* RDP 3-point filter */
        uint32_t o = 0, k;
        for (k = 0; k < 32u; k += 8u) {
            int32_t p00 = (int32_t)((a00 >> k) & 0xFFu), p10 = (int32_t)((a10 >> k) & 0xFFu);
            int32_t p01 = (int32_t)((a01 >> k) & 0xFFu), p11 = (int32_t)((a11 >> k) & 0xFFu), r;
            if (fs + ft < 32) r = p00 + (((p10 - p00) * fs + (p01 - p00) * ft + 16) >> 5);
            else r = p11 + (((p01 - p11) * (32 - fs) + (p10 - p11) * (32 - ft) + 16) >> 5);
            o |= (uint32_t)clampi(r, 0, 255) << k;
        }
        return o;
    }
}

/* ---------------------------------------------------------- combiner */
typedef struct cc_in {
    int32_t comb[4], tex0[4], tex1[4], prim[4], shade[4], env[4];
    int32_t noise, prim_lod;
} cc_in_t;

static void unpack(uint32_t c, int32_t *o)
{ o[0] = (int32_t)(c >> 24); o[1] = (int32_t)((c >> 16) & 0xFFu); o[2] = (int32_t)((c >> 8) & 0xFFu); o[3] = (int32_t)(c & 0xFFu); }

static int32_t sel_rgb_ab(const cc_in_t *in, uint32_t s, uint32_t ch, int is_a)
{
    switch (s) {
    case 0: return in->comb[ch];
    case 1: return in->tex0[ch];
    case 2: return in->tex1[ch];
    case 3: return in->prim[ch];
    case 4: return in->shade[ch];
    case 5: return in->env[ch];
    case 6: return is_a ? 255 : 0;      /* 1 (sub A) / key centre (sub B) */
    case 7: return is_a ? in->noise : 0;
    default: return 0;
    }
}

static int32_t sel_rgb_c(const cc_in_t *in, uint32_t s, uint32_t ch)
{
    switch (s) {
    case 0: return in->comb[ch];
    case 1: return in->tex0[ch];
    case 2: return in->tex1[ch];
    case 3: return in->prim[ch];
    case 4: return in->shade[ch];
    case 5: return in->env[ch];
    case 7: return in->comb[3];
    case 8: return in->tex0[3];
    case 9: return in->tex1[3];
    case 10: return in->prim[3];
    case 11: return in->shade[3];
    case 12: return in->env[3];
    case 14: return in->prim_lod;
    default: return 0;
    }
}

static int32_t sel_rgb_d(const cc_in_t *in, uint32_t s, uint32_t ch)
{
    switch (s) {
    case 0: return in->comb[ch];
    case 1: return in->tex0[ch];
    case 2: return in->tex1[ch];
    case 3: return in->prim[ch];
    case 4: return in->shade[ch];
    case 5: return in->env[ch];
    case 6: return 255;
    default: return 0;
    }
}

static int32_t sel_a(const cc_in_t *in, uint32_t s, int is_mul)
{
    switch (s) {
    case 0: return is_mul ? 0 : in->comb[3];
    case 1: return in->tex0[3];
    case 2: return in->tex1[3];
    case 3: return in->prim[3];
    case 4: return in->shade[3];
    case 5: return in->env[3];
    case 6: return is_mul ? in->prim_lod : 255;
    default: return 0;
    }
}

typedef struct cc_sel { uint32_t a[2], b[2], c[2], d[2], aa[2], ab[2], ac[2], ad[2]; } cc_sel_t;

static void cc_decode(const n64_rstate_t *rs, cc_sel_t *s)
{
    uint32_t h = rs->comb_hi, l = rs->comb_lo;
    s->a[0] = (h >> 20) & 15u; s->c[0] = (h >> 15) & 31u; s->aa[0] = (h >> 12) & 7u; s->ac[0] = (h >> 9) & 7u;
    s->a[1] = (h >> 5) & 15u;  s->c[1] = h & 31u;
    s->b[0] = (l >> 28) & 15u; s->b[1] = (l >> 24) & 15u; s->aa[1] = (l >> 21) & 7u; s->ac[1] = (l >> 18) & 7u;
    s->d[0] = (l >> 15) & 7u;  s->ab[0] = (l >> 12) & 7u; s->ad[0] = (l >> 9) & 7u;
    s->d[1] = (l >> 6) & 7u;   s->ab[1] = (l >> 3) & 7u;  s->ad[1] = l & 7u;
}

static void cc_cycle(const cc_sel_t *s, uint32_t cy, const cc_in_t *in, int32_t *out)
{
    uint32_t ch;
    for (ch = 0; ch < 3u; ch++) {
        int32_t a = sel_rgb_ab(in, s->a[cy], ch, 1), b = sel_rgb_ab(in, s->b[cy], ch, 0);
        int32_t c = sel_rgb_c(in, s->c[cy], ch), d = sel_rgb_d(in, s->d[cy], ch);
        out[ch] = clampi((((a - b) * c + 128) >> 8) + d, 0, 255);
    }
    {
        int32_t a = sel_a(in, s->aa[cy], 0), b = sel_a(in, s->ab[cy], 0);
        int32_t c = sel_a(in, s->ac[cy], 1), d = sel_a(in, s->ad[cy], 0);
        out[3] = clampi((((a - b) * c + 128) >> 8) + d, 0, 255);
    }
}

/* ---------------------------------------------------------- blender */
static uint32_t blend_px(const n64_rstate_t *rs, uint32_t cy, const int32_t *in, int32_t shade_a, uint32_t mem, int force)
{
    uint32_t om = rs->om_lo;
    uint32_t ps = (om >> (30u - 2u * cy)) & 3u, as = (om >> (26u - 2u * cy)) & 3u;
    uint32_t ms = (om >> (22u - 2u * cy)) & 3u, bs = (om >> (18u - 2u * cy)) & 3u;
    int32_t p[3], m[3], fa, fb, k;
    int32_t memc[3], blc[4], fgc[4];
    unpack(rs->blend, blc); unpack(rs->fog, fgc);
    memc[0] = (int32_t)((mem >> 16) & 0xFFu); memc[1] = (int32_t)((mem >> 8) & 0xFFu); memc[2] = (int32_t)(mem & 0xFFu);
    for (k = 0; k < 3; k++) {
        p[k] = ps == 0 ? in[k] : (ps == 1 ? memc[k] : (ps == 2 ? blc[k] : fgc[k]));
        m[k] = ms == 0 ? in[k] : (ms == 1 ? memc[k] : (ms == 2 ? blc[k] : fgc[k]));
    }
    if (!force) return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    fa = as == 0 ? in[3] : (as == 1 ? fgc[3] : (as == 2 ? shade_a : 0));
    fb = bs == 0 ? 255 - fa : (bs == 1 ? 255 : (bs == 2 ? 255 : 0));
    if (fa + fb == 0) return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    {
        uint32_t o = 0;
        for (k = 0; k < 3; k++) {
            int32_t v = (p[k] * fa + m[k] * fb + (fa + fb) / 2) / (fa + fb);
            o |= (uint32_t)clampi(v, 0, 255) << (16 - 8 * k);
        }
        return o;
    }
}

/* full pixel pipeline after texturing: combiner -> alpha compare -> blender */
static int shade_pixel(const rctx_t *c, const n64_rstate_t *rs, const cc_sel_t *sel, cc_in_t *in,
                       uint32_t *dst)
{
    uint32_t cyc = (rs->om_hi >> 20) & 3u;
    int32_t out[4];
    uint32_t px;
    (void)c;
    memset(in->comb, 0, sizeof(in->comb));
    cc_cycle(sel, 0, in, out);
    if (cyc == 1u) { memcpy(in->comb, out, sizeof(out)); cc_cycle(sel, 1, in, out); }
    if ((rs->om_lo & 3u) == 1u && out[3] < (int32_t)(rs->blend & 0xFFu)) return 0;
    if ((rs->om_lo & 3u) == 1u && out[3] == 0) return 0;
    if (cyc == 1u) {
        uint32_t c0 = blend_px(rs, 0, out, in->shade[3], *dst, 1);
        int32_t mid[4];
        mid[0] = (int32_t)((c0 >> 16) & 0xFFu); mid[1] = (int32_t)((c0 >> 8) & 0xFFu); mid[2] = (int32_t)(c0 & 0xFFu); mid[3] = out[3];
        px = blend_px(rs, 1, mid, in->shade[3], *dst, (rs->om_lo & 0x4000u) != 0);
    } else {
        px = blend_px(rs, 0, out, in->shade[3], *dst, (rs->om_lo & 0x4000u) != 0);
    }
    *dst = px;
    return 1;
}

/* ---------------------------------------------------------- scissor */
static void scissor_virtual(const rctx_t *c, const n64_rstate_t *rs, int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1)
{
    /* 10.2 native -> virtual pixel bounds [x0,x1) */
    int64_t a = ((int64_t)rs->sc_x0 << 14) * c->kx >> 32, b = ((int64_t)rs->sc_x1 << 14) * c->kx >> 32;
    int64_t d = ((int64_t)rs->sc_y0 << 14) * c->ky >> 32, e = ((int64_t)rs->sc_y1 << 14) * c->ky >> 32;
    int32_t ys0 = (int32_t)c->y0, ys1 = (int32_t)(c->y0 + c->rows);
    *x0 = (int32_t)(a < 0 ? 0 : a); *x1 = (int32_t)(b > (int64_t)c->sw ? (int64_t)c->sw : b);
    *y0 = (int32_t)(d < ys0 ? ys0 : d); *y1 = (int32_t)(e > ys1 ? ys1 : e);
}

/* ---------------------------------------------------------- triangles */
static int64_t d2fx(double v) { return (int64_t)(v >= 0.0 ? v + 0.5 : v - 0.5); }

static void draw_tri(rctx_t *c, const n64_rstate_t *rs, const uint32_t *w)
{
    uint32_t flags = w[0] & 0xFFu, tile = (w[0] >> 8) & 7u;
    int64_t X[3], Y[3];
    double attr[3][8];
    int64_t A0[8], DX[8], DY[8];
    int64_t area, e_dx[3], e_dy[3], e_c[3], bias[3];
    int32_t sx0, sy0, sx1, sy1, bx0, by0, bx1, by1, px, py;
    uint32_t i, k, cyc = (rs->om_hi >> 20) & 3u;
    int persp = (rs->om_hi >> 19) & 1u, filt = ((rs->om_hi >> 12) & 3u) >= 2u && cyc < 2u;
    int zc = (flags & N64_TRI_Z) && (rs->om_lo & 0x10u), zu = (flags & N64_TRI_Z) && (rs->om_lo & 0x20u);
    int tex = (flags & N64_TRI_TEX) != 0;
    int64_t wmin = 0x7FFFFFFFLL;
    double area2, x0d, y0d;
    cc_sel_t sel;
    cc_in_t in;
    int idx[3] = {0, 1, 2};
    for (i = 0; i < 3u; i++) {
        X[i] = ((int64_t)(int32_t)w[1 + i * 7u] * c->kx) >> 24;
        Y[i] = ((int64_t)(int32_t)w[2 + i * 7u] * c->ky) >> 24;
        if ((int32_t)w[4 + i * 7u] < wmin) wmin = (int32_t)w[4 + i * 7u];
    }
    area = (X[1] - X[0]) * (Y[2] - Y[0]) - (X[2] - X[0]) * (Y[1] - Y[0]);
    if (area == 0) return;
    if (area < 0) { idx[1] = 2; idx[2] = 1; area = -area; }
    {
        int64_t tx[3], ty[3];
        for (i = 0; i < 3u; i++) { tx[i] = X[idx[i]]; ty[i] = Y[idx[i]]; }
        memcpy(X, tx, sizeof(X)); memcpy(Y, ty, sizeof(Y));
    }
    /* bounding box in virtual pixels, clipped to strip + scissor */
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    {
        int64_t mnx = X[0], mxx = X[0], mny = Y[0], mxy = Y[0];
        for (i = 1; i < 3u; i++) {
            if (X[i] < mnx) mnx = X[i]; if (X[i] > mxx) mxx = X[i];
            if (Y[i] < mny) mny = Y[i]; if (Y[i] > mxy) mxy = Y[i];
        }
        bx0 = (int32_t)(mnx >> 8); bx1 = (int32_t)((mxx >> 8) + 1);
        by0 = (int32_t)(mny >> 8); by1 = (int32_t)((mxy >> 8) + 1);
        if (bx0 < sx0) bx0 = sx0;
        if (by0 < sy0) by0 = sy0;
        if (bx1 > sx1) bx1 = sx1;
        if (by1 > sy1) by1 = sy1;
        if (bx0 >= bx1 || by0 >= by1) return;
    }
    /* edge functions E_i for edge (a -> b): (bx-ax)(py-ay) - (by-ay)(px-ax) */
    for (i = 0; i < 3u; i++) {
        uint32_t a = (i + 1u) % 3u, b = (i + 2u) % 3u;
        int64_t dx = X[b] - X[a], dy = Y[b] - Y[a];
        e_dx[i] = -dy * 256;          /* per +1 virtual pixel in x */
        e_dy[i] = dx * 256;
        /* value at sample centre of pixel (bx0, by0) */
        e_c[i] = dx * (((int64_t)by0 * 256 + 128) - Y[a]) - dy * (((int64_t)bx0 * 256 + 128) - X[a]);
        bias[i] = (dy < 0 || (dy == 0 && dx > 0)) ? 0 : -1;
    }
    (void)area;
    /* attributes: r g b a z q sq tq */
    for (i = 0; i < 3u; i++) {
        const uint32_t *v = &w[1 + (uint32_t)idx[i] * 7u];
        uint32_t col = v[6];
        double wv = (double)(int32_t)v[3], q = persp && tex ? (double)wmin / (wv > 1.0 ? wv : 1.0) : 1.0;
        attr[i][0] = (double)(col >> 24); attr[i][1] = (double)((col >> 16) & 0xFFu);
        attr[i][2] = (double)((col >> 8) & 0xFFu); attr[i][3] = (double)(col & 0xFFu);
        attr[i][4] = (double)v[2];
        attr[i][5] = q * 65536.0;
        attr[i][6] = (double)(int32_t)v[4] * q;
        attr[i][7] = (double)(int32_t)v[5] * q;
    }
    x0d = (double)X[0] / 256.0; y0d = (double)Y[0] / 256.0;
    area2 = ((double)(X[1] - X[0]) * (double)(Y[2] - Y[0]) - (double)(X[2] - X[0]) * (double)(Y[1] - Y[0])) / 65536.0;
    for (k = 0; k < 8u; k++) {
        double d1 = attr[1][k] - attr[0][k], d2 = attr[2][k] - attr[0][k];
        double ddx = (d1 * ((double)(Y[2] - Y[0]) / 256.0) - d2 * ((double)(Y[1] - Y[0]) / 256.0)) / area2;
        double ddy = (d2 * ((double)(X[1] - X[0]) / 256.0) - d1 * ((double)(X[2] - X[0]) / 256.0)) / area2;
        double at = attr[0][k] + ddx * (((double)bx0 + 0.5) - x0d) + ddy * (((double)by0 + 0.5) - y0d);
        A0[k] = d2fx(at * 65536.0);
        DX[k] = d2fx(ddx * 65536.0);
        DY[k] = d2fx(ddy * 65536.0);
    }
    cc_decode(rs, &sel);
    unpack(rs->prim, in.prim); unpack(rs->env, in.env);
    memset(in.tex0, 0, sizeof(in.tex0)); memset(in.tex1, 0, sizeof(in.tex1));
    in.prim_lod = (int32_t)rs->prim_lod;
    for (py = by0; py < by1; py++) {
        int64_t e0 = e_c[0], e1 = e_c[1], e2 = e_c[2];
        int64_t at[8];
        uint32_t *row = c->strip + (size_t)(py - (int32_t)c->y0) * c->sw;
        uint16_t *zrow = c->z + (size_t)(py - (int32_t)c->y0) * c->sw;
        for (k = 0; k < 8u; k++) at[k] = A0[k];
        for (px = bx0; px < bx1; px++) {
            if (e0 + bias[0] >= 0 && e1 + bias[1] >= 0 && e2 + bias[2] >= 0) {
                int32_t zv = clampi((int32_t)(at[4] >> 16), 0, 65535);
                if (rs->om_lo & 0x4u) zv = (int32_t)(rs->prim_z & 0xFFFFu);
                if (!zc || (uint32_t)zv <= zrow[px]) {
                    uint32_t k2;
                    for (k2 = 0; k2 < 4u; k2++) in.shade[k2] = clampi((int32_t)(at[k2] >> 16), 0, 255);
                    in.noise = (int32_t)(((uint32_t)px * 0x9E37u ^ (uint32_t)py * 0x7F4Bu) >> 4 & 0xFFu);
                    if (tex) {
                        int64_t qd = at[5] >> 16;
                        int32_t s16, t16;
                        if (qd < 1) qd = 1;
                        s16 = (int32_t)(at[6] / qd);
                        t16 = (int32_t)(at[7] / qd);
                        unpack(sample_tex(c, rs, tile, s16, t16, filt), in.tex0);
                        if (sel.a[0] == 2u || sel.a[1] == 2u || sel.c[0] == 2u || sel.c[1] == 2u ||
                            sel.b[0] == 2u || sel.b[1] == 2u || sel.d[0] == 2u || sel.d[1] == 2u)
                            unpack(sample_tex(c, rs, tile + 1u, s16, t16, filt), in.tex1);
                    }
                    if (shade_pixel(c, rs, &sel, &in, &row[px])) {
                        if (zu) zrow[px] = (uint16_t)zv;
                        (*c->samples)++;
                    }
                }
            }
            e0 += e_dx[0]; e1 += e_dx[1]; e2 += e_dx[2];
            for (k = 0; k < 8u; k++) at[k] += DX[k];
        }
        e_c[0] += e_dy[0]; e_c[1] += e_dy[1]; e_c[2] += e_dy[2];
        for (k = 0; k < 8u; k++) A0[k] += DY[k];
    }
    (*c->prims)++;
}

/* ---------------------------------------------------------- rectangles */
static uint32_t fill_rgb(const n64_rstate_t *rs)
{
    if (rs->cimg_siz == 3u) return rs->fill >> 8;
    {
        uint32_t c = c5551(rs->fill >> 16);
        return c >> 8;
    }
}

static void fill_rect(rctx_t *c, const n64_rstate_t *rs, uint32_t w0, uint32_t w1)
{
    uint32_t cyc = (rs->om_hi >> 20) & 3u;
    int32_t xl = (int32_t)((w0 >> 12) & 0xFFFu), yl = (int32_t)(w0 & 0xFFFu);
    int32_t xh = (int32_t)((w1 >> 12) & 0xFFFu), yh = (int32_t)(w1 & 0xFFFu);
    int32_t sx0, sy0, sx1, sy1, x0, y0, x1, y1, px, py;
    int ztarget = rs->cimg_addr == rs->zimg_addr && rs->zimg_addr != 0u;
    if (cyc >= 2u) { xl += 4; yl += 4; }             /* fill/copy: inclusive lower-right */
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    x0 = (int32_t)((((int64_t)xh << 14) * c->kx) >> 32); x1 = (int32_t)((((int64_t)xl << 14) * c->kx) >> 32);
    y0 = (int32_t)((((int64_t)yh << 14) * c->ky) >> 32); y1 = (int32_t)((((int64_t)yl << 14) * c->ky) >> 32);
    if (x0 < sx0) x0 = sx0;
    if (y0 < sy0) y0 = sy0;
    if (x1 > sx1) x1 = sx1;
    if (y1 > sy1) y1 = sy1;
    if (x0 >= x1 || y0 >= y1) return;
    if (ztarget) {
        uint16_t zv = (uint16_t)((rs->fill >> 16) >= 0xFFF0u ? 0xFFFFu : (rs->fill >> 16));
        for (py = y0; py < y1; py++)
            for (px = x0; px < x1; px++) c->z[(size_t)(py - (int32_t)c->y0) * c->sw + (uint32_t)px] = zv;
        return;
    }
    if (cyc == 3u) {
        uint32_t col = fill_rgb(rs);
        for (py = y0; py < y1; py++)
            for (px = x0; px < x1; px++) c->strip[(size_t)(py - (int32_t)c->y0) * c->sw + (uint32_t)px] = col;
    } else {
        cc_sel_t sel;
        cc_in_t in;
        cc_decode(rs, &sel);
        memset(&in, 0, sizeof(in));
        unpack(rs->prim, in.prim); unpack(rs->env, in.env);
        in.prim_lod = (int32_t)rs->prim_lod;
        for (py = y0; py < y1; py++)
            for (px = x0; px < x1; px++)
                shade_pixel(c, rs, &sel, &in, &c->strip[(size_t)(py - (int32_t)c->y0) * c->sw + (uint32_t)px]);
    }
    (*c->prims)++;
}

static void tex_rect(rctx_t *c, const n64_rstate_t *rs, const uint32_t *w)
{
    uint32_t cyc = (rs->om_hi >> 20) & 3u, tile = (w[1] >> 24) & 7u;
    int32_t xl = (int32_t)((w[0] >> 12) & 0xFFFu), yl = (int32_t)(w[0] & 0xFFFu);
    int32_t xh = (int32_t)((w[1] >> 12) & 0xFFFu), yh = (int32_t)(w[1] & 0xFFFu);
    int32_t s0 = (int16_t)(w[2] >> 16), t0 = (int16_t)(w[2] & 0xFFFFu);       /* s10.5 */
    int32_t dsdx = (int16_t)(w[3] >> 16), dtdy = (int16_t)(w[3] & 0xFFFFu);   /* s5.10 */
    int32_t sx0, sy0, sx1, sy1, x0, y0, x1, y1, px, py;
    int filt = ((rs->om_hi >> 12) & 3u) >= 2u && cyc < 2u;
    int flip = ((w[0] >> 24) & 0x3Fu) == 0x25u;
    cc_sel_t sel;
    cc_in_t in;
    if (cyc >= 2u) { xl += 4; yl += 4; }
    if (cyc == 2u) dsdx >>= 2;
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    x0 = (int32_t)((((int64_t)xh << 14) * c->kx) >> 32); x1 = (int32_t)((((int64_t)xl << 14) * c->kx) >> 32);
    y0 = (int32_t)((((int64_t)yh << 14) * c->ky) >> 32); y1 = (int32_t)((((int64_t)yl << 14) * c->ky) >> 32);
    if (x0 < sx0) x0 = sx0;
    if (y0 < sy0) y0 = sy0;
    if (x1 > sx1) x1 = sx1;
    if (y1 > sy1) y1 = sy1;
    if (x0 >= x1 || y0 >= y1) return;
    cc_decode(rs, &sel);
    memset(&in, 0, sizeof(in));
    unpack(rs->prim, in.prim); unpack(rs->env, in.env);
    for (py = y0; py < y1; py++) {
        /* native position of the sample centre, s15.16 */
        int64_t ny = (((int64_t)py * 256 + 128) << 24) / c->ky;
        int64_t dyr = ny - ((int64_t)yh << 14);
        for (px = x0; px < x1; px++) {
            int64_t nx = (((int64_t)px * 256 + 128) << 24) / c->kx;
            int64_t dx = nx - ((int64_t)xh << 14), dy = dyr;   /* v0.2 fix: flip used to leak dx into later pixels */
            int64_t s16, t16;
            uint32_t *dst = &c->strip[(size_t)(py - (int32_t)c->y0) * c->sw + (uint32_t)px];
            if (flip) { int64_t tmp = dx; dx = dy; dy = tmp; }
            s16 = ((int64_t)s0 << 11) + ((dx * dsdx) >> 10);
            t16 = ((int64_t)t0 << 11) + ((dy * dtdy) >> 10);
            unpack(sample_tex(c, rs, tile, (int32_t)s16, (int32_t)t16, filt), in.tex0);
            if (cyc == 2u) {
                if ((rs->om_lo & 3u) && in.tex0[3] == 0) continue;
                *dst = ((uint32_t)in.tex0[0] << 16) | ((uint32_t)in.tex0[1] << 8) | (uint32_t)in.tex0[2];
            } else {
                memset(in.shade, 0, sizeof(in.shade));
                shade_pixel(c, rs, &sel, &in, dst);
            }
        }
    }
    (*c->prims)++;
}

/* ============================================================ fast path
 * Phase-1 fast path, selected unless HS_N64_CFG_REF_RDP is set. It computes
 * exactly the same integers as the reference functions above:
 *   - per-row exact integer span limits from the three edge functions
 *     (no per-pixel edge tests outside the span); attributes start at
 *     A0 + n*DX, the same int64 sum as n repeated additions
 *   - combiner/blender selectors, tile parameters and TLUT colours are
 *     resolved once per primitive (TLUT once per load) instead of per pixel
 *   - texture-rectangle column -> native position uses an exact
 *     quotient/remainder recurrence instead of one 64-bit divide per pixel
 *   - constant-q (non-perspective) s/t uses a shift when q is a power of 2
 * No floating point is added; the double setup is still the reference one. */
enum { V_COMB = 0, V_TEX0 = 4, V_TEX1 = 8, V_PRIM = 12, V_SHADE = 16, V_ENV = 20,
       V_K255 = 24, V_ZERO = 28, V_NOISE = 32, V_LOD = 33, V_N = 36 };

typedef struct px_pipe {
    uint8_t ro[2][4], rst[2][4];   /* rgb a,b,c,d: offset into v[], channel stride 0/1 */
    uint8_t ao[2][4];              /* alpha a,b,c,d offsets */
    uint8_t oc[2][3][4];           /* [cycle][channel][a,b,c,d] resolved v[] index */
    uint32_t two, acmp, force;
    int32_t  athr;
    uint32_t ps[2], as[2], ms[2], bs[2];
    int32_t  blc[4], fgc[4];
} px_pipe_t;

static void sel_ab_f(uint32_t s, int is_a, uint8_t *o, uint8_t *st)
{
    if (s <= 5u) { *o = (uint8_t)(s * 4u); *st = 1; return; }
    *st = 0;
    if (s == 6u) *o = is_a ? V_K255 : V_ZERO;
    else if (s == 7u) *o = is_a ? V_NOISE : V_ZERO;
    else *o = V_ZERO;
}
static void sel_c_f(uint32_t s, uint8_t *o, uint8_t *st)
{
    if (s <= 5u) { *o = (uint8_t)(s * 4u); *st = 1; return; }
    *st = 0;
    if (s >= 7u && s <= 12u) *o = (uint8_t)((s - 7u) * 4u + 3u);
    else if (s == 14u) *o = V_LOD;
    else *o = V_ZERO;
}
static void sel_d_f(uint32_t s, uint8_t *o, uint8_t *st)
{
    if (s <= 5u) { *o = (uint8_t)(s * 4u); *st = 1; return; }
    *st = 0;
    *o = (s == 6u) ? V_K255 : V_ZERO;
}
static uint8_t sel_a_f(uint32_t s, int is_mul)
{
    if (s == 0u) return is_mul ? V_ZERO : (uint8_t)(V_COMB + 3);
    if (s <= 5u) return (uint8_t)(s * 4u + 3u);
    if (s == 6u) return is_mul ? V_LOD : V_K255;
    return V_ZERO;
}

static void pipe_setup(const n64_rstate_t *rs, px_pipe_t *pp)
{
    cc_sel_t s;
    uint32_t cy, om = rs->om_lo;
    cc_decode(rs, &s);
    for (cy = 0; cy < 2u; cy++) {
        sel_ab_f(s.a[cy], 1, &pp->ro[cy][0], &pp->rst[cy][0]);
        sel_ab_f(s.b[cy], 0, &pp->ro[cy][1], &pp->rst[cy][1]);
        sel_c_f(s.c[cy], &pp->ro[cy][2], &pp->rst[cy][2]);
        sel_d_f(s.d[cy], &pp->ro[cy][3], &pp->rst[cy][3]);
        pp->ao[cy][0] = sel_a_f(s.aa[cy], 0); pp->ao[cy][1] = sel_a_f(s.ab[cy], 0);
        pp->ao[cy][2] = sel_a_f(s.ac[cy], 1); pp->ao[cy][3] = sel_a_f(s.ad[cy], 0);
        {
            uint32_t ch, j;
            for (ch = 0; ch < 3u; ch++)
                for (j = 0; j < 4u; j++) pp->oc[cy][ch][j] = (uint8_t)(pp->ro[cy][j] + ch * pp->rst[cy][j]);
        }
        pp->ps[cy] = (om >> (30u - 2u * cy)) & 3u; pp->as[cy] = (om >> (26u - 2u * cy)) & 3u;
        pp->ms[cy] = (om >> (22u - 2u * cy)) & 3u; pp->bs[cy] = (om >> (18u - 2u * cy)) & 3u;
    }
    pp->two = ((rs->om_hi >> 20) & 3u) == 1u;
    pp->acmp = (om & 3u) == 1u;
    pp->force = (om & 0x4000u) != 0;
    pp->athr = (int32_t)(rs->blend & 0xFFu);
    unpack(rs->blend, pp->blc); unpack(rs->fog, pp->fgc);
}

static void v_init(int32_t *v)
{
    uint32_t i;
    for (i = 0; i < (uint32_t)V_N; i++) v[i] = 0;
    v[V_K255] = v[V_K255 + 1] = v[V_K255 + 2] = v[V_K255 + 3] = 255;
}

static void cc_cycle_f(const px_pipe_t *pp, uint32_t cy, const int32_t *v, int32_t *out)
{
    const uint8_t *ao = pp->ao[cy];
    uint32_t ch;
    for (ch = 0; ch < 3u; ch++) {
        const uint8_t *o = pp->oc[cy][ch];
        out[ch] = clampi((((v[o[0]] - v[o[1]]) * v[o[2]] + 128) >> 8) + v[o[3]], 0, 255);
    }
    out[3] = clampi((((v[ao[0]] - v[ao[1]]) * v[ao[2]] + 128) >> 8) + v[ao[3]], 0, 255);
}

static uint32_t blend_f(const px_pipe_t *pp, uint32_t cy, const int32_t *in, int32_t shade_a, uint32_t mem, int force)
{
    uint32_t ps = pp->ps[cy], as = pp->as[cy], ms = pp->ms[cy], bs = pp->bs[cy];
    int32_t p[3], m[3], memc[3], fa, fb, k;
    memc[0] = (int32_t)((mem >> 16) & 0xFFu); memc[1] = (int32_t)((mem >> 8) & 0xFFu); memc[2] = (int32_t)(mem & 0xFFu);
    for (k = 0; k < 3; k++) {
        p[k] = ps == 0 ? in[k] : (ps == 1 ? memc[k] : (ps == 2 ? pp->blc[k] : pp->fgc[k]));
        m[k] = ms == 0 ? in[k] : (ms == 1 ? memc[k] : (ms == 2 ? pp->blc[k] : pp->fgc[k]));
    }
    if (!force) return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    fa = as == 0 ? in[3] : (as == 1 ? pp->fgc[3] : (as == 2 ? shade_a : 0));
    if (bs == 0u) {                 /* fb = 255 - fa: denominator is exactly 255, all terms >= 0 */
        uint32_t o = 0;
        fb = 255 - fa;
        for (k = 0; k < 3; k++) {
            uint32_t vv = ((uint32_t)(p[k] * fa + m[k] * fb) + 127u) / 255u;
            o |= (vv > 255u ? 255u : vv) << (16 - 8 * k);
        }
        return o;
    }
    fb = bs == 3u ? 0 : 255;
    if (fa + fb == 0) return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    {
        uint32_t o = 0;
        for (k = 0; k < 3; k++) {
            int32_t vv = (p[k] * fa + m[k] * fb + (fa + fb) / 2) / (fa + fb);
            o |= (uint32_t)clampi(vv, 0, 255) << (16 - 8 * k);
        }
        return o;
    }
}

static int shade_f(const px_pipe_t *pp, int32_t *v, uint32_t *dst)
{
    int32_t out[4];
    uint32_t px;
    v[V_COMB] = v[V_COMB + 1] = v[V_COMB + 2] = v[V_COMB + 3] = 0;
    cc_cycle_f(pp, 0, v, out);
    if (pp->two) {
        v[V_COMB] = out[0]; v[V_COMB + 1] = out[1]; v[V_COMB + 2] = out[2]; v[V_COMB + 3] = out[3];
        cc_cycle_f(pp, 1, v, out);
    }
    if (pp->acmp && (out[3] < pp->athr || out[3] == 0)) return 0;
    if (pp->two) {
        uint32_t c0 = blend_f(pp, 0, out, v[V_SHADE + 3], *dst, 1);
        int32_t mid[4];
        mid[0] = (int32_t)((c0 >> 16) & 0xFFu); mid[1] = (int32_t)((c0 >> 8) & 0xFFu); mid[2] = (int32_t)(c0 & 0xFFu); mid[3] = out[3];
        px = blend_f(pp, 1, mid, v[V_SHADE + 3], *dst, (int)pp->force);
    } else {
        px = blend_f(pp, 0, out, v[V_SHADE + 3], *dst, (int)pp->force);
    }
    *dst = px;
    return 1;
}

/* ---- textures */
typedef struct tex_setup {
    uint32_t shifts, shiftt, masks, maskt, cms, cmt;
    int32_t  sl8, tl8, cs, ct;
    uint32_t base, line8, code, pal4;
    const uint32_t *pal;
    uint32_t (*smp)(const struct rctx *c, const uint8_t *m, const struct tex_setup *ts, int32_t s16, int32_t t16, int filt);
} tex_setup_t;

static void tex_setup(const n64_rstate_t *rs, uint32_t tile, tex_setup_t *ts)
{
    const n64_tile_t *tl = &rs->tile[tile & 7u];
    ts->shifts = tl->shifts; ts->shiftt = tl->shiftt;
    ts->masks = tl->masks; ts->maskt = tl->maskt; ts->cms = tl->cms; ts->cmt = tl->cmt;
    ts->sl8 = (int32_t)tl->sl << 3; ts->tl8 = (int32_t)tl->tl << 3;
    ts->cs = (int32_t)((tl->sh >> 2) - (tl->sl >> 2)); ts->ct = (int32_t)((tl->th >> 2) - (tl->tl >> 2));
    if (ts->cs < 0) ts->cs = 0;
    if (ts->ct < 0) ts->ct = 0;
    ts->base = tl->tmem * 8u; ts->line8 = tl->line * 8u;
    ts->code = (tl->fmt << 2) | tl->siz; ts->pal4 = tl->pal << 4;
}

static void pal_build(rctx_t *c, const n64_rstate_t *rs)
{
    uint32_t i;
    for (i = 0; i < 256u; i++) c->pal[i] = tlut_entry(rs, i);
    c->pal_valid = 1;
}

/* per-format texel fetch (same arithmetic as fetch_texel; CI reads the TLUT
 * colours resolved by pal_build for the current TLUT) */
#define FX_ARGS const tex_setup_t *ts, const uint8_t *m, uint32_t base, uint32_t s
static uint32_t fx_rgba16(FX_ARGS) { uint32_t a = (base + s * 2u) & 0xFFEu; (void)ts; return c5551(((uint32_t)m[a] << 8) | m[a + 1u]); }
static uint32_t fx_rgba32(FX_ARGS)
{ uint32_t a = (base + s * 4u) & 0xFFCu; (void)ts;
  return ((uint32_t)m[a] << 24) | ((uint32_t)m[a + 1u] << 16) | ((uint32_t)m[a + 2u] << 8) | m[a + 3u]; }
static uint32_t fx_ci4(FX_ARGS)
{ uint32_t a = (base + (s >> 1)) & 0xFFFu, v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4); return ts->pal[(ts->pal4 | v) & 0xFFu]; }
static uint32_t fx_ci8(FX_ARGS) { uint32_t a = (base + s) & 0xFFFu; return ts->pal[m[a]]; }
static uint32_t fx_ia4(FX_ARGS)
{ uint32_t a = (base + (s >> 1)) & 0xFFFu, v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4), i;
  (void)ts; i = (v >> 1) * 36u + ((v >> 1) >> 1); i = (uint32_t)clampi((int32_t)i, 0, 255);
  return (i << 24) | (i << 16) | (i << 8) | ((v & 1u) ? 255u : 0u); }
static uint32_t fx_ia8(FX_ARGS)
{ uint32_t a = (base + s) & 0xFFFu, v = m[a], i = (v >> 4) * 17u; (void)ts;
  return (i << 24) | (i << 16) | (i << 8) | ((v & 15u) * 17u); }
static uint32_t fx_ia16(FX_ARGS)
{ uint32_t a = (base + s * 2u) & 0xFFEu, i = m[a]; (void)ts; return (i << 24) | (i << 16) | (i << 8) | m[a + 1u]; }
static uint32_t fx_i4(FX_ARGS)
{ uint32_t a = (base + (s >> 1)) & 0xFFFu, v = (s & 1u) ? (m[a] & 15u) : (m[a] >> 4), i = v * 17u; (void)ts;
  return (i << 24) | (i << 16) | (i << 8) | i; }
static uint32_t fx_i8(FX_ARGS) { uint32_t a = (base + s) & 0xFFFu, i = m[a]; (void)ts; return (i << 24) | (i << 16) | (i << 8) | i; }

static uint32_t filt3(uint32_t a00, uint32_t a10, uint32_t a01, uint32_t a11, int32_t fs, int32_t ft)
{
    uint32_t o = 0, k;
    if (fs + ft < 32) {
        for (k = 0; k < 32u; k += 8u) {
            int32_t p00 = (int32_t)((a00 >> k) & 0xFFu), p10 = (int32_t)((a10 >> k) & 0xFFu);
            int32_t p01 = (int32_t)((a01 >> k) & 0xFFu);
            o |= (uint32_t)clampi(p00 + (((p10 - p00) * fs + (p01 - p00) * ft + 16) >> 5), 0, 255) << k;
        }
    } else {
        for (k = 0; k < 32u; k += 8u) {
            int32_t p10 = (int32_t)((a10 >> k) & 0xFFu), p01 = (int32_t)((a01 >> k) & 0xFFu);
            int32_t p11 = (int32_t)((a11 >> k) & 0xFFu);
            o |= (uint32_t)clampi(p11 + (((p01 - p11) * (32 - fs) + (p10 - p11) * (32 - ft) + 16) >> 5), 0, 255) << k;
        }
    }
    return o;
}

#define DEF_SAMPLER(NAME, FX)                                                                         \
static uint32_t NAME(const rctx_t *c, const uint8_t *m, const tex_setup_t *ts, int32_t s16, int32_t t16, int filt) \
{                                                                                                     \
    int32_t s = shift_coord(s16 >> 11, ts->shifts) - ts->sl8, t = shift_coord(t16 >> 11, ts->shiftt) - ts->tl8; \
    int32_t si = s >> 5, ti = t >> 5;                                                                 \
    uint32_t s0 = wrap_coord(si, ts->masks, ts->cms, ts->cs), t0 = wrap_coord(ti, ts->maskt, ts->cmt, ts->ct); \
    uint32_t s1, t1, b0, b1, a00, a10, a01, a11;                                                      \
    if (!filt) return FX(ts, m, ts->base + t0 * ts->line8, s0);                                  \
    s1 = wrap_coord(si + 1, ts->masks, ts->cms, ts->cs);                                              \
    t1 = wrap_coord(ti + 1, ts->maskt, ts->cmt, ts->ct);                                              \
    b0 = ts->base + t0 * ts->line8; b1 = ts->base + t1 * ts->line8;                                   \
    a00 = FX(ts, m, b0, s0); a10 = FX(ts, m, b0, s1);                                       \
    a01 = FX(ts, m, b1, s0); a11 = FX(ts, m, b1, s1);                                       \
    if (c->bilinear4) return lerp_c(lerp_c(a00, a10, (uint32_t)(s & 31)), lerp_c(a01, a11, (uint32_t)(s & 31)), (uint32_t)(t & 31)); \
    return filt3(a00, a10, a01, a11, s & 31, t & 31);                                                 \
}
DEF_SAMPLER(smp_rgba16, fx_rgba16)
DEF_SAMPLER(smp_rgba32, fx_rgba32)
DEF_SAMPLER(smp_ci4, fx_ci4)
DEF_SAMPLER(smp_ci8, fx_ci8)
DEF_SAMPLER(smp_ia4, fx_ia4)
DEF_SAMPLER(smp_ia8, fx_ia8)
DEF_SAMPLER(smp_ia16, fx_ia16)
DEF_SAMPLER(smp_i4, fx_i4)
DEF_SAMPLER(smp_i8, fx_i8)

static void tex_bind(rctx_t *c, const n64_rstate_t *rs, uint32_t tile, tex_setup_t *ts)
{
    tex_setup(rs, tile, ts);
    switch (ts->code) {
    case (0u << 2) | 3u: ts->smp = smp_rgba32; break;
    case (2u << 2) | 0u: ts->smp = smp_ci4; break;
    case (2u << 2) | 1u: ts->smp = smp_ci8; break;
    case (3u << 2) | 0u: ts->smp = smp_ia4; break;
    case (3u << 2) | 1u: ts->smp = smp_ia8; break;
    case (3u << 2) | 2u: ts->smp = smp_ia16; break;
    case (4u << 2) | 0u: ts->smp = smp_i4; break;
    case (4u << 2) | 1u: ts->smp = smp_i8; break;
    default: ts->smp = smp_rgba16; break;
    }
    ts->pal = c->pal;
    if (ts->code == ((2u << 2) | 0u) || ts->code == ((2u << 2) | 1u)) {
        if (!c->pal_valid) pal_build(c, rs);
    }
}

static void unpack_v(uint32_t col, int32_t *o)
{ o[0] = (int32_t)(col >> 24); o[1] = (int32_t)((col >> 16) & 0xFFu); o[2] = (int32_t)((col >> 8) & 0xFFu); o[3] = (int32_t)(col & 0xFFu); }

/* floor(a / b) for b > 0 */
static int64_t floor_div(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b) != 0 && a < 0) q--;
    return q;
}

static void draw_tri_fast(rctx_t *c, const n64_rstate_t *rs, const uint32_t *w)
{
    uint32_t flags = w[0] & 0xFFu, tile = (w[0] >> 8) & 7u;
    int64_t X[3], Y[3];
    double attr[3][8];
    int64_t A0[8], DX[8], DY[8];
    int64_t area, e_dx[3], e_dy[3], e_c[3], bias[3];
    int32_t sx0, sy0, sx1, sy1, bx0, by0, bx1, by1, py;
    uint32_t i, k, cyc = (rs->om_hi >> 20) & 3u;
    int persp = (rs->om_hi >> 19) & 1u, filt = ((rs->om_hi >> 12) & 3u) >= 2u && cyc < 2u;
    int zc = (flags & N64_TRI_Z) && (rs->om_lo & 0x10u), zu = (flags & N64_TRI_Z) && (rs->om_lo & 0x20u);
    int tex = (flags & N64_TRI_TEX) != 0, primz = (rs->om_lo & 0x4u) != 0, need_t1;
    int64_t wmin = 0x7FFFFFFFLL, qconst = 0;
    uint32_t qshift = 0, qmode = 0;     /* 0 per-pixel divide, 1 constant divide, 2 constant shift */
    double area2, x0d, y0d;
    cc_sel_t sel;
    px_pipe_t pp;
    tex_setup_t ts0, ts1;
    int32_t v[V_N];
    int idx[3] = {0, 1, 2};
    uint64_t shaded = 0;
    for (i = 0; i < 3u; i++) {
        X[i] = ((int64_t)(int32_t)w[1 + i * 7u] * c->kx) >> 24;
        Y[i] = ((int64_t)(int32_t)w[2 + i * 7u] * c->ky) >> 24;
        if ((int32_t)w[4 + i * 7u] < wmin) wmin = (int32_t)w[4 + i * 7u];
    }
    area = (X[1] - X[0]) * (Y[2] - Y[0]) - (X[2] - X[0]) * (Y[1] - Y[0]);
    if (area == 0) return;
    if (area < 0) { idx[1] = 2; idx[2] = 1; }
    {
        int64_t tx[3], ty[3];
        for (i = 0; i < 3u; i++) { tx[i] = X[idx[i]]; ty[i] = Y[idx[i]]; }
        memcpy(X, tx, sizeof(X)); memcpy(Y, ty, sizeof(Y));
    }
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    {
        int64_t mnx = X[0], mxx = X[0], mny = Y[0], mxy = Y[0];
        for (i = 1; i < 3u; i++) {
            if (X[i] < mnx) mnx = X[i];
            if (X[i] > mxx) mxx = X[i];
            if (Y[i] < mny) mny = Y[i];
            if (Y[i] > mxy) mxy = Y[i];
        }
        bx0 = (int32_t)(mnx >> 8); bx1 = (int32_t)((mxx >> 8) + 1);
        by0 = (int32_t)(mny >> 8); by1 = (int32_t)((mxy >> 8) + 1);
        if (bx0 < sx0) bx0 = sx0;
        if (by0 < sy0) by0 = sy0;
        if (bx1 > sx1) bx1 = sx1;
        if (by1 > sy1) by1 = sy1;
        if (bx0 >= bx1 || by0 >= by1) return;
    }
    for (i = 0; i < 3u; i++) {
        uint32_t a = (i + 1u) % 3u, b = (i + 2u) % 3u;
        int64_t dx = X[b] - X[a], dy = Y[b] - Y[a];
        e_dx[i] = -dy * 256;
        e_dy[i] = dx * 256;
        e_c[i] = dx * (((int64_t)by0 * 256 + 128) - Y[a]) - dy * (((int64_t)bx0 * 256 + 128) - X[a]);
        bias[i] = (dy < 0 || (dy == 0 && dx > 0)) ? 0 : -1;
    }
    for (i = 0; i < 3u; i++) {
        const uint32_t *vv = &w[1 + (uint32_t)idx[i] * 7u];
        uint32_t col = vv[6];
        double wv = (double)(int32_t)vv[3], q = persp && tex ? (double)wmin / (wv > 1.0 ? wv : 1.0) : 1.0;
        attr[i][0] = (double)(col >> 24); attr[i][1] = (double)((col >> 16) & 0xFFu);
        attr[i][2] = (double)((col >> 8) & 0xFFu); attr[i][3] = (double)(col & 0xFFu);
        attr[i][4] = (double)vv[2];
        attr[i][5] = q * 65536.0;
        attr[i][6] = (double)(int32_t)vv[4] * q;
        attr[i][7] = (double)(int32_t)vv[5] * q;
    }
    x0d = (double)X[0] / 256.0; y0d = (double)Y[0] / 256.0;
    area2 = ((double)(X[1] - X[0]) * (double)(Y[2] - Y[0]) - (double)(X[2] - X[0]) * (double)(Y[1] - Y[0])) / 65536.0;
    for (k = 0; k < 8u; k++) {
        double d1 = attr[1][k] - attr[0][k], d2 = attr[2][k] - attr[0][k];
        double ddx = (d1 * ((double)(Y[2] - Y[0]) / 256.0) - d2 * ((double)(Y[1] - Y[0]) / 256.0)) / area2;
        double ddy = (d2 * ((double)(X[1] - X[0]) / 256.0) - d1 * ((double)(X[2] - X[0]) / 256.0)) / area2;
        double at = attr[0][k] + ddx * (((double)bx0 + 0.5) - x0d) + ddy * (((double)by0 + 0.5) - y0d);
        A0[k] = d2fx(at * 65536.0);
        DX[k] = d2fx(ddx * 65536.0);
        DY[k] = d2fx(ddy * 65536.0);
    }
    cc_decode(rs, &sel);
    pipe_setup(rs, &pp);
    v_init(v);
    unpack_v(rs->prim, v + V_PRIM); unpack_v(rs->env, v + V_ENV);
    v[V_LOD] = (int32_t)rs->prim_lod;
    need_t1 = sel.a[0] == 2u || sel.a[1] == 2u || sel.c[0] == 2u || sel.c[1] == 2u ||
              sel.b[0] == 2u || sel.b[1] == 2u || sel.d[0] == 2u || sel.d[1] == 2u;
    if (tex) {
        tex_bind(c, rs, tile, &ts0);
        tex_bind(c, rs, tile + 1u, &ts1);
        if (DX[5] == 0 && DY[5] == 0) {
            qconst = A0[5] >> 16;
            if (qconst < 1) qconst = 1;
            qmode = 1;
            if ((qconst & (qconst - 1)) == 0) { qmode = 2; while ((1LL << qshift) < qconst) qshift++; }
        }
    }
    for (py = by0; py < by1; py++) {
        int64_t at[8], lo = 0, hi = (int64_t)(bx1 - bx0) - 1;
        int32_t px, xs, xe;
        uint32_t *row = c->strip + (size_t)(py - (int32_t)c->y0) * c->sw;
        uint16_t *zrow = c->z + (size_t)(py - (int32_t)c->y0) * c->sw;
        /* exact span: E_i(t) = e_c[i] + t*e_dx[i] must satisfy E_i + bias_i >= 0 */
        for (i = 0; i < 3u && lo <= hi; i++) {
            int64_t e0 = e_c[i] + bias[i], d = e_dx[i];
            if (d == 0) { if (e0 < 0) hi = -1; }
            else if (d > 0) { int64_t t = -floor_div(e0, d); if (t > lo) lo = t; }
            else { int64_t t = floor_div(e0, -d); if (t < hi) hi = t; }
        }
        if (lo <= hi) {
            xs = bx0 + (int32_t)lo; xe = bx0 + (int32_t)hi + 1;
            for (k = 0; k < 8u; k++) at[k] = A0[k] + lo * DX[k];
            for (px = xs; px < xe; px++) {
                int32_t zv = primz ? (int32_t)(rs->prim_z & 0xFFFFu) : clampi((int32_t)(at[4] >> 16), 0, 65535);
                if (!zc || (uint32_t)zv <= zrow[px]) {
                    v[V_SHADE] = clampi((int32_t)(at[0] >> 16), 0, 255);
                    v[V_SHADE + 1] = clampi((int32_t)(at[1] >> 16), 0, 255);
                    v[V_SHADE + 2] = clampi((int32_t)(at[2] >> 16), 0, 255);
                    v[V_SHADE + 3] = clampi((int32_t)(at[3] >> 16), 0, 255);
                    v[V_NOISE] = (int32_t)(((uint32_t)px * 0x9E37u ^ (uint32_t)py * 0x7F4Bu) >> 4 & 0xFFu);
                    if (tex) {
                        int32_t s16, t16;
                        if (qmode == 2u) {
                            int64_t a6 = at[6], a7 = at[7], bias6 = (int64_t)((1ULL << qshift) - 1u);
                            s16 = (int32_t)((a6 + (a6 < 0 ? bias6 : 0)) >> qshift);
                            t16 = (int32_t)((a7 + (a7 < 0 ? bias6 : 0)) >> qshift);
                        } else {
                            int64_t qd = qconst;
                            if (qmode == 0u) { qd = at[5] >> 16; if (qd < 1) qd = 1; }
                            s16 = (int32_t)(at[6] / qd);
                            t16 = (int32_t)(at[7] / qd);
                        }
                        unpack_v(ts0.smp(c, rs->tmem, &ts0, s16, t16, filt), v + V_TEX0);
                        if (need_t1) unpack_v(ts1.smp(c, rs->tmem, &ts1, s16, t16, filt), v + V_TEX1);
                    }
                    if (shade_f(&pp, v, &row[px])) {
                        if (zu) zrow[px] = (uint16_t)zv;
                        shaded++;
                    }
                }
                for (k = 0; k < 8u; k++) at[k] += DX[k];
            }
        }
        e_c[0] += e_dy[0]; e_c[1] += e_dy[1]; e_c[2] += e_dy[2];
        for (k = 0; k < 8u; k++) A0[k] += DY[k];
    }
    *c->samples += shaded;
    (*c->prims)++;
}

static void fill_rect_fast(rctx_t *c, const n64_rstate_t *rs, uint32_t w0, uint32_t w1)
{
    uint32_t cyc = (rs->om_hi >> 20) & 3u;
    int32_t xl = (int32_t)((w0 >> 12) & 0xFFFu), yl = (int32_t)(w0 & 0xFFFu);
    int32_t xh = (int32_t)((w1 >> 12) & 0xFFFu), yh = (int32_t)(w1 & 0xFFFu);
    int32_t sx0, sy0, sx1, sy1, x0, y0, x1, y1, px, py;
    int ztarget = rs->cimg_addr == rs->zimg_addr && rs->zimg_addr != 0u;
    if (cyc >= 2u) { xl += 4; yl += 4; }
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    x0 = (int32_t)((((int64_t)xh << 14) * c->kx) >> 32); x1 = (int32_t)((((int64_t)xl << 14) * c->kx) >> 32);
    y0 = (int32_t)((((int64_t)yh << 14) * c->ky) >> 32); y1 = (int32_t)((((int64_t)yl << 14) * c->ky) >> 32);
    if (x0 < sx0) x0 = sx0;
    if (y0 < sy0) y0 = sy0;
    if (x1 > sx1) x1 = sx1;
    if (y1 > sy1) y1 = sy1;
    if (x0 >= x1 || y0 >= y1) return;
    if (ztarget) {
        uint16_t zv = (uint16_t)((rs->fill >> 16) >= 0xFFF0u ? 0xFFFFu : (rs->fill >> 16));
        for (py = y0; py < y1; py++) {
            uint16_t *zr = c->z + (size_t)(py - (int32_t)c->y0) * c->sw;
            for (px = x0; px < x1; px++) zr[px] = zv;
        }
        return;
    }
    if (cyc == 3u) {
        uint32_t col = fill_rgb(rs);
        for (py = y0; py < y1; py++) {
            uint32_t *r = c->strip + (size_t)(py - (int32_t)c->y0) * c->sw;
            for (px = x0; px < x1; px++) r[px] = col;
        }
    } else {
        px_pipe_t pp;
        int32_t v[V_N];
        pipe_setup(rs, &pp);
        v_init(v);
        unpack_v(rs->prim, v + V_PRIM); unpack_v(rs->env, v + V_ENV);
        v[V_LOD] = (int32_t)rs->prim_lod;
        for (py = y0; py < y1; py++) {
            uint32_t *r = c->strip + (size_t)(py - (int32_t)c->y0) * c->sw;
            for (px = x0; px < x1; px++) shade_f(&pp, v, &r[px]);
        }
    }
    (*c->prims)++;
}

static void tex_rect_fast(rctx_t *c, const n64_rstate_t *rs, const uint32_t *w)
{
    uint32_t cyc = (rs->om_hi >> 20) & 3u, tile = (w[1] >> 24) & 7u;
    int32_t xl = (int32_t)((w[0] >> 12) & 0xFFFu), yl = (int32_t)(w[0] & 0xFFFu);
    int32_t xh = (int32_t)((w[1] >> 12) & 0xFFFu), yh = (int32_t)(w[1] & 0xFFFu);
    int32_t s0 = (int16_t)(w[2] >> 16), t0 = (int16_t)(w[2] & 0xFFFFu);
    int32_t dsdx = (int16_t)(w[3] >> 16), dtdy = (int16_t)(w[3] & 0xFFFFu);
    int32_t sx0, sy0, sx1, sy1, x0, y0, x1, y1, px, py;
    int filt = ((rs->om_hi >> 12) & 3u) >= 2u && cyc < 2u;
    int flip = ((w[0] >> 24) & 0x3Fu) == 0x25u;
    px_pipe_t pp;
    tex_setup_t ts;
    int32_t v[V_N];
    int64_t nxq0, nxr0, stq, str;
    if (cyc >= 2u) { xl += 4; yl += 4; }
    if (cyc == 2u) dsdx >>= 2;
    scissor_virtual(c, rs, &sx0, &sy0, &sx1, &sy1);
    x0 = (int32_t)((((int64_t)xh << 14) * c->kx) >> 32); x1 = (int32_t)((((int64_t)xl << 14) * c->kx) >> 32);
    y0 = (int32_t)((((int64_t)yh << 14) * c->ky) >> 32); y1 = (int32_t)((((int64_t)yl << 14) * c->ky) >> 32);
    if (x0 < sx0) x0 = sx0;
    if (y0 < sy0) y0 = sy0;
    if (x1 > sx1) x1 = sx1;
    if (y1 > sy1) y1 = sy1;
    if (x0 >= x1 || y0 >= y1) return;
    pipe_setup(rs, &pp);
    tex_bind(c, rs, tile, &ts);
    v_init(v);
    unpack_v(rs->prim, v + V_PRIM); unpack_v(rs->env, v + V_ENV);
    /* nx(px) = ((px*256+128) << 24) / kx, numerators >= 0 (x0 >= 0): step by 2^32 */
    {
        int64_t num = ((int64_t)x0 * 256 + 128) << 24;
        nxq0 = num / c->kx; nxr0 = num % c->kx;
        stq = ((int64_t)1 << 32) / c->kx; str = ((int64_t)1 << 32) % c->kx;
    }
    for (py = y0; py < y1; py++) {
        int64_t ny = (((int64_t)py * 256 + 128) << 24) / c->ky;
        int64_t dyr = ny - ((int64_t)yh << 14);
        int64_t nq = nxq0, nr = nxr0;
        uint32_t *rowp = c->strip + (size_t)(py - (int32_t)c->y0) * c->sw;
        for (px = x0; px < x1; px++) {
            int64_t dx = nq - ((int64_t)xh << 14), dy = dyr;
            int64_t s16, t16;
            uint32_t *dst = &rowp[px];
            nq += stq; nr += str;
            if (nr >= c->kx) { nr -= c->kx; nq++; }
            if (flip) { int64_t tmp = dx; dx = dy; dy = tmp; }
            s16 = ((int64_t)s0 << 11) + ((dx * dsdx) >> 10);
            t16 = ((int64_t)t0 << 11) + ((dy * dtdy) >> 10);
            unpack_v(ts.smp(c, rs->tmem, &ts, (int32_t)s16, (int32_t)t16, filt), v + V_TEX0);
            if (cyc == 2u) {
                if ((rs->om_lo & 3u) && v[V_TEX0 + 3] == 0) continue;
                *dst = ((uint32_t)v[V_TEX0] << 16) | ((uint32_t)v[V_TEX0 + 1] << 8) | (uint32_t)v[V_TEX0 + 2];
            } else {
                v[V_SHADE] = v[V_SHADE + 1] = v[V_SHADE + 2] = v[V_SHADE + 3] = 0;
                shade_f(&pp, v, dst);
            }
        }
    }
    (*c->prims)++;
}

/* ---------------------------------------------------------- replay */
static void replay(rctx_t *c, n64_rstate_t *rs)
{
    const n64_snap_t *s = c->s;
    uint32_t pos = 0, guard;
    for (guard = 0; guard < N64_RDP_MAX_WORDS && pos + 1u < s->rdp_n; guard++) {
        const uint32_t *w = &s->rdp[pos];
        uint32_t op = (w[0] >> 24) & 0x3Fu;
        switch (op) {
        case N64_OP_HS_TRI:
            if (pos + N64_TRI_WORDS > s->rdp_n) return;
            if (c->ref) draw_tri(c, rs, w); else draw_tri_fast(c, rs, w);
            pos += N64_TRI_WORDS;
            continue;
        case N64_OP_HS_TIMG:
            if (pos + 4u > s->rdp_n) return;
            rs->timg_fmt = (w[0] >> 21) & 7u; rs->timg_siz = (w[0] >> 19) & 3u;
            rs->timg_w = (w[0] & 0x3FFu) + 1u; rs->timg_off = w[1]; rs->timg_row0 = w[2]; rs->timg_len = w[3];
            pos += 4u;
            continue;
        case 0x24: case 0x25:
            if (pos + 4u > s->rdp_n) return;
            if (c->ref) tex_rect(c, rs, w); else tex_rect_fast(c, rs, w);
            pos += 4u;
            continue;
        case 0x2D: rs->sc_x0 = (int32_t)((w[0] >> 12) & 0xFFFu); rs->sc_y0 = (int32_t)(w[0] & 0xFFFu);
                   rs->sc_x1 = (int32_t)((w[1] >> 12) & 0xFFFu); rs->sc_y1 = (int32_t)(w[1] & 0xFFFu); break;
        case 0x2E: rs->prim_z = w[1] >> 16; break;
        case 0x2F: rs->om_hi = w[0] & 0x00FFFFFFu; rs->om_lo = w[1]; c->pal_valid = 0; break;
        case 0x30: case 0x33: case 0x34: do_load(s, rs, op, w[0], w[1]); c->pal_valid = 0; break;
        case 0x32: set_tile_size(rs, w[0], w[1]); break;
        case 0x35: set_tile(rs, w[0], w[1]); break;
        case 0x36: if (c->ref) fill_rect(c, rs, w[0], w[1]); else fill_rect_fast(c, rs, w[0], w[1]); break;
        case 0x37: rs->fill = w[1]; break;
        case 0x38: rs->fog = w[1]; break;
        case 0x39: rs->blend = w[1]; break;
        case 0x3A: rs->prim = w[1]; rs->prim_lod = w[0] & 0xFFu; break;
        case 0x3B: rs->env = w[1]; break;
        case 0x3C: rs->comb_hi = w[0] & 0x00FFFFFFu; rs->comb_lo = w[1]; break;
        case 0x3E: rs->zimg_addr = w[1]; break;
        case 0x3F: rs->cimg_addr = w[1]; rs->cimg_w = (w[0] & 0xFFFu) + 1u; rs->cimg_siz = (w[0] >> 19) & 3u; break;
        default: break;
        }
        pos += 2u;
    }
}

static void strip_cb(void *user, uint32_t *strip, uint32_t strip_w, uint32_t y0_hi, uint32_t rows_hi, uint32_t ss)
{
    rctx_t *c = (rctx_t *)user;
    n64_rstate_t *rs = c->rs;
    size_t i, cnt = (size_t)strip_w * rows_hi;
    c->strip = strip; c->sw = strip_w; c->y0 = y0_hi; c->rows = rows_hi; c->ss = ss;
    for (i = 0; i < cnt; i++) { strip[i] = 0; c->z[i] = 0xFFFFu; }
    n64_rstate_reset(rs);
    c->pal_valid = 0;
    rs->sc_x1 = (int32_t)(c->nw << 2); rs->sc_y1 = (int32_t)(c->nh << 2);
    replay(c, rs);
}

static void ctx_init(rctx_t *c, const hs_n64_t *n, uint32_t ss, int fxaa)
{
    const n64_snap_t *s = n64_front(n);
    memset(c, 0, sizeof(*c));
    c->s = s;
    c->nw = s->native_w ? s->native_w : 320u;
    c->nh = s->native_h ? s->native_h : 240u;
    c->bilinear4 = (n->cfg_flags & HS_N64_CFG_BILINEAR4) != 0;
    c->ref = (n->cfg_flags & HS_N64_CFG_REF_RDP) != 0;
    if (fxaa) {
        c->kx = ((int64_t)HS_FB_W << 16) / c->nw;
        c->ky = ((int64_t)HS_FB_H << 16) / c->nh;
    } else {
        c->kx = ((int64_t)HS_FB_W * ss << 16) / c->nw;
        c->ky = ((int64_t)HS_FB_H * ss << 16) / c->nh;
    }
}

/* Same strip partition as hs_ssaa_render (the strip y0 fixes each primitive's
 * setup origin, so the partition is part of the bit-exact output). Strips
 * index, index+count, ... are rendered: any subset of strips writes exactly
 * the rows it would write in a full render. At 1x with a 640-wide target the
 * strip is rendered in place (hs_ssaa_downsample at ss=1 is a plain copy). */
static void render_strips(rctx_t *c, uint32_t *strip_buf, uint32_t ss, uint32_t *dst, size_t stride,
                          uint32_t index, uint32_t count)
{
    uint32_t rows = hs_ssaa_rows_per_strip(ss), y, k = 0;
    for (y = 0; y < HS_FB_H; y += rows, k++) {
        uint32_t r = rows;
        if (k % count != index) continue;
        if (y + r > HS_FB_H) r = HS_FB_H - y;
        if (ss == 1u && stride == HS_FB_W && !c->ref) {
            strip_cb(c, dst + (size_t)y * stride, HS_FB_W, y, r, 1u);
        } else {
            strip_cb(c, strip_buf, HS_FB_W * ss, y * ss, r * ss, ss);
            hs_ssaa_downsample(strip_buf, ss, r, dst + (size_t)y * stride, stride);
        }
    }
}

void n64_render(hs_n64_t *n, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    n64_video_t *v = &n->video;
    rctx_t c;
    uint32_t ss = hs_ssaa_factor(mode);
    ctx_init(&c, n, ss, mode == HS_AA_FXAA);
    c.v = v; c.z = v->zstrip; c.rs = &v->rs; c.samples = &v->samples_shaded; c.prims = &v->prims_drawn;
    v->src = c.s;
    v->samples_shaded = 0;
    v->prims_drawn = 0;
    if (mode == HS_AA_FXAA) {
        if (c.ref) hs_ssaa_render(1u, strip_cb, &c, v->strip, v->fb1x, HS_FB_W);
        else render_strips(&c, v->strip, 1u, v->fb1x, HS_FB_W, 0u, 1u);
        hs_fxaa(v->fb1x, HS_FB_W, fb, stride, HS_FB_W, HS_FB_H, v->luma);
        return;
    }
    if (c.ref) hs_ssaa_render(ss, strip_cb, &c, v->strip, fb, stride);
    else render_strips(&c, v->strip, ss, fb, stride, 0u, 1u);
}

/* ---- slices: strip-interleaved partitions of one frame for separate cores */
size_t hs_n64_slice_size(void) { return sizeof(n64_slice_scratch_t); }

int hs_n64_render_slice_ctx(const void *core, void *scratch, size_t scratch_size,
                            uint32_t *fb, size_t stride, hs_video_mode_t mode,
                            uint32_t index, uint32_t count)
{
    const hs_n64_t *n = (const hs_n64_t *)core;
    n64_slice_scratch_t *sc = (n64_slice_scratch_t *)scratch;
    rctx_t c;
    uint32_t ss = hs_ssaa_factor(mode);
    if (!n || !sc || scratch_size < sizeof(*sc) || !fb || mode == HS_AA_FXAA) return -1;
    if (count < 1u || count > HS_N64_MAX_SLICES || index >= count) return -1;
    ctx_init(&c, n, ss, 0);
    c.v = NULL; c.z = sc->zstrip; c.rs = &sc->rs; c.samples = &sc->samples; c.prims = &sc->prims;
    sc->samples = 0; sc->prims = 0;
    render_strips(&c, sc->strip, ss, fb, stride, index, count);
    return 0;
}

void hs_n64_render_slice(void *core, uint32_t *fb, size_t stride, hs_video_mode_t mode,
                         uint32_t index, uint32_t count)
{
    hs_n64_t *n = (hs_n64_t *)core;
    n64_video_t *v = &n->video;
    rctx_t c;
    uint32_t ss = hs_ssaa_factor(mode);
    if (mode == HS_AA_FXAA || count < 1u || count > HS_N64_MAX_SLICES || index >= count) {
        if (index == 0u) n64_render(n, fb, stride, mode);
        return;
    }
    ctx_init(&c, n, ss, 0);
    c.v = v; c.z = v->zstrip; c.rs = &v->rs; c.samples = &v->samples_shaded; c.prims = &v->prims_drawn;
    if (index == 0u) { v->samples_shaded = 0; v->prims_drawn = 0; v->src = c.s; }
    render_strips(&c, v->strip, ss, fb, stride, index, count);
}

/* ---------------------------------------------- Panfrost/GLES backend walk */
static void bk_state(const n64_rstate_t *rs, n64_bk_state_t *st, uint32_t tile, uint32_t flags)
{
    uint32_t i;
    st->combine_hi = rs->comb_hi; st->combine_lo = rs->comb_lo;
    st->othermode_hi = rs->om_hi; st->othermode_lo = rs->om_lo;
    st->prim_color = rs->prim; st->env_color = rs->env; st->fog_color = rs->fog;
    st->blend_color = rs->blend; st->fill_color = rs->fill;
    st->tile = tile; st->tmem = rs->tmem; st->flags = flags;
    for (i = 0; i < 8u; i++) {
        st->tile_desc[i][0] = rs->tile_raw[i][0]; st->tile_desc[i][1] = rs->tile_raw[i][1];
        st->tile_size[i][0] = rs->tsize_raw[i][0]; st->tile_size[i][1] = rs->tsize_raw[i][1];
    }
}

uint32_t hs_n64_walk_backend(const void *core, const n64_rdp_backend_t *bk, uint32_t ss)
{
    const hs_n64_t *n = (const hs_n64_t *)core;
    const n64_snap_t *s = n64_front(n);
    n64_rstate_t rs;
    n64_bk_state_t st;
    uint32_t pos = 0, guard, prims = 0, i;
    n64_rstate_reset(&rs);
    if (bk->begin_frame) bk->begin_frame(bk->user, s->native_w ? s->native_w : 320u, s->native_h ? s->native_h : 240u, ss);
    for (guard = 0; guard < N64_RDP_MAX_WORDS && pos + 1u < s->rdp_n; guard++) {
        const uint32_t *w = &s->rdp[pos];
        uint32_t op = (w[0] >> 24) & 0x3Fu;
        if (op == N64_OP_HS_TRI) {
            n64_bk_vertex_t v[3];
            if (pos + N64_TRI_WORDS > s->rdp_n) break;
            for (i = 0; i < 3u; i++) {
                const uint32_t *p = &w[1 + i * 7u];
                v[i].x = (int32_t)p[0]; v[i].y = (int32_t)p[1]; v[i].z = p[2]; v[i].w = (int32_t)p[3];
                v[i].s = (int32_t)p[4]; v[i].t = (int32_t)p[5]; v[i].rgba = p[6];
            }
            bk_state(&rs, &st, (w[0] >> 8) & 7u, w[0] & 0xFFu);
            if (bk->draw_tri) bk->draw_tri(bk->user, v, &st);
            prims++;
            pos += N64_TRI_WORDS;
            continue;
        }
        if (op == N64_OP_HS_TIMG) {
            if (pos + 4u > s->rdp_n) break;
            rs.timg_fmt = (w[0] >> 21) & 7u; rs.timg_siz = (w[0] >> 19) & 3u;
            rs.timg_w = (w[0] & 0x3FFu) + 1u; rs.timg_off = w[1]; rs.timg_row0 = w[2]; rs.timg_len = w[3];
            pos += 4u;
            continue;
        }
        if (op == 0x24u || op == 0x25u) {
            if (pos + 4u > s->rdp_n) break;
            bk_state(&rs, &st, (w[1] >> 24) & 7u, 0);
            if (bk->tex_rect)
                bk->tex_rect(bk->user, (int32_t)((w[1] >> 12) & 0xFFFu), (int32_t)(w[1] & 0xFFFu),
                             (int32_t)((w[0] >> 12) & 0xFFFu), (int32_t)(w[0] & 0xFFFu),
                             (int16_t)(w[2] >> 16), (int16_t)(w[2] & 0xFFFFu),
                             (int16_t)(w[3] >> 16), (int16_t)(w[3] & 0xFFFFu), &st);
            prims++;
            pos += 4u;
            continue;
        }
        switch (op) {
        case 0x2D: rs.sc_x0 = (int32_t)((w[0] >> 12) & 0xFFFu); rs.sc_y0 = (int32_t)(w[0] & 0xFFFu);
                   rs.sc_x1 = (int32_t)((w[1] >> 12) & 0xFFFu); rs.sc_y1 = (int32_t)(w[1] & 0xFFFu); break;
        case 0x2E: rs.prim_z = w[1] >> 16; break;
        case 0x2F: rs.om_hi = w[0] & 0x00FFFFFFu; rs.om_lo = w[1]; break;
        case 0x30: case 0x33: case 0x34: do_load(s, &rs, op, w[0], w[1]); break;
        case 0x32: set_tile_size(&rs, w[0], w[1]); break;
        case 0x35: set_tile(&rs, w[0], w[1]); break;
        case 0x36:
            bk_state(&rs, &st, 0, 0);
            if (bk->fill_rect)
                bk->fill_rect(bk->user, (int32_t)((w[1] >> 12) & 0xFFFu), (int32_t)(w[1] & 0xFFFu),
                              (int32_t)((w[0] >> 12) & 0xFFFu), (int32_t)(w[0] & 0xFFFu),
                              rs.cimg_addr == rs.zimg_addr && rs.zimg_addr != 0u, &st);
            prims++;
            break;
        case 0x37: rs.fill = w[1]; break;
        case 0x38: rs.fog = w[1]; break;
        case 0x39: rs.blend = w[1]; break;
        case 0x3A: rs.prim = w[1]; rs.prim_lod = w[0] & 0xFFu; break;
        case 0x3B: rs.env = w[1]; break;
        case 0x3C: rs.comb_hi = w[0] & 0x00FFFFFFu; rs.comb_lo = w[1]; break;
        case 0x3E: rs.zimg_addr = w[1]; break;
        case 0x3F: rs.cimg_addr = w[1]; rs.cimg_w = (w[0] & 0xFFFu) + 1u; rs.cimg_siz = (w[0] >> 19) & 3u; break;
        default: break;
        }
        pos += 2u;
    }
    if (bk->end_frame) bk->end_frame(bk->user);
    return prims;
}
