/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_rdp_ref.c - FROZEN REFERENCE: the hs-n64 v0 RDP rasterizer exactly as
 * it shipped in hs-cores e15d81a (src/n64_rdp.c), kept only so that
 * tests/test_n64_equiv.c can prove the optimised src/n64_rdp.c bit-identical.
 * The only edits: (a) the four scratch-buffer field paths now point at
 * video.part[0] (strip, zstrip, rs); (b) REF FIX in tex_rect: v0 declared
 * the row offset `dy` outside the pixel loop and the TEXRECT_FLIP (0x25)
 * swap overwrote it, so every pixel after the first in a row sampled s from
 * the previous column. The differential RDP fuzz found it; the fix is also
 * in src/n64_rdp.c. The demo ROM never issues 0x25, so no demo hash moves;
 * (c) REF FIX 2 in d2fx: out-of-range double->int64 casts (UB; x86_64 and
 * AArch64 give different results) now saturate, as in src/n64_rdp.c. Only
 * sliver triangles with huge attribute gradients reach that range; the demo
 * never does, so no demo hash moves.
 * The file is compiled through n64_ref_rdp.c with renamed entry points.
 * Not linked into the core.
 *
 * Original description: clean-room RDP rasterizer for hs-n64 (Phase 1 only).
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
/* REF FIX 2 (see header): saturating. double -> int64 fixed point, round half away from zero. Out-of-range values
 * (attribute gradients of sliver triangles) saturate: a plain C cast is
 * undefined there and x86_64 / AArch64 disagree (INT64_MIN vs saturation). */
static int64_t d2fx(double v)
{
    if (v != v) return 0;
    if (v >= 9223372036854774784.0) return INT64_MAX;
    if (v <= -9223372036854774784.0) return INT64_MIN;
    return (int64_t)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

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
                        c->v->samples_shaded++;
                    }
                }
            }
            e0 += e_dx[0]; e1 += e_dx[1]; e2 += e_dx[2];
            for (k = 0; k < 8u; k++) at[k] += DX[k];
        }
        e_c[0] += e_dy[0]; e_c[1] += e_dy[1]; e_c[2] += e_dy[2];
        for (k = 0; k < 8u; k++) A0[k] += DY[k];
    }
    c->v->prims_drawn++;
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
    c->v->prims_drawn++;
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
        int64_t dyr = ny - ((int64_t)yh << 14);   /* REF FIX (see header): was `dy`, mutated by the flip swap */
        for (px = x0; px < x1; px++) {
            int64_t nx = (((int64_t)px * 256 + 128) << 24) / c->kx;
            int64_t dx = nx - ((int64_t)xh << 14), dy = dyr;
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
    c->v->prims_drawn++;
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
            draw_tri(c, rs, w);
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
            tex_rect(c, rs, w);
            pos += 4u;
            continue;
        case 0x2D: rs->sc_x0 = (int32_t)((w[0] >> 12) & 0xFFFu); rs->sc_y0 = (int32_t)(w[0] & 0xFFFu);
                   rs->sc_x1 = (int32_t)((w[1] >> 12) & 0xFFFu); rs->sc_y1 = (int32_t)(w[1] & 0xFFFu); break;
        case 0x2E: rs->prim_z = w[1] >> 16; break;
        case 0x2F: rs->om_hi = w[0] & 0x00FFFFFFu; rs->om_lo = w[1]; break;
        case 0x30: case 0x33: case 0x34: do_load(s, rs, op, w[0], w[1]); break;
        case 0x32: set_tile_size(rs, w[0], w[1]); break;
        case 0x35: set_tile(rs, w[0], w[1]); break;
        case 0x36: fill_rect(c, rs, w[0], w[1]); break;
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
    n64_rstate_t *rs = &c->v->part[0].rs;
    size_t i, cnt = (size_t)strip_w * rows_hi;
    c->strip = strip; c->sw = strip_w; c->y0 = y0_hi; c->rows = rows_hi; c->ss = ss;
    for (i = 0; i < cnt; i++) { strip[i] = 0; c->z[i] = 0xFFFFu; }
    n64_rstate_reset(rs);
    rs->sc_x1 = (int32_t)(c->nw << 2); rs->sc_y1 = (int32_t)(c->nh << 2);
    replay(c, rs);
}

void n64_render(hs_n64_t *n, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    n64_video_t *v = &n->video;
    const n64_snap_t *s = n64_front(n);
    rctx_t c;
    uint32_t ss = hs_ssaa_factor(mode);
    memset(&c, 0, sizeof(c));
    c.v = v; c.s = s; c.z = v->part[0].zstrip;
    c.nw = s->native_w ? s->native_w : 320u;
    c.nh = s->native_h ? s->native_h : 240u;
    c.bilinear4 = (n->cfg_flags & HS_N64_CFG_BILINEAR4) != 0;
    v->src = s;
    v->samples_shaded = 0;
    v->prims_drawn = 0;
    if (mode == HS_AA_FXAA) {
        c.kx = ((int64_t)HS_FB_W << 16) / c.nw;
        c.ky = ((int64_t)HS_FB_H << 16) / c.nh;
        hs_ssaa_render(1u, strip_cb, &c, v->part[0].strip, v->fb1x, HS_FB_W);
        hs_fxaa(v->fb1x, HS_FB_W, fb, stride, HS_FB_W, HS_FB_H, v->luma);
        return;
    }
    c.kx = ((int64_t)HS_FB_W * ss << 16) / c.nw;
    c.ky = ((int64_t)HS_FB_H * ss << 16) / c.nh;
    hs_ssaa_render(ss, strip_cb, &c, v->part[0].strip, fb, stride);
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
