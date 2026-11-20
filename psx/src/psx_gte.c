/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_gte.c - clean-room PS1 Geometry Transformation Engine (COP2).
 * Implemented from the publicly documented fixed-point equations:
 * 44-bit MAC1..3 overflow checks per accumulation step, 32-bit MAC0
 * checks, IR/colour/SZ/SXY/IR0 saturation with FLAG bits, the RTPS/RTPT
 * IR3 flag quirk, the UNR (Newton-Raphson) perspective divide, the
 * MVMVA cv=FC hardware quirk, screen-XY / Z / colour FIFOs, IRGB/ORGB and
 * LZCS/LZCR. All integer; bit-identical on every host.
 */
#include "psx_internal.h"

/* data register indices */
enum { VXY0 = 0, VZ0, VXY1, VZ1, VXY2, VZ2, RGBC, OTZ, IR0, IR1, IR2, IR3,
       SXY0, SXY1, SXY2, SXYP, SZ0, SZ1, SZ2, SZ3, RGB0, RGB1, RGB2, RES1,
       MAC0, MAC1, MAC2, MAC3, IRGB, ORGB, LZCS, LZCR };
/* control register indices */
enum { RT0 = 0, RT1, RT2, RT3, RT4, TRX, TRY, TRZ, L0, L1, L2, L3, L4,
       RBK, GBK, BBK, LR0, LR1, LR2, LR3, LR4, RFC, GFC, BFC, OFX, OFY, H,
       DQA, DQB, ZSF3, ZSF4, FLAG };

#define F_MAC_POS(i) (1u << (31 - (i)))   /* i=1..3 -> bits 30..28 */
#define F_MAC_NEG(i) (1u << (28 - (i)))   /* i=1..3 -> bits 27..25 */
#define F_IR(i)      (1u << (25 - (i)))   /* i=1..3 -> bits 24..22 */
#define F_COL(i)     (1u << (22 - (i)))   /* i=1..3 -> bits 21..19 */
#define F_SZ         (1u << 18)
#define F_DIV        (1u << 17)
#define F_MAC0_POS   (1u << 16)
#define F_MAC0_NEG   (1u << 15)
#define F_SX         (1u << 14)
#define F_SY         (1u << 13)
#define F_IR0        (1u << 12)

typedef struct gte_ctx { psx_cpu_t *c; uint32_t flag; int sf; int lm; } gte_ctx_t;

static inline int16_t lo16(int32_t v) { return (int16_t)(uint16_t)((uint32_t)v & 0xFFFFu); }
static inline int16_t hi16(int32_t v) { return (int16_t)(uint16_t)((uint32_t)v >> 16); }

/* 44-bit MAC1..3 check of an intermediate value; returns it sign-extended to 44 bits */
static inline int64_t chk44(gte_ctx_t *g, int i, int64_t v)
{
    if (v > 0x7FFFFFFFFFFll) g->flag |= F_MAC_POS(i);
    else if (v < -0x80000000000ll) g->flag |= F_MAC_NEG(i);
    return (int64_t)((uint64_t)v << 20) >> 20;
}

static inline int64_t chk_mac0(gte_ctx_t *g, int64_t v)
{
    if (v > 0x7FFFFFFFll) g->flag |= F_MAC0_POS;
    else if (v < -0x80000000ll) g->flag |= F_MAC0_NEG;
    return v;
}

/* MAC i = value >> (sf*12); stored (32-bit) and returned */
static inline int32_t set_mac(gte_ctx_t *g, int i, int64_t v)
{
    int64_t r;
    v = chk44(g, i, v);
    r = g->sf ? (v >> 12) : v;
    g->c->gd[MAC0 + i] = (int32_t)r;
    return (int32_t)r;
}

static inline int32_t sat_ir(gte_ctx_t *g, int i, int32_t v, int lm)
{
    int32_t lo = lm ? 0 : -0x8000;
    if (v < lo) { g->flag |= F_IR(i); return lo; }
    if (v > 0x7FFF) { g->flag |= F_IR(i); return 0x7FFF; }
    return v;
}

static inline void set_ir(gte_ctx_t *g, int i, int32_t mac, int lm)
{
    g->c->gd[IR0 + i] = sat_ir(g, i, mac, lm);
}

static inline uint32_t sat_col(gte_ctx_t *g, int i, int32_t v)
{
    if (v < 0) { g->flag |= F_COL(i); return 0; }
    if (v > 255) { g->flag |= F_COL(i); return 255; }
    return (uint32_t)v;
}

static void push_color(gte_ctx_t *g)
{
    psx_cpu_t *c = g->c;
    uint32_t r = sat_col(g, 1, c->gd[MAC1] >> 4);
    uint32_t gg = sat_col(g, 2, c->gd[MAC2] >> 4);
    uint32_t b = sat_col(g, 3, c->gd[MAC3] >> 4);
    uint32_t code = (uint32_t)c->gd[RGBC] >> 24;
    c->gd[RGB0] = c->gd[RGB1];
    c->gd[RGB1] = c->gd[RGB2];
    c->gd[RGB2] = (int32_t)(r | (gg << 8) | (b << 16) | (code << 24));
}

static void push_sz(gte_ctx_t *g, int64_t v)
{
    psx_cpu_t *c = g->c;
    if (v < 0) { v = 0; g->flag |= F_SZ; }
    else if (v > 0xFFFF) { v = 0xFFFF; g->flag |= F_SZ; }
    c->gd[SZ0] = c->gd[SZ1];
    c->gd[SZ1] = c->gd[SZ2];
    c->gd[SZ2] = c->gd[SZ3];
    c->gd[SZ3] = (int32_t)v;
}

static void push_sxy(gte_ctx_t *g, int64_t x, int64_t y)
{
    psx_cpu_t *c = g->c;
    if (x < -0x400) { x = -0x400; g->flag |= F_SX; } else if (x > 0x3FF) { x = 0x3FF; g->flag |= F_SX; }
    if (y < -0x400) { y = -0x400; g->flag |= F_SY; } else if (y > 0x3FF) { y = 0x3FF; g->flag |= F_SY; }
    c->gd[SXY0] = c->gd[SXY1];
    c->gd[SXY1] = c->gd[SXY2];
    c->gd[SXY2] = (int32_t)(((uint32_t)x & 0xFFFFu) | ((uint32_t)y << 16));
}

/* UNR division: (H*0x20000/SZ3+1)/2 approximated exactly like hardware */
static uint32_t unr_div(gte_ctx_t *g, uint32_t h, uint32_t sz3)
{
    uint32_t z, n, d, u, idx;
    uint64_t q;
    if (h >= sz3 * 2u) { g->flag |= F_DIV; return 0x1FFFFu; }
    z = 0;
    { uint32_t t = sz3 & 0xFFFFu; uint32_t k; for (k = 0; k < 16u && !(t & 0x8000u); k++) { t <<= 1; z++; } }
    n = h << z;
    d = sz3 << z;
    idx = (d - 0x7FC0u) >> 7;
    {   int32_t t = (int32_t)((0x40000u / (idx + 0x100u) + 1u) / 2u) - 0x101;
        u = (uint32_t)(t < 0 ? 0 : t) + 0x101u; }
    d = (uint32_t)((0x2000080u - (uint64_t)d * u) >> 8);
    d = (uint32_t)((0x0000080u + (uint64_t)d * u) >> 8);
    q = (((uint64_t)n * d) + 0x8000u) >> 16;
    if (q > 0x1FFFFu) { q = 0x1FFFFu; g->flag |= F_DIV; }
    return (uint32_t)q;
}

/* ---- matrix / vector helpers ---- */
static void mat(const psx_cpu_t *c, int base, int16_t m[3][3])
{
    m[0][0] = lo16(c->gc[base + 0]); m[0][1] = hi16(c->gc[base + 0]);
    m[0][2] = lo16(c->gc[base + 1]); m[1][0] = hi16(c->gc[base + 1]);
    m[1][1] = lo16(c->gc[base + 2]); m[1][2] = hi16(c->gc[base + 2]);
    m[2][0] = lo16(c->gc[base + 3]); m[2][1] = hi16(c->gc[base + 3]);
    m[2][2] = lo16(c->gc[base + 4]);
}

static void vec(const psx_cpu_t *c, int n, int32_t v[3])
{
    if (n < 3) {
        v[0] = lo16(c->gd[VXY0 + 2 * n]);
        v[1] = hi16(c->gd[VXY0 + 2 * n]);
        v[2] = lo16(c->gd[VZ0 + 2 * n]);
    } else {
        v[0] = c->gd[IR1]; v[1] = c->gd[IR2]; v[2] = c->gd[IR3];
    }
}

/* MAC = (T<<12 + M*V) >> sf ; IR = sat(MAC, lm) */
static void mvmva_core(gte_ctx_t *g, int16_t m[3][3], const int32_t v[3], const int32_t t[3], int lm)
{
    int i;
    for (i = 0; i < 3; i++) {
        int64_t a = chk44(g, i + 1, (int64_t)t[i] * 4096);
        a = chk44(g, i + 1, a + (int64_t)m[i][0] * v[0]);
        a = chk44(g, i + 1, a + (int64_t)m[i][1] * v[1]);
        a = a + (int64_t)m[i][2] * v[2];
        set_ir(g, i + 1, set_mac(g, i + 1, a), lm);
    }
}

static void rtp(gte_ctx_t *g, int n, int last)
{
    psx_cpu_t *c = g->c;
    int16_t m[3][3];
    int32_t v[3], t[3];
    int64_t mac3_raw = 0;
    int i;
    uint32_t q;
    int64_t sx, sy;
    mat(c, RT0, m);
    vec(c, n, v);
    t[0] = c->gc[TRX]; t[1] = c->gc[TRY]; t[2] = c->gc[TRZ];
    for (i = 0; i < 3; i++) {
        int64_t a = chk44(g, i + 1, (int64_t)t[i] * 4096);
        a = chk44(g, i + 1, a + (int64_t)m[i][0] * v[0]);
        a = chk44(g, i + 1, a + (int64_t)m[i][1] * v[1]);
        a = a + (int64_t)m[i][2] * v[2];
        if (i < 2) {
            set_ir(g, i + 1, set_mac(g, i + 1, a), g->lm);
        } else {
            int32_t mac3 = set_mac(g, 3, a);
            int32_t r3 = mac3, lo = g->lm ? 0 : -0x8000;
            int64_t chk = chk44(g, 3, a) >> 12;      /* flag uses the sf=1 value */
            mac3_raw = chk44(g, 3, a);
            if (r3 < lo) r3 = lo; else if (r3 > 0x7FFF) r3 = 0x7FFF;
            c->gd[IR3] = r3;
            if (chk < -0x8000 || chk > 0x7FFF) g->flag |= F_IR(3);
        }
    }
    push_sz(g, mac3_raw >> 12);
    q = unr_div(g, (uint32_t)c->gc[H] & 0xFFFFu, (uint32_t)c->gd[SZ3]);
    sx = chk_mac0(g, (int64_t)q * c->gd[IR1] + c->gc[OFX]);
    sy = chk_mac0(g, (int64_t)q * c->gd[IR2] + c->gc[OFY]);
    c->gd[MAC0] = (int32_t)sy;
    push_sxy(g, sx >> 16, sy >> 16);
    if (last) {
        int64_t dq = chk_mac0(g, (int64_t)q * lo16(c->gc[DQA]) + c->gc[DQB]);
        int64_t ir0 = dq >> 12;
        c->gd[MAC0] = (int32_t)dq;
        if (ir0 < 0) { ir0 = 0; g->flag |= F_IR0; } else if (ir0 > 0x1000) { ir0 = 0x1000; g->flag |= F_IR0; }
        c->gd[IR0] = (int32_t)ir0;
    }
}

/* MAC = MAC + (FC - MAC) * IR0 (the "interpolate toward far colour" step).
 * Input mac[] are full-precision (<<12 domain) values. */
static void interp_fc(gte_ctx_t *g, int64_t mac[3])
{
    psx_cpu_t *c = g->c;
    int32_t fc[3];
    int i;
    fc[0] = c->gc[RFC]; fc[1] = c->gc[GFC]; fc[2] = c->gc[BFC];
    for (i = 0; i < 3; i++) {
        int64_t d = chk44(g, i + 1, ((int64_t)fc[i] * 4096) - mac[i]);
        int32_t ir = sat_ir(g, i + 1, (int32_t)(g->sf ? (d >> 12) : d), 0);
        mac[i] = chk44(g, i + 1, (int64_t)ir * c->gd[IR0] + mac[i]);
    }
}

static void finish_color(gte_ctx_t *g, int64_t mac[3])
{
    int i;
    for (i = 0; i < 3; i++)
        set_ir(g, i + 1, set_mac(g, i + 1, mac[i]), g->lm);
    push_color(g);
}

/* light pipeline helpers */
static void light_llm(gte_ctx_t *g, int n)
{
    int16_t m[3][3];
    int32_t v[3], z[3] = { 0, 0, 0 };
    mat(g->c, L0, m);
    vec(g->c, n, v);
    mvmva_core(g, m, v, z, g->lm);
}

static void light_lcm(gte_ctx_t *g)
{
    psx_cpu_t *c = g->c;
    int16_t m[3][3];
    int32_t v[3], bk[3];
    mat(c, LR0, m);
    vec(c, 3, v);
    bk[0] = c->gc[RBK]; bk[1] = c->gc[GBK]; bk[2] = c->gc[BBK];
    mvmva_core(g, m, v, bk, g->lm);
}

/* [MAC] = [R*IR1, G*IR2, B*IR3] << 4 ; mode: 0 plain (NCC/CC), 1 + FC interp (NCD/CDP/DCPL) */
static void color_mul(gte_ctx_t *g, int interp)
{
    psx_cpu_t *c = g->c;
    uint32_t rgbc = (uint32_t)c->gd[RGBC];
    int64_t mac[3];
    mac[0] = ((int64_t)(rgbc & 0xFFu) * c->gd[IR1]) << 4;
    mac[1] = ((int64_t)((rgbc >> 8) & 0xFFu) * c->gd[IR2]) << 4;
    mac[2] = ((int64_t)((rgbc >> 16) & 0xFFu) * c->gd[IR3]) << 4;
    if (interp)
        interp_fc(g, mac);
    finish_color(g, mac);
}

static void nc_common(gte_ctx_t *g, int n, int kind) /* kind 0 NCS, 1 NCC, 2 NCD */
{
    light_llm(g, n);
    light_lcm(g);
    if (kind == 0) {
        push_color(g);
    } else {
        color_mul(g, kind == 2);
    }
}

static void dpc(gte_ctx_t *g, uint32_t rgb)
{
    int64_t mac[3];
    mac[0] = (int64_t)(rgb & 0xFFu) << 16;
    mac[1] = (int64_t)((rgb >> 8) & 0xFFu) << 16;
    mac[2] = (int64_t)((rgb >> 16) & 0xFFu) << 16;
    interp_fc(g, mac);
    finish_color(g, mac);
}

void psx_gte_command(psx_cpu_t *c, uint32_t cmd)
{
    gte_ctx_t g;
    uint32_t op = cmd & 0x3Fu;
    g.c = c;
    g.flag = 0;
    g.sf = (cmd >> 19) & 1u;
    g.lm = (cmd >> 10) & 1u;

    switch (op) {
    case 0x01: rtp(&g, 0, 1); break;                                   /* RTPS */
    case 0x30: rtp(&g, 0, 0); rtp(&g, 1, 0); rtp(&g, 2, 1); break;    /* RTPT */
    case 0x06: {                                                       /* NCLIP */
        int64_t x0 = lo16(c->gd[SXY0]), y0 = hi16(c->gd[SXY0]);
        int64_t x1 = lo16(c->gd[SXY1]), y1 = hi16(c->gd[SXY1]);
        int64_t x2 = lo16(c->gd[SXY2]), y2 = hi16(c->gd[SXY2]);
        int64_t v = x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1;
        c->gd[MAC0] = (int32_t)chk_mac0(&g, v);
        } break;
    case 0x2D: case 0x2E: {                                            /* AVSZ3 / AVSZ4 */
        int64_t v;
        if (op == 0x2D)
            v = (int64_t)lo16(c->gc[ZSF3]) * (c->gd[SZ1] + c->gd[SZ2] + c->gd[SZ3]);
        else
            v = (int64_t)lo16(c->gc[ZSF4]) * (c->gd[SZ0] + c->gd[SZ1] + c->gd[SZ2] + c->gd[SZ3]);
        v = chk_mac0(&g, v);
        c->gd[MAC0] = (int32_t)v;
        v >>= 12;
        if (v < 0) { v = 0; g.flag |= F_SZ; } else if (v > 0xFFFF) { v = 0xFFFF; g.flag |= F_SZ; }
        c->gd[OTZ] = (int32_t)v;
        } break;
    case 0x12: {                                                       /* MVMVA */
        uint32_t mx = (cmd >> 17) & 3u, vv = (cmd >> 15) & 3u, cv = (cmd >> 13) & 3u;
        int16_t m[3][3];
        int32_t v[3], t[3] = { 0, 0, 0 };
        int i;
        if (mx == 0) mat(c, RT0, m);
        else if (mx == 1) mat(c, L0, m);
        else if (mx == 2) mat(c, LR0, m);
        else {   /* "garbage" matrix documented for mx=3 */
            int32_t r = (int32_t)(((uint32_t)c->gd[RGBC] & 0xFFu) << 4);
            m[0][0] = (int16_t)-r; m[0][1] = (int16_t)r; m[0][2] = (int16_t)c->gd[IR0];
            m[1][0] = lo16(c->gc[RT1]); m[1][1] = lo16(c->gc[RT1]); m[1][2] = lo16(c->gc[RT1]);
            m[2][0] = lo16(c->gc[RT2]); m[2][1] = lo16(c->gc[RT2]); m[2][2] = lo16(c->gc[RT2]);
        }
        vec(c, (int)vv, v);
        if (cv == 0) { t[0] = c->gc[TRX]; t[1] = c->gc[TRY]; t[2] = c->gc[TRZ]; }
        else if (cv == 1) { t[0] = c->gc[RBK]; t[1] = c->gc[GBK]; t[2] = c->gc[BBK]; }
        else if (cv == 2) { t[0] = c->gc[RFC]; t[1] = c->gc[GFC]; t[2] = c->gc[BFC]; }
        if (cv == 2) {
            /* hardware quirk: FC + first column only affects flags, result = M12*VY + M13*VZ */
            for (i = 0; i < 3; i++) {
                int64_t a = chk44(&g, i + 1, (int64_t)t[i] * 4096);
                a = chk44(&g, i + 1, a + (int64_t)m[i][0] * v[0]);
                (void)sat_ir(&g, i + 1, (int32_t)(g.sf ? (a >> 12) : a), 0);
                a = chk44(&g, i + 1, (int64_t)m[i][1] * v[1]);
                a = a + (int64_t)m[i][2] * v[2];
                set_ir(&g, i + 1, set_mac(&g, i + 1, a), g.lm);
            }
        } else {
            mvmva_core(&g, m, v, t, g.lm);
        }
        } break;
    case 0x0C: {                                                       /* OP */
        int64_t d1 = lo16(c->gc[RT0]), d2 = lo16(c->gc[RT2]), d3 = lo16(c->gc[RT4]);
        int64_t i1 = c->gd[IR1], i2 = c->gd[IR2], i3 = c->gd[IR3];
        set_ir(&g, 1, set_mac(&g, 1, i3 * d2 - i2 * d3), g.lm);
        set_ir(&g, 2, set_mac(&g, 2, i1 * d3 - i3 * d1), g.lm);
        set_ir(&g, 3, set_mac(&g, 3, i2 * d1 - i1 * d2), g.lm);
        } break;
    case 0x28: {                                                       /* SQR */
        int i;
        for (i = 1; i <= 3; i++) {
            int64_t ir = c->gd[IR0 + i];
            set_ir(&g, i, set_mac(&g, i, ir * ir), g.lm);
        }
        } break;
    case 0x1E: nc_common(&g, 0, 0); break;                             /* NCS */
    case 0x20: nc_common(&g, 0, 0); nc_common(&g, 1, 0); nc_common(&g, 2, 0); break; /* NCT */
    case 0x1B: nc_common(&g, 0, 1); break;                             /* NCCS */
    case 0x3F: nc_common(&g, 0, 1); nc_common(&g, 1, 1); nc_common(&g, 2, 1); break; /* NCCT */
    case 0x13: nc_common(&g, 0, 2); break;                             /* NCDS */
    case 0x16: nc_common(&g, 0, 2); nc_common(&g, 1, 2); nc_common(&g, 2, 2); break; /* NCDT */
    case 0x1C: light_lcm(&g); color_mul(&g, 0); break;                 /* CC */
    case 0x14: light_lcm(&g); color_mul(&g, 1); break;                 /* CDP */
    case 0x29: color_mul(&g, 1); break;                                /* DCPL */
    case 0x10: dpc(&g, (uint32_t)c->gd[RGBC]); break;                  /* DPCS */
    case 0x2A: dpc(&g, (uint32_t)c->gd[RGB0]); dpc(&g, (uint32_t)c->gd[RGB0]);
               dpc(&g, (uint32_t)c->gd[RGB0]); break;                  /* DPCT */
    case 0x11: {                                                       /* INTPL */
        int64_t mac[3];
        mac[0] = (int64_t)c->gd[IR1] << 12;
        mac[1] = (int64_t)c->gd[IR2] << 12;
        mac[2] = (int64_t)c->gd[IR3] << 12;
        interp_fc(&g, mac);
        finish_color(&g, mac);
        } break;
    case 0x3D: {                                                       /* GPF */
        int64_t mac[3];
        int i;
        for (i = 0; i < 3; i++) mac[i] = (int64_t)c->gd[IR1 + i] * c->gd[IR0];
        finish_color(&g, mac);
        } break;
    case 0x3E: {                                                       /* GPL */
        int64_t mac[3];
        int i;
        for (i = 0; i < 3; i++) {
            int64_t base = (int64_t)c->gd[MAC1 + i] << (g.sf ? 12 : 0);
            mac[i] = chk44(&g, i + 1, base + (int64_t)c->gd[IR1 + i] * c->gd[IR0]);
        }
        finish_color(&g, mac);
        } break;
    default:
        break;   /* unknown opcodes: no effect besides FLAG reset */
    }
    if (g.flag & 0x7F87E000u)
        g.flag |= 0x80000000u;
    c->gc[FLAG] = (int32_t)g.flag;
}

/* ---- register access ---- */
uint32_t psx_gte_read_data(psx_cpu_t *c, uint32_t r)
{
    switch (r & 31u) {
    case VXY0: case VXY1: case VXY2: case RGBC: case SXY0: case SXY1: case SXY2: case SXYP:
    case RGB0: case RGB1: case RGB2: case RES1: case MAC0: case MAC1: case MAC2: case MAC3:
    case LZCS: case LZCR:
        if ((r & 31u) == SXYP) return (uint32_t)c->gd[SXY2];
        return (uint32_t)c->gd[r & 31u];
    case VZ0: case VZ1: case VZ2: case IR0: case IR1: case IR2: case IR3:
        return (uint32_t)(int32_t)(int16_t)c->gd[r & 31u];
    case OTZ: case SZ0: case SZ1: case SZ2: case SZ3:
        return (uint32_t)c->gd[r & 31u] & 0xFFFFu;
    case IRGB: case ORGB: {
        int32_t v[3];
        uint32_t i, o = 0;
        for (i = 0; i < 3; i++) {
            int32_t t = c->gd[IR1 + i] >> 7;
            if (t < 0) t = 0; else if (t > 0x1F) t = 0x1F;
            v[i] = t;
            o |= (uint32_t)t << (5u * i);
        }
        (void)v;
        return o;
        }
    default: return 0;
    }
}

void psx_gte_write_data(psx_cpu_t *c, uint32_t r, uint32_t v)
{
    r &= 31u;
    switch (r) {
    case VZ0: case VZ1: case VZ2: case IR0: case IR1: case IR2: case IR3:
        c->gd[r] = (int32_t)(int16_t)(uint16_t)v; break;
    case OTZ: case SZ0: case SZ1: case SZ2: case SZ3:
        c->gd[r] = (int32_t)(v & 0xFFFFu); break;
    case SXYP:
        c->gd[SXY0] = c->gd[SXY1]; c->gd[SXY1] = c->gd[SXY2]; c->gd[SXY2] = (int32_t)v; break;
    case IRGB:
        c->gd[IR1] = (int32_t)((v & 0x1Fu) << 7);
        c->gd[IR2] = (int32_t)(((v >> 5) & 0x1Fu) << 7);
        c->gd[IR3] = (int32_t)(((v >> 10) & 0x1Fu) << 7);
        break;
    case ORGB: break;
    case LZCS: {
        uint32_t x = (v & 0x80000000u) ? ~v : v, n = 0, k;
        c->gd[LZCS] = (int32_t)v;
        for (k = 0; k < 32u && !(x & 0x80000000u); k++) { x <<= 1; n++; }
        c->gd[LZCR] = (int32_t)n;
        } break;
    case LZCR: break;
    default: c->gd[r] = (int32_t)v; break;
    }
}

uint32_t psx_gte_read_ctrl(const psx_cpu_t *c, uint32_t r)
{
    r &= 31u;
    switch (r) {
    case RT4: case L4: case LR4: case DQA: case ZSF3: case ZSF4:
        return (uint32_t)(int32_t)lo16(c->gc[r]);
    case H:   /* hardware bug: H reads back sign-extended */
        return (uint32_t)(int32_t)lo16(c->gc[r]);
    default: return (uint32_t)c->gc[r];
    }
}

void psx_gte_write_ctrl(psx_cpu_t *c, uint32_t r, uint32_t v)
{
    r &= 31u;
    switch (r) {
    case RT4: case L4: case LR4: case DQA: case ZSF3: case ZSF4:
        c->gc[r] = (int32_t)(int16_t)(uint16_t)v; break;
    case H:
        c->gc[r] = (int32_t)(v & 0xFFFFu); break;
    case FLAG: {
        uint32_t f = v & 0x7FFFF000u;
        if (f & 0x7F87E000u) f |= 0x80000000u;
        c->gc[r] = (int32_t)f;
        } break;
    default: c->gc[r] = (int32_t)v; break;
    }
}
