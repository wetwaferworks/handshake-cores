/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * mk_n64_demo - builds roms/hs_n64_demo.z64 (pure C11), a REAL big-endian N64 ROM:
 *   * valid 64-byte header (PI config 0x80371240, entry 0x80000400,
 *     CIC-6102 CRC1/CRC2 computed over 0x1000..0x101000)
 *   * original MIPS III VR4300 machine code (assembled below) that
 *       - installs its own exception vector at 0x80000180 (VI interrupt)
 *       - PI-DMAs its asset block from the cartridge into RDRAM
 *       - programs VI / AI / MI / SI (reads the joypad through the PIF)
 *       - every frame rotates two objects with COP1 single-precision maths,
 *         converts the 4x4 matrices to Fast3D s15.16 Mtx with CVT.W.S,
 *         submits an F3DEX-encoded display list (Z-buffered, textured,
 *         Gouraud/lit cube + textured pyramid + fogged CI8 checker ground
 *         + copy-mode texture rectangle) as an RSP gfx OSTask,
 *       - builds an HS-ABI audio command list (ADPCM -> resample -> pan ->
 *         interleave) as an RSP audio OSTask and streams the result via AI DMA
 * Everything here is original: no IPL3, libultra or commercial microcode.
 * usage: mk_n64_demo [OUT.z64]   (default roms/hs_n64_demo.z64) */
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846 /* POSIX value; -std=c11 hides it (REVIEW HS-L9) */
#endif
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fail(const char *m) { fprintf(stderr, "mk_n64_demo: %s\n", m); exit(1); }
static double radians(double x) { return x * (M_PI / 180.0); }
static uint32_t fbits(double v) { float f = (float)v; uint32_t b; memcpy(&b, &f, 4); return b; }
static int64_t floordiv2048(int64_t a) { return a >= 0 ? a / 2048 : -((-a + 2047) / 2048); }

/* ------------------------------------------------------------ assembler */
enum { zero, at, v0, v1, a0, a1, a2, a3, t0, t1, t2, t3, t4, t5, t6, t7,
       s0, s1, s2, s3, s4, s5, s6, s7, t8, t9, k0, k1, gp, sp, fp, ra };
#define MAXW 2048
static uint32_t W[MAXW];
static int NW;
static const uint32_t ABASE = 0x80000400u;
#define MAXL 64
static struct { const char *n; uint32_t a; } LAB[MAXL];
static int NL;
static struct { int idx; char kind; const char *lab; } FIX[MAXL];
static int NF;

static uint32_t pc(void) { return ABASE + 4u * (uint32_t)NW; }
static void label(const char *n) { if (NL >= MAXL) fail("labels"); LAB[NL].n = n; LAB[NL++].a = pc(); }
static uint32_t lab(const char *n) { int i; for (i = NL - 1; i >= 0; i--) if (!strcmp(LAB[i].n, n)) return LAB[i].a; fail(n); return 0; }
static void w(uint32_t v) { if (NW >= MAXW) fail("code"); W[NW++] = v; }
static void fixup(char kind, const char *l) { if (NF >= MAXL) fail("fixups"); FIX[NF].idx = NW; FIX[NF].kind = kind; FIX[NF++].lab = l; }
static void r_(uint32_t fn, uint32_t rs, uint32_t rt, uint32_t rd, uint32_t sa) { w(rs << 21 | rt << 16 | rd << 11 | sa << 6 | fn); }
static void i_(uint32_t op, uint32_t rs, uint32_t rt, int32_t imm) { w(op << 26 | rs << 21 | rt << 16 | ((uint32_t)imm & 0xFFFF)); }
static void br(uint32_t op, uint32_t rs, uint32_t rt, const char *l) { fixup('b', l); w(op << 26 | rs << 21 | rt << 16); }
static void jj(uint32_t op, const char *l) { fixup('j', l); w(op << 26); }
static void nop(void) { w(0); }
static void lui(uint32_t rt, uint32_t v) { i_(0x0F, zero, rt, (int32_t)v); }
static void ori(uint32_t rt, uint32_t rs, uint32_t v) { i_(0x0D, rs, rt, (int32_t)v); }
static void andi(uint32_t rt, uint32_t rs, uint32_t v) { i_(0x0C, rs, rt, (int32_t)v); }
static void addiu(uint32_t rt, uint32_t rs, int32_t v) { i_(0x09, rs, rt, v); }
static void sltu(uint32_t rd, uint32_t rs, uint32_t rt) { r_(0x2B, rs, rt, rd, 0); }
static void addu(uint32_t rd, uint32_t rs, uint32_t rt) { r_(0x21, rs, rt, rd, 0); }
static void or_(uint32_t rd, uint32_t rs, uint32_t rt) { r_(0x25, rs, rt, rd, 0); }
static void sll(uint32_t rd, uint32_t rt, uint32_t sa) { r_(0x00, 0, rt, rd, sa); }
static void srl(uint32_t rd, uint32_t rt, uint32_t sa) { r_(0x02, 0, rt, rd, sa); }
static void dsll32(uint32_t rd, uint32_t rt, uint32_t sa) { r_(0x3C, 0, rt, rd, sa); }
static void dsrl32(uint32_t rd, uint32_t rt, uint32_t sa) { r_(0x3E, 0, rt, rd, sa); }
static void daddu(uint32_t rd, uint32_t rs, uint32_t rt) { r_(0x2D, rs, rt, rd, 0); }
static void multu(uint32_t rs, uint32_t rt) { r_(0x19, rs, rt, 0, 0); }
static void mflo(uint32_t rd) { r_(0x12, 0, 0, rd, 0); }
static void jr(uint32_t rs) { r_(0x08, rs, 0, 0, 0); }
static void lw(uint32_t rt, int32_t off, uint32_t b) { i_(0x23, b, rt, off); }
static void lhu(uint32_t rt, int32_t off, uint32_t b) { i_(0x25, b, rt, off); }
static void sw(uint32_t rt, int32_t off, uint32_t b) { i_(0x2B, b, rt, off); }
static void sh(uint32_t rt, int32_t off, uint32_t b) { i_(0x29, b, rt, off); }
static void sd(uint32_t rt, int32_t off, uint32_t b) { i_(0x3F, b, rt, off); }
static void ld(uint32_t rt, int32_t off, uint32_t b) { i_(0x37, b, rt, off); }
static void beq(uint32_t a, uint32_t b, const char *l) { br(0x04, a, b, l); }
static void bne(uint32_t a, uint32_t b, const char *l) { br(0x05, a, b, l); }
static void beql(uint32_t a, uint32_t b, const char *l) { br(0x14, a, b, l); }
static void bgtz(uint32_t a, const char *l) { br(0x07, a, 0, l); }
static void jmp(const char *l) { jj(0x02, l); }
static void jal(const char *l) { jj(0x03, l); }
static void li(uint32_t rt, uint32_t v) {
    if (v < 0x10000) ori(rt, zero, v);
    else { lui(rt, v >> 16); if (v & 0xFFFF) ori(rt, rt, v & 0xFFFF); }
}
static void la(uint32_t rt, const char *l) { fixup('h', l); lui(rt, 0); fixup('l', l); ori(rt, rt, 0); }
static void mtc0(uint32_t rt, uint32_t rd) { w(0x10u << 26 | 4u << 21 | rt << 16 | rd << 11); }
static void eret(void) { w(0x42000018); }
static void cache(uint32_t op, int32_t off, uint32_t b) { i_(0x2F, b, zero, off); W[NW - 1] |= op << 16; }
static void mtc1(uint32_t rt, uint32_t fs) { w(0x11u << 26 | 4u << 21 | rt << 16 | fs << 11); }
static void mfc1(uint32_t rt, uint32_t fs) { w(0x11u << 26 | 0u << 21 | rt << 16 | fs << 11); }
static void ctc1(uint32_t rt, uint32_t fs) { w(0x11u << 26 | 6u << 21 | rt << 16 | fs << 11); }
static void fop(uint32_t fmt, uint32_t fn, uint32_t fd, uint32_t fs, uint32_t ft) { w(0x11u << 26 | fmt << 21 | ft << 16 | fs << 11 | fd << 6 | fn); }
static void add_s(uint32_t d, uint32_t s, uint32_t t) { fop(16, 0, d, s, t); }
static void sub_s(uint32_t d, uint32_t s, uint32_t t) { fop(16, 1, d, s, t); }
static void mul_s(uint32_t d, uint32_t s, uint32_t t) { fop(16, 2, d, s, t); }
static void mov_s(uint32_t d, uint32_t s) { fop(16, 6, d, s, 0); }
static void neg_s(uint32_t d, uint32_t s) { fop(16, 7, d, s, 0); }
static void cvt_w_s(uint32_t d, uint32_t s) { fop(16, 0x24, d, s, 0); }
static void lwc1(uint32_t ft, int32_t off, uint32_t b) { i_(0x31, b, zero, off); W[NW - 1] |= ft << 16; }
static void swc1(uint32_t ft, int32_t off, uint32_t b) { i_(0x39, b, zero, off); W[NW - 1] |= ft << 16; }
static void lis(uint32_t f, double val) { li(at, fbits(val)); mtc1(at, f); }

/* ------------------------------------------------------------ memory map */
#define ASSET_RD 0x00100000u
#define MTX_CUBE 0x00200000u
#define MTX_PYR 0x00200040u
#define ADPCM_STATE 0x00200100u
#define RS_STATE 0x00200110u
#define AOUT0 0x00210000u
#define FB0 0x00300000u
#define FB1 0x00325800u
#define ZBUF 0x00380000u
#define FRAMECTR_OFF (-0xFC)       /* 0xA01FFF04 */
#define SCRATCH_K1 0xA01FFF80u
#define K1 0xA0000000u

/* ------------------------------------------------------------ assets */
static unsigned char assets[1 << 18];
static size_t nassets;
enum { A_PROJ, A_IDENT, A_VP, A_LIGHT0, A_AMB, A_TEX_CUBE, A_PAL, A_TEX_GROUND, A_TEX_PYR, A_VTX_CUBE, A_VTX_PYR,
       A_VTX_GND, A_PIF, A_BOOK, A_ADPCM, A_ACMD, A_DL, A_N };
static uint32_t addr[A_N];
static void put(int name, const void *data, size_t n, size_t align) {
    while (nassets % align) assets[nassets++] = 0;
    if (nassets + n > sizeof assets) fail("assets");
    addr[name] = ASSET_RD + (uint32_t)nassets;
    memcpy(assets + nassets, data, n);
    nassets += n;
}
typedef struct { unsigned char b[1 << 16]; size_t n; } bb;
static void b8(bb *o, unsigned v) { if (o->n >= sizeof o->b) fail("bytes"); o->b[o->n++] = (unsigned char)v; }
static void b16(bb *o, uint32_t v) { b8(o, (v >> 8) & 0xFF); b8(o, v & 0xFF); }
static void b32(bb *o, uint32_t v) { b16(o, v >> 16); b16(o, v & 0xFFFF); }

static void matrix_bytes(const double m[4][4], bb *o) {   /* 4x4 floats (row-major, row-vector convention) -> Fast3D Mtx */
    uint32_t fx[16];
    int r, c;
    for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) fx[r * 4 + c] = (uint32_t)(int64_t)rint(m[r][c] * 65536.0);
    for (r = 0; r < 16; r++) b16(o, (fx[r] >> 16) & 0xFFFF);
    for (r = 0; r < 16; r++) b16(o, fx[r] & 0xFFFF);
}
static void mmul(const double a[4][4], const double b[4][4], double o[4][4]) {
    int i, j, k;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) { double s = 0; for (k = 0; k < 4; k++) s += a[i][k] * b[k][j]; o[i][j] = s; }
}
static uint32_t rgba5551(int r, int g, int b, int a) { return (uint32_t)((r >> 3) << 11 | (g >> 3) << 6 | (b >> 3) << 1 | a); }
static void vtx(bb *o, int x, int y, int z, int s, int t, int r, int g, int b, int a) {
    b16(o, (uint32_t)x & 0xFFFF); b16(o, (uint32_t)y & 0xFFFF); b16(o, (uint32_t)z & 0xFFFF); b16(o, 0);
    b16(o, (uint32_t)s & 0xFFFF); b16(o, (uint32_t)t & 0xFFFF); b8(o, (unsigned)r & 0xFF); b8(o, (unsigned)g & 0xFF); b8(o, (unsigned)b & 0xFF); b8(o, (unsigned)a);
}

/* ---------------- display list (F3DEX encodings) */
static uint32_t DL[1024][2];
static int NDL;
static void g(uint64_t w0, uint64_t w1) { if (NDL >= 1024) fail("dl"); DL[NDL][0] = (uint32_t)(w0 & 0xFFFFFFFF); DL[NDL][1] = (uint32_t)(w1 & 0xFFFFFFFF); NDL++; }
static void cc(uint64_t a0_, uint64_t b0, uint64_t c0, uint64_t d0, uint64_t Aa0, uint64_t Ab0, uint64_t Ac0, uint64_t Ad0,
               uint64_t a1_, uint64_t b1, uint64_t c1, uint64_t d1, uint64_t Aa1, uint64_t Ab1, uint64_t Ac1, uint64_t Ad1) {
    g(0xFCull << 24 | (a0_ & 15) << 20 | (c0 & 31) << 15 | (Aa0 & 7) << 12 | (Ac0 & 7) << 9 | (a1_ & 15) << 5 | (c1 & 31),
      (b0 & 15) << 28 | (b1 & 15) << 24 | (Aa1 & 7) << 21 | (Ac1 & 7) << 18 | (d0 & 7) << 15 | (Ab0 & 7) << 12 | (Ad0 & 7) << 9 | (d1 & 7) << 6 | (Ab1 & 7) << 3 | (Ad1 & 7));
}
static void om_h(uint64_t v) { g(0xBAull << 24 | 0 << 8 | 24, v); }
static void om_l(uint64_t v) { g(0xB9ull << 24 | 0 << 8 | 32, v); }
static void settimg(uint64_t fmt, uint64_t siz, uint64_t wd, uint64_t a) { g(0xFDull << 24 | fmt << 21 | siz << 19 | (wd - 1), a); }
static void settile(uint64_t fmt, uint64_t siz, uint64_t line, uint64_t tmem, uint64_t tile, uint64_t pal, uint64_t cmt, uint64_t maskt,
                    uint64_t shiftt, uint64_t cms, uint64_t masks, uint64_t shifts) {
    g(0xF5ull << 24 | fmt << 21 | siz << 19 | line << 9 | tmem, tile << 24 | pal << 20 | cmt << 18 | maskt << 14 | shiftt << 10 | cms << 8 | masks << 4 | shifts);
}
static void tilesize(uint64_t tile, uint64_t wd, uint64_t h) { g(0xF2ull << 24, tile << 24 | ((wd - 1) << 2) << 12 | ((h - 1) << 2)); }
static void vtxcmd(uint64_t a, uint64_t n, uint64_t vv0) { g(0x04ull << 24 | (vv0 * 2) << 16 | n << 10 | (n * 16 - 1), a); }
static void tri2(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f_) { g(0xB1ull << 24 | a * 2 << 16 | b * 2 << 8 | c * 2, d * 2 << 16 | e * 2 << 8 | f_ * 2); }
static void fillrect(uint64_t x0, uint64_t y0, uint64_t x1, uint64_t y1) { g(0xF6ull << 24 | (x1 << 2) << 12 | (y1 << 2), (x0 << 2) << 12 | (y0 << 2)); }
#define G_ZBUFFER 1ull
#define G_SHADE 4ull
#define G_SMOOTH 0x200ull
#define G_CULL_BACK 0x2000ull
#define G_FOG 0x10000ull
#define G_LIGHTING 0x20000ull
#define CYC1 (0ull << 20)
#define CYC2 (1ull << 20)
#define COPY (2ull << 20)
#define FILL (3ull << 20)
#define PERSP (1ull << 19)
#define BILERP (2ull << 12)
#define TLUT_RGBA (2ull << 14)
#define ZCU 0x30ull
#define BL_OPA ((0ull << 30) | (0ull << 26) | (1ull << 22) | (1ull << 18) | (0ull << 28) | (0ull << 24) | (1ull << 20) | (1ull << 16))
#define BL_FOG2 ((3ull << 30) | (2ull << 26) | (0ull << 22) | (0ull << 18) | (0ull << 28) | (0ull << 24) | (1ull << 20) | (1ull << 16))

static const double near_ = 40.0, far_ = 5000.0, fovy = 60.0, aspect = 4.0 / 3.0;
static double zndc(double d) { return ((near_ + far_) / (near_ - far_) * -d + 2 * near_ * far_ / (near_ - far_)) / d; }

static const int BOOK0 = 3481, BOOK1 = -1597;   /* int(1.70 * 2048), int(-0.78 * 2048) */
static int dec(int p1, int p2, int64_t resid) {
    int64_t acc = (int64_t)BOOK0 * p1 + (int64_t)BOOK1 * p2 + resid * 2048;
    acc = floordiv2048(acc);
    return (int)(acc < -32768 ? -32768 : acc > 32767 ? 32767 : acc);
}

static uint32_t rol(uint32_t v, uint32_t s) { s &= 31; return s ? ((v << s) | (v >> (32 - s))) : v; }

int main(int argc, char **argv) {
    const char *out = argc > 1 ? argv[1] : "roms/hs_n64_demo.z64";
    static bb o;
    double f, cp, sp_, tmpm[4][4], proj[4][4];
    int x, y, k, i, n;
    uint32_t sky, ASSET_ROM;
    int fm, fo, cnt_idx, asset_cart_idx, asset_len_idx, h0, hwords, ADPCM_LEN, acmd_len;
    unsigned char *rom;
    size_t romsize;
    FILE *fp_;

    if (BOOK0 != (int)(1.70 * 2048) || BOOK1 != (int)(-0.78 * 2048)) fail("book");
    /* projection = view(camera at y=+170, pitched down 12 deg) * perspective (row vectors) */
    f = 1.0 / tan(radians(fovy) / 2);
    {
        const double persp[4][4] = { { f / aspect, 0, 0, 0 }, { 0, f, 0, 0 }, { 0, 0, (near_ + far_) / (near_ - far_), -1 }, { 0, 0, 2 * near_ * far_ / (near_ - far_), 0 } };
        const double pitch = radians(12.0);
        const double trans[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, -170, 0, 1 } };
        double rotx[4][4] = { { 1, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 1 } };
        const double ident[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        cp = cos(pitch); sp_ = sin(pitch);
        rotx[1][1] = cp; rotx[1][2] = sp_; rotx[2][1] = -sp_; rotx[2][2] = cp;
        mmul(trans, rotx, tmpm); mmul(tmpm, persp, proj);
        o.n = 0; matrix_bytes(proj, &o); put(A_PROJ, o.b, o.n, 16);
        o.n = 0; matrix_bytes(ident, &o); put(A_IDENT, o.b, o.n, 16);
    }
    o.n = 0; b16(&o, 640); b16(&o, 480); b16(&o, 511); b16(&o, 0); b16(&o, 640); b16(&o, 480); b16(&o, 511); b16(&o, 0); put(A_VP, o.b, o.n, 16);
    {
        double ldf[3] = { 0.45, 0.75, 0.50 }, ln, ss = 0;
        int ldi[3];
        for (k = 0; k < 3; k++) ss += ldf[k] * ldf[k];
        ln = sqrt(ss);
        for (k = 0; k < 3; k++) ldi[k] = (int)rint(ldf[k] / ln * 120);
        o.n = 0;
        b8(&o, 235); b8(&o, 225); b8(&o, 205); b8(&o, 0); b8(&o, 235); b8(&o, 225); b8(&o, 205); b8(&o, 0);
        for (k = 0; k < 3; k++) b8(&o, (unsigned)ldi[k] & 0xFF);
        for (k = 0; k < 5; k++) b8(&o, 0);
        put(A_LIGHT0, o.b, o.n, 16);
    }
    o.n = 0; b8(&o, 55); b8(&o, 55); b8(&o, 70); b8(&o, 0); b8(&o, 55); b8(&o, 55); b8(&o, 70); b8(&o, 0);
    for (k = 0; k < 8; k++) b8(&o, 0);
    put(A_AMB, o.b, o.n, 16);

    /* cube texture RGBA16 32x32: four coloured quadrants, white grid, diagonal band */
    o.n = 0;
    for (y = 0; y < 32; y++)
        for (x = 0; x < 32; x++) {
            static const int basec[4][3] = { { 220, 60, 50 }, { 60, 170, 230 }, { 250, 200, 40 }, { 90, 200, 90 } };
            int q = (x >= 16) + 2 * (y >= 16), c[3];
            if (x % 8 == 0 || y % 8 == 0) { c[0] = c[1] = c[2] = 250; }
            else if (abs(x - y) <= 1 || abs(x + y - 31) <= 1) { c[0] = 30; c[1] = 30; c[2] = 40; }
            else {
                double kk = 0.75 + 0.25 * ((x % 8) + (y % 8)) / 14.0;
                for (k = 0; k < 3; k++) { int v = (int)(basec[q][k] * kk); c[k] = v < 255 ? v : 255; }
            }
            b16(&o, rgba5551(c[0], c[1], c[2], 1));
        }
    put(A_TEX_CUBE, o.b, o.n, 8);
    /* ground CI8 16x16 checker + palette (RGBA16 TLUT, 16 entries) */
    o.n = 0;
    b16(&o, rgba5551(40, 120, 50, 1)); b16(&o, rgba5551(150, 205, 110, 1)); b16(&o, rgba5551(55, 140, 60, 1)); b16(&o, rgba5551(170, 220, 125, 1));
    for (k = 0; k < 12; k++) b16(&o, rgba5551(255, 0, 255, 1));
    put(A_PAL, o.b, o.n, 8);
    o.n = 0;
    for (y = 0; y < 16; y++)
        for (x = 0; x < 16; x++) {
            int chk = ((x / 8) + (y / 8)) & 1;
            b8(&o, (unsigned)(chk + ((x % 8 == 0 || x % 8 == 7 || y % 8 == 0 || y % 8 == 7) ? 2 : 0)));
        }
    put(A_TEX_GROUND, o.b, o.n, 8);
    /* pyramid I8 32x32: concentric rings */
    o.n = 0;
    for (y = 0; y < 32; y++)
        for (x = 0; x < 32; x++) {
            double d = hypot(x - 15.5, y - 31.0);
            b8(&o, (unsigned)((int)(150 + 100 * cos(d * 0.9)) & 0xFF));
        }
    put(A_TEX_PYR, o.b, o.n, 8);

    /* cube: 6 faces, (normal, u, v) with u x v = normal */
    o.n = 0;
    {
        static const int fuv[6][3][3] = { { { 0, 0, 1 }, { 1, 0, 0 }, { 0, 1, 0 } }, { { 0, 0, -1 }, { -1, 0, 0 }, { 0, 1, 0 } },
                                          { { 1, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 } }, { { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } },
                                          { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } }, { { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } } };
        static const int corner[4][4] = { { -1, -1, 0, 32 }, { 1, -1, 32, 32 }, { 1, 1, 32, 0 }, { -1, 1, 0, 0 } };
        const int S = 80;
        int fc, cn;
        for (fc = 0; fc < 6; fc++)
            for (cn = 0; cn < 4; cn++) {
                int p[3];
                for (k = 0; k < 3; k++) p[k] = fuv[fc][0][k] * S + (fuv[fc][1][k] * (S * corner[cn][0]) + fuv[fc][2][k] * (S * corner[cn][1]));
                vtx(&o, p[0], p[1], p[2], corner[cn][2] << 5, corner[cn][3] << 5, fuv[fc][0][0] * 127, fuv[fc][0][1] * 127, fuv[fc][0][2] * 127, 255);
            }
    }
    put(A_VTX_CUBE, o.b, o.n, 16);
    /* pyramid: 4 sides + base (16 verts), Gouraud vertex colours */
    o.n = 0;
    {
        const int H = 90, B = 85;
        const int base[4][3] = { { -B, -H, B }, { B, -H, B }, { B, -H, -B }, { -B, -H, -B } };
        static const int cols[4][3] = { { 255, 70, 60 }, { 70, 255, 90 }, { 60, 110, 255 }, { 255, 230, 60 } };
        static const int order[4] = { 3, 2, 1, 0 };
        for (k = 0; k < 4; k++) {
            const int *pa = base[k], *pb = base[(k + 1) % 4];
            vtx(&o, pa[0], pa[1], pa[2], 0, 32 << 5, cols[k][0], cols[k][1], cols[k][2], 255);
            vtx(&o, pb[0], pb[1], pb[2], 32 << 5, 32 << 5, cols[(k + 1) % 4][0], cols[(k + 1) % 4][1], cols[(k + 1) % 4][2], 255);
            vtx(&o, 0, H, 0, 16 << 5, 0, 255, 255, 255, 255);
        }
        for (i = 0; i < 4; i++) {
            k = order[i];
            vtx(&o, base[k][0], base[k][1], base[k][2], (k & 1) * 1024, (k >> 1) * 1024, 120, 120, 120, 255);
        }
    }
    put(A_VTX_PYR, o.b, o.n, 16);
    /* ground: 9x9 grid y=-150, x in [-2400,2400], z in [+500,-4300] */
    o.n = 0;
    {
        int zi, xi;
        for (zi = 0; zi < 9; zi++)
            for (xi = 0; xi < 9; xi++) {
                int z = 500 - zi * 600, xx = -2400 + xi * 600;
                int shade = 255 - ((xi * 7 + zi * 13) % 5) * 10;
                int su = xi * 4 * 16 * 32, tv = zi * 4 * 16 * 32;
                su = su < 32768 ? (su & 0xFFFF) : su - 65536;
                tv = tv - (tv >= 32768 ? 65536 : 0);
                vtx(&o, xx, -150, z, su, tv, shade, shade, shade, 255);
            }
    }
    put(A_VTX_GND, o.b, o.n, 16);
    /* PIF joybus block: channel 0 read buttons */
    {
        unsigned char pif[64] = { 0xFF, 0x01, 0x04, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
        pif[63] = 0x01;
        put(A_PIF, pif, 64, 64);
    }

    /* ---------------- audio: original arpeggio, order-2 predictive 4-bit ADPCM */
    {
        enum { NSAMP = 48000 };
        const int AI_FREQ = 48681812 / (1013 + 1);
        const double SRC_RATE = AI_FREQ / 2.0;
        static const double notes[12] = { 261.63, 329.63, 392.00, 523.25, 392.00, 329.63, 293.66, 349.23, 440.00, 587.33, 440.00, 349.23 };
        static int pcm[NSAMP];
        int p1 = 0, p2 = 0, fr;
        for (n = 0; n < NSAMP; n++) {
            double t = n / SRC_RATE;
            double note = notes[(int64_t)(t * 6) % 12];
            double ph = fmod(t * 6, 1.0), env, v, ev;
            if (ph < 0) ph += 1.0;
            ev = ph * 40;
            env = (ev < 1.0 ? ev : 1.0) * exp(-ph * 3.0);
            v = 0.55 * sin(2 * M_PI * note * t) + 0.25 * sin(4 * M_PI * note * t) + 0.12 * sin(2 * M_PI * note * 0.5 * t);
            v = v * env * 15000;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            pcm[n] = (int)v;
        }
        o.n = 0;
        for (fr = 0; fr < NSAMP / 16; fr++) {
            const int *blk = pcm + fr * 16;
            int have = 0, best_scale = 0, best_nibs[16], best_q1 = 0, best_q2 = 0, scale;
            int64_t best_err = 0;
            for (scale = 0; scale < 13; scale++) {
                int q1 = p1, q2 = p2, nibs[16], s;
                int64_t err = 0;
                for (s = 0; s < 16; s++) {
                    int pred = dec(q1, q2, 0), nb, ov;
                    double rr = rint((double)(blk[s] - pred) / (double)(1 << scale));
                    nb = rr > 7 ? 7 : rr < -8 ? -8 : (int)rr;
                    ov = dec(q1, q2, (int64_t)nb * (1 << scale));
                    err += (int64_t)(ov - blk[s]) * (ov - blk[s]);
                    nibs[s] = nb; q2 = q1; q1 = ov;
                }
                if (!have || err < best_err) { have = 1; best_err = err; best_scale = scale; memcpy(best_nibs, nibs, sizeof nibs); best_q1 = q1; best_q2 = q2; }
            }
            p1 = best_q1; p2 = best_q2;
            b8(&o, (unsigned)(best_scale << 4));
            for (k = 0; k < 16; k += 2) b8(&o, (unsigned)(((best_nibs[k] & 15) << 4) | (best_nibs[k + 1] & 15)));
        }
        {
            bb *bk = (bb *)calloc(1, sizeof(bb));
            if (!bk) fail("oom");
            b16(bk, (uint32_t)BOOK0 & 0xFFFF); b16(bk, (uint32_t)BOOK1 & 0xFFFF);
            for (k = 0; k < 4; k++) b8(bk, 0);
            put(A_BOOK, bk->b, bk->n, 16);
            free(bk);
        }
        put(A_ADPCM, o.b, o.n, 16);
        ADPCM_LEN = (int)o.n;
        if (ADPCM_LEN % 225) fail("adpcm length");
    }
    o.n = 0;
    b32(&o, 0x0Bu << 24); b32(&o, addr[A_BOOK]);
    {
        int ch;
        const uint32_t cmds[10][2] = { { 0x04u << 24 | 0x000u << 12 | 45, 0 }, { 0x08u << 24 | 0x000u << 12 | 0x040, 80 }, { 0x01u << 24, ADPCM_STATE },
                                       { 0x08u << 24 | 0x040u << 12 | 0x100, 160 }, { 0x05u << 24 | 0x2000, RS_STATE }, { 0x09u << 24, 0 },
                                       { 0x08u << 24 | 0x100u << 12 | 0x000, 160 }, { 0x03u << 24, 0x240u << 16 | 0x380 },
                                       { 0x0Du << 24 | 0x4C0, 0x240u << 16 | 0x380 }, { 0x06u << 24 | 0x4C0u << 12 | 640, 0 } };
        for (ch = 0; ch < 5; ch++) for (k = 0; k < 10; k++) { b32(&o, cmds[k][0]); b32(&o, cmds[k][1]); }
    }
    put(A_ACMD, o.b, o.n, 16);
    acmd_len = (int)o.n;

    sky = rgba5551(120, 170, 235, 1);
    g(0xE7ull << 24, 0);
    g(0xFFull << 24 | 0 << 21 | 2 << 19 | 319, ZBUF);                                  /* colour image = Z buffer (clear) */
    om_h(FILL); om_l(0);
    g(0xF7ull << 24, 0xFFFCFFFC);
    fillrect(0, 0, 319, 239);
    g(0xE7ull << 24, 0);
    g(0xFFull << 24 | 0 << 21 | 2 << 19 | 319, FB0);
    g(0xFEull << 24, ZBUF);
    g(0xF7ull << 24, (uint64_t)sky << 16 | sky);
    fillrect(0, 0, 319, 239);
    g(0xE7ull << 24, 0);
    g(0xEDull << 24 | 0, (320ull << 2) << 12 | (240 << 2));
    g(0x03ull << 24 | 0x80 << 16 | 16, addr[A_VP]);
    g(0x01ull << 24 | 3 << 16 | 64, addr[A_PROJ]);
    g(0xF8ull << 24, 120ull << 24 | 170 << 16 | 235 << 8 | 255);                     /* fog colour = sky */
    /* fog: depth units 0..1000 = zndc*500+500; fog from 4000..5200 world-ish */
    {
        double fmin = zndc(1100) * 500 + 500, fmax = zndc(4400) * 500 + 500;
        fm = (int)(128000 / (fmax - fmin)); fo = (int)((500 - fmin) * 256 / (fmax - fmin));
        g(0xBCull << 24 | 0 << 8 | 0x08, ((uint64_t)(uint32_t)fm & 0xFFFF) << 16 | ((uint32_t)fo & 0xFFFF));
    }
    /* ---- ground: 2-cycle, CI8 + TLUT, fog blender */
    g(0xB6ull << 24, 0xFFFFFFFF);
    g(0xB7ull << 24, G_ZBUFFER | G_SHADE | G_SMOOTH | G_FOG | G_CULL_BACK);
    om_h(CYC2 | PERSP | BILERP | TLUT_RGBA); om_l(ZCU | BL_FOG2);
    cc(1, 15, 4, 7, 7, 7, 7, 4, 15, 15, 31, 0, 7, 7, 7, 0);
    g(0xBBull << 24 | 0 << 8 | 1, 0xFFFFull << 16 | 0xFFFF);
    settimg(0, 2, 1, addr[A_PAL]); settile(0, 2, 0, 256, 7, 0, 0, 0, 0, 0, 0, 0); g(0xF0ull << 24, 7ull << 24 | (15 << 2) << 12);
    g(0xE7ull << 24, 0);
    settimg(2, 1, 16, addr[A_TEX_GROUND]); settile(2, 1, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0); g(0xF3ull << 24, 7ull << 24 | 255 << 12 | 0x800);
    settile(2, 1, 2, 0, 0, 0, 0, 4, 0, 0, 4, 0); tilesize(0, 16, 16);
    g(0x01ull << 24 | 2 << 16 | 64, addr[A_IDENT]);
    {
        int zi, xi;
        for (zi = 0; zi < 8; zi++) {
            vtxcmd(addr[A_VTX_GND] + (uint32_t)zi * 9 * 16, 18, 0);
            for (xi = 0; xi < 8; xi++) {
                int aa = xi, bq = xi + 1, cq = 9 + xi + 1, dq = 9 + xi;
                tri2((uint64_t)aa, (uint64_t)bq, (uint64_t)cq, (uint64_t)aa, (uint64_t)cq, (uint64_t)dq);
            }
        }
    }
    /* ---- cube: 1-cycle, RGBA16, lit */
    g(0xE7ull << 24, 0);
    g(0xB6ull << 24, 0xFFFFFFFF);
    g(0xB7ull << 24, G_ZBUFFER | G_SHADE | G_SMOOTH | G_LIGHTING | G_CULL_BACK);
    om_h(CYC1 | PERSP | BILERP); om_l(ZCU | BL_OPA);
    cc(1, 15, 4, 7, 7, 7, 7, 4, 1, 15, 4, 7, 7, 7, 7, 4);
    g(0xBCull << 24 | 0 << 8 | 0x02, 24);                                             /* 1 directional light (F3DEX encoding) */
    g(0x03ull << 24 | 0x86 << 16 | 16, addr[A_LIGHT0]);
    g(0x03ull << 24 | 0x88 << 16 | 16, addr[A_AMB]);
    settimg(0, 2, 32, addr[A_TEX_CUBE]); settile(0, 2, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0); g(0xF3ull << 24, 7ull << 24 | 1023 << 12 | 0x100);
    settile(0, 2, 8, 0, 0, 0, 0, 5, 0, 0, 5, 0); tilesize(0, 32, 32);
    g(0x01ull << 24 | 2 << 16 | 64, MTX_CUBE);
    vtxcmd(addr[A_VTX_CUBE], 24, 0);
    for (k = 0; k < 6; k++) { uint64_t oo = (uint64_t)k * 4; tri2(oo, oo + 1, oo + 2, oo, oo + 2, oo + 3); }
    /* ---- HUD: copy-mode texture rectangle of the cube texture (top-left) */
    g(0xE7ull << 24, 0);
    om_h(COPY); om_l(0);
    g(0xE4ull << 24 | ((8 + 31) << 2) << 12 | ((8 + 31) << 2), 0 << 24 | (8 << 2) << 12 | (8 << 2));
    g(0xB4ull << 24, 0);
    g(0xB3ull << 24, (4ull << 10) << 16 | (1 << 10));
    /* ---- pyramid: 1-cycle, I8, (tex - env) * shade + env */
    g(0xE7ull << 24, 0);
    g(0xB6ull << 24, 0xFFFFFFFF);
    g(0xB7ull << 24, G_ZBUFFER | G_SHADE | G_SMOOTH | G_CULL_BACK);
    om_h(CYC1 | PERSP | BILERP); om_l(ZCU | BL_OPA);
    g(0xFBull << 24, 40ull << 24 | 30 << 16 | 60 << 8 | 255);
    cc(1, 5, 4, 5, 7, 7, 7, 4, 1, 5, 4, 5, 7, 7, 7, 4);
    settimg(4, 1, 32, addr[A_TEX_PYR]); settile(4, 1, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0); g(0xF3ull << 24, 7ull << 24 | 1023 << 12 | 0x200);
    settile(4, 1, 4, 0, 0, 0, 0, 5, 0, 0, 5, 0); tilesize(0, 32, 32);
    g(0x01ull << 24 | 2 << 16 | 64, MTX_PYR);
    vtxcmd(addr[A_VTX_PYR], 16, 0);
    tri2(0, 1, 2, 3, 4, 5); tri2(6, 7, 8, 9, 10, 11); tri2(12, 13, 14, 12, 14, 15);
    g(0xE9ull << 24, 0);
    g(0xB8ull << 24, 0);
    o.n = 0;
    for (k = 0; k < NDL; k++) { b32(&o, DL[k][0]); b32(&o, DL[k][1]); }
    put(A_DL, o.b, o.n, 8);

    /* ------------------------------------------------------------ program */
    {
        double c1 = cos(radians(1.6)), sn1 = sin(radians(1.6));
        double c2 = cos(radians(0.9)), sn2 = sin(radians(0.9));
        double b0 = radians(25);
        static const int rot[2][4] = { { 20, 21, 22, 23 }, { 24, 25, 26, 27 } };
        label("entry");
        li(sp, 0x803FFF00);
        ctc1(zero, 31);
        la(t0, "handler"); li(t1, 0xA0000180); li(t2, 0);                               /* count patched below */
        cnt_idx = NW - 1;
        label("copy_h");
        lw(t3, 0, t0); sw(t3, 0, t1); addiu(t0, t0, 4); addiu(t1, t1, 4);
        addiu(t2, t2, -1); bgtz(t2, "copy_h"); nop();
        lui(t0, 0x8000); cache(0x10, 0x180, t0);
        /* PI DMA asset block -> RDRAM 0x00100000 */
        lui(t0, 0xA460);
        li(t1, ASSET_RD); sw(t1, 0, t0);
        li(t1, 0); asset_cart_idx = NW; nop(); nop();                                    /* patched: li t1, cart addr */
        sw(t1, 4, t0);
        li(t1, 0); asset_len_idx = NW; nop(); nop();
        sw(t1, 0xC, t0);
        label("pi_wait");
        lw(t1, 0x10, t0); andi(t1, t1, 3); bne(t1, zero, "pi_wait"); nop();
        li(t1, 2); sw(t1, 0x10, t0);
        /* 64-bit sanity: write a doubleword pattern built with DSLL32/DADDU and read it back */
        li(t1, 0x48534E36); dsll32(t1, t1, 0); li(t2, 0x34210001); daddu(t1, t1, t2);
        lui(t0, 0xA020); sd(t1, -0xF8, t0); ld(t2, -0xF8, t0); dsrl32(t2, t2, 0); sw(t2, -0xF0, t0);
        sw(zero, -0x100, t0); sw(zero, FRAMECTR_OFF, t0);
        /* VI */
        lui(t0, 0xA440);
        li(t1, 0x320E); sw(t1, 0, t0);
        li(t1, FB0); sw(t1, 4, t0);
        li(t1, 320); sw(t1, 8, t0);
        li(t1, 0x200); sw(t1, 0xC, t0);
        li(t1, 0x200); sw(t1, 0x30, t0);
        li(t1, 0x400); sw(t1, 0x34, t0);
        /* AI */
        lui(t0, 0xA450);
        li(t1, 1013); sw(t1, 0x10, t0);
        li(t1, 15); sw(t1, 0x14, t0);
        li(t1, 1); sw(t1, 8, t0);
        /* FPU state */
        lis(20, 1.0); lis(21, 0.0); lis(22, c1); lis(23, sn1);
        lis(24, cos(b0)); lis(25, sin(b0)); lis(26, c2); lis(27, sn2);
        lis(28, 0.5); lis(29, 1.5); lis(30, 65536.0); lis(31, 1.0); lis(1, 0.0);
        /* MI mask (VI) + Status: CU1|CU0|FR|IE|IM2 */
        lui(t0, 0xA430); li(t1, 0x80); sw(t1, 0xC, t0);
        li(t1, 0x34000401); mtc0(t1, 12);
        addu(s0, zero, zero); addu(s1, zero, zero); addu(s2, zero, zero);
        label("main");
        lui(t0, 0xA020);
        label("wait");
        lw(t1, -0x100, t0); beq(t1, s0, "wait"); nop();
        addu(s0, t1, zero);
        /* SI joypad read through the PIF */
        lui(t0, 0xA480);
        li(t1, addr[A_PIF]); sw(t1, 0, t0);
        li(t2, 0x1FC007C0); sw(t2, 0x10, t0);
        sw(t1, 0, t0); sw(t2, 4, t0); sw(zero, 0x18, t0);
        li(t3, addr[A_PIF] | K1); lhu(s7, 4, t3);
        andi(t4, s7, 0x1000); bne(t4, zero, "skip_rot"); nop();
        for (k = 0; k < 2; k++) {
            uint32_t C = (uint32_t)rot[k][0], S_ = (uint32_t)rot[k][1], CD = (uint32_t)rot[k][2], SD = (uint32_t)rot[k][3];
            mul_s(0, C, CD); mul_s(2, S_, SD); sub_s(4, 0, 2);
            mul_s(0, S_, CD); mul_s(2, C, SD); add_s(S_, 0, 2); mov_s(C, 4);
            mul_s(0, C, C); mul_s(2, S_, S_); add_s(0, 0, 2); mul_s(0, 0, 28); sub_s(0, 29, 0);
            mul_s(C, C, 0); mul_s(S_, S_, 0);
        }
        label("skip_rot");
        li(a0, MTX_CUBE | K1);
        mov_s(12, 20); mov_s(13, 21); mov_s(14, 24); mov_s(15, 25);
        lis(16, -125.0); lis(0, 35.0); mul_s(0, 0, 25); lis(2, 5.0); add_s(17, 2, 0);
        lis(18, -720.0); lis(19, 1.0);
        jal("build_mtx"); nop();
        li(a0, MTX_PYR | K1);
        mov_s(12, 20); neg_s(13, 21); mov_s(14, 31); mov_s(15, 1);
        lis(16, 185.0); lis(17, -60.0); lis(18, -560.0); lis(19, 1.0);
        jal("build_mtx"); nop();
        /* gfx OSTask */
        lui(t0, 0xA400);
        li(t1, 1); sw(t1, 0xFC0, t0); sw(zero, 0xFC4, t0);
        li(t1, addr[A_DL]); sw(t1, 0xFF0, t0);
        li(t1, (uint32_t)NDL * 8); sw(t1, 0xFF4, t0);
        lui(t0, 0xA404); li(t1, 5); sw(t1, 0x10, t0);
        label("sp_wait1");
        lw(t1, 0x10, t0); andi(t1, t1, 1); beq(t1, zero, "sp_wait1"); nop();
        /* audio list patch: volumes from sin(a), output buffer by frame parity */
        lis(0, 12000.0); mul_s(0, 0, 21); lis(2, 16384.0); add_s(4, 2, 0); sub_s(6, 2, 0);
        cvt_w_s(4, 4); cvt_w_s(6, 6); mfc1(t6, 4); mfc1(t7, 6);
        sll(t6, t6, 16); andi(t7, t7, 0xFFFF); or_(t6, t6, t7);
        andi(t8, s1, 1); li(t9, 3200); multu(t8, t9); mflo(t9);
        li(a1, AOUT0); addu(a1, a1, t9); addu(a2, a1, zero);
        li(t3, addr[A_ADPCM]); addu(t3, t3, s2);
        li(t2, addr[A_ACMD] | K1); addiu(t2, t2, 8);
        li(t4, 5);
        label("chunk");
        sw(t3, 4, t2); sw(t6, 44, t2); sw(a1, 76, t2);
        addiu(t3, t3, 45); addiu(a1, a1, 640); addiu(t2, t2, 80);
        addiu(t4, t4, -1); bgtz(t4, "chunk"); nop();
        addiu(s2, s2, 225);
        li(t1, (uint32_t)ADPCM_LEN); sltu(t4, s2, t1);
        bne(t4, zero, "src_ok"); nop(); addu(s2, zero, zero);
        label("src_ok");
        lui(t0, 0xA400);
        li(t1, 2); sw(t1, 0xFC0, t0);
        li(t1, addr[A_ACMD]); sw(t1, 0xFF0, t0);
        li(t1, (uint32_t)acmd_len); sw(t1, 0xFF4, t0);
        lui(t0, 0xA404); li(t1, 5); sw(t1, 0x10, t0);
        label("sp_wait2");
        lw(t1, 0x10, t0); andi(t1, t1, 1); beq(t1, zero, "sp_wait2"); nop();
        lui(t0, 0xA450); sw(a2, 0, t0); li(t1, 3200); sw(t1, 4, t0);
        /* VI origin double buffer (branch-likely selects FB1 on odd frames) */
        li(t1, FB0); andi(t2, s1, 1);
        beql(t2, zero, "vi_set"); nop();
        li(t1, FB1);
        label("vi_set");
        lui(t0, 0xA440); sw(t1, 4, t0);
        addiu(s1, s1, 1);
        lui(t0, 0xA020); sw(s1, FRAMECTR_OFF, t0);
        jmp("main"); nop();
        /* ---- build_mtx(a0 = KSEG1 Mtx dst; f12 cA f13 sA f14 cB f15 sB f16..18 translation f19 scale) */
        label("build_mtx");
        li(t0, SCRATCH_K1);
        mul_s(0, 12, 19); swc1(0, 0, t0);
        mul_s(0, 13, 15); mul_s(0, 0, 19); swc1(0, 4, t0);
        mul_s(0, 13, 14); mul_s(0, 0, 19); neg_s(0, 0); swc1(0, 8, t0);
        swc1(1, 12, t0); swc1(1, 16, t0);
        mul_s(0, 14, 19); swc1(0, 20, t0);
        mul_s(0, 15, 19); swc1(0, 24, t0);
        swc1(1, 28, t0);
        mul_s(0, 13, 19); swc1(0, 32, t0);
        mul_s(0, 12, 15); mul_s(0, 0, 19); neg_s(0, 0); swc1(0, 36, t0);
        mul_s(0, 12, 14); mul_s(0, 0, 19); swc1(0, 40, t0);
        swc1(1, 44, t0);
        swc1(16, 48, t0); swc1(17, 52, t0); swc1(18, 56, t0); swc1(31, 60, t0);
        li(t2, 16); addu(t3, a0, zero);
        label("cv");
        lwc1(0, 0, t0); mul_s(0, 0, 30); cvt_w_s(0, 0); mfc1(t1, 0);
        srl(t4, t1, 16); sh(t4, 0, t3); sh(t1, 32, t3);
        addiu(t0, t0, 4); addiu(t3, t3, 2); addiu(t2, t2, -1);
        bgtz(t2, "cv"); nop();
        jr(ra); nop();
        /* ---- exception handler (copied to 0x80000180): VI interrupt -> vicount++ */
        label("handler");
        h0 = NW;
        lui(k0, 0xA430); lw(k1, 8, k0); andi(k1, k1, 0x08); beq(k1, zero, "h_done"); nop();
        lui(k0, 0xA440); sw(zero, 0x10, k0);
        lui(k0, 0xA020); lw(k1, -0x100, k0); addiu(k1, k1, 1); sw(k1, -0x100, k0);
        label("h_done");
        eret(); nop();
        hwords = NW - h0;
        W[cnt_idx] = (0x0Du << 26) | ((uint32_t)zero << 21) | ((uint32_t)t2 << 16) | (uint32_t)hwords;   /* ori t2, zero, hwords */
    }
    for (i = 0; i < NF; i++) {                                                         /* resolve_la, then resolve */
        uint32_t tt = lab(FIX[i].lab);
        if (FIX[i].kind == 'h') W[FIX[i].idx] |= (tt >> 16) & 0xFFFF;
        else if (FIX[i].kind == 'l') W[FIX[i].idx] |= tt & 0xFFFF;
    }
    for (i = 0; i < NF; i++) {
        uint32_t tt = lab(FIX[i].lab);
        if (FIX[i].kind == 'b') {
            int64_t d = (int64_t)tt - (int64_t)(ABASE + 4u * (uint32_t)FIX[i].idx + 4);
            int64_t off = d >= 0 ? d / 4 : -((-d + 3) / 4);
            if (off < -32768 || off >= 32768) fail("branch range");
            W[FIX[i].idx] |= (uint32_t)off & 0xFFFF;
        } else if (FIX[i].kind == 'j') {
            W[FIX[i].idx] |= (tt >> 2) & 0x03FFFFFF;
        }
    }
    ASSET_ROM = 0x1000u + (((uint32_t)NW * 4 + 0xFFF) & ~0xFFFu);
    {
        uint32_t ca = 0x10000000u + ASSET_ROM, ln = (uint32_t)nassets - 1;
        W[asset_cart_idx - 1] = (0x0Du << 26) | ((uint32_t)t1 << 16);                      /* ori t1, zero, 0 (harmless) */
        W[asset_cart_idx] = (0x0Fu << 26) | ((uint32_t)t1 << 16) | (ca >> 16);
        W[asset_cart_idx + 1] = (0x0Du << 26) | ((uint32_t)t1 << 21) | ((uint32_t)t1 << 16) | (ca & 0xFFFF);
        W[asset_len_idx - 1] = (0x0Du << 26) | ((uint32_t)t1 << 16);
        W[asset_len_idx] = (0x0Fu << 26) | ((uint32_t)t1 << 16) | (ln >> 16);
        W[asset_len_idx + 1] = (0x0Du << 26) | ((uint32_t)t1 << 21) | ((uint32_t)t1 << 16) | (ln & 0xFFFF);
    }

    /* ------------------------------------------------------------ ROM image */
    romsize = ASSET_ROM + nassets + 0x1000;
    if (romsize < 0x101000) romsize = 0x101000;
    rom = (unsigned char *)calloc(1, romsize);
    if (!rom) fail("oom");
    memcpy(rom + ASSET_ROM, assets, nassets);
    for (i = 0; i < NW; i++) { rom[0x1000 + i * 4] = (unsigned char)(W[i] >> 24); rom[0x1001 + i * 4] = (unsigned char)(W[i] >> 16); rom[0x1002 + i * 4] = (unsigned char)(W[i] >> 8); rom[0x1003 + i * 4] = (unsigned char)W[i]; }
    {
        static const uint32_t hdr[4] = { 0x80371240, 0x0000000F, 0x80000400, 0x0000144B };
        static const char stub[] = "HandshakeOS clean-room boot area: IPL3 behaviour provided by PIF HLE. MIT.";
        for (i = 0; i < 4; i++) { rom[i * 4] = (unsigned char)(hdr[i] >> 24); rom[i * 4 + 1] = (unsigned char)(hdr[i] >> 16); rom[i * 4 + 2] = (unsigned char)(hdr[i] >> 8); rom[i * 4 + 3] = (unsigned char)hdr[i]; }
        memcpy(rom + 0x20, "HANDSHAKEOS N64 DEMO", 20);
        memcpy(rom + 0x3B, "NHSE\0", 5);
        /* boot-code area: original placeholder (IPL3 is executed by the emulator's PIF/IPL3 HLE) */
        memcpy(rom + 0x40, stub, sizeof stub - 1);
    }
    {
        uint32_t seed = 0xF8CA4DDC, c1_ = seed, c2_ = seed, c3 = seed, c4 = seed, c5 = seed, c6 = seed, crc1, crc2;
        size_t p;
        for (p = 0x1000; p < 0x101000; p += 4) {
            uint32_t d = (uint32_t)rom[p] << 24 | (uint32_t)rom[p + 1] << 16 | (uint32_t)rom[p + 2] << 8 | rom[p + 3], r;
            if ((uint32_t)(c6 + d) < c6) c4 = c4 + 1;
            c6 = c6 + d; c3 ^= d; r = rol(d, d & 31); c5 = c5 + r;
            c2_ = c2_ > d ? (c2_ ^ r) : (c2_ ^ c6 ^ d);
            c1_ = c1_ + (c5 ^ d);
        }
        crc1 = c6 ^ c4 ^ c3; crc2 = c5 ^ c2_ ^ c1_;
        for (i = 0; i < 4; i++) { rom[0x10 + i] = (unsigned char)(crc1 >> (24 - 8 * i)); rom[0x14 + i] = (unsigned char)(crc2 >> (24 - 8 * i)); }
        fp_ = fopen(out, "wb");
        if (!fp_ || fwrite(rom, 1, romsize, fp_) != romsize || fclose(fp_)) { perror(out); return 1; }
        printf("wrote %s: %zu bytes, code %d bytes (%d MIPS instructions), assets %zu bytes @ROM 0x%X -> RDRAM 0x%X, DL %d cmds, "
               "ADPCM %d bytes, CRC1 %08X CRC2 %08X, fog fm=%d fo=%d\n",
               out, romsize, NW * 4, NW, nassets, ASSET_ROM, ASSET_RD, NDL, ADPCM_LEN, crc1, crc2, fm, fo);
    }
    free(rom);
    return 0;
}
