/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_z80.c - clean-room Zilog Z80 interpreter for the hs-genesis sound
 * phase. Decoded with the classic x/y/z/p/q opcode field scheme from the
 * public Z80 user manual; DD/FD prefixes substitute IX/IY for HL (including
 * the undocumented IXH/IXL halves), DDCB/FDCB copy results to registers.
 * Flags include the undocumented Y/X (bits 5/3) copies of the result.
 * Pure C11, no allocation, no global mutable state, zero locks.
 */
#include "gen_z80.h"
#include <string.h>

enum { FC = 0x01, FN = 0x02, FP = 0x04, FX = 0x08, FH = 0x10, FY = 0x20, FZ = 0x40, FS = 0x80 };

static const uint8_t k_cyc_main[256] = {
     4,10, 7, 6, 4, 4, 7, 4, 4,11, 7, 6, 4, 4, 7, 4,
     8,10, 7, 6, 4, 4, 7, 4,12,11, 7, 6, 4, 4, 7, 4,
     7,10,16, 6, 4, 4, 7, 4, 7,11,16, 6, 4, 4, 7, 4,
     7,10,13, 6,11,11,10, 4, 7,11,13, 6, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     7, 7, 7, 7, 7, 7, 4, 7, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
     5,10,10,10,10,11, 7,11, 5,10,10, 0,10,17, 7,11,
     5,10,10,11,10,11, 7,11, 5, 4,10,11,10, 0, 7,11,
     5,10,10,19,10,11, 7,11, 5, 4,10, 4,10, 0, 7,11,
     5,10,10, 4,10,11, 7,11, 5, 6,10, 4,10, 0, 7,11
};

static inline uint8_t parity(uint8_t v)
{
    v ^= (uint8_t)(v >> 4); v ^= (uint8_t)(v >> 2); v ^= (uint8_t)(v >> 1);
    return (v & 1) ? 0 : FP;
}
static inline uint8_t szyx(uint8_t v) { return (uint8_t)((v & (FS | FY | FX)) | (v ? 0 : FZ)); }
static inline uint8_t szyxp(uint8_t v) { return (uint8_t)(szyx(v) | parity(v)); }

static inline uint8_t rd(gen_z80_t *z, uint16_t a) { return z->bus.read(z->bus.ctx, a); }
static inline void wr(gen_z80_t *z, uint16_t a, uint8_t v) { z->bus.write(z->bus.ctx, a, v); }
static inline uint8_t f8(gen_z80_t *z) { return rd(z, z->pc++); }
static inline uint16_t f16(gen_z80_t *z) { uint16_t lo = f8(z); return (uint16_t)(lo | (f8(z) << 8)); }
static inline uint16_t rd16(gen_z80_t *z, uint16_t a) { return (uint16_t)(rd(z, a) | (rd(z, (uint16_t)(a + 1)) << 8)); }
static inline void wr16(gen_z80_t *z, uint16_t a, uint16_t v) { wr(z, a, (uint8_t)v); wr(z, (uint16_t)(a + 1), (uint8_t)(v >> 8)); }
static inline void push(gen_z80_t *z, uint16_t v) { z->sp = (uint16_t)(z->sp - 2); wr16(z, z->sp, v); }
static inline uint16_t pop(gen_z80_t *z) { uint16_t v = rd16(z, z->sp); z->sp = (uint16_t)(z->sp + 2); return v; }

#define BC ((uint16_t)((z->b << 8) | z->c))
#define DE ((uint16_t)((z->d << 8) | z->e))
#define HL ((uint16_t)((z->h << 8) | z->l))
#define AF ((uint16_t)((z->a << 8) | z->f))

static inline void set_bc(gen_z80_t *z, uint16_t v) { z->b = (uint8_t)(v >> 8); z->c = (uint8_t)v; }
static inline void set_de(gen_z80_t *z, uint16_t v) { z->d = (uint8_t)(v >> 8); z->e = (uint8_t)v; }
static inline void set_hl(gen_z80_t *z, uint16_t v) { z->h = (uint8_t)(v >> 8); z->l = (uint8_t)v; }

/* pfx: 0 = HL, 1 = IX, 2 = IY */
static inline uint16_t get_hlx(gen_z80_t *z, int pfx) { return pfx == 0 ? HL : (pfx == 1 ? z->ix : z->iy); }
static inline void set_hlx(gen_z80_t *z, int pfx, uint16_t v)
{
    if (pfx == 0) set_hl(z, v); else if (pfx == 1) z->ix = v; else z->iy = v;
}

static uint16_t get_rp(gen_z80_t *z, int p, int pfx)
{
    switch (p) { case 0: return BC; case 1: return DE; case 2: return get_hlx(z, pfx); default: return z->sp; }
}
static void set_rp(gen_z80_t *z, int p, int pfx, uint16_t v)
{
    switch (p) { case 0: set_bc(z, v); break; case 1: set_de(z, v); break;
    case 2: set_hlx(z, pfx, v); break; default: z->sp = v; break; }
}
static uint16_t get_rp2(gen_z80_t *z, int p, int pfx) { return p == 3 ? AF : get_rp(z, p, pfx); }
static void set_rp2(gen_z80_t *z, int p, int pfx, uint16_t v)
{
    if (p == 3) { z->a = (uint8_t)(v >> 8); z->f = (uint8_t)v; } else set_rp(z, p, pfx, v);
}

/* 8-bit register r (0..7, r != 6). pfx selects IXH/IXL for 4/5. */
static uint8_t get_r(gen_z80_t *z, int r, int pfx)
{
    switch (r) {
    case 0: return z->b; case 1: return z->c; case 2: return z->d; case 3: return z->e;
    case 4: return pfx == 0 ? z->h : (uint8_t)((pfx == 1 ? z->ix : z->iy) >> 8);
    case 5: return pfx == 0 ? z->l : (uint8_t)(pfx == 1 ? z->ix : z->iy);
    default: return z->a;
    }
}
static void set_r(gen_z80_t *z, int r, int pfx, uint8_t v)
{
    switch (r) {
    case 0: z->b = v; break; case 1: z->c = v; break; case 2: z->d = v; break; case 3: z->e = v; break;
    case 4:
        if (pfx == 0) z->h = v;
        else if (pfx == 1) z->ix = (uint16_t)((z->ix & 0xFF) | (v << 8));
        else z->iy = (uint16_t)((z->iy & 0xFF) | (v << 8));
        break;
    case 5:
        if (pfx == 0) z->l = v;
        else if (pfx == 1) z->ix = (uint16_t)((z->ix & 0xFF00) | v);
        else z->iy = (uint16_t)((z->iy & 0xFF00) | v);
        break;
    default: z->a = v; break;
    }
}

static uint16_t idx_addr(gen_z80_t *z, int pfx)
{
    if (pfx == 0) return HL;
    {
        int8_t dd = (int8_t)f8(z);
        z->cyc += 8;
        return (uint16_t)(get_hlx(z, pfx) + dd);
    }
}

static int cc(const gen_z80_t *z, int y)
{
    switch (y) {
    case 0: return !(z->f & FZ); case 1: return (z->f & FZ) != 0;
    case 2: return !(z->f & FC); case 3: return (z->f & FC) != 0;
    case 4: return !(z->f & FP); case 5: return (z->f & FP) != 0;
    case 6: return !(z->f & FS); default: return (z->f & FS) != 0;
    }
}

static void alu(gen_z80_t *z, int op, uint8_t v)
{
    uint8_t a = z->a, cin = (z->f & FC) ? 1 : 0;
    unsigned r;
    if (op == 0 || op == 2) cin = 0;
    switch (op) {
    case 0: case 1:
        r = (unsigned)a + v + cin;
        z->a = (uint8_t)r;
        z->f = (uint8_t)(szyx((uint8_t)r) | ((a ^ v ^ r) & FH) |
                         ((((a ^ ~v) & (a ^ r)) & 0x80) ? FP : 0) | (r > 0xFF ? FC : 0));
        return;
    case 2: case 3:
        r = (unsigned)a - v - cin;
        z->a = (uint8_t)r;
        z->f = (uint8_t)(szyx((uint8_t)r) | ((a ^ v ^ r) & FH) | FN |
                         ((((a ^ v) & (a ^ r)) & 0x80) ? FP : 0) | ((r & 0x100) ? FC : 0));
        return;
    case 4: z->a = a & v; z->f = (uint8_t)(szyxp(z->a) | FH); return;
    case 5: z->a = a ^ v; z->f = szyxp(z->a); return;
    case 6: z->a = a | v; z->f = szyxp(z->a); return;
    default:
        r = (unsigned)a - v;
        z->f = (uint8_t)((szyx((uint8_t)r) & (FS | FZ)) | (v & (FY | FX)) | ((a ^ v ^ r) & FH) | FN |
                         ((((a ^ v) & (a ^ r)) & 0x80) ? FP : 0) | ((r & 0x100) ? FC : 0));
        return;
    }
}

static uint8_t inc8(gen_z80_t *z, uint8_t v)
{
    uint8_t r = (uint8_t)(v + 1);
    z->f = (uint8_t)((z->f & FC) | szyx(r) | ((r & 0xF) == 0 ? FH : 0) | (r == 0x80 ? FP : 0));
    return r;
}
static uint8_t dec8(gen_z80_t *z, uint8_t v)
{
    uint8_t r = (uint8_t)(v - 1);
    z->f = (uint8_t)((z->f & FC) | FN | szyx(r) | ((r & 0xF) == 0xF ? FH : 0) | (r == 0x7F ? FP : 0));
    return r;
}

static uint8_t rot(gen_z80_t *z, int y, uint8_t v)
{
    uint8_t c, r;
    switch (y) {
    case 0: c = v >> 7; r = (uint8_t)((v << 1) | c); break;
    case 1: c = v & 1; r = (uint8_t)((v >> 1) | (c << 7)); break;
    case 2: c = v >> 7; r = (uint8_t)((v << 1) | ((z->f & FC) ? 1 : 0)); break;
    case 3: c = v & 1; r = (uint8_t)((v >> 1) | ((z->f & FC) ? 0x80 : 0)); break;
    case 4: c = v >> 7; r = (uint8_t)(v << 1); break;
    case 5: c = v & 1; r = (uint8_t)((v >> 1) | (v & 0x80)); break;
    case 6: c = v >> 7; r = (uint8_t)((v << 1) | 1); break;
    default: c = v & 1; r = (uint8_t)(v >> 1); break;
    }
    z->f = (uint8_t)(szyxp(r) | (c ? FC : 0));
    return r;
}

static void add16(gen_z80_t *z, int pfx, uint16_t v)
{
    uint16_t hl = get_hlx(z, pfx);
    uint32_t r = (uint32_t)hl + v;
    z->f = (uint8_t)((z->f & (FS | FZ | FP)) | ((r >> 8) & (FY | FX)) |
                     (((hl ^ v ^ r) >> 8) & FH) | (r > 0xFFFF ? FC : 0));
    set_hlx(z, pfx, (uint16_t)r);
}
static void adc16(gen_z80_t *z, uint16_t v)
{
    uint16_t hl = HL;
    uint32_t r = (uint32_t)hl + v + ((z->f & FC) ? 1u : 0u);
    uint16_t r16 = (uint16_t)r;
    z->f = (uint8_t)(((r16 >> 8) & (FS | FY | FX)) | (r16 ? 0 : FZ) | (((hl ^ v ^ r) >> 8) & FH) |
                     ((((hl ^ ~v) & (hl ^ r)) & 0x8000) ? FP : 0) | (r > 0xFFFF ? FC : 0));
    set_hl(z, r16);
}
static void sbc16(gen_z80_t *z, uint16_t v)
{
    uint16_t hl = HL;
    uint32_t r = (uint32_t)hl - v - ((z->f & FC) ? 1u : 0u);
    uint16_t r16 = (uint16_t)r;
    z->f = (uint8_t)(((r16 >> 8) & (FS | FY | FX)) | (r16 ? 0 : FZ) | (((hl ^ v ^ r) >> 8) & FH) | FN |
                     ((((hl ^ v) & (hl ^ r)) & 0x8000) ? FP : 0) | ((r & 0x10000) ? FC : 0));
    set_hl(z, r16);
}

static void daa(gen_z80_t *z)
{
    uint8_t a = z->a, diff = 0, carry = (z->f & FC) ? 1 : 0, lo = a & 0xF, hf;
    if ((z->f & FH) || lo > 9) diff |= 0x06;
    if (carry || a > 0x99) { diff |= 0x60; carry = 1; }
    if (z->f & FN) { hf = ((z->f & FH) && lo < 6) ? FH : 0; z->a = (uint8_t)(a - diff); }
    else { hf = lo > 9 ? FH : 0; z->a = (uint8_t)(a + diff); }
    z->f = (uint8_t)(szyxp(z->a) | (z->f & FN) | (carry ? FC : 0) | hf);
}

/* ---- CB page (with optional indexed address) --------------------------- */
static void exec_cb(gen_z80_t *z, int pfx)
{
    uint16_t addr = 0;
    uint8_t op, v, r;
    int x, y, zz;
    if (pfx) { int8_t dd = (int8_t)f8(z); addr = (uint16_t)(get_hlx(z, pfx) + dd); op = f8(z); z->cyc += 15; }
    else { op = f8(z); z->r = (uint8_t)((z->r & 0x80) | ((z->r + 1) & 0x7F)); z->cyc += 8; }
    x = op >> 6; y = (op >> 3) & 7; zz = op & 7;
    if (pfx || zz == 6) {
        if (!pfx) { addr = HL; z->cyc += 7; }
        v = rd(z, addr);
    } else v = get_r(z, zz, 0);
    switch (x) {
    case 0: r = rot(z, y, v); break;
    case 1:
        z->f = (uint8_t)((z->f & FC) | FH | (v & (FY | FX)) |
                         ((v & (1u << y)) ? 0 : (FZ | FP)) | ((y == 7 && (v & 0x80)) ? FS : 0));
        return;
    case 2: r = (uint8_t)(v & ~(1u << y)); break;
    default: r = (uint8_t)(v | (1u << y)); break;
    }
    if (pfx || zz == 6) { wr(z, addr, r); if (pfx && zz != 6) set_r(z, zz, 0, r); }
    else set_r(z, zz, 0, r);
}

/* ---- ED page ------------------------------------------------------------- */
static void exec_ed(gen_z80_t *z)
{
    uint8_t op = f8(z);
    int x = op >> 6, y = (op >> 3) & 7, zz = op & 7, p = y >> 1, q = y & 1;
    z->r = (uint8_t)((z->r & 0x80) | ((z->r + 1) & 0x7F));
    z->cyc += 8;
    if (x == 1) {
        switch (zz) {
        case 0: { uint8_t v = z->bus.in(z->bus.ctx, BC); if (y != 6) set_r(z, y, 0, v);
                  z->f = (uint8_t)((z->f & FC) | szyxp(v)); z->cyc += 4; return; }
        case 1: z->bus.out(z->bus.ctx, BC, y == 6 ? 0 : get_r(z, y, 0)); z->cyc += 4; return;
        case 2: if (q) adc16(z, get_rp(z, p, 0)); else sbc16(z, get_rp(z, p, 0)); z->cyc += 7; return;
        case 3: { uint16_t nn = f16(z);
                  if (q) set_rp(z, p, 0, rd16(z, nn)); else wr16(z, nn, get_rp(z, p, 0));
                  z->cyc += 12; return; }
        case 4: { uint8_t v = z->a; z->a = 0; alu(z, 2, v); return; }
        case 5: z->pc = pop(z); z->iff1 = z->iff2; z->cyc += 6; return;
        case 6: { static const uint8_t ims[8] = { 0, 0, 1, 2, 0, 0, 1, 2 }; z->im = ims[y]; return; }
        default:
            switch (y) {
            case 0: z->i = z->a; z->cyc += 1; return;
            case 1: z->r = z->a; z->cyc += 1; return;
            case 2: z->a = z->i; z->f = (uint8_t)((z->f & FC) | szyx(z->a) | (z->iff2 ? FP : 0)); z->cyc += 1; return;
            case 3: z->a = z->r; z->f = (uint8_t)((z->f & FC) | szyx(z->a) | (z->iff2 ? FP : 0)); z->cyc += 1; return;
            case 4: { uint8_t m = rd(z, HL);
                      wr(z, HL, (uint8_t)((z->a << 4) | (m >> 4)));
                      z->a = (uint8_t)((z->a & 0xF0) | (m & 0x0F));
                      z->f = (uint8_t)((z->f & FC) | szyxp(z->a)); z->cyc += 10; return; }
            case 5: { uint8_t m = rd(z, HL);
                      wr(z, HL, (uint8_t)((m << 4) | (z->a & 0x0F)));
                      z->a = (uint8_t)((z->a & 0xF0) | (m >> 4));
                      z->f = (uint8_t)((z->f & FC) | szyxp(z->a)); z->cyc += 10; return; }
            default: return;
            }
        }
    }
    if (x == 2 && y >= 4 && zz <= 3) {
        int dir = (y & 1) ? -1 : 1, rep = y >= 6;
        uint16_t bc;
        z->cyc += 8;
        switch (zz) {
        case 0: {
            uint8_t v = rd(z, HL), n;
            wr(z, DE, v);
            set_hl(z, (uint16_t)(HL + dir)); set_de(z, (uint16_t)(DE + dir));
            bc = (uint16_t)(BC - 1); set_bc(z, bc);
            n = (uint8_t)(v + z->a);
            z->f = (uint8_t)((z->f & (FS | FZ | FC)) | (bc ? FP : 0) | (n & FX) | ((n << 4) & FY));
            if (rep && bc) { z->pc = (uint16_t)(z->pc - 2); z->cyc += 5; }
            return;
        }
        case 1: {
            uint8_t v = rd(z, HL), r = (uint8_t)(z->a - v), n;
            uint8_t hf = (uint8_t)((z->a ^ v ^ r) & FH);
            set_hl(z, (uint16_t)(HL + dir));
            bc = (uint16_t)(BC - 1); set_bc(z, bc);
            n = (uint8_t)(r - (hf ? 1 : 0));
            z->f = (uint8_t)((z->f & FC) | FN | (r & FS) | (r ? 0 : FZ) | hf | (bc ? FP : 0) |
                             (n & FX) | ((n << 4) & FY));
            if (rep && bc && r) { z->pc = (uint16_t)(z->pc - 2); z->cyc += 5; }
            return;
        }
        case 2: {
            uint8_t v = z->bus.in(z->bus.ctx, BC);
            wr(z, HL, v);
            set_hl(z, (uint16_t)(HL + dir));
            z->b = (uint8_t)(z->b - 1);
            z->f = (uint8_t)(szyx(z->b) | FN);
            if (rep && z->b) { z->pc = (uint16_t)(z->pc - 2); z->cyc += 5; }
            return;
        }
        default: {
            uint8_t v = rd(z, HL);
            z->b = (uint8_t)(z->b - 1);
            z->bus.out(z->bus.ctx, BC, v);
            set_hl(z, (uint16_t)(HL + dir));
            z->f = (uint8_t)(szyx(z->b) | FN);
            if (rep && z->b) { z->pc = (uint16_t)(z->pc - 2); z->cyc += 5; }
            return;
        }
        }
    }
    /* undefined ED opcodes behave as 8-cycle NOPs */
}

static void exec_one(gen_z80_t *z)
{
    int pfx = 0;
    uint8_t op;
    int x, y, zz, p, q;

    op = f8(z);
    z->r = (uint8_t)((z->r & 0x80) | ((z->r + 1) & 0x7F));
    while (op == 0xDD || op == 0xFD) {
        pfx = op == 0xDD ? 1 : 2;
        z->cyc += 4;
        op = f8(z);
        z->r = (uint8_t)((z->r & 0x80) | ((z->r + 1) & 0x7F));
        if (z->cyc > (uint64_t)1 << 62) break;
    }
    if (op == 0xCB) { exec_cb(z, pfx); return; }
    if (op == 0xED) { exec_ed(z); return; }
    z->cyc += k_cyc_main[op];
    x = op >> 6; y = (op >> 3) & 7; zz = op & 7; p = y >> 1; q = y & 1;

    switch (x) {
    case 0:
        switch (zz) {
        case 0:
            switch (y) {
            case 0: return;
            case 1: { uint8_t t = z->a; z->a = z->a2; z->a2 = t; t = z->f; z->f = z->f2; z->f2 = t; return; }
            case 2: { int8_t d = (int8_t)f8(z); z->b = (uint8_t)(z->b - 1);
                      if (z->b) { z->pc = (uint16_t)(z->pc + d); z->cyc += 5; } return; }
            case 3: { int8_t d = (int8_t)f8(z); z->pc = (uint16_t)(z->pc + d); return; }
            default: { int8_t d = (int8_t)f8(z);
                       if (cc(z, y - 4)) { z->pc = (uint16_t)(z->pc + d); z->cyc += 5; } return; }
            }
        case 1:
            if (q == 0) set_rp(z, p, pfx, f16(z));
            else add16(z, pfx, get_rp(z, p, pfx));
            return;
        case 2:
            if (q == 0) {
                switch (p) {
                case 0: wr(z, BC, z->a); return;
                case 1: wr(z, DE, z->a); return;
                case 2: wr16(z, f16(z), get_hlx(z, pfx)); return;
                default: wr(z, f16(z), z->a); return;
                }
            }
            switch (p) {
            case 0: z->a = rd(z, BC); return;
            case 1: z->a = rd(z, DE); return;
            case 2: set_hlx(z, pfx, rd16(z, f16(z))); return;
            default: z->a = rd(z, f16(z)); return;
            }
        case 3:
            set_rp(z, p, pfx, (uint16_t)(get_rp(z, p, pfx) + (q ? -1 : 1)));
            return;
        case 4: case 5:
            if (y == 6) {
                uint16_t a = idx_addr(z, pfx);
                uint8_t v = rd(z, a);
                wr(z, a, zz == 4 ? inc8(z, v) : dec8(z, v));
            } else {
                uint8_t v = get_r(z, y, pfx);
                set_r(z, y, pfx, zz == 4 ? inc8(z, v) : dec8(z, v));
            }
            return;
        case 6:
            if (y == 6) { uint16_t a = idx_addr(z, pfx); wr(z, a, f8(z)); }
            else set_r(z, y, pfx, f8(z));
            return;
        default:
            switch (y) {
            case 0: { uint8_t c = z->a >> 7; z->a = (uint8_t)((z->a << 1) | c);
                      z->f = (uint8_t)((z->f & (FS | FZ | FP)) | (z->a & (FY | FX)) | c); return; }
            case 1: { uint8_t c = z->a & 1; z->a = (uint8_t)((z->a >> 1) | (c << 7));
                      z->f = (uint8_t)((z->f & (FS | FZ | FP)) | (z->a & (FY | FX)) | c); return; }
            case 2: { uint8_t c = z->a >> 7; z->a = (uint8_t)((z->a << 1) | (z->f & FC));
                      z->f = (uint8_t)((z->f & (FS | FZ | FP)) | (z->a & (FY | FX)) | c); return; }
            case 3: { uint8_t c = z->a & 1; z->a = (uint8_t)((z->a >> 1) | ((z->f & FC) << 7));
                      z->f = (uint8_t)((z->f & (FS | FZ | FP)) | (z->a & (FY | FX)) | c); return; }
            case 4: daa(z); return;
            case 5: z->a = (uint8_t)~z->a;
                    z->f = (uint8_t)((z->f & (FS | FZ | FP | FC)) | FH | FN | (z->a & (FY | FX))); return;
            case 6: z->f = (uint8_t)((z->f & (FS | FZ | FP)) | FC | (z->a & (FY | FX))); return;
            default: z->f = (uint8_t)((z->f & (FS | FZ | FP)) | ((z->f & FC) ? FH : FC) | (z->a & (FY | FX))); return;
            }
        }
    case 1:
        if (op == 0x76) { z->halted = 1; return; }
        if (zz == 6) { uint16_t a = idx_addr(z, pfx); set_r(z, y, 0, rd(z, a)); return; }
        if (y == 6) { uint16_t a = idx_addr(z, pfx); wr(z, a, get_r(z, zz, 0)); return; }
        set_r(z, y, pfx, get_r(z, zz, pfx));
        return;
    case 2:
        if (zz == 6) { uint16_t a = idx_addr(z, pfx); alu(z, y, rd(z, a)); }
        else alu(z, y, get_r(z, zz, pfx));
        return;
    default:
        switch (zz) {
        case 0: if (cc(z, y)) { z->pc = pop(z); z->cyc += 6; } return;
        case 1:
            if (q == 0) { set_rp2(z, p, pfx, pop(z)); return; }
            switch (p) {
            case 0: z->pc = pop(z); return;
            case 1: { uint8_t t;
                      t = z->b; z->b = z->b2; z->b2 = t; t = z->c; z->c = z->c2; z->c2 = t;
                      t = z->d; z->d = z->d2; z->d2 = t; t = z->e; z->e = z->e2; z->e2 = t;
                      t = z->h; z->h = z->h2; z->h2 = t; t = z->l; z->l = z->l2; z->l2 = t; return; }
            case 2: z->pc = get_hlx(z, pfx); return;
            default: z->sp = get_hlx(z, pfx); return;
            }
        case 2: { uint16_t nn = f16(z); if (cc(z, y)) z->pc = nn; return; }
        case 3:
            switch (y) {
            case 0: z->pc = f16(z); return;
            case 2: z->bus.out(z->bus.ctx, (uint16_t)((z->a << 8) | f8(z)), z->a); return;
            case 3: z->a = z->bus.in(z->bus.ctx, (uint16_t)((z->a << 8) | f8(z))); return;
            case 4: { uint16_t v = rd16(z, z->sp); wr16(z, z->sp, get_hlx(z, pfx)); set_hlx(z, pfx, v); return; }
            case 5: { uint16_t t = DE; set_de(z, HL); set_hl(z, t); return; }
            case 6: z->iff1 = z->iff2 = 0; return;
            case 7: z->iff1 = z->iff2 = 1; z->ei_delay = 1; return;
            default: return;
            }
        case 4: { uint16_t nn = f16(z); if (cc(z, y)) { push(z, z->pc); z->pc = nn; z->cyc += 7; } return; }
        case 5:
            if (q == 0) { push(z, get_rp2(z, p, pfx)); return; }
            if (p == 0) { uint16_t nn = f16(z); push(z, z->pc); z->pc = nn; }
            return;
        case 6: alu(z, y, f8(z)); return;
        default: push(z, z->pc); z->pc = (uint16_t)(y * 8); return;
        }
    }
}

void z80_init(gen_z80_t *z, const z80_bus_t *bus)
{
    memset(z, 0, sizeof(*z));
    z->bus = *bus;
    z80_reset(z);
}

void z80_reset(gen_z80_t *z)
{
    z->pc = 0; z->sp = 0xFFFF; z->i = 0; z->r = 0;
    z->iff1 = z->iff2 = 0; z->im = 0; z->halted = 0; z->ei_delay = 0;
    z->a = z->f = 0xFF;
    z->ix = z->iy = 0xFFFF;
}

void z80_run_until(gen_z80_t *z, uint64_t target)
{
    while (z->cyc < target) {
        if (z->irq_line && z->iff1 && !z->ei_delay) {
            z->iff1 = z->iff2 = 0;
            z->halted = 0;
            push(z, z->pc);
            if (z->im == 2) { z->pc = rd16(z, (uint16_t)((z->i << 8) | 0xFF)); z->cyc += 19; }
            else { z->pc = 0x38; z->cyc += 13; }
            continue;
        }
        z->ei_delay = 0;
        if (z->halted) {
            uint64_t n = (target - z->cyc + 3) / 4;
            z->cyc += n * 4;
            z->r = (uint8_t)((z->r & 0x80) | ((z->r + n) & 0x7F));
            break;
        }
        exec_one(z);
        z->insns++;
    }
}
