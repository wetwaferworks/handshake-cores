/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * m68k_cpu.c - clean-room Motorola 68000 interpreter for hs-genesis.
 *
 * Written from the public 68000 programmer's reference model: 8 data and
 * 8 address registers, USP/SSP banking, SR (T S I2-I0 X N Z V C), big-endian
 * 24-bit bus, 14 addressing modes, autovectored interrupts. Decoding is a
 * 65536-entry opcode -> class table built once at init (EA-mode validity is
 * resolved at table-build time, so execution never re-validates).
 * Pure C11, no allocation, no global mutable state, zero locks.
 */
#include "m68k_cpu.h"
#include <string.h>

enum { CF = 1, VF = 2, ZF = 4, NF = 8, XF = 16 };

enum {
    O_ILL = 0, O_LINEA, O_LINEF,
    O_ORI_CCR, O_ORI_SR, O_ANDI_CCR, O_ANDI_SR, O_EORI_CCR, O_EORI_SR,
    O_ORI, O_ANDI, O_SUBI, O_ADDI, O_EORI, O_CMPI,
    O_BIT_R, O_BIT_I, O_MOVEP,
    O_MOVE, O_MOVEA,
    O_NEGX, O_MOVE_FR_SR, O_CHK, O_LEA, O_CLR, O_MOVE_TO_CCR, O_NEG, O_NOT,
    O_MOVE_TO_SR, O_NBCD, O_PEA, O_SWAP, O_MOVEM_RM, O_MOVEM_MR, O_EXT, O_TST,
    O_TAS, O_TRAP, O_LINK, O_UNLK, O_MOVE_USP, O_RESET, O_NOP, O_STOP, O_RTE,
    O_RTS, O_TRAPV, O_RTR, O_JSR, O_JMP,
    O_ADDQ, O_SCC, O_DBCC, O_BCC, O_MOVEQ,
    O_OR, O_DIVU, O_DIVS, O_SBCD,
    O_SUB, O_SUBA, O_SUBX,
    O_CMP, O_CMPA, O_CMPM, O_EOR,
    O_AND, O_MULU, O_MULS, O_ABCD, O_EXG,
    O_ADD, O_ADDA, O_ADDX,
    O_SHIFT_R, O_SHIFT_M,
    O_COUNT
};

/* ---- EA mode classes ---------------------------------------------------- */
#define M_DN  0x001u
#define M_AN  0x002u
#define M_IND 0x004u
#define M_PI  0x008u
#define M_PD  0x010u
#define M_D16 0x020u
#define M_IDX 0x040u
#define M_AW  0x080u
#define M_AL  0x100u
#define M_PCD 0x200u
#define M_PCI 0x400u
#define M_IMM 0x800u
#define EA_ALL  0xFFFu
#define EA_DATA (EA_ALL & ~M_AN)
#define EA_MEM  (EA_ALL & ~(M_DN | M_AN))
#define EA_ALT  0x1FFu
#define EA_DALT (EA_ALT & ~M_AN)
#define EA_MALT (EA_ALT & ~(M_DN | M_AN))
#define EA_CTRL (M_IND | M_D16 | M_IDX | M_AW | M_AL | M_PCD | M_PCI)
#define EA_CALT (M_IND | M_D16 | M_IDX | M_AW | M_AL)

static int ea_ok(int mode, int reg, uint32_t mask)
{
    int idx;
    if (mode < 7) idx = mode;
    else if (reg <= 4) idx = 7 + reg;
    else return 0;
    return (mask >> idx) & 1u;
}

static uint8_t decode(uint16_t op)
{
    int mode = (op >> 3) & 7, reg = op & 7, sz2 = (op >> 6) & 3;
#define OK(m) ea_ok(mode, reg, (m))
    switch (op >> 12) {
    case 0x0:
        if (op & 0x100) {
            if (mode == 1) return O_MOVEP;
            if (sz2 == 0) return OK(EA_DATA) ? O_BIT_R : O_ILL;
            return OK(EA_DALT) ? O_BIT_R : O_ILL;
        }
        switch ((op >> 9) & 7) {
        case 0:
            if (op == 0x003C) return O_ORI_CCR;
            if (op == 0x007C) return O_ORI_SR;
            return (sz2 != 3 && OK(EA_DALT)) ? O_ORI : O_ILL;
        case 1:
            if (op == 0x023C) return O_ANDI_CCR;
            if (op == 0x027C) return O_ANDI_SR;
            return (sz2 != 3 && OK(EA_DALT)) ? O_ANDI : O_ILL;
        case 2: return (sz2 != 3 && OK(EA_DALT)) ? O_SUBI : O_ILL;
        case 3: return (sz2 != 3 && OK(EA_DALT)) ? O_ADDI : O_ILL;
        case 4:
            if (sz2 == 0) return OK(EA_DATA & ~M_IMM) ? O_BIT_I : O_ILL;
            return OK(EA_DALT) ? O_BIT_I : O_ILL;
        case 5:
            if (op == 0x0A3C) return O_EORI_CCR;
            if (op == 0x0A7C) return O_EORI_SR;
            return (sz2 != 3 && OK(EA_DALT)) ? O_EORI : O_ILL;
        case 6: return (sz2 != 3 && OK(EA_DALT)) ? O_CMPI : O_ILL;
        default: return O_ILL;
        }
    case 0x1: case 0x2: case 0x3: {
        int dmode = (op >> 6) & 7, dreg = (op >> 9) & 7;
        int sz = (op >> 12) == 1 ? 1 : ((op >> 12) == 3 ? 2 : 4);
        if (!OK(EA_ALL)) return O_ILL;
        if (sz == 1 && mode == 1) return O_ILL;
        if (dmode == 1) return sz == 1 ? O_ILL : O_MOVEA;
        return ea_ok(dmode, dreg, EA_DALT) ? O_MOVE : O_ILL;
    }
    case 0x4:
        if ((op & 0x1C0) == 0x1C0) return OK(EA_CTRL) ? O_LEA : O_ILL;
        if ((op & 0x1C0) == 0x180) return OK(EA_DATA) ? O_CHK : O_ILL;
        if (op & 0x100) return O_ILL;
        switch ((op >> 8) & 0xF) {
        case 0x0: return OK(EA_DALT) ? (sz2 == 3 ? O_MOVE_FR_SR : O_NEGX) : O_ILL;
        case 0x2: return (sz2 != 3 && OK(EA_DALT)) ? O_CLR : O_ILL;
        case 0x4:
            if (sz2 == 3) return OK(EA_DATA) ? O_MOVE_TO_CCR : O_ILL;
            return OK(EA_DALT) ? O_NEG : O_ILL;
        case 0x6:
            if (sz2 == 3) return OK(EA_DATA) ? O_MOVE_TO_SR : O_ILL;
            return OK(EA_DALT) ? O_NOT : O_ILL;
        case 0x8:
            if (sz2 == 0) return OK(EA_DALT) ? O_NBCD : O_ILL;
            if (sz2 == 1) return mode == 0 ? O_SWAP : (OK(EA_CTRL) ? O_PEA : O_ILL);
            if (mode == 0) return O_EXT;
            return OK(EA_CALT | M_PD) ? O_MOVEM_RM : O_ILL;
        case 0xA:
            if (op == 0x4AFC) return O_ILL;
            if (sz2 == 3) return OK(EA_DALT) ? O_TAS : O_ILL;
            return OK(EA_DALT) ? O_TST : O_ILL;
        case 0xC:
            if (sz2 >= 2) return OK(EA_CTRL | M_PI) ? O_MOVEM_MR : O_ILL;
            return O_ILL;
        case 0xE:
            if ((op & 0xF0) == 0x40) return O_TRAP;
            if ((op & 0xF8) == 0x50) return O_LINK;
            if ((op & 0xF8) == 0x58) return O_UNLK;
            if ((op & 0xF0) == 0x60) return O_MOVE_USP;
            switch (op) {
            case 0x4E70: return O_RESET;
            case 0x4E71: return O_NOP;
            case 0x4E72: return O_STOP;
            case 0x4E73: return O_RTE;
            case 0x4E75: return O_RTS;
            case 0x4E76: return O_TRAPV;
            case 0x4E77: return O_RTR;
            default: break;
            }
            if ((op & 0xC0) == 0x80) return OK(EA_CTRL) ? O_JSR : O_ILL;
            if ((op & 0xC0) == 0xC0) return OK(EA_CTRL) ? O_JMP : O_ILL;
            return O_ILL;
        default: return O_ILL;
        }
    case 0x5:
        if (sz2 == 3) {
            if (mode == 1) return O_DBCC;
            return OK(EA_DALT) ? O_SCC : O_ILL;
        }
        if (mode == 1 && sz2 == 0) return O_ILL;
        return OK(EA_ALT) ? O_ADDQ : O_ILL;
    case 0x6: return O_BCC;
    case 0x7: return (op & 0x100) ? O_ILL : O_MOVEQ;
    case 0x8:
        if (sz2 == 3) return OK(EA_DATA) ? ((op & 0x100) ? O_DIVS : O_DIVU) : O_ILL;
        if ((op & 0x1F0) == 0x100) return O_SBCD;
        if (op & 0x100) return OK(EA_MALT) ? O_OR : O_ILL;
        return OK(EA_DATA) ? O_OR : O_ILL;
    case 0x9: case 0xD: {
        int add = (op >> 12) == 0xD;
        if (sz2 == 3) return OK(EA_ALL) ? (add ? O_ADDA : O_SUBA) : O_ILL;
        if ((op & 0x130) == 0x100) return add ? O_ADDX : O_SUBX;
        if (op & 0x100) return OK(EA_MALT) ? (add ? O_ADD : O_SUB) : O_ILL;
        if (sz2 == 0 && mode == 1) return O_ILL;
        return OK(EA_ALL) ? (add ? O_ADD : O_SUB) : O_ILL;
    }
    case 0xB:
        if (sz2 == 3) return OK(EA_ALL) ? O_CMPA : O_ILL;
        if (op & 0x100) {
            if (mode == 1) return O_CMPM;
            return OK(EA_DALT) ? O_EOR : O_ILL;
        }
        if (sz2 == 0 && mode == 1) return O_ILL;
        return OK(EA_ALL) ? O_CMP : O_ILL;
    case 0xC:
        if (sz2 == 3) return OK(EA_DATA) ? ((op & 0x100) ? O_MULS : O_MULU) : O_ILL;
        if ((op & 0x1F0) == 0x100) return O_ABCD;
        if ((op & 0x1F8) == 0x140 || (op & 0x1F8) == 0x148 || (op & 0x1F8) == 0x188) return O_EXG;
        if (op & 0x100) return OK(EA_MALT) ? O_AND : O_ILL;
        return OK(EA_DATA) ? O_AND : O_ILL;
    case 0xE:
        if (sz2 == 3) {
            if (op & 0x800) return O_ILL;
            return OK(EA_MALT) ? O_SHIFT_M : O_ILL;
        }
        return O_SHIFT_R;
    case 0xA: return O_LINEA;
    default: return O_LINEF;
    }
#undef OK
}

/* ---- bus helpers -------------------------------------------------------- */
static inline uint32_t szmask(int sz) { return sz == 1 ? 0xFFu : (sz == 2 ? 0xFFFFu : 0xFFFFFFFFu); }
static inline uint32_t szmsb(int sz)  { return sz == 1 ? 0x80u : (sz == 2 ? 0x8000u : 0x80000000u); }
static inline uint32_t sx8(uint32_t v)  { return (uint32_t)(int32_t)(int8_t)(uint8_t)v; }
static inline uint32_t sx16(uint32_t v) { return (uint32_t)(int32_t)(int16_t)(uint16_t)v; }

static inline uint8_t rd8(m68k_t *c, uint32_t a)
{
    c->cyc += 4;
    return c->bus.read8(c->bus.ctx, a & 0xFFFFFFu);
}
static inline uint16_t rd16(m68k_t *c, uint32_t a)
{
    c->cyc += 4;
    return c->bus.read16(c->bus.ctx, a & 0xFFFFFEu);
}
static inline uint32_t rd32(m68k_t *c, uint32_t a)
{
    uint32_t h = rd16(c, a);
    return (h << 16) | rd16(c, a + 2);
}
static inline void wr8(m68k_t *c, uint32_t a, uint32_t v)
{
    c->cyc += 4;
    c->bus.write8(c->bus.ctx, a & 0xFFFFFFu, (uint8_t)v);
}
static inline void wr16(m68k_t *c, uint32_t a, uint32_t v)
{
    c->cyc += 4;
    c->bus.write16(c->bus.ctx, a & 0xFFFFFEu, (uint16_t)v);
}
static inline void wr32(m68k_t *c, uint32_t a, uint32_t v)
{
    wr16(c, a, v >> 16);
    wr16(c, a + 2, v & 0xFFFFu);
}
static inline uint32_t rdsz(m68k_t *c, uint32_t a, int sz)
{
    return sz == 1 ? rd8(c, a) : (sz == 2 ? rd16(c, a) : rd32(c, a));
}
static inline void wrsz(m68k_t *c, uint32_t a, int sz, uint32_t v)
{
    if (sz == 1) wr8(c, a, v);
    else if (sz == 2) wr16(c, a, v);
    else wr32(c, a, v);
}
static inline uint16_t fetch16(m68k_t *c)
{
    uint16_t v = rd16(c, c->pc);
    c->pc += 2;
    return v;
}
static inline uint32_t fetch32(m68k_t *c)
{
    uint32_t h = fetch16(c);
    return (h << 16) | fetch16(c);
}
static inline void push16(m68k_t *c, uint32_t v) { c->a[7] -= 2; wr16(c, c->a[7], v); }
static inline void push32(m68k_t *c, uint32_t v) { c->a[7] -= 4; wr32(c, c->a[7], v); }
static inline uint32_t pop16(m68k_t *c) { uint32_t v = rd16(c, c->a[7]); c->a[7] += 2; return v; }
static inline uint32_t pop32(m68k_t *c) { uint32_t v = rd32(c, c->a[7]); c->a[7] += 4; return v; }

static inline void setd(m68k_t *c, int r, uint32_t v, int sz)
{
    uint32_t m = szmask(sz);
    c->d[r] = (c->d[r] & ~m) | (v & m);
}

/* ---- flags -------------------------------------------------------------- */
static inline void setf(m68k_t *c, uint16_t f, int on)
{
    if (on) c->sr = (uint16_t)(c->sr | f);
    else c->sr = (uint16_t)(c->sr & ~f);
}
static inline void nz(m68k_t *c, uint32_t r, int sz)
{
    r &= szmask(sz);
    setf(c, ZF, r == 0);
    setf(c, NF, (r & szmsb(sz)) != 0);
}
static inline void logic(m68k_t *c, uint32_t r, int sz)
{
    nz(c, r, sz);
    c->sr = (uint16_t)(c->sr & ~(VF | CF));
}

static void set_sr(m68k_t *c, uint32_t v)
{
    uint16_t nv = (uint16_t)(v & 0xA71Fu);
    int olds = (c->sr & M68K_SR_S) != 0, news = (nv & M68K_SR_S) != 0;
    if (olds && !news) { c->ssp = c->a[7]; c->a[7] = c->usp; }
    else if (!olds && news) { c->usp = c->a[7]; c->a[7] = c->ssp; }
    c->sr = nv;
}

static void exception(m68k_t *c, uint32_t vec, uint32_t ret_pc)
{
    uint16_t old = c->sr;
    set_sr(c, (uint32_t)((old | M68K_SR_S) & ~M68K_SR_T));
    push32(c, ret_pc);
    push16(c, old);
    c->pc = rd32(c, vec * 4u);
    c->cyc += 14;
}

static int cond(const m68k_t *c, int cc)
{
    int C = (c->sr & CF) != 0, V = (c->sr & VF) != 0;
    int Z = (c->sr & ZF) != 0, N = (c->sr & NF) != 0;
    switch (cc & 15) {
    case 0:  return 1;
    case 1:  return 0;
    case 2:  return !C && !Z;
    case 3:  return C || Z;
    case 4:  return !C;
    case 5:  return C;
    case 6:  return !Z;
    case 7:  return Z;
    case 8:  return !V;
    case 9:  return V;
    case 10: return !N;
    case 11: return N;
    case 12: return N == V;
    case 13: return N != V;
    case 14: return !Z && (N == V);
    default: return Z || (N != V);
    }
}

/* ---- effective addresses ------------------------------------------------ */
enum { L_D = 0, L_A, L_M, L_I };
typedef struct { uint8_t kind; uint8_t reg; uint32_t addr; } loc_t;

static uint32_t ea_index(m68k_t *c, uint32_t base)
{
    uint16_t ext = fetch16(c);
    uint32_t idx = (ext & 0x8000u) ? c->a[(ext >> 12) & 7] : c->d[(ext >> 12) & 7];
    if (!(ext & 0x800u)) idx = sx16(idx);
    c->cyc += 2;
    return base + sx8(ext & 0xFFu) + idx;
}

static loc_t ea(m68k_t *c, int mode, int reg, int sz)
{
    loc_t l;
    uint32_t inc, b;
    l.kind = L_M; l.reg = (uint8_t)reg; l.addr = 0;
    switch (mode) {
    case 0: l.kind = L_D; break;
    case 1: l.kind = L_A; break;
    case 2: l.addr = c->a[reg]; break;
    case 3:
        inc = (sz == 1 && reg == 7) ? 2u : (uint32_t)sz;
        l.addr = c->a[reg]; c->a[reg] += inc;
        break;
    case 4:
        inc = (sz == 1 && reg == 7) ? 2u : (uint32_t)sz;
        c->a[reg] -= inc; l.addr = c->a[reg]; c->cyc += 2;
        break;
    case 5: l.addr = c->a[reg] + sx16(fetch16(c)); break;
    case 6: l.addr = ea_index(c, c->a[reg]); break;
    default:
        switch (reg) {
        case 0: l.addr = sx16(fetch16(c)); break;
        case 1: l.addr = fetch32(c); break;
        case 2: b = c->pc; l.addr = b + sx16(fetch16(c)); break;
        case 3: b = c->pc; l.addr = ea_index(c, b); break;
        default:
            l.kind = L_I;
            if (sz == 4) l.addr = fetch32(c);
            else if (sz == 2) l.addr = fetch16(c);
            else l.addr = fetch16(c) & 0xFFu;
            break;
        }
        break;
    }
    return l;
}

/* Control-mode address without (An)+ / -(An) side effects (MOVEM, LEA...). */
static uint32_t ea_ctrl(m68k_t *c, int mode, int reg)
{
    loc_t l = ea(c, mode == 3 || mode == 4 ? 2 : mode, reg, 4);
    return l.addr;
}

static uint32_t lrd(m68k_t *c, const loc_t *l, int sz)
{
    switch (l->kind) {
    case L_D: return c->d[l->reg] & szmask(sz);
    case L_A: return c->a[l->reg] & szmask(sz);
    case L_I: return l->addr & szmask(sz);
    default:  return rdsz(c, l->addr, sz);
    }
}

static void lwr(m68k_t *c, const loc_t *l, int sz, uint32_t v)
{
    switch (l->kind) {
    case L_D: setd(c, l->reg, v, sz); break;
    case L_A: c->a[l->reg] = v; break;
    case L_M: wrsz(c, l->addr, sz, v); break;
    default: break;
    }
}

/* ---- arithmetic cores ------------------------------------------------- */
static uint32_t do_add(m68k_t *c, uint32_t s, uint32_t d, int sz, uint32_t x, int extended)
{
    uint32_t m = szmask(sz), msb = szmsb(sz);
    uint64_t full = (uint64_t)(s & m) + (uint64_t)(d & m) + x;
    uint32_t r = (uint32_t)full & m;
    int carry = full > m;
    setf(c, CF, carry);
    setf(c, XF, carry);
    setf(c, VF, ((s ^ r) & (d ^ r) & msb) != 0);
    setf(c, NF, (r & msb) != 0);
    if (extended) { if (r) c->sr = (uint16_t)(c->sr & ~ZF); }
    else setf(c, ZF, r == 0);
    return r;
}

/* d - s - x. setx=0 for CMP (X untouched). */
static uint32_t do_sub(m68k_t *c, uint32_t s, uint32_t d, int sz, uint32_t x, int extended, int setx)
{
    uint32_t m = szmask(sz), msb = szmsb(sz);
    uint32_t r = (d - s - x) & m;
    int borrow = ((uint64_t)(s & m) + x) > (uint64_t)(d & m);
    setf(c, CF, borrow);
    if (setx) setf(c, XF, borrow);
    setf(c, VF, ((s ^ d) & (r ^ d) & msb) != 0);
    setf(c, NF, (r & msb) != 0);
    if (extended) { if (r) c->sr = (uint16_t)(c->sr & ~ZF); }
    else setf(c, ZF, r == 0);
    return r;
}

static uint32_t do_abcd(m68k_t *c, uint32_t s, uint32_t d)
{
    uint32_t x = (c->sr & XF) ? 1u : 0u;
    uint32_t r = (s & 0xF) + (d & 0xF) + x;
    uint32_t bin = (s & 0xFF) + (d & 0xFF) + x;
    int carry;
    if (r > 9) r += 6;
    r += (s & 0xF0) + (d & 0xF0);
    carry = r > 0x99;
    if (carry) r -= 0xA0;
    r &= 0xFF;
    setf(c, CF, carry);
    setf(c, XF, carry);
    setf(c, VF, (~bin & r & 0x80) != 0);
    setf(c, NF, (r & 0x80) != 0);
    if (r) c->sr = (uint16_t)(c->sr & ~ZF);
    return r;
}

static uint32_t do_sbcd(m68k_t *c, uint32_t s, uint32_t d)
{
    uint32_t x = (c->sr & XF) ? 1u : 0u;
    uint32_t r = (d & 0xF) - (s & 0xF) - x;
    uint32_t bin = (d & 0xFF) - (s & 0xFF) - x;
    int carry;
    if (r > 9) r -= 6;
    r += (d & 0xF0) - (s & 0xF0);
    carry = r > 0x99;
    if (carry) r += 0xA0;
    r &= 0xFF;
    setf(c, CF, carry);
    setf(c, XF, carry);
    setf(c, VF, (bin & ~r & 0x80) != 0);
    setf(c, NF, (r & 0x80) != 0);
    if (r) c->sr = (uint16_t)(c->sr & ~ZF);
    return r;
}

/* type: 0 AS, 1 LS, 2 ROX, 3 RO */
static uint32_t do_shift(m68k_t *c, int type, int left, uint32_t v, uint32_t cnt, int sz)
{
    uint32_t m = szmask(sz), msb = szmsb(sz);
    uint32_t x = (c->sr & XF) ? 1u : 0u, carry = 0, i;
    int vflag = 0;
    v &= m;
    c->cyc += 2 + 2 * cnt;
    if (cnt == 0) {
        setf(c, CF, type == 2 ? (int)x : 0);
        setf(c, VF, 0);
        nz(c, v, sz);
        return v;
    }
    for (i = 0; i < cnt && i < 64; i++) {
        uint32_t nv;
        if (left) {
            carry = (v & msb) ? 1u : 0u;
            switch (type) {
            case 0: nv = (v << 1) & m; if ((nv ^ v) & msb) vflag = 1; break;
            case 1: nv = (v << 1) & m; break;
            case 2: nv = ((v << 1) | x) & m; x = carry; break;
            default: nv = ((v << 1) | carry) & m; break;
            }
        } else {
            carry = v & 1u;
            switch (type) {
            case 0: nv = (v >> 1) | (v & msb); break;
            case 1: nv = v >> 1; break;
            case 2: nv = (v >> 1) | (x ? msb : 0u); x = carry; break;
            default: nv = (v >> 1) | (carry ? msb : 0u); break;
            }
        }
        v = nv;
    }
    setf(c, CF, carry != 0);
    if (type != 3) setf(c, XF, carry != 0);
    setf(c, VF, type == 0 ? vflag : 0);
    nz(c, v, sz);
    return v;
}

static int priv_ok(m68k_t *c)
{
    if (c->sr & M68K_SR_S) return 1;
    exception(c, 8, c->ir_pc);
    return 0;
}

static int sz_std(uint16_t op)
{
    int s = (op >> 6) & 3;
    return s == 0 ? 1 : (s == 1 ? 2 : 4);
}

/* ---- execute one instruction ------------------------------------------- */
static void execute(m68k_t *c, uint16_t op)
{
    int mode = (op >> 3) & 7, reg = op & 7, rx = (op >> 9) & 7;
    int sz;
    loc_t l;
    uint32_t s, d, r;

    switch (c->optab[op]) {
    case O_ILL:   exception(c, 4, c->ir_pc); return;
    case O_LINEA: exception(c, 10, c->ir_pc); return;
    case O_LINEF: exception(c, 11, c->ir_pc); return;

    case O_ORI_CCR:  s = fetch16(c); c->sr = (uint16_t)(c->sr | (s & 0x1F)); return;
    case O_ANDI_CCR: s = fetch16(c); c->sr = (uint16_t)(c->sr & (0xFF00u | (s & 0x1F))); return;
    case O_EORI_CCR: s = fetch16(c); c->sr = (uint16_t)(c->sr ^ (s & 0x1F)); return;
    case O_ORI_SR:  if (!priv_ok(c)) return; s = fetch16(c); set_sr(c, c->sr | s); return;
    case O_ANDI_SR: if (!priv_ok(c)) return; s = fetch16(c); set_sr(c, c->sr & s); return;
    case O_EORI_SR: if (!priv_ok(c)) return; s = fetch16(c); set_sr(c, c->sr ^ s); return;

    case O_ORI: case O_ANDI: case O_SUBI: case O_ADDI: case O_EORI: case O_CMPI: {
        uint8_t cls = c->optab[op];
        sz = sz_std(op);
        s = sz == 4 ? fetch32(c) : (fetch16(c) & szmask(sz));
        l = ea(c, mode, reg, sz);
        d = lrd(c, &l, sz);
        switch (cls) {
        case O_ORI:  r = d | s; logic(c, r, sz); break;
        case O_ANDI: r = d & s; logic(c, r, sz); break;
        case O_EORI: r = d ^ s; logic(c, r, sz); break;
        case O_SUBI: r = do_sub(c, s, d, sz, 0, 0, 1); break;
        case O_ADDI: r = do_add(c, s, d, sz, 0, 0); break;
        default: do_sub(c, s, d, sz, 0, 0, 0); return;
        }
        lwr(c, &l, sz, r);
        return;
    }

    case O_BIT_R: case O_BIT_I: {
        uint32_t bit = c->optab[op] == O_BIT_R ? c->d[rx] : fetch16(c);
        int type = (op >> 6) & 3;
        sz = mode == 0 ? 4 : 1;
        bit &= (sz == 4) ? 31u : 7u;
        l = ea(c, mode, reg, sz);
        d = lrd(c, &l, sz);
        setf(c, ZF, ((d >> bit) & 1u) == 0);
        if (type == 0) return;
        if (type == 1) d ^= (1u << bit);
        else if (type == 2) d &= ~(1u << bit);
        else d |= (1u << bit);
        lwr(c, &l, sz, d);
        c->cyc += 2;
        return;
    }

    case O_MOVEP: {
        uint32_t a = c->a[reg] + sx16(fetch16(c));
        int opm = (op >> 6) & 7;
        if (opm == 4) {
            r = ((uint32_t)rd8(c, a) << 8) | rd8(c, a + 2);
            setd(c, rx, r, 2);
        } else if (opm == 5) {
            r = ((uint32_t)rd8(c, a) << 24) | ((uint32_t)rd8(c, a + 2) << 16) |
                ((uint32_t)rd8(c, a + 4) << 8) | rd8(c, a + 6);
            c->d[rx] = r;
        } else if (opm == 6) {
            wr8(c, a, c->d[rx] >> 8); wr8(c, a + 2, c->d[rx]);
        } else {
            wr8(c, a, c->d[rx] >> 24); wr8(c, a + 2, c->d[rx] >> 16);
            wr8(c, a + 4, c->d[rx] >> 8); wr8(c, a + 6, c->d[rx]);
        }
        return;
    }

    case O_MOVE: case O_MOVEA: {
        int top = op >> 12;
        sz = top == 1 ? 1 : (top == 3 ? 2 : 4);
        l = ea(c, mode, reg, sz);
        s = lrd(c, &l, sz);
        if (c->optab[op] == O_MOVEA) {
            c->a[rx] = sz == 2 ? sx16(s) : s;
            return;
        }
        l = ea(c, (op >> 6) & 7, rx, sz);
        lwr(c, &l, sz, s);
        logic(c, s, sz);
        return;
    }

    case O_NEGX: sz = sz_std(op); l = ea(c, mode, reg, sz); d = lrd(c, &l, sz);
        r = do_sub(c, d, 0, sz, (c->sr & XF) ? 1u : 0u, 1, 1); lwr(c, &l, sz, r); return;
    case O_NEG: sz = sz_std(op); l = ea(c, mode, reg, sz); d = lrd(c, &l, sz);
        r = do_sub(c, d, 0, sz, 0, 0, 1); lwr(c, &l, sz, r); return;
    case O_NOT: sz = sz_std(op); l = ea(c, mode, reg, sz); d = lrd(c, &l, sz);
        r = ~d & szmask(sz); logic(c, r, sz); lwr(c, &l, sz, r); return;
    case O_CLR: sz = sz_std(op); l = ea(c, mode, reg, sz);
        lwr(c, &l, sz, 0); logic(c, 0, sz); return;
    case O_TST: sz = sz_std(op); l = ea(c, mode, reg, sz); logic(c, lrd(c, &l, sz), sz); return;
    case O_TAS: l = ea(c, mode, reg, 1); d = lrd(c, &l, 1); logic(c, d, 1);
        lwr(c, &l, 1, d | 0x80u); return;
    case O_NBCD: l = ea(c, mode, reg, 1); d = lrd(c, &l, 1);
        r = do_sbcd(c, d, 0); lwr(c, &l, 1, r); return;

    case O_MOVE_FR_SR: l = ea(c, mode, reg, 2); lwr(c, &l, 2, c->sr); return;
    case O_MOVE_TO_CCR: l = ea(c, mode, reg, 2); s = lrd(c, &l, 2);
        c->sr = (uint16_t)((c->sr & 0xFF00u) | (s & 0x1Fu)); return;
    case O_MOVE_TO_SR: if (!priv_ok(c)) return; l = ea(c, mode, reg, 2);
        set_sr(c, lrd(c, &l, 2)); return;

    case O_CHK: {
        int32_t v, bound;
        l = ea(c, mode, reg, 2);
        bound = (int16_t)(uint16_t)lrd(c, &l, 2);
        v = (int16_t)(uint16_t)c->d[rx];
        if (v < 0) { setf(c, NF, 1); exception(c, 6, c->pc); }
        else if (v > bound) { setf(c, NF, 0); exception(c, 6, c->pc); }
        return;
    }
    case O_LEA: c->a[rx] = ea_ctrl(c, mode, reg); return;
    case O_PEA: s = ea_ctrl(c, mode, reg); push32(c, s); return;
    case O_SWAP: r = (c->d[reg] >> 16) | (c->d[reg] << 16); c->d[reg] = r; logic(c, r, 4); return;
    case O_EXT:
        if (((op >> 6) & 7) == 2) { r = sx8(c->d[reg]); setd(c, reg, r, 2); logic(c, r, 2); }
        else { r = sx16(c->d[reg]); c->d[reg] = r; logic(c, r, 4); }
        return;

    case O_MOVEM_RM: {
        uint16_t mask = fetch16(c);
        int i;
        sz = (op & 0x40) ? 4 : 2;
        if (mode == 4) {
            uint32_t a = c->a[reg];
            for (i = 0; i < 16; i++) {
                if (mask & (1u << i)) {
                    int ri = 15 - i;
                    uint32_t v = ri < 8 ? c->d[ri] : c->a[ri - 8];
                    a -= (uint32_t)sz;
                    wrsz(c, a, sz, v);
                }
            }
            c->a[reg] = a;
        } else {
            uint32_t a = ea_ctrl(c, mode, reg);
            for (i = 0; i < 16; i++) {
                if (mask & (1u << i)) {
                    uint32_t v = i < 8 ? c->d[i] : c->a[i - 8];
                    wrsz(c, a, sz, v);
                    a += (uint32_t)sz;
                }
            }
        }
        return;
    }
    case O_MOVEM_MR: {
        uint16_t mask = fetch16(c);
        uint32_t a;
        int i;
        sz = (op & 0x40) ? 4 : 2;
        a = (mode == 3) ? c->a[reg] : ea_ctrl(c, mode, reg);
        for (i = 0; i < 16; i++) {
            if (mask & (1u << i)) {
                uint32_t v = rdsz(c, a, sz);
                if (sz == 2) v = sx16(v);
                if (i < 8) c->d[i] = v; else c->a[i - 8] = v;
                a += (uint32_t)sz;
            }
        }
        if (mode == 3) c->a[reg] = a;
        return;
    }

    case O_TRAP: exception(c, 32u + (op & 15u), c->pc); return;
    case O_TRAPV: if (c->sr & VF) exception(c, 7, c->pc); return;
    case O_LINK: push32(c, c->a[reg]); c->a[reg] = c->a[7];
        c->a[7] += sx16(fetch16(c)); return;
    case O_UNLK: c->a[7] = c->a[reg]; c->a[reg] = pop32(c); return;
    case O_MOVE_USP: if (!priv_ok(c)) return;
        if (op & 8) c->a[reg] = c->usp; else c->usp = c->a[reg];
        return;
    case O_RESET: if (!priv_ok(c)) return; c->cyc += 128; return;
    case O_NOP: return;
    case O_STOP: if (!priv_ok(c)) return; s = fetch16(c); set_sr(c, s); c->stopped = 1; return;
    case O_RTE: {
        uint32_t nsr;
        if (!priv_ok(c)) return;
        nsr = pop16(c);
        c->pc = pop32(c);
        set_sr(c, nsr);
        return;
    }
    case O_RTS: c->pc = pop32(c); return;
    case O_RTR: s = pop16(c); c->sr = (uint16_t)((c->sr & 0xFF00u) | (s & 0x1Fu));
        c->pc = pop32(c); return;
    case O_JSR: s = ea_ctrl(c, mode, reg); push32(c, c->pc); c->pc = s; return;
    case O_JMP: c->pc = ea_ctrl(c, mode, reg); return;

    case O_ADDQ: {
        uint32_t q = (uint32_t)rx; if (q == 0) q = 8;
        sz = sz_std(op);
        if (mode == 1) {
            if (op & 0x100) c->a[reg] -= q; else c->a[reg] += q;
            return;
        }
        l = ea(c, mode, reg, sz);
        d = lrd(c, &l, sz);
        r = (op & 0x100) ? do_sub(c, q, d, sz, 0, 0, 1) : do_add(c, q, d, sz, 0, 0);
        lwr(c, &l, sz, r);
        return;
    }
    case O_SCC: l = ea(c, mode, reg, 1); lwr(c, &l, 1, cond(c, (op >> 8) & 15) ? 0xFFu : 0u); return;
    case O_DBCC: {
        uint32_t base = c->pc;
        uint32_t disp = sx16(fetch16(c));
        if (!cond(c, (op >> 8) & 15)) {
            uint32_t cnt = (c->d[reg] - 1u) & 0xFFFFu;
            setd(c, reg, cnt, 2);
            if (cnt != 0xFFFFu) c->pc = base + disp;
            c->cyc += 2;
        }
        return;
    }
    case O_BCC: {
        uint32_t base = c->pc;
        uint32_t disp = sx8(op & 0xFFu);
        int cc = (op >> 8) & 15;
        if ((op & 0xFF) == 0) disp = sx16(fetch16(c));
        if (cc == 1) { push32(c, c->pc); c->pc = base + disp; return; }
        if (cond(c, cc)) { c->pc = base + disp; c->cyc += 2; }
        return;
    }
    case O_MOVEQ: r = sx8(op & 0xFFu); c->d[rx] = r; logic(c, r, 4); return;

    case O_OR: case O_AND: case O_EOR: case O_ADD: case O_SUB: case O_CMP: {
        uint8_t cls = c->optab[op];
        sz = sz_std(op);
        l = ea(c, mode, reg, sz);
        if ((op & 0x100) && cls != O_CMP) {         /* Dn op <ea> -> <ea> */
            d = lrd(c, &l, sz);
            s = c->d[rx];
            switch (cls) {
            case O_OR:  r = d | s; logic(c, r, sz); break;
            case O_AND: r = d & s; logic(c, r, sz); break;
            case O_EOR: r = d ^ s; logic(c, r, sz); break;
            case O_ADD: r = do_add(c, s, d, sz, 0, 0); break;
            default:    r = do_sub(c, s, d, sz, 0, 0, 1); break;
            }
            lwr(c, &l, sz, r);
        } else {                                       /* <ea> op Dn -> Dn */
            s = lrd(c, &l, sz);
            d = c->d[rx];
            switch (cls) {
            case O_OR:  r = d | s; logic(c, r, sz); break;
            case O_AND: r = d & s; logic(c, r, sz); break;
            case O_ADD: r = do_add(c, s, d, sz, 0, 0); break;
            case O_SUB: r = do_sub(c, s, d, sz, 0, 0, 1); break;
            default:    do_sub(c, s, d, sz, 0, 0, 0); return;
            }
            setd(c, rx, r, sz);
        }
        return;
    }
    case O_ADDA: case O_SUBA: case O_CMPA: {
        sz = (op & 0x100) ? 4 : 2;
        l = ea(c, mode, reg, sz);
        s = lrd(c, &l, sz);
        if (sz == 2) s = sx16(s);
        if (c->optab[op] == O_ADDA) c->a[rx] += s;
        else if (c->optab[op] == O_SUBA) c->a[rx] -= s;
        else do_sub(c, s, c->a[rx], 4, 0, 0, 0);
        c->cyc += 2;
        return;
    }
    case O_ADDX: case O_SUBX: {
        uint32_t x = (c->sr & XF) ? 1u : 0u;
        sz = sz_std(op);
        if (op & 8) {
            loc_t ls = ea(c, 4, reg, sz);
            loc_t ld;
            s = lrd(c, &ls, sz);
            ld = ea(c, 4, rx, sz);
            d = lrd(c, &ld, sz);
            r = c->optab[op] == O_ADDX ? do_add(c, s, d, sz, x, 1) : do_sub(c, s, d, sz, x, 1, 1);
            lwr(c, &ld, sz, r);
        } else {
            s = c->d[reg]; d = c->d[rx];
            r = c->optab[op] == O_ADDX ? do_add(c, s, d, sz, x, 1) : do_sub(c, s, d, sz, x, 1, 1);
            setd(c, rx, r, sz);
        }
        return;
    }
    case O_ABCD: case O_SBCD: {
        int add = c->optab[op] == O_ABCD;
        if (op & 8) {
            loc_t ls = ea(c, 4, reg, 1);
            loc_t ld;
            s = lrd(c, &ls, 1);
            ld = ea(c, 4, rx, 1);
            d = lrd(c, &ld, 1);
            r = add ? do_abcd(c, s, d) : do_sbcd(c, s, d);
            lwr(c, &ld, 1, r);
        } else {
            s = c->d[reg]; d = c->d[rx];
            r = add ? do_abcd(c, s, d) : do_sbcd(c, s, d);
            setd(c, rx, r, 1);
        }
        return;
    }
    case O_CMPM: {
        loc_t ls, ld;
        sz = sz_std(op);
        ls = ea(c, 3, reg, sz);
        s = lrd(c, &ls, sz);
        ld = ea(c, 3, rx, sz);
        d = lrd(c, &ld, sz);
        do_sub(c, s, d, sz, 0, 0, 0);
        return;
    }
    case O_MULU: case O_MULS:
        l = ea(c, mode, reg, 2);
        s = lrd(c, &l, 2);
        if (c->optab[op] == O_MULU) r = (c->d[rx] & 0xFFFFu) * s;
        else r = (uint32_t)((int32_t)(int16_t)(uint16_t)c->d[rx] * (int32_t)(int16_t)(uint16_t)s);
        c->d[rx] = r;
        logic(c, r, 4);
        c->cyc += 34;
        return;
    case O_DIVU: {
        uint32_t q, rem;
        l = ea(c, mode, reg, 2);
        s = lrd(c, &l, 2);
        if (s == 0) { setf(c, CF, 0); exception(c, 5, c->pc); return; }
        q = c->d[rx] / s; rem = c->d[rx] % s;
        c->cyc += 136;
        if (q > 0xFFFFu) { setf(c, VF, 1); setf(c, NF, 1); setf(c, ZF, 0); setf(c, CF, 0); return; }
        c->d[rx] = (rem << 16) | q;
        logic(c, q, 2);
        return;
    }
    case O_DIVS: {
        int64_t dvd, dvs, q, rem;
        l = ea(c, mode, reg, 2);
        s = lrd(c, &l, 2);
        if (s == 0) { setf(c, CF, 0); exception(c, 5, c->pc); return; }
        dvd = (int32_t)c->d[rx];
        dvs = (int16_t)(uint16_t)s;
        q = dvd / dvs; rem = dvd % dvs;
        c->cyc += 154;
        if (q < -32768 || q > 32767) { setf(c, VF, 1); setf(c, NF, 1); setf(c, ZF, 0); setf(c, CF, 0); return; }
        c->d[rx] = (((uint32_t)rem & 0xFFFFu) << 16) | ((uint32_t)q & 0xFFFFu);
        logic(c, (uint32_t)q, 2);
        return;
    }
    case O_EXG: {
        uint32_t t;
        int k = op & 0x1F8;
        if (k == 0x140) { t = c->d[rx]; c->d[rx] = c->d[reg]; c->d[reg] = t; }
        else if (k == 0x148) { t = c->a[rx]; c->a[rx] = c->a[reg]; c->a[reg] = t; }
        else { t = c->d[rx]; c->d[rx] = c->a[reg]; c->a[reg] = t; }
        c->cyc += 2;
        return;
    }
    case O_SHIFT_R: {
        uint32_t cnt = (op & 0x20) ? (c->d[rx] & 63u) : (rx ? (uint32_t)rx : 8u);
        sz = sz_std(op);
        r = do_shift(c, (op >> 3) & 3, (op >> 8) & 1, c->d[reg], cnt, sz);
        setd(c, reg, r, sz);
        return;
    }
    case O_SHIFT_M:
        l = ea(c, mode, reg, 2);
        d = lrd(c, &l, 2);
        r = do_shift(c, (op >> 9) & 3, (op >> 8) & 1, d, 1, 2);
        lwr(c, &l, 2, r);
        return;
    default:
        exception(c, 4, c->ir_pc);
        return;
    }
}

/* ---- public API --------------------------------------------------------- */
void m68k_init(m68k_t *c, const m68k_bus_t *bus)
{
    uint32_t op;
    memset(c, 0, sizeof(*c));
    c->bus = *bus;
    for (op = 0; op < 65536u; op++) c->optab[op] = decode((uint16_t)op);
}

uint8_t m68k_op_class(const m68k_t *c, uint16_t op) { return c->optab[op]; }

void m68k_reset(m68k_t *c)
{
    int i;
    for (i = 0; i < 8; i++) { c->d[i] = 0; c->a[i] = 0; }
    c->sr = 0x2700;
    c->usp = 0;
    c->stopped = 0;
    c->a[7] = rd32(c, 0);
    c->ssp = c->a[7];
    c->pc = rd32(c, 4);
}

uint32_t m68k_run(m68k_t *c, uint32_t cycles)
{
    uint64_t start = c->cyc, target = c->cyc + cycles;
    while (c->cyc < target) {
        int lvl = c->irq_level & 7;
        if (lvl && (lvl == 7 || lvl > (int)((c->sr >> 8) & 7))) {
            uint16_t old = c->sr;
            uint32_t vec;
            c->stopped = 0;
            set_sr(c, (uint32_t)((old | M68K_SR_S) & ~M68K_SR_T));
            c->sr = (uint16_t)((c->sr & ~0x0700u) | ((uint32_t)lvl << 8));
            push32(c, c->pc);
            push16(c, old);
            vec = c->bus.iack(c->bus.ctx, lvl);
            c->pc = rd32(c, vec * 4u);
            c->cyc += 22;
            continue;
        }
        if (c->stopped) { c->cyc = target; break; }
        c->ir_pc = c->pc;
        execute(c, fetch16(c));
        c->insns++;
    }
    return (uint32_t)(c->cyc - start);
}
