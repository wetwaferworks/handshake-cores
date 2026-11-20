/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_cpu.c — clean-room WDC 65C816 interpreter for hs-snes.
 *
 * Written from the public WDC W65C816S datasheet behaviour description:
 * all 256 opcodes, 6502 emulation mode (E=1) and native mode (E=0) with
 * dynamic 8/16-bit accumulator (M) and index (X) widths, all 24 addressing
 * modes, BCD ADC/SBC in 8 and 16 bit, MVN/MVP block moves, WAI/STP,
 * NMI/IRQ/BRK/COP/ABORT vectors.  Cycle accounting: every bus access is
 * charged by the bus at the speed of the addressed region (6/8/12 master
 * cycles) and every datasheet "IO" cycle is charged as 6 master cycles.
 */
#include "snes_internal.h"

#define DPW 0x80000000u   /* effective-address flag: 16-bit wrap in bank 0 */

static inline uint8_t rd(hs_snes *s, uint32_t a) { return snes_cpu_read(s, a & 0xFFFFFFu); }
static inline void wr(hs_snes *s, uint32_t a, uint8_t v) { snes_cpu_write(s, a & 0xFFFFFFu, v); }
static inline void io(hs_snes *s) { snes_cpu_idle(s); }

static inline uint8_t rd8(hs_snes *s, uint32_t ea) { return rd(s, ea & 0xFFFFFFu); }
static inline uint16_t rd16(hs_snes *s, uint32_t ea)
{
    uint8_t lo, hi;
    if (ea & DPW) {
        uint16_t a = (uint16_t)ea;
        lo = rd(s, a);
        hi = rd(s, (uint16_t)(a + 1u));
    } else {
        lo = rd(s, ea);
        hi = rd(s, ea + 1u);
    }
    return (uint16_t)(lo | (hi << 8));
}
static inline void wr8(hs_snes *s, uint32_t ea, uint8_t v) { wr(s, ea & 0xFFFFFFu, v); }
static inline void wr16(hs_snes *s, uint32_t ea, uint16_t v)
{
    if (ea & DPW) {
        uint16_t a = (uint16_t)ea;
        wr(s, a, (uint8_t)v);
        wr(s, (uint16_t)(a + 1u), (uint8_t)(v >> 8));
    } else {
        wr(s, ea, (uint8_t)v);
        wr(s, ea + 1u, (uint8_t)(v >> 8));
    }
}
static inline void wr16_rev(hs_snes *s, uint32_t ea, uint16_t v)  /* RMW order: high, low */
{
    if (ea & DPW) {
        uint16_t a = (uint16_t)ea;
        wr(s, (uint16_t)(a + 1u), (uint8_t)(v >> 8));
        wr(s, a, (uint8_t)v);
    } else {
        wr(s, ea + 1u, (uint8_t)(v >> 8));
        wr(s, ea, (uint8_t)v);
    }
}

#define CPU   (&s->cpu)
#define M8(c) ((c)->p & CPU_FM)
#define X8(c) ((c)->p & CPU_FX)

static inline void nz8(snes_cpu *c, uint8_t v)
{
    c->p = (uint8_t)((c->p & ~(CPU_FN | CPU_FZ)) | (v & 0x80u) | (v ? 0u : CPU_FZ));
}
static inline void nz16(snes_cpu *c, uint16_t v)
{
    c->p = (uint8_t)((c->p & ~(CPU_FN | CPU_FZ)) | ((v >> 8) & 0x80u) | (v ? 0u : CPU_FZ));
}
static inline void setflag(snes_cpu *c, uint8_t f, int on) { c->p = (uint8_t)(on ? (c->p | f) : (c->p & ~f)); }

static inline uint8_t f8(hs_snes *s)
{
    snes_cpu *c = CPU;
    uint8_t v = rd(s, ((uint32_t)c->pb << 16) | c->pc);
    c->pc++;
    return v;
}
static inline uint16_t f16(hs_snes *s) { uint16_t lo = f8(s); return (uint16_t)(lo | (f8(s) << 8)); }
static inline uint32_t f24(hs_snes *s) { uint32_t lo = f16(s); return lo | ((uint32_t)f8(s) << 16); }

static inline void push8(hs_snes *s, uint8_t v)
{
    snes_cpu *c = CPU;
    wr(s, c->sp, v);
    if (c->e) c->sp = (uint16_t)(0x100u | ((c->sp - 1u) & 0xFFu));
    else c->sp--;
}
static inline uint8_t pull8(hs_snes *s)
{
    snes_cpu *c = CPU;
    if (c->e) c->sp = (uint16_t)(0x100u | ((c->sp + 1u) & 0xFFu));
    else c->sp++;
    return rd(s, c->sp);
}
static inline void push16(hs_snes *s, uint16_t v) { push8(s, (uint8_t)(v >> 8)); push8(s, (uint8_t)v); }
static inline uint16_t pull16(hs_snes *s) { uint16_t lo = pull8(s); return (uint16_t)(lo | (pull8(s) << 8)); }

static void setp(snes_cpu *c, uint8_t v)
{
    if (c->e) v |= (CPU_FM | CPU_FX);
    if (v & CPU_FX) { c->x &= 0xFFu; c->y &= 0xFFu; }
    c->p = v;
}

/* ------------------------------------------------------------ addressing */
static inline int dp_pagewrap(const snes_cpu *c) { return c->e && !(c->dp & 0xFFu); }
static inline void dlio(hs_snes *s) { if (s->cpu.dp & 0xFFu) io(s); }

static uint16_t ptr16(hs_snes *s, uint16_t a, int pw)
{
    uint8_t lo = rd(s, a);
    uint8_t hi = rd(s, pw ? (uint16_t)((a & 0xFF00u) | ((a + 1u) & 0xFFu)) : (uint16_t)(a + 1u));
    return (uint16_t)(lo | (hi << 8));
}

static uint32_t am_dp(hs_snes *s) { uint8_t o = f8(s); dlio(s); return DPW | (uint16_t)(s->cpu.dp + o); }
static uint32_t am_dpidx(hs_snes *s, uint16_t idx)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    dlio(s); io(s);
    if (dp_pagewrap(c)) return DPW | (c->dp & 0xFF00u) | ((o + idx) & 0xFFu);
    return DPW | (uint16_t)(c->dp + o + idx);
}
static uint32_t am_dpi(hs_snes *s)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    dlio(s);
    return ((uint32_t)c->db << 16) | ptr16(s, (uint16_t)(c->dp + o), dp_pagewrap(c));
}
static uint32_t am_dpix(hs_snes *s)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    uint16_t a;
    dlio(s); io(s);
    if (dp_pagewrap(c)) a = (uint16_t)((c->dp & 0xFF00u) | ((o + c->x) & 0xFFu));
    else a = (uint16_t)(c->dp + o + c->x);
    return ((uint32_t)c->db << 16) | ptr16(s, a, dp_pagewrap(c));
}
static uint32_t am_dpiy(hs_snes *s, int w)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    uint32_t base, ea;
    dlio(s);
    base = ((uint32_t)c->db << 16) | ptr16(s, (uint16_t)(c->dp + o), dp_pagewrap(c));
    ea = (base + c->y) & 0xFFFFFFu;
    if (w || !X8(c) || ((base ^ ea) & 0xFF00u)) io(s);
    return ea;
}
static uint32_t am_dpil(hs_snes *s, uint16_t idx)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    uint16_t a;
    uint32_t p;
    dlio(s);
    a = (uint16_t)(c->dp + o);
    p = rd(s, a);
    p |= (uint32_t)rd(s, (uint16_t)(a + 1u)) << 8;
    p |= (uint32_t)rd(s, (uint16_t)(a + 2u)) << 16;
    return (p + idx) & 0xFFFFFFu;
}
static uint32_t am_abs(hs_snes *s) { return ((uint32_t)s->cpu.db << 16) | f16(s); }
static uint32_t am_absidx(hs_snes *s, uint16_t idx, int w)
{
    snes_cpu *c = CPU;
    uint32_t base = ((uint32_t)c->db << 16) | f16(s);
    uint32_t ea = (base + idx) & 0xFFFFFFu;
    if (w || !X8(c) || ((base ^ ea) & 0xFF00u)) io(s);
    return ea;
}
static uint32_t am_long(hs_snes *s, uint16_t idx) { return (f24(s) + idx) & 0xFFFFFFu; }
static uint32_t am_sr(hs_snes *s) { uint8_t o = f8(s); io(s); return DPW | (uint16_t)(s->cpu.sp + o); }
static uint32_t am_sriy(hs_snes *s)
{
    snes_cpu *c = CPU;
    uint8_t o = f8(s);
    uint16_t p;
    io(s);
    p = ptr16(s, (uint16_t)(c->sp + o), 0);
    io(s);
    return ((((uint32_t)c->db << 16) | p) + c->y) & 0xFFFFFFu;
}

/* ALU-group addressing: low 5 bits of the opcode select the mode */
static uint32_t ea_group(hs_snes *s, uint8_t op, int w)
{
    snes_cpu *c = CPU;
    switch (op & 0x1Fu) {
    case 0x01: return am_dpix(s);
    case 0x03: return am_sr(s);
    case 0x05: return am_dp(s);
    case 0x07: return am_dpil(s, 0);
    case 0x0D: return am_abs(s);
    case 0x0F: return am_long(s, 0);
    case 0x11: return am_dpiy(s, w);
    case 0x12: return am_dpi(s);
    case 0x13: return am_sriy(s);
    case 0x15: return am_dpidx(s, c->x);
    case 0x17: return am_dpil(s, c->y);
    case 0x19: return am_absidx(s, c->y, w);
    case 0x1D: return am_absidx(s, c->x, w);
    default:   return am_long(s, c->x);   /* 0x1F */
    }
}

/* ------------------------------------------------------------------ ALU */
static void op_ora(snes_cpu *c, uint16_t v)
{
    if (M8(c)) { c->a = (uint16_t)((c->a & 0xFF00u) | ((c->a | v) & 0xFFu)); nz8(c, (uint8_t)c->a); }
    else { c->a |= v; nz16(c, c->a); }
}
static void op_and(snes_cpu *c, uint16_t v)
{
    if (M8(c)) { c->a = (uint16_t)((c->a & 0xFF00u) | (c->a & v & 0xFFu)); nz8(c, (uint8_t)c->a); }
    else { c->a &= v; nz16(c, c->a); }
}
static void op_eor(snes_cpu *c, uint16_t v)
{
    if (M8(c)) { c->a = (uint16_t)((c->a & 0xFF00u) | ((c->a ^ v) & 0xFFu)); nz8(c, (uint8_t)c->a); }
    else { c->a ^= v; nz16(c, c->a); }
}
static void op_adc(snes_cpu *c, uint16_t v)
{
    int cy = c->p & CPU_FC, r;
    int dec = (c->p & CPU_FD) != 0;
    if (M8(c)) {
        int a = c->a & 0xFF, b = v & 0xFF;
        if (dec) {
            r = (a & 0x0F) + (b & 0x0F) + cy;
            if (r > 0x09) r += 0x06;
            cy = r > 0x0F;
            r = (a & 0xF0) + (b & 0xF0) + (cy << 4) + (r & 0x0F);
        } else {
            r = a + b + cy;
        }
        setflag(c, CPU_FV, (~(a ^ b) & (a ^ r) & 0x80) != 0);
        if (dec && r > 0x9F) r += 0x60;
        setflag(c, CPU_FC, r > 0xFF);
        c->a = (uint16_t)((c->a & 0xFF00u) | (uint8_t)r);
        nz8(c, (uint8_t)r);
    } else {
        int a = c->a, b = v;
        if (dec) {
            r = (a & 0x000F) + (b & 0x000F) + cy;
            if (r > 0x0009) r += 0x0006;
            cy = r > 0x000F;
            r = (a & 0x00F0) + (b & 0x00F0) + (cy << 4) + (r & 0x000F);
            if (r > 0x009F) r += 0x0060;
            cy = r > 0x00FF;
            r = (a & 0x0F00) + (b & 0x0F00) + (cy << 8) + (r & 0x00FF);
            if (r > 0x09FF) r += 0x0600;
            cy = r > 0x0FFF;
            r = (a & 0xF000) + (b & 0xF000) + (cy << 12) + (r & 0x0FFF);
        } else {
            r = a + b + cy;
        }
        setflag(c, CPU_FV, (~(a ^ b) & (a ^ r) & 0x8000) != 0);
        if (dec && r > 0x9FFF) r += 0x6000;
        setflag(c, CPU_FC, r > 0xFFFF);
        c->a = (uint16_t)r;
        nz16(c, c->a);
    }
}
static void op_sbc(snes_cpu *c, uint16_t v)
{
    int cy = c->p & CPU_FC, r;
    int dec = (c->p & CPU_FD) != 0;
    if (M8(c)) {
        int a = c->a & 0xFF, b = (~v) & 0xFF;
        if (dec) {
            r = (a & 0x0F) + (b & 0x0F) + cy;
            if (r <= 0x0F) r -= 0x06;
            cy = r > 0x0F;
            r = (a & 0xF0) + (b & 0xF0) + (cy << 4) + (r & 0x0F);
        } else {
            r = a + b + cy;
        }
        setflag(c, CPU_FV, (~(a ^ b) & (a ^ r) & 0x80) != 0);
        if (dec && r <= 0xFF) r -= 0x60;
        setflag(c, CPU_FC, r > 0xFF);
        c->a = (uint16_t)((c->a & 0xFF00u) | (uint8_t)r);
        nz8(c, (uint8_t)r);
    } else {
        int a = c->a, b = (~v) & 0xFFFF;
        if (dec) {
            r = (a & 0x000F) + (b & 0x000F) + cy;
            if (r <= 0x000F) r -= 0x0006;
            cy = r > 0x000F;
            r = (a & 0x00F0) + (b & 0x00F0) + (cy << 4) + (r & 0x000F);
            if (r <= 0x00FF) r -= 0x0060;
            cy = r > 0x00FF;
            r = (a & 0x0F00) + (b & 0x0F00) + (cy << 8) + (r & 0x00FF);
            if (r <= 0x0FFF) r -= 0x0600;
            cy = r > 0x0FFF;
            r = (a & 0xF000) + (b & 0xF000) + (cy << 12) + (r & 0x0FFF);
        } else {
            r = a + b + cy;
        }
        setflag(c, CPU_FV, (~(a ^ b) & (a ^ r) & 0x8000) != 0);
        if (dec && r <= 0xFFFF) r -= 0x6000;
        setflag(c, CPU_FC, r > 0xFFFF);
        c->a = (uint16_t)r;
        nz16(c, c->a);
    }
}
static void op_cmp_w(snes_cpu *c, uint16_t reg, uint16_t v, int eight)
{
    if (eight) {
        int r = (reg & 0xFF) - (v & 0xFF);
        setflag(c, CPU_FC, r >= 0);
        nz8(c, (uint8_t)r);
    } else {
        int r = (int)reg - (int)v;
        setflag(c, CPU_FC, r >= 0);
        nz16(c, (uint16_t)r);
    }
}
static void op_bit(snes_cpu *c, uint16_t v, int imm)
{
    if (M8(c)) {
        setflag(c, CPU_FZ, ((c->a & v) & 0xFFu) == 0);
        if (!imm) { setflag(c, CPU_FN, v & 0x80u); setflag(c, CPU_FV, v & 0x40u); }
    } else {
        setflag(c, CPU_FZ, (c->a & v) == 0);
        if (!imm) { setflag(c, CPU_FN, v & 0x8000u); setflag(c, CPU_FV, v & 0x4000u); }
    }
}

enum { RMW_ASL, RMW_ROL, RMW_LSR, RMW_ROR, RMW_DEC, RMW_INC, RMW_TSB, RMW_TRB };

static uint16_t rmw_apply(snes_cpu *c, int kind, uint16_t v, int eight)
{
    uint16_t msb = eight ? 0x80u : 0x8000u;
    uint16_t mask = eight ? 0xFFu : 0xFFFFu;
    uint16_t r = v;
    switch (kind) {
    case RMW_ASL: setflag(c, CPU_FC, v & msb); r = (uint16_t)((v << 1) & mask); break;
    case RMW_ROL: r = (uint16_t)(((v << 1) | (c->p & CPU_FC)) & mask); setflag(c, CPU_FC, v & msb); break;
    case RMW_LSR: setflag(c, CPU_FC, v & 1u); r = (uint16_t)(v >> 1); break;
    case RMW_ROR: r = (uint16_t)((v >> 1) | ((c->p & CPU_FC) ? msb : 0u)); setflag(c, CPU_FC, v & 1u); break;
    case RMW_DEC: r = (uint16_t)((v - 1u) & mask); break;
    case RMW_INC: r = (uint16_t)((v + 1u) & mask); break;
    case RMW_TSB: setflag(c, CPU_FZ, ((c->a & mask) & v) == 0); return (uint16_t)((v | c->a) & mask);
    default:      setflag(c, CPU_FZ, ((c->a & mask) & v) == 0); return (uint16_t)(v & ~c->a & mask);
    }
    if (eight) nz8(c, (uint8_t)r); else nz16(c, r);
    return r;
}
static void rmw_mem(hs_snes *s, uint32_t ea, int kind)
{
    snes_cpu *c = CPU;
    if (M8(c)) {
        uint8_t v = rd8(s, ea);
        io(s);
        wr8(s, ea, (uint8_t)rmw_apply(c, kind, v, 1));
    } else {
        uint16_t v = rd16(s, ea);
        io(s);
        wr16_rev(s, ea, rmw_apply(c, kind, v, 0));
    }
}
static void rmw_acc(hs_snes *s, int kind)
{
    snes_cpu *c = CPU;
    io(s);
    if (M8(c)) c->a = (uint16_t)((c->a & 0xFF00u) | rmw_apply(c, kind, c->a & 0xFFu, 1));
    else c->a = rmw_apply(c, kind, c->a, 0);
}
static inline uint16_t rdm(hs_snes *s, uint32_t ea) { return M8(&s->cpu) ? rd8(s, ea) : rd16(s, ea); }
static inline uint16_t rdx(hs_snes *s, uint32_t ea) { return X8(&s->cpu) ? rd8(s, ea) : rd16(s, ea); }
static inline void wrm(hs_snes *s, uint32_t ea, uint16_t v) { if (M8(&s->cpu)) wr8(s, ea, (uint8_t)v); else wr16(s, ea, v); }
static inline void wrx(hs_snes *s, uint32_t ea, uint16_t v) { if (X8(&s->cpu)) wr8(s, ea, (uint8_t)v); else wr16(s, ea, v); }
static inline uint16_t immm(hs_snes *s) { return M8(&s->cpu) ? f8(s) : f16(s); }
static inline uint16_t immx(hs_snes *s) { return X8(&s->cpu) ? f8(s) : f16(s); }

static void lda(snes_cpu *c, uint16_t v)
{
    if (M8(c)) { c->a = (uint16_t)((c->a & 0xFF00u) | (v & 0xFFu)); nz8(c, (uint8_t)v); }
    else { c->a = v; nz16(c, v); }
}
static void ldxy(snes_cpu *c, uint16_t *r, uint16_t v)
{
    if (X8(c)) { *r = v & 0xFFu; nz8(c, (uint8_t)v); }
    else { *r = v; nz16(c, v); }
}

static void branch(hs_snes *s, int cond)
{
    snes_cpu *c = CPU;
    int8_t o = (int8_t)f8(s);
    if (cond) {
        uint16_t old = c->pc;
        io(s);
        c->pc = (uint16_t)(c->pc + o);
        if (c->e && ((old ^ c->pc) & 0xFF00u)) io(s);
    }
}

static void interrupt(hs_snes *s, uint16_t vec_native, uint16_t vec_emu, int brk)
{
    snes_cpu *c = CPU;
    uint16_t vec;
    if (!c->e) push8(s, c->pb);
    push16(s, c->pc);
    if (c->e) push8(s, brk ? (uint8_t)(c->p | 0x30u) : (uint8_t)((c->p | 0x20u) & ~0x10u));
    else push8(s, c->p);
    c->p = (uint8_t)((c->p | CPU_FI) & ~CPU_FD);
    c->pb = 0;
    vec = c->e ? vec_emu : vec_native;
    c->pc = (uint16_t)(rd(s, vec) | (rd(s, (uint16_t)(vec + 1u)) << 8));
}

void snes_cpu_reset(hs_snes *s)
{
    snes_cpu *c = CPU;
    c->e = 1;
    c->p = CPU_FI | CPU_FM | CPU_FX;
    c->a = 0; c->x = 0; c->y = 0;
    c->dp = 0; c->db = 0; c->pb = 0;
    c->sp = 0x01FF;
    c->waiting = 0; c->stopped = 0;
    c->nmi_pending = 0; c->irq_line = 0;
    c->instr_count = 0;
    c->pc = (uint16_t)(rd(s, 0xFFFC) | (rd(s, 0xFFFD) << 8));
}

#define ALU_CASES(b) \
    case (b) + 0x01: case (b) + 0x03: case (b) + 0x05: case (b) + 0x07: \
    case (b) + 0x0D: case (b) + 0x0F: case (b) + 0x11: case (b) + 0x12: \
    case (b) + 0x13: case (b) + 0x15: case (b) + 0x17: case (b) + 0x19: \
    case (b) + 0x1D: case (b) + 0x1F

void snes_cpu_step(hs_snes *s)
{
    snes_cpu *c = CPU;
    uint8_t op;
    uint32_t ea;

    if (c->stopped) { io(s); return; }
    if (c->waiting) {
        if (c->nmi_pending || c->irq_line) c->waiting = 0;
        else { io(s); return; }
    }
    if (c->nmi_pending) {
        c->nmi_pending = 0;
        io(s); io(s);
        interrupt(s, 0xFFEA, 0xFFFA, 0);
        return;
    }
    if (c->irq_line && !(c->p & CPU_FI)) {
        io(s); io(s);
        interrupt(s, 0xFFEE, 0xFFFE, 0);
        return;
    }

    c->instr_count++;
    op = f8(s);
    switch (op) {
    /* ---- ALU group: ORA AND EOR ADC STA LDA CMP SBC ---- */
    ALU_CASES(0x00): ea = ea_group(s, op, 0); op_ora(c, rdm(s, ea)); break;
    ALU_CASES(0x20): ea = ea_group(s, op, 0); op_and(c, rdm(s, ea)); break;
    ALU_CASES(0x40): ea = ea_group(s, op, 0); op_eor(c, rdm(s, ea)); break;
    ALU_CASES(0x60): ea = ea_group(s, op, 0); op_adc(c, rdm(s, ea)); break;
    ALU_CASES(0x80): ea = ea_group(s, op, 1); wrm(s, ea, c->a); break;
    ALU_CASES(0xA0): ea = ea_group(s, op, 0); lda(c, rdm(s, ea)); break;
    ALU_CASES(0xC0): ea = ea_group(s, op, 0); op_cmp_w(c, c->a, rdm(s, ea), M8(c) != 0); break;
    ALU_CASES(0xE0): ea = ea_group(s, op, 0); op_sbc(c, rdm(s, ea)); break;
    case 0x09: op_ora(c, immm(s)); break;
    case 0x29: op_and(c, immm(s)); break;
    case 0x49: op_eor(c, immm(s)); break;
    case 0x69: op_adc(c, immm(s)); break;
    case 0x89: op_bit(c, immm(s), 1); break;
    case 0xA9: lda(c, immm(s)); break;
    case 0xC9: op_cmp_w(c, c->a, immm(s), M8(c) != 0); break;
    case 0xE9: op_sbc(c, immm(s)); break;

    /* ---- read-modify-write ---- */
    case 0x06: rmw_mem(s, am_dp(s), RMW_ASL); break;
    case 0x0E: rmw_mem(s, am_abs(s), RMW_ASL); break;
    case 0x16: rmw_mem(s, am_dpidx(s, c->x), RMW_ASL); break;
    case 0x1E: rmw_mem(s, am_absidx(s, c->x, 1), RMW_ASL); break;
    case 0x26: rmw_mem(s, am_dp(s), RMW_ROL); break;
    case 0x2E: rmw_mem(s, am_abs(s), RMW_ROL); break;
    case 0x36: rmw_mem(s, am_dpidx(s, c->x), RMW_ROL); break;
    case 0x3E: rmw_mem(s, am_absidx(s, c->x, 1), RMW_ROL); break;
    case 0x46: rmw_mem(s, am_dp(s), RMW_LSR); break;
    case 0x4E: rmw_mem(s, am_abs(s), RMW_LSR); break;
    case 0x56: rmw_mem(s, am_dpidx(s, c->x), RMW_LSR); break;
    case 0x5E: rmw_mem(s, am_absidx(s, c->x, 1), RMW_LSR); break;
    case 0x66: rmw_mem(s, am_dp(s), RMW_ROR); break;
    case 0x6E: rmw_mem(s, am_abs(s), RMW_ROR); break;
    case 0x76: rmw_mem(s, am_dpidx(s, c->x), RMW_ROR); break;
    case 0x7E: rmw_mem(s, am_absidx(s, c->x, 1), RMW_ROR); break;
    case 0xC6: rmw_mem(s, am_dp(s), RMW_DEC); break;
    case 0xCE: rmw_mem(s, am_abs(s), RMW_DEC); break;
    case 0xD6: rmw_mem(s, am_dpidx(s, c->x), RMW_DEC); break;
    case 0xDE: rmw_mem(s, am_absidx(s, c->x, 1), RMW_DEC); break;
    case 0xE6: rmw_mem(s, am_dp(s), RMW_INC); break;
    case 0xEE: rmw_mem(s, am_abs(s), RMW_INC); break;
    case 0xF6: rmw_mem(s, am_dpidx(s, c->x), RMW_INC); break;
    case 0xFE: rmw_mem(s, am_absidx(s, c->x, 1), RMW_INC); break;
    case 0x04: rmw_mem(s, am_dp(s), RMW_TSB); break;
    case 0x0C: rmw_mem(s, am_abs(s), RMW_TSB); break;
    case 0x14: rmw_mem(s, am_dp(s), RMW_TRB); break;
    case 0x1C: rmw_mem(s, am_abs(s), RMW_TRB); break;
    case 0x0A: rmw_acc(s, RMW_ASL); break;
    case 0x2A: rmw_acc(s, RMW_ROL); break;
    case 0x4A: rmw_acc(s, RMW_LSR); break;
    case 0x6A: rmw_acc(s, RMW_ROR); break;
    case 0x1A: rmw_acc(s, RMW_INC); break;
    case 0x3A: rmw_acc(s, RMW_DEC); break;

    /* ---- BIT ---- */
    case 0x24: op_bit(c, rdm(s, am_dp(s)), 0); break;
    case 0x2C: op_bit(c, rdm(s, am_abs(s)), 0); break;
    case 0x34: op_bit(c, rdm(s, am_dpidx(s, c->x)), 0); break;
    case 0x3C: op_bit(c, rdm(s, am_absidx(s, c->x, 0)), 0); break;

    /* ---- index loads / stores / compares ---- */
    case 0xA0: ldxy(c, &c->y, immx(s)); break;
    case 0xA2: ldxy(c, &c->x, immx(s)); break;
    case 0xA4: ldxy(c, &c->y, rdx(s, am_dp(s))); break;
    case 0xA6: ldxy(c, &c->x, rdx(s, am_dp(s))); break;
    case 0xAC: ldxy(c, &c->y, rdx(s, am_abs(s))); break;
    case 0xAE: ldxy(c, &c->x, rdx(s, am_abs(s))); break;
    case 0xB4: ldxy(c, &c->y, rdx(s, am_dpidx(s, c->x))); break;
    case 0xB6: ldxy(c, &c->x, rdx(s, am_dpidx(s, c->y))); break;
    case 0xBC: ldxy(c, &c->y, rdx(s, am_absidx(s, c->x, 0))); break;
    case 0xBE: ldxy(c, &c->x, rdx(s, am_absidx(s, c->y, 0))); break;
    case 0x84: wrx(s, am_dp(s), c->y); break;
    case 0x86: wrx(s, am_dp(s), c->x); break;
    case 0x8C: wrx(s, am_abs(s), c->y); break;
    case 0x8E: wrx(s, am_abs(s), c->x); break;
    case 0x94: wrx(s, am_dpidx(s, c->x), c->y); break;
    case 0x96: wrx(s, am_dpidx(s, c->y), c->x); break;
    case 0x64: wrm(s, am_dp(s), 0); break;
    case 0x74: wrm(s, am_dpidx(s, c->x), 0); break;
    case 0x9C: wrm(s, am_abs(s), 0); break;
    case 0x9E: wrm(s, am_absidx(s, c->x, 1), 0); break;
    case 0xC0: op_cmp_w(c, c->y, immx(s), X8(c) != 0); break;
    case 0xC4: op_cmp_w(c, c->y, rdx(s, am_dp(s)), X8(c) != 0); break;
    case 0xCC: op_cmp_w(c, c->y, rdx(s, am_abs(s)), X8(c) != 0); break;
    case 0xE0: op_cmp_w(c, c->x, immx(s), X8(c) != 0); break;
    case 0xE4: op_cmp_w(c, c->x, rdx(s, am_dp(s)), X8(c) != 0); break;
    case 0xEC: op_cmp_w(c, c->x, rdx(s, am_abs(s)), X8(c) != 0); break;

    /* ---- increments / decrements of index ---- */
    case 0xE8: io(s); c->x = (uint16_t)(X8(c) ? ((c->x + 1u) & 0xFFu) : (c->x + 1u)); if (X8(c)) nz8(c, (uint8_t)c->x); else nz16(c, c->x); break;
    case 0xC8: io(s); c->y = (uint16_t)(X8(c) ? ((c->y + 1u) & 0xFFu) : (c->y + 1u)); if (X8(c)) nz8(c, (uint8_t)c->y); else nz16(c, c->y); break;
    case 0xCA: io(s); c->x = (uint16_t)(X8(c) ? ((c->x - 1u) & 0xFFu) : (c->x - 1u)); if (X8(c)) nz8(c, (uint8_t)c->x); else nz16(c, c->x); break;
    case 0x88: io(s); c->y = (uint16_t)(X8(c) ? ((c->y - 1u) & 0xFFu) : (c->y - 1u)); if (X8(c)) nz8(c, (uint8_t)c->y); else nz16(c, c->y); break;

    /* ---- transfers ---- */
    case 0xAA: io(s); ldxy(c, &c->x, c->a); break;                      /* TAX */
    case 0xA8: io(s); ldxy(c, &c->y, c->a); break;                      /* TAY */
    case 0x8A: io(s); lda(c, c->x); break;                              /* TXA */
    case 0x98: io(s); lda(c, c->y); break;                              /* TYA */
    case 0x9B: io(s); ldxy(c, &c->y, c->x); break;                      /* TXY */
    case 0xBB: io(s); ldxy(c, &c->x, c->y); break;                      /* TYX */
    case 0xBA: io(s); ldxy(c, &c->x, c->sp); break;                     /* TSX */
    case 0x9A: io(s); c->sp = c->e ? (uint16_t)(0x100u | (c->x & 0xFFu)) : c->x; break; /* TXS */
    case 0x1B: io(s); c->sp = c->e ? (uint16_t)(0x100u | (c->a & 0xFFu)) : c->a; break; /* TCS */
    case 0x3B: io(s); c->a = c->sp; nz16(c, c->a); break;              /* TSC */
    case 0x5B: io(s); c->dp = c->a; nz16(c, c->dp); break;             /* TCD */
    case 0x7B: io(s); c->a = c->dp; nz16(c, c->a); break;              /* TDC */
    case 0xEB: io(s); io(s); c->a = (uint16_t)((c->a >> 8) | (c->a << 8)); nz8(c, (uint8_t)c->a); break; /* XBA */
    case 0xFB: {                                                        /* XCE */
        uint8_t oc = c->p & CPU_FC;
        io(s);
        setflag(c, CPU_FC, c->e);
        c->e = oc;
        if (c->e) { c->p |= CPU_FM | CPU_FX; c->x &= 0xFFu; c->y &= 0xFFu; c->sp = (uint16_t)(0x100u | (c->sp & 0xFFu)); }
        break;
    }

    /* ---- flags ---- */
    case 0x18: io(s); c->p &= (uint8_t)~CPU_FC; break;
    case 0x38: io(s); c->p |= CPU_FC; break;
    case 0x58: io(s); c->p &= (uint8_t)~CPU_FI; break;
    case 0x78: io(s); c->p |= CPU_FI; break;
    case 0xB8: io(s); c->p &= (uint8_t)~CPU_FV; break;
    case 0xD8: io(s); c->p &= (uint8_t)~CPU_FD; break;
    case 0xF8: io(s); c->p |= CPU_FD; break;
    case 0xC2: { uint8_t v = f8(s); io(s); setp(c, (uint8_t)(c->p & ~v)); break; }   /* REP */
    case 0xE2: { uint8_t v = f8(s); io(s); setp(c, (uint8_t)(c->p | v)); break; }    /* SEP */

    /* ---- stack ---- */
    case 0x48: io(s); if (M8(c)) push8(s, (uint8_t)c->a); else push16(s, c->a); break;
    case 0xDA: io(s); if (X8(c)) push8(s, (uint8_t)c->x); else push16(s, c->x); break;
    case 0x5A: io(s); if (X8(c)) push8(s, (uint8_t)c->y); else push16(s, c->y); break;
    case 0x68: io(s); io(s); lda(c, M8(c) ? pull8(s) : pull16(s)); break;
    case 0xFA: io(s); io(s); ldxy(c, &c->x, X8(c) ? pull8(s) : pull16(s)); break;
    case 0x7A: io(s); io(s); ldxy(c, &c->y, X8(c) ? pull8(s) : pull16(s)); break;
    case 0x8B: io(s); push8(s, c->db); break;
    case 0xAB: io(s); io(s); c->db = pull8(s); nz8(c, c->db); break;
    case 0x4B: io(s); push8(s, c->pb); break;
    case 0x0B: io(s); push16(s, c->dp); break;
    case 0x2B: io(s); io(s); c->dp = pull16(s); nz16(c, c->dp); break;
    case 0x08: io(s); push8(s, c->e ? (uint8_t)(c->p | 0x30u) : c->p); break;
    case 0x28: io(s); io(s); setp(c, pull8(s)); break;
    case 0xF4: push16(s, f16(s)); break;                                          /* PEA */
    case 0xD4: { uint8_t o = f8(s); dlio(s); push16(s, ptr16(s, (uint16_t)(c->dp + o), 0)); break; } /* PEI */
    case 0x62: { uint16_t o = f16(s); io(s); push16(s, (uint16_t)(c->pc + o)); break; }           /* PER */

    /* ---- branches ---- */
    case 0x10: branch(s, !(c->p & CPU_FN)); break;
    case 0x30: branch(s, (c->p & CPU_FN) != 0); break;
    case 0x50: branch(s, !(c->p & CPU_FV)); break;
    case 0x70: branch(s, (c->p & CPU_FV) != 0); break;
    case 0x90: branch(s, !(c->p & CPU_FC)); break;
    case 0xB0: branch(s, (c->p & CPU_FC) != 0); break;
    case 0xD0: branch(s, !(c->p & CPU_FZ)); break;
    case 0xF0: branch(s, (c->p & CPU_FZ) != 0); break;
    case 0x80: branch(s, 1); break;
    case 0x82: { uint16_t o = f16(s); io(s); c->pc = (uint16_t)(c->pc + o); break; }   /* BRL */

    /* ---- jumps / calls ---- */
    case 0x4C: c->pc = f16(s); break;
    case 0x5C: { uint32_t a = f24(s); c->pb = (uint8_t)(a >> 16); c->pc = (uint16_t)a; break; }
    case 0x6C: { uint16_t a = f16(s); c->pc = (uint16_t)(rd(s, a) | (rd(s, (uint16_t)(a + 1u)) << 8)); break; }
    case 0x7C: {
        uint16_t a = (uint16_t)(f16(s) + c->x);
        uint32_t b = (uint32_t)c->pb << 16;
        io(s);
        c->pc = (uint16_t)(rd(s, b | a) | (rd(s, b | (uint16_t)(a + 1u)) << 8));
        break;
    }
    case 0xDC: {
        uint16_t a = f16(s);
        uint16_t lo = (uint16_t)(rd(s, a) | (rd(s, (uint16_t)(a + 1u)) << 8));
        c->pb = rd(s, (uint16_t)(a + 2u));
        c->pc = lo;
        break;
    }
    case 0x20: { uint16_t a = f16(s); io(s); push16(s, (uint16_t)(c->pc - 1u)); c->pc = a; break; }
    case 0xFC: {
        uint16_t a = f16(s);
        uint32_t b = (uint32_t)c->pb << 16;
        push16(s, (uint16_t)(c->pc - 1u));
        io(s);
        a = (uint16_t)(a + c->x);
        c->pc = (uint16_t)(rd(s, b | a) | (rd(s, b | (uint16_t)(a + 1u)) << 8));
        break;
    }
    case 0x22: {
        uint16_t a = f16(s);
        uint8_t bank;
        push8(s, c->pb);
        io(s);
        bank = f8(s);
        push16(s, (uint16_t)(c->pc - 1u));
        c->pb = bank; c->pc = a;
        break;
    }
    case 0x60: io(s); io(s); c->pc = (uint16_t)(pull16(s) + 1u); io(s); break;
    case 0x6B: io(s); io(s); c->pc = (uint16_t)(pull16(s) + 1u); c->pb = pull8(s); break;
    case 0x40:
        io(s); io(s);
        setp(c, pull8(s));
        c->pc = pull16(s);
        if (!c->e) c->pb = pull8(s);
        break;

    /* ---- software interrupts ---- */
    case 0x00: f8(s); interrupt(s, 0xFFE6, 0xFFFE, 1); break;   /* BRK */
    case 0x02: f8(s); interrupt(s, 0xFFE4, 0xFFF4, 1); break;   /* COP */

    /* ---- block move ---- */
    case 0x54: case 0x44: {
        uint8_t dst = f8(s), src = f8(s);
        uint8_t v;
        c->db = dst;
        v = rd(s, ((uint32_t)src << 16) | c->x);
        wr(s, ((uint32_t)dst << 16) | c->y, v);
        io(s); io(s);
        if (op == 0x54) { c->x++; c->y++; } else { c->x--; c->y--; }
        if (X8(c)) { c->x &= 0xFFu; c->y &= 0xFFu; }
        c->a--;
        if (c->a != 0xFFFFu) c->pc = (uint16_t)(c->pc - 3u);
        break;
    }

    /* ---- misc ---- */
    case 0xEA: io(s); break;                       /* NOP */
    case 0x42: f8(s); break;                       /* WDM */
    case 0xCB: io(s); io(s); c->waiting = 1; break; /* WAI */
    case 0xDB: io(s); io(s); c->stopped = 1; break; /* STP */
    default: break;  /* unreachable: every opcode is decoded above */
    }
}
