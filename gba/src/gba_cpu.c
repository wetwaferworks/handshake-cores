/* SPDX-License-Identifier: MIT
 * gba_cpu.c - clean-room ARM7TDMI (ARMv4T) interpreter: 32-bit ARM + 16-bit
 * Thumb, all banked modes, barrel shifter, multiplies, block transfers,
 * interworking, IRQ entry, and SWI dispatch to the HLE BIOS.
 * Written from the public ARMv4T architecture description; no third-party code.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include "gba_internal.h"

/* ---------- ARM decode classes (12-bit LUT: bits 27..20 + 7..4) ---------- */
enum {
  A_DP = 0, A_MUL, A_MULL, A_SWP, A_BX, A_HALF, A_MRS, A_MSR, A_SDT, A_LDM,
  A_B, A_SWI, A_UND, A_COP
};
/* ---------- Thumb decode formats (10-bit LUT: bits 15..6) ---------- */
enum {
  T_SHIFT = 0, T_ADDSUB, T_IMM, T_ALU, T_HIREG, T_LDRPC, T_LSREG, T_LSHALF,
  T_LSIMM, T_LSH_IMM, T_LSSP, T_LDADDR, T_ADDSP, T_PUSHPOP, T_LDMSTM,
  T_BCOND, T_SWI, T_B, T_BL, T_UND
};

static uint8_t arm_classify(uint32_t i) {
  uint32_t hi = (i >> 4) & 0xFFu, lo = i & 0xFu;
  if ((hi & 0xE0u) == 0x00u) {
    if (lo == 0x9u) {
      if ((hi & 0xFCu) == 0x00u) return A_MUL;
      if ((hi & 0xF8u) == 0x08u) return A_MULL;
      if ((hi & 0xFBu) == 0x10u) return A_SWP;
      return A_UND;
    }
    if ((lo & 0x9u) == 0x9u) return A_HALF;
    if (hi == 0x12u && lo == 0x1u) return A_BX;
    if ((hi & 0xFBu) == 0x10u && lo == 0u) return A_MRS;
    if ((hi & 0xFBu) == 0x12u && lo == 0u) return A_MSR;
    if ((hi & 0x19u) == 0x10u) return A_UND;
    return A_DP;
  }
  if ((hi & 0xE0u) == 0x20u) {
    if ((hi & 0xFBu) == 0x32u) return A_MSR;
    if ((hi & 0x19u) == 0x10u) return A_UND;
    return A_DP;
  }
  if ((hi & 0xE0u) == 0x40u) return A_SDT;
  if ((hi & 0xE0u) == 0x60u) return (lo & 1u) ? A_UND : A_SDT;
  if ((hi & 0xE0u) == 0x80u) return A_LDM;
  if ((hi & 0xE0u) == 0xA0u) return A_B;
  if ((hi & 0xF0u) == 0xF0u) return A_SWI;
  return A_COP;
}

static uint8_t thumb_classify(uint32_t i) {
  uint32_t op = i << 6; /* reconstruct top 10 bits */
  if ((op & 0xF800u) == 0x1800u) return T_ADDSUB;
  if ((op & 0xE000u) == 0x0000u) return T_SHIFT;
  if ((op & 0xE000u) == 0x2000u) return T_IMM;
  if ((op & 0xFC00u) == 0x4000u) return T_ALU;
  if ((op & 0xFC00u) == 0x4400u) return T_HIREG;
  if ((op & 0xF800u) == 0x4800u) return T_LDRPC;
  if ((op & 0xF200u) == 0x5000u) return T_LSREG;
  if ((op & 0xF200u) == 0x5200u) return T_LSHALF;
  if ((op & 0xE000u) == 0x6000u) return T_LSIMM;
  if ((op & 0xF000u) == 0x8000u) return T_LSH_IMM;
  if ((op & 0xF000u) == 0x9000u) return T_LSSP;
  if ((op & 0xF000u) == 0xA000u) return T_LDADDR;
  if ((op & 0xFF00u) == 0xB000u) return T_ADDSP;
  if ((op & 0xF600u) == 0xB400u) return T_PUSHPOP;
  if ((op & 0xF000u) == 0xC000u) return T_LDMSTM;
  if ((op & 0xFF00u) == 0xDF00u) return T_SWI;
  if ((op & 0xFF00u) == 0xDE00u) return T_UND;
  if ((op & 0xF000u) == 0xD000u) return T_BCOND;
  if ((op & 0xF800u) == 0xE000u) return T_B;
  if ((op & 0xF000u) == 0xF000u) return T_BL;
  return T_UND;
}

void gba_cpu_init_tables(gba_cpu *c) {
  for (uint32_t i = 0; i < 4096u; i++) c->arm_lut[i] = arm_classify(i);
  for (uint32_t i = 0; i < 1024u; i++) c->thumb_lut[i] = thumb_classify(i);
}

/* ---------- modes / banking ---------- */
static int bank_of(uint32_t mode) {
  switch (mode & 0x1Fu) {
    case MODE_FIQ: return 1;
    case MODE_IRQ: return 2;
    case MODE_SVC: return 3;
    case MODE_ABT: return 4;
    case MODE_UND: return 5;
    default: return 0;
  }
}

void gba_cpu_set_cpsr(gba_cpu *c, uint32_t v) {
  int ob = bank_of(c->cpsr), nb = bank_of(v);
  if (ob != nb) {
    c->bank_sp[ob] = c->r[13];
    c->bank_lr[ob] = c->r[14];
    c->bank_spsr[ob] = c->spsr;
    if (ob == 1) {
      for (int i = 0; i < 5; i++) { c->bank_fiq[i] = c->r[8 + i]; c->r[8 + i] = c->bank_usr[i]; }
    }
    if (nb == 1) {
      for (int i = 0; i < 5; i++) { c->bank_usr[i] = c->r[8 + i]; c->r[8 + i] = c->bank_fiq[i]; }
    }
    c->r[13] = c->bank_sp[nb];
    c->r[14] = c->bank_lr[nb];
    c->spsr = c->bank_spsr[nb];
  }
  c->cpsr = v | 0x10u;
}

static inline int has_spsr(const gba_cpu *c) { return bank_of(c->cpsr) != 0; }

uint32_t gba_cpu_get_user_reg(const gba_cpu *c, int r) {
  int b = bank_of(c->cpsr);
  if (r >= 8 && r <= 12 && b == 1) return c->bank_usr[r - 8];
  if (r == 13 && b != 0) return c->bank_sp[0];
  if (r == 14 && b != 0) return c->bank_lr[0];
  return c->r[r];
}

static void set_user_reg(gba_cpu *c, int r, uint32_t v) {
  int b = bank_of(c->cpsr);
  if (r >= 8 && r <= 12 && b == 1) { c->bank_usr[r - 8] = v; return; }
  if (r == 13 && b != 0) { c->bank_sp[0] = v; return; }
  if (r == 14 && b != 0) { c->bank_lr[0] = v; return; }
  c->r[r] = v;
}

void gba_cpu_jump(gba_cpu *c, uint32_t addr, int thumb) {
  if (thumb) { c->cpsr |= CPSR_T; c->pc_next = addr & ~1u; }
  else { c->cpsr &= ~CPSR_T; c->pc_next = addr & ~3u; }
  c->branched = 1;
}

static inline void write_pc(gba_cpu *c, uint32_t v) {
  c->pc_next = (c->cpsr & CPSR_T) ? (v & ~1u) : (v & ~3u);
  c->branched = 1;
}

static inline void wreg(gba_cpu *c, int rd, uint32_t v) {
  if (rd == 15) write_pc(c, v); else c->r[rd] = v;
}

void gba_cpu_reset(hs_gba *g, int skip_bios) {
  gba_cpu *c = &g->cpu;
  uint8_t alut[4096], tlut[1024];
  memcpy(alut, c->arm_lut, sizeof alut);
  memcpy(tlut, c->thumb_lut, sizeof tlut);
  memset(c, 0, sizeof *c);
  memcpy(c->arm_lut, alut, sizeof alut);
  memcpy(c->thumb_lut, tlut, sizeof tlut);
  c->cpsr = MODE_SVC | CPSR_I | CPSR_F;
  if (skip_bios) {
    c->bank_sp[0] = 0x03007F00u;
    c->bank_sp[2] = 0x03007FA0u;
    c->bank_sp[3] = 0x03007FE0u;
    c->r[13] = c->bank_sp[3];
    gba_cpu_set_cpsr(c, MODE_SYS);
    c->pc_next = 0x08000000u;
  } else {
    c->pc_next = 0;
  }
}

/* ---------- flags / ALU helpers ---------- */
static inline void set_nz(gba_cpu *c, uint32_t v) {
  c->cpsr = (c->cpsr & ~(CPSR_N | CPSR_Z)) | (v & CPSR_N) | (v ? 0u : CPSR_Z);
}
static inline void set_nzc(gba_cpu *c, uint32_t v, uint32_t carry) {
  c->cpsr = (c->cpsr & ~(CPSR_N | CPSR_Z | CPSR_C)) | (v & CPSR_N) | (v ? 0u : CPSR_Z) |
            (carry ? CPSR_C : 0u);
}
static inline uint32_t add_core(gba_cpu *c, uint32_t a, uint32_t b, uint32_t cin, int s) {
  uint64_t w = (uint64_t)a + b + cin;
  uint32_t r = (uint32_t)w;
  if (s) {
    uint32_t f = c->cpsr & 0x0FFFFFFFu;
    f |= r & CPSR_N;
    if (!r) f |= CPSR_Z;
    if (w >> 32) f |= CPSR_C;
    if ((~(a ^ b) & (a ^ r)) >> 31) f |= CPSR_V;
    c->cpsr = f;
  }
  return r;
}
static inline uint32_t ror32(uint32_t v, uint32_t n) { n &= 31u; return n ? (v >> n) | (v << (32u - n)) : v; }
static inline uint32_t cflag(const gba_cpu *c) { return (c->cpsr >> 29) & 1u; }

/* shift by immediate (encoding semantics: amount 0 has special meanings) */
static inline uint32_t shift_imm(uint32_t v, uint32_t type, uint32_t amt, uint32_t cin, uint32_t *co) {
  switch (type) {
    case 0:
      if (!amt) { *co = cin; return v; }
      *co = (v >> (32u - amt)) & 1u; return v << amt;
    case 1:
      if (!amt) { *co = v >> 31; return 0; }
      *co = (v >> (amt - 1u)) & 1u; return v >> amt;
    case 2:
      if (!amt) { *co = v >> 31; return (uint32_t)((int32_t)v >> 31); }
      *co = (v >> (amt - 1u)) & 1u; return (uint32_t)((int32_t)v >> amt);
    default:
      if (!amt) { *co = v & 1u; return (v >> 1) | (cin << 31); } /* RRX */
      *co = (v >> (amt - 1u)) & 1u; return ror32(v, amt);
  }
}
/* shift by register (bottom byte of Rs) */
static inline uint32_t shift_reg(uint32_t v, uint32_t type, uint32_t amt, uint32_t cin, uint32_t *co) {
  if (!amt) { *co = cin; return v; }
  switch (type) {
    case 0:
      if (amt < 32u) { *co = (v >> (32u - amt)) & 1u; return v << amt; }
      *co = (amt == 32u) ? (v & 1u) : 0u; return 0;
    case 1:
      if (amt < 32u) { *co = (v >> (amt - 1u)) & 1u; return v >> amt; }
      *co = (amt == 32u) ? (v >> 31) : 0u; return 0;
    case 2:
      if (amt < 32u) { *co = (v >> (amt - 1u)) & 1u; return (uint32_t)((int32_t)v >> amt); }
      *co = v >> 31; return (uint32_t)((int32_t)v >> 31);
    default:
      amt &= 31u;
      if (!amt) { *co = v >> 31; return v; }
      *co = (v >> (amt - 1u)) & 1u; return ror32(v, amt);
  }
}

static inline int cond_pass(uint32_t cond, uint32_t f) {
  uint32_t n = (f >> 31) & 1u, z = (f >> 30) & 1u, cc = (f >> 29) & 1u, v = (f >> 28) & 1u;
  switch (cond) {
    case 0x0: return z;
    case 0x1: return !z;
    case 0x2: return cc;
    case 0x3: return !cc;
    case 0x4: return n;
    case 0x5: return !n;
    case 0x6: return v;
    case 0x7: return !v;
    case 0x8: return cc && !z;
    case 0x9: return !cc || z;
    case 0xA: return n == v;
    case 0xB: return n != v;
    case 0xC: return !z && (n == v);
    case 0xD: return z || (n != v);
    case 0xE: return 1;
    default: return 0; /* 0xF: never on ARMv4 */
  }
}

/* rotated / sign-extended loads as seen by the ARM7TDMI */
static inline uint32_t ld_word(hs_gba *g, uint32_t a) { return ror32(gba_read32(g, a & ~3u), (a & 3u) * 8u); }
static inline uint32_t ld_half(hs_gba *g, uint32_t a) { return ror32(gba_read16(g, a & ~1u), (a & 1u) * 8u); }
static inline uint32_t ld_shalf(hs_gba *g, uint32_t a) {
  if (a & 1u) return (uint32_t)(int32_t)(int8_t)gba_read8(g, a);
  return (uint32_t)(int32_t)(int16_t)gba_read16(g, a);
}
static inline uint32_t ld_sbyte(hs_gba *g, uint32_t a) { return (uint32_t)(int32_t)(int8_t)gba_read8(g, a); }

static uint32_t mul_cycles(uint32_t rs) {
  if ((rs >> 8) == 0u || (rs >> 8) == 0xFFFFFFu) return 1;
  if ((rs >> 16) == 0u || (rs >> 16) == 0xFFFFu) return 2;
  if ((rs >> 24) == 0u || (rs >> 24) == 0xFFu) return 3;
  return 4;
}

static uint32_t do_swi(hs_gba *g, uint32_t num) {
  gba_cpu *c = &g->cpu;
  if (g->has_bios) {
    uint32_t old = c->cpsr;
    gba_cpu_set_cpsr(c, (old & ~(0x1Fu | CPSR_T)) | MODE_SVC | CPSR_I);
    c->spsr = old;
    c->r[14] = c->pc_next;
    c->pc_next = 0x08u;
    c->branched = 1;
    return 3;
  }
  return gba_hle_swi(g, num);
}

static uint32_t do_undefined(hs_gba *g) {
  gba_cpu *c = &g->cpu;
  c->undefined_count++;
  if (g->has_bios) {
    uint32_t old = c->cpsr;
    gba_cpu_set_cpsr(c, (old & ~(0x1Fu | CPSR_T)) | MODE_UND | CPSR_I);
    c->spsr = old;
    c->r[14] = c->pc_next;
    c->pc_next = 0x04u;
    c->branched = 1;
  }
  return 3;
}

int gba_cpu_irq_check(hs_gba *g) {
  gba_cpu *c = &g->cpu;
  uint16_t pend = (uint16_t)(g->ie & g->ifl & 0x3FFFu);
  if (!pend) return 0;
  c->halted = 0;
  if (!(g->ime & 1u) || (c->cpsr & CPSR_I)) return 0;
  uint32_t old = c->cpsr;
  uint32_t ret = c->pc_next + 4u;
  gba_cpu_set_cpsr(c, (old & ~(0x1Fu | CPSR_T)) | MODE_IRQ | CPSR_I);
  c->spsr = old;
  c->r[14] = ret;
  c->pc_next = 0x18u;
  return 1;
}

/* ---------------------------- ARM execution ---------------------------- */
static uint32_t arm_dp(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t cin = cflag(c), carry = cin, op2, cyc = 1;
  uint32_t rn = (op >> 16) & 15u, rd = (op >> 12) & 15u, opc = (op >> 21) & 15u;
  int s = (op >> 20) & 1u;
  uint32_t a = c->r[rn];
  if (op & (1u << 25)) {
    uint32_t rot = ((op >> 8) & 15u) * 2u;
    op2 = ror32(op & 0xFFu, rot);
    if (rot) carry = op2 >> 31;
  } else {
    uint32_t rm = op & 15u, type = (op >> 5) & 3u;
    if (op & 0x10u) {
      uint32_t amt = c->r[(op >> 8) & 15u] & 0xFFu;
      uint32_t mv = c->r[rm] + (rm == 15u ? 4u : 0u);
      if (rn == 15u) a += 4u;
      op2 = shift_reg(mv, type, amt, cin, &carry);
      cyc++;
    } else {
      op2 = shift_imm(c->r[rm], type, (op >> 7) & 31u, cin, &carry);
    }
  }
  uint32_t r = 0;
  int write = 1, logical = 0;
  switch (opc) {
    case 0x0: r = a & op2; logical = 1; break;
    case 0x1: r = a ^ op2; logical = 1; break;
    case 0x2: r = add_core(c, a, ~op2, 1, s && rd != 15u); break;
    case 0x3: r = add_core(c, op2, ~a, 1, s && rd != 15u); break;
    case 0x4: r = add_core(c, a, op2, 0, s && rd != 15u); break;
    case 0x5: r = add_core(c, a, op2, cin, s && rd != 15u); break;
    case 0x6: r = add_core(c, a, ~op2, cin, s && rd != 15u); break;
    case 0x7: r = add_core(c, op2, ~a, cin, s && rd != 15u); break;
    case 0x8: r = a & op2; logical = 1; write = 0; break;
    case 0x9: r = a ^ op2; logical = 1; write = 0; break;
    case 0xA: r = add_core(c, a, ~op2, 1, 1); write = 0; break;
    case 0xB: r = add_core(c, a, op2, 0, 1); write = 0; break;
    case 0xC: r = a | op2; logical = 1; break;
    case 0xD: r = op2; logical = 1; break;
    case 0xE: r = a & ~op2; logical = 1; break;
    default: r = ~op2; logical = 1; break;
  }
  if (!write) {
    if (logical) set_nzc(c, r, carry);
    return cyc;
  }
  if (rd == 15u) {
    if (s && has_spsr(c)) gba_cpu_set_cpsr(c, c->spsr);
    write_pc(c, r);
    return cyc + 2u;
  }
  if (s && logical) set_nzc(c, r, carry);
  c->r[rd] = r;
  return cyc;
}

static uint32_t arm_mul(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rd = (op >> 16) & 15u, rn = (op >> 12) & 15u, rs = (op >> 8) & 15u, rm = op & 15u;
  uint32_t r = c->r[rm] * c->r[rs];
  uint32_t cyc = 1u + mul_cycles(c->r[rs]);
  if (op & (1u << 21)) { r += c->r[rn]; cyc++; }
  if (op & (1u << 20)) set_nz(c, r);
  wreg(c, (int)rd, r);
  return cyc;
}

static uint32_t arm_mull(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t hi = (op >> 16) & 15u, lo = (op >> 12) & 15u, rs = (op >> 8) & 15u, rm = op & 15u;
  uint64_t r;
  if (op & (1u << 22)) r = (uint64_t)((int64_t)(int32_t)c->r[rm] * (int64_t)(int32_t)c->r[rs]);
  else r = (uint64_t)c->r[rm] * (uint64_t)c->r[rs];
  uint32_t cyc = 2u + mul_cycles(c->r[rs]);
  if (op & (1u << 21)) { r += ((uint64_t)c->r[hi] << 32) | c->r[lo]; cyc++; }
  if (op & (1u << 20)) {
    c->cpsr = (c->cpsr & ~(CPSR_N | CPSR_Z)) | ((uint32_t)(r >> 32) & CPSR_N) | (r ? 0u : CPSR_Z);
  }
  c->r[lo] = (uint32_t)r;
  c->r[hi] = (uint32_t)(r >> 32);
  return cyc;
}

static uint32_t arm_swp(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rn = (op >> 16) & 15u, rd = (op >> 12) & 15u, rm = op & 15u;
  uint32_t addr = c->r[rn], src = c->r[rm], tmp;
  if (op & (1u << 22)) { tmp = gba_read8(g, addr); gba_write8(g, addr, (uint8_t)src); }
  else { tmp = ld_word(g, addr); gba_write32(g, addr & ~3u, src); }
  wreg(c, (int)rd, tmp);
  return 4;
}

static uint32_t arm_bx(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t v = c->r[op & 15u];
  gba_cpu_jump(c, v, (int)(v & 1u));
  return 3;
}

static uint32_t arm_mrs(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rd = (op >> 12) & 15u;
  c->r[rd] = ((op >> 22) & 1u) ? (has_spsr(c) ? c->spsr : c->cpsr) : c->cpsr;
  return 1;
}

static uint32_t arm_msr(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t v;
  if (op & (1u << 25)) v = ror32(op & 0xFFu, ((op >> 8) & 15u) * 2u);
  else v = c->r[op & 15u];
  uint32_t mask = 0;
  if (op & (1u << 16)) mask |= 0x000000FFu;
  if (op & (1u << 17)) mask |= 0x0000FF00u;
  if (op & (1u << 18)) mask |= 0x00FF0000u;
  if (op & (1u << 19)) mask |= 0xFF000000u;
  if (op & (1u << 22)) {
    if (has_spsr(c)) c->spsr = (c->spsr & ~mask) | (v & mask);
  } else {
    if ((c->cpsr & 0x1Fu) == MODE_USR) mask &= 0xFF000000u;
    uint32_t nv = (c->cpsr & ~mask) | (v & mask);
    nv = (nv & ~CPSR_T) | (c->cpsr & CPSR_T);
    gba_cpu_set_cpsr(c, nv);
  }
  return 1;
}

static uint32_t arm_sdt(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rn = (op >> 16) & 15u, rd = (op >> 12) & 15u;
  int P = (op >> 24) & 1u, U = (op >> 23) & 1u, B = (op >> 22) & 1u, W = (op >> 21) & 1u,
      L = (op >> 20) & 1u;
  uint32_t off;
  if (op & (1u << 25)) {
    uint32_t dummy;
    off = shift_imm(c->r[op & 15u], (op >> 5) & 3u, (op >> 7) & 31u, cflag(c), &dummy);
  } else {
    off = op & 0xFFFu;
  }
  uint32_t base = c->r[rn];
  uint32_t eff = U ? base + off : base - off;
  uint32_t addr = P ? eff : base;
  int wb = !P || W;
  if (L) {
    uint32_t v = B ? gba_read8(g, addr) : ld_word(g, addr);
    if (wb && rn != 15u) c->r[rn] = eff;
    if (rd == 15u) { write_pc(c, v); return 5; }
    c->r[rd] = v;
    return 3;
  }
  uint32_t v = c->r[rd] + (rd == 15u ? 4u : 0u);
  if (B) gba_write8(g, addr, (uint8_t)v);
  else gba_write32(g, addr & ~3u, v);
  if (wb && rn != 15u) c->r[rn] = eff;
  return 2;
}

static uint32_t arm_half(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rn = (op >> 16) & 15u, rd = (op >> 12) & 15u, sh = (op >> 5) & 3u;
  int P = (op >> 24) & 1u, U = (op >> 23) & 1u, I = (op >> 22) & 1u, W = (op >> 21) & 1u,
      L = (op >> 20) & 1u;
  uint32_t off = I ? (((op >> 4) & 0xF0u) | (op & 0xFu)) : c->r[op & 15u];
  uint32_t base = c->r[rn];
  uint32_t eff = U ? base + off : base - off;
  uint32_t addr = P ? eff : base;
  int wb = !P || W;
  if (L) {
    uint32_t v;
    if (sh == 1u) v = ld_half(g, addr);
    else if (sh == 2u) v = ld_sbyte(g, addr);
    else v = ld_shalf(g, addr);
    if (wb && rn != 15u) c->r[rn] = eff;
    wreg(c, (int)rd, v);
    return 3;
  }
  if (sh == 1u) gba_write16(g, addr & ~1u, (uint16_t)(c->r[rd] + (rd == 15u ? 4u : 0u)));
  if (wb && rn != 15u) c->r[rn] = eff;
  return 2;
}

static uint32_t arm_ldm(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rn = (op >> 16) & 15u;
  int P = (op >> 24) & 1u, U = (op >> 23) & 1u, S = (op >> 22) & 1u, W = (op >> 21) & 1u,
      L = (op >> 20) & 1u;
  uint32_t list = op & 0xFFFFu;
  uint32_t n = 0;
  for (int i = 0; i < 16; i++) n += (list >> i) & 1u;
  uint32_t base = c->r[rn];
  uint32_t span = n ? n * 4u : 0x40u;
  if (!list) list = 1u << 15; /* ARM7 empty-list quirk: transfers PC, base +/- 0x40 */
  uint32_t start, nb;
  if (U) { start = base + (P ? 4u : 0u); nb = base + span; }
  else { start = base - span + (P ? 0u : 4u); nb = base - span; }
  int user = S && (!L || !(list & 0x8000u));
  uint32_t a = start;
  if (L) {
    if (W) c->r[rn] = nb;
    for (int i = 0; i < 16; i++) {
      if (!(list & (1u << i))) continue;
      uint32_t v = gba_read32(g, a & ~3u);
      a += 4u;
      if (i == 15) {
        if (S && has_spsr(c)) gba_cpu_set_cpsr(c, c->spsr);
        write_pc(c, v);
      } else if (user) {
        set_user_reg(c, i, v);
      } else {
        c->r[i] = v;
      }
    }
    return n + 2u;
  }
  int first = 1;
  for (int i = 0; i < 16; i++) {
    if (!(list & (1u << i))) continue;
    uint32_t v;
    if (i == 15) v = c->r[15] + 4u;
    else if (user) v = gba_cpu_get_user_reg(c, i);
    else if ((uint32_t)i == rn && W && !first) v = nb;
    else v = c->r[i];
    gba_write32(g, a & ~3u, v);
    a += 4u;
    first = 0;
  }
  if (W) c->r[rn] = nb;
  return n + 1u;
}

static uint32_t exec_arm(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  if ((op >> 28) != 0xEu && !cond_pass(op >> 28, c->cpsr)) return 1;
  switch (c->arm_lut[((op >> 16) & 0xFF0u) | ((op >> 4) & 0xFu)]) {
    case A_DP: return arm_dp(g, op);
    case A_MUL: return arm_mul(g, op);
    case A_MULL: return arm_mull(g, op);
    case A_SWP: return arm_swp(g, op);
    case A_BX: return arm_bx(g, op);
    case A_HALF: return arm_half(g, op);
    case A_MRS: return arm_mrs(g, op);
    case A_MSR: return arm_msr(g, op);
    case A_SDT: return arm_sdt(g, op);
    case A_LDM: return arm_ldm(g, op);
    case A_B: {
      int32_t off = (int32_t)(op << 8) >> 6;
      if (op & (1u << 24)) c->r[14] = c->pc_next;
      write_pc(c, c->r[15] + (uint32_t)off);
      return 3;
    }
    case A_SWI: return do_swi(g, (op >> 16) & 0xFFu);
    default: return do_undefined(g);
  }
}

/* ---------------------------- Thumb execution ---------------------------- */
static uint32_t thumb_alu(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t rd = op & 7u, rs = (op >> 3) & 7u, o = (op >> 6) & 15u;
  uint32_t a = c->r[rd], b = c->r[rs], r, co = cflag(c);
  switch (o) {
    case 0x0: r = a & b; set_nz(c, r); c->r[rd] = r; return 1;
    case 0x1: r = a ^ b; set_nz(c, r); c->r[rd] = r; return 1;
    case 0x2: r = shift_reg(a, 0, b & 0xFFu, co, &co); set_nzc(c, r, co); c->r[rd] = r; return 2;
    case 0x3: r = shift_reg(a, 1, b & 0xFFu, co, &co); set_nzc(c, r, co); c->r[rd] = r; return 2;
    case 0x4: r = shift_reg(a, 2, b & 0xFFu, co, &co); set_nzc(c, r, co); c->r[rd] = r; return 2;
    case 0x5: c->r[rd] = add_core(c, a, b, cflag(c), 1); return 1;
    case 0x6: c->r[rd] = add_core(c, a, ~b, cflag(c), 1); return 1;
    case 0x7: r = shift_reg(a, 3, b & 0xFFu, co, &co); set_nzc(c, r, co); c->r[rd] = r; return 2;
    case 0x8: set_nz(c, a & b); return 1;
    case 0x9: c->r[rd] = add_core(c, 0, ~b, 1, 1); return 1;
    case 0xA: add_core(c, a, ~b, 1, 1); return 1;
    case 0xB: add_core(c, a, b, 0, 1); return 1;
    case 0xC: r = a | b; set_nz(c, r); c->r[rd] = r; return 1;
    case 0xD: r = a * b; set_nz(c, r); c->r[rd] = r; return 1u + mul_cycles(a);
    case 0xE: r = a & ~b; set_nz(c, r); c->r[rd] = r; return 1;
    default: r = ~b; set_nz(c, r); c->r[rd] = r; return 1;
  }
}

static uint32_t thumb_block(hs_gba *g, uint32_t rb, uint32_t list, int load) {
  gba_cpu *c = &g->cpu;
  uint32_t addr = c->r[rb], n = 0;
  for (int i = 0; i < 8; i++) n += (list >> i) & 1u;
  if (!list) { /* empty-list quirk */
    if (load) write_pc(c, gba_read32(g, addr & ~3u));
    else gba_write32(g, addr & ~3u, c->r[15] + 2u);
    c->r[rb] = addr + 0x40u;
    return 3;
  }
  uint32_t nb = addr + n * 4u;
  int first = 1;
  for (int i = 0; i < 8; i++) {
    if (!(list & (1u << i))) continue;
    if (load) c->r[i] = gba_read32(g, addr & ~3u);
    else gba_write32(g, addr & ~3u, ((uint32_t)i == rb && !first) ? nb : c->r[i]);
    addr += 4u;
    first = 0;
  }
  if (!load || !(list & (1u << rb))) c->r[rb] = nb;
  return n + (load ? 2u : 1u);
}

static uint32_t exec_thumb(hs_gba *g, uint32_t op) {
  gba_cpu *c = &g->cpu;
  uint32_t co;
  switch (c->thumb_lut[op >> 6]) {
    case T_SHIFT: {
      uint32_t rd = op & 7u, rs = (op >> 3) & 7u, amt = (op >> 6) & 31u, type = (op >> 11) & 3u;
      uint32_t r = shift_imm(c->r[rs], type, amt, cflag(c), &co);
      set_nzc(c, r, co);
      c->r[rd] = r;
      return 1;
    }
    case T_ADDSUB: {
      uint32_t rd = op & 7u, rs = (op >> 3) & 7u, rn = (op >> 6) & 7u;
      uint32_t b = (op & (1u << 10)) ? rn : c->r[rn];
      c->r[rd] = (op & (1u << 9)) ? add_core(c, c->r[rs], ~b, 1, 1) : add_core(c, c->r[rs], b, 0, 1);
      return 1;
    }
    case T_IMM: {
      uint32_t rd = (op >> 8) & 7u, imm = op & 0xFFu;
      switch ((op >> 11) & 3u) {
        case 0: c->r[rd] = imm; set_nz(c, imm); break;
        case 1: add_core(c, c->r[rd], ~imm, 1, 1); break;
        case 2: c->r[rd] = add_core(c, c->r[rd], imm, 0, 1); break;
        default: c->r[rd] = add_core(c, c->r[rd], ~imm, 1, 1); break;
      }
      return 1;
    }
    case T_ALU: return thumb_alu(g, op);
    case T_HIREG: {
      uint32_t rd = (op & 7u) | ((op >> 4) & 8u), rs = (op >> 3) & 15u;
      switch ((op >> 8) & 3u) {
        case 0: wreg(c, (int)rd, c->r[rd] + c->r[rs]); return rd == 15u ? 3u : 1u;
        case 1: add_core(c, c->r[rd], ~c->r[rs], 1, 1); return 1;
        case 2: wreg(c, (int)rd, c->r[rs]); return rd == 15u ? 3u : 1u;
        default: {
          uint32_t v = c->r[rs];
          gba_cpu_jump(c, v, (int)(v & 1u));
          return 3;
        }
      }
    }
    case T_LDRPC: {
      uint32_t rd = (op >> 8) & 7u;
      c->r[rd] = gba_read32(g, ((c->r[15] & ~2u) + ((op & 0xFFu) << 2)) & ~3u);
      return 3;
    }
    case T_LSREG: {
      uint32_t rd = op & 7u, rb = (op >> 3) & 7u, ro = (op >> 6) & 7u;
      uint32_t a = c->r[rb] + c->r[ro];
      switch ((op >> 10) & 3u) {
        case 0: gba_write32(g, a & ~3u, c->r[rd]); return 2;
        case 1: gba_write8(g, a, (uint8_t)c->r[rd]); return 2;
        case 2: c->r[rd] = ld_word(g, a); return 3;
        default: c->r[rd] = gba_read8(g, a); return 3;
      }
    }
    case T_LSHALF: {
      uint32_t rd = op & 7u, rb = (op >> 3) & 7u, ro = (op >> 6) & 7u;
      uint32_t a = c->r[rb] + c->r[ro];
      switch ((op >> 10) & 3u) {
        case 0: gba_write16(g, a & ~1u, (uint16_t)c->r[rd]); return 2;
        case 1: c->r[rd] = ld_sbyte(g, a); return 3;
        case 2: c->r[rd] = ld_half(g, a); return 3;
        default: c->r[rd] = ld_shalf(g, a); return 3;
      }
    }
    case T_LSIMM: {
      uint32_t rd = op & 7u, rb = (op >> 3) & 7u, imm = (op >> 6) & 31u;
      int B = (op >> 12) & 1u, L = (op >> 11) & 1u;
      uint32_t a = c->r[rb] + (B ? imm : imm << 2);
      if (L) { c->r[rd] = B ? gba_read8(g, a) : ld_word(g, a); return 3; }
      if (B) gba_write8(g, a, (uint8_t)c->r[rd]); else gba_write32(g, a & ~3u, c->r[rd]);
      return 2;
    }
    case T_LSH_IMM: {
      uint32_t rd = op & 7u, rb = (op >> 3) & 7u, a = c->r[rb] + (((op >> 6) & 31u) << 1);
      if (op & (1u << 11)) { c->r[rd] = ld_half(g, a); return 3; }
      gba_write16(g, a & ~1u, (uint16_t)c->r[rd]);
      return 2;
    }
    case T_LSSP: {
      uint32_t rd = (op >> 8) & 7u, a = c->r[13] + ((op & 0xFFu) << 2);
      if (op & (1u << 11)) { c->r[rd] = ld_word(g, a); return 3; }
      gba_write32(g, a & ~3u, c->r[rd]);
      return 2;
    }
    case T_LDADDR: {
      uint32_t rd = (op >> 8) & 7u, imm = (op & 0xFFu) << 2;
      c->r[rd] = ((op & (1u << 11)) ? c->r[13] : (c->r[15] & ~2u)) + imm;
      return 1;
    }
    case T_ADDSP: {
      uint32_t imm = (op & 0x7Fu) << 2;
      c->r[13] = (op & 0x80u) ? c->r[13] - imm : c->r[13] + imm;
      return 1;
    }
    case T_PUSHPOP: {
      uint32_t list = op & 0xFFu, R = (op >> 8) & 1u, n = R;
      for (int i = 0; i < 8; i++) n += (list >> i) & 1u;
      if (op & (1u << 11)) { /* POP */
        uint32_t a = c->r[13];
        for (int i = 0; i < 8; i++) {
          if (list & (1u << i)) { c->r[i] = gba_read32(g, a & ~3u); a += 4u; }
        }
        if (R) { write_pc(c, gba_read32(g, a & ~3u)); a += 4u; }
        c->r[13] = a;
        return n + 2u;
      }
      uint32_t a = c->r[13] - n * 4u;
      c->r[13] = a;
      for (int i = 0; i < 8; i++) {
        if (list & (1u << i)) { gba_write32(g, a & ~3u, c->r[i]); a += 4u; }
      }
      if (R) gba_write32(g, a & ~3u, c->r[14]);
      return n + 1u;
    }
    case T_LDMSTM: return thumb_block(g, (op >> 8) & 7u, op & 0xFFu, (int)((op >> 11) & 1u));
    case T_BCOND: {
      if (!cond_pass((op >> 8) & 15u, c->cpsr)) return 1;
      int32_t off = (int32_t)(int8_t)(op & 0xFFu) * 2;
      write_pc(c, c->r[15] + (uint32_t)off);
      return 3;
    }
    case T_SWI: return do_swi(g, op & 0xFFu);
    case T_B: {
      int32_t off = ((int32_t)(op << 21) >> 20);
      write_pc(c, c->r[15] + (uint32_t)off);
      return 3;
    }
    case T_BL: {
      if (!(op & (1u << 11))) {
        int32_t off = ((int32_t)(op << 21) >> 9);
        c->r[14] = c->r[15] + (uint32_t)off;
        return 1;
      }
      uint32_t nxt = c->pc_next;
      write_pc(c, c->r[14] + ((op & 0x7FFu) << 1));
      c->r[14] = nxt | 1u;
      return 3;
    }
    default: return do_undefined(g);
  }
}

uint32_t gba_cpu_step(hs_gba *g) {
  gba_cpu *c = &g->cpu;
  if (c->halted) return 1;
  uint32_t a = c->pc_next, cyc;
  c->branched = 0;
  c->insn_count++;
  uint32_t ws = (a >= 0x08000000u) ? 1u : 0u;
  if (c->cpsr & CPSR_T) {
    uint32_t op = gba_read16(g, a);
    c->r[15] = a + 4u;
    c->pc_next = a + 2u;
    cyc = exec_thumb(g, op) + ws;
  } else {
    uint32_t op = gba_read32(g, a);
    c->r[15] = a + 8u;
    c->pc_next = a + 4u;
    cyc = exec_arm(g, op) + ws * 2u;
  }
  return cyc;
}
