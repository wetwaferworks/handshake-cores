/* SPDX-License-Identifier: MIT
 * test_asm.h - tiny clean-room ARMv4T / Thumb encoder used by the hs-gba tests
 * to assemble synthetic GBA ROMs directly in C.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#ifndef HS_GBA_TEST_ASM_H
#define HS_GBA_TEST_ASM_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct { uint8_t *mem; uint32_t base, pos; } asmb;

static inline uint32_t here(const asmb *a) { return a->base + a->pos; }
static inline void e32(asmb *a, uint32_t v) { memcpy(a->mem + a->pos, &v, 4); a->pos += 4; }
static inline void e16(asmb *a, uint16_t v) { memcpy(a->mem + a->pos, &v, 2); a->pos += 2; }
static inline void align4(asmb *a) { while (a->pos & 3u) e16(a, 0x46C0u); /* thumb nop (mov r8,r8) */ }
static inline void put32(asmb *a, uint32_t addr, uint32_t v) { memcpy(a->mem + (addr - a->base), &v, 4); }
static inline void put16(asmb *a, uint32_t addr, uint16_t v) { memcpy(a->mem + (addr - a->base), &v, 2); }

/* ---------------- ARM ---------------- */
#define AL 0xEu
enum { AND = 0, EOR, SUB, RSB, ADD, ADC, SBC, RSC, TST, TEQ, CMP, CMN, ORR, MOV, BIC, MVN };
enum { LSL = 0, LSR, ASR, ROR };
enum { EQ = 0, NE, CS, CC, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE };

static inline uint32_t IMM(uint32_t v) {
  for (uint32_t rot = 0; rot < 16u; rot++) {
    uint32_t n = 2u * rot;
    uint32_t imm = n ? ((v << n) | (v >> (32u - n))) : v;
    if (imm < 256u) return (1u << 25) | (rot << 8) | imm;
  }
  fprintf(stderr, "IMM: unencodable 0x%08x\n", v);
  exit(2);
}
static inline uint32_t RSH(uint32_t rm, uint32_t type, uint32_t amt) { return (amt << 7) | (type << 5) | rm; }
static inline uint32_t RSR(uint32_t rm, uint32_t type, uint32_t rs) { return (rs << 8) | (type << 5) | 0x10u | rm; }
static inline uint32_t CC_(uint32_t cond, uint32_t insn) { return (insn & 0x0FFFFFFFu) | (cond << 28); }

static inline uint32_t a_dp(uint32_t opc, uint32_t s, uint32_t rd, uint32_t rn, uint32_t op2) {
  return (AL << 28) | (opc << 21) | (s << 20) | (rn << 16) | (rd << 12) | op2;
}
static inline uint32_t a_mul(uint32_t rd, uint32_t rm, uint32_t rs, uint32_t s) {
  return (AL << 28) | (s << 20) | (rd << 16) | (rs << 8) | 0x90u | rm;
}
static inline uint32_t a_mla(uint32_t rd, uint32_t rm, uint32_t rs, uint32_t rn, uint32_t s) {
  return a_mul(rd, rm, rs, s) | (1u << 21) | (rn << 12);
}
static inline uint32_t a_mull(uint32_t sign, uint32_t acc, uint32_t s, uint32_t lo, uint32_t hi, uint32_t rm, uint32_t rs) {
  return (AL << 28) | 0x00800090u | (sign << 22) | (acc << 21) | (s << 20) | (hi << 16) | (lo << 12) | (rs << 8) | rm;
}
/* single data transfer: imm offset (I=0) */
static inline uint32_t a_sdt(uint32_t L, uint32_t B, uint32_t P, uint32_t U, uint32_t W, uint32_t rd, uint32_t rn, uint32_t off) {
  return (AL << 28) | 0x04000000u | (P << 24) | (U << 23) | (B << 22) | (W << 21) | (L << 20) | (rn << 16) | (rd << 12) | off;
}
/* register offset form: off = RSH(rm,type,amt) */
static inline uint32_t a_sdtr(uint32_t L, uint32_t B, uint32_t P, uint32_t U, uint32_t W, uint32_t rd, uint32_t rn, uint32_t roff) {
  return a_sdt(L, B, P, U, W, rd, rn, roff) | (1u << 25);
}
static inline uint32_t a_ldr(uint32_t rd, uint32_t rn, uint32_t off) { return a_sdt(1, 0, 1, 1, 0, rd, rn, off); }
static inline uint32_t a_str(uint32_t rd, uint32_t rn, uint32_t off) { return a_sdt(0, 0, 1, 1, 0, rd, rn, off); }
/* halfword: sh 1=H 2=SB 3=SH, immediate offset */
static inline uint32_t a_half(uint32_t L, uint32_t P, uint32_t U, uint32_t W, uint32_t sh, uint32_t rd, uint32_t rn, uint32_t imm) {
  return (AL << 28) | (P << 24) | (U << 23) | (1u << 22) | (W << 21) | (L << 20) | (rn << 16) | (rd << 12) |
         ((imm >> 4) << 8) | 0x90u | (sh << 5) | (imm & 15u);
}
static inline uint32_t a_strh(uint32_t rd, uint32_t rn, uint32_t imm) { return a_half(0, 1, 1, 0, 1, rd, rn, imm); }
static inline uint32_t a_ldrh(uint32_t rd, uint32_t rn, uint32_t imm) { return a_half(1, 1, 1, 0, 1, rd, rn, imm); }
static inline uint32_t a_ldm(uint32_t L, uint32_t P, uint32_t U, uint32_t S, uint32_t W, uint32_t rn, uint32_t list) {
  return (AL << 28) | 0x08000000u | (P << 24) | (U << 23) | (S << 22) | (W << 21) | (L << 20) | (rn << 16) | list;
}
static inline uint32_t a_b(uint32_t link, uint32_t from, uint32_t to) {
  return (AL << 28) | 0x0A000000u | (link << 24) | (((to - (from + 8u)) >> 2) & 0xFFFFFFu);
}
static inline uint32_t a_bx(uint32_t rm) { return (AL << 28) | 0x012FFF10u | rm; }
static inline uint32_t a_swi(uint32_t n) { return (AL << 28) | 0x0F000000u | (n << 16); }
static inline uint32_t a_mrs(uint32_t rd, uint32_t spsr) { return (AL << 28) | 0x010F0000u | (spsr << 22) | (rd << 12); }
static inline uint32_t a_msr_r(uint32_t spsr, uint32_t mask, uint32_t rm) { return (AL << 28) | 0x0120F000u | (spsr << 22) | (mask << 16) | rm; }
static inline uint32_t a_msr_i(uint32_t spsr, uint32_t mask, uint32_t v) { return (AL << 28) | 0x0320F000u | (spsr << 22) | (mask << 16) | (IMM(v) & 0xFFFu); }
static inline uint32_t a_swp(uint32_t b, uint32_t rd, uint32_t rm, uint32_t rn) {
  return (AL << 28) | 0x01000090u | (b << 22) | (rn << 16) | (rd << 12) | rm;
}

static inline void a_li(asmb *a, uint32_t rd, uint32_t v) {
  e32(a, a_dp(MOV, 0, rd, 0, IMM(v & 0xFFu)));
  if (v & 0xFF00u) e32(a, a_dp(ORR, 0, rd, rd, IMM(v & 0xFF00u)));
  if (v & 0xFF0000u) e32(a, a_dp(ORR, 0, rd, rd, IMM(v & 0xFF0000u)));
  if (v & 0xFF000000u) e32(a, a_dp(ORR, 0, rd, rd, IMM(v & 0xFF000000u)));
}
/* trace helpers: r13 is the trace pointer in ARM snippets */
static inline void a_tr(asmb *a, uint32_t r) { e32(a, a_sdt(0, 0, 0, 1, 0, r, 13, 4)); }
static inline void a_tf(asmb *a) { e32(a, a_mrs(14, 0)); a_tr(a, 14); }
static inline uint32_t a_end(asmb *a) { uint32_t p = here(a); e32(a, 0xEAFFFFFEu); return p; }

/* ---------------- Thumb ---------------- */
static inline uint16_t t_sh(uint32_t type, uint32_t rd, uint32_t rs, uint32_t imm) { return (uint16_t)((type << 11) | (imm << 6) | (rs << 3) | rd); }
static inline uint16_t t_add3(uint32_t rd, uint32_t rs, uint32_t rn) { return (uint16_t)(0x1800u | (rn << 6) | (rs << 3) | rd); }
static inline uint16_t t_sub3(uint32_t rd, uint32_t rs, uint32_t rn) { return (uint16_t)(0x1A00u | (rn << 6) | (rs << 3) | rd); }
static inline uint16_t t_addi3(uint32_t rd, uint32_t rs, uint32_t imm) { return (uint16_t)(0x1C00u | (imm << 6) | (rs << 3) | rd); }
static inline uint16_t t_i8(uint32_t op, uint32_t rd, uint32_t imm) { return (uint16_t)(0x2000u | (op << 11) | (rd << 8) | imm); }
#define T_MOVI(rd, i) t_i8(0, rd, i)
#define T_CMPI(rd, i) t_i8(1, rd, i)
#define T_ADDI(rd, i) t_i8(2, rd, i)
#define T_SUBI(rd, i) t_i8(3, rd, i)
enum { TA_AND = 0, TA_EOR, TA_LSL, TA_LSR, TA_ASR, TA_ADC, TA_SBC, TA_ROR, TA_TST, TA_NEG, TA_CMP, TA_CMN, TA_ORR, TA_MUL, TA_BIC, TA_MVN };
static inline uint16_t t_alu(uint32_t op, uint32_t rd, uint32_t rs) { return (uint16_t)(0x4000u | (op << 6) | (rs << 3) | rd); }
static inline uint16_t t_hi(uint32_t op, uint32_t rd, uint32_t rs) {
  return (uint16_t)(0x4400u | (op << 8) | ((rd & 8u) << 4) | (rs << 3) | (rd & 7u));
}
static inline uint16_t t_bx(uint32_t rs) { return (uint16_t)(0x4700u | (rs << 3)); }
static inline uint16_t t_ldrpc(uint32_t rd, uint32_t imm8) { return (uint16_t)(0x4800u | (rd << 8) | imm8); }
static inline uint16_t t_lsr(uint32_t L, uint32_t B, uint32_t rd, uint32_t rb, uint32_t ro) { return (uint16_t)(0x5000u | (L << 11) | (B << 10) | (ro << 6) | (rb << 3) | rd); }
static inline uint16_t t_lsx(uint32_t H, uint32_t S, uint32_t rd, uint32_t rb, uint32_t ro) { return (uint16_t)(0x5200u | (H << 11) | (S << 10) | (ro << 6) | (rb << 3) | rd); }
static inline uint16_t t_lsi(uint32_t B, uint32_t L, uint32_t rd, uint32_t rb, uint32_t imm5) { return (uint16_t)(0x6000u | (B << 12) | (L << 11) | (imm5 << 6) | (rb << 3) | rd); }
static inline uint16_t t_lshi(uint32_t L, uint32_t rd, uint32_t rb, uint32_t imm5) { return (uint16_t)(0x8000u | (L << 11) | (imm5 << 6) | (rb << 3) | rd); }
static inline uint16_t t_sprel(uint32_t L, uint32_t rd, uint32_t imm8) { return (uint16_t)(0x9000u | (L << 11) | (rd << 8) | imm8); }
static inline uint16_t t_ldaddr(uint32_t sp, uint32_t rd, uint32_t imm8) { return (uint16_t)(0xA000u | (sp << 11) | (rd << 8) | imm8); }
static inline uint16_t t_addsp(uint32_t neg, uint32_t imm7) { return (uint16_t)(0xB000u | (neg << 7) | imm7); }
static inline uint16_t t_push(uint32_t R, uint32_t list) { return (uint16_t)(0xB400u | (R << 8) | list); }
static inline uint16_t t_pop(uint32_t R, uint32_t list) { return (uint16_t)(0xBC00u | (R << 8) | list); }
static inline uint16_t t_stmia(uint32_t rb, uint32_t list) { return (uint16_t)(0xC000u | (rb << 8) | list); }
static inline uint16_t t_ldmia(uint32_t rb, uint32_t list) { return (uint16_t)(0xC800u | (rb << 8) | list); }
static inline uint16_t t_bc(uint32_t cond, uint32_t from, uint32_t to) { return (uint16_t)(0xD000u | (cond << 8) | (((to - (from + 4u)) >> 1) & 0xFFu)); }
static inline uint16_t t_swi(uint32_t n) { return (uint16_t)(0xDF00u | n); }
static inline uint16_t t_b(uint32_t from, uint32_t to) { return (uint16_t)(0xE000u | (((to - (from + 4u)) >> 1) & 0x7FFu)); }
static inline void t_bl(asmb *a, uint32_t to) {
  uint32_t from = here(a), off = to - (from + 4u);
  e16(a, (uint16_t)(0xF000u | ((off >> 12) & 0x7FFu)));
  e16(a, (uint16_t)(0xF800u | ((off >> 1) & 0x7FFu)));
}
/* thumb trace: r6 is the trace pointer */
static inline void t_tr(asmb *a, uint32_t r) { e16(a, t_lsi(0, 0, r, 6, 0)); e16(a, T_ADDI(6, 4)); }
static inline uint32_t t_end(asmb *a) { uint32_t p = here(a); e16(a, 0xE7FEu); return p; }
/* ldr rd, =literal : emits ldr pc-rel, returns patch position; literal placed by t_lit */
static inline uint32_t t_ldr_lit(asmb *a, uint32_t rd) { uint32_t p = here(a); e16(a, t_ldrpc(rd, 0)); (void)rd; return p; }
static inline void t_lit_patch(asmb *a, uint32_t insn_addr, uint32_t lit_addr) {
  uint16_t op;
  memcpy(&op, a->mem + (insn_addr - a->base), 2);
  uint32_t imm = (lit_addr - ((insn_addr + 4u) & ~2u)) >> 2;
  op = (uint16_t)((op & 0xFF00u) | imm);
  memcpy(a->mem + (insn_addr - a->base), &op, 2);
}
#endif
