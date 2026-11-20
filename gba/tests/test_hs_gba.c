/* SPDX-License-Identifier: MIT
 * test_hs_gba.c - rigorous verification for the hs-gba clean-room core:
 * ARM7TDMI ARM/Thumb torture tests, interworking, banked IRQ entry/return,
 * DMA3, timer cascade, HLE BIOS, saves, and two synthetic GBA ROMs assembled
 * in C (Mode 0 tiles+sprites+Direct Sound; Mode 3 bitmap + Mode 2 affine)
 * with bit-exact video/audio hashes and 640x480 PNG previews.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gba_internal.h"
#include "test_asm.h"

enum { C_ARM, C_THUMB, C_INTER, C_IRQ, C_DMA, C_TIMER, C_BIOS, C_SAVE, C_PPU, C_APU, C_DET, C_N };
static const char *k_cat[C_N] = {"ARM", "Thumb", "Interwork", "IRQ", "DMA", "Timer", "HLE-BIOS",
                                 "Saves", "PPU/Video", "APU/Audio", "Determinism"};
static int g_pass[C_N], g_fail[C_N];

static void check(int cat, int ok, const char *what, uint64_t got, uint64_t want) {
  if (ok) g_pass[cat]++;
  else {
    g_fail[cat]++;
    printf("  FAIL [%s] %s: got 0x%llx want 0x%llx\n", k_cat[cat], what, (unsigned long long)got,
           (unsigned long long)want);
  }
}
#define EQ32(cat, what, got, want) check(cat, (uint32_t)(got) == (uint32_t)(want), what, (uint32_t)(got), (uint32_t)(want))
#define TRUE_(cat, what, cond) check(cat, (cond) ? 1 : 0, what, (uint64_t)((cond) ? 1 : 0), 1)

#define TROM_SIZE 0x40000u
static uint8_t *g_rom;
static hs_gba *g_t;

static uint64_t fnv(uint64_t h, const void *p, size_t n) {
  const uint8_t *b = (const uint8_t *)p;
  for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 0x100000001b3ull; }
  return h;
}

static void snip_begin(asmb *a) {
  memset(g_rom, 0, TROM_SIZE);
  a->mem = g_rom;
  a->base = 0x08000000u;
  a->pos = 0;
}
static int snip_run(uint32_t end_addr, uint32_t max_steps) {
  hs_gba_load_rom(g_t, g_rom, TROM_SIZE);
  for (uint32_t i = 0; i < max_steps; i++) {
    gba_cpu_step(g_t);
    if (g_t->cpu.pc_next == end_addr) return 1;
  }
  return 0;
}
#define TRACE_BASE 0x03001000u
static uint32_t trace(uint32_t i) { return gba_read32(g_t, TRACE_BASE + i * 4u); }

/* ================================ ARM tests ================================ */
static void test_arm_alu(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, 0x7FFFFFFFu); a_li(&a, 1, 1);
  e32(&a, a_dp(ADD, 1, 2, 0, 1)); a_tr(&a, 2); a_tf(&a);
  a_li(&a, 0, 1); a_li(&a, 1, 2);
  e32(&a, a_dp(SUB, 1, 4, 0, 1)); a_tr(&a, 4); a_tf(&a);
  e32(&a, a_dp(SUB, 1, 6, 1, 1)); a_tr(&a, 6); a_tf(&a);
  e32(&a, a_dp(CMP, 1, 0, 1, 0));
  e32(&a, a_dp(ADC, 0, 8, 0, 1)); a_tr(&a, 8);
  e32(&a, a_dp(CMP, 1, 0, 0, 1));
  e32(&a, a_dp(SBC, 0, 9, 1, 0)); a_tr(&a, 9);
  e32(&a, a_dp(CMP, 1, 0, 1, 0));
  e32(&a, a_dp(RSC, 0, 10, 0, 1)); a_tr(&a, 10);
  e32(&a, a_dp(RSB, 0, 11, 0, IMM(10))); a_tr(&a, 11);
  a_li(&a, 12, 0xFFFFFFFFu);
  e32(&a, a_dp(CMN, 1, 0, 12, 0)); a_tf(&a);
  a_li(&a, 2, 0xF0F0u); a_li(&a, 3, 0xFF00u);
  e32(&a, a_dp(AND, 0, 4, 2, 3)); a_tr(&a, 4);
  e32(&a, a_dp(EOR, 0, 4, 2, 3)); a_tr(&a, 4);
  e32(&a, a_dp(ORR, 0, 4, 2, 3)); a_tr(&a, 4);
  e32(&a, a_dp(BIC, 0, 4, 2, 3)); a_tr(&a, 4);
  e32(&a, a_dp(MVN, 1, 4, 0, 2)); a_tr(&a, 4); a_tf(&a);
  e32(&a, a_dp(TST, 1, 0, 2, IMM(0x0F))); a_tf(&a);
  e32(&a, a_dp(TEQ, 1, 0, 2, 2)); a_tf(&a);
  uint32_t end = a_end(&a);
  TRUE_(C_ARM, "alu snippet completes", snip_run(end, 1000));
  EQ32(C_ARM, "ADDS overflow result", trace(0), 0x80000000u);
  EQ32(C_ARM, "ADDS NZCV=1001", trace(1) >> 28, 0x9u);
  EQ32(C_ARM, "SUBS borrow result", trace(2), 0xFFFFFFFFu);
  EQ32(C_ARM, "SUBS borrow NZCV=1000", trace(3) >> 28, 0x8u);
  EQ32(C_ARM, "SUBS equal result", trace(4), 0u);
  EQ32(C_ARM, "SUBS equal NZCV=0110", trace(5) >> 28, 0x6u);
  EQ32(C_ARM, "ADC with carry", trace(6), 4u);
  EQ32(C_ARM, "SBC with borrow", trace(7), 0u);
  EQ32(C_ARM, "RSC", trace(8), 1u);
  EQ32(C_ARM, "RSB imm", trace(9), 9u);
  EQ32(C_ARM, "CMN wrap Z,C", (trace(10) >> 28) & 0x6u, 0x6u);
  EQ32(C_ARM, "AND", trace(11), 0xF000u);
  EQ32(C_ARM, "EOR", trace(12), 0x0FF0u);
  EQ32(C_ARM, "ORR", trace(13), 0xFFF0u);
  EQ32(C_ARM, "BIC", trace(14), 0x00F0u);
  EQ32(C_ARM, "MVNS", trace(15), ~0xF0F0u);
  EQ32(C_ARM, "MVNS N flag", trace(16) >> 31, 1u);
  EQ32(C_ARM, "TST Z", (trace(17) >> 30) & 1u, 1u);
  EQ32(C_ARM, "TEQ self Z", (trace(18) >> 30) & 1u, 1u);
  EQ32(C_ARM, "mode stays SYS", g_t->cpu.cpsr & 0x1Fu, MODE_SYS);
}

static void test_arm_shifter(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, 0x80000001u);
  e32(&a, a_dp(MOV, 1, 1, 0, RSH(0, LSL, 1))); a_tr(&a, 1); a_tf(&a);
  e32(&a, a_dp(MOV, 1, 1, 0, RSH(0, LSR, 0))); a_tr(&a, 1); a_tf(&a);
  e32(&a, a_dp(MOV, 1, 1, 0, RSH(0, ASR, 0))); a_tr(&a, 1); a_tf(&a);
  e32(&a, a_dp(MOV, 1, 1, 0, RSH(0, ROR, 4))); a_tr(&a, 1); a_tf(&a);
  e32(&a, a_dp(CMP, 1, 0, 0, 0));
  e32(&a, a_dp(MOV, 1, 1, 0, RSH(0, ROR, 0))); a_tr(&a, 1); a_tf(&a);
  a_li(&a, 8, 1);
  a_li(&a, 7, 33); e32(&a, a_dp(MOV, 1, 1, 0, RSR(8, LSL, 7))); a_tr(&a, 1); a_tf(&a);
  a_li(&a, 7, 32); e32(&a, a_dp(MOV, 1, 1, 0, RSR(8, LSL, 7))); a_tr(&a, 1); a_tf(&a);
  a_li(&a, 7, 40); e32(&a, a_dp(MOV, 0, 1, 0, RSR(0, ROR, 7))); a_tr(&a, 1);
  a_li(&a, 7, 32); e32(&a, a_dp(MOV, 1, 1, 0, RSR(0, LSR, 7))); a_tr(&a, 1); a_tf(&a);
  a_li(&a, 7, 200); e32(&a, a_dp(MOV, 1, 1, 0, RSR(0, ASR, 7))); a_tr(&a, 1); a_tf(&a);
  a_li(&a, 7, 0x100); e32(&a, a_dp(MOV, 0, 1, 0, RSR(0, LSL, 7))); a_tr(&a, 1);
  e32(&a, a_dp(ADD, 0, 1, 0, RSH(0, LSL, 31))); a_tr(&a, 1);
  e32(&a, a_dp(MOV, 1, 1, 0, IMM(0xF0000000u))); a_tf(&a);
  uint32_t end = a_end(&a);
  TRUE_(C_ARM, "shifter snippet completes", snip_run(end, 1000));
  EQ32(C_ARM, "LSL#1", trace(0), 2u);           EQ32(C_ARM, "LSL#1 C", (trace(1) >> 29) & 1u, 1u);
  EQ32(C_ARM, "LSR#32", trace(2), 0u);          EQ32(C_ARM, "LSR#32 C", (trace(3) >> 29) & 1u, 1u);
  EQ32(C_ARM, "LSR#32 Z", (trace(3) >> 30) & 1u, 1u);
  EQ32(C_ARM, "ASR#32", trace(4), 0xFFFFFFFFu); EQ32(C_ARM, "ASR#32 C", (trace(5) >> 29) & 1u, 1u);
  EQ32(C_ARM, "ROR#4", trace(6), 0x18000000u);  EQ32(C_ARM, "ROR#4 C", (trace(7) >> 29) & 1u, 0u);
  EQ32(C_ARM, "RRX", trace(8), 0xC0000000u);    EQ32(C_ARM, "RRX C", (trace(9) >> 29) & 1u, 1u);
  EQ32(C_ARM, "LSL reg 33", trace(10), 0u);     EQ32(C_ARM, "LSL reg 33 C", (trace(11) >> 29) & 1u, 0u);
  EQ32(C_ARM, "LSL reg 32", trace(12), 0u);     EQ32(C_ARM, "LSL reg 32 C", (trace(13) >> 29) & 1u, 1u);
  EQ32(C_ARM, "ROR reg 40", trace(14), 0x01800000u);
  EQ32(C_ARM, "LSR reg 32", trace(15), 0u);     EQ32(C_ARM, "LSR reg 32 C", (trace(16) >> 29) & 1u, 1u);
  EQ32(C_ARM, "ASR reg 200", trace(17), 0xFFFFFFFFu); EQ32(C_ARM, "ASR reg 200 C", (trace(18) >> 29) & 1u, 1u);
  EQ32(C_ARM, "LSL reg 256 (amt byte 0)", trace(19), 0x80000001u);
  EQ32(C_ARM, "ADD lsl#31", trace(20), 0x80000001u + 0x80000000u);
  EQ32(C_ARM, "MOVS rotated imm carry", (trace(21) >> 29) & 1u, 1u);
}

static void test_arm_mul(void) {
  const uint32_t x = 0x12345678u, y = 0x9ABCDEF0u;
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, x); a_li(&a, 1, y);
  e32(&a, a_mul(2, 0, 1, 0)); a_tr(&a, 2);
  a_li(&a, 3, 5); e32(&a, a_mla(4, 0, 1, 3, 0)); a_tr(&a, 4);
  e32(&a, a_mull(0, 0, 0, 5, 6, 0, 1)); a_tr(&a, 5); a_tr(&a, 6);
  a_li(&a, 7, 1); a_li(&a, 8, 2);
  e32(&a, a_mull(0, 1, 0, 7, 8, 0, 1)); a_tr(&a, 7); a_tr(&a, 8);
  e32(&a, a_mull(1, 0, 0, 9, 10, 0, 1)); a_tr(&a, 9); a_tr(&a, 10);
  a_li(&a, 11, 0xFFFFFFFFu); a_li(&a, 12, 0);
  e32(&a, a_mull(1, 1, 0, 11, 12, 0, 1)); a_tr(&a, 11); a_tr(&a, 12);
  a_li(&a, 0, 0); e32(&a, a_mul(2, 0, 1, 1)); a_tf(&a);
  a_li(&a, 0, 0xFFFFFFFFu); a_li(&a, 1, 0xFFFFFFFFu);
  e32(&a, a_mull(1, 0, 1, 2, 3, 0, 1)); a_tr(&a, 2); a_tr(&a, 3); a_tf(&a);
  uint32_t end = a_end(&a);
  TRUE_(C_ARM, "mul snippet completes", snip_run(end, 1000));
  uint64_t u = (uint64_t)x * y;
  int64_t s = (int64_t)(int32_t)x * (int32_t)y;
  uint64_t ua = u + ((2ull << 32) | 1u);
  uint64_t sa = (uint64_t)s + 0xFFFFFFFFull;
  EQ32(C_ARM, "MUL", trace(0), x * y);
  EQ32(C_ARM, "MLA", trace(1), x * y + 5u);
  EQ32(C_ARM, "UMULL lo", trace(2), (uint32_t)u);  EQ32(C_ARM, "UMULL hi", trace(3), (uint32_t)(u >> 32));
  EQ32(C_ARM, "UMLAL lo", trace(4), (uint32_t)ua); EQ32(C_ARM, "UMLAL hi", trace(5), (uint32_t)(ua >> 32));
  EQ32(C_ARM, "SMULL lo", trace(6), (uint32_t)s);  EQ32(C_ARM, "SMULL hi", trace(7), (uint32_t)((uint64_t)s >> 32));
  EQ32(C_ARM, "SMLAL lo", trace(8), (uint32_t)sa); EQ32(C_ARM, "SMLAL hi", trace(9), (uint32_t)(sa >> 32));
  EQ32(C_ARM, "MULS zero Z", (trace(10) >> 30) & 1u, 1u);
  EQ32(C_ARM, "SMULLS -1*-1 lo", trace(11), 1u); EQ32(C_ARM, "SMULLS -1*-1 hi", trace(12), 0u);
  EQ32(C_ARM, "SMULLS N=0,Z=0", trace(13) >> 30, 0u);
}

static void test_arm_ldst(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 10, 0x03002000u); a_li(&a, 0, 0x11223344u);
  e32(&a, a_str(0, 10, 0));
  e32(&a, a_sdt(0, 0, 1, 1, 1, 0, 10, 4)); a_tr(&a, 10);
  e32(&a, a_sdt(1, 0, 0, 0, 0, 1, 10, 4)); a_tr(&a, 1); a_tr(&a, 10);
  e32(&a, a_sdt(1, 1, 1, 1, 0, 2, 10, 1)); a_tr(&a, 2);
  e32(&a, a_ldr(3, 10, 1)); a_tr(&a, 3);
  a_li(&a, 4, 0xAB); e32(&a, a_sdt(0, 1, 1, 1, 0, 4, 10, 2));
  e32(&a, a_ldr(5, 10, 0)); a_tr(&a, 5);
  a_li(&a, 6, 1); e32(&a, a_sdtr(1, 0, 1, 1, 0, 7, 10, RSH(6, LSL, 2))); a_tr(&a, 7);
  a_li(&a, 0, 0xFFFF8081u); e32(&a, a_strh(0, 10, 8));
  e32(&a, a_ldrh(1, 10, 8)); a_tr(&a, 1);
  e32(&a, a_half(1, 1, 1, 0, 3, 2, 10, 8)); a_tr(&a, 2);
  e32(&a, a_half(1, 1, 1, 0, 2, 3, 10, 8)); a_tr(&a, 3);
  e32(&a, a_half(1, 1, 1, 0, 2, 4, 10, 9)); a_tr(&a, 4);
  e32(&a, a_ldrh(5, 10, 9)); a_tr(&a, 5);
  e32(&a, a_half(1, 1, 1, 0, 3, 5, 10, 9)); a_tr(&a, 5);
  e32(&a, a_half(1, 1, 1, 1, 1, 6, 10, 8)); a_tr(&a, 10);
  a_li(&a, 0, 0xCAFEBABEu);
  e32(&a, a_swp(0, 1, 0, 10)); a_tr(&a, 1);
  e32(&a, a_ldr(2, 10, 0)); a_tr(&a, 2);
  a_li(&a, 0, 0x5Au);
  e32(&a, a_swp(1, 3, 0, 10)); a_tr(&a, 3);
  e32(&a, a_ldr(2, 10, 0)); a_tr(&a, 2);
  uint32_t end = a_end(&a);
  TRUE_(C_ARM, "ldst snippet completes", snip_run(end, 1000));
  EQ32(C_ARM, "STR pre-index writeback", trace(0), 0x03002004u);
  EQ32(C_ARM, "LDR post-index value", trace(1), 0x11223344u);
  EQ32(C_ARM, "LDR post-index base", trace(2), 0x03002000u);
  EQ32(C_ARM, "LDRB", trace(3), 0x33u);
  EQ32(C_ARM, "LDR unaligned rotate", trace(4), 0x44112233u);
  EQ32(C_ARM, "STRB merge", trace(5), 0x11AB3344u);
  EQ32(C_ARM, "LDR reg lsl#2", trace(6), 0x11223344u);
  EQ32(C_ARM, "LDRH", trace(7), 0x8081u);
  EQ32(C_ARM, "LDRSH", trace(8), 0xFFFF8081u);
  EQ32(C_ARM, "LDRSB", trace(9), 0xFFFFFF81u);
  EQ32(C_ARM, "LDRSB hi", trace(10), 0xFFFFFF80u);
  EQ32(C_ARM, "LDRH misaligned rotate", trace(11), 0x81000080u);
  EQ32(C_ARM, "LDRSH misaligned -> LDRSB", trace(12), 0xFFFFFF80u);
  EQ32(C_ARM, "LDRH pre writeback", trace(13), 0x03002008u);
  EQ32(C_ARM, "SWP old", trace(14), 0x00008081u);
  EQ32(C_ARM, "SWP new", trace(15), 0xCAFEBABEu);
  EQ32(C_ARM, "SWPB old", trace(16), 0xBEu);
  EQ32(C_ARM, "SWPB new", trace(17), 0xCAFEBA5Au);
}

static void test_arm_block(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, 0x11); a_li(&a, 1, 0x22); a_li(&a, 2, 0x33); a_li(&a, 3, 0x44);
  a_li(&a, 10, 0x03003000u);
  e32(&a, a_ldm(0, 0, 1, 0, 1, 10, 0x000Fu)); a_tr(&a, 10);
  e32(&a, a_ldm(0, 1, 0, 0, 1, 10, 0x0003u)); a_tr(&a, 10);
  e32(&a, a_ldm(1, 0, 1, 0, 0, 10, 0x0030u)); a_tr(&a, 4); a_tr(&a, 5);
  a_li(&a, 11, 0x03003000u);
  e32(&a, a_ldm(1, 1, 1, 0, 1, 11, 0x00C0u)); a_tr(&a, 6); a_tr(&a, 7); a_tr(&a, 11);
  e32(&a, a_ldm(1, 0, 0, 0, 1, 11, 0x0300u)); a_tr(&a, 8); a_tr(&a, 9); a_tr(&a, 11);
  a_li(&a, 12, 0x03003100u);
  e32(&a, a_ldm(0, 0, 1, 0, 1, 12, 0x1001u));
  a_li(&a, 4, 0x03003104u); e32(&a, a_ldr(5, 4, 0)); a_tr(&a, 5);
  a_li(&a, 12, 0x03003300u);
  e32(&a, a_ldm(0, 0, 1, 0, 1, 12, 0x1000u));
  a_li(&a, 4, 0x03003300u); e32(&a, a_ldr(5, 4, 0)); a_tr(&a, 5);
  uint32_t end = a_end(&a);
  TRUE_(C_ARM, "block snippet completes", snip_run(end, 1000));
  EQ32(C_ARM, "STMIA! wb", trace(0), 0x03003010u);
  EQ32(C_ARM, "STMDB! wb", trace(1), 0x03003008u);
  EQ32(C_ARM, "LDMIA r4", trace(2), 0x11u); EQ32(C_ARM, "LDMIA r5", trace(3), 0x22u);
  EQ32(C_ARM, "LDMIB r6", trace(4), 0x22u); EQ32(C_ARM, "LDMIB r7", trace(5), 0x11u);
  EQ32(C_ARM, "LDMIB! wb", trace(6), 0x03003008u);
  EQ32(C_ARM, "LDMDA r8", trace(7), 0x22u); EQ32(C_ARM, "LDMDA r9", trace(8), 0x11u);
  EQ32(C_ARM, "LDMDA! wb", trace(9), 0x03003000u);
  EQ32(C_ARM, "STM base-in-list (not first) stores new base", trace(10), 0x03003108u);
  EQ32(C_ARM, "STM base-in-list (first) stores old base", trace(11), 0x03003300u);
  EQ32(C_ARM, "STMIA mem[0]", gba_read32(g_t, 0x03003000u), 0x11u);
  EQ32(C_ARM, "STMDB mem[3]", gba_read32(g_t, 0x0300300Cu), 0x22u);
}

static void test_arm_psr_banks_cond(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, 0xF0000000u);
  e32(&a, a_msr_r(0, 8, 0));
  e32(&a, a_mrs(1, 0)); a_tr(&a, 1);
  e32(&a, a_msr_i(0, 1, 0xD2));
  a_li(&a, 13, 0x03007000u); a_li(&a, 14, 0x1234u);
  e32(&a, a_mrs(2, 0));
  e32(&a, a_msr_i(1, 9, 0x1F));
  e32(&a, a_mrs(3, 1));
  e32(&a, a_msr_i(0, 1, 0x1F));
  a_tr(&a, 2); a_tr(&a, 3);
  e32(&a, a_dp(MOV, 0, 4, 0, 13)); a_tr(&a, 4);
  e32(&a, a_dp(MOV, 0, 4, 0, 14)); a_tr(&a, 4);
  a_li(&a, 8, 0x88);
  e32(&a, a_msr_i(0, 1, 0xD1));
  a_li(&a, 8, 0x99);
  e32(&a, a_msr_i(0, 1, 0x1F));
  a_tr(&a, 8);
  e32(&a, a_msr_i(0, 1, 0xD1));
  e32(&a, a_dp(MOV, 0, 0, 0, 8));
  e32(&a, a_msr_i(0, 1, 0x1F));
  a_tr(&a, 0);
  for (uint32_t r = 1; r <= 8; r++) a_li(&a, r, 0);
  a_li(&a, 0, 5);
  e32(&a, a_dp(CMP, 1, 0, 0, IMM(5)));
  e32(&a, CC_(EQ, a_dp(MOV, 0, 1, 0, IMM(1)))); e32(&a, CC_(NE, a_dp(MOV, 0, 2, 0, IMM(1))));
  e32(&a, CC_(GT, a_dp(MOV, 0, 3, 0, IMM(1)))); e32(&a, CC_(GE, a_dp(MOV, 0, 4, 0, IMM(1))));
  e32(&a, CC_(LT, a_dp(MOV, 0, 5, 0, IMM(1)))); e32(&a, CC_(LE, a_dp(MOV, 0, 6, 0, IMM(1))));
  e32(&a, CC_(HI, a_dp(MOV, 0, 7, 0, IMM(1)))); e32(&a, CC_(LS, a_dp(MOV, 0, 8, 0, IMM(1))));
  for (uint32_t r = 1; r <= 8; r++) a_tr(&a, r);
  uint32_t bl_at = here(&a); e32(&a, 0);
  a_tr(&a, 0);
  uint32_t end = a_end(&a);
  uint32_t fn = here(&a);
  a_li(&a, 0, 0x77);
  e32(&a, a_dp(MOV, 0, 15, 0, 14));
  put32(&a, bl_at, a_b(1, bl_at, fn));
  TRUE_(C_ARM, "psr snippet completes", snip_run(end, 2000));
  EQ32(C_ARM, "MSR flags / MRS", trace(0), 0xF000001Fu);
  EQ32(C_ARM, "MRS in IRQ mode", trace(1), 0xF00000D2u);
  EQ32(C_ARM, "MSR/MRS SPSR_irq", trace(2), 0x0000001Fu);
  EQ32(C_ARM, "banked SP restored after IRQ round trip", trace(3), TRACE_BASE + 12u);
  EQ32(C_ARM, "banked LR untouched", trace(4), 0u);
  EQ32(C_ARM, "FIQ r8 bank isolates user r8", trace(5), 0x88u);
  EQ32(C_ARM, "FIQ r8 bank retained", trace(6), 0x99u);
  static const uint32_t want[8] = {1, 0, 0, 1, 0, 1, 0, 1};
  for (int i = 0; i < 8; i++) EQ32(C_ARM, "cond EQ/NE/GT/GE/LT/LE/HI/LS", trace(7u + (uint32_t)i), want[i]);
  EQ32(C_ARM, "BL + MOV pc,lr", trace(15), 0x77u);
  EQ32(C_ARM, "IRQ bank sp kept", g_t->cpu.bank_sp[2], 0x03007000u);
}

/* =============================== Thumb tests =============================== */
static void patch_li(asmb *a, uint32_t at, uint32_t rd, uint32_t v) {
  asmb p = *a; p.pos = at - a->base;
  a_li(&p, rd, v);
}
static void test_thumb(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 6, TRACE_BASE);
  uint32_t li_at = here(&a);
  for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_bx(0));
  uint32_t tstart = here(&a);
  patch_li(&a, li_at, 0, tstart | 1u);
  e16(&a, T_MOVI(0, 200)); e16(&a, T_MOVI(1, 100));
  e16(&a, t_add3(2, 0, 1)); t_tr(&a, 2);
  e16(&a, t_sub3(3, 1, 0)); t_tr(&a, 3);
  e16(&a, T_MOVI(5, 0)); { uint32_t p = here(&a); e16(&a, 0); e16(&a, T_MOVI(5, 1)); put16(&a, p, t_bc(CS, p, here(&a))); }
  t_tr(&a, 5);
  e16(&a, t_sh(0, 4, 0, 24)); t_tr(&a, 4);
  e16(&a, t_sh(1, 5, 4, 28)); t_tr(&a, 5);
  e16(&a, t_sh(2, 5, 4, 28)); t_tr(&a, 5);
  e16(&a, T_ADDI(0, 55)); e16(&a, T_SUBI(0, 5)); t_tr(&a, 0);
  e16(&a, T_MOVI(1, 0xF0)); e16(&a, T_MOVI(2, 0x3C));
  e16(&a, t_alu(TA_AND, 1, 2)); t_tr(&a, 1);
  e16(&a, T_MOVI(1, 0xF0)); e16(&a, t_alu(TA_EOR, 1, 2)); t_tr(&a, 1);
  e16(&a, t_alu(TA_ORR, 1, 2)); t_tr(&a, 1);
  e16(&a, t_alu(TA_BIC, 1, 2)); t_tr(&a, 1);
  e16(&a, t_alu(TA_MVN, 3, 2)); t_tr(&a, 3);
  e16(&a, t_alu(TA_NEG, 3, 2)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 7)); e16(&a, t_alu(TA_MUL, 3, 2)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 1)); e16(&a, T_MOVI(4, 1)); e16(&a, t_alu(TA_ROR, 3, 4)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 1)); e16(&a, T_MOVI(4, 33)); e16(&a, t_alu(TA_LSL, 3, 4)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 5)); e16(&a, T_MOVI(4, 3)); e16(&a, t_alu(TA_CMP, 4, 4));
  e16(&a, t_alu(TA_ADC, 3, 4)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 10)); e16(&a, t_alu(TA_CMP, 3, 4));
  e16(&a, t_alu(TA_SBC, 3, 4)); t_tr(&a, 3);
  e16(&a, T_MOVI(3, 0x80)); e16(&a, T_MOVI(4, 1)); e16(&a, t_alu(TA_ASR, 3, 4)); t_tr(&a, 3);
  e16(&a, T_MOVI(4, 2)); e16(&a, t_alu(TA_LSR, 3, 4)); t_tr(&a, 3);
  e16(&a, t_hi(2, 8, 0)); e16(&a, t_hi(0, 8, 0)); e16(&a, t_hi(2, 2, 8)); t_tr(&a, 2);
  e16(&a, t_hi(1, 8, 2)); e16(&a, T_MOVI(5, 0)); { uint32_t p = here(&a); e16(&a, 0); e16(&a, T_MOVI(5, 1)); put16(&a, p, t_bc(NE, p, here(&a))); }
  t_tr(&a, 5);
  e16(&a, T_MOVI(0, 0)); e16(&a, T_MOVI(1, 10));
  uint32_t loop = here(&a);
  e16(&a, t_add3(0, 0, 1)); e16(&a, T_SUBI(1, 1));
  { uint32_t p = here(&a); e16(&a, t_bc(NE, p, loop)); }
  t_tr(&a, 0);
  e16(&a, T_MOVI(4, 42)); e16(&a, T_MOVI(0, 3));
  uint32_t bl_at = here(&a); e16(&a, 0); e16(&a, 0);
  t_tr(&a, 0); t_tr(&a, 4);
  uint32_t lit1 = t_ldr_lit(&a, 7);
  t_tr(&a, 7);
  e16(&a, T_MOVI(0, 0x81)); e16(&a, t_lsi(1, 0, 0, 7, 1)); e16(&a, t_lsi(1, 1, 1, 7, 1)); t_tr(&a, 1);
  e16(&a, T_MOVI(2, 1)); e16(&a, t_lsx(1, 1, 3, 7, 2)); t_tr(&a, 3);
  e16(&a, t_lshi(0, 0, 7, 2)); e16(&a, t_lshi(1, 4, 7, 2)); t_tr(&a, 4);
  e16(&a, T_MOVI(0, 0x80)); e16(&a, t_sh(0, 0, 0, 8)); e16(&a, t_lshi(0, 0, 7, 3));
  e16(&a, T_MOVI(2, 6)); e16(&a, t_lsx(1, 1, 5, 7, 2)); t_tr(&a, 5);
  e16(&a, t_lsx(1, 0, 5, 7, 2)); t_tr(&a, 5);
  e16(&a, t_lsi(0, 0, 0, 7, 2)); e16(&a, t_lsi(0, 1, 1, 7, 2)); t_tr(&a, 1);
  e16(&a, T_MOVI(2, 12)); e16(&a, t_lsr(0, 0, 0, 7, 2)); e16(&a, t_lsr(1, 0, 1, 7, 2)); t_tr(&a, 1);
  e16(&a, t_lsr(0, 1, 0, 7, 2)); e16(&a, t_lsr(1, 1, 1, 7, 2)); t_tr(&a, 1);
  e16(&a, t_addsp(1, 2)); e16(&a, t_sprel(0, 0, 1)); e16(&a, t_sprel(1, 1, 1)); t_tr(&a, 1);
  e16(&a, t_ldaddr(1, 2, 2)); t_tr(&a, 2);
  e16(&a, t_addsp(0, 2));
  e16(&a, t_ldaddr(1, 2, 0)); t_tr(&a, 2);
  uint32_t ldpc = here(&a); e16(&a, t_ldaddr(0, 3, 1)); t_tr(&a, 3);
  e16(&a, T_MOVI(0, 1)); e16(&a, T_MOVI(1, 2)); e16(&a, T_MOVI(2, 3));
  e16(&a, t_addi3(3, 7, 0)); e16(&a, t_stmia(3, 0x07)); t_tr(&a, 3);
  e16(&a, t_addi3(3, 7, 0)); e16(&a, t_ldmia(3, 0x30)); t_tr(&a, 4); t_tr(&a, 5); t_tr(&a, 3);
  uint32_t end = t_end(&a);
  uint32_t fn = here(&a);
  e16(&a, t_push(1, 0x10)); e16(&a, T_MOVI(4, 9)); e16(&a, t_add3(0, 0, 4)); e16(&a, t_pop(1, 0x10));
  { asmb p = a; p.pos = bl_at - a.base; t_bl(&p, fn); }
  align4(&a);
  uint32_t lit_addr = here(&a); e32(&a, 0x03002000u);
  t_lit_patch(&a, lit1, lit_addr);
  TRUE_(C_THUMB, "thumb snippet completes", snip_run(end, 4000));
  EQ32(C_THUMB, "ADD reg", trace(0), 300u);
  EQ32(C_THUMB, "SUB reg", trace(1), (uint32_t)-100);
  EQ32(C_THUMB, "SUB borrow -> BCS not taken", trace(2), 1u);
  EQ32(C_THUMB, "LSL imm", trace(3), 0xC8000000u);
  EQ32(C_THUMB, "LSR imm", trace(4), 0xCu);
  EQ32(C_THUMB, "ASR imm", trace(5), 0xFFFFFFFCu);
  EQ32(C_THUMB, "ADD/SUB imm8", trace(6), 250u);
  EQ32(C_THUMB, "AND", trace(7), 0x30u); EQ32(C_THUMB, "EOR", trace(8), 0xCCu);
  EQ32(C_THUMB, "ORR", trace(9), 0xFCu); EQ32(C_THUMB, "BIC", trace(10), 0xC0u);
  EQ32(C_THUMB, "MVN", trace(11), ~0x3Cu); EQ32(C_THUMB, "NEG", trace(12), (uint32_t)-0x3C);
  EQ32(C_THUMB, "MUL", trace(13), 420u); EQ32(C_THUMB, "ROR reg", trace(14), 0x80000000u);
  EQ32(C_THUMB, "LSL reg 33", trace(15), 0u); EQ32(C_THUMB, "ADC", trace(16), 9u);
  EQ32(C_THUMB, "SBC", trace(17), 7u); EQ32(C_THUMB, "ASR reg", trace(18), 0x40u);
  EQ32(C_THUMB, "LSR reg", trace(19), 0x10u);
  EQ32(C_THUMB, "hi-reg MOV/ADD", trace(20), 500u);
  EQ32(C_THUMB, "hi-reg CMP", trace(21), 1u);
  EQ32(C_THUMB, "Bcc loop sum", trace(22), 55u);
  EQ32(C_THUMB, "BL + PUSH{lr}/POP{pc}", trace(23), 12u);
  EQ32(C_THUMB, "POP restores r4", trace(24), 42u);
  EQ32(C_THUMB, "LDR pc-relative", trace(25), 0x03002000u);
  EQ32(C_THUMB, "STRB/LDRB imm", trace(26), 0x81u);
  EQ32(C_THUMB, "LDSB reg", trace(27), 0xFFFFFF81u);
  EQ32(C_THUMB, "STRH/LDRH imm", trace(28), 0x81u);
  EQ32(C_THUMB, "LDSH reg", trace(29), 0xFFFF8000u);
  EQ32(C_THUMB, "LDRH reg", trace(30), 0x8000u);
  EQ32(C_THUMB, "STR/LDR imm", trace(31), 0x8000u);
  EQ32(C_THUMB, "STR/LDR reg", trace(32), 0x8000u);
  EQ32(C_THUMB, "STRB/LDRB reg", trace(33), 0x00u);
  EQ32(C_THUMB, "SP-relative STR/LDR", trace(34), 0x8000u);
  EQ32(C_THUMB, "ADD rd,SP,#imm", trace(35), 0x03007F00u);
  EQ32(C_THUMB, "ADD SP,#imm restore", trace(36), 0x03007F00u);
  EQ32(C_THUMB, "ADD rd,PC,#imm", trace(37), ((ldpc + 4u) & ~2u) + 4u);
  EQ32(C_THUMB, "STMIA! wb", trace(38), 0x0300200Cu);
  EQ32(C_THUMB, "LDMIA r4", trace(39), 1u); EQ32(C_THUMB, "LDMIA r5", trace(40), 2u);
  EQ32(C_THUMB, "LDMIA! wb", trace(41), 0x03002008u);
  EQ32(C_THUMB, "still in Thumb state", (g_t->cpu.cpsr >> 5) & 1u, 1u);
}

/* ============================= interworking ============================= */
static void test_interwork(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 13, TRACE_BASE + 0x100u);
  a_li(&a, 6, TRACE_BASE);
  uint32_t li_at = here(&a);
  for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_bx(0));
  uint32_t t1 = here(&a);
  e16(&a, T_MOVI(1, 0x11)); t_tr(&a, 1);
  uint32_t lit = t_ldr_lit(&a, 2);
  e16(&a, t_bx(2));
  align4(&a);
  uint32_t armpart = here(&a);
  e32(&a, a_mrs(3, 0)); e32(&a, a_str(3, 6, 0)); e32(&a, a_dp(ADD, 0, 6, 6, IMM(4)));
  uint32_t li2 = here(&a);
  for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_bx(4));
  uint32_t t2 = here(&a);
  e16(&a, T_MOVI(1, 0x22)); t_tr(&a, 1);
  uint32_t end = t_end(&a);
  align4(&a);
  uint32_t lit_addr = here(&a); e32(&a, armpart);
  t_lit_patch(&a, lit, lit_addr);
  patch_li(&a, li_at, 0, t1 | 1u);
  patch_li(&a, li2, 4, t2 | 1u);
  TRUE_(C_INTER, "interwork snippet completes", snip_run(end, 1000));
  EQ32(C_INTER, "ARM->Thumb BX executes Thumb", trace(0), 0x11u);
  EQ32(C_INTER, "Thumb->ARM BX clears T", (trace(1) >> 5) & 1u, 0u);
  EQ32(C_INTER, "ARM->Thumb again", trace(2), 0x22u);
  EQ32(C_INTER, "final T set", (g_t->cpu.cpsr >> 5) & 1u, 1u);
  EQ32(C_INTER, "final pc halfword aligned", g_t->cpu.pc_next & 1u, 0u);
}

/* ========================= IRQ / DMA / Timer (sys) ========================= */
static void run_frames(int n) { for (int i = 0; i < n; i++) hs_gba_step_cpu_phase(g_t, 0); }

static void test_irq_banked(void) {
  asmb a; snip_begin(&a);
  a_li(&a, 0, 0x03007FFCu);
  uint32_t li_h = here(&a); for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_str(1, 0, 0));
  a_li(&a, 0, 0x04000000u);
  a_li(&a, 1, 0x00C0FF00u); e32(&a, a_str(1, 0, 0x100));
  e32(&a, a_dp(ADD, 0, 2, 0, IMM(0x200)));
  a_li(&a, 1, 8); e32(&a, a_strh(1, 2, 0));
  a_li(&a, 1, 1); e32(&a, a_strh(1, 2, 8));
  a_li(&a, 4, 0x03004000u);
  e32(&a, a_dp(MOV, 0, 5, 0, 13));
  uint32_t loop = here(&a);
  e32(&a, a_ldr(1, 4, 0));
  e32(&a, a_dp(CMP, 1, 0, 1, IMM(3)));
  { uint32_t p = here(&a); e32(&a, CC_(LT, a_b(0, p, loop))); }
  a_li(&a, 1, 0); e32(&a, a_strh(1, 2, 8));
  e32(&a, a_dp(MOV, 0, 6, 0, 13));
  uint32_t end = a_end(&a);
  uint32_t h = here(&a);
  e32(&a, a_mrs(1, 0)); a_li(&a, 2, 0x03004010u); e32(&a, a_str(1, 2, 0));
  e32(&a, a_dp(MOV, 0, 1, 0, 13)); e32(&a, a_str(1, 2, 4));
  e32(&a, a_mrs(1, 1)); e32(&a, a_str(1, 2, 8));
  a_li(&a, 2, 0x03004000u); e32(&a, a_ldr(1, 2, 0)); e32(&a, a_dp(ADD, 0, 1, 1, IMM(1))); e32(&a, a_str(1, 2, 0));
  a_li(&a, 0, 0x04000202u); a_li(&a, 1, 8); e32(&a, a_strh(1, 0, 0));
  e32(&a, a_bx(14));
  patch_li(&a, li_h, 1, h);
  hs_gba_load_rom(g_t, g_rom, TROM_SIZE);
  int done = 0;
  for (int f = 0; f < 4 && !done; f++) { hs_gba_step_cpu_phase(g_t, 0); done = g_t->cpu.pc_next == end; }
  TRUE_(C_IRQ, "IRQ program reaches end", done);
  uint32_t cnt = gba_read32(g_t, 0x03004000u);
  TRUE_(C_IRQ, "timer IRQ handler ran >= 3x", cnt >= 3u);
  EQ32(C_IRQ, "handler runs in IRQ mode", gba_read32(g_t, 0x03004010u) & 0x1Fu, MODE_IRQ);
  EQ32(C_IRQ, "handler has I masked", (gba_read32(g_t, 0x03004010u) >> 7) & 1u, 1u);
  EQ32(C_IRQ, "handler ARM state", (gba_read32(g_t, 0x03004010u) >> 5) & 1u, 0u);
  EQ32(C_IRQ, "banked SP_irq after stub push", gba_read32(g_t, 0x03004014u), 0x03007FA0u - 24u);
  EQ32(C_IRQ, "SPSR_irq holds SYS mode", gba_read32(g_t, 0x03004018u) & 0x1Fu, MODE_SYS);
  EQ32(C_IRQ, "returns to SYS", g_t->cpu.cpsr & 0x1Fu, MODE_SYS);
  EQ32(C_IRQ, "SYS sp preserved", g_t->cpu.r[6], g_t->cpu.r[5]);
  EQ32(C_IRQ, "SYS sp value", g_t->cpu.r[6], 0x03007F00u);
  EQ32(C_IRQ, "IRQ sp restored", g_t->cpu.bank_sp[2], 0x03007FA0u);
}

static void test_dma3(void) {
  asmb a; snip_begin(&a);
  const uint32_t data = 0x08001000u;
  for (uint32_t i = 0; i < 64; i++) put16(&a, data + i * 2u, (uint16_t)(0x1000u + i * 37u));
  put32(&a, 0x08001100u, 0xA5A5C3C3u);
  a_li(&a, 4, 0x040000D4u);
  a_li(&a, 1, data); e32(&a, a_str(1, 4, 0));
  a_li(&a, 1, 0x06000000u); e32(&a, a_str(1, 4, 4));
  a_li(&a, 1, 0xC0000040u); e32(&a, a_str(1, 4, 8));
  a_li(&a, 1, data); e32(&a, a_str(1, 4, 0));
  a_li(&a, 1, 0x06008000u); e32(&a, a_str(1, 4, 4));
  a_li(&a, 1, 0x84000010u); e32(&a, a_str(1, 4, 8));
  a_li(&a, 1, 0x08001100u); e32(&a, a_str(1, 4, 0));
  a_li(&a, 1, 0x06009000u); e32(&a, a_str(1, 4, 4));
  a_li(&a, 1, 0x85000008u); e32(&a, a_str(1, 4, 8));
  a_li(&a, 1, 0x0600A010u); e32(&a, a_str(1, 4, 4));
  a_li(&a, 1, data); e32(&a, a_str(1, 4, 0));
  a_li(&a, 1, 0x80200004u); e32(&a, a_str(1, 4, 8));
  uint32_t end = a_end(&a);
  TRUE_(C_DMA, "dma snippet completes", snip_run(end, 1000));
  int ok16 = 1, ok32 = 1, okf = 1;
  for (uint32_t i = 0; i < 64; i++) ok16 &= gba_read16(g_t, 0x06000000u + i * 2u) == (uint16_t)(0x1000u + i * 37u);
  for (uint32_t i = 0; i < 32; i++) ok32 &= gba_read16(g_t, 0x06008000u + i * 2u) == (uint16_t)(0x1000u + i * 37u);
  for (uint32_t i = 0; i < 8; i++) okf &= gba_read32(g_t, 0x06009000u + i * 4u) == 0xA5A5C3C3u;
  TRUE_(C_DMA, "DMA3 16-bit ROM->VRAM 64 units", ok16);
  TRUE_(C_DMA, "DMA3 32-bit ROM->VRAM 16 units", ok32);
  TRUE_(C_DMA, "DMA3 fixed-source fill", okf);
  EQ32(C_DMA, "DMA3 dest-decrement [0]", gba_read16(g_t, 0x0600A010u), 0x1000u);
  EQ32(C_DMA, "DMA3 dest-decrement [-3]", gba_read16(g_t, 0x0600A00Au), 0x1000u + 3u * 37u);
  EQ32(C_DMA, "DMA3 enable auto-cleared", gba_read16(g_t, 0x040000DEu) >> 15, 0u);
  EQ32(C_DMA, "DMA3 IRQ flag raised", (g_t->ifl >> 11) & 1u, 1u);
  hs_gba_reset(g_t);
  gba_write32(g_t, 0x03000000u, 0x7C1F03E0u);
  gba_write32(g_t, 0x040000B0u, 0x03000000u);
  gba_write32(g_t, 0x040000B4u, 0x05000010u);
  gba_write32(g_t, 0x040000B8u, 0x96600001u); /* enable, vblank, repeat, 32-bit, dst reload */
  g_t->cpu.halted = 1; g_t->ime = 0;
  run_frames(1);
  EQ32(C_DMA, "VBlank DMA0 executed", gba_read32(g_t, 0x05000010u), 0x7C1F03E0u);
  EQ32(C_DMA, "repeat DMA stays enabled", gba_read16(g_t, 0x040000BAu) >> 15, 1u);
}

static void test_timers(void) {
  hs_gba_reset(g_t);
  gba_write16(g_t, 0x04000100u, 0xFF00u); gba_write16(g_t, 0x04000102u, 0x0080u);
  gba_write16(g_t, 0x04000104u, 0xFFFCu); gba_write16(g_t, 0x04000106u, 0x00C4u);
  gba_write16(g_t, 0x04000108u, 0x0000u); gba_write16(g_t, 0x0400010Au, 0x0081u);
  gba_timers_advance(g_t, 1023);
  EQ32(C_TIMER, "T0 counter after 1023", gba_read16(g_t, 0x04000100u), 0xFFFFu);
  EQ32(C_TIMER, "T1 cascade counter", gba_read16(g_t, 0x04000104u), 0xFFFFu);
  EQ32(C_TIMER, "T1 IRQ not yet", (g_t->ifl >> 4) & 1u, 0u);
  EQ32(C_TIMER, "T2 /64 prescaler", gba_read16(g_t, 0x04000108u), 1023u / 64u);
  gba_timers_advance(g_t, 1);
  EQ32(C_TIMER, "T0 reload on overflow", gba_read16(g_t, 0x04000100u), 0xFF00u);
  EQ32(C_TIMER, "T1 cascade overflow reload", gba_read16(g_t, 0x04000104u), 0xFFFCu);
  EQ32(C_TIMER, "T1 cascade IRQ", (g_t->ifl >> 4) & 1u, 1u);
  EQ32(C_TIMER, "T0 no IRQ (disabled)", (g_t->ifl >> 3) & 1u, 0u);
  gba_write16(g_t, 0x0400010Eu, 0x0083u);
  gba_timers_advance(g_t, 4096);
  EQ32(C_TIMER, "T3 /1024 prescaler", gba_read16(g_t, 0x0400010Cu), 4u);
  gba_write16(g_t, 0x0400010Au, 0x0000u); gba_timers_advance(g_t, 6400);
  EQ32(C_TIMER, "T2 stopped holds", gba_read16(g_t, 0x04000108u), (1024u + 4096u) / 64u);
}

/* ================================ HLE BIOS ================================ */
static uint32_t lz77_compress(const uint8_t *in, uint32_t n, uint8_t *out) {
  uint32_t o = 0, i = 0;
  out[o++] = 0x10; out[o++] = (uint8_t)n; out[o++] = (uint8_t)(n >> 8); out[o++] = (uint8_t)(n >> 16);
  while (i < n) {
    uint32_t flag_at = o++;
    uint8_t flags = 0;
    for (int k = 0; k < 8 && i < n; k++) {
      uint32_t best_len = 0, best_d = 0;
      for (uint32_t d = 2; d <= 4096u && d <= i; d++) {
        uint32_t l = 0;
        while (l < 18u && i + l < n && in[i + l] == in[i - d + l]) l++;
        if (l > best_len) { best_len = l; best_d = d; }
      }
      if (best_len >= 3u) {
        flags |= (uint8_t)(0x80u >> k);
        uint32_t dd = best_d - 1u;
        out[o++] = (uint8_t)(((best_len - 3u) << 4) | (dd >> 8));
        out[o++] = (uint8_t)dd;
        i += best_len;
      } else {
        out[o++] = in[i++];
      }
    }
    out[flag_at] = flags;
  }
  while (o & 3u) out[o++] = 0;
  return o;
}

static uint32_t rl_compress(const uint8_t *in, uint32_t n, uint8_t *out) {
  uint32_t o = 0, i = 0;
  out[o++] = 0x30; out[o++] = (uint8_t)n; out[o++] = (uint8_t)(n >> 8); out[o++] = (uint8_t)(n >> 16);
  while (i < n) {
    uint32_t r = 1;
    while (i + r < n && r < 130u && in[i + r] == in[i]) r++;
    if (r >= 3u) { out[o++] = (uint8_t)(0x80u | (r - 3u)); out[o++] = in[i]; i += r; continue; }
    uint32_t s = i, l = 0;
    while (i < n && l < 128u) {
      uint32_t rr = 1;
      while (i + rr < n && rr < 3u && in[i + rr] == in[i]) rr++;
      if (rr >= 3u) break;
      i++; l++;
    }
    out[o++] = (uint8_t)(l - 1u);
    for (uint32_t k = 0; k < l; k++) out[o++] = in[s + k];
  }
  while (o & 3u) out[o++] = 0;
  return o;
}

static void test_bios(void) {
  asmb a; snip_begin(&a);
  uint8_t plain[512], comp[1200], rle[1200];
  for (uint32_t i = 0; i < 512; i++) plain[i] = (uint8_t)((i % 48u) < 24u ? (i & 7u) : (i * 13u) >> 2);
  uint32_t clen = lz77_compress(plain, 512, comp);
  const uint32_t lzsrc = 0x08004000u; memcpy(g_rom + 0x4000, comp, clen);
  uint32_t rlen = rl_compress(plain, 512, rle);
  const uint32_t rlsrc = 0x08006000u; memcpy(g_rom + 0x6000, rle, rlen);
  const uint32_t hsrc = 0x08007000u;
  const char *msg = "ABACABCCABBAACAB";
  uint8_t hb[64] = {0};
  hb[0] = 0x28; hb[1] = 16; hb[4] = 3; hb[5] = 0x80; hb[6] = 'A'; hb[7] = 0xC0; hb[8] = 'B'; hb[9] = 'C';
  {
    uint32_t nb = 0, wpos = 12; uint64_t acc = 0;
    for (int i = 0; i < 16; i++) {
      uint32_t code = msg[i] == 'A' ? 0u : (msg[i] == 'B' ? 2u : 3u), len = msg[i] == 'A' ? 1u : 2u;
      acc = (acc << len) | code; nb += len;
      if (nb >= 32u) { uint32_t w = (uint32_t)(acc >> (nb - 32u)); memcpy(hb + wpos, &w, 4); wpos += 4; nb -= 32u; acc &= (1ull << nb) - 1u; }
    }
    if (nb) { uint32_t w = (uint32_t)(acc << (32u - nb)); memcpy(hb + wpos, &w, 4); }
    memcpy(g_rom + 0x7000, hb, sizeof hb);
  }
  for (uint32_t i = 0; i < 32; i++) put32(&a, 0x08008000u + i * 4u, 0x01010101u * i + 0x10203040u);
  put32(&a, 0x08008100u, 0xDEADBEEFu);
  put16(&a, 0x08008200u, 1); g_rom[0x8202] = 1; g_rom[0x8203] = 4; put32(&a, 0x08008204u, 3);
  g_rom[0x8210] = 0xA5;
  put32(&a, 0x08008300u, 0x1000); put32(&a, 0x08008304u, 0x2000);
  put16(&a, 0x08008308u, 10); put16(&a, 0x0800830Au, 20);
  put16(&a, 0x0800830Cu, 0x100); put16(&a, 0x0800830Eu, 0x100); put16(&a, 0x08008310u, 0x4000);
  put16(&a, 0x08008320u, 0x200); put16(&a, 0x08008322u, 0x100); put16(&a, 0x08008324u, 0x0000);

  a_li(&a, 13, TRACE_BASE);
  a_li(&a, 0, (uint32_t)-100); a_li(&a, 1, 7); e32(&a, a_swi(6));
  a_tr(&a, 0); a_tr(&a, 1); a_tr(&a, 3);
  a_li(&a, 0, 7); a_li(&a, 1, (uint32_t)-100); e32(&a, a_swi(7)); a_tr(&a, 0);
  a_li(&a, 0, 1000000); e32(&a, a_swi(8)); a_tr(&a, 0);
  a_li(&a, 0, 0x7FFFFFFFu); e32(&a, a_swi(8)); a_tr(&a, 0);
  a_li(&a, 0, 0x4000); a_li(&a, 1, 0x4000); e32(&a, a_swi(0x0A)); a_tr(&a, 0);
  a_li(&a, 0, 0); a_li(&a, 1, 0x4000); e32(&a, a_swi(0x0A)); a_tr(&a, 0);
  a_li(&a, 0, (uint32_t)-0x4000); a_li(&a, 1, 0); e32(&a, a_swi(0x0A)); a_tr(&a, 0);
  a_li(&a, 0, 0); a_li(&a, 1, (uint32_t)-0x4000); e32(&a, a_swi(0x0A)); a_tr(&a, 0);
  a_li(&a, 0, 0x4000); e32(&a, a_swi(0x09)); a_tr(&a, 0);
  a_li(&a, 0, 0x08008000u); a_li(&a, 1, 0x03005000u); a_li(&a, 2, 8); e32(&a, a_swi(0x0B));
  a_li(&a, 0, 0x08008100u); a_li(&a, 1, 0x03005100u); a_li(&a, 2, 4u | (1u << 24) | (1u << 26)); e32(&a, a_swi(0x0B));
  a_li(&a, 0, 0x08008000u); a_li(&a, 1, 0x06004000u); a_li(&a, 2, 32); e32(&a, a_swi(0x0C));
  a_li(&a, 0, 0x08008100u); a_li(&a, 1, 0x03005200u); a_li(&a, 2, 3u | (1u << 24)); e32(&a, a_swi(0x0C));
  a_li(&a, 0, lzsrc); a_li(&a, 1, 0x06005000u); e32(&a, a_swi(0x12));
  a_li(&a, 0, lzsrc); a_li(&a, 1, 0x02000000u); e32(&a, a_swi(0x11));
  a_li(&a, 0, rlsrc); a_li(&a, 1, 0x02001000u); e32(&a, a_swi(0x14));
  a_li(&a, 0, rlsrc); a_li(&a, 1, 0x06006000u); e32(&a, a_swi(0x15));
  a_li(&a, 0, hsrc); a_li(&a, 1, 0x02002000u); e32(&a, a_swi(0x13));
  a_li(&a, 0, 0x08008210u); a_li(&a, 1, 0x03005300u); a_li(&a, 2, 0x08008200u); e32(&a, a_swi(0x10));
  a_li(&a, 0, 0x08008300u); a_li(&a, 1, 0x03005400u); a_li(&a, 2, 1); e32(&a, a_swi(0x0E));
  a_li(&a, 0, 0x08008320u); a_li(&a, 1, 0x03005500u); a_li(&a, 2, 1); a_li(&a, 3, 2); e32(&a, a_swi(0x0F));
  e32(&a, a_swi(0x0D)); a_tr(&a, 0);
  uint32_t end = a_end(&a);
  TRUE_(C_BIOS, "bios snippet completes", snip_run(end, 4000));
  EQ32(C_BIOS, "Div quotient", trace(0), (uint32_t)-14);
  EQ32(C_BIOS, "Div remainder", trace(1), (uint32_t)-2);
  EQ32(C_BIOS, "Div |quotient|", trace(2), 14u);
  EQ32(C_BIOS, "DivArm quotient", trace(3), (uint32_t)-14);
  EQ32(C_BIOS, "Sqrt 1e6", trace(4), 1000u);
  EQ32(C_BIOS, "Sqrt 0x7FFFFFFF", trace(5), 46340u);
  TRUE_(C_BIOS, "ArcTan2 45deg ~0x2000", (trace(6) >= 0x1FFEu && trace(6) <= 0x2002u));
  TRUE_(C_BIOS, "ArcTan2 90deg ~0x4000", (trace(7) >= 0x3FFEu && trace(7) <= 0x4002u));
  TRUE_(C_BIOS, "ArcTan2 180deg ~0x8000", (trace(8) >= 0x7FFEu && trace(8) <= 0x8002u));
  TRUE_(C_BIOS, "ArcTan2 270deg ~0xC000", (trace(9) >= 0xBFFEu && trace(9) <= 0xC002u));
  TRUE_(C_BIOS, "ArcTan(1.0) ~0x2000", ((trace(10) & 0xFFFFu) >= 0x1FFEu && (trace(10) & 0xFFFFu) <= 0x2002u));
  EQ32(C_BIOS, "GetBiosChecksum", trace(11), 0xBAAE187Fu);
  int ok = 1;
  for (uint32_t i = 0; i < 4; i++) ok &= gba_read32(g_t, 0x03005000u + i * 4u) == 0x01010101u * i + 0x10203040u;
  TRUE_(C_BIOS, "CpuSet 16-bit copy", ok);
  ok = 1; for (uint32_t i = 0; i < 4; i++) ok &= gba_read32(g_t, 0x03005100u + i * 4u) == 0xDEADBEEFu;
  ok &= gba_read32(g_t, 0x03005110u) == 0u;
  TRUE_(C_BIOS, "CpuSet 32-bit fill (exact count)", ok);
  ok = 1; for (uint32_t i = 0; i < 32; i++) ok &= gba_read32(g_t, 0x06004000u + i * 4u) == 0x01010101u * i + 0x10203040u;
  TRUE_(C_BIOS, "CpuFastSet copy -> VRAM", ok);
  ok = 1; for (uint32_t i = 0; i < 8; i++) ok &= gba_read32(g_t, 0x03005200u + i * 4u) == 0xDEADBEEFu;
  ok &= gba_read32(g_t, 0x03005220u) == 0u;
  TRUE_(C_BIOS, "CpuFastSet fill rounds to 8 words", ok);
  ok = 1; for (uint32_t i = 0; i < 512; i++) ok &= gba_read8(g_t, 0x06005000u + i) == plain[i];
  TRUE_(C_BIOS, "LZ77UnCompVram", ok);
  ok = 1; for (uint32_t i = 0; i < 512; i++) ok &= gba_read8(g_t, 0x02000000u + i) == plain[i];
  TRUE_(C_BIOS, "LZ77UnCompWram", ok);
  ok = 1; for (uint32_t i = 0; i < 512; i++) ok &= gba_read8(g_t, 0x02001000u + i) == plain[i];
  TRUE_(C_BIOS, "RLUnCompWram", ok);
  ok = 1; for (uint32_t i = 0; i < 512; i++) ok &= gba_read8(g_t, 0x06006000u + i) == plain[i];
  TRUE_(C_BIOS, "RLUnCompVram", ok);
  ok = 1; for (uint32_t i = 0; i < 16; i++) ok &= gba_read8(g_t, 0x02002000u + i) == (uint8_t)msg[i];
  TRUE_(C_BIOS, "HuffUnComp 8-bit", ok);
  EQ32(C_BIOS, "BitUnPack 1->4 bpp +3", gba_read32(g_t, 0x03005300u), 0x40400404u);
  EQ32(C_BIOS, "BgAffineSet pa (90deg)", gba_read16(g_t, 0x03005400u), 0u);
  EQ32(C_BIOS, "BgAffineSet pb", gba_read16(g_t, 0x03005402u), 0xFF00u);
  EQ32(C_BIOS, "BgAffineSet pc", gba_read16(g_t, 0x03005404u), 0x0100u);
  EQ32(C_BIOS, "BgAffineSet pd", gba_read16(g_t, 0x03005406u), 0u);
  EQ32(C_BIOS, "BgAffineSet startx", gba_read32(g_t, 0x03005408u), 0x1000u + 20u * 0x100u);
  EQ32(C_BIOS, "BgAffineSet starty", gba_read32(g_t, 0x0300540Cu), 0x2000u - 10u * 0x100u);
  EQ32(C_BIOS, "ObjAffineSet pa (scale 2x)", gba_read16(g_t, 0x03005500u), 0x200u);
  EQ32(C_BIOS, "ObjAffineSet pd", gba_read16(g_t, 0x03005506u), 0x100u);
  EQ32(C_BIOS, "ObjAffineSet pb", gba_read16(g_t, 0x03005502u), 0u);
  TRUE_(C_BIOS, "isqrt self-test 10k values", gba_hle_isqrt_selftest());
}

/* ================================== saves ================================== */
static void flash_cmd(uint8_t c) {
  gba_write8(g_t, 0x0E005555u, 0xAA); gba_write8(g_t, 0x0E002AAAu, 0x55); gba_write8(g_t, 0x0E005555u, c);
}
static void test_saves(void) {
  size_t n;
  memset(g_rom, 0, TROM_SIZE);
  memcpy(g_rom + 0x200, "SRAM_V113", 9);
  hs_gba_load_rom(g_t, g_rom, TROM_SIZE);
  EQ32(C_SAVE, "detect SRAM", hs_gba_save_kind(g_t), HS_GBA_SAVE_SRAM);
  gba_write8(g_t, 0x0E000123u, 0x5Au);
  EQ32(C_SAVE, "SRAM rw", gba_read8(g_t, 0x0E000123u), 0x5Au);
  hs_gba_save_data(g_t, &n);
  EQ32(C_SAVE, "SRAM size 32K", (uint32_t)n, 0x8000u);

  memset(g_rom, 0, TROM_SIZE);
  memcpy(g_rom + 0x400, "FLASH1M_V103", 12);
  hs_gba_load_rom(g_t, g_rom, TROM_SIZE);
  EQ32(C_SAVE, "detect Flash128", hs_gba_save_kind(g_t), HS_GBA_SAVE_FLASH128);
  flash_cmd(0x90);
  EQ32(C_SAVE, "Flash ID maker (Sanyo)", gba_read8(g_t, 0x0E000000u), 0x62u);
  EQ32(C_SAVE, "Flash ID device", gba_read8(g_t, 0x0E000001u), 0x13u);
  flash_cmd(0xF0);
  flash_cmd(0xA0); gba_write8(g_t, 0x0E001234u, 0x42);
  EQ32(C_SAVE, "Flash byte program", gba_read8(g_t, 0x0E001234u), 0x42u);
  flash_cmd(0xB0); gba_write8(g_t, 0x0E000000u, 1);
  flash_cmd(0xA0); gba_write8(g_t, 0x0E001234u, 0x77);
  EQ32(C_SAVE, "Flash bank1 program", gba_read8(g_t, 0x0E001234u), 0x77u);
  EQ32(C_SAVE, "Flash bank1 lands at +64K", g_t->save[0x11234], 0x77u);
  flash_cmd(0x80);
  gba_write8(g_t, 0x0E005555u, 0xAA); gba_write8(g_t, 0x0E002AAAu, 0x55); gba_write8(g_t, 0x0E001000u, 0x30);
  EQ32(C_SAVE, "Flash sector erase", gba_read8(g_t, 0x0E001234u), 0xFFu);
  EQ32(C_SAVE, "Flash bank0 untouched", g_t->save[0x1234], 0x42u);
  TRUE_(C_SAVE, "save dirty flag", hs_gba_save_dirty(g_t));

  memset(g_rom, 0, TROM_SIZE);
  memcpy(g_rom + 0x800, "EEPROM_V124", 11);
  hs_gba_load_rom(g_t, g_rom, TROM_SIZE);
  EQ32(C_SAVE, "detect EEPROM", hs_gba_save_kind(g_t), HS_GBA_SAVE_EEPROM);
  const uint64_t val = 0x0123456789ABCDEFull;
  uint32_t p = 0x02000000u, k = 0;
  uint16_t bits[96];
  bits[k++] = 1; bits[k++] = 0;
  for (int i = 5; i >= 0; i--) bits[k++] = (uint16_t)((5u >> i) & 1u);
  for (int i = 63; i >= 0; i--) bits[k++] = (uint16_t)((val >> i) & 1u);
  bits[k++] = 0;
  for (uint32_t i = 0; i < k; i++) gba_write16(g_t, p + i * 2u, bits[i]);
  gba_write32(g_t, 0x040000D4u, p); gba_write32(g_t, 0x040000D8u, 0x0D000000u);
  gba_write32(g_t, 0x040000DCu, 0x80000000u | k);
  EQ32(C_SAVE, "EEPROM 6-bit addr from DMA len 73", (uint32_t)g_t->eeprom.addr_bits, 6u);
  k = 0; bits[k++] = 1; bits[k++] = 1;
  for (int i = 5; i >= 0; i--) bits[k++] = (uint16_t)((5u >> i) & 1u);
  bits[k++] = 0;
  for (uint32_t i = 0; i < k; i++) gba_write16(g_t, p + i * 2u, bits[i]);
  gba_write32(g_t, 0x040000D4u, p); gba_write32(g_t, 0x040000D8u, 0x0D000000u);
  gba_write32(g_t, 0x040000DCu, 0x80000000u | k);
  gba_write32(g_t, 0x040000D4u, 0x0D000000u); gba_write32(g_t, 0x040000D8u, 0x02000400u);
  gba_write32(g_t, 0x040000DCu, 0x80000000u | 68u);
  uint64_t got = 0;
  for (uint32_t i = 4; i < 68; i++) got = (got << 1) | (gba_read16(g_t, 0x02000400u + i * 2u) & 1u);
  check(C_SAVE, got == val, "EEPROM write+read 64-bit block", got, val);
  hs_gba_save_data(g_t, &n);
  EQ32(C_SAVE, "EEPROM 512B image", (uint32_t)n, 0x200u);
}

/* ============================ synthetic ROM #1 ============================ */
/* Mode 0 scrolling tiled BG + regular and semi-transparent sprites + Direct
 * Sound FIFO A (Timer0 + DMA1) + PSG square, ARM init -> Thumb main loop,
 * VBlank IRQ (ARM handler) + DMA3 uploads. */
static void build_rom1(uint8_t *rom) {
  asmb a; a.mem = rom; a.base = 0x08000000u; a.pos = 0;
  memset(rom, 0, TROM_SIZE);
  const uint32_t PAL = 0x08002000u, TILES = 0x08002400u, MAP = 0x08002800u, SPR = 0x08003000u,
                 OBJPAL = 0x08003200u, FILL = 0x08003400u, SND = 0x08010000u;
  static const uint16_t bgpal[8] = {0x2884, 0x03FF, 0x7C1F, 0x7FE0, 0x001F, 0x7C00, 0x03E0, 0x5294};
  for (uint32_t i = 0; i < 8; i++) put16(&a, PAL + i * 2u, bgpal[i]);
  for (uint32_t i = 8; i < 256; i++) put16(&a, PAL + i * 2u, (uint16_t)((i * 0x0421u) & 0x7FFFu));
  static const uint16_t objpal[4] = {0, 0x7FFF, 0x001F, 0x4210};
  for (uint32_t i = 0; i < 4; i++) put16(&a, OBJPAL + i * 2u, objpal[i]);
  for (uint32_t t = 0; t < 4; t++)
    for (uint32_t y = 0; y < 8; y++)
      for (uint32_t x = 0; x < 8; x += 2) {
        uint32_t c[2];
        for (uint32_t k = 0; k < 2; k++) {
          uint32_t xx = x + k;
          switch (t) {
            case 0: c[k] = 1; break;
            case 1: c[k] = ((xx ^ y) & 1u) ? 2 : 3; break;
            case 2: c[k] = ((xx + y) & 4u) ? 4 : 5; break;
            default: c[k] = (xx == 0 || y == 0 || xx == 7 || y == 7) ? 6 : 7; break;
          }
        }
        rom[(TILES - a.base) + t * 32u + y * 4u + x / 2u] = (uint8_t)(c[0] | (c[1] << 4));
      }
  for (uint32_t y = 0; y < 32; y++)
    for (uint32_t x = 0; x < 32; x++)
      put16(&a, MAP + (y * 32u + x) * 2u, (uint16_t)(((x + y) & 3u) | (((x * 7u + y) & 4u) ? 0x400u : 0u)));
  for (uint32_t y = 0; y < 16; y++)
    for (uint32_t x = 0; x < 16; x++) {
      int32_t dx = (int32_t)x * 2 - 15, dy = (int32_t)y * 2 - 15;
      int32_t d = dx * dx + dy * dy;
      uint32_t c = d < 140 ? 1u : (d < 220 ? 2u : 0u);
      uint32_t tile = (y / 8u) * 2u + x / 8u;
      uint32_t off = (SPR - a.base) + tile * 32u + (y & 7u) * 4u + (x & 7u) / 2u;
      rom[off] = (uint8_t)(rom[off] | (c << ((x & 1u) * 4u)));
    }
  put32(&a, FILL, 0x02000200u);
  for (uint32_t i = 0; i < 0x8000u; i++) {
    int32_t ph = (int32_t)(i & 63u);
    int32_t v = ph < 32 ? ph * 6 - 96 : (63 - ph) * 6 - 96;
    rom[(SND - a.base) + i] = (uint8_t)(int8_t)v;
  }
  memcpy(rom + 0xA0, "HSGBA MODE0 ", 12);
  e32(&a, a_b(0, here(&a), 0x080000C0u));
  a.pos = 0xC0;
  a_li(&a, 0, 0x03007FFCu);
  uint32_t li_irq = here(&a); for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_str(1, 0, 0));
  a_li(&a, 4, 0x040000D4u);
  const uint32_t dmas[6][3] = {
    {PAL, 0x05000000u, 0x84000080u}, {TILES, 0x06000000u, 0x84000020u},
    {MAP, 0x0600F800u, 0x84000200u}, {SPR, 0x06010000u, 0x84000020u},
    {OBJPAL, 0x05000200u, 0x84000002u}, {FILL, 0x07000000u, 0x85000100u}};
  for (int i = 0; i < 6; i++) {
    a_li(&a, 1, dmas[i][0]); e32(&a, a_str(1, 4, 0));
    a_li(&a, 1, dmas[i][1]); e32(&a, a_str(1, 4, 4));
    a_li(&a, 1, dmas[i][2]); e32(&a, a_str(1, 4, 8));
  }
  a_li(&a, 2, 0x07000000u);
  a_li(&a, 1, 0x003C); e32(&a, a_strh(1, 2, 0)); a_li(&a, 1, 0x4064); e32(&a, a_strh(1, 2, 2));
  a_li(&a, 1, 0x0000); e32(&a, a_strh(1, 2, 4));
  a_li(&a, 1, 0x045A); e32(&a, a_strh(1, 2, 8)); a_li(&a, 1, 0x4096); e32(&a, a_strh(1, 2, 10));
  a_li(&a, 1, 0x0000); e32(&a, a_strh(1, 2, 12));
  a_li(&a, 0, 0x04000000u);
  a_li(&a, 1, 0x1F00); e32(&a, a_strh(1, 0, 0x08));
  a_li(&a, 1, 0x0100); e32(&a, a_strh(1, 0, 0x50));
  a_li(&a, 1, 0x0808); e32(&a, a_strh(1, 0, 0x52));
  a_li(&a, 1, 0x1140); e32(&a, a_strh(1, 0, 0x00));
  a_li(&a, 1, 0x0080); e32(&a, a_strh(1, 0, 0x84));
  a_li(&a, 1, 0x2277); e32(&a, a_strh(1, 0, 0x80));
  a_li(&a, 1, 0x0B06); e32(&a, a_strh(1, 0, 0x82));
  a_li(&a, 1, 0xA080); e32(&a, a_strh(1, 0, 0x68));
  a_li(&a, 1, 0x86D6); e32(&a, a_strh(1, 0, 0x6C));
  a_li(&a, 1, SND); e32(&a, a_str(1, 0, 0xBC));
  a_li(&a, 1, 0x040000A0u); e32(&a, a_str(1, 0, 0xC0));
  a_li(&a, 1, 0xB6400004u); e32(&a, a_str(1, 0, 0xC4));
  a_li(&a, 1, 0x0080FC00u); e32(&a, a_str(1, 0, 0x100));
  a_li(&a, 1, 0x0084FFF0u); e32(&a, a_str(1, 0, 0x104));
  a_li(&a, 1, 0x0008); e32(&a, a_strh(1, 0, 0x04));
  e32(&a, a_dp(ADD, 0, 2, 0, IMM(0x200)));
  a_li(&a, 1, 1); e32(&a, a_strh(1, 2, 0)); e32(&a, a_strh(1, 2, 8));
  uint32_t li_th = here(&a); for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_bx(0));
  uint32_t th = here(&a);
  e16(&a, T_MOVI(7, 0));
  uint32_t loop = here(&a);
  e16(&a, t_swi(5));
  e16(&a, T_ADDI(7, 1));
  e16(&a, T_MOVI(0, 1)); e16(&a, t_sh(0, 0, 0, 26));
  e16(&a, t_sh(1, 1, 7, 1)); e16(&a, t_lshi(0, 1, 0, 9));
  { uint32_t p = here(&a); e16(&a, t_b(p, loop)); }
  align4(&a);
  uint32_t irq = here(&a);
  a_li(&a, 0, 0x04000200u);
  e32(&a, a_ldr(1, 0, 0));
  e32(&a, a_dp(AND, 0, 1, 1, RSH(1, LSR, 16)));
  e32(&a, a_strh(1, 0, 2));
  a_li(&a, 2, 0x03007FF8u);
  e32(&a, a_ldrh(3, 2, 0)); e32(&a, a_dp(ORR, 0, 3, 3, 1)); e32(&a, a_strh(3, 2, 0));
  a_li(&a, 2, 0x03000000u);
  e32(&a, a_ldr(3, 2, 0)); e32(&a, a_dp(ADD, 0, 3, 3, IMM(1))); e32(&a, a_str(3, 2, 0));
  e32(&a, a_dp(SUB, 0, 0, 0, IMM(0x200)));
  e32(&a, a_strh(3, 0, 0x10));
  a_li(&a, 2, 0x07000000u);
  e32(&a, a_dp(MOV, 0, 1, 0, RSH(3, LSL, 1)));
  e32(&a, a_dp(AND, 0, 1, 1, IMM(0xFF)));
  e32(&a, a_dp(ORR, 0, 1, 1, IMM(0x4000)));
  e32(&a, a_strh(1, 2, 2));
  e32(&a, a_bx(14));
  patch_li(&a, li_irq, 1, irq);
  patch_li(&a, li_th, 0, th | 1u);
}

/* ============================ synthetic ROM #2 ============================ */
/* Mode 3 full-colour gradient for 20 frames, then Mode 2 affine BG2 rotating
 * checkerboard driven by HLE BgAffineSet each VBlank; tiles via
 * LZ77UnCompVram, map via CpuFastSet, palette via CpuSet. */
static void build_rom2(uint8_t *rom) {
  asmb a; a.mem = rom; a.base = 0x08000000u; a.pos = 0;
  memset(rom, 0, TROM_SIZE);
  const uint32_t LZT = 0x08004000u, MAP2 = 0x08006000u, PAL2 = 0x08007000u;
  uint8_t tiles[256], comp[600];
  for (uint32_t t = 0; t < 4; t++)
    for (uint32_t y = 0; y < 8; y++)
      for (uint32_t x = 0; x < 8; x++) {
        uint8_t c;
        switch (t) {
          case 0: c = 1; break;
          case 1: c = 2; break;
          case 2: c = (x == y || x == 7 - y) ? 3 : 2; break;
          default: c = (x < 2 || y < 2) ? 4 : 1; break;
        }
        tiles[t * 64u + y * 8u + x] = c;
      }
  uint32_t cl = lz77_compress(tiles, 256, comp);
  memcpy(rom + (LZT - a.base), comp, cl);
  for (uint32_t y = 0; y < 32; y++)
    for (uint32_t x = 0; x < 32; x++)
      rom[(MAP2 - a.base) + y * 32u + x] =
          (uint8_t)((((x >> 1) ^ (y >> 1)) & 1u) ? ((x + y) % 3u == 0 ? 2u : 1u) : (((x ^ y) & 3u) == 0 ? 3u : 0u));
  static const uint16_t pal[5] = {0x0000, 0x7FFF, 0x10A5, 0x03FF, 0x7C10};
  for (uint32_t i = 0; i < 5; i++) put16(&a, PAL2 + i * 2u, pal[i]);
  memcpy(rom + 0xA0, "HSGBA AFFINE", 12);
  e32(&a, a_b(0, here(&a), 0x080000C0u));
  a.pos = 0xC0;
  a_li(&a, 0, 0x03007FFCu);
  uint32_t li_irq = here(&a); for (int i = 0; i < 4; i++) e32(&a, 0xE1A00000u);
  e32(&a, a_str(1, 0, 0));
  a_li(&a, 0, 0x04000000u);
  a_li(&a, 1, 0x0008); e32(&a, a_strh(1, 0, 0x04));
  e32(&a, a_dp(ADD, 0, 2, 0, IMM(0x200)));
  a_li(&a, 1, 1); e32(&a, a_strh(1, 2, 0)); e32(&a, a_strh(1, 2, 8));
  a_li(&a, 0, 0x06000000u);
  a_li(&a, 1, 0);
  uint32_t yl = here(&a);
  a_li(&a, 2, 0);
  uint32_t xl = here(&a);
  e32(&a, a_dp(MOV, 0, 3, 0, RSH(2, LSR, 3)));
  e32(&a, a_dp(ADD, 0, 3, 3, IMM(2)));
  e32(&a, a_dp(MOV, 0, 4, 0, RSH(1, LSR, 3)));
  e32(&a, a_dp(ORR, 0, 3, 3, RSH(4, LSL, 5)));
  e32(&a, a_dp(ADD, 0, 4, 1, 2));
  e32(&a, a_dp(MOV, 0, 4, 0, RSH(4, LSR, 4)));
  e32(&a, a_dp(ORR, 0, 3, 3, RSH(4, LSL, 10)));
  e32(&a, a_half(0, 0, 1, 0, 1, 3, 0, 2));
  e32(&a, a_dp(ADD, 0, 2, 2, IMM(1)));
  e32(&a, a_dp(CMP, 1, 0, 2, IMM(240)));
  { uint32_t p = here(&a); e32(&a, CC_(LT, a_b(0, p, xl))); }
  e32(&a, a_dp(ADD, 0, 1, 1, IMM(1)));
  e32(&a, a_dp(CMP, 1, 0, 1, IMM(160)));
  { uint32_t p = here(&a); e32(&a, CC_(LT, a_b(0, p, yl))); }
  a_li(&a, 0, 0x04000000u);
  a_li(&a, 1, 0x0403); e32(&a, a_strh(1, 0, 0));
  a_li(&a, 5, 20);
  uint32_t wl = here(&a);
  e32(&a, a_swi(5));
  e32(&a, a_dp(SUB, 1, 5, 5, IMM(1)));
  { uint32_t p = here(&a); e32(&a, CC_(NE, a_b(0, p, wl))); }
  a_li(&a, 0, LZT); a_li(&a, 1, 0x06000000u); e32(&a, a_swi(0x12));
  a_li(&a, 0, MAP2); a_li(&a, 1, 0x06004000u); a_li(&a, 2, 256); e32(&a, a_swi(0x0C));
  a_li(&a, 0, PAL2); a_li(&a, 1, 0x05000000u); a_li(&a, 2, 5); e32(&a, a_swi(0x0B));
  a_li(&a, 0, 0x04000000u);
  a_li(&a, 1, 0x6800); e32(&a, a_strh(1, 0, 0x0C));
  a_li(&a, 1, 0x0402); e32(&a, a_strh(1, 0, 0x00));
  a_li(&a, 8, 0x03000100u);
  a_li(&a, 1, 128u << 8); e32(&a, a_str(1, 8, 0)); e32(&a, a_str(1, 8, 4));
  a_li(&a, 1, 120); e32(&a, a_strh(1, 8, 8)); a_li(&a, 1, 80); e32(&a, a_strh(1, 8, 10));
  a_li(&a, 9, 0);
  uint32_t ml = here(&a);
  e32(&a, a_swi(5));
  e32(&a, a_dp(ADD, 0, 9, 9, IMM(1)));
  e32(&a, a_dp(MOV, 0, 1, 0, RSH(9, LSL, 9)));
  e32(&a, a_strh(1, 8, 16));
  e32(&a, a_dp(AND, 0, 1, 9, IMM(0x3F)));
  e32(&a, a_dp(ADD, 0, 1, 1, IMM(0xC0)));
  e32(&a, a_strh(1, 8, 12)); e32(&a, a_strh(1, 8, 14));
  e32(&a, a_dp(MOV, 0, 0, 0, 8)); a_li(&a, 1, 0x04000020u); a_li(&a, 2, 1);
  e32(&a, a_swi(0x0E));
  { uint32_t p = here(&a); e32(&a, a_b(0, p, ml)); }
  uint32_t irq = here(&a);
  a_li(&a, 0, 0x04000200u);
  e32(&a, a_ldr(1, 0, 0));
  e32(&a, a_dp(AND, 0, 1, 1, RSH(1, LSR, 16)));
  e32(&a, a_strh(1, 0, 2));
  a_li(&a, 2, 0x03007FF8u);
  e32(&a, a_ldrh(3, 2, 0)); e32(&a, a_dp(ORR, 0, 3, 3, 1)); e32(&a, a_strh(3, 2, 0));
  e32(&a, a_bx(14));
  patch_li(&a, li_irq, 1, irq);
}

/* ================================== PNG ================================== */
static uint32_t crc_tab[256];
static void crc_init(void) {
  for (uint32_t n = 0; n < 256; n++) {
    uint32_t c = n;
    for (int k = 0; k < 8; k++) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    crc_tab[n] = c;
  }
}
static uint32_t crc_upd(uint32_t c, const uint8_t *b, size_t n) {
  for (size_t i = 0; i < n; i++) c = crc_tab[(c ^ b[i]) & 0xFFu] ^ (c >> 8);
  return c;
}
static void be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static void png_chunk(FILE *f, const char *type, const uint8_t *data, uint32_t n) {
  uint8_t h[8]; be32(h, n); memcpy(h + 4, type, 4);
  fwrite(h, 1, 8, f);
  if (n) fwrite(data, 1, n, f);
  uint32_t c = crc_upd(0xFFFFFFFFu, (const uint8_t *)type, 4);
  if (n) c = crc_upd(c, data, n);
  c ^= 0xFFFFFFFFu;
  uint8_t t[4]; be32(t, c); fwrite(t, 1, 4, f);
}
static int write_png(const char *path, const uint32_t *px, uint32_t w, uint32_t h) {
  uint32_t raw_n = h * (1u + w * 3u);
  uint8_t *raw = (uint8_t *)malloc(raw_n);
  if (!raw) return -1;
  for (uint32_t y = 0; y < h; y++) {
    uint8_t *r = raw + y * (1u + w * 3u);
    r[0] = 0;
    for (uint32_t x = 0; x < w; x++) {
      uint32_t c = px[y * w + x];
      r[1 + x * 3] = (uint8_t)(c >> 16); r[2 + x * 3] = (uint8_t)(c >> 8); r[3 + x * 3] = (uint8_t)c;
    }
  }
  uint32_t nblk = (raw_n + 65534u) / 65535u;
  uint32_t zn = 2u + raw_n + nblk * 5u + 4u;
  uint8_t *z = (uint8_t *)malloc(zn);
  if (!z) { free(raw); return -1; }
  uint32_t o = 0; z[o++] = 0x78; z[o++] = 0x01;
  uint32_t s1 = 1, s2 = 0;
  for (uint32_t i = 0; i < raw_n; i++) { s1 = (s1 + raw[i]) % 65521u; s2 = (s2 + s1) % 65521u; }
  for (uint32_t p = 0; p < raw_n; p += 65535u) {
    uint32_t len = raw_n - p > 65535u ? 65535u : raw_n - p;
    z[o++] = (p + len >= raw_n) ? 1 : 0;
    z[o++] = (uint8_t)len; z[o++] = (uint8_t)(len >> 8);
    z[o++] = (uint8_t)~len; z[o++] = (uint8_t)(~len >> 8);
    memcpy(z + o, raw + p, len); o += len;
  }
  be32(z + o, (s2 << 16) | s1); o += 4;
  FILE *f = fopen(path, "wb");
  if (!f) { free(raw); free(z); return -1; }
  static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  fwrite(sig, 1, 8, f);
  uint8_t ih[13]; be32(ih, w); be32(ih + 4, h); ih[8] = 8; ih[9] = 2; ih[10] = 0; ih[11] = 0; ih[12] = 0;
  png_chunk(f, "IHDR", ih, 13);
  png_chunk(f, "IDAT", z, o);
  png_chunk(f, "IEND", NULL, 0);
  fclose(f);
  free(raw); free(z);
  return 0;
}

/* ============================ frame-level runs ============================ */
static uint32_t *g_out;
static int16_t g_pcm[HS_GBA_MAX_SAMPLES_PER_FRAME * 2];

typedef struct { uint64_t video, audio, out, last_frame_hash; uint32_t samples, nonzero; } run_hashes;

static int edges_filled(const uint32_t *o) {
  for (uint32_t x = 0; x < HS_GBA_OUT_W; x++)
    if ((o[x] & 0xFFFFFFu) == 0 || (o[(HS_GBA_OUT_H - 1u) * HS_GBA_OUT_W + x] & 0xFFFFFFu) == 0) return 0;
  for (uint32_t y = 0; y < HS_GBA_OUT_H; y++)
    if ((o[y * HS_GBA_OUT_W] & 0xFFFFFFu) == 0 || (o[y * HS_GBA_OUT_W + HS_GBA_OUT_W - 1u] & 0xFFFFFFu) == 0) return 0;
  return 1;
}

static run_hashes run_rom(hs_gba *g, int frames, int png_frame, const char *png_path, int *edges_ok) {
  run_hashes r; memset(&r, 0, sizeof r);
  r.video = r.audio = r.out = 0xcbf29ce484222325ull;
  int eok = 1;
  for (int f = 0; f < frames; f++) {
    hs_gba_step_cpu_phase(g, 0);
    hs_gba_render_video_phase(g, g_out);
    int n = hs_gba_synth_audio_phase(g, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
    uint64_t fh = hs_gba_frame_hash(g);
    r.video = fnv(r.video, &fh, sizeof fh);
    r.audio = fnv(r.audio, g_pcm, (size_t)n * 4u);
    r.samples += (uint32_t)n;
    for (int i = 0; i < n * 2; i++) r.nonzero += g_pcm[i] != 0;
    if (f == frames - 1) r.out = fnv(r.out, g_out, (size_t)HS_GBA_OUT_W * HS_GBA_OUT_H * 4u);
    if (f >= 5) eok &= edges_filled(g_out);
    if (f == png_frame && png_path) write_png(png_path, g_out, HS_GBA_OUT_W, HS_GBA_OUT_H);
    r.last_frame_hash = fh;
  }
  if (edges_ok) *edges_ok = eok;
  return r;
}

static uint32_t count_color(const hs_gba *g, uint16_t c) {
  const uint16_t *fb = hs_gba_native_frame(g);
  uint32_t n = 0;
  for (uint32_t i = 0; i < HS_GBA_SCREEN_W * HS_GBA_SCREEN_H; i++) n += fb[i] == c;
  return n;
}

static int bench(int frames, const char *phases) {
  uint8_t *rom = (uint8_t *)calloc(1, TROM_SIZE);
  hs_gba *g = hs_gba_create();
  if (!rom || !g) return 1;
  if (strchr(phases, 'b')) { /* busy CPU stress: never halts; ARM+Thumb ALU/IWRAM ld-st/VRAM strh loop */
    uint32_t arm_loop = 0, th = 0;
    for (int pass = 0; pass < 2; pass++) { /* pass 0 resolves addresses, pass 1 emits final code */
      asmb a; a.mem = rom; a.base = 0x08000000u; a.pos = 0;
      a_li(&a, 3, 0x03000000u); a_li(&a, 0, 0x04000000u); a_li(&a, 1, 0x0403u); e32(&a, a_strh(1, 0, 0));
      a_li(&a, 9, 0x06000000u);
      e32(&a, a_dp(MOV, 0, 6, 0, IMM(0))); a_li(&a, 6, th | 1u);
      e32(&a, a_dp(MOV, 0, 8, 0, IMM(0))); a_li(&a, 8, arm_loop);
      arm_loop = here(&a);
      e32(&a, a_dp(ADD, 0, 4, 4, IMM(1)));
      e32(&a, a_dp(EOR, 0, 1, 1, RSH(4, ROR, 3)));
      e32(&a, a_ldr(2, 3, 0)); e32(&a, a_dp(ADD, 0, 2, 2, 1)); e32(&a, a_str(2, 3, 4));
      e32(&a, a_dp(MOV, 0, 10, 0, RSH(4, LSL, 17))); e32(&a, a_dp(MOV, 0, 10, 0, RSH(10, LSR, 16)));
      e32(&a, a_dp(ADD, 0, 11, 9, 10)); e32(&a, a_strh(1, 11, 0));
      e32(&a, a_mul(5, 4, 1, 0));
      e32(&a, a_bx(6));
      th = here(&a);
      e16(&a, t_add3(0, 0, 4)); e16(&a, T_ADDI(0, 2)); e16(&a, t_alu(TA_EOR, 1, 4));
      e16(&a, t_lsi(0, 1, 2, 3, 1)); e16(&a, t_sh(0, 7, 0, 3)); e16(&a, t_alu(TA_ORR, 7, 2));
      e16(&a, t_lsi(0, 0, 7, 3, 2));
      e16(&a, t_bx(8));
      align4(&a);
    }
    hs_gba_load_rom(g, rom, TROM_SIZE);
  } else {
    build_rom1(rom);
    hs_gba_load_rom(g, rom, TROM_SIZE);
  }
  int v = strchr(phases, 'v') != NULL, au = strchr(phases, 'a') != NULL;
  uint64_t h = 0xcbf29ce484222325ull;
  for (int f = 0; f < frames; f++) {
    hs_gba_step_cpu_phase(g, 0);
    if (v) hs_gba_render_video_phase(g, g_out);
    if (au) { int n = hs_gba_synth_audio_phase(g, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME); h = fnv(h, g_pcm, (size_t)n * 4u); }
  }
  printf("bench frames=%d phases=%s frame_hash=%016llx audio=%016llx insns=%llu\n", frames, phases,
         (unsigned long long)hs_gba_frame_hash(g), (unsigned long long)h, (unsigned long long)g->cpu.insn_count);
  hs_gba_destroy(g);
  free(rom);
  return 0;
}

/* ============================ APU unit tests ============================ */
static void test_apu(void) {
  gba_snapshot *s = (gba_snapshot *)calloc(1, sizeof(gba_snapshot));
  gba_apu *ap = (gba_apu *)calloc(1, sizeof(gba_apu));
  if (!s || !ap) { free(s); free(ap); return; }
  gba_apu_reset(ap);
  const uint16_t ev[][2] = {{0x84, 0x0080}, {0x80, 0x2277}, {0x82, 0x0002}, {0x68, 0xF080}, {0x6C, 0x86D6}};
  for (uint32_t i = 0; i < 5; i++) { s->ev[i].t = i; s->ev[i].reg = ev[i][0]; s->ev[i].val = ev[i][1]; }
  s->n_ev = 5; s->frame_no = 0;
  int n = gba_apu_synth(ap, s, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
  EQ32(C_APU, "frame 0 sample count (ceil 280896*48000/2^24)", (uint32_t)n, 804u);
  uint32_t flips = 0; int32_t peak = 0;
  for (int i = 1; i < n; i++) {
    if ((g_pcm[i * 2] > 0) != (g_pcm[i * 2 - 2] > 0)) flips++;
    if (g_pcm[i * 2] > peak) peak = g_pcm[i * 2];
  }
  /* 131072/(2048-1750) = 439.8 Hz -> 7.36 cycles/frame -> ~14.7 sign flips (+1 from silent start) */
  check(C_APU, flips >= 14u && flips <= 16u, "PSG square 439.8 Hz (14..16 sign flips/frame)", flips, 15);
  EQ32(C_APU, "PSG square amplitude vol15*8*16", (uint32_t)peak, 15u * 8u * 16u);
  s->n_ev = 0; s->frame_no = 1;
  n = gba_apu_synth(ap, s, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
  EQ32(C_APU, "frame 1 sample count (ceil boundary 1608-804)", (uint32_t)n, 804u);
  gba_apu_reset(ap);
  const uint16_t ev2[][2] = {{0x84, 0x0080}, {0x82, 0x0304}, {0xF0, 0x0040}};
  for (uint32_t i = 0; i < 3; i++) { s->ev[i].t = 1000u * i; s->ev[i].reg = ev2[i][0]; s->ev[i].val = ev2[i][1]; }
  s->ev[3].t = 140000; s->ev[3].reg = 0xF0; s->ev[3].val = 0xC0;
  s->n_ev = 4; s->frame_no = 0;
  n = gba_apu_synth(ap, s, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
  EQ32(C_APU, "FIFO A +64 @100% -> L", (uint32_t)(int32_t)g_pcm[200], 64u * 4u * 16u);
  EQ32(C_APU, "FIFO A +64 @100% -> R", (uint32_t)(int32_t)g_pcm[201], 64u * 4u * 16u);
  EQ32(C_APU, "FIFO A -64 after t=140000 (sample-accurate)", (uint32_t)(int32_t)g_pcm[(int)(140000ull * 48000ull / 16777216ull + 2u) * 2],
       (uint32_t)(-64 * 4 * 16));
  EQ32(C_APU, "FIFO A value before switch", (uint32_t)(int32_t)g_pcm[(int)(140000ull * 48000ull / 16777216ull - 2u) * 2], 64u * 4u * 16u);
  gba_apu_reset(ap);
  s->ev[0].t = 0; s->ev[0].reg = 0xF0; s->ev[0].val = 0x40; s->n_ev = 1;
  n = gba_apu_synth(ap, s, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
  int silent = 1;
  for (int i = 0; i < n * 2; i++) silent &= g_pcm[i] == 0;
  TRUE_(C_APU, "master enable off -> silence", silent);
  free(s); free(ap);
}

static void test_contract(hs_gba *g, const uint8_t *rom1) {
  hs_gba_load_rom(g, rom1, TROM_SIZE);
  hs_gba_step_cpu_phase(g, 0);
  hs_gba_step_cpu_phase(g, 0);
  int pub = g->snap_pub;
  uint64_t before = fnv(0xcbf29ce484222325ull, &g->snap[pub], sizeof(gba_snapshot));
  hs_gba_render_video_phase(g, g_out);
  uint64_t vh = hs_gba_frame_hash(g);
  hs_gba_synth_audio_phase(g, g_pcm, HS_GBA_MAX_SAMPLES_PER_FRAME);
  uint64_t after = fnv(0xcbf29ce484222325ull, &g->snap[pub], sizeof(gba_snapshot));
  TRUE_(C_DET, "CPU phase targets the non-published buffer", g->snap_w != pub);
  check(C_DET, before == after, "video+audio phases never write the snapshot", after, before);
  gba_snapshot *keep = (gba_snapshot *)malloc(sizeof(gba_snapshot));
  gba_video *vv = (gba_video *)malloc(sizeof(gba_video));
  if (!keep || !vv) { free(keep); free(vv); return; }
  memcpy(keep, &g->snap[pub], sizeof *keep);
  hs_gba_step_cpu_phase(g, 0); /* CPU frame N+1 (pipelined) */
  uint64_t still = fnv(0xcbf29ce484222325ull, &g->snap[pub], sizeof(gba_snapshot));
  check(C_DET, still == before, "frame-N snapshot untouched by CPU frame N+1", still, before);
  gba_video_init(vv);
  gba_ppu_render(vv, keep);
  check(C_DET, vv->hash == vh, "pipelined re-render of frame N bit-identical", vv->hash, vh);
  free(keep);
  free(vv);
}

int main(int argc, char **argv) {
  const char *outdir = ".";
  g_out = (uint32_t *)malloc((size_t)HS_GBA_OUT_W * HS_GBA_OUT_H * 4u);
  if (!g_out) return 1;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--bench") && i + 2 < argc) return bench(atoi(argv[i + 1]), argv[i + 2]);
    if (!strcmp(argv[i], "--out") && i + 1 < argc) outdir = argv[++i];
  }
  crc_init();
  g_rom = (uint8_t *)calloc(1, TROM_SIZE);
  g_t = hs_gba_create();
  if (!g_rom || !g_t) return 1;

  test_arm_alu(); test_arm_shifter(); test_arm_mul(); test_arm_ldst(); test_arm_block();
  test_arm_psr_banks_cond();
  test_thumb();
  test_interwork();
  test_irq_banked();
  test_dma3();
  test_timers();
  test_bios();
  test_saves();
  test_apu();

  char p0[512], p2[512], p3[512];
  snprintf(p0, sizeof p0, "%s/gba_mode0_640x480.png", outdir);
  snprintf(p2, sizeof p2, "%s/gba_mode2_affine_640x480.png", outdir);
  snprintf(p3, sizeof p3, "%s/gba_mode3_bitmap_640x480.png", outdir);
  uint8_t *rom1 = (uint8_t *)calloc(1, TROM_SIZE), *rom2 = (uint8_t *)calloc(1, TROM_SIZE);
  hs_gba *g = hs_gba_create();
  if (!rom1 || !rom2 || !g) return 1;
  build_rom1(rom1); build_rom2(rom2);

  /* ---- ROM 1 ---- */
  hs_gba_load_rom(g, rom1, TROM_SIZE);
  int e1 = 0;
  run_hashes r1 = run_rom(g, 90, 89, p0, &e1);
  uint32_t irqs = gba_read32(g, 0x03000000u);
  TRUE_(C_IRQ, "ROM1 VBlank IRQ count tracks frames (88..90)", irqs >= 88u && irqs <= 90u);
  EQ32(C_INTER, "ROM1 main loop runs in Thumb", (g->cpu.cpsr >> 5) & 1u, 1u);
  TRUE_(C_PPU, "ROM1 edge-to-edge 640x480 (no black bars)", e1);
  TRUE_(C_PPU, "ROM1 sprite white body visible", count_color(g, 0x7FFF) > 60u);
  TRUE_(C_PPU, "ROM1 sprite red rim visible", count_color(g, 0x001F) > 10u);
  {
    const uint16_t *fb = hs_gba_native_frame(g);
    static const uint16_t bgc[8] = {0x2884, 0x03FF, 0x7C1F, 0x7FE0, 0x001F, 0x7C00, 0x03E0, 0x5294};
    uint32_t blended = 0;
    for (uint32_t y = 90; y < 106; y++)
      for (uint32_t x = 150; x < 166; x++) {
        uint16_t c = fb[y * 240u + x];
        for (int k = 0; k < 8; k++) {
          uint16_t b = bgc[k];
          uint16_t e = (uint16_t)((((31u * 8u + (b & 31u) * 8u) >> 4)) | ((((31u * 8u + ((b >> 5) & 31u) * 8u) >> 4)) << 5) |
                                  ((((31u * 8u + ((b >> 10) & 31u) * 8u) >> 4)) << 10));
          if (c == e) { blended++; break; }
        }
      }
    TRUE_(C_PPU, "ROM1 semi-transparent sprite = exact 8/8 alpha blend of white over BG", blended > 60u);
  }
  TRUE_(C_APU, "ROM1 Direct Sound + PSG audio non-silent", r1.nonzero > 1000u);
  TRUE_(C_APU, "ROM1 48 kHz sample count (90 frames ~72328)", r1.samples >= 72320u && r1.samples <= 72335u);
  EQ32(C_APU, "ROM1 no audio events dropped", hs_gba_audio_events_dropped(g), 0u);
  TRUE_(C_DMA, "ROM1 DMA1 sound FIFO kept fed", g->dma[1].src > 0x08010000u + 90u * 250u);
  TRUE_(C_TIMER, "ROM1 Timer1 cascade counting", g->tm[1].counter >= 0xFFF0u);
  uint64_t rom1_frame = hs_gba_frame_hash(g);
  hs_gba_load_rom(g, rom1, TROM_SIZE);
  run_hashes r1b = run_rom(g, 90, -1, NULL, NULL);
  check(C_DET, r1.video == r1b.video && r1.audio == r1b.audio && r1.out == r1b.out, "ROM1 rerun bit-identical",
        r1b.video, r1.video);
  test_contract(g, rom1);

  /* ---- ROM 2 ---- */
  hs_gba_load_rom(g, rom2, TROM_SIZE);
  int e2 = 0;
  run_hashes r2 = run_rom(g, 15, 14, p3, &e2);
  {
    const uint16_t *fb = hs_gba_native_frame(g);
    EQ32(C_PPU, "Mode3 pixel (0,0)", fb[0], 2u);
    EQ32(C_PPU, "Mode3 pixel (239,159)", fb[159u * 240u + 239u],
         ((239u >> 3) + 2u) | ((159u >> 3) << 5) | (((239u + 159u) >> 4) << 10));
    EQ32(C_PPU, "Mode3 pixel (100,50)", fb[50u * 240u + 100u],
         ((100u >> 3) + 2u) | ((50u >> 3) << 5) | ((150u >> 4) << 10));
  }
  TRUE_(C_PPU, "Mode3 edge-to-edge 640x480", e2);
  run_hashes r2b = run_rom(g, 75, 20, p2, &e2);
  TRUE_(C_PPU, "Mode2 edge-to-edge 640x480 (wrap)", e2);
  TRUE_(C_PPU, "Mode2 frame differs from Mode3", r2b.last_frame_hash != r2.last_frame_hash);
  {
    uint64_t a1, a2;
    hs_gba_step_cpu_phase(g, 0); hs_gba_render_video_phase(g, NULL); a1 = hs_gba_frame_hash(g);
    hs_gba_step_cpu_phase(g, 0); hs_gba_render_video_phase(g, g_out); a2 = hs_gba_frame_hash(g);
    TRUE_(C_PPU, "Mode2 affine BG rotates frame-to-frame", a1 != a2);
    TRUE_(C_PPU, "Mode2 checker colours both present", count_color(g, 0x7FFF) > 1000u && count_color(g, 0x10A5) > 1000u);
    EQ32(C_PPU, "Mode2 DISPCNT", gba_read16(g, 0x04000000u), 0x0402u);
    TRUE_(C_PPU, "BgAffineSet drove BG2PA/PB (rotating)", gba_read16(g, 0x04000020u) != 0x100u || gba_read16(g, 0x04000022u) != 0u);
    int vdup = 1;
    for (uint32_t y = 0; y < 160; y++)
      vdup &= !memcmp(&g_out[(y * 3u) * 640u], &g_out[(y * 3u + 1u) * 640u], 640u * 4u) &&
              !memcmp(&g_out[(y * 3u) * 640u], &g_out[(y * 3u + 2u) * 640u], 640u * 4u);
    TRUE_(C_PPU, "upsampler: exact 3x vertical", vdup);
    TRUE_(C_PPU, "upsampler: LUT spans src 0..239", g->video.lut_src0[0] == 0 && g->video.lut_src1[639] == 239);
  }
  printf("\n== hs-gba bit-exact hashes ==\n");
  printf("ROM1 mode0+obj+dsound 90f: video=%016llx audio=%016llx out640=%016llx last=%016llx samples=%u\n",
         (unsigned long long)r1.video, (unsigned long long)r1.audio, (unsigned long long)r1.out,
         (unsigned long long)rom1_frame, r1.samples);
  printf("ROM2 mode3->mode2   90f: video=%016llx audio=%016llx out640=%016llx last=%016llx\n",
         (unsigned long long)fnv(r2.video, &r2b.video, 8), (unsigned long long)fnv(r2.audio, &r2b.audio, 8),
         (unsigned long long)r2b.out, (unsigned long long)r2b.last_frame_hash);
  printf("PNG: %s\nPNG: %s\nPNG: %s\n", p0, p2, p3);

  int tp = 0, tf = 0;
  printf("\n== hs-gba test summary ==\n");
  for (int c = 0; c < C_N; c++) {
    printf("%-12s %4d pass %3d fail\n", k_cat[c], g_pass[c], g_fail[c]);
    tp += g_pass[c]; tf += g_fail[c];
  }
  printf("TOTAL        %4d pass %3d fail\n", tp, tf);
  hs_gba_destroy(g); hs_gba_destroy(g_t);
  free(rom1); free(rom2); free(g_rom); free(g_out);
  return tf ? 1 : 0;
}
