/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * test_hs_n64.c - hs-n64 torture suite + real .z64 demo ROM run:
 *   VR4300 ALU/64-bit/branch-delay/branch-likely/loads-stores/exceptions,
 *   COP0 timer + MI interrupts + TLB, COP1 S/D precision + rounding + NaN,
 *   PI/SP/SI/AI DMA, .z64/.v64/.n64 loader, RSP fixed-point maths + HLE
 *   gfx/audio + scalar LLE, RDP raster features, zero-mutex snapshot
 *   isolation + pipelined equivalence, the demo ROM at 1x/FXAA/2x/4x/8x.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hs_test.h"
#include "../src/n64_internal.h"
#include "../src/n64_pf.h"

static hs_tstats_t TS;
static hs_n64_t *N;
static const char *OUT = "out/x86_64";
static int QUICK = 0;

/* ---------------------------------------------------------- encoders */
#define R_(fn, rs, rt, rd, sa) ((uint32_t)(((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((sa) << 6) | (fn)))
#define I_(op, rs, rt, imm) ((uint32_t)(((uint32_t)(op) << 26) | ((rs) << 21) | ((rt) << 16) | ((uint32_t)(imm) & 0xFFFFu)))
#define NOP 0u
#define ADDIU(t, s, i) I_(9, s, t, i)
#define DADDIU(t, s, i) I_(25, s, t, i)
#define ORI(t, s, i) I_(13, s, t, i)
#define LUI(t, i) I_(15, 0, t, i)
#define SLTI(t, s, i) I_(10, s, t, i)
#define SLTIU(t, s, i) I_(11, s, t, i)
#define ADD(d, s, t) R_(0x20, s, t, d, 0)
#define ADDU(d, s, t) R_(0x21, s, t, d, 0)
#define SUBU(d, s, t) R_(0x23, s, t, d, 0)
#define AND(d, s, t) R_(0x24, s, t, d, 0)
#define OR(d, s, t) R_(0x25, s, t, d, 0)
#define XOR(d, s, t) R_(0x26, s, t, d, 0)
#define NOR(d, s, t) R_(0x27, s, t, d, 0)
#define SLT(d, s, t) R_(0x2A, s, t, d, 0)
#define SLTU(d, s, t) R_(0x2B, s, t, d, 0)
#define DADD(d, s, t) R_(0x2C, s, t, d, 0)
#define DADDU(d, s, t) R_(0x2D, s, t, d, 0)
#define DSUBU(d, s, t) R_(0x2F, s, t, d, 0)
#define SLL(d, t, a) R_(0x00, 0, t, d, a)
#define SRL(d, t, a) R_(0x02, 0, t, d, a)
#define SRA(d, t, a) R_(0x03, 0, t, d, a)
#define SLLV(d, t, s) R_(0x04, s, t, d, 0)
#define SRAV(d, t, s) R_(0x07, s, t, d, 0)
#define DSLL(d, t, a) R_(0x38, 0, t, d, a)
#define DSRL(d, t, a) R_(0x3A, 0, t, d, a)
#define DSRA(d, t, a) R_(0x3B, 0, t, d, a)
#define DSLL32(d, t, a) R_(0x3C, 0, t, d, a)
#define DSRL32(d, t, a) R_(0x3E, 0, t, d, a)
#define DSRA32(d, t, a) R_(0x3F, 0, t, d, a)
#define DSLLV(d, t, s) R_(0x14, s, t, d, 0)
#define MULT(s, t) R_(0x18, s, t, 0, 0)
#define MULTU(s, t) R_(0x19, s, t, 0, 0)
#define DIV(s, t) R_(0x1A, s, t, 0, 0)
#define DIVU(s, t) R_(0x1B, s, t, 0, 0)
#define DMULT(s, t) R_(0x1C, s, t, 0, 0)
#define DMULTU(s, t) R_(0x1D, s, t, 0, 0)
#define DDIV(s, t) R_(0x1E, s, t, 0, 0)
#define DDIVU(s, t) R_(0x1F, s, t, 0, 0)
#define MFHI(d) R_(0x10, 0, 0, d, 0)
#define MFLO(d) R_(0x12, 0, 0, d, 0)
#define JR(s) R_(0x08, s, 0, 0, 0)
#define JALR(d, s) R_(0x09, s, 0, d, 0)
#define SYSCALL R_(0x0C, 0, 0, 0, 0)
#define TEQ(s, t) R_(0x34, s, t, 0, 0)
#define BEQ(s, t, o) I_(4, s, t, o)
#define BNE(s, t, o) I_(5, s, t, o)
#define BLEZ(s, o) I_(6, s, 0, o)
#define BGTZ(s, o) I_(7, s, 0, o)
#define BEQL(s, t, o) I_(20, s, t, o)
#define BNEL(s, t, o) I_(21, s, t, o)
#define BLTZ(s, o) I_(1, s, 0, o)
#define BGEZ(s, o) I_(1, s, 1, o)
#define BLTZL(s, o) I_(1, s, 2, o)
#define BGEZAL(s, o) I_(1, s, 17, o)
#define J_(op, tgt) ((uint32_t)(((uint32_t)(op) << 26) | (((tgt) >> 2) & 0x03FFFFFFu)))
#define LB(t, o, b) I_(32, b, t, o)
#define LH(t, o, b) I_(33, b, t, o)
#define LWL(t, o, b) I_(34, b, t, o)
#define LW(t, o, b) I_(35, b, t, o)
#define LBU(t, o, b) I_(36, b, t, o)
#define LHU(t, o, b) I_(37, b, t, o)
#define LWR(t, o, b) I_(38, b, t, o)
#define LWU(t, o, b) I_(39, b, t, o)
#define SB(t, o, b) I_(40, b, t, o)
#define SH(t, o, b) I_(41, b, t, o)
#define SWL(t, o, b) I_(42, b, t, o)
#define SW(t, o, b) I_(43, b, t, o)
#define SDL(t, o, b) I_(44, b, t, o)
#define SDR(t, o, b) I_(45, b, t, o)
#define SWR(t, o, b) I_(46, b, t, o)
#define LDL(t, o, b) I_(26, b, t, o)
#define LDR(t, o, b) I_(27, b, t, o)
#define LD(t, o, b) I_(55, b, t, o)
#define SD(t, o, b) I_(63, b, t, o)
#define LWC1(t, o, b) I_(49, b, t, o)
#define SWC1(t, o, b) I_(57, b, t, o)
#define LDC1(t, o, b) I_(53, b, t, o)
#define SDC1(t, o, b) I_(61, b, t, o)
#define MTC0(t, d) ((0x10u << 26) | (4u << 21) | ((uint32_t)(t) << 16) | ((uint32_t)(d) << 11))
#define MFC0(t, d) ((0x10u << 26) | (0u << 21) | ((uint32_t)(t) << 16) | ((uint32_t)(d) << 11))
#define ERET 0x42000018u
#define TLBWI 0x42000002u
#define TLBP 0x42000008u
#define TLBR 0x42000001u
#define MTC1(t, f) ((0x11u << 26) | (4u << 21) | ((uint32_t)(t) << 16) | ((uint32_t)(f) << 11))
#define MFC1(t, f) ((0x11u << 26) | (0u << 21) | ((uint32_t)(t) << 16) | ((uint32_t)(f) << 11))
#define FOP(fmt, fn, d, s, t) ((0x11u << 26) | ((uint32_t)(fmt) << 21) | ((uint32_t)(t) << 16) | ((uint32_t)(s) << 11) | ((uint32_t)(d) << 6) | (uint32_t)(fn))
#define BC1(tf, lk, o) ((0x11u << 26) | (8u << 21) | ((uint32_t)(lk) << 17) | ((uint32_t)(tf) << 16) | ((uint32_t)(o) & 0xFFFFu))
#define FS 16
#define FD 17
#define FW 20
#define FL 21

enum { T0 = 8, T1, T2, T3, T4, T5, T6, T7, S0 = 16, S1, S2, S3, K0 = 26, K1 = 27, RA = 31 };

static uint32_t ppos;
static void P(uint32_t w) { n64_wbe32(N->rdram + ppos, w); ppos += 4u; }
static void HALT(void) { P(BEQ(0, 0, -1)); P(NOP); }
static void LI(uint32_t r, uint32_t v) { P(LUI(r, v >> 16)); P(ORI(r, r, v & 0xFFFFu)); }

static void cpu_setup(uint32_t status)
{
    n64_cpu_reset(N);
    memset(N->rdram, 0, 0x20000);
    N->mi_intr = 0; N->mi_mask = 0;
    N->cpu.cp0[C0_STATUS] = status;
    N->cpu.pc = 0xFFFFFFFF80001000ull;
    N->cpu.npc = N->cpu.pc + 4u;
    ppos = 0x1000u;
}
static void at(uint32_t pa) { ppos = pa; }
static void run(uint32_t n) { n64_cpu_run(N, n); }
static uint64_t R(uint32_t r) { return N->cpu.gpr[r]; }
static uint32_t K1CODE(void) { return (uint32_t)(N->cpu.gpr[K1] >> 2) & 31u; }
/* handler @0x180: k1 = Cause, EPC += 4, ERET */
static void std_handler(void)
{
    uint32_t keep = ppos;
    at(0x180u);
    P(MFC0(K1, C0_CAUSE)); P(MFC0(K0, C0_EPC)); P(ADDIU(K0, K0, 4)); P(MTC0(K0, C0_EPC)); P(ERET);
    ppos = keep;
}

/* ================================================================= CPU */
static void test_cpu_alu(void)
{
    printf("[cpu] ALU 32-bit\n");
    cpu_setup(0);
    P(LUI(T0, 0x1234)); P(ORI(T0, T0, 0x5678));
    LI(T1, 0x7FFFFFFFu); P(ADDIU(T2, T1, 1));
    P(ADDIU(T3, 0, -5)); P(SLTI(T4, T3, 0)); P(SLTIU(T5, T3, 3));
    P(SUBU(T6, 0, T1)); P(NOR(T7, 0, 0));
    LI(S0, 0xF0F0F0F0u); LI(S1, 0x0FF00FF0u);
    P(AND(S2, S0, S1)); P(OR(S3, S0, S1)); P(XOR(2, S0, S1));
    P(SLT(3, T3, 0)); P(SLTU(4, T3, 0));
    HALT();
    run(30);
    HS_CHECK(&TS, R(T0) == 0x12345678u, "lui/ori %llx", (unsigned long long)R(T0));
    HS_CHECK(&TS, R(T2) == 0xFFFFFFFF80000000ull, "addiu sign-extended wrap %llx", (unsigned long long)R(T2));
    HS_CHECK(&TS, R(T3) == 0xFFFFFFFFFFFFFFFBull, "addiu negative");
    HS_CHECK(&TS, R(T4) == 1u && R(T5) == 0u, "slti/sltiu");
    HS_CHECK(&TS, R(T6) == 0xFFFFFFFF80000001ull, "subu");
    HS_CHECK(&TS, R(T7) == ~0ull, "nor");
    HS_CHECK(&TS, R(S2) == 0x00F000F0u, "and");
    HS_CHECK(&TS, R(S3) == 0xFFFFFFFFFFF0FFF0ull, "or sign-ext source");
    HS_CHECK(&TS, R(2) == 0xFFFFFFFFFF00FF00ull, "xor");
    HS_CHECK(&TS, R(3) == 1u && R(4) == 0u, "slt/sltu signedness");

    cpu_setup(0); std_handler();
    LI(T1, 0x7FFFFFFFu); P(ADDIU(T2, 0, 7)); P(ADD(T2, T1, T1)); P(ADDIU(T3, 0, 9)); HALT();
    run(20);
    HS_CHECK(&TS, K1CODE() == EXC_OV, "ADD overflow raises Ov (code %u)", K1CODE());
    HS_CHECK(&TS, R(T2) == 7u, "ADD overflow leaves rd unchanged");
    HS_CHECK(&TS, R(T3) == 9u, "execution resumes after ERET");

    printf("[cpu] shifts\n");
    cpu_setup(0);
    LI(T0, 0x80000001u); P(SLL(T1, T0, 4)); P(SRL(T2, T0, 4)); P(SRA(T3, T0, 4));
    P(ADDIU(T4, 0, 33)); P(SLLV(T5, T0, T4)); P(SRAV(T6, T0, T4)); HALT();
    run(20);
    HS_CHECK(&TS, R(T1) == 0x10u, "sll %llx", (unsigned long long)R(T1));
    HS_CHECK(&TS, R(T2) == 0x08000000u, "srl");
    HS_CHECK(&TS, R(T3) == 0xFFFFFFFFF8000000ull, "sra");
    HS_CHECK(&TS, R(T5) == 0x2u, "sllv uses low 5 bits");
    HS_CHECK(&TS, R(T6) == 0xFFFFFFFFC0000000ull, "srav");
}

static void test_cpu_64(void)
{
    printf("[cpu] MIPS III 64-bit\n");
    cpu_setup(0); std_handler();
    LI(T0, 0x12345678u); P(DSLL32(T0, T0, 0)); LI(T1, 0x9ABCDEF0u); P(DSLL32(T1, T1, 0)); P(DSRL32(T1, T1, 0));
    P(DADDU(T2, T0, T1));
    P(DSRL(T3, T2, 8)); P(DSRA32(T4, T2, 4)); P(DSLL(T5, T2, 4));
    P(DSUBU(T6, T2, T0)); P(DADDIU(T7, 0, -1)); P(DSRA(S0, T7, 31)); P(DSRA32(S0, S0, 31));
    P(ADDIU(S1, 0, 36)); P(DSLLV(S2, T1, S1));
    LI(S3, 0x7FFFFFFFu); P(DSLL32(S3, S3, 0)); P(ORI(2, 0, 0)); P(ADDIU(K1, 0, 0));
    P(DADD(2, S3, S3));
    HALT();
    run(40);
    HS_CHECK(&TS, R(T2) == 0x123456789ABCDEF0ull, "64-bit compose %llx", (unsigned long long)R(T2));
    HS_CHECK(&TS, R(T3) == 0x00123456789ABCDEull, "dsrl");
    HS_CHECK(&TS, R(T4) == 0x0000000001234567ull, "dsra32");
    HS_CHECK(&TS, R(T5) == 0x23456789ABCDEF00ull, "dsll");
    HS_CHECK(&TS, R(T6) == 0x9ABCDEF0ull, "dsubu");
    HS_CHECK(&TS, R(S0) == ~0ull, "dsra/dsra32 of -1");
    HS_CHECK(&TS, R(S2) == 0xABCDEF0000000000ull, "dsllv 36 %llx", (unsigned long long)R(S2));
    HS_CHECK(&TS, K1CODE() == EXC_OV && R(2) == 0u, "DADD 64-bit overflow trap");

    printf("[cpu] multiply / divide\n");
    cpu_setup(0);
    P(ADDIU(T0, 0, -3)); P(ADDIU(T1, 0, 7)); P(MULT(T0, T1)); P(MFLO(T2)); P(MFHI(T3));
    P(MULTU(T0, T1)); P(MFLO(T4)); P(MFHI(T5));
    P(ADDIU(T6, 0, -7)); P(ADDIU(T7, 0, 2)); P(DIV(T6, T7)); P(MFLO(S0)); P(MFHI(S1));
    P(DIV(T6, 0)); P(MFLO(S2)); P(MFHI(S3));
    HALT();
    run(40);
    HS_CHECK(&TS, R(T2) == (uint64_t)-21LL && R(T3) == ~0ull, "mult signed");
    HS_CHECK(&TS, R(T4) == 0xFFFFFFFFFFFFFFEBull && R(T5) == 6u, "multu %llx %llx", (unsigned long long)R(T4), (unsigned long long)R(T5));
    HS_CHECK(&TS, R(S0) == (uint64_t)-3LL && R(S1) == (uint64_t)-1LL, "div truncates toward zero");
    HS_CHECK(&TS, R(S2) == 1u && R(S3) == (uint64_t)-7LL, "div by zero (negative dividend)");
    cpu_setup(0);
    LI(T0, 0x80000000u); P(ADDIU(T1, 0, -1)); P(DIV(T0, T1)); P(MFLO(T2)); P(MFHI(T3));
    P(DIVU(T1, 0)); P(MFLO(T4));
    P(DADDIU(T5, 0, -1)); P(DMULTU(T5, T5)); P(MFLO(T6)); P(MFHI(T7));
    P(DMULT(T5, T5)); P(MFLO(S0)); P(MFHI(S1));
    LI(S2, 1000000007u); P(DSLL32(S3, S2, 0)); P(DSRL32(S2, S3, 0)); P(DDIVU(S3, S2)); P(MFLO(2)); P(MFHI(3));
    P(ADDIU(4, 0, -9)); P(ADDIU(5, 0, 4)); P(DDIV(4, 5)); P(MFLO(6)); P(MFHI(7));
    HALT();
    run(60);
    HS_CHECK(&TS, R(T2) == 0xFFFFFFFF80000000ull && R(T3) == 0u, "INT_MIN / -1");
    HS_CHECK(&TS, R(T4) == ~0ull, "divu by zero");
    HS_CHECK(&TS, R(T6) == 1u && R(T7) == 0xFFFFFFFFFFFFFFFEull, "dmultu 128-bit hi/lo");
    HS_CHECK(&TS, R(S0) == 1u && R(S1) == 0u, "dmult (-1*-1)");
    HS_CHECK(&TS, R(2) == 4294967296ull && R(3) == 0u, "ddivu %llu", (unsigned long long)R(2));
    HS_CHECK(&TS, R(6) == (uint64_t)-2LL && R(7) == (uint64_t)-1LL, "ddiv");
}

static void test_cpu_branch(void)
{
    printf("[cpu] branch delay slots + branch-likely\n");
    cpu_setup(0);
    P(BEQ(0, 0, 2)); P(ADDIU(T0, 0, 1)); P(ADDIU(T1, 0, 1)); P(ADDIU(T2, 0, 1));
    P(ADDIU(T3, 0, 5)); P(BNE(T3, T3, 2)); P(ADDIU(T4, 0, 1)); P(ADDIU(T5, 0, 1));
    P(BEQL(T3, 0, 2)); P(ADDIU(T6, 0, 1)); P(ADDIU(T7, 0, 1));
    P(BNEL(T3, 0, 2)); P(ADDIU(S0, 0, 1)); P(ADDIU(S1, 0, 1)); P(ADDIU(S2, 0, 1));
    P(BGEZAL(0, 2)); P(NOP); P(ADDIU(S3, 0, 1)); P(NOP);
    HALT();
    run(40);
    HS_CHECK(&TS, R(T0) == 1u && R(T1) == 0u && R(T2) == 1u, "BEQ taken executes delay slot only");
    HS_CHECK(&TS, R(T4) == 1u && R(T5) == 1u, "BNE not taken falls through");
    HS_CHECK(&TS, R(T6) == 0u && R(T7) == 1u, "BEQL not taken nullifies delay slot");
    HS_CHECK(&TS, R(S0) == 1u && R(S1) == 0u && R(S2) == 1u, "BNEL taken executes delay slot");
    HS_CHECK(&TS, R(RA) == 0xFFFFFFFF80001044ull, "BGEZAL links pc+8 (%llx)", (unsigned long long)R(RA));
    HS_CHECK(&TS, R(S3) == 0u, "BGEZAL taken skips");

    cpu_setup(0);
    P(J_(3, 0x80001020u)); P(ADDIU(T0, 0, 3)); P(ADDIU(T1, 0, 1)); P(NOP);
    P(NOP); P(NOP); P(NOP); P(NOP);
    /* 0x1020 */ P(ADDIU(T2, RA, 0)); LI(T3, 0x80001040u); P(JALR(T4, T3)); P(ADDIU(T5, 0, 9));
    /* 0x1034 */ P(ADDIU(T7, 0, 6)); HALT();
    at(0x1040u); P(JR(T4)); P(ADDIU(T6, 0, 4));
    run(40);
    HS_CHECK(&TS, R(T0) == 3u && R(T1) == 0u, "JAL delay slot");
    HS_CHECK(&TS, R(T2) == 0xFFFFFFFF80001008ull, "JAL link register");
    HS_CHECK(&TS, R(T4) == 0xFFFFFFFF80001034ull && R(T5) == 9u, "JALR link + delay slot (%llx)", (unsigned long long)R(T4));
    HS_CHECK(&TS, R(T6) == 4u && R(T7) == 6u, "JR returns through link");

    cpu_setup(0);
    P(ADDIU(T0, 0, -1)); P(BLTZ(T0, 2)); P(NOP); P(ADDIU(T1, 0, 1)); P(BLEZ(0, 2)); P(NOP); P(ADDIU(T2, 0, 1));
    P(BGTZ(0, 2)); P(NOP); P(ADDIU(T3, 0, 1)); P(BLTZL(0, 2)); P(ADDIU(T4, 0, 1)); P(BGEZ(0, 2)); P(NOP); P(ADDIU(T5, 0, 1));
    HALT();
    run(40);
    HS_CHECK(&TS, R(T1) == 0u && R(T2) == 0u && R(T3) == 1u && R(T4) == 0u && R(T5) == 0u, "BLTZ/BLEZ/BGTZ/BLTZL/BGEZ");
}

static void test_cpu_mem(void)
{
    printf("[cpu] loads / stores (big-endian, unaligned family)\n");
    cpu_setup(0);
    LI(S0, 0x80002000u);
    LI(T0, 0x8899AABBu); P(SW(T0, 0, S0)); P(SH(T0, 4, S0)); P(SB(T0, 6, S0));
    P(LB(T1, 0, S0)); P(LBU(T2, 0, S0)); P(LH(T3, 2, S0)); P(LHU(T4, 2, S0)); P(LW(T5, 0, S0)); P(LWU(T6, 0, S0));
    LI(T7, 0x11223344u); P(SW(T7, 8, S0)); LI(T7, 0x55667788u); P(SW(T7, 12, S0));
    P(ADDIU(S1, 0, 0)); P(LWL(S1, 9, S0)); P(LWR(S1, 12, S0));
    P(LD(S2, 8, S0));
    P(ADDIU(S3, 0, 0)); P(LDL(S3, 10, S0)); P(LDR(S3, 17, S0));
    HALT();
    at(0x2010u); P(0x99AABBCCu); P(0xDDEEFF00u);
    run(60);
    HS_CHECK(&TS, n64_be32(N->rdram + 0x2000) == 0x8899AABBu, "SW big-endian bytes");
    HS_CHECK(&TS, n64_be16(N->rdram + 0x2004) == 0xAABBu && N->rdram[0x2006] == 0xBBu, "SH/SB");
    HS_CHECK(&TS, R(T1) == 0xFFFFFFFFFFFFFF88ull && R(T2) == 0x88u, "LB/LBU");
    HS_CHECK(&TS, R(T3) == 0xFFFFFFFFFFFFAABBull && R(T4) == 0xAABBu, "LH/LHU");
    HS_CHECK(&TS, R(T5) == 0xFFFFFFFF8899AABBull && R(T6) == 0x8899AABBull, "LW/LWU");
    HS_CHECK(&TS, R(S1) == 0x22334455u, "LWL+LWR unaligned word (%llx)", (unsigned long long)R(S1));
    HS_CHECK(&TS, R(S2) == 0x1122334455667788ull, "LD");
    HS_CHECK(&TS, R(S3) == 0x33445566778899AAull, "LDL+LDR unaligned doubleword (%llx)", (unsigned long long)R(S3));

    cpu_setup(0);
    LI(S0, 0x80002000u); LI(T0, 0xA1B2C3D4u);
    P(SWL(T0, 1, S0)); P(SWR(T0, 4, S0));
    LI(T1, 0x01020304u); P(DSLL32(T1, T1, 0)); LI(T2, 0x05060708u); P(DSLL32(T2, T2, 0)); P(DSRL32(T2, T2, 0)); P(DADDU(T1, T1, T2));
    P(SDL(T1, 0x13, S0)); P(SDR(T1, 0x1A, S0));
    P(SD(T1, 0x28, S0));
    HALT();
    memset(N->rdram + 0x2000, 0xEE, 0x40);
    run(40);
    HS_CHECK(&TS, n64_be32(N->rdram + 0x2000) == 0xEEA1B2C3u && n64_be32(N->rdram + 0x2004) == 0xD4EEEEEEu, "SWL+SWR unaligned store (%08x %08x)",
             n64_be32(N->rdram + 0x2000), n64_be32(N->rdram + 0x2004));
    HS_CHECK(&TS, n64_be32(N->rdram + 0x2010) == 0xEEEEEE01u && n64_be32(N->rdram + 0x2014) == 0x02030405u &&
                  n64_be32(N->rdram + 0x2018) == 0x060708EEu, "SDL+SDR unaligned doubleword store (%08x %08x %08x)",
             n64_be32(N->rdram + 0x2010), n64_be32(N->rdram + 0x2014), n64_be32(N->rdram + 0x2018));
    HS_CHECK(&TS, n64_be32(N->rdram + 0x2028) == 0x01020304u && n64_be32(N->rdram + 0x202C) == 0x05060708u, "SD");

    cpu_setup(0); std_handler();
    LI(S0, 0x80002001u); P(LW(T0, 0, S0)); P(ADDIU(T1, 0, 1)); HALT();
    run(20);
    HS_CHECK(&TS, K1CODE() == EXC_ADEL && N->cpu.cp0[C0_BADVADDR] == 0xFFFFFFFF80002001ull, "misaligned LW -> AdEL + BadVAddr");
    HS_CHECK(&TS, R(T1) == 1u, "resume after AdEL");
}

static void test_cpu_exc(void)
{
    printf("[cpu] COP0 exceptions / timer / interrupts\n");
    cpu_setup(0); std_handler();
    P(SYSCALL); P(ADDIU(T0, 0, 1)); P(ADDIU(T1, 0, 2)); P(TEQ(0, 0)); P(ADDIU(T2, 0, 3)); HALT();
    run(30);
    HS_CHECK(&TS, R(T0) == 1u && R(T1) == 2u && R(T2) == 3u, "SYSCALL + TEQ trap + ERET resume");
    HS_CHECK(&TS, K1CODE() == EXC_TR, "last exception = Tr");
    HS_CHECK(&TS, !(N->cpu.cp0[C0_STATUS] & ST_EXL), "ERET clears EXL");

    cpu_setup(0);
    at(0x180u); P(MFC0(K1, C0_CAUSE)); P(MFC0(K0, C0_EPC)); HALT();
    at(0x1000u); P(BEQ(0, 0, 4)); P(SYSCALL); HALT();
    run(10);
    HS_CHECK(&TS, (R(K1) >> 31) & 1u, "Cause.BD set for delay-slot exception");
    HS_CHECK(&TS, R(K0) == 0xFFFFFFFF80001000ull, "EPC points at the branch");
    HS_CHECK(&TS, K1CODE() == EXC_SYS, "Sys code");

    cpu_setup(0x00008001u);
    at(0x180u); P(MFC0(K1, C0_CAUSE)); P(MFC0(K0, C0_COUNT)); P(MTC0(0, C0_COMPARE)); P(ERET);
    at(0x1000u); P(MFC0(T0, C0_COUNT)); P(ADDIU(T0, T0, 50)); P(MTC0(T0, C0_COMPARE));
    P(BEQ(0, 0, -1)); P(NOP);
    run(200);
    HS_CHECK(&TS, (R(K1) & 0x8000u) && K1CODE() == EXC_INT, "Count==Compare raises IP7 interrupt");
    HS_CHECK(&TS, (uint32_t)R(K0) >= (uint32_t)R(T0), "interrupt taken at/after Compare");
    HS_CHECK(&TS, !(N->cpu.cp0[C0_CAUSE] & 0x8000u), "MTC0 Compare acks IP7");

    cpu_setup(0x00000401u);
    at(0x180u); P(MFC0(K1, C0_CAUSE)); LI(K0, 0xA4400010u); P(SW(0, 0, K0)); P(ERET);
    at(0x1000u); P(ADDIU(T0, 0, 0)); P(BEQ(0, 0, -1)); P(ADDIU(T0, T0, 1));
    run(10);
    HS_CHECK(&TS, R(K1) == 0u, "no interrupt while MI mask clear");
    n64_phys_write32(N, 0x0430000Cu, 0x80u, 0xFFFFFFFFu);
    n64_mi_raise(N, MI_VI);
    run(20);
    HS_CHECK(&TS, (R(K1) & 0x400u) && K1CODE() == EXC_INT, "VI via MI raises IP2");
    HS_CHECK(&TS, !(N->mi_intr & MI_VI), "VI_CURRENT write acks the VI interrupt");
    HS_CHECK(&TS, R(T0) > 3u, "main loop keeps running after interrupt");

    printf("[cpu] TLB\n");
    cpu_setup(0);
    at(0x000u); P(MFC0(K1, C0_CAUSE)); P(MFC0(K0, C0_BADVADDR)); HALT();
    at(0x1000u);
    P(ADDIU(T0, 0, 5)); P(MTC0(T0, C0_INDEX)); P(MTC0(0, C0_PAGEMASK));
    LI(T1, 0x00400000u); P(MTC0(T1, C0_ENTRYHI));
    LI(T2, ((0x00200000u >> 12) << 6) | 0x7u); P(MTC0(T2, C0_ENTRYLO0));
    LI(T2, ((0x00201000u >> 12) << 6) | 0x3u); P(MTC0(T2, C0_ENTRYLO1));
    P(TLBWI);
    LI(S0, 0x00400010u); P(LW(T3, 0, S0)); P(LW(T4, 0x1000, S0));
    P(MTC0(0, C0_INDEX)); P(TLBP); P(MFC0(T5, C0_INDEX));
    P(MTC0(0, C0_ENTRYLO0)); P(TLBR); P(MFC0(T6, C0_ENTRYLO0));
    LI(S1, 0x00800000u); P(LW(T7, 0, S1));
    HALT();
    n64_wbe32(N->rdram + 0x200010, 0xCAFEF00Du);
    n64_wbe32(N->rdram + 0x201010, 0x0BADBEEFu);
    run(60);
    HS_CHECK(&TS, R(T3) == 0xFFFFFFFFCAFEF00Dull, "TLB even page maps 0x00400000->0x00200000");
    HS_CHECK(&TS, R(T4) == 0x0BADBEEFu, "TLB odd page");
    HS_CHECK(&TS, R(T5) == 5u, "TLBP finds entry 5");
    HS_CHECK(&TS, (R(T6) >> 6) == (0x00200000u >> 12), "TLBR reads back EntryLo0");
    HS_CHECK(&TS, K1CODE() == EXC_TLBL && (uint32_t)R(K0) == 0x00800000u, "TLB miss -> refill vector, TLBL, BadVAddr");
    cpu_setup(0);
    at(0x180u); P(MFC0(K1, C0_CAUSE)); HALT();
    at(0x1000u);
    P(MTC0(0, C0_INDEX)); P(MTC0(0, C0_PAGEMASK)); LI(T1, 0x00400000u); P(MTC0(T1, C0_ENTRYHI));
    LI(T2, ((0x00200000u >> 12) << 6) | 0x3u); P(MTC0(T2, C0_ENTRYLO0)); P(MTC0(T2, C0_ENTRYLO1)); P(TLBWI);
    LI(S0, 0x00400000u); P(SW(0, 0, S0)); HALT();
    run(40);
    HS_CHECK(&TS, K1CODE() == EXC_MOD, "store to clean (D=0) page -> TLB Mod");
}

/* ================================================================= FPU */
static uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static uint64_t d2u(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }

static void fexec(uint32_t ins)
{
    at(0x1000u); P(ins); HALT();
    N->cpu.pc = 0xFFFFFFFF80001000ull; N->cpu.npc = N->cpu.pc + 4u; N->cpu.next_delay = 0;
    run(1);
}

static void fpu_reset(uint32_t status)
{
    uint32_t i;
    cpu_setup(status);
    for (i = 0; i < 32u; i++) N->cpu.fpr[i] = 0;
}

static void test_fpu(void)
{
    uint32_t i;
    printf("[fpu] COP1 single/double precision\n");
    fpu_reset(ST_CU1 | ST_FR);
    N->cpu.fpr[1] = f2u(1.5f); N->cpu.fpr[2] = f2u(2.25f);
    fexec(FOP(FS, 0, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 0x40700000u, "ADD.S 1.5+2.25");
    fexec(FOP(FS, 1, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(-0.75f), "SUB.S");
    fexec(FOP(FS, 2, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(3.375f), "MUL.S");
    N->cpu.fpr[1] = f2u(1.0f); N->cpu.fpr[2] = f2u(3.0f);
    fexec(FOP(FS, 3, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 0x3EAAAAABu, "DIV.S 1/3 correctly rounded");
    N->cpu.fpr[1] = f2u(2.0f);
    fexec(FOP(FS, 4, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 0x3FB504F3u, "SQRT.S 2");
    N->cpu.fpr[1] = d2u(1.0); N->cpu.fpr[2] = d2u(3.0);
    fexec(FOP(FD, 3, 3, 1, 2)); HS_CHECK(&TS, N->cpu.fpr[3] == 0x3FD5555555555555ull, "DIV.D 1/3");
    N->cpu.fpr[1] = d2u(2.0);
    fexec(FOP(FD, 4, 3, 1, 0)); HS_CHECK(&TS, N->cpu.fpr[3] == 0x3FF6A09E667F3BCDull, "SQRT.D 2");
    N->cpu.fpr[1] = d2u(0.1); N->cpu.fpr[2] = d2u(0.2);
    fexec(FOP(FD, 0, 3, 1, 2)); HS_CHECK(&TS, N->cpu.fpr[3] == 0x3FD3333333333334ull, "ADD.D 0.1+0.2 exact IEEE");
    N->cpu.fpr[1] = f2u(-3.5f);
    fexec(FOP(FS, 5, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(3.5f), "ABS.S");
    fexec(FOP(FS, 7, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(3.5f), "NEG.S");
    fexec(FOP(FS, 6, 4, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[4] == f2u(-3.5f), "MOV.S");
    N->cpu.fpr[1] = 0; N->cpu.fpr[2] = 0;
    fexec(FOP(FS, 3, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 0x7FBFFFFFu, "0/0 -> VR4300 canonical single qNaN");
    fexec(FOP(FD, 3, 3, 1, 2)); HS_CHECK(&TS, N->cpu.fpr[3] == 0x7FF7FFFFFFFFFFFFull, "0/0 -> canonical double qNaN");
    N->cpu.fpr[1] = 0x00800000u; N->cpu.fpr[2] = f2u(0.5f);
    fexec(FOP(FS, 2, 3, 1, 2)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 0x00400000u, "denormal result kept (no flush)");

    printf("[fpu] conversions + FCR31 rounding modes\n");
    {
        static const float in[4] = { 2.5f, -2.5f, 1.5f, -0.5f };
        static const int32_t ex[4][4] = { { 2, 2, 3, 2 }, { -2, -2, -2, -3 }, { 2, 1, 2, 1 }, { 0, 0, 0, -1 } };
        uint32_t rm, k;
        for (k = 0; k < 4u; k++)
            for (rm = 0; rm < 4u; rm++) {
                N->cpu.fpr[1] = f2u(in[k]); N->cpu.fcr31 = rm;
                fexec(FOP(FS, 0x24, 3, 1, 0));
                HS_CHECK(&TS, (int32_t)(uint32_t)N->cpu.fpr[3] == ex[k][rm], "CVT.W.S %g rm=%u -> %d", (double)in[k], rm, (int32_t)(uint32_t)N->cpu.fpr[3]);
            }
        N->cpu.fcr31 = 0;
        N->cpu.fpr[1] = f2u(1.5f);
        fexec(FOP(FS, 0x0C, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 2u, "ROUND.W.S");
        fexec(FOP(FS, 0x0D, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 1u, "TRUNC.W.S");
        fexec(FOP(FS, 0x0E, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 2u, "CEIL.W.S");
        fexec(FOP(FS, 0x0F, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == 1u, "FLOOR.W.S");
        N->cpu.fpr[1] = d2u(-1e10);
        fexec(FOP(FD, 0x09, 3, 1, 0)); HS_CHECK(&TS, N->cpu.fpr[3] == (uint64_t)-10000000000LL, "TRUNC.L.D");
        N->cpu.fpr[1] = (uint64_t)-7LL;
        fexec(FOP(FL, 0x21, 3, 1, 0)); HS_CHECK(&TS, N->cpu.fpr[3] == d2u(-7.0), "CVT.D.L");
        N->cpu.fpr[1] = 16777217u;
        fexec(FOP(FW, 0x20, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(16777216.0f), "CVT.S.W rounds to nearest even");
        N->cpu.fpr[1] = f2u(0.1f);
        fexec(FOP(FS, 0x21, 3, 1, 0)); HS_CHECK(&TS, N->cpu.fpr[3] == d2u((double)0.1f), "CVT.D.S exact widening");
        N->cpu.fpr[1] = d2u(0.1);
        fexec(FOP(FD, 0x20, 3, 1, 0)); HS_CHECK(&TS, (uint32_t)N->cpu.fpr[3] == f2u(0.1f), "CVT.S.D rounding");
    }

    printf("[fpu] compare + BC1 + FR=0 pairs + CU1\n");
    fpu_reset(ST_CU1 | ST_FR);
    N->cpu.fpr[1] = f2u(1.0f); N->cpu.fpr[2] = f2u(2.0f);
    at(0x1000u);
    P(FOP(FS, 0x3C, 0, 1, 2));
    P(BC1(1, 0, 2)); P(ADDIU(T0, 0, 1)); P(ADDIU(T1, 0, 1));
    P(FOP(FD, 0x32, 0, 1, 2));
    P(BC1(0, 0, 2)); P(NOP); P(ADDIU(T2, 0, 1));
    P(BC1(1, 1, 2)); P(ADDIU(T3, 0, 1)); P(ADDIU(T4, 0, 1));
    HALT();
    run(30);
    HS_CHECK(&TS, R(T0) == 1u && R(T1) == 0u, "C.LT.S + BC1T taken");
    HS_CHECK(&TS, R(T2) == 0u, "C.EQ.D false + BC1F taken");
    HS_CHECK(&TS, R(T3) == 0u && R(T4) == 1u, "BC1TL not taken nullifies slot");
    fpu_reset(ST_CU1);
    at(0x1000u);
    LI(T0, 0x3F800000u); P(MTC1(T0, 0)); LI(T1, 0x40000000u); P(MTC1(T1, 1));
    LI(S0, 0x80003000u); P(SDC1(0, 0, S0)); P(LDC1(2, 0, S0)); P(MFC1(T2, 3)); P(LWC1(5, 4, S0)); P(SWC1(5, 8, S0));
    HALT();
    run(30);
    HS_CHECK(&TS, N->cpu.fpr[0] == 0x400000003F800000ull, "FR=0 even/odd single pairing");
    HS_CHECK(&TS, n64_be32(N->rdram + 0x3000) == 0x40000000u && n64_be32(N->rdram + 0x3004) == 0x3F800000u, "SDC1 big-endian");
    HS_CHECK(&TS, R(T2) == 0x40000000u, "LDC1 + MFC1 odd register");
    HS_CHECK(&TS, n64_be32(N->rdram + 0x3008) == 0x3F800000u, "LWC1/SWC1 (odd single = high half)");
    cpu_setup(0); std_handler();
    P(MTC1(0, 1)); P(ADDIU(T0, 0, 1)); HALT();
    run(10);
    HS_CHECK(&TS, K1CODE() == EXC_CPU && ((R(K1) >> 28) & 3u) == 1u, "COP1 unusable -> CpU CE=1");
    fpu_reset(ST_CU1 | ST_FR);
    N->cpu.fpr[1] = f2u(0.7f); N->cpu.fpr[2] = f2u(1.0001f); N->cpu.fpr[4] = d2u(0.3); N->cpu.fpr[5] = d2u(1.00000001);
    for (i = 0; i < 300u; i++) {
        fexec(FOP(FS, 2, 1, 1, 2)); fexec(FOP(FS, 0, 1, 1, 2)); fexec(FOP(FS, 4, 3, 1, 0)); fexec(FOP(FS, 3, 1, 1, 3));
        fexec(FOP(FD, 2, 4, 4, 5)); fexec(FOP(FD, 4, 6, 4, 0)); fexec(FOP(FD, 0, 4, 4, 6));
    }
    {
        uint64_t h = hs_fnv1a64(HS_FNV64_OFFSET, N->cpu.fpr, sizeof(N->cpu.fpr));
        printf("HASH fpu_stress %016llx\n", (unsigned long long)h);
        HS_CHECK(&TS, h != 0, "fpu stress hash");
    }
}

/* ================================================================= BUS */
static uint8_t *ROM;
static size_t ROMLEN;
static uint8_t *TROM[3];

static void core_init(hs_n64_t *n, uint32_t flags)
{
    hs_core_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.console_id = HS_SYS_N64;
    cfg.flags = flags;
    hs_n64_core_ops.init(n, sizeof(*n), &cfg);
}

static void test_bus(void)
{
    const size_t tl = 0x101000u;
    uint32_t c1, c2, i, f;
    uint32_t w0[3];
    printf("[bus] .z64/.v64/.n64 loader + CIC-6102 CRC + IPL3 HLE\n");
    for (f = 0; f < 3u; f++) TROM[f] = (uint8_t *)calloc(1, tl);
    {
        uint8_t *z = TROM[0];
        z[0] = 0x80; z[1] = 0x37; z[2] = 0x12; z[3] = 0x40;
        n64_wbe32(z + 8, 0x80000400u);
        memcpy(z + 0x20, "HS N64 LOADER TEST  ", 20);
        n64_wbe32(z + 0x1000, LUI(T0, 0x1234)); n64_wbe32(z + 0x1004, ORI(T0, T0, 0x5678));
        n64_wbe32(z + 0x1008, BEQ(0, 0, -1)); n64_wbe32(z + 0x100C, 0);
        for (i = 0x1010u; i < 0x2000u; i++) z[i] = (uint8_t)(i * 7u + 3u);
        n64_rom_crc(z, tl, &c1, &c2);
        n64_wbe32(z + 0x10, c1); n64_wbe32(z + 0x14, c2);
        for (i = 0; i < tl; i += 2u) { TROM[1][i] = z[i + 1u]; TROM[1][i + 1u] = z[i]; }
        for (i = 0; i < tl; i += 4u) { TROM[2][i] = z[i + 3u]; TROM[2][i + 1u] = z[i + 2u]; TROM[2][i + 2u] = z[i + 1u]; TROM[2][i + 3u] = z[i]; }
    }
    for (f = 0; f < 3u; f++) {
        core_init(N, 0);
        HS_CHECK(&TS, n64_rom_detect_swap(TROM[f], tl) == (int)f, "detect byte order %u", f);
        HS_CHECK(&TS, hs_n64_core_ops.load_rom(N, TROM[f], tl) == 0, "load_rom fmt %u", f);
        HS_CHECK(&TS, N->rom_crc_ok == 1u, "CIC-6102 CRC verifies (fmt %u)", f);
        w0[f] = n64_phys_read32(N, 0x10001010u);
        HS_CHECK(&TS, n64_be32(N->rdram + 0x400) == LUI(T0, 0x1234), "IPL3 HLE PI-DMA copied boot code to entry (fmt %u)", f);
        HS_CHECK(&TS, n64_be32(N->rdram + 0x318) == 4u << 20, "osMemSize = 4 MiB");
        n64_cpu_run(N, 10);
        HS_CHECK(&TS, R(T0) == 0x12345678u, "boot code executes from entry (fmt %u)", f);
        HS_CHECK(&TS, N->cpu.gpr[29] == 0xFFFFFFFFA4001FF0ull && N->cpu.gpr[22] == 0x3Fu, "CIC register state");
    }
    HS_CHECK(&TS, w0[0] == w0[1] && w0[1] == w0[2], "cart bus reads identical across z64/v64/n64");
    {
        static const uint8_t junk[64] = { 1, 2, 3 };
        core_init(N, 0);
        HS_CHECK(&TS, hs_n64_core_ops.load_rom(N, junk, sizeof(junk)) != 0, "rejects non-N64 image");
        core_init(N, HS_N64_CFG_EXPANSION_PAK);
        HS_CHECK(&TS, N->rdram_size == 8u << 20, "Expansion Pak = 8 MiB RDRAM");
    }

    printf("[bus] PI / SP / SI / AI / DP / MI\n");
    core_init(N, 0);
    hs_n64_core_ops.load_rom(N, TROM[0], tl);
    n64_phys_write32(N, 0x04600000u, 0x00100000u, ~0u);
    n64_phys_write32(N, 0x04600004u, 0x10001010u, ~0u);
    n64_phys_write32(N, 0x0460000Cu, 0xFFu, ~0u);
    HS_CHECK(&TS, memcmp(N->rdram + 0x100000, TROM[0] + 0x1010, 256) == 0, "PI DMA cart->RDRAM 256 bytes");
    HS_CHECK(&TS, (N->mi_intr & MI_PI) && (n64_phys_read32(N, 0x04600010u) & 8u), "PI DMA completion interrupt");
    n64_phys_write32(N, 0x04600010u, 2u, ~0u);
    HS_CHECK(&TS, !(N->mi_intr & MI_PI), "PI status clear acks");
    HS_CHECK(&TS, n64_phys_read32(N, 0x04600000u) == 0x00100100u, "PI DRAM address advances");

    for (i = 0; i < 64u; i++) N->rdram[0x2000 + i] = (uint8_t)(0xA0 + i);
    n64_phys_write32(N, 0x04040000u, 0x0u, ~0u);
    n64_phys_write32(N, 0x04040004u, 0x2000u, ~0u);
    n64_phys_write32(N, 0x04040008u, (8u << 20) | (1u << 12) | 15u, ~0u);
    HS_CHECK(&TS, memcmp(N->rsp.dmem, N->rdram + 0x2000, 16) == 0 && memcmp(N->rsp.dmem + 16, N->rdram + 0x2018, 16) == 0,
             "SP DMA RDRAM->DMEM with count/skip");
    n64_phys_write32(N, 0x04040000u, 0x0u, ~0u);
    n64_phys_write32(N, 0x04040004u, 0x3000u, ~0u);
    n64_phys_write32(N, 0x0404000Cu, 31u, ~0u);
    HS_CHECK(&TS, memcmp(N->rdram + 0x3000, N->rsp.dmem, 32) == 0, "SP DMA DMEM->RDRAM");
    HS_CHECK(&TS, n64_phys_read32(N, 0x0404001Cu) == 0u && n64_phys_read32(N, 0x0404001Cu) == 1u, "SP hardware flag read-sets");
    n64_phys_write32(N, 0x0404001Cu, 0u, ~0u);
    HS_CHECK(&TS, n64_phys_read32(N, 0x0404001Cu) == 0u, "SP hardware flag write-clears");

    memset(N->rdram + 0x4000, 0, 64);
    N->rdram[0x4000] = 0xFF; N->rdram[0x4001] = 0x01; N->rdram[0x4002] = 0x04; N->rdram[0x4003] = 0x01;
    N->rdram[0x4008] = 0xFE;
    N->input = HS_BTN_START | HS_N64_A | HS_BTN_RIGHT;
    n64_phys_write32(N, 0x04800000u, 0x4000u, ~0u);
    n64_phys_write32(N, 0x04800010u, 0x1FC007C0u, ~0u);
    n64_phys_write32(N, 0x04800000u, 0x4100u, ~0u);
    n64_phys_write32(N, 0x04800004u, 0x1FC007C0u, ~0u);
    HS_CHECK(&TS, N->rdram[0x4104] == 0x91u && N->rdram[0x4105] == 0x00u, "SI joybus controller read buttons (%02x %02x)",
             N->rdram[0x4104], N->rdram[0x4105]);
    HS_CHECK(&TS, (int8_t)N->rdram[0x4106] == 80 && N->rdram[0x4107] == 0u, "stick X from d-pad");
    HS_CHECK(&TS, (N->mi_intr & MI_SI) != 0, "SI interrupt");
    n64_phys_write32(N, 0x04800018u, 0u, ~0u);
    HS_CHECK(&TS, !(N->mi_intr & MI_SI), "SI status write acks");

    n64_back(N)->ai_n = 0;
    for (i = 0; i < 16u; i++) { n64_wbe16(N->rdram + 0x5000 + i * 4u, (uint16_t)(i * 100u)); n64_wbe16(N->rdram + 0x5002 + i * 4u, (uint16_t)(-(int)(i * 100u))); }
    n64_phys_write32(N, 0x04500010u, 1013u, ~0u);
    n64_phys_write32(N, 0x04500000u, 0x5000u, ~0u);
    n64_phys_write32(N, 0x04500004u, 64u, ~0u);
    HS_CHECK(&TS, n64_back(N)->ai_n == 16u && n64_back(N)->ai[2 * 15] == 1500 && n64_back(N)->ai[2 * 15 + 1] == -1500, "AI DMA captured into BACK snapshot");
    HS_CHECK(&TS, n64_back(N)->ai_freq == 48009u, "AI rate = VI clock/(dacrate+1) = %u", n64_back(N)->ai_freq);
    HS_CHECK(&TS, (N->mi_intr & MI_AI) != 0, "AI interrupt");
    n64_phys_write32(N, 0x0450000Cu, 0u, ~0u);
    HS_CHECK(&TS, !(N->mi_intr & MI_AI), "AI status write acks");

    n64_back(N)->rdp_n = 0;
    n64_wbe32(N->rdram + 0x6000, 0x37000000u); n64_wbe32(N->rdram + 0x6004, 0xFF0000FFu);
    n64_wbe32(N->rdram + 0x6008, 0x36000000u | (100u << 12) | 100u); n64_wbe32(N->rdram + 0x600C, 0u);
    n64_wbe32(N->rdram + 0x6010, 0x29000000u); n64_wbe32(N->rdram + 0x6014, 0u);
    n64_phys_write32(N, 0x04100000u, 0x6000u, ~0u);
    n64_phys_write32(N, 0x04100004u, 0x6018u, ~0u);
    HS_CHECK(&TS, n64_back(N)->rdp_n == 4u && n64_back(N)->rdp[1] == 0xFF0000FFu, "DP_START/DP_END command pass-through (%u words)", n64_back(N)->rdp_n);
    HS_CHECK(&TS, (N->mi_intr & MI_DP) && n64_phys_read32(N, 0x04100008u) == 0x6018u, "Full Sync -> DP interrupt, DP_CURRENT");
    n64_phys_write32(N, 0x04300000u, 0x800u, ~0u);
    HS_CHECK(&TS, !(N->mi_intr & MI_DP), "MI_MODE clear-DP");
    n64_phys_write32(N, 0x0430000Cu, 0xAAAu, ~0u);
    HS_CHECK(&TS, N->mi_mask == 0x3Fu, "MI mask set all");
    n64_phys_write32(N, 0x0430000Cu, 0x55u, ~0u);
    HS_CHECK(&TS, N->mi_mask == 0x30u, "MI mask clear SP/SI/AI/VI (%02x)", N->mi_mask);
    HS_CHECK(&TS, n64_phys_read32(N, 0x04300004u) == 0x02020102u, "MI version");
    n64_phys_write32(N, 0x04400004u, 0x00300000u, ~0u);
    HS_CHECK(&TS, n64_phys_read32(N, 0x04400004u) == 0x00300000u, "VI origin register");
    N->frame_instr = N64_INSTR_PER_FRAME / 2u;
    HS_CHECK(&TS, n64_phys_read32(N, 0x04400010u) == 262u, "VI_CURRENT tracks the frame position (%u)", n64_phys_read32(N, 0x04400010u));
    for (f = 0; f < 3u; f++) { free(TROM[f]); TROM[f] = NULL; }
}

/* ============================================================ RSP math */
static void test_rsp_math(void)
{
    static const uint64_t sq[] = { 0, 1, 2, 3, 99, 100, 65535, 4294967296ull, 0x3FFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull, 123456789012345ull };
    uint32_t i;
    int32_t a[16], b[16], m[16], o[4];
    uint8_t raw[64];
    printf("[rsp] fixed-point maths\n");
    for (i = 0; i < sizeof(sq) / sizeof(sq[0]); i++) {
        uint64_t r = n64_isqrt64(sq[i]);
        HS_CHECK(&TS, r * r <= sq[i] && (r == 0xFFFFFFFFu || (r + 1u) * (r + 1u) > sq[i]), "isqrt64(%llu)=%llu", (unsigned long long)sq[i], (unsigned long long)r);
    }
    memset(raw, 0, sizeof(raw));
    for (i = 0; i < 4u; i++) n64_wbe16(raw + (i * 5u) * 2u, 1);
    n64_wbe16(raw + 2u * 12u, 0xFFFF); n64_wbe16(raw + 32u + 2u * 12u, 0x8000);
    n64_rsp_load_mtx(raw, m);
    HS_CHECK(&TS, m[0] == 65536 && m[5] == 65536 && m[15] == 65536 && m[1] == 0, "Mtx load integer/fraction halves");
    HS_CHECK(&TS, m[12] == -32768, "Mtx negative fixed-point (%d)", m[12]);
    memset(a, 0, sizeof(a)); memset(b, 0, sizeof(b));
    a[0] = a[5] = a[10] = 2 * 65536; a[15] = 65536;
    b[0] = b[5] = b[10] = b[15] = 65536; b[12] = 10 * 65536; b[13] = 20 * 65536; b[14] = 30 * 65536;
    n64_rsp_matmul(a, b, m);
    n64_rsp_xform(m, 1, 2, 3, o);
    HS_CHECK(&TS, o[0] == 12 * 65536 && o[1] == 24 * 65536 && o[2] == 36 * 65536 && o[3] == 65536, "row-vector scale*translate");
    n64_rsp_matmul(b, a, m);
    n64_rsp_xform(m, 1, 2, 3, o);
    HS_CHECK(&TS, o[0] == 22 * 65536 && o[1] == 44 * 65536 && o[2] == 66 * 65536, "order matters: translate*scale");
    {
        n64_gfx_t *g = &N->gfx;
        int32_t id[16];
        uint32_t c;
        memset(g, 0, sizeof(*g));
        memset(id, 0, sizeof(id)); id[0] = id[5] = id[10] = id[15] = 65536;
        g->num_lights = 1;
        g->light_col[0][0] = 200; g->light_dir[0][2] = 127;
        g->light_col[1][0] = 30; g->light_col[1][1] = 30; g->light_col[1][2] = 30;
        c = n64_rsp_light(g, id, 0, 0, 127);
        HS_CHECK(&TS, c == 0xE61E1E00u, "directional + ambient light facing (%08x)", c);
        c = n64_rsp_light(g, id, 0, 0, -127);
        HS_CHECK(&TS, c == 0x1E1E1E00u, "back-facing normal = ambient only (%08x)", c);
        c = n64_rsp_light(g, id, 90, 0, 90);
        HS_CHECK(&TS, (c >> 24) >= 165u && (c >> 24) <= 175u, "45 degree normal ~ cos (%u)", c >> 24);
        id[0] = id[5] = id[10] = 4 * 65536;
        c = n64_rsp_light(g, id, 0, 0, 127);
        HS_CHECK(&TS, c == 0xE61E1E00u, "normal renormalised after scaled modelview (%08x)", c);
    }
    {
        static const int16_t book[2] = { 4096, -2048 };
        HS_CHECK(&TS, n64_adpcm_sample(100, 50, 3, book) == 153, "ADPCM order-2 predictor");
        HS_CHECK(&TS, n64_adpcm_sample(32767, -32768, 0, book) == 32767, "ADPCM saturates");
    }
}

/* ============================================================= RSP HLE */
static uint32_t dlp;
static void DLW(uint32_t w0, uint32_t w1) { n64_wbe32(N->rdram + dlp, w0); n64_wbe32(N->rdram + dlp + 4u, w1); dlp += 8u; }
static void write_mtx(uint32_t addr, const int32_t *m)
{
    uint32_t i;
    for (i = 0; i < 16u; i++) {
        n64_wbe16(N->rdram + addr + i * 2u, (uint16_t)((uint32_t)m[i] >> 16));
        n64_wbe16(N->rdram + addr + 32u + i * 2u, (uint16_t)((uint32_t)m[i] & 0xFFFFu));
    }
}
static void put_vtx(uint32_t addr, int16_t x, int16_t y, int16_t z, int16_t s, int16_t t, uint32_t rgba)
{
    n64_wbe16(N->rdram + addr, (uint16_t)x); n64_wbe16(N->rdram + addr + 2u, (uint16_t)y);
    n64_wbe16(N->rdram + addr + 4u, (uint16_t)z); n64_wbe16(N->rdram + addr + 6u, 0);
    n64_wbe16(N->rdram + addr + 8u, (uint16_t)s); n64_wbe16(N->rdram + addr + 10u, (uint16_t)t);
    n64_wbe32(N->rdram + addr + 12u, rgba);
}
static void back_reset(void)
{
    n64_snap_t *b = n64_back(N);
    b->rdp_n = 0; b->tex_n = 0; b->ntris = 0; b->overflow = 0; b->gfx_tasks = 0; b->audio_tasks = 0;
    b->native_w = 0; b->native_h = 0; b->ai_n = 0;
}
static void start_task(uint32_t type, uint32_t flags, uint32_t data, uint32_t size)
{
    n64_wbe32(N->rsp.dmem + 0xFC0u, type); n64_wbe32(N->rsp.dmem + 0xFC4u, flags);
    n64_wbe32(N->rsp.dmem + 0xFF0u, data); n64_wbe32(N->rsp.dmem + 0xFF4u, size);
    N->rsp.status = 1u;
    n64_phys_write32(N, 0x04040010u, 0x1u, ~0u);          /* clear halt -> task runs */
}
static int find_op(const n64_snap_t *b, uint32_t op, uint32_t w1, int match_w1)
{
    uint32_t p = 0;
    while (p + 1u < b->rdp_n) {
        uint32_t o = (b->rdp[p] >> 24) & 0x3Fu;
        if (o == op && (!match_w1 || b->rdp[p + 1u] == w1)) return (int)p;
        p += o == N64_OP_HS_TRI ? N64_TRI_WORDS : (o == N64_OP_HS_TIMG || o == 0x24u || o == 0x25u) ? 4u : 2u;
    }
    return -1;
}

static void test_rsp_hle(void)
{
    const uint32_t S1B = 0x102000u;
    int32_t m[16];
    n64_snap_t *b;
    uint32_t i;
    int p;
    printf("[rsp] HLE gfx (F3DEX + Fast3D encodings)\n");
    core_init(N, 0);
    memset(m, 0, sizeof(m));
    n64_wbe16(N->rdram + S1B + 0, 640); n64_wbe16(N->rdram + S1B + 2, 480); n64_wbe16(N->rdram + S1B + 4, 511);
    n64_wbe16(N->rdram + S1B + 8, 640); n64_wbe16(N->rdram + S1B + 10, 480); n64_wbe16(N->rdram + S1B + 12, 511);
    m[0] = m[5] = m[10] = 1024; m[15] = 65536; write_mtx(S1B + 0x40u, m);
    memset(m, 0, sizeof(m)); m[0] = m[5] = m[10] = m[15] = 65536; write_mtx(S1B + 0x80u, m);
    put_vtx(S1B + 0x400u, -32, -32, 0, 0, 0, 0xFF0000FFu);
    put_vtx(S1B + 0x410u, 32, -32, 0, 0, 0, 0x00FF00FFu);
    put_vtx(S1B + 0x420u, 0, 32, 0, 0, 0, 0x0000FFFFu);
    put_vtx(S1B + 0x430u, -32, -32, 0, 0, 0, 0xFFFFFFFFu);
    put_vtx(S1B + 0x440u, 32, -32, 0, 0, 0, 0xFFFFFFFFu);
    put_vtx(S1B + 0x450u, 0, 32, -300, 0, 0, 0xFFFFFFFFu);
    put_vtx(S1B + 0x460u, 100, 0, 0, 0, 0, 0xFFFFFFFFu);
    put_vtx(S1B + 0x470u, 120, 0, 0, 0, 0, 0xFFFFFFFFu);
    put_vtx(S1B + 0x480u, 110, 10, 0, 0, 0, 0xFFFFFFFFu);
    for (i = 0; i < 128u; i++) N->rdram[S1B + 0x300u + i] = (uint8_t)(i * 13u + 1u);
    dlp = S1B + 0x200u;
    DLW(0xFA000000u, 0x11223344u); DLW(0xB8000000u, 0);
    dlp = 0x101000u;
    DLW(0xBC000406u, S1B);
    DLW(0x03800010u, 0x01000000u);
    DLW(0x01030040u, 0x01000040u);
    DLW(0x01060040u, 0x01000080u);
    DLW(0xB7000000u, 0x2205u);
    DLW(0x04000000u | (9u << 10) | (9u * 16u - 1u), 0x01000400u);
    DLW(0xBF000000u, (0u << 16) | (2u << 8) | 4u);
    DLW(0xBF000000u, (0u << 16) | (4u << 8) | 2u);
    DLW(0xB1000000u | (6u << 16) | (8u << 8) | 10u, (12u << 16) | (14u << 8) | 16u);
    DLW(0x06000000u, 0x01000200u);
    DLW(0xFD10000Fu, 0x01000300u);
    DLW(0xF5100000u | (4u << 9), 0);
    DLW(0xF3000000u, 0x07000000u | (63u << 12));
    DLW(0xBA001402u, 0x00100000u);
    DLW(0xBD000000u, 0);
    DLW(0xE9000000u, 0);
    DLW(0xB8000000u, 0);
    DLW(0xFA000000u, 0xDEADDEADu);                     /* after ENDDL: must never run */
    back_reset();
    N->mi_intr = 0;
    start_task(1, 0, 0x101000u, 0);
    b = n64_back(N);
    HS_CHECK(&TS, b->gfx_tasks == 1u && N->rsp.hle_gfx == 1u, "gfx OSTask dispatched to HLE");
    HS_CHECK(&TS, b->ntris == 3u, "1 visible + back-face culled + near-clip split into 2 + trivial reject = 3 (%u)", b->ntris);
    p = find_op(b, N64_OP_HS_TRI, 0, 0);
    HS_CHECK(&TS, p >= 0, "HS_TRI emitted");
    if (p >= 0) {
        const uint32_t *t = &b->rdp[p];
        HS_CHECK(&TS, (int32_t)t[1] == 80 * 65536 && (int32_t)t[2] == 180 * 65536, "viewport mapping (%d,%d)", (int32_t)t[1] >> 16, (int32_t)t[2] >> 16);
        HS_CHECK(&TS, t[3] == 32704u, "depth mapping z=%u", t[3]);
        HS_CHECK(&TS, t[7] == 0xFF0000FFu && t[14] == 0x00FF00FFu, "smooth shade colours");
        HS_CHECK(&TS, (t[0] & 0xFFu) == (N64_TRI_Z | N64_TRI_SHADE), "tri flags Z|SHADE");
    }
    HS_CHECK(&TS, find_op(b, 0x3A, 0x11223344u, 1) >= 0, "G_DL through segment table + return");
    HS_CHECK(&TS, find_op(b, 0x3A, 0xDEADDEADu, 1) < 0, "ENDDL terminates the task");
    p = find_op(b, N64_OP_HS_TIMG, 0, 0);
    HS_CHECK(&TS, p >= 0 && b->rdp[p + 3] == 128u && b->tex_n == 128u, "LoadBlock resolved into snapshot tex blob (%u bytes)", b->tex_n);
    HS_CHECK(&TS, memcmp(b->tex, N->rdram + S1B + 0x300u, 128) == 0, "tex blob bytes == RDRAM texture");
    HS_CHECK(&TS, p >= 0 && ((b->rdp[p + 4] >> 24) & 0x3Fu) == 0x33u, "LoadBlock follows its HS_TIMG");
    p = find_op(b, 0x2F, 0, 0);
    HS_CHECK(&TS, p >= 0 && (b->rdp[p] & 0x00FFFFFFu) == 0x00100000u, "SETOTHERMODE_H -> RDP other modes (2-cycle)");
    HS_CHECK(&TS, N->gfx.mv_top == 0u, "G_POPMTX");
    HS_CHECK(&TS, (N->mi_intr & MI_DP) != 0, "FullSync raises DP interrupt");
    HS_CHECK(&TS, (N->rsp.status & 0x203u) == 0x203u, "RSP halted+broke+SIG2 after task");

    dlp = 0x101800u;
    DLW(0xBC000406u, S1B);
    DLW(0x03800010u, 0x01000000u);
    DLW(0x01030040u, 0x01000040u);
    DLW(0x01020040u, 0x01000080u);
    DLW(0xB7000000u, 0x0205u);
    DLW(0x04000000u | (2u << 20) | (0u << 16) | 0x30u, 0x01000400u);
    DLW(0xBF000000u, (0u << 16) | (10u << 8) | 20u);
    DLW(0xB8000000u, 0);
    back_reset();
    start_task(1, 0x8000u, 0x101800u, 0);
    HS_CHECK(&TS, n64_back(N)->ntris == 1u, "Fast3D vertex/index encoding (%u tris)", n64_back(N)->ntris);

    printf("[rsp] scalar-unit LLE\n");
    {
        static const uint32_t prog[] = { 0x2401000Au /* addiu 1,0,10 */, 0x24020000u, 0x00411021u /* addu 2,2,1 */,
                                         0x2421FFFFu, 0x1420FFFDu /* bne 1,0,-3 */, 0x00000000u, 0xAC020100u /* sw 2,0x100(0) */, 0x0000000Du };
        for (i = 0; i < 8u; i++) n64_wbe32(N->rsp.imem + i * 4u, prog[i]);
        n64_phys_write32(N, 0x04080000u, 0u, ~0u);
        N->rsp.lle_tasks = 0;
        start_task(0, 0, 0, 0);
        HS_CHECK(&TS, n64_be32(N->rsp.dmem + 0x100) == 55u, "LLE sum 1..10 = %u", n64_be32(N->rsp.dmem + 0x100));
        HS_CHECK(&TS, N->rsp.lle_tasks == 1u && (N->rsp.status & 3u) == 3u, "LLE BREAK halts RSP");
    }

    printf("[rsp] HLE audio (HS-ABI)\n");
    {
        const uint32_t A = 0x8000u;
        uint32_t k = 0, ok = 1;
        n64_wbe16(N->rdram + A, 2048); n64_wbe16(N->rdram + A + 2u, 0);
        N->rdram[A + 0x200u] = 0x10;
        for (i = 1; i < 9u; i++) N->rdram[A + 0x200u + i] = 0x11;
        dlp = 0x9000u;
        DLW(0x04000009u, A + 0x200u);                  /* LOADBUFF 9 bytes -> DMEM 0 */
        DLW(0x0B000000u, A);                            /* LOADADPCM */
        DLW(0x08000100u, 16u);                          /* SETBUFF in 0 out 0x100 n 16 */
        DLW(0x01010000u, A + 0x100u);                   /* ADPCM (init) */
        DLW(0x09000000u, 0x40007FFFu);                  /* SETVOL */
        DLW(0x08100000u, 16u);                          /* SETBUFF in 0x100 out 0 */
        DLW(0x03000000u, (0x200u << 16) | 0x300u);     /* PAN */
        DLW(0x08100400u, 8u);
        DLW(0x05018000u, A + 0x300u);                   /* RESAMPLE pitch 2.0 (init) */
        DLW(0x08000000u, 4u);
        DLW(0x0D000500u, (0x200u << 16) | 0x300u);     /* INTERLEAVE */
        DLW(0x06500010u, A + 0x400u);                   /* SAVEBUFF 16 bytes */
        DLW(0x08000000u, 8u);
        DLW(0x0C004000u, (0x400u << 16) | 0x200u);     /* MIXER 0.5 * resampled into L */
        DLW(0x02000600u, 32u);                          /* CLEARBUFF */
        back_reset();
        start_task(2, 0, 0x9000u, (dlp - 0x9000u));
        for (i = 0; i < 16u; i++) if ((int16_t)n64_be16(N->rsp.dmem + 0x100 + i * 2u) != (int16_t)(2 * (i + 1u))) ok = 0;
        HS_CHECK(&TS, ok, "ADPCM decode (integrating predictor)");
        HS_CHECK(&TS, n64_back(N)->audio_tasks == 1u && N->rsp.hle_audio == 1u, "audio OSTask dispatched");
        for (i = 0; i < 8u; i++) if ((int16_t)n64_be16(N->rsp.dmem + 0x400 + i * 2u) != (int16_t)(4 * i)) { ok = 0; k = i; }
        HS_CHECK(&TS, ok, "RESAMPLE 2.0 picks every 2nd sample (bad idx %u)", k);
        HS_CHECK(&TS, n64_be16(N->rdram + A + 0x400u) == 1u && n64_be16(N->rdram + A + 0x402u) == 1u &&
                      n64_be16(N->rdram + A + 0x404u) == 2u && n64_be16(N->rdram + A + 0x406u) == 3u, "PAN + INTERLEAVE + SAVEBUFF");
        HS_CHECK(&TS, n64_be16(N->rsp.dmem + 0x202) == 2u + 2u, "MIXER accumulate (%d)", (int16_t)n64_be16(N->rsp.dmem + 0x202));
        HS_CHECK(&TS, n64_be16(N->rdram + A + 0x100u) == 32u, "ADPCM state saved to RDRAM");
        HS_CHECK(&TS, n64_be16(N->rsp.dmem + 0x600) == 0u && N->rsp.dmem[0x61F] == 0u, "CLEARBUFF");
    }
}

/* ================================================================= RDP */
static uint32_t *FB, *FB2;
static n64_snap_t *RB;

static void rdp_begin(void)
{
    back_reset();
    RB = n64_back(N);
    RB->native_w = 320u; RB->native_h = 240u;
}
static void RP(uint32_t w0, uint32_t w1) { RB->rdp[RB->rdp_n++] = w0; RB->rdp[RB->rdp_n++] = w1; }
static void RP4(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) { RP(w0, w1); RP(w2, w3); }
static void rdp_show(hs_video_mode_t mode, uint32_t *fb)
{
    N->front ^= 1u;
    n64_render(N, fb, HS_FB_W, mode);
}
static void rerender(hs_video_mode_t mode, uint32_t *fb) { n64_render(N, fb, HS_FB_W, mode); }
static uint32_t PX(const uint32_t *fb, uint32_t x, uint32_t y) { return fb[(size_t)(2u * y) * HS_FB_W + 2u * x] & 0xFFFFFFu; }
static void cc(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t aa, uint32_t ab, uint32_t ac, uint32_t ad)
{
    uint32_t hi = (a << 20) | (c << 15) | (aa << 12) | (ac << 9) | (a << 5) | c;
    uint32_t lo = (b << 28) | (b << 24) | (aa << 21) | (ac << 18) | (d << 15) | (ab << 12) | (ad << 9) | (d << 6) | (ab << 3) | ad;
    RP(0xFC000000u | hi, lo);
}
static void om(uint32_t hi, uint32_t lo) { RP(0xEF000000u | hi, lo); }
static uint32_t R102(double v) { return (uint32_t)(int32_t)(v * 4.0 + 0.5); }
static void rect36(double x0, double y0, double x1, double y1)
{ RP(0x36000000u | (R102(x1) << 12) | R102(y1), (R102(x0) << 12) | R102(y0)); }
static void scissor(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{ RP(0x2D000000u | ((x0 << 2) << 12) | (y0 << 2), ((x1 << 2) << 12) | (y1 << 2)); }
static void tri(uint32_t flags, double x0, double y0, uint32_t z0, uint32_t c0, double x1, double y1, uint32_t z1, uint32_t c1,
                double x2, double y2, uint32_t z2, uint32_t c2)
{
    const double X[3] = { x0, x1, x2 }, Y[3] = { y0, y1, y2 };
    const uint32_t Z[3] = { z0, z1, z2 }, C[3] = { c0, c1, c2 };
    uint32_t i;
    RB->rdp[RB->rdp_n++] = (N64_OP_HS_TRI << 24) | flags;
    for (i = 0; i < 3u; i++) {
        RB->rdp[RB->rdp_n++] = (uint32_t)(int32_t)(X[i] * 65536.0);
        RB->rdp[RB->rdp_n++] = (uint32_t)(int32_t)(Y[i] * 65536.0);
        RB->rdp[RB->rdp_n++] = Z[i];
        RB->rdp[RB->rdp_n++] = 65536u;
        RB->rdp[RB->rdp_n++] = 0; RB->rdp[RB->rdp_n++] = 0;
        RB->rdp[RB->rdp_n++] = C[i];
    }
}
static void rdp_targets(void)
{
    RP(0x3F180000u | 319u, 0x300000u);
    RP(0x3E000000u, 0x380000u);
    scissor(0, 0, 320, 240);
}
static void timg(uint32_t fmt, uint32_t siz, uint32_t w, const uint8_t *data, uint32_t len)
{
    uint32_t off = RB->tex_n;
    memcpy(RB->tex + off, data, len);
    RB->tex_n = (off + len + 7u) & ~7u;
    RP4((N64_OP_HS_TIMG << 24) | (fmt << 21) | (siz << 19) | (w - 1u), off, 0, len);
}
static void settile(uint32_t tile, uint32_t fmt, uint32_t siz, uint32_t line, uint32_t tmem, uint32_t pal)
{ RP(0xF5000000u | (fmt << 21) | (siz << 19) | (line << 9) | tmem, (tile << 24) | (pal << 20)); }
static void loadblock(uint32_t tile, uint32_t texels) { RP(0xF3000000u, (tile << 24) | ((texels - 1u) << 12)); }
static void texrect_row(uint32_t tile, uint32_t x0, uint32_t x1, uint32_t y)
{ RP4(0xE4000000u | ((x1 << 2) << 12) | (y << 2), (tile << 24) | ((x0 << 2) << 12) | (y << 2), 0, (0x1000u << 16) | 0x400u); }

static void test_rdp(void)
{
    uint32_t c, x, y, ok, i;
    printf("[rdp] fill / scissor / combiner / blender / Z / Gouraud / TMEM formats\n");
    core_init(N, 0);
    rdp_begin();
    rdp_targets();
    om(0x300000u, 0);
    RP(0x37000000u, 0xFF0000FFu); rect36(0, 0, 319, 239);
    RP(0x37000000u, 0x00FF00FFu); rect36(10, 10, 19, 19);
    scissor(40, 40, 60, 60);
    RP(0x37000000u, 0x0000FFFFu); rect36(0, 0, 319, 239);
    scissor(0, 0, 320, 240);
    /* 1-cycle combiner: (prim - env) * prim_alpha + env */
    om(0, 0);
    RP(0xFA000000u, 0xFF000080u); RP(0xFB000000u, 0x0000FF00u);
    cc(3, 5, 10, 5, 7, 7, 7, 6);
    rect36(100, 10, 110, 20);
    /* blender: in*a + mem*(1-a) */
    RP(0xFA000000u, 0x0000FF80u);
    cc(0, 0, 31, 3, 7, 7, 7, 3);
    om(0, 0x00404000u);
    rect36(120, 10, 130, 20);
    /* Z buffer */
    cc(0, 0, 31, 4, 7, 7, 7, 4);
    om(0, 0x30u);
    tri(N64_TRI_Z | N64_TRI_SHADE, 200, 100, 30000, 0xFF0000FFu, 300, 100, 30000, 0xFF0000FFu, 200, 200, 30000, 0xFF0000FFu);
    tri(N64_TRI_Z | N64_TRI_SHADE, 200, 100, 40000, 0x00FF00FFu, 300, 100, 40000, 0x00FF00FFu, 200, 200, 40000, 0x00FF00FFu);
    tri(N64_TRI_Z | N64_TRI_SHADE, 240, 150, 30000, 0xFF00FFFFu, 300, 150, 30000, 0xFF00FFFFu, 240, 210, 30000, 0xFF00FFFFu);
    tri(N64_TRI_Z | N64_TRI_SHADE, 240, 150, 20000, 0x0000FFFFu, 300, 150, 20000, 0x0000FFFFu, 240, 210, 20000, 0x0000FFFFu);
    /* Z-target fill (colour image == Z image) then a farther triangle fails */
    RP(0x3F180000u | 319u, 0x380000u);
    om(0x300000u, 0); RP(0x37000000u, 0x00000000u); rect36(0, 200, 40, 239);
    RP(0x3F180000u | 319u, 0x300000u);
    om(0, 0x30u);
    tri(N64_TRI_Z | N64_TRI_SHADE, 0, 200, 100, 0x00FF00FFu, 80, 200, 100, 0x00FF00FFu, 0, 260, 100, 0x00FF00FFu);
    /* Gouraud */
    om(0, 0);
    tri(N64_TRI_SHADE, 100, 130, 0, 0xFF0000FFu, 180, 130, 0, 0x00FF00FFu, 100, 210, 0, 0x0000FFFFu);
    /* TMEM formats through copy-mode texture rectangles */
    {
        static const uint8_t rgba16[16] = { 0xF8, 0x01, 0x07, 0xC1 };
        static const uint8_t rgba32[32] = { 0x12, 0x34, 0x56, 0xFF, 0xAB, 0xCD, 0xEF, 0xFF };
        static const uint8_t ia8[8] = { 0xF8, 0x30 }, ia4[4] = { 0xE4 }, i4[4] = { 0x8F }, i8[8] = { 0x5A, 0x00 };
        static const uint8_t ia16[16] = { 0x7F, 0xFF, 0x20, 0xFF }, ci8[8] = { 0x01, 0x02 }, ci4[4] = { 0x21 };
        static const uint8_t tlut[6] = { 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF };
        om(0x200000u, 0);
        timg(0, 2, 3, tlut, 6); settile(1, 0, 2, 1, 0x100, 0); loadblock(1, 3);
        timg(0, 2, 8, rgba16, 16); settile(0, 0, 2, 2, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 225);
        timg(0, 3, 8, rgba32, 32); settile(0, 0, 3, 4, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 226);
        timg(3, 1, 8, ia8, 8); settile(0, 3, 1, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 227);
        timg(3, 0, 8, ia4, 4); settile(0, 3, 0, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 228);
        timg(4, 0, 8, i4, 4); settile(0, 4, 0, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 229);
        timg(4, 1, 8, i8, 8); settile(0, 4, 1, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 230);
        timg(3, 2, 8, ia16, 16); settile(0, 3, 2, 2, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 231);
        timg(2, 1, 8, ci8, 8); settile(0, 2, 1, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 232);
        timg(2, 0, 8, ci4, 4); settile(0, 2, 0, 1, 0, 0); loadblock(0, 8); texrect_row(0, 250, 257, 233);
    }
    rdp_show(HS_SSAA_1X, FB);
    HS_CHECK(&TS, PX(FB, 15, 15) == 0x00FF00u && PX(FB, 19, 19) == 0x00FF00u, "fill rect (inclusive lower-right in fill mode)");
    HS_CHECK(&TS, PX(FB, 9, 9) == 0xFF0000u && PX(FB, 20, 20) == 0xFF0000u, "fill rect bounds");
    HS_CHECK(&TS, PX(FB, 50, 50) == 0x0000FFu && PX(FB, 59, 59) == 0x0000FFu, "scissor interior");
    HS_CHECK(&TS, PX(FB, 39, 39) == 0xFF0000u && PX(FB, 60, 60) == 0xFF0000u && PX(FB, 300, 5) == 0xFF0000u, "scissor clips (exclusive)");
    c = PX(FB, 105, 15);
    HS_CHECK(&TS, c == 0x800080u, "combiner lerp (prim-env)*prim_a+env = %06x", c);
    HS_CHECK(&TS, PX(FB, 110, 15) == 0xFF0000u, "1-cycle fill rect exclusive lower-right");
    c = PX(FB, 125, 15);
    HS_CHECK(&TS, c == 0x7F0080u, "blender 50%% over memory = %06x", c);
    HS_CHECK(&TS, PX(FB, 210, 110) == 0xFF0000u, "Z: farther triangle rejected");
    HS_CHECK(&TS, PX(FB, 250, 160) == 0x0000FFu, "Z: nearer triangle wins");
    HS_CHECK(&TS, PX(FB, 20, 220) == 0xFF0000u, "Z-target fill rect writes depth (tri behind it rejected)");
    HS_CHECK(&TS, PX(FB, 60, 205) == 0x00FF00u, "triangle outside the Z fill passes");
    c = PX(FB, 126, 156);
    HS_CHECK(&TS, ((c >> 16) & 255u) > 60u && ((c >> 16) & 255u) < 110u && ((c >> 8) & 255u) > 60u && ((c >> 8) & 255u) < 110u &&
                  (c & 255u) > 60u && (c & 255u) < 110u, "Gouraud centroid blends all three (%06x)", c);
    {
        static const uint32_t ex[9][2] = { { 0xFF0000u, 0x00FF00u }, { 0x123456u, 0xABCDEFu }, { 0xFFFFFFu, 0x333333u },
                                           { 0xFFFFFFu, 0x494949u }, { 0x888888u, 0xFFFFFFu }, { 0x5A5A5Au, 0x000000u },
                                           { 0x7F7F7Fu, 0x202020u }, { 0x0000FFu, 0xFFFFFFu }, { 0xFFFFFFu, 0x0000FFu } };
        static const char *nm[9] = { "RGBA16", "RGBA32", "IA8", "IA4", "I4", "I8", "IA16", "CI8+TLUT", "CI4+TLUT" };
        for (i = 0; i < 9u; i++) {
            uint32_t a = PX(FB, 250, 225 + i), bb = PX(FB, 251, 225 + i);
            HS_CHECK(&TS, a == ex[i][0] && bb == ex[i][1], "texel format %s: %06x %06x", nm[i], a, bb);
        }
        HS_CHECK(&TS, PX(FB, 257, 225) == 0u && PX(FB, 258, 225) == 0xFF0000u && PX(FB, 249, 225) == 0xFF0000u, "copy-mode texrect inclusive bounds");
    }

    printf("[rdp] SSAA coverage + watertight shared edges\n");
    rdp_begin();
    rdp_targets();
    cc(0, 0, 31, 4, 7, 7, 7, 4);
    om(0, 0);
    tri(N64_TRI_SHADE, 20.3, 20.7, 0, 0xFFFFFFFFu, 300.1, 60.2, 0, 0xFFFFFFFFu, 40.5, 220.9, 0, 0xFFFFFFFFu);
    rdp_show(HS_SSAA_1X, FB);
    {
        uint32_t lv[4] = { 0, 0, 0, 0 }, m, bad1 = 0, fxd = 0, fxsame = 1;
        static uint8_t seen[256];
        static const hs_video_mode_t md[3] = { HS_SSAA_2X, HS_SSAA_4X, HS_SSAA_8X };
        for (i = 0; i < HS_FB_W * HS_FB_H; i++) { c = FB[i] & 0xFFu; if (c != 0u && c != 255u) bad1++; }
        HS_CHECK(&TS, bad1 == 0u, "1x: hard edges only (%u grey px)", bad1);
        for (m = 0; m < 3u; m++) {
            memset(seen, 0, sizeof(seen));
            rerender(md[m], FB2);
            for (i = 0; i < HS_FB_W * HS_FB_H; i++) seen[FB2[i] & 0xFFu] = 1;
            for (i = 1; i < 255u; i++) lv[m + 1u] += seen[i];
        }
        printf("INFO ssaa_grey_levels 2x=%u 4x=%u 8x=%u\n", lv[1], lv[2], lv[3]);
        HS_CHECK(&TS, lv[1] >= 1u && lv[1] <= 3u, "2x: 2x2 coverage levels (%u)", lv[1]);
        HS_CHECK(&TS, lv[2] > lv[1] && lv[2] <= 15u, "4x: more coverage levels (%u)", lv[2]);
        HS_CHECK(&TS, lv[3] > lv[2] && lv[3] <= 63u, "8x: most coverage levels (%u)", lv[3]);
        rerender(HS_AA_FXAA, FB2);
        for (i = 0; i < HS_FB_W * HS_FB_H; i++) if ((FB2[i] & 0xFFFFFFu) != (FB[i] & 0xFFFFFFu)) fxd++;
        for (y = 200; y < 240; y++) for (x = 200; x < 240; x++) if ((FB2[y * HS_FB_W + x] & 0xFFFFFFu) != (FB[y * HS_FB_W + x] & 0xFFFFFFu)) fxsame = 0;
        HS_CHECK(&TS, fxd > 100u && fxd < 20000u, "FXAA touches edge pixels only (%u)", fxd);
        HS_CHECK(&TS, fxsame, "FXAA leaves flat interior untouched");
    }
    rdp_begin();
    rdp_targets();
    cc(0, 0, 31, 4, 7, 7, 7, 4);
    om(0, 0x00444000u);
    tri(N64_TRI_SHADE, 30.25, 150.5, 0, 0xFF000040u, 90.75, 150.5, 0, 0xFF000040u, 90.75, 200.25, 0, 0xFF000040u);
    tri(N64_TRI_SHADE, 30.25, 150.5, 0, 0xFF000040u, 90.75, 200.25, 0, 0xFF000040u, 30.25, 200.25, 0, 0xFF000040u);
    tri(N64_TRI_SHADE, 200.1, 20.2, 0, 0xFF000040u, 260.7, 90.3, 0, 0xFF000040u, 150.4, 70.9, 0, 0xFF000040u);
    tri(N64_TRI_SHADE, 200.1, 20.2, 0, 0xFF000040u, 230.3, 10.6, 0, 0xFF000040u, 260.7, 90.3, 0, 0xFF000040u);
    rdp_show(HS_SSAA_8X, FB);
    ok = 1;
    for (y = 304; y < 398; y++) for (x = 64; x < 179; x++) if ((FB[y * HS_FB_W + x] & 0xFFFFFFu) != 0x330000u) { ok = 0; c = FB[y * HS_FB_W + x]; }
    HS_CHECK(&TS, ok, "8x: shared diagonal has no gaps or double hits (51 everywhere)");
    {
        uint32_t over = 0;
        for (i = 0; i < HS_FB_W * HS_FB_H; i++) if (((FB[i] >> 16) & 255u) > 51u) over++;
        HS_CHECK(&TS, over == 0u, "8x: no sample blended twice anywhere (%u px)", over);
        rerender(HS_SSAA_1X, FB2);
        over = 0;
        for (i = 0; i < HS_FB_W * HS_FB_H; i++) if (((FB2[i] >> 16) & 255u) > 51u) over++;
        HS_CHECK(&TS, over == 0u, "1x: no pixel blended twice (%u px)", over);
    }
}

/* ===================================================== helpers (host) */
static uint64_t fb_hash(const uint32_t *fb)
{
    uint64_t h = HS_FNV64_OFFSET;
    uint32_t i;
    for (i = 0; i < HS_FB_W * HS_FB_H; i++) {
        uint8_t b[4];
        uint32_t v = fb[i] & 0xFFFFFFu;
        b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = 0;
        h = hs_fnv1a64(h, b, 4);
    }
    return h;
}
static uint64_t pcm_hash(uint64_t h, const int16_t *p, uint32_t frames)
{
    uint32_t i;
    for (i = 0; i < frames * 2u; i++) { uint8_t b[2]; b[0] = (uint8_t)p[i]; b[1] = (uint8_t)((uint16_t)p[i] >> 8); h = hs_fnv1a64(h, b, 2); }
    return h;
}
static int write_wav(const char *path, const int16_t *pcm, uint32_t frames)
{
    FILE *f = fopen(path, "wb");
    uint8_t h[44];
    uint32_t data = frames * 4u, i, br = HS_AUDIO_RATE * 4u;
    if (!f) return -1;
    memcpy(h, "RIFF", 4); h[4] = (uint8_t)(data + 36u); h[5] = (uint8_t)((data + 36u) >> 8); h[6] = (uint8_t)((data + 36u) >> 16); h[7] = (uint8_t)((data + 36u) >> 24);
    memcpy(h + 8, "WAVEfmt ", 8);
    h[16] = 16; h[17] = h[18] = h[19] = 0; h[20] = 1; h[21] = 0; h[22] = 2; h[23] = 0;
    h[24] = (uint8_t)HS_AUDIO_RATE; h[25] = (uint8_t)(HS_AUDIO_RATE >> 8); h[26] = (uint8_t)(HS_AUDIO_RATE >> 16); h[27] = 0;
    h[28] = (uint8_t)br; h[29] = (uint8_t)(br >> 8); h[30] = (uint8_t)(br >> 16); h[31] = (uint8_t)(br >> 24);
    h[32] = 4; h[33] = 0; h[34] = 16; h[35] = 0;
    memcpy(h + 36, "data", 4); h[40] = (uint8_t)data; h[41] = (uint8_t)(data >> 8); h[42] = (uint8_t)(data >> 16); h[43] = (uint8_t)(data >> 24);
    fwrite(h, 1, 44, f);
    for (i = 0; i < frames * 2u; i++) { uint8_t b2[2]; b2[0] = (uint8_t)pcm[i]; b2[1] = (uint8_t)((uint16_t)pcm[i] >> 8); fwrite(b2, 1, 2, f); }
    fclose(f);
    return 0;
}
/* RMS x1000 of per-channel differences (integer only), optionally over an edge mask */
static uint32_t rms_x1000(const uint32_t *a, const uint32_t *ref, const uint8_t *mask)
{
    uint64_t se = 0, n = 0;
    uint32_t i, k;
    for (i = 0; i < HS_FB_W * HS_FB_H; i++) {
        if (mask && !mask[i]) continue;
        for (k = 0; k < 24u; k += 8u) {
            int32_t d = (int32_t)((a[i] >> k) & 0xFFu) - (int32_t)((ref[i] >> k) & 0xFFu);
            se += (uint64_t)((int64_t)d * d);
        }
        n += 3u;
    }
    if (!n) return 0;
    return n64_isqrt64(se * 1000000u / n);
}

/* ========================================================== isolation */
static void test_isolation(void)
{
    hs_n64_t *A = N, *B;
    int16_t pa[2048 * 2], pb[2048 * 2];
    uint32_t f, mism = 0, render_mut = 0, step_mut = 0, na, nb;
    const uint32_t frames = 20u;
    printf("[isolation] zero-mutex snapshot isolation + pipelined equivalence\n");
    if (!ROM) { HS_CHECK(&TS, 0, "demo ROM missing"); return; }
    B = (hs_n64_t *)aligned_alloc(64, (sizeof(hs_n64_t) + 63u) & ~(size_t)63u);
    if (!B) { HS_CHECK(&TS, 0, "alloc"); return; }
    core_init(A, 0); core_init(B, 0);
    hs_n64_core_ops.load_rom(A, ROM, ROMLEN); hs_n64_core_ops.load_rom(B, ROM, ROMLEN);
    hs_n64_core_ops.step_cpu_phase(B, 0, 0);
    hs_n64_core_ops.commit_phase(B);
    for (f = 0; f < frames; f++) {
        uint64_t v0, v1, a0, c0, c1, ha, hb;
        /* A: sequential  step -> commit -> render -> synth */
        hs_n64_core_ops.step_cpu_phase(A, 0, 0);
        hs_n64_core_ops.commit_phase(A);
        c0 = n64_cpu_state_hash(A);
        hs_n64_core_ops.render_video_phase(A, FB, HS_FB_W, HS_SSAA_1X);
        na = hs_n64_core_ops.synth_audio_phase(A, pa, 2048);
        c1 = n64_cpu_state_hash(A);
        if (c0 != c1) render_mut++;
        /* B: overlapped  step(f+1) into BACK while Phase 1/2 read FRONT(f) */
        hs_n64_core_ops.hash_state(B, &v0, &a0);
        hs_n64_core_ops.step_cpu_phase(B, 0, 0);
        hs_n64_core_ops.hash_state(B, &v1, NULL);
        if (v0 != v1) step_mut++;
        hs_n64_core_ops.render_video_phase(B, FB2, HS_FB_W, HS_SSAA_1X);
        nb = hs_n64_core_ops.synth_audio_phase(B, pb, 2048);
        hs_n64_core_ops.commit_phase(B);
        ha = fb_hash(FB); hb = fb_hash(FB2);
        if (ha != hb || na != nb || pcm_hash(HS_FNV64_OFFSET, pa, na) != pcm_hash(HS_FNV64_OFFSET, pb, nb)) mism++;
    }
    HS_CHECK(&TS, render_mut == 0u, "Phase 1/2 never mutate CPU/RDRAM/DMEM state (%u frames)", render_mut);
    HS_CHECK(&TS, step_mut == 0u, "Phase 0 never mutates the FRONT snapshot (%u frames)", step_mut);
    HS_CHECK(&TS, mism == 0u, "pipelined schedule == sequential schedule, video+audio, %u frames (%u mismatches)", frames, mism);
    free(B);
}

/* ============================================================ demo ROM */
typedef struct bk_count { uint32_t tris, fills, texrects, begins, ends, textured; } bk_count_t;
static void bk_begin(void *u, uint32_t w, uint32_t h, uint32_t ss) { (void)w; (void)h; (void)ss; ((bk_count_t *)u)->begins++; }
static void bk_tri(void *u, const n64_bk_vertex_t v[3], const n64_bk_state_t *st)
{ (void)v; ((bk_count_t *)u)->tris++; if (st->flags & N64_TRI_TEX) ((bk_count_t *)u)->textured++; }
static void bk_fill(void *u, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int z, const n64_bk_state_t *st)
{ (void)x0; (void)y0; (void)x1; (void)y1; (void)z; (void)st; ((bk_count_t *)u)->fills++; }
static void bk_trect(void *u, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t s, int32_t t, int32_t ds, int32_t dt, const n64_bk_state_t *st)
{ (void)x0; (void)y0; (void)x1; (void)y1; (void)s; (void)t; (void)ds; (void)dt; (void)st; ((bk_count_t *)u)->texrects++; }
static void bk_end(void *u) { ((bk_count_t *)u)->ends++; }

static void blit_zoom(uint32_t *dst, uint32_t dw, uint32_t dx, uint32_t dy, const uint32_t *src, uint32_t sx, uint32_t sy, uint32_t w, uint32_t h, uint32_t z)
{
    uint32_t x, y;
    for (y = 0; y < h * z; y++)
        for (x = 0; x < w * z; x++)
            dst[(size_t)(dy + y) * dw + dx + x] = src[(size_t)(sy + y / z) * HS_FB_W + sx + x / z];
}

static void test_demo_rom(void)
{
    static const hs_video_mode_t modes[5] = { HS_SSAA_1X, HS_AA_FXAA, HS_SSAA_2X, HS_SSAA_4X, HS_SSAA_8X };
    static const char *mname[5] = { "1x", "fxaa", "2x", "4x", "8x" };
    const uint32_t frames = QUICK ? 64u : 90u, reps = QUICK ? 1u : 3u;
    uint32_t *img[5], f, m, i, badf = 0, badpcm = 0, total_pcm = 0, peak = 0, max_tris = 0;
    int16_t *pcm_all, pcm[2048 * 2];
    uint64_t t_step = 0, t_render1 = 0, t_synth = 0, t_mode[5], vroll = HS_FNV64_OFFSET, vh, ah;
    char path[512];
    printf("[demo] hs_n64_demo.z64: real MIPS code, Fast3D DL, AI audio, %u frames\n", frames);
    if (!ROM) { HS_CHECK(&TS, 0, "demo ROM missing"); return; }
    pcm_all = (int16_t *)calloc((size_t)frames * 2048u * 2u, sizeof(int16_t));
    for (m = 0; m < 5u; m++) img[m] = (uint32_t *)calloc(HS_FB_W * HS_FB_H, 4);
    core_init(N, 0);
    HS_CHECK(&TS, hs_n64_core_ops.load_rom(N, ROM, ROMLEN) == 0 && N->rom_fmt == 0u, "load .z64");
    HS_CHECK(&TS, N->rom_crc_ok == 1u, "demo ROM CIC-6102 CRC %08X %08X verifies", N->rom_crc1, N->rom_crc2);
    for (f = 0; f < frames; f++) {
        uint64_t t0, t1, t2, t3;
        const n64_snap_t *s;
        uint32_t n;
        t0 = hs_host_now_ns();
        hs_n64_core_ops.step_cpu_phase(N, 0, 0);
        t1 = hs_host_now_ns();
        hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.render_video_phase(N, FB, HS_FB_W, HS_SSAA_1X);
        t2 = hs_host_now_ns();
        n = hs_n64_core_ops.synth_audio_phase(N, pcm, 2048);
        t3 = hs_host_now_ns();
        t_step += t1 - t0; t_render1 += t2 - t1; t_synth += t3 - t2;
        memcpy(pcm_all + (size_t)total_pcm * 2u, pcm, (size_t)n * 4u);
        total_pcm += n;
        s = n64_front(N);
        hs_n64_core_ops.hash_state(N, &vh, &ah);
        vroll ^= vh;
        vroll *= HS_FNV64_PRIME;
        if (s->ntris > max_tris) max_tris = s->ntris;
        if (f >= 4u && (s->gfx_tasks < 1u || s->audio_tasks < 1u || s->ntris < 64u || s->ai_n != 800u || s->overflow)) {
            if (badf < 3u) printf("  frame %u: gfx=%u audio=%u tris=%u ai_n=%u overflow=%u\n", f, s->gfx_tasks, s->audio_tasks, s->ntris, s->ai_n, s->overflow);
            badf++;
        }
        if (f >= 4u && (n < 795u || n > 805u)) badpcm++;
    }
    for (i = 0; i < total_pcm * 2u; i++) { int32_t a = pcm_all[i] < 0 ? -pcm_all[i] : pcm_all[i]; if ((uint32_t)a > peak) peak = (uint32_t)a; }
    HS_CHECK(&TS, n64_be32(N->rdram + 0x1FFF10) == 0x48534E36u && n64_be32(N->rdram + 0x1FFF0C) == 0x34210001u, "ROM 64-bit SD/LD self-check");
    HS_CHECK(&TS, n64_be32(N->rdram + 0x1FFF00) >= frames - 2u, "VI interrupt handler ran every frame (vicount=%u)", n64_be32(N->rdram + 0x1FFF00));
    HS_CHECK(&TS, n64_be32(N->rdram + 0x1FFF04) >= frames - 4u, "ROM main loop frame counter=%u", n64_be32(N->rdram + 0x1FFF04));
    HS_CHECK(&TS, badf == 0u, "every frame: gfx task + audio task + >=64 visible tris + 800 AI frames (%u bad)", badf);
    HS_CHECK(&TS, badpcm == 0u && peak > 2000u, "Phase 2 resampled ~800 frames/video frame at 48 kHz, peak %u", peak);
    HS_CHECK(&TS, N->cpu.halted_bad == 0u, "no reserved-instruction exceptions");
    HS_CHECK(&TS, n64_front(N)->ai_freq == 48009u, "AI DAC rate 48009 Hz");
    printf("INFO demo frames=%u max_tris=%u pcm_frames=%u instret=%llu pi_dma=%u si_dma=%u ai_dma=%u sp_dma=%u gfx_hle=%u audio_hle=%u\n",
           frames, max_tris, total_pcm, (unsigned long long)N->cpu.instret, N->pi_dma_count, N->si_dma_count, N->ai_dma_count,
           N->sp_dma_count, N->rsp.hle_gfx, N->rsp.hle_audio);
    snprintf(path, sizeof(path), "%s/n64_audio_48k.wav", OUT);
    HS_CHECK(&TS, write_wav(path, pcm_all, total_pcm) == 0, "write %s", path);
    printf("HASH demo_video_rolling %016llx\n", (unsigned long long)vroll);
    printf("HASH demo_video_last %016llx\n", (unsigned long long)vh);
    printf("HASH demo_audio_last %016llx\n", (unsigned long long)ah);
    printf("HASH demo_pcm48k_all %016llx\n", (unsigned long long)pcm_hash(HS_FNV64_OFFSET, pcm_all, total_pcm));
    printf("HASH demo_cpu_state %016llx\n", (unsigned long long)n64_cpu_state_hash(N));
    printf("TIME phase0_cpu_rsp_ms_per_frame %.3f\n", (double)t_step / frames / 1e6);
    printf("TIME phase1_render_1x_ms_per_frame %.3f\n", (double)t_render1 / frames / 1e6);
    printf("TIME phase2_audio_ms_per_frame %.3f\n", (double)t_synth / frames / 1e6);

    for (m = 0; m < 5u; m++) {
        uint64_t t0 = hs_host_now_ns(), h;
        uint32_t r, edge_black = 0;
        for (r = 0; r < reps; r++) hs_n64_core_ops.render_video_phase(N, img[m], HS_FB_W, modes[m]);
        t_mode[m] = (hs_host_now_ns() - t0) / reps;
        h = fb_hash(img[m]);
        printf("HASH demo_fb_%s %016llx\n", mname[m], (unsigned long long)h);
        printf("TIME phase1_render_%s_ms %.3f\n", mname[m], (double)t_mode[m] / 1e6);
        for (i = 0; i < HS_FB_W; i++) { edge_black += (img[m][i] & 0xFFFFFFu) == 0u; edge_black += (img[m][(HS_FB_H - 1u) * HS_FB_W + i] & 0xFFFFFFu) == 0u; }
        for (i = 0; i < HS_FB_H; i++) { edge_black += (img[m][i * HS_FB_W] & 0xFFFFFFu) == 0u; edge_black += (img[m][i * HS_FB_W + HS_FB_W - 1u] & 0xFFFFFFu) == 0u; }
        HS_CHECK(&TS, edge_black == 0u, "%s: full 640x480 edge-to-edge, no black border pixels (%u)", mname[m], edge_black);
        snprintf(path, sizeof(path), "%s/n64_3d_%s_640x480.png", OUT, mname[m]);
        HS_CHECK(&TS, hs_png_write_xrgb(path, img[m], HS_FB_W, HS_FB_H, HS_FB_W) == 0, "write %s", path);
    }
    {
        uint8_t *mask = (uint8_t *)calloc(HS_FB_W * HS_FB_H, 1);
        uint32_t x, y, nedge = 0, rall[4], redge[4];
        const uint32_t *ref = img[4];
        for (y = 0; y + 1u < HS_FB_H; y++)
            for (x = 0; x + 1u < HS_FB_W; x++) {
                uint32_t p = ref[y * HS_FB_W + x], q = ref[y * HS_FB_W + x + 1u], d = ref[(y + 1u) * HS_FB_W + x], k, g = 0;
                for (k = 0; k < 24u; k += 8u) {
                    int32_t a = (int32_t)((p >> k) & 255u), b = (int32_t)((q >> k) & 255u), c = (int32_t)((d >> k) & 255u);
                    g += (uint32_t)(a > b ? a - b : b - a) + (uint32_t)(a > c ? a - c : c - a);
                }
                if (g > 60u) { mask[y * HS_FB_W + x] = 1; nedge++; }
            }
        for (m = 0; m < 4u; m++) { rall[m] = rms_x1000(img[m], ref, NULL); redge[m] = rms_x1000(img[m], ref, mask); }
        for (m = 0; m < 4u; m++)
            printf("RMS vs_8x %s all=%u.%03u edge=%u.%03u (edge px=%u)\n", mname[m], rall[m] / 1000u, rall[m] % 1000u, redge[m] / 1000u, redge[m] % 1000u, nedge);
        HS_CHECK(&TS, redge[2] < redge[0] && redge[3] < redge[2], "edge RMS vs 8x falls 1x > 2x > 4x");
        {
            uint32_t changed = 0, near = 0, yy, xx;
            int32_t dy, dx;
            for (yy = 0; yy < HS_FB_H; yy++)
                for (xx = 0; xx < HS_FB_W; xx++) {
                    uint32_t hit = 0;
                    if ((img[1][yy * HS_FB_W + xx] & 0xFFFFFFu) == (img[0][yy * HS_FB_W + xx] & 0xFFFFFFu)) continue;
                    changed++;
                    for (dy = -2; dy <= 2 && !hit; dy++)
                        for (dx = -2; dx <= 2 && !hit; dx++) {
                            int32_t qy = (int32_t)yy + dy, qx = (int32_t)xx + dx;
                            if (qy >= 0 && qx >= 0 && qy < HS_FB_H && qx < HS_FB_W && mask[qy * HS_FB_W + qx]) hit = 1;
                        }
                    near += hit;
                }
            printf("INFO fxaa_changed_px=%u within_2px_of_8x_edge=%u\n", changed, near);
            HS_CHECK(&TS, changed > 1000u && near * 100u >= changed * 90u, "FXAA only touches pixels near real geometry edges (%u/%u)", near, changed);
        }
        free(mask);
    }
    {
        const uint32_t W = 1600u, H = 720u, cw = 80u, ch = 60u, z = 4u;
        uint32_t *sheet = (uint32_t *)calloc((size_t)W * H, 4), x, y, bx = 0, by = 0;
        uint64_t best = 0;
        for (i = 0; i < W * H; i++) sheet[i] = 0x202020u;
        for (y = 0; y + ch <= HS_FB_H; y += 10u)
            for (x = 0; x + cw <= HS_FB_W; x += 10u) {
                uint64_t e = 0;
                uint32_t yy, xx;
                for (yy = y; yy < y + ch; yy += 2u)
                    for (xx = x; xx < x + cw; xx += 2u) {
                        uint32_t a = img[0][yy * HS_FB_W + xx], b = img[4][yy * HS_FB_W + xx], k;
                        for (k = 0; k < 24u; k += 8u) { int32_t d = (int32_t)((a >> k) & 255u) - (int32_t)((b >> k) & 255u); e += (uint64_t)(d < 0 ? -d : d); }
                    }
                if (e > best) { best = e; bx = x; by = y; }
            }
        for (m = 0; m < 5u; m++) blit_zoom(sheet, W, m * 320u, 0, img[m], bx, by, cw, ch, z);
        blit_zoom(sheet, W, 0, 240, img[0], 0, 0, HS_FB_W, HS_FB_H, 1);
        blit_zoom(sheet, W, 640, 240, img[4], 0, 0, HS_FB_W, HS_FB_H, 1);
        for (y = 240u + by; y < 240u + by + ch; y++) { sheet[y * W + bx] = 0xFFFF00u; sheet[y * W + bx + cw - 1u] = 0xFFFF00u; sheet[y * W + 640u + bx] = 0xFFFF00u; sheet[y * W + 640u + bx + cw - 1u] = 0xFFFF00u; }
        for (x = bx; x < bx + cw; x++) { sheet[(240u + by) * W + x] = 0xFFFF00u; sheet[(240u + by + ch - 1u) * W + x] = 0xFFFF00u; sheet[(240u + by) * W + 640u + x] = 0xFFFF00u; sheet[(240u + by + ch - 1u) * W + 640u + x] = 0xFFFF00u; }
        printf("INFO compare_crop x=%u y=%u w=%u h=%u zoom=%u (top row: 1x fxaa 2x 4x 8x; bottom: full 1x | full 8x)\n", bx, by, cw, ch, z);
        snprintf(path, sizeof(path), "%s/n64_ssaa_compare.png", OUT);
        HS_CHECK(&TS, hs_png_write_xrgb(path, sheet, W, H, W) == 0, "write %s", path);
        free(sheet);
    }
    {
        bk_count_t cnt;
        n64_rdp_backend_t bk;
        uint32_t prims;
        memset(&cnt, 0, sizeof(cnt));
        memset(&bk, 0, sizeof(bk));
        bk.user = &cnt; bk.begin_frame = bk_begin; bk.draw_tri = bk_tri; bk.fill_rect = bk_fill; bk.tex_rect = bk_trect; bk.end_frame = bk_end;
        prims = hs_n64_walk_backend(N, &bk, 4);
        printf("INFO backend_walk prims=%u tris=%u textured=%u fills=%u texrects=%u\n", prims, cnt.tris, cnt.textured, cnt.fills, cnt.texrects);
        HS_CHECK(&TS, cnt.begins == 1u && cnt.ends == 1u && cnt.tris == n64_front(N)->ntris && cnt.texrects == 1u && cnt.fills == 2u &&
                      cnt.textured == cnt.tris, "Panfrost backend hook walks the frozen list (tris=%u fills=%u texrects=%u)", cnt.tris, cnt.fills, cnt.texrects);
        HS_CHECK(&TS, fb_hash(img[4]) == (hs_n64_core_ops.render_video_phase(N, FB, HS_FB_W, HS_SSAA_8X), fb_hash(FB)), "backend walk is read-only (re-render identical)");
    }
    {
        uint64_t p0, p1, p2;
        hs_n64_core_ops.step_cpu_phase(N, HS_BTN_START, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.step_cpu_phase(N, HS_BTN_START, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.hash_state(N, &p0, NULL);
        hs_n64_core_ops.step_cpu_phase(N, HS_BTN_START, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.step_cpu_phase(N, HS_BTN_START, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.hash_state(N, &p1, NULL);
        hs_n64_core_ops.step_cpu_phase(N, 0, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.step_cpu_phase(N, 0, 0); hs_n64_core_ops.commit_phase(N);
        hs_n64_core_ops.hash_state(N, &p2, NULL);
        HS_CHECK(&TS, p0 == p1 && p1 != p2, "joypad via SI: START freezes the scene, release resumes");
    }
    {
        double p0 = (double)t_step / frames / 1e6, p1 = (double)t_render1 / frames / 1e6, p2 = (double)t_synth / frames / 1e6;
        double mx = p0 > p1 ? p0 : p1;
        if (p2 > mx) mx = p2;
        printf("TIME sequential_1x_ms_per_frame %.3f\n", p0 + p1 + p2);
        printf("TIME pipelined_3core_bound_1x_ms_per_frame %.3f (max of phases; computed, no threads in this test)\n", mx);
    }
    for (m = 0; m < 5u; m++) free(img[m]);
    free(pcm_all);
}

/* ======================================================= fast paths == reference
 * Added with the v0.2 speed work. The decode-cache interpreter, the exact
 * idle-loop fast-forward and the fast RDP path must reproduce the reference
 * paths bit for bit (HS_N64_CFG_REF_INTERP / HS_N64_CFG_REF_RDP). */
static uint32_t fz_state;
static uint32_t fz(void) { fz_state = fz_state * 1664525u + 1013904223u; return fz_state >> 8; }

typedef struct cpu_ck { n64_cpu_t cpu; uint32_t frame_instr; uint64_t ram; } cpu_ck_t;

static void idle_prog(uint32_t wired)
{
    cpu_setup(0x8001u);                     /* IE + IM7, EXL/ERL/BEV clear */
    at(0x180u);
    P(LUI(K0, 0x8000)); P(LW(K1, 0x2000, K0)); P(ADDIU(K1, K1, 1)); P(SW(K1, 0x2000, K0));
    P(MFC0(K1, C0_COMPARE)); P(ADDIU(K1, K1, 997)); P(MTC0(K1, C0_COMPARE)); P(ERET);
    at(0x1000u);
    P(LUI(S0, 0x8000)); LI(T0, 500); P(MTC0(T0, C0_COMPARE)); LI(T0, wired); P(MTC0(T0, C0_WIRED));
    LI(T3, 10);
    P(LW(T1, 0x2000, S0)); P(BNE(T1, T3, -2)); P(NOP);          /* poll until 10 interrupts */
    P(ADDIU(T4, T4, 1));
    P(LW(T2, 0x2004, S0)); P(BEQ(T2, 0, -2)); P(NOP);           /* poll forever (word stays 0) */
    HALT();
}

static uint32_t idle_run(uint32_t flags, uint32_t wired, cpu_ck_t *ck, uint32_t nck)
{
    static const uint32_t chunk[8] = { 1, 2, 3, 7, 64, 1000, 4093, 25013 };
    uint32_t i;
    N->cfg_flags = flags;
    idle_prog(wired);
    for (i = 0; i < nck; i++) {
        run(chunk[i % 8u]);
        ck[i].cpu = N->cpu; ck[i].frame_instr = N->frame_instr;
        ck[i].ram = hs_fnv1a64(HS_FNV64_OFFSET, N->rdram, 0x3000u);
    }
    N->cfg_flags = 0;
    return (uint32_t)n64_be32(N->rdram + 0x2000u);
}

static void fz_scene(uint32_t seed)
{
    uint32_t k, nprim;
    uint8_t texb[512];
    fz_state = seed * 2654435761u + 1u;
    rdp_begin();
    rdp_targets();
    om(0x300000u, 0); RP(0x37000000u, fz() & 0xFFFFFFFFu); rect36(0, 0, 319, 239);
    RP(0x3F180000u | 319u, 0x380000u);                          /* Z clear through a Z-target fill */
    om(0x300000u, 0); RP(0x37000000u, 0xFFFCFFFCu); rect36(0, 0, 319, 239);
    RP(0x3F180000u | 319u, 0x300000u);
    for (k = 0; k < sizeof(texb); k++) texb[k] = (uint8_t)fz();
    nprim = 6u + fz() % 10u;
    for (k = 0; k < nprim; k++) {
        /* every draw is taken in its own statement (or a fixed q[] slot), so the
         * scene stream is identical whatever order a compiler evaluates operands in */
        uint32_t fmt = fz() % 5u, siz = fz() % 4u, cyc = fz() % 4u, kind = fz() % 8u, flags, t, j, q[24];
        uint32_t lo = fz(), hi;
        for (j = 0; j < 24u; j++) q[j] = fz();
        if (fmt == 1u) fmt = 3u;
        hi = (cyc << 20) | ((q[0] & 1u) << 19) | ((q[1] % 4u) << 12) | ((q[2] & 3u) << 14);
        if (k == 0u) { timg(0, 2, 16, texb, 32); settile(7, 0, 2, 4, 0x100, 0); loadblock(7, 16); } /* TLUT at 0x800 */
        RP(0xF5000000u | (fmt << 21) | (siz << 19) | ((1u + q[3] % 4u) << 9) | (q[4] % 64u), (q[5] & 0x00FFFFFFu) & ~0x07000000u);
        RP(0xF5000000u | (fmt << 21) | (siz << 19) | (2u << 9) | 32u, 0x01000000u | (q[6] & 0x00FFFFFFu));
        timg(fmt, siz, 16, texb + (q[7] % 128u), 256);
        loadblock(0, 64);
        RP(0xF2000000u | ((q[8] % 64u) << 12) | (q[9] % 64u), ((q[10] % 256u) << 12) | (q[11] % 256u));
        RP(0xFC000000u | (q[12] & 0x00FFFFFFu), q[13] ^ (q[14] << 8));
        RP(0xFA000000u | (q[15] & 0xFFu), q[16] ^ (q[17] << 8));
        RP(0xFB000000u, q[18] ^ (q[19] << 8));
        RP(0xF8000000u, q[20] ^ (q[21] << 8));
        RP(0xF9000000u, q[22] ^ (q[23] << 8));
        for (j = 0; j < 24u; j++) q[j] = fz();
        RP(0xEE000000u, q[0] << 8);
        om(hi, (lo << 8) ^ q[1]);
        if (q[2] % 6u == 0u) scissor(q[3] % 100u, q[4] % 80u, 160u + q[5] % 160u, 120u + q[6] % 120u);
        if (kind < 5u) {
            flags = N64_TRI_SHADE | ((q[7] & 1u) ? N64_TRI_Z : 0u) | ((q[8] % 3u) ? N64_TRI_TEX : 0u);
            RB->rdp[RB->rdp_n++] = (N64_OP_HS_TRI << 24) | ((q[9] % 8u) << 8) | flags;
            for (t = 0; t < 3u; t++) {
                for (j = 0; j < 11u; j++) q[j] = fz();
                RB->rdp[RB->rdp_n++] = (uint32_t)((int32_t)(q[0] % 420u) - 50) << 16 | (q[1] & 0xFFFFu);
                RB->rdp[RB->rdp_n++] = (uint32_t)((int32_t)(q[2] % 330u) - 45) << 16 | (q[3] & 0xFFFFu);
                RB->rdp[RB->rdp_n++] = q[4] & 0xFFFFu;
                RB->rdp[RB->rdp_n++] = 0x8000u + (q[5] % 0x60000u);
                RB->rdp[RB->rdp_n++] = (uint32_t)((int32_t)(q[6] % 4096u) - 1024) << 12;
                RB->rdp[RB->rdp_n++] = (uint32_t)((int32_t)(q[7] % 4096u) - 1024) << 12;
                RB->rdp[RB->rdp_n++] = q[8] ^ (q[9] << 8);
            }
        } else if (kind < 7u) {
            uint32_t x0 = q[10] % 300u, y0 = q[11] % 220u;
            RP4(((kind == 5u) ? 0xE4000000u : 0xE5000000u) | (((x0 + 4u + q[12] % 120u) << 2) << 12) | ((y0 + 2u + q[13] % 60u) << 2),
                ((q[14] % 8u) << 24) | ((x0 << 2) << 12) | (y0 << 2), q[15], (q[16] & 0xFFFFu) | ((q[17] & 0x3FFFu) << 16));
        } else {
            uint32_t x0 = q[10] % 300u, y0 = q[11] % 220u;
            RP(0x36000000u | (((x0 + q[12] % 100u) << 2) << 12) | ((y0 + q[13] % 60u) << 2), ((x0 << 2) << 12) | (y0 << 2));
        }
    }
    N->front ^= 1u;
}

static void test_fastpath(void)
{
    static const hs_video_mode_t modes[5] = { HS_SSAA_1X, HS_AA_FXAA, HS_SSAA_2X, HS_SSAA_4X, HS_SSAA_8X };
    hs_n64_t *A = N, *B;
    cpu_ck_t *ca, *cb;
    const uint32_t nck = 64u, frames = 90u, scenes = QUICK ? 40u : 160u;
    uint32_t i, f, w, m, bad = 0, badfb = 0, badsl = 0, badm = 0, badsc = 0, cnt_a, cnt_b, wireds[3] = { 0, 5, 31 };
    void *scratch[HS_N64_MAX_SLICES];
    uint64_t fzh = HS_FNV64_OFFSET;
    printf("[fastpath] decode cache + idle fast-forward + fast RDP == reference paths\n");
    ca = (cpu_ck_t *)calloc(nck, sizeof(cpu_ck_t)); cb = (cpu_ck_t *)calloc(nck, sizeof(cpu_ck_t));
    B = (hs_n64_t *)aligned_alloc(64, (sizeof(hs_n64_t) + 63u) & ~(size_t)63u);
    for (i = 0; i < HS_N64_MAX_SLICES; i++) scratch[i] = aligned_alloc(64, (hs_n64_slice_size() + 63u) & ~(size_t)63u);
    if (!ca || !cb || !B || !scratch[0] || !scratch[1] || !scratch[2] || !scratch[3]) { HS_CHECK(&TS, 0, "alloc"); return; }
    /* 1. synthetic idle loops with Count/Compare interrupts, odd run chunks */
    for (w = 0; w < 3u; w++) {
        uint32_t diff = 0;
        core_init(N, 0);
        cnt_a = idle_run(HS_N64_CFG_REF_INTERP, wireds[w], ca, nck);
        core_init(N, 0);
        cnt_b = idle_run(0u, wireds[w], cb, nck);
        for (i = 0; i < nck; i++)
            if (memcmp(&ca[i].cpu, &cb[i].cpu, sizeof(n64_cpu_t)) != 0 || ca[i].frame_instr != cb[i].frame_instr || ca[i].ram != cb[i].ram) diff++;
        HS_CHECK(&TS, diff == 0u && cnt_a == cnt_b && cnt_a > 10u && ca[nck - 1u].cpu.gpr[T4] == 1u,
                 "idle fast-forward == reference: Wired=%u, %u checkpoints, %u interrupts, 0 diffs (%u)", wireds[w], nck, cnt_a, diff);
    }
    /* 2. demo ROM: reference interpreter+RDP vs fast paths, every frame */
    core_init(A, HS_N64_CFG_REF_INTERP | HS_N64_CFG_REF_RDP); core_init(B, 0);
    hs_n64_core_ops.load_rom(A, ROM, ROMLEN); hs_n64_core_ops.load_rom(B, ROM, ROMLEN);
    for (f = 0; f < frames; f++) {
        uint64_t va, vb, aa, ab;
        hs_n64_core_ops.step_cpu_phase(A, 0, 0); hs_n64_core_ops.step_cpu_phase(B, 0, 0);
        hs_n64_core_ops.commit_phase(A); hs_n64_core_ops.commit_phase(B);
        hs_n64_core_ops.hash_state(A, &va, &aa); hs_n64_core_ops.hash_state(B, &vb, &ab);
        if (memcmp(&A->cpu, &B->cpu, sizeof(n64_cpu_t)) != 0 || A->frame_instr != B->frame_instr ||
            n64_cpu_state_hash(A) != n64_cpu_state_hash(B) || va != vb || aa != ab) bad++;
        hs_n64_core_ops.render_video_phase(A, FB, HS_FB_W, HS_SSAA_1X);
        hs_n64_core_ops.render_video_phase(B, FB2, HS_FB_W, HS_SSAA_1X);
        if (memcmp(FB, FB2, HS_FB_W * HS_FB_H * 4u) != 0) badfb++;
        if (f % 15u == 7u) {
            uint32_t cntsl = 1u + (f / 15u) % HS_N64_MAX_SLICES;
            memset(FB2, 0, HS_FB_W * HS_FB_H * 4u);
            for (i = 0; i < cntsl; i++)
                if (hs_n64_render_slice_ctx(B, scratch[i], hs_n64_slice_size(), FB2, HS_FB_W, HS_SSAA_1X, i, cntsl) != 0) badsl++;
            if (memcmp(FB, FB2, HS_FB_W * HS_FB_H * 4u) != 0) badsl++;
        }
    }
    HS_CHECK(&TS, bad == 0u, "demo: decode-cache + idle fast-forward CPU == reference, %u frames (%u diffs)", frames, bad);
    HS_CHECK(&TS, badfb == 0u, "demo: fast RDP 1x == reference RDP 1x, %u frames (%u diffs)", frames, badfb);
    HS_CHECK(&TS, badsl == 0u, "demo: 1..4 strip-interleaved slices (own scratch) == whole frame (%u diffs)", badsl);
    for (m = 1; m < (QUICK ? 3u : 5u); m++) {
        hs_n64_core_ops.render_video_phase(A, FB, HS_FB_W, modes[m]);
        hs_n64_core_ops.render_video_phase(B, FB2, HS_FB_W, modes[m]);
        if (memcmp(FB, FB2, HS_FB_W * HS_FB_H * 4u) != 0) badm++;
    }
    HS_CHECK(&TS, badm == 0u, "demo last frame: fast == reference in FXAA/2x/4x/8x (%u diffs)", badm);
    /* 3. RDP fuzz: random combiner/blender/other modes, formats, TLUT, tris, texrects, fills */
    for (i = 0; i < scenes; i++) {
        hs_video_mode_t md = (i % 4u == 3u) ? HS_SSAA_2X : ((i % 4u == 2u) ? HS_AA_FXAA : HS_SSAA_1X);
        core_init(N, (i & 1u) ? HS_N64_CFG_BILINEAR4 : 0u);
        fz_scene(i);
        fzh = hs_fnv1a64(fzh, n64_front(N)->rdp, (size_t)n64_front(N)->rdp_n * 4u);
        N->cfg_flags |= HS_N64_CFG_REF_RDP;
        rerender(md, FB);
        N->cfg_flags &= ~HS_N64_CFG_REF_RDP;
        rerender(md, FB2);
        if (memcmp(FB, FB2, HS_FB_W * HS_FB_H * 4u) != 0) badsc++;
    }
    printf("INFO fuzz_stream_hash %016llx (must match across arches)\n", (unsigned long long)fzh);
    HS_CHECK(&TS, badsc == 0u, "RDP fuzz: fast == reference on %u random scenes (1x/FXAA/2x, 3-point+bilinear) (%u diffs)", scenes, badsc);
    for (i = 0; i < HS_N64_MAX_SLICES; i++) free(scratch[i]);
    free(B); free(ca); free(cb);
    core_init(N, 0);
}

/* ======================================================= Panfrost step 1
 * Independent hex constants (computed by hand from the kernel UAPI:
 * dir<<30 | size<<16 | 'd'<<8 | nr) are compared with the macro encoder. */
static void test_panfrost(void)
{
    hs_pf_os_t os;
    hs_pf_mock_t *mk;
    hs_pf_dev_t d;
    hs_pf_bo_t img;
    hs_pf_fill_result_t fr;
    uint8_t job[64];
    uint32_t x, y, outside = 0;
    int r;
    printf("[panfrost] UAPI encoding, mock-device probe, WRITE_VALUE fill, RDP batch builder\n");
    HS_CHECK(&TS, HS_DRM_IOCTL_VERSION == 0xC0406400ul && HS_DRM_IOCTL_GEM_CLOSE == 0x40086409ul &&
                  HS_PF_IOCTL_SUBMIT == 0x40306440ul && HS_PF_IOCTL_WAIT_BO == 0x40106441ul &&
                  HS_PF_IOCTL_CREATE_BO == 0xC0186442ul && HS_PF_IOCTL_MMAP_BO == 0xC0106443ul &&
                  HS_PF_IOCTL_GET_PARAM == 0xC0106444ul,
             "DRM/Panfrost ioctl numbers match the hand-computed UAPI values");
    hs_pf_encode_job_header(job, HS_PF_JOB_WRITE_VALUE, 3u, 2u, 0x1122334455667788ull);
    hs_pf_encode_write_value(job + 32, 0x10002000ull, HS_PF_WV_IMM64, 0xAABBCCDDEEFF0011ull);
    HS_CHECK(&TS, job[16] == 0x05u && job[17] == 0 && job[18] == 3u && job[20] == 2u && job[24] == 0x88u && job[31] == 0x11u &&
                  job[0] == 0 && job[32] == 0x00u && job[33] == 0x20u && job[35] == 0x10u && job[40] == 7u && job[48] == 0x11u && job[55] == 0xAAu,
             "job header + WRITE_VALUE payload byte layout");
    mk = (hs_pf_mock_t *)malloc(sizeof(hs_pf_mock_t));
    if (!mk) { HS_CHECK(&TS, 0, "alloc mock"); return; }
    hs_pf_mock_init(mk, &os, HS_PF_PROD_G31);
    r = hs_pf_open(&d, &os, "/dev/dri/renderD128");
    HS_CHECK(&TS, r == HS_PF_OK && d.info.is_g31 && d.info.shader_cores == 2u && strcmp(d.info.driver, "panfrost") == 0,
             "probe on mock: driver=%s prod=0x%llx cores=%u g31=%u", d.info.driver, (unsigned long long)d.info.prod_id, d.info.shader_cores, d.info.is_g31);
    r = hs_pf_bo_new(&d, 80u * 70u * 4u, HS_PF_BO_NOEXEC, &img);
    if (r == HS_PF_OK) memset(img.cpu, 0x5A, img.size);
    r = r ? r : hs_pf_fill_write_value(&d, &img, 64u, 64u, 80u, 0xFF3C8859u, &fr);
    for (y = 0; y < 70u && r == HS_PF_OK; y++)
        for (x = 0; x < 80u; x++)
            if ((x >= 64u || y >= 64u) && n64_be32(img.cpu + (y * 80u + x) * 4u) != 0x5A5A5A5Au) outside++;
    HS_CHECK(&TS, r == HS_PF_OK && fr.jobs == 2048u && fr.done_jobs == 2048u && fr.bad_pixels == 0u && outside == 0u &&
                  mk->submits == 1u && mk->waits == 1u && mk->bad_jobs == 0u,
             "WRITE_VALUE chain fills 64x64 in an 80-stride BO on the mock: %u jobs DONE, 0 bad pixels, 0 outside writes (r=%d out=%u)",
             fr.done_jobs, r, outside);
    hs_pf_bo_free(&d, &img);
    hs_pf_close(&d);
    hs_pf_mock_init(mk, &os, 0x0750u);          /* a Midgard T760 id */
    r = hs_pf_open(&d, &os, "x");
    HS_CHECK(&TS, r == HS_PF_OK && !d.info.is_g31, "probe reports non-G31 GPUs as such");
    hs_pf_close(&d);
    free(mk);
    /* batch builder over the demo frame */
    {
        hs_pf_batch_t b;
        hs_pf_vtx_t *vtx = (hs_pf_vtx_t *)malloc(sizeof(hs_pf_vtx_t) * 8192u);
        hs_pf_draw_t draw[512];
        n64_rdp_backend_t bk;
        uint32_t prims, f, sum = 0, i, inside = 0;
        if (!vtx) { HS_CHECK(&TS, 0, "alloc vtx"); return; }
        core_init(N, 0);
        hs_n64_core_ops.load_rom(N, ROM, ROMLEN);
        for (f = 0; f < 30u; f++) { hs_n64_core_ops.step_cpu_phase(N, 0, 0); hs_n64_core_ops.commit_phase(N); }
        hs_pf_batch_init(&b, vtx, 8192u, draw, 512u);
        memset(&bk, 0, sizeof(bk));
        hs_pf_batch_backend(&b, &bk);
        prims = hs_n64_walk_backend(N, &bk, 1);
        for (i = 0; i < b.ndraw; i++) sum += draw[i].count;
        for (i = 0; i < b.nvtx; i++) if (vtx[i].x >= -64.0f && vtx[i].x <= 64.0f && vtx[i].y >= -64.0f && vtx[i].y <= 64.0f && vtx[i].rhw > 0.0f) inside++;   /* finite, guard band */
        printf("INFO pf_batch prims=%u draws=%u vertices=%u textures=%u fills=%u texrects=%u ntris=%u inside=%u sum=%u\n", prims, b.ndraw, b.nvtx, b.ntmem, b.fills, b.texrects, n64_front(N)->ntris, inside, sum);
        HS_CHECK(&TS, b.overflow == 0u && b.fills == 2u && b.texrects == 1u && sum == b.nvtx &&
                      b.nvtx == 3u * n64_front(N)->ntris + 6u * 3u && b.ndraw >= 2u && b.ndraw < prims && inside == b.nvtx,
                 "RDP->GPU batch: %u prims -> %u draws, %u vertices, %u TMEM uploads", prims, b.ndraw, b.nvtx, b.ntmem);
        free(vtx);
        core_init(N, 0);
    }
}

/* ===================================================== commercial-title compat
 * Synthetic checks only (no commercial ROM or microcode is in this tree):
 * microcode banner classification, F3DEX2 decoder == F3DEX decoder on the same
 * scene, EEPROM / SRAM / FlashRAM / controller pak protocols, CIC handling. */
static void put16(uint32_t a, uint32_t v) { n64_wbe16(N->rdram + a, (uint16_t)v); }
static void put32(uint32_t a, uint32_t v) { n64_wbe32(N->rdram + a, v); }
static void put_mtx(uint32_t a, const int32_t *m)
{
    uint32_t i;
    for (i = 0; i < 16u; i++) { put16(a + i * 2u, (uint32_t)m[i] >> 16); put16(a + 32u + i * 2u, (uint32_t)m[i] & 0xFFFFu); }
}
static void put_str(uint32_t a, const char *s) { uint32_t i; for (i = 0; s[i]; i++) N->rdram[a + i] = (uint8_t)s[i]; N->rdram[a + i] = 0; }

#define CA_PROJ 0x100000u
#define CA_MV   0x100040u
#define CA_VP   0x100080u
#define CA_LT   0x1000A0u
#define CA_VTX  0x100100u
#define CA_DL   0x101000u
#define CA_UC   0x102000u

static void compat_scene_data(void)
{
    int32_t p[16], m[16];
    uint32_t i;
    static const int16_t vx[8][3] = { { -80, -60, -20 }, { 80, -60, -10 }, { 0, 70, 0 }, { -90, 50, 10 },
                                      { 90, 55, 20 }, { -30, -80, 30 }, { 60, 0, -30 }, { -60, 10, 5 } };
    memset(p, 0, sizeof(p)); memset(m, 0, sizeof(m));
    p[0] = p[5] = p[10] = 0x290;  p[15] = 65536;          /* ~1/100 scale, w = 1 */
    m[0] = m[5] = m[10] = m[15] = 65536; m[12] = 5 << 16; /* translate x by 5 */
    put_mtx(CA_PROJ, p); put_mtx(CA_MV, m);
    put16(CA_VP + 0, 640); put16(CA_VP + 2, 480); put16(CA_VP + 4, 511); put16(CA_VP + 6, 0);
    put16(CA_VP + 8, 640); put16(CA_VP + 10, 480); put16(CA_VP + 12, 511); put16(CA_VP + 14, 0);
    put32(CA_LT + 0, 0xF0C08000u); put32(CA_LT + 4, 0xF0C08000u); put32(CA_LT + 8, 0x30508000u); put32(CA_LT + 12, 0);   /* light 1 */
    put32(CA_LT + 16, 0x20304000u); put32(CA_LT + 20, 0x20304000u); put32(CA_LT + 24, 0); put32(CA_LT + 28, 0);       /* ambient */
    for (i = 0; i < 8u; i++) {
        uint32_t a = CA_VTX + i * 16u;
        put16(a, (uint16_t)vx[i][0]); put16(a + 2u, (uint16_t)vx[i][1]); put16(a + 4u, (uint16_t)vx[i][2]); put16(a + 6u, 0);
        put16(a + 8u, i * 64u); put16(a + 10u, i * 48u);
        put32(a + 12u, 0x40000000u ^ (i * 0x1F3D5B7Fu) ^ 0xFFu);
    }
}

/* the same scene in two encodings; dialect 0 = F3DEX, 1 = F3DEX2 */
static void compat_dl(int x2, int lit)
{
    uint32_t a = CA_DL;
#define DL(w0, w1) do { put32(a, (w0)); put32(a + 4u, (w1)); a += 8u; } while (0)
    DL(x2 ? 0xDA380007u : 0x01030040u, CA_PROJ);                      /* projection, load */
    DL(x2 ? 0xDA380003u : 0x01020040u, CA_MV);                        /* modelview, load, no push */
    DL(x2 ? 0xDC080008u : 0x03800010u, CA_VP);                        /* viewport */
    DL(x2 ? 0xD9FFFFFFu : 0xB7000000u, x2 ? (0x1u | 0x4u | 0x200000u | 0x400u | (lit ? 0x20000u : 0u))
                                          : (0x1u | 0x4u | 0x200u | 0x2000u | (lit ? 0x20000u : 0u)));
    if (lit) {
        DL(x2 ? 0xDB020000u : 0xBC000002u, x2 ? 24u : 0x80000040u);   /* one directional light */
        DL(x2 ? 0xDC08060Au : 0x03860010u, CA_LT);                    /* light 1 */
        DL(x2 ? 0xDC08090Au : 0x03880010u, CA_LT + 16u);              /* ambient = light 2 */
    }
    DL(x2 ? 0xE3000A01u : 0xBA001402u, 0x00000000u);                 /* other mode H: cycle type = 1-cycle */
    DL(x2 ? 0xD7000002u : 0xBB000001u, 0x80008000u);                 /* texture on, tile 0, scale 0.5 */
    DL(0xFC121824u, 0xFF33FFFFu);                                     /* RDP pass-through: combine */
    DL(0xFA000000u, 0x80FF40C0u);                                     /* prim colour */
    DL(x2 ? (0x01000000u | (8u << 12) | (8u << 1)) : (0x04000000u | (0u << 16) | (8u << 10) | 127u), CA_VTX);
    DL(x2 ? 0x05000204u : 0xBF000000u, x2 ? 0u : 0x00000204u);        /* tri 0,1,2 */
    DL(x2 ? 0x06060a08u : 0xB1060a08u, 0x000a0c0eu);                  /* tri2 (3,5,4) (0,6,7) */
    DL(x2 ? 0xDF000000u : 0xB8000000u, 0);
#undef DL
}

static uint32_t compat_run_gfx(const char *banner, uint64_t *h)
{
    n64_snap_t *b = n64_back(N);
    b->rdp_n = 0; b->tex_n = 0; b->ntris = 0;
    put_str(CA_UC, banner);
    memset(N->rsp.dmem + 0xFC0u, 0, 64);
    n64_wbe32(N->rsp.dmem + 0xFC0u, 1u);
    n64_wbe32(N->rsp.dmem + 0xFD8u, banner[0] ? CA_UC : 0u);
    n64_wbe32(N->rsp.dmem + 0xFDCu, 0x100u);
    n64_wbe32(N->rsp.dmem + 0xFF0u, CA_DL);
    N->ucinfo.valid = 0;
    n64_rsp_start(N);
    *h = hs_fnv1a64(HS_FNV64_OFFSET, b->rdp, (size_t)b->rdp_n * 4u);
    return b->ntris;
}

static void pif_frame(const uint8_t *f, uint32_t len) { memset(N->pif_ram, 0, 64); memcpy(N->pif_ram, f, len); n64_pif_process(N); }

static void test_compat(void)
{
    uint64_t h1, h2, h3;
    uint32_t t1, t2, t3, i, ok;
    uint8_t blk[32];
    printf("[compat] microcode HLE dialects, cartridge saves, controller pak, CIC\n");
    {
        static const char *ban[6] = { "RSP SW Version: 2.0D, test", "RSP Gfx ucode F3DEX       fifo 0.95",
                                      "RSP Gfx ucode F3DEX2.NoN fifo 2.04", "RSP Gfx ucode F3DZEX.NoN fifo 2.06H",
                                      "RSP Gfx ucode S2DEX  fifo 1.06", "no banner here" };
        static const uint32_t want[6] = { N64_UC_FAST3D, N64_UC_F3DEX, N64_UC_F3DEX2, N64_UC_F3DEX2, N64_UC_S2DEX, N64_UC_UNKNOWN_GFX };
        ok = 0;
        for (i = 0; i < 6u; i++) ok += n64_ucode_classify((const uint8_t *)ban[i], (uint32_t)strlen(ban[i])) == want[i];
        HS_CHECK(&TS, ok == 6u, "microcode banner classifier: Fast3D / F3DEX / F3DEX2 / F3DZEX / S2DEX / unknown (%u/6)", ok);
    }
    core_init(N, 0);
    compat_scene_data();
    compat_dl(0, 0); t1 = compat_run_gfx("RSP Gfx ucode F3DEX       fifo 0.95", &h1);
    compat_dl(0, 0); t3 = compat_run_gfx("", &h3);
    compat_dl(1, 0); t2 = compat_run_gfx("RSP Gfx ucode F3DZEX.NoN fifo 2.06H", &h2);
    HS_CHECK(&TS, t1 == 3u && t2 == t1 && t3 == t1 && h1 == h2 && h1 == h3 && N->ucinfo.unhandled == 0u,
             "F3DZEX/F3DEX2 decoder emits the identical RDP stream as F3DEX for the same scene (%u tris, %016llx vs %016llx)",
             t2, (unsigned long long)h1, (unsigned long long)h2);
    compat_dl(0, 1); t1 = compat_run_gfx("RSP Gfx ucode F3DEX       fifo 0.95", &h1);
    compat_dl(1, 1); t2 = compat_run_gfx("RSP Gfx ucode F3DEX2.NoN fifo 2.04", &h2);
    HS_CHECK(&TS, t1 == t2 && t1 > 0u && h1 == h2, "F3DEX2 lighting (MOVEMEM light offsets, NUMLIGHT) == F3DEX (%u tris)", t2);
    {
        uint32_t na = N->ucinfo.audio_skipped;
        uint64_t ram = hs_fnv1a64(HS_FNV64_OFFSET, N->rdram, 0x200000u);
        memset(N->rsp.dmem + 0xFC0u, 0, 64);
        n64_wbe32(N->rsp.dmem + 0xFC0u, 2u); n64_wbe32(N->rsp.dmem + 0xFD8u, CA_UC); n64_wbe32(N->rsp.dmem + 0xFF0u, CA_DL);
        n64_rsp_start(N);
        HS_CHECK(&TS, N->ucinfo.audio_skipped == na + 1u && ram == hs_fnv1a64(HS_FNV64_OFFSET, N->rdram, 0x200000u) && (N->rsp.status & 1u),
                 "Nintendo-ABI audio task: skipped safely (RDRAM untouched, RSP halted)");
    }
    /* EEPROM 4K over joybus channel 4 */
    core_init(N, HS_N64_CFG_SAVE(2));
    N->rom_len = 0; n64_save_setup(N);
    {
        uint8_t st[] = { 0, 0, 0, 0, 0x01, 0x03, 0x00, 0, 0, 0, 0xFE };
        uint8_t wr[] = { 0, 0, 0, 0, 0x0A, 0x01, 0x05, 0x07, 1, 2, 3, 4, 5, 6, 7, 8, 0, 0xFE };
        uint8_t rd[] = { 0, 0, 0, 0, 0x02, 0x08, 0x04, 0x07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE };
        pif_frame(st, sizeof(st));
        ok = N->pif_ram[7] == 0x00 && N->pif_ram[8] == 0x80 && N->pif_ram[9] == 0x00;
        pif_frame(wr, sizeof(wr));
        ok &= N->save.data[56] == 1u && N->save.data[63] == 8u && N->save.dirty == 1u;
        pif_frame(rd, sizeof(rd));
        ok &= N->pif_ram[8] == 1u && N->pif_ram[15] == 8u;
        HS_CHECK(&TS, ok, "EEPROM 4 Kbit: info 00 80 00, write + read block 7 over joybus channel 4");
        {
            uint8_t ctl[] = { 0xFF, 0x01, 0x03, 0x00, 0xFF, 0xFF, 0xFF, 0xFE };
            pif_frame(ctl, sizeof(ctl));
            HS_CHECK(&TS, N->pif_ram[6] == 0x02, "no controller pak configured: status reports none (0x02)");
        }
    }
    /* SRAM via PI DMA both ways + persistence across reset + save file round trip */
    core_init(N, HS_N64_CFG_SAVE(4));
    N->rom_len = 0; n64_save_setup(N);
    for (i = 0; i < 256u; i++) N->rdram[0x200000u + i] = (uint8_t)(i * 7u + 1u);
    n64_phys_write32(N, 0x04600000u, 0x200000u, 0xFFFFFFFFu); n64_phys_write32(N, 0x04600004u, 0x08000100u, 0xFFFFFFFFu);
    n64_phys_write32(N, 0x04600008u, 255u, 0xFFFFFFFFu);                                  /* RDRAM -> SRAM */
    memset(N->rdram + 0x210000u, 0, 256);
    n64_phys_write32(N, 0x04600000u, 0x210000u, 0xFFFFFFFFu); n64_phys_write32(N, 0x04600004u, 0x08000100u, 0xFFFFFFFFu);
    n64_phys_write32(N, 0x0460000Cu, 255u, 0xFFFFFFFFu);                                  /* SRAM -> RDRAM */
    ok = memcmp(N->rdram + 0x200000u, N->rdram + 0x210000u, 256) == 0 && N->save.data[0x100] == 1u && N->save.size == 32768u;
    hs_n64_core_ops.reset(N);
    ok &= N->save.data[0x101] == 8u && N->save.type == N64_SAVE_SRAM;
    {
        size_t need = hs_n64_core_ops.save_sram(N, NULL, 0), got;
        uint8_t *buf = (uint8_t *)malloc(need);
        got = buf ? hs_n64_core_ops.save_sram(N, buf, need) : 0u;
        if (buf) { buf[0x102] ^= 0xFFu; ok &= hs_n64_core_ops.load_sram(N, buf, got) == 0 && N->save.data[0x102] == (uint8_t)~15u; free(buf); }
        ok &= need == 32768u && got == need;
    }
    HS_CHECK(&TS, ok, "SRAM 256 Kbit: PI DMA write + read back, survives reset, save file round trip");
    /* FlashRAM: ID, sector erase, page program, read array, status */
    core_init(N, HS_N64_CFG_SAVE(5));
    N->rom_len = 0; n64_save_setup(N);
    {
        uint32_t st;
        ok = 1;
        n64_phys_write32(N, 0x08010000u, 0xE1000000u, 0xFFFFFFFFu);                       /* read ID */
        n64_phys_write32(N, 0x04600000u, 0x220000u, 0xFFFFFFFFu); n64_phys_write32(N, 0x04600004u, 0x08000000u, 0xFFFFFFFFu);
        n64_phys_write32(N, 0x0460000Cu, 7u, 0xFFFFFFFFu);
        ok &= n64_be32(N->rdram + 0x220000u) == 0x11118001u && n64_be32(N->rdram + 0x220004u) == 0x00C2001Eu;
        memset(N->save.data + 0x4000u, 0x00, 0x4000u);
        n64_phys_write32(N, 0x08010000u, 0x4B000080u, 0xFFFFFFFFu);                       /* sector erase page 128 */
        n64_phys_write32(N, 0x08010000u, 0x78000000u, 0xFFFFFFFFu);
        n64_phys_write32(N, 0x08010000u, 0xD2000000u, 0xFFFFFFFFu);
        st = n64_phys_read32(N, 0x08000000u);
        ok &= N->save.data[0x4000u] == 0xFFu && N->save.data[0x7FFFu] == 0xFFu && (st & 0xFFu) == 0x08u;
        for (i = 0; i < 128u; i++) N->rdram[0x230000u + i] = (uint8_t)(0xA0u + i);
        n64_phys_write32(N, 0x08010000u, 0xB4000000u, 0xFFFFFFFFu);                       /* page buffer */
        n64_phys_write32(N, 0x04600000u, 0x230000u, 0xFFFFFFFFu); n64_phys_write32(N, 0x04600004u, 0x08000000u, 0xFFFFFFFFu);
        n64_phys_write32(N, 0x04600008u, 127u, 0xFFFFFFFFu);
        n64_phys_write32(N, 0x08010000u, 0xA5000085u, 0xFFFFFFFFu);                       /* program page 133 */
        n64_phys_write32(N, 0x08010000u, 0xD2000000u, 0xFFFFFFFFu);
        ok &= (n64_phys_read32(N, 0x08000000u) & 0xFFu) == 0x04u && N->save.data[133u * 128u] == 0xA0u;
        n64_phys_write32(N, 0x08010000u, 0xF0000000u, 0xFFFFFFFFu);                       /* read array, page 133 */
        memset(N->rdram + 0x240000u, 0, 128);
        n64_phys_write32(N, 0x04600000u, 0x240000u, 0xFFFFFFFFu); n64_phys_write32(N, 0x04600004u, 0x08000000u + 133u * 64u, 0xFFFFFFFFu);
        n64_phys_write32(N, 0x0460000Cu, 127u, 0xFFFFFFFFu);
        ok &= memcmp(N->rdram + 0x240000u, N->rdram + 0x230000u, 128) == 0 && N->save.size == 131072u;
        HS_CHECK(&TS, ok, "FlashRAM 1 Mbit: ID 11118001 00C2001E, sector erase, page program, read array, status");
    }
    /* controller pak */
    core_init(N, HS_N64_CFG_CONTROLLER_PAK);
    N->rom_len = 0; n64_save_setup(N);
    {
        uint8_t f[64];
        memset(blk, 0, sizeof(blk));
        ok = n64_pak_crc8(blk, 32) == 0u;
        for (i = 0; i < 32u; i++) blk[i] = (uint8_t)(i * 13u + 5u);
        memset(f, 0, sizeof(f));
        f[0] = 35; f[1] = 1; f[2] = 0x03; f[3] = 0x01; f[4] = 0x20 | 0x15; memcpy(f + 5, blk, 32); f[38] = 0xFE;   /* write 0x0120 */
        pif_frame(f, 39);
        ok &= N->pif_ram[37] == n64_pak_crc8(blk, 32) && N->save.pak[0x120] == 5u && N->save.pak_dirty == 1u;
        memset(f, 0, sizeof(f));
        f[0] = 3; f[1] = 33; f[2] = 0x02; f[3] = 0x01; f[4] = 0x20; f[38] = 0xFE;                                    /* read 0x0120 */
        pif_frame(f, 39);
        ok &= memcmp(N->pif_ram + 5, blk, 32) == 0 && N->pif_ram[37] == n64_pak_crc8(blk, 32);
        memset(f, 0, sizeof(f)); f[0] = 1; f[1] = 3; f[2] = 0x00; f[6] = 0xFE;
        pif_frame(f, 7);
        ok &= N->pif_ram[5] == 0x01;
        HS_CHECK(&TS, ok, "controller pak: status 0x01, 32-byte write/read at 0x0120, data CRC (crc8 %02x)", n64_pak_crc8(blk, 32));
    }
    /* CIC: CRC-32 check value, unknown boot code -> 6102 path, header-checksum variants */
    core_init(N, 0);
    {
        static const char cv[] = "123456789";
        uint8_t *rom = (uint8_t *)calloc(1, 0x102000u);
        uint32_t c1, c2, cic, c32;
        if (!rom) { HS_CHECK(&TS, 0, "alloc rom"); return; }
        rom[0] = 0x80; rom[1] = 0x37; rom[2] = 0x12; rom[3] = 0x40;
        n64_wbe32(rom + 8, 0x80000400u);
        for (i = 0x1000u; i < 0x102000u; i += 4u) n64_wbe32(rom + i, i * 2654435761u);
        N->rom = rom; N->rom_len = 0x102000u; N->rom_fmt = 0;
        memcpy(rom + 0x40, cv, 9);
        cic = n64_cic_detect(N, &c32);
        ok = cic == 0u && n64_rom_crc32(N, 0x40u, 0x49u) == 0xCBF43926u;
        n64_rom_crc_cic(N, N64_CIC_6102, &c1, &c2);
        n64_wbe32(rom + 0x10, c1); n64_wbe32(rom + 0x14, c2);
        ok &= hs_n64_core_ops.load_rom(N, rom, 0x102000u) == 0 && N->rom_crc_ok == 1u && N->save.cic == N64_CIC_6102 && !N->save.cic_known &&
              N->cpu.gpr[22] == 0x3Fu && N->cpu.pc == 0xFFFFFFFF80000400ull;
        {
            uint32_t a1, a2, b1, b2, d1, d2;
            n64_rom_crc_cic(N, N64_CIC_6103, &a1, &a2); n64_rom_crc_cic(N, N64_CIC_6105, &b1, &b2); n64_rom_crc_cic(N, N64_CIC_6106, &d1, &d2);
            ok &= a1 != c1 && b1 != c1 && d1 != c1 && a2 != b2 && b2 != d2;
        }
        {
            static const char ids[3][3] = { "SM", "ZL", "ZS" };
            static const uint32_t ty[3] = { N64_SAVE_EEP4K, N64_SAVE_SRAM, N64_SAVE_FLASH }, mem[3] = { 4u << 20, 4u << 20, 8u << 20 };
            uint32_t k, okid = 1;
            for (k = 0; k < 3u; k++) {
                core_init(N, 0);
                rom[0x3C] = (uint8_t)ids[k][0]; rom[0x3D] = (uint8_t)ids[k][1];
                okid &= hs_n64_core_ops.load_rom(N, rom, 0x102000u) == 0 && N->save.type == ty[k] && N->rdram_size == mem[k] &&
                        n64_be32(N->rdram + 0x318u) == mem[k];
            }
            HS_CHECK(&TS, okid, "header ID table: SM -> EEPROM 4K, ZL -> SRAM, ZS -> FlashRAM + 8 MiB (synthetic headers)");
        }
        HS_CHECK(&TS, ok, "CIC: CRC-32 check value CBF43926, unknown IPL3 -> 6102 boot (seed 0x3F), 6103/6105/6106 checksum variants distinct");
        N->rom = NULL; N->rom_len = 0;
        free(rom);
    }
    core_init(N, 0);
}

/* ================================================================ main */
int main(int argc, char **argv)
{
    int i;
    size_t sz = (sizeof(hs_n64_t) + 63u) & ~(size_t)63u;
    for (i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--out=", 6) == 0) OUT = argv[i] + 6;
        else if (strcmp(argv[i], "--quick") == 0) QUICK = 1;
    }
    N = (hs_n64_t *)aligned_alloc(64, sz);
    FB = (uint32_t *)aligned_alloc(64, HS_FB_W * HS_FB_H * 4u);
    FB2 = (uint32_t *)aligned_alloc(64, HS_FB_W * HS_FB_H * 4u);
    ROM = (uint8_t *)malloc(4u << 20);
    if (!N || !FB || !FB2 || !ROM) { printf("alloc failed\n"); return 2; }
    ROMLEN = hs_read_file("roms/hs_n64_demo.z64", ROM, 4u << 20);
    if (ROMLEN < 0x101000u) { free(ROM); ROM = NULL; ROMLEN = 0; }
    printf("hs-n64 torture suite: core state %zu bytes, ROM %zu bytes\n", sizeof(hs_n64_t), ROMLEN);
    memset(N, 0, sz);
    core_init(N, 0);
    test_cpu_alu();
    test_cpu_64();
    test_cpu_branch();
    test_cpu_mem();
    test_cpu_exc();
    test_fpu();
    test_bus();
    test_rsp_math();
    test_rsp_hle();
    test_rdp();
    test_isolation();
    test_demo_rom();
    test_fastpath();
    test_panfrost();
    test_compat();
    printf("RESULT %u/%u\n", TS.pass, TS.pass + TS.fail);
    free(ROM); free(FB); free(FB2); free(N);
    return TS.fail ? 1 : 0;
}
