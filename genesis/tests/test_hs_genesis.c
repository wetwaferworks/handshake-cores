/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * test_hs_genesis.c - hs-genesis verification harness:
 *   1. M68000 torture tests (flat 64 KB test bus)
 *   2. VDP DMA (68k->VRAM/CRAM, fill, copy) + VSRAM + 3/6-button pad I/O
 *   3. Z80 instruction tests, YM2612 + SN76489 unit tests
 *   4. Synthetic Genesis ROM assembled in C (real "SEGA MEGA DRIVE " header,
 *      vector table, VDP init, V-Int scroll + sprites, H-Int raster colours,
 *      Z80-driven YM2612 FM chord, 68k-driven SN76489 melody), 120 frames
 *      through the full 3-phase ABI, FNV-1a hashes of every 640x480 frame
 *      and every PCM block, edge-fill check, preview PNG, phase timing.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/gen_internal.h"
#include "m68k_asm.h"

uint64_t hs_bench_now_ns(void);

static int g_pass, g_fail;
#define CHECK(name, cond) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL %s (%s:%d)\n", name, __FILE__, __LINE__); } } while (0)
#define CHECKX(name, got, want) do { uint64_t g_ = (uint64_t)(got), w_ = (uint64_t)(want); \
    if (g_ == w_) g_pass++; else { g_fail++; printf("  FAIL %s: got 0x%llx want 0x%llx (line %d)\n", name, \
    (unsigned long long)g_, (unsigned long long)w_, __LINE__); } } while (0)

/* ======================================================================
 * 1. M68000 torture tests on a flat 64 KB bus
 * ==================================================================== */
typedef struct tctx { m68k_t cpu; uint8_t mem[65536]; } tctx_t;
static tctx_t *T;

static uint8_t  tr8(void *c, uint32_t a)  { return ((tctx_t *)c)->mem[a & 0xFFFF]; }
static uint16_t tr16(void *c, uint32_t a) { tctx_t *t = (tctx_t *)c; return (uint16_t)((t->mem[a & 0xFFFF] << 8) | t->mem[(a + 1) & 0xFFFF]); }
static void tw8(void *c, uint32_t a, uint8_t v) { ((tctx_t *)c)->mem[a & 0xFFFF] = v; }
static void tw16(void *c, uint32_t a, uint16_t v) { tctx_t *t = (tctx_t *)c; t->mem[a & 0xFFFF] = (uint8_t)(v >> 8); t->mem[(a + 1) & 0xFFFF] = (uint8_t)v; }
static uint32_t tiack(void *c, int level) { ((tctx_t *)c)->cpu.irq_level = 0; return 24u + (uint32_t)level; }

static uint32_t m_l(uint32_t a) { return ((uint32_t)tr16(T, a) << 16) | tr16(T, a + 2); }
static void m_wl(uint32_t a, uint32_t v) { tw16(T, a, (uint16_t)(v >> 16)); tw16(T, a + 2, (uint16_t)v); }

#define SNAP 0x7000u          /* SR snapshots */
#define TPROG 0x0400u

static void t_begin(asm68_t *a)
{
    uint32_t i;
    m68k_bus_t b = { tr8, tr16, tw8, tw16, tiack, NULL };
    b.ctx = T;
    m68k_init(&T->cpu, &b);
    memset(T->mem, 0, sizeof(T->mem));
    m_wl(0, 0x8000);
    m_wl(4, TPROG);
    for (i = 2; i < 64; i++) m_wl(i * 4, 0x0300);     /* unexpected -> 0x300 */
    tw16(T, 0x300, 0x4E72); tw16(T, 0x302, 0x2700);    /* STOP #$2700 */
    m_wl(0x7FF0, 0xDEAD);                               /* marker */
    a_init(a, T->mem, 0x10000, TPROG);
}
static void snap(asm68_t *a, int k) { a_move_from_sr(a, EA_AW(SNAP + 2u * (uint32_t)k)); }
static uint16_t ccr(int k) { return (uint16_t)(tr16(T, SNAP + 2u * (uint32_t)k) & 0x1F); }
static void t_go(asm68_t *a, uint32_t budget)
{
    uint32_t used = 0;
    a_finish(a);
    m68k_reset(&T->cpu);
    while (!T->cpu.stopped && used < budget) used += m68k_run(&T->cpu, 256);
}

static void test_m68k(void)
{
    asm68_t A, *a = &A;
    printf("[m68k] ALU/flags\n");
    t_begin(a);
    a_moveq(a, 0x7F, 0); a_imm(a, A_ADDI, 1, 1, EA_D(0)); snap(a, 0);
    a_move(a, 4, EA_IMM(4, 0xFFFFFFFFu), EA_D(1)); a_addq(a, 4, 1, EA_D(1)); snap(a, 1);
    a_move(a, 4, EA_IMM(4, 0x80000000u), EA_D(2)); a_subq(a, 4, 1, EA_D(2)); snap(a, 2);
    a_moveq(a, 5, 3); a_imm(a, A_CMPI, 2, 7, EA_D(3)); snap(a, 3);
    a_moveq(a, 1, 4); a_unary(a, A_NEG, 4, EA_D(4)); snap(a, 4);
    /* 64-bit add with ADDX: d0:d1 = 1:FFFFFFFF + 0:1 */
    a_moveq(a, 1, 0); a_moveq(a, -1, 1); a_moveq(a, 0, 2); a_moveq(a, 1, 3);
    a_op_ead(a, A_ADD, 4, EA_D(3), 1); a_addx(a, 4, 2, 0); snap(a, 5);
    /* 64-bit sub with SUBX: d5:d6 = 2:0 - 0:1 */
    a_moveq(a, 2, 5); a_moveq(a, 0, 6); a_moveq(a, 1, 7);
    a_op_ead(a, A_SUB, 4, EA_D(7), 6); a_moveq(a, 0, 7); a_subx(a, 4, 7, 5); snap(a, 6);
    a_stop(a, 0x2700);
    t_go(a, 100000);
    CHECKX("addi.b 7F+1 ccr(NV)", ccr(0), 0x0A);
    CHECKX("addq.l FFFFFFFF+1 ccr(XZC)", ccr(1), 0x15);
    CHECKX("subq.l 80000000-1 ccr(V)", ccr(2), 0x02);
    CHECKX("cmpi.w 5-7 ccr(NC)", ccr(3), 0x09);
    CHECKX("neg.l 1 ccr(XNC)", ccr(4), 0x19);
    CHECKX("neg.l result", T->cpu.d[4], 0xFFFFFFFFu);
    CHECKX("addx hi", T->cpu.d[0], 2); CHECKX("addx lo", T->cpu.d[1], 0);
    CHECKX("subx hi", T->cpu.d[5], 1); CHECKX("subx lo", T->cpu.d[6], 0xFFFFFFFFu);

    printf("[m68k] MULU/MULS/DIVU/DIVS\n");
    t_begin(a);
    m_wl(0x14, 0x0380);                            /* vector 5: divide by zero */
    a_org(a, 0x380); a_addq(a, 2, 1, EA_AW(0x7100)); a_rte(a);
    a_org(a, TPROG);
    a_move(a, 4, EA_IMM(4, 0xFFFF), EA_D(0)); a_mulu(a, EA_IMM(2, 0xFFFF), 0);
    a_move(a, 4, EA_IMM(4, 0xFFFE), EA_D(1)); a_muls(a, EA_IMM(2, 300), 1); snap(a, 0);
    a_move(a, 4, EA_IMM(4, 100000), EA_D(2)); a_divu(a, EA_IMM(2, 7), 2);
    a_move(a, 4, EA_IMM(4, (uint32_t)-100000), EA_D(3)); a_divs(a, EA_IMM(2, 7), 3);
    a_move(a, 4, EA_IMM(4, 0x00100000), EA_D(4)); a_divu(a, EA_IMM(2, 1), 4); snap(a, 1);
    a_moveq(a, 9, 5); a_moveq(a, 0, 6); a_divu(a, EA_D(6), 5);
    a_moveq(a, 0, 6); a_divs(a, EA_D(6), 5);
    a_stop(a, 0x2700);
    t_go(a, 100000);
    CHECKX("mulu FFFF*FFFF", T->cpu.d[0], 0xFFFE0001u);
    CHECKX("muls -2*300", T->cpu.d[1], 0xFFFFFDA8u);
    CHECKX("muls ccr N", ccr(0), 0x08);
    CHECKX("divu 100000/7", T->cpu.d[2], 0x000537CDu);
    CHECKX("divs -100000/7", T->cpu.d[3], 0xFFFBC833u);
    CHECKX("divu overflow keeps Dn", T->cpu.d[4], 0x00100000u);
    CHECK("divu overflow sets V", (ccr(1) & 2) != 0);
    CHECKX("div-by-zero traps x2", tr16(T, 0x7100), 2);
    CHECKX("div0 Dn untouched", T->cpu.d[5], 9);

    printf("[m68k] MOVEM / LINK / UNLK / BSR / RTS / DBcc\n");
    t_begin(a);
    {
        int i;
        for (i = 0; i < 8; i++) a_move(a, 4, EA_IMM(4, 0x11111111u * (uint32_t)(i + 1)), EA_D(i));
        for (i = 0; i < 7; i++) a_move(a, 4, EA_IMM(4, 0x01010101u * (uint32_t)(i + 0x20)), EA_A(i));
        a_movem(a, 4, 1, 0xFFFE, EA_PD(7));               /* d0-d7/a0-a6 */
        a_move(a, 4, EA_A(7), EA_AW(0x7200));
        for (i = 0; i < 8; i++) a_moveq(a, 0, i);
        for (i = 0; i < 7; i++) a_movea(a, 4, EA_IMM(4, 0), i);
        a_movem(a, 4, 0, 0x7FFF, EA_PI(7));
        a_move(a, 4, EA_D(3), EA_AW(0x7230)); a_move(a, 4, EA_D(7), EA_AW(0x7234));
        a_move(a, 4, EA_A(2), EA_AW(0x7238));
        /* word MOVEM sign extension */
        a_move(a, 2, EA_IMM(2, 0x8000), EA_AW(0x7300)); a_move(a, 2, EA_IMM(2, 0x1234), EA_AW(0x7302));
        a_lea(a, EA_AW(0x7300), 0);
        a_movem(a, 2, 0, 0x00C0, EA_IND(0));               /* d6,d7 */
        a_movem(a, 2, 0, 0x0300, EA_IND(0));               /* a0,a1 (mask bits 8,9) */
        /* DBRA loop: d1 = 10*3 */
        a_moveq(a, 9, 0); a_moveq(a, 0, 1);
        a_label(a, 1); a_addq(a, 4, 3, EA_D(1)); a_dbra(a, 0, 1);
        /* DBEQ early exit */
        a_moveq(a, 0, 2); a_moveq(a, 20, 3);
        a_label(a, 2); a_addq(a, 2, 1, EA_D(2)); a_imm(a, A_CMPI, 2, 5, EA_D(2)); a_dbcc(a, CC_EQ, 3, 2);
        /* BSR/RTS */
        a_move(a, 4, EA_A(7), EA_D(4));
        a_bsr(a, 3);
        a_move(a, 4, EA_A(7), EA_AW(0x7210));
        /* LINK/UNLK */
        a_movea(a, 4, EA_IMM(4, 0x1234), 6);
        a_link(a, 6, -8);
        a_move(a, 4, EA_A(7), EA_AW(0x7220));
        a_move(a, 4, EA_A(6), EA_AW(0x7224));
        a_move(a, 4, EA_IMM(4, 0xCAFEBABEu), EA_DISP(6, -4));
        a_unlk(a, 6);
        a_move(a, 4, EA_A(6), EA_AW(0x7228));
        a_move(a, 4, EA_A(7), EA_AW(0x722C));
        a_stop(a, 0x2700);
        a_label(a, 3); a_moveq(a, 42, 5); a_rts(a);
    }
    t_go(a, 200000);
    CHECKX("movem pushed 15 longs", m_l(0x7200), 0x8000u - 60u);
    CHECKX("movem restore d3", m_l(0x7230), 0x44444444u);
    CHECKX("movem restore d7", m_l(0x7234), 0x88888888u);
    CHECKX("movem restore a2", m_l(0x7238), 0x22222222u);
    CHECKX("movem restore a5", T->cpu.a[5], 0x25252525u);
    CHECKX("movem.w sign-extend a0", T->cpu.a[0], 0xFFFF8000u);
    CHECKX("movem.w a1", T->cpu.a[1], 0x00001234u);
    CHECKX("movem.w d6", T->cpu.d[6], 0xFFFF8000u);
    CHECKX("movem.w d7", T->cpu.d[7], 0x00001234u);
    CHECKX("dbra sum", T->cpu.d[1], 30);
    CHECKX("dbra end count", T->cpu.d[0] & 0xFFFF, 0xFFFF);
    CHECKX("dbeq exit value", T->cpu.d[2] & 0xFFFF, 5);
    CHECKX("dbeq counter", T->cpu.d[3] & 0xFFFF, 16);
    CHECKX("bsr/rts value", T->cpu.d[5], 42);
    CHECKX("bsr/rts sp balanced", m_l(0x7210), T->cpu.d[4]);
    CHECKX("link sp", m_l(0x7220), 0x8000u - 4u - 8u);
    CHECKX("link fp", m_l(0x7224), 0x8000u - 4u);
    CHECKX("link local", m_l(0x8000u - 8u), 0xCAFEBABEu);
    CHECKX("unlk fp", m_l(0x7228), 0x1234);
    CHECKX("unlk sp", m_l(0x722C), 0x8000);

    printf("[m68k] shifts / rotates\n");
    t_begin(a);
    a_move(a, 4, EA_IMM(4, 0x40000000u), EA_D(0)); a_shi(a, 0, 1, 4, 1, 0); snap(a, 0);
    a_move(a, 4, EA_IMM(4, 0x80000001u), EA_D(1)); a_shi(a, 0, 0, 4, 1, 1); snap(a, 1);
    a_moveq(a, 0, 2); a_imm(a, A_ORI, 1, 0x80, EA_D(2)); a_ori_ccr(a, 0x10); a_shi(a, 2, 1, 1, 1, 2); snap(a, 2);
    a_move(a, 4, EA_IMM(4, 0x1234), EA_D(3)); a_shi(a, 1, 0, 2, 4, 3); snap(a, 3);
    a_move(a, 4, EA_IMM(4, 0x12345678u), EA_D(4)); a_shi(a, 3, 0, 4, 8, 4);
    a_moveq(a, 33, 6); a_moveq(a, 1, 7); a_shr(a, 1, 1, 4, 6, 7); snap(a, 4);
    a_move(a, 4, EA_IMM(4, 0x0000C000u), EA_D(5)); a_shi(a, 0, 1, 2, 2, 5); snap(a, 5);
    a_move(a, 2, EA_IMM(2, 0x8001), EA_AW(0x7400)); a_shm(a, 3, 1, EA_AW(0x7400));
    a_stop(a, 0x2700);
    t_go(a, 100000);
    CHECKX("asl.l 40000000", T->cpu.d[0], 0x80000000u);
    CHECKX("asl ccr NV", ccr(0), 0x0A);
    CHECKX("asr.l 80000001", T->cpu.d[1], 0xC0000000u);
    CHECKX("asr ccr XNC", ccr(1), 0x19);
    CHECKX("roxl.b X-in", T->cpu.d[2] & 0xFF, 0x01);
    CHECKX("roxl ccr XC", ccr(2), 0x11);
    CHECKX("lsr.w #4", T->cpu.d[3] & 0xFFFF, 0x0123);
    CHECKX("ror.l #8", T->cpu.d[4], 0x78123456u);
    CHECKX("lsl.l d6=33", T->cpu.d[7], 0);
    CHECKX("lsl 33 ccr Z", ccr(4), 0x04);
    CHECKX("asl.w #2 C000", T->cpu.d[5] & 0xFFFF, 0x0000);
    CHECKX("asl.w overflow ccr XZVC", ccr(5), 0x17);
    CHECKX("rol.w mem", tr16(T, 0x7400), 0x0003);

    printf("[m68k] BCD + bit ops + misc\n");
    t_begin(a);
    a_moveq(a, 0x45, 0); a_moveq(a, 0x38, 1);
    a_move_to_ccr(a, EA_IMM(2, 0x04)); a_abcd(a, 0, 1); snap(a, 0);
    a_move(a, 4, EA_IMM(4, 0x99), EA_D(2)); a_moveq(a, 0x01, 3);
    a_move_to_ccr(a, EA_IMM(2, 0x04)); a_abcd(a, 2, 3); snap(a, 1);
    a_moveq(a, 0x10, 4); a_moveq(a, 0x01, 5);
    a_move_to_ccr(a, EA_IMM(2, 0x00)); a_sbcd(a, 5, 4); snap(a, 2);
    a_moveq(a, 0x01, 6);
    a_move_to_ccr(a, EA_IMM(2, 0x00)); a_nbcd(a, EA_D(6)); snap(a, 3);
    a_stop(a, 0x2700);
    t_go(a, 100000);
    CHECKX("abcd 45+38", T->cpu.d[1] & 0xFF, 0x83);
    CHECKX("abcd non-zero clears Z", ccr(0) & 0x15, 0x00);
    CHECKX("abcd 99+01", T->cpu.d[3] & 0xFF, 0x00);
    CHECKX("abcd carry ccr XZC", ccr(1) & 0x15, 0x15);
    CHECKX("sbcd 10-01", T->cpu.d[4] & 0xFF, 0x09);
    CHECKX("nbcd 01", T->cpu.d[6] & 0xFF, 0x99);
    CHECK("nbcd borrow C", (ccr(3) & 0x11) == 0x11);

    t_begin(a);
    a_moveq(a, 0, 0); a_bit_i(a, 3, 5, EA_D(0)); snap(a, 0);
    a_bit_i(a, 0, 5, EA_D(0)); snap(a, 1);
    a_bit_i(a, 1, 31, EA_D(0)); a_bit_i(a, 2, 5, EA_D(0));
    a_bit_i(a, 3, 7, EA_AW(0x7500));
    a_moveq(a, 3, 1); a_moveq(a, 0, 2); a_bit_r(a, 3, 1, EA_D(2));
    a_move(a, 4, EA_IMM(4, 0x12345678u), EA_D(3)); a_swap(a, 3);
    a_move(a, 4, EA_IMM(4, 0xF0), EA_D(4)); a_extw(a, 4); a_move(a, 4, EA_D(4), EA_D(5)); a_extl(a, 5);
    a_tas(a, EA_AW(0x7510)); snap(a, 2);
    a_moveq(a, 0, 6); a_unary(a, A_TST, 4, EA_D(6)); a_scc(a, CC_EQ, EA_D(6));
    a_lea(a, EA_AW(0x7600), 0);
    a_move(a, 4, EA_IMM(4, 0x11223344u), EA_D(7)); a_movep(a, 7, 7, 0, 0);
    a_movep(a, 4, 1, 0, 0);
    a_exg_dd(a, 3, 4);
    a_stop(a, 0x2700);
    t_go(a, 100000);
    CHECK("bset Z(was 0)", (ccr(0) & 4) != 0);
    CHECK("btst Z clear", (ccr(1) & 4) == 0);
    CHECKX("bchg/bclr", T->cpu.d[0], 0x80000000u);
    CHECKX("bset mem", tr8(T, 0x7500), 0x80);
    CHECKX("bset dyn", T->cpu.d[2], 0x08);
    CHECKX("swap (then exg)", T->cpu.d[4], 0x56781234u);
    CHECKX("ext.w (then exg)", T->cpu.d[3], 0x0000FFF0u);
    CHECKX("ext.l", T->cpu.d[5], 0xFFFFFFF0u);
    CHECKX("tas mem", tr8(T, 0x7510), 0x80);
    CHECK("tas Z", (ccr(2) & 4) != 0);
    CHECKX("seq", T->cpu.d[6] & 0xFF, 0xFF);
    CHECKX("movep.l bytes", (uint32_t)tr8(T, 0x7600) << 24 | (uint32_t)tr8(T, 0x7602) << 16 |
           (uint32_t)tr8(T, 0x7604) << 8 | tr8(T, 0x7606), 0x11223344u);
    CHECKX("movep.w read", T->cpu.d[1] & 0xFFFF, 0x1122);

    printf("[m68k] TRAP / CHK / privilege / USP\n");
    t_begin(a);
    m_wl(32u * 4u + 12u, 0x0380);          /* TRAP #3 */
    m_wl(6u * 4u, 0x03A0);                 /* CHK */
    m_wl(8u * 4u, 0x03C0);                 /* privilege */
    a_org(a, 0x380); a_moveq(a, 77, 0); a_move(a, 2, EA_IND(7), EA_AW(0x7700)); a_rte(a);
    a_org(a, 0x3A0); a_addq(a, 2, 1, EA_AW(0x7702)); a_rte(a);
    a_org(a, 0x3C0); a_addq(a, 2, 1, EA_AW(0x7704)); a_addq(a, 4, 4, EA_DISP(7, 2)); a_rte(a);
    a_org(a, TPROG);
    a_move_to_ccr(a, EA_IMM(2, 0x03));
    a_trap(a, 3);
    a_moveq(a, 50, 7); a_chk(a, EA_IMM(2, 10), 7);
    a_moveq(a, -1, 7); a_chk(a, EA_IMM(2, 10), 7);
    a_moveq(a, 5, 7); a_chk(a, EA_IMM(2, 10), 7);
    a_movea(a, 4, EA_IMM(4, 0x6000), 1); a_move_usp(a, 1, 1);
    a_andi_sr(a, 0xDFFF);                      /* -> user mode */
    a_move(a, 4, EA_A(7), EA_AW(0x7710));
    a_move_to_sr(a, EA_IMM(2, 0x2700));        /* privileged: traps, skipped */
    a_move_from_sr(a, EA_AW(0x7714));
    a_label(a, 9); a_bra(a, 9);
    t_go(a, 20000);
    CHECKX("trap handler ran", T->cpu.d[0], 77);
    CHECKX("trap stacked SR", tr16(T, 0x7700) & 0xFF1F, 0x2703);
    CHECKX("chk traps x2", tr16(T, 0x7702), 2);
    CHECKX("privilege trap", tr16(T, 0x7704), 1);
    CHECKX("user sp = usp", m_l(0x7710), 0x6000);
    CHECKX("still user mode", tr16(T, 0x7714) & 0x2000, 0);

    printf("[m68k] Level 6 V-Int / Level 4 H-Int exception frames + RTE\n");
    t_begin(a);
    m_wl(30u * 4u, 0x0380);                     /* level 6 autovector */
    m_wl(28u * 4u, 0x03C0);                     /* level 4 autovector */
    a_org(a, 0x380);
    a_move(a, 2, EA_IND(7), EA_AW(0x7800)); a_move(a, 4, EA_DISP(7, 2), EA_AW(0x7802));
    a_move_from_sr(a, EA_AW(0x7806)); a_addq(a, 2, 1, EA_AW(0x7810)); a_rte(a);
    a_org(a, 0x3C0);
    a_move(a, 2, EA_IND(7), EA_AW(0x7820)); a_move(a, 4, EA_DISP(7, 2), EA_AW(0x7822));
    a_move_from_sr(a, EA_AW(0x7826)); a_addq(a, 2, 1, EA_AW(0x7830)); a_rte(a);
    a_org(a, TPROG);
    a_move_to_sr(a, EA_IMM(2, 0x2000));
    a_label(a, 1); a_unary(a, A_TST, 2, EA_AW(0x7810)); a_bcc(a, CC_EQ, 1);
    a_move_to_sr(a, EA_IMM(2, 0x2700));
    a_label(a, 2); a_unary(a, A_TST, 2, EA_AW(0x7840)); a_bcc(a, CC_EQ, 2);
    a_move_to_sr(a, EA_IMM(2, 0x2300));
    a_label(a, 3); a_unary(a, A_TST, 2, EA_AW(0x7830)); a_bcc(a, CC_EQ, 3);
    a_move_from_sr(a, EA_AW(0x7850));
    a_stop(a, 0x2700);
    a_finish(a);
    m68k_reset(&T->cpu);
    m68k_run(&T->cpu, 400);
    T->cpu.irq_level = 6;
    m68k_run(&T->cpu, 2000);
    CHECKX("vint taken once", tr16(T, 0x7810), 1);
    CHECKX("vint stacked SR", tr16(T, 0x7800) & 0xFF00, 0x2000);
    CHECK("vint stacked PC in poll loop", m_l(0x7802) >= TPROG && m_l(0x7802) < TPROG + 0x20);
    CHECKX("vint handler mask=6 S=1", tr16(T, 0x7806) & 0xFF00, 0x2600);
    T->cpu.irq_level = 4;
    m68k_run(&T->cpu, 2000);
    CHECKX("hint masked at level 7", tr16(T, 0x7830), 0);
    tw16(T, 0x7840, 1);
    {
        uint32_t used = 0;
        while (!T->cpu.stopped && used < 20000) used += m68k_run(&T->cpu, 256);
    }
    CHECKX("hint taken after mask=3", tr16(T, 0x7830), 1);
    CHECKX("hint stacked SR", tr16(T, 0x7820) & 0xFF00, 0x2300);
    CHECKX("hint handler mask=4", tr16(T, 0x7826) & 0xFF00, 0x2400);
    CHECKX("RTE restored SR", tr16(T, 0x7850) & 0xFF00, 0x2300);
    CHECK("cpu stopped cleanly", T->cpu.stopped == 1);

    {
        uint32_t op, ill = 0;
        for (op = 0; op < 65536; op++) if (m68k_op_class(&T->cpu, (uint16_t)op) == 0) ill++;
        printf("  dispatch table: %u/65536 opcodes decode to valid 68000 instructions\n", 65536u - ill);
        CHECK("dispatch table coverage plausible", (65536u - ill) > 40000u && (65536u - ill) < 60000u);
    }
}

/* ======================================================================
 * 2. Genesis bus: VDP DMA / VSRAM / pads
 * ==================================================================== */
static hs_gen_t *G;
static uint8_t *ROMBUF;
#define ROM_SIZE 0x20000u

static void put32(uint8_t *r, uint32_t at, uint32_t v)
{ r[at] = (uint8_t)(v >> 24); r[at + 1] = (uint8_t)(v >> 16); r[at + 2] = (uint8_t)(v >> 8); r[at + 3] = (uint8_t)v; }
static void putstr(uint8_t *r, uint32_t at, const char *s, uint32_t len)
{ uint32_t i, n = (uint32_t)strlen(s); for (i = 0; i < len; i++) r[at + i] = (uint8_t)(i < n ? s[i] : ' '); }

static void write_header(uint8_t *r, uint32_t size)
{
    uint32_t i, sum = 0;
    putstr(r, 0x100, "SEGA MEGA DRIVE ", 16);
    putstr(r, 0x110, "(C)HSOS 2026.NOV", 16);
    putstr(r, 0x120, "HANDSHAKEOS HS-GENESIS SYNTHETIC TEST", 48);
    putstr(r, 0x150, "HANDSHAKEOS HS-GENESIS SYNTHETIC TEST", 48);
    putstr(r, 0x180, "GM HSGEN001-00", 14);
    putstr(r, 0x190, "J6", 16);
    put32(r, 0x1A0, 0); put32(r, 0x1A4, size - 1);
    put32(r, 0x1A8, 0xFF0000); put32(r, 0x1AC, 0xFFFFFF);
    putstr(r, 0x1B0, "", 12);
    putstr(r, 0x1BC, "", 12);
    putstr(r, 0x1C8, "CLEAN-ROOM MIT TEST CART", 40);
    putstr(r, 0x1F0, "JUE", 16);
    for (i = 0x200; i < size; i += 2) sum += (uint32_t)((r[i] << 8) | r[i + 1]);
    r[0x18E] = (uint8_t)(sum >> 8); r[0x18F] = (uint8_t)sum;
}

#define VRAM_W(ad)  (0x40000000u | (((uint32_t)(ad) & 0x3FFFu) << 16) | (((uint32_t)(ad) >> 14) & 3u))
#define VRAM_DMA(ad) (VRAM_W(ad) | 0x80u)

static void core_boot(uint32_t flags)
{
    hs_core_config_t cfg = { HS_SYS_GENESIS, 0, flags };
    hs_genesis_core_ops.init(G, sizeof(*G), &cfg);
    hs_genesis_core_ops.load_rom(G, ROMBUF, ROM_SIZE);
}

static void dma_regs(asm68_t *a, uint32_t src, uint32_t words)
{
    a_move(a, 2, EA_IMM(2, 0x9300 | (words & 0xFF)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9400 | ((words >> 8) & 0xFF)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9500 | ((src >> 1) & 0xFF)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9600 | ((src >> 9) & 0xFF)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9700 | ((src >> 17) & 0x7F)), EA_IND(4));
}

static void test_vdp_bus(void)
{
    asm68_t A, *a = &A;
    uint32_t i, ok;
    printf("[vdp] 68k->VRAM/CRAM DMA, VRAM fill, VRAM copy, VSRAM, status\n");
    memset(ROMBUF, 0, ROM_SIZE);
    a_init(a, ROMBUF, ROM_SIZE, 0x200);
    put32(ROMBUF, 0, 0x00FFFE00); put32(ROMBUF, 4, 0x200);
    for (i = 2; i < 64; i++) put32(ROMBUF, i * 4, 0x1F0);
    ROMBUF[0x1F0] = 0x4E; ROMBUF[0x1F1] = 0x73;
    for (i = 0; i < 128; i++) { ROMBUF[0x1000 + i] = (uint8_t)(i * 7 + 3); }
    for (i = 0; i < 16; i++) { ROMBUF[0x1100 + 2 * i] = (uint8_t)(i & 0x0E); ROMBUF[0x1101 + 2 * i] = (uint8_t)((i * 0x22) & 0xEE); }
    a_lea(a, EA_AL(0xC00004), 4); a_lea(a, EA_AL(0xC00000), 5);
    a_move(a, 2, EA_IMM(2, 0x8114), EA_IND(4));            /* DMA enable, display off */
    a_move(a, 2, EA_IMM(2, 0x8F02), EA_IND(4));
    dma_regs(a, 0x1000, 64);
    a_move(a, 4, EA_IMM(4, VRAM_DMA(0x2000)), EA_IND(4));
    dma_regs(a, 0x1100, 16);
    a_move(a, 4, EA_IMM(4, 0xC0000080u), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x8F01), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9300), EA_IND(4)); a_move(a, 2, EA_IMM(2, 0x9401), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9780), EA_IND(4));
    a_move(a, 4, EA_IMM(4, VRAM_DMA(0x4000)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0xABAB), EA_IND(5));
    a_move(a, 2, EA_IMM(2, 0x9380), EA_IND(4)); a_move(a, 2, EA_IMM(2, 0x9400), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9500), EA_IND(4)); a_move(a, 2, EA_IMM(2, 0x9620), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x97C0), EA_IND(4));
    a_move(a, 4, EA_IMM(4, 0x000000C1u | ((0x6000u & 0x3FFFu) << 16) | 0x0u), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x8F02), EA_IND(4));
    a_move(a, 4, EA_IMM(4, 0x40000010u), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x0123), EA_IND(5)); a_move(a, 2, EA_IMM(2, 0x0456), EA_IND(5));
    a_move(a, 2, EA_IND(4), EA_AL(0xFF0100));
    a_move(a, 4, EA_IMM(4, 0x20000000u), EA_IND(4));       /* VRAM read 0x2000 */
    a_move(a, 2, EA_IND(5), EA_AL(0xFF0102));
    a_stop(a, 0x2700);
    a_finish(a);
    write_header(ROMBUF, ROM_SIZE);
    core_boot(0);
    hs_genesis_core_ops.step_cpu_phase(G, 0, 0);
    ok = 1;
    for (i = 0; i < 128; i++) if (G->cpu.vdp.vram[0x2000 + i] != (uint8_t)(i * 7 + 3)) ok = 0;
    CHECK("68k->VRAM DMA 64 words", ok);
    ok = 1;
    for (i = 0; i < 16; i++) {
        uint16_t w = (uint16_t)((ROMBUF[0x1100 + 2 * i] << 8) | ROMBUF[0x1101 + 2 * i]);
        if (G->cpu.vdp.cram[i] != (w & 0x0EEE)) ok = 0;
    }
    CHECK("68k->CRAM DMA 16 words", ok);
    ok = 1;
    for (i = 0; i < 0x100; i++) if (G->cpu.vdp.vram[0x4000 + i] != 0xAB) ok = 0;
    CHECK("VRAM fill 256 bytes", ok);
    CHECKX("VRAM fill stops", G->cpu.vdp.vram[0x4100], 0);
    ok = 1;
    for (i = 0; i < 0x80; i++) if (G->cpu.vdp.vram[0x6000 + i] != G->cpu.vdp.vram[0x2000 + i]) ok = 0;
    CHECK("VRAM copy 128 bytes", ok);
    CHECKX("VSRAM[0]", G->cpu.vdp.vsram[0], 0x0123);
    CHECKX("VSRAM[1]", G->cpu.vdp.vsram[1], 0x0456);
    CHECK("status FIFO-empty + vblank(display off)", (((G->cpu.ram[0x100] << 8) | G->cpu.ram[0x101]) & 0x0208) == 0x0208);
    CHECKX("VRAM data-port read", (G->cpu.ram[0x102] << 8) | G->cpu.ram[0x103], (3u << 8) | 10u);
    CHECKX("DMA words total", G->cpu.vdp.dma_words, 64u + 16u + 256u + 128u);

    printf("[io] 3-button + 6-button pad protocol, version register\n");
    {
        gen_cpu_domain_t *d = &G->cpu;
        uint8_t r1, r0, x;
        int k;
        d->pad = HS_BTN_UP | HS_GEN_A | HS_BTN_C | HS_BTN_START | HS_GEN_X | HS_BTN_MODE;
        gen_bus_write8(d, 0xA10009, 0x40);
        gen_bus_write8(d, 0xA10003, 0x40); r1 = gen_bus_read8(d, 0xA10003);
        gen_bus_write8(d, 0xA10003, 0x00); r0 = gen_bus_read8(d, 0xA10003);
        CHECKX("pad TH=1 ?1CBRLDU (C,Up pressed)", r1 & 0x7F, 0x40 | 0x1E);
        CHECKX("pad TH=0 ?0SA00DU (S,A,Up pressed)", r0 & 0x7F, 0x02);
        for (k = 0; k < 2; k++) { gen_bus_write8(d, 0xA10003, 0x40); gen_bus_write8(d, 0xA10003, 0x00); }
        x = gen_bus_read8(d, 0xA10003);
        CHECKX("6btn id phase: low nibble 0", x & 0x0F, 0x00);
        gen_bus_write8(d, 0xA10003, 0x40); x = gen_bus_read8(d, 0xA10003);
        CHECKX("6btn XYZM phase (Mode,X pressed)", x & 0x7F, 0x40 | 0x10 | 0x03);
        CHECKX("version reg overseas NTSC", gen_bus_read8(d, 0xA10001), 0xA0);
    }
}

/* ======================================================================
 * 3. Z80, YM2612, SN76489 unit tests
 * ==================================================================== */
typedef struct zctx { gen_z80_t z; uint8_t mem[65536]; } zctx_t;
static zctx_t *Z;
static uint8_t zr(void *c, uint16_t a) { return ((zctx_t *)c)->mem[a]; }
static void zw(void *c, uint16_t a, uint8_t v) { ((zctx_t *)c)->mem[a] = v; }
static uint8_t zi(void *c, uint16_t p) { (void)c; return (uint8_t)p; }
static void zo(void *c, uint16_t p, uint8_t v) { ((zctx_t *)c)->mem[0xFF00 | (p & 0xFF)] = v; }

static void z_load(const uint8_t *prog, uint32_t n)
{
    z80_bus_t b = { zr, zw, zi, zo, NULL };
    b.ctx = Z;
    memset(Z->mem, 0, sizeof(Z->mem));
    memcpy(Z->mem, prog, n);
    z80_init(&Z->z, &b);
    Z->z.sp = 0xF000;
}
static void z_run(uint64_t cyc) { z80_run_until(&Z->z, Z->z.cyc + cyc); }

static void test_z80(void)
{
    printf("[z80] ALU/DAA, DJNZ, LDIR, IX/IY, CB, ED 16-bit, EX/EXX, IM1 interrupt\n");
    {
        static const uint8_t p[] = {
            0x3E, 0x15, 0xC6, 0x27, 0x27,             /* LD A,15; ADD A,27; DAA -> 42 */
            0x32, 0x00, 0x90,                         /* LD (9000),A */
            0x06, 0x0A, 0xAF,                         /* LD B,10; XOR A */
            0xC6, 0x03, 0x10, 0xFC,                   /* L: ADD A,3; DJNZ L */
            0x32, 0x01, 0x90,
            0x21, 0x00, 0x80, 0x11, 0x00, 0x81, 0x01, 0x05, 0x00, 0xED, 0xB0, /* LDIR 5 */
            0xDD, 0x21, 0x00, 0x82, 0xDD, 0x36, 0x05, 0x77, 0xDD, 0x34, 0x05, /* (IX+5)=77; INC */
            0xDD, 0x7E, 0x05, 0x32, 0x02, 0x90,       /* LD A,(IX+5) -> (9002) */
            0xFD, 0x21, 0x10, 0x82, 0xFD, 0x35, 0xFE, /* LD IY,8210; DEC (IY-2) */
            0x3E, 0x81, 0xCB, 0x07, 0x32, 0x03, 0x90, /* LD A,81; RLC A -> 03 */
            0xF5, 0xE1, 0x7D, 0x32, 0x04, 0x90,       /* PUSH AF; POP HL; LD A,L -> flags */
            0x21, 0x00, 0x83, 0xCB, 0xFE, 0xCB, 0x7E, /* SET 7,(HL); BIT 7,(HL) */
            0xF5, 0xE1, 0x7D, 0x32, 0x05, 0x90,
            0x21, 0x00, 0x10, 0x11, 0x01, 0x00, 0x37, 0xED, 0x52, /* HL=1000; DE=1; SCF; SBC HL,DE */
            0x22, 0x06, 0x90,                         /* LD (9006),HL -> 0FFE */
            0x01, 0x34, 0x12, 0xD9, 0x01, 0x78, 0x56, 0xD9, /* BC=1234; EXX; BC=5678; EXX */
            0xED, 0x43, 0x08, 0x90,                   /* LD (9008),BC -> 1234 */
            0x3E, 0x05, 0xED, 0x44, 0x32, 0x0A, 0x90, /* NEG 5 -> FB */
            0xCD, 0x00, 0x04, 0x32, 0x0B, 0x90,       /* CALL 0400 */
            0xDD, 0xCB, 0x05, 0xC6,                   /* SET 0,(IX+5) */
            0x76                                      /* HALT */
        };
        uint8_t sub[] = { 0x3E, 0x99, 0xC9 };
        uint8_t i;
        z_load(p, sizeof(p));
        memcpy(Z->mem + 0x400, sub, sizeof(sub));
        for (i = 0; i < 5; i++) Z->mem[0x8000 + i] = (uint8_t)(0xA0 + i);
        Z->mem[0x820E] = 0x10;
        z_run(4000);
        CHECKX("z80 DAA 15+27", Z->mem[0x9000], 0x42);
        CHECKX("z80 DJNZ sum", Z->mem[0x9001], 30);
        CHECK("z80 LDIR copy", memcmp(Z->mem + 0x8100, Z->mem + 0x8000, 5) == 0 && Z->mem[0x8105] == 0);
        CHECKX("z80 (IX+d) inc", Z->mem[0x9002], 0x78);
        CHECKX("z80 (IY-d) dec", Z->mem[0x820E], 0x0F);
        CHECKX("z80 RLC A", Z->mem[0x9003], 0x03);
        CHECK("z80 RLC carry", (Z->mem[0x9004] & 1) == 1);
        CHECKX("z80 SET 7,(HL)", Z->mem[0x8300], 0x80);
        CHECK("z80 BIT 7 -> NZ", (Z->mem[0x9005] & 0x40) == 0);
        CHECKX("z80 SBC HL,DE", Z->mem[0x9006] | (Z->mem[0x9007] << 8), 0x0FFE);
        CHECKX("z80 EXX", Z->mem[0x9008] | (Z->mem[0x9009] << 8), 0x1234);
        CHECKX("z80 NEG", Z->mem[0x900A], 0xFB);
        CHECKX("z80 CALL/RET", Z->mem[0x900B], 0x99);
        CHECKX("z80 DDCB SET", Z->mem[0x8205], 0x79);
        CHECK("z80 halted", Z->z.halted == 1);
    }
    {
        static const uint8_t p[] = {
            0x31, 0x00, 0xF0, 0xED, 0x56, 0xFB,       /* LD SP; IM 1; EI */
            0x76, 0x18, 0xFD                          /* HALT; JR -3 */
        };
        static const uint8_t isr[] = { 0xF5, 0x3A, 0x00, 0x90, 0x3C, 0x32, 0x00, 0x90, 0xF1, 0xFB, 0xED, 0x4D };
        z_load(p, sizeof(p));
        memcpy(Z->mem + 0x38, isr, sizeof(isr));
        z_run(200);
        Z->z.irq_line = 1; z_run(60); Z->z.irq_line = 0; z_run(200);
        Z->z.irq_line = 1; z_run(60); Z->z.irq_line = 0; z_run(200);
        CHECKX("z80 IM1 interrupts serviced", Z->mem[0x9000], 2);
        CHECK("z80 back in HALT", Z->z.halted == 1);
    }
}

static void test_ym_psg(void)
{
    gen_ym_t ym;
    gen_psg_t psg;
    int32_t l, r, peak = 0, prev = 0;
    uint32_t i, cross = 0;
    printf("[ym2612] FM pitch, envelope, timers, DAC\n");
    ym_reset(&ym);
    ym_write(&ym, 0, 0xB0); ym_write(&ym, 1, 0x07);        /* alg 7 */
    ym_write(&ym, 0, 0xB4); ym_write(&ym, 1, 0xC0);
    ym_write(&ym, 0, 0x30); ym_write(&ym, 1, 0x01);        /* OP1 mul 1 */
    ym_write(&ym, 0, 0x40); ym_write(&ym, 1, 0x00);        /* OP1 TL 0 */
    ym_write(&ym, 0, 0x44); ym_write(&ym, 1, 0x7F);
    ym_write(&ym, 0, 0x48); ym_write(&ym, 1, 0x7F);
    ym_write(&ym, 0, 0x4C); ym_write(&ym, 1, 0x7F);
    ym_write(&ym, 0, 0x50); ym_write(&ym, 1, 0x1F);        /* AR 31 */
    ym_write(&ym, 0, 0x80); ym_write(&ym, 1, 0x0F);        /* SL 0, RR 15 */
    ym_write(&ym, 0, 0xA4); ym_write(&ym, 1, 0x22);        /* block 4, fnum 644 */
    ym_write(&ym, 0, 0xA0); ym_write(&ym, 1, 0x84);
    ym_write(&ym, 0, 0x28); ym_write(&ym, 1, 0x10);        /* key on OP1 ch1 */
    for (i = 0; i < 53267; i++) {
        ym_clock(&ym, &l, &r);
        if (l > peak) peak = l;
        if (prev < 0 && l >= 0) cross++;
        prev = l;
    }
    printf("  C4 test tone: %u Hz (expect ~262), peak %d\n", cross, peak);
    CHECK("ym pitch C4 ~262 Hz", cross >= 259 && cross <= 265);
    CHECK("ym full-scale peak", peak > 7000);
    CHECK("ym L==R panning", l == r);
    ym_write(&ym, 0, 0x28); ym_write(&ym, 1, 0x00);        /* key off */
    for (i = 0; i < 20000; i++) ym_clock(&ym, &l, &r);
    CHECK("ym release to silence", l > -64 && l < 64);
    ym_write(&ym, 0, 0x24); ym_write(&ym, 1, 0xFF);
    ym_write(&ym, 0, 0x25); ym_write(&ym, 1, 0x03);        /* TA = 1023 */
    ym_write(&ym, 0, 0x27); ym_write(&ym, 1, 0x05);
    ym_clock(&ym, &l, &r); ym_clock(&ym, &l, &r);
    CHECKX("ym timer A overflow flag", ym_read_status(&ym) & 1, 1);
    ym_write(&ym, 0, 0x27); ym_write(&ym, 1, 0x10);
    CHECKX("ym timer A flag reset", ym_read_status(&ym) & 1, 0);
    ym_write(&ym, 0, 0x2B); ym_write(&ym, 1, 0x80);
    ym_write(&ym, 0, 0x2A); ym_write(&ym, 1, 0xFF);
    ym_write(&ym, 2, 0xB6); ym_write(&ym, 3, 0x80);        /* ch6 L only */
    ym_clock(&ym, &l, &r);
    CHECKX("ym ch6 DAC sample", (uint32_t)ym.ch[5].out, (uint32_t)(127 << 6));
    CHECK("ym ch6 DAC panned L only", l > r);

    printf("[sn76489] tone period, volume, periodic + white noise LFSR\n");
    psg_reset(&psg);
    psg_write(&psg, 0x80 | 0x0); psg_write(&psg, 0x10);    /* ch0 period 0x100 */
    psg_write(&psg, 0x90);                                  /* ch0 vol 0 (max) */
    prev = 0; cross = 0; peak = 0;
    for (i = 0; i < 223722; i++) {
        int32_t s = psg_tick(&psg);
        if ((s > 0) != (prev > 0)) cross++;
        prev = s;
        if (s > peak) peak = s;
    }
    printf("  tone toggles/s: %u (expect ~874), peak %d\n", cross, peak);
    CHECK("psg tone frequency", cross >= 870 && cross <= 878);
    CHECKX("psg max volume", peak, 2000);
    psg_write(&psg, 0x9F);
    psg_write(&psg, 0xE0);                                  /* periodic noise, rate 0 */
    psg_write(&psg, 0xF0);
    {
        uint32_t ones = 0, shifts = 0;
        uint16_t last = psg.lfsr;
        for (i = 0; i < 16 * 32 * 2; i++) {
            psg_tick(&psg);
            if (psg.lfsr != last) { shifts++; if (psg.lfsr & 1) ones++; last = psg.lfsr; }
        }
        CHECK("psg periodic noise: 1 in 16 outputs", shifts > 0 && ones * 16 == shifts);
    }
    psg_write(&psg, 0xE4);
    {
        uint32_t hi = 0, n = 0;
        for (i = 0; i < 200000; i++) { psg_tick(&psg); n++; if (psg.lfsr & 1) hi++; }
        CHECK("psg white noise balanced", hi > n / 4 && hi < (n * 3) / 4);
    }
}

/* ======================================================================
 * 4. Synthetic Genesis ROM
 * ==================================================================== */
enum { L_START = 1, L_W1, L_Z1, L_R1, L_PBY, L_PBX, L_PAY, L_PAX, L_PAZ, L_PAW, L_WN, L_SP,
       L_MAIN, L_VINT, L_HINT, L_DFLT, L_HS, L_SPX, L_NOPSG, L_Z80, L_VDPREGS, L_WAVE, L_MELODY,
       L_SPRITES, L_TILES, L_PAL, L_Z80END };

#define D_TILES   0x8000u
#define D_PAL     0x9000u
#define D_WAVE    0x9100u
#define D_MELODY  0x9200u
#define D_SPRITES 0x9300u
#define D_VDPREGS 0x9400u
#define D_Z80     0xA000u

static uint32_t build_z80(uint8_t *o)
{
    static const uint8_t isr[] = {
        0xF5, 0xE5, 0xC5, 0xDD, 0xE5, 0x06, 0x28, 0x10, 0xFE,     /* push; LD B,40; DJNZ $ */
        0xDD, 0x21, 0x00, 0x1F, 0xDD, 0x34, 0x00, 0xDD, 0x7E, 0x00,
        0xE6, 0x3F, 0x20, 0x06, 0x21, 0x00, 0x03, 0xCD, 0x60, 0x01,
        0xDD, 0xE1, 0xC1, 0xE1, 0xF1, 0xFB, 0xED, 0x4D
    };
    static const uint8_t init[] = {
        0x21, 0x00, 0x02, 0x11, 0x00, 0x10, 0x01, 0x00, 0x01, 0xED, 0xB0,
        0x21, 0x00, 0x60,
        0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00, 0x36, 0x00,
        0x3A, 0x00, 0x81, 0x32, 0x10, 0x1F,
        0x21, 0x00, 0x10, 0xCD, 0x60, 0x01,
        0xFB, 0x76, 0x18, 0xFD
    };
    static const uint8_t wait_ym[] = { 0xF5, 0x3A, 0x00, 0x40, 0x07, 0x38, 0xFA, 0xF1, 0xC9 };
    static const uint8_t write_tab[] = {
        0x7E, 0xFE, 0xFF, 0xC8, 0xCD, 0x40, 0x01, 0x32, 0x00, 0x40, 0x23, 0x7E, 0x32, 0x01, 0x40, 0x23, 0x18, 0xEE
    };
    static const uint8_t keyseq[] = { 0x28, 0x00, 0x28, 0x01, 0x28, 0x02, 0x28, 0xF0, 0x28, 0xF1, 0x28, 0xF2, 0xFF };
    static const uint16_t fnum[3] = { 644, 811, 965 };
    uint32_t n = 0, ch, s;
    memset(o, 0, 0x320);
    o[0] = 0xF3; o[1] = 0x31; o[2] = 0x00; o[3] = 0x20; o[4] = 0xED; o[5] = 0x56; o[6] = 0xC3; o[7] = 0x00; o[8] = 0x01;
    memcpy(o + 0x38, isr, sizeof(isr));
    memcpy(o + 0x100, init, sizeof(init));
    memcpy(o + 0x140, wait_ym, sizeof(wait_ym));
    memcpy(o + 0x160, write_tab, sizeof(write_tab));
    memcpy(o + 0x300, keyseq, sizeof(keyseq));
    n = 0x200;
#define YM(r, v) do { o[n++] = (uint8_t)(r); o[n++] = (uint8_t)(v); } while (0)
    YM(0x22, 0x00); YM(0x27, 0x00); YM(0x2B, 0x00);
    for (ch = 0; ch < 3; ch++) {
        YM(0xB0 + ch, 0x34); YM(0xB4 + ch, 0xC0);
        for (s = 0; s < 4; s++) {
            uint32_t base = s * 4 + ch;
            YM(0x30 + base, s == 1 ? 0x02 : 0x01);
            YM(0x40 + base, s >= 2 ? 0x10 : 0x28);
            YM(0x50 + base, 0x1F);
            YM(0x60 + base, 0x08);
            YM(0x70 + base, 0x03);
            YM(0x80 + base, 0x37);
        }
        YM(0xA4 + ch, 0x20 | (fnum[ch] >> 8));
        YM(0xA0 + ch, fnum[ch] & 0xFF);
    }
    YM(0x28, 0xF0); YM(0x28, 0xF1); YM(0x28, 0xF2);
#undef YM
    o[n++] = 0xFF;
    return 0x320;
}

static void build_tiles(uint8_t *t)
{
    uint32_t k, x, y;
    memset(t, 0, 32 * 32);
    for (k = 1; k <= 32; k++) {
        uint8_t *tile = t + (k - 1) * 32;
        for (y = 0; y < 8; y++) for (x = 0; x < 8; x++) {
            uint32_t c = 0;
            if (k <= 8) c = ((x + y + k) & 7) < 5 ? 1 + ((x ^ y ^ k) & 7) : 0;
            else if (k <= 12) c = (x == 0) ? 0 : 8 + ((x * y + k) & 7);
            else if (k <= 16) c = (y == 0 || y == 7) ? 15 : (((x + k) & 1) ? 12 : 10);
            else if (k <= 20) {
                uint32_t q = k - 17, px = (q >> 1) * 8 + x, py = (q & 1) * 8 + y;
                int32_t dx = (int32_t)(2 * px) - 15, dy = (int32_t)(2 * py) - 15;
                uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
                c = d2 <= 225 ? 1 + (d2 / 40) : 0;
            } else if (k <= 29) {
                uint32_t q = k - 21, px = (q / 3) * 8 + x, py = (q % 3) * 8 + y;
                int32_t dx = (int32_t)px - 12, dy = (int32_t)py - 12;
                uint32_t md = (uint32_t)((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy));
                c = md <= 11 ? 4 + (md & 7) : 0;
            }
            tile[y * 4 + (x >> 1)] |= (uint8_t)((x & 1) ? c : (c << 4));
        }
    }
}

static void build_rom(uint8_t *r)
{
    asm68_t A, *a = &A;
    uint32_t i, zlen;
    static const uint16_t vdpregs[] = {
        0x8004, 0x8134, 0x8230, 0x832C, 0x8407, 0x857C, 0x8600, 0x8711,
        0x8800, 0x8900, 0x8A1F, 0x8B03, 0x8C81, 0x8D3F, 0x8E00, 0x8F02,
        0x9001, 0x9100, 0x9202
    };
    static const uint16_t melody[16] = { 214, 170, 143, 170, 160, 127, 143, 170, 190, 160, 170, 214, 143, 113, 107, 214 };
    memset(r, 0, ROM_SIZE);
    a_init(a, r, ROM_SIZE, 0x200);

    /* ---- reset / init ---- */
    a_label(a, L_START);
    a_move_to_sr(a, EA_IMM(2, 0x2700));
    a_move(a, 4, EA_IMM(4, 0x53454741u), EA_AL(0xA14000));
    a_lea(a, EA_AL(0xC00004), 4); a_lea(a, EA_AL(0xC00000), 5);
    a_unary(a, A_TST, 2, EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x0100), EA_AL(0xA11100));
    a_move(a, 2, EA_IMM(2, 0x0100), EA_AL(0xA11200));
    a_label(a, L_W1); a_bit_i(a, 0, 0, EA_AL(0xA11100)); a_bcc(a, CC_NE, L_W1);
    a_lea(a, EA_AL(D_Z80), 0); a_lea(a, EA_AL(0xA00000), 1);
    a_move(a, 2, EA_IMM(2, 0x320 - 1), EA_D(0));
    a_label(a, L_Z1); a_move(a, 1, EA_PI(0), EA_PI(1)); a_dbra(a, 0, L_Z1);
    a_move(a, 2, EA_IMM(2, 0), EA_AL(0xA11200));
    a_move(a, 2, EA_IMM(2, 0), EA_AL(0xA11100));
    a_move(a, 2, EA_IMM(2, 0x0100), EA_AL(0xA11200));
    a_lea(a, EA_AL(D_VDPREGS), 0);
    a_moveq(a, (int)(sizeof(vdpregs) / 2) - 1, 0);
    a_label(a, L_R1); a_move(a, 2, EA_PI(0), EA_IND(4)); a_dbra(a, 0, L_R1);
    /* clear all VRAM with a DMA fill */
    a_move(a, 2, EA_IMM(2, 0x8F01), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x93FF), EA_IND(4)); a_move(a, 2, EA_IMM(2, 0x94FF), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x9780), EA_IND(4));
    a_move(a, 4, EA_IMM(4, VRAM_DMA(0x0000)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x0000), EA_IND(5));
    a_move(a, 2, EA_IMM(2, 0x8F02), EA_IND(4));
    /* tiles + palettes by 68k DMA */
    dma_regs(a, D_TILES, 32u * 16u);
    a_move(a, 4, EA_IMM(4, VRAM_DMA(0x0020)), EA_IND(4));
    dma_regs(a, D_PAL, 64);
    a_move(a, 4, EA_IMM(4, 0xC0000080u), EA_IND(4));
    /* plane B 64x32 */
    a_move(a, 4, EA_IMM(4, VRAM_W(0xE000)), EA_IND(4));
    a_moveq(a, 0, 1);
    a_label(a, L_PBY); a_moveq(a, 0, 2);
    a_label(a, L_PBX);
    a_move(a, 2, EA_D(2), EA_D(3)); a_op_ead(a, A_ADD, 2, EA_D(1), 3);
    a_imm(a, A_ANDI, 2, 7, EA_D(3)); a_addq(a, 2, 1, EA_D(3));
    a_move(a, 2, EA_D(1), EA_D(4)); a_imm(a, A_ANDI, 2, 3, EA_D(4));
    a_moveq(a, 13, 5); a_shr(a, 1, 1, 2, 5, 4); a_op_ead(a, A_OR, 2, EA_D(4), 3);
    a_move(a, 2, EA_D(3), EA_IND(5));
    a_addq(a, 2, 1, EA_D(2)); a_imm(a, A_CMPI, 2, 64, EA_D(2)); a_bcc(a, CC_NE, L_PBX);
    a_addq(a, 2, 1, EA_D(1)); a_imm(a, A_CMPI, 2, 32, EA_D(1)); a_bcc(a, CC_NE, L_PBY);
    /* plane A diagonal band (high priority, palette 2, alternating H-flip) */
    a_move(a, 4, EA_IMM(4, VRAM_W(0xC000)), EA_IND(4));
    a_moveq(a, 0, 1);
    a_label(a, L_PAY); a_moveq(a, 0, 2);
    a_label(a, L_PAX);
    a_move(a, 2, EA_D(2), EA_D(3)); a_op_ead(a, A_SUB, 2, EA_D(1), 3);
    a_imm(a, A_ANDI, 2, 15, EA_D(3)); a_imm(a, A_CMPI, 2, 3, EA_D(3)); a_bcc(a, CC_CC, L_PAZ);
    a_move(a, 2, EA_D(2), EA_D(3)); a_shi(a, 1, 0, 2, 1, 3); a_imm(a, A_ANDI, 2, 3, EA_D(3));
    a_imm(a, A_ADDI, 2, 0x8000 | 0x4000 | 9, EA_D(3));
    a_bit_i(a, 0, 0, EA_D(2)); a_bcc(a, CC_EQ, L_PAW);
    a_imm(a, A_ORI, 2, 0x0800, EA_D(3)); a_bra(a, L_PAW);
    a_label(a, L_PAZ); a_moveq(a, 0, 3);
    a_label(a, L_PAW); a_move(a, 2, EA_D(3), EA_IND(5));
    a_addq(a, 2, 1, EA_D(2)); a_imm(a, A_CMPI, 2, 64, EA_D(2)); a_bcc(a, CC_NE, L_PAX);
    a_addq(a, 2, 1, EA_D(1)); a_imm(a, A_CMPI, 2, 32, EA_D(1)); a_bcc(a, CC_NE, L_PAY);
    /* window HUD rows 0-1 */
    a_move(a, 4, EA_IMM(4, VRAM_W(0xB000)), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 127), EA_D(0)); a_moveq(a, 0, 2);
    a_label(a, L_WN);
    a_move(a, 2, EA_D(2), EA_D(3)); a_imm(a, A_ANDI, 2, 3, EA_D(3));
    a_imm(a, A_ADDI, 2, 0x8000 | 0x6000 | 13, EA_D(3)); a_move(a, 2, EA_D(3), EA_IND(5));
    a_addq(a, 2, 1, EA_D(2)); a_dbra(a, 0, L_WN);
    /* sprite attribute table */
    a_lea(a, EA_AL(D_SPRITES), 0);
    a_move(a, 4, EA_IMM(4, VRAM_W(0xF800)), EA_IND(4));
    a_moveq(a, 31, 0);
    a_label(a, L_SP); a_move(a, 2, EA_PI(0), EA_IND(5)); a_dbra(a, 0, L_SP);
    /* PSG silence */
    a_move(a, 1, EA_IMM(1, 0x9F), EA_AL(0xC00011)); a_move(a, 1, EA_IMM(1, 0xBF), EA_AL(0xC00011));
    a_move(a, 1, EA_IMM(1, 0xDF), EA_AL(0xC00011)); a_move(a, 1, EA_IMM(1, 0xFF), EA_AL(0xC00011));
    /* pad port as TH output */
    a_move(a, 1, EA_IMM(1, 0x40), EA_AL(0xA10009));
    /* display + V-Int + DMA, H-Int */
    a_move(a, 2, EA_IMM(2, 0x8174), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0x8014), EA_IND(4));
    a_move_to_sr(a, EA_IMM(2, 0x2000));
    a_label(a, L_MAIN);
    a_move(a, 4, EA_AL(0xFF0010), EA_D(0)); a_addq(a, 4, 1, EA_D(0)); a_move(a, 4, EA_D(0), EA_AL(0xFF0010));
    a_move(a, 2, EA_D(0), EA_D(1)); a_mulu(a, EA_IMM(2, 1234), 1); a_divu(a, EA_IMM(2, 77), 1);
    a_op_dea(a, A_ADD, 4, 1, EA_AL(0xFF0014));
    a_bra(a, L_MAIN);

    /* ---- V-Int ---- */
    a_label(a, L_VINT);
    a_movem(a, 4, 1, 0xFFC0, EA_PD(7));
    a_addq(a, 2, 1, EA_AL(0xFF0000));
    a_unary(a, A_CLR, 2, EA_AL(0xFF0004));
    a_move(a, 2, EA_IMM(2, 0x8711), EA_IND(4));
    a_move(a, 4, EA_IMM(4, 0x40000010u), EA_IND(4));
    a_move(a, 2, EA_IMM(2, 0), EA_IND(5));
    a_move(a, 2, EA_AL(0xFF0000), EA_D(0)); a_shi(a, 1, 0, 2, 1, 0); a_move(a, 2, EA_D(0), EA_IND(5));
    a_move(a, 4, EA_IMM(4, VRAM_W(0xFC00)), EA_IND(4));
    a_move(a, 2, EA_AL(0xFF0000), EA_D(2));
    a_move(a, 2, EA_D(2), EA_D(3)); a_unary(a, A_NEG, 2, EA_D(3));
    a_lea(a, EA_AL(D_WAVE), 0);
    a_moveq(a, 0, 1); a_move(a, 2, EA_IMM(2, 223), EA_D(0));
    a_label(a, L_HS);
    a_move(a, 2, EA_D(1), EA_D(4)); a_op_ead(a, A_ADD, 2, EA_D(2), 4);
    a_imm(a, A_ANDI, 2, 63, EA_D(4)); a_op_ead(a, A_ADD, 2, EA_D(4), 4);
    a_move(a, 2, EA_IDX(0, 4, 0), EA_IND(5)); a_move(a, 2, EA_D(3), EA_IND(5));
    a_addq(a, 2, 1, EA_D(1)); a_dbra(a, 0, L_HS);
    a_moveq(a, 7, 0); a_moveq(a, 0, 1);
    a_label(a, L_SPX);
    a_move(a, 2, EA_D(1), EA_D(4)); a_shi(a, 1, 1, 2, 3, 4); a_imm(a, A_ADDI, 2, 0xF800 + 6, EA_D(4));
    a_move(a, 2, EA_D(4), EA_D(5)); a_imm(a, A_ANDI, 2, 0x3FFF, EA_D(5)); a_imm(a, A_ORI, 2, 0x4000, EA_D(5));
    a_move(a, 2, EA_D(5), EA_IND(4));
    a_move(a, 2, EA_D(4), EA_D(5)); a_moveq(a, 14, 6); a_shr(a, 1, 0, 2, 6, 5); a_move(a, 2, EA_D(5), EA_IND(4));
    a_move(a, 2, EA_D(1), EA_D(5)); a_addq(a, 2, 1, EA_D(5)); a_mulu(a, EA_D(2), 5); a_imm(a, A_ANDI, 2, 31, EA_D(5));
    a_move(a, 2, EA_D(1), EA_D(6)); a_mulu(a, EA_IMM(2, 36), 6); a_op_ead(a, A_ADD, 2, EA_D(6), 5);
    a_imm(a, A_ADDI, 2, 144, EA_D(5)); a_move(a, 2, EA_D(5), EA_IND(5));
    a_addq(a, 2, 1, EA_D(1)); a_dbra(a, 0, L_SPX);
    /* PSG melody every 8 frames */
    a_move(a, 2, EA_D(2), EA_D(0)); a_imm(a, A_ANDI, 2, 7, EA_D(0)); a_bcc(a, CC_NE, L_NOPSG);
    a_move(a, 2, EA_D(2), EA_D(0)); a_shi(a, 1, 0, 2, 3, 0); a_imm(a, A_ANDI, 2, 15, EA_D(0));
    a_op_ead(a, A_ADD, 2, EA_D(0), 0);
    a_lea(a, EA_AL(D_MELODY), 0); a_move(a, 2, EA_IDX(0, 0, 0), EA_D(1));
    a_move(a, 2, EA_D(1), EA_D(3)); a_imm(a, A_ANDI, 2, 0xF, EA_D(3)); a_imm(a, A_ORI, 2, 0x80, EA_D(3));
    a_move(a, 1, EA_D(3), EA_AL(0xC00011));
    a_move(a, 2, EA_D(1), EA_D(3)); a_shi(a, 1, 0, 2, 4, 3); a_imm(a, A_ANDI, 2, 0x3F, EA_D(3));
    a_move(a, 1, EA_D(3), EA_AL(0xC00011));
    a_move(a, 1, EA_IMM(1, 0x92), EA_AL(0xC00011));
    a_op_ead(a, A_ADD, 2, EA_D(1), 1);
    a_move(a, 2, EA_D(1), EA_D(3)); a_imm(a, A_ANDI, 2, 0xF, EA_D(3)); a_imm(a, A_ORI, 2, 0xA0, EA_D(3));
    a_move(a, 1, EA_D(3), EA_AL(0xC00011));
    a_move(a, 2, EA_D(1), EA_D(3)); a_shi(a, 1, 0, 2, 4, 3); a_imm(a, A_ANDI, 2, 0x3F, EA_D(3));
    a_move(a, 1, EA_D(3), EA_AL(0xC00011));
    a_move(a, 1, EA_IMM(1, 0xB4), EA_AL(0xC00011));
    a_move(a, 1, EA_IMM(1, 0xE5), EA_AL(0xC00011));
    a_label(a, L_NOPSG);
    a_move(a, 2, EA_D(2), EA_D(0)); a_imm(a, A_ANDI, 2, 7, EA_D(0)); a_addq(a, 2, 6, EA_D(0));
    a_imm(a, A_ORI, 2, 0xF0, EA_D(0)); a_move(a, 1, EA_D(0), EA_AL(0xC00011));
    /* poll the pad (TH high / low) */
    a_move(a, 1, EA_IMM(1, 0x40), EA_AL(0xA10003)); a_move(a, 1, EA_AL(0xA10003), EA_AL(0xFF0020));
    a_move(a, 1, EA_IMM(1, 0x00), EA_AL(0xA10003)); a_move(a, 1, EA_AL(0xA10003), EA_AL(0xFF0021));
    a_movem(a, 4, 0, 0x03FF, EA_PI(7));
    a_rte(a);

    /* ---- H-Int: raster backdrop colour cycling ---- */
    a_label(a, L_HINT);
    a_move(a, 4, EA_D(0), EA_PD(7));
    a_move(a, 2, EA_AL(0xFF0004), EA_D(0)); a_addq(a, 2, 1, EA_D(0)); a_move(a, 2, EA_D(0), EA_AL(0xFF0004));
    a_imm(a, A_ANDI, 2, 3, EA_D(0)); a_imm(a, A_ADDI, 2, 0x8711, EA_D(0));
    a_move(a, 2, EA_D(0), EA_AL(0xC00004));
    a_move(a, 4, EA_PI(7), EA_D(0));
    a_rte(a);

    a_label(a, L_DFLT);
    a_rte(a);
    a_finish(a);
    if (a->err || a->pc >= D_TILES) printf("  ROM assembler overflow!\n");

    /* vectors */
    put32(r, 0, 0x00FFFE00); put32(r, 4, a->label[L_START]);
    for (i = 2; i < 64; i++) put32(r, i * 4, a->label[L_DFLT]);
    put32(r, 28 * 4, a->label[L_HINT]);
    put32(r, 30 * 4, a->label[L_VINT]);

    /* data */
    build_tiles(r + D_TILES);
    for (i = 0; i < 64; i++) {
        uint32_t p = i >> 4, k = i & 15, rr = (k + p * 2) & 7, gg = (k * 3 + p) & 7, bb = (7 - k + p) & 7;
        uint16_t c;
        if (!rr && !gg && !bb) bb = 2;
        if (k == 1 && p == 1) { rr = 1; gg = 2; bb = 5; }
        c = (uint16_t)((bb << 9) | (gg << 5) | (rr << 1));
        r[D_PAL + 2 * i] = (uint8_t)(c >> 8); r[D_PAL + 2 * i + 1] = (uint8_t)c;
    }
    for (i = 0; i < 64; i++) {
        int32_t t = (int32_t)(i & 31) - 16, v = (i < 32) ? (16 - (t < 0 ? -t : t)) : -(16 - (t < 0 ? -t : t));
        uint16_t w = (uint16_t)(int16_t)(v * 3 / 4);
        r[D_WAVE + 2 * i] = (uint8_t)(w >> 8); r[D_WAVE + 2 * i + 1] = (uint8_t)w;
    }
    for (i = 0; i < 16; i++) { r[D_MELODY + 2 * i] = (uint8_t)(melody[i] >> 8); r[D_MELODY + 2 * i + 1] = (uint8_t)melody[i]; }
    for (i = 0; i < 8; i++) {
        uint8_t *e = r + D_SPRITES + i * 8;
        uint32_t y = 128 + 40 + i * 20, size = (i & 1) ? 0x0A : 0x05, link = i < 7 ? i + 1 : 0;
        uint32_t attr = ((i & 1) ? 0x8000u : 0u) | ((i & 3) << 13) | ((i & 1) ? 21u : 17u);
        uint32_t x = 128 + 20 + i * 36;
        e[0] = (uint8_t)(y >> 8); e[1] = (uint8_t)y; e[2] = (uint8_t)size; e[3] = (uint8_t)link;
        e[4] = (uint8_t)(attr >> 8); e[5] = (uint8_t)attr; e[6] = (uint8_t)(x >> 8); e[7] = (uint8_t)x;
    }
    for (i = 0; i < sizeof(vdpregs) / 2; i++) { r[D_VDPREGS + 2 * i] = (uint8_t)(vdpregs[i] >> 8); r[D_VDPREGS + 2 * i + 1] = (uint8_t)vdpregs[i]; }
    zlen = build_z80(r + D_Z80);
    (void)zlen;
    write_header(r, ROM_SIZE);
}

/* ---- PNG writer (stored deflate, no dependencies) ---- */
static uint32_t crc32_upd(uint32_t crc, const uint8_t *p, size_t n, const uint32_t *tab)
{
    size_t i;
    for (i = 0; i < n; i++) crc = tab[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}
static void be32w(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static int write_png(const char *path, const uint32_t *px, uint32_t w, uint32_t h)
{
    uint32_t tab[256], i, k, y;
    size_t raw_len = (size_t)(w * 3 + 1) * h, nblk = (raw_len + 65534) / 65535;
    size_t zlen = 2 + raw_len + nblk * 5 + 4, pos = 0;
    uint8_t *raw = (uint8_t *)malloc(raw_len), *z = (uint8_t *)malloc(zlen);
    uint8_t hdr[8] = { 137, 'P', 'N', 'G', 13, 10, 26, 10 }, b4[4], ihdr[17];
    uint32_t a1 = 1, a2 = 0, crc;
    FILE *f;
    if (!raw || !z) { free(raw); free(z); return -1; }
    for (i = 0; i < 256; i++) { uint32_t c = i; for (k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1; tab[i] = c; }
    for (y = 0; y < h; y++) {
        uint8_t *row = raw + (size_t)y * (w * 3 + 1);
        row[0] = 0;
        for (i = 0; i < w; i++) { uint32_t p = px[(size_t)y * w + i]; row[1 + 3 * i] = (uint8_t)(p >> 16); row[2 + 3 * i] = (uint8_t)(p >> 8); row[3 + 3 * i] = (uint8_t)p; }
    }
    z[pos++] = 0x78; z[pos++] = 0x01;
    for (i = 0; i < nblk; i++) {
        size_t off = (size_t)i * 65535, n = raw_len - off > 65535 ? 65535 : raw_len - off;
        z[pos++] = (uint8_t)(i + 1 == nblk ? 1 : 0);
        z[pos++] = (uint8_t)n; z[pos++] = (uint8_t)(n >> 8);
        z[pos++] = (uint8_t)~n; z[pos++] = (uint8_t)(~n >> 8);
        memcpy(z + pos, raw + off, n); pos += n;
    }
    for (i = 0; i < raw_len; i++) { a1 = (a1 + raw[i]) % 65521u; a2 = (a2 + a1) % 65521u; }
    be32w(z + pos, (a2 << 16) | a1); pos += 4;
    f = fopen(path, "wb");
    if (!f) { free(raw); free(z); return -1; }
    fwrite(hdr, 1, 8, f);
    memcpy(ihdr, "IHDR", 4); be32w(ihdr + 4, w); be32w(ihdr + 8, h);
    ihdr[12] = 8; ihdr[13] = 2; ihdr[14] = 0; ihdr[15] = 0; ihdr[16] = 0;
    be32w(b4, 13); fwrite(b4, 1, 4, f); fwrite(ihdr, 1, 17, f);
    crc = crc32_upd(0xFFFFFFFFu, ihdr, 17, tab) ^ 0xFFFFFFFFu; be32w(b4, crc); fwrite(b4, 1, 4, f);
    be32w(b4, (uint32_t)pos); fwrite(b4, 1, 4, f);
    fwrite("IDAT", 1, 4, f); fwrite(z, 1, pos, f);
    crc = crc32_upd(0xFFFFFFFFu, (const uint8_t *)"IDAT", 4, tab);
    crc = crc32_upd(crc, z, pos, tab) ^ 0xFFFFFFFFu; be32w(b4, crc); fwrite(b4, 1, 4, f);
    be32w(b4, 0); fwrite(b4, 1, 4, f); fwrite("IEND", 1, 4, f);
    crc = crc32_upd(0xFFFFFFFFu, (const uint8_t *)"IEND", 4, tab) ^ 0xFFFFFFFFu; be32w(b4, crc); fwrite(b4, 1, 4, f);
    fclose(f);
    free(raw); free(z);
    return 0;
}

static void test_rom(const char *png_path, uint32_t bench_frames)
{
    const hs_core_ops_t *ops = &hs_genesis_core_ops;
    uint32_t *fb = (uint32_t *)malloc(640u * 480u * 4u);
    int16_t *pcm = (int16_t *)malloc(2048u * 2u * 2u);
    uint64_t vh = HS_FNV64_OFFSET, ahash = HS_FNV64_OFFSET, sv = 0, sa = 0;
    uint64_t t0, t_cpu = 0, t_vid = 0, t_aud = 0, t_all0, t_all;
    uint32_t f, total_samples = 0, x, y, edge_bad = 0, nframes = 120, distinct = 0;
    uint32_t seen[64];
    if (!fb || !pcm) { printf("oom\n"); g_fail++; return; }
    printf("[rom] synthetic Genesis ROM: 120 frames through 3-phase ABI\n");
    build_rom(ROMBUF);
    CHECK("header magic", memcmp(ROMBUF + 0x100, "SEGA MEGA DRIVE ", 16) == 0);
    core_boot(0);
    CHECKX("reset vector", G->cpu.cpu.pc, 0x200);
    CHECKX("reset SSP", G->cpu.cpu.a[7], 0x00FFFE00u);
    t_all0 = hs_bench_now_ns();
    for (f = 0; f < nframes; f++) {
        uint32_t n;
        t0 = hs_bench_now_ns();
        ops->step_cpu_phase(G, (f & 16) ? HS_BTN_RIGHT : 0, 0);
        t_cpu += hs_bench_now_ns() - t0;
        ops->commit_phase(G);
        ops->hash_state(G, &sv, &sa);
        t0 = hs_bench_now_ns();
        ops->render_video_phase(G, fb, 640, HS_SCALE_STRETCH_SHARP);
        t_vid += hs_bench_now_ns() - t0;
        t0 = hs_bench_now_ns();
        n = ops->synth_audio_phase(G, pcm, 2048);
        t_aud += hs_bench_now_ns() - t0;
        total_samples += n;
        vh = hs_fnv1a64(vh, fb, 640u * 480u * 4u);
        ahash = hs_fnv1a64(ahash, pcm, (size_t)n * 4u);
        vh = hs_fnv1a64(vh, &sv, 8);
        ahash = hs_fnv1a64(ahash, &sa, 8);
    }
    t_all = hs_bench_now_ns() - t_all0;
    printf("  frames=%u 68k_insns=%llu z80_insns=%llu vint_count=%u z80_isr=%u main_loop=%u\n",
           nframes, (unsigned long long)G->cpu.cpu.insns, (unsigned long long)G->aud.z80.insns,
           (G->cpu.ram[0] << 8) | G->cpu.ram[1], G->aud.zram[0x1F00],
           ((uint32_t)G->cpu.ram[0x10] << 24) | ((uint32_t)G->cpu.ram[0x11] << 16) | ((uint32_t)G->cpu.ram[0x12] << 8) | G->cpu.ram[0x13]);
    CHECK("V-Int ran every frame after init", ((G->cpu.ram[0] << 8) | G->cpu.ram[1]) >= nframes - 2 && ((G->cpu.ram[0] << 8) | G->cpu.ram[1]) <= nframes);
    CHECK("68k main loop progressed", G->cpu.ram[0x12] | G->cpu.ram[0x11] | G->cpu.ram[0x13]);
    CHECK("Z80 V-blank IRQ serviced ~every frame", G->aud.zram[0x1F00] >= nframes - 2 && G->aud.zram[0x1F00] <= nframes);
    CHECKX("Z80 bank window read ROM 'S'", G->aud.zram[0x1F10], 'S');
    CHECK("Z80 copied YM table via LDIR", G->aud.zram[0x1000] == 0x22 && G->aud.zram[0x1001] == 0x00);
    CHECK("pad read via TH protocol", G->cpu.ram[0x20] != 0 || G->cpu.ram[0x21] != 0);
    CHECKX("no dropped audio events", G->slot[G->back ^ 1u].a.dropped, 0);
    printf("  audio: %u stereo frames @48kHz (expect ~%u), peak YM %d, peak PSG %d\n",
           total_samples, (uint32_t)((uint64_t)nframes * 48000ull * 896040ull / 53693175ull), G->aud.peak_ym, G->aud.peak_psg);
    CHECKX("48 kHz sample count exact-rate", total_samples, (uint32_t)((uint64_t)nframes * 48000ull * 896040ull / 53693175ull));
    CHECK("YM2612 FM chord audible", G->aud.peak_ym > 2000);
    CHECK("SN76489 melody audible", G->aud.peak_psg > 1000);
    {
        int32_t pl = 0, pr = 0;
        uint32_t i;
        for (i = 0; i < 400; i++) {
            int32_t L = pcm[2 * i], R = pcm[2 * i + 1];
            if ((L < 0 ? -L : L) > pl) pl = L < 0 ? -L : L;
            if ((R < 0 ? -R : R) > pr) pr = R < 0 ? -R : R;
        }
        CHECK("final PCM block non-silent stereo", pl > 500 && pr > 500);
    }
    for (x = 0; x < 640; x++) { if (!(fb[x] & 0xFFFFFF)) edge_bad++; if (!(fb[479u * 640u + x] & 0xFFFFFF)) edge_bad++; }
    for (y = 0; y < 480; y++) { if (!(fb[y * 640u] & 0xFFFFFF)) edge_bad++; if (!(fb[y * 640u + 639u] & 0xFFFFFF)) edge_bad++; }
    CHECKX("edge-to-edge 640x480: zero black edge pixels", edge_bad, 0);
    memset(seen, 0, sizeof(seen));
    for (y = 0; y < 480; y += 3) for (x = 0; x < 640; x += 3) {
        uint32_t p = fb[y * 640u + x] & 0xFFFFFF, k, hit = 0;
        for (k = 0; k < distinct; k++) if (seen[k] == p) { hit = 1; break; }
        if (!hit && distinct < 64) seen[distinct++] = p;
    }
    printf("  distinct colours sampled: %u\n", distinct);
    CHECK("frame has rich content (>=24 colours)", distinct >= 24);
    printf("VIDEO_HASH=%016llx\nAUDIO_HASH=%016llx\nSTATE_HASH=%016llx/%016llx\n",
           (unsigned long long)vh, (unsigned long long)ahash, (unsigned long long)sv, (unsigned long long)sa);
    if (png_path) {
        int rc = write_png(png_path, fb, 640, 480);
        CHECK("preview PNG written", rc == 0);
        if (!rc) printf("  preview: %s\n", png_path);
    }
    {
        uint64_t tc = 0, tv = 0, ta = 0, b0, b1;
        printf("[perf] 120-frame run: %.1f fps total (incl. hashing)\n", (double)nframes * 1e9 / (double)t_all);
        b0 = hs_bench_now_ns();
        for (f = 0; f < bench_frames; f++) {
            t0 = hs_bench_now_ns(); ops->step_cpu_phase(G, 0, 0); tc += hs_bench_now_ns() - t0;
            ops->commit_phase(G);
            t0 = hs_bench_now_ns(); ops->render_video_phase(G, fb, 640, HS_SCALE_STRETCH_SHARP); tv += hs_bench_now_ns() - t0;
            t0 = hs_bench_now_ns(); ops->synth_audio_phase(G, pcm, 2048); ta += hs_bench_now_ns() - t0;
        }
        b1 = hs_bench_now_ns();
        if (bench_frames) {
            double ms_c = (double)tc / bench_frames / 1e6, ms_v = (double)tv / bench_frames / 1e6, ms_a = (double)ta / bench_frames / 1e6;
            double mx = ms_c > ms_v ? (ms_c > ms_a ? ms_c : ms_a) : (ms_v > ms_a ? ms_v : ms_a);
            printf("[perf] %u frames serial: %.1f fps (%.3f ms/frame)\n", bench_frames,
                   (double)bench_frames * 1e9 / (double)(b1 - b0), (double)(b1 - b0) / bench_frames / 1e6);
            printf("[perf] Core0 68k+bus+VDP ports : %.3f ms/frame\n", ms_c);
            printf("[perf] Core1 VDP raster+640x480: %.3f ms/frame\n", ms_v);
            printf("[perf] Core2 Z80+YM2612+PSG    : %.3f ms/frame\n", ms_a);
            printf("[perf] 3-core pipelined bound   : %.1f fps (slowest phase %.3f ms)\n", 1000.0 / mx, mx);
        }
        (void)t_cpu; (void)t_vid; (void)t_aud;
    }
    free(fb); free(pcm);
}

int main(int argc, char **argv)
{
    const char *png = argc > 1 ? argv[1] : NULL;
    uint32_t bench = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 10) : 600u;
    T = (tctx_t *)calloc(1, sizeof(tctx_t));
    Z = (zctx_t *)calloc(1, sizeof(zctx_t));
    G = (hs_gen_t *)aligned_alloc(64, (sizeof(hs_gen_t) + 63) & ~(size_t)63);
    ROMBUF = (uint8_t *)calloc(1, ROM_SIZE);
    if (!T || !Z || !G || !ROMBUF) { printf("oom\n"); return 2; }
    printf("hs-genesis test: state %zu bytes\n", hs_genesis_core_ops.state_size());
    test_m68k();
    test_vdp_bus();
    test_z80();
    test_ym_psg();
    test_rom(png, bench);
    printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
    free(T); free(Z); free(G); free(ROMBUF);
    return g_fail ? 1 : 0;
}
