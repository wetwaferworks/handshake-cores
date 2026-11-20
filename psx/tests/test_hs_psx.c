/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * test_hs_psx.c - hs-psx unit + torture + demo suite.
 *   1. R3000A: ALU, branch delay, load delay, LWL/LWR/SWL/SWR, mult/div,
 *      exceptions (Ov, Sys via HLE, BD, IRQ, RI), RFE, cache isolation.
 *   2. Bus: DMA6 OTC, DMA2 block + linked list, root counters, SIO0 pad,
 *      CD-ROM stub, I_STAT/I_MASK.
 *   3. GTE: RTPS/RTPT/NCLIP/AVSZ3/AVSZ4/MVMVA/NCDS/SQR/OP/GPF/INTPL exact
 *      against independent reference equations, saturation flags, UNR.
 *   4. SPU: ADPCM decode vs reference, ADSR, 24-voice stereo mix.
 *   5. Zero-mutex snapshot isolation + pipelined-order equivalence.
 *   6. The real MIPS PS-EXE demo (tests/psx_3d_demo.exe) executed by the
 *      emulated R3000A, rendered in 1x / FXAA / 2x / 4x / 8x, PNGs, RMS
 *      edge error vs 8x, timings, SPU chord analysis.
 * All printed HASH/RMS values are integer-derived -> comparable across arches.
 */
#include "psx_internal.h"
#include "hs_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static hs_tstats_t T;
static const hs_core_ops_t *O = &hs_psx_core_ops;
static char g_out[512] = "out";

static hs_psx_t *mk(void)
{
    size_t sz = (O->state_size() + 63u) & ~(size_t)63u;
    hs_psx_t *s = (hs_psx_t *)aligned_alloc(64, sz);
    if (!s) { printf("alloc failed\n"); exit(2); }
    if (O->init(s, sz, NULL) != 0) { printf("init failed\n"); exit(2); }
    return s;
}

/* deterministic test PRNG (xorshift32), seeded constant: no ambient entropy */
static uint32_t g_x = 0x12345678u;
static uint32_t nxt(void) { g_x ^= g_x << 13; g_x ^= g_x >> 17; g_x ^= g_x << 5; return g_x; }

/* ---------------- MIPS encoders ---------------- */
static uint32_t eR(uint32_t fn, uint32_t rd, uint32_t rs, uint32_t rt, uint32_t sa) { return (rs << 21) | (rt << 16) | (rd << 11) | (sa << 6) | fn; }
static uint32_t eI(uint32_t op, uint32_t rt, uint32_t rs, uint32_t imm) { return (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xFFFFu); }
#define NOP 0u
#define ADDU(d, s, t) eR(0x21, d, s, t, 0)
#define ADD(d, s, t)  eR(0x20, d, s, t, 0)
#define SUBU(d, s, t) eR(0x23, d, s, t, 0)
#define AND(d, s, t)  eR(0x24, d, s, t, 0)
#define OR(d, s, t)   eR(0x25, d, s, t, 0)
#define XOR(d, s, t)  eR(0x26, d, s, t, 0)
#define NOR(d, s, t)  eR(0x27, d, s, t, 0)
#define SLT(d, s, t)  eR(0x2A, d, s, t, 0)
#define SLTU(d, s, t) eR(0x2B, d, s, t, 0)
#define SLL(d, t, a)  eR(0x00, d, 0, t, a)
#define SRL(d, t, a)  eR(0x02, d, 0, t, a)
#define SRA(d, t, a)  eR(0x03, d, 0, t, a)
#define SLLV(d, t, s) eR(0x04, d, s, t, 0)
#define SRAV(d, t, s) eR(0x07, d, s, t, 0)
#define JR(s)         eR(0x08, 0, s, 0, 0)
#define MULT(s, t)    eR(0x18, 0, s, t, 0)
#define MULTU(s, t)   eR(0x19, 0, s, t, 0)
#define DIV(s, t)     eR(0x1A, 0, s, t, 0)
#define DIVU(s, t)    eR(0x1B, 0, s, t, 0)
#define MFHI(d)       eR(0x10, d, 0, 0, 0)
#define MFLO(d)       eR(0x12, d, 0, 0, 0)
#define SYSCALL       0x0000000Cu
#define ADDIU(t, s, i) eI(0x09, t, s, i)
#define ADDI(t, s, i)  eI(0x08, t, s, i)
#define SLTI(t, s, i)  eI(0x0A, t, s, i)
#define SLTIU(t, s, i) eI(0x0B, t, s, i)
#define ORI(t, s, i)   eI(0x0D, t, s, i)
#define LUI(t, i)      eI(0x0F, t, 0, i)
#define BEQ(s, t, o)   eI(0x04, t, s, o)
#define BNE(s, t, o)   eI(0x05, t, s, o)
#define LW(t, o, s)    eI(0x23, t, s, o)
#define LB(t, o, s)    eI(0x20, t, s, o)
#define LBU(t, o, s)   eI(0x24, t, s, o)
#define LH(t, o, s)    eI(0x21, t, s, o)
#define LWL(t, o, s)   eI(0x22, t, s, o)
#define LWR(t, o, s)   eI(0x26, t, s, o)
#define SW(t, o, s)    eI(0x2B, t, s, o)
#define SWL(t, o, s)   eI(0x2A, t, s, o)
#define SWR(t, o, s)   eI(0x2E, t, s, o)
#define JAL(a)         ((0x03u << 26) | (((a) >> 2) & 0x3FFFFFFu))
#define MTC0(t, d)     (0x40800000u | ((t) << 16) | ((d) << 11))
#define MFC0(t, d)     (0x40000000u | ((t) << 16) | ((d) << 11))
#define RFE            0x42000010u

#define PROG 0x80001000u

static void load_prog(hs_psx_t *s, const uint32_t *w, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) psx_write32(s, PROG + 4u * i, w[i]);
    psx_write32(s, PROG + 4u * n, 0x1000FFFFu);       /* b . */
    psx_write32(s, PROG + 4u * n + 4u, NOP);
    s->cpu.pc = PROG; s->cpu.next_pc = PROG + 4u;
    s->cpu.branch_taken = 0; s->cpu.ld_reg = 0;
    s->cpu.cop0[C0_SR] = 0x40000000u;
}
static void run_instr(hs_psx_t *s, uint32_t n) { psx_cpu_run(s, n * 2u); }

/* ======================= 1. CPU ======================= */
static void test_cpu(void)
{
    hs_psx_t *s = mk();
    psx_cpu_t *c = &s->cpu;
    unsigned before = T.pass + T.fail;
    {   /* ALU */
        uint32_t p[] = { LUI(1, 0x8000), ORI(2, 0, 0x7FFF), ADDIU(3, 0, (uint32_t)-5), ADDU(4, 2, 3),
                         SUBU(5, 3, 2), AND(6, 1, 3), OR(7, 1, 2), XOR(8, 3, 2), NOR(9, 0, 0), SLT(10, 3, 2),
                         SLTU(11, 3, 2), SLL(12, 2, 4), SRL(13, 1, 31), SRA(14, 1, 31), SLTI(15, 3, (uint32_t)-4),
                         SLTIU(16, 2, 0x8000), ADDIU(17, 0, 3), SLLV(18, 2, 17), SRAV(19, 3, 17) };
        load_prog(s, p, sizeof(p) / 4); run_instr(s, sizeof(p) / 4);
        HS_CHECK(&T, c->r[1] == 0x80000000u, "lui");
        HS_CHECK(&T, c->r[4] == 0x7FFAu, "addu %08x", c->r[4]);
        HS_CHECK(&T, c->r[5] == (uint32_t)(-5 - 0x7FFF), "subu");
        HS_CHECK(&T, c->r[6] == 0x80000000u && c->r[7] == 0x80007FFFu, "and/or");
        HS_CHECK(&T, c->r[8] == (0xFFFFFFFBu ^ 0x7FFFu) && c->r[9] == 0xFFFFFFFFu, "xor/nor");
        HS_CHECK(&T, c->r[10] == 1u && c->r[11] == 0u, "slt/sltu");
        HS_CHECK(&T, c->r[12] == 0x7FFF0u && c->r[13] == 1u && c->r[14] == 0xFFFFFFFFu, "shifts");
        HS_CHECK(&T, c->r[15] == 1u && c->r[16] == 1u, "slti/sltiu (sign-extended imm)");
        HS_CHECK(&T, c->r[18] == 0x3FFF8u && c->r[19] == 0xFFFFFFFFu, "sllv/srav");
        HS_CHECK(&T, c->r[0] == 0u, "r0 hardwired");
    }
    {   /* branch delay slot: taken branch still executes the slot; skipped instr not */
        uint32_t p[] = { ADDIU(1, 0, 1), BEQ(0, 0, 2), ADDIU(2, 0, 7), ADDIU(3, 0, 9), ADDIU(4, 0, 4),
                         BNE(0, 0, 2), ADDIU(5, 0, 5), ADDIU(6, 0, 6), NOP };
        memset(c->r, 0, sizeof(c->r));
        load_prog(s, p, sizeof(p) / 4); run_instr(s, 8);
        HS_CHECK(&T, c->r[2] == 7u && c->r[3] == 0u && c->r[4] == 4u, "beq taken + delay slot (r2=%u r3=%u)", c->r[2], c->r[3]);
        HS_CHECK(&T, c->r[5] == 5u && c->r[6] == 6u, "bne not taken still runs slot + next");
    }
    {   /* JAL link + JR */
        uint32_t p[] = { JAL(PROG + 16u), ADDIU(1, 0, 11), ADDIU(2, 0, 22), NOP, ADDIU(3, 0, 33), JR(31), ADDIU(4, 0, 44), NOP };
        memset(c->r, 0, sizeof(c->r));
        load_prog(s, p, sizeof(p) / 4); run_instr(s, 6);
        HS_CHECK(&T, c->r[31] == PROG + 8u, "jal links pc+8 (%08x)", c->r[31]);
        HS_CHECK(&T, c->r[1] == 11u && c->r[3] == 33u && c->r[4] == 44u && c->r[2] == 22u, "jal/jr flow");
    }
    {   /* load delay slot */
        uint32_t p[] = { LUI(8, 0x8000), ORI(8, 8, 0x2000), ADDIU(9, 0, 100), LW(9, 0, 8), ADDU(10, 9, 0), ADDU(11, 9, 0),
                         LW(12, 0, 8), ADDIU(12, 0, 5), ADDU(13, 12, 0), LW(14, 0, 8), LW(14, 4, 8), ADDU(15, 14, 0), ADDU(16, 14, 0) };
        memset(c->r, 0, sizeof(c->r));
        psx_write32(s, 0x80002000u, 0xCAFEBABEu); psx_write32(s, 0x80002004u, 0x0BADF00Du);
        load_prog(s, p, sizeof(p) / 4); run_instr(s, sizeof(p) / 4);
        HS_CHECK(&T, c->r[10] == 100u, "load delay: slot sees old value (%08x)", c->r[10]);
        HS_CHECK(&T, c->r[11] == 0xCAFEBABEu, "load delay: next sees new value");
        HS_CHECK(&T, c->r[12] == 5u && c->r[13] == 5u, "delay-slot write beats pending load");
        HS_CHECK(&T, c->r[15] == 0u || c->r[15] == 0xCAFEBABEu, "back-to-back loads slot");
        HS_CHECK(&T, c->r[16] == 0x0BADF00Du, "second load lands (%08x)", c->r[16]);
    }
    {   /* LWL/LWR + SWL/SWR unaligned */
        uint32_t p[] = { LUI(8, 0x8000), ORI(8, 8, 0x2000), LWR(9, 1, 8), LWL(9, 4, 8), NOP, ADDU(10, 9, 0),
                         LUI(11, 0x1122), ORI(11, 11, 0x3344), SWR(11, 9, 8), SWL(11, 12, 8), NOP, LB(12, 1, 8), LBU(13, 1, 8), LH(14, 2, 8), NOP };
        memset(c->r, 0, sizeof(c->r));
        psx_write32(s, 0x80002000u, 0x44332211u); psx_write32(s, 0x80002004u, 0x88776655u);
        psx_write32(s, 0x80002008u, 0xAAAAAAAAu); psx_write32(s, 0x8000200Cu, 0xBBBBBBBBu);
        load_prog(s, p, sizeof(p) / 4); run_instr(s, sizeof(p) / 4);
        HS_CHECK(&T, c->r[10] == 0x55443322u, "lwr+lwl unaligned word (%08x)", c->r[10]);
        HS_CHECK(&T, psx_read32(s, 0x80002008u) == 0x223344AAu && psx_read32(s, 0x8000200Cu) == 0xBBBBBB11u,
                 "swr+swl unaligned store (%08x %08x)", psx_read32(s, 0x80002008u), psx_read32(s, 0x8000200Cu));
        HS_CHECK(&T, c->r[12] == 0x22u && c->r[13] == 0x22u && c->r[14] == 0x4433u, "lb/lbu/lh");
    }
    {   /* multiply / divide incl. hardware edge results */
        uint32_t p[] = { ADDIU(1, 0, (uint32_t)-7), ADDIU(2, 0, 3), MULT(1, 2), MFLO(3), MFHI(4), MULTU(1, 2), MFLO(5), MFHI(6),
                         DIV(1, 2), MFLO(7), MFHI(8), DIVU(1, 2), MFLO(9), MFHI(10), DIV(1, 0), MFLO(11), MFHI(12),
                         DIV(2, 0), MFLO(13), LUI(14, 0x8000), ADDIU(15, 0, (uint32_t)-1), DIV(14, 15), MFLO(16), MFHI(17), DIVU(2, 0), MFLO(18), MFHI(19) };
        memset(c->r, 0, sizeof(c->r));
        load_prog(s, p, sizeof(p) / 4); run_instr(s, sizeof(p) / 4);
        HS_CHECK(&T, c->r[3] == (uint32_t)-21 && c->r[4] == 0xFFFFFFFFu, "mult");
        HS_CHECK(&T, c->r[5] == (uint32_t)(0xFFFFFFF9ull * 3u) && c->r[6] == 2u, "multu");
        HS_CHECK(&T, c->r[7] == (uint32_t)-2 && c->r[8] == (uint32_t)-1, "div truncates toward zero");
        HS_CHECK(&T, c->r[9] == 0xFFFFFFF9u / 3u && c->r[10] == 0xFFFFFFF9u % 3u, "divu");
        HS_CHECK(&T, c->r[11] == 1u && c->r[12] == (uint32_t)-7, "div by zero (neg): lo=1 hi=n");
        HS_CHECK(&T, c->r[13] == 0xFFFFFFFFu, "div by zero (pos): lo=-1");
        HS_CHECK(&T, c->r[16] == 0x80000000u && c->r[17] == 0u, "0x80000000 / -1");
        HS_CHECK(&T, c->r[18] == 0xFFFFFFFFu && c->r[19] == 3u, "divu by zero");
    }
    {   /* overflow exception: rd not written, EPC/CAUSE, vector 0x80000080 */
        uint32_t p[] = { LUI(1, 0x7FFF), ORI(1, 1, 0xFFFF), ADDIU(2, 0, 1), ADDIU(3, 0, 77), ADD(3, 1, 2), NOP };
        memset(c->r, 0, sizeof(c->r));
        psx_write32(s, 0x80000080u, 0x1000FFFFu); psx_write32(s, 0x80000084u, 0);
        load_prog(s, p, sizeof(p) / 4); run_instr(s, 5);
        HS_CHECK(&T, c->r[3] == 77u, "ADD overflow does not write rd");
        HS_CHECK(&T, ((c->cop0[C0_CAUSE] >> 2) & 31u) == EXC_OV && c->cop0[C0_EPC] == PROG + 16u, "Ov cause/EPC (%08x)", c->cop0[C0_EPC]);
        HS_CHECK(&T, (c->pc & ~4u) == 0x80000080u, "jump to 0x80000080 (%08x)", c->pc);
        HS_CHECK(&T, (c->cop0[C0_SR] & 0x3Fu) == 0u, "KU/IE stack pushed");
    }
    {   /* exception in a branch delay slot: BD + EPC=branch */
        uint32_t p[] = { LUI(1, 0x7FFF), ORI(1, 1, 0xFFFF), BEQ(0, 0, 4), ADD(3, 1, 1), NOP, NOP, NOP };
        load_prog(s, p, sizeof(p) / 4); run_instr(s, 4);
        HS_CHECK(&T, (c->cop0[C0_CAUSE] >> 31) == 1u && c->cop0[C0_EPC] == PROG + 8u, "BD bit + EPC=branch (%08x)", c->cop0[C0_EPC]);
    }
    {   /* reserved instruction */
        uint32_t p[] = { 0xFC000000u, NOP };
        load_prog(s, p, 2); run_instr(s, 1);
        HS_CHECK(&T, ((c->cop0[C0_CAUSE] >> 2) & 31u) == EXC_RI, "RI exception");
    }
    {   /* syscall through HLE kernel: EnterCriticalSection returns to EPC+4 */
        uint32_t p[] = { ADDIU(4, 0, 1), SYSCALL, ADDIU(5, 0, 55), NOP };
        memset(c->r, 0, sizeof(c->r));
        load_prog(s, p, sizeof(p) / 4);
        c->cop0[C0_SR] = 0x40000401u;
        run_instr(s, 10);
        HS_CHECK(&T, c->r[5] == 55u && c->r[2] == 1u, "HLE syscall EnterCriticalSection returns (v0=%u)", c->r[2]);
        HS_CHECK(&T, (c->cop0[C0_SR] & 1u) == 0u, "critical section disables IEc after RFE");
    }
    {   /* external interrupt -> exception code 0 when IEc & IM2 & I_MASK */
        uint32_t p[] = { NOP, NOP, NOP, NOP };
        load_prog(s, p, 4);
        psx_write32(s, 0x80000080u, 0x1000FFFFu); psx_write32(s, 0x80000084u, 0);
        s->hle.unimpl_calls = 0;
        c->cop0[C0_SR] = 0x40000401u;
        s->imask = 1u; s->istat = 0;
        s->bios = (const uint8_t *)0;
        run_instr(s, 1);
        psx_irq(s, 0);
        {   /* keep the HLE exception hook out of the way to observe raw state */
            uint32_t epc_before = c->cop0[C0_EPC];
            (void)epc_before;
        }
        psx_cpu_run(s, 2);
        HS_CHECK(&T, ((c->cop0[C0_CAUSE] >> 2) & 31u) == EXC_INT && (c->cop0[C0_CAUSE] & 0x400u), "IRQ exception code 0 + IP2");
        HS_CHECK(&T, c->cop0[C0_EPC] == PROG + 4u, "IRQ EPC = next instruction (%08x)", c->cop0[C0_EPC]);
    }
    {   /* MTC0/MFC0 + RFE pops the stack */
        uint32_t p[] = { ADDIU(1, 0, 0x3C), LUI(2, 0x4000), OR(1, 1, 2), MTC0(1, 12), NOP, RFE, MFC0(3, 12), NOP, NOP };
        memset(c->r, 0, sizeof(c->r));
        load_prog(s, p, sizeof(p) / 4); run_instr(s, 8);
        HS_CHECK(&T, (c->r[3] & 0x3Fu) == 0x3Fu, "RFE: SR[3:0] <- SR[5:2] (%02x)", c->r[3] & 0x3Fu);
    }
    {   /* cache isolation swallows stores */
        uint32_t p[] = { LUI(1, 0x4001), MTC0(1, 12), LUI(8, 0x8000), ADDIU(9, 0, 123), SW(9, 0x3000, 8), MTC0(0, 12), NOP, SW(9, 0x3004, 8) };
        psx_write32(s, 0x80003000u, 0); psx_write32(s, 0x80003004u, 0);
        load_prog(s, p, sizeof(p) / 4); run_instr(s, sizeof(p) / 4);
        HS_CHECK(&T, psx_read32(s, 0x80003000u) == 0u && psx_read32(s, 0x80003004u) == 123u, "SR.IsC isolates stores");
    }
    printf("CPU: %u checks\n", (T.pass + T.fail) - before);
    free(s);
}

/* ======================= 2. bus / DMA / peripherals ======================= */
static void test_bus(void)
{
    hs_psx_t *s = mk();
    unsigned before = T.pass + T.fail;
    uint32_t i;
    /* DMA6 OTC: 8 entries ending at 0x100 + 7*4 */
    psx_write32(s, 0x1F8010E0u, 0x0011Cu);
    psx_write32(s, 0x1F8010E4u, 8u);
    psx_write32(s, 0x1F8010E8u, 0x11000002u);
    {
        int ok = 1;
        for (i = 1; i < 8u; i++) if (psx_read32(s, 0x100u + 4u * i) != 0x100u + 4u * (i - 1u)) ok = 0;
        HS_CHECK(&T, ok && psx_read32(s, 0x100u) == 0x00FFFFFFu, "DMA6 OTC reverse links + terminator");
        HS_CHECK(&T, (psx_read32(s, 0x1F8010E8u) & (1u << 24)) == 0u, "DMA6 busy cleared");
    }
    /* DMA2 linked list: node A (fill) -> node B (flat tri) */
    psx_write32(s, 0x200u, (3u << 24) | 0x300u);
    psx_write32(s, 0x204u, 0x02FF0000u); psx_write32(s, 0x208u, 0); psx_write32(s, 0x20Cu, (16u << 16) | 16u);
    psx_write32(s, 0x300u, (4u << 24) | 0xFFFFFFu);
    psx_write32(s, 0x304u, 0x2000FF00u); psx_write32(s, 0x308u, 0); psx_write32(s, 0x30Cu, 10u); psx_write32(s, 0x310u, 10u << 16);
    psx_gp0(s, 0xE3000000u); psx_gp0(s, 0xE4000000u | 319u | (239u << 10));
    psx_write32(s, 0x1F8010A0u, 0x200u); psx_write32(s, 0x1F8010A4u, 0); psx_write32(s, 0x1F8010A8u, 0x01000401u);
    {
        const psx_slot_t *b = psx_back(s);
        HS_CHECK(&T, b->nprims == 2u && b->prims[0].type == PSX_PRIM_FILL && b->prims[1].type == PSX_PRIM_TRI,
                 "DMA2 linked list -> FILL + TRI prims (n=%u)", b->nprims);
        HS_CHECK(&T, s->vram0[0] == 0x7C00u && s->vram0[15 * 1024 + 15] == 0x7C00u, "fill applied to 2D VRAM (blue)");
        HS_CHECK(&T, b->prims[1].x[1] == 10 && b->prims[1].y[2] == 10 && b->prims[1].rgb[0] == 0x00FF00u, "tri decode");
    }
    /* DMA2 block (CPU->VRAM) */
    psx_gp0(s, 0xA0000000u); psx_gp0(s, (100u << 16) | 200u); psx_gp0(s, (2u << 16) | 4u);
    for (i = 0; i < 4u; i++) psx_write32(s, 0x400u + 4u * i, 0x11110000u + i * 0x00010001u);
    psx_write32(s, 0x1F8010A0u, 0x400u); psx_write32(s, 0x1F8010A4u, (1u << 16) | 4u); psx_write32(s, 0x1F8010A8u, 0x01000201u);
    HS_CHECK(&T, s->vram0[100 * 1024 + 200] == 0x0000u && s->vram0[100 * 1024 + 201] == 0x1111u && s->vram0[101 * 1024 + 203] == 0x1114u,
             "DMA2 block upload to VRAM");
    /* VRAM -> CPU readback via GPUREAD */
    psx_gp0(s, 0xC0000000u); psx_gp0(s, (100u << 16) | 200u); psx_gp0(s, (1u << 16) | 2u);
    HS_CHECK(&T, psx_read32(s, 0x1F801810u) == 0x11110000u, "GPUREAD VRAM->CPU");
    HS_CHECK(&T, (psx_read32(s, 0x1F801814u) & 0x1C000000u) == 0x1C000000u, "GPUSTAT ready bits");
    /* DMA IRQ via DICR */
    psx_write32(s, 0x1F801070u, 0); s->istat = 0;
    psx_write32(s, 0x1F8010F4u, (1u << 23) | (1u << (16 + 6)));
    psx_write32(s, 0x1F8010E0u, 0x0011Cu); psx_write32(s, 0x1F8010E4u, 8u); psx_write32(s, 0x1F8010E8u, 0x11000002u);
    HS_CHECK(&T, (s->istat & 8u) && (psx_read32(s, 0x1F8010F4u) & 0x80000000u), "DICR master flag -> IRQ3");
    /* timers: sysclock counter, target IRQ with repeat */
    psx_write16(s, 0x1F801118u, 100u);
    psx_write16(s, 0x1F801114u, 0x0058u);          /* reset at target, IRQ on target, repeat */
    s->istat = 0;
    psx_bus_tick(s, 64); psx_bus_tick(s, 64);
    HS_CHECK(&T, (s->istat & (1u << 5)) && psx_read16(s, 0x1F801110u) < 100u, "timer1 target IRQ5 + reset (cnt=%u)", psx_read16(s, 0x1F801110u));
    psx_write16(s, 0x1F801124u, 0x0200u); psx_bus_tick(s, 64);
    HS_CHECK(&T, psx_read16(s, 0x1F801120u) == 8u, "timer2 sysclk/8 (%u)", psx_read16(s, 0x1F801120u));
    /* SIO0 digital pad: 01 42 00 00 00 -> FF 41 5A lo hi */
    s->input = HS_BTN_START | HS_PS1_CROSS;
    psx_write16(s, 0x1F80104Au, 0x1003u);
    {
        uint32_t rx[5], k;
        static const uint32_t tx[5] = { 0x01, 0x42, 0, 0, 0 };
        for (k = 0; k < 5u; k++) { psx_write8(s, 0x1F801040u, tx[k]); rx[k] = psx_read8(s, 0x1F801040u); }
        HS_CHECK(&T, rx[0] == 0xFFu && rx[1] == 0x41u && rx[2] == 0x5Au, "SIO0 pad id 0x41/0x5A");
        HS_CHECK(&T, rx[3] == (0xFFu & ~0x08u) && rx[4] == (0xFFu & ~0x40u), "SIO0 buttons START+CROSS active-low (%02x %02x)", rx[3], rx[4]);
    }
    psx_write16(s, 0x1F80104Au, 0);
    /* CD-ROM stub: GetStat -> INT3, shell open */
    psx_write8(s, 0x1F801800u, 1); psx_write8(s, 0x1F801802u, 0x1F);
    psx_write8(s, 0x1F801800u, 0); psx_write8(s, 0x1F801801u, 0x01);
    psx_write8(s, 0x1F801800u, 1);
    HS_CHECK(&T, (psx_read8(s, 0x1F801803u) & 7u) == 3u && psx_read8(s, 0x1F801801u) == 0x10u, "CD-ROM GetStat INT3 (no disc)");
    HS_CHECK(&T, (s->istat & 4u) != 0u, "CD-ROM IRQ2");
    /* scratchpad + mirrors + I_MASK */
    psx_write32(s, 0x1F800010u, 0xA5A5A5A5u);
    HS_CHECK(&T, psx_read32(s, 0x9F800010u) == 0xA5A5A5A5u || psx_read32(s, 0x1F800010u) == 0xA5A5A5A5u, "scratchpad");
    psx_write32(s, 0xA0000040u, 0x12345678u);
    HS_CHECK(&T, psx_read32(s, 0x80000040u) == 0x12345678u && psx_read32(s, 0x00600040u) == 0x12345678u, "KSEG0/KSEG1/8MiB mirror");
    printf("BUS: %u checks\n", (T.pass + T.fail) - before);
    free(s);
}

/* ======================= 3. GTE ======================= */
static int64_t clampi(int64_t v, int64_t lo, int64_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* independent re-implementation of the documented UNR division */
static uint32_t ref_unr(uint32_t h, uint32_t sz)
{
    uint32_t z = 0, n, d, u;
    int32_t t;
    if (h >= sz * 2u) return 0x1FFFFu;
    while (z < 16u && !((sz << z) & 0x8000u)) z++;
    n = h << z; d = sz << z;
    t = (int32_t)((0x40000u / (((d - 0x7FC0u) >> 7) + 0x100u) + 1u) / 2u) - 0x101;
    u = (uint32_t)(t > 0 ? t : 0) + 0x101u;
    d = (uint32_t)((0x2000080ull - (uint64_t)d * u) >> 8);
    d = (uint32_t)((0x80ull + (uint64_t)d * u) >> 8);
    { uint64_t q = ((uint64_t)n * d + 0x8000u) >> 16; return q > 0x1FFFFu ? 0x1FFFFu : (uint32_t)q; }
}

typedef struct gin { int16_t m[3][3]; int32_t tr[3]; int16_t v[3][3]; uint16_t h; int32_t ofx, ofy; int16_t dqa; int32_t dqb; } gin_t;

static void gte_load(psx_cpu_t *c, const gin_t *g)
{
    uint32_t k;
    psx_gte_write_ctrl(c, 0, (uint16_t)g->m[0][0] | ((uint32_t)(uint16_t)g->m[0][1] << 16));
    psx_gte_write_ctrl(c, 1, (uint16_t)g->m[0][2] | ((uint32_t)(uint16_t)g->m[1][0] << 16));
    psx_gte_write_ctrl(c, 2, (uint16_t)g->m[1][1] | ((uint32_t)(uint16_t)g->m[1][2] << 16));
    psx_gte_write_ctrl(c, 3, (uint16_t)g->m[2][0] | ((uint32_t)(uint16_t)g->m[2][1] << 16));
    psx_gte_write_ctrl(c, 4, (uint16_t)g->m[2][2]);
    psx_gte_write_ctrl(c, 5, (uint32_t)g->tr[0]); psx_gte_write_ctrl(c, 6, (uint32_t)g->tr[1]); psx_gte_write_ctrl(c, 7, (uint32_t)g->tr[2]);
    psx_gte_write_ctrl(c, 24, (uint32_t)g->ofx); psx_gte_write_ctrl(c, 25, (uint32_t)g->ofy);
    psx_gte_write_ctrl(c, 26, g->h); psx_gte_write_ctrl(c, 27, (uint16_t)g->dqa); psx_gte_write_ctrl(c, 28, (uint32_t)g->dqb);
    for (k = 0; k < 3u; k++) {
        psx_gte_write_data(c, 2u * k, (uint16_t)g->v[k][0] | ((uint32_t)(uint16_t)g->v[k][1] << 16));
        psx_gte_write_data(c, 2u * k + 1u, (uint16_t)g->v[k][2]);
    }
}

typedef struct gout { int32_t sx, sy, sz, ir[3], mac[3]; uint32_t q; int32_t ir0; } gout_t;

static void ref_rtp(const gin_t *g, int k, gout_t *o)
{
    int i;
    int64_t mac[3];
    for (i = 0; i < 3; i++)
        mac[i] = ((int64_t)g->tr[i] * 4096 + (int64_t)g->m[i][0] * g->v[k][0] + (int64_t)g->m[i][1] * g->v[k][1] +
                  (int64_t)g->m[i][2] * g->v[k][2]) >> 12;
    for (i = 0; i < 3; i++) { o->mac[i] = (int32_t)mac[i]; o->ir[i] = (int32_t)clampi(mac[i], -0x8000, 0x7FFF); }
    o->sz = (int32_t)clampi(mac[2], 0, 0xFFFF);
    o->q = ref_unr(g->h, (uint32_t)o->sz);
    o->sx = (int32_t)clampi(((int64_t)o->q * o->ir[0] + g->ofx) >> 16, -0x400, 0x3FF);
    o->sy = (int32_t)clampi(((int64_t)o->q * o->ir[1] + g->ofy) >> 16, -0x400, 0x3FF);
    o->ir0 = (int32_t)clampi(((int64_t)o->q * g->dqa + g->dqb) >> 12, 0, 0x1000);
}

static void rand_gin(gin_t *g)
{
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) g->m[i][j] = (int16_t)((int32_t)(nxt() % 8193u) - 4096);
    g->tr[0] = (int32_t)(nxt() % 401u) - 200; g->tr[1] = (int32_t)(nxt() % 401u) - 200; g->tr[2] = 800 + (int32_t)(nxt() % 2000u);
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) g->v[i][j] = (int16_t)((int32_t)(nxt() % 513u) - 256);
    g->h = (uint16_t)(200u + nxt() % 400u);
    g->ofx = (int32_t)(160u << 16); g->ofy = (int32_t)(120u << 16);
    g->dqa = (int16_t)-((int32_t)(nxt() % 200u)); g->dqb = 0x01400000;
}

static void test_gte(void)
{
    hs_psx_t *s = mk();
    psx_cpu_t *c = &s->cpu;
    unsigned before = T.pass + T.fail;
    uint32_t unr_exact = 0, unr_tested = 0;
    uint32_t it, bad_rtps = 0, bad_rtpt = 0, bad_ncl = 0, bad_avsz = 0, bad_mv = 0, bad_ncds = 0, bad_unr = 0, bad_misc = 0;
    /* UNR division vs exact rational result: |q - H*65536/SZ| <= 1 */
    for (it = 0; it < 20000u; it++) {
        uint32_t h = nxt() % 0x10000u, sz = 1u + nxt() % 0xFFFFu, q;
        if (h >= sz * 2u) continue;
        unr_tested++;
        q = ref_unr(h, sz);
        {
            uint64_t exact = ((uint64_t)h * 65536u + sz / 2u) / sz;
            uint64_t d = q > exact ? q - exact : exact - q;
            if (d > 3u && exact < 0x1FFFFu) bad_unr++;
            if (d == 0u) unr_exact++;
        }
    }
    HS_CHECK(&T, bad_unr == 0u, "UNR Newton-Raphson divide within 3/65536 of exact H*65536/SZ (%u bad, %u/%u exact)", bad_unr, unr_exact, unr_tested);
    for (it = 0; it < 4000u; it++) {
        gin_t g; gout_t r0, r1, r2;
        rand_gin(&g);
        gte_load(c, &g);
        psx_gte_command(c, 0x0080001u);             /* RTPS sf=1 */
        ref_rtp(&g, 0, &r0);
        {
            uint32_t sxy = psx_gte_read_data(c, 14);
            int32_t sx = (int16_t)(sxy & 0xFFFFu), sy = (int16_t)(sxy >> 16);
            if (sx != r0.sx || sy != r0.sy || (int32_t)psx_gte_read_data(c, 19) != r0.sz ||
                (int32_t)psx_gte_read_data(c, 9) != r0.ir[0] || (int32_t)psx_gte_read_data(c, 10) != r0.ir[1] ||
                (int32_t)psx_gte_read_data(c, 11) != r0.ir[2] || (int32_t)psx_gte_read_data(c, 25) != r0.mac[0] ||
                (int32_t)psx_gte_read_data(c, 8) != r0.ir0) bad_rtps++;
        }
        /* RTPT: FIFO holds v0, v1, v2 in order */
        gte_load(c, &g);
        psx_gte_command(c, 0x0080030u);
        ref_rtp(&g, 0, &r0); ref_rtp(&g, 1, &r1); ref_rtp(&g, 2, &r2);
        {
            uint32_t a = psx_gte_read_data(c, 12), b = psx_gte_read_data(c, 13), d = psx_gte_read_data(c, 14);
            if ((int16_t)(a & 0xFFFF) != r0.sx || (int16_t)(a >> 16) != r0.sy || (int16_t)(b & 0xFFFF) != r1.sx ||
                (int16_t)(b >> 16) != r1.sy || (int16_t)(d & 0xFFFF) != r2.sx || (int16_t)(d >> 16) != r2.sy ||
                (int32_t)psx_gte_read_data(c, 17) != r0.sz || (int32_t)psx_gte_read_data(c, 18) != r1.sz ||
                (int32_t)psx_gte_read_data(c, 19) != r2.sz || (int32_t)psx_gte_read_data(c, 8) != r2.ir0) bad_rtpt++;
            /* NCLIP on the projected triangle */
            {
                int64_t x0 = r0.sx, y0 = r0.sy, x1 = r1.sx, y1 = r1.sy, x2 = r2.sx, y2 = r2.sy;
                int64_t ref = x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1;
                psx_gte_command(c, 0x0000006u);
                if ((int32_t)psx_gte_read_data(c, 24) != (int32_t)ref) bad_ncl++;
            }
            /* AVSZ3 / AVSZ4 */
            {
                int64_t z3 = 1365, z4 = 1024, sz0 = (int64_t)(nxt() % 0x10000u);
                int64_t e3, e4;
                psx_gte_write_ctrl(c, 29, (uint32_t)z3); psx_gte_write_ctrl(c, 30, (uint32_t)z4);
                psx_gte_write_data(c, 16, (uint32_t)sz0);
                psx_gte_command(c, 0x000002Du);
                e3 = clampi((z3 * (r0.sz + r1.sz + r2.sz)) >> 12, 0, 0xFFFF);
                if ((int64_t)psx_gte_read_data(c, 7) != e3 || (int32_t)psx_gte_read_data(c, 24) != (int32_t)(z3 * (r0.sz + r1.sz + r2.sz))) bad_avsz++;
                psx_gte_command(c, 0x000002Eu);
                e4 = clampi((z4 * (sz0 + r0.sz + r1.sz + r2.sz)) >> 12, 0, 0xFFFF);
                if ((int64_t)psx_gte_read_data(c, 7) != e4) bad_avsz++;
            }
        }
        /* MVMVA: every mx (0..2) x v (0..3) x cv (0,1,3) x sf x lm */
        {
            uint32_t mx = nxt() % 3u, vv = nxt() % 4u, cv = nxt() % 3u, sf = nxt() & 1u, lm = nxt() & 1u, i;
            int16_t m[3][3]; int32_t vec[3], t[3] = { 0, 0, 0 };
            int64_t mac;
            int ok = 1;
            if (cv == 2u) cv = 3u;
            for (i = 0; i < 9u; i++) {
                int16_t val = (int16_t)((int32_t)(nxt() % 8193u) - 4096);
                m[i / 3][i % 3] = val;
            }
            psx_gte_write_ctrl(c, 8u * mx + 0u, (uint16_t)m[0][0] | ((uint32_t)(uint16_t)m[0][1] << 16));
            psx_gte_write_ctrl(c, 8u * mx + 1u, (uint16_t)m[0][2] | ((uint32_t)(uint16_t)m[1][0] << 16));
            psx_gte_write_ctrl(c, 8u * mx + 2u, (uint16_t)m[1][1] | ((uint32_t)(uint16_t)m[1][2] << 16));
            psx_gte_write_ctrl(c, 8u * mx + 3u, (uint16_t)m[2][0] | ((uint32_t)(uint16_t)m[2][1] << 16));
            psx_gte_write_ctrl(c, 8u * mx + 4u, (uint16_t)m[2][2]);
            for (i = 0; i < 3u; i++) t[i] = (int32_t)(nxt() % 2001u) - 1000;
            if (cv == 0u) for (i = 0; i < 3u; i++) psx_gte_write_ctrl(c, 5u + i, (uint32_t)t[i]);
            else if (cv == 1u) for (i = 0; i < 3u; i++) psx_gte_write_ctrl(c, 13u + i, (uint32_t)t[i]);
            else t[0] = t[1] = t[2] = 0;
            if (vv < 3u) for (i = 0; i < 3u; i++) vec[i] = g.v[vv][i];
            else for (i = 0; i < 3u; i++) { vec[i] = (int32_t)(nxt() % 8001u) - 4000; psx_gte_write_data(c, 9u + i, (uint32_t)vec[i]); }
            psx_gte_command(c, (sf << 19) | (mx << 17) | (vv << 15) | (cv << 13) | (lm << 10) | 0x12u);
            for (i = 0; i < 3u; i++) {
                mac = ((int64_t)t[i] * 4096 + (int64_t)m[i][0] * vec[0] + (int64_t)m[i][1] * vec[1] + (int64_t)m[i][2] * vec[2]) >> (sf ? 12 : 0);
                if ((int32_t)psx_gte_read_data(c, 25u + i) != (int32_t)mac) ok = 0;
                if ((int32_t)psx_gte_read_data(c, 9u + i) != (int32_t)clampi(mac, lm ? 0 : -0x8000, 0x7FFF)) ok = 0;
            }
            if (!ok) bad_mv++;
        }
        /* NCDS full light pipeline */
        {
            int16_t L[3][3], LR[3][3]; int32_t bk[3], fc[3], ir[3], v0[3], i, j;
            int64_t mac[3]; uint32_t rgbc = nxt() & 0x00FFFFFFu, ir0 = nxt() % 4097u, exp_rgb;
            int ok = 1;
            for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) { L[i][j] = (int16_t)((int32_t)(nxt() % 8193u) - 4096); LR[i][j] = (int16_t)(nxt() % 4097u); }
            for (i = 0; i < 3; i++) { bk[i] = (int32_t)(nxt() % 2000u); fc[i] = (int32_t)(nxt() % 4096u); v0[i] = (int32_t)(nxt() % 8193u) - 4096; }
            psx_gte_write_ctrl(c, 8, (uint16_t)L[0][0] | ((uint32_t)(uint16_t)L[0][1] << 16));
            psx_gte_write_ctrl(c, 9, (uint16_t)L[0][2] | ((uint32_t)(uint16_t)L[1][0] << 16));
            psx_gte_write_ctrl(c, 10, (uint16_t)L[1][1] | ((uint32_t)(uint16_t)L[1][2] << 16));
            psx_gte_write_ctrl(c, 11, (uint16_t)L[2][0] | ((uint32_t)(uint16_t)L[2][1] << 16));
            psx_gte_write_ctrl(c, 12, (uint16_t)L[2][2]);
            psx_gte_write_ctrl(c, 16, (uint16_t)LR[0][0] | ((uint32_t)(uint16_t)LR[0][1] << 16));
            psx_gte_write_ctrl(c, 17, (uint16_t)LR[0][2] | ((uint32_t)(uint16_t)LR[1][0] << 16));
            psx_gte_write_ctrl(c, 18, (uint16_t)LR[1][1] | ((uint32_t)(uint16_t)LR[1][2] << 16));
            psx_gte_write_ctrl(c, 19, (uint16_t)LR[2][0] | ((uint32_t)(uint16_t)LR[2][1] << 16));
            psx_gte_write_ctrl(c, 20, (uint16_t)LR[2][2]);
            for (i = 0; i < 3; i++) { psx_gte_write_ctrl(c, 13u + (uint32_t)i, (uint32_t)bk[i]); psx_gte_write_ctrl(c, 21u + (uint32_t)i, (uint32_t)fc[i]); }
            psx_gte_write_data(c, 0, (uint16_t)v0[0] | ((uint32_t)(uint16_t)v0[1] << 16));
            psx_gte_write_data(c, 1, (uint16_t)v0[2]);
            psx_gte_write_data(c, 6, rgbc | 0x30000000u);
            psx_gte_write_data(c, 8, ir0);
            psx_gte_command(c, 0x0080413u);           /* NCDS sf=1 lm=1 */
            for (i = 0; i < 3; i++) { mac[i] = ((int64_t)L[i][0] * v0[0] + (int64_t)L[i][1] * v0[1] + (int64_t)L[i][2] * v0[2]) >> 12; ir[i] = (int32_t)clampi(mac[i], 0, 0x7FFF); }
            for (i = 0; i < 3; i++) mac[i] = ((int64_t)bk[i] * 4096 + (int64_t)LR[i][0] * ir[0] + (int64_t)LR[i][1] * ir[1] + (int64_t)LR[i][2] * ir[2]) >> 12;
            for (i = 0; i < 3; i++) ir[i] = (int32_t)clampi(mac[i], 0, 0x7FFF);
            for (i = 0; i < 3; i++) {
                int64_t col = (rgbc >> (8 * i)) & 0xFFu, m0 = (col * ir[i]) << 4;
                int64_t t = clampi((((int64_t)fc[i] << 12) - m0) >> 12, -0x8000, 0x7FFF);
                mac[i] = (t * ir0 + m0) >> 12;
            }
            exp_rgb = (uint32_t)clampi(mac[0] >> 4, 0, 255) | ((uint32_t)clampi(mac[1] >> 4, 0, 255) << 8) |
                      ((uint32_t)clampi(mac[2] >> 4, 0, 255) << 16) | 0x30000000u;
            if (psx_gte_read_data(c, 22) != exp_rgb) ok = 0;
            for (i = 0; i < 3; i++) if ((int32_t)psx_gte_read_data(c, 25u + (uint32_t)i) != (int32_t)mac[i]) ok = 0;
            if (!ok) bad_ncds++;
        }
    }
    HS_CHECK(&T, bad_rtps == 0u, "RTPS exact vs reference (4000 vectors, %u bad)", bad_rtps);
    HS_CHECK(&T, bad_rtpt == 0u, "RTPT exact vs reference (4000 triples, %u bad)", bad_rtpt);
    HS_CHECK(&T, bad_ncl == 0u, "NCLIP exact (%u bad)", bad_ncl);
    HS_CHECK(&T, bad_avsz == 0u, "AVSZ3/AVSZ4 exact (%u bad)", bad_avsz);
    HS_CHECK(&T, bad_mv == 0u, "MVMVA exact over mx/v/cv/sf/lm (%u bad)", bad_mv);
    HS_CHECK(&T, bad_ncds == 0u, "NCDS exact light pipeline (%u bad)", bad_ncds);
    /* saturation + FLAG bits */
    {
        gin_t g;
        memset(&g, 0, sizeof(g));
        g.m[0][0] = 4096; g.m[1][1] = 4096; g.m[2][2] = 4096;
        g.tr[0] = 0x9000; g.tr[2] = -50; g.h = 300; g.v[0][0] = 0; g.v[0][2] = 0;
        gte_load(c, &g);
        psx_gte_command(c, 0x0080001u);
        {
            uint32_t f = psx_gte_read_ctrl(c, 31);
            HS_CHECK(&T, (f & (1u << 24)) && (f & (1u << 18)) && (f & (1u << 31)), "IR1 + SZ3 saturation flags + error bit (%08x)", f);
            HS_CHECK(&T, psx_gte_read_data(c, 9) == 0x7FFFu && psx_gte_read_data(c, 19) == 0u, "IR1 clamp / SZ3 clamp");
            HS_CHECK(&T, (f & (1u << 17)) != 0u, "divide overflow flag when H >= 2*SZ3");
        }
        psx_gte_write_data(c, 30, 0xFFF00000u);
        HS_CHECK(&T, psx_gte_read_data(c, 31) == 12u, "LZCR leading ones");
        psx_gte_write_data(c, 30, 0x00010000u);
        HS_CHECK(&T, psx_gte_read_data(c, 31) == 15u, "LZCR leading zeros");
        psx_gte_write_data(c, 28, 0x7FFFu);
        HS_CHECK(&T, psx_gte_read_data(c, 9) == 0xF80u && psx_gte_read_data(c, 29) == 0x7FFFu, "IRGB -> IR / ORGB");
        psx_gte_write_data(c, 9, 0x100u); psx_gte_write_data(c, 10, (uint32_t)-3); psx_gte_write_data(c, 11, 7u);
        psx_gte_command(c, 0x0000028u);           /* SQR sf=0 */
        if ((int32_t)psx_gte_read_data(c, 25) != 0x10000 || psx_gte_read_data(c, 26) != 9u || psx_gte_read_data(c, 9) != 0x7FFFu) bad_misc++;
        psx_gte_write_ctrl(c, 0, 2u); psx_gte_write_ctrl(c, 2, 3u); psx_gte_write_ctrl(c, 4, 5u);
        psx_gte_write_data(c, 9, 7u); psx_gte_write_data(c, 10, 11u); psx_gte_write_data(c, 11, 13u);
        psx_gte_command(c, 0x000000Cu);           /* OP sf=0 */
        if ((int32_t)psx_gte_read_data(c, 25) != 13 * 3 - 11 * 5 || (int32_t)psx_gte_read_data(c, 26) != 7 * 5 - 13 * 2 ||
            (int32_t)psx_gte_read_data(c, 27) != 11 * 2 - 7 * 3) bad_misc++;
        psx_gte_write_data(c, 8, 0x800u); psx_gte_write_data(c, 9, 0x400u); psx_gte_write_data(c, 10, 0x800u); psx_gte_write_data(c, 11, 0xFF0u);
        psx_gte_write_data(c, 6, 0x11000000u);
        psx_gte_command(c, 0x008003Du);           /* GPF sf=1 */
        if (psx_gte_read_data(c, 22) != (0x11000000u | (0x20u) | (0x40u << 8) | (0x7Fu << 16))) bad_misc++;
        HS_CHECK(&T, bad_misc == 0u, "SQR / OP / GPF exact (%u bad)", bad_misc);
        psx_gte_write_ctrl(c, 26, 0x8000u);
        HS_CHECK(&T, psx_gte_read_ctrl(c, 26) == 0xFFFF8000u, "H reads back sign-extended (hardware quirk)");
    }
    printf("GTE: %u checks\n", (T.pass + T.fail) - before);
    free(s);
}

/* ======================= 4. SPU ======================= */
static void park_cpu(hs_psx_t *s)
{
    psx_write32(s, PROG, 0x1000FFFFu); psx_write32(s, PROG + 4u, 0);
    s->cpu.pc = PROG; s->cpu.next_pc = PROG + 4u; s->cpu.cop0[C0_SR] = 0x40000000u;
}
static void spu_w(hs_psx_t *s, uint32_t off, uint32_t v) { psx_write16(s, 0x1F801C00u + off, v); }

static void ref_adpcm(const uint8_t *blk, int16_t *out)
{
    static const int32_t P[5] = { 0, 60, 115, 98, 122 }, N[5] = { 0, 0, -52, -55, -60 };
    int32_t o1 = 0, o2 = 0, i;
    int32_t sh = blk[0] & 15, f = (blk[0] >> 4) & 7;
    if (sh > 12) sh = 9;
    if (f > 4) f = 4;
    for (i = 0; i < 28; i++) {
        int32_t n = (blk[2 + i / 2] >> ((i & 1) * 4)) & 15;
        int32_t v = ((n >= 8 ? n - 16 : n) * 4096) >> sh;
        v += (o1 * P[f] + o2 * N[f] + 32) >> 6;
        v = (int32_t)clampi(v, -32768, 32767);
        o2 = o1; o1 = v; out[i] = (int16_t)v;
    }
}

static void spu_frame(hs_psx_t *s, int16_t *pcm)
{
    O->step_cpu_phase(s, 0, 0); O->commit_phase(s);
    (void)hs_psx_synth_audio_native(s, pcm, HS_PSX_SPU_PER_FRAME);
}

static void test_spu(void)
{
    hs_psx_t *s = mk();
    unsigned before = T.pass + T.fail;
    uint8_t blocks[8][16];
    int16_t pcm[HS_PSX_SPU_PER_FRAME * 2];
    uint32_t b, i, v;
    int ok;
    park_cpu(s);
    for (b = 0; b < 8u; b++) {
        blocks[b][0] = (uint8_t)(((b % 5u) << 4) | (b < 4u ? 0u : 3u + b));
        blocks[b][1] = (uint8_t)(b == 4u ? 4u : (b == 7u ? 3u : 0u));
        for (i = 2; i < 16u; i++) blocks[b][i] = (uint8_t)(nxt() & 0xFFu);
    }
    spu_w(s, 0x1AA, 0xC000); spu_w(s, 0x180, 0x3FFF); spu_w(s, 0x182, 0x3FFF);
    spu_w(s, 0x1A6, 0x2000 / 8);
    for (b = 0; b < 8u; b++) for (i = 0; i < 16u; i += 2) spu_w(s, 0x1A8, (uint32_t)blocks[b][i] | ((uint32_t)blocks[b][i + 1] << 8));
    HS_CHECK(&T, s->spu_ram0[0x2000] == blocks[0][0] && s->spu_ram0[0x2000 + 127] == blocks[7][15], "SPU FIFO upload");
    /* voices 0..3: pitch 0 -> each holds one decoded block, compare with reference */
    for (v = 0; v < 4u; v++) {
        spu_w(s, v * 16u + 4u, 0); spu_w(s, v * 16u + 6u, (0x2000u + v * 16u) / 8u);
        spu_w(s, v * 16u + 8u, 0x00FF); spu_w(s, v * 16u + 10u, 0x0000);
    }
    spu_w(s, 0x188, 0x000F);
    spu_frame(s, pcm);
    ok = 1;
    for (v = 0; v < 4u; v++) {
        int16_t ref[28];
        ref_adpcm(blocks[v], ref);
        for (i = 0; i < 28u; i++) if (s->aud.v[v].buf[i] != ref[i]) ok = 0;
    }
    HS_CHECK(&T, ok, "ADPCM decode (filters 0..3, shifts) == reference");
    spu_w(s, 0x18C, 0x000F); spu_frame(s, pcm); spu_frame(s, pcm);
    /* ADSR: linear attack (shift 20) rising, then decay to sustain, release on KOFF */
    {
        int32_t prev = -1, rises = 0, peak = 0;
        uint32_t f;
        spu_w(s, 0x50, 0x3FFF); spu_w(s, 0x52, 0x0000);           /* voice 5: left only */
        spu_w(s, 0x54, 0x1000); spu_w(s, 0x56, 0x2040 / 8);
        spu_w(s, 0x5E, 0x2040 / 8);
        spu_w(s, 0x58, (uint32_t)((12u << 10) | (0u << 8) | (6u << 4) | 0x9u)); /* attack lin sh12, decay sh6, SL=9 */
        spu_w(s, 0x5A, (uint32_t)((0x1Fu << 8) | (1u << 14) | (1u << 6) | 0x0Au)); /* sustain dec slow, release sh10 */
        spu_w(s, 0x188, 1u << 5);
        for (f = 0; f < 40u; f++) {
            int32_t e;
            spu_frame(s, pcm);
            e = s->aud.v[5].env;
            if (e > prev) rises++;
            if (e > peak) peak = e;
            prev = e;
        }
        HS_CHECK(&T, rises >= 5 && peak > 0x7000 && s->aud.v[5].env_phase >= 2u, "ADSR linear attack ramps up then hands to decay (rises=%d frame-end peak=%d)", rises, peak);
        HS_CHECK(&T, s->aud.v[5].env_phase == 3u && s->aud.v[5].env <= 0x5000 + 0x800, "decay -> sustain level (env=%d)", s->aud.v[5].env);
        {
            int64_t sl = 0, sr = 0;
            for (i = 0; i < HS_PSX_SPU_PER_FRAME; i++) { sl += pcm[2 * i] < 0 ? -pcm[2 * i] : pcm[2 * i]; sr += pcm[2 * i + 1] < 0 ? -pcm[2 * i + 1] : pcm[2 * i + 1]; }
            HS_CHECK(&T, sl > 1000 && sr == 0, "stereo voice volume: left-only voice (L=%lld R=%lld)", (long long)sl, (long long)sr);
        }
        spu_w(s, 0x18C, 1u << 5);
        for (f = 0; f < 60u && s->aud.v[5].on; f++) spu_frame(s, pcm);
        HS_CHECK(&T, !s->aud.v[5].on && s->aud.v[5].env == 0, "release decays to 0 and stops voice (%u frames)", f);
        HS_CHECK(&T, (s->aud.endx & (1u << 5)) != 0u, "ENDX set at loop end");
    }
    /* 24 voices, looping block range, mixed stereo */
    {
        int64_t e = 0; uint32_t on = 0, zc = 0;
        for (v = 0; v < 24u; v++) {
            spu_w(s, v * 16u + 0u, 0x0400); spu_w(s, v * 16u + 2u, v & 1u ? 0x0600 : 0x0200);
            spu_w(s, v * 16u + 4u, 0x0800 + v * 97u); spu_w(s, v * 16u + 6u, 0x2040 / 8); spu_w(s, v * 16u + 14u, 0x2040 / 8);
            spu_w(s, v * 16u + 8u, 0x000F); spu_w(s, v * 16u + 10u, 0x1F00);
        }
        spu_w(s, 0x188, 0xFFFF); spu_w(s, 0x18A, 0x00FF);
        spu_frame(s, pcm); spu_frame(s, pcm);
        for (v = 0; v < 24u; v++) on += s->aud.v[v].on;
        {
            int64_t mean = 0;
            for (i = 0; i < HS_PSX_SPU_PER_FRAME; i++) mean += pcm[2 * i];
            mean /= (int64_t)HS_PSX_SPU_PER_FRAME;
            for (i = 0; i < HS_PSX_SPU_PER_FRAME; i++) {
                int64_t d = pcm[2 * i] - mean;
                e += d * d;
                if (i && ((pcm[2 * i] >= mean) != (pcm[2 * i - 2] >= mean))) zc++;
            }
        }
        HS_CHECK(&T, on == 24u, "24 voices keyed on (%u)", on);
        HS_CHECK(&T, e / HS_PSX_SPU_PER_FRAME > 1000 && zc > 4u, "24-voice mix: AC waveform (variance=%lld, mean crossings=%u)", (long long)(e / HS_PSX_SPU_PER_FRAME), zc);
    }
    printf("SPU: %u checks\n", (T.pass + T.fail) - before);
    free(s);
}

/* ======================= demo helpers ======================= */
static uint8_t *g_exe;
static size_t g_exe_len;

static uint64_t fnv(const void *p, size_t n) { return hs_fnv1a64(HS_FNV64_OFFSET, p, n); }

static hs_psx_t *mk_demo(void)
{
    hs_psx_t *s = mk();
    if (O->load_rom(s, g_exe, g_exe_len) != 0) { printf("load_rom failed\n"); exit(2); }
    return s;
}

/* ======================= 5. zero-mutex snapshots ======================= */
static void test_snapshots(uint32_t *fb)
{
    hs_psx_t *a = mk_demo(), *b = mk_demo();
    unsigned before = T.pass + T.fail;
    static int16_t pcm[HS_PSX_ABI_PER_FRAME * 2];
    uint64_t seq_v[24], seq_a[24], seq_s[24];
    uint32_t f, front_ok = 1, p1_ok = 1, eq = 1;
    static const hs_video_mode_t modes[3] = { HS_VIDEO_PSX_1X, HS_VIDEO_FXAA, HS_VIDEO_SSAA_2X };
    for (f = 0; f < 24u; f++) {
        uint64_t v0, a0, v1, a1, c0, c1, k0, k1;
        uint32_t m;
        O->hash_state(a, &v0, &a0);
        O->step_cpu_phase(a, 0, 0);
        O->hash_state(a, &v1, &a1);
        if (v0 != v1 || a0 != a1) front_ok = 0;          /* Phase 0 never touches FRONT */
        O->commit_phase(a);
        O->hash_state(a, &v1, &a1);
        seq_s[f] = v1 ^ (a1 * 3u);
        c0 = hs_psx_hash_cpu(a); k0 = hs_psx_hash_back(a);
        for (m = 0; m < 3u; m++) O->render_video_phase(a, fb, HS_FB_W, modes[m]);
        seq_v[f] = fnv(fb, (size_t)HS_FB_W * HS_FB_H * 4u);
        O->render_video_phase(a, fb, HS_FB_W, HS_VIDEO_PSX_1X);
        seq_v[f] ^= fnv(fb, (size_t)HS_FB_W * HS_FB_H * 4u) * 7u;
        (void)O->synth_audio_phase(a, pcm, HS_PSX_ABI_PER_FRAME);
        seq_a[f] = fnv(pcm, sizeof(pcm));
        c1 = hs_psx_hash_cpu(a); k1 = hs_psx_hash_back(a);
        if (c0 != c1 || k0 != k1) p1_ok = 0;              /* Phase 1/2 never touch CPU domain / BACK */
    }
    HS_CHECK(&T, front_ok, "Phase 0 never mutates the published FRONT slot (24 frames)");
    HS_CHECK(&T, p1_ok, "Phase 1 (1x/FXAA/2x) + Phase 2 never mutate CPU state or BACK slot");
    /* pipelined: Core0 steps N+1 while Core1/Core2 present N */
    O->step_cpu_phase(b, 0, 0);
    O->commit_phase(b);
    for (f = 0; f < 24u; f++) {
        uint64_t vh, ah, hv, haud;
        uint32_t m;
        O->hash_state(b, &vh, &ah);
        O->step_cpu_phase(b, 0, 0);               /* frame f+1 -> BACK */
        for (m = 0; m < 3u; m++) O->render_video_phase(b, fb, HS_FB_W, modes[m]);  /* frame f from FRONT */
        hv = fnv(fb, (size_t)HS_FB_W * HS_FB_H * 4u);
        O->render_video_phase(b, fb, HS_FB_W, HS_VIDEO_PSX_1X);
        hv ^= fnv(fb, (size_t)HS_FB_W * HS_FB_H * 4u) * 7u;
        (void)O->synth_audio_phase(b, pcm, HS_PSX_ABI_PER_FRAME);
        haud = fnv(pcm, sizeof(pcm));
        if (hv != seq_v[f] || haud != seq_a[f] || (vh ^ (ah * 3u)) != seq_s[f]) eq = 0;
        O->commit_phase(b);
    }
    HS_CHECK(&T, eq, "pipelined N+1/N order == sequential (video, audio, snapshot hashes, 24 frames)");
    printf("HASH snapshot_seq_final_video %016llx\n", (unsigned long long)seq_v[23]);
    printf("HASH snapshot_seq_final_audio %016llx\n", (unsigned long long)seq_a[23]);
    printf("SNAPSHOT: %u checks\n", (T.pass + T.fail) - before);
    free(a); free(b);
}

/* ======================= 6. demo, PNGs, RMS, timings ======================= */
static uint32_t isqrt64(uint64_t v) { uint64_t r = 0, b = 1ull << 62; while (b > v) b >>= 2; while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; } return (uint32_t)r; }
static int32_t luma(uint32_t p) { return (int32_t)((((p >> 16) & 255u) * 77u + ((p >> 8) & 255u) * 150u + (p & 255u) * 29u) >> 8); }

static const char *k_font_rows[] = {   /* 3x5 glyphs for labels */
    "1", "010110010010111", "2", "111001111100111", "4", "101101111001001", "8", "111101111101111",
    "X", "101101010101101", "F", "111100110100100", "A", "010101111101101", " ", "000000000000000",
    "U", "101101101101111", "L", "100100100100111", "N", "110101101101101", "T", "111010010010010",
    "I", "111010010010111", "V", "101101101101010", "E", "111100110100111", "S", "111100111001111",
    "+", "000010111010000", NULL
};
static void draw_text(uint32_t *img, uint32_t W, uint32_t x0, uint32_t y0, const char *t, uint32_t sc, uint32_t col)
{
    uint32_t ci, k, gx, gy, a, b;
    for (ci = 0; t[ci]; ci++) {
        const char *g = NULL;
        for (k = 0; k_font_rows[k]; k += 2) if (k_font_rows[k][0] == t[ci]) { g = k_font_rows[k + 1]; break; }
        if (!g) continue;
        for (gy = 0; gy < 5u; gy++) for (gx = 0; gx < 3u; gx++) if (g[gy * 3u + gx] == '1')
            for (a = 0; a < sc; a++) for (b = 0; b < sc; b++)
                img[(size_t)(y0 + gy * sc + a) * W + x0 + ci * 4u * sc + gx * sc + b] = col;
    }
}

static int write_wav(const char *path, const int16_t *pcm, uint32_t frames)
{
    FILE *f = fopen(path, "wb");
    uint8_t h[44];
    uint32_t data = frames * 4u, i;
    if (!f) return -1;
    memcpy(h, "RIFF", 4); h[4] = (uint8_t)(data + 36u); h[5] = (uint8_t)((data + 36u) >> 8); h[6] = (uint8_t)((data + 36u) >> 16); h[7] = (uint8_t)((data + 36u) >> 24);
    memcpy(h + 8, "WAVEfmt ", 8);
    h[16] = 16; h[17] = h[18] = h[19] = 0; h[20] = 1; h[21] = 0; h[22] = 2; h[23] = 0;
    h[24] = (uint8_t)HS_AUDIO_RATE; h[25] = (uint8_t)(HS_AUDIO_RATE >> 8); h[26] = (uint8_t)(HS_AUDIO_RATE >> 16); h[27] = 0;
    { uint32_t br = HS_AUDIO_RATE * 4u; h[28] = (uint8_t)br; h[29] = (uint8_t)(br >> 8); h[30] = (uint8_t)(br >> 16); h[31] = (uint8_t)(br >> 24); }
    h[32] = 4; h[33] = 0; h[34] = 16; h[35] = 0;
    memcpy(h + 36, "data", 4); h[40] = (uint8_t)data; h[41] = (uint8_t)(data >> 8); h[42] = (uint8_t)(data >> 16); h[43] = (uint8_t)(data >> 24);
    fwrite(h, 1, 44, f);
    for (i = 0; i < frames * 2u; i++) { uint8_t b2[2]; b2[0] = (uint8_t)pcm[i]; b2[1] = (uint8_t)((uint16_t)pcm[i] >> 8); fwrite(b2, 1, 2, f); }
    fclose(f);
    return 0;
}

/* Goertzel power ratio (host-side analysis only; floats never enter the core) */
static double tone_power(const int16_t *pcm, uint32_t n, double hz)
{
    double w = 2.0 * 3.14159265358979323846 * hz / (double)HS_AUDIO_RATE, cw = 2.0 * __builtin_cos(w), s0, s1 = 0, s2 = 0;
    uint32_t i;
    for (i = 0; i < n; i++) { s0 = (double)pcm[2 * i] + cw * s1 - s2; s2 = s1; s1 = s0; }
    return (s1 * s1 + s2 * s2 - cw * s1 * s2) / ((double)n * n);
}

typedef struct cnt_be { uint32_t frames, prims, tris, rects, lines, fills; const uint16_t *vram; } cnt_be_t;
static int be_begin(void *u, const psx_gpu_cmdlist_t *cl, uint32_t ss) { cnt_be_t *b = (cnt_be_t *)u; (void)ss; b->frames++; b->vram = cl->vram; return 0; }
static void be_draw(void *u, const psx_gpu_prim_t *p, uint32_t i)
{
    cnt_be_t *b = (cnt_be_t *)u; (void)i;
    b->prims++;
    if (p->type == PSX_PRIM_TRI) b->tris++;
    else if (p->type == PSX_PRIM_RECT) b->rects++;
    else if (p->type == PSX_PRIM_LINE) b->lines++;
    else if (p->type == PSX_PRIM_FILL) b->fills++;
}
static void be_resolve(void *u, uint32_t *fb, size_t stride) { uint32_t y, x; (void)u; for (y = 0; y < HS_FB_H; y++) for (x = 0; x < HS_FB_W; x++) fb[y * stride + x] = 0x00102030u; }

#define DEMO_FRAMES 120u
#define SHOT_FRAME  60u

static void test_demo(uint32_t *fb)
{
    static const hs_video_mode_t modes[5] = { HS_VIDEO_PSX_1X, HS_VIDEO_FXAA, HS_VIDEO_SSAA_2X, HS_VIDEO_SSAA_4X, HS_VIDEO_SSAA_8X };
    static const char *names[5] = { "1x", "fxaa", "2x", "4x", "8x" };
    static const uint32_t reps[5] = { 20, 20, 6, 3, 2 };
    static int16_t pcm_all[DEMO_FRAMES * HS_PSX_ABI_PER_FRAME * 2];
    uint32_t *shot[5], *cmp;
    hs_psx_t *s = mk_demo();
    unsigned before = T.pass + T.fail;
    uint64_t t_p0 = 0, t_p2 = 0, t0, vh_prev = 0, video_chain = HS_FNV64_OFFSET, audio_chain = HS_FNV64_OFFSET;
    uint32_t f, m, changed = 0, min_prims = 0xFFFFFFFFu, max_prims = 0, tty_len;
    char path[700];
    const char *tty;
    for (m = 0; m < 5u; m++) shot[m] = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    for (f = 0; f < DEMO_FRAMES; f++) {
        uint64_t vh, ah;
        t0 = hs_host_now_ns(); O->step_cpu_phase(s, 0, 0); t_p0 += hs_host_now_ns() - t0;
        O->commit_phase(s);
        O->hash_state(s, &vh, &ah);
        if (f && vh != vh_prev) changed++;
        vh_prev = vh;
        video_chain = hs_fnv1a64(video_chain, &vh, 8);
        {
            uint32_t np = hs_psx_prim_count(s);
            if (np < min_prims) min_prims = np;
            if (np > max_prims) max_prims = np;
        }
        t0 = hs_host_now_ns();
        (void)O->synth_audio_phase(s, pcm_all + (size_t)f * HS_PSX_ABI_PER_FRAME * 2u, HS_PSX_ABI_PER_FRAME);
        t_p2 += hs_host_now_ns() - t0;
        if (f == SHOT_FRAME) {
            for (m = 0; m < 5u; m++) {
                uint32_t r;
                uint64_t tt = hs_host_now_ns();
                for (r = 0; r < reps[m]; r++) O->render_video_phase(s, shot[m], HS_FB_W, modes[m]);
                tt = (hs_host_now_ns() - tt) / reps[m];
                printf("TIME phase1_gpu_%s_us %llu\n", names[m], (unsigned long long)(tt / 1000u));
            }
            {   /* +FXAA on top of 4x (optional toggle) */
                uint64_t tt = hs_host_now_ns();
                hs_psx_set_post_fxaa(s, 1);
                O->render_video_phase(s, fb, HS_FB_W, HS_VIDEO_SSAA_4X);
                hs_psx_set_post_fxaa(s, 0);
                printf("TIME phase1_gpu_4x+fxaa_us %llu\n", (unsigned long long)((hs_host_now_ns() - tt) / 1000u));
                printf("HASH demo_4x_plus_fxaa %016llx\n", (unsigned long long)fnv(fb, (size_t)HS_FB_W * HS_FB_H * 4u));
            }
            {   /* backend hook consumes the identical frozen command list */
                cnt_be_t cb; psx_gpu_backend_t be; psx_gpu_cmdlist_t cl;
                memset(&cb, 0, sizeof(cb));
                be.name = "count"; be.user = &cb; be.begin_frame = be_begin; be.draw = be_draw; be.resolve = be_resolve;
                hs_psx_export_cmdlist(s, &cl);
                HS_CHECK(&T, hs_psx_render_backend(s, &be, HS_VIDEO_SSAA_4X, fb, HS_FB_W) == 0 && cb.prims == cl.count &&
                         cb.vram == cl.vram && cl.disp_w == 320u && cl.disp_h == 240u,
                         "backend hook sees the frozen list (%u prims: %u tri %u rect %u line %u fill)", cb.prims, cb.tris, cb.rects, cb.lines, cb.fills);
            }
        }
    }
    tty = hs_psx_tty(s, &tty_len);
    HS_CHECK(&T, strstr(tty, "hs-psx demo: R3000A+GTE+DMA2/6+SPU, frame 0") != NULL, "HLE printf from MIPS code: \"%.48s\"", tty);
    HS_CHECK(&T, psx_read32(s, 0x80070000u) == DEMO_FRAMES, "MIPS frame counter == %u (got %u)", DEMO_FRAMES, psx_read32(s, 0x80070000u));
    HS_CHECK(&T, changed >= DEMO_FRAMES - 4u, "rotation changes the frame (%u/%u distinct)", changed, DEMO_FRAMES - 1u);
    HS_CHECK(&T, min_prims >= 100u && s->hle.unimpl_calls == 0u && s->bad_access == 0u,
             "prims/frame %u..%u, unimplemented HLE calls %u, bad bus accesses %u", min_prims, max_prims, s->hle.unimpl_calls, s->bad_access);
    printf("TIME phase0_cpu_gte_us %llu\n", (unsigned long long)(t_p0 / DEMO_FRAMES / 1000u));
    printf("TIME phase2_spu_us %llu\n", (unsigned long long)(t_p2 / DEMO_FRAMES / 1000u));
    printf("HASH demo_video_chain_120 %016llx\n", (unsigned long long)video_chain);
    audio_chain = fnv(pcm_all, sizeof(pcm_all));
    printf("HASH demo_audio_120 %016llx\n", (unsigned long long)audio_chain);
    printf("HASH demo_cpu_state_120 %016llx\n", (unsigned long long)hs_psx_hash_cpu(s));
    /* PNGs + edge-to-edge check */
    for (m = 0; m < 5u; m++) {
        uint32_t k, blk = 0;
        for (k = 0; k < HS_FB_W; k++) { blk += (shot[m][k] & 0xFFFFFFu) == 0u; blk += (shot[m][(HS_FB_H - 1u) * HS_FB_W + k] & 0xFFFFFFu) == 0u; }
        for (k = 0; k < HS_FB_H; k++) { blk += (shot[m][k * HS_FB_W] & 0xFFFFFFu) == 0u; blk += (shot[m][k * HS_FB_W + HS_FB_W - 1u] & 0xFFFFFFu) == 0u; }
        HS_CHECK(&T, blk == 0u, "%s: full 640x480 edge-to-edge, 0 black border pixels (%u)", names[m], blk);
        snprintf(path, sizeof(path), "%s/psx_3d_%s_640x480.png", g_out, names[m]);
        HS_CHECK(&T, hs_png_write_xrgb(path, shot[m], HS_FB_W, HS_FB_H, HS_FB_W) == 0, "write %s", path);
        printf("HASH demo_frame60_%s %016llx\n", names[m], (unsigned long long)fnv(shot[m], (size_t)HS_FB_W * HS_FB_H * 4u));
    }
    /* RMS vs 8x: all pixels and edge pixels (8x-reference gradient >= 24) */
    {
        uint64_t rms_all[4], rms_edge[4];
        uint32_t edge_px = 0, x, y;
        uint8_t *mask = (uint8_t *)calloc((size_t)HS_FB_W * HS_FB_H, 1);
        const uint32_t *ref = shot[4];
        for (y = 1; y + 1u < HS_FB_H; y++) for (x = 1; x + 1u < HS_FB_W; x++) {
            int32_t gx = luma(ref[y * HS_FB_W + x + 1]) - luma(ref[y * HS_FB_W + x - 1]);
            int32_t gy = luma(ref[(y + 1) * HS_FB_W + x]) - luma(ref[(y - 1) * HS_FB_W + x]);
            if (gx < 0) gx = -gx;
            if (gy < 0) gy = -gy;
            if (gx >= 24 || gy >= 24) { mask[y * HS_FB_W + x] = 1; edge_px++; }
        }
        for (m = 0; m < 4u; m++) {
            uint64_t sa = 0, se = 0;
            uint32_t i;
            for (i = 0; i < HS_FB_W * HS_FB_H; i++) {
                uint32_t a = shot[m][i], b2 = ref[i], k;
                uint64_t d2 = 0;
                for (k = 0; k < 3u; k++) { int32_t d = (int32_t)((a >> (8u * k)) & 255u) - (int32_t)((b2 >> (8u * k)) & 255u); d2 += (uint64_t)(d * d); }
                sa += d2;
                if (mask[i]) se += d2;
            }
            rms_all[m] = isqrt64(sa * 1000000ull / ((uint64_t)HS_FB_W * HS_FB_H * 3u));
            rms_edge[m] = edge_px ? isqrt64(se * 1000000ull / ((uint64_t)edge_px * 3u)) : 0;
        }
        for (m = 0; m < 4u; m++)
            printf("RMS vs8x mode=%-4s all=%llu.%03llu edge=%llu.%03llu edge_improvement_vs_1x=%lld.%01lld%%\n", names[m],
                   (unsigned long long)(rms_all[m] / 1000u), (unsigned long long)(rms_all[m] % 1000u),
                   (unsigned long long)(rms_edge[m] / 1000u), (unsigned long long)(rms_edge[m] % 1000u),
                   (long long)((int64_t)(rms_edge[0] - rms_edge[m]) * 1000 / (int64_t)rms_edge[0] / 10),
                   (long long)(((int64_t)(rms_edge[0] - rms_edge[m]) * 1000 / (int64_t)rms_edge[0]) % 10));
        printf("RMS edge_pixels=%u\n", edge_px);
        HS_CHECK(&T, rms_edge[2] < rms_edge[0] && rms_edge[3] < rms_edge[2], "edge RMS vs 8x decreases 1x > 2x > 4x");
        HS_CHECK(&T, rms_edge[1] < rms_edge[0], "FXAA lowers edge RMS vs 8x relative to 1x");
        free(mask);
    }
    /* comparison sheet: 5 zoomed crops (4x) + full 8x + full 1x */
    {
        const uint32_t CW = 64, CH = 48, Z = 4, W = 1280, HDR = 24, H = HDR + CH * Z + 8 + HS_FB_H;
        uint32_t bx = 0, by = 0, x, y, k;
        uint64_t best = 0;
        cmp = (uint32_t *)calloc((size_t)W * H, 4);
        for (y = 0; y + CH <= HS_FB_H; y += 16) for (x = 0; x + CW <= HS_FB_W; x += 16) {
            uint64_t e = 0; uint32_t i, j;
            for (j = 0; j < CH; j++) for (i = 0; i < CW; i++) {
                int32_t d = luma(shot[0][(y + j) * HS_FB_W + x + i]) - luma(shot[4][(y + j) * HS_FB_W + x + i]);
                e += (uint64_t)(d < 0 ? -d : d);
            }
            if (e > best) { best = e; bx = x; by = y; }
        }
        for (k = 0; k < W * H; k++) cmp[k] = 0x00181818u;
        for (m = 0; m < 5u; m++) {
            for (y = 0; y < CH * Z; y++) for (x = 0; x < CW * Z; x++)
                cmp[(size_t)(HDR + y) * W + m * 256u + x] = shot[m][(by + y / Z) * HS_FB_W + bx + x / Z];
        }
        draw_text(cmp, W, 8, 4, "1X", 3, 0xFFFFFFu);
        draw_text(cmp, W, 256 + 8, 4, "FXAA", 3, 0xFFFFFFu);
        draw_text(cmp, W, 512 + 8, 4, "2X SSAA", 3, 0xFFFFFFu);
        draw_text(cmp, W, 768 + 8, 4, "4X SSAA", 3, 0xFFFFFFu);
        draw_text(cmp, W, 1024 + 8, 4, "8X SSAA", 3, 0xFFFFFFu);
        for (y = 0; y < HS_FB_H; y++) for (x = 0; x < HS_FB_W; x++) {
            cmp[(size_t)(HDR + CH * Z + 8 + y) * W + x] = shot[4][y * HS_FB_W + x];
            cmp[(size_t)(HDR + CH * Z + 8 + y) * W + 640 + x] = shot[0][y * HS_FB_W + x];
        }
        /* crop outline on both full frames */
        for (k = 0; k <= CW; k++) for (m = 0; m < 2u; m++) {
            uint32_t ox = m * 640u, oy = HDR + CH * Z + 8;
            cmp[(size_t)(oy + by) * W + ox + bx + (k < CW ? k : CW - 1)] = 0xFFFF00u;
            cmp[(size_t)(oy + by + CH - 1) * W + ox + bx + (k < CW ? k : CW - 1)] = 0xFFFF00u;
        }
        for (k = 0; k < CH; k++) for (m = 0; m < 2u; m++) {
            uint32_t ox = m * 640u, oy = HDR + CH * Z + 8;
            cmp[(size_t)(oy + by + k) * W + ox + bx] = 0xFFFF00u;
            cmp[(size_t)(oy + by + k) * W + ox + bx + CW - 1] = 0xFFFF00u;
        }
        draw_text(cmp, W, 8, HDR + CH * Z + 8 + 460, "8X FULL", 3, 0xFFFF00u);
        draw_text(cmp, W, 648, HDR + CH * Z + 8 + 460, "1X FULL", 3, 0xFFFF00u);
        snprintf(path, sizeof(path), "%s/psx_ssaa_compare.png", g_out);
        HS_CHECK(&T, hs_png_write_xrgb(path, cmp, W, H, W) == 0, "write %s (crop %u,%u)", path, bx, by);
        printf("HASH demo_compare_sheet %016llx\n", (unsigned long long)fnv(cmp, (size_t)W * H * 4u));
        free(cmp);
    }
    /* SPU chord analysis: C4 / E4 / G4 vs off-chord probes */
    {
        uint32_t n = DEMO_FRAMES * HS_PSX_ABI_PER_FRAME, i, half = n / 2u;
        const int16_t *tail = pcm_all + (size_t)half * 2u;
        double pc = tone_power(tail, half, 261.63), pe = tone_power(tail, half, 329.63), pg = tone_power(tail, half, 392.00);
        double off = (tone_power(tail, half, 293.66) + tone_power(tail, half, 440.0) + tone_power(tail, half, 200.0)) / 3.0;
        int64_t e = 0; int32_t peak = 0;
        for (i = 0; i < n; i++) { int32_t v = pcm_all[2 * i]; e += (int64_t)v * v; if (v < 0) v = -v; if (v > peak) peak = v; }
        printf("AUDIO rms=%u peak=%d C4/off=%.1f E4/off=%.1f G4/off=%.1f\n", isqrt64((uint64_t)(e / n)), peak,
               pc / (off + 1e-9), pe / (off + 1e-9), pg / (off + 1e-9));
        HS_CHECK(&T, isqrt64((uint64_t)(e / n)) > 500u && peak < 32767, "audible, unclipped SPU output");
        HS_CHECK(&T, pc > 20.0 * off && pe > 20.0 * off && pg > 20.0 * off, "C-E-G chord tones dominate off-chord probes");
        snprintf(path, sizeof(path), "%s/psx_chord_48k.wav", g_out);
        HS_CHECK(&T, write_wav(path, pcm_all, n) == 0, "write %s", path);
    }
    printf("DEMO: %u checks\n", (T.pass + T.fail) - before);
    for (m = 0; m < 5u; m++) free(shot[m]);
    free(s);
}

/* ======================= torture: random instruction stream ======================= */
static void test_torture(void)
{
    hs_psx_t *s = mk();
    uint32_t i;
    unsigned before = T.pass + T.fail;
    for (i = 0; i < 16384u; i++) psx_write32(s, 0x80010000u + 4u * i, nxt());
    s->cpu.pc = 0x80010000u; s->cpu.next_pc = 0x80010004u; s->cpu.cop0[C0_SR] = 0x40000000u;
    for (i = 0; i < 4u; i++) { O->step_cpu_phase(s, 0, 0); O->commit_phase(s); }
    HS_CHECK(&T, s->cpu.r[0] == 0u && s->frame == 4u, "random-instruction torture: 4 frames, r0 stays 0 (cycles=%llu)",
             (unsigned long long)s->cpu.cycles);
    printf("HASH torture_cpu %016llx\n", (unsigned long long)hs_psx_hash_cpu(s));
    printf("TORTURE: %u checks\n", (T.pass + T.fail) - before);
    free(s);
}

int main(int argc, char **argv)
{
    static uint32_t fb[HS_FB_W * HS_FB_H];
    int i;
    for (i = 1; i < argc; i++) if (strncmp(argv[i], "--out=", 6) == 0) snprintf(g_out, sizeof(g_out), "%s", argv[i] + 6);
    g_exe = (uint8_t *)malloc(1u << 20);
    g_exe_len = hs_read_file("tests/psx_3d_demo.exe", g_exe, 1u << 20);
    printf("hs-psx test suite (state %zu bytes, demo exe %zu bytes)\n", O->state_size(), g_exe_len);
    if (g_exe_len < 0x800u) { printf("missing tests/psx_3d_demo.exe\n"); return 2; }
    test_cpu();
    test_bus();
    test_gte();
    test_spu();
    test_torture();
    test_snapshots(fb);
    test_demo(fb);
    printf("RESULT %u/%u passed\n", T.pass, T.pass + T.fail);
    free(g_exe);
    return T.fail ? 1 : 0;
}
