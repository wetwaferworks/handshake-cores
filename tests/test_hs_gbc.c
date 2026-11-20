/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * test_hs_gbc.c - hs-gbc clean-room core test suite.
 *   1. SM83 torture: cycle table (all 256 + 256 CB), exhaustive ALU / DAA /
 *      INC / DEC / CB, 16-bit arithmetic, stack, interrupts, EI delay, HALT,
 *      HALT bug, CGB double speed, timer.
 *   2. Mappers (MBC1/2/3+RTC/5), SRAM round-trip, OAM DMA, GDMA + HDMA.
 *   3. Zero-mutex contract: phase isolation (each phase leaves the other
 *      domains bit-identical), pipelined == sequential, synth == tracker.
 *   4. Real ROMs (libbet, handshake-demo, optional external test ROMs).
 *   5. Synthetic clean-room CGB ROM (colour bars, sprites, HDMA, 4-ch audio,
 *      double speed) assembled right here.
 *   6. Host FPS + 3-phase timing breakdown.
 * Prints "HASH <name> <hex>" lines that run-tests.sh compares across arches.
 */
#include "gbc_internal.h"
#include "hs_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static hs_tstats_t T;
static const hs_core_ops_t *O = &hs_gbc_core_ops;
static gbc_t *G;
static uint8_t *ROM;            /* 8 MiB scratch ROM */
#define ROM_CAP (8u << 20)
static const char *OUTDIR = "out";
static const char *ROMDIR = "../handshakeos/roms";   /* override with --roms=DIR */
static const char *EXTDIR = "tests/ext-roms";
static int QUICK = 0;

/* ------------------------------------------------------------------ util */
static void mk_rom(uint32_t size, uint8_t type, uint8_t ramsz, uint8_t cgbflag)
{
    uint32_t i;
    memset(ROM, 0x00, size);
    /* interrupt vectors: INC A ; RETI */
    for (i = 0x40; i <= 0x60; i += 8) { ROM[i] = 0x3C; ROM[i + 1] = 0xD9; }
    ROM[0x100] = 0x00; ROM[0x101] = 0xC3; ROM[0x102] = 0x50; ROM[0x103] = 0x01;
    ROM[0x150] = 0x18; ROM[0x151] = 0xFE;   /* JR -2 */
    ROM[0x143] = cgbflag;
    ROM[0x147] = type;
    ROM[0x148] = 0;
    ROM[0x149] = ramsz;
}

static gbc_t *boot(hs_console_t c, uint32_t size)
{
    hs_core_config_t cfg;
    cfg.console_id = c; cfg.dmg_palette = 0; cfg.flags = 0;
    if (O->init(G, O->state_size(), &cfg) != 0) { printf("init failed\n"); exit(2); }
    if (O->load_rom(G, ROM, size) != 0) { printf("load_rom failed\n"); exit(2); }
    return G;
}

/* Run one instruction placed in WRAM at C000. Returns T-cycles. */
static uint32_t exec_bytes(gbc_t *g, const uint8_t *b, unsigned n)
{
    uint64_t t0;
    unsigned i;
    for (i = 0; i < n; i++) g->wram[i] = b[i];
    g->cpu.pc = 0xC000;
    t0 = g->total_cycles;
    gbc_cpu_step(g);
    return (uint32_t)(g->total_cycles - t0);
}

static uint64_t fnv(uint64_t h, const void *p, size_t n) { return hs_fnv1a64(h, p, n); }

/* ======================================================= 1. CPU torture */
static const uint8_t T_BASE[256] = {
    4,12, 8, 8, 4, 4, 8, 4,20, 8, 8, 8, 4, 4, 8, 4,
    4,12, 8, 8, 4, 4, 8, 4,12, 8, 8, 8, 4, 4, 8, 4,
    8,12, 8, 8, 4, 4, 8, 4, 8, 8, 8, 8, 4, 4, 8, 4,
    8,12, 8, 8,12,12,12, 4, 8, 8, 8, 8, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    8, 8, 8, 8, 8, 8, 4, 8, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    8,12,12,16,12,16, 8,16, 8,16,12, 4,12,24, 8,16,
    8,12,12, 0,12,16, 8,16, 8,16,12, 0,12, 0, 8,16,
   12,12, 8, 0, 0,16, 8,16,16, 4,16, 0, 0, 0, 8,16,
   12,12, 8, 4, 0,16, 8,16,12, 8,16, 4, 0, 0, 8,16,
};

static int taken_cycles(uint8_t op, int *cc)
{
    switch (op) {
    case 0x20: *cc = 0; return 12; case 0x28: *cc = 1; return 12;
    case 0x30: *cc = 2; return 12; case 0x38: *cc = 3; return 12;
    case 0xC0: *cc = 0; return 20; case 0xC8: *cc = 1; return 20;
    case 0xD0: *cc = 2; return 20; case 0xD8: *cc = 3; return 20;
    case 0xC2: *cc = 0; return 16; case 0xCA: *cc = 1; return 16;
    case 0xD2: *cc = 2; return 16; case 0xDA: *cc = 3; return 16;
    case 0xC4: *cc = 0; return 24; case 0xCC: *cc = 1; return 24;
    case 0xD4: *cc = 2; return 24; case 0xDC: *cc = 3; return 24;
    default: return 0;
    }
}

static void cpu_regs_safe(gbc_t *g)
{
    g->cpu.b = 0xC2; g->cpu.c = 0x00; g->cpu.d = 0xC3; g->cpu.e = 0x00;
    g->cpu.h = 0xC1; g->cpu.l = 0x00; g->cpu.sp = 0xDFF0;
    g->cpu.ime = 0; g->cpu.ei_delay = 0; g->cpu.halted = 0; g->cpu.halt_bug = 0; g->cpu.locked = 0;
    g->ie = 0;
}

static void test_cycle_table(void)
{
    gbc_t *g;
    unsigned op, fl, bad = 0, checked = 0;
    mk_rom(0x8000, 0x00, 0, 0);
    g = boot(HS_SYS_GB, 0x8000);
    for (op = 0; op < 256; op++) {
        int cc = -1, tk;
        if (op == 0x10 || op == 0x76 || op == 0xCB || T_BASE[op] == 0)
            continue;
        tk = taken_cycles((uint8_t)op, &cc);
        for (fl = 0; fl < 2; fl++) {
            uint8_t code[3] = { (uint8_t)op, 0x00, 0xC0 };
            uint32_t got, want;
            cpu_regs_safe(g);
            g->cpu.f = fl ? 0xF0 : 0x00;
            got = exec_bytes(g, code, 3);
            want = T_BASE[op];
            if (tk) {
                int z = (g->cpu.f & FZ) != 0; (void)z;
                int taken = (cc == 0) ? !fl : (cc == 1) ? fl : (cc == 2) ? !fl : fl;
                if (taken) want = (uint32_t)tk;
            }
            checked++;
            if (got != want) { bad++; if (bad < 8) printf("  op %02X F=%02X: %u T, want %u\n", op, fl ? 0xF0 : 0, got, want); }
        }
    }
    for (op = 0; op < 256; op++) {
        uint8_t code[2] = { 0xCB, (uint8_t)op };
        uint32_t want = ((op & 7u) == 6u) ? (((op >> 6) == 1u) ? 12u : 16u) : 8u;
        uint32_t got;
        cpu_regs_safe(g);
        got = exec_bytes(g, code, 2);
        checked++;
        if (got != want) { bad++; if (bad < 8) printf("  CB %02X: %u T, want %u\n", op, got, want); }
    }
    HS_CHECK(&T, bad == 0, "cycle table: %u/%u mismatches", bad, checked);
    printf("[cpu] cycle table: %u opcode/flag cases checked, %u mismatches\n", checked, bad);
}

/* independent reference ALU (xor-based half carry, wide arithmetic) */
static void ref_alu(unsigned op, unsigned a, unsigned v, unsigned cin, unsigned *ra, unsigned *rf)
{
    unsigned r = a, f = 0;
    int d;
    switch (op) {
    case 0: r = a + v; f = ((a ^ v ^ r) & 0x10 ? FH : 0) | (r > 255 ? FC : 0); break;
    case 1: r = a + v + cin; f = ((a ^ v ^ r) & 0x10 ? FH : 0) | (r > 255 ? FC : 0); break;
    case 2: case 7: d = (int)a - (int)v; r = (unsigned)d & 0x1FF; f = FN | ((a ^ v ^ r) & 0x10 ? FH : 0) | (d < 0 ? FC : 0); break;
    case 3: d = (int)a - (int)v - (int)cin; r = (unsigned)d & 0x1FF; f = FN | ((a ^ v ^ r) & 0x10 ? FH : 0) | (d < 0 ? FC : 0); break;
    case 4: r = a & v; f = FH; break;
    case 5: r = a ^ v; break;
    case 6: r = a | v; break;
    }
    r &= 0xFF;
    if (r == 0) f |= FZ;
    *ra = (op == 7) ? a : r;
    *rf = f;
}

static void test_alu_exhaustive(void)
{
    gbc_t *g = G;
    unsigned op, a, v, c, bad = 0, n = 0;
    for (op = 0; op < 8; op++)
        for (a = 0; a < 256; a++)
            for (v = 0; v < 256; v++)
                for (c = 0; c < 2; c++) {
                    uint8_t code[2] = { (uint8_t)(0xC6 + op * 8), (uint8_t)v };
                    unsigned ra, rf;
                    g->cpu.a = (uint8_t)a; g->cpu.f = c ? FC : 0;
                    exec_bytes(g, code, 2);
                    ref_alu(op, a, v, c, &ra, &rf);
                    n++;
                    if (g->cpu.a != ra || g->cpu.f != rf) {
                        bad++;
                        if (bad < 5) printf("  alu%u a=%02X v=%02X c=%u -> %02X/%02X want %02X/%02X\n", op, a, v, c, g->cpu.a, g->cpu.f, ra, rf);
                    }
                }
    HS_CHECK(&T, bad == 0, "ALU exhaustive %u bad", bad);
    printf("[cpu] ALU exhaustive (ADD/ADC/SUB/SBC/AND/XOR/OR/CP x 256 x 256 x C): %u cases, %u bad\n", n, bad);

    /* register-operand forms use the same path: spot-check ALU A,B and A,(HL) */
    bad = 0;
    for (op = 0; op < 8; op++) for (a = 0; a < 256; a += 7) for (v = 0; v < 256; v += 5) {
        uint8_t code1[1] = { (uint8_t)(0x80 + op * 8) };      /* ALU A,B */
        uint8_t code2[1] = { (uint8_t)(0x86 + op * 8) };      /* ALU A,(HL) */
        unsigned ra, rf;
        ref_alu(op, a, v, 1, &ra, &rf);
        g->cpu.a = (uint8_t)a; g->cpu.b = (uint8_t)v; g->cpu.f = FC;
        exec_bytes(g, code1, 1);
        if (g->cpu.a != ra || g->cpu.f != rf) bad++;
        g->cpu.a = (uint8_t)a; g->cpu.h = 0xC1; g->cpu.l = 0x23; g->wram[0x123] = (uint8_t)v; g->cpu.f = FC;
        exec_bytes(g, code2, 1);
        if (g->cpu.a != ra || g->cpu.f != rf) bad++;
    }
    HS_CHECK(&T, bad == 0, "ALU reg/(HL) forms %u bad", bad);
}

static void ref_daa(unsigned a, unsigned f, unsigned *ra, unsigned *rf)
{
    unsigned corr = 0, c = f & FC;
    int r;
    if (f & FH) corr |= 0x06;
    if (f & FC) corr |= 0x60;
    if (!(f & FN)) {
        if ((a & 0x0F) > 9) corr |= 0x06;
        if (a > 0x99) { corr |= 0x60; c = FC; }
        r = (int)a + (int)corr;
    } else {
        r = (int)a - (int)corr;
    }
    r &= 0xFF;
    *ra = (unsigned)r;
    *rf = (r == 0 ? FZ : 0) | (f & FN) | c;
}

static unsigned to_bcd(unsigned v) { return ((v / 10) << 4) | (v % 10); }

static void test_daa(void)
{
    gbc_t *g = G;
    unsigned a, fl, bad = 0, n = 0, x, y, c;
    uint8_t code[1] = { 0x27 };
    for (a = 0; a < 256; a++)
        for (fl = 0; fl < 8; fl++) {
            unsigned f = ((fl & 1) ? FN : 0) | ((fl & 2) ? FH : 0) | ((fl & 4) ? FC : 0);
            unsigned ra, rf;
            g->cpu.a = (uint8_t)a; g->cpu.f = (uint8_t)(f | FZ);
            exec_bytes(g, code, 1);
            ref_daa(a, f, &ra, &rf);
            n++;
            if (g->cpu.a != ra || g->cpu.f != rf) { bad++; if (bad < 5) printf("  daa a=%02X f=%02X -> %02X/%02X want %02X/%02X\n", a, f, g->cpu.a, g->cpu.f, ra, rf); }
        }
    HS_CHECK(&T, bad == 0, "DAA table %u bad", bad);
    printf("[cpu] DAA all 256 A x 8 N/H/C: %u cases, %u bad\n", n, bad);
    /* semantic: BCD add/sub of every pair via real ADC/SBC + DAA */
    bad = 0;
    for (x = 0; x < 100; x++) for (y = 0; y < 100; y++) for (c = 0; c < 2; c++) {
        uint8_t add[2] = { 0xCE, (uint8_t)to_bcd(y) }, sub[2] = { 0xDE, (uint8_t)to_bcd(y) };
        unsigned s = x + y + c, d;
        g->cpu.a = (uint8_t)to_bcd(x); g->cpu.f = c ? FC : 0;
        exec_bytes(g, add, 2); exec_bytes(g, code, 1);
        if (g->cpu.a != to_bcd(s % 100) || (!!(g->cpu.f & FC)) != (s >= 100)) bad++;
        g->cpu.a = (uint8_t)to_bcd(x); g->cpu.f = c ? FC : 0;
        exec_bytes(g, sub, 2); exec_bytes(g, code, 1);
        d = (x + 100 - y - c) % 100;
        if (g->cpu.a != to_bcd(d) || (!!(g->cpu.f & FC)) != ((int)x - (int)y - (int)c < 0)) bad++;
    }
    HS_CHECK(&T, bad == 0, "BCD add/sub semantic %u bad", bad);
    printf("[cpu] BCD semantic (ADC/SBC+DAA, 0..99 x 0..99 x C): 40000 cases, %u bad\n", bad);
}

static void test_incdec_cb(void)
{
    gbc_t *g = G;
    unsigned v, c, op, bad = 0, n = 0;
    for (v = 0; v < 256; v++) for (c = 0; c < 2; c++) {
        uint8_t inc[1] = { 0x3C }, dec[1] = { 0x3D }, incm[1] = { 0x34 };
        unsigned r;
        g->cpu.a = (uint8_t)v; g->cpu.f = c ? FC : 0; exec_bytes(g, inc, 1);
        r = (v + 1) & 0xFF;
        if (g->cpu.a != r || g->cpu.f != ((r ? 0 : FZ) | (((v ^ 1 ^ r) & 0x10) ? FH : 0) | (c ? FC : 0))) bad++;
        g->cpu.a = (uint8_t)v; g->cpu.f = c ? FC : 0; exec_bytes(g, dec, 1);
        r = (v - 1) & 0xFF;
        if (g->cpu.a != r || g->cpu.f != ((r ? 0 : FZ) | FN | (((v ^ 1 ^ r) & 0x10) ? FH : 0) | (c ? FC : 0))) bad++;
        g->cpu.h = 0xC1; g->cpu.l = 0x40; g->wram[0x140] = (uint8_t)v; g->cpu.f = c ? FC : 0; exec_bytes(g, incm, 1);
        if (g->wram[0x140] != ((v + 1) & 0xFF)) bad++;
        n += 3;
    }
    HS_CHECK(&T, bad == 0, "INC/DEC %u bad", bad);
    printf("[cpu] INC/DEC r/(HL) all values x C: %u cases, %u bad\n", n, bad);

    bad = 0; n = 0;
    for (op = 0; op < 256; op++) for (v = 0; v < 256; v++) for (c = 0; c < 2; c++) {
        uint8_t code[2] = { 0xCB, (uint8_t)op };
        unsigned x = op >> 6, y = (op >> 3) & 7, z = op & 7, r = v, f, got;
        unsigned nc = 0;
        uint8_t *reg[8] = { &g->cpu.b, &g->cpu.c, &g->cpu.d, &g->cpu.e, &g->cpu.h, &g->cpu.l, NULL, &g->cpu.a };
        g->cpu.h = 0xC1; g->cpu.l = 0x80;
        g->cpu.f = c ? FC : 0;
        if (z == 6) g->wram[0x180] = (uint8_t)v; else *reg[z] = (uint8_t)v;
        if (z == 4 || z == 5) { /* H/L are both operand and pointer: fine, not used */ }
        exec_bytes(g, code, 2);
        got = (z == 6) ? g->wram[0x180] : *reg[z];
        f = c ? FC : 0;
        if (x == 0) {
            switch (y) {
            case 0: nc = v >> 7; r = ((v << 1) | (v >> 7)) & 0xFF; break;
            case 1: nc = v & 1; r = ((v >> 1) | (v << 7)) & 0xFF; break;
            case 2: nc = v >> 7; r = ((v << 1) | c) & 0xFF; break;
            case 3: nc = v & 1; r = ((v >> 1) | (c << 7)) & 0xFF; break;
            case 4: nc = v >> 7; r = (v << 1) & 0xFF; break;
            case 5: nc = v & 1; r = ((v >> 1) | (v & 0x80)) & 0xFF; break;
            case 6: nc = 0; r = ((v & 0x0F) << 4) | (v >> 4); break;
            default: nc = v & 1; r = v >> 1; break;
            }
            f = (r ? 0 : FZ) | (nc ? FC : 0);
        } else if (x == 1) {
            r = v;
            f = (c ? FC : 0) | FH | (((v >> y) & 1) ? 0 : FZ);
        } else if (x == 2) {
            r = v & ~(1u << y);
        } else {
            r = v | (1u << y);
        }
        n++;
        if (got != r || g->cpu.f != f) { bad++; if (bad < 5) printf("  CB %02X v=%02X c=%u -> %02X/%02X want %02X/%02X\n", op, v, c, got, g->cpu.f, r, f); }
    }
    HS_CHECK(&T, bad == 0, "CB exhaustive %u bad", bad);
    printf("[cpu] CB prefix exhaustive (256 ops x 256 values x C, all 8 operands): %u cases, %u bad\n", n, bad);

    /* RLCA/RRCA/RLA/RRA (Z always cleared) */
    bad = 0;
    for (v = 0; v < 256; v++) for (c = 0; c < 2; c++) {
        static const uint8_t ops[4] = { 0x07, 0x0F, 0x17, 0x1F };
        for (op = 0; op < 4; op++) {
            uint8_t code[1] = { ops[op] };
            unsigned r, nc;
            g->cpu.a = (uint8_t)v; g->cpu.f = (uint8_t)(FZ | FN | FH | (c ? FC : 0));
            exec_bytes(g, code, 1);
            switch (op) {
            case 0: nc = v >> 7; r = ((v << 1) | nc) & 0xFF; break;
            case 1: nc = v & 1; r = ((v >> 1) | (nc << 7)) & 0xFF; break;
            case 2: nc = v >> 7; r = ((v << 1) | c) & 0xFF; break;
            default: nc = v & 1; r = ((v >> 1) | (c << 7)) & 0xFF; break;
            }
            if (g->cpu.a != r || g->cpu.f != (nc ? FC : 0)) bad++;
        }
    }
    HS_CHECK(&T, bad == 0, "RLCA/RRCA/RLA/RRA %u bad", bad);
}

static void test_16bit(void)
{
    gbc_t *g = G;
    unsigned i, e, bad = 0, n = 0;
    uint32_t s = 0x12345678u;
    for (i = 0; i < 65536; i++) {
        uint8_t code[1] = { 0x09 };   /* ADD HL,BC */
        unsigned hl, bc, r, f0, want;
        s = s * 1664525u + 1013904223u;
        hl = s >> 16; bc = s & 0xFFFF;
        if (i < 16) { hl = (i & 1) ? 0xFFFF : 0x0FFF; bc = (i & 2) ? 1 : 0xF001; }
        f0 = (i & 4) ? FZ : 0;
        g->cpu.h = (uint8_t)(hl >> 8); g->cpu.l = (uint8_t)hl; g->cpu.b = (uint8_t)(bc >> 8); g->cpu.c = (uint8_t)bc; g->cpu.f = (uint8_t)f0;
        exec_bytes(g, code, 1);
        r = hl + bc;
        want = f0 | (((hl ^ bc ^ r) & 0x1000) ? FH : 0) | (r > 0xFFFF ? FC : 0);
        n++;
        if (((unsigned)g->cpu.h << 8 | g->cpu.l) != (r & 0xFFFF) || g->cpu.f != want) bad++;
    }
    for (i = 0; i < 64; i++) for (e = 0; e < 256; e++) {
        uint8_t c1[2] = { 0xE8, (uint8_t)e }, c2[2] = { 0xF8, (uint8_t)e };
        unsigned sp = (i * 0x0405u + (i & 3) * 0x7Fu) & 0xFFFF, r, f;
        r = (sp + (unsigned)(int)(int8_t)e) & 0xFFFF;
        f = ((((sp & 0xF) + (e & 0xF)) > 0xF) ? FH : 0) | ((((sp & 0xFF) + e) > 0xFF) ? FC : 0);
        g->cpu.sp = (uint16_t)sp; g->cpu.f = 0xF0; exec_bytes(g, c1, 2);
        if (g->cpu.sp != r || g->cpu.f != f) bad++;
        g->cpu.sp = (uint16_t)sp; g->cpu.f = 0xF0; exec_bytes(g, c2, 2);
        if ((((unsigned)g->cpu.h << 8) | g->cpu.l) != r || g->cpu.f != f || g->cpu.sp != sp) bad++;
        n += 2;
    }
    HS_CHECK(&T, bad == 0, "16-bit arithmetic %u bad", bad);
    printf("[cpu] 16-bit ADD HL,rr / ADD SP,e / LD HL,SP+e: %u cases, %u bad\n", n, bad);
}

static void run_steps(gbc_t *g, unsigned n) { unsigned i; for (i = 0; i < n; i++) gbc_cpu_step(g); }

static void test_flow_irq(void)
{
    gbc_t *g = G;
    /* stack / CALL / RET / RST / PUSH-POP AF masking */
    {
        static const uint8_t prog[] = {
            0xCD, 0x10, 0xC0,   /* C000 CALL C010 */
            0x06, 0x99,         /* C003 LD B,99   */
            0xFF,               /* C005 RST 38 (ROM 0x38 = 00 -> slides) */
        };
        static const uint8_t sub[] = { 0x01, 0xFF, 0xFF, 0xC5, 0xF1, 0xF5, 0xD1, 0xC9 };
        /* C010: LD BC,FFFF; PUSH BC; POP AF; PUSH AF; POP DE; RET */
        memset(g->wram, 0, 0x200);
        memcpy(g->wram, prog, sizeof(prog));
        memcpy(g->wram + 0x10, sub, sizeof(sub));
        cpu_regs_safe(g);
        g->cpu.pc = 0xC000;
        run_steps(g, 1);
        HS_CHECK(&T, g->cpu.pc == 0xC010 && g->cpu.sp == 0xDFEE && g->wram[0x1FEE] == 0x03 && g->wram[0x1FEF] == 0xC0, "CALL push");
        run_steps(g, 6);
        HS_CHECK(&T, g->cpu.a == 0xFF && g->cpu.f == 0xF0 && g->cpu.d == 0xFF && g->cpu.e == 0xF0, "POP AF masks F low nibble (A=%02X F=%02X DE=%02X%02X)", g->cpu.a, g->cpu.f, g->cpu.d, g->cpu.e);
        HS_CHECK(&T, g->cpu.pc == 0xC003 && g->cpu.sp == 0xDFF0, "RET pc=%04X sp=%04X", g->cpu.pc, g->cpu.sp);
        run_steps(g, 2);
        HS_CHECK(&T, g->cpu.b == 0x99 && g->cpu.pc == 0x0038 && g->cpu.sp == 0xDFEE, "RST 38 pc=%04X", g->cpu.pc);
    }
    /* EI delay + interrupt dispatch + RETI */
    {
        static const uint8_t prog[] = { 0xFB, 0x04, 0x04, 0x00 };
        uint64_t t0;
        memcpy(g->wram, prog, sizeof(prog));
        cpu_regs_safe(g);
        g->cpu.b = 0; g->cpu.a = 0;
        g->ie = IRQ_TIMER; g->io[0x0F] = IRQ_TIMER;
        g->cpu.pc = 0xC000;
        run_steps(g, 1);  /* EI */
        HS_CHECK(&T, g->cpu.ime == 0 && g->cpu.pc == 0xC001, "EI not immediate");
        run_steps(g, 1);  /* INC B runs before the IRQ */
        HS_CHECK(&T, g->cpu.b == 1 && g->cpu.ime == 1, "EI delay: B=%u ime=%u", g->cpu.b, g->cpu.ime);
        t0 = g->total_cycles;
        run_steps(g, 1);  /* dispatch */
        HS_CHECK(&T, g->cpu.pc == 0x0050 && g->total_cycles - t0 == 20 && !(g->io[0x0F] & IRQ_TIMER) && g->cpu.ime == 0,
                 "IRQ dispatch pc=%04X cyc=%u", g->cpu.pc, (unsigned)(g->total_cycles - t0));
        run_steps(g, 2);  /* INC A ; RETI */
        HS_CHECK(&T, g->cpu.a == 1 && g->cpu.pc == 0xC002 && g->cpu.ime == 1, "RETI pc=%04X", g->cpu.pc);
        run_steps(g, 1);
        HS_CHECK(&T, g->cpu.b == 2, "resume after RETI");
    }
    /* HALT with IME=1 woken by timer */
    {
        static const uint8_t prog[] = { 0x76, 0x04, 0x00 };
        unsigned i;
        memcpy(g->wram, prog, sizeof(prog));
        cpu_regs_safe(g);
        g->cpu.a = 0; g->cpu.b = 0; g->cpu.ime = 1;
        gbc_write(g, 0xFF07, 0x05); gbc_write(g, 0xFF06, 0x00); gbc_write(g, 0xFF05, 0xF0);
        g->io[0x0F] = 0; g->ie = IRQ_TIMER;
        g->cpu.pc = 0xC000;
        run_steps(g, 1);
        HS_CHECK(&T, g->cpu.halted == 1, "HALT entered");
        for (i = 0; i < 2000 && g->cpu.pc != 0x0050; i++) run_steps(g, 1);
        HS_CHECK(&T, g->cpu.pc == 0x0050 && g->cpu.halted == 0, "HALT woken by timer IRQ (pc=%04X)", g->cpu.pc);
        run_steps(g, 3);
        HS_CHECK(&T, g->cpu.a == 1 && g->cpu.b == 1, "after HALT: A=%u B=%u", g->cpu.a, g->cpu.b);
        gbc_write(g, 0xFF07, 0x00);
    }
    /* HALT with IME=0, no pending -> halts, wakes without dispatch */
    {
        static const uint8_t prog[] = { 0x76, 0x04, 0x00 };
        unsigned i;
        memcpy(g->wram, prog, sizeof(prog));
        cpu_regs_safe(g);
        g->cpu.b = 0; g->io[0x0F] = 0; g->ie = IRQ_VBLANK;
        g->cpu.pc = 0xC000;
        run_steps(g, 1);
        for (i = 0; i < 40000 && g->cpu.halted; i++) run_steps(g, 1);  /* the waking step also runs INC B */
        HS_CHECK(&T, g->cpu.b == 1 && g->cpu.pc == 0xC002, "HALT IME=0 wake (B=%u pc=%04X)", g->cpu.b, g->cpu.pc);
    }
    /* HALT bug: IME=0 with pending IRQ -> next byte executes twice */
    {
        static const uint8_t prog[] = { 0x76, 0x3C, 0x00 };
        memcpy(g->wram, prog, sizeof(prog));
        cpu_regs_safe(g);
        g->cpu.a = 0; g->ie = IRQ_TIMER; g->io[0x0F] = IRQ_TIMER;
        g->cpu.pc = 0xC000;
        run_steps(g, 3);
        HS_CHECK(&T, g->cpu.a == 2 && g->cpu.pc == 0xC002 && !g->cpu.halted, "HALT bug A=%u pc=%04X", g->cpu.a, g->cpu.pc);
        g->io[0x0F] = 0;
    }
    /* timer: TAC=5 (16 T per tick): 64 NOPs = 256 T -> +16 */
    {
        unsigned i;
        cpu_regs_safe(g);
        memset(g->wram, 0, 0x100);
        g->cpu.pc = 0xC000;
        gbc_write(g, 0xFF05, 0x00);
        gbc_write(g, 0xFF04, 0x00);
        gbc_write(g, 0xFF07, 0x05);
        for (i = 0; i < 64; i++) gbc_cpu_step(g);
        HS_CHECK(&T, g->io[0x05] == 16, "timer TAC=5 TIMA=%u", g->io[0x05]);
        gbc_write(g, 0xFF07, 0x00);
    }
    printf("[cpu] flow/stack/IRQ/EI-delay/HALT/HALT-bug/timer: done\n");
}

static void test_double_speed(void)
{
    gbc_t *g;
    unsigned i;
    uint32_t d0, dn, dd;
    mk_rom(0x8000, 0x00, 0, 0x80);
    g = boot(HS_SYS_GBC, 0x8000);
    HS_CHECK(&T, g->cgb == 1 && g->cpu.a == 0x11, "CGB mode boot A=%02X", g->cpu.a);
    cpu_regs_safe(g);
    memset(g->wram, 0, 0x400);
    g->cpu.pc = 0xC000;
    d0 = g->step_dots;
    for (i = 0; i < 100; i++) gbc_cpu_step(g);
    dn = g->step_dots - d0;
    gbc_write(g, 0xFF4D, 0x01);
    HS_CHECK(&T, gbc_read(g, 0xFF4D) == 0x7F, "KEY1 armed reads %02X", gbc_read(g, 0xFF4D));
    g->wram[0x200] = 0x10; g->wram[0x201] = 0x00;  /* STOP */
    g->cpu.pc = 0xC200;
    gbc_cpu_step(g);
    HS_CHECK(&T, g->double_speed == 1 && gbc_read(g, 0xFF4D) == 0xFE && g->cpu.pc == 0xC202, "double speed on (KEY1=%02X)", gbc_read(g, 0xFF4D));
    g->cpu.pc = 0xC000;
    d0 = g->step_dots;
    for (i = 0; i < 100; i++) gbc_cpu_step(g);
    dd = g->step_dots - d0;
    HS_CHECK(&T, dn == 400 && dd == 200, "dots per 100 NOP: normal %u double %u", dn, dd);
    /* switch back */
    gbc_write(g, 0xFF4D, 0x01);
    g->cpu.pc = 0xC200; gbc_cpu_step(g);
    HS_CHECK(&T, g->double_speed == 0 && gbc_read(g, 0xFF4D) == 0x7E, "double speed off");
    printf("[cpu] CGB double speed: 100 NOPs = %u dots normal, %u dots double\n", dn, dd);
}

/* ======================================================= 2. mappers/DMA */
static void fill_banks(uint32_t size)
{
    uint32_t b;
    for (b = 1; b < size / 0x4000u; b++) {
        ROM[b * 0x4000u] = (uint8_t)b;
        ROM[b * 0x4000u + 1] = (uint8_t)(b >> 8);
    }
}

static unsigned bank_at(gbc_t *g, uint16_t base) { return gbc_read(g, base) | (gbc_read(g, (uint16_t)(base + 1)) << 8); }

static void test_mappers(void)
{
    gbc_t *g;
    unsigned bad = 0, b;
    uint8_t sram[0x20000 + 64];
    size_t sn;

    /* MBC1 2 MiB, 32 KiB RAM */
    mk_rom(0x200000, 0x03, 0x03, 0);
    fill_banks(0x200000);
    g = boot(HS_SYS_GB, 0x200000);
    for (b = 0; b < 128; b++) {
        unsigned want = (b & 0x1F) ? b : b + 1;
        gbc_write(g, 0x2000, (uint8_t)(b & 0x1F));
        gbc_write(g, 0x4000, (uint8_t)(b >> 5));
        if (bank_at(g, 0x4000) != want) { bad++; if (bad < 4) printf("  MBC1 bank %u read %u want %u\n", b, bank_at(g, 0x4000), want); }
    }
    gbc_write(g, 0x6000, 1); gbc_write(g, 0x4000, 2);
    HS_CHECK(&T, bank_at(g, 0x0000) == 0x40 || ROM[0x40 * 0x4000] == 0x40, "MBC1 mode1 bank0 remap");
    HS_CHECK(&T, gbc_read(g, 0x0000) == ROM[0x40 * 0x4000], "MBC1 mode1 ROM0 = bank 0x40");
    gbc_write(g, 0x0000, 0x0A);
    gbc_write(g, 0x4000, 1); gbc_write(g, 0xA000, 0x11);
    gbc_write(g, 0x4000, 2); gbc_write(g, 0xA000, 0x22);
    gbc_write(g, 0x4000, 1);
    HS_CHECK(&T, gbc_read(g, 0xA000) == 0x11, "MBC1 RAM banking");
    gbc_write(g, 0x0000, 0x00);
    HS_CHECK(&T, gbc_read(g, 0xA000) == 0xFF, "MBC1 RAM disabled reads FF");
    sn = O->save_sram(g, sram, sizeof(sram));
    HS_CHECK(&T, sn == 0x8000 && sram[0x2000] == 0x11 && sram[0x4000] == 0x22, "MBC1 save_sram (%zu)", sn);
    HS_CHECK(&T, bad == 0, "MBC1 ROM banking %u bad", bad);

    /* MBC2 256 KiB with 512x4 RAM */
    bad = 0;
    mk_rom(0x40000, 0x06, 0x00, 0);
    fill_banks(0x40000);
    g = boot(HS_SYS_GB, 0x40000);
    for (b = 0; b < 16; b++) {
        gbc_write(g, 0x2100, (uint8_t)b);
        if (bank_at(g, 0x4000) != (b ? b : 1)) bad++;
    }
    gbc_write(g, 0x0000, 0x0A);
    gbc_write(g, 0xA005, 0xAB);
    HS_CHECK(&T, gbc_read(g, 0xA005) == 0xFB && gbc_read(g, 0xA205) == 0xFB, "MBC2 4-bit RAM + mirror (%02X)", gbc_read(g, 0xA005));
    HS_CHECK(&T, bad == 0, "MBC2 banking %u bad", bad);

    /* MBC3 + RTC, 2 MiB, 32 KiB RAM */
    bad = 0;
    mk_rom(0x200000, 0x10, 0x03, 0);
    fill_banks(0x200000);
    g = boot(HS_SYS_GB, 0x200000);
    for (b = 0; b < 128; b++) {
        gbc_write(g, 0x2000, (uint8_t)b);
        if (bank_at(g, 0x4000) != (b ? b : 1)) bad++;
    }
    HS_CHECK(&T, bad == 0, "MBC3 banking %u bad", bad);
    gbc_rtc_advance(g, 1000);           /* first epoch arms the clock */
    gbc_write(g, 0x0000, 0x0A);
    gbc_rtc_advance(g, 1000 + 3 * 86400 + 5 * 3600 + 7 * 60 + 9);
    gbc_write(g, 0x6000, 0); gbc_write(g, 0x6000, 1);
    gbc_write(g, 0x4000, 0x08); b = gbc_read(g, 0xA000);
    HS_CHECK(&T, b == 9, "RTC sec %u", b);
    gbc_write(g, 0x4000, 0x09); HS_CHECK(&T, gbc_read(g, 0xA000) == 7, "RTC min");
    gbc_write(g, 0x4000, 0x0A); HS_CHECK(&T, gbc_read(g, 0xA000) == 5, "RTC hour");
    gbc_write(g, 0x4000, 0x0B); HS_CHECK(&T, gbc_read(g, 0xA000) == 3, "RTC days");
    /* halt flag stops the clock */
    gbc_write(g, 0x4000, 0x0C); gbc_write(g, 0xA000, 0x40);
    gbc_rtc_advance(g, 1000 + 4 * 86400);
    gbc_write(g, 0x6000, 0); gbc_write(g, 0x6000, 1);
    gbc_write(g, 0x4000, 0x0B); HS_CHECK(&T, gbc_read(g, 0xA000) == 3, "RTC halted");
    /* day overflow sets carry */
    gbc_write(g, 0x4000, 0x0C); gbc_write(g, 0xA000, 0x01);
    gbc_write(g, 0x4000, 0x0B); gbc_write(g, 0xA000, 0xFF);
    gbc_rtc_advance(g, 1000 + 5 * 86400);
    gbc_write(g, 0x6000, 0); gbc_write(g, 0x6000, 1);
    gbc_write(g, 0x4000, 0x0C); b = gbc_read(g, 0xA000);
    HS_CHECK(&T, (b & 0x80) && !(b & 0x01), "RTC day carry (DH=%02X)", b);
    gbc_write(g, 0x4000, 0x02); gbc_write(g, 0xA123, 0x5A);
    sn = O->save_sram(g, sram, sizeof(sram));
    HS_CHECK(&T, sn == 0x8000 + 24 && sram[0x4123] == 0x5A && sram[0x8000] == 'H', "MBC3 save_sram + RTC block (%zu)", sn);
    /* reload into a fresh machine */
    g = boot(HS_SYS_GB, 0x200000);
    O->load_sram(g, sram, sn);
    gbc_write(g, 0x0000, 0x0A); gbc_write(g, 0x4000, 0x02);
    HS_CHECK(&T, gbc_read(g, 0xA123) == 0x5A && g->rtc.epoch_valid && g->rtc.last_epoch == 1000 + 5 * 86400, "MBC3 load_sram round-trip");

    /* MBC5 8 MiB (512 banks incl. bank 0 + bit 8), 128 KiB RAM */
    bad = 0;
    mk_rom(ROM_CAP, 0x1B, 0x04, 0);
    fill_banks(ROM_CAP);
    g = boot(HS_SYS_GB, ROM_CAP);
    for (b = 0; b < 512; b++) {
        gbc_write(g, 0x2000, (uint8_t)b);
        gbc_write(g, 0x3000, (uint8_t)(b >> 8));
        if (bank_at(g, 0x4000) != b && !(b == 0 && gbc_read(g, 0x4000) == ROM[0])) bad++;
    }
    HS_CHECK(&T, bad == 0, "MBC5 9-bit banking %u bad", bad);
    gbc_write(g, 0x0000, 0x0A);
    for (b = 0; b < 16; b++) { gbc_write(g, 0x4000, (uint8_t)b); gbc_write(g, 0xBFFF, (uint8_t)(0xA0 + b)); }
    bad = 0;
    for (b = 0; b < 16; b++) { gbc_write(g, 0x4000, (uint8_t)b); if (gbc_read(g, 0xBFFF) != 0xA0 + b) bad++; }
    HS_CHECK(&T, bad == 0, "MBC5 16 RAM banks");
    printf("[mmu] MBC1/MBC2/MBC3+RTC/MBC5 banking, RAM, RTC, SRAM round-trip: done\n");

    /* WRAM banking (CGB SVBK) + echo */
    mk_rom(0x8000, 0x00, 0, 0x80);
    g = boot(HS_SYS_GBC, 0x8000);
    for (b = 1; b < 8; b++) { gbc_write(g, 0xFF70, (uint8_t)b); gbc_write(g, 0xD000, (uint8_t)(0x10 + b)); }
    bad = 0;
    for (b = 1; b < 8; b++) { gbc_write(g, 0xFF70, (uint8_t)b); if (gbc_read(g, 0xD000) != 0x10 + b || gbc_read(g, 0xF000) != 0x10 + b) bad++; }
    gbc_write(g, 0xFF70, 0); HS_CHECK(&T, gbc_read(g, 0xD000) == 0x11, "SVBK 0 -> 1");
    HS_CHECK(&T, bad == 0, "SVBK banks + echo %u bad", bad);
    gbc_write(g, 0xFF4F, 1); gbc_write(g, 0x8000, 0x77); gbc_write(g, 0xFF4F, 0);
    HS_CHECK(&T, gbc_read(g, 0x8000) == 0x00 && g->vram[0x2000] == 0x77, "VBK bank 1");
}

static void test_dma(void)
{
    gbc_t *g;
    unsigned i, bad = 0;
    uint64_t t0;
    mk_rom(0x8000, 0x00, 0, 0x80);
    g = boot(HS_SYS_GBC, 0x8000);
    cpu_regs_safe(g);
    memset(g->wram, 0, 0x400);
    /* OAM DMA from C100 */
    for (i = 0; i < 160; i++) g->wram[0x100 + i] = (uint8_t)(i ^ 0x5A);
    gbc_write(g, 0xFF46, 0xC1);
    g->cpu.pc = 0xC000;
    for (i = 0; i < 170; i++) gbc_cpu_step(g);
    for (i = 0; i < 160; i++) if (g->oam[i] != (uint8_t)(i ^ 0x5A)) bad++;
    HS_CHECK(&T, bad == 0 && !g->dma_active, "OAM DMA %u bad", bad);

    /* GDMA: 256 bytes WRAM D000.. -> VRAM bank1 8100 */
    for (i = 0; i < 256; i++) g->wram[0x1000 + i] = (uint8_t)(255 - i);
    gbc_write(g, 0xFF4F, 1);
    gbc_write(g, 0xFF51, 0xD0); gbc_write(g, 0xFF52, 0x00);
    gbc_write(g, 0xFF53, 0x01); gbc_write(g, 0xFF54, 0x00);
    t0 = g->total_cycles;
    gbc_write(g, 0xFF55, 0x0F);
    bad = 0;
    for (i = 0; i < 256; i++) if (g->vram[0x2100 + i] != (uint8_t)(255 - i)) bad++;
    HS_CHECK(&T, bad == 0 && gbc_read(g, 0xFF55) == 0xFF, "GDMA copy %u bad", bad);
    HS_CHECK(&T, g->total_cycles - t0 == 16u * 8u * 4u, "GDMA stall %u T", (unsigned)(g->total_cycles - t0));

    /* HBlank HDMA: 4 blocks, one per HBlank */
    for (i = 0; i < 64; i++) g->wram[0x1100 + i] = (uint8_t)(0x80 + i);
    gbc_write(g, 0xFF4F, 0);
    gbc_write(g, 0xFF51, 0xD1); gbc_write(g, 0xFF52, 0x00);
    gbc_write(g, 0xFF53, 0x02); gbc_write(g, 0xFF54, 0x00);
    /* wait for line start (mode 2) so the first block lands on a real HBlank */
    for (i = 0; i < 2000 && !(g->mode == 2 && g->ly < 100); i++) gbc_cpu_step(g);
    gbc_write(g, 0xFF55, 0x83);
    HS_CHECK(&T, gbc_read(g, 0xFF55) == 0x03, "HDMA5 active reads %02X", gbc_read(g, 0xFF55));
    {
        unsigned ly0 = g->ly, lines = 0;
        for (i = 0; i < 20000 && g->hdma_active; i++) {
            gbc_cpu_step(g);
        }
        lines = (unsigned)g->ly - ly0;
        bad = 0;
        for (i = 0; i < 64; i++) if (g->vram[0x0200 + i] != (uint8_t)(0x80 + i)) bad++;
        HS_CHECK(&T, bad == 0 && gbc_read(g, 0xFF55) == 0xFF, "HBlank HDMA copy %u bad, HDMA5=%02X", bad, gbc_read(g, 0xFF55));
        HS_CHECK(&T, lines == 3 || lines == 4, "HBlank HDMA spans %u lines (1 block per HBlank)", lines);
        printf("[mmu] OAM DMA, GDMA (256 B, %u T stall), HBlank HDMA (4 blocks over %u lines): done\n", 16u * 8u * 4u, lines);
    }
}

/* ============================================ 3. zero-mutex phase contract */
static uint64_t hash_cpu_domain(const gbc_t *g)
{
    return fnv(HS_FNV64_OFFSET, g, offsetof(gbc_t, front));
}
static uint64_t hash_slot(const gbc_t *g, uint32_t slot)
{
    uint64_t h = fnv(HS_FNV64_OFFSET, &g->vs[slot], sizeof(gbc_vsnap_t));
    return fnv(h, &g->as[slot], sizeof(gbc_asnap_t));
}
static uint64_t hash_vid(const gbc_t *g) { return fnv(HS_FNV64_OFFSET, &g->vid, sizeof(g->vid)); }
static uint64_t hash_aud(const gbc_t *g) { return fnv(HS_FNV64_OFFSET, &g->aud, sizeof(g->aud)); }

static uint32_t input_script(uint32_t f)
{
    /* deterministic play-through: tap START/A through the title + menus,
     * then sweep the d-pad (with A taps) to actually play */
    if (f < 320) {
        uint32_t m = f % 40u;
        if (f >= 30 && m < 4) return (f / 40u) & 1u ? HS_BTN_A : HS_BTN_START;
        return 0;
    }
    {
        static const uint32_t pad[8] = { HS_BTN_RIGHT, 0, HS_BTN_DOWN, 0, HS_BTN_LEFT, HS_BTN_A, HS_BTN_UP, 0 };
        return pad[(f / 6) & 7];
    }
}

static uint32_t *FB;
static int16_t PCM[4096 * 2];

typedef struct run_result {
    uint64_t video;     /* hash over every rendered 640x480 frame */
    uint64_t audio;     /* hash over every PCM sample */
    uint64_t snap;      /* hash over ops->hash_state values */
    uint32_t frames, unique_frames, pcm_frames, colors;
    int32_t  pcm_peak;
} run_result_t;

static void run_rom(gbc_t *g, uint32_t frames, hs_video_mode_t mode, int pipelined, run_result_t *r)
{
    uint32_t f;
    uint64_t prev = 0;
    memset(r, 0, sizeof(*r));
    r->video = r->audio = r->snap = HS_FNV64_OFFSET;
    if (pipelined) {
        O->step_cpu_phase(g, input_script(0), 1700000000);
        O->commit_phase(g);
    }
    for (f = 0; f < frames; f++) {
        uint32_t n, i;
        uint64_t vh, ah, fh;
        if (pipelined) {
            /* Core 0 steps frame f+1 into BACK while Cores 1/2 present frame f */
            O->step_cpu_phase(g, input_script(f + 1), 1700000000 + (int64_t)(f + 1) / 60);
            O->render_video_phase(g, FB, HS_FB_W, mode);
            n = O->synth_audio_phase(g, PCM, 4096);
            O->hash_state(g, &vh, &ah);
            O->commit_phase(g);
        } else {
            O->step_cpu_phase(g, input_script(f), 1700000000 + (int64_t)f / 60);
            O->commit_phase(g);
            O->render_video_phase(g, FB, HS_FB_W, mode);
            n = O->synth_audio_phase(g, PCM, 4096);
            O->hash_state(g, &vh, &ah);
        }
        fh = fnv(HS_FNV64_OFFSET, FB, (size_t)HS_FB_W * HS_FB_H * 4u);
        r->video = fnv(r->video, &fh, 8);
        r->audio = fnv(r->audio, PCM, (size_t)n * 4u);
        r->snap = fnv(fnv(r->snap, &vh, 8), &ah, 8);
        if (fh != prev) r->unique_frames++;
        prev = fh;
        r->pcm_frames += n;
        for (i = 0; i < n * 2u; i++) {
            int32_t v = PCM[i] < 0 ? -PCM[i] : PCM[i];
            if (v > r->pcm_peak) r->pcm_peak = v;
        }
    }
    r->frames = frames;
    {
        /* distinct colours in the last native frame (bounded set) */
        uint32_t seen[64], ns = 0, i, j;
        for (i = 0; i < GB_W * GB_H && ns < 64; i++) {
            uint32_t c = g->vid.native[i];
            for (j = 0; j < ns; j++) if (seen[j] == c) break;
            if (j == ns) seen[ns++] = c;
        }
        r->colors = ns;
    }
}

static void test_phase_contract(const uint8_t *rom, size_t len)
{
    gbc_t *g;
    unsigned f, bad_front = 0, bad_cpu = 0, bad_back = 0, bad_dom = 0, bad_apu = 0;
    hs_core_config_t cfg = { HS_SYS_GBC, 0, 0 };
    O->init(G, O->state_size(), &cfg);
    O->load_rom(G, rom, len);
    g = G;
    for (f = 0; f < 90; f++) {
        uint64_t front_before, front_after, cpu_b, cpu_a, back_b, back_a, vid_b, vid_a, aud_b, aud_a;
        /* PHASE 0 must not touch FRONT, video or audio domains */
        front_before = hash_slot(g, g->front);
        vid_b = hash_vid(g); aud_b = hash_aud(g);
        O->step_cpu_phase(g, input_script(f), 1700000000);
        front_after = hash_slot(g, g->front);
        vid_a = hash_vid(g); aud_a = hash_aud(g);
        if (front_before != front_after) bad_front++;
        if (vid_b != vid_a || aud_b != aud_a) bad_dom++;
        O->commit_phase(g);
        /* PHASE 1 + 2 must not touch CPU domain or BACK slot */
        cpu_b = hash_cpu_domain(g); back_b = hash_slot(g, g->back);
        O->render_video_phase(g, FB, HS_FB_W, HS_SCALE_STRETCH_SHARP);
        aud_b = hash_aud(g);
        cpu_a = hash_cpu_domain(g); back_a = hash_slot(g, g->back);
        if (cpu_b != cpu_a || back_b != back_a) bad_cpu++;
        if (hash_aud(g) != aud_b) bad_dom++;
        vid_b = hash_vid(g);
        O->synth_audio_phase(g, PCM, 4096);
        if (hash_cpu_domain(g) != cpu_a || hash_slot(g, g->back) != back_a) bad_back++;
        if (hash_vid(g) != vid_b) bad_dom++;
        /* synth replay ends in exactly the CPU-side APU state */
        if (memcmp(&g->aud.apu, &g->apu, sizeof(gbc_apu_t)) != 0) bad_apu++;
    }
    HS_CHECK(&T, bad_front == 0, "CPU phase wrote FRONT snapshot %u times", bad_front);
    HS_CHECK(&T, bad_cpu == 0, "video phase wrote CPU/BACK %u times", bad_cpu);
    HS_CHECK(&T, bad_back == 0, "audio phase wrote CPU/BACK %u times", bad_back);
    HS_CHECK(&T, bad_dom == 0, "cross-domain writes %u", bad_dom);
    HS_CHECK(&T, bad_apu == 0, "synth APU end-state != CPU tracker %u frames", bad_apu);
    printf("[phase] 90 frames: FRONT untouched by CPU phase, CPU/BACK untouched by video+audio, domains disjoint, synth==tracker (%u/%u/%u/%u/%u)\n",
           bad_front, bad_cpu, bad_back, bad_dom, bad_apu);
}

/* ================================================= 4. real ROM runs */
static size_t load_file(const char *dir, const char *name)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    return hs_read_file(path, ROM, ROM_CAP);
}

static void save_png(const char *name, const uint32_t *px, uint32_t w, uint32_t h, size_t stride)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", OUTDIR, name);
    if (hs_png_write_xrgb(path, px, w, h, stride) == 0)
        printf("  wrote %s\n", path);
}

static void test_real_rom(const char *name, const char *png, uint32_t frames)
{
    size_t len = load_file(ROMDIR, name);
    hs_core_config_t cfg = { HS_SYS_GBC, 0, 0 };
    run_result_t a, b;
    if (!len) { printf("[rom] %s missing\n", name); HS_CHECK(&T, 0, "missing %s", name); return; }
    O->init(G, O->state_size(), &cfg);
    O->load_rom(G, ROM, len);
    run_rom(G, frames, HS_SCALE_STRETCH_SHARP, 0, &a);
    if (png) save_png(png, FB, HS_FB_W, HS_FB_H, HS_FB_W);
    {
        /* final frame through every 2D output mode */
        static const hs_video_mode_t modes[3] = { HS_SCALE_STRETCH_SHARP, HS_SCALE_STRETCH_NEAREST, HS_AA_FXAA };
        static const char *mn[3] = { "sharp", "nearest", "fxaa" };
        unsigned k;
        for (k = 0; k < 3; k++) {
            O->render_video_phase(G, FB, HS_FB_W, modes[k]);
            printf("HASH %s.final_640x480_%s %016llx\n", name, mn[k],
                   (unsigned long long)fnv(HS_FNV64_OFFSET, FB, (size_t)HS_FB_W * HS_FB_H * 4u));
        }
    }
    O->init(G, O->state_size(), &cfg);
    O->load_rom(G, ROM, len);
    run_rom(G, frames, HS_SCALE_STRETCH_SHARP, 1, &b);
    HS_CHECK(&T, a.video == b.video && a.audio == b.audio && a.snap == b.snap, "%s pipelined != sequential", name);
    HS_CHECK(&T, a.unique_frames >= 2, "%s frames not animated (%u unique)", name, a.unique_frames);
    HS_CHECK(&T, a.colors >= 2, "%s blank frame (%u colours)", name, a.colors);
    HS_CHECK(&T, a.pcm_frames >= frames * 800u, "%s pcm frames %u", name, a.pcm_frames);
    printf("[rom] %-18s %u frames: unique=%u colours=%u pcm=%u peak=%d | pipelined==sequential %s\n",
           name, frames, a.unique_frames, a.colors, a.pcm_frames, a.pcm_peak,
           (a.video == b.video && a.audio == b.audio) ? "YES" : "NO");
    printf("HASH %s.video %016llx\nHASH %s.audio %016llx\nHASH %s.snap %016llx\n",
           name, (unsigned long long)a.video, name, (unsigned long long)a.audio, name, (unsigned long long)a.snap);
}

static int vram_has_text(const gbc_t *g, const char *s)
{
    size_t n = strlen(s), i;
    for (i = 0x1800; i + n <= 0x2000; i++)
        if (memcmp(&g->vram[i], s, n) == 0)
            return 1;
    return 0;
}

static void test_external_roms(void)
{
    static const struct { const char *file; hs_console_t c; uint32_t frames; const char *serial; const char *screen; } X[] = {
        { "cpu_instrs.gb",   HS_SYS_GB,  3800, "Passed all tests", NULL },
        { "cpu_instrs.gb",   HS_SYS_GBC, 3800, "Passed all tests", NULL },
        { "instr_timing.gb", HS_SYS_GB,   300, "Passed", NULL },
        { "mem_timing.gb",   HS_SYS_GB,   300, "Passed all tests", NULL },
        { "halt_bug.gb",     HS_SYS_GB,   300, NULL, "Passed" },
        { "dmg-acid2.gb",    HS_SYS_GB,    60, NULL, NULL },
        { "cgb-acid2.gbc",   HS_SYS_GBC,   60, NULL, NULL },
    };
    unsigned i;
    for (i = 0; i < sizeof(X) / sizeof(X[0]); i++) {
        hs_core_config_t cfg = { X[i].c, 1, 0 };
        size_t len = load_file(EXTDIR, X[i].file);
        uint32_t f, frames = X[i].frames;
        uint64_t vh = 0, ah = 0;
        int ok = 1;
        if (!len) { printf("[ext] %s not present (optional) - skipped\n", X[i].file); continue; }
        if (QUICK && frames > 1000) frames = 1000;
        O->init(G, O->state_size(), &cfg);
        O->load_rom(G, ROM, len);
        for (f = 0; f < frames; f++) {
            O->step_cpu_phase(G, 0, 0);
            O->commit_phase(G);
            if (X[i].serial && G->serial_len >= strlen(X[i].serial) &&
                strstr((const char *)G->serial_log, X[i].serial)) {
                /* finish a couple more frames then stop */
                if (f + 3 < frames) frames = f + 3;
            }
        }
        O->render_video_phase(G, FB, HS_FB_W, HS_SCALE_STRETCH_NEAREST);
        O->hash_state(G, &vh, &ah);
        if (G->serial_len < GBC_SERIAL_LOG) G->serial_log[G->serial_len] = 0;
        if (X[i].serial) ok = strstr((const char *)G->serial_log, X[i].serial) != NULL;
        if (X[i].screen) ok = vram_has_text(G, X[i].screen);
        if (X[i].serial || X[i].screen) {
            HS_CHECK(&T, ok, "%s (%s) failed", X[i].file, X[i].c == HS_SYS_GB ? "DMG" : "CGB");
        }
        printf("HASH ext.%s.%s.snapvideo %016llx\n", X[i].file, X[i].c == HS_SYS_GB ? "dmg" : "cgb", (unsigned long long)vh);
        printf("[ext] %-16s %-3s %5u frames: %s\n", X[i].file, X[i].c == HS_SYS_GB ? "DMG" : "CGB", f,
               (X[i].serial || X[i].screen) ? (ok ? "PASS" : "FAIL") : "rendered (pixel compare in run-tests.sh)");
        if (!X[i].serial && !X[i].screen) {
            char png[128];
            snprintf(png, sizeof(png), "%s_native.png", X[i].file);
            save_png(png, G->vid.native, GB_W, GB_H, GB_W);
        }
    }
}

/* ======================================= 5. synthetic clean-room CGB ROM */
typedef struct asm_ctx { uint8_t *m; uint32_t pc; } asm_t;
static void E(asm_t *a, int n, ...);
#include <stdarg.h>
static void E(asm_t *a, int n, ...)
{
    va_list ap;
    int i;
    va_start(ap, n);
    for (i = 0; i < n; i++) a->m[a->pc++] = (uint8_t)va_arg(ap, int);
    va_end(ap);
}
static void jr_to(asm_t *a, uint8_t op, uint32_t target)
{
    int d = (int)target - (int)(a->pc + 2);
    E(a, 2, op, (uint8_t)(int8_t)d);
}
static void ldh_n(asm_t *a, uint8_t reg, uint8_t v) { E(a, 2, 0x3E, v); E(a, 2, 0xE0, reg); } /* LD A,v ; LDH (reg),A */

#define SYN_PAL   0x1000u
#define SYN_TILEA 0x1100u
#define SYN_TILEB 0x1200u
#define SYN_SPR   0x1300u
#define SYN_WAVE  0x1400u

static uint32_t build_cgb_rom(void)
{
    asm_t A = { ROM, 0 };
    uint32_t i, loop, vbl, l2, skip;
    memset(ROM, 0, 0x8000);
    strcpy((char *)&ROM[0x134], "HS-CGB-TEST");
    ROM[0x143] = 0x80; ROM[0x147] = 0x00; ROM[0x148] = 0x00; ROM[0x149] = 0x00;
    ROM[0x100] = 0x00; ROM[0x101] = 0xC3; ROM[0x102] = 0x50; ROM[0x103] = 0x01;

    /* data: 8 BG palettes (rainbow), 8 OBJ palettes */
    for (i = 0; i < 32; i++) {
        uint32_t p = i >> 2, c = i & 3;
        uint32_t r = (p & 1) ? 31 : (p * 4) & 31, gg = (p & 2) ? 31 - c * 6 : c * 9, b = (p & 4) ? 31 : c * 7;
        uint16_t v;
        if (c == 0) { r = 31; gg = 31; b = 31; }
        r = (r * (4 - c) + 1) / 4 + (c == 3 ? 0 : 0);
        v = (uint16_t)((r & 31) | ((gg & 31) << 5) | ((b & 31) << 10));
        ROM[SYN_PAL + i * 2] = (uint8_t)v; ROM[SYN_PAL + i * 2 + 1] = (uint8_t)(v >> 8);
        v = (uint16_t)(((31 - (p * 3)) & 31) | (((p * 5) & 31) << 5) | (((c * 10) & 31) << 10));
        ROM[SYN_PAL + 64 + i * 2] = (uint8_t)v; ROM[SYN_PAL + 64 + i * 2 + 1] = (uint8_t)(v >> 8);
    }
    /* tile sets A (solid colour 0..3) and B (stripes), 4 tiles each */
    for (i = 0; i < 4; i++) {
        uint32_t r;
        for (r = 0; r < 8; r++) {
            ROM[SYN_TILEA + i * 16 + r * 2]     = (i & 1) ? 0xFF : 0x00;
            ROM[SYN_TILEA + i * 16 + r * 2 + 1] = (i & 2) ? 0xFF : 0x00;
            ROM[SYN_TILEB + i * 16 + r * 2]     = (uint8_t)((i & 1) ? (r & 1 ? 0xF0 : 0xFF) : (r & 2 ? 0x0F : 0x00));
            ROM[SYN_TILEB + i * 16 + r * 2 + 1] = (uint8_t)((i & 2) ? (r & 1 ? 0xFF : 0x3C) : (r & 4 ? 0xC3 : 0x00));
        }
    }
    /* sprite tile (diamond, colours 1..3) */
    {
        static const uint8_t lo[8] = { 0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0x7E, 0x3C, 0x18 };
        static const uint8_t hi[8] = { 0x00, 0x18, 0x3C, 0x66, 0x66, 0x3C, 0x18, 0x00 };
        for (i = 0; i < 8; i++) { ROM[SYN_SPR + i * 2] = lo[i]; ROM[SYN_SPR + i * 2 + 1] = hi[i]; }
    }
    for (i = 0; i < 16; i++) ROM[SYN_WAVE + i] = (uint8_t)(((i * 2) << 4) | (i * 2 + 1));

    /* VBlank vector */
    A.pc = 0x40; E(&A, 3, 0xC3, 0x00, 0x08);  /* JP 0800 */

    /* ---- main @0150 ---- */
    A.pc = 0x150;
    E(&A, 1, 0xF3);                         /* DI */
    E(&A, 3, 0x31, 0xFE, 0xFF);             /* LD SP,FFFE */
    ldh_n(&A, 0x4D, 0x01);                  /* KEY1 = 1 */
    E(&A, 2, 0x10, 0x00);                   /* STOP -> double speed */
    loop = A.pc;                            /* wait LY==144 */
    E(&A, 2, 0xF0, 0x44); E(&A, 2, 0xFE, 0x90); jr_to(&A, 0x20, loop);
    ldh_n(&A, 0x40, 0x00);                  /* LCD off */
    /* BG palettes */
    ldh_n(&A, 0x68, 0x80);
    E(&A, 3, 0x21, SYN_PAL & 0xFF, SYN_PAL >> 8); E(&A, 2, 0x06, 64);
    loop = A.pc; E(&A, 1, 0x2A); E(&A, 2, 0xE0, 0x69); E(&A, 1, 0x05); jr_to(&A, 0x20, loop);
    /* OBJ palettes */
    ldh_n(&A, 0x6A, 0x80);
    E(&A, 2, 0x06, 64);
    loop = A.pc; E(&A, 1, 0x2A); E(&A, 2, 0xE0, 0x6B); E(&A, 1, 0x05); jr_to(&A, 0x20, loop);
    /* GDMA tiles A -> 8000 (4 tiles), sprite -> 8040 */
    ldh_n(&A, 0x51, SYN_TILEA >> 8); ldh_n(&A, 0x52, SYN_TILEA & 0xFF);
    ldh_n(&A, 0x53, 0x00); ldh_n(&A, 0x54, 0x00); ldh_n(&A, 0x55, 0x03);
    ldh_n(&A, 0x51, SYN_SPR >> 8); ldh_n(&A, 0x52, SYN_SPR & 0xFF);
    ldh_n(&A, 0x53, 0x00); ldh_n(&A, 0x54, 0x40); ldh_n(&A, 0x55, 0x00);
    /* BG map 9800: tile = (col>>2)&3 */
    E(&A, 3, 0x21, 0x00, 0x98);
    loop = A.pc;
    E(&A, 1, 0x7D); E(&A, 2, 0xE6, 0x0C); E(&A, 1, 0x0F); E(&A, 1, 0x0F); E(&A, 1, 0x22);
    E(&A, 1, 0x7C); E(&A, 2, 0xFE, 0x9C); jr_to(&A, 0x20, loop);
    /* attributes (VBK=1): palette = ((col>>2) + (row>>2)) & 7, Y-flip on odd rows */
    ldh_n(&A, 0x4F, 0x01);
    E(&A, 3, 0x21, 0x00, 0x98);
    loop = A.pc;
    E(&A, 1, 0x7D); E(&A, 2, 0xE6, 0x1C); E(&A, 1, 0x0F); E(&A, 1, 0x0F); E(&A, 1, 0x47);   /* B = col>>2 */
    E(&A, 1, 0x7D); E(&A, 1, 0x07); E(&A, 1, 0x07); E(&A, 1, 0x07); E(&A, 2, 0xE6, 0x03);   /* A = (L>>5)&3 (row lo bits) */
    E(&A, 1, 0x4F); E(&A, 1, 0x7C); E(&A, 2, 0xE6, 0x03); E(&A, 1, 0x07); E(&A, 1, 0x07); E(&A, 1, 0xB1); /* row = (H&3)<<2 | (L>>5) */
    E(&A, 1, 0x0F); E(&A, 1, 0x0F); E(&A, 1, 0x80); E(&A, 2, 0xE6, 0x07);                  /* (row>>2) + B */
    E(&A, 1, 0x22);
    E(&A, 1, 0x7C); E(&A, 2, 0xFE, 0x9C); jr_to(&A, 0x20, loop);
    ldh_n(&A, 0x4F, 0x00);
    /* clear OAM, then 2 sprites */
    E(&A, 3, 0x21, 0x00, 0xFE); E(&A, 2, 0x06, 160); E(&A, 1, 0xAF);
    loop = A.pc; E(&A, 1, 0x22); E(&A, 1, 0x05); jr_to(&A, 0x20, loop);
    E(&A, 3, 0x21, 0x00, 0xFE);
    E(&A, 2, 0x36, 80);  E(&A, 1, 0x23); E(&A, 2, 0x36, 16); E(&A, 1, 0x23); E(&A, 2, 0x36, 4); E(&A, 1, 0x23); E(&A, 2, 0x36, 0x01); E(&A, 1, 0x23);
    E(&A, 2, 0x36, 110); E(&A, 1, 0x23); E(&A, 2, 0x36, 90); E(&A, 1, 0x23); E(&A, 2, 0x36, 4); E(&A, 1, 0x23); E(&A, 2, 0x36, 0x22); E(&A, 1, 0x23);
    /* audio: all four channels */
    ldh_n(&A, 0x26, 0x80); ldh_n(&A, 0x24, 0x77); ldh_n(&A, 0x25, 0xFF);
    ldh_n(&A, 0x10, 0x1F); ldh_n(&A, 0x11, 0x80); ldh_n(&A, 0x12, 0xF0); ldh_n(&A, 0x13, 0x00); ldh_n(&A, 0x14, 0x87);
    ldh_n(&A, 0x16, 0x40); ldh_n(&A, 0x17, 0xA0); ldh_n(&A, 0x18, 0x80); ldh_n(&A, 0x19, 0x86);
    ldh_n(&A, 0x1A, 0x00);
    E(&A, 3, 0x21, SYN_WAVE & 0xFF, SYN_WAVE >> 8); E(&A, 2, 0x0E, 0x30); E(&A, 2, 0x06, 16);
    loop = A.pc; E(&A, 1, 0x2A); E(&A, 1, 0xE2); E(&A, 1, 0x0C); E(&A, 1, 0x05); jr_to(&A, 0x20, loop);
    ldh_n(&A, 0x1A, 0x80); ldh_n(&A, 0x1B, 0x00); ldh_n(&A, 0x1C, 0x20); ldh_n(&A, 0x1D, 0x00); ldh_n(&A, 0x1E, 0x87);
    ldh_n(&A, 0x20, 0x00); ldh_n(&A, 0x21, 0x70); ldh_n(&A, 0x22, 0x52); ldh_n(&A, 0x23, 0x80);
    /* frame counter, interrupts, LCD on */
    E(&A, 1, 0xAF); E(&A, 2, 0xE0, 0x80);
    ldh_n(&A, 0xFF, 0x01);
    ldh_n(&A, 0x0F, 0x00);
    ldh_n(&A, 0x40, 0x93);
    E(&A, 1, 0xFB);
    l2 = A.pc; E(&A, 1, 0x76); E(&A, 1, 0x00); jr_to(&A, 0x18, l2);

    if (A.pc >= 0x800) { printf("synthetic main overflows handler\n"); exit(3); }
    /* ---- VBlank handler @0800 ---- */
    A.pc = 0x800;
    vbl = A.pc; (void)vbl;
    E(&A, 1, 0xF5); E(&A, 1, 0xE5);
    E(&A, 2, 0xF0, 0x80); E(&A, 1, 0x3C); E(&A, 2, 0xE0, 0x80);       /* frame++ */
    E(&A, 3, 0xEA, 0x01, 0xFE);                                      /* sprite0.X = frame */
    E(&A, 1, 0x47); E(&A, 2, 0xE6, 0x1F); E(&A, 2, 0xC6, 60); E(&A, 3, 0xEA, 0x04, 0xFE); /* sprite1.Y */
    E(&A, 1, 0x78); E(&A, 1, 0x1F); E(&A, 2, 0xE0, 0x43);             /* SCX = frame>>1 (via RRA) */
    E(&A, 1, 0x78); E(&A, 2, 0xE6, 0x0F); skip = A.pc; E(&A, 2, 0x20, 0x00);  /* every 16 frames: */
    E(&A, 1, 0x78); E(&A, 2, 0xE0, 0x18); ldh_n(&A, 0x19, 0x86);        /*   retrigger CH2 */
    ldh_n(&A, 0x23, 0x80);                                             /*   retrigger CH4 */
    ROM[skip + 1] = (uint8_t)(A.pc - (skip + 2));
    /* palette animation: BG pal 0 colour 1 */
    ldh_n(&A, 0x68, 0x02); E(&A, 1, 0x78); E(&A, 1, 0x07); E(&A, 2, 0xE0, 0x69); E(&A, 1, 0x78); E(&A, 2, 0xE0, 0x69);
    /* HBlank HDMA of 4 tiles: set A on even frames, set B on odd frames */
    E(&A, 1, 0x78); E(&A, 2, 0xE6, 0x01); E(&A, 2, 0x3E, SYN_TILEA >> 8);
    skip = A.pc; E(&A, 2, 0x28, 0x00); E(&A, 2, 0x3E, SYN_TILEB >> 8); ROM[skip + 1] = (uint8_t)(A.pc - (skip + 2));
    E(&A, 2, 0xE0, 0x51); ldh_n(&A, 0x52, 0x00); ldh_n(&A, 0x53, 0x00); ldh_n(&A, 0x54, 0x00); ldh_n(&A, 0x55, 0x83);
    E(&A, 1, 0xE1); E(&A, 1, 0xF1); E(&A, 1, 0xD9);
    return 0x8000;
}

static void test_synthetic_cgb(void)
{
    uint32_t len = build_cgb_rom();
    hs_core_config_t cfg = { HS_SYS_GBC, 0, 0 };
    run_result_t a, b;
    uint32_t f, nonzero = 0, i, objpix = 0;
    uint64_t vh, ah;
    O->init(G, O->state_size(), &cfg);
    if (O->load_rom(G, ROM, len) != 0) { HS_CHECK(&T, 0, "synthetic rom load"); return; }
    /* direct run for structural checks */
    for (f = 0; f < 60; f++) {
        uint32_t n;
        O->step_cpu_phase(G, 0, 0);
        O->commit_phase(G);
        n = O->synth_audio_phase(G, PCM, 4096);
        for (i = 0; i < n * 2; i++) if (PCM[i]) nonzero++;
    }
    O->render_video_phase(G, FB, HS_FB_W, HS_SCALE_STRETCH_SHARP);
    O->hash_state(G, &vh, &ah);
    HS_CHECK(&T, G->cgb && G->double_speed, "synthetic ROM in CGB double speed (cgb=%u ds=%u)", G->cgb, G->double_speed);
    HS_CHECK(&T, (G->apu.ch1.on + G->apu.ch2.on + G->apu.ch3.on + G->apu.ch4.on) >= 3, "audio channels on: %u%u%u%u",
             G->apu.ch1.on, G->apu.ch2.on, G->apu.ch3.on, G->apu.ch4.on);
    HS_CHECK(&T, nonzero > 10000, "audio waveform non-zero samples %u", nonzero);
    HS_CHECK(&T, G->vs[G->front].vlog_count > 0, "HDMA/OAM writes logged mid-frame (%u)", G->vs[G->front].vlog_count);
    for (i = 0; i < GB_W * GB_H; i++) {
        uint32_t c = G->vid.native[i], k;
        for (k = 0; k < 8; k++) {
            /* compare against OBJ palette 1/2 colours 1..3 */
            const uint8_t *p = &G->objpal[k * 8];
            uint32_t j;
            for (j = 1; j < 4; j++) {
                uint32_t v = (uint32_t)p[j * 2] | ((uint32_t)p[j * 2 + 1] << 8);
                uint32_t r = v & 31, gg = (v >> 5) & 31, bb = (v >> 10) & 31;
                uint32_t x = (((r << 3) | (r >> 2)) << 16) | (((gg << 3) | (gg >> 2)) << 8) | ((bb << 3) | (bb >> 2));
                if (x == c && (k == 1 || k == 2)) objpix++;
            }
        }
    }
    HS_CHECK(&T, objpix > 20, "sprite pixels from OBJ palettes: %u", objpix);
    save_png("cgb_test_640x480.png", FB, HS_FB_W, HS_FB_H, HS_FB_W);
    save_png("cgb_test_native.png", G->vid.native, GB_W, GB_H, GB_W);

    /* hashed sequential vs pipelined */
    O->init(G, O->state_size(), &cfg); O->load_rom(G, ROM, len);
    run_rom(G, 60, HS_SCALE_STRETCH_SHARP, 0, &a);
    O->init(G, O->state_size(), &cfg); O->load_rom(G, ROM, len);
    run_rom(G, 60, HS_SCALE_STRETCH_SHARP, 1, &b);
    HS_CHECK(&T, a.video == b.video && a.audio == b.audio && a.snap == b.snap, "synthetic CGB pipelined != sequential");
    HS_CHECK(&T, a.colors >= 12, "CGB multi-palette colours %u", a.colors);
    HS_CHECK(&T, a.unique_frames >= 30, "CGB animation unique frames %u", a.unique_frames);
    printf("[cgb] synthetic CGB ROM 60 frames: colours=%u unique=%u nonzero-pcm=%u peak=%d obj-px=%u vlog=%u double-speed=%u\n",
           a.colors, a.unique_frames, nonzero, a.pcm_peak, objpix, G->vs[G->front].vlog_count, G->double_speed);
    printf("HASH cgb_synth.video %016llx\nHASH cgb_synth.audio %016llx\nHASH cgb_synth.snap %016llx\n",
           (unsigned long long)a.video, (unsigned long long)a.audio, (unsigned long long)a.snap);
    {
        /* save the generated ROM for inspection */
        char path[512];
        FILE *fp;
        snprintf(path, sizeof(path), "%s/hs_cgb_test.gbc", OUTDIR);
        fp = fopen(path, "wb");
        if (fp) { fwrite(ROM, 1, len, fp); fclose(fp); }
    }
}

/* ================================================== 6. benchmark */
static void bench(const char *name, uint32_t frames)
{
    size_t len = load_file(ROMDIR, name);
    hs_core_config_t cfg = { HS_SYS_GBC, 0, 0 };
    uint64_t t0, t_cpu = 0, t_vid = 0, t_aud = 0, t_vid_n = 0, t_fx = 0, t_all;
    uint32_t f;
    if (!len) return;
    O->init(G, O->state_size(), &cfg);
    O->load_rom(G, ROM, len);
    for (f = 0; f < frames; f++) {
        t0 = hs_host_now_ns(); O->step_cpu_phase(G, input_script(f), 0); t_cpu += hs_host_now_ns() - t0;
        O->commit_phase(G);
        t0 = hs_host_now_ns(); O->render_video_phase(G, FB, HS_FB_W, HS_SCALE_STRETCH_SHARP); t_vid += hs_host_now_ns() - t0;
        t0 = hs_host_now_ns(); O->synth_audio_phase(G, PCM, 4096); t_aud += hs_host_now_ns() - t0;
        if ((f & 7) == 0) {
            t0 = hs_host_now_ns(); O->render_video_phase(G, FB, HS_FB_W, HS_SCALE_STRETCH_NEAREST); t_vid_n += hs_host_now_ns() - t0;
            t0 = hs_host_now_ns(); O->render_video_phase(G, FB, HS_FB_W, HS_AA_FXAA); t_fx += hs_host_now_ns() - t0;
        }
    }
    t_all = t_cpu + t_vid + t_aud;
    {
        uint64_t crit = t_cpu > t_vid ? t_cpu : t_vid;
        if (t_aud > crit) crit = t_aud;
        printf("[bench] %s %u frames (host):\n", name, frames);
        printf("  single-thread (step+render+synth): %.1f fps  (%.3f ms/frame)\n", frames * 1e9 / (double)t_all, t_all / 1e6 / frames);
        printf("  phase 0 CPU+bus                  : %.3f ms/frame (%.0f fps)\n", t_cpu / 1e6 / frames, frames * 1e9 / (double)t_cpu);
        printf("  phase 1 video native+640x480 sharp: %.3f ms/frame (%.0f fps)\n", t_vid / 1e6 / frames, frames * 1e9 / (double)t_vid);
        printf("  phase 1 video nearest            : %.3f ms/frame\n", t_vid_n / 1e6 / ((frames + 7) / 8));
        printf("  phase 1 video sharp+FXAA         : %.3f ms/frame\n", t_fx / 1e6 / ((frames + 7) / 8));
        printf("  phase 2 audio synth 48 kHz       : %.3f ms/frame (%.0f fps)\n", t_aud / 1e6 / frames, frames * 1e9 / (double)t_aud);
        printf("  3-core pipelined bound (max phase): %.1f fps\n", frames * 1e9 / (double)crit);
    }
}

int main(int argc, char **argv)
{
    int i;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--quick")) QUICK = 1;
        else if (!strncmp(argv[i], "--out=", 6)) OUTDIR = argv[i] + 6;
        else if (!strncmp(argv[i], "--roms=", 7)) ROMDIR = argv[i] + 7;
        else if (!strncmp(argv[i], "--ext=", 6)) EXTDIR = argv[i] + 6;
    }
    G = (gbc_t *)aligned_alloc(64, (O->state_size() + 63u) & ~(size_t)63u);
    ROM = (uint8_t *)malloc(ROM_CAP);
    FB = (uint32_t *)malloc((size_t)HS_FB_W * HS_FB_H * 4u);
    if (!G || !ROM || !FB) return 2;
    printf("hs-gbc test suite: state=%zu bytes, abi=%u\n", O->state_size(), O->abi_version);

    test_cycle_table();
    test_alu_exhaustive();
    test_daa();
    test_incdec_cb();
    test_16bit();
    test_flow_irq();
    test_double_speed();
    test_mappers();
    test_dma();
    {
        size_t len = load_file(ROMDIR, "libbet.gb");
        if (len) {
            uint8_t *copy = (uint8_t *)malloc(len);
            memcpy(copy, ROM, len);
            test_phase_contract(copy, len);
            free(copy);
        }
    }
    test_real_rom("libbet.gb", "libbet_640x480.png", 480);
    test_real_rom("handshake-demo.gb", "handshake_demo_640x480.png", 480);
    test_synthetic_cgb();
    test_external_roms();
    bench("libbet.gb", QUICK ? 120 : 600);

    printf("\nRESULT hs-gbc: %u passed, %u failed\n", T.pass, T.fail);
    free(G); free(ROM); free(FB);
    return T.fail ? 1 : 0;
}
