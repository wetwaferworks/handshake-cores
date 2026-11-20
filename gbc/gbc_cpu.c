/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_cpu.c - clean-room Sharp SM83 (LR35902) interpreter.
 * All 256 primary + 256 CB opcodes, exact Z/N/H/C flags, DAA, HALT + HALT
 * bug, STOP + CGB double-speed switch, EI delay, interrupt dispatch.
 * Every memory access is one M-cycle; internal delays tick explicitly, so
 * instruction T-cycle counts fall out of the access pattern.
 */
#include "gbc_internal.h"

#define CPU (&g->cpu)

static inline uint8_t rd(gbc_t *g, uint16_t a) { gbc_tick(g); return gbc_read(g, a); }
static inline void wr(gbc_t *g, uint16_t a, uint8_t v) { gbc_tick(g); gbc_write(g, a, v); }
static inline void idle(gbc_t *g) { gbc_tick(g); }

static inline uint8_t imm8(gbc_t *g) { uint8_t v = rd(g, CPU->pc); CPU->pc++; return v; }
static inline uint16_t imm16(gbc_t *g) { uint8_t lo = imm8(g); uint8_t hi = imm8(g); return (uint16_t)(lo | (hi << 8)); }

static inline uint16_t BC(gbc_t *g) { return (uint16_t)((CPU->b << 8) | CPU->c); }
static inline uint16_t DE(gbc_t *g) { return (uint16_t)((CPU->d << 8) | CPU->e); }
static inline uint16_t HL(gbc_t *g) { return (uint16_t)((CPU->h << 8) | CPU->l); }
static inline void setBC(gbc_t *g, uint16_t v) { CPU->b = (uint8_t)(v >> 8); CPU->c = (uint8_t)v; }
static inline void setDE(gbc_t *g, uint16_t v) { CPU->d = (uint8_t)(v >> 8); CPU->e = (uint8_t)v; }
static inline void setHL(gbc_t *g, uint16_t v) { CPU->h = (uint8_t)(v >> 8); CPU->l = (uint8_t)v; }

static inline void push16(gbc_t *g, uint16_t v)
{
    CPU->sp--; wr(g, CPU->sp, (uint8_t)(v >> 8));
    CPU->sp--; wr(g, CPU->sp, (uint8_t)v);
}
static inline uint16_t pop16(gbc_t *g)
{
    uint8_t lo = rd(g, CPU->sp); CPU->sp++;
    uint8_t hi = rd(g, CPU->sp); CPU->sp++;
    return (uint16_t)(lo | (hi << 8));
}

/* register index: 0 B 1 C 2 D 3 E 4 H 5 L 6 (HL) 7 A */
static uint8_t getr(gbc_t *g, unsigned i)
{
    switch (i & 7u) {
    case 0: return CPU->b; case 1: return CPU->c;
    case 2: return CPU->d; case 3: return CPU->e;
    case 4: return CPU->h; case 5: return CPU->l;
    case 6: return rd(g, HL(g));
    default: return CPU->a;
    }
}
static void setr(gbc_t *g, unsigned i, uint8_t v)
{
    switch (i & 7u) {
    case 0: CPU->b = v; break; case 1: CPU->c = v; break;
    case 2: CPU->d = v; break; case 3: CPU->e = v; break;
    case 4: CPU->h = v; break; case 5: CPU->l = v; break;
    case 6: wr(g, HL(g), v); break;
    default: CPU->a = v; break;
    }
}

/* ---- ALU -------------------------------------------------------------- */
static void alu(gbc_t *g, unsigned op, uint8_t v)
{
    uint8_t a = CPU->a;
    unsigned c = (CPU->f & FC) ? 1u : 0u;
    unsigned r;
    switch (op & 7u) {
    case 0: /* ADD */
        r = (unsigned)a + v;
        CPU->f = (uint8_t)(((r & 0xFFu) ? 0 : FZ) | (((a & 0xFu) + (v & 0xFu)) > 0xFu ? FH : 0) | (r > 0xFFu ? FC : 0));
        CPU->a = (uint8_t)r; break;
    case 1: /* ADC */
        r = (unsigned)a + v + c;
        CPU->f = (uint8_t)(((r & 0xFFu) ? 0 : FZ) | (((a & 0xFu) + (v & 0xFu) + c) > 0xFu ? FH : 0) | (r > 0xFFu ? FC : 0));
        CPU->a = (uint8_t)r; break;
    case 2: /* SUB */
        r = (unsigned)a - v;
        CPU->f = (uint8_t)(((r & 0xFFu) ? 0 : FZ) | FN | ((a & 0xFu) < (v & 0xFu) ? FH : 0) | (a < v ? FC : 0));
        CPU->a = (uint8_t)r; break;
    case 3: /* SBC */
        r = (unsigned)a - v - c;
        CPU->f = (uint8_t)(((r & 0xFFu) ? 0 : FZ) | FN | ((a & 0xFu) < (v & 0xFu) + c ? FH : 0) | ((unsigned)a < (unsigned)v + c ? FC : 0));
        CPU->a = (uint8_t)r; break;
    case 4: /* AND */
        CPU->a = a & v; CPU->f = (uint8_t)((CPU->a ? 0 : FZ) | FH); break;
    case 5: /* XOR */
        CPU->a = a ^ v; CPU->f = (uint8_t)(CPU->a ? 0 : FZ); break;
    case 6: /* OR */
        CPU->a = a | v; CPU->f = (uint8_t)(CPU->a ? 0 : FZ); break;
    default: /* CP */
        r = (unsigned)a - v;
        CPU->f = (uint8_t)(((r & 0xFFu) ? 0 : FZ) | FN | ((a & 0xFu) < (v & 0xFu) ? FH : 0) | (a < v ? FC : 0));
        break;
    }
}

static uint8_t inc8(gbc_t *g, uint8_t v)
{
    uint8_t r = (uint8_t)(v + 1u);
    CPU->f = (uint8_t)((CPU->f & FC) | (r ? 0 : FZ) | ((v & 0xFu) == 0xFu ? FH : 0));
    return r;
}
static uint8_t dec8(gbc_t *g, uint8_t v)
{
    uint8_t r = (uint8_t)(v - 1u);
    CPU->f = (uint8_t)((CPU->f & FC) | (r ? 0 : FZ) | FN | ((v & 0xFu) == 0u ? FH : 0));
    return r;
}

static void add_hl(gbc_t *g, uint16_t v)
{
    uint16_t hl = HL(g);
    uint32_t r = (uint32_t)hl + v;
    CPU->f = (uint8_t)((CPU->f & FZ) | (((hl & 0xFFFu) + (v & 0xFFFu)) > 0xFFFu ? FH : 0) | (r > 0xFFFFu ? FC : 0));
    setHL(g, (uint16_t)r);
}

static uint16_t sp_plus_e(gbc_t *g, uint8_t e)
{
    uint16_t sp = CPU->sp;
    uint16_t r = (uint16_t)(sp + (uint16_t)(int16_t)(int8_t)e);
    CPU->f = (uint8_t)((((sp & 0xFu) + (e & 0xFu)) > 0xFu ? FH : 0) | (((sp & 0xFFu) + e) > 0xFFu ? FC : 0));
    return r;
}

static void daa(gbc_t *g)
{
    uint8_t a = CPU->a;
    uint8_t f = CPU->f;
    if (!(f & FN)) {
        if ((f & FC) || a > 0x99u) { a = (uint8_t)(a + 0x60u); f |= FC; }
        if ((f & FH) || (a & 0x0Fu) > 0x09u) { a = (uint8_t)(a + 0x06u); }
    } else {
        if (f & FC) a = (uint8_t)(a - 0x60u);
        if (f & FH) a = (uint8_t)(a - 0x06u);
    }
    f = (uint8_t)((f & (FN | FC)) | (a ? 0 : FZ));
    CPU->a = a;
    CPU->f = f;
}

static int cond(gbc_t *g, unsigned cc)
{
    switch (cc & 3u) {
    case 0: return !(CPU->f & FZ);
    case 1: return (CPU->f & FZ) != 0;
    case 2: return !(CPU->f & FC);
    default: return (CPU->f & FC) != 0;
    }
}

/* ---- CB prefix ---------------------------------------------------------- */
static void exec_cb(gbc_t *g)
{
    uint8_t op = imm8(g);
    unsigned x = op >> 6, y = (op >> 3) & 7u, z = op & 7u;
    uint8_t v = getr(g, z);
    uint8_t r = 0;
    unsigned c = (CPU->f & FC) ? 1u : 0u;
    if (x == 0) {
        uint8_t nc = 0;
        switch (y) {
        case 0: nc = v >> 7; r = (uint8_t)((v << 1) | nc); break;            /* RLC */
        case 1: nc = v & 1u; r = (uint8_t)((v >> 1) | (nc << 7)); break;     /* RRC */
        case 2: nc = v >> 7; r = (uint8_t)((v << 1) | c); break;             /* RL  */
        case 3: nc = v & 1u; r = (uint8_t)((v >> 1) | (c << 7)); break;      /* RR  */
        case 4: nc = v >> 7; r = (uint8_t)(v << 1); break;                   /* SLA */
        case 5: nc = v & 1u; r = (uint8_t)((v >> 1) | (v & 0x80u)); break;   /* SRA */
        case 6: nc = 0; r = (uint8_t)((v << 4) | (v >> 4)); break;           /* SWAP */
        default: nc = v & 1u; r = (uint8_t)(v >> 1); break;                  /* SRL */
        }
        CPU->f = (uint8_t)((r ? 0 : FZ) | (nc ? FC : 0));
        setr(g, z, r);
    } else if (x == 1) { /* BIT */
        CPU->f = (uint8_t)((CPU->f & FC) | FH | ((v >> y) & 1u ? 0 : FZ));
    } else if (x == 2) { /* RES */
        setr(g, z, (uint8_t)(v & ~(1u << y)));
    } else { /* SET */
        setr(g, z, (uint8_t)(v | (1u << y)));
    }
}

/* ---- interrupts ------------------------------------------------------- */
static int service_interrupt(gbc_t *g)
{
    uint8_t pend = (uint8_t)(g->ie & g->io[0x0F] & 0x1Fu);
    unsigned bit;
    uint16_t pc;
    if (!CPU->ime || !pend)
        return 0;
    CPU->ime = 0;
    CPU->ei_delay = 0;
    idle(g);
    idle(g);
    pc = CPU->pc;
    CPU->sp--; wr(g, CPU->sp, (uint8_t)(pc >> 8));
    /* the high-byte push may have overwritten IE: re-sample */
    pend = (uint8_t)(g->ie & g->io[0x0F] & 0x1Fu);
    CPU->sp--; wr(g, CPU->sp, (uint8_t)pc);
    if (!pend) {
        CPU->pc = 0x0000;
    } else {
        for (bit = 0; bit < 5u; bit++)
            if (pend & (1u << bit))
                break;
        g->io[0x0F] &= (uint8_t)~(1u << bit);
        CPU->pc = (uint16_t)(0x40u + bit * 8u);
    }
    idle(g);
    return 1;
}

void gbc_cpu_reset(gbc_t *g)
{
    gbc_cpu_t *c = CPU;
    if (g->cgb) {
        c->a = 0x11; c->f = 0x80; c->b = 0x00; c->c = 0x00;
        c->d = 0xFF; c->e = 0x56; c->h = 0x00; c->l = 0x0D;
    } else {
        c->a = 0x01; c->f = 0xB0; c->b = 0x00; c->c = 0x13;
        c->d = 0x00; c->e = 0xD8; c->h = 0x01; c->l = 0x4D;
    }
    c->sp = 0xFFFE; c->pc = 0x0100;
    c->ime = 0; c->ei_delay = 0; c->halted = 0; c->stopped = 0;
    c->halt_bug = 0; c->locked = 0;
}

static void do_stop(gbc_t *g)
{
    (void)imm8(g);  /* STOP is two bytes */
    if (g->cgb && (g->key1 & 1u)) {
        unsigned i;
        g->double_speed ^= 1u;
        g->key1 = (uint8_t)(g->double_speed ? 0x80u : 0x00u);
        g->div = 0;
        /* the switch stalls the CPU for ~2050 M-cycles */
        for (i = 0; i < 2050u; i++)
            idle(g);
    } else {
        g->div = 0;
        CPU->stopped = 1;
    }
}

void gbc_cpu_step(gbc_t *g)
{
    uint8_t op;
    unsigned x, y, z;

    if (g->hdma_block_pending)
        gbc_hdma_service(g);

    if (CPU->locked) { idle(g); return; }
    if (CPU->stopped) {
        idle(g);
        return;   /* joypad press clears stopped (mmu) */
    }
    if (CPU->halted) {
        if (g->ie & g->io[0x0F] & 0x1Fu) {
            CPU->halted = 0;
        } else {
            idle(g);
            return;
        }
    }
    if (service_interrupt(g))
        return;

    /* fetch (HALT bug: PC fails to advance once) */
    op = rd(g, CPU->pc);
    if (CPU->halt_bug) CPU->halt_bug = 0; else CPU->pc++;
    g->instr_count++;

    x = op >> 6; y = (op >> 3) & 7u; z = op & 7u;

    if (x == 1) {
        if (op == 0x76) { /* HALT */
            if (!CPU->ime && (g->ie & g->io[0x0F] & 0x1Fu))
                CPU->halt_bug = 1;
            else
                CPU->halted = 1;
        } else {
            setr(g, y, getr(g, z));
        }
        goto ei_tail;
    }
    if (x == 2) {
        alu(g, y, getr(g, z));
        goto ei_tail;
    }

    switch (op) {
    case 0x00: break;
    case 0x01: setBC(g, imm16(g)); break;
    case 0x11: setDE(g, imm16(g)); break;
    case 0x21: setHL(g, imm16(g)); break;
    case 0x31: CPU->sp = imm16(g); break;
    case 0x02: wr(g, BC(g), CPU->a); break;
    case 0x12: wr(g, DE(g), CPU->a); break;
    case 0x22: wr(g, HL(g), CPU->a); setHL(g, (uint16_t)(HL(g) + 1u)); break;
    case 0x32: wr(g, HL(g), CPU->a); setHL(g, (uint16_t)(HL(g) - 1u)); break;
    case 0x0A: CPU->a = rd(g, BC(g)); break;
    case 0x1A: CPU->a = rd(g, DE(g)); break;
    case 0x2A: CPU->a = rd(g, HL(g)); setHL(g, (uint16_t)(HL(g) + 1u)); break;
    case 0x3A: CPU->a = rd(g, HL(g)); setHL(g, (uint16_t)(HL(g) - 1u)); break;
    case 0x03: idle(g); setBC(g, (uint16_t)(BC(g) + 1u)); break;
    case 0x13: idle(g); setDE(g, (uint16_t)(DE(g) + 1u)); break;
    case 0x23: idle(g); setHL(g, (uint16_t)(HL(g) + 1u)); break;
    case 0x33: idle(g); CPU->sp++; break;
    case 0x0B: idle(g); setBC(g, (uint16_t)(BC(g) - 1u)); break;
    case 0x1B: idle(g); setDE(g, (uint16_t)(DE(g) - 1u)); break;
    case 0x2B: idle(g); setHL(g, (uint16_t)(HL(g) - 1u)); break;
    case 0x3B: idle(g); CPU->sp--; break;
    case 0x04: case 0x0C: case 0x14: case 0x1C: case 0x24: case 0x2C: case 0x34: case 0x3C:
        setr(g, y, inc8(g, getr(g, y))); break;
    case 0x05: case 0x0D: case 0x15: case 0x1D: case 0x25: case 0x2D: case 0x35: case 0x3D:
        setr(g, y, dec8(g, getr(g, y))); break;
    case 0x06: case 0x0E: case 0x16: case 0x1E: case 0x26: case 0x2E: case 0x36: case 0x3E:
        setr(g, y, imm8(g)); break;
    case 0x07: { uint8_t a = CPU->a; CPU->a = (uint8_t)((a << 1) | (a >> 7)); CPU->f = (uint8_t)((a & 0x80u) ? FC : 0); } break;
    case 0x0F: { uint8_t a = CPU->a; CPU->a = (uint8_t)((a >> 1) | (a << 7)); CPU->f = (uint8_t)((a & 1u) ? FC : 0); } break;
    case 0x17: { uint8_t a = CPU->a; CPU->a = (uint8_t)((a << 1) | ((CPU->f & FC) ? 1u : 0u)); CPU->f = (uint8_t)((a & 0x80u) ? FC : 0); } break;
    case 0x1F: { uint8_t a = CPU->a; CPU->a = (uint8_t)((a >> 1) | ((CPU->f & FC) ? 0x80u : 0u)); CPU->f = (uint8_t)((a & 1u) ? FC : 0); } break;
    case 0x08: { uint16_t a = imm16(g); wr(g, a, (uint8_t)CPU->sp); wr(g, (uint16_t)(a + 1u), (uint8_t)(CPU->sp >> 8)); } break;
    case 0x09: idle(g); add_hl(g, BC(g)); break;
    case 0x19: idle(g); add_hl(g, DE(g)); break;
    case 0x29: idle(g); add_hl(g, HL(g)); break;
    case 0x39: idle(g); add_hl(g, CPU->sp); break;
    case 0x10: do_stop(g); break;
    case 0x18: { int8_t e = (int8_t)imm8(g); idle(g); CPU->pc = (uint16_t)(CPU->pc + e); } break;
    case 0x20: case 0x28: case 0x30: case 0x38: {
        int8_t e = (int8_t)imm8(g);
        if (cond(g, y - 4u)) { idle(g); CPU->pc = (uint16_t)(CPU->pc + e); }
    } break;
    case 0x27: daa(g); break;
    case 0x2F: CPU->a = (uint8_t)~CPU->a; CPU->f = (uint8_t)(CPU->f | FN | FH); break;
    case 0x37: CPU->f = (uint8_t)((CPU->f & FZ) | FC); break;
    case 0x3F: CPU->f = (uint8_t)((CPU->f & FZ) | ((CPU->f & FC) ? 0 : FC)); break;

    /* x == 3 */
    case 0xC0: case 0xC8: case 0xD0: case 0xD8:
        idle(g);
        if (cond(g, y)) { CPU->pc = pop16(g); idle(g); }
        break;
    case 0xC9: CPU->pc = pop16(g); idle(g); break;
    case 0xD9: CPU->pc = pop16(g); idle(g); CPU->ime = 1; CPU->ei_delay = 0; break;
    case 0xC1: setBC(g, pop16(g)); break;
    case 0xD1: setDE(g, pop16(g)); break;
    case 0xE1: setHL(g, pop16(g)); break;
    case 0xF1: { uint16_t v = pop16(g); CPU->a = (uint8_t)(v >> 8); CPU->f = (uint8_t)(v & 0xF0u); } break;
    case 0xC5: idle(g); push16(g, BC(g)); break;
    case 0xD5: idle(g); push16(g, DE(g)); break;
    case 0xE5: idle(g); push16(g, HL(g)); break;
    case 0xF5: idle(g); push16(g, (uint16_t)((CPU->a << 8) | CPU->f)); break;
    case 0xC2: case 0xCA: case 0xD2: case 0xDA: {
        uint16_t a = imm16(g);
        if (cond(g, y)) { idle(g); CPU->pc = a; }
    } break;
    case 0xC3: { uint16_t a = imm16(g); idle(g); CPU->pc = a; } break;
    case 0xE9: CPU->pc = HL(g); break;
    case 0xC4: case 0xCC: case 0xD4: case 0xDC: {
        uint16_t a = imm16(g);
        if (cond(g, y)) { idle(g); push16(g, CPU->pc); CPU->pc = a; }
    } break;
    case 0xCD: { uint16_t a = imm16(g); idle(g); push16(g, CPU->pc); CPU->pc = a; } break;
    case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF:
        idle(g); push16(g, CPU->pc); CPU->pc = (uint16_t)(y * 8u); break;
    case 0xC6: case 0xCE: case 0xD6: case 0xDE: case 0xE6: case 0xEE: case 0xF6: case 0xFE:
        alu(g, y, imm8(g)); break;
    case 0xCB: exec_cb(g); break;
    case 0xE0: { uint8_t n = imm8(g); wr(g, (uint16_t)(0xFF00u | n), CPU->a); } break;
    case 0xF0: { uint8_t n = imm8(g); CPU->a = rd(g, (uint16_t)(0xFF00u | n)); } break;
    case 0xE2: wr(g, (uint16_t)(0xFF00u | CPU->c), CPU->a); break;
    case 0xF2: CPU->a = rd(g, (uint16_t)(0xFF00u | CPU->c)); break;
    case 0xEA: { uint16_t a = imm16(g); wr(g, a, CPU->a); } break;
    case 0xFA: { uint16_t a = imm16(g); CPU->a = rd(g, a); } break;
    case 0xE8: { uint8_t e = imm8(g); CPU->sp = sp_plus_e(g, e); idle(g); idle(g); } break;
    case 0xF8: { uint8_t e = imm8(g); setHL(g, sp_plus_e(g, e)); idle(g); } break;
    case 0xF9: idle(g); CPU->sp = HL(g); break;
    case 0xF3: CPU->ime = 0; CPU->ei_delay = 0; break;
    case 0xFB: if (!CPU->ime && !CPU->ei_delay) CPU->ei_delay = 2; goto ei_skip;
    default:
        /* D3 DB DD E3 E4 EB EC ED F4 FC FD: hard lock-up on hardware */
        CPU->locked = 1;
        break;
    }
ei_tail:
    if (CPU->ei_delay) {
        CPU->ei_delay--;
        if (CPU->ei_delay == 0)
            CPU->ime = 1;
    }
    return;
ei_skip:
    CPU->ei_delay = CPU->ei_delay ? 1u : 0u;
}
