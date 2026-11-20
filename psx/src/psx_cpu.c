/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_cpu.c - clean-room MIPS R3000A interpreter (Phase 0).
 *   - all R/I/J integer instructions, HI/LO mult/div (incl. div-by-zero and
 *     0x80000000/-1 hardware results), LWL/LWR/SWL/SWR,
 *   - one-instruction branch delay slot and load delay slot (a write by the
 *     delay-slot instruction wins; LWL/LWR forward the pending load),
 *   - COP0: SR/CAUSE/EPC/BADVADDR/PRID, exceptions (Int, AdEL, AdES, Sys,
 *     Bp, RI, CpU, Ov) with the KU/IE 3-level stack, RFE, BEV vector select,
 *   - COP2 (GTE) moves, LWC2/SWC2 and commands,
 *   - SR.IsC cache isolation (stores swallowed),
 *   - HLE BIOS vectors 0xA0/0xB0/0xC0 + 0x80000080 when no BIOS image.
 * Timing model: 2 cycles per instruction (documented approximation).
 */
#include "psx_internal.h"

#define R(n) (c->r[(n)])

static inline void set_reg(psx_cpu_t *c, uint32_t r, uint32_t v)
{
    c->r[r] = v;
    c->r[0] = 0;
    if (c->ld_reg == r)
        c->ld_reg = 0;     /* the instruction's own write beats the pending load */
}

void psx_cpu_reset(hs_psx_t *s, uint32_t pc)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t i;
    for (i = 0; i < 32; i++) { c->r[i] = 0; c->cop0[i] = 0; c->gd[i] = 0; c->gc[i] = 0; }
    c->pc = pc;
    c->next_pc = pc + 4u;
    c->hi = c->lo = 0;
    c->ld_reg = c->ld_val = 0;
    c->in_delay = c->branch_taken = 0;
    c->cop0[C0_SR] = 0x10400000u;     /* BEV=1, CU2 usable after BIOS init */
    c->cop0[C0_PRID] = 0x00000002u;
    c->cycles = 0;
}

void psx_cpu_exception(hs_psx_t *s, uint32_t code)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t sr = c->cop0[C0_SR];
    uint32_t epc = c->cur_pc;
    uint32_t cause = c->cop0[C0_CAUSE] & ~(0x8000007Cu);
    if (c->in_delay) {
        epc -= 4u;
        cause |= 0x80000000u;
    }
    cause |= (code & 31u) << 2;
    c->cop0[C0_CAUSE] = cause;
    c->cop0[C0_EPC] = epc;
    c->cop0[C0_SR] = (sr & ~0x3Fu) | ((sr << 2) & 0x3Fu);
    /* the pending load still lands */
    if (c->ld_reg) { c->r[c->ld_reg] = c->ld_val; c->ld_reg = 0; }
    c->r[0] = 0;
    c->pc = (sr & (1u << 22)) ? 0xBFC00180u : 0x80000080u;
    c->next_pc = c->pc + 4u;
    c->branch_taken = 0;
    c->in_delay = 0;
}

static inline int cop_usable(psx_cpu_t *c, uint32_t n)
{
    if (n == 0u)
        return 1;   /* COP0 always usable in kernel mode (we run kernel-mode only) */
    return (c->cop0[C0_SR] >> (28u + n)) & 1u;
}

static inline uint32_t fetch(hs_psx_t *s, uint32_t pc)
{
    uint32_t p = pc & 0x1FFFFFFFu;
    if (p < 0x00800000u) {
        const uint8_t *m = s->ram + (p & (HS_PSX_RAM_SIZE - 1u));
        return (uint32_t)m[0] | ((uint32_t)m[1] << 8) | ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24);
    }
    return psx_read32(s, pc);
}

static void do_load(hs_psx_t *s, psx_cpu_t *c, uint32_t rt, uint32_t v,
                    uint32_t *nreg, uint32_t *nval)
{
    (void)s;
    if (c->ld_reg == rt)
        c->ld_reg = 0;            /* superseded */
    *nreg = rt;
    *nval = v;
}

static inline int isolated(const psx_cpu_t *c) { return (c->cop0[C0_SR] >> 16) & 1u; }

static void cop0_write(psx_cpu_t *c, uint32_t rd, uint32_t v)
{
    switch (rd) {
    case C0_SR:    c->cop0[C0_SR] = v; break;
    case C0_CAUSE: c->cop0[C0_CAUSE] = (c->cop0[C0_CAUSE] & ~0x300u) | (v & 0x300u); break;
    case 3: case 5: case 6: case 7: case 9: case 11: c->cop0[rd] = v; break;
    default: break;   /* BADV, EPC, PRID read-only */
    }
}

/* Execute exactly one instruction. */
static void step(hs_psx_t *s)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t in, op, rs, rt, rd, sa, fn, imm, simm, nreg = 0, nval = 0;
    uint32_t pc = c->pc;

    /* external BIOS + PS-EXE: sideload when the BIOS reaches the shell entry */
    if (s->bios && s->exe && !s->exe_loaded && pc == 0x80030000u) {
        if (psx_exe_load(s) == 0) pc = c->pc;
        else s->exe_loaded = 1;
    }
    /* HLE BIOS hooks (only without an external BIOS image) */
    if (!s->bios) {
        uint32_t p = pc & 0x1FFFFFFFu;
        if (p == 0xA0u || p == 0xB0u || p == 0xC0u) {
            if (c->ld_reg) { c->r[c->ld_reg] = c->ld_val; c->ld_reg = 0; }
            c->r[0] = 0;
            if (psx_hle_call(s, p)) {
                c->pc = c->r[31];
                c->next_pc = c->pc + 4u;
                c->branch_taken = 0;
                c->cycles += 8u;
                return;
            }
        } else if (pc == 0x80000080u) {
            if (c->ld_reg) { c->r[c->ld_reg] = c->ld_val; c->ld_reg = 0; }
            if (psx_hle_exception(s)) {
                c->branch_taken = 0;
                c->cycles += 8u;
                return;
            }
        }
    }

    if (pc & 3u) {
        c->cur_pc = pc;
        c->in_delay = c->branch_taken;
        c->cop0[C0_BADV] = pc;
        psx_cpu_exception(s, EXC_ADEL);
        return;
    }

    in = fetch(s, pc);
    c->cur_pc = pc;
    c->in_delay = c->branch_taken;
    c->branch_taken = 0;
    c->pc = c->next_pc;
    c->next_pc = c->pc + 4u;
    c->cycles += 2u;

    op = in >> 26;
    rs = (in >> 21) & 31u;
    rt = (in >> 16) & 31u;
    rd = (in >> 11) & 31u;
    sa = (in >> 6) & 31u;
    fn = in & 63u;
    imm = in & 0xFFFFu;
    simm = (uint32_t)(int32_t)(int16_t)imm;

#define BRANCH(cond) do { if (cond) { c->next_pc = c->pc + (simm << 2); } c->branch_taken = 1; } while (0)
    /* Note: the delay slot executes whether or not the branch is taken;
     * branch_taken marks "next instruction is a delay slot". */

    switch (op) {
    case 0x00:
        switch (fn) {
        case 0x00: set_reg(c, rd, R(rt) << sa); break;
        case 0x02: set_reg(c, rd, R(rt) >> sa); break;
        case 0x03: set_reg(c, rd, (uint32_t)((int32_t)R(rt) >> sa)); break;
        case 0x04: set_reg(c, rd, R(rt) << (R(rs) & 31u)); break;
        case 0x06: set_reg(c, rd, R(rt) >> (R(rs) & 31u)); break;
        case 0x07: set_reg(c, rd, (uint32_t)((int32_t)R(rt) >> (R(rs) & 31u))); break;
        case 0x08: c->next_pc = R(rs); c->branch_taken = 1; break;              /* JR */
        case 0x09: { uint32_t t = R(rs); set_reg(c, rd, c->pc + 4u);            /* JALR */
                     c->next_pc = t; c->branch_taken = 1; } break;
        case 0x0C: psx_cpu_exception(s, EXC_SYS); return;
        case 0x0D: psx_cpu_exception(s, EXC_BP); return;
        case 0x10: set_reg(c, rd, c->hi); break;
        case 0x11: c->hi = R(rs); break;
        case 0x12: set_reg(c, rd, c->lo); break;
        case 0x13: c->lo = R(rs); break;
        case 0x18: { int64_t p = (int64_t)(int32_t)R(rs) * (int64_t)(int32_t)R(rt);
                     c->lo = (uint32_t)(uint64_t)p; c->hi = (uint32_t)((uint64_t)p >> 32); } break;
        case 0x19: { uint64_t p = (uint64_t)R(rs) * (uint64_t)R(rt);
                     c->lo = (uint32_t)p; c->hi = (uint32_t)(p >> 32); } break;
        case 0x1A: { int32_t n = (int32_t)R(rs), d = (int32_t)R(rt);
                     if (d == 0) { c->hi = (uint32_t)n; c->lo = n >= 0 ? 0xFFFFFFFFu : 1u; }
                     else if ((uint32_t)n == 0x80000000u && d == -1) { c->hi = 0; c->lo = 0x80000000u; }
                     else { c->lo = (uint32_t)(n / d); c->hi = (uint32_t)(n % d); } } break;
        case 0x1B: { uint32_t n = R(rs), d = R(rt);
                     if (d == 0) { c->hi = n; c->lo = 0xFFFFFFFFu; }
                     else { c->lo = n / d; c->hi = n % d; } } break;
        case 0x20: { uint32_t a = R(rs), b = R(rt), r = a + b;
                     if (((a ^ r) & (b ^ r)) >> 31) { psx_cpu_exception(s, EXC_OV); return; }
                     set_reg(c, rd, r); } break;
        case 0x21: set_reg(c, rd, R(rs) + R(rt)); break;
        case 0x22: { uint32_t a = R(rs), b = R(rt), r = a - b;
                     if (((a ^ b) & (a ^ r)) >> 31) { psx_cpu_exception(s, EXC_OV); return; }
                     set_reg(c, rd, r); } break;
        case 0x23: set_reg(c, rd, R(rs) - R(rt)); break;
        case 0x24: set_reg(c, rd, R(rs) & R(rt)); break;
        case 0x25: set_reg(c, rd, R(rs) | R(rt)); break;
        case 0x26: set_reg(c, rd, R(rs) ^ R(rt)); break;
        case 0x27: set_reg(c, rd, ~(R(rs) | R(rt))); break;
        case 0x2A: set_reg(c, rd, (int32_t)R(rs) < (int32_t)R(rt) ? 1u : 0u); break;
        case 0x2B: set_reg(c, rd, R(rs) < R(rt) ? 1u : 0u); break;
        default: psx_cpu_exception(s, EXC_RI); return;
        }
        break;
    case 0x01: { /* BcondZ: bit16 = GEZ, bits 20..17 == 8 -> link */
        int32_t v = (int32_t)R(rs);
        int ge = (rt & 1u) != 0;
        int cond = ge ? (v >= 0) : (v < 0);
        if ((rt & 0x1Eu) == 0x10u)
            set_reg(c, 31, c->pc + 4u);
        BRANCH(cond);
        } break;
    case 0x02: c->next_pc = (c->pc & 0xF0000000u) | ((in & 0x3FFFFFFu) << 2); c->branch_taken = 1; break;
    case 0x03: set_reg(c, 31, c->pc + 4u);
               c->next_pc = (c->pc & 0xF0000000u) | ((in & 0x3FFFFFFu) << 2); c->branch_taken = 1; break;
    case 0x04: BRANCH(R(rs) == R(rt)); break;
    case 0x05: BRANCH(R(rs) != R(rt)); break;
    case 0x06: BRANCH((int32_t)R(rs) <= 0); break;
    case 0x07: BRANCH((int32_t)R(rs) > 0); break;
    case 0x08: { uint32_t a = R(rs), r = a + simm;
                 if (((a ^ r) & (simm ^ r)) >> 31) { psx_cpu_exception(s, EXC_OV); return; }
                 set_reg(c, rt, r); } break;
    case 0x09: set_reg(c, rt, R(rs) + simm); break;
    case 0x0A: set_reg(c, rt, (int32_t)R(rs) < (int32_t)simm ? 1u : 0u); break;
    case 0x0B: set_reg(c, rt, R(rs) < simm ? 1u : 0u); break;
    case 0x0C: set_reg(c, rt, R(rs) & imm); break;
    case 0x0D: set_reg(c, rt, R(rs) | imm); break;
    case 0x0E: set_reg(c, rt, R(rs) ^ imm); break;
    case 0x0F: set_reg(c, rt, imm << 16); break;
    case 0x10: /* COP0 */
        if (rs == 0x00) { do_load(s, c, rt, c->cop0[rd], &nreg, &nval); }        /* MFC0 */
        else if (rs == 0x04) { cop0_write(c, rd, R(rt)); }                         /* MTC0 */
        else if (rs == 0x10 && fn == 0x10) {                                       /* RFE */
            uint32_t sr = c->cop0[C0_SR];
            c->cop0[C0_SR] = (sr & ~0xFu) | ((sr >> 2) & 0xFu);
        } else { psx_cpu_exception(s, EXC_RI); return; }
        break;
    case 0x11: case 0x13:
        c->cop0[C0_CAUSE] = (c->cop0[C0_CAUSE] & ~0x30000000u) | ((op & 3u) << 28);
        psx_cpu_exception(s, EXC_CPU); return;
    case 0x12: /* COP2 = GTE */
        if (!cop_usable(c, 2)) {
            c->cop0[C0_CAUSE] = (c->cop0[C0_CAUSE] & ~0x30000000u) | (2u << 28);
            psx_cpu_exception(s, EXC_CPU); return;
        }
        if (in & (1u << 25)) { psx_gte_command(c, in & 0x1FFFFFFu); c->cycles += 6u; }
        else if (rs == 0x00) do_load(s, c, rt, psx_gte_read_data(c, rd), &nreg, &nval);   /* MFC2 */
        else if (rs == 0x02) do_load(s, c, rt, psx_gte_read_ctrl(c, rd), &nreg, &nval);   /* CFC2 */
        else if (rs == 0x04) psx_gte_write_data(c, rd, R(rt));                            /* MTC2 */
        else if (rs == 0x06) psx_gte_write_ctrl(c, rd, R(rt));                            /* CTC2 */
        else { psx_cpu_exception(s, EXC_RI); return; }
        break;
    case 0x20: { uint32_t a = R(rs) + simm;                                                /* LB */
                 uint32_t v = (uint32_t)(int32_t)(int8_t)psx_read8(s, a);
                 do_load(s, c, rt, v, &nreg, &nval); } break;
    case 0x21: { uint32_t a = R(rs) + simm;                                                /* LH */
                 if (a & 1u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADEL); return; }
                 do_load(s, c, rt, (uint32_t)(int32_t)(int16_t)psx_read16(s, a), &nreg, &nval); } break;
    case 0x22: case 0x26: {                                                                /* LWL / LWR */
                 uint32_t a = R(rs) + simm;
                 uint32_t cur = (c->ld_reg == rt) ? c->ld_val : R(rt);
                 uint32_t w = psx_read32(s, a & ~3u), v, sh = a & 3u;
                 if (op == 0x22) v = (cur & (0x00FFFFFFu >> (sh * 8u))) | (w << (24u - sh * 8u));
                 else            v = (cur & (0xFFFFFF00u << ((3u - sh) * 8u))) | (w >> (sh * 8u));
                 if (c->ld_reg == rt) c->ld_reg = 0;
                 nreg = rt; nval = v; } break;
    case 0x23: { uint32_t a = R(rs) + simm;                                                /* LW */
                 if (a & 3u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADEL); return; }
                 do_load(s, c, rt, psx_read32(s, a), &nreg, &nval); } break;
    case 0x24: { uint32_t a = R(rs) + simm; do_load(s, c, rt, psx_read8(s, a) & 0xFFu, &nreg, &nval); } break;
    case 0x25: { uint32_t a = R(rs) + simm;
                 if (a & 1u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADEL); return; }
                 do_load(s, c, rt, psx_read16(s, a) & 0xFFFFu, &nreg, &nval); } break;
    case 0x28: { uint32_t a = R(rs) + simm; if (!isolated(c)) psx_write8(s, a, R(rt) & 0xFFu); } break;
    case 0x29: { uint32_t a = R(rs) + simm;
                 if (a & 1u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADES); return; }
                 if (!isolated(c)) psx_write16(s, a, R(rt) & 0xFFFFu); } break;
    case 0x2A: case 0x2E: {                                                                /* SWL / SWR */
                 uint32_t a = R(rs) + simm, sh = a & 3u, w, v = R(rt);
                 if (isolated(c)) break;
                 w = psx_read32(s, a & ~3u);
                 if (op == 0x2A) w = (w & (0xFFFFFF00u << (sh * 8u))) | (v >> (24u - sh * 8u));
                 else            w = (w & (0x00FFFFFFu >> ((3u - sh) * 8u))) | (v << (sh * 8u));
                 psx_write32(s, a & ~3u, w); } break;
    case 0x2B: { uint32_t a = R(rs) + simm;
                 if (a & 3u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADES); return; }
                 if (!isolated(c)) psx_write32(s, a, R(rt)); } break;
    case 0x32: { uint32_t a = R(rs) + simm;                                                /* LWC2 */
                 if (a & 3u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADEL); return; }
                 psx_gte_write_data(c, rt, psx_read32(s, a)); } break;
    case 0x3A: { uint32_t a = R(rs) + simm;                                                /* SWC2 */
                 if (a & 3u) { c->cop0[C0_BADV] = a; psx_cpu_exception(s, EXC_ADES); return; }
                 if (!isolated(c)) psx_write32(s, a, psx_gte_read_data(c, rt)); } break;
    case 0x30: case 0x31: case 0x33: case 0x38: case 0x39: case 0x3B:
        c->cop0[C0_CAUSE] = (c->cop0[C0_CAUSE] & ~0x30000000u) | ((op & 3u) << 28);
        psx_cpu_exception(s, EXC_CPU); return;
    default:
        psx_cpu_exception(s, EXC_RI); return;
    }
#undef BRANCH
    /* retire: the previous instruction's load lands now, this one's is queued */
    if (c->ld_reg) c->r[c->ld_reg] = c->ld_val;
    c->ld_reg = nreg;
    c->ld_val = nval;
    c->r[0] = 0;
}

static int irq_ready(hs_psx_t *s)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t sr = c->cop0[C0_SR];
    if (psx_irq_pending(s)) c->cop0[C0_CAUSE] |= 0x400u;
    else                    c->cop0[C0_CAUSE] &= ~0x400u;
    return (sr & 1u) && (sr & c->cop0[C0_CAUSE] & 0xFF00u);
}

void psx_cpu_run(hs_psx_t *s, uint32_t cycles)
{
    psx_cpu_t *c = &s->cpu;
    uint64_t end = c->cycles + cycles;
    uint32_t guard = cycles;            /* static bound: <= 1 instruction per cycle */
    while (c->cycles < end && guard--) {
        uint64_t before = c->cycles;
        if (irq_ready(s)) {
            /* hardware executes a pending GTE command before taking the IRQ */
            uint32_t nxt = fetch(s, c->pc);
            if ((nxt >> 25) == 0x25u)
                step(s);
            c->cur_pc = c->pc;
            c->in_delay = c->branch_taken;
            psx_cpu_exception(s, EXC_INT);
        }
        step(s);
        psx_bus_tick(s, (uint32_t)(c->cycles - before));
    }
}
