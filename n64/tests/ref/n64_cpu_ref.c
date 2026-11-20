/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_cpu_ref.c - FROZEN REFERENCE: the hs-n64 v0 VR4300 interpreter exactly
 * as it shipped in hs-cores e15d81a (src/n64_cpu.c), unmodified, kept only so
 * tests/test_n64_equiv.c can run it in lockstep with the optimised core.
 * Compiled through n64_ref.c with renamed entry points. Not linked into the core.
 *
 * Original description: clean-room NEC VR4300 (MIPS III) interpreter for hs-n64.
 *   - 64-bit GPRs, HI/LO, branch + branch-likely with exact delay slots
 *   - all integer loads/stores incl. LWL/LWR/SWL/SWR/LDL/LDR/SDL/SDR, LL/SC
 *   - COP0: Count/Compare timer, Status/Cause/EPC, 32-entry TLB, exceptions
 *   - COP1: IEEE-754 S/D/W/L, FR=0/1 register files, canonical VR4300 NaNs
 * Written from the public MIPS III / VR4300 architecture description.
 * No host floating-point environment is touched: arithmetic is IEEE
 * round-to-nearest (bit-identical on x86_64 SSE2 and AArch64 with
 * -ffp-contract=off); conversions honour FCR31.RM exactly in software.
 */
#include <string.h>
#include <math.h>
#include "n64_internal.h"

#define SX32(v) ((uint64_t)(int64_t)(int32_t)(uint32_t)(v))
#define RS ((ins >> 21) & 31u)
#define RT ((ins >> 16) & 31u)
#define RD ((ins >> 11) & 31u)
#define SA ((ins >> 6) & 31u)
#define IMM16 ((uint64_t)(int64_t)(int16_t)(uint16_t)ins)
#define UIMM16 ((uint64_t)(uint16_t)ins)

/* ---------------------------------------------------------------- reset */
void n64_cpu_reset(hs_n64_t *n)
{
    n64_cpu_t *c = &n->cpu;
    memset(c, 0, sizeof(*c));
    c->pc = SX32(0xBFC00000u);
    c->npc = c->pc + 4u;
    c->cp0[C0_STATUS] = 0x34000000u | ST_ERL | ST_BEV;
    c->cp0[C0_RANDOM] = 31u;
    c->cp0[C0_PRID] = 0x00000B22u;
    c->cp0[C0_CONFIG] = 0x7006E463u;
    c->fcr31 = 0;
}

void n64_cpu_update_irq(hs_n64_t *n)
{
    uint64_t *cause = &n->cpu.cp0[C0_CAUSE];
    if (n->mi_intr & n->mi_mask) *cause |= 0x400u;
    else *cause &= ~(uint64_t)0x400u;
}

/* ----------------------------------------------------------- exceptions */
static void take_exc(hs_n64_t *n, uint32_t code, uint32_t vec_off, uint32_t ce)
{
    n64_cpu_t *c = &n->cpu;
    uint64_t status = c->cp0[C0_STATUS];
    uint64_t cause = c->cp0[C0_CAUSE];
    uint32_t base;
    if (!(status & ST_EXL)) {
        if (c->in_delay) { c->cp0[C0_EPC] = c->pc - 4u; cause |= 0x80000000u; }
        else             { c->cp0[C0_EPC] = c->pc;      cause &= ~(uint64_t)0x80000000u; }
    } else {
        vec_off = 0x180u;
    }
    cause = (cause & ~(uint64_t)0x3000007Cu) | ((uint64_t)code << 2) | ((uint64_t)ce << 28);
    c->cp0[C0_CAUSE] = cause;
    c->cp0[C0_STATUS] = status | ST_EXL;
    base = (status & ST_BEV) ? 0xBFC00200u : 0x80000000u;
    c->pc = SX32(base + vec_off);
    c->npc = c->pc + 4u;
    c->next_delay = 0;
    c->exc = 1;
}

static void addr_exc(hs_n64_t *n, uint64_t va, uint32_t code, int refill)
{
    n64_cpu_t *c = &n->cpu;
    c->cp0[C0_BADVADDR] = va;
    if (code == EXC_TLBL || code == EXC_TLBS || code == EXC_MOD) {
        c->cp0[C0_CONTEXT] = (c->cp0[C0_CONTEXT] & ~(uint64_t)0x7FFFF0u) | (((va >> 13) & 0x7FFFFu) << 4);
        c->cp0[C0_ENTRYHI] = (va & ~(uint64_t)0x1FFFu) | (c->cp0[C0_ENTRYHI] & 0xFFu);
    }
    take_exc(n, code, refill ? 0u : 0x180u, 0);
}

/* ------------------------------------------------- address translation */
static int translate(hs_n64_t *n, uint64_t va, int write, uint32_t *pa)
{
    n64_cpu_t *c = &n->cpu;
    uint32_t v = (uint32_t)va;
    uint32_t i;
    if ((v & 0xC0000000u) == 0x80000000u) { *pa = v & 0x1FFFFFFFu; return 1; }
    for (i = 0; i < 32u; i++) {
        const n64_tlb_entry_t *e = &c->tlb[i];
        uint32_t mask = e->pagemask | 0x1FFFu;
        uint64_t lo;
        int global = (int)(e->lo0 & e->lo1 & 1u);
        if ((v & ~mask) != ((uint32_t)e->entryhi & ~mask)) continue;
        if (!global && (e->entryhi & 0xFFu) != (c->cp0[C0_ENTRYHI] & 0xFFu)) continue;
        lo = (v & ((mask + 1u) >> 1)) ? e->lo1 : e->lo0;
        if (!(lo & 2u)) { addr_exc(n, va, write ? EXC_TLBS : EXC_TLBL, 0); return 0; }
        if (write && !(lo & 4u)) { addr_exc(n, va, EXC_MOD, 0); return 0; }
        *pa = (uint32_t)((((lo >> 6) & 0xFFFFFu) << 12) | (v & (mask >> 1)));
        return 1;
    }
    addr_exc(n, va, write ? EXC_TLBS : EXC_TLBL, (c->cp0[C0_STATUS] & ST_EXL) ? 0 : 1);
    return 0;
}

/* ---------------------------------------------------------- memory */
static int vload(hs_n64_t *n, uint64_t va, uint32_t size, uint64_t *out)
{
    uint32_t pa;
    if (va & (size - 1u)) { addr_exc(n, va, EXC_ADEL, 0); return 0; }
    if (!translate(n, va, 0, &pa)) return 0;
    if (pa + size <= n->rdram_size) {
        const uint8_t *p = n->rdram + pa;
        switch (size) {
        case 1: *out = p[0]; break;
        case 2: *out = n64_be16(p); break;
        case 4: *out = n64_be32(p); break;
        default: *out = ((uint64_t)n64_be32(p) << 32) | n64_be32(p + 4); break;
        }
        return 1;
    }
    if (size == 8) {
        *out = ((uint64_t)n64_phys_read32(n, pa) << 32) | n64_phys_read32(n, pa + 4u);
    } else {
        uint32_t w = n64_phys_read32(n, pa & ~3u);
        uint32_t sh = (4u - size - (pa & 3u)) * 8u;
        *out = (w >> sh) & (size == 4 ? 0xFFFFFFFFu : ((1u << (size * 8u)) - 1u));
    }
    return 1;
}

static int vstore(hs_n64_t *n, uint64_t va, uint32_t size, uint64_t val)
{
    uint32_t pa;
    if (va & (size - 1u)) { addr_exc(n, va, EXC_ADES, 0); return 0; }
    if (!translate(n, va, 1, &pa)) return 0;
    if (pa + size <= n->rdram_size) {
        uint8_t *p = n->rdram + pa;
        switch (size) {
        case 1: p[0] = (uint8_t)val; break;
        case 2: n64_wbe16(p, (uint16_t)val); break;
        case 4: n64_wbe32(p, (uint32_t)val); break;
        default: n64_wbe32(p, (uint32_t)(val >> 32)); n64_wbe32(p + 4, (uint32_t)val); break;
        }
        return 1;
    }
    if (size == 8) {
        n64_phys_write32(n, pa, (uint32_t)(val >> 32), 0xFFFFFFFFu);
        n64_phys_write32(n, pa + 4u, (uint32_t)val, 0xFFFFFFFFu);
    } else {
        uint32_t sh = (4u - size - (pa & 3u)) * 8u;
        uint32_t m = (size == 4 ? 0xFFFFFFFFu : ((1u << (size * 8u)) - 1u)) << sh;
        n64_phys_write32(n, pa & ~3u, (uint32_t)(val << sh), m);
    }
    return 1;
}

/* word-granular helpers for the unaligned load/store family */
static int load_aligned(hs_n64_t *n, uint64_t va, uint32_t size, uint64_t *out)
{ return vload(n, va & ~(uint64_t)(size - 1u), size, out); }
static int store_aligned(hs_n64_t *n, uint64_t va, uint32_t size, uint64_t v)
{ return vstore(n, va & ~(uint64_t)(size - 1u), size, v); }

/* ------------------------------------------------------- 128-bit mul */
static void mulu64(uint64_t a, uint64_t b, uint64_t *hi, uint64_t *lo)
{
    uint64_t a0 = a & 0xFFFFFFFFu, a1 = a >> 32, b0 = b & 0xFFFFFFFFu, b1 = b >> 32;
    uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    uint64_t mid = (p00 >> 32) + (p01 & 0xFFFFFFFFu) + (p10 & 0xFFFFFFFFu);
    *lo = (p00 & 0xFFFFFFFFu) | (mid << 32);
    *hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
}

static void muls64(int64_t a, int64_t b, uint64_t *hi, uint64_t *lo)
{
    uint64_t ua = (uint64_t)a, ub = (uint64_t)b, h, l;
    mulu64(ua, ub, &h, &l);
    if (a < 0) h -= ub;
    if (b < 0) h -= ua;
    *hi = h; *lo = l;
}

/* ------------------------------------------------------------- COP1 */
#define FP_QNAN_S 0x7FBFFFFFu
#define FP_QNAN_D 0x7FF7FFFFFFFFFFFFull

static uint32_t fgetw(const n64_cpu_t *c, uint32_t r)
{
    if (c->cp0[C0_STATUS] & ST_FR) return (uint32_t)c->fpr[r];
    return (r & 1u) ? (uint32_t)(c->fpr[r & ~1u] >> 32) : (uint32_t)c->fpr[r];
}
static void fsetw(n64_cpu_t *c, uint32_t r, uint32_t v)
{
    if (c->cp0[C0_STATUS] & ST_FR) { c->fpr[r] = (c->fpr[r] & 0xFFFFFFFF00000000ull) | v; return; }
    if (r & 1u) c->fpr[r & ~1u] = (c->fpr[r & ~1u] & 0xFFFFFFFFu) | ((uint64_t)v << 32);
    else        c->fpr[r] = (c->fpr[r] & 0xFFFFFFFF00000000ull) | v;
}
static uint64_t fgetl(const n64_cpu_t *c, uint32_t r)
{ return (c->cp0[C0_STATUS] & ST_FR) ? c->fpr[r] : c->fpr[r & ~1u]; }
static void fsetl(n64_cpu_t *c, uint32_t r, uint64_t v)
{ if (c->cp0[C0_STATUS] & ST_FR) c->fpr[r] = v; else c->fpr[r & ~1u] = v; }

static float  b2f(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
static double b2d(uint64_t b) { double d; memcpy(&d, &b, 8); return d; }
static uint32_t f2b(float f)  { uint32_t b; if (f != f) return FP_QNAN_S; memcpy(&b, &f, 4); return b; }
static uint64_t d2b(double d) { uint64_t b; if (d != d) return FP_QNAN_D; memcpy(&b, &d, 8); return b; }

/* software rounding of a double to integer under rm (0 RN,1 RZ,2 RP,3 RM) */
static int64_t round_i64(double v, uint32_t rm, int *inv)
{
    int64_t t;
    double fr;
    *inv = 0;
    if (v != v || v >= 9223372036854775808.0 || v < -9223372036854775808.0) { *inv = 1; return INT64_MAX; }
    t = (int64_t)v;
    fr = v - (double)t;
    switch (rm & 3u) {
    case 1: break;
    case 2: if (fr > 0.0) t++; break;
    case 3: if (fr < 0.0) t--; break;
    default:
        if (fr > 0.5 || (fr == 0.5 && (t & 1))) t++;
        else if (fr < -0.5 || (fr == -0.5 && (t & 1))) t--;
        break;
    }
    return t;
}

uint32_t n64_fpu_round_w(double v, uint32_t rm, int32_t *out)
{
    int inv;
    int64_t t = round_i64(v, rm, &inv);
    if (inv || t > 2147483647LL || t < -2147483648LL) { *out = 0x7FFFFFFF; return 1; }
    *out = (int32_t)t;
    return 0;
}

static void cop1_exec(hs_n64_t *n, uint32_t ins, int *branch, uint64_t *target, int *likely_skip)
{
    n64_cpu_t *c = &n->cpu;
    uint32_t fmt = RS, ft = RT, fs = RD, fd = SA, fn = ins & 63u;
    if (!(c->cp0[C0_STATUS] & ST_CU1)) { take_exc(n, EXC_CPU, 0x180u, 1); return; }
    switch (fmt) {
    case 0x00: c->gpr[ft] = SX32(fgetw(c, fs)); return;                 /* MFC1 */
    case 0x01: c->gpr[ft] = fgetl(c, fs); return;                       /* DMFC1 */
    case 0x02: c->gpr[ft] = SX32(fs == 31u ? c->fcr31 : (fs == 0u ? 0x00000A00u : 0u)); return; /* CFC1 */
    case 0x04: fsetw(c, fs, (uint32_t)c->gpr[ft]); return;              /* MTC1 */
    case 0x05: fsetl(c, fs, c->gpr[ft]); return;                        /* DMTC1 */
    case 0x06: if (fs == 31u) c->fcr31 = (uint32_t)c->gpr[ft] & 0x0183FFFFu; return; /* CTC1 */
    case 0x08: {                                                        /* BC1 */
        uint32_t cc = (c->fcr31 >> 23) & 1u;
        uint32_t tf = (ins >> 16) & 1u, lk = (ins >> 17) & 1u;
        if (cc == tf) { *branch = 1; *target = c->pc + 4u + (IMM16 << 2); }
        else if (lk) *likely_skip = 1;
        return;
    }
    default: break;
    }
    if (fmt == 16u) {                    /* S */
        float a = b2f(fgetw(c, fs)), b = b2f(fgetw(c, ft));
        int32_t w; int inv; int64_t l;
        switch (fn) {
        case 0x00: fsetw(c, fd, f2b(a + b)); return;
        case 0x01: fsetw(c, fd, f2b(a - b)); return;
        case 0x02: fsetw(c, fd, f2b(a * b)); return;
        case 0x03: fsetw(c, fd, f2b(a / b)); return;
        case 0x04: fsetw(c, fd, f2b(sqrtf(a))); return;
        case 0x05: fsetw(c, fd, fgetw(c, fs) & 0x7FFFFFFFu); return;
        case 0x06: fsetw(c, fd, fgetw(c, fs)); return;
        case 0x07: fsetw(c, fd, fgetw(c, fs) ^ 0x80000000u); return;
        case 0x08: case 0x09: case 0x0A: case 0x0B:
            l = round_i64((double)a, fn & 3u, &inv); fsetl(c, fd, inv ? 0x7FFFFFFFFFFFFFFFull : (uint64_t)l); return;
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
            n64_fpu_round_w((double)a, fn & 3u, &w); fsetw(c, fd, (uint32_t)w); return;
        case 0x21: fsetl(c, fd, d2b((double)a)); return;
        case 0x24: n64_fpu_round_w((double)a, c->fcr31 & 3u, &w); fsetw(c, fd, (uint32_t)w); return;
        case 0x25: l = round_i64((double)a, c->fcr31 & 3u, &inv); fsetl(c, fd, inv ? 0x7FFFFFFFFFFFFFFFull : (uint64_t)l); return;
        default: break;
        }
        if (fn >= 0x30u) {
            uint32_t cond = fn & 15u, r = 0;
            int un = (a != a) || (b != b);
            if ((cond & 1u) && un) r = 1;
            if ((cond & 2u) && !un && a == b) r = 1;
            if ((cond & 4u) && !un && a < b) r = 1;
            c->fcr31 = (c->fcr31 & ~0x00800000u) | (r << 23);
            return;
        }
    } else if (fmt == 17u) {             /* D */
        double a = b2d(fgetl(c, fs)), b = b2d(fgetl(c, ft));
        int32_t w; int inv; int64_t l;
        switch (fn) {
        case 0x00: fsetl(c, fd, d2b(a + b)); return;
        case 0x01: fsetl(c, fd, d2b(a - b)); return;
        case 0x02: fsetl(c, fd, d2b(a * b)); return;
        case 0x03: fsetl(c, fd, d2b(a / b)); return;
        case 0x04: fsetl(c, fd, d2b(sqrt(a))); return;
        case 0x05: fsetl(c, fd, fgetl(c, fs) & 0x7FFFFFFFFFFFFFFFull); return;
        case 0x06: fsetl(c, fd, fgetl(c, fs)); return;
        case 0x07: fsetl(c, fd, fgetl(c, fs) ^ 0x8000000000000000ull); return;
        case 0x08: case 0x09: case 0x0A: case 0x0B:
            l = round_i64(a, fn & 3u, &inv); fsetl(c, fd, inv ? 0x7FFFFFFFFFFFFFFFull : (uint64_t)l); return;
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
            n64_fpu_round_w(a, fn & 3u, &w); fsetw(c, fd, (uint32_t)w); return;
        case 0x20: fsetw(c, fd, f2b((float)a)); return;
        case 0x24: n64_fpu_round_w(a, c->fcr31 & 3u, &w); fsetw(c, fd, (uint32_t)w); return;
        case 0x25: l = round_i64(a, c->fcr31 & 3u, &inv); fsetl(c, fd, inv ? 0x7FFFFFFFFFFFFFFFull : (uint64_t)l); return;
        default: break;
        }
        if (fn >= 0x30u) {
            uint32_t cond = fn & 15u, r = 0;
            int un = (a != a) || (b != b);
            if ((cond & 1u) && un) r = 1;
            if ((cond & 2u) && !un && a == b) r = 1;
            if ((cond & 4u) && !un && a < b) r = 1;
            c->fcr31 = (c->fcr31 & ~0x00800000u) | (r << 23);
            return;
        }
    } else if (fmt == 20u) {             /* W */
        int32_t v = (int32_t)fgetw(c, fs);
        if (fn == 0x20) { fsetw(c, fd, f2b((float)v)); return; }
        if (fn == 0x21) { fsetl(c, fd, d2b((double)v)); return; }
    } else if (fmt == 21u) {             /* L */
        int64_t v = (int64_t)fgetl(c, fs);
        if (fn == 0x20) { fsetw(c, fd, f2b((float)v)); return; }
        if (fn == 0x21) { fsetl(c, fd, d2b((double)v)); return; }
    }
    /* unimplemented operation */
    c->fcr31 |= 0x00020000u;
    take_exc(n, EXC_FPE, 0x180u, 0);
}

/* ------------------------------------------------------------- COP0 */
static void tlb_write(n64_cpu_t *c, uint32_t i)
{
    n64_tlb_entry_t *e = &c->tlb[i & 31u];
    e->pagemask = (uint32_t)c->cp0[C0_PAGEMASK] & 0x01FFE000u;
    e->entryhi = c->cp0[C0_ENTRYHI] & ~(uint64_t)e->pagemask & ~(uint64_t)0x1F00u;
    e->lo0 = c->cp0[C0_ENTRYLO0] & 0x3FFFFFFFu;
    e->lo1 = c->cp0[C0_ENTRYLO1] & 0x3FFFFFFFu;
}

static int cop0_exec(hs_n64_t *n, uint32_t ins)
{
    n64_cpu_t *c = &n->cpu;
    uint32_t i;
    switch (RS) {
    case 0x00: c->gpr[RT] = SX32(c->cp0[RD]); return 0;                 /* MFC0 */
    case 0x01: c->gpr[RT] = c->cp0[RD]; return 0;                       /* DMFC0 */
    case 0x04: case 0x05: {                                             /* MTC0/DMTC0 */
        uint64_t v = (RS == 0x04) ? SX32(c->gpr[RT]) : c->gpr[RT];
        switch (RD) {
        case C0_CAUSE: c->cp0[C0_CAUSE] = (c->cp0[C0_CAUSE] & ~(uint64_t)0x300u) | (v & 0x300u); break;
        case C0_COMPARE: c->cp0[C0_COMPARE] = v & 0xFFFFFFFFu; c->cp0[C0_CAUSE] &= ~(uint64_t)0x8000u; break;
        case C0_COUNT: c->cp0[C0_COUNT] = v & 0xFFFFFFFFu; break;
        case C0_WIRED: c->cp0[C0_WIRED] = v & 31u; c->cp0[C0_RANDOM] = 31u; break;
        case C0_RANDOM: case C0_PRID: case C0_BADVADDR: break;
        default: c->cp0[RD] = v; break;
        }
        return 0;
    }
    case 0x10:                                                          /* CO */
        switch (ins & 63u) {
        case 0x01:                                                      /* TLBR */
            i = (uint32_t)c->cp0[C0_INDEX] & 31u;
            c->cp0[C0_PAGEMASK] = c->tlb[i].pagemask;
            c->cp0[C0_ENTRYHI] = c->tlb[i].entryhi;
            c->cp0[C0_ENTRYLO0] = c->tlb[i].lo0;
            c->cp0[C0_ENTRYLO1] = c->tlb[i].lo1;
            return 0;
        case 0x02: tlb_write(c, (uint32_t)c->cp0[C0_INDEX]); return 0;  /* TLBWI */
        case 0x06: tlb_write(c, (uint32_t)c->cp0[C0_RANDOM]); return 0; /* TLBWR */
        case 0x08:                                                      /* TLBP */
            c->cp0[C0_INDEX] = 0x80000000u;
            for (i = 0; i < 32u; i++) {
                const n64_tlb_entry_t *e = &c->tlb[i];
                uint64_t m = ~(uint64_t)(e->pagemask | 0x1FFFu);
                if (((e->entryhi ^ c->cp0[C0_ENTRYHI]) & m & 0xFFFFFFFFull) == 0 &&
                    ((e->lo0 & e->lo1 & 1u) || ((e->entryhi ^ c->cp0[C0_ENTRYHI]) & 0xFFu) == 0)) {
                    c->cp0[C0_INDEX] = i;
                    break;
                }
            }
            return 0;
        case 0x18:                                                      /* ERET */
            if (c->cp0[C0_STATUS] & ST_ERL) { c->pc = c->cp0[C0_ERROREPC]; c->cp0[C0_STATUS] &= ~(uint64_t)ST_ERL; }
            else { c->pc = c->cp0[C0_EPC]; c->cp0[C0_STATUS] &= ~(uint64_t)ST_EXL; }
            c->npc = c->pc + 4u;
            c->llbit = 0;
            return 1;
        default: return 0;
        }
    default: return 0;
    }
}

/* ------------------------------------------------------------- trap */
static int trap_if(hs_n64_t *n, int cond)
{
    if (cond) { take_exc(n, EXC_TR, 0x180u, 0); return 1; }
    return 0;
}

/* --------------------------------------------------------- execute */
/* return: 0 = normal advance, 1 = pc/npc already set (exception / ERET) */
static int exec_one(hs_n64_t *n, uint32_t ins)
{
    n64_cpu_t *c = &n->cpu;
    uint64_t *r = c->gpr;
    int branch = 0, likely_skip = 0;
    uint64_t target = 0, v;
    uint32_t op = ins >> 26;

    switch (op) {
    case 0x00: {                                       /* SPECIAL */
        uint32_t fn = ins & 63u;
        switch (fn) {
        case 0x00: r[RD] = SX32((uint32_t)r[RT] << SA); break;
        case 0x02: r[RD] = SX32((uint32_t)r[RT] >> SA); break;
        case 0x03: r[RD] = SX32((uint32_t)((int32_t)(uint32_t)r[RT] >> SA)); break;
        case 0x04: r[RD] = SX32((uint32_t)r[RT] << (r[RS] & 31u)); break;
        case 0x06: r[RD] = SX32((uint32_t)r[RT] >> (r[RS] & 31u)); break;
        case 0x07: r[RD] = SX32((uint32_t)((int32_t)(uint32_t)r[RT] >> (r[RS] & 31u))); break;
        case 0x08: branch = 1; target = r[RS]; break;
        case 0x09: target = r[RS]; r[RD] = c->pc + 8u; branch = 1; break;
        case 0x0C: take_exc(n, EXC_SYS, 0x180u, 0); return 1;
        case 0x0D: take_exc(n, EXC_BP, 0x180u, 0); return 1;
        case 0x0F: break;                              /* SYNC */
        case 0x10: r[RD] = c->hi; break;
        case 0x11: c->hi = r[RS]; break;
        case 0x12: r[RD] = c->lo; break;
        case 0x13: c->lo = r[RS]; break;
        case 0x14: r[RD] = r[RT] << (r[RS] & 63u); break;
        case 0x16: r[RD] = r[RT] >> (r[RS] & 63u); break;
        case 0x17: r[RD] = (uint64_t)((int64_t)r[RT] >> (r[RS] & 63u)); break;
        case 0x18: { int64_t p = (int64_t)(int32_t)(uint32_t)r[RS] * (int64_t)(int32_t)(uint32_t)r[RT];
                     c->lo = SX32((uint32_t)p); c->hi = SX32((uint32_t)((uint64_t)p >> 32)); break; }
        case 0x19: { uint64_t p = (uint64_t)(uint32_t)r[RS] * (uint64_t)(uint32_t)r[RT];
                     c->lo = SX32((uint32_t)p); c->hi = SX32((uint32_t)(p >> 32)); break; }
        case 0x1A: { int32_t a = (int32_t)(uint32_t)r[RS], b = (int32_t)(uint32_t)r[RT];
                     if (b == 0) { c->lo = (a < 0) ? 1u : SX32(0xFFFFFFFFu); c->hi = SX32((uint32_t)a); }
                     else if (a == INT32_MIN && b == -1) { c->lo = SX32(0x80000000u); c->hi = 0; }
                     else { c->lo = SX32((uint32_t)(a / b)); c->hi = SX32((uint32_t)(a % b)); }
                     break; }
        case 0x1B: { uint32_t a = (uint32_t)r[RS], b = (uint32_t)r[RT];
                     if (b == 0) { c->lo = SX32(0xFFFFFFFFu); c->hi = SX32(a); }
                     else { c->lo = SX32(a / b); c->hi = SX32(a % b); }
                     break; }
        case 0x1C: muls64((int64_t)r[RS], (int64_t)r[RT], &c->hi, &c->lo); break;
        case 0x1D: mulu64(r[RS], r[RT], &c->hi, &c->lo); break;
        case 0x1E: { int64_t a = (int64_t)r[RS], b = (int64_t)r[RT];
                     if (b == 0) { c->lo = (a < 0) ? 1u : ~(uint64_t)0; c->hi = (uint64_t)a; }
                     else if (a == INT64_MIN && b == -1) { c->lo = (uint64_t)a; c->hi = 0; }
                     else { c->lo = (uint64_t)(a / b); c->hi = (uint64_t)(a % b); }
                     break; }
        case 0x1F: { uint64_t a = r[RS], b = r[RT];
                     if (b == 0) { c->lo = ~(uint64_t)0; c->hi = a; }
                     else { c->lo = a / b; c->hi = a % b; }
                     break; }
        case 0x20: { int64_t s = (int64_t)(int32_t)(uint32_t)r[RS] + (int64_t)(int32_t)(uint32_t)r[RT];
                     if (s != (int64_t)(int32_t)s) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                     r[RD] = (uint64_t)s; break; }
        case 0x21: r[RD] = SX32((uint32_t)r[RS] + (uint32_t)r[RT]); break;
        case 0x22: { int64_t s = (int64_t)(int32_t)(uint32_t)r[RS] - (int64_t)(int32_t)(uint32_t)r[RT];
                     if (s != (int64_t)(int32_t)s) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                     r[RD] = (uint64_t)s; break; }
        case 0x23: r[RD] = SX32((uint32_t)r[RS] - (uint32_t)r[RT]); break;
        case 0x24: r[RD] = r[RS] & r[RT]; break;
        case 0x25: r[RD] = r[RS] | r[RT]; break;
        case 0x26: r[RD] = r[RS] ^ r[RT]; break;
        case 0x27: r[RD] = ~(r[RS] | r[RT]); break;
        case 0x2A: r[RD] = (int64_t)r[RS] < (int64_t)r[RT]; break;
        case 0x2B: r[RD] = r[RS] < r[RT]; break;
        case 0x2C: { uint64_t a = r[RS], b = r[RT], s = a + b;
                     if ((~(a ^ b) & (a ^ s)) >> 63) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                     r[RD] = s; break; }
        case 0x2D: r[RD] = r[RS] + r[RT]; break;
        case 0x2E: { uint64_t a = r[RS], b = r[RT], s = a - b;
                     if (((a ^ b) & (a ^ s)) >> 63) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                     r[RD] = s; break; }
        case 0x2F: r[RD] = r[RS] - r[RT]; break;
        case 0x30: if (trap_if(n, (int64_t)r[RS] >= (int64_t)r[RT])) return 1; break;
        case 0x31: if (trap_if(n, r[RS] >= r[RT])) return 1; break;
        case 0x32: if (trap_if(n, (int64_t)r[RS] < (int64_t)r[RT])) return 1; break;
        case 0x33: if (trap_if(n, r[RS] < r[RT])) return 1; break;
        case 0x34: if (trap_if(n, r[RS] == r[RT])) return 1; break;
        case 0x36: if (trap_if(n, r[RS] != r[RT])) return 1; break;
        case 0x38: r[RD] = r[RT] << SA; break;
        case 0x3A: r[RD] = r[RT] >> SA; break;
        case 0x3B: r[RD] = (uint64_t)((int64_t)r[RT] >> SA); break;
        case 0x3C: r[RD] = r[RT] << (SA + 32u); break;
        case 0x3E: r[RD] = r[RT] >> (SA + 32u); break;
        case 0x3F: r[RD] = (uint64_t)((int64_t)r[RT] >> (SA + 32u)); break;
        default: c->halted_bad++; take_exc(n, EXC_RI, 0x180u, 0); return 1;
        }
        break;
    }
    case 0x01: {                                       /* REGIMM */
        int64_t s = (int64_t)r[RS];
        uint64_t t = c->pc + 4u + (IMM16 << 2);
        uint32_t rt = RT;
        int take;
        switch (rt) {
        case 0x00: case 0x02: case 0x10: case 0x12: take = s < 0; break;
        case 0x01: case 0x03: case 0x11: case 0x13: take = s >= 0; break;
        case 0x08: if (trap_if(n, s >= (int64_t)IMM16)) return 1; take = -1; break;
        case 0x09: if (trap_if(n, r[RS] >= IMM16)) return 1; take = -1; break;
        case 0x0A: if (trap_if(n, s < (int64_t)IMM16)) return 1; take = -1; break;
        case 0x0B: if (trap_if(n, r[RS] < IMM16)) return 1; take = -1; break;
        case 0x0C: if (trap_if(n, r[RS] == IMM16)) return 1; take = -1; break;
        case 0x0E: if (trap_if(n, r[RS] != IMM16)) return 1; take = -1; break;
        default: c->halted_bad++; take_exc(n, EXC_RI, 0x180u, 0); return 1;
        }
        if (take >= 0) {
            if (rt & 0x10u) r[31] = c->pc + 8u;
            if (take) { branch = 1; target = t; }
            else if (rt & 0x02u) likely_skip = 1;
        }
        break;
    }
    case 0x02: branch = 1; target = ((c->pc + 4u) & ~(uint64_t)0x0FFFFFFFu) | ((uint64_t)(ins & 0x03FFFFFFu) << 2); break;
    case 0x03: branch = 1; target = ((c->pc + 4u) & ~(uint64_t)0x0FFFFFFFu) | ((uint64_t)(ins & 0x03FFFFFFu) << 2);
               r[31] = c->pc + 8u; break;
    case 0x04: case 0x05: case 0x06: case 0x07:
    case 0x14: case 0x15: case 0x16: case 0x17: {
        int take;
        switch (op & 3u) {
        case 0: take = r[RS] == r[RT]; break;
        case 1: take = r[RS] != r[RT]; break;
        case 2: take = (int64_t)r[RS] <= 0; break;
        default: take = (int64_t)r[RS] > 0; break;
        }
        if (take) { branch = 1; target = c->pc + 4u + (IMM16 << 2); }
        else if (op & 0x10u) likely_skip = 1;
        break;
    }
    case 0x08: { int64_t s = (int64_t)(int32_t)(uint32_t)r[RS] + (int64_t)(int16_t)(uint16_t)ins;
                 if (s != (int64_t)(int32_t)s) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                 r[RT] = (uint64_t)s; break; }
    case 0x09: r[RT] = SX32((uint32_t)r[RS] + (uint32_t)IMM16); break;
    case 0x0A: r[RT] = (int64_t)r[RS] < (int64_t)IMM16; break;
    case 0x0B: r[RT] = r[RS] < IMM16; break;
    case 0x0C: r[RT] = r[RS] & UIMM16; break;
    case 0x0D: r[RT] = r[RS] | UIMM16; break;
    case 0x0E: r[RT] = r[RS] ^ UIMM16; break;
    case 0x0F: r[RT] = SX32((uint32_t)(ins << 16)); break;
    case 0x10: if (cop0_exec(n, ins)) return 1; break;
    case 0x11: cop1_exec(n, ins, &branch, &target, &likely_skip); if (c->exc) return 1; break;
    case 0x12: if (!(c->cp0[C0_STATUS] & 0x40000000u)) { take_exc(n, EXC_CPU, 0x180u, 2); return 1; } break;
    case 0x18: { uint64_t a = r[RS], b = IMM16, s = a + b;
                 if ((~(a ^ b) & (a ^ s)) >> 63) { take_exc(n, EXC_OV, 0x180u, 0); return 1; }
                 r[RT] = s; break; }
    case 0x19: r[RT] = r[RS] + IMM16; break;
    case 0x1A: case 0x1B: {                            /* LDL / LDR */
        uint64_t va = r[RS] + IMM16, w;
        uint32_t sh;
        if (!load_aligned(n, va, 8, &w)) return 1;
        if (op == 0x1A) { sh = (uint32_t)(va & 7u) * 8u; r[RT] = (w << sh) | (sh ? (r[RT] & ((1ull << sh) - 1u)) : 0); }
        else { sh = (7u - (uint32_t)(va & 7u)) * 8u; r[RT] = (w >> sh) | (sh ? (r[RT] & ~(~0ull >> sh)) : 0); }
        break;
    }
    case 0x20: if (!vload(n, r[RS] + IMM16, 1, &v)) return 1; r[RT] = (uint64_t)(int64_t)(int8_t)(uint8_t)v; break;
    case 0x21: if (!vload(n, r[RS] + IMM16, 2, &v)) return 1; r[RT] = (uint64_t)(int64_t)(int16_t)(uint16_t)v; break;
    case 0x22: case 0x26: {                            /* LWL / LWR */
        uint64_t va = r[RS] + IMM16, w;
        uint32_t sh, old = (uint32_t)r[RT], res;
        if (!load_aligned(n, va, 4, &w)) return 1;
        if (op == 0x22) { sh = (uint32_t)(va & 3u) * 8u; res = ((uint32_t)w << sh) | (old & (uint32_t)((1ull << sh) - 1u)); }
        else { sh = (3u - (uint32_t)(va & 3u)) * 8u; res = ((uint32_t)w >> sh) | (old & ~(uint32_t)(0xFFFFFFFFull >> sh)); }
        r[RT] = SX32(res);
        break;
    }
    case 0x23: if (!vload(n, r[RS] + IMM16, 4, &v)) return 1; r[RT] = SX32(v); break;
    case 0x24: if (!vload(n, r[RS] + IMM16, 1, &v)) return 1; r[RT] = v; break;
    case 0x25: if (!vload(n, r[RS] + IMM16, 2, &v)) return 1; r[RT] = v; break;
    case 0x27: if (!vload(n, r[RS] + IMM16, 4, &v)) return 1; r[RT] = v & 0xFFFFFFFFu; break;
    case 0x28: if (!vstore(n, r[RS] + IMM16, 1, r[RT])) return 1; break;
    case 0x29: if (!vstore(n, r[RS] + IMM16, 2, r[RT])) return 1; break;
    case 0x2A: case 0x2E: {                            /* SWL / SWR */
        uint64_t va = r[RS] + IMM16, w;
        uint32_t sh, val = (uint32_t)r[RT], res;
        if (!load_aligned(n, va, 4, &w)) return 1;
        if (op == 0x2A) { sh = (uint32_t)(va & 3u) * 8u; res = ((uint32_t)w & ~(uint32_t)(0xFFFFFFFFull >> sh)) | (val >> sh); }
        else { sh = (3u - (uint32_t)(va & 3u)) * 8u; res = ((uint32_t)w & (uint32_t)((1ull << sh) - 1u)) | (val << sh); }
        if (!store_aligned(n, va, 4, res)) return 1;
        break;
    }
    case 0x2B: if (!vstore(n, r[RS] + IMM16, 4, r[RT])) return 1; break;
    case 0x2C: case 0x2D: {                            /* SDL / SDR */
        uint64_t va = r[RS] + IMM16, w, val = r[RT], res;
        uint32_t sh;
        if (!load_aligned(n, va, 8, &w)) return 1;
        if (op == 0x2C) { sh = (uint32_t)(va & 7u) * 8u; res = (w & (sh ? ~(~0ull >> sh) : 0)) | (val >> sh); }
        else { sh = (7u - (uint32_t)(va & 7u)) * 8u; res = (w & (sh ? ((1ull << sh) - 1u) : 0)) | (val << sh); }
        if (!store_aligned(n, va, 8, res)) return 1;
        break;
    }
    case 0x2F: break;                                  /* CACHE */
    case 0x30: if (!vload(n, r[RS] + IMM16, 4, &v)) return 1; r[RT] = SX32(v); c->llbit = 1; break;
    case 0x31: if (!(c->cp0[C0_STATUS] & ST_CU1)) { take_exc(n, EXC_CPU, 0x180u, 1); return 1; }
               if (!vload(n, r[RS] + IMM16, 4, &v)) return 1; fsetw(c, RT, (uint32_t)v); break;
    case 0x34: if (!vload(n, r[RS] + IMM16, 8, &v)) return 1; r[RT] = v; c->llbit = 1; break;
    case 0x35: if (!(c->cp0[C0_STATUS] & ST_CU1)) { take_exc(n, EXC_CPU, 0x180u, 1); return 1; }
               if (!vload(n, r[RS] + IMM16, 8, &v)) return 1; fsetl(c, RT, v); break;
    case 0x37: if (!vload(n, r[RS] + IMM16, 8, &v)) return 1; r[RT] = v; break;
    case 0x38: if (c->llbit) { if (!vstore(n, r[RS] + IMM16, 4, r[RT])) return 1; }
               r[RT] = c->llbit; break;
    case 0x39: if (!(c->cp0[C0_STATUS] & ST_CU1)) { take_exc(n, EXC_CPU, 0x180u, 1); return 1; }
               if (!vstore(n, r[RS] + IMM16, 4, fgetw(c, RT))) return 1; break;
    case 0x3C: if (c->llbit) { if (!vstore(n, r[RS] + IMM16, 8, r[RT])) return 1; }
               r[RT] = c->llbit; break;
    case 0x3D: if (!(c->cp0[C0_STATUS] & ST_CU1)) { take_exc(n, EXC_CPU, 0x180u, 1); return 1; }
               if (!vstore(n, r[RS] + IMM16, 8, fgetl(c, RT))) return 1; break;
    case 0x3F: if (!vstore(n, r[RS] + IMM16, 8, r[RT])) return 1; break;
    default: c->halted_bad++; take_exc(n, EXC_RI, 0x180u, 0); return 1;
    }
    r[0] = 0;
    if (likely_skip) {
        c->pc = c->npc + 4u;
        c->npc = c->pc + 4u;
        return 1;
    }
    if (branch) {
        c->pc = c->npc;
        c->npc = target;
        c->next_delay = 1;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------- run */
void n64_cpu_run(hs_n64_t *n, uint32_t instrs)
{
    n64_cpu_t *c = &n->cpu;
    uint32_t k;
    for (k = 0; k < instrs; k++) {
        uint64_t st, w;
        uint32_t cnt = (uint32_t)c->cp0[C0_COUNT] + 1u;
        c->cp0[C0_COUNT] = cnt;
        if (cnt == (uint32_t)c->cp0[C0_COMPARE]) c->cp0[C0_CAUSE] |= 0x8000u;
        if (c->cp0[C0_RANDOM] <= c->cp0[C0_WIRED]) c->cp0[C0_RANDOM] = 31u;
        else c->cp0[C0_RANDOM]--;
        c->instret++;
        n->frame_instr++;
        c->exc = 0;
        c->in_delay = c->next_delay;
        st = c->cp0[C0_STATUS];
        if ((c->cp0[C0_CAUSE] & st & 0xFF00u) && (st & ST_IE) && !(st & (ST_EXL | ST_ERL))) {
            take_exc(n, EXC_INT, 0x180u, 0);
            continue;
        }
        c->next_delay = 0;
        if (!vload(n, c->pc, 4, &w)) continue;
        if (exec_one(n, (uint32_t)w)) continue;
        c->pc = c->npc;
        c->npc = c->pc + 4u;
    }
}
