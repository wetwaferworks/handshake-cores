/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_bios_hle.c - original clean-room PS-EXE loader + minimal BIOS HLE.
 * No Sony code/data required. Hooks 0xA0/0xB0/0xC0 service vectors, plus
 * the kernel exception vector for syscall and IRQ return. Implements the
 * common memory/string/heap/TTY/pad/event bootstrap services used by simple
 * PS-EXE homebrew. This is not a complete replacement BIOS for commercial
 * software; unsupported services return 0 and are counted in hle.unimpl_calls.
 */
#include "psx_internal.h"

#include <string.h>

uint32_t psx_pad_word(const hs_psx_t *s);

static inline uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static void tty_char(hs_psx_t *s, uint32_t ch)
{
    if (s->tty_len < HS_PSX_TTY_SIZE - 1u) { s->tty[s->tty_len++] = (char)ch; s->tty[s->tty_len] = 0; }
}

static void tty_str(hs_psx_t *s, uint32_t p)
{
    uint32_t i;
    for (i = 0; i < HS_PSX_TTY_SIZE; i++) { uint32_t ch = psx_read8(s, p + i); if (!ch) break; tty_char(s, ch); }
}

static uint32_t str_len(hs_psx_t *s, uint32_t p)
{
    uint32_t i;
    for (i = 0; i < HS_PSX_RAM_SIZE; i++) if (!psx_read8(s, p + i)) break;
    return i;
}

static void fmt_uint(hs_psx_t *s, uint32_t v, uint32_t base, uint32_t width, int upper)
{
    char b[32];
    uint32_t n = 0, k;
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    do { b[n++] = digits[v % base]; v /= base; } while (v && n < 32u);
    for (k = n; k < width && k < 32u; k++) tty_char(s, '0');
    while (n) tty_char(s, (uint32_t)b[--n]);
}

static uint32_t fmt_arg(hs_psx_t *s, uint32_t i)
{
    if (i < 3u) return s->cpu.r[5u + i];
    return psx_read32(s, s->cpu.r[29] + 16u + (i - 3u) * 4u);
}

static void tty_printf(hs_psx_t *s, uint32_t fmt)
{
    uint32_t k, ai = 0;
    for (k = 0; k < 2048u; k++) {
        uint32_t ch = psx_read8(s, fmt + k), width = 0;
        if (!ch) break;
        if (ch != '%') { tty_char(s, ch); continue; }
        ch = psx_read8(s, fmt + ++k);
        if (ch == '%') { tty_char(s, '%'); continue; }
        if (ch == '0') ch = psx_read8(s, fmt + ++k);
        while (ch >= '0' && ch <= '9') { width = width * 10u + ch - '0'; ch = psx_read8(s, fmt + ++k); }
        if (ch == 'l') ch = psx_read8(s, fmt + ++k);
        { uint32_t a = fmt_arg(s, ai++);
          switch (ch) {
          case 's': tty_str(s, a); break;
          case 'c': tty_char(s, a); break;
          case 'd': case 'i': if ((int32_t)a < 0) { tty_char(s, '-'); a = 0u - a; } fmt_uint(s, a, 10, width, 0); break;
          case 'u': fmt_uint(s, a, 10, width, 0); break;
          case 'x': case 'p': fmt_uint(s, a, 16, width, 0); break;
          case 'X': fmt_uint(s, a, 16, width, 1); break;
          default: tty_char(s, '?'); break;
          }
        }
    }
}

void psx_hle_reset(hs_psx_t *s)
{
    memset(&s->hle, 0, sizeof(s->hle));
    s->hle.lcg = 1;
    s->hle.heap_base = s->hle.heap_ptr = 0x80100000u;
    s->hle.heap_end = 0x801F0000u;
    s->hle.event_next = 1;
    s->tty_len = 0;
    s->tty[0] = 0;
}

/* PS-X EXE header: PC @0x10, GP @0x14, load_addr @0x18, size @0x1C,
 * bss_addr @0x28, bss_size @0x2C, sp_base @0x30, sp_off @0x34. */
int psx_exe_load(hs_psx_t *s)
{
    const uint8_t *p = s->exe;
    uint32_t pc, gp, addr, n, sp, bss, bn, i;
    if (!p || s->exe_len < 0x800u || memcmp(p, "PS-X EXE", 8) != 0) return -1;
    pc = le32(p + 0x10); gp = le32(p + 0x14);
    addr = le32(p + 0x18); n = le32(p + 0x1C);
    bss = le32(p + 0x28); bn = le32(p + 0x2C);
    sp = le32(p + 0x30) + le32(p + 0x34);
    if (n > HS_PSX_RAM_SIZE || (size_t)n > s->exe_len - 0x800u || (addr & 0x1FFFFFFFu) >= 0x00800000u) return -1;
    if (bn > HS_PSX_RAM_SIZE) return -1;
    for (i = 0; i < n; i++) psx_write8(s, addr + i, p[0x800u + i]);
    if (bss) for (i = 0; i < bn; i++) psx_write8(s, bss + i, 0);
    /* minimal kernel bootstrap state */
    s->cpu.r[28] = gp;
    s->cpu.r[29] = sp ? sp : 0x801FFF00u;
    s->cpu.r[30] = s->cpu.r[29];
    s->cpu.r[31] = 0x8000F000u;          /* return-to-self if main returns */
    psx_write32(s, 0x8000F000u, 0x08003C00u); psx_write32(s, 0x8000F004u, 0);
    s->cpu.pc = pc;
    s->cpu.next_pc = pc + 4u;
    s->cpu.branch_taken = 0;
    s->cpu.ld_reg = 0;
    s->cpu.cop0[C0_SR] = 0x40000000u;    /* CU2=1, BEV=0, IRQ initially disabled */
    s->exe_loaded = 1;
    return 0;
}

static uint32_t hle_memcpy(hs_psx_t *s, uint32_t dst, uint32_t src, uint32_t n, int backwards)
{
    uint32_t i;
    if (n > HS_PSX_RAM_SIZE) n = HS_PSX_RAM_SIZE;
    if (backwards && dst > src && dst - src < n) { for (i = n; i > 0; i--) psx_write8(s, dst + i - 1u, psx_read8(s, src + i - 1u)); }
    else { for (i = 0; i < n; i++) psx_write8(s, dst + i, psx_read8(s, src + i)); }
    return dst;
}

static uint32_t heap_alloc(hs_psx_t *s, uint32_t n)
{
    uint32_t p = (s->hle.heap_ptr + 7u) & ~7u;
    if (n > HS_PSX_RAM_SIZE || p > s->hle.heap_end || n > s->hle.heap_end - p) return 0;
    s->hle.heap_ptr = p + n;
    return p;
}

/* A(xxh) services (numbers per the public BIOS function tables) */
static int call_a0(hs_psx_t *s, uint32_t fn)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t a = c->r[4], b = c->r[5], d = c->r[6], ret = 0, i;
    switch (fn) {
    case 0x00: case 0x01: case 0x02: case 0x04: case 0x05: ret = 0xFFFFFFFFu; break;   /* no filesystem */
    case 0x03:                                                                        /* write */
        if (a == 1u) { for (i = 0; i < d && i < HS_PSX_TTY_SIZE; i++) tty_char(s, psx_read8(s, b + i)); ret = d; }
        else { ret = 0xFFFFFFFFu; }
        break;
    case 0x09: tty_char(s, a); ret = a; break;                                        /* putc */
    case 0x0E: case 0x0F: ret = (uint32_t)((int32_t)a < 0 ? (0u - a) : a); break;     /* abs/labs */
    case 0x17: case 0x18: {                                                           /* strcmp/strncmp */
        uint32_t n = fn == 0x18 ? d : HS_PSX_RAM_SIZE;
        if (n > HS_PSX_RAM_SIZE) n = HS_PSX_RAM_SIZE;
        for (i = 0; i < n; i++) { uint32_t x = psx_read8(s, a + i), y = psx_read8(s, b + i); ret = x - y; if (x != y || !x) break; }
        } break;
    case 0x19: case 0x1A: {                                                           /* strcpy/strncpy */
        uint32_t n = fn == 0x1A ? d : HS_PSX_RAM_SIZE;
        if (n > HS_PSX_RAM_SIZE) n = HS_PSX_RAM_SIZE;
        for (i = 0; i < n; i++) { uint32_t x = psx_read8(s, b + i); psx_write8(s, a + i, x); if (!x) break; }
        ret = a;
        } break;
    case 0x1B: ret = str_len(s, a); break;                                            /* strlen */
    case 0x25: ret = (a >= 'a' && a <= 'z') ? a - 32u : a; break;                     /* toupper */
    case 0x26: ret = (a >= 'A' && a <= 'Z') ? a + 32u : a; break;                     /* tolower */
    case 0x27: hle_memcpy(s, b, a, d, 1); break;                                      /* bcopy(src,dst,n) */
    case 0x28: for (i = 0; i < b && i < HS_PSX_RAM_SIZE; i++) psx_write8(s, a + i, 0); break;   /* bzero */
    case 0x29: case 0x2D: {                                                           /* bcmp/memcmp */
        uint32_t n = d > HS_PSX_RAM_SIZE ? HS_PSX_RAM_SIZE : d;
        for (i = 0; i < n; i++) { uint32_t x = psx_read8(s, a + i), y = psx_read8(s, b + i); ret = x - y; if (x != y) break; }
        } break;
    case 0x2A: ret = hle_memcpy(s, a, b, d, 0); break;                                /* memcpy */
    case 0x2B: for (i = 0; i < d && i < HS_PSX_RAM_SIZE; i++) psx_write8(s, a + i, b & 0xFFu); ret = a; break; /* memset */
    case 0x2C: ret = hle_memcpy(s, a, b, d, 1); break;                                /* memmove */
    case 0x2E: for (i = 0; i < d && i < HS_PSX_RAM_SIZE; i++) if (psx_read8(s, a + i) == (b & 0xFFu)) { ret = a + i; break; } break;
    case 0x2F: s->hle.lcg = s->hle.lcg * 0x41C64E6Du + 0x3039u; ret = (s->hle.lcg >> 16) & 0x7FFFu; break; /* fixed-seed LCG */
    case 0x30: s->hle.lcg = a; break;                                                 /* seed set by the program */
    case 0x33: ret = heap_alloc(s, a); break;                                         /* malloc */
    case 0x34: break;                                                                 /* free (arena) */
    case 0x37: { uint64_t n = (uint64_t)a * b; if (n <= HS_PSX_RAM_SIZE) { ret = heap_alloc(s, (uint32_t)n);
                 if (ret) for (i = 0; i < (uint32_t)n; i++) psx_write8(s, ret + i, 0); } } break;  /* calloc */
    case 0x38: ret = heap_alloc(s, b); if (ret && a) hle_memcpy(s, ret, a, b, 0); break; /* realloc */
    case 0x39: s->hle.heap_base = s->hle.heap_ptr = a; s->hle.heap_end = a + b; break; /* InitHeap */
    case 0x3C: tty_char(s, a); ret = a; break;                                        /* putchar */
    case 0x3E: tty_str(s, a); tty_char(s, '\n'); break;                               /* puts */
    case 0x3F: tty_printf(s, a); break;                                               /* printf */
    case 0x44: break;                                                                 /* FlushCache */
    case 0x48: psx_gp1(s, a); break;                                                  /* SendGP1Command */
    case 0x49: psx_gp0(s, a); break;                                                  /* GPU_cw */
    case 0x4A: { uint32_t n = b > 0x10000u ? 0x10000u : b; for (i = 0; i < n; i++) psx_gp0(s, psx_read32(s, a + i * 4u)); } break;
    default: return 0;
    }
    c->r[2] = ret;
    return 1;
}

/* B(xxh) services */
static int call_b0(hs_psx_t *s, uint32_t fn)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t a = c->r[4], b = c->r[5], d = c->r[6], ret = 0;
    (void)b;
    switch (fn) {
    case 0x00: ret = heap_alloc(s, a); break;                                         /* alloc_kernel_memory */
    case 0x01: case 0x02: case 0x04: case 0x05: case 0x06: ret = 1; break;           /* timer services */
    case 0x03: ret = 0; break;
    case 0x07: ret = 1; break;                                                        /* DeliverEvent */
    case 0x08: ret = 0xF1000000u | (s->hle.event_next++ & 0xFFFFu); break;            /* OpenEvent */
    case 0x09: case 0x0A: case 0x0C: case 0x0D: ret = 1; break;                       /* Close/Wait/Enable/Disable */
    case 0x0B: ret = 0; break;                                                        /* TestEvent */
    case 0x12: s->hle.pad_buf1 = a; s->hle.pad_buf2 = d; ret = 2; break;              /* InitPad */
    case 0x13: s->hle.pad_started = 1; ret = 1; break;                                /* StartPad */
    case 0x14: s->hle.pad_started = 0; ret = 1; break;                                /* StopPad */
    case 0x15: s->hle.pad_buf1 = a; s->hle.pad_started = 2; ret = 2; break;           /* PAD_init */
    case 0x16: ret = psx_pad_word(s) | 0xFFFF0000u; break;                            /* PAD_dr */
    case 0x18: case 0x19: case 0x5B: ret = 1; break;
    case 0x3D: tty_char(s, a); ret = a; break;                                        /* putchar */
    case 0x3F: tty_str(s, a); tty_char(s, '\n'); break;                               /* puts */
    default: return 0;
    }
    c->r[2] = ret;
    return 1;
}

int psx_hle_call(hs_psx_t *s, uint32_t vec)
{
    uint32_t fn = s->cpu.r[9] & 0xFFu;
    int ok = 0;
    if (vec == 0xA0u) ok = call_a0(s, fn);
    else if (vec == 0xB0u) ok = call_b0(s, fn);
    else if (vec == 0xC0u) {
        if (fn == 0x00u || fn == 0x01u || fn == 0x02u || fn == 0x03u || fn == 0x07u || fn == 0x08u || fn == 0x0Au) { s->cpu.r[2] = 0; ok = 1; }
    }
    if (!ok) { s->hle.unimpl_calls++; s->hle.last_unimpl = (vec << 8) | fn; s->cpu.r[2] = 0; }
    return 1;
}

int psx_hle_exception(hs_psx_t *s)
{
    psx_cpu_t *c = &s->cpu;
    uint32_t code = (c->cop0[C0_CAUSE] >> 2) & 31u, epc = c->cop0[C0_EPC];
    if (code == EXC_SYS) {
        uint32_t n = c->r[4];
        if (n == 1u) { c->r[2] = (c->cop0[C0_SR] >> 2) & 1u; c->cop0[C0_SR] &= ~4u; }   /* EnterCriticalSection */
        else if (n == 2u) { c->r[2] = 1; c->cop0[C0_SR] |= 4u; }                         /* ExitCriticalSection */
        else c->r[2] = 0;
        epc += 4u;
    } else if (code == EXC_INT) {
        s->istat = 0;                      /* minimal HLE IRQ acknowledge */
    } else return 0;
    c->pc = epc;
    c->next_pc = epc + 4u;
    c->cop0[C0_SR] = (c->cop0[C0_SR] & ~0xFu) | ((c->cop0[C0_SR] >> 2) & 0xFu);
    return 1;
}

void psx_hle_vblank(hs_psx_t *s)
{
    uint32_t bits = psx_pad_word(s), a[2], k;
    if (!s->hle.pad_started) return;
    a[0] = s->hle.pad_buf1; a[1] = s->hle.pad_buf2;
    if (s->hle.pad_started == 2u) { if (a[0]) psx_write32(s, a[0], bits | 0xFFFF0000u); return; }
    for (k = 0; k < 2u; k++) if (a[k]) {
        psx_write8(s, a[k] + 0u, k ? 0xFFu : 0u);
        psx_write8(s, a[k] + 1u, 0x41u);
        psx_write16(s, a[k] + 2u, k ? 0xFFFFu : bits);
    }
}
