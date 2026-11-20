/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * m68k_cpu.h - clean-room Motorola 68000 interpreter (pure C11).
 * 16-bit opcode -> instruction-class dispatch table built at init into the
 * caller-owned CPU struct (no global mutable state, no allocation).
 */
#ifndef HS_M68K_CPU_H
#define HS_M68K_CPU_H

#include <stdint.h>

typedef struct m68k_bus {
    uint8_t  (*read8)(void *ctx, uint32_t addr);
    uint16_t (*read16)(void *ctx, uint32_t addr);
    void     (*write8)(void *ctx, uint32_t addr, uint8_t v);
    void     (*write16)(void *ctx, uint32_t addr, uint16_t v);
    /* Interrupt acknowledge for `level`; returns the vector number. */
    uint32_t (*iack)(void *ctx, int level);
    void     *ctx;
} m68k_bus_t;

typedef struct m68k {
    uint32_t d[8];
    uint32_t a[8];      /* a[7] = active stack pointer */
    uint32_t pc;
    uint32_t usp, ssp;  /* inactive stack pointer is parked here */
    uint32_t ir_pc;     /* address of the instruction being executed */
    uint16_t sr;
    uint8_t  stopped;
    uint8_t  irq_level; /* asserted interrupt level 0..7 (set by the bus) */
    uint64_t cyc;       /* monotonically increasing 68k clock count */
    uint64_t insns;
    m68k_bus_t bus;
    uint8_t  optab[65536]; /* opcode -> instruction class */
} m68k_t;

#define M68K_SR_C 0x0001u
#define M68K_SR_V 0x0002u
#define M68K_SR_Z 0x0004u
#define M68K_SR_N 0x0008u
#define M68K_SR_X 0x0010u
#define M68K_SR_S 0x2000u
#define M68K_SR_T 0x8000u

void     m68k_init(m68k_t *c, const m68k_bus_t *bus);
void     m68k_reset(m68k_t *c);
/* Execute until at least `cycles` more clocks elapsed; returns clocks used. */
uint32_t m68k_run(m68k_t *c, uint32_t cycles);
uint8_t  m68k_op_class(const m68k_t *c, uint16_t op);

#endif
