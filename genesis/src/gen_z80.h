/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_z80.h - clean-room Zilog Z80 interpreter (main, CB, DD, ED, FD, DDCB,
 * FDCB opcode pages, IM 0/1/2, NMI-free maskable interrupt line).
 */
#ifndef HS_GEN_Z80_H
#define HS_GEN_Z80_H

#include <stdint.h>

typedef struct z80_bus {
    uint8_t (*read)(void *ctx, uint16_t addr);
    void    (*write)(void *ctx, uint16_t addr, uint8_t v);
    uint8_t (*in)(void *ctx, uint16_t port);
    void    (*out)(void *ctx, uint16_t port, uint8_t v);
    void    *ctx;
} z80_bus_t;

typedef struct gen_z80 {
    uint8_t a, f, b, c, d, e, h, l;
    uint8_t a2, f2, b2, c2, d2, e2, h2, l2;
    uint16_t ix, iy, sp, pc;
    uint8_t i, r, iff1, iff2, im, halted, irq_line, ei_delay;
    uint64_t cyc;
    uint64_t insns;
    z80_bus_t bus;
} gen_z80_t;

void     z80_init(gen_z80_t *z, const z80_bus_t *bus);
void     z80_reset(gen_z80_t *z);
/* Run until z->cyc >= target_cycle. */
void     z80_run_until(gen_z80_t *z, uint64_t target_cycle);

#endif
