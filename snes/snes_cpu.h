/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_cpu.h — clean-room WDC 65C816 core (emulation + native mode).
 */
#ifndef HS_SNES_CPU_H
#define HS_SNES_CPU_H

#include <stdint.h>

enum {
    CPU_FC = 0x01, CPU_FZ = 0x02, CPU_FI = 0x04, CPU_FD = 0x08,
    CPU_FX = 0x10, CPU_FM = 0x20, CPU_FV = 0x40, CPU_FN = 0x80
};

typedef struct snes_cpu {
    uint16_t a, x, y, sp, dp, pc;
    uint8_t  pb, db, p, e;
    uint8_t  waiting, stopped;
    uint8_t  nmi_pending, irq_line;
    uint32_t instr_count;
} snes_cpu;

struct hs_snes;

/* bus hooks implemented by snes_bus.c — each charges master cycles */
uint8_t snes_cpu_read(struct hs_snes *s, uint32_t addr);
void    snes_cpu_write(struct hs_snes *s, uint32_t addr, uint8_t val);
void    snes_cpu_idle(struct hs_snes *s);

void    snes_cpu_reset(struct hs_snes *s);
/* executes one instruction (or services one interrupt / idle slot) */
void    snes_cpu_step(struct hs_snes *s);

#endif
