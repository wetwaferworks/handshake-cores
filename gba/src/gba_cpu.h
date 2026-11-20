/* SPDX-License-Identifier: MIT
 * gba_cpu.h - clean-room ARM7TDMI (ARMv4T) interpreter for hs-gba.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#ifndef HS_GBA_CPU_H
#define HS_GBA_CPU_H

#include <stdint.h>

struct hs_gba;

#define CPSR_N (1u << 31)
#define CPSR_Z (1u << 30)
#define CPSR_C (1u << 29)
#define CPSR_V (1u << 28)
#define CPSR_I (1u << 7)
#define CPSR_F (1u << 6)
#define CPSR_T (1u << 5)

enum { MODE_USR = 0x10, MODE_FIQ = 0x11, MODE_IRQ = 0x12, MODE_SVC = 0x13,
       MODE_ABT = 0x17, MODE_UND = 0x1B, MODE_SYS = 0x1F };

typedef struct gba_cpu {
  uint32_t r[16];        /* r[15] = address of instruction being executed + 8/4 during exec */
  uint32_t pc_next;      /* address of the next instruction to fetch */
  uint32_t cpsr, spsr;
  uint32_t bank_sp[6], bank_lr[6], bank_spsr[6];
  uint32_t bank_fiq[5], bank_usr[5];
  int halted;
  int branched;
  int intrwait_active;
  uint32_t undefined_count;
  uint64_t insn_count;
  uint8_t arm_lut[4096];
  uint8_t thumb_lut[1024];
} gba_cpu;

void gba_cpu_init_tables(gba_cpu *c);
void gba_cpu_reset(struct hs_gba *g, int skip_bios);
void gba_cpu_set_cpsr(gba_cpu *c, uint32_t v);
void gba_cpu_jump(gba_cpu *c, uint32_t addr, int thumb);
/* Executes one instruction (or services a pending IRQ); returns cycles. */
uint32_t gba_cpu_step(struct hs_gba *g);
int gba_cpu_irq_check(struct hs_gba *g);
uint32_t gba_cpu_get_user_reg(const gba_cpu *c, int r);

#endif
