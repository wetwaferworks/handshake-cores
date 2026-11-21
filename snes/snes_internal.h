/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_internal.h — phase-0 machine layout shared by the hs-snes sources.
 * Clean-room implementation from public hardware documentation.
 */
#ifndef HS_SNES_INTERNAL_H
#define HS_SNES_INTERNAL_H

#include "hs_snes.h"
#include "snes_cpu.h"

/* NTSC master clock is the single timebase for CPU, PPU, and audio.
   Frame rate is SNES_MASTER_HZ / SNES_MASTER_PER_FRAME ≈ 60.0988 Hz.
   32 kHz samples per frame = 32000 * SNES_MASTER_PER_FRAME / SNES_MASTER_HZ
   = 532 + 9867296/21477272, emitted with a Bresenham 532/533 accumulator. */
#define SNES_MASTER_PER_LINE         1364u
#define SNES_LINES_PER_FRAME         262u
#define SNES_MASTER_PER_FRAME        (SNES_MASTER_PER_LINE * SNES_LINES_PER_FRAME)
#define SNES_MASTER_HZ               21477272ull
#define SNES_SPC_HZ                  1024000ull
#define SNES_DSP_RATE_HZ             32000u
#define SNES_SAMPLES_PER_FRAME_BASE  532u
#define SNES_SAMPLES_BRES_STEP       9867296u
#define SNES_SAMPLES_BRES_MOD        21477272u
/* Mix 8 voices + echo in 32-bit, then apply this right-shift before the
   final clamp. 1 = -6 dB of headroom so loud echo-heavy scores do not
   hard-clip mid-mix (a regular crackle source on A Link to the Past). */
#define SNES_MIX_HEADROOM_SHIFT      1

/* CPU-side PPU register file (mutated only by phase 0). */
typedef struct snes_ppu_io {
    uint16_t vram[0x8000];
    uint16_t cgram[256];
    uint8_t  oam[544];
    uint8_t  inidisp, obsel, bgmode, mosaic;
    uint8_t  bgsc[4], bgnba[2];
    uint16_t hofs[4], vofs[4];
    uint8_t  ofs_latch, hofs_latch;
    uint8_t  vmain;
    uint16_t vmaddr, vram_prefetch;
    uint8_t  m7sel, m7_latch, m7b_last;
    int16_t  m7a, m7b, m7c, m7d, m7x, m7y;
    uint16_t m7hofs, m7vofs;
    uint16_t cgaddr;            /* byte address 0..511 */
    uint8_t  cg_latch;
    uint16_t oamaddr_reload;    /* $2102/$2103 value (word addr + prio bit) */
    uint16_t oam_ptr;           /* internal byte pointer 0..1023 */
    uint8_t  oam_latch;
    uint8_t  w12sel, w34sel, wobjsel, wbglog, wobjlog;
    uint8_t  wh0, wh1, wh2, wh3;
    uint8_t  tm, ts, tmw, tsw, cgwsel, cgadsub;
    uint8_t  fixed_r, fixed_g, fixed_b;
    uint8_t  setini;
    uint16_t hlatch, vlatch;
    uint8_t  hflip_rd, vflip_rd, latched;
    uint8_t  ppu1_mdr, ppu2_mdr;
    uint8_t  cgram_dirty;
} snes_ppu_io;

typedef struct snes_dma_ch {
    uint8_t  dmap, bbad, a1b, dasb, nltr;
    uint16_t a1t, das, a2a;
    uint8_t  unused;
    uint8_t  hdma_active, hdma_do;
} snes_dma_ch;

typedef struct snes_timer {
    uint8_t  enabled, target, stage, counter;
    uint16_t divider;
} snes_timer;

/* Sony SPC700 (runs lock-step with the 65C816 inside phase 0). */
typedef struct snes_spc {
    uint8_t  a, x, y, sp, psw;
    uint16_t pc;
    uint8_t  ram[65536];
    uint8_t  in[4];             /* CPU -> APU */
    uint8_t  out[4];            /* APU -> CPU */
    uint8_t  dsp_addr;
    uint8_t  dsp_shadow[128];
    uint8_t  control, ipl_on, stopped;
    uint8_t  test;
    snes_timer t[3];
    uint64_t cycles;
} snes_spc;

struct hs_snes {
    snes_cpu cpu;

    const uint8_t *rom;
    uint32_t rom_size;
    uint8_t  mapper, fastrom;
    uint32_t rom_chunk[256];    /* 32 KiB chunk -> mirrored rom offset */
    uint8_t  title[22];
    uint8_t  chip_type, map_byte;
    uint8_t  sram[0x20000];
    uint32_t sram_size;

    uint8_t  wram[0x20000];
    uint32_t wram_addr;

    snes_ppu_io ppu;
    snes_spc    spc;

    snes_dma_ch dma[8];
    uint8_t  hdmaen;

    uint8_t  nmitimen, wrio, wrmpya, wrmpyb;
    uint16_t wrdiv, rddiv, rdmpy;
    uint16_t htime, vtime;
    uint8_t  memsel;
    uint8_t  nmi_flag, irq_flag, autojoy_busy;
    uint16_t joy_auto[4];
    uint16_t pad[2];
    uint32_t joy_shift[2];
    uint8_t  joy_strobe;
    uint8_t  mdr;
    uint8_t  in_vblank, irq_fired_line;

    /* timing */
    uint64_t master;            /* total master cycles since reset */
    uint64_t line_start;        /* master cycle at start of current line */
    uint64_t frame_start;
    uint16_t vpos;
    uint16_t vis_lines;
    uint32_t frame_no;

    /* phase-0 output under construction */
    hs_snes_slot *slot;
    uint32_t sample_base;
};

/* bus (snes_bus.c) */
void     snes_bus_reset(hs_snes *s);
uint8_t  snes_bus_peek(const hs_snes *s, uint32_t addr);
void     snes_hdma_init(hs_snes *s);
void     snes_hdma_line(hs_snes *s);
void     snes_autojoy(hs_snes *s);
int      snes_cart_load(hs_snes *s, const uint8_t *rom, size_t size);

/* ppu io (snes_ppu.c) */
void     snes_ppu_reset(snes_ppu_io *p);
uint8_t  snes_ppu_read(hs_snes *s, uint8_t reg);
void     snes_ppu_write(hs_snes *s, uint8_t reg, uint8_t val);
void     snes_ppu_snapshot_line(hs_snes *s, int line);
void     snes_ppu_snapshot_frame(hs_snes *s);

/* apu (snes_apu.c) */
void     snes_spc_reset(snes_spc *p);
void     snes_apu_catchup(hs_snes *s);
uint8_t  snes_apu_cpu_read(hs_snes *s, uint8_t port);
void     snes_apu_cpu_write(hs_snes *s, uint8_t port, uint8_t val);
int      snes_spc_step(hs_snes *s);   /* executes one SPC700 instruction, returns cycles */

#endif
