/* SPDX-License-Identifier: MIT
 * gba_internal.h - private state of the hs-gba clean-room core.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#ifndef HS_GBA_INTERNAL_H
#define HS_GBA_INTERNAL_H

#include <stdint.h>
#include <string.h>
#include "hs_gba.h"
#include "gba_cpu.h"

#define GBA_CYC_LINE 1232u
#define GBA_CYC_HDRAW 960u
#define GBA_LINES 228u
#define GBA_VIS_LINES 160u
#define GBA_ROM_MAX 0x2000000u
#define GBA_SAVE_MAX 0x20000u
#define GBA_LINE_IO 0x60u          /* DISPCNT..BLDY captured per scanline */
#define GBA_AUDIO_EV_MAX 12288u

/* IRQ bits */
enum {
  IRQ_VBLANK = 1u << 0, IRQ_HBLANK = 1u << 1, IRQ_VCOUNT = 1u << 2,
  IRQ_TIMER0 = 1u << 3, IRQ_DMA0 = 1u << 8, IRQ_KEYPAD = 1u << 12
};

/* Per-scanline PPU register capture (read-only in the video phase). */
typedef struct {
  uint8_t io[GBA_LINE_IO];
  int32_t bgx[2], bgy[2]; /* internal affine reference points BG2/BG3 */
} gba_line_regs;

/* Audio event: a register write or a Direct Sound FIFO output sample. */
typedef struct {
  uint32_t t;     /* cycle within frame */
  uint16_t reg;   /* IO offset 0x60..0x9F, or 0xF0 (FIFO A) / 0xF1 (FIFO B) */
  uint16_t val;
} gba_audio_ev;

/* Immutable frame snapshot produced by the CPU phase. */
typedef struct {
  uint8_t pal[0x400];
  uint8_t vram[0x18000];
  uint8_t oam[0x400];
  gba_line_regs line[GBA_VIS_LINES];
  uint64_t frame_no;
  uint32_t n_ev;
  gba_audio_ev ev[GBA_AUDIO_EV_MAX];
} gba_snapshot;

typedef struct {
  uint16_t reload, counter, ctrl;
  uint32_t presc;  /* cycle accumulator */
} gba_timer;

typedef struct {
  uint32_t src, dst, cnt;      /* internal (latched) registers */
  uint16_t ctrl;
} gba_dma;

/* Audio-phase-owned DMG PSG + mixer state. */
typedef struct {
  uint16_t reg[0x20];          /* mirror of 0x60..0x9F (16-bit units) */
  uint8_t wave[2][16];
  /* square 1/2 */
  struct {
    int on, duty_pos, len, vol, env_period, env_timer, env_up;
    uint32_t freq, timer;
    int sweep_period, sweep_timer, sweep_shift, sweep_neg; uint32_t sweep_shadow;
    int sweep_on;
  } sq[2];
  struct { int on, len, pos; uint32_t freq, timer; } wv;
  struct {
    int on, len, vol, env_period, env_timer, env_up;
    uint16_t lfsr; uint32_t timer;
  } nz;
  uint32_t seq_timer; int seq_step;
  int8_t fifo[2];
  uint64_t sample_index;       /* global 48 kHz sample counter */
  uint64_t frames;
} gba_apu;

typedef struct {
  uint16_t lut_src0[HS_GBA_OUT_W], lut_src1[HS_GBA_OUT_W];
  uint16_t lut_w[HS_GBA_OUT_W];
  uint16_t fb[HS_GBA_SCREEN_W * HS_GBA_SCREEN_H];
  uint64_t hash;
} gba_video;

struct hs_gba {
  gba_cpu cpu;
  /* memory */
  uint8_t bios[0x4000];
  int has_bios;
  uint8_t ewram[0x40000];
  uint8_t iwram[0x8000];
  uint8_t io[0x400];
  uint8_t pal[0x400];
  uint8_t vram[0x18000];
  uint8_t oam[0x400];
  uint8_t *rom;
  uint32_t rom_size;
  /* saves */
  hs_gba_save_type save_type;
  uint8_t save[GBA_SAVE_MAX];
  int save_dirty;
  struct { int state, bank, id_mode, erase_armed, write_armed, bank_armed; } flash;
  struct {
    int addr_bits, mode, nbits; uint64_t shift; uint32_t addr;
    uint64_t rdata; int rbits; int busy_reads;
  } eeprom;
  /* sys regs */
  uint16_t ie, ifl, ime, keys, waitcnt;
  gba_timer tm[4];
  gba_dma dma[4];
  int in_dma;
  uint32_t dma_stall;
  /* timing */
  uint32_t vcount, line_cycle;
  uint64_t frame_count;
  int32_t bgx[2], bgy[2];
  /* direct sound (CPU side) */
  int8_t fifo[2][32];
  uint8_t fifo_r[2], fifo_w[2], fifo_n[2];
  /* phase double buffers */
  gba_snapshot snap[2];
  int snap_w, snap_pub;
  uint32_t ev_dropped;
  /* consumer-owned state */
  gba_video video;
  gba_apu apu;
  /* scratch for HLE decompressors */
  uint8_t scratch[0x40000];
};

/* bus */
uint8_t gba_read8(hs_gba *g, uint32_t a);
uint16_t gba_read16(hs_gba *g, uint32_t a);
uint32_t gba_read32(hs_gba *g, uint32_t a);
void gba_write8(hs_gba *g, uint32_t a, uint8_t v);
void gba_write16(hs_gba *g, uint32_t a, uint16_t v);
void gba_write32(hs_gba *g, uint32_t a, uint32_t v);
void gba_bus_reset(hs_gba *g);
void gba_timers_advance(hs_gba *g, uint32_t cycles);
void gba_dma_trigger(hs_gba *g, int timing);
void gba_raise_irq(hs_gba *g, uint16_t bits);
void gba_detect_save(hs_gba *g);
void gba_audio_log(hs_gba *g, uint16_t reg, uint16_t val);

/* bios */
void gba_hle_install_stub(hs_gba *g);
uint32_t gba_hle_swi(hs_gba *g, uint32_t num);
int gba_hle_isqrt_selftest(void);

/* ppu / apu */
void gba_video_init(gba_video *v);
void gba_ppu_render(gba_video *v, const gba_snapshot *s);
void gba_upscale_640x480(const gba_video *v, uint32_t *out);
void gba_apu_reset(gba_apu *a);
int gba_apu_synth(gba_apu *a, const gba_snapshot *s, int16_t *out, int max_frames);

static inline uint16_t gba_ld16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t gba_ld32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline void gba_st16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static inline void gba_st32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

#endif
