/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_internal.h - hs-genesis internal state, split into strictly disjoint
 * phase domains:
 *
 *   cpu  (Core 0) : 68000, work RAM, cartridge/SRAM, I/O, VDP port state,
 *                   VRAM/CRAM/VSRAM, Z80 RAM mirror. Writes slots[back].
 *   vid  (Core 1) : private line/frame scratch. Reads slots[front].v.
 *   aud  (Core 2) : Z80, Z80 RAM, YM2612, PSG, resampler. Reads slots[front].a.
 *   slots[2]      : immutable-after-commit snapshots, swapped by commit.
 *
 * No field is written by two phases between a pair of barriers.
 */
#ifndef HS_GEN_INTERNAL_H
#define HS_GEN_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include "hs_genesis.h"
#include "m68k_cpu.h"
#include "gen_z80.h"
#include "gen_sound.h"

#define GEN_MAGIC        0x47454E31u   /* 'GEN1' */
#define GEN_MAX_LINES    240
#define GEN_MAX_EVENTS   24576
#define GEN_MASTER_LINE  3420u          /* master clocks per scanline */
#define GEN_MASTER_NTSC  53693175u
#define GEN_MASTER_PAL   53203424u
#define GEN_LINES_NTSC   262u
#define GEN_LINES_PAL    313u
#define GEN_HBLANK_68K   92u            /* 68k clocks of the line spent in hblank */

/* ---- snapshots (BACK written by Core 0, FRONT read by Cores 1/2) ---- */
typedef struct gen_line_snap {
    uint8_t  reg[24];
    uint16_t cram[64];
    uint16_t vsram[40];
} gen_line_snap_t;

typedef struct gen_video_snap {
    uint8_t         vram[65536];
    gen_line_snap_t line[GEN_MAX_LINES];
    uint32_t        active_h;     /* 224 or 240 */
    uint32_t        frame;
} gen_video_snap_t;

enum {
    GEV_YM = 1,      /* a = port 0..3, v = data */
    GEV_PSG,         /* v = data */
    GEV_ZRAM,        /* a = data, v = address */
    GEV_BUSREQ,      /* a = 1 requested by 68k */
    GEV_RESET,       /* a = 1 reset asserted */
    GEV_ZIRQ         /* a = 1 assert, 0 release */
};

typedef struct gen_audio_ev {
    uint32_t ts;     /* 68k clocks since frame start */
    uint8_t  kind;
    uint8_t  a;
    uint16_t v;
} gen_audio_ev_t;

typedef struct gen_audio_snap {
    uint32_t       n_ev;
    uint32_t       dropped;
    uint32_t       frame_68k;     /* 68k clocks in this frame */
    uint32_t       frame_z80;     /* Z80 clocks in this frame */
    uint32_t       frame;
    uint32_t       pal;
    gen_audio_ev_t ev[GEN_MAX_EVENTS];
} gen_audio_snap_t;

typedef struct gen_slot {
    gen_video_snap_t v;
    gen_audio_snap_t a;
} gen_slot_t;

/* ---- Core 0 domain ---- */
typedef struct gen_vdp {
    uint8_t  vram[65536];
    uint16_t cram[64];
    uint16_t vsram[40];
    uint8_t  reg[24];
    uint16_t addr;
    uint8_t  code;
    uint8_t  pending;
    uint8_t  vint_pending, hint_pending;
    uint8_t  vblank, hblank;
    uint8_t  fill_pending;
    uint8_t  odd;
    int32_t  hint_counter;
    uint32_t line;
    uint64_t line_start_cyc;
    uint32_t dma_words;           /* statistics */
} gen_vdp_t;

typedef struct gen_cpu_domain {
    m68k_t          cpu;
    gen_vdp_t       vdp;
    uint8_t         ram[65536];
    uint8_t         sram[65536];
    uint8_t         zmirror[8192];
    const uint8_t  *rom;
    uint32_t        rom_size;
    uint32_t        sram_start, sram_end;
    uint8_t         has_sram, sram_on, sram_dirty, ssf2;
    uint8_t         bank[8];
    uint8_t         z80_busreq, z80_reset;
    uint8_t         io_data[3], io_ctrl[3];
    uint8_t         th_phase;
    uint8_t         pal, six_button, japan;
    uint64_t        th_last_cyc;
    uint32_t        pad;          /* active-high Genesis buttons */
    uint64_t        frame_start_cyc;
    uint32_t        master_rem;
    uint32_t        frame;
    gen_audio_snap_t *ev_out;     /* points into slots[back].a during step */
} gen_cpu_domain_t;

/* ---- Core 1 domain ---- */
typedef struct gen_vid_domain {
    uint32_t rows[GEN_MAX_LINES][640];   /* horizontally stretched lines */
    uint32_t line_px[320];
    uint8_t  pix_b[336], pix_a[336], pix_s[336];   /* pal<<4|idx, bit7 = prio */
    uint16_t xlut_x0[2][640];            /* [0] H32 256->640, [1] H40 320->640 */
    uint8_t  xlut_w[2][640];
    uint16_t ylut_y0[2][480];            /* [0] 224->480, [1] 240->480 */
    uint8_t  ylut_w[2][480];
    uint16_t nlut_x0[2][640];            /* nearest */
    uint16_t nlut_y0[2][480];
    uint32_t color[3][512];              /* shadow / normal / highlight XRGB */
    uint32_t frames;
} gen_vid_domain_t;

/* ---- Core 2 domain ---- */
typedef struct gen_aud_domain {
    gen_z80_t  z80;
    gen_ym_t   ym;
    gen_psg_t  psg;
    uint8_t    zram[8192];
    uint16_t   bank;
    uint8_t    busreq, reset;
    uint8_t    zirq;
    uint8_t    _pad[3];
    const uint8_t *rom;
    uint32_t   rom_size;
    uint64_t   z80_base;          /* z80->cyc at frame start */
    uint64_t   ym_acc, psg_acc, smp_acc;
    int32_t    ym_l, ym_r;        /* held last native YM sample */
    int32_t    peak_ym, peak_psg; /* diagnostics (audio-private) */
    uint32_t   frames;
    uint64_t   samples_out;
} gen_aud_domain_t;

typedef struct hs_gen {
    uint32_t         magic;
    uint32_t         back;        /* index of the slot Core 0 writes */
    uint32_t         config_flags;
    uint32_t         _pad;
    gen_cpu_domain_t cpu;
    gen_slot_t       slot[2];
    gen_vid_domain_t vid;
    gen_aud_domain_t aud;
} hs_gen_t;

/* gen_bus.c (Core 0) */
void     gen_bus_init(hs_gen_t *g);
void     gen_bus_reset(hs_gen_t *g);
uint8_t  gen_bus_read8(void *ctx, uint32_t a);
uint16_t gen_bus_read16(void *ctx, uint32_t a);
void     gen_bus_write8(void *ctx, uint32_t a, uint8_t v);
void     gen_bus_write16(void *ctx, uint32_t a, uint16_t v);
void     gen_log_event(gen_cpu_domain_t *d, uint8_t kind, uint8_t a, uint16_t v);

/* gen_vdp.c (Core 0) */
void     gen_vdp_reset(gen_cpu_domain_t *d);
uint16_t gen_vdp_read(gen_cpu_domain_t *d, uint32_t a);
void     gen_vdp_write(gen_cpu_domain_t *d, uint32_t a, uint16_t v);
void     gen_vdp_update_irq(gen_cpu_domain_t *d);
uint32_t gen_vdp_iack(void *ctx, int level);

/* gen_render.c (Core 1) */
void     gen_render_init(gen_vid_domain_t *v);
void     gen_render_frame(gen_vid_domain_t *v, const gen_video_snap_t *s,
                          uint32_t *fb, size_t stride, hs_video_mode_t mode);

/* gen_audio.c (Core 2) */
void     gen_audio_init(gen_aud_domain_t *a);
void     gen_audio_reset(gen_aud_domain_t *a);
uint32_t gen_audio_frame(gen_aud_domain_t *a, const gen_audio_snap_t *s,
                         int16_t *pcm, uint32_t max_frames);

#endif
