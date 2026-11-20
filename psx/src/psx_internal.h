/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 * psx_internal.h - hs-psx private state. Every byte of mutable state lives
 * inside hs_psx_t (caller-provided memory); there are no mutable globals.
 */
#ifndef PSX_INTERNAL_H
#define PSX_INTERNAL_H

#include <stdint.h>
#include <stddef.h>
#include "hs_psx.h"
#include "hs_video.h"

#define PSX_VRAM_W 1024u
#define PSX_VRAM_H 512u
#define PSX_SPU_EV_MAX 8192u
#define PSX_SPU_BLK 1024u                       /* SPU RAM delta granule */
#define PSX_SPU_NBLK (HS_PSX_SPU_RAM / PSX_SPU_BLK)

/* ---------------- snapshot slot (BACK written by Phase 0, FRONT const) --- */
typedef struct psx_spu_ev {
    uint16_t sample;   /* 0..734 within the frame */
    uint16_t reg;      /* halfword index 0..255 of 0x1F801C00.. */
    uint16_t val;
    uint16_t _pad;
} psx_spu_ev_t;

typedef struct psx_slot {
    psx_gpu_prim_t prims[PSX_MAX_PRIMS];
    uint16_t vram[PSX_VRAM_W * PSX_VRAM_H];
    uint8_t  spu_ram[HS_PSX_SPU_RAM];
    psx_spu_ev_t spu_ev[PSX_SPU_EV_MAX];
    uint32_t nprims, dropped;
    uint32_t n_ev, ev_dropped;
    uint16_t present_x, present_y, disp_w, disp_h;
    uint8_t  disp_24bpp, disp_enabled, _p0[2];
    uint32_t frame;
} psx_slot_t;

/* ---------------- CPU / GTE ------------------------------------------------ */
typedef struct psx_cpu {
    uint32_t r[32];
    uint32_t pc, next_pc, hi, lo;
    uint32_t cop0[32];
    uint32_t ld_reg, ld_val;          /* pending load-delay write (reg 0 = none) */
    uint32_t cur_pc;                  /* pc of the executing instruction */
    uint8_t  in_delay, branch_taken, _p[2];
    uint64_t cycles;                  /* total since reset */
    int32_t  gd[32];                  /* GTE data regs */
    int32_t  gc[32];                  /* GTE control regs */
} psx_cpu_t;

enum { C0_BADV = 8, C0_SR = 12, C0_CAUSE = 13, C0_EPC = 14, C0_PRID = 15 };
enum { EXC_INT = 0, EXC_ADEL = 4, EXC_ADES = 5, EXC_IBE = 6, EXC_DBE = 7, EXC_SYS = 8,
       EXC_BP = 9, EXC_RI = 10, EXC_CPU = 11, EXC_OV = 12 };

/* ---------------- peripherals --------------------------------------------- */
typedef struct psx_dma_ch { uint32_t madr, bcr, chcr; } psx_dma_ch_t;
typedef struct psx_timer {
    uint32_t count, mode, target;
    uint32_t frac;                    /* sub-tick accumulator */
} psx_timer_t;

typedef struct psx_gpu0 {             /* Phase 0 GPU front-end state */
    uint32_t fifo[16];
    uint32_t nfifo, need;             /* words collected / needed */
    uint32_t mode;                    /* 0 cmd, 1 cpu->vram, 2 polyline */
    uint32_t xfer_x, xfer_y, xfer_w, xfer_h, xfer_i, xfer_n;
    uint32_t rd_x, rd_y, rd_w, rd_h, rd_i, rd_n, reading;
    uint32_t gpuread;
    uint32_t texpage;                 /* E1 */
    uint32_t texwin;                  /* E2 */
    int32_t  da_x0, da_y0, da_x1, da_y1;
    int32_t  off_x, off_y;
    uint32_t mask_bits;               /* E6 */
    uint32_t disp_x, disp_y, h_range, v_range, disp_mode, disp_off, dma_dir;
    uint32_t pl_cmd, pl_n;            /* polyline state */
    uint32_t pl_last_xy, pl_last_rgb, pl_rgb_next;
    uint8_t  present_set, _p[3];
    uint16_t present_x, present_y;
} psx_gpu0_t;

typedef struct psx_sio {
    uint8_t  rx[8];
    uint32_t rx_n, rx_r;
    uint16_t mode, ctrl, baud, stat;
    uint32_t seq;                     /* byte index within pad transaction */
    uint32_t dev;                     /* 0 none, 1 pad */
    uint32_t ack_pending;
} psx_sio_t;

typedef struct psx_cdrom {
    uint8_t  index, ie, ifl, _p;
    uint8_t  resp[16];
    uint32_t resp_n, resp_r;
    uint8_t  param[16];
    uint32_t param_n;
    uint32_t pending_irq_delay;
    uint8_t  pending_int, pending_resp[16];
    uint32_t pending_n;
} psx_cdrom_t;

typedef struct psx_hle {
    uint32_t heap_base, heap_end, heap_ptr;
    uint32_t pad_buf1, pad_buf2, pad_started;
    uint32_t lcg;                     /* BIOS rand(): fixed-seed LCG, deterministic */
    uint32_t unimpl_calls;
    uint32_t last_unimpl;
    uint32_t event_next;
} psx_hle_t;

/* ---------------- Phase 1 (video-private) --------------------------------- */
#define PSX_NATIVE_MAX (640u * 480u)
typedef struct psx_video_priv {
    uint32_t native[PSX_NATIVE_MAX];          /* native-res target */
    uint32_t tmp640[HS_FB_W * HS_FB_H];       /* pre-FXAA frame */
    uint8_t  luma[HS_FB_W * HS_FB_H];
    uint32_t strip[HS_SSAA_STRIP_BUDGET / 4u];
    uint32_t row_tmp[2u * HS_FB_W];
    uint16_t colmap[HS_FB_W * 8u];            /* target column -> VRAM x */
    hs_scaler_t scaler;
    int      post_fxaa;
} psx_video_priv_t;

/* ---------------- Phase 2 (audio-private) --------------------------------- */
typedef struct psx_voice {
    uint32_t addr;                    /* current ADPCM block byte address */
    uint32_t repeat;                  /* loop address (bytes) */
    uint32_t counter;                 /* 4.12 pitch phase in current block */
    int16_t  buf[28];                 /* decoded block */
    int16_t  hist[4];                 /* last 4 samples for interpolation */
    int16_t  old1, old2;              /* ADPCM filter history */
    int32_t  env;                     /* 0..0x7FFF */
    uint32_t env_phase;               /* 0 off, 1 attack, 2 decay, 3 sustain, 4 release */
    uint32_t env_cnt;
    int32_t  last_out;                /* for pitch modulation */
    uint8_t  on, flags, _p[2];
} psx_voice_t;

typedef struct psx_audio_priv {
    psx_voice_t v[24];
    uint16_t reg[256];                /* SPU register mirror (Phase 2 view) */
    uint32_t noise_lvl, noise_cnt;
    uint32_t endx;
    int16_t  native[HS_PSX_SPU_PER_FRAME * 2u];
    int16_t  prev_l, prev_r;          /* last native sample of previous frame */
    uint32_t frames;
} psx_audio_priv_t;

/* ---------------- the core ------------------------------------------------- */
struct hs_psx {
    uint32_t magic;
    /* ---- Phase 0 (CPU domain) ---- */
    psx_cpu_t   cpu;
    uint8_t     ram[HS_PSX_RAM_SIZE];
    uint8_t     scratch[1024];
    uint16_t    vram0[PSX_VRAM_W * PSX_VRAM_H];   /* 2D-transfer VRAM */
    uint8_t     spu_ram0[HS_PSX_SPU_RAM];
    uint8_t     vram_stale[2][PSX_VRAM_H];        /* rows to copy into slot s */
    uint8_t     spu_stale[2][PSX_SPU_NBLK];
    uint16_t    spu_reg[256];                     /* CPU-visible SPU regs */
    uint32_t    spu_xfer_addr;
    uint32_t    istat, imask;
    psx_dma_ch_t dma[7];
    uint32_t    dpcr, dicr;
    psx_timer_t tmr[3];
    psx_gpu0_t  gpu;
    psx_sio_t   sio;
    psx_cdrom_t cd;
    psx_hle_t   hle;
    uint32_t    memctl[16], ram_size, cache_ctl;
    const uint8_t *bios;              /* borrowed external BIOS or NULL */
    const uint8_t *exe;               /* borrowed PS-EXE */
    size_t      exe_len;
    uint32_t    exe_loaded;
    uint32_t    input;
    uint32_t    frame_cyc;            /* cycles into current frame */
    uint32_t    frame;
    uint32_t    tmr_accum;
    char        tty[HS_PSX_TTY_SIZE];
    uint32_t    tty_len;
    uint32_t    bad_access;
    /* ---- snapshots ---- */
    psx_slot_t  slot[2];
    uint32_t    front;                /* index of FRONT slot */
    /* ---- Phase 1 / Phase 2 private ---- */
    psx_video_priv_t vid;
    psx_audio_priv_t aud;
};

#define PSX_MAGIC 0x50535831u

static inline psx_slot_t *psx_back(hs_psx_t *s) { return &s->slot[s->front ^ 1u]; }
static inline const psx_slot_t *psx_front(const hs_psx_t *s) { return &s->slot[s->front]; }

/* psx_cpu.c */
void psx_cpu_reset(hs_psx_t *s, uint32_t pc);
void psx_cpu_run(hs_psx_t *s, uint32_t cycles);
void psx_cpu_exception(hs_psx_t *s, uint32_t code);
/* psx_gte.c */
uint32_t psx_gte_read_data(psx_cpu_t *c, uint32_t r);
void     psx_gte_write_data(psx_cpu_t *c, uint32_t r, uint32_t v);
uint32_t psx_gte_read_ctrl(const psx_cpu_t *c, uint32_t r);
void     psx_gte_write_ctrl(psx_cpu_t *c, uint32_t r, uint32_t v);
void     psx_gte_command(psx_cpu_t *c, uint32_t cmd);
/* psx_bus.c */
uint32_t psx_read32(hs_psx_t *s, uint32_t a);
uint32_t psx_read16(hs_psx_t *s, uint32_t a);
uint32_t psx_read8(hs_psx_t *s, uint32_t a);
void     psx_write32(hs_psx_t *s, uint32_t a, uint32_t v);
void     psx_write16(hs_psx_t *s, uint32_t a, uint32_t v);
void     psx_write8(hs_psx_t *s, uint32_t a, uint32_t v);
void     psx_bus_reset(hs_psx_t *s);
void     psx_bus_tick(hs_psx_t *s, uint32_t cycles);
void     psx_irq(hs_psx_t *s, uint32_t bit);
int      psx_irq_pending(const hs_psx_t *s);
/* psx_gpu_cap.c */
void     psx_gpu_reset(hs_psx_t *s);
void     psx_gp0(hs_psx_t *s, uint32_t w);
void     psx_gp1(hs_psx_t *s, uint32_t w);
uint32_t psx_gpuread(hs_psx_t *s);
uint32_t psx_gpustat(const hs_psx_t *s);
void     psx_gpu_frame_begin(hs_psx_t *s);
void     psx_gpu_frame_end(hs_psx_t *s);
/* psx_bios_hle.c */
int      psx_hle_call(hs_psx_t *s, uint32_t vec);  /* 1 if handled */
int      psx_hle_exception(hs_psx_t *s);           /* 1 if handled */
int      psx_exe_load(hs_psx_t *s);
void     psx_hle_reset(hs_psx_t *s);
void     psx_hle_vblank(hs_psx_t *s);
/* psx_gpu_raster.c */
void     psx_raster_frame(hs_psx_t *s, uint32_t *fb, size_t stride, hs_video_mode_t mode);
/* psx_spu.c */
void     psx_spu_write(hs_psx_t *s, uint32_t off, uint32_t v);  /* Phase 0 */
uint32_t psx_spu_read(hs_psx_t *s, uint32_t off);
void     psx_spu_dma_write(hs_psx_t *s, uint32_t word);
uint32_t psx_spu_dma_read(hs_psx_t *s);
void     psx_spu_frame_begin(hs_psx_t *s);
void     psx_spu_synth_frame(hs_psx_t *s);              /* Phase 2 */
void     psx_spu_audio_reset(psx_audio_priv_t *a);

#endif
