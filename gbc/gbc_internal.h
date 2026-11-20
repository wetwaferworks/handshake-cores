/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_internal.h - hs-gbc clean-room Game Boy (DMG) + Game Boy Color (CGB)
 * core: shared internal state. Written from public hardware documentation
 * only; no code from any other emulator.
 *
 * Memory ownership (zero-mutex contract, see hs_core.h):
 *   gbc_t.<cpu domain fields>  : Core 0 only (step_cpu_phase)
 *   gbc_t.vs[back], as[back]   : Core 0 only (written during step)
 *   gbc_t.vs[front], as[front] : read-only for Cores 1/2 until next commit
 *   gbc_t.vid                  : Core 1 only (render_video_phase)
 *   gbc_t.aud                  : Core 2 only (synth_audio_phase)
 *   gbc_t.front/back           : barrier (commit_phase) only
 */
#ifndef GBC_INTERNAL_H
#define GBC_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include "hs_core.h"
#include "hs_video.h"

#define GB_W 160
#define GB_H 144
#define GB_LINES 154
#define GB_DOTS_PER_LINE 456u
#define GB_DOTS_PER_FRAME 70224u          /* 4194304 Hz / 70224 = 59.7275 fps */
#define GB_CLOCK_HZ 4194304u

#define GBC_VLOG_MAX 65536u
#define GBC_AEVT_MAX 8192u
#define GBC_SERIAL_LOG 4096u

/* interrupt bits */
#define IRQ_VBLANK 0x01u
#define IRQ_STAT   0x02u
#define IRQ_TIMER  0x04u
#define IRQ_SERIAL 0x08u
#define IRQ_JOYPAD 0x10u

/* flags */
#define FZ 0x80u
#define FN 0x40u
#define FH 0x20u
#define FC 0x10u

enum { MBC_NONE = 0, MBC_1, MBC_2, MBC_3, MBC_5 };

/* ---------------- APU (shared by CPU-side tracker and synth) ------------ */
typedef struct gbc_square {
    uint8_t  on, dac, duty, duty_pos;
    uint8_t  vol, env_init, env_dir, env_period, env_timer;
    uint8_t  len_en;
    uint16_t len;          /* counts down from 64 */
    uint16_t freq;
    int32_t  timer;        /* APU cycles until next duty step */
    /* sweep (ch1 only) */
    uint8_t  sw_period, sw_neg, sw_shift, sw_timer, sw_en, sw_neg_used;
    uint16_t sw_shadow;
} gbc_square_t;

typedef struct gbc_wave {
    uint8_t  on, dac, vol_code, pos, sample, len_en;
    uint16_t len;          /* from 256 */
    uint16_t freq;
    int32_t  timer;
    uint8_t  ram[16];
} gbc_wave_t;

typedef struct gbc_noise {
    uint8_t  on, dac, vol, env_init, env_dir, env_period, env_timer, len_en;
    uint8_t  shift, width7, div_code, _pad;
    uint16_t len;
    uint16_t lfsr;
    int32_t  timer;
} gbc_noise_t;

typedef struct gbc_apu {
    uint8_t      regs[0x30];     /* raw NR10..NR52 + wave RAM shadow (0xFF10-0xFF3F) */
    uint8_t      power, nr50, nr51, fs_step;
    uint32_t     fs_timer;       /* APU cycles until next 512 Hz frame-sequencer tick */
    uint32_t     sample_acc;     /* fractional 48 kHz sample clock (units of 1/48000 cyc) */
    int32_t      hp_l, hp_r;     /* DC-blocking high-pass state (Q16) */
    int32_t      hp_prev_l, hp_prev_r;
    gbc_square_t ch1, ch2;
    gbc_wave_t   ch3;
    gbc_noise_t  ch4;
} gbc_apu_t;

void    gbc_apu_reset(gbc_apu_t *a, int cgb);
uint8_t gbc_apu_read(const gbc_apu_t *a, uint32_t reg);  /* reg = addr-0xFF10 */
void    gbc_apu_write(gbc_apu_t *a, uint32_t reg, uint8_t v);
/* Advance `cycles` APU clocks (4.194304 MHz). out may be NULL (state only).
 * Returns number of stereo frames produced (written if out && < cap). */
uint32_t gbc_apu_run(gbc_apu_t *a, uint32_t cycles, int16_t *out, uint32_t out_cap);

/* ---------------- snapshots --------------------------------------------- */
typedef struct gbc_vlog {
    uint16_t addr;   /* 0x0000-0x3FFF VRAM (bank1 at +0x2000), 0x4000+ OAM */
    uint8_t  val;
    uint8_t  line;   /* takes effect before this visible line is drawn */
} gbc_vlog_t;

typedef struct gbc_line {
    uint8_t latched, lcdc, scx, scy;
    uint8_t wx, win_draw, win_line, bgp;
    uint8_t obp0, obp1, opri, _pad[5];
    uint8_t bgpal[64];
    uint8_t objpal[64];
} gbc_line_t;

typedef struct gbc_vsnap {
    uint32_t   frame_no;
    uint8_t    cgb, lcd_on, dmg_palette, vlog_overflow;
    uint32_t   vlog_count;
    uint8_t    vram[0x4000];
    uint8_t    oam[0xA0];
    gbc_line_t line[GB_H];
    gbc_vlog_t vlog[GBC_VLOG_MAX];
} gbc_vsnap_t;

typedef struct gbc_aevt {
    uint32_t cyc;    /* APU cycles since the start of this step */
    uint8_t  reg;    /* addr - 0xFF10 */
    uint8_t  val;
    uint16_t _pad;
} gbc_aevt_t;

typedef struct gbc_asnap {
    uint32_t   frame_no;
    uint32_t   cycles;        /* APU cycles covered by this step */
    uint32_t   count;
    uint32_t   overflow;
    gbc_apu_t  start;         /* APU state at the start of this step */
    gbc_aevt_t ev[GBC_AEVT_MAX];
} gbc_asnap_t;

/* ---------------- CPU --------------------------------------------------- */
typedef struct gbc_cpu {
    uint8_t  a, f, b, c, d, e, h, l;
    uint16_t sp, pc;
    uint8_t  ime, ei_delay, halted, stopped, halt_bug, locked;
    uint8_t  _pad[2];
} gbc_cpu_t;

typedef struct gbc_rtc {
    uint8_t  sec, min, hour, dl, dh;    /* live counter (dh: bit0 day8, bit6 halt, bit7 carry) */
    uint8_t  lsec, lmin, lhour, ldl, ldh; /* latched copy */
    uint8_t  latch_prev, _pad;
    int64_t  last_epoch;
    uint8_t  epoch_valid, _pad2[7];
} gbc_rtc_t;

/* ---------------- whole machine ------------------------------------------ */
typedef struct gbc {
    /* ======== CPU domain (Core 0) ======== */
    gbc_cpu_t      cpu;
    hs_console_t   console_id;
    uint32_t       dmg_palette;
    uint8_t        cgb;            /* running in CGB mode */
    uint8_t        double_speed, key1, rom_loaded;

    const uint8_t *rom;
    uint32_t       rom_size, rom_banks;
    uint8_t        mbc, has_battery, has_rtc, ram_enable;
    uint32_t       ram_size;
    uint16_t       rom_bank;       /* ROMX bank */
    uint8_t        ram_bank, mbc1_hi, mbc1_mode, mbc1_lo;
    uint8_t        rom_bank_hi, _mp[3];
    uint32_t       rom0_off, romx_off, eram_off;
    gbc_rtc_t      rtc;

    uint8_t        vram[0x4000];
    uint8_t        wram[0x8000];
    uint8_t        eram[0x20000];
    uint8_t        oam[0xA0];
    uint8_t        hram[0x80];
    uint8_t        io[0x80];
    uint8_t        ie;
    uint8_t        vbk, svbk, bcps, ocps, opri;
    uint8_t        bgpal[64], objpal[64];

    /* timer */
    uint16_t       div;
    uint8_t        tima_reload;    /* 1 = TIMA overflowed last M-cycle */
    /* serial */
    uint32_t       serial_cycles;
    uint8_t        serial_active;
    uint32_t       serial_len;
    uint8_t        serial_log[GBC_SERIAL_LOG];
    /* joypad */
    uint32_t       input;
    uint8_t        p1_sel;
    /* OAM DMA */
    uint8_t        dma_active, dma_delay;
    uint16_t       dma_src, dma_pos;
    /* CGB HDMA */
    uint16_t       hdma_src, hdma_dst;
    uint8_t        hdma_active, hdma_remain, hdma_block_pending, _hp;

    /* PPU timing */
    uint32_t       dot;            /* 0..455 within line */
    uint8_t        ly, mode, stat_line, win_trig;
    uint8_t        win_counter, frame_done, logging, lcd_was_on;
    uint32_t       step_dots;      /* dots elapsed in this step */
    uint32_t       frame_no;

    /* APU tracker */
    gbc_apu_t      apu;
    uint32_t       apu_now;        /* APU cycles since step start */
    uint32_t       apu_done;       /* APU cycles already run */

    /* stats */
    uint64_t       total_cycles;
    uint32_t       instr_count;

    /* ======== snapshots ======== */
    uint32_t       front, back;    /* barrier only */
    gbc_vsnap_t    vs[2];
    gbc_asnap_t    as[2];

    /* ======== video domain (Core 1) ======== */
    struct {
        uint8_t     vram[0x4000];
        uint8_t     oam[0xA0];
        uint32_t    native[GB_W * GB_H];
        uint32_t    row_tmp[2 * HS_FB_W];
        hs_scaler_t sc_sharp, sc_near;
        uint8_t     luma[HS_FB_W * HS_FB_H];
        uint32_t    fx_tmp[HS_FB_W * HS_FB_H];
    } vid;

    /* ======== audio domain (Core 2) ======== */
    struct {
        gbc_apu_t apu;
        uint32_t  produced;
    } aud;
} gbc_t;

/* mmu */
uint8_t gbc_read(gbc_t *g, uint16_t addr);
void    gbc_write(gbc_t *g, uint16_t addr, uint8_t v);
void    gbc_tick(gbc_t *g);               /* one CPU M-cycle */
void    gbc_mmu_reset(gbc_t *g);
int     gbc_mmu_load_rom(gbc_t *g, const uint8_t *rom, size_t len);
void    gbc_rtc_advance(gbc_t *g, int64_t epoch);
void    gbc_hdma_service(gbc_t *g);       /* runs a pending HBlank block */
void    gbc_joypad_update(gbc_t *g, uint32_t input);
void    gbc_apu_catchup(gbc_t *g);

/* cpu */
void    gbc_cpu_reset(gbc_t *g);
void    gbc_cpu_step(gbc_t *g);           /* one instruction or interrupt */

/* ppu (CPU-domain timing side) */
void    gbc_ppu_reset(gbc_t *g);
void    gbc_ppu_tick(gbc_t *g, uint32_t dots);
void    gbc_ppu_lcdc_write(gbc_t *g, uint8_t v);
void    gbc_ppu_stat_update(gbc_t *g);
void    gbc_ppu_log_vram(gbc_t *g, uint16_t vaddr, uint8_t v);
void    gbc_ppu_log_oam(gbc_t *g, uint16_t idx, uint8_t v);
void    gbc_ppu_begin_step(gbc_t *g);
/* ppu (video-domain rendering side) */
void    gbc_ppu_render_native(const gbc_vsnap_t *s, uint8_t *vram, uint8_t *oam, uint32_t *out);

#endif
