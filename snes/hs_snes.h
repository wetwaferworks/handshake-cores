/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * hs_snes.h — public 3-phase zero-mutex ABI of the clean-room HandshakeOS
 * Super Nintendo core.  Mirrors the CPU / Video-640x480 / Audio phase ABI:
 *
 *   Phase 0 (core 0)  hs_snes_step_cpu_phase()    65C816 + bus + DMA/HDMA +
 *                                                 SPC700 (lock-step, port
 *                                                 causality) -> writes ONE
 *                                                 immutable frame slot
 *   Phase 1 (core 1)  hs_snes_render_video_phase() reads ONLY a committed
 *                                                 slot, rasterises modes 0-7 +
 *                                                 OBJ + color math and writes
 *                                                 640x480 XRGB8888 directly
 *   Phase 2 (core 2)  hs_snes_synth_audio_phase()  reads ONLY a committed
 *                                                 slot, runs the S-DSP (8 voice
 *                                                 BRR, ADSR/GAIN, echo FIR) and
 *                                                 emits 48 kHz stereo int16
 *
 * Every structure that crosses a phase boundary (hs_snes_slot, hs_snes_video,
 * hs_snes_dsp) is pointer-free and position independent, so it can live in
 * shared memory and be handed between cores by a double-buffered publish
 * sequence (the Timing Handshake).  No locks, no threads, no wall clock, no
 * ambient RNG anywhere in the core.
 */
#ifndef HS_SNES_H
#define HS_SNES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HS_SNES_OUT_W            640
#define HS_SNES_OUT_H            480
#define HS_SNES_MAX_LINES        240     /* index 1..239 used            */
#define HS_SNES_CGRAM_POOL       241     /* CGRAM versions per frame      */
#define HS_SNES_APU_LOG_MAX      8192    /* >= worst case SPC writes/frame*/
#define HS_SNES_PORT_LOG_MAX     4096
#define HS_SNES_AUDIO_MAX        1024    /* 48 kHz stereo frames / video frame */

/* Joypad bits (SNES serial order, bit 15 first): B Y Sel St Up Dn Lt Rt A X L R */
enum {
    HS_SNES_BTN_B = 1u << 15, HS_SNES_BTN_Y = 1u << 14,
    HS_SNES_BTN_SELECT = 1u << 13, HS_SNES_BTN_START = 1u << 12,
    HS_SNES_BTN_UP = 1u << 11, HS_SNES_BTN_DOWN = 1u << 10,
    HS_SNES_BTN_LEFT = 1u << 9, HS_SNES_BTN_RIGHT = 1u << 8,
    HS_SNES_BTN_A = 1u << 7, HS_SNES_BTN_X = 1u << 6,
    HS_SNES_BTN_L = 1u << 5, HS_SNES_BTN_R = 1u << 4
};

enum { HS_SNES_MAP_LOROM = 0, HS_SNES_MAP_HIROM = 1, HS_SNES_MAP_EXHIROM = 2 };

/* ---- immutable per-scanline PPU register snapshot (render side) -------- */
typedef struct hs_snes_line {
    uint8_t  inidisp, obsel, bgmode, mosaic;
    uint8_t  bgsc[4], bgnba[2];
    uint16_t hofs[4], vofs[4];
    uint8_t  m7sel, setini;
    int16_t  m7a, m7b, m7c, m7d, m7x, m7y;
    uint16_t m7hofs, m7vofs;
    uint8_t  w12sel, w34sel, wobjsel, wbglog, wobjlog;
    uint8_t  wh0, wh1, wh2, wh3;
    uint8_t  tm, ts, tmw, tsw, cgwsel, cgadsub;
    uint8_t  fixed_r, fixed_g, fixed_b;
    uint8_t  oam_first;         /* OAM priority rotation start sprite   */
    uint16_t cgram_index;       /* index into slot->cgram pool          */
} hs_snes_line;

/* ---- S-DSP / ARAM write event logged by the SPC700 in phase 0 ---------- */
typedef struct hs_snes_apu_event {
    uint32_t sample;            /* absolute 32 kHz sample index          */
    uint16_t addr;              /* ARAM address or DSP register          */
    uint8_t  val;
    uint8_t  kind;              /* 0 = ARAM byte, 1 = DSP register       */
} hs_snes_apu_event;

typedef struct hs_snes_port_event {
    uint32_t master;            /* master cycle within the frame         */
    uint8_t  port;              /* 0-3 | 0x10 = CPU->APU write | 0x20 = APU->CPU */
    uint8_t  val;
    uint16_t pad;
} hs_snes_port_event;

/* ---- one committed frame slot: everything phases 1 and 2 may read ----- */
typedef struct hs_snes_slot {
    uint32_t frame_no;
    uint16_t lines;             /* 224 or 239 visible lines              */
    uint16_t cgram_count;
    hs_snes_line line[HS_SNES_MAX_LINES];
    uint16_t vram[0x8000];
    uint8_t  oam[544];
    uint16_t cgram[HS_SNES_CGRAM_POOL][256];
    /* audio */
    uint32_t sample_begin, sample_end;   /* [begin,end) 32 kHz samples   */
    uint32_t apu_log_count, apu_log_dropped;
    hs_snes_apu_event apu_log[HS_SNES_APU_LOG_MAX];
    /* deterministic CPU<->APU port-sync log (debug / ledger) */
    uint32_t port_log_count;
    uint64_t port_log_hash;
    hs_snes_port_event port_log[HS_SNES_PORT_LOG_MAX];
} hs_snes_slot;

/* ---- phase-1 worker private state (LUTs + line buffers) -------------- */
typedef struct hs_snes_hlut { uint16_t i0[HS_SNES_OUT_W]; uint16_t w[HS_SNES_OUT_W]; } hs_snes_hlut;
typedef struct hs_snes_vlut { uint16_t r0[HS_SNES_OUT_H]; uint16_t w[HS_SNES_OUT_H]; } hs_snes_vlut;

typedef struct hs_snes_video {
    hs_snes_hlut h256, h512;
    hs_snes_vlut v224, v239;
    uint8_t  bright[16][32];          /* 5-bit channel x brightness -> 8 bit */
    /* scratch (worker-private) */
    uint16_t lay_col[6][512];         /* BG1-4, OBJ resolved BGR555        */
    uint8_t  lay_z[6][512];           /* 0 = transparent, else rank        */
    uint8_t  obj_pal[256];            /* sprite palette (math eligibility) */
    uint8_t  win[6][256];             /* 1 = inside effective window       */
    uint16_t line15[512];
    uint32_t line32[512];
    uint32_t hrow[2][HS_SNES_OUT_W];  /* horizontally scaled native lines  */
    int      hrow_line[2];
} hs_snes_video;

/* ---- phase-2 worker private state (S-DSP) ---------------------------- */
typedef struct hs_snes_voice {
    int16_t  buf[32];                 /* decoded BRR ring (power of two)    */
    uint32_t pos;                     /* 20.12 fixed read position          */
    uint32_t decoded;                 /* samples decoded so far             */
    uint32_t stop_at;                 /* sample index where END w/o LOOP stops */
    uint16_t loop_addr;
    uint16_t brr_addr;                /* next BRR block to decode           */
    uint16_t pitch_counter;
    int16_t  env;
    int16_t  out;                     /* last output (for PMON / OUTX)      */
    uint8_t  env_mode, brr_hdr, brr_pos, buf_pos, keyed, active;
    uint16_t rate_counter;
} hs_snes_voice;

typedef struct hs_snes_dsp {
    uint8_t  aram[65536];
    uint8_t  reg[128];
    hs_snes_voice v[8];
    int16_t  fir_l[8], fir_r[8];
    uint8_t  fir_pos;
    uint16_t echo_pos;
    uint16_t noise;
    uint32_t global_counter;
    uint16_t rate_cnt[32];            /* per-rate countdown (no divides)    */
    uint32_t sample_now;
    uint32_t endx;
    /* 32k -> 48k resampler */
    int16_t  prev_l, prev_r;
    uint32_t rs_phase;                /* 0..2 thirds                        */
    uint32_t total_out;
    /* v5.7 audio: Bresenham remainder + mix diagnostics (phase 2 only) */
    uint32_t samp_err;                /* 32 kHz 532/533 remainder           */
    uint32_t last_n32;                /* 32 kHz samples last frame          */
    uint32_t last_n48;                /* 48 kHz frames last frame           */
    uint32_t clips;                   /* samples that hit the final clamp   */
    uint32_t peak;                    /* max |sample| before the clamp      */
#ifdef HS_SNES_AUDIO_DEBUG
#define HS_SNES_DBG_FRAMES 600
    uint16_t dbg_n32[HS_SNES_DBG_FRAMES];
    uint16_t dbg_n48[HS_SNES_DBG_FRAMES];
    uint32_t dbg_n;
#endif
} hs_snes_dsp;

typedef struct hs_snes hs_snes;       /* phase-0 machine (opaque size)     */

/* ---- lifecycle --------------------------------------------------------- */
size_t   hs_snes_sizeof(void);
/* rom stays owned by the caller and must outlive the machine. Returns 0 on success. */
int      hs_snes_init(hs_snes *s, const uint8_t *rom, size_t rom_size);
void     hs_snes_reset(hs_snes *s);
int      hs_snes_mapper(const hs_snes *s);
const char *hs_snes_title(const hs_snes *s, char out[22]);
uint8_t *hs_snes_sram(hs_snes *s, size_t *size);   /* battery SRAM load/save */
const char *hs_snes_chip_name(const hs_snes *s);  /* detected coprocessor */
void     hs_snes_set_input(hs_snes *s, uint16_t pad1, uint16_t pad2);

void     hs_snes_video_init(hs_snes_video *v);
void     hs_snes_dsp_init(hs_snes_dsp *d);

/* ---- the three phases -------------------------------------------------- */
/* Phase 0: run exactly one 60 Hz frame and fill *out (owned by phase 0 until published). */
void     hs_snes_step_cpu_phase(hs_snes *s, hs_snes_slot *out);
/* Phase 1: full 640x480 frame, or output rows [y0,y1) for split PPU workers. */
void     hs_snes_render_video_phase(const hs_snes_slot *slot, hs_snes_video *v, uint32_t *fb640x480);
void     hs_snes_render_video_rows(const hs_snes_slot *slot, hs_snes_video *v, uint32_t *fb640x480, int y0, int y1);
/* Phase 2: returns number of stereo frames written to pcm (interleaved L,R). */
int      hs_snes_synth_audio_phase(const hs_snes_slot *slot, hs_snes_dsp *d, int16_t *pcm, int max_frames);

uint64_t hs_snes_frame_hash(uint64_t h, const uint32_t *fb640x480, const int16_t *pcm, int frames);

/* ---- debug / test hooks (read-only views) ------------------------------ */
typedef struct hs_snes_cpu_view {
    uint16_t a, x, y, sp, dp, pc;
    uint8_t  pb, db, p, e, stopped, waiting;
    uint64_t master;
} hs_snes_cpu_view;
void     hs_snes_cpu_state(const hs_snes *s, hs_snes_cpu_view *v);
uint8_t  hs_snes_peek(const hs_snes *s, uint32_t addr24);        /* WRAM/ROM/SRAM only */
uint8_t  hs_snes_aram_peek(const hs_snes *s, uint16_t addr);
uint8_t  hs_snes_apu_port_out(const hs_snes *s, int port);
uint64_t hs_snes_spc_cycles(const hs_snes *s);

#ifdef __cplusplus
}
#endif
#endif
