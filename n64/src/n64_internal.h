/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_internal.h - private state of the hs-n64 clean-room core.
 * Every byte of mutable state lives inside hs_n64_t (caller memory).
 * Ownership by phase:
 *   Phase 0 : cpu, bus regs, rdram, rsp, gfx, snap[back]
 *   Phase 1 : video (private scratch), reads snap[front] only
 *   Phase 2 : audio (private synth state), reads snap[front] only
 *   Barrier : front ^= 1
 */
#ifndef N64_INTERNAL_H
#define N64_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include "hs_core.h"
#include "hs_video.h"
#include "hs_n64.h"

#define N64_RDRAM_BASE_SIZE   (4u << 20)
#define N64_RDRAM_MAX         (8u << 20)
#define N64_RDP_MAX_WORDS     (96u * 1024u)
#define N64_TEXBLOB_MAX       (256u * 1024u)
#define N64_AI_MAX_FRAMES     4096u
#define N64_INSTR_PER_FRAME   781250u      /* 93.75 MHz / 60 Hz / ~2 PClock per instr */
#define N64_VI_CLOCK_NTSC     48681812u
#define N64_DL_MAX_CMDS       65536u       /* bound on display-list commands per task */
#define N64_AUDIO_MAX_CMDS    4096u
#define N64_LLE_MAX_STEPS     1000000u

/* private RDP-stream opcodes (RDP opcodes 0x00-0x07 are no-ops on hardware) */
#define N64_OP_HS_TRI   0x01u   /* screen-space triangle, 1 + 3*7 words */
#define N64_OP_HS_TIMG  0x02u   /* texture image resolved into the tex blob, 4 words */
#define N64_TRI_WORDS   22u
#define N64_TRI_Z       0x01u
#define N64_TRI_SHADE   0x02u
#define N64_TRI_TEX     0x04u

/* exception codes */
enum { EXC_INT = 0, EXC_MOD = 1, EXC_TLBL = 2, EXC_TLBS = 3, EXC_ADEL = 4, EXC_ADES = 5,
       EXC_SYS = 8, EXC_BP = 9, EXC_RI = 10, EXC_CPU = 11, EXC_OV = 12, EXC_TR = 13, EXC_FPE = 15 };

/* COP0 register numbers */
enum { C0_INDEX = 0, C0_RANDOM = 1, C0_ENTRYLO0 = 2, C0_ENTRYLO1 = 3, C0_CONTEXT = 4,
       C0_PAGEMASK = 5, C0_WIRED = 6, C0_BADVADDR = 8, C0_COUNT = 9, C0_ENTRYHI = 10,
       C0_COMPARE = 11, C0_STATUS = 12, C0_CAUSE = 13, C0_EPC = 14, C0_PRID = 15,
       C0_CONFIG = 16, C0_LLADDR = 17, C0_XCONTEXT = 20, C0_ERROREPC = 30 };

#define ST_IE   0x00000001u
#define ST_EXL  0x00000002u
#define ST_ERL  0x00000004u
#define ST_UX   0x00000020u
#define ST_SX   0x00000040u
#define ST_KX   0x00000080u
#define ST_BEV  0x00400000u
#define ST_FR   0x04000000u
#define ST_CU1  0x20000000u

/* MI interrupt bits */
#define MI_SP 0x01u
#define MI_SI 0x02u
#define MI_AI 0x04u
#define MI_VI 0x08u
#define MI_PI 0x10u
#define MI_DP 0x20u

typedef struct n64_tlb_entry {
    uint32_t pagemask;
    uint32_t pad;
    uint64_t entryhi;
    uint64_t lo0, lo1;
} n64_tlb_entry_t;

typedef struct n64_decoded {
    uint32_t raw;
    uint8_t op, rs, rt, rd, sa, fn, valid, pad;
    int32_t imm;
} n64_decoded_t;
#define N64_DECODE_ENTRIES 2048u

typedef struct n64_cpu {
    uint64_t gpr[32];
    uint64_t hi, lo;
    uint64_t pc, npc;
    uint64_t cp0[32];
    uint64_t fpr[32];
    uint32_t fcr31;
    uint32_t llbit;
    uint32_t in_delay;     /* the instruction being executed sits in a delay slot */
    uint32_t next_delay;   /* the next instruction will be a delay slot */
    uint32_t exc;          /* exception raised by the current instruction */
    uint32_t halted_bad;   /* count of reserved-instruction exceptions */
    n64_tlb_entry_t tlb[32];
    uint64_t instret;
} n64_cpu_t;

typedef struct n64_tile {
    uint32_t fmt, siz, line, tmem, pal;
    uint32_t cmt, maskt, shiftt, cms, masks, shifts;
    uint32_t sl, tl, sh, th;   /* 10.2 */
} n64_tile_t;

/* RDP raster state, replayed from the frozen command list by Phase 1 (and
 * by the backend walker). Lives in Phase-1-private memory. */
typedef struct n64_rstate {
    n64_tile_t tile[8];
    uint32_t tile_raw[8][2], tsize_raw[8][2];
    uint32_t comb_hi, comb_lo;
    uint32_t om_hi, om_lo;
    uint32_t prim, env, fog, blend, fill;
    uint32_t prim_lod;
    uint32_t prim_z;
    uint32_t timg_off, timg_fmt, timg_siz, timg_w, timg_row0, timg_len;
    uint32_t cimg_addr, cimg_w, cimg_siz, zimg_addr;
    int32_t  sc_x0, sc_y0, sc_x1, sc_y1;   /* scissor, 10.2 */
    uint8_t  tmem[4096];
} n64_rstate_t;

typedef struct n64_snap {
    uint32_t frame;
    uint32_t vi_origin, vi_width, vi_status, vi_xscale, vi_yscale;
    uint32_t native_w, native_h;
    uint32_t ntris, overflow, gfx_tasks, audio_tasks;
    uint32_t rdp_n;
    uint32_t tex_n;
    uint32_t ai_freq;
    uint32_t ai_n;           /* stereo frames */
    uint32_t rdp[N64_RDP_MAX_WORDS];
    uint8_t  tex[N64_TEXBLOB_MAX];
    int16_t  ai[N64_AI_MAX_FRAMES * 2];
} n64_snap_t;

typedef struct n64_vtx {
    int32_t cx, cy, cz, cw;  /* clip coords s15.16 */
    int32_t s, t;            /* texel coords s15.16 */
    uint8_t r, g, b, a;
    uint32_t clip;           /* outcode */
} n64_vtx_t;

typedef struct n64_gfx {
    int32_t  mv[10][16];     /* modelview stack, s15.16 */
    int32_t  proj[16];
    int32_t  mvp[16];
    uint32_t mv_top;
    uint32_t mvp_valid;
    n64_vtx_t vbuf[64];
    uint32_t geom;
    uint32_t seg[16];
    uint32_t num_lights;
    uint8_t  light_col[8][3];
    int8_t   light_dir[8][3];
    int16_t  vp_scale[4], vp_trans[4];
    uint32_t tex_on, tex_tile, tex_sscale, tex_tscale;
    int32_t  fog_mul, fog_off;
    uint32_t rdphalf1, rdphalf2;
    uint32_t f3d_mode;       /* 1 = Fast3D index encoding, 0 = F3DEX */
    /* RDP front-end state (Phase 0) */
    uint32_t timg_addr, timg_fmt, timg_siz, timg_w;
    uint32_t cimg_w;
    int32_t  sc_x1, sc_y1;
    uint32_t om_hi, om_lo;
} n64_gfx_t;

typedef struct n64_rsp {
    uint8_t  dmem[4096];
    uint8_t  imem[4096];
    uint32_t mem_addr, dram_addr, rd_len, wr_len;
    uint32_t status;
    uint32_t hwflag;         /* SP "semaphore" register (hardware flag) */
    uint32_t pc;
    uint32_t sgpr[32];       /* scalar-unit LLE registers */
    uint32_t lle_steps, lle_unimpl;
    uint32_t hle_gfx, hle_audio, lle_tasks;
    /* HLE audio state */
    uint32_t a_in, a_out, a_count;
    int16_t  a_vol_l, a_vol_r;
    int16_t  a_book[2];
} n64_rsp_t;

typedef struct n64_video {
    uint32_t strip[HS_SSAA_STRIP_BUDGET / 4u];
    uint16_t zstrip[HS_SSAA_STRIP_BUDGET / 4u];
    uint32_t fb1x[HS_FB_W * HS_FB_H];
    uint8_t  luma[HS_FB_W * HS_FB_H];
    n64_rstate_t rs;
    const n64_snap_t *src;
    uint64_t samples_shaded;
    uint32_t prims_drawn;
} n64_video_t;

/* caller-owned scratch for hs_n64_render_slice_ctx (one per rendering core) */
typedef struct n64_slice_scratch {
    uint32_t strip[HS_SSAA_STRIP_BUDGET / 4u];
    uint16_t zstrip[HS_SSAA_STRIP_BUDGET / 4u];
    n64_rstate_t rs;
    uint64_t samples;
    uint32_t prims;
    uint32_t pad;
} n64_slice_scratch_t;

/* cartridge save memory + controller pak (n64_save.c) */
enum { N64_SAVE_NONE = 0, N64_SAVE_EEP4K = 1, N64_SAVE_EEP16K = 2, N64_SAVE_SRAM = 3, N64_SAVE_FLASH = 4 };
#define N64_SAVE_MAX   131072u
#define N64_PAK_SIZE   32768u
enum { N64_CIC_6101 = 6101, N64_CIC_6102 = 6102, N64_CIC_6103 = 6103, N64_CIC_6105 = 6105, N64_CIC_6106 = 6106 };
typedef struct n64_save {
    uint32_t type, size;     /* N64_SAVE_* and bytes */
    uint32_t dirty;          /* bumped on every write; the frontend flushes when it changes */
    uint32_t writes;
    uint32_t fl_mode;        /* FlashRAM: 0 idle, 1 status, 2 ID, 3 read, 4 page-buffer write, 5 erase armed */
    uint32_t fl_erase_off, fl_erase_len;
    uint32_t pak_on, pak_dirty;
    uint32_t cic, cic_known; /* IPL3 identity, from the CRC32 of ROM 0x40..0xFFF */
    uint32_t pad;
    uint64_t fl_status;
    uint8_t  fl_buf[128];
    uint8_t  data[N64_SAVE_MAX];
    uint8_t  pak[N64_PAK_SIZE];
} n64_save_t;

enum { N64_UC_HS = 0, N64_UC_FAST3D = 1, N64_UC_F3DEX = 2, N64_UC_F3DEX2 = 3, N64_UC_S2DEX = 4, N64_UC_UNKNOWN_GFX = 5 };
typedef struct n64_ucinfo {
    uint32_t addr, valid, kind, detects, hits;
    uint32_t unhandled;      /* F3DEX2 commands seen but not implemented */
    uint32_t unknown_tasks;  /* gfx tasks with an unrecognised microcode */
    uint32_t audio_skipped;  /* audio tasks with a Nintendo ABI microcode */
} n64_ucinfo_t;

typedef struct n64_audio {
    uint32_t phase;          /* 16.16 position within the current source frame */
    int16_t  last_l, last_r;
    uint32_t frames_out;
} n64_audio_t;

typedef struct hs_n64 {
    uint32_t magic;
    uint32_t rdram_size;
    uint32_t front;          /* index of the published snapshot slot */
    uint32_t frame;
    const uint8_t *rom;      /* borrowed, big-endian (.z64) after load */
    size_t   rom_len;
    uint32_t rom_crc1, rom_crc2, rom_crc_ok, rom_fmt;  /* 0 z64, 1 v64, 2 n64 */
    uint32_t input;
    uint32_t cfg_flags;
    n64_cpu_t cpu;
    n64_decoded_t decoded[N64_DECODE_ENTRIES];
    /* bus registers */
    uint32_t mi_mode, mi_intr, mi_mask;
    uint32_t vi_reg[14];
    uint32_t vi_line_intr_done;
    uint32_t ai_dram, ai_len, ai_control, ai_dacrate, ai_bitrate, ai_status;
    uint32_t pi_dram, pi_cart, pi_status, pi_dom[8];
    uint32_t si_dram, si_status;
    uint32_t ri_reg[8];
    uint32_t dp_start, dp_end, dp_current, dp_status;
    uint32_t pi_dma_count, si_dma_count, ai_dma_count, sp_dma_count;
    uint32_t frame_instr;    /* instructions executed in the current frame */
    uint8_t  pif_ram[64];
    n64_rsp_t rsp;
    n64_gfx_t gfx;
    n64_snap_t snap[2];
    n64_video_t video;
    n64_audio_t audio;
    n64_ucinfo_t ucinfo;
    uint8_t  rdram[N64_RDRAM_MAX];
    n64_save_t save;         /* cartridge save + controller pak: survives reset */
} hs_n64_t;

#define N64_MAGIC 0x4E363421u

static inline uint32_t n64_be32(const uint8_t *p)
{ return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline void n64_wbe32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static inline uint16_t n64_be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline void n64_wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

static inline n64_snap_t *n64_back(hs_n64_t *n) { return &n->snap[n->front ^ 1u]; }
static inline const n64_snap_t *n64_front(const hs_n64_t *n) { return &n->snap[n->front]; }

/* n64_cpu.c */
void     n64_cpu_reset(hs_n64_t *n);
void     n64_cpu_run(hs_n64_t *n, uint32_t instrs);
void     n64_cpu_update_irq(hs_n64_t *n);
uint32_t n64_fpu_round_w(double v, uint32_t rm, int32_t *out);  /* returns 1 if invalid */
/* n64_bus.c */
uint32_t n64_phys_read32(hs_n64_t *n, uint32_t pa);
void     n64_phys_write32(hs_n64_t *n, uint32_t pa, uint32_t v, uint32_t mask);
int      n64_rom_load(hs_n64_t *n, const uint8_t *rom, size_t len);
uint32_t n64_rom_word(const hs_n64_t *n, uint32_t off);   /* big-endian word through the byte-order view */
void     n64_rom_crc(const uint8_t *z64, size_t len, uint32_t *crc1, uint32_t *crc2);
int      n64_rom_detect_swap(const uint8_t *rom, size_t len); /* 0 z64, 1 v64, 2 n64, -1 */
void     n64_pif_boot(hs_n64_t *n);
void     n64_mi_raise(hs_n64_t *n, uint32_t bit);
void     n64_mi_clear(hs_n64_t *n, uint32_t bit);
void     n64_pif_process(hs_n64_t *n);
void     n64_vi_frame_start(hs_n64_t *n);
/* n64_rsp.c */
void     n64_rsp_start(hs_n64_t *n);
void     n64_rdp_ingest(hs_n64_t *n, const uint32_t *w, uint32_t nwords);
void     n64_dp_run(hs_n64_t *n);
/* n64_save.c */
void     n64_save_setup(hs_n64_t *n);               /* pick the save type for the loaded ROM */
uint32_t n64_rom_crc32(const hs_n64_t *n, uint32_t from, uint32_t to);   /* CRC-32 (IEEE) of ROM bytes */
uint32_t n64_cic_detect(const hs_n64_t *n, uint32_t *crc32_out);
void     n64_rom_crc_cic(const hs_n64_t *n, uint32_t cic, uint32_t *crc1, uint32_t *crc2);
int      n64_save_joybus(hs_n64_t *n, uint32_t ch, const uint8_t *cmd, uint32_t tx, uint8_t *res, uint32_t rx);
int      n64_pak_cmd(hs_n64_t *n, const uint8_t *cmd, uint32_t tx, uint8_t *res, uint32_t rx);
uint8_t  n64_pak_crc8(const uint8_t *d, uint32_t len);
uint32_t n64_cart2_read32(hs_n64_t *n, uint32_t pa);
void     n64_cart2_write32(hs_n64_t *n, uint32_t pa, uint32_t v);
void     n64_cart2_dma_read(hs_n64_t *n, uint32_t cart, uint8_t *dst, uint32_t len);   /* cart -> RDRAM */
void     n64_cart2_dma_write(hs_n64_t *n, uint32_t cart, const uint8_t *src, uint32_t len); /* RDRAM -> cart */
uint32_t n64_ucode_classify(const uint8_t *data, uint32_t len);
void     n64_rsp_matmul(const int32_t *a, const int32_t *b, int32_t *out);
void     n64_rsp_load_mtx(const uint8_t *src, int32_t *out);
void     n64_rsp_xform(const int32_t *m, int32_t x, int32_t y, int32_t z, int32_t *out4);
uint32_t n64_rsp_light(const n64_gfx_t *g, const int32_t *mv, int8_t nx, int8_t ny, int8_t nz);
uint32_t n64_isqrt64(uint64_t v);
int16_t  n64_adpcm_sample(int16_t p1, int16_t p2, int32_t resid, const int16_t *book);
/* n64_rdp.c */
void     n64_rstate_reset(n64_rstate_t *rs);
void     n64_render(hs_n64_t *n, uint32_t *fb, size_t stride, hs_video_mode_t mode);
/* hs_n64.c */
uint64_t n64_snap_video_hash(const n64_snap_t *s);
uint64_t n64_snap_audio_hash(const n64_snap_t *s);
uint64_t n64_cpu_state_hash(const hs_n64_t *n);
/* n64_audio.c */
uint32_t n64_audio_synth(hs_n64_t *n, int16_t *pcm, uint32_t max_frames);

#endif
