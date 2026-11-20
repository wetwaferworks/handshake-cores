/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_core.h - HandshakeOS clean-room emulator core ABI (all consoles).
 *
 * Zero-mutex 3-phase contract, designed for the HandshakeOS 4-core
 * Timing Handshake:
 *
 *   Core 0 (CPU phase)   : ops->step_cpu_phase()  writes ONLY the CPU domain
 *                          and the BACK snapshot slot.
 *   Core 1 (video phase) : ops->render_video_phase() reads ONLY the FRONT
 *                          (committed) video snapshot and writes ONLY the
 *                          video-private scratch + caller framebuffer.
 *   Core 2 (audio phase) : ops->synth_audio_phase() reads ONLY the FRONT
 *                          audio snapshot and writes ONLY the audio-private
 *                          synth state + caller PCM buffer.
 *   Barrier (1 thread)   : ops->commit_phase() publishes BACK -> FRONT.
 *
 * Because the three phases touch disjoint memory between two barriers,
 * Core 0 can step frame N+1 while Cores 1/2 present frame N, with no
 * locks, atomics or shared mutable state. Cores never allocate, never read
 * a clock and never draw entropy: time enters only through rtc_epoch_sec.
 */
#ifndef HS_CORE_H
#define HS_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HS_CORE_ABI_VERSION 1u

#define HS_FB_W 640
#define HS_FB_H 480
#define HS_AUDIO_RATE 48000u

/* ---- consoles ---------------------------------------------------------- */
typedef enum hs_console {
    HS_SYS_GB = 0,
    HS_SYS_GBC,
    HS_SYS_GBA,
    HS_SYS_SNES,
    HS_SYS_GENESIS,
    HS_SYS_PS1,
    HS_SYS_N64,
    HS_SYS_COUNT
} hs_console_t;

/* ---- video output modes ------------------------------------------------ */
typedef enum hs_video_mode {
    HS_SCALE_STRETCH_SHARP = 0, /* default: 640x480 edge-to-edge, area-weighted sharp bilinear */
    HS_SCALE_STRETCH_NEAREST,   /* 640x480 edge-to-edge nearest neighbour */
    HS_AA_FXAA,                 /* sharp stretch (2D) / native raster (3D) + integer FXAA */
    HS_SSAA_1X,                 /* 3D cores: 1 sample/pixel  (2D cores: = SHARP) */
    HS_SSAA_2X,                 /* 3D cores: 2x2 samples     (2D cores: = SHARP) */
    HS_SSAA_4X,                 /* 3D cores: 4x4 samples     (2D cores: = SHARP) */
    HS_SSAA_8X,                 /* 3D cores: 8x8 samples     (2D cores: = SHARP) */
    HS_VIDEO_MODE_COUNT
} hs_video_mode_t;

/* ---- unified input bitmask ---------------------------------------------
 * One 32-bit word covers every pad we target. Per-console aliases below. */
enum {
    HS_BTN_UP      = 1u << 0,
    HS_BTN_DOWN    = 1u << 1,
    HS_BTN_LEFT    = 1u << 2,
    HS_BTN_RIGHT   = 1u << 3,
    HS_BTN_A       = 1u << 4,  /* east face   */
    HS_BTN_B       = 1u << 5,  /* south face  */
    HS_BTN_X       = 1u << 6,  /* north face  */
    HS_BTN_Y       = 1u << 7,  /* west face   */
    HS_BTN_L       = 1u << 8,  /* L / L1      */
    HS_BTN_R       = 1u << 9,  /* R / R1      */
    HS_BTN_L2      = 1u << 10,
    HS_BTN_R2      = 1u << 11,
    HS_BTN_L3      = 1u << 12,
    HS_BTN_R3      = 1u << 13,
    HS_BTN_START   = 1u << 14,
    HS_BTN_SELECT  = 1u << 15,
    HS_BTN_C       = 1u << 16, /* Genesis C   */
    HS_BTN_Z       = 1u << 17, /* Genesis Z / N64 Z trigger */
    HS_BTN_MODE    = 1u << 18, /* Genesis 6-button MODE */
    HS_BTN_C_UP    = 1u << 19, /* N64 C buttons */
    HS_BTN_C_DOWN  = 1u << 20,
    HS_BTN_C_LEFT  = 1u << 21,
    HS_BTN_C_RIGHT = 1u << 22,
    HS_BTN_MENU    = 1u << 31  /* reserved for HandshakeOS (never reaches a core) */
};
/* Per-console aliases (documentation + convenience). */
#define HS_GEN_A      HS_BTN_Y
#define HS_GEN_B      HS_BTN_B
#define HS_GEN_C      HS_BTN_C
#define HS_GEN_X      HS_BTN_L
#define HS_GEN_Y      HS_BTN_X
#define HS_GEN_Z      HS_BTN_Z
#define HS_PS1_CROSS    HS_BTN_B
#define HS_PS1_CIRCLE   HS_BTN_A
#define HS_PS1_SQUARE   HS_BTN_Y
#define HS_PS1_TRIANGLE HS_BTN_X
#define HS_N64_A      HS_BTN_B
#define HS_N64_B      HS_BTN_Y

/* ---- configuration ----------------------------------------------------- */
typedef struct hs_core_config {
    hs_console_t console_id;    /* requested console (GB core: GB or GBC) */
    uint32_t     dmg_palette;   /* GB core: 0 olive, 1 pocket, 2 purple, 3 matrix */
    uint32_t     flags;         /* reserved, 0 */
} hs_core_config_t;

/* ---- the 3-phase zero-mutex ops table ---------------------------------- */
typedef struct hs_core_ops {
    uint32_t     abi_version;
    hs_console_t console_id;
    const char  *name;
    uint32_t     native_w, native_h;  /* nominal native frame size */
    uint32_t     fps_num, fps_den;    /* exact frame rate */

    /* Bytes of caller-provided, 64-byte aligned memory the core lives in.
     * Cores never call malloc. */
    size_t (*state_size)(void);
    int    (*init)(void *mem, size_t mem_size, const hs_core_config_t *cfg);
    void   (*reset)(void *core);
    /* ROM buffer is borrowed (not copied) and must outlive the core. */
    int    (*load_rom)(void *core, const uint8_t *rom, size_t len);
    int    (*load_sram)(void *core, const uint8_t *buf, size_t len);
    size_t (*save_sram)(const void *core, uint8_t *buf, size_t cap);

    /* PHASE 0 (Core 0): advance exactly one emulated frame of CPU + bus,
     * writing an immutable video snapshot and audio event snapshot into
     * the BACK slot. rtc_epoch_sec is the only source of wall time. */
    void   (*step_cpu_phase)(void *core, uint32_t input_bits, int64_t rtc_epoch_sec);

    /* BARRIER (exactly one thread, all phases quiescent): BACK -> FRONT. */
    void   (*commit_phase)(void *core);

    /* PHASE 1 (Core 1): read-only over FRONT video snapshot. Writes a full
     * 640x480 XRGB8888 frame (every pixel, no borders). fb_stride in pixels. */
    void   (*render_video_phase)(void *core, uint32_t *fb_xrgb8888, size_t fb_stride,
                                 hs_video_mode_t mode);

    /* PHASE 2 (Core 2): read-only over FRONT audio snapshot. Writes up to
     * max_frames interleaved stereo int16 frames at 48 kHz, returns count. */
    uint32_t (*synth_audio_phase)(void *core, int16_t *pcm_stereo_i16, uint32_t max_frames);

    /* Deterministic FNV-1a 64 checksums of the FRONT snapshots. */
    void   (*hash_state)(const void *core, uint64_t *video_hash, uint64_t *audio_hash);
} hs_core_ops_t;

/* ---- FNV-1a 64 helper -------------------------------------------------- */
#define HS_FNV64_OFFSET 0xcbf29ce484222325ull
#define HS_FNV64_PRIME  0x00000100000001b3ull

static inline uint64_t hs_fnv1a64(uint64_t h, const void *data, size_t n)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;
    for (i = 0; i < n; i++) {
        h ^= p[i];
        h *= HS_FNV64_PRIME;
    }
    return h;
}

/* Core registry. */
extern const hs_core_ops_t hs_gbc_core_ops;   /* HS_SYS_GB + HS_SYS_GBC */

#ifdef __cplusplus
}
#endif
#endif /* HS_CORE_H */
