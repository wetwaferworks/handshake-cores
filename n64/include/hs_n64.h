/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_n64.h - public entry point of the hs-n64 clean-room Nintendo 64 core.
 * Implements the shared 3-phase zero-mutex HandshakeOS ABI from hs_core.h:
 *   Core 0  step_cpu_phase     : VR4300 + COP0/COP1 + RDRAM + MI/PI/SI/VI/AI
 *                                + RSP task dispatch (HLE gfx/audio, scalar
 *                                LLE) -> bounded RDP command list + AI stream
 *                                written into the BACK snapshot slot
 *   Barrier commit_phase       : BACK -> FRONT publish (slot index flip)
 *   Core 1  render_video_phase : RDP rasterizer over the FRONT command list,
 *                                1x / FXAA / 2x / 4x / 8x strip-tiled SSAA,
 *                                640x480 edge-to-edge
 *   Core 2  synth_audio_phase  : AI FIFO playback -> 48 kHz s16 stereo
 */
#ifndef HS_N64_H
#define HS_N64_H

#include "hs_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* hs_core_config_t.flags bits for the N64 core */
#define HS_N64_CFG_EXPANSION_PAK 0x1u   /* 8 MiB RDRAM instead of 4 MiB */
#define HS_N64_CFG_BILINEAR4     0x2u   /* 4-tap bilinear instead of the RDP 3-point filter */
#define HS_N64_CFG_REF_INTERP    0x4u   /* diagnostic: reference interpreter (no fetch fast path / decode cache) */
#define HS_N64_CFG_REF_RDP       0x8u   /* diagnostic: reference per-pixel RDP path */
#define HS_N64_CFG_SAVE_SHIFT    4u     /* bits 4-6: 0 auto (ROM header ID), 1 none, 2 EEPROM 4K, 3 EEPROM 16K, 4 SRAM, 5 FlashRAM */
#define HS_N64_CFG_SAVE(t)       ((uint32_t)(t) << HS_N64_CFG_SAVE_SHIFT)
#define HS_N64_CFG_CONTROLLER_PAK 0x80u /* 32 KiB controller pak in port 1 (saved after the cartridge save) */

#define HS_N64_MAX_SLICES 4u
/* Independent strip-aligned slices. Each index needs its own scratch context
 * (hs_n64_slice_size bytes, 64-byte aligned); use hs_n64_render_slice_ctx.
 * The convenience form below is sequential only, reuses core video scratch. */
size_t hs_n64_slice_size(void);
int hs_n64_render_slice_ctx(const void *core, void *scratch, size_t scratch_size,
                            uint32_t *fb, size_t stride, hs_video_mode_t mode,
                            uint32_t index, uint32_t count);
void hs_n64_render_slice(void *core, uint32_t *fb, size_t stride, hs_video_mode_t mode,
                          uint32_t index, uint32_t count);

extern const hs_core_ops_t hs_n64_core_ops;

/* ---- Mali-G31 Panfrost / GLES backend hook ------------------------------
 * A hardware backend consumes the exact same frozen FRONT RDP command list
 * the software rasterizer uses. hs_n64_walk_backend() decodes the list,
 * tracks RDP state (combiner, other modes, tiles, TMEM loads) and hands the
 * backend fully resolved primitives. All pointers are valid only for the
 * duration of the callback. Read-only over the FRONT snapshot. */
typedef struct n64_bk_vertex {
    int32_t  x, y;        /* native screen position, s15.16 pixels */
    uint32_t z;           /* depth, 0..65535 */
    int32_t  w;           /* clip w, s15.16 (perspective) */
    int32_t  s, t;        /* texel coords, s15.16 */
    uint32_t rgba;        /* shade colour 0xRRGGBBAA */
} n64_bk_vertex_t;

typedef struct n64_bk_state {
    uint32_t combine_hi, combine_lo;   /* raw SetCombine words */
    uint32_t othermode_hi, othermode_lo;
    uint32_t prim_color, env_color, fog_color, blend_color, fill_color;
    uint32_t tile;                     /* tile index used by the primitive */
    const uint8_t *tmem;               /* 4096-byte TMEM image */
    uint32_t tile_desc[8][2];          /* raw SetTile words per tile */
    uint32_t tile_size[8][2];          /* raw SetTileSize words per tile */
    uint32_t flags;                    /* N64_TRI_* flags for triangles */
} n64_bk_state_t;

typedef struct n64_rdp_backend {
    void *user;
    void (*begin_frame)(void *user, uint32_t native_w, uint32_t native_h, uint32_t ss);
    void (*draw_tri)(void *user, const n64_bk_vertex_t v[3], const n64_bk_state_t *st);
    void (*fill_rect)(void *user, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                      int z_target, const n64_bk_state_t *st);
    void (*tex_rect)(void *user, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                     int32_t s, int32_t t, int32_t dsdx, int32_t dtdy, const n64_bk_state_t *st);
    void (*end_frame)(void *user);
} n64_rdp_backend_t;

/* Returns the number of primitives handed to the backend. */
uint32_t hs_n64_walk_backend(const void *core, const n64_rdp_backend_t *bk, uint32_t ss);

#ifdef __cplusplus
}
#endif
#endif
