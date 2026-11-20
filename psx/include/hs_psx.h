/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_psx.h - hs-psx: clean-room Sony PlayStation (PS1/PSX) core for the
 * HandshakeOS zero-mutex 3-phase Timing Handshake (see hs_core.h).
 *
 *   Phase 0 (Core 0): R3000A CPU + GTE + bus + DMA + timers + IRQ + SIO0 pad
 *                     + CD-ROM status stub + HLE BIOS. GP0/GP1 traffic is
 *                     DECODED into a bounded primitive list in the BACK slot;
 *                     Phase 0 never rasterizes a polygon.
 *   Phase 1 (Core 1): software GPU rasterizer over the const FRONT slot:
 *                     native 1x, 1x+FXAA, strip-tiled 2x/4x/8x SSAA, always a
 *                     full 640x480 edge-to-edge frame.
 *   Phase 2 (Core 2): 24-voice SPU over the const FRONT SPU snapshot:
 *                     ADPCM, interpolation, ADSR, 44.1 kHz native mix,
 *                     resampled to the 48 kHz hs_core ABI rate.
 */
#ifndef HS_PSX_H
#define HS_PSX_H

#include <stddef.h>
#include <stdint.h>
#include "hs_core.h"
#include "psx_gpu_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PS1 video-mode names (aliases of the shared hs_video_mode_t values, so the
 * shared headers stay untouched):
 *   HS_VIDEO_PSX_1X   authentic native raster (e.g. 320x240, 15-bit + dither),
 *                     stretched edge-to-edge to 640x480 (sharp scaler)
 *   HS_VIDEO_FXAA     native raster + integer FXAA at 640x480
 *   HS_VIDEO_SSAA_2X  geometry re-rasterized at 1280x960 (2x2 samples per
 *                     output pixel), box-downsampled to 640x480
 *   HS_VIDEO_SSAA_4X  2560x1920 virtual (4x4 samples) -> 640x480
 *   HS_VIDEO_SSAA_8X  5120x3840 virtual (8x8 samples) -> 640x480
 * hs_psx_set_post_fxaa() adds the optional "+FXAA" pass on top of SSAA. */
#define HS_VIDEO_PSX_1X  HS_SSAA_1X
#define HS_VIDEO_FXAA    HS_AA_FXAA
#define HS_VIDEO_SSAA_2X HS_SSAA_2X
#define HS_VIDEO_SSAA_4X HS_SSAA_4X
#define HS_VIDEO_SSAA_8X HS_SSAA_8X

#define HS_PSX_CPU_HZ          33868800u
#define HS_PSX_FPS             60u
#define HS_PSX_CYCLES_PER_FRAME (HS_PSX_CPU_HZ / HS_PSX_FPS)   /* 564480 */
#define HS_PSX_SPU_RATE        44100u
#define HS_PSX_SPU_PER_FRAME   (HS_PSX_SPU_RATE / HS_PSX_FPS)  /* 735 */
#define HS_PSX_ABI_PER_FRAME   (HS_AUDIO_RATE / HS_PSX_FPS)    /* 800 */

#define HS_PSX_RAM_SIZE   (2u * 1024u * 1024u)
#define HS_PSX_BIOS_SIZE  (512u * 1024u)
#define HS_PSX_SPU_RAM    (512u * 1024u)
#define HS_PSX_TTY_SIZE   4096u

typedef struct hs_psx hs_psx_t;

extern const hs_core_ops_t hs_psx_core_ops;  /* HS_SYS_PS1 */

/* Optional 512 KiB external BIOS image (borrowed). Must be called before
 * load_rom; without it the clean-room HLE BIOS is used. */
int  hs_psx_load_bios(void *core, const uint8_t *bios, size_t len);
/* Optional FXAA after SSAA (video-private setting, never touches CPU state). */
void hs_psx_set_post_fxaa(void *core, int enable);
/* Native 44.1 kHz SPU output of the FRONT snapshot (735 frames/frame).
 * Use either this or ops->synth_audio_phase per frame, not both. */
uint32_t hs_psx_synth_audio_native(void *core, int16_t *pcm_stereo_i16, uint32_t max_frames);
/* Command-list export of the FRONT snapshot for an external GPU backend. */
void hs_psx_export_cmdlist(const void *core, psx_gpu_cmdlist_t *out);
/* Run a backend over the FRONT snapshot (Phase 1 alternative). */
int  hs_psx_render_backend(void *core, const psx_gpu_backend_t *be, hs_video_mode_t mode,
                           uint32_t *fb_xrgb8888, size_t fb_stride);

/* Debug / test accessors (read-only). */
uint64_t hs_psx_hash_cpu(const void *core);       /* CPU-domain state */
uint64_t hs_psx_hash_back(const void *core);      /* BACK slot (video+audio) */
const char *hs_psx_tty(const void *core, uint32_t *len);
uint32_t hs_psx_frame_count(const void *core);
uint32_t hs_psx_prim_count(const void *core);     /* FRONT prims */

#ifdef __cplusplus
}
#endif
#endif
