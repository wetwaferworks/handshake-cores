/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_video.h - shared HandshakeOS video library (pure C11, integer only):
 *   - 640x480 edge-to-edge stretched upsampler for any native size
 *     (sharp area-weighted bilinear or nearest) via precomputed LUTs,
 *   - integer FXAA post-pass,
 *   - strip-tiled 1x/2x/4x/8x supersampling downsampler.
 * No floats, no allocation, no global mutable state: deterministic and
 * bit-identical on x86_64 and ARM64.
 */
#ifndef HS_VIDEO_H
#define HS_VIDEO_H

#include <stddef.h>
#include <stdint.h>
#include "hs_core.h"

#define HS_SCALER_MAX_SRC 1024u

typedef struct hs_scaler {
    uint16_t x0[HS_FB_W];  /* left source column per output column */
    uint8_t  xw[HS_FB_W];  /* 8-bit weight of column x0+1 (0 = pure x0) */
    uint16_t y0[HS_FB_H];  /* top source row per output row */
    uint8_t  yw[HS_FB_H];  /* 8-bit weight of row y0+1 */
    uint16_t src_w, src_h;
    uint8_t  sharp;        /* 1 = sharp area-weighted, 0 = nearest */
    uint8_t  _pad[3];
} hs_scaler_t;

/* Build LUTs for src_w x src_h -> 640x480 (src dims 1..640 x 1..480).
 * Returns 0 on success. Divisions happen only here, never per pixel. */
int hs_scaler_init(hs_scaler_t *sc, uint32_t src_w, uint32_t src_h, hs_video_mode_t mode);

/* Stretch src (XRGB8888, src_stride pixels) to fill all 640x480 of dst.
 * row_tmp must hold 2*640 uint32_t (caller-owned scratch). */
void hs_scaler_run(const hs_scaler_t *sc, const uint32_t *src, size_t src_stride,
                   uint32_t *dst, size_t dst_stride, uint32_t *row_tmp);

/* Integer FXAA over w x h XRGB8888. src != dst. luma_tmp: w*h bytes. */
void hs_fxaa(const uint32_t *src, size_t src_stride, uint32_t *dst, size_t dst_stride,
             uint32_t w, uint32_t h, uint8_t *luma_tmp);

/* ---- supersampling ----
 * The rasterizer callback renders a strip of the virtual (640*ss)x(480*ss)
 * hi-res target: rows [y0_hi, y0_hi+rows_hi) into strip (stride 640*ss). */
typedef void (*hs_raster_strip_fn)(void *user, uint32_t *strip, uint32_t strip_w,
                                   uint32_t y0_hi, uint32_t rows_hi, uint32_t ss);

#define HS_SSAA_STRIP_BUDGET (320u * 1024u)

uint32_t hs_ssaa_factor(hs_video_mode_t mode);         /* 1,2,4,8 (else 1) */
uint32_t hs_ssaa_rows_per_strip(uint32_t ss);          /* output rows per strip */
size_t   hs_ssaa_strip_bytes(uint32_t ss);             /* <= HS_SSAA_STRIP_BUDGET */

/* Downsample one strip (rows_out output rows) with an ss x ss box filter. */
void hs_ssaa_downsample(const uint32_t *strip, uint32_t ss, uint32_t rows_out,
                        uint32_t *dst, size_t dst_stride);

/* Full 640x480 SSAA frame: loops strips -> raster -> downsample. */
void hs_ssaa_render(uint32_t ss, hs_raster_strip_fn fn, void *user,
                    uint32_t *strip_buf, uint32_t *dst, size_t dst_stride);

#endif /* HS_VIDEO_H */
