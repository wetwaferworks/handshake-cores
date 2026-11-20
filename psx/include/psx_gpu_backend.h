/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_gpu_backend.h - frozen PS1 GPU command-list export + backend hook.
 *
 * Phase 0 decodes every GP0 packet (CPU writes, DMA2 block + linked-list)
 * into this fixed-size, pointer-free primitive list in the BACK snapshot.
 * After commit_phase the FRONT list is immutable until the next barrier, so
 * any Phase 1 consumer (the built-in software rasterizer, or a future
 * Mali-G31 Panfrost/GLES2-3 path) can read it concurrently with Phase 0
 * stepping the next frame, with no locks.
 *
 * Coordinate space: all x/y are absolute VRAM pixel coordinates (drawing
 * offset already applied, 11-bit sign-extended like hardware). A backend
 * renders the rectangle (present_x, present_y, disp_w, disp_h) of VRAM into
 * a render target of any size: scale = target_w / disp_w. For Panfrost:
 *   - upload `vram` (1024x512 R5G5B5A1, little-endian) as a texture once
 *     per frame (or only rows marked in vram_row_dirty),
 *   - for each prim in order: TRI -> 3 vertices, RECT -> 2 triangles,
 *     LINE -> thin quad, FILL/COPY -> scissored clear / blit,
 *   - texture lookup in the fragment shader: tpage_x/y, tex_depth
 *     (0=4-bit CLUT, 1=8-bit CLUT, 2=15-bit direct), clut_x/y, texture
 *     window (tw_mask_x/y, tw_off_x/y in 8-pixel units), texel 0x0000 =
 *     transparent, texel bit 15 = semi-transparent when PSX_PRIM_SEMI,
 *   - blending per abr: 0 B/2+F/2, 1 B+F, 2 B-F, 3 B+F/4,
 *   - modulation: out = texel * vertex_rgb / 128 unless PSX_PRIM_RAW,
 *   - scissor to clip_x0..clip_x1 / clip_y0..clip_y1 (inclusive).
 * Then MSAA/SSAA resolve to 640x480. The same list feeds both paths, so
 * output can be validated against the software rasterizer.
 */
#ifndef PSX_GPU_BACKEND_H
#define PSX_GPU_BACKEND_H

#include <stdint.h>
#include <stddef.h>

#define PSX_MAX_PRIMS 16384u

enum psx_prim_type {
    PSX_PRIM_NONE = 0,
    PSX_PRIM_TRI  = 1,   /* x/y/u/v/rgb [0..2] */
    PSX_PRIM_RECT = 2,   /* x[0],y[0] top-left, w,h; u[0],v[0] */
    PSX_PRIM_LINE = 3,   /* x/y/rgb [0..1] */
    PSX_PRIM_FILL = 4,   /* x[0],y[0],w,h, rgb[0]; ignores clip + mask */
    PSX_PRIM_COPY = 5    /* src x[0],y[0] -> dst x[1],y[1], w,h */
};

enum psx_prim_flags {
    PSX_PRIM_TEXTURED = 1u << 0,
    PSX_PRIM_GOURAUD  = 1u << 1,
    PSX_PRIM_SEMI     = 1u << 2,
    PSX_PRIM_RAW      = 1u << 3,  /* raw texture: no modulation */
    PSX_PRIM_DITHER   = 1u << 4,
    PSX_PRIM_MASK_SET = 1u << 5,
    PSX_PRIM_MASK_CHK = 1u << 6
};

typedef struct psx_gpu_prim {
    uint8_t  type, flags, abr, tex_depth;
    int16_t  x[3], y[3];
    uint8_t  u[3], v[3];
    uint16_t w, h;
    uint16_t tpage_x, tpage_y;    /* texture page base (VRAM px) */
    uint16_t clut_x, clut_y;      /* CLUT base (VRAM px) */
    int16_t  clip_x0, clip_y0, clip_x1, clip_y1;  /* inclusive draw area */
    uint8_t  tw_mask_x, tw_mask_y, tw_off_x, tw_off_y;  /* 8-px units */
    uint32_t rgb[3];              /* 0x00BBGGRR (hardware order) */
} psx_gpu_prim_t;                 /* 64 bytes */

typedef struct psx_gpu_cmdlist {
    const psx_gpu_prim_t *prims;
    uint32_t              count;
    uint32_t              dropped;       /* prims beyond PSX_MAX_PRIMS */
    const uint16_t       *vram;          /* 1024*512, 2D-transfer state */
    uint16_t present_x, present_y;       /* VRAM rect to present */
    uint16_t disp_w, disp_h;             /* 256/320/368/512/640 x 240/480 */
    uint8_t  disp_24bpp, disp_enabled;
    uint16_t frame;
} psx_gpu_cmdlist_t;

/* Backend hook: a Phase 1 consumer. All callbacks receive the frozen list.
 * begin_frame returns 0 to continue. Rendering targets the backend's own
 * surface; `resolve` must leave a full 640x480 frame in fb (stride px). */
typedef struct psx_gpu_backend {
    const char *name;
    void       *user;
    int  (*begin_frame)(void *user, const psx_gpu_cmdlist_t *cl, uint32_t ss_factor);
    void (*draw)(void *user, const psx_gpu_prim_t *p, uint32_t index);
    void (*resolve)(void *user, uint32_t *fb_xrgb8888, size_t stride);
} psx_gpu_backend_t;

#endif
