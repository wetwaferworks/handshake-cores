/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_pf.h - hs-n64 Mali-G31 (Bifrost) Panfrost path, step 1.
 *
 * Everything here talks to the kernel ONLY through an hs_pf_os_t table, so
 * the job encoders, the probe logic and the write-value fill can be tested
 * against the in-process mock device (hs_pf_mock_*) without hardware. The
 * real table (open/ioctl/mmap) lives in the runtime-profile TU n64_pf_sys.c.
 *
 * Hardware facts used here (DRM UAPI numbers, job header layout, write-value
 * payload) are interface facts; every one that the mock cannot prove is
 * listed in docs/PANFROST_PLAN.md as "verify on the RG35XX".
 */
#ifndef N64_PF_H
#define N64_PF_H

#include <stddef.h>
#include <stdint.h>
#include "hs_n64.h"

/* ---- DRM / Panfrost UAPI, written out from the kernel ABI (LP64) -------- */
#define HS_IOC(dir, nr, size) ((unsigned long)(((uint32_t)(dir) << 30) | ((uint32_t)(size) << 16) | (0x64u << 8) | (uint32_t)(nr)))
#define HS_IOC_W  1u
#define HS_IOC_RW 3u
#define HS_DRM_CMD 0x40u

typedef struct hs_drm_version {          /* DRM_IOCTL_VERSION, 64 bytes */
    int32_t major, minor, patch, pad;
    uint64_t name_len;  uint64_t name;   /* user pointers carried as u64 */
    uint64_t date_len;  uint64_t date;
    uint64_t desc_len;  uint64_t desc;
} hs_drm_version_t;
typedef struct hs_drm_gem_close { uint32_t handle, pad; } hs_drm_gem_close_t;
typedef struct hs_pf_submit {            /* DRM_IOCTL_PANFROST_SUBMIT, 48 bytes (jm_ctx form; */
    uint64_t jc;                         /*  older 40-byte kernels ignore the tail) */
    uint64_t in_syncs;
    uint32_t in_sync_count, out_sync;
    uint64_t bo_handles;                 /* user pointer to u32 handles */
    uint32_t bo_handle_count, requirements;
    uint32_t jm_ctx_handle, pad;         /* 0 = default job-manager context */
} hs_pf_submit_t;
typedef struct hs_pf_wait_bo { uint32_t handle, pad; int64_t timeout_ns; } hs_pf_wait_bo_t;
typedef struct hs_pf_create_bo { uint32_t size, flags, handle, pad; uint64_t offset; } hs_pf_create_bo_t;
typedef struct hs_pf_mmap_bo { uint32_t handle, flags; uint64_t offset; } hs_pf_mmap_bo_t;
typedef struct hs_pf_get_param { uint32_t param, pad; uint64_t value; } hs_pf_get_param_t;

#define HS_DRM_IOCTL_VERSION        HS_IOC(HS_IOC_RW, 0x00u, sizeof(hs_drm_version_t))
#define HS_DRM_IOCTL_GEM_CLOSE      HS_IOC(HS_IOC_W, 0x09u, sizeof(hs_drm_gem_close_t))
#define HS_PF_IOCTL_SUBMIT          HS_IOC(HS_IOC_W, HS_DRM_CMD + 0x00u, sizeof(hs_pf_submit_t))
#define HS_PF_IOCTL_WAIT_BO         HS_IOC(HS_IOC_W, HS_DRM_CMD + 0x01u, sizeof(hs_pf_wait_bo_t))
#define HS_PF_IOCTL_CREATE_BO       HS_IOC(HS_IOC_RW, HS_DRM_CMD + 0x02u, sizeof(hs_pf_create_bo_t))
#define HS_PF_IOCTL_MMAP_BO         HS_IOC(HS_IOC_RW, HS_DRM_CMD + 0x03u, sizeof(hs_pf_mmap_bo_t))
#define HS_PF_IOCTL_GET_PARAM       HS_IOC(HS_IOC_RW, HS_DRM_CMD + 0x04u, sizeof(hs_pf_get_param_t))

enum {                                   /* DRM_PANFROST_PARAM_* */
    HS_PF_PARAM_GPU_PROD_ID = 0, HS_PF_PARAM_GPU_REVISION = 1, HS_PF_PARAM_SHADER_PRESENT = 2,
    HS_PF_PARAM_TILER_PRESENT = 3, HS_PF_PARAM_L2_PRESENT = 4, HS_PF_PARAM_STACK_PRESENT = 5,
    HS_PF_PARAM_AS_PRESENT = 6, HS_PF_PARAM_JS_PRESENT = 7, HS_PF_PARAM_L2_FEATURES = 8,
    HS_PF_PARAM_CORE_FEATURES = 9, HS_PF_PARAM_TILER_FEATURES = 10, HS_PF_PARAM_MEM_FEATURES = 11,
    HS_PF_PARAM_MMU_FEATURES = 12, HS_PF_PARAM_THREAD_FEATURES = 13, HS_PF_PARAM_MAX_THREADS = 14,
    HS_PF_PARAM_TEXTURE_FEATURES0 = 18, HS_PF_PARAM_NR_CORE_GROUPS = 38, HS_PF_PARAM_THREAD_TLS_ALLOC = 39
};
#define HS_PF_BO_NOEXEC 0x1u
#define HS_PF_PROD_G31  0x7093u          /* GPU_ID[31:16] of Mali-G31 */

/* ---- OS table: the only way this code reaches the kernel ---------------- */
typedef struct hs_pf_os {
    void *user;
    int   (*open_dev)(void *user, const char *path);              /* returns fd or -1 */
    int   (*ioctl)(void *user, int fd, unsigned long req, void *arg); /* 0 or -errno */
    void *(*map)(void *user, int fd, uint64_t offset, size_t len); /* NULL on failure */
    void  (*unmap)(void *user, void *p, size_t len);
    void  (*close_dev)(void *user, int fd);
    uint64_t (*now_ns)(void *user);                                /* CLOCK_MONOTONIC (WAIT_BO is absolute) */
} hs_pf_os_t;

typedef struct hs_pf_info {
    char     driver[16];
    int32_t  drm_major, drm_minor, drm_patch;
    uint64_t prod_id, revision, shader_present, tiler_present, l2_present, js_present;
    uint64_t core_features, thread_features, max_threads, texture_features0, mmu_features;
    uint32_t shader_cores;               /* popcount(shader_present) */
    uint32_t is_g31;
} hs_pf_info_t;

typedef struct hs_pf_bo {
    uint32_t handle, size;
    uint64_t gpu_va;
    uint8_t *cpu;
} hs_pf_bo_t;

typedef struct hs_pf_dev {
    const hs_pf_os_t *os;
    int fd;
    hs_pf_info_t info;
} hs_pf_dev_t;

/* error codes */
enum { HS_PF_OK = 0, HS_PF_ENODEV = -1, HS_PF_ENOTPANFROST = -2, HS_PF_EPARAM = -3, HS_PF_EBO = -4,
       HS_PF_ESUBMIT = -5, HS_PF_EWAIT = -6, HS_PF_EJOB = -7, HS_PF_EVERIFY = -8, HS_PF_EARG = -9 };

int  hs_pf_open(hs_pf_dev_t *d, const hs_pf_os_t *os, const char *path);   /* probe included */
void hs_pf_close(hs_pf_dev_t *d);
int  hs_pf_bo_new(hs_pf_dev_t *d, uint32_t size, uint32_t flags, hs_pf_bo_t *bo);
void hs_pf_bo_free(hs_pf_dev_t *d, hs_pf_bo_t *bo);

/* ---- job encoding (64-bit job descriptors, 64-byte aligned) ------------- */
#define HS_PF_JOB_WRITE_VALUE 2u
#define HS_PF_JOB_FRAGMENT    9u
#define HS_PF_WV_IMM32 6u
#define HS_PF_WV_IMM64 7u
#define HS_PF_JOB_STRIDE 64u
#define HS_PF_STATUS_DONE 1u
void hs_pf_encode_job_header(uint8_t *p, uint32_t type, uint32_t index, uint32_t dep1, uint64_t next_va);
void hs_pf_encode_write_value(uint8_t *p, uint64_t addr, uint32_t wv_type, uint64_t imm);

/* Fill rows [0,h) x [0,w) of a 32-bit image (stride in pixels) at img_va with
 * colour using a chain of WRITE_VALUE jobs (one per 8 bytes), submit, wait,
 * check every job's completion status and read back every pixel. */
typedef struct hs_pf_fill_result {
    uint32_t jobs, done_jobs, bad_pixels;
    uint32_t first_status;
} hs_pf_fill_result_t;
int hs_pf_fill_write_value(hs_pf_dev_t *d, hs_pf_bo_t *img, uint32_t w, uint32_t h, uint32_t stride,
                           uint32_t colour, hs_pf_fill_result_t *res);

/* ---- in-process mock Panfrost device (for tests without hardware) ------- */
#define HS_PF_MOCK_ARENA (8u << 20)
typedef struct hs_pf_mock {
    uint8_t  arena[HS_PF_MOCK_ARENA];
    uint32_t used;
    uint32_t nbo;
    uint32_t bo_off[64], bo_size[64];
    uint32_t open_fd, submits, jobs_run, bad_jobs, ioctls, waits;
    uint64_t prod_id;
    uint32_t last_requirements;
} hs_pf_mock_t;
void hs_pf_mock_init(hs_pf_mock_t *m, hs_pf_os_t *os, uint64_t prod_id);

/* ---- RDP -> GPU batch builder (front end of the future draw path) ---------
 * Fed by hs_n64_walk_backend(); groups consecutive primitives whose full RDP
 * state (combiner, other modes, colours, tiles, TMEM contents) is identical
 * into one GPU draw and converts vertices to the float layout the Mali
 * vertex/tiler jobs will read (x,y in NDC, z 0..1, 1/w, s,t texels, rgba). */
typedef struct hs_pf_vtx { float x, y, z, rhw, s, t; uint32_t rgba; uint32_t pad; } hs_pf_vtx_t;
typedef struct hs_pf_draw { uint32_t first, count, state_key_lo, state_key_hi, tmem_id, kind; } hs_pf_draw_t;
typedef struct hs_pf_batch {
    hs_pf_vtx_t *vtx; uint32_t vtx_cap, nvtx;
    hs_pf_draw_t *draw; uint32_t draw_cap, ndraw;
    uint64_t tmem_hash[64]; uint32_t ntmem;  /* distinct TMEM images = texture uploads */
    uint32_t native_w, native_h, overflow, fills, texrects;
    uint64_t last_key, last_tmem;
} hs_pf_batch_t;
void     hs_pf_batch_init(hs_pf_batch_t *b, hs_pf_vtx_t *vtx, uint32_t vtx_cap, hs_pf_draw_t *draw, uint32_t draw_cap);
void     hs_pf_batch_backend(hs_pf_batch_t *b, n64_rdp_backend_t *bk);  /* fills bk callbacks */

/* n64_pf_sys.c (runtime profile, on-device tool only): the real kernel table */
void     hs_pf_sys_os(hs_pf_os_t *os);

#endif
