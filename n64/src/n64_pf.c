/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_pf.c - Mali-G31 Panfrost path, step 1 (component profile: no syscalls).
 *   - device probe through DRM_IOCTL_VERSION + DRM_IOCTL_PANFROST_GET_PARAM
 *   - buffer objects (CREATE_BO / MMAP_BO / GEM_CLOSE)
 *   - 64-bit job descriptor encoder + WRITE_VALUE payload
 *   - a GPU memory fill built from a WRITE_VALUE job chain, with completion
 *     status check and full read-back verification
 *   - an in-process mock Panfrost device that executes WRITE_VALUE chains,
 *     so all of the above is exercised in the test suite without hardware
 *   - the RDP -> GPU batch builder fed by hs_n64_walk_backend()
 * All state lives in caller structs; nothing here is a writable global.
 */
#include <string.h>
#include "n64_pf.h"

_Static_assert(sizeof(hs_drm_version_t) == 64, "drm_version is 64 bytes on LP64");
_Static_assert(sizeof(hs_pf_submit_t) == 48, "drm_panfrost_submit");
_Static_assert(sizeof(hs_pf_wait_bo_t) == 16, "drm_panfrost_wait_bo");
_Static_assert(sizeof(hs_pf_create_bo_t) == 24, "drm_panfrost_create_bo");
_Static_assert(sizeof(hs_pf_mmap_bo_t) == 16, "drm_panfrost_mmap_bo");
_Static_assert(sizeof(hs_pf_get_param_t) == 16, "drm_panfrost_get_param");

static void wr32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32); }

/* ------------------------------------------------------------ probe */
static int get_param(hs_pf_dev_t *d, uint32_t param, uint64_t *v)
{
    hs_pf_get_param_t gp;
    memset(&gp, 0, sizeof(gp));
    gp.param = param;
    if (d->os->ioctl(d->os->user, d->fd, HS_PF_IOCTL_GET_PARAM, &gp) != 0) return HS_PF_EPARAM;
    *v = gp.value;
    return HS_PF_OK;
}

int hs_pf_open(hs_pf_dev_t *d, const hs_pf_os_t *os, const char *path)
{
    hs_drm_version_t ver;
    char name[16];
    hs_pf_info_t *in;
    uint64_t sp;
    int r = HS_PF_OK;
    memset(d, 0, sizeof(*d));
    d->os = os;
    d->fd = os->open_dev(os->user, path);
    if (d->fd < 0) return HS_PF_ENODEV;
    memset(&ver, 0, sizeof(ver));
    memset(name, 0, sizeof(name));
    ver.name_len = sizeof(name) - 1u;
    ver.name = (uint64_t)(uintptr_t)name;
    if (os->ioctl(os->user, d->fd, HS_DRM_IOCTL_VERSION, &ver) != 0 || strcmp(name, "panfrost") != 0) {
        os->close_dev(os->user, d->fd); d->fd = -1;
        return HS_PF_ENOTPANFROST;
    }
    in = &d->info;
    memcpy(in->driver, name, sizeof(in->driver));
    in->drm_major = ver.major; in->drm_minor = ver.minor; in->drm_patch = ver.patch;
    r |= get_param(d, HS_PF_PARAM_GPU_PROD_ID, &in->prod_id);
    r |= get_param(d, HS_PF_PARAM_GPU_REVISION, &in->revision);
    r |= get_param(d, HS_PF_PARAM_SHADER_PRESENT, &in->shader_present);
    r |= get_param(d, HS_PF_PARAM_TILER_PRESENT, &in->tiler_present);
    r |= get_param(d, HS_PF_PARAM_L2_PRESENT, &in->l2_present);
    r |= get_param(d, HS_PF_PARAM_JS_PRESENT, &in->js_present);
    r |= get_param(d, HS_PF_PARAM_CORE_FEATURES, &in->core_features);
    r |= get_param(d, HS_PF_PARAM_THREAD_FEATURES, &in->thread_features);
    r |= get_param(d, HS_PF_PARAM_MAX_THREADS, &in->max_threads);
    r |= get_param(d, HS_PF_PARAM_TEXTURE_FEATURES0, &in->texture_features0);
    r |= get_param(d, HS_PF_PARAM_MMU_FEATURES, &in->mmu_features);
    if (r != HS_PF_OK) { os->close_dev(os->user, d->fd); d->fd = -1; return HS_PF_EPARAM; }
    for (sp = in->shader_present; sp; sp &= sp - 1u) in->shader_cores++;
    in->is_g31 = (in->prod_id == HS_PF_PROD_G31);
    return HS_PF_OK;
}

void hs_pf_close(hs_pf_dev_t *d)
{
    if (d->fd >= 0 && d->os) d->os->close_dev(d->os->user, d->fd);
    d->fd = -1;
}

int hs_pf_bo_new(hs_pf_dev_t *d, uint32_t size, uint32_t flags, hs_pf_bo_t *bo)
{
    hs_pf_create_bo_t cb;
    hs_pf_mmap_bo_t mb;
    memset(bo, 0, sizeof(*bo));
    memset(&cb, 0, sizeof(cb));
    cb.size = (size + 4095u) & ~4095u; cb.flags = flags;
    if (d->os->ioctl(d->os->user, d->fd, HS_PF_IOCTL_CREATE_BO, &cb) != 0 || cb.offset == 0) return HS_PF_EBO;
    memset(&mb, 0, sizeof(mb));
    mb.handle = cb.handle;
    if (d->os->ioctl(d->os->user, d->fd, HS_PF_IOCTL_MMAP_BO, &mb) != 0) goto fail;
    bo->cpu = (uint8_t *)d->os->map(d->os->user, d->fd, mb.offset, cb.size);
    if (!bo->cpu) goto fail;
    bo->handle = cb.handle; bo->size = cb.size; bo->gpu_va = cb.offset;
    return HS_PF_OK;
fail:
    {
        hs_drm_gem_close_t gc;
        memset(&gc, 0, sizeof(gc));
        gc.handle = cb.handle;
        (void)d->os->ioctl(d->os->user, d->fd, HS_DRM_IOCTL_GEM_CLOSE, &gc);
    }
    return HS_PF_EBO;
}

void hs_pf_bo_free(hs_pf_dev_t *d, hs_pf_bo_t *bo)
{
    hs_drm_gem_close_t gc;
    if (!bo->handle) return;
    if (bo->cpu) d->os->unmap(d->os->user, bo->cpu, bo->size);
    memset(&gc, 0, sizeof(gc));
    gc.handle = bo->handle;
    (void)d->os->ioctl(d->os->user, d->fd, HS_DRM_IOCTL_GEM_CLOSE, &gc);
    memset(bo, 0, sizeof(*bo));
}

/* --------------------------------------------------------- job encoding
 * 64-bit job descriptor header (32 bytes, little endian):
 *   +0  u32 exception status   (written by the GPU, 1 = DONE)
 *   +4  u32 first incomplete task
 *   +8  u64 fault pointer
 *   +16 u32 bit0 64-bit descriptor, bits1-7 job type, bit8 barrier, bits16-31 job index
 *   +20 u32 bits0-15 dependency 1, bits16-31 dependency 2 (job indices)
 *   +24 u64 next job (GPU VA, 0 ends the chain)
 * WRITE_VALUE payload (+32): u64 target VA, u32 value type, u32 0, u64 immediate. */
void hs_pf_encode_job_header(uint8_t *p, uint32_t type, uint32_t index, uint32_t dep1, uint64_t next_va)
{
    memset(p, 0, 32);
    wr32(p + 16, 1u | ((type & 0x7Fu) << 1) | ((index & 0xFFFFu) << 16));
    wr32(p + 20, dep1 & 0xFFFFu);
    wr64(p + 24, next_va);
}

void hs_pf_encode_write_value(uint8_t *p, uint64_t addr, uint32_t wv_type, uint64_t imm)
{
    memset(p, 0, 24);
    wr64(p, addr);
    wr32(p + 8, wv_type);
    wr64(p + 16, imm);
}

int hs_pf_fill_write_value(hs_pf_dev_t *d, hs_pf_bo_t *img, uint32_t w, uint32_t h, uint32_t stride,
                           uint32_t colour, hs_pf_fill_result_t *res)
{
    hs_pf_bo_t jobs;
    hs_pf_submit_t sb;
    hs_pf_wait_bo_t wb;
    uint32_t handles[2], njobs, per_row, x, y, j = 0, i;
    uint64_t imm = ((uint64_t)colour << 32) | colour;
    int r;
    memset(res, 0, sizeof(*res));
    if (!img || !img->cpu || (w & 1u) || w == 0 || h == 0 || stride < w ||
        (uint64_t)stride * h * 4u > img->size || (img->gpu_va & 7u)) return HS_PF_EARG;
    per_row = w / 2u;
    njobs = per_row * h;
    if (njobs > 0xFFFEu) return HS_PF_EARG;              /* 16-bit job indices */
    r = hs_pf_bo_new(d, njobs * HS_PF_JOB_STRIDE, HS_PF_BO_NOEXEC, &jobs);
    if (r) return r;
    for (y = 0; y < h; y++)
        for (x = 0; x < per_row; x++, j++) {
            uint8_t *p = jobs.cpu + (size_t)j * HS_PF_JOB_STRIDE;
            uint64_t next = (j + 1u < njobs) ? jobs.gpu_va + (uint64_t)(j + 1u) * HS_PF_JOB_STRIDE : 0u;
            hs_pf_encode_job_header(p, HS_PF_JOB_WRITE_VALUE, j + 1u, 0u, next);
            hs_pf_encode_write_value(p + 32, img->gpu_va + ((uint64_t)y * stride + 2u * x) * 4u, HS_PF_WV_IMM64, imm);
        }
    memset(&sb, 0, sizeof(sb));
    handles[0] = jobs.handle; handles[1] = img->handle;
    sb.jc = jobs.gpu_va;
    sb.bo_handles = (uint64_t)(uintptr_t)handles; sb.bo_handle_count = 2;
    if (d->os->ioctl(d->os->user, d->fd, HS_PF_IOCTL_SUBMIT, &sb) != 0) { hs_pf_bo_free(d, &jobs); return HS_PF_ESUBMIT; }
    memset(&wb, 0, sizeof(wb));
    wb.handle = img->handle;
    wb.timeout_ns = (int64_t)(d->os->now_ns(d->os->user) + 2000000000ull);
    if (d->os->ioctl(d->os->user, d->fd, HS_PF_IOCTL_WAIT_BO, &wb) != 0) { hs_pf_bo_free(d, &jobs); return HS_PF_EWAIT; }
    res->jobs = njobs;
    res->first_status = rd32(jobs.cpu);
    for (i = 0; i < njobs; i++) if (rd32(jobs.cpu + (size_t)i * HS_PF_JOB_STRIDE) == HS_PF_STATUS_DONE) res->done_jobs++;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            if (rd32(img->cpu + ((size_t)y * stride + x) * 4u) != colour) res->bad_pixels++;
    hs_pf_bo_free(d, &jobs);
    if (res->done_jobs != njobs) return HS_PF_EJOB;
    return res->bad_pixels ? HS_PF_EVERIFY : HS_PF_OK;
}

/* ------------------------------------------------------------ mock device
 * Emulates the kernel side of the ioctls used above and executes 64-bit
 * WRITE_VALUE job chains against its arena, writing exception status like
 * the GPU does. Any other job type is recorded as a bad job (status 0x40). */
static hs_pf_mock_t *mk(void *u) { return (hs_pf_mock_t *)u; }
#define MOCK_VA_BASE 0x0000000010000000ull

static uint8_t *mock_va(hs_pf_mock_t *m, uint64_t va, uint32_t len)
{
    if (va < MOCK_VA_BASE || va - MOCK_VA_BASE + len > m->used) return NULL;
    return m->arena + (va - MOCK_VA_BASE);
}

static int mock_open(void *u, const char *path) { (void)path; mk(u)->open_fd = 7u; return 7; }
static void mock_close(void *u, int fd) { (void)fd; mk(u)->open_fd = 0; }
static void *mock_map(void *u, int fd, uint64_t off, size_t len)
{
    hs_pf_mock_t *m = mk(u);
    (void)fd;
    if (off + len > m->used) return NULL;
    return m->arena + off;
}
static void mock_unmap(void *u, void *p, size_t len) { (void)u; (void)p; (void)len; }
static uint64_t mock_now(void *u) { (void)u; return 1000u; }

static int mock_run_chain(hs_pf_mock_t *m, uint64_t jc)
{
    uint32_t guard;
    for (guard = 0; jc && guard < 0x10000u; guard++) {
        uint8_t *h = mock_va(m, jc, 64);
        uint32_t ctl, type;
        if (!h) return -14;
        ctl = rd32(h + 16); type = (ctl >> 1) & 0x7Fu;
        if (!(ctl & 1u) || type != HS_PF_JOB_WRITE_VALUE) { wr32(h, 0x40u); m->bad_jobs++; }
        else {
            uint64_t addr = rd64(h + 32), imm = rd64(h + 48);
            uint32_t wt = rd32(h + 40);
            uint8_t *t = mock_va(m, addr, wt == HS_PF_WV_IMM64 ? 8u : 4u);
            if (!t || (wt != HS_PF_WV_IMM64 && wt != HS_PF_WV_IMM32)) { wr32(h, 0x40u); m->bad_jobs++; }
            else {
                if (wt == HS_PF_WV_IMM64) wr64(t, imm); else wr32(t, (uint32_t)imm);
                wr32(h, HS_PF_STATUS_DONE);
                m->jobs_run++;
            }
        }
        jc = rd64(h + 24);
    }
    return 0;
}

static int mock_ioctl(void *u, int fd, unsigned long req, void *arg)
{
    hs_pf_mock_t *m = mk(u);
    (void)fd;
    m->ioctls++;
    if (req == HS_DRM_IOCTL_VERSION) {
        hs_drm_version_t *v = (hs_drm_version_t *)arg;
        static const char nm[] = "panfrost";
        v->major = 1; v->minor = 3; v->patch = 0;
        if (v->name && v->name_len >= sizeof(nm) - 1u) memcpy((void *)(uintptr_t)v->name, nm, sizeof(nm) - 1u);
        v->name_len = sizeof(nm) - 1u;
        return 0;
    }
    if (req == HS_PF_IOCTL_GET_PARAM) {
        hs_pf_get_param_t *g = (hs_pf_get_param_t *)arg;
        switch (g->param) {
        case HS_PF_PARAM_GPU_PROD_ID: g->value = m->prod_id; break;
        case HS_PF_PARAM_SHADER_PRESENT: g->value = 0x3u; break;      /* MP2 */
        case HS_PF_PARAM_TILER_PRESENT: case HS_PF_PARAM_L2_PRESENT: g->value = 1u; break;
        case HS_PF_PARAM_JS_PRESENT: g->value = 0x7u; break;
        case HS_PF_PARAM_MAX_THREADS: g->value = 512u; break;
        default: g->value = 0u; break;
        }
        return 0;
    }
    if (req == HS_PF_IOCTL_CREATE_BO) {
        hs_pf_create_bo_t *c = (hs_pf_create_bo_t *)arg;
        uint32_t sz = (c->size + 4095u) & ~4095u;
        if (m->nbo >= 64u || sz == 0 || m->used + sz > HS_PF_MOCK_ARENA) return -12;
        m->bo_off[m->nbo] = m->used; m->bo_size[m->nbo] = sz;
        memset(m->arena + m->used, 0, sz);
        c->handle = m->nbo + 1u; c->offset = MOCK_VA_BASE + m->used;
        m->used += sz; m->nbo++;
        return 0;
    }
    if (req == HS_PF_IOCTL_MMAP_BO) {
        hs_pf_mmap_bo_t *b = (hs_pf_mmap_bo_t *)arg;
        if (b->handle == 0 || b->handle > m->nbo) return -2;
        b->offset = m->bo_off[b->handle - 1u];
        return 0;
    }
    if (req == HS_DRM_IOCTL_GEM_CLOSE) return 0;
    if (req == HS_PF_IOCTL_SUBMIT) {
        hs_pf_submit_t *s = (hs_pf_submit_t *)arg;
        m->submits++; m->last_requirements = s->requirements;
        return mock_run_chain(m, s->jc);
    }
    if (req == HS_PF_IOCTL_WAIT_BO) { m->waits++; return 0; }
    return -25;                                              /* ENOTTY */
}

void hs_pf_mock_init(hs_pf_mock_t *m, hs_pf_os_t *os, uint64_t prod_id)
{
    m->used = 0; m->nbo = 0; m->open_fd = 0; m->submits = 0; m->jobs_run = 0; m->bad_jobs = 0;
    m->ioctls = 0; m->waits = 0; m->last_requirements = 0; m->prod_id = prod_id;
    os->user = m; os->open_dev = mock_open; os->ioctl = mock_ioctl; os->map = mock_map;
    os->unmap = mock_unmap; os->close_dev = mock_close; os->now_ns = mock_now;
}

/* ------------------------------------------------------- batch builder */
static uint64_t fnv64(uint64_t h, const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    size_t i;
    for (i = 0; i < n; i++) { h ^= b[i]; h *= 0x00000100000001b3ull; }
    return h;
}

static uint64_t state_key(const n64_bk_state_t *st)
{
    uint64_t h = 0xcbf29ce484222325ull;
    uint32_t w[11];
    w[0] = st->combine_hi; w[1] = st->combine_lo; w[2] = st->othermode_hi; w[3] = st->othermode_lo;
    w[4] = st->prim_color; w[5] = st->env_color; w[6] = st->fog_color; w[7] = st->blend_color;
    w[8] = st->fill_color; w[9] = st->tile; w[10] = st->flags;
    h = fnv64(h, w, sizeof(w));
    return fnv64(h, st->tile_desc, sizeof(st->tile_desc)) ^ (fnv64(h, st->tile_size, sizeof(st->tile_size)) << 1);
}

static uint32_t tmem_id(hs_pf_batch_t *b, const n64_bk_state_t *st)
{
    uint64_t th = fnv64(0xcbf29ce484222325ull, st->tmem, 4096);
    uint32_t i;
    if (th == b->last_tmem && b->ntmem) {
        for (i = 0; i < b->ntmem; i++) if (b->tmem_hash[i] == th) return i;
    }
    for (i = 0; i < b->ntmem; i++) if (b->tmem_hash[i] == th) { b->last_tmem = th; return i; }
    if (b->ntmem < 64u) b->tmem_hash[b->ntmem++] = th; else b->overflow++;
    b->last_tmem = th;
    return b->ntmem - 1u;
}

static void push_vtx(hs_pf_batch_t *b, float x, float y, float z, float rhw, float s, float t, uint32_t rgba)
{
    hs_pf_vtx_t *v;
    if (b->nvtx >= b->vtx_cap) { b->overflow++; return; }
    v = &b->vtx[b->nvtx++];
    v->x = x; v->y = y; v->z = z; v->rhw = rhw; v->s = s; v->t = t; v->rgba = rgba; v->pad = 0;
}

static void open_draw(hs_pf_batch_t *b, const n64_bk_state_t *st, uint32_t kind)
{
    uint64_t k = state_key(st) ^ ((uint64_t)kind << 60);
    uint32_t tid = tmem_id(b, st);
    if (b->ndraw && b->last_key == k && b->draw[b->ndraw - 1u].tmem_id == tid) return;
    if (b->ndraw >= b->draw_cap) { b->overflow++; return; }
    b->draw[b->ndraw].first = b->nvtx; b->draw[b->ndraw].count = 0;
    b->draw[b->ndraw].state_key_lo = (uint32_t)k; b->draw[b->ndraw].state_key_hi = (uint32_t)(k >> 32);
    b->draw[b->ndraw].tmem_id = tid; b->draw[b->ndraw].kind = kind;
    b->ndraw++;
    b->last_key = k;
}

static float ndc_x(const hs_pf_batch_t *b, int32_t x16) { return (float)((double)x16 / 65536.0 / (double)b->native_w * 2.0 - 1.0); }
static float ndc_y(const hs_pf_batch_t *b, int32_t y16) { return (float)(1.0 - (double)y16 / 65536.0 / (double)b->native_h * 2.0); }

static void bb_begin(void *u, uint32_t w, uint32_t h, uint32_t ss)
{
    hs_pf_batch_t *b = (hs_pf_batch_t *)u;
    (void)ss;
    b->native_w = w ? w : 320u; b->native_h = h ? h : 240u;
    b->nvtx = 0; b->ndraw = 0; b->ntmem = 0; b->overflow = 0; b->fills = 0; b->texrects = 0;
    b->last_key = 0; b->last_tmem = 0;
}

static void bb_tri(void *u, const n64_bk_vertex_t v[3], const n64_bk_state_t *st)
{
    hs_pf_batch_t *b = (hs_pf_batch_t *)u;
    uint32_t i, before;
    open_draw(b, st, 0u);
    before = b->nvtx;
    for (i = 0; i < 3u; i++)
        push_vtx(b, ndc_x(b, v[i].x), ndc_y(b, v[i].y), (float)((double)v[i].z / 65535.0),
                 (float)(65536.0 / (double)(v[i].w > 1 ? v[i].w : 1)),
                 (float)((double)v[i].s / 65536.0), (float)((double)v[i].t / 65536.0), v[i].rgba);
    if (b->ndraw) b->draw[b->ndraw - 1u].count += b->nvtx - before;
}

static void quad(hs_pf_batch_t *b, int32_t x0, int32_t y0, int32_t x1, int32_t y1, float s0, float t0, float s1, float t1)
{
    /* 10.2 corners -> two triangles */
    float ax = ndc_x(b, x0 << 14), ay = ndc_y(b, y0 << 14), bx = ndc_x(b, x1 << 14), by = ndc_y(b, y1 << 14);
    uint32_t before = b->nvtx;
    push_vtx(b, ax, ay, 0.0f, 1.0f, s0, t0, 0xFFFFFFFFu); push_vtx(b, bx, ay, 0.0f, 1.0f, s1, t0, 0xFFFFFFFFu);
    push_vtx(b, ax, by, 0.0f, 1.0f, s0, t1, 0xFFFFFFFFu); push_vtx(b, bx, ay, 0.0f, 1.0f, s1, t0, 0xFFFFFFFFu);
    push_vtx(b, bx, by, 0.0f, 1.0f, s1, t1, 0xFFFFFFFFu); push_vtx(b, ax, by, 0.0f, 1.0f, s0, t1, 0xFFFFFFFFu);
    if (b->ndraw) b->draw[b->ndraw - 1u].count += b->nvtx - before;
}

static void bb_fill(void *u, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int z, const n64_bk_state_t *st)
{
    hs_pf_batch_t *b = (hs_pf_batch_t *)u;
    open_draw(b, st, z ? 2u : 1u);
    quad(b, x0, y0, x1, y1, 0.0f, 0.0f, 0.0f, 0.0f);
    b->fills++;
}

static void bb_trect(void *u, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t s, int32_t t, int32_t ds, int32_t dt,
                     const n64_bk_state_t *st)
{
    hs_pf_batch_t *b = (hs_pf_batch_t *)u;
    float s0 = (float)s / 32.0f, t0 = (float)t / 32.0f;
    float s1 = s0 + (float)((double)(x1 - x0) / 4.0 * (double)ds / 1024.0);
    float t1 = t0 + (float)((double)(y1 - y0) / 4.0 * (double)dt / 1024.0);
    open_draw(b, st, 3u);
    quad(b, x0, y0, x1, y1, s0, t0, s1, t1);
    b->texrects++;
}

static void bb_end(void *u) { (void)u; }

void hs_pf_batch_init(hs_pf_batch_t *b, hs_pf_vtx_t *vtx, uint32_t vtx_cap, hs_pf_draw_t *draw, uint32_t draw_cap)
{
    memset(b, 0, sizeof(*b));
    b->vtx = vtx; b->vtx_cap = vtx_cap; b->draw = draw; b->draw_cap = draw_cap;
    b->native_w = 320u; b->native_h = 240u;
}

void hs_pf_batch_backend(hs_pf_batch_t *b, n64_rdp_backend_t *bk)
{
    bk->user = b; bk->begin_frame = bb_begin; bk->draw_tri = bb_tri; bk->fill_rect = bb_fill;
    bk->tex_rect = bb_trect; bk->end_frame = bb_end;
}
