/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_psx.c - hs_core_ops_t glue for hs-psx: init/reset/load, the three
 * phases, the commit barrier, delta snapshot publication and hashing.
 */
#include "psx_internal.h"

#include <string.h>

static size_t psx_state_size(void) { return sizeof(hs_psx_t); }

static void mark_all_stale(hs_psx_t *s)
{
    memset(s->vram_stale, 1, sizeof(s->vram_stale));
    memset(s->spu_stale, 1, sizeof(s->spu_stale));
}

static void psx_reset(void *core)
{
    hs_psx_t *s = (hs_psx_t *)core;
    psx_bus_reset(s);
    psx_gpu_reset(s);
    psx_hle_reset(s);
    memset(s->vram0, 0, sizeof(s->vram0));
    memset(s->spu_ram0, 0, sizeof(s->spu_ram0));
    memset(s->spu_reg, 0, sizeof(s->spu_reg));
    s->spu_xfer_addr = 0;
    s->frame = 0;
    s->frame_cyc = 0;
    s->exe_loaded = 0;
    s->bad_access = 0;
    s->input = 0;
    mark_all_stale(s);
    s->front = 0;
    s->slot[0].nprims = s->slot[1].nprims = 0;
    s->slot[0].n_ev = s->slot[1].n_ev = 0;
    s->slot[0].dropped = s->slot[1].dropped = 0;
    s->slot[0].ev_dropped = s->slot[1].ev_dropped = 0;
    psx_cpu_reset(s, 0xBFC00000u);
    if (s->exe && !s->bios)
        (void)psx_exe_load(s);
    psx_spu_audio_reset(&s->aud);
}

static int psx_init(void *mem, size_t mem_size, const hs_core_config_t *cfg)
{
    hs_psx_t *s = (hs_psx_t *)mem;
    (void)cfg;
    if (!mem || mem_size < sizeof(hs_psx_t) || ((uintptr_t)mem & 63u)) return -1;
    memset(s, 0, sizeof(*s));
    s->magic = PSX_MAGIC;
    psx_reset(s);
    return 0;
}

int hs_psx_load_bios(void *core, const uint8_t *bios, size_t len)
{
    hs_psx_t *s = (hs_psx_t *)core;
    if (!bios || len != HS_PSX_BIOS_SIZE) return -1;
    s->bios = bios;
    psx_reset(s);
    return 0;
}

static int psx_load_rom(void *core, const uint8_t *rom, size_t len)
{
    hs_psx_t *s = (hs_psx_t *)core;
    if (!rom) return -1;
    if (len >= 0x800u && memcmp(rom, "PS-X EXE", 8) == 0) {
        s->exe = rom;
        s->exe_len = len;
        psx_reset(s);
        return s->bios ? 0 : (s->exe_loaded ? 0 : -1);
    }
    if (len == HS_PSX_BIOS_SIZE) return hs_psx_load_bios(core, rom, len);
    return -1;
}

static int psx_load_sram(void *core, const uint8_t *buf, size_t len) { (void)core; (void)buf; (void)len; return -1; }
static size_t psx_save_sram(const void *core, uint8_t *buf, size_t cap) { (void)core; (void)buf; (void)cap; return 0; }

/* ---------------- PHASE 0 ---------------- */
static void publish_deltas(hs_psx_t *s)
{
    uint32_t bi = s->front ^ 1u, y, k;
    psx_slot_t *b = &s->slot[bi];
    for (y = 0; y < PSX_VRAM_H; y++)
        if (s->vram_stale[bi][y]) {
            memcpy(b->vram + y * PSX_VRAM_W, s->vram0 + y * PSX_VRAM_W, PSX_VRAM_W * 2u);
            s->vram_stale[bi][y] = 0;
        }
    for (k = 0; k < PSX_SPU_NBLK; k++)
        if (s->spu_stale[bi][k]) {
            memcpy(b->spu_ram + k * PSX_SPU_BLK, s->spu_ram0 + k * PSX_SPU_BLK, PSX_SPU_BLK);
            s->spu_stale[bi][k] = 0;
        }
}

static void psx_step_cpu_phase(void *core, uint32_t input_bits, int64_t rtc_epoch_sec)
{
    hs_psx_t *s = (hs_psx_t *)core;
    (void)rtc_epoch_sec;               /* PS1 has no RTC */
    s->input = input_bits & ~(uint32_t)HS_BTN_MENU;
    psx_gpu_frame_begin(s);
    psx_spu_frame_begin(s);
    psx_irq(s, 0);                     /* VBLANK opens the frame */
    psx_hle_vblank(s);
    psx_cpu_run(s, HS_PSX_CYCLES_PER_FRAME);
    psx_gpu_frame_end(s);
    publish_deltas(s);
    psx_back(s)->frame = s->frame;
    s->frame++;
    s->frame_cyc = 0;                  /* host pokes between frames land at sample 0 */
}

/* ---------------- BARRIER ---------------- */
static void psx_commit_phase(void *core)
{
    hs_psx_t *s = (hs_psx_t *)core;
    psx_slot_t *nb;
    s->front ^= 1u;
    /* the new BACK slot starts empty: Phase 0 appends to it until the next barrier */
    nb = psx_back(s);
    nb->nprims = 0; nb->dropped = 0;
    nb->n_ev = 0; nb->ev_dropped = 0;
}

/* ---------------- PHASE 1 ---------------- */
static void psx_render_video_phase(void *core, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    psx_raster_frame((hs_psx_t *)core, fb, stride, mode);
}

void hs_psx_set_post_fxaa(void *core, int enable) { ((hs_psx_t *)core)->vid.post_fxaa = enable ? 1 : 0; }

/* ---------------- PHASE 2 ---------------- */
uint32_t hs_psx_synth_audio_native(void *core, int16_t *pcm, uint32_t max_frames)
{
    hs_psx_t *s = (hs_psx_t *)core;
    uint32_t n = HS_PSX_SPU_PER_FRAME;
    psx_spu_synth_frame(s);
    if (n > max_frames) n = max_frames;
    memcpy(pcm, s->aud.native, n * 4u);
    s->aud.prev_l = s->aud.native[2u * (HS_PSX_SPU_PER_FRAME - 1u)];
    s->aud.prev_r = s->aud.native[2u * (HS_PSX_SPU_PER_FRAME - 1u) + 1u];
    return n;
}

static uint32_t psx_synth_audio_phase(void *core, int16_t *pcm, uint32_t max_frames)
{
    hs_psx_t *s = (hs_psx_t *)core;
    psx_audio_priv_t *a = &s->aud;
    uint32_t j, n = HS_PSX_ABI_PER_FRAME;
    psx_spu_synth_frame(s);
    if (n > max_frames) n = max_frames;
    for (j = 0; j < n; j++) {
        uint32_t pos = j * HS_PSX_SPU_PER_FRAME;          /* in 1/800 native samples */
        uint32_t idx = pos / HS_PSX_ABI_PER_FRAME, fr = pos % HS_PSX_ABI_PER_FRAME;
        int32_t l0 = idx ? a->native[2u * (idx - 1u)] : a->prev_l;
        int32_t r0 = idx ? a->native[2u * (idx - 1u) + 1u] : a->prev_r;
        int32_t l1 = a->native[2u * idx], r1 = a->native[2u * idx + 1u];
        pcm[2u * j]      = (int16_t)(l0 + ((l1 - l0) * (int32_t)fr) / (int32_t)HS_PSX_ABI_PER_FRAME);
        pcm[2u * j + 1u] = (int16_t)(r0 + ((r1 - r0) * (int32_t)fr) / (int32_t)HS_PSX_ABI_PER_FRAME);
    }
    a->prev_l = a->native[2u * (HS_PSX_SPU_PER_FRAME - 1u)];
    a->prev_r = a->native[2u * (HS_PSX_SPU_PER_FRAME - 1u) + 1u];
    return n;
}

/* ---------------- hashing ---------------- */
static uint64_t hash_slot_video(uint64_t h, const psx_slot_t *f)
{
    h = hs_fnv1a64(h, &f->nprims, 4);
    h = hs_fnv1a64(h, f->prims, (size_t)f->nprims * sizeof(psx_gpu_prim_t));
    h = hs_fnv1a64(h, &f->present_x, 2); h = hs_fnv1a64(h, &f->present_y, 2);
    h = hs_fnv1a64(h, &f->disp_w, 2); h = hs_fnv1a64(h, &f->disp_h, 2);
    h = hs_fnv1a64(h, &f->disp_24bpp, 1); h = hs_fnv1a64(h, &f->disp_enabled, 1);
    h = hs_fnv1a64(h, f->vram, sizeof(f->vram));
    return h;
}

static uint64_t hash_slot_audio(uint64_t h, const psx_slot_t *f)
{
    h = hs_fnv1a64(h, &f->n_ev, 4);
    h = hs_fnv1a64(h, f->spu_ev, (size_t)f->n_ev * sizeof(psx_spu_ev_t));
    h = hs_fnv1a64(h, f->spu_ram, sizeof(f->spu_ram));
    return h;
}

static void psx_hash_state(const void *core, uint64_t *vh, uint64_t *ah)
{
    const hs_psx_t *s = (const hs_psx_t *)core;
    const psx_slot_t *f = psx_front(s);
    if (vh) *vh = hash_slot_video(HS_FNV64_OFFSET, f);
    if (ah) *ah = hash_slot_audio(HS_FNV64_OFFSET, f);
}

uint64_t hs_psx_hash_back(const void *core)
{
    const hs_psx_t *s = (const hs_psx_t *)core;
    const psx_slot_t *b = &s->slot[s->front ^ 1u];
    return hash_slot_audio(hash_slot_video(HS_FNV64_OFFSET, b), b);
}

uint64_t hs_psx_hash_cpu(const void *core)
{
    const hs_psx_t *s = (const hs_psx_t *)core;
    uint64_t h = HS_FNV64_OFFSET;
    h = hs_fnv1a64(h, &s->cpu, sizeof(s->cpu));
    h = hs_fnv1a64(h, s->ram, sizeof(s->ram));
    h = hs_fnv1a64(h, s->scratch, sizeof(s->scratch));
    h = hs_fnv1a64(h, s->vram0, sizeof(s->vram0));
    h = hs_fnv1a64(h, s->spu_ram0, sizeof(s->spu_ram0));
    h = hs_fnv1a64(h, s->spu_reg, sizeof(s->spu_reg));
    h = hs_fnv1a64(h, &s->istat, 4); h = hs_fnv1a64(h, &s->imask, 4);
    h = hs_fnv1a64(h, s->dma, sizeof(s->dma));
    h = hs_fnv1a64(h, s->tmr, sizeof(s->tmr));
    h = hs_fnv1a64(h, &s->gpu, sizeof(s->gpu));
    h = hs_fnv1a64(h, &s->frame, 4);
    return h;
}

const char *hs_psx_tty(const void *core, uint32_t *len)
{
    const hs_psx_t *s = (const hs_psx_t *)core;
    if (len) *len = s->tty_len;
    return s->tty;
}

uint32_t hs_psx_frame_count(const void *core) { return ((const hs_psx_t *)core)->frame; }
uint32_t hs_psx_prim_count(const void *core) { return psx_front((const hs_psx_t *)core)->nprims; }

/* ---------------- backend export ---------------- */
void hs_psx_export_cmdlist(const void *core, psx_gpu_cmdlist_t *out)
{
    const psx_slot_t *f = psx_front((const hs_psx_t *)core);
    out->prims = f->prims;
    out->count = f->nprims;
    out->dropped = f->dropped;
    out->vram = f->vram;
    out->present_x = f->present_x; out->present_y = f->present_y;
    out->disp_w = f->disp_w; out->disp_h = f->disp_h;
    out->disp_24bpp = f->disp_24bpp; out->disp_enabled = f->disp_enabled;
    out->frame = (uint16_t)f->frame;
}

int hs_psx_render_backend(void *core, const psx_gpu_backend_t *be, hs_video_mode_t mode,
                          uint32_t *fb, size_t stride)
{
    psx_gpu_cmdlist_t cl;
    uint32_t i;
    if (!be || !be->begin_frame || !be->draw || !be->resolve) return -1;
    hs_psx_export_cmdlist(core, &cl);
    if (be->begin_frame(be->user, &cl, hs_ssaa_factor(mode)) != 0) return -1;
    for (i = 0; i < cl.count; i++) be->draw(be->user, &cl.prims[i], i);
    be->resolve(be->user, fb, stride);
    return 0;
}

const hs_core_ops_t hs_psx_core_ops = {
    HS_CORE_ABI_VERSION,
    HS_SYS_PS1,
    "hs-psx",
    320u, 240u,
    60u, 1u,
    psx_state_size,
    psx_init,
    psx_reset,
    psx_load_rom,
    psx_load_sram,
    psx_save_sram,
    psx_step_cpu_phase,
    psx_commit_phase,
    psx_render_video_phase,
    psx_synth_audio_phase,
    psx_hash_state
};
