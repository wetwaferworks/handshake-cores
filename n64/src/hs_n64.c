/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_n64.c - hs_core_ops_t glue for the hs-n64 clean-room N64 core.
 */
#include <stddef.h>
#include <string.h>
#include "n64_internal.h"

static size_t n64_state_size(void) { return sizeof(hs_n64_t); }

static void init_state(hs_n64_t *n, uint32_t flags, int keep_save)
{
    /* the cartridge save + controller pak (last member) survive a console reset */
    memset(n, 0, keep_save ? offsetof(hs_n64_t, save) : sizeof(*n));
    n->magic = N64_MAGIC;
    n->cfg_flags = flags;
    n->rdram_size = (n->cfg_flags & HS_N64_CFG_EXPANSION_PAK) ? N64_RDRAM_MAX : N64_RDRAM_BASE_SIZE;
    n64_cpu_reset(n);
    n->rsp.status = 1u;
}

static int n64_init(void *mem, size_t mem_size, const hs_core_config_t *cfg)
{
    if (!mem || mem_size < sizeof(hs_n64_t)) return -1;
    init_state((hs_n64_t *)mem, cfg ? cfg->flags : 0u, 0);
    return 0;
}

static void n64_reset(void *core)
{
    hs_n64_t *n = (hs_n64_t *)core;
    const uint8_t *rom = n->rom;
    size_t len = n->rom_len;
    init_state(n, n->cfg_flags, 1);
    if (rom && n64_rom_load(n, rom, len) == 0) n64_pif_boot(n);
}

static int n64_load_rom(void *core, const uint8_t *rom, size_t len)
{
    hs_n64_t *n = (hs_n64_t *)core;
    if (n64_rom_load(n, rom, len) != 0) return -1;
    n64_pif_boot(n);
    return 0;
}

/* save file = cartridge save memory (type-sized) followed, when the controller
 * pak is enabled, by the 32 KiB pak image */
static int n64_load_sram(void *core, const uint8_t *buf, size_t len)
{
    hs_n64_t *n = (hs_n64_t *)core;
    n64_save_t *s = &n->save;
    size_t want = s->size + (s->pak_on ? N64_PAK_SIZE : 0u);
    if (!buf || (len != want && len != s->size) || (s->size == 0u && !s->pak_on)) return -1;
    memcpy(s->data, buf, s->size);
    if (len == want && s->pak_on) memcpy(s->pak, buf + s->size, N64_PAK_SIZE);
    return 0;
}

static size_t n64_save_sram(const void *core, uint8_t *buf, size_t cap)
{
    const hs_n64_t *n = (const hs_n64_t *)core;
    const n64_save_t *s = &n->save;
    size_t want = s->size + (s->pak_on ? N64_PAK_SIZE : 0u);
    if (!buf) return want;
    if (cap < want) return 0;
    memcpy(buf, s->data, s->size);
    if (s->pak_on) memcpy(buf + s->size, s->pak, N64_PAK_SIZE);
    return want;
}

static void n64_step(void *core, uint32_t input_bits, int64_t rtc_epoch_sec)
{
    hs_n64_t *n = (hs_n64_t *)core;
    n64_snap_t *b = n64_back(n);
    const n64_snap_t *f = n64_front(n);
    (void)rtc_epoch_sec;
    n->input = input_bits;
    b->frame = n->frame;
    b->rdp_n = 0; b->tex_n = 0; b->ai_n = 0; b->ntris = 0; b->overflow = 0;
    b->gfx_tasks = 0; b->audio_tasks = 0; b->native_w = 0; b->native_h = 0;
    b->ai_freq = n->ai_dacrate ? N64_VI_CLOCK_NTSC / (n->ai_dacrate + 1u) : 0u;
    n64_vi_frame_start(n);
    n64_cpu_run(n, N64_INSTR_PER_FRAME);
    if (b->rdp_n == 0u && f->rdp_n != 0u) {          /* no new display list: re-present the last one */
        memcpy(b->rdp, f->rdp, (size_t)f->rdp_n * 4u);
        memcpy(b->tex, f->tex, f->tex_n);
        b->rdp_n = f->rdp_n; b->tex_n = f->tex_n; b->ntris = f->ntris;
        b->native_w = f->native_w; b->native_h = f->native_h;
    }
    if (!b->native_w) b->native_w = 320u;
    if (!b->native_h) b->native_h = 240u;
    b->vi_origin = n->vi_reg[1]; b->vi_width = n->vi_reg[2]; b->vi_status = n->vi_reg[0];
    b->vi_xscale = n->vi_reg[12]; b->vi_yscale = n->vi_reg[13];
    n->frame++;
}

static void n64_commit(void *core)
{
    hs_n64_t *n = (hs_n64_t *)core;
    n->front ^= 1u;
}

static void n64_render_phase(void *core, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    n64_render((hs_n64_t *)core, fb, stride, mode);
}

static uint32_t n64_synth_phase(void *core, int16_t *pcm, uint32_t max_frames)
{
    return n64_audio_synth((hs_n64_t *)core, pcm, max_frames);
}

static uint64_t fnv_u32(uint64_t h, uint32_t v)
{
    uint8_t b[4];
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
    return hs_fnv1a64(h, b, 4);
}

uint64_t n64_snap_video_hash(const n64_snap_t *s)
{
    uint64_t h = HS_FNV64_OFFSET;
    uint32_t i;
    h = fnv_u32(h, s->native_w); h = fnv_u32(h, s->native_h);
    h = fnv_u32(h, s->vi_origin); h = fnv_u32(h, s->vi_width);
    h = fnv_u32(h, s->rdp_n);
    for (i = 0; i < s->rdp_n; i++) h = fnv_u32(h, s->rdp[i]);
    h = fnv_u32(h, s->tex_n);
    return hs_fnv1a64(h, s->tex, s->tex_n);
}

uint64_t n64_snap_audio_hash(const n64_snap_t *s)
{
    uint64_t h = HS_FNV64_OFFSET;
    uint32_t i;
    h = fnv_u32(h, s->ai_freq); h = fnv_u32(h, s->ai_n);
    for (i = 0; i < s->ai_n * 2u; i++) h = fnv_u32(h, (uint32_t)(uint16_t)s->ai[i]);
    return h;
}

uint64_t n64_cpu_state_hash(const hs_n64_t *n)
{
    uint64_t h = HS_FNV64_OFFSET;
    uint32_t i;
    for (i = 0; i < 32u; i++) { h = fnv_u32(h, (uint32_t)n->cpu.gpr[i]); h = fnv_u32(h, (uint32_t)(n->cpu.gpr[i] >> 32)); }
    for (i = 0; i < 32u; i++) { h = fnv_u32(h, (uint32_t)n->cpu.fpr[i]); h = fnv_u32(h, (uint32_t)(n->cpu.fpr[i] >> 32)); }
    h = fnv_u32(h, (uint32_t)n->cpu.pc);
    h = fnv_u32(h, (uint32_t)n->cpu.cp0[C0_COUNT]);
    h = hs_fnv1a64(h, n->rdram, n->rdram_size);
    return hs_fnv1a64(h, n->rsp.dmem, sizeof(n->rsp.dmem));
}

static void n64_hash(const void *core, uint64_t *vh, uint64_t *ah)
{
    const hs_n64_t *n = (const hs_n64_t *)core;
    const n64_snap_t *s = n64_front(n);
    if (vh) *vh = n64_snap_video_hash(s);
    if (ah) *ah = n64_snap_audio_hash(s);
}

const hs_core_ops_t hs_n64_core_ops = {
    HS_CORE_ABI_VERSION,
    HS_SYS_N64,
    "hs-n64",
    320u, 240u,
    60u, 1u,
    n64_state_size,
    n64_init,
    n64_reset,
    n64_load_rom,
    n64_load_sram,
    n64_save_sram,
    n64_step,
    n64_commit,
    n64_render_phase,
    n64_synth_phase,
    n64_hash
};
