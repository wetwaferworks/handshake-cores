/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_core.c - hs_core_ops_t wiring for hs-gbc (DMG + CGB).
 * Double-buffered snapshots; see the ownership table in gbc_internal.h.
 */
#include "gbc_internal.h"

#include <string.h>

static size_t gbc_state_size(void) { return sizeof(gbc_t); }

static int gbc_init(void *mem, size_t mem_size, const hs_core_config_t *cfg)
{
    gbc_t *g = (gbc_t *)mem;
    if (!mem || mem_size < sizeof(gbc_t))
        return -1;
    memset(g, 0, sizeof(*g));
    g->console_id = cfg ? cfg->console_id : HS_SYS_GBC;
    if (g->console_id != HS_SYS_GB && g->console_id != HS_SYS_GBC)
        return -2;
    g->dmg_palette = cfg ? (cfg->dmg_palette & 3u) : 0u;
    g->front = 0;
    g->back = 1;
    hs_scaler_init(&g->vid.sc_sharp, GB_W, GB_H, HS_SCALE_STRETCH_SHARP);
    hs_scaler_init(&g->vid.sc_near, GB_W, GB_H, HS_SCALE_STRETCH_NEAREST);
    return 0;
}

static void gbc_reset(void *core)
{
    gbc_t *g = (gbc_t *)core;
    g->cgb = (uint8_t)(g->console_id == HS_SYS_GBC && g->rom_loaded && (g->rom[0x143] & 0x80u));
    gbc_mmu_reset(g);
    gbc_ppu_reset(g);
    gbc_apu_reset(&g->apu, g->cgb);
    gbc_cpu_reset(g);
    g->apu_now = 0;
    g->apu_done = 0;
    g->frame_no = 0;
    g->total_cycles = 0;
    g->instr_count = 0;
    g->front = 0;
    g->back = 1;
    memset(g->vs, 0, sizeof(g->vs));
    memset(g->as, 0, sizeof(g->as));
    g->as[0].start = g->apu;
    g->as[1].start = g->apu;
    g->vs[0].dmg_palette = g->vs[1].dmg_palette = (uint8_t)g->dmg_palette;
    g->vs[0].cgb = g->vs[1].cgb = g->cgb;
}

static int gbc_load_rom(void *core, const uint8_t *rom, size_t len)
{
    gbc_t *g = (gbc_t *)core;
    int rc = gbc_mmu_load_rom(g, rom, len);
    if (rc)
        return rc;
    memset(g->eram, 0, sizeof(g->eram));
    gbc_reset(g);
    return 0;
}

#define RTC_BLOCK 24u

static int gbc_load_sram(void *core, const uint8_t *buf, size_t len)
{
    gbc_t *g = (gbc_t *)core;
    size_t n = len < g->ram_size ? len : g->ram_size;
    if (!buf)
        return -1;
    memcpy(g->eram, buf, n);
    if (g->has_rtc && len >= (size_t)g->ram_size + RTC_BLOCK) {
        const uint8_t *r = buf + g->ram_size;
        if (r[0] == 'H' && r[1] == 'R' && r[2] == 'T' && r[3] == 'C') {
            uint64_t e = 0;
            int i;
            g->rtc.sec = r[4]; g->rtc.min = r[5]; g->rtc.hour = r[6]; g->rtc.dl = r[7]; g->rtc.dh = r[8];
            g->rtc.lsec = r[9]; g->rtc.lmin = r[10]; g->rtc.lhour = r[11]; g->rtc.ldl = r[12]; g->rtc.ldh = r[13];
            for (i = 0; i < 8; i++)
                e |= (uint64_t)r[16 + i] << (8 * i);
            g->rtc.last_epoch = (int64_t)e;
            g->rtc.epoch_valid = 1;
        }
    }
    return 0;
}

static size_t gbc_save_sram(const void *core, uint8_t *buf, size_t cap)
{
    const gbc_t *g = (const gbc_t *)core;
    size_t need = g->ram_size + (g->has_rtc ? RTC_BLOCK : 0u);
    if (!g->has_battery || !buf || cap < need)
        return 0;
    memcpy(buf, g->eram, g->ram_size);
    if (g->has_rtc) {
        uint8_t *r = buf + g->ram_size;
        uint64_t e = (uint64_t)g->rtc.last_epoch;
        int i;
        memset(r, 0, RTC_BLOCK);
        r[0] = 'H'; r[1] = 'R'; r[2] = 'T'; r[3] = 'C';
        r[4] = g->rtc.sec; r[5] = g->rtc.min; r[6] = g->rtc.hour; r[7] = g->rtc.dl; r[8] = g->rtc.dh;
        r[9] = g->rtc.lsec; r[10] = g->rtc.lmin; r[11] = g->rtc.lhour; r[12] = g->rtc.ldl; r[13] = g->rtc.ldh;
        for (i = 0; i < 8; i++)
            r[16 + i] = (uint8_t)(e >> (8 * i));
    }
    return need;
}

/* ---------------- PHASE 0 (Core 0) ---------------- */
static void gbc_step_cpu_phase(void *core, uint32_t input_bits, int64_t rtc_epoch_sec)
{
    gbc_t *g = (gbc_t *)core;
    gbc_asnap_t *as = &g->as[g->back];
    uint32_t guard;

    if (!g->rom_loaded)
        return;
    gbc_rtc_advance(g, rtc_epoch_sec);
    gbc_joypad_update(g, input_bits);

    as->start = g->apu;
    as->count = 0;
    as->overflow = 0;
    g->apu_now = 0;
    g->apu_done = 0;
    gbc_ppu_begin_step(g);

    /* bounded: one frame is 70224 dots; hard cap at 3 frames */
    for (guard = 0; guard < 3u * GB_DOTS_PER_FRAME; guard++) {
        gbc_cpu_step(g);
        if (g->frame_done)
            break;
        if (!(g->io[0x40] & 0x80u) && g->step_dots >= GB_DOTS_PER_FRAME)
            break;
        if (g->step_dots >= 3u * GB_DOTS_PER_FRAME)
            break;
    }
    gbc_apu_catchup(g);
    as->cycles = g->apu_now;
    as->frame_no = g->frame_no;
    g->vs[g->back].frame_no = g->frame_no;
    g->frame_no++;
}

/* ---------------- BARRIER ---------------- */
static void gbc_commit_phase(void *core)
{
    gbc_t *g = (gbc_t *)core;
    g->front = g->back;
    g->back ^= 1u;
}

/* ---------------- PHASE 1 (Core 1) ---------------- */
static void gbc_render_video_phase(void *core, uint32_t *fb, size_t fb_stride, hs_video_mode_t mode)
{
    gbc_t *g = (gbc_t *)core;
    const gbc_vsnap_t *s = &g->vs[g->front];
    gbc_ppu_render_native(s, g->vid.vram, g->vid.oam, g->vid.native);
    if (mode == HS_SCALE_STRETCH_NEAREST) {
        hs_scaler_run(&g->vid.sc_near, g->vid.native, GB_W, fb, fb_stride, g->vid.row_tmp);
    } else if (mode == HS_AA_FXAA) {
        hs_scaler_run(&g->vid.sc_sharp, g->vid.native, GB_W, g->vid.fx_tmp, HS_FB_W, g->vid.row_tmp);
        hs_fxaa(g->vid.fx_tmp, HS_FB_W, fb, fb_stride, HS_FB_W, HS_FB_H, g->vid.luma);
    } else {
        hs_scaler_run(&g->vid.sc_sharp, g->vid.native, GB_W, fb, fb_stride, g->vid.row_tmp);
    }
}

/* ---------------- PHASE 2 (Core 2) ---------------- */
static uint32_t gbc_synth_audio_phase(void *core, int16_t *pcm, uint32_t max_frames)
{
    gbc_t *g = (gbc_t *)core;
    const gbc_asnap_t *as = &g->as[g->front];
    gbc_apu_t *a = &g->aud.apu;
    uint32_t produced = 0, t = 0, i, n = as->count;
    if (n > GBC_AEVT_MAX)
        n = GBC_AEVT_MAX;
    *a = as->start;
    for (i = 0; i < n; i++) {
        const gbc_aevt_t *e = &as->ev[i];
        if (e->cyc > t) {
            int16_t *o = (pcm && produced < max_frames) ? pcm + produced * 2u : NULL;
            uint32_t cap = produced < max_frames ? max_frames - produced : 0u;
            produced += gbc_apu_run(a, e->cyc - t, o, cap);
            t = e->cyc;
        }
        gbc_apu_write(a, e->reg, e->val);
    }
    if (as->cycles > t) {
        int16_t *o = (pcm && produced < max_frames) ? pcm + produced * 2u : NULL;
        uint32_t cap = produced < max_frames ? max_frames - produced : 0u;
        produced += gbc_apu_run(a, as->cycles - t, o, cap);
    }
    g->aud.produced = produced;
    return produced < max_frames ? produced : max_frames;
}

static void gbc_hash_state(const void *core, uint64_t *vh, uint64_t *ah)
{
    const gbc_t *g = (const gbc_t *)core;
    const gbc_vsnap_t *s = &g->vs[g->front];
    const gbc_asnap_t *a = &g->as[g->front];
    uint32_t vn = s->vlog_count < GBC_VLOG_MAX ? s->vlog_count : GBC_VLOG_MAX;
    uint32_t an = a->count < GBC_AEVT_MAX ? a->count : GBC_AEVT_MAX;
    uint64_t h = HS_FNV64_OFFSET;
    if (vh) {
        h = hs_fnv1a64(h, &s->frame_no, sizeof(s->frame_no));
        h = hs_fnv1a64(h, &s->cgb, 4);
        h = hs_fnv1a64(h, &s->vlog_count, sizeof(s->vlog_count));
        h = hs_fnv1a64(h, s->vram, sizeof(s->vram));
        h = hs_fnv1a64(h, s->oam, sizeof(s->oam));
        h = hs_fnv1a64(h, s->line, sizeof(s->line));
        h = hs_fnv1a64(h, s->vlog, vn * sizeof(gbc_vlog_t));
        *vh = h;
    }
    if (ah) {
        h = HS_FNV64_OFFSET;
        h = hs_fnv1a64(h, &a->frame_no, sizeof(a->frame_no));
        h = hs_fnv1a64(h, &a->cycles, sizeof(a->cycles));
        h = hs_fnv1a64(h, &a->count, sizeof(a->count));
        h = hs_fnv1a64(h, &a->start, sizeof(a->start));
        h = hs_fnv1a64(h, a->ev, an * sizeof(gbc_aevt_t));
        *ah = h;
    }
}

const hs_core_ops_t hs_gbc_core_ops = {
    HS_CORE_ABI_VERSION,
    HS_SYS_GBC,
    "hs-gbc (Game Boy / Game Boy Color, clean-room)",
    GB_W, GB_H,
    GB_CLOCK_HZ, GB_DOTS_PER_FRAME,
    gbc_state_size,
    gbc_init,
    gbc_reset,
    gbc_load_rom,
    gbc_load_sram,
    gbc_save_sram,
    gbc_step_cpu_phase,
    gbc_commit_phase,
    gbc_render_video_phase,
    gbc_synth_audio_phase,
    gbc_hash_state,
};
