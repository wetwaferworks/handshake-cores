/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_genesis.c - hs-genesis core: HandshakeOS 3-phase ops table, frame
 * scheduler (262/313 lines of 3420 master clocks, 68k = master/7) and the
 * BACK/FRONT snapshot double buffer.
 */
#include "gen_internal.h"
#include <string.h>

static size_t gen_state_size(void) { return sizeof(hs_gen_t); }

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static int gen_init(void *mem, size_t mem_size, const hs_core_config_t *cfg)
{
    hs_gen_t *g = (hs_gen_t *)mem;
    uint32_t flags = cfg ? cfg->flags : 0;
    if (!mem || mem_size < sizeof(hs_gen_t)) return -1;
    if (cfg && cfg->console_id != HS_SYS_GENESIS) return -2;
    memset(g, 0, sizeof(*g));
    g->magic = GEN_MAGIC;
    g->config_flags = flags;
    g->cpu.pal = (flags & HS_GEN_CFG_PAL) ? 1 : 0;
    g->cpu.six_button = (flags & HS_GEN_CFG_3BUTTON) ? 0 : 1;
    g->cpu.japan = (flags & HS_GEN_CFG_JAPAN) ? 1 : 0;
    gen_bus_init(g);
    gen_render_init(&g->vid);
    gen_audio_init(&g->aud);
    g->back = 0;
    return 0;
}

static void gen_reset(void *core)
{
    hs_gen_t *g = (hs_gen_t *)core;
    if (!g || g->magic != GEN_MAGIC) return;
    gen_bus_reset(g);
    memset(g->slot, 0, sizeof(g->slot));
    g->back = 0;
    g->cpu.frame = 0;
    g->cpu.master_rem = 0;
    g->cpu.ev_out = NULL;
    g->cpu.cpu.cyc = 0;
    if (g->cpu.rom) m68k_reset(&g->cpu.cpu);
    gen_audio_reset(&g->aud);
    g->aud.z80_base = g->aud.z80.cyc;
}

static int gen_load_rom(void *core, const uint8_t *rom, size_t len)
{
    hs_gen_t *g = (hs_gen_t *)core;
    gen_cpu_domain_t *d;
    if (!g || g->magic != GEN_MAGIC || !rom || len < 0x200 || len > 0x1000000u) return -1;
    d = &g->cpu;
    d->rom = rom;
    d->rom_size = (uint32_t)len;
    g->aud.rom = rom;
    g->aud.rom_size = (uint32_t)len;
    d->has_sram = 1;
    d->sram_start = 0x200000u;
    d->sram_end = 0x20FFFFu;
    if (rom[0x1B0] == 'R' && rom[0x1B1] == 'A') {
        uint32_t s = be32(rom + 0x1B4) & 0xFFFFFFu, e = be32(rom + 0x1B8) & 0xFFFFFFu;
        if (e >= s && s < 0x400000u) {
            if (e - s > 0xFFFFu) e = s + 0xFFFFu;
            d->sram_start = s;
            d->sram_end = e;
        }
    }
    memset(d->sram, 0xFF, sizeof(d->sram));
    d->sram_dirty = 0;
    gen_reset(core);
    return 0;
}

static int gen_load_sram(void *core, const uint8_t *buf, size_t len)
{
    hs_gen_t *g = (hs_gen_t *)core;
    size_t n;
    if (!g || g->magic != GEN_MAGIC || !buf) return -1;
    n = (size_t)(g->cpu.sram_end - g->cpu.sram_start + 1u);
    if (len < n) n = len;
    memcpy(g->cpu.sram, buf, n);
    return 0;
}

static size_t gen_save_sram(const void *core, uint8_t *buf, size_t cap)
{
    const hs_gen_t *g = (const hs_gen_t *)core;
    size_t n;
    if (!g || g->magic != GEN_MAGIC || !g->cpu.has_sram) return 0;
    n = (size_t)(g->cpu.sram_end - g->cpu.sram_start + 1u);
    if (!buf) return n;
    if (n > cap) n = cap;
    memcpy(buf, g->cpu.sram, n);
    return n;
}

/* PHASE 0 (Core 0) */
static void gen_step_cpu_phase(void *core, uint32_t input_bits, int64_t rtc_epoch_sec)
{
    hs_gen_t *g = (hs_gen_t *)core;
    gen_cpu_domain_t *d = &g->cpu;
    gen_vdp_t *v = &d->vdp;
    gen_slot_t *slot;
    uint32_t lines, active, line;
    (void)rtc_epoch_sec;
    if (!g || g->magic != GEN_MAGIC || !d->rom) return;
    slot = &g->slot[g->back & 1];
    slot->a.n_ev = 0;
    slot->a.dropped = 0;
    slot->a.pal = d->pal;
    slot->a.frame = d->frame;
    d->ev_out = &slot->a;
    d->pad = input_bits;

    lines = d->pal ? GEN_LINES_PAL : GEN_LINES_NTSC;
    active = (d->pal && (v->reg[1] & 0x08)) ? 240u : 224u;
    memcpy(slot->v.vram, v->vram, sizeof(v->vram));
    slot->v.active_h = active;
    slot->v.frame = d->frame;
    d->frame_start_cyc = d->cpu.cyc;

    for (line = 0; line < lines; line++) {
        uint32_t m = d->master_rem + GEN_MASTER_LINE;
        uint32_t c68 = m / 7u;
        d->master_rem = m % 7u;
        v->line = line;
        v->line_start_cyc = d->cpu.cyc;
        if (line == 0) v->vblank = 0;
        if (line < active) {
            gen_line_snap_t *ls = &slot->v.line[line];
            memcpy(ls->reg, v->reg, sizeof(ls->reg));
            memcpy(ls->cram, v->cram, sizeof(ls->cram));
            memcpy(ls->vsram, v->vsram, sizeof(ls->vsram));
        } else if (line == active) {
            v->vblank = 1;
            v->vint_pending = 1;
            gen_vdp_update_irq(d);
            gen_log_event(d, GEV_ZIRQ, 1, 0);
        } else if (line == active + 1) {
            gen_log_event(d, GEV_ZIRQ, 0, 0);
        }
        m68k_run(&d->cpu, c68 - GEN_HBLANK_68K);
        v->hblank = 1;
        if (line < active) {
            if (--v->hint_counter < 0) {
                v->hint_counter = v->reg[10];
                v->hint_pending = 1;
                gen_vdp_update_irq(d);
            }
        } else {
            v->hint_counter = v->reg[10];
        }
        {
            uint64_t end = v->line_start_cyc + c68;
            if (d->cpu.cyc < end) m68k_run(&d->cpu, (uint32_t)(end - d->cpu.cyc));
        }
        v->hblank = 0;
    }
    slot->a.frame_68k = (uint32_t)(d->cpu.cyc - d->frame_start_cyc);
    slot->a.frame_z80 = (GEN_MASTER_LINE * lines) / 15u;
    d->ev_out = NULL;
    d->frame++;
}

/* BARRIER (single thread, all phases quiescent) */
static void gen_commit_phase(void *core)
{
    hs_gen_t *g = (hs_gen_t *)core;
    const gen_audio_snap_t *a;
    uint32_t i, n;
    if (!g || g->magic != GEN_MAGIC) return;
    g->back ^= 1u;
    a = &g->slot[g->back ^ 1u].a;
    /* 68k view of Z80 RAM = Core 2 RAM as of its last frame + the 68k writes
     * just published (not yet replayed by Core 2). */
    memcpy(g->cpu.zmirror, g->aud.zram, sizeof(g->cpu.zmirror));
    n = a->n_ev < GEN_MAX_EVENTS ? a->n_ev : GEN_MAX_EVENTS;
    for (i = 0; i < n; i++)
        if (a->ev[i].kind == GEV_ZRAM) g->cpu.zmirror[a->ev[i].v & 0x1FFF] = a->ev[i].a;
}

/* PHASE 1 (Core 1) */
static void gen_render_video_phase(void *core, uint32_t *fb, size_t stride, hs_video_mode_t mode)
{
    hs_gen_t *g = (hs_gen_t *)core;
    if (!g || g->magic != GEN_MAGIC || !fb) return;
    gen_render_frame(&g->vid, &g->slot[g->back ^ 1u].v, fb, stride, mode);
}

/* PHASE 2 (Core 2) */
static uint32_t gen_synth_audio_phase(void *core, int16_t *pcm, uint32_t max_frames)
{
    hs_gen_t *g = (hs_gen_t *)core;
    if (!g || g->magic != GEN_MAGIC) return 0;
    return gen_audio_frame(&g->aud, &g->slot[g->back ^ 1u].a, pcm, max_frames);
}

static void gen_hash_state(const void *core, uint64_t *vh, uint64_t *ah)
{
    const hs_gen_t *g = (const hs_gen_t *)core;
    const gen_slot_t *f;
    uint64_t h;
    uint32_t n, lines;
    if (!g || g->magic != GEN_MAGIC) return;
    f = &g->slot[g->back ^ 1u];
    lines = f->v.active_h <= GEN_MAX_LINES ? f->v.active_h : GEN_MAX_LINES;
    h = hs_fnv1a64(HS_FNV64_OFFSET, f->v.vram, sizeof(f->v.vram));
    h = hs_fnv1a64(h, f->v.line, sizeof(gen_line_snap_t) * lines);
    if (vh) *vh = h;
    n = f->a.n_ev < GEN_MAX_EVENTS ? f->a.n_ev : GEN_MAX_EVENTS;
    h = hs_fnv1a64(HS_FNV64_OFFSET, &n, sizeof(n));
    h = hs_fnv1a64(h, f->a.ev, sizeof(gen_audio_ev_t) * n);
    if (ah) *ah = h;
}

const hs_core_ops_t hs_genesis_core_ops = {
    HS_CORE_ABI_VERSION,
    HS_SYS_GENESIS,
    "hs-genesis",
    320, 224,
    GEN_MASTER_NTSC, GEN_MASTER_LINE * GEN_LINES_NTSC,
    gen_state_size,
    gen_init,
    gen_reset,
    gen_load_rom,
    gen_load_sram,
    gen_save_sram,
    gen_step_cpu_phase,
    gen_commit_phase,
    gen_render_video_phase,
    gen_synth_audio_phase,
    gen_hash_state
};
