/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_bus.c - clean-room Sega Genesis 68000 bus (Core 0 domain only).
 *
 *   000000-3FFFFF  cartridge ROM (SSF2 512 KB banking above 4 MB),
 *                  battery SRAM window from the header (default 200000)
 *   A00000-A0FFFF  Z80 space: 8 KB RAM (mirror kept in Core 0, writes are
 *                  forwarded to Core 2 as timestamped events), YM2612 ports
 *   A10000-A1001F  version register, 3/6-button pads, port control
 *   A11100 / A11200  Z80 BUSREQ / RESET
 *   A130F1-A130FF  SRAM enable / SSF2 bank registers
 *   A14000         TMSS (accepted, ignored)
 *   C00000-C0001F  VDP data / control / HV counter, PSG at C00011
 *   E00000-FFFFFF  64 KB work RAM (mirrored)
 */
#include "gen_internal.h"
#include <string.h>

void gen_log_event(gen_cpu_domain_t *d, uint8_t kind, uint8_t a, uint16_t v)
{
    gen_audio_snap_t *s = d->ev_out;
    uint64_t ts;
    if (!s) return;
    if (s->n_ev >= GEN_MAX_EVENTS) { s->dropped++; return; }
    ts = d->cpu.cyc - d->frame_start_cyc;
    s->ev[s->n_ev].ts = (uint32_t)ts;
    s->ev[s->n_ev].kind = kind;
    s->ev[s->n_ev].a = a;
    s->ev[s->n_ev].v = v;
    s->n_ev++;
}

/* ---- cartridge ---- */
static inline uint32_t rom_addr(const gen_cpu_domain_t *d, uint32_t a)
{
    if (d->ssf2) return ((uint32_t)d->bank[(a >> 19) & 7] << 19) | (a & 0x7FFFFu);
    return a;
}

static inline int sram_hit(const gen_cpu_domain_t *d, uint32_t a)
{
    return d->has_sram && d->sram_on && a >= d->sram_start && a <= d->sram_end;
}

static uint8_t cart_read8(const gen_cpu_domain_t *d, uint32_t a)
{
    uint32_t ra;
    if (sram_hit(d, a)) return d->sram[(a - d->sram_start) & 0xFFFFu];
    ra = rom_addr(d, a);
    return ra < d->rom_size ? d->rom[ra] : 0xFF;
}

/* ---- controllers ---- */
static uint8_t pad_read(gen_cpu_domain_t *d, int port)
{
    uint8_t ctrl = d->io_ctrl[port], data = d->io_data[port];
    uint8_t th = (ctrl & 0x40) ? (data & 0x40) : 0x40;
    uint8_t lines; /* active-low lines 0..5 */
    uint32_t p = d->pad;
    if (port != 0) return (uint8_t)((data & ctrl & 0x7F) | (0x7F & ~ctrl) | (data & 0x80));
    if (d->cpu.cyc - d->th_last_cyc > 11500u) d->th_phase = 0;
    if (th) {
        uint8_t pr;
        if (d->six_button && d->th_phase == 6) {
            pr = (uint8_t)(((p & HS_BTN_C) ? 0x20 : 0) | ((p & HS_GEN_B) ? 0x10 : 0) |
                           ((p & HS_BTN_MODE) ? 0x08 : 0) | ((p & HS_GEN_X) ? 0x04 : 0) |
                           ((p & HS_GEN_Y) ? 0x02 : 0) | ((p & HS_GEN_Z) ? 0x01 : 0));
        } else {
            pr = (uint8_t)(((p & HS_BTN_C) ? 0x20 : 0) | ((p & HS_GEN_B) ? 0x10 : 0) |
                           ((p & HS_BTN_RIGHT) ? 0x08 : 0) | ((p & HS_BTN_LEFT) ? 0x04 : 0) |
                           ((p & HS_BTN_DOWN) ? 0x02 : 0) | ((p & HS_BTN_UP) ? 0x01 : 0));
        }
        lines = (uint8_t)(0x40 | (~pr & 0x3F));
    } else {
        uint8_t pr = (uint8_t)(((p & HS_BTN_START) ? 0x20 : 0) | ((p & HS_GEN_A) ? 0x10 : 0));
        if (d->six_button && d->th_phase == 5) {
            lines = (uint8_t)(~pr & 0x30);                  /* low nibble all 0 */
        } else if (d->six_button && d->th_phase == 7) {
            lines = (uint8_t)((~pr & 0x30) | 0x0F);
        } else {
            pr = (uint8_t)(pr | ((p & HS_BTN_DOWN) ? 0x02 : 0) | ((p & HS_BTN_UP) ? 0x01 : 0));
            lines = (uint8_t)(~pr & 0x33);                   /* bits 2,3 read 0 */
        }
    }
    return (uint8_t)((data & ctrl & 0x7F) | (lines & ~ctrl & 0x7F) | (data & 0x80));
}

static void pad_write_data(gen_cpu_domain_t *d, int port, uint8_t v)
{
    uint8_t old_th = (d->io_ctrl[port] & 0x40) ? (d->io_data[port] & 0x40) : 0x40;
    uint8_t new_th;
    d->io_data[port] = v;
    new_th = (d->io_ctrl[port] & 0x40) ? (v & 0x40) : 0x40;
    if (port == 0 && old_th != new_th) {
        if (d->cpu.cyc - d->th_last_cyc > 11500u) d->th_phase = 0;
        d->th_phase = (uint8_t)((d->th_phase + 1) & 7);
        d->th_last_cyc = d->cpu.cyc;
    }
}

static uint8_t io_read8(gen_cpu_domain_t *d, uint32_t a)
{
    switch (a & 0x1F) {
    case 0x01:
        return (uint8_t)((d->japan ? 0x00 : 0x80) | (d->pal ? 0x40 : 0x00) | 0x20);
    case 0x03: return pad_read(d, 0);
    case 0x05: return pad_read(d, 1);
    case 0x07: return pad_read(d, 2);
    case 0x09: return d->io_ctrl[0];
    case 0x0B: return d->io_ctrl[1];
    case 0x0D: return d->io_ctrl[2];
    default: return 0;
    }
}

static void io_write8(gen_cpu_domain_t *d, uint32_t a, uint8_t v)
{
    switch (a & 0x1F) {
    case 0x03: pad_write_data(d, 0, v); break;
    case 0x05: pad_write_data(d, 1, v); break;
    case 0x07: pad_write_data(d, 2, v); break;
    case 0x09: d->io_ctrl[0] = v; break;
    case 0x0B: d->io_ctrl[1] = v; break;
    case 0x0D: d->io_ctrl[2] = v; break;
    default: break;
    }
}

/* ---- Z80 window seen from the 68k ---- */
static uint8_t z80win_read8(gen_cpu_domain_t *d, uint32_t a)
{
    uint32_t z = a & 0x7FFF;
    if (z < 0x4000) return d->zmirror[z & 0x1FFF];
    if (z < 0x6000) return 0;          /* YM status as seen by 68k: never busy */
    return 0xFF;
}

static void z80win_write8(gen_cpu_domain_t *d, uint32_t a, uint8_t v)
{
    uint32_t z = a & 0x7FFF;
    if (z < 0x4000) {
        d->zmirror[z & 0x1FFF] = v;
        gen_log_event(d, GEV_ZRAM, v, (uint16_t)(z & 0x1FFF));
    } else if (z < 0x6000) {
        gen_log_event(d, GEV_YM, (uint8_t)(z & 3), v);
    } else if (z == 0x7F11) {
        gen_log_event(d, GEV_PSG, 0, v);
    }
}

static void ctrl_write(gen_cpu_domain_t *d, uint32_t a, uint8_t v_hi_or_byte, int is_word, uint16_t w)
{
    uint32_t r = a & 0xFFFF00u;
    if (r == 0xA11100u) {
        uint8_t req = (uint8_t)((is_word ? (w >> 8) : v_hi_or_byte) & 1);
        if ((a & 1) && !is_word) return;
        if (req != d->z80_busreq) { d->z80_busreq = req; gen_log_event(d, GEV_BUSREQ, req, 0); }
    } else if (r == 0xA11200u) {
        uint8_t rel = (uint8_t)((is_word ? (w >> 8) : v_hi_or_byte) & 1);
        uint8_t rst = (uint8_t)!rel;
        if ((a & 1) && !is_word) return;
        if (rst != d->z80_reset) { d->z80_reset = rst; gen_log_event(d, GEV_RESET, rst, 0); }
    } else if (r == 0xA13000u) {
        uint8_t v = is_word ? (uint8_t)w : v_hi_or_byte;
        uint32_t lo = a & 0xFF;
        if (is_word) lo |= 1;
        if (lo == 0xF1) d->sram_on = (uint8_t)(v & 1);
        else if (lo >= 0xF3 && (lo & 1) && d->rom_size > 0x400000u) {
            d->bank[(lo - 0xF1) >> 1] = (uint8_t)(v & 0x3F);
            d->ssf2 = 1;
        }
    }
}

/* ---- public bus entry points ---- */
uint8_t gen_bus_read8(void *ctx, uint32_t a)
{
    gen_cpu_domain_t *d = (gen_cpu_domain_t *)ctx;
    a &= 0xFFFFFFu;
    if (a < 0x400000u) return cart_read8(d, a);
    if (a >= 0xE00000u) return d->ram[a & 0xFFFF];
    if (a >= 0xC00000u && a < 0xC00020u) {
        uint16_t w = gen_vdp_read(d, a & ~1u);
        return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
    }
    if (a >= 0xA00000u && a < 0xA10000u) return z80win_read8(d, a);
    if (a >= 0xA10000u && a < 0xA10020u) return io_read8(d, a | 1);
    if ((a & 0xFFFF00u) == 0xA11100u) return (uint8_t)((d->z80_busreq && !d->z80_reset) ? 0x00 : 0x01);
    return 0;
}

uint16_t gen_bus_read16(void *ctx, uint32_t a)
{
    gen_cpu_domain_t *d = (gen_cpu_domain_t *)ctx;
    a &= 0xFFFFFEu;
    if (a < 0x400000u) {
        if (sram_hit(d, a)) return (uint16_t)((d->sram[(a - d->sram_start) & 0xFFFF] << 8) |
                                              d->sram[(a + 1 - d->sram_start) & 0xFFFF]);
        {
            uint32_t ra = rom_addr(d, a);
            if (ra + 1 < d->rom_size) return (uint16_t)((d->rom[ra] << 8) | d->rom[ra + 1]);
            return 0xFFFF;
        }
    }
    if (a >= 0xE00000u) return (uint16_t)((d->ram[a & 0xFFFF] << 8) | d->ram[(a + 1) & 0xFFFF]);
    if (a >= 0xC00000u && a < 0xC00020u) return gen_vdp_read(d, a);
    if (a >= 0xA00000u && a < 0xA10000u) { uint8_t b = z80win_read8(d, a); return (uint16_t)((b << 8) | b); }
    if (a >= 0xA10000u && a < 0xA10020u) { uint8_t b = io_read8(d, a | 1); return (uint16_t)((b << 8) | b); }
    if ((a & 0xFFFF00u) == 0xA11100u) return (uint16_t)((d->z80_busreq && !d->z80_reset) ? 0x0000 : 0x0100);
    return 0;
}

void gen_bus_write8(void *ctx, uint32_t a, uint8_t v)
{
    gen_cpu_domain_t *d = (gen_cpu_domain_t *)ctx;
    a &= 0xFFFFFFu;
    if (a >= 0xE00000u) { d->ram[a & 0xFFFF] = v; return; }
    if (a < 0x400000u) {
        if (sram_hit(d, a)) { d->sram[(a - d->sram_start) & 0xFFFF] = v; d->sram_dirty = 1; }
        return;
    }
    if (a >= 0xC00000u && a < 0xC00020u) {
        if ((a & 0x1F) >= 0x10 && (a & 0x1F) < 0x18) { if (a & 1) gen_log_event(d, GEV_PSG, 0, v); return; }
        gen_vdp_write(d, a & ~1u, (uint16_t)((v << 8) | v));
        return;
    }
    if (a >= 0xA00000u && a < 0xA10000u) { z80win_write8(d, a, v); return; }
    if (a >= 0xA10000u && a < 0xA10020u) { io_write8(d, a, v); return; }
    if (a >= 0xA11000u && a < 0xA14000u) { ctrl_write(d, a, v, 0, 0); return; }
}

void gen_bus_write16(void *ctx, uint32_t a, uint16_t v)
{
    gen_cpu_domain_t *d = (gen_cpu_domain_t *)ctx;
    a &= 0xFFFFFEu;
    if (a >= 0xE00000u) { d->ram[a & 0xFFFF] = (uint8_t)(v >> 8); d->ram[(a + 1) & 0xFFFF] = (uint8_t)v; return; }
    if (a < 0x400000u) {
        if (sram_hit(d, a)) {
            d->sram[(a - d->sram_start) & 0xFFFF] = (uint8_t)(v >> 8);
            d->sram[(a + 1 - d->sram_start) & 0xFFFF] = (uint8_t)v;
            d->sram_dirty = 1;
        }
        return;
    }
    if (a >= 0xC00000u && a < 0xC00020u) {
        if ((a & 0x1F) >= 0x10 && (a & 0x1F) < 0x18) { gen_log_event(d, GEV_PSG, 0, (uint8_t)v); return; }
        gen_vdp_write(d, a, v);
        return;
    }
    if (a >= 0xA00000u && a < 0xA10000u) { z80win_write8(d, a, (uint8_t)(v >> 8)); return; }
    if (a >= 0xA10000u && a < 0xA10020u) { io_write8(d, a | 1, (uint8_t)v); return; }
    if (a >= 0xA11000u && a < 0xA14000u) { ctrl_write(d, a, (uint8_t)(v >> 8), 1, v); return; }
}

void gen_bus_reset(hs_gen_t *g)
{
    gen_cpu_domain_t *d = &g->cpu;
    int i;
    memset(d->ram, 0, sizeof(d->ram));
    memset(d->zmirror, 0, sizeof(d->zmirror));
    for (i = 0; i < 8; i++) d->bank[i] = (uint8_t)i;
    d->ssf2 = 0;
    d->z80_busreq = 0;
    d->z80_reset = 1;
    for (i = 0; i < 3; i++) { d->io_data[i] = 0x7F; d->io_ctrl[i] = 0; }
    d->th_phase = 0;
    d->th_last_cyc = 0;
    d->sram_on = d->has_sram && d->rom_size <= d->sram_start;
    gen_vdp_reset(d);
}

void gen_bus_init(hs_gen_t *g)
{
    m68k_bus_t b;
    b.read8 = gen_bus_read8;
    b.read16 = gen_bus_read16;
    b.write8 = gen_bus_write8;
    b.write16 = gen_bus_write16;
    b.iack = gen_vdp_iack;
    b.ctx = &g->cpu;
    m68k_init(&g->cpu.cpu, &b);
}
