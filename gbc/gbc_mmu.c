/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_mmu.c - clean-room DMG/CGB memory map, mappers (ROM-only, MBC1,
 * MBC2, MBC3 + injected deterministic RTC, MBC5), timer, serial, joypad,
 * OAM DMA, CGB GDMA/HDMA, and the M-cycle clock (gbc_tick).
 */
#include "gbc_internal.h"

#include <string.h>

/* ---------------------------------------------------------------- mapper */
static void mbc_update(gbc_t *g)
{
    uint32_t banks = g->rom_banks ? g->rom_banks : 1u;
    uint32_t romx = 1, rom0 = 0, rb = 0;
    switch (g->mbc) {
    case MBC_1: {
        uint32_t lo = g->mbc1_lo & 0x1Fu;
        if (lo == 0) lo = 1;
        romx = ((uint32_t)g->mbc1_hi << 5) | lo;
        if (g->mbc1_mode) {
            rom0 = (uint32_t)g->mbc1_hi << 5;
            rb = g->mbc1_hi;
        }
    } break;
    case MBC_2:
        romx = g->rom_bank & 0x0Fu;
        if (romx == 0) romx = 1;
        break;
    case MBC_3:
        romx = g->rom_bank & 0x7Fu;
        if (romx == 0) romx = 1;
        rb = g->ram_bank & 0x03u;
        break;
    case MBC_5:
        romx = (uint32_t)g->rom_bank | ((uint32_t)(g->rom_bank_hi & 1u) << 8);
        rb = g->ram_bank & 0x0Fu;
        break;
    default:
        romx = 1;
        break;
    }
    g->romx_off = (romx % banks) * 0x4000u;
    g->rom0_off = (rom0 % banks) * 0x4000u;
    g->eram_off = g->ram_size ? ((rb * 0x2000u) % g->ram_size) : 0u;
}

static void rtc_latch(gbc_t *g)
{
    gbc_rtc_t *r = &g->rtc;
    r->lsec = r->sec; r->lmin = r->min; r->lhour = r->hour; r->ldl = r->dl; r->ldh = r->dh;
}

void gbc_rtc_advance(gbc_t *g, int64_t epoch)
{
    gbc_rtc_t *r = &g->rtc;
    int64_t delta;
    uint64_t total, days;
    if (!g->has_rtc)
        return;
    if (!r->epoch_valid) {
        r->last_epoch = epoch;
        r->epoch_valid = 1;
        return;
    }
    delta = epoch - r->last_epoch;
    r->last_epoch = epoch;
    if (delta <= 0 || (r->dh & 0x40u))
        return;
    total = (uint64_t)(r->sec % 60u) + (uint64_t)delta;
    r->sec = (uint8_t)(total % 60u);
    total = total / 60u + (r->min % 60u);
    r->min = (uint8_t)(total % 60u);
    total = total / 60u + (r->hour % 24u);
    r->hour = (uint8_t)(total % 24u);
    days = total / 24u + (uint64_t)r->dl + ((uint64_t)(r->dh & 1u) << 8);
    if (days >= 512u) {
        r->dh |= 0x80u;
        days %= 512u;
    }
    r->dl = (uint8_t)days;
    r->dh = (uint8_t)((r->dh & 0xFEu) | ((days >> 8) & 1u));
}

static void mbc_write(gbc_t *g, uint16_t a, uint8_t v)
{
    switch (g->mbc) {
    case MBC_1:
        if (a < 0x2000) g->ram_enable = (uint8_t)((v & 0x0Fu) == 0x0Au);
        else if (a < 0x4000) g->mbc1_lo = v & 0x1Fu;
        else if (a < 0x6000) g->mbc1_hi = v & 0x03u;
        else g->mbc1_mode = v & 0x01u;
        break;
    case MBC_2:
        if (a < 0x4000) {
            if (a & 0x0100u) g->rom_bank = v & 0x0Fu;
            else g->ram_enable = (uint8_t)((v & 0x0Fu) == 0x0Au);
        }
        break;
    case MBC_3:
        if (a < 0x2000) g->ram_enable = (uint8_t)((v & 0x0Fu) == 0x0Au);
        else if (a < 0x4000) g->rom_bank = v & 0x7Fu;
        else if (a < 0x6000) g->ram_bank = v;
        else {
            if (g->rtc.latch_prev == 0 && v == 1) rtc_latch(g);
            g->rtc.latch_prev = v;
        }
        break;
    case MBC_5:
        if (a < 0x2000) g->ram_enable = (uint8_t)((v & 0x0Fu) == 0x0Au);
        else if (a < 0x3000) g->rom_bank = v;
        else if (a < 0x4000) g->rom_bank_hi = v & 1u;
        else if (a < 0x6000) g->ram_bank = v & 0x0Fu;
        break;
    default:
        break;
    }
    mbc_update(g);
}

static uint8_t eram_read(gbc_t *g, uint16_t a)
{
    if (!g->ram_enable)
        return 0xFF;
    if (g->mbc == MBC_2)
        return (uint8_t)(0xF0u | (g->eram[a & 0x1FFu] & 0x0Fu));
    if (g->mbc == MBC_3 && g->ram_bank >= 0x08 && g->ram_bank <= 0x0C) {
        switch (g->ram_bank) {
        case 0x08: return g->rtc.lsec;
        case 0x09: return g->rtc.lmin;
        case 0x0A: return g->rtc.lhour;
        case 0x0B: return g->rtc.ldl;
        default:   return g->rtc.ldh;
        }
    }
    if (!g->ram_size)
        return 0xFF;
    return g->eram[(g->eram_off + (a & 0x1FFFu)) % g->ram_size];
}

static void eram_write(gbc_t *g, uint16_t a, uint8_t v)
{
    if (!g->ram_enable)
        return;
    if (g->mbc == MBC_2) {
        g->eram[a & 0x1FFu] = (uint8_t)(v & 0x0Fu);
        return;
    }
    if (g->mbc == MBC_3 && g->ram_bank >= 0x08 && g->ram_bank <= 0x0C) {
        switch (g->ram_bank) {
        case 0x08: g->rtc.sec = v & 0x3Fu; break;
        case 0x09: g->rtc.min = v & 0x3Fu; break;
        case 0x0A: g->rtc.hour = v & 0x1Fu; break;
        case 0x0B: g->rtc.dl = v; break;
        default:   g->rtc.dh = v & 0xC1u; break;
        }
        return;
    }
    if (!g->ram_size)
        return;
    g->eram[(g->eram_off + (a & 0x1FFFu)) % g->ram_size] = v;
}

int gbc_mmu_load_rom(gbc_t *g, const uint8_t *rom, size_t len)
{
    uint8_t type, rs;
    if (!rom || len < 0x8000u || len > (8u << 20))
        return -1;
    g->rom = rom;
    g->rom_size = (uint32_t)len;
    g->rom_banks = (uint32_t)(len / 0x4000u);
    type = rom[0x147];
    rs = rom[0x149];
    g->has_battery = 0; g->has_rtc = 0;
    switch (type) {
    case 0x00: case 0x08: case 0x09: g->mbc = MBC_NONE; g->has_battery = (type == 0x09); break;
    case 0x01: case 0x02: case 0x03: g->mbc = MBC_1; g->has_battery = (type == 0x03); break;
    case 0x05: case 0x06: g->mbc = MBC_2; g->has_battery = (type == 0x06); break;
    case 0x0F: case 0x10: case 0x11: case 0x12: case 0x13:
        g->mbc = MBC_3;
        g->has_battery = (type == 0x0F || type == 0x10 || type == 0x13);
        g->has_rtc = (type == 0x0F || type == 0x10);
        break;
    case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E:
        g->mbc = MBC_5; g->has_battery = (type == 0x1B || type == 0x1E); break;
    default:
        return -2; /* unsupported mapper */
    }
    switch (rs) {
    case 0x02: g->ram_size = 0x2000u; break;
    case 0x03: g->ram_size = 0x8000u; break;
    case 0x04: g->ram_size = 0x20000u; break;
    case 0x05: g->ram_size = 0x10000u; break;
    default:   g->ram_size = 0; break;
    }
    if (g->mbc == MBC_2) g->ram_size = 0x200u;
    if (g->mbc == MBC_NONE && (type == 0x08 || type == 0x09) && g->ram_size == 0) g->ram_size = 0x2000u;
    g->rom_loaded = 1;
    return 0;
}

/* ---------------------------------------------------------------- reset */
void gbc_mmu_reset(gbc_t *g)
{
    memset(g->vram, 0, sizeof(g->vram));
    memset(g->wram, 0, sizeof(g->wram));
    memset(g->oam, 0, sizeof(g->oam));
    memset(g->hram, 0, sizeof(g->hram));
    memset(g->io, 0xFF, sizeof(g->io));
    g->ie = 0;
    g->vbk = 0; g->svbk = 1; g->bcps = 0; g->ocps = 0; g->opri = g->cgb ? 0 : 1;
    memset(g->bgpal, 0xFF, sizeof(g->bgpal));
    memset(g->objpal, 0xFF, sizeof(g->objpal));
    g->ram_enable = 0; g->rom_bank = 1; g->rom_bank_hi = 0; g->ram_bank = 0;
    g->mbc1_hi = 0; g->mbc1_mode = 0; g->mbc1_lo = 1;
    g->double_speed = 0; g->key1 = 0;
    g->div = g->cgb ? 0x1EA0u : 0xABCCu;
    g->tima_reload = 0;
    g->serial_active = 0; g->serial_cycles = 0; g->serial_len = 0;
    g->input = 0; g->p1_sel = 0x30;
    g->dma_active = 0; g->dma_delay = 0; g->dma_src = 0; g->dma_pos = 0;
    g->hdma_src = 0; g->hdma_dst = 0x8000; g->hdma_active = 0; g->hdma_remain = 0x7F;
    g->hdma_block_pending = 0;
    /* post-boot IO */
    g->io[0x00] = 0xCF; g->io[0x01] = 0x00; g->io[0x02] = 0x7E;
    g->io[0x05] = 0x00; g->io[0x06] = 0x00; g->io[0x07] = 0xF8;
    g->io[0x0F] = 0xE1;
    g->io[0x40] = 0x91; g->io[0x41] = 0x00; g->io[0x42] = 0; g->io[0x43] = 0;
    g->io[0x45] = 0; g->io[0x46] = 0xFF; g->io[0x47] = 0xFC; g->io[0x48] = 0xFF;
    g->io[0x49] = 0xFF; g->io[0x4A] = 0; g->io[0x4B] = 0;
    memset(&g->rtc, 0, sizeof(g->rtc));
    mbc_update(g);
}

/* ---------------------------------------------------------------- timer */
static const uint16_t tac_bit[4] = { 0x200u, 0x008u, 0x020u, 0x080u };

static void tima_inc(gbc_t *g)
{
    if (g->io[0x05] == 0xFFu) {
        g->io[0x05] = 0x00;
        g->tima_reload = 1;
    } else {
        g->io[0x05]++;
    }
}

static inline int timer_signal(const gbc_t *g, uint16_t div, uint8_t tac)
{
    (void)g;
    return (tac & 4u) && (div & tac_bit[tac & 3u]);
}

/* ---------------------------------------------------------------- joypad */
static uint8_t p1_lines(const gbc_t *g)
{
    uint8_t low = 0x0F;
    uint32_t in = g->input;
    if (!(g->p1_sel & 0x10u)) {
        if (in & HS_BTN_RIGHT) low &= (uint8_t)~0x01u;
        if (in & HS_BTN_LEFT)  low &= (uint8_t)~0x02u;
        if (in & HS_BTN_UP)    low &= (uint8_t)~0x04u;
        if (in & HS_BTN_DOWN)  low &= (uint8_t)~0x08u;
    }
    if (!(g->p1_sel & 0x20u)) {
        if (in & HS_BTN_A)      low &= (uint8_t)~0x01u;
        if (in & HS_BTN_B)      low &= (uint8_t)~0x02u;
        if (in & HS_BTN_SELECT) low &= (uint8_t)~0x04u;
        if (in & HS_BTN_START)  low &= (uint8_t)~0x08u;
    }
    return low;
}

void gbc_joypad_update(gbc_t *g, uint32_t input)
{
    uint8_t before = p1_lines(g), after;
    const uint32_t gb_mask = HS_BTN_UP | HS_BTN_DOWN | HS_BTN_LEFT | HS_BTN_RIGHT |
                             HS_BTN_A | HS_BTN_B | HS_BTN_START | HS_BTN_SELECT;
    g->input = input & gb_mask;
    after = p1_lines(g);
    if (before & (uint8_t)~after)
        g->io[0x0F] |= IRQ_JOYPAD;
    if (g->cpu.stopped && g->input)
        g->cpu.stopped = 0;
}

/* ---------------------------------------------------------------- VRAM */
static inline void vram_store(gbc_t *g, uint16_t vaddr, uint8_t v)
{
    vaddr &= 0x3FFFu;
    if (g->vram[vaddr] != v) {
        gbc_ppu_log_vram(g, vaddr, v);
        g->vram[vaddr] = v;
    }
}

static inline void oam_store(gbc_t *g, uint16_t idx, uint8_t v)
{
    if (g->oam[idx] != v) {
        gbc_ppu_log_oam(g, idx, v);
        g->oam[idx] = v;
    }
}

/* ---------------------------------------------------------------- HDMA */
static void hdma_copy_block(gbc_t *g)
{
    unsigned i;
    for (i = 0; i < 16u; i++) {
        uint8_t v = gbc_read(g, g->hdma_src);
        uint16_t dst = (uint16_t)(g->hdma_dst & 0x1FFFu);
        vram_store(g, (uint16_t)(g->vbk * 0x2000u + dst), v);
        g->hdma_src++;
        g->hdma_dst = (uint16_t)(0x8000u | ((g->hdma_dst + 1u) & 0x1FFFu));
    }
}

static void hdma_stall(gbc_t *g)
{
    unsigned i, n = g->double_speed ? 16u : 8u;
    for (i = 0; i < n; i++)
        gbc_tick(g);
}

void gbc_hdma_service(gbc_t *g)
{
    g->hdma_block_pending = 0;
    if (!g->hdma_active)
        return;
    hdma_copy_block(g);
    hdma_stall(g);
    if (g->hdma_remain == 0) {
        g->hdma_active = 0;
        g->hdma_remain = 0x7F;
    } else {
        g->hdma_remain--;
    }
}

static void hdma5_write(gbc_t *g, uint8_t v)
{
    if (g->hdma_active && !(v & 0x80u)) {
        g->hdma_active = 0;            /* cancel: HDMA5 reads 0x80|remain */
        return;
    }
    if (v & 0x80u) {
        g->hdma_active = 1;
        g->hdma_remain = v & 0x7Fu;
        /* started during HBlank or with the LCD off: first block now */
        if (!(g->io[0x40] & 0x80u) || (g->mode == 0 && g->ly < GB_H))
            g->hdma_block_pending = 1;
    } else {
        unsigned n = (unsigned)(v & 0x7Fu) + 1u, i;
        for (i = 0; i < n; i++) {
            hdma_copy_block(g);
            hdma_stall(g);
        }
        g->hdma_remain = 0x7F;
    }
}

/* ---------------------------------------------------------------- APU glue */
void gbc_apu_catchup(gbc_t *g)
{
    if (g->apu_now > g->apu_done) {
        gbc_apu_run(&g->apu, g->apu_now - g->apu_done, NULL, 0);
        g->apu_done = g->apu_now;
    }
}

static void apu_reg_write(gbc_t *g, uint32_t reg, uint8_t v)
{
    gbc_asnap_t *as = &g->as[g->back];
    gbc_apu_catchup(g);
    gbc_apu_write(&g->apu, reg, v);
    if (as->count < GBC_AEVT_MAX) {
        gbc_aevt_t *e = &as->ev[as->count++];
        e->cyc = g->apu_now;
        e->reg = (uint8_t)reg;
        e->val = v;
        e->_pad = 0;
    } else {
        as->overflow++;
    }
}

/* ---------------------------------------------------------------- IO */
static uint8_t io_read(gbc_t *g, uint8_t r)
{
    switch (r) {
    case 0x00: return (uint8_t)(0xC0u | g->p1_sel | p1_lines(g));
    case 0x01: return g->io[0x01];
    case 0x02: return (uint8_t)(g->io[0x02] | (g->cgb ? 0x7Cu : 0x7Eu));
    case 0x04: return (uint8_t)(g->div >> 8);
    case 0x05: return g->io[0x05];
    case 0x06: return g->io[0x06];
    case 0x07: return (uint8_t)(g->io[0x07] | 0xF8u);
    case 0x0F: return (uint8_t)(g->io[0x0F] | 0xE0u);
    case 0x40: return g->io[0x40];
    case 0x41: {
        uint8_t m = (g->io[0x40] & 0x80u) ? g->mode : 0;
        return (uint8_t)(0x80u | (g->io[0x41] & 0x78u) | ((g->ly == g->io[0x45]) ? 4u : 0u) | m);
    }
    case 0x44: return g->ly;
    case 0x4D: return g->cgb ? (uint8_t)(0x7Eu | g->key1) : 0xFF;
    case 0x4F: return g->cgb ? (uint8_t)(0xFEu | g->vbk) : 0xFF;
    case 0x55:
        if (!g->cgb) return 0xFF;
        return g->hdma_active ? g->hdma_remain : (uint8_t)(0x80u | g->hdma_remain);
    case 0x68: return g->cgb ? (uint8_t)(0x40u | g->bcps) : 0xFF;
    case 0x69: return g->cgb ? g->bgpal[g->bcps & 0x3Fu] : 0xFF;
    case 0x6A: return g->cgb ? (uint8_t)(0x40u | g->ocps) : 0xFF;
    case 0x6B: return g->cgb ? g->objpal[g->ocps & 0x3Fu] : 0xFF;
    case 0x6C: return g->cgb ? (uint8_t)(0xFEu | g->opri) : 0xFF;
    case 0x70: return g->cgb ? (uint8_t)(0xF8u | g->svbk) : 0xFF;
    default: break;
    }
    if (r >= 0x10 && r < 0x40) {
        gbc_apu_catchup(g);
        return gbc_apu_read(&g->apu, (uint32_t)r - 0x10u);
    }
    if (r >= 0x42 && r <= 0x4B)
        return g->io[r];
    return 0xFF;
}

static void io_write(gbc_t *g, uint8_t r, uint8_t v)
{
    if (r >= 0x10 && r < 0x40) {
        apu_reg_write(g, (uint32_t)r - 0x10u, v);
        return;
    }
    switch (r) {
    case 0x00: {
        uint8_t before = p1_lines(g);
        g->p1_sel = v & 0x30u;
        if (before & (uint8_t)~p1_lines(g)) g->io[0x0F] |= IRQ_JOYPAD;
    } break;
    case 0x01: g->io[0x01] = v; break;
    case 0x02:
        g->io[0x02] = v & 0x83u;
        if ((v & 0x81u) == 0x81u) {
            g->serial_active = 1;
            g->serial_cycles = 0;
            if (g->serial_len < GBC_SERIAL_LOG)
                g->serial_log[g->serial_len++] = g->io[0x01];
        }
        break;
    case 0x04:
        if (timer_signal(g, g->div, g->io[0x07])) tima_inc(g);
        g->div = 0;
        break;
    case 0x05: g->io[0x05] = v; g->tima_reload = 0; break;
    case 0x06: g->io[0x06] = v; break;
    case 0x07: {
        int before = timer_signal(g, g->div, g->io[0x07]);
        g->io[0x07] = v & 0x07u;
        if (before && !timer_signal(g, g->div, g->io[0x07])) tima_inc(g);
    } break;
    case 0x0F: g->io[0x0F] = v & 0x1Fu; break;
    case 0x40: gbc_ppu_lcdc_write(g, v); break;
    case 0x41: g->io[0x41] = v & 0x78u; gbc_ppu_stat_update(g); break;
    case 0x44: break;
    case 0x45: g->io[0x45] = v; gbc_ppu_stat_update(g); break;
    case 0x46:
        g->io[0x46] = v;
        g->dma_src = (uint16_t)(v << 8);
        g->dma_pos = 0;
        g->dma_active = 1;
        g->dma_delay = 1;
        break;
    case 0x42: case 0x43: case 0x47: case 0x48: case 0x49: case 0x4A: case 0x4B:
        g->io[r] = v; break;
    case 0x4D: if (g->cgb) g->key1 = (uint8_t)((g->key1 & 0x80u) | (v & 1u)); break;
    case 0x4F: if (g->cgb) g->vbk = v & 1u; break;
    case 0x51: if (g->cgb) g->hdma_src = (uint16_t)((g->hdma_src & 0x00F0u) | (v << 8)); break;
    case 0x52: if (g->cgb) g->hdma_src = (uint16_t)((g->hdma_src & 0xFF00u) | (v & 0xF0u)); break;
    case 0x53: if (g->cgb) g->hdma_dst = (uint16_t)(0x8000u | ((g->hdma_dst & 0x00F0u) | ((v & 0x1Fu) << 8))); break;
    case 0x54: if (g->cgb) g->hdma_dst = (uint16_t)((g->hdma_dst & 0xFF00u) | (v & 0xF0u)); break;
    case 0x55: if (g->cgb) hdma5_write(g, v); break;
    case 0x68: if (g->cgb) g->bcps = v & 0xBFu; break;
    case 0x69:
        if (g->cgb) {
            g->bgpal[g->bcps & 0x3Fu] = v;
            if (g->bcps & 0x80u) g->bcps = (uint8_t)(0x80u | ((g->bcps + 1u) & 0x3Fu));
        }
        break;
    case 0x6A: if (g->cgb) g->ocps = v & 0xBFu; break;
    case 0x6B:
        if (g->cgb) {
            g->objpal[g->ocps & 0x3Fu] = v;
            if (g->ocps & 0x80u) g->ocps = (uint8_t)(0x80u | ((g->ocps + 1u) & 0x3Fu));
        }
        break;
    case 0x6C: if (g->cgb) g->opri = v & 1u; break;
    case 0x70: if (g->cgb) { g->svbk = v & 7u; if (!g->svbk) g->svbk = 1; } break;
    default: g->io[r] = v; break;
    }
}

/* ---------------------------------------------------------------- bus */
uint8_t gbc_read(gbc_t *g, uint16_t a)
{
    if (a < 0x4000u) {
        uint32_t off = g->rom0_off + a;
        return off < g->rom_size ? g->rom[off] : 0xFF;
    }
    if (a < 0x8000u) {
        uint32_t off = g->romx_off + (a & 0x3FFFu);
        return off < g->rom_size ? g->rom[off] : 0xFF;
    }
    if (a < 0xA000u)
        return g->vram[(uint32_t)g->vbk * 0x2000u + (a & 0x1FFFu)];
    if (a < 0xC000u)
        return eram_read(g, a);
    if (a < 0xD000u)
        return g->wram[a & 0x0FFFu];
    if (a < 0xFE00u) {
        /* D000-DFFF banked WRAM; E000-FDFF echoes C000-DDFF */
        uint16_t w = (a >= 0xE000u) ? (uint16_t)(a - 0x2000u) : a;
        if (w < 0xD000u)
            return g->wram[w & 0x0FFFu];
        return g->wram[(uint32_t)(g->cgb ? g->svbk : 1u) * 0x1000u + (w & 0x0FFFu)];
    }
    if (a < 0xFEA0u)
        return g->dma_active && !g->dma_delay ? 0xFF : g->oam[a - 0xFE00u];
    if (a < 0xFF00u)
        return 0x00;
    if (a < 0xFF80u)
        return io_read(g, (uint8_t)(a & 0x7Fu));
    if (a < 0xFFFFu)
        return g->hram[a & 0x7Fu];
    return g->ie;
}

void gbc_write(gbc_t *g, uint16_t a, uint8_t v)
{
    if (a < 0x8000u) { mbc_write(g, a, v); return; }
    if (a < 0xA000u) { vram_store(g, (uint16_t)((uint32_t)g->vbk * 0x2000u + (a & 0x1FFFu)), v); return; }
    if (a < 0xC000u) { eram_write(g, a, v); return; }
    if (a < 0xD000u) { g->wram[a & 0x0FFFu] = v; return; }
    if (a < 0xFE00u) {
        uint16_t w = (a >= 0xE000u) ? (uint16_t)(a - 0x2000u) : a;
        if (w < 0xD000u) g->wram[w & 0x0FFFu] = v;
        else g->wram[(uint32_t)(g->cgb ? g->svbk : 1u) * 0x1000u + (w & 0x0FFFu)] = v;
        return;
    }
    if (a < 0xFEA0u) { if (!(g->dma_active && !g->dma_delay)) oam_store(g, (uint16_t)(a - 0xFE00u), v); return; }
    if (a < 0xFF00u) return;
    if (a < 0xFF80u) { io_write(g, (uint8_t)(a & 0x7Fu), v); return; }
    if (a < 0xFFFFu) { g->hram[a & 0x7Fu] = v; return; }
    g->ie = v;
}

/* ---------------------------------------------------------------- clock */
void gbc_tick(gbc_t *g)
{
    uint16_t old;
    uint32_t dots = g->double_speed ? 2u : 4u;

    /* timer (CPU clock) */
    if (g->tima_reload) {
        g->tima_reload = 0;
        g->io[0x05] = g->io[0x06];
        g->io[0x0F] |= IRQ_TIMER;
    }
    old = g->div;
    g->div = (uint16_t)(g->div + 4u);
    if (timer_signal(g, old, g->io[0x07]) && !timer_signal(g, g->div, g->io[0x07]))
        tima_inc(g);

    /* serial (internal clock, no partner: shifts in 1s) */
    if (g->serial_active) {
        g->serial_cycles += 4u;
        if (g->serial_cycles >= 4096u) {
            g->serial_active = 0;
            g->io[0x01] = 0xFF;
            g->io[0x02] &= 0x7Fu;
            g->io[0x0F] |= IRQ_SERIAL;
        }
    }

    /* OAM DMA: 1 byte per M-cycle after a 1-cycle start delay */
    if (g->dma_active) {
        if (g->dma_delay) {
            g->dma_delay = 0;
        } else {
            uint16_t src = (uint16_t)(g->dma_src + g->dma_pos);
            uint8_t v = gbc_read(g, (src >= 0xE000u) ? (uint16_t)(src - 0x2000u) : src);
            oam_store(g, g->dma_pos, v);
            g->dma_pos++;
            if (g->dma_pos >= 0xA0u)
                g->dma_active = 0;
        }
    }

    gbc_ppu_tick(g, dots);
    g->apu_now += dots;
    g->total_cycles += 4u;
}
