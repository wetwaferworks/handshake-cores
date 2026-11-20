/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_vdp.c - clean-room Genesis VDP port side (Core 0 domain): register
 * writes, two-word address/code latch, VRAM/CRAM/VSRAM data port, status
 * register, HV counter, H/V interrupt generation + acknowledge and all three
 * DMA modes (68k->VRAM/CRAM/VSRAM, VRAM fill, VRAM copy). DMA completes
 * instantly (no FIFO/bus-stall timing). Rendering lives in gen_render.c and
 * only ever reads published snapshots.
 */
#include "gen_internal.h"
#include <string.h>

void gen_vdp_reset(gen_cpu_domain_t *d)
{
    gen_vdp_t *v = &d->vdp;
    memset(v->vram, 0, sizeof(v->vram));
    memset(v->cram, 0, sizeof(v->cram));
    memset(v->vsram, 0, sizeof(v->vsram));
    memset(v->reg, 0, sizeof(v->reg));
    v->addr = 0; v->code = 0; v->pending = 0;
    v->vint_pending = v->hint_pending = 0;
    v->vblank = 0; v->hblank = 0; v->fill_pending = 0; v->odd = 0;
    v->hint_counter = 0; v->line = 0; v->line_start_cyc = 0;
    v->dma_words = 0;
}

void gen_vdp_update_irq(gen_cpu_domain_t *d)
{
    gen_vdp_t *v = &d->vdp;
    uint8_t lvl = 0;
    if (v->hint_pending && (v->reg[0] & 0x10)) lvl = 4;
    if (v->vint_pending && (v->reg[1] & 0x20)) lvl = 6;
    d->cpu.irq_level = lvl;
}

uint32_t gen_vdp_iack(void *ctx, int level)
{
    gen_cpu_domain_t *d = (gen_cpu_domain_t *)ctx;
    if (level == 6) d->vdp.vint_pending = 0;
    else if (level == 4) d->vdp.hint_pending = 0;
    gen_vdp_update_irq(d);
    return 24u + (uint32_t)level;
}

static inline void vram_write_word(gen_vdp_t *v, uint16_t addr, uint16_t w)
{
    if (addr & 1) { v->vram[addr & 0xFFFE] = (uint8_t)w; v->vram[addr | 1] = (uint8_t)(w >> 8); }
    else { v->vram[addr] = (uint8_t)(w >> 8); v->vram[addr | 1] = (uint8_t)w; }
}

static void data_write(gen_vdp_t *v, uint16_t w)
{
    switch (v->code & 0x0F) {
    case 0x1: vram_write_word(v, v->addr, w); break;
    case 0x3: v->cram[(v->addr >> 1) & 63] = (uint16_t)(w & 0x0EEE); break;
    case 0x5: if (((v->addr >> 1) & 63) < 40) v->vsram[(v->addr >> 1) & 63] = (uint16_t)(w & 0x07FF); break;
    default: break;
    }
    v->addr = (uint16_t)(v->addr + v->reg[15]);
}

static uint16_t data_read(gen_vdp_t *v)
{
    uint16_t r = 0;
    switch (v->code & 0x0F) {
    case 0x0: r = (uint16_t)((v->vram[v->addr & 0xFFFE] << 8) | v->vram[v->addr | 1]); break;
    case 0x4: { uint32_t i = (v->addr >> 1) & 63; r = i < 40 ? v->vsram[i] : 0; break; }
    case 0x8: r = v->cram[(v->addr >> 1) & 63]; break;
    default: break;
    }
    v->addr = (uint16_t)(v->addr + v->reg[15]);
    return r;
}

static uint32_t dma_len(const gen_vdp_t *v)
{
    uint32_t n = (uint32_t)v->reg[19] | ((uint32_t)v->reg[20] << 8);
    return n ? n : 0x10000u;
}

static void dma_68k(gen_cpu_domain_t *d)
{
    gen_vdp_t *v = &d->vdp;
    uint32_t n = dma_len(v), i;
    uint32_t src = ((uint32_t)v->reg[21] << 1) | ((uint32_t)v->reg[22] << 9) | ((uint32_t)(v->reg[23] & 0x7F) << 17);
    for (i = 0; i < n; i++) {
        uint16_t w = gen_bus_read16(d, src);
        data_write(v, w);
        src = (src & 0xFE0000u) | ((src + 2) & 0x1FFFFu);
    }
    d->cpu.cyc += n;         /* coarse bus-stall estimate */
    v->dma_words += n;
    v->reg[19] = v->reg[20] = 0;
    v->reg[21] = (uint8_t)(src >> 1);
    v->reg[22] = (uint8_t)(src >> 9);
}

static void dma_fill(gen_vdp_t *v, uint16_t w)
{
    uint32_t n = dma_len(v), i;
    vram_write_word(v, v->addr, w);
    for (i = 0; i < n; i++) {
        v->vram[(v->addr ^ 1) & 0xFFFF] = (uint8_t)(w >> 8);
        v->addr = (uint16_t)(v->addr + v->reg[15]);
    }
    v->dma_words += n;
    v->reg[19] = v->reg[20] = 0;
}

static void dma_copy(gen_vdp_t *v)
{
    uint32_t n = dma_len(v), i;
    uint16_t src = (uint16_t)(v->reg[21] | (v->reg[22] << 8));
    for (i = 0; i < n; i++) {
        v->vram[v->addr ^ 1] = v->vram[src ^ 1];
        src = (uint16_t)(src + 1);
        v->addr = (uint16_t)(v->addr + v->reg[15]);
    }
    v->dma_words += n;
    v->reg[19] = v->reg[20] = 0;
    v->reg[21] = (uint8_t)src;
    v->reg[22] = (uint8_t)(src >> 8);
}

static void ctrl_write(gen_cpu_domain_t *d, uint16_t w)
{
    gen_vdp_t *v = &d->vdp;
    if (!v->pending) {
        if ((w & 0xC000) == 0x8000) {
            uint32_t r = (w >> 8) & 0x1F;
            if (r < 24) {
                uint8_t val = (uint8_t)w;
                v->reg[r] = val;
                gen_vdp_update_irq(d);
            }
            v->code = 0;
            return;
        }
        v->code = (uint8_t)((v->code & 0x3C) | ((w >> 14) & 3));
        v->addr = (uint16_t)((v->addr & 0xC000) | (w & 0x3FFF));
        v->pending = 1;
        return;
    }
    v->pending = 0;
    v->code = (uint8_t)((v->code & 0x03) | ((w >> 2) & 0x3C));
    v->addr = (uint16_t)((v->addr & 0x3FFF) | ((w & 3) << 14));
    if ((v->code & 0x20) && (v->reg[1] & 0x10)) {
        switch (v->reg[23] >> 6) {
        case 0: case 1: dma_68k(d); break;
        case 2: v->fill_pending = 1; break;
        default: dma_copy(v); break;
        }
    }
}

static uint16_t status_read(gen_cpu_domain_t *d)
{
    gen_vdp_t *v = &d->vdp;
    uint16_t s = 0x3400 | 0x0200;                        /* FIFO empty */
    if (v->vint_pending) s |= 0x0080;
    if (d->frame & 1) s |= 0x0010;
    if (v->vblank || !(v->reg[1] & 0x40)) s |= 0x0008;
    if (v->hblank) s |= 0x0004;
    if (d->pal) s |= 0x0001;
    v->pending = 0;
    return s;
}

static uint16_t hv_read(gen_cpu_domain_t *d)
{
    gen_vdp_t *v = &d->vdp;
    uint32_t line = v->line, vc, hc;
    uint64_t in_line = d->cpu.cyc - v->line_start_cyc;
    if (d->pal) vc = line <= 0x102 ? line : line - 57;
    else vc = line <= 0xEA ? line : line - 6;
    hc = (uint32_t)((in_line > 487 ? 487 : in_line) * ((v->reg[12] & 0x81) ? 210u : 171u) / 488u);
    return (uint16_t)(((vc & 0xFF) << 8) | (hc & 0xFF));
}

uint16_t gen_vdp_read(gen_cpu_domain_t *d, uint32_t a)
{
    switch (a & 0x1E) {
    case 0x00: case 0x02: d->vdp.pending = 0; return data_read(&d->vdp);
    case 0x04: case 0x06: return status_read(d);
    case 0x08: case 0x0A: case 0x0C: case 0x0E: return hv_read(d);
    default: return 0;
    }
}

void gen_vdp_write(gen_cpu_domain_t *d, uint32_t a, uint16_t w)
{
    gen_vdp_t *v = &d->vdp;
    switch (a & 0x1E) {
    case 0x00: case 0x02:
        v->pending = 0;
        if (v->fill_pending) { v->fill_pending = 0; dma_fill(v, w); return; }
        data_write(v, w);
        return;
    case 0x04: case 0x06:
        ctrl_write(d, w);
        return;
    default:
        return;
    }
}
