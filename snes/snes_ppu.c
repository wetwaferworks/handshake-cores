/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_ppu.c — clean-room scanline-snapshot SNES PPU for hs-snes.
 *
 *  - Phase-0 side: $2100-$213F register file, VRAM/CGRAM/OAM ports, and the
 *    per-line immutable snapshot writer (snes_ppu_snapshot_line/frame).
 *  - Phase-1 side: hs_snes_render_video_phase() rasterises ONLY from a
 *    committed hs_snes_slot: BG modes 0-7 (incl. offset-per-tile, mosaic,
 *    hi-res 512, Mode 7 affine + EXTBG + direct color), 128 OAM sprites with
 *    32-per-line / 34-tile limits, windows, main/sub color math, master
 *    brightness, and a built-in full-screen 640x480 sharp-bilinear
 *    upsampler (integer LUTs, no floats, no per-pixel divides).
 */
#include <string.h>
#include "snes_internal.h"

/* ====================================================== phase-0 register side */
static uint16_t vram_remap(uint16_t a, uint8_t vmain)
{
    switch ((vmain >> 2) & 3u) {
    case 0: return a & 0x7FFFu;
    case 1: return (uint16_t)(((a & 0xFF00u) | ((a & 0x1Fu) << 3) | ((a >> 5) & 7u)) & 0x7FFFu);
    case 2: return (uint16_t)(((a & 0xFE00u) | ((a & 0x3Fu) << 3) | ((a >> 6) & 7u)) & 0x7FFFu);
    default: return (uint16_t)(((a & 0xFC00u) | ((a & 0x7Fu) << 3) | ((a >> 7) & 7u)) & 0x7FFFu);
    }
}
static const uint16_t k_vinc[4] = {1, 32, 128, 128};

void snes_ppu_reset(snes_ppu_io *p)
{
    memset(p, 0, sizeof *p);
    p->inidisp = 0x80;
    p->vmain = 0x80;
}

static inline int16_t sext13(uint16_t v) { return (int16_t)((v & 0x1000u) ? (v | 0xE000u) : (v & 0x1FFFu)); }

uint8_t snes_ppu_read(hs_snes *s, uint8_t reg)
{
    snes_ppu_io *p = &s->ppu;
    uint8_t v;
    switch (reg) {
    case 0x34: case 0x35: case 0x36: {
        int32_t prod = (int32_t)p->m7a * (int32_t)(int8_t)(p->m7b >> 8);
        v = (uint8_t)(prod >> ((reg - 0x34) * 8));
        p->ppu1_mdr = v;
        return v;
    }
    case 0x37:
        p->hlatch = (uint16_t)(((s->master - s->line_start) >> 2) & 0x1FFu);
        p->vlatch = s->vpos;
        p->latched = 1;
        return s->mdr;
    case 0x38: {
        uint16_t a = p->oam_ptr;
        v = (a < 0x200u) ? p->oam[a] : p->oam[0x200u + (a & 0x1Fu)];
        p->oam_ptr = (uint16_t)((a + 1u) & 0x3FFu);
        p->ppu1_mdr = v;
        return v;
    }
    case 0x39:
        v = (uint8_t)p->vram_prefetch;
        if (!(p->vmain & 0x80u)) {
            p->vram_prefetch = p->vram[vram_remap(p->vmaddr, p->vmain)];
            p->vmaddr = (uint16_t)(p->vmaddr + k_vinc[p->vmain & 3u]);
        }
        p->ppu1_mdr = v;
        return v;
    case 0x3A:
        v = (uint8_t)(p->vram_prefetch >> 8);
        if (p->vmain & 0x80u) {
            p->vram_prefetch = p->vram[vram_remap(p->vmaddr, p->vmain)];
            p->vmaddr = (uint16_t)(p->vmaddr + k_vinc[p->vmain & 3u]);
        }
        p->ppu1_mdr = v;
        return v;
    case 0x3B:
        if (!(p->cgaddr & 1u)) v = (uint8_t)p->cgram[p->cgaddr >> 1];
        else v = (uint8_t)(((p->cgram[p->cgaddr >> 1] >> 8) & 0x7Fu) | (p->ppu2_mdr & 0x80u));
        p->cgaddr = (uint16_t)((p->cgaddr + 1u) & 0x1FFu);
        p->ppu2_mdr = v;
        return v;
    case 0x3C:
        v = p->hflip_rd ? (uint8_t)((p->hlatch >> 8) & 1u) : (uint8_t)p->hlatch;
        p->hflip_rd ^= 1u;
        p->ppu2_mdr = v;
        return v;
    case 0x3D:
        v = p->vflip_rd ? (uint8_t)((p->vlatch >> 8) & 1u) : (uint8_t)p->vlatch;
        p->vflip_rd ^= 1u;
        p->ppu2_mdr = v;
        return v;
    case 0x3E:
        v = (uint8_t)(0x01u | (p->ppu1_mdr & 0x10u));
        p->ppu1_mdr = v;
        return v;
    case 0x3F:
        v = (uint8_t)((p->latched ? 0x40u : 0u) | 0x03u | (p->ppu2_mdr & 0x20u));
        p->latched = 0;
        p->hflip_rd = 0;
        p->vflip_rd = 0;
        p->ppu2_mdr = v;
        return v;
    default:
        return p->ppu1_mdr;
    }
}

void snes_ppu_write(hs_snes *s, uint8_t reg, uint8_t v)
{
    snes_ppu_io *p = &s->ppu;
    switch (reg) {
    case 0x00: p->inidisp = v; break;
    case 0x01: p->obsel = v; break;
    case 0x02:
        p->oamaddr_reload = (uint16_t)((p->oamaddr_reload & 0x8100u) | v);
        p->oam_ptr = (uint16_t)((p->oamaddr_reload & 0x1FFu) << 1);
        break;
    case 0x03:
        p->oamaddr_reload = (uint16_t)((p->oamaddr_reload & 0x00FFu) | ((v & 1u) << 8) | ((v & 0x80u) << 8));
        p->oam_ptr = (uint16_t)((p->oamaddr_reload & 0x1FFu) << 1);
        break;
    case 0x04: {
        uint16_t a = p->oam_ptr;
        if (a < 0x200u) {
            if (!(a & 1u)) p->oam_latch = v;
            else { p->oam[a - 1u] = p->oam_latch; p->oam[a] = v; }
        } else {
            p->oam[0x200u + (a & 0x1Fu)] = v;
        }
        p->oam_ptr = (uint16_t)((a + 1u) & 0x3FFu);
        break;
    }
    case 0x05: p->bgmode = v; break;
    case 0x06: p->mosaic = v; break;
    case 0x07: case 0x08: case 0x09: case 0x0A: p->bgsc[reg - 0x07] = v; break;
    case 0x0B: p->bgnba[0] = v; break;
    case 0x0C: p->bgnba[1] = v; break;
    case 0x0D: case 0x0F: case 0x11: case 0x13: {
        int i = (reg - 0x0D) >> 1;
        if (reg == 0x0D) { p->m7hofs = (uint16_t)(((v << 8) | p->m7_latch) & 0x1FFFu); p->m7_latch = v; }
        p->hofs[i] = (uint16_t)(((v << 8) | (p->ofs_latch & ~7u) | (p->hofs_latch & 7u)) & 0x3FFu);
        p->ofs_latch = v; p->hofs_latch = v;
        break;
    }
    case 0x0E: case 0x10: case 0x12: case 0x14: {
        int i = (reg - 0x0E) >> 1;
        if (reg == 0x0E) { p->m7vofs = (uint16_t)(((v << 8) | p->m7_latch) & 0x1FFFu); p->m7_latch = v; }
        p->vofs[i] = (uint16_t)(((v << 8) | p->ofs_latch) & 0x3FFu);
        p->ofs_latch = v;
        break;
    }
    case 0x15: p->vmain = v; break;
    case 0x16:
        p->vmaddr = (uint16_t)((p->vmaddr & 0xFF00u) | v);
        p->vram_prefetch = p->vram[vram_remap(p->vmaddr, p->vmain)];
        break;
    case 0x17:
        p->vmaddr = (uint16_t)((p->vmaddr & 0x00FFu) | (v << 8));
        p->vram_prefetch = p->vram[vram_remap(p->vmaddr, p->vmain)];
        break;
    case 0x18: {
        uint16_t a = vram_remap(p->vmaddr, p->vmain);
        p->vram[a] = (uint16_t)((p->vram[a] & 0xFF00u) | v);
        if (!(p->vmain & 0x80u)) p->vmaddr = (uint16_t)(p->vmaddr + k_vinc[p->vmain & 3u]);
        break;
    }
    case 0x19: {
        uint16_t a = vram_remap(p->vmaddr, p->vmain);
        p->vram[a] = (uint16_t)((p->vram[a] & 0x00FFu) | (v << 8));
        if (p->vmain & 0x80u) p->vmaddr = (uint16_t)(p->vmaddr + k_vinc[p->vmain & 3u]);
        break;
    }
    case 0x1A: p->m7sel = v; break;
    case 0x1B: p->m7a = (int16_t)((v << 8) | p->m7_latch); p->m7_latch = v; break;
    case 0x1C: p->m7b = (int16_t)((v << 8) | p->m7_latch); p->m7_latch = v; p->m7b_last = v; break;
    case 0x1D: p->m7c = (int16_t)((v << 8) | p->m7_latch); p->m7_latch = v; break;
    case 0x1E: p->m7d = (int16_t)((v << 8) | p->m7_latch); p->m7_latch = v; break;
    case 0x1F: p->m7x = sext13((uint16_t)((v << 8) | p->m7_latch)); p->m7_latch = v; break;
    case 0x20: p->m7y = sext13((uint16_t)((v << 8) | p->m7_latch)); p->m7_latch = v; break;
    case 0x21: p->cgaddr = (uint16_t)(v << 1); break;
    case 0x22:
        if (!(p->cgaddr & 1u)) p->cg_latch = v;
        else { p->cgram[p->cgaddr >> 1] = (uint16_t)(((v & 0x7Fu) << 8) | p->cg_latch); p->cgram_dirty = 1; }
        p->cgaddr = (uint16_t)((p->cgaddr + 1u) & 0x1FFu);
        break;
    case 0x23: p->w12sel = v; break;
    case 0x24: p->w34sel = v; break;
    case 0x25: p->wobjsel = v; break;
    case 0x26: p->wh0 = v; break;
    case 0x27: p->wh1 = v; break;
    case 0x28: p->wh2 = v; break;
    case 0x29: p->wh3 = v; break;
    case 0x2A: p->wbglog = v; break;
    case 0x2B: p->wobjlog = v; break;
    case 0x2C: p->tm = v; break;
    case 0x2D: p->ts = v; break;
    case 0x2E: p->tmw = v; break;
    case 0x2F: p->tsw = v; break;
    case 0x30: p->cgwsel = v; break;
    case 0x31: p->cgadsub = v; break;
    case 0x32:
        if (v & 0x20u) p->fixed_r = v & 0x1Fu;
        if (v & 0x40u) p->fixed_g = v & 0x1Fu;
        if (v & 0x80u) p->fixed_b = v & 0x1Fu;
        break;
    case 0x33: p->setini = v; break;
    default: break;
    }
}

void snes_ppu_snapshot_frame(hs_snes *s)
{
    hs_snes_slot *o = s->slot;
    snes_ppu_io *p = &s->ppu;
    memcpy(o->vram, p->vram, sizeof o->vram);
    memcpy(o->oam, p->oam, sizeof o->oam);
    memcpy(o->cgram[0], p->cgram, sizeof p->cgram);
    o->cgram_count = 1;
    p->cgram_dirty = 0;
}

void snes_ppu_snapshot_line(hs_snes *s, int line)
{
    hs_snes_slot *o = s->slot;
    snes_ppu_io *p = &s->ppu;
    hs_snes_line *L;
    if (line < 1 || line >= HS_SNES_MAX_LINES) return;
    L = &o->line[line];
    if (p->cgram_dirty && o->cgram_count < HS_SNES_CGRAM_POOL) {
        memcpy(o->cgram[o->cgram_count], p->cgram, sizeof p->cgram);
        o->cgram_count++;
    }
    p->cgram_dirty = 0;
    L->inidisp = p->inidisp; L->obsel = p->obsel; L->bgmode = p->bgmode; L->mosaic = p->mosaic;
    memcpy(L->bgsc, p->bgsc, 4); L->bgnba[0] = p->bgnba[0]; L->bgnba[1] = p->bgnba[1];
    memcpy(L->hofs, p->hofs, sizeof L->hofs); memcpy(L->vofs, p->vofs, sizeof L->vofs);
    L->m7sel = p->m7sel; L->setini = p->setini;
    L->m7a = p->m7a; L->m7b = p->m7b; L->m7c = p->m7c; L->m7d = p->m7d;
    L->m7x = p->m7x; L->m7y = p->m7y; L->m7hofs = p->m7hofs; L->m7vofs = p->m7vofs;
    L->w12sel = p->w12sel; L->w34sel = p->w34sel; L->wobjsel = p->wobjsel;
    L->wbglog = p->wbglog; L->wobjlog = p->wobjlog;
    L->wh0 = p->wh0; L->wh1 = p->wh1; L->wh2 = p->wh2; L->wh3 = p->wh3;
    L->tm = p->tm; L->ts = p->ts; L->tmw = p->tmw; L->tsw = p->tsw;
    L->cgwsel = p->cgwsel; L->cgadsub = p->cgadsub;
    L->fixed_r = p->fixed_r; L->fixed_g = p->fixed_g; L->fixed_b = p->fixed_b;
    L->oam_first = (p->oamaddr_reload & 0x8000u) ? (uint8_t)((p->oamaddr_reload >> 1) & 0x7Fu) : 0;
    L->cgram_index = (uint16_t)(o->cgram_count - 1u);
}

/* ====================================================== phase-1 renderer side */
static const uint8_t k_bpp[8][4] = {
    {2, 2, 2, 2}, {4, 4, 2, 0}, {4, 4, 0, 0}, {8, 4, 0, 0},
    {8, 2, 0, 0}, {4, 2, 0, 0}, {4, 0, 0, 0}, {8, 0, 0, 0}
};
static const uint8_t k_obj_w[8][2] = {{8, 16}, {8, 32}, {8, 64}, {16, 32}, {16, 64}, {32, 64}, {16, 32}, {16, 32}};
static const uint8_t k_obj_h[8][2] = {{8, 16}, {8, 32}, {8, 64}, {16, 32}, {16, 64}, {32, 64}, {32, 64}, {32, 32}};

typedef struct zmap { uint8_t bg[4][2]; uint8_t obj[4]; } zmap;

static void build_z(zmap *z, uint8_t bgmode, uint8_t setini)
{
    int mode = bgmode & 7;
    memset(z, 0, sizeof *z);
    switch (mode) {
    case 0:
        z->obj[3] = 12; z->bg[0][1] = 11; z->bg[1][1] = 10; z->obj[2] = 9;
        z->bg[0][0] = 8; z->bg[1][0] = 7; z->obj[1] = 6; z->bg[2][1] = 5;
        z->bg[3][1] = 4; z->obj[0] = 3; z->bg[2][0] = 2; z->bg[3][0] = 1;
        break;
    case 1:
        if (bgmode & 0x08u) {
            z->bg[2][1] = 10; z->obj[3] = 9; z->bg[0][1] = 8; z->bg[1][1] = 7; z->obj[2] = 6;
            z->bg[0][0] = 5; z->bg[1][0] = 4; z->obj[1] = 3; z->obj[0] = 2; z->bg[2][0] = 1;
        } else {
            z->obj[3] = 10; z->bg[0][1] = 9; z->bg[1][1] = 8; z->obj[2] = 7; z->bg[0][0] = 6;
            z->bg[1][0] = 5; z->obj[1] = 4; z->bg[2][1] = 3; z->obj[0] = 2; z->bg[2][0] = 1;
        }
        break;
    case 6:
        z->obj[3] = 6; z->bg[0][1] = 5; z->obj[2] = 4; z->obj[1] = 3; z->bg[0][0] = 2; z->obj[0] = 1;
        break;
    case 7:
        z->obj[3] = 7; z->obj[2] = 6; z->obj[1] = 4; z->obj[0] = 2;
        z->bg[0][0] = 3; z->bg[0][1] = 3;
        if (setini & 0x40u) { z->bg[1][1] = 5; z->bg[1][0] = 1; }
        break;
    default: /* modes 2-5 */
        z->obj[3] = 8; z->bg[0][1] = 7; z->obj[2] = 6; z->bg[1][1] = 5;
        z->obj[1] = 4; z->bg[0][0] = 3; z->obj[0] = 2; z->bg[1][0] = 1;
        break;
    }
}

static inline uint16_t direct_color(uint8_t c, uint8_t pal)
{
    uint16_t r = (uint16_t)(((c & 0x07u) << 2) | ((pal & 1u) << 1));
    uint16_t g = (uint16_t)(((c & 0x38u) >> 1) | (pal & 2u));
    uint16_t b = (uint16_t)(((c & 0xC0u) >> 3) | ((pal & 4u) << 0));
    return (uint16_t)(r | (g << 5) | (b << 10));
}

static inline uint16_t map_entry(const uint16_t *vram, uint8_t sc, int tx, int ty)
{
    uint32_t a = ((uint32_t)(sc & 0xFCu) << 8) + (uint32_t)((ty & 31) << 5) + (uint32_t)(tx & 31);
    if ((tx & 32) && (sc & 1u)) a += 0x400u;
    if ((ty & 32) && (sc & 2u)) a += (sc & 1u) ? 0x800u : 0x400u;
    return vram[a & 0x7FFFu];
}

/* decode one 8-pixel row of a tile into color indices (leftmost first) */
static inline void decode_row(const uint16_t *vram, uint32_t addr, int bpp, int hflip, uint8_t out[8])
{
    uint16_t p01 = vram[addr & 0x7FFFu];
    uint16_t p23 = bpp >= 4 ? vram[(addr + 8u) & 0x7FFFu] : 0;
    uint16_t p45 = bpp == 8 ? vram[(addr + 16u) & 0x7FFFu] : 0;
    uint16_t p67 = bpp == 8 ? vram[(addr + 24u) & 0x7FFFu] : 0;
    int px;
    for (px = 0; px < 8; px++) {
        int b = hflip ? px : 7 - px;
        uint8_t c = (uint8_t)(((p01 >> b) & 1u) | (((p01 >> (b + 8)) & 1u) << 1));
        if (bpp >= 4) c |= (uint8_t)((((p23 >> b) & 1u) << 2) | (((p23 >> (b + 8)) & 1u) << 3));
        if (bpp == 8) c |= (uint8_t)((((p45 >> b) & 1u) << 4) | (((p45 >> (b + 8)) & 1u) << 5) |
                                     (((p67 >> b) & 1u) << 6) | (((p67 >> (b + 8)) & 1u) << 7));
        out[px] = c;
    }
}

typedef struct rline {
    const hs_snes_line *L;
    const uint16_t *vram;
    const uint16_t *cg;
    int v;        /* 1-based scanline */
    int mode;
    int hires;    /* BG layers sampled at 512 */
} rline;

static void render_bg(hs_snes_video *vd, const rline *r, int layer, int bpp, uint8_t zlo, uint8_t zhi)
{
    const hs_snes_line *L = r->L;
    const uint16_t *vram = r->vram;
    uint16_t *col = vd->lay_col[layer];
    uint8_t *zz = vd->lay_z[layer];
    int width = r->hires ? 512 : 256;
    int t16 = (L->bgmode >> (4 + layer)) & 1;
    int tw16 = r->hires ? 1 : t16;
    int hofs = L->hofs[layer], vofs = L->vofs[layer];
    uint8_t sc = L->bgsc[layer];
    uint32_t chr = (uint32_t)((layer < 2 ? (L->bgnba[0] >> (layer * 4)) : (L->bgnba[1] >> ((layer - 2) * 4))) & 0x0Fu) << 12;
    uint32_t words_per_tile = (uint32_t)bpp * 4u;
    int msz = (L->mosaic & (1u << layer)) ? ((L->mosaic >> 4) + 1) : 1;
    int y = r->v;
    int opt = (r->mode == 2 || r->mode == 4 || r->mode == 6) && layer < 2;
    int direct = (bpp == 8) && (L->cgwsel & 1u);
    uint16_t palbase = (r->mode == 0) ? (uint16_t)(layer * 32) : 0;
    int x, last_key = -1, last_col = -1, hs = hofs, vs = vofs, mcnt = 0, xs_m = 0;
    uint8_t row[8];
    uint16_t ent = 0;
    if (msz > 1) y = ((y - 1) / msz) * msz + 1;   /* per-line, not per-pixel */
    for (x = 0; x < width; x++) {
        int xs = x;
        int X, Y, key, sub;
        if (msz > 1) {                     /* mosaic via counter: no per-pixel divide */
            if (mcnt == 0) xs_m = x;
            if (++mcnt == msz) mcnt = 0;
            xs = xs_m;
        }
        uint8_t c;
        if (opt) {
            int colidx = (xs + (hofs & 7)) >> 3;
            if (colidx != last_col) {
                last_col = colidx;
                hs = hofs; vs = vofs;
                if (colidx >= 1) {
                    int tx3 = ((L->hofs[2] >> 3) + colidx - 1) & 63;
                    int ty3 = (L->vofs[2] >> 3) & 63;
                    uint16_t hv = map_entry(vram, L->bgsc[2], tx3, ty3);
                    uint16_t vv = (r->mode == 4) ? hv : map_entry(vram, L->bgsc[2], tx3, ty3 + 1);
                    uint16_t valid = (uint16_t)(layer == 0 ? 0x2000u : 0x4000u);
                    if (r->mode == 4) {
                        if (hv & valid) {
                            if (hv & 0x8000u) vs = hv & 0x3FF;
                            else hs = (hv & 0x3F8) | (hofs & 7);
                        }
                    } else {
                        if (hv & valid) hs = (hv & 0x3F8) | (hofs & 7);
                        if (vv & valid) vs = vv & 0x3FF;
                    }
                }
                last_key = -1;
            }
        }
        X = r->hires ? (xs + (hs << 1)) : (xs + hs);
        Y = y + vs;
        key = (X >> 3) & 0x3FF;
        if (key != last_key) {
            int tx = tw16 ? (X >> 4) : (X >> 3);
            int ty = t16 ? (Y >> 4) : (Y >> 3);
            int fine_y, tile, hflip, vflip;
            last_key = key;
            ent = map_entry(vram, sc, tx & 63, ty & 63);
            hflip = (ent & 0x4000u) != 0;
            vflip = (ent & 0x8000u) != 0;
            tile = ent & 0x3FF;
            if (tw16) { sub = (X >> 3) & 1; if (hflip) sub ^= 1; tile += sub; }
            if (t16) { sub = (Y >> 3) & 1; if (vflip) sub ^= 1; tile += sub * 16; }
            fine_y = Y & 7;
            if (vflip) fine_y = 7 - fine_y;
            decode_row(vram, chr + (uint32_t)(tile & 0x3FF) * words_per_tile + (uint32_t)fine_y, bpp, hflip, row);
        }
        c = row[X & 7];
        if (!c) { zz[x] = 0; continue; }
        zz[x] = (ent & 0x2000u) ? zhi : zlo;
        {
            uint8_t pal = (uint8_t)((ent >> 10) & 7u);
            if (direct) col[x] = direct_color(c, pal);
            else if (bpp == 8) col[x] = r->cg[c];
            else if (bpp == 4) col[x] = r->cg[(pal << 4) + c];
            else col[x] = r->cg[(palbase + (pal << 2) + c) & 0xFFu];
        }
    }
}

static inline int clip10(int a) { return (a & 0x2000) ? (a | ~0x3FF) : (a & 0x3FF); }

static void render_m7(hs_snes_video *vd, const rline *r, uint8_t z1, uint8_t z2lo, uint8_t z2hi, int ext)
{
    const hs_snes_line *L = r->L;
    const uint16_t *vram = r->vram;
    int a = L->m7a, b = L->m7b, c = L->m7c, d = L->m7d;
    int cx = L->m7x, cy = L->m7y;
    int hofs = sext13(L->m7hofs), vofs = sext13(L->m7vofs);
    int sy = (L->m7sel & 2u) ? 255 - r->v : r->v;
    int xx = clip10(hofs - cx), yy = clip10(vofs - cy);
    int ox = ((a * xx) & ~63) + ((b * yy) & ~63) + ((b * sy) & ~63) + (cx * 256);
    int oy = ((c * xx) & ~63) + ((d * yy) & ~63) + ((d * sy) & ~63) + (cy * 256);
    int oob = (L->m7sel >> 6) & 3;
    int hflip = L->m7sel & 1u;
    int direct = L->cgwsel & 1u;
    int x;
    for (x = 0; x < 256; x++) {
        int sx = hflip ? 255 - x : x;
        int X = ox + a * sx, Y = oy + c * sx;
        uint8_t tile, pix;
        if (((X | Y) & ~0x3FFFF) && oob >= 2) {
            if (oob == 2) { vd->lay_z[0][x] = 0; if (ext) vd->lay_z[1][x] = 0; continue; }
            tile = 0;
        } else {
            tile = (uint8_t)vram[(((Y >> 11) & 127) << 7) | ((X >> 11) & 127)];
        }
        pix = (uint8_t)(vram[((uint32_t)tile << 6) | (uint32_t)(((Y >> 8) & 7) << 3) | (uint32_t)((X >> 8) & 7)] >> 8);
        if (pix) {
            vd->lay_z[0][x] = z1;
            vd->lay_col[0][x] = direct ? direct_color(pix, 0) : r->cg[pix];
        } else {
            vd->lay_z[0][x] = 0;
        }
        if (ext) {
            uint8_t p7 = pix & 0x7Fu;
            if (p7) { vd->lay_z[1][x] = (pix & 0x80u) ? z2hi : z2lo; vd->lay_col[1][x] = r->cg[p7]; }
            else vd->lay_z[1][x] = 0;
        }
    }
}

static void render_obj(hs_snes_video *vd, const hs_snes_slot *slot, const rline *r, const zmap *z)
{
    const hs_snes_line *L = r->L;
    const uint8_t *oam = slot->oam;
    int row = r->v - 1;
    int sz = (L->obsel >> 5) & 7;
    uint32_t base = ((uint32_t)L->obsel & 7u) << 13;
    uint32_t gap = ((((uint32_t)L->obsel >> 3) & 3u) + 1u) << 12;
    uint8_t list[32];
    int n = 0, i, tiles = 0, limit;
    memset(vd->lay_z[4], 0, 256);
    for (i = 0; i < 128 && n < 32; i++) {
        int sidx = (L->oam_first + i) & 127;
        uint8_t hi = (uint8_t)(oam[512 + (sidx >> 2)] >> ((sidx & 3) * 2));
        int large = (hi >> 1) & 1;
        int w = k_obj_w[sz][large], h = k_obj_h[sz][large];
        int sx = oam[sidx * 4] | ((hi & 1) << 8);
        int dy;
        if (sx & 0x100) sx -= 512;
        dy = (row - oam[sidx * 4 + 1]) & 0xFF;
        if (dy >= h) continue;
        if (sx <= -w || sx >= 256) continue;
        list[n++] = (uint8_t)sidx;
    }
    /* 34-tile (time-over) limit, counted in priority order */
    limit = n;
    for (i = 0; i < n; i++) {
        int sidx = list[i];
        uint8_t hi = (uint8_t)(oam[512 + (sidx >> 2)] >> ((sidx & 3) * 2));
        tiles += k_obj_w[sz][(hi >> 1) & 1] >> 3;
        if (tiles > 34) { limit = i + 1; break; }
    }
    for (i = limit - 1; i >= 0; i--) {
        int sidx = list[i];
        uint8_t hi = (uint8_t)(oam[512 + (sidx >> 2)] >> ((sidx & 3) * 2));
        int large = (hi >> 1) & 1;
        int w = k_obj_w[sz][large], h = k_obj_h[sz][large];
        int sx = oam[sidx * 4] | ((hi & 1) << 8);
        uint8_t tile = oam[sidx * 4 + 2], attr = oam[sidx * 4 + 3];
        int hf = (attr & 0x40u) != 0, vf = (attr & 0x80u) != 0;
        int pal = (attr >> 1) & 7, prio = (attr >> 4) & 3;
        int dy = (row - oam[sidx * 4 + 1]) & 0xFF, tc, ty, fr;
        uint32_t tbase = base + ((attr & 1u) ? gap : 0u);
        if (sx & 0x100) sx -= 512;
        if (vf) dy = h - 1 - dy;
        ty = dy >> 3; fr = dy & 7;
        for (tc = 0; tc < (w >> 3); tc++) {
            int px0 = sx + tc * 8, colt, px;
            uint8_t t, rowc[8];
            if (px0 <= -8 || px0 >= 256) continue;
            colt = hf ? ((w >> 3) - 1 - tc) : tc;
            t = (uint8_t)(((tile + ty * 16) & 0xF0) | ((tile + colt) & 0x0F));
            decode_row(r->vram, tbase + (uint32_t)t * 16u + (uint32_t)fr, 4, hf, rowc);
            for (px = 0; px < 8; px++) {
                int X = px0 + px;
                uint8_t c = rowc[px];
                if (X < 0 || X >= 256 || !c) continue;
                vd->lay_col[4][X] = r->cg[128 + (pal << 4) + c];
                vd->lay_z[4][X] = z->obj[prio];
                vd->obj_pal[X] = (uint8_t)pal;
            }
        }
    }
}

static int build_window(uint8_t *out, uint8_t sel, uint8_t logic, const hs_snes_line *L)
{
    int e1 = (sel & 2u) != 0, i1 = sel & 1u, e2 = (sel & 8u) != 0, i2 = (sel & 4u) != 0;
    int x;
    if (!e1 && !e2) return 0;
    for (x = 0; x < 256; x++) {
        int a = (x >= L->wh0 && x <= L->wh1) ^ i1;
        int b = (x >= L->wh2 && x <= L->wh3) ^ i2;
        int m;
        if (e1 && e2) {
            switch (logic & 3u) {
            case 0: m = a | b; break;
            case 1: m = a & b; break;
            case 2: m = a ^ b; break;
            default: m = !(a ^ b); break;
            }
        } else {
            m = e1 ? a : b;
        }
        out[x] = (uint8_t)m;
    }
    return 1;
}

static inline uint16_t color_math(uint16_t a, uint16_t b, int sub, int half)
{
    int ar = a & 31, ag = (a >> 5) & 31, ab = (a >> 10) & 31;
    int br = b & 31, bg = (b >> 5) & 31, bb = (b >> 10) & 31;
    int r, g, bl;
    if (sub) {
        r = ar - br; g = ag - bg; bl = ab - bb;
        if (r < 0) r = 0;
        if (g < 0) g = 0;
        if (bl < 0) bl = 0;
        if (half) { r >>= 1; g >>= 1; bl >>= 1; }
    } else {
        r = ar + br; g = ag + bg; bl = ab + bb;
        if (half) { r >>= 1; g >>= 1; bl >>= 1; }
        if (r > 31) r = 31;
        if (g > 31) g = 31;
        if (bl > 31) bl = 31;
    }
    return (uint16_t)(r | (g << 5) | (bl << 10));
}

/* renders native scanline v (1-based) into vd->line32; returns width 256/512 */
static int render_native_line(const hs_snes_slot *slot, hs_snes_video *vd, int v)
{
    const hs_snes_line *L = &slot->line[v];
    rline r;
    zmap z;
    int mode = L->bgmode & 7, l, x, width, hires_out;
    uint8_t present = 0, haswin[6];
    uint8_t bright = L->inidisp & 0x0Fu;
    uint16_t fixed = (uint16_t)(L->fixed_r | (L->fixed_g << 5) | (L->fixed_b << 10));
    const uint8_t *bl = vd->bright[bright];

    r.L = L; r.vram = slot->vram;
    r.cg = slot->cgram[L->cgram_index < slot->cgram_count ? L->cgram_index : 0];
    r.v = v; r.mode = mode;
    r.hires = (mode == 5 || mode == 6);
    hires_out = r.hires || (L->setini & 0x08u);
    width = hires_out ? 512 : 256;

    if (L->inidisp & 0x80u) {
        memset(vd->line32, 0, sizeof(uint32_t) * (size_t)width);
        return width;
    }
    build_z(&z, L->bgmode, L->setini);

    /* layers */
    if (mode == 7) {
        int ext = (L->setini & 0x40u) != 0;
        if ((L->tm | L->ts) & (ext ? 3u : 1u)) {
            render_m7(vd, &r, z.bg[0][0], z.bg[1][0], z.bg[1][1], ext);
            present |= 1u;
            if (ext) present |= 2u;
        }
    } else {
        for (l = 0; l < 4; l++) {
            int bpp = k_bpp[mode][l];
            if (!bpp || !((L->tm | L->ts) & (1u << l))) continue;
            render_bg(vd, &r, l, bpp, z.bg[l][0], z.bg[l][1]);
            present |= (uint8_t)(1u << l);
        }
    }
    if ((L->tm | L->ts) & 0x10u) { render_obj(vd, slot, &r, &z); present |= 0x10u; }

    haswin[0] = (uint8_t)build_window(vd->win[0], L->w12sel & 0x0Fu, L->wbglog, L);
    haswin[1] = (uint8_t)build_window(vd->win[1], L->w12sel >> 4, (uint8_t)(L->wbglog >> 2), L);
    haswin[2] = (uint8_t)build_window(vd->win[2], L->w34sel & 0x0Fu, (uint8_t)(L->wbglog >> 4), L);
    haswin[3] = (uint8_t)build_window(vd->win[3], L->w34sel >> 4, (uint8_t)(L->wbglog >> 6), L);
    haswin[4] = (uint8_t)build_window(vd->win[4], L->wobjsel & 0x0Fu, L->wobjlog, L);
    haswin[5] = (uint8_t)build_window(vd->win[5], L->wobjsel >> 4, (uint8_t)(L->wobjlog >> 2), L);

    for (x = 0; x < 256; x++) {
        int mi = r.hires ? 2 * x + 1 : x, si = r.hires ? 2 * x : x;
        uint8_t mz = 0, sz = 0;
        int ml = 5, sl = 5;
        uint16_t mc = r.cg[0], sc = fixed;
        int cw, clip, prevent, math;
        for (l = 0; l < 5; l++) {
            int idx_m, idx_s;
            uint8_t bit = (uint8_t)(1u << l);
            if (!(present & bit)) continue;
            idx_m = (l == 4) ? x : mi;
            idx_s = (l == 4) ? x : si;
            if ((L->tm & bit) && vd->lay_z[l][idx_m] > mz &&
                !((L->tmw & bit) && haswin[l] && vd->win[l][x])) {
                mz = vd->lay_z[l][idx_m]; ml = l; mc = vd->lay_col[l][idx_m];
            }
            if ((L->ts & bit) && vd->lay_z[l][idx_s] > sz &&
                !((L->tsw & bit) && haswin[l] && vd->win[l][x])) {
                sz = vd->lay_z[l][idx_s]; sl = l; sc = vd->lay_col[l][idx_s];
            }
        }
        cw = haswin[5] ? vd->win[5][x] : 0;
        switch (L->cgwsel >> 6) { case 0: clip = 0; break; case 1: clip = !cw; break; case 2: clip = cw; break; default: clip = 1; break; }
        switch ((L->cgwsel >> 4) & 3u) { case 0: prevent = 0; break; case 1: prevent = !cw; break; case 2: prevent = cw; break; default: prevent = 1; break; }
        math = !prevent && (L->cgadsub & (1u << ml)) && !(ml == 4 && vd->obj_pal[x] < 4);
        if (clip) mc = 0;
        if (math) {
            int use_sub = (L->cgwsel & 2u) != 0;
            uint16_t operand = use_sub ? sc : fixed;
            int half = (L->cgadsub & 0x40u) && !clip && !(use_sub && sl == 5);
            mc = color_math(mc, operand, (L->cgadsub & 0x80u) != 0, half);
        }
        if (hires_out) {
            vd->line15[2 * x] = sc;
            vd->line15[2 * x + 1] = mc;
        } else {
            vd->line15[x] = mc;
        }
        (void)sl;
    }
    for (x = 0; x < width; x++) {
        uint16_t c = vd->line15[x];
        vd->line32[x] = ((uint32_t)bl[c & 31] << 16) | ((uint32_t)bl[(c >> 5) & 31] << 8) | bl[(c >> 10) & 31];
    }
    return width;
}

/* --------------------------------------------- 640x480 sharp-bilinear upsampler */
static void build_hlut(hs_snes_hlut *h, int src)
{
    int k = HS_SNES_OUT_W / src, i;
    if (k < 1) k = 1;
    for (i = 0; i < HS_SNES_OUT_W; i++) {
        int p = (int)(((int64_t)(2 * i + 1) * k * src * 128) / HS_SNES_OUT_W) - 128;
        int p0, f, a, b;
        if (p < 0) p = 0;
        p0 = p >> 8; f = p & 255;
        a = p0 / k; b = (p0 + 1) / k;
        if (a > src - 1) a = src - 1;
        if (b > src - 1) b = src - 1;
        h->i0[i] = (uint16_t)a;
        h->w[i] = (uint16_t)((a == b) ? 0 : f);
    }
}
static void build_vlut(hs_snes_vlut *v, int src)
{
    int k = HS_SNES_OUT_H / src, i;
    if (k < 1) k = 1;
    for (i = 0; i < HS_SNES_OUT_H; i++) {
        int p = (int)(((int64_t)(2 * i + 1) * k * src * 128) / HS_SNES_OUT_H) - 128;
        int p0, f, a, b;
        if (p < 0) p = 0;
        p0 = p >> 8; f = p & 255;
        a = p0 / k; b = (p0 + 1) / k;
        if (a > src - 1) a = src - 1;
        if (b > src - 1) b = src - 1;
        v->r0[i] = (uint16_t)a;
        v->w[i] = (uint16_t)((a == b) ? 0 : f);
    }
}

void hs_snes_video_init(hs_snes_video *v)
{
    int b, c;
    memset(v, 0, sizeof *v);
    build_hlut(&v->h256, 256);
    build_hlut(&v->h512, 512);
    build_vlut(&v->v224, 224);
    build_vlut(&v->v239, 239);
    for (b = 0; b < 16; b++)
        for (c = 0; c < 32; c++)
            v->bright[b][c] = (uint8_t)((c * 255 * (b + 1)) / (31 * 16));
    v->hrow_line[0] = v->hrow_line[1] = -1;
}

static inline uint32_t mix32(uint32_t a, uint32_t b, uint32_t w)
{
    uint32_t iw = 256u - w;
    uint32_t rb = (((a & 0xFF00FFu) * iw + (b & 0xFF00FFu) * w) >> 8) & 0xFF00FFu;
    uint32_t g = (((a & 0x00FF00u) * iw + (b & 0x00FF00u) * w) >> 8) & 0x00FF00u;
    return rb | g;
}

static void hscale_line(const hs_snes_video *v, const uint32_t *src, int width, uint32_t *dst)
{
    const hs_snes_hlut *h = (width == 512) ? &v->h512 : &v->h256;
    int x;
    for (x = 0; x < HS_SNES_OUT_W; x++) {
        uint32_t w = h->w[x];
        uint32_t a = src[h->i0[x]];
        dst[x] = w ? mix32(a, src[h->i0[x] + 1u], w) : a;
    }
}

static void ensure_row(const hs_snes_slot *slot, hs_snes_video *v, int r)
{
    int k = r & 1, width;
    if (v->hrow_line[k] == r) return;
    width = render_native_line(slot, v, r + 1);
    hscale_line(v, v->line32, width, v->hrow[k]);
    v->hrow_line[k] = r;
}

void hs_snes_render_video_rows(const hs_snes_slot *slot, hs_snes_video *v, uint32_t *fb, int y0, int y1)
{
    int lines = (slot->lines == 239) ? 239 : 224;
    const hs_snes_vlut *vl = (lines == 239) ? &v->v239 : &v->v224;
    int y;
    if (y0 < 0) y0 = 0;
    if (y1 > HS_SNES_OUT_H) y1 = HS_SNES_OUT_H;
    v->hrow_line[0] = v->hrow_line[1] = -1;
    for (y = y0; y < y1; y++) {
        int r0 = vl->r0[y], x;
        uint32_t w = vl->w[y];
        uint32_t *dst = fb + (size_t)y * HS_SNES_OUT_W;
        ensure_row(slot, v, r0);
        if (!w) {
            memcpy(dst, v->hrow[r0 & 1], sizeof(uint32_t) * HS_SNES_OUT_W);
        } else {
            const uint32_t *a, *b;
            ensure_row(slot, v, r0 + 1);
            a = v->hrow[r0 & 1];
            b = v->hrow[(r0 + 1) & 1];
            for (x = 0; x < HS_SNES_OUT_W; x++) dst[x] = mix32(a[x], b[x], w);
        }
    }
}

void hs_snes_render_video_phase(const hs_snes_slot *slot, hs_snes_video *v, uint32_t *fb)
{
    hs_snes_render_video_rows(slot, v, fb, 0, HS_SNES_OUT_H);
}
