/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gbc_ppu.c - clean-room DMG/CGB PPU.
 *
 * Timing side (Core 0): mode 2/3/0/1 state machine, STAT/VBlank IRQs,
 * HBlank HDMA trigger, and per-scanline register latching into the BACK
 * video snapshot. VRAM/OAM writes during the visible frame are appended
 * to a write log tagged with the first scanline they can affect.
 *
 * Render side (Core 1): replays base VRAM/OAM + log in scanline order out
 * of the FRONT snapshot and draws BG / window / sprites with full CGB
 * attributes, priorities and RGB555 palettes. Pure function of snapshot.
 */
#include "gbc_internal.h"
#include "gbc_compat_pal.h"

#include <string.h>

/* =================================================== timing side (Core 0) */
void gbc_ppu_stat_update(gbc_t *g)
{
    uint8_t st = g->io[0x41];
    int on = (g->io[0x40] & 0x80u) != 0;
    int line = on && (((st & 0x40u) && g->ly == g->io[0x45]) ||
                      ((st & 0x08u) && g->mode == 0) ||
                      ((st & 0x20u) && g->mode == 2) ||
                      ((st & 0x10u) && g->mode == 1));
    if (line && !g->stat_line)
        g->io[0x0F] |= IRQ_STAT;
    g->stat_line = (uint8_t)line;
}

static void begin_frame(gbc_t *g)
{
    gbc_vsnap_t *s = &g->vs[g->back];
    unsigned i;
    memcpy(s->vram, g->vram, sizeof(s->vram));
    memcpy(s->oam, g->oam, sizeof(s->oam));
    for (i = 0; i < GB_H; i++)
        s->line[i].latched = 0;
    s->vlog_count = 0;
    s->vlog_overflow = 0;
    s->lcd_on = 1;
    s->cgb = g->cgb;
    s->dmg_palette = (uint8_t)g->dmg_palette;
    s->dmg_pal_auto = g->dmg_pal_auto;
    memcpy(s->dmg_bg, g->dmg_bg, sizeof(s->dmg_bg));
    memcpy(s->dmg_obp0, g->dmg_obp0, sizeof(s->dmg_obp0));
    memcpy(s->dmg_obp1, g->dmg_obp1, sizeof(s->dmg_obp1));
    g->logging = 1;
    g->win_counter = 0;
    g->win_trig = 0;
}

void gbc_ppu_begin_step(gbc_t *g)
{
    gbc_vsnap_t *s = &g->vs[g->back];
    unsigned i;
    for (i = 0; i < GB_H; i++)
        s->line[i].latched = 0;
    s->vlog_count = 0;
    s->vlog_overflow = 0;
    s->lcd_on = 0;
    s->cgb = g->cgb;
    s->dmg_palette = (uint8_t)g->dmg_palette;
    s->dmg_pal_auto = g->dmg_pal_auto;
    memcpy(s->dmg_bg, g->dmg_bg, sizeof(s->dmg_bg));
    memcpy(s->dmg_obp0, g->dmg_obp0, sizeof(s->dmg_obp0));
    memcpy(s->dmg_obp1, g->dmg_obp1, sizeof(s->dmg_obp1));
    g->logging = 0;
    g->frame_done = 0;
    g->step_dots = 0;
    if ((g->io[0x40] & 0x80u) && g->ly < GB_H) {
        /* mid-frame start (power-on / LCD just enabled): capture from here */
        begin_frame(g);
    }
}

void gbc_ppu_reset(gbc_t *g)
{
    g->dot = 0; g->ly = 0; g->mode = 2; g->stat_line = 0;
    g->win_trig = 0; g->win_counter = 0; g->frame_done = 0;
    g->logging = 0; g->step_dots = 0;
}

static void latch_line(gbc_t *g)
{
    gbc_vsnap_t *s = &g->vs[g->back];
    gbc_line_t *L;
    uint8_t lcdc = g->io[0x40];
    if (!g->logging || g->ly >= GB_H)
        return;
    L = &s->line[g->ly];
    if (g->io[0x4A] == g->ly)
        g->win_trig = 1;
    L->latched = 1;
    L->lcdc = lcdc;
    L->scx = g->io[0x43];
    L->scy = g->io[0x42];
    L->wx = g->io[0x4B];
    L->bgp = g->io[0x47];
    L->obp0 = g->io[0x48];
    L->obp1 = g->io[0x49];
    L->opri = g->opri;
    L->win_draw = (uint8_t)((lcdc & 0x20u) && g->win_trig && g->io[0x4B] <= 166u &&
                            (g->cgb || (lcdc & 0x01u)));
    L->win_line = L->win_draw ? g->win_counter++ : 0;
    if (g->cgb) {
        memcpy(L->bgpal, g->bgpal, 64);
        memcpy(L->objpal, g->objpal, 64);
    }
}

static int log_line(const gbc_t *g)
{
    if (!g->logging || g->ly >= GB_H)
        return -1;
    if (g->mode == 2)
        return g->ly;
    return (g->ly + 1u < GB_H) ? (int)g->ly + 1 : -1;
}

void gbc_ppu_log_vram(gbc_t *g, uint16_t vaddr, uint8_t v)
{
    gbc_vsnap_t *s;
    int line = log_line(g);
    if (line < 0)
        return;
    s = &g->vs[g->back];
    if (s->vlog_count >= GBC_VLOG_MAX) {
        s->vlog_overflow = 1;
        return;
    }
    s->vlog[s->vlog_count].addr = vaddr;
    s->vlog[s->vlog_count].val = v;
    s->vlog[s->vlog_count].line = (uint8_t)line;
    s->vlog_count++;
}

void gbc_ppu_log_oam(gbc_t *g, uint16_t idx, uint8_t v)
{
    gbc_ppu_log_vram(g, (uint16_t)(0x4000u + idx), v);
}

void gbc_ppu_lcdc_write(gbc_t *g, uint8_t v)
{
    uint8_t old = g->io[0x40];
    g->io[0x40] = v;
    if ((old & 0x80u) && !(v & 0x80u)) {
        g->ly = 0; g->dot = 0; g->mode = 0;
        g->logging = 0;
        g->stat_line = 0;
    } else if (!(old & 0x80u) && (v & 0x80u)) {
        g->ly = 0; g->dot = 0; g->mode = 2;
        begin_frame(g);
        gbc_ppu_stat_update(g);
    }
}

void gbc_ppu_tick(gbc_t *g, uint32_t dots)
{
    g->step_dots += dots;
    if (!(g->io[0x40] & 0x80u))
        return;
    g->dot += dots;
    if (g->ly < GB_H) {
        if (g->mode == 2 && g->dot >= 80u) {
            g->mode = 3;
            latch_line(g);
            gbc_ppu_stat_update(g);
        }
        if (g->mode == 3 && g->dot >= 252u) {
            g->mode = 0;
            if (g->hdma_active)
                g->hdma_block_pending = 1;
            gbc_ppu_stat_update(g);
        }
    }
    if (g->dot >= GB_DOTS_PER_LINE) {
        g->dot -= GB_DOTS_PER_LINE;
        g->ly++;
        if (g->ly == GB_H) {
            g->mode = 1;
            g->io[0x0F] |= IRQ_VBLANK;
            g->frame_done = 1;
            g->logging = 0;
        } else if (g->ly >= GB_LINES) {
            g->ly = 0;
            g->mode = 2;
            begin_frame(g);
        } else if (g->ly < GB_H) {
            g->mode = 2;
        }
        gbc_ppu_stat_update(g);
    }
}

/* =================================================== render side (Core 1) */
static const uint32_t dmg_pals[4][4] = {
    { 0x9BBC0Fu, 0x8BAC0Fu, 0x306230u, 0x0F380Fu },  /* classic olive */
    { 0xE0E0D8u, 0xA8A8A0u, 0x585850u, 0x101010u },  /* pocket b&w   */
    { 0xF0E0FFu, 0xB088D8u, 0x6A3D9Au, 0x2A0A4Au },  /* deep purple  */
    { 0xB8FFB0u, 0x50D050u, 0x0B7A20u, 0x001A06u },  /* matrix green */
};

void gbc_refresh_dmg_palettes(gbc_t *g)
{
    int i;
    if (g->dmg_pal_auto && g->rom_loaded && g->rom != NULL && !g->cgb) {
        struct hs_dmg_compat c;
        hs_dmg_compat_from_rom(g->rom, g->rom_size, &c);
        for (i = 0; i < 4; i++) {
            g->dmg_bg[i]   = hs_cgb_lcd_rgb888(c.bg[i]);
            g->dmg_obp0[i] = hs_cgb_lcd_rgb888(c.obp0[i]);
            g->dmg_obp1[i] = hs_cgb_lcd_rgb888(c.obp1[i]);
        }
    } else {
        const uint32_t *dp = dmg_pals[g->dmg_palette & 3u];
        for (i = 0; i < 4; i++) {
            g->dmg_bg[i] = g->dmg_obp0[i] = g->dmg_obp1[i] = dp[i];
        }
    }
}


static inline uint32_t rgb555(uint8_t lo, uint8_t hi)
{
    uint32_t c = (uint32_t)lo | ((uint32_t)hi << 8);
    uint32_t r = c & 31u, gg = (c >> 5) & 31u, b = (c >> 10) & 31u;
    r = (r << 3) | (r >> 2);
    gg = (gg << 3) | (gg >> 2);
    b = (b << 3) | (b >> 2);
    return (r << 16) | (gg << 8) | b;
}

static inline uint16_t tile_addr(uint8_t lcdc, uint8_t idx)
{
    if (lcdc & 0x10u)
        return (uint16_t)(idx * 16u);
    return (uint16_t)(0x1000 + (int)(int8_t)idx * 16);
}

/* Fetch one BG/window map row's pixels for x in [x0, x1). */
static void draw_bg_span(const uint8_t *vram, const gbc_line_t *L, int cgb, uint16_t map,
                         uint32_t px_y, int src_x, int x0, int x1,
                         uint8_t *col, uint8_t *attr)
{
    int x = x0;
    uint32_t row = (px_y >> 3) & 31u, fine = px_y & 7u;
    while (x < x1) {
        uint32_t sx = (uint32_t)(src_x + (x - x0)) & 255u;
        uint32_t tcol = sx >> 3;
        uint16_t moff = (uint16_t)(map + row * 32u + tcol);
        uint8_t idx = vram[moff];
        uint8_t at = cgb ? vram[0x2000u + moff] : 0;
        uint32_t fy = (at & 0x40u) ? 7u - fine : fine;
        uint16_t ta = (uint16_t)(tile_addr(L->lcdc, idx) + fy * 2u + ((at & 0x08u) ? 0x2000u : 0u));
        uint8_t lo = vram[ta], hi = vram[ta + 1u];
        uint32_t px = sx & 7u;
        for (; px < 8u && x < x1; px++, x++) {
            uint32_t b = (at & 0x20u) ? px : 7u - px;
            col[x] = (uint8_t)((((hi >> b) & 1u) << 1) | ((lo >> b) & 1u));
            attr[x] = at;
        }
    }
}

static void render_line(const gbc_vsnap_t *s, const gbc_line_t *L, const uint8_t *vram,
                        const uint8_t *oam, uint32_t ly, uint32_t *out)
{
    uint8_t col[GB_W], attr[GB_W];
    uint8_t scol[GB_W], sattr[GB_W];
    int cgb = s->cgb;
    uint8_t lcdc = L->lcdc;
    const uint32_t *dp_bg = s->dmg_bg;
    const uint32_t *dp_obp0 = s->dmg_obp0;
    const uint32_t *dp_obp1 = s->dmg_obp1;
    int x;

    memset(col, 0, sizeof(col));
    memset(attr, 0, sizeof(attr));
    memset(scol, 0, sizeof(scol));
    memset(sattr, 0, sizeof(sattr));

    if (cgb || (lcdc & 0x01u)) {
        int wx0 = L->win_draw ? (int)L->wx - 7 : GB_W;
        int bg_end = wx0 < 0 ? 0 : (wx0 > GB_W ? GB_W : wx0);
        uint16_t bgmap = (lcdc & 0x08u) ? 0x1C00u : 0x1800u;
        if (bg_end > 0)
            draw_bg_span(vram, L, cgb, bgmap, (ly + L->scy) & 255u, L->scx, 0, bg_end, col, attr);
        if (L->win_draw && bg_end < GB_W) {
            uint16_t wmap = (lcdc & 0x40u) ? 0x1C00u : 0x1800u;
            int skip = wx0 < 0 ? -wx0 : 0;
            draw_bg_span(vram, L, cgb, wmap, L->win_line, skip, bg_end, GB_W, col, attr);
        }
    }

    if (lcdc & 0x02u) {
        uint32_t h = (lcdc & 0x04u) ? 16u : 8u;
        uint8_t sel[10];
        int n = 0, i, j;
        for (i = 0; i < 40 && n < 10; i++) {
            int sy = (int)oam[i * 4] - 16;
            if ((int)ly >= sy && (int)ly < sy + (int)h)
                sel[n++] = (uint8_t)i;
        }
        /* DMG (or CGB with OPRI=1): lower X first, then OAM index */
        if (!cgb || (L->opri & 1u)) {
            for (i = 1; i < n; i++) {
                uint8_t k = sel[i];
                j = i - 1;
                while (j >= 0 && oam[sel[j] * 4 + 1] > oam[k * 4 + 1]) {
                    sel[j + 1] = sel[j];
                    j--;
                }
                sel[j + 1] = k;
            }
        }
        for (i = 0; i < n; i++) {
            const uint8_t *o = &oam[sel[i] * 4];
            int sy = (int)o[0] - 16, sx = (int)o[1] - 8;
            uint8_t tile = o[2], fl = o[3];
            uint32_t r = ly - (uint32_t)sy;
            uint16_t ta;
            uint8_t lo, hi;
            int px;
            if (h == 16u) tile &= 0xFEu;
            if (fl & 0x40u) r = h - 1u - r;
            ta = (uint16_t)(tile * 16u + r * 2u + ((cgb && (fl & 0x08u)) ? 0x2000u : 0u));
            lo = vram[ta]; hi = vram[ta + 1u];
            for (px = 0; px < 8; px++) {
                int xx = sx + px;
                uint32_t b = (fl & 0x20u) ? (uint32_t)px : 7u - (uint32_t)px;
                uint8_t c;
                if (xx < 0 || xx >= GB_W || scol[xx])
                    continue;
                c = (uint8_t)((((hi >> b) & 1u) << 1) | ((lo >> b) & 1u));
                if (!c)
                    continue;
                scol[xx] = c;
                sattr[xx] = fl;
            }
        }
    }

    for (x = 0; x < GB_W; x++) {
        uint8_t c = col[x];
        int use_obj = 0;
        if (scol[x]) {
            if (cgb) {
                if (!(lcdc & 0x01u)) use_obj = 1;
                else if (c && ((attr[x] & 0x80u) || (sattr[x] & 0x80u))) use_obj = 0;
                else use_obj = 1;
            } else {
                use_obj = !(c && (sattr[x] & 0x80u));
            }
        }
        if (cgb) {
            if (use_obj) {
                const uint8_t *p = &L->objpal[(sattr[x] & 7u) * 8u + scol[x] * 2u];
                out[x] = rgb555(p[0], p[1]);
            } else {
                const uint8_t *p = &L->bgpal[(attr[x] & 7u) * 8u + c * 2u];
                out[x] = rgb555(p[0], p[1]);
            }
        } else {
            uint8_t shade;
            if (use_obj) {
                uint8_t pal = (sattr[x] & 0x10u) ? L->obp1 : L->obp0;
                shade = (uint8_t)((pal >> (scol[x] * 2u)) & 3u);
            } else {
                shade = (uint8_t)((L->bgp >> (c * 2u)) & 3u);
            }
            out[x] = use_obj ? ((sattr[x] & 0x10u) ? dp_obp1[shade] : dp_obp0[shade]) : dp_bg[shade];
        }
    }
}

void gbc_ppu_render_native(const gbc_vsnap_t *s, uint8_t *vram, uint8_t *oam, uint32_t *out)
{
    uint32_t ly, k = 0, x;
    uint32_t blank = s->cgb ? 0xFFFFFFu : s->dmg_bg[0];
    uint32_t n = s->vlog_count;
    if (n > GBC_VLOG_MAX)
        n = GBC_VLOG_MAX;
    memcpy(vram, s->vram, 0x4000u);
    memcpy(oam, s->oam, 0xA0u);
    for (ly = 0; ly < GB_H; ly++) {
        const gbc_line_t *L = &s->line[ly];
        uint32_t *row = out + ly * GB_W;
        while (k < n && s->vlog[k].line <= ly) {
            const gbc_vlog_t *e = &s->vlog[k++];
            if (e->addr < 0x4000u) vram[e->addr] = e->val;
            else if (e->addr < 0x40A0u) oam[e->addr - 0x4000u] = e->val;
        }
        if (!s->lcd_on || !L->latched) {
            for (x = 0; x < GB_W; x++)
                row[x] = blank;
            continue;
        }
        render_line(s, L, vram, oam, ly, row);
    }
}
