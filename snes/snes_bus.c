/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_bus.c — clean-room SNES memory map, cartridge mapper detection
 * (LoROM / HiROM / ExHiROM by header scoring), 128 KiB WRAM + $2180 port,
 * battery SRAM, hardware multiplier/divider, 8-channel general DMA + HDMA,
 * joypad auto-read and manual strobe, NMI/IRQ status registers.
 * Region access speeds follow the public SNES timing description
 * (6 / 8 / 12 master cycles, FastROM via MEMSEL).
 */
#include <string.h>
#include "snes_internal.h"

static const uint8_t k_dma_off[8][4] = {
    {0, 0, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 0}, {0, 0, 1, 1},
    {0, 1, 2, 3}, {0, 1, 0, 1}, {0, 0, 0, 0}, {0, 0, 1, 1}
};
static const uint8_t k_dma_len[8] = {1, 2, 2, 4, 4, 4, 2, 4};

/* ------------------------------------------------------------- cartridge */
static uint32_t rom_mirror(uint32_t off, uint32_t size)
{
    uint32_t base = 0, bit = 1u << 23;
    unsigned guard;
    if (size == 0) return 0;
    for (guard = 0; guard < 64 && off >= size; guard++) {
        while (bit && !(off & bit)) bit >>= 1;
        if (!bit) break;
        off -= bit;
        if (size > bit) { size -= bit; base += bit; }
        bit >>= 1;
    }
    return base + off;
}

static inline uint8_t rom_at(const hs_snes *s, uint32_t off)
{
    off &= 0x7FFFFFu;
    return s->rom[s->rom_chunk[off >> 15] + (off & 0x7FFFu)];
}

static int is_print(uint8_t c) { return c >= 0x20 && c < 0x7F; }

static int score_header(const uint8_t *rom, size_t size, uint32_t h, int mapper)
{
    int score = 0, i, printable = 1;
    uint8_t map;
    uint16_t chk, cmp, reset;
    uint32_t roff;
    if ((size_t)h + 0x40u > size) return -1000;
    map = rom[h + 0x15];
    chk = (uint16_t)(rom[h + 0x1E] | (rom[h + 0x1F] << 8));
    cmp = (uint16_t)(rom[h + 0x1C] | (rom[h + 0x1D] << 8));
    reset = (uint16_t)(rom[h + 0x3C] | (rom[h + 0x3D] << 8));
    if ((uint16_t)(chk ^ cmp) == 0xFFFFu) score += 4;
    if (mapper == HS_SNES_MAP_LOROM && (map & 0xEFu) == 0x20u) score += 3;
    if (mapper == HS_SNES_MAP_HIROM && (map & 0xEFu) == 0x21u) score += 3;
    if (mapper == HS_SNES_MAP_EXHIROM && (map & 0xEFu) == 0x25u) score += 3;
    if (mapper == HS_SNES_MAP_LOROM && (map & 0xEFu) == 0x22u) score += 2;   /* S-DD1/SA-1 style */
    if (reset >= 0x8000u) score += 3; else score -= 4;
    for (i = 0; i < 21; i++) if (!is_print(rom[h + (uint32_t)i])) printable = 0;
    if (printable) score += 1;
    if (rom[h + 0x17] >= 7 && rom[h + 0x17] <= 13) score += 1;
    if (mapper == HS_SNES_MAP_LOROM) roff = (uint32_t)reset - 0x8000u;
    else if (mapper == HS_SNES_MAP_HIROM) roff = reset;
    else roff = 0x400000u + reset;
    if (reset >= 0x8000u && roff < size) {
        uint8_t op = rom[roff];
        if (op == 0x78 || op == 0x18 || op == 0x38 || op == 0x5C || op == 0x4C ||
            op == 0xC2 || op == 0xE2 || op == 0x9C || op == 0xA9 || op == 0xA2 ||
            op == 0x20 || op == 0x22 || op == 0x8D || op == 0xAD) score += 2;
    }
    return score;
}

int snes_cart_load(hs_snes *s, const uint8_t *rom, size_t size)
{
    int best = -1, best_score = -1000, m, i;
    uint32_t h, k;
    static const uint32_t hdr_at[3] = {0x7FC0u, 0xFFC0u, 0x40FFC0u};
    if (!rom || size < 0x8000u || size > 0x800000u + 512u) return -1;
    if ((size & 0x3FFu) == 0x200u) { rom += 0x200; size -= 0x200; }  /* copier header */
    for (m = 0; m < 3; m++) {
        int sc = score_header(rom, size, hdr_at[m], m);
        if (sc > best_score) { best_score = sc; best = m; }
    }
    if (best < 0) return -1;
    s->rom = rom;
    s->rom_size = (uint32_t)size;
    s->mapper = (uint8_t)best;
    for (k = 0; k < 256; k++) s->rom_chunk[k] = rom_mirror(k << 15, s->rom_size) & ~0x7FFFu;
    /* a non-32K-multiple tail is handled by mirror of chunk starts; clamp */
    for (k = 0; k < 256; k++) if (s->rom_chunk[k] + 0x8000u > s->rom_size) s->rom_chunk[k] = 0;
    h = hdr_at[best];
    for (i = 0; i < 21; i++) {
        uint8_t c = rom[h + (uint32_t)i];
        s->title[i] = is_print(c) ? c : (uint8_t)'?';
    }
    s->title[21] = 0;
    s->map_byte = rom[h + 0x15];
    s->chip_type = rom[h + 0x16];
    s->fastrom = (s->map_byte & 0x10u) != 0;
    s->sram_size = rom[h + 0x18] ? (1024u << (rom[h + 0x18] & 0x0Fu)) : 0;
    if (s->sram_size > sizeof s->sram) s->sram_size = sizeof s->sram;
    return 0;
}

const char *hs_snes_chip_name(const hs_snes *s)
{
    uint8_t t = s->chip_type, m = s->map_byte;
    if (t <= 0x02) return "none";
    if (t >= 0x03 && t <= 0x05 && (m & 0x0F) != 0x05) return "DSP-n (unsupported)";
    if (t >= 0x13 && t <= 0x1A) return "SuperFX/GSU (unsupported)";
    if (t >= 0x33 && t <= 0x36) return "SA-1 (unsupported)";
    if (t >= 0x43 && t <= 0x45) return "S-DD1 (unsupported)";
    if (t == 0x55) return "S-RTC (unsupported)";
    if (t == 0xE3) return "Super Game Boy (unsupported)";
    if (t == 0xF3) return "CX4 (unsupported)";
    if (t == 0xF5 || t == 0xF9) return "SPC7110 (unsupported)";
    if (t == 0xF6) return "ST010/011 (unsupported)";
    if (t >= 0xF0) return "custom coprocessor (unsupported)";
    return "none";
}

/* --------------------------------------------------------------- mapping */
static inline unsigned access_speed(const hs_snes *s, uint32_t a)
{
    uint8_t bank = (uint8_t)(a >> 16);
    uint16_t off = (uint16_t)a;
    unsigned fast = (s->memsel & 1u) ? 6u : 8u;
    if (bank >= 0x40u && bank < 0x80u) return 8u;
    if (bank >= 0xC0u) return fast;
    if (off >= 0x8000u) return (bank & 0x80u) ? fast : 8u;
    if (off < 0x2000u) return 8u;
    if (off < 0x4000u) return 6u;
    if (off < 0x4200u) return 12u;
    if (off < 0x6000u) return 6u;
    return 8u;
}

static int sram_index(const hs_snes *s, uint8_t bank, uint16_t off, uint32_t *idx)
{
    uint32_t mask;
    if (!s->sram_size) return 0;
    mask = s->sram_size - 1u;
    if (s->mapper == HS_SNES_MAP_LOROM) {
        if ((bank & 0x7Fu) >= 0x70u && (bank & 0x7Fu) < 0x7Eu && off < 0x8000u) {
            *idx = ((((uint32_t)bank & 0x0Fu) << 15) | off) & mask;
            return 1;
        }
        return 0;
    }
    if ((bank & 0x7Fu) >= 0x20u && (bank & 0x7Fu) < 0x40u && off >= 0x6000u && off < 0x8000u) {
        *idx = ((((uint32_t)bank & 0x1Fu) << 13) | (off - 0x6000u)) & mask;
        return 1;
    }
    return 0;
}

static int rom_index(const hs_snes *s, uint8_t bank, uint16_t off, uint32_t *idx)
{
    switch (s->mapper) {
    case HS_SNES_MAP_LOROM:
        if (off >= 0x8000u) { *idx = (((uint32_t)bank & 0x7Fu) << 15) | (off & 0x7FFFu); return 1; }
        if ((bank & 0x7Fu) >= 0x40u && (bank & 0x7Fu) < 0x70u) { *idx = (((uint32_t)bank & 0x7Fu) << 15) | (off & 0x7FFFu); return 1; }
        return 0;
    case HS_SNES_MAP_HIROM:
        if ((bank & 0x7Fu) >= 0x40u || off >= 0x8000u) { *idx = (((uint32_t)bank & 0x3Fu) << 16) | off; return 1; }
        return 0;
    default: /* ExHiROM */
        if ((bank & 0x7Fu) >= 0x40u || off >= 0x8000u) {
            *idx = (((uint32_t)bank & 0x3Fu) << 16) | off;
            if (!(bank & 0x80u)) *idx += 0x400000u;
            return 1;
        }
        return 0;
    }
}

static uint8_t cart_read(const hs_snes *s, uint8_t bank, uint16_t off, uint8_t open)
{
    uint32_t idx;
    if (sram_index(s, bank, off, &idx)) return s->sram[idx];
    if (rom_index(s, bank, off, &idx)) return rom_at(s, idx);
    return open;
}

uint8_t snes_bus_peek(const hs_snes *s, uint32_t addr)
{
    uint8_t bank = (uint8_t)(addr >> 16);
    uint16_t off = (uint16_t)addr;
    if (bank == 0x7Eu || bank == 0x7Fu) return s->wram[(((uint32_t)bank & 1u) << 16) | off];
    if ((bank & 0x7Fu) < 0x40u && off < 0x2000u) return s->wram[off];
    return cart_read(s, bank, off, 0);
}

/* --------------------------------------------------------------- DMA/IO */
static uint8_t io_read(hs_snes *s, uint16_t off);
static void io_write(hs_snes *s, uint16_t off, uint8_t v);

static uint8_t bbus_read(hs_snes *s, uint8_t b)
{
    if (b < 0x40u) return snes_ppu_read(s, b);
    if (b < 0x80u) return snes_apu_cpu_read(s, b & 3u);
    if (b == 0x80u) { uint8_t v = s->wram[s->wram_addr]; s->wram_addr = (s->wram_addr + 1u) & 0x1FFFFu; return v; }
    return s->mdr;
}
static void bbus_write(hs_snes *s, uint8_t b, uint8_t v)
{
    if (b < 0x40u) { snes_ppu_write(s, b, v); return; }
    if (b < 0x80u) { snes_apu_cpu_write(s, b & 3u, v); return; }
    switch (b) {
    case 0x80: s->wram[s->wram_addr] = v; s->wram_addr = (s->wram_addr + 1u) & 0x1FFFFu; break;
    case 0x81: s->wram_addr = (s->wram_addr & 0x1FF00u) | v; break;
    case 0x82: s->wram_addr = (s->wram_addr & 0x100FFu) | ((uint32_t)v << 8); break;
    case 0x83: s->wram_addr = (s->wram_addr & 0x0FFFFu) | (((uint32_t)v & 1u) << 16); break;
    default: break;
    }
}

/* A-bus side of DMA: no B-bus / DMA register access, no cycle charge */
static int abus_invalid(uint32_t a)
{
    uint16_t off = (uint16_t)a;
    if (((a >> 16) & 0x7Fu) >= 0x40u) return 0;
    return (off >= 0x2100u && off < 0x2200u) || (off >= 0x4000u && off < 0x4400u);
}
static uint8_t abus_read(hs_snes *s, uint32_t a)
{
    if (abus_invalid(a)) return s->mdr;
    return snes_bus_peek(s, a);
}
static void abus_write(hs_snes *s, uint32_t a, uint8_t v)
{
    uint8_t bank = (uint8_t)(a >> 16);
    uint16_t off = (uint16_t)a;
    uint32_t idx;
    if (abus_invalid(a)) return;
    if (bank == 0x7Eu || bank == 0x7Fu) { s->wram[(((uint32_t)bank & 1u) << 16) | off] = v; return; }
    if ((bank & 0x7Fu) < 0x40u && off < 0x2000u) { s->wram[off] = v; return; }
    if (sram_index(s, bank, off, &idx)) s->sram[idx] = v;
}

static void run_dma(hs_snes *s, uint8_t mask)
{
    unsigned ch;
    s->master += 8;
    for (ch = 0; ch < 8; ch++) {
        snes_dma_ch *d = &s->dma[ch];
        unsigned mode, i = 0;
        uint32_t count;
        if (!(mask & (1u << ch))) continue;
        s->master += 8;
        mode = d->dmap & 7u;
        count = d->das ? d->das : 0x10000u;
        while (count) {
            uint8_t b = (uint8_t)(d->bbad + k_dma_off[mode][i & 3u]);
            uint32_t a = ((uint32_t)d->a1b << 16) | d->a1t;
            if (d->dmap & 0x80u) abus_write(s, a, bbus_read(s, b));
            else bbus_write(s, b, abus_read(s, a));
            s->master += 8;
            if (!(d->dmap & 0x08u)) d->a1t = (uint16_t)((d->dmap & 0x10u) ? d->a1t - 1u : d->a1t + 1u);
            i++;
            count--;
        }
        d->das = 0;
    }
}

void snes_hdma_init(hs_snes *s)
{
    unsigned ch;
    for (ch = 0; ch < 8; ch++) {
        snes_dma_ch *d = &s->dma[ch];
        d->hdma_active = 0;
        d->hdma_do = 0;
        if (!(s->hdmaen & (1u << ch))) continue;
        d->a2a = d->a1t;
        d->nltr = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a);
        d->a2a++;
        if (d->dmap & 0x40u) {
            uint8_t lo = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a); d->a2a++;
            uint8_t hi = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a); d->a2a++;
            d->das = (uint16_t)(lo | (hi << 8));
            s->master += 16;
        }
        s->master += 8;
        if (d->nltr) { d->hdma_active = 1; d->hdma_do = 1; }
    }
    s->master += 18;
}

void snes_hdma_line(hs_snes *s)
{
    unsigned ch, any = 0;
    for (ch = 0; ch < 8; ch++) {
        snes_dma_ch *d = &s->dma[ch];
        if (!(s->hdmaen & (1u << ch)) || !d->hdma_active) continue;
        any = 1;
        s->master += 8;
        if (d->hdma_do) {
            unsigned mode = d->dmap & 7u, i;
            for (i = 0; i < k_dma_len[mode]; i++) {
                uint32_t a;
                uint8_t b = (uint8_t)(d->bbad + k_dma_off[mode][i]);
                if (d->dmap & 0x40u) { a = ((uint32_t)d->dasb << 16) | d->das; d->das++; }
                else { a = ((uint32_t)d->a1b << 16) | d->a2a; d->a2a++; }
                if (d->dmap & 0x80u) abus_write(s, a, bbus_read(s, b));
                else bbus_write(s, b, abus_read(s, a));
                s->master += 8;
            }
        }
        d->nltr = (uint8_t)((d->nltr & 0x80u) | ((d->nltr - 1u) & 0x7Fu));
        d->hdma_do = (d->nltr & 0x80u) != 0;
        if ((d->nltr & 0x7Fu) == 0) {
            d->nltr = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a);
            d->a2a++;
            if (d->dmap & 0x40u) {
                uint8_t lo = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a); d->a2a++;
                uint8_t hi = abus_read(s, ((uint32_t)d->a1b << 16) | d->a2a); d->a2a++;
                d->das = (uint16_t)(lo | (hi << 8));
                s->master += 16;
            }
            if (d->nltr == 0) d->hdma_active = 0;
            d->hdma_do = 1;
        }
    }
    if (any) s->master += 18;
}

void snes_autojoy(hs_snes *s)
{
    s->joy_auto[0] = s->pad[0];
    s->joy_auto[1] = s->pad[1];
    s->joy_auto[2] = 0;
    s->joy_auto[3] = 0;
    s->joy_shift[0] = 0xFFFF0000u;   /* serial port reads 1s after auto-read */
    s->joy_shift[1] = 0xFFFF0000u;
}

static uint8_t joy_serial(hs_snes *s, int port)
{
    uint8_t bit;
    if (s->joy_strobe) return (uint8_t)((s->mdr & 0xFCu) | ((s->pad[port] >> 15) & 1u));
    bit = (uint8_t)((s->joy_shift[port] >> 15) & 1u);
    s->joy_shift[port] = (s->joy_shift[port] << 1) | 1u;
    return (uint8_t)((s->mdr & 0xFCu) | bit);
}

static uint8_t io_read(hs_snes *s, uint16_t off)
{
    uint8_t v;
    if (off >= 0x4300u && off < 0x4380u) {
        snes_dma_ch *d = &s->dma[(off >> 4) & 7u];
        switch (off & 0x0Fu) {
        case 0x0: return d->dmap;
        case 0x1: return d->bbad;
        case 0x2: return (uint8_t)d->a1t;
        case 0x3: return (uint8_t)(d->a1t >> 8);
        case 0x4: return d->a1b;
        case 0x5: return (uint8_t)d->das;
        case 0x6: return (uint8_t)(d->das >> 8);
        case 0x7: return d->dasb;
        case 0x8: return (uint8_t)d->a2a;
        case 0x9: return (uint8_t)(d->a2a >> 8);
        case 0xA: return d->nltr;
        case 0xB: case 0xF: return d->unused;
        default: return s->mdr;
        }
    }
    switch (off) {
    case 0x4016: return joy_serial(s, 0);
    case 0x4017: return (uint8_t)(joy_serial(s, 1) | 0x1Cu);
    case 0x4210: v = (uint8_t)((s->nmi_flag ? 0x80u : 0u) | 0x02u | (s->mdr & 0x70u)); s->nmi_flag = 0; return v;
    case 0x4211: v = (uint8_t)((s->irq_flag ? 0x80u : 0u) | (s->mdr & 0x7Fu)); s->irq_flag = 0; s->cpu.irq_line = 0; return v;
    case 0x4212: {
        uint64_t hp = s->master - s->line_start;
        v = (uint8_t)((s->in_vblank ? 0x80u : 0u) | ((hp >= 1096u || hp < 4u) ? 0x40u : 0u) |
                      (s->autojoy_busy ? 1u : 0u) | (s->mdr & 0x3Eu));
        return v;
    }
    case 0x4213: return s->wrio;
    case 0x4214: return (uint8_t)s->rddiv;
    case 0x4215: return (uint8_t)(s->rddiv >> 8);
    case 0x4216: return (uint8_t)s->rdmpy;
    case 0x4217: return (uint8_t)(s->rdmpy >> 8);
    case 0x4218: case 0x421A: case 0x421C: case 0x421E: return (uint8_t)s->joy_auto[(off - 0x4218u) >> 1];
    case 0x4219: case 0x421B: case 0x421D: case 0x421F: return (uint8_t)(s->joy_auto[(off - 0x4218u) >> 1] >> 8);
    default: return s->mdr;
    }
}

static void io_write(hs_snes *s, uint16_t off, uint8_t v)
{
    if (off >= 0x4300u && off < 0x4380u) {
        snes_dma_ch *d = &s->dma[(off >> 4) & 7u];
        switch (off & 0x0Fu) {
        case 0x0: d->dmap = v; break;
        case 0x1: d->bbad = v; break;
        case 0x2: d->a1t = (uint16_t)((d->a1t & 0xFF00u) | v); break;
        case 0x3: d->a1t = (uint16_t)((d->a1t & 0x00FFu) | (v << 8)); break;
        case 0x4: d->a1b = v; break;
        case 0x5: d->das = (uint16_t)((d->das & 0xFF00u) | v); break;
        case 0x6: d->das = (uint16_t)((d->das & 0x00FFu) | (v << 8)); break;
        case 0x7: d->dasb = v; break;
        case 0x8: d->a2a = (uint16_t)((d->a2a & 0xFF00u) | v); break;
        case 0x9: d->a2a = (uint16_t)((d->a2a & 0x00FFu) | (v << 8)); break;
        case 0xA: d->nltr = v; break;
        case 0xB: case 0xF: d->unused = v; break;
        default: break;
        }
        return;
    }
    switch (off) {
    case 0x4016: {
        uint8_t ns = v & 1u;
        if (s->joy_strobe && !ns) {
            s->joy_shift[0] = (uint32_t)s->pad[0] | 0xFFFF0000u;
            s->joy_shift[1] = (uint32_t)s->pad[1] | 0xFFFF0000u;
        }
        s->joy_strobe = ns;
        break;
    }
    case 0x4200:
        if (!(s->nmitimen & 0x80u) && (v & 0x80u) && s->nmi_flag) s->cpu.nmi_pending = 1;
        s->nmitimen = v;
        if (!(v & 0x30u)) { s->irq_flag = 0; s->cpu.irq_line = 0; }
        break;
    case 0x4201: s->wrio = v; break;
    case 0x4202: s->wrmpya = v; break;
    case 0x4203: s->wrmpyb = v; s->rdmpy = (uint16_t)(s->wrmpya * v); break;
    case 0x4204: s->wrdiv = (uint16_t)((s->wrdiv & 0xFF00u) | v); break;
    case 0x4205: s->wrdiv = (uint16_t)((s->wrdiv & 0x00FFu) | (v << 8)); break;
    case 0x4206:
        if (v) { s->rddiv = (uint16_t)(s->wrdiv / v); s->rdmpy = (uint16_t)(s->wrdiv % v); }
        else { s->rddiv = 0xFFFFu; s->rdmpy = s->wrdiv; }
        break;
    case 0x4207: s->htime = (uint16_t)((s->htime & 0x100u) | v); break;
    case 0x4208: s->htime = (uint16_t)((s->htime & 0x0FFu) | ((v & 1u) << 8)); break;
    case 0x4209: s->vtime = (uint16_t)((s->vtime & 0x100u) | v); break;
    case 0x420A: s->vtime = (uint16_t)((s->vtime & 0x0FFu) | ((v & 1u) << 8)); break;
    case 0x420B: run_dma(s, v); break;
    case 0x420C: s->hdmaen = v; break;
    case 0x420D: s->memsel = v; break;
    default: break;
    }
}

static uint8_t bus_read(hs_snes *s, uint32_t addr)
{
    uint8_t bank = (uint8_t)(addr >> 16);
    uint16_t off = (uint16_t)addr;
    if ((bank & 0x7Fu) < 0x40u) {
        if (off < 0x2000u) return s->wram[off];
        if (off < 0x2100u) return s->mdr;
        if (off < 0x2140u) return snes_ppu_read(s, (uint8_t)off);
        if (off < 0x2180u) return snes_apu_cpu_read(s, off & 3u);
        if (off == 0x2180u) return bbus_read(s, 0x80);
        if (off < 0x4000u) return s->mdr;
        if (off < 0x4400u) return io_read(s, off);
        if (off < 0x6000u) return s->mdr;
        return cart_read(s, bank, off, s->mdr);
    }
    if (bank == 0x7Eu || bank == 0x7Fu) return s->wram[(((uint32_t)bank & 1u) << 16) | off];
    return cart_read(s, bank, off, s->mdr);
}

static void bus_write(hs_snes *s, uint32_t addr, uint8_t v)
{
    uint8_t bank = (uint8_t)(addr >> 16);
    uint16_t off = (uint16_t)addr;
    uint32_t idx;
    if ((bank & 0x7Fu) < 0x40u) {
        if (off < 0x2000u) { s->wram[off] = v; return; }
        if (off >= 0x2100u && off < 0x2184u) { bbus_write(s, (uint8_t)off, v); return; }
        if (off >= 0x4000u && off < 0x4400u) { io_write(s, off, v); return; }
    } else if (bank == 0x7Eu || bank == 0x7Fu) {
        s->wram[(((uint32_t)bank & 1u) << 16) | off] = v;
        return;
    }
    if (sram_index(s, bank, off, &idx)) s->sram[idx] = v;
}

uint8_t snes_cpu_read(hs_snes *s, uint32_t addr)
{
    uint8_t v;
    s->master += access_speed(s, addr);
    v = bus_read(s, addr);
    s->mdr = v;
    return v;
}

void snes_cpu_write(hs_snes *s, uint32_t addr, uint8_t val)
{
    s->master += access_speed(s, addr);
    s->mdr = val;
    bus_write(s, addr, val);
}

void snes_cpu_idle(hs_snes *s) { s->master += 6; }

void snes_bus_reset(hs_snes *s)
{
    memset(s->wram, 0x55, sizeof s->wram);   /* deterministic power-on pattern */
    memset(s->dma, 0xFF, sizeof s->dma);
    {
        unsigned ch;
        for (ch = 0; ch < 8; ch++) { s->dma[ch].hdma_active = 0; s->dma[ch].hdma_do = 0; }
    }
    s->wram_addr = 0;
    s->hdmaen = 0;
    s->nmitimen = 0; s->wrio = 0xFF; s->wrmpya = 0xFF; s->wrmpyb = 0xFF;
    s->wrdiv = 0xFFFF; s->rddiv = 0; s->rdmpy = 0;
    s->htime = 0x1FF; s->vtime = 0x1FF;
    s->memsel = 0;
    s->nmi_flag = 0; s->irq_flag = 0; s->autojoy_busy = 0;
    memset(s->joy_auto, 0, sizeof s->joy_auto);
    s->joy_shift[0] = s->joy_shift[1] = 0xFFFFFFFFu;
    s->joy_strobe = 0;
    s->mdr = 0;
}
