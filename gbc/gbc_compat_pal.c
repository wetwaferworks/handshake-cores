/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 HandshakeOS contributors */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "gbc_compat_pal.h"

#include <string.h>

/* Title checksums, $0134-$0143 8-bit sum. Index 0..64 are unique; 65..93
 * share a checksum and are disambiguated by the fourth title letter ($0137)
 * against k_fourth[i-65]. Order and values: Pan Docs / Liji tables. */
static const uint8_t k_checksums[94] = {
    0x00, 0x88, 0x16, 0x36, 0xD1, 0xDB, 0xF2, 0x3C, 0x8C, 0x92,
    0x3D, 0x5C, 0x58, 0xC9, 0x3E, 0x70, 0x1D, 0x59, 0x69, 0x19,
    0x35, 0xA8, 0x14, 0xAA, 0x75, 0x95, 0x99, 0x34, 0x6F, 0x15,
    0xFF, 0x97, 0x4B, 0x90, 0x17, 0x10, 0x39, 0xF7, 0xF6, 0xA2,
    0x49, 0x4E, 0x43, 0x68, 0xE0, 0x8B, 0xF0, 0xCE, 0x0C, 0x29,
    0xE8, 0xB7, 0x86, 0x9A, 0x52, 0x01, 0x9D, 0x71, 0x9C, 0xBD,
    0x5D, 0x6D, 0x67, 0x3F, 0x6B,
    /* FirstChecksumWithDuplicate (indices 65-93), 4th-letter rows of 14. */
    0xB3, 0x46, 0x28, 0xA5, 0xC6, 0xD3, 0x27, 0x61, 0x18, 0x66,
    0x6A, 0xBF, 0x0D, 0xF4,
    0xB3, 0x46, 0x28, 0xA5, 0xC6, 0xD3, 0x27, 0x61, 0x18, 0x66,
    0x6A, 0xBF, 0x0D, 0xF4,
    0xB3
};

/* Parallel to k_checksums[65..93]. "BEFAARBEKEK R-URAR INAILICE R" */
static const uint8_t k_fourth[29] = {
    'B','E','F','A','A','R','B','E','K','E','K',' ','R','-',
    'U','R','A','R',' ','I','N','A','I','L','I','C','E',' ',
    'R'
};

/* Palette combination id for each checksum index (bit 7 = DMG logo tilemap,
 * which we ignore for colorization). Pokémon Yellow is index 29 -> 3. */
static const uint8_t k_combo[94] = {
     0,  4,  5, 35, 34,  3, 31, 15, 10,  5,
    19, 36,  7, 37, 30, 44, 21, 32, 31, 20,
     5, 33, 13, 14,  5, 29,  5, 18,  9,  3,
     2, 26, 25, 25, 41, 42, 26, 45, 42, 45,
    36, 38, 26, 42, 30, 41, 34, 34,  5, 42,
     6,  5, 33, 25, 42, 42, 40,  2, 16, 25,
    42, 42,  5,  0, 39, 36, 22, 25,  6, 32,
    12, 36, 11, 39, 18, 39, 24, 31, 50, 17,
    46,  6, 27,  0, 47, 41, 41,  0,  0, 19,
    34, 23, 18, 29
};

/* Each combination: word offsets into k_pal (each palette is 4 words).
 * "palette_comb a,b,c"  -> a*4, b*4, c*4
 * "raw_palette_comb x,y,z" -> x, y, z   (byte offsets in the boot ROM are
 * 2*x etc.; a few combinations start mid-palette). Order is OBJ0, OBJ1, BG. */
static const uint16_t k_combos[51][3] = {
    {  4*4,  4*4, 29*4 }, /*  0 Right+A */
    { 18*4, 18*4, 18*4 }, /*  1 Right */
    { 20*4, 20*4, 20*4 }, /*  2 */
    { 24*4, 24*4, 24*4 }, /*  3 Down+A  (Pokémon Yellow, Tetris, ...) */
    {  9*4,  9*4,  9*4 }, /*  4 */
    {  0*4,  0*4,  0*4 }, /*  5 Up (grayscale-ish palette 0) */
    { 27*4, 27*4, 27*4 }, /*  6 Right+B */
    {  5*4,  5*4,  5*4 }, /*  7 Left+B */
    { 12*4, 12*4, 12*4 }, /*  8 Down */
    { 26*4, 26*4, 26*4 }, /*  9 */
    { 16*4,  8*4,  8*4 }, /* 10 */
    {  4*4, 28*4, 28*4 }, /* 11 Pokémon Blue */
    {  4*4,  2*4,  2*4 }, /* 12 */
    {  3*4,  4*4,  4*4 }, /* 13 Pokémon Red */
    {  4*4, 29*4, 29*4 }, /* 14 Pokémon Green */
    { 28*4,  4*4, 28*4 }, /* 15 */
    {  2*4, 17*4,  2*4 }, /* 16 */
    { 16*4, 16*4,  8*4 }, /* 17 */
    {  4*4,  4*4,  7*4 }, /* 18 */
    {  4*4,  4*4, 18*4 }, /* 19 */
    {  4*4,  4*4, 20*4 }, /* 20 */
    { 19*4, 19*4,  9*4 }, /* 21 */
    {  4*4-1,  4*4-1, 11*4 }, /* 22 raw 4*4-1, 4*4-1, 11*4 */
    { 17*4, 17*4,  2*4 }, /* 23 */
    {  4*4,  4*4,  2*4 }, /* 24 */
    {  4*4,  4*4,  3*4 }, /* 25 */
    { 28*4, 28*4,  0*4 }, /* 26 */
    {  3*4,  3*4,  0*4 }, /* 27 */
    {  0*4,  0*4,  1*4 }, /* 28 Up+B */
    { 18*4, 22*4, 18*4 }, /* 29 */
    { 20*4, 22*4, 20*4 }, /* 30 */
    { 24*4, 22*4, 24*4 }, /* 31 */
    { 16*4, 22*4,  8*4 }, /* 32 */
    { 17*4,  4*4, 13*4 }, /* 33 */
    { 28*4-1,  0*4, 14*4 }, /* 34 raw */
    { 28*4-1,  4*4, 15*4 }, /* 35 raw */
    { 19*4, 23*4-1,  9*4 }, /* 36 raw 19*4, 23*4-1, 9*4 */
    { 16*4, 28*4, 10*4 }, /* 37 */
    {  4*4, 23*4, 28*4 }, /* 38 */
    { 17*4, 22*4,  2*4 }, /* 39 */
    {  4*4,  0*4,  2*4 }, /* 40 Left+A */
    {  4*4, 28*4,  3*4 }, /* 41 Vegas Stakes */
    { 28*4,  3*4,  0*4 }, /* 42 */
    {  3*4, 28*4,  4*4 }, /* 43 Up+A */
    { 21*4, 28*4,  4*4 }, /* 44 */
    {  3*4, 28*4,  0*4 }, /* 45 */
    { 25*4,  3*4, 28*4 }, /* 46 */
    {  0*4, 28*4,  8*4 }, /* 47 */
    {  4*4,  3*4, 28*4 }, /* 48 Left */
    { 28*4,  3*4,  6*4 }, /* 49 Down+B */
    {  4*4, 28*4, 29*4 }, /* 50 */
};

/* 30 hardware palettes, 4 RGB555 colors each. SameBoy exclusives (CGA / DMG
 * LCD, ids 30-31) are deliberately omitted — they are not in the CGB boot
 * ROM. */
static const uint16_t k_pal[30 * 4] = {
    0x7FFF, 0x32BF, 0x00D0, 0x0000, /*  0 */
    0x639F, 0x4279, 0x15B0, 0x04CB, /*  1 */
    0x7FFF, 0x6E31, 0x454A, 0x0000, /*  2 */
    0x7FFF, 0x1BEF, 0x0200, 0x0000, /*  3 */
    0x7FFF, 0x421F, 0x1CF2, 0x0000, /*  4 */
    0x7FFF, 0x5294, 0x294A, 0x0000, /*  5 */
    0x7FFF, 0x03FF, 0x012F, 0x0000, /*  6 */
    0x7FFF, 0x03EF, 0x01D6, 0x0000, /*  7 */
    0x7FFF, 0x42B5, 0x3DC8, 0x0000, /*  8 */
    0x7E74, 0x03FF, 0x0180, 0x0000, /*  9 */
    0x67FF, 0x77AC, 0x1A13, 0x2D6B, /* 10 */
    0x7ED6, 0x4BFF, 0x2175, 0x0000, /* 11 */
    0x53FF, 0x4A5F, 0x7E52, 0x0000, /* 12 */
    0x4FFF, 0x7ED2, 0x3A4C, 0x1CE0, /* 13 */
    0x03ED, 0x7FFF, 0x255F, 0x0000, /* 14 */
    0x036A, 0x021F, 0x03FF, 0x7FFF, /* 15 */
    0x7FFF, 0x01DF, 0x0112, 0x0000, /* 16 */
    0x231F, 0x035F, 0x00F2, 0x0009, /* 17 */
    0x7FFF, 0x03EA, 0x011F, 0x0000, /* 18 */
    0x299F, 0x001A, 0x000C, 0x0000, /* 19 */
    0x7FFF, 0x027F, 0x001F, 0x0000, /* 20 */
    0x7FFF, 0x03E0, 0x0206, 0x0120, /* 21 */
    0x7FFF, 0x7EEB, 0x001F, 0x7C00, /* 22 */
    0x7FFF, 0x3FFF, 0x7E00, 0x001F, /* 23 */
    0x7FFF, 0x03FF, 0x001F, 0x0000, /* 24  Yellow / Tetris */
    0x03FF, 0x001F, 0x000C, 0x0000, /* 25 */
    0x7FFF, 0x033F, 0x0193, 0x0000, /* 26 */
    0x0000, 0x4200, 0x037F, 0x7FFF, /* 27 */
    0x7FFF, 0x7E8C, 0x7C00, 0x0000, /* 28 */
    0x7FFF, 0x1BEF, 0x6180, 0x0000, /* 29 */
};

static const uint8_t k_yellow_title[14] = {
    'P','O','K','E','M','O','N',' ','Y','E','L','L','O','W'
};

uint32_t hs_rgb555_raw888(uint16_t c)
{
    unsigned r = (unsigned)(c & 31u);
    unsigned g = (unsigned)((c >> 5) & 31u);
    unsigned b = (unsigned)((c >> 10) & 31u);
    r = (r << 3) | (r >> 2);
    g = (g << 3) | (g >> 2);
    b = (b << 3) | (b >> 2);
    return (r << 16) | (g << 8) | b;
}

uint32_t hs_cgb_lcd_rgb888(uint16_t c)
{
    unsigned r = (unsigned)(c & 31u);
    unsigned g = (unsigned)((c >> 5) & 31u);
    unsigned b = (unsigned)((c >> 10) & 31u);
    /* Crosstalk matrix, rows sum to 32 so a gray stays gray. */
    unsigned R = 26u * r +  4u * g +  2u * b;
    unsigned G =  4u * r + 24u * g +  4u * b;
    unsigned B =  2u * r +  4u * g + 26u * b;
    if (R > 960u) R = 960u;
    if (G > 960u) G = 960u;
    if (B > 960u) B = 960u;
    /* >> 2: peak 240, matching Pan Docs "highest intensity is light gray". */
    return ((R >> 2) << 16) | ((G >> 2) << 8) | (B >> 2);
}

static void load_combo(unsigned combo, struct hs_dmg_compat *out)
{
    unsigned i;
    uint16_t o0, o1, bg;
    if (combo >= 51u) {
        combo = 0u;
    }
    out->combo = combo;
    o0 = k_combos[combo][0];
    o1 = k_combos[combo][1];
    bg = k_combos[combo][2];
    for (i = 0; i < 4u; i++) {
        /* Offsets can start mid-palette (raw_palette_comb); clamp so a
         * short tail never reads off the end of k_pal. */
        unsigned a = (unsigned)o0 + i, b = (unsigned)o1 + i, c = (unsigned)bg + i;
        if (a >= 30u * 4u) a = 30u * 4u - 1u;
        if (b >= 30u * 4u) b = 30u * 4u - 1u;
        if (c >= 30u * 4u) c = 30u * 4u - 1u;
        out->obp0[i] = k_pal[a];
        out->obp1[i] = k_pal[b];
        out->bg[i]   = k_pal[c];
    }
}

static int nintendo_licensee(const uint8_t *hdr, size_t n)
{
    uint8_t old;
    if (n <= 0x4Bu) {
        return 0;
    }
    old = hdr[0x4B];
    if (old == 0x33u) {
        if (n <= 0x45u) {
            return 0;
        }
        return hdr[0x44] == (uint8_t)'0' && hdr[0x45] == (uint8_t)'1';
    }
    return old == 0x01u;
}

static unsigned title_sum(const uint8_t *hdr, size_t n)
{
    unsigned s = 0, i;
    for (i = 0x34u; i <= 0x43u; i++) {
        if (i < n) {
            s = (s + hdr[i]) & 0xFFu;
        }
    }
    return s;
}

static int title_is_yellow(const uint8_t *hdr, size_t n)
{
    if (n < 0x34u + 14u) {
        return 0;
    }
    return memcmp(hdr + 0x34u, k_yellow_title, 14) == 0;
}

static int lookup_index(unsigned sum, uint8_t fourth)
{
    int i;
    for (i = 0; i < 94; i++) {
        if (k_checksums[i] != (uint8_t)sum) {
            continue;
        }
        if (i >= 65 && k_fourth[i - 65] != fourth) {
            continue;
        }
        return i;
    }
    return -1;
}

void hs_dmg_compat_from_header(const uint8_t *hdr, size_t n, struct hs_dmg_compat *out)
{
    unsigned sum;
    int idx;
    memset(out, 0, sizeof(*out));
    sum = title_sum(hdr, n);
    out->checksum = sum;

    /* Yellow first, as specified. Title "POKEMON YELLOW" (14 bytes);
     * the last two title bytes are padding / the CGB flag and are ignored
     * for this match so both the DMG Japanese cart and a flag-stripped
     * international header select the same boot-ROM combination. */
    if (title_is_yellow(hdr, n)) {
        load_combo(HS_DMG_COMPAT_COMBO_YELLOW, out);
        out->is_yellow = 1;
        out->used_default = 0;
        return;
    }

    if (!nintendo_licensee(hdr, n)) {
        load_combo(HS_DMG_COMPAT_COMBO_DEFAULT, out);
        out->used_default = 1;
        return;
    }
    idx = lookup_index(sum, (n > 0x37u) ? hdr[0x37] : 0);
    if (idx < 0) {
        load_combo(HS_DMG_COMPAT_COMBO_DEFAULT, out);
        out->used_default = 1;
        return;
    }
    load_combo((unsigned)(k_combo[idx] & 0x7Fu), out);
    out->used_default = (out->combo == 0u && idx == 0) ? 1 : 0;
    /* "used_default" means the hardware default combination was selected
     * because the title was unknown or not Nintendo. A Nintendo title that
     * happens to use combination 0 (e.g. Tetris Plus) is a real hit. */
    if (idx > 0) {
        out->used_default = 0;
    }
}

void hs_dmg_compat_from_rom(const uint8_t *rom, size_t n, struct hs_dmg_compat *out)
{
    if (rom == NULL || n < 0x100u) {
        memset(out, 0, sizeof(*out));
        load_combo(HS_DMG_COMPAT_COMBO_DEFAULT, out);
        out->used_default = 1;
        return;
    }
    hs_dmg_compat_from_header(rom + 0x100u, n - 0x100u, out);
}
