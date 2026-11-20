/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 HandshakeOS contributors */
/* gbc_compat_pal.h - CGB boot-ROM DMG colorization (clean-room).
 *
 * When a monochrome (DMG) cart runs on a Game Boy Color, the CGB boot ROM
 * chooses three palettes (BG, OBJ0, OBJ1) from the title checksum and, for
 * a handful of collisions, the fourth title letter at $0137. The algorithm
 * and the published RGB555 values come from public hardware documentation:
 *   - Pan Docs, "Power-Up Sequence / Compatibility palettes"
 *     https://gbdev.io/pandocs/Power_Up_Sequence.html
 *   - Pan Docs, "RGB Translation by CGBs"
 *     https://gbdev.io/pandocs/Palettes.html
 *
 * Tables were transcribed as facts from that documentation and from Liji
 * Halphon's freely published CGB boot-ROM reimplementation (the source Pan
 * Docs itself points at). This file is original C; it does not include or
 * assemble any official boot ROM.
 *
 * Pokémon Yellow is matched first by title, then every other known title
 * is looked up in the checksum / fourth-letter table. An unknown title or
 * a non-Nintendo licensee selects palette combination $00 (hardware default).
 */
#ifndef GBC_COMPAT_PAL_H
#define GBC_COMPAT_PAL_H

#include <stddef.h>
#include <stdint.h>

#define HS_DMG_COMPAT_COMBO_DEFAULT  0u
#define HS_DMG_COMPAT_COMBO_YELLOW   3u  /* Down+A: white / yellow / red / black */

struct hs_dmg_compat {
    uint16_t bg[4];          /* RGB555, bit0-4 R, 5-9 G, 10-14 B */
    uint16_t obp0[4];
    uint16_t obp1[4];
    unsigned combo;          /* 0..50, hardware combination id */
    unsigned checksum;       /* sum of $0134..$0143, 8-bit */
    int      used_default;   /* 1 = combination $00 (unknown / not Nintendo) */
    int      is_yellow;      /* 1 = Pokémon Yellow title matched first */
};

/* Fill *out from a cart ROM image (needs at least $014C bytes). Always
 * succeeds; unknown / short / non-Nintendo headers yield the default. */
void hs_dmg_compat_from_rom(const uint8_t *rom, size_t n, struct hs_dmg_compat *out);

/* Same lookup given an already-sliced header that starts at $0100
 * (title at +0x34). n is the number of bytes from $0100. */
void hs_dmg_compat_from_header(const uint8_t *hdr, size_t n, struct hs_dmg_compat *out);

/* GBC LCD color correction: RGB555 -> 0x00RRGGBB (no alpha).
 *
 * Curve (documented in docs/CGB-LCD-COLOR.md):
 *   R' = min(960, 26*r +  4*g +  2*b)
 *   G' = min(960,  4*r + 24*g +  4*b)
 *   B' = min(960,  2*r +  4*g + 26*b)
 *   8-bit = x >> 2          (peak 240: the CGB LCD's "white" is light gray)
 *   The matrix rows sum to 32, so a gray stays a (slightly dimmer) gray.
 *
 * r,g,b are the 5-bit channels. This is the widely published mixing form of
 * the CGB screen's unsaturated pigments (Pan Docs: $03EF, a neon green on
 * a PC, is a washed yellow on the CGB). */
uint32_t hs_cgb_lcd_rgb888(uint16_t rgb555);

/* 5-to-8 bit expand with no mixing (for tests / comparison). */
uint32_t hs_rgb555_raw888(uint16_t rgb555);

#endif
