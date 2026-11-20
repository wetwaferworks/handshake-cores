/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_testroms.h — clean-room synthetic SNES test ROMs assembled in C
 * (tiny 65C816 + SPC700 emitters, LoROM header/checksum, integer-only data
 * generation so ROM bytes are identical on every architecture).
 */
#ifndef SNES_TESTROMS_H
#define SNES_TESTROMS_H
#include <stddef.h>
#include <stdint.h>

#define TR_RES 0x7E1000u   /* result block in WRAM written by the test ROMs */

typedef struct tr_info {
    size_t   size;
    uint32_t spc_blob_off;    /* ROM offset of the uploaded SPC block */
    uint16_t spc_blob_len;
    uint16_t spc_dest;        /* ARAM destination */
    uint16_t per_addr;        /* expected PER push value (CPU ROM) */
} tr_info;

/* each builder fills rom (zeroed by caller, capacity >= returned size) */
size_t tr_build_cpu_rom(uint8_t *rom, size_t cap, tr_info *info);
size_t tr_build_mode1_rom(uint8_t *rom, size_t cap, tr_info *info);   /* 32 KiB */
size_t tr_build_mode7_rom(uint8_t *rom, size_t cap, tr_info *info);   /* 128 KiB */

#endif
