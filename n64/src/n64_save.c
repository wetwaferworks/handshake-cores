/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_save.c - cartridge save memory, controller pak and IPL3 (CIC) identity
 * for hs-n64 (component profile, Phase 0 only, all state inside hs_n64_t).
 *
 *   EEPROM 4 Kbit / 16 Kbit  joybus channel 4: 0x00/0xFF info, 0x04 read 8 B, 0x05 write 8 B
 *   SRAM 256 Kbit            PI domain 2, 0x08000000..0x08007FFF, DMA both ways
 *   FlashRAM 1 Mbit          PI domain 2: command register 0x08010000, status / ID / read
 *                            array / page-buffer program / sector + chip erase
 *   Controller pak 32 KiB    joybus channel 0: 0x02 read 32 B + data CRC, 0x03 write 32 B
 *   CIC                      CRC-32 of the IPL3 boot code (ROM 0x40..0xFFF) selects the
 *                            boot seed, entry offset and the header checksum variant
 *
 * Save type: HS_N64_CFG_SAVE_* override, else a small table keyed by the
 * cartridge ID in the ROM header (only titles we target), else none.
 * Save memory survives n64 reset; it is exported with save_sram/load_sram.
 */
#include <string.h>
#include "n64_internal.h"

/* --------------------------------------------------------------- CIC */
uint32_t n64_rom_crc32(const hs_n64_t *n, uint32_t from, uint32_t to)
{
    uint32_t c = 0xFFFFFFFFu, i, k;
    for (i = from; i < to; i++) {
        c ^= (uint32_t)(n64_rom_word(n, i & ~3u) >> (8u * (3u - (i & 3u)))) & 0xFFu;
        for (k = 0; k < 8u; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

uint32_t n64_cic_detect(const hs_n64_t *n, uint32_t *crc32_out)
{
    uint32_t c = n64_rom_crc32(n, 0x40u, 0x1000u);
    if (crc32_out) *crc32_out = c;
    switch (c) {
    case 0x6170A4A1u: return N64_CIC_6101;
    case 0x90BB6CB5u: return N64_CIC_6102;
    case 0x0B050EE0u: return N64_CIC_6103;
    case 0x98BC2C86u: return N64_CIC_6105;
    case 0xACC8580Au: return N64_CIC_6106;
    case 0x009E9EA3u: return N64_CIC_6101;  /* NUS-7102 (PAL 6101 family) */
    default: return 0;
    }
}

static uint32_t rol32(uint32_t v, uint32_t s) { s &= 31u; return s ? (v << s) | (v >> (32u - s)) : v; }

/* header checksum over ROM 0x1000..0x101000 for each IPL3 variant */
void n64_rom_crc_cic(const hs_n64_t *n, uint32_t cic, uint32_t *crc1, uint32_t *crc2)
{
    uint32_t seed = 0xF8CA4DDCu, t1, t2, t3, t4, t5, t6, i;
    if (cic == N64_CIC_6103) seed = 0xA3886759u;
    else if (cic == N64_CIC_6105) seed = 0xDF26F436u;
    else if (cic == N64_CIC_6106) seed = 0x1FEA617Au;
    t1 = t2 = t3 = t4 = t5 = t6 = seed;
    for (i = 0x1000u; i < 0x101000u; i += 4u) {
        uint32_t d = n64_rom_word(n, i), r;
        if (t6 + d < t6) t4++;
        t6 += d;
        t3 ^= d;
        r = rol32(d, d & 31u);
        t5 += r;
        if (t2 > d) t2 ^= r; else t2 ^= t6 ^ d;
        if (cic == N64_CIC_6105) t1 += n64_rom_word(n, 0x40u + 0x0710u + (i & 0xFFu)) ^ d;
        else t1 += t5 ^ d;
    }
    if (cic == N64_CIC_6103) { *crc1 = (t6 ^ t4) + t3; *crc2 = (t5 ^ t2) + t1; }
    else if (cic == N64_CIC_6106) { *crc1 = (t6 * t4) + t3; *crc2 = (t5 * t2) + t1; }
    else { *crc1 = t6 ^ t4 ^ t3; *crc2 = t5 ^ t2 ^ t1; }
}

/* ---------------------------------------------------------- save type */
typedef struct { char id[3]; uint8_t type; uint8_t need_8mb; } save_db_t;
static const save_db_t SAVE_DB[] = {
    { "SM", N64_SAVE_EEP4K, 0 },   /* Super Mario 64 */
    { "ZL", N64_SAVE_SRAM,  0 },   /* The Legend of Zelda: Ocarina of Time */
    { "ZS", N64_SAVE_FLASH, 1 },   /* The Legend of Zelda: Majora's Mask (requires the Expansion Pak) */
};

void n64_save_setup(hs_n64_t *n)
{
    n64_save_t *s = &n->save;
    uint32_t cfg = (n->cfg_flags >> HS_N64_CFG_SAVE_SHIFT) & 7u, t = N64_SAVE_NONE, i;
    if (n->rom_len >= 0x40u) {
        uint32_t w = n64_rom_word(n, 0x3Cu);
        char a = (char)(w >> 24), b = (char)(w >> 16);
        for (i = 0; i < sizeof(SAVE_DB) / sizeof(SAVE_DB[0]); i++)
            if (SAVE_DB[i].id[0] == a && SAVE_DB[i].id[1] == b) {
                t = SAVE_DB[i].type;
                if (SAVE_DB[i].need_8mb) n->rdram_size = N64_RDRAM_MAX;   /* auto Expansion Pak */
            }
    }
    if (cfg) t = cfg - 1u;
    if (t > N64_SAVE_FLASH) t = N64_SAVE_NONE;
    if (s->type != t) {                       /* new cartridge type: blank (erased) memory */
        s->type = t;
        memset(s->data, 0xFF, sizeof(s->data));
        s->dirty = 0; s->writes = 0;
    }
    s->size = t == N64_SAVE_EEP4K ? 512u : t == N64_SAVE_EEP16K ? 2048u : t == N64_SAVE_SRAM ? 32768u : t == N64_SAVE_FLASH ? 131072u : 0u;
    s->pak_on = (n->cfg_flags & HS_N64_CFG_CONTROLLER_PAK) ? 1u : 0u;
    s->fl_mode = 0; s->fl_status = 0x1111800100C2001Eull;
    s->cic = n64_cic_detect(n, NULL);
    s->cic_known = s->cic ? 1u : 0u;
    if (!s->cic) s->cic = N64_CIC_6102;
}

static void touched(n64_save_t *s) { s->dirty++; s->writes++; }

/* ------------------------------------------------------------- EEPROM */
int n64_save_joybus(hs_n64_t *n, uint32_t ch, const uint8_t *cmd, uint32_t tx, uint8_t *res, uint32_t rx)
{
    n64_save_t *s = &n->save;
    uint32_t blocks;
    if (ch != 4u || (s->type != N64_SAVE_EEP4K && s->type != N64_SAVE_EEP16K) || tx < 1u) return -1;
    blocks = s->size / 8u;
    switch (cmd[0]) {
    case 0x00: case 0xFF:
        if (rx >= 3u) { res[0] = 0x00; res[1] = s->type == N64_SAVE_EEP16K ? 0xC0 : 0x80; res[2] = 0x00; }
        return 0;
    case 0x04:
        if (tx >= 2u && rx >= 8u) {
            if (cmd[1] < blocks) memcpy(res, s->data + (uint32_t)cmd[1] * 8u, 8);
            else memset(res, 0xFF, 8);
        }
        return 0;
    case 0x05:
        if (tx >= 10u) {
            if (cmd[1] < blocks) { memcpy(s->data + (uint32_t)cmd[1] * 8u, cmd + 2, 8); touched(s); }
            if (rx >= 1u) res[0] = 0x00;
        }
        return 0;
    default:
        return -1;
    }
}

/* ------------------------------------------------------ controller pak */
uint8_t n64_pak_crc8(const uint8_t *d, uint32_t len)
{
    uint32_t i, j, c = 0;
    for (i = 0; i <= len; i++)
        for (j = 8; j-- > 0;) {
            uint32_t x = (c & 0x80u) ? 0x85u : 0u;
            c = (c << 1) & 0xFFu;
            if (i < len && ((d[i] >> j) & 1u)) c |= 1u;
            c ^= x;
        }
    return (uint8_t)c;
}

int n64_pak_cmd(hs_n64_t *n, const uint8_t *cmd, uint32_t tx, uint8_t *res, uint32_t rx)
{
    n64_save_t *s = &n->save;
    uint32_t addr;
    if (!s->pak_on || tx < 3u) return -1;
    addr = (((uint32_t)cmd[1] << 8) | cmd[2]) & 0xFFE0u;    /* low 5 bits: address CRC */
    if (cmd[0] == 0x02 && rx >= 33u) {
        if (addr < N64_PAK_SIZE) memcpy(res, s->pak + addr, 32); else memset(res, 0, 32);
        res[32] = n64_pak_crc8(res, 32);
        return 0;
    }
    if (cmd[0] == 0x03 && tx >= 35u && rx >= 1u) {
        if (addr < N64_PAK_SIZE) { memcpy(s->pak + addr, cmd + 3, 32); s->pak_dirty++; }
        res[0] = n64_pak_crc8(cmd + 3, 32);
        return 0;
    }
    return -1;
}

/* ------------------------------------------------- PI domain 2 (SRAM / Flash) */
static uint32_t sram_off(uint32_t cart) { return (cart - 0x08000000u) & 0x7FFFu; }

static void flash_cmd(n64_save_t *s, uint32_t v)
{
    switch (v >> 24) {
    case 0xD2:                                          /* execute (erase or program) / status */
        if (s->fl_mode == 5u) {
            memset(s->data + s->fl_erase_off, 0xFF, s->fl_erase_len);
            s->fl_status = (s->fl_status & ~0xFF00000000ull) | (0x08ull << 32);
            touched(s);
            s->fl_mode = 0;
        } else s->fl_mode = 1u;
        break;
    case 0xE1: s->fl_mode = 2u; s->fl_status = 0x1111800100C2001Eull; break;   /* read ID */
    case 0xF0: s->fl_mode = 3u; break;                  /* read array */
    case 0xB4: s->fl_mode = 4u; break;                  /* page-buffer load */
    case 0x4B:                                          /* sector erase: 128 pages of 128 B */
        s->fl_erase_off = ((v & 0xFFFFu) * 128u) & ~0x3FFFu & (N64_SAVE_MAX - 1u);
        s->fl_erase_len = 0x4000u;
        break;
    case 0x3C: s->fl_erase_off = 0; s->fl_erase_len = N64_SAVE_MAX; break;      /* chip erase */
    case 0x78: s->fl_mode = 5u; break;                  /* arm the erase */
    case 0xA5: {                                        /* program page from the buffer */
        uint32_t off = ((v & 0xFFFFu) * 128u) & (N64_SAVE_MAX - 1u);
        memcpy(s->data + off, s->fl_buf, 128);
        s->fl_status = (s->fl_status & ~0xFF00000000ull) | (0x04ull << 32);
        touched(s);
        s->fl_mode = 0;
        break;
    }
    default: break;
    }
}

uint32_t n64_cart2_read32(hs_n64_t *n, uint32_t pa)
{
    n64_save_t *s = &n->save;
    if (s->type == N64_SAVE_SRAM) return n64_be32(s->data + (sram_off(pa) & ~3u));
    if (s->type == N64_SAVE_FLASH) return (uint32_t)(s->fl_status >> 32);
    return 0;
}

void n64_cart2_write32(hs_n64_t *n, uint32_t pa, uint32_t v)
{
    n64_save_t *s = &n->save;
    if (s->type == N64_SAVE_SRAM) { n64_wbe32(s->data + (sram_off(pa) & ~3u), v); touched(s); return; }
    if (s->type != N64_SAVE_FLASH) return;
    if (pa == 0x08010000u) flash_cmd(s, v);
    else if (pa == 0x08000000u) s->fl_status &= ~0xFF00000000ull;              /* clear status */
}

void n64_cart2_dma_read(hs_n64_t *n, uint32_t cart, uint8_t *dst, uint32_t len)
{
    n64_save_t *s = &n->save;
    uint32_t i;
    if (s->type == N64_SAVE_SRAM) {
        for (i = 0; i < len; i++) dst[i] = s->data[sram_off(cart + i)];
    } else if (s->type == N64_SAVE_FLASH && (s->fl_mode == 1u || s->fl_mode == 2u)) {
        for (i = 0; i < len; i++) dst[i] = i < 8u ? (uint8_t)(s->fl_status >> (56u - 8u * i)) : 0u;
    } else if (s->type == N64_SAVE_FLASH && s->fl_mode == 3u) {
        uint32_t base = ((cart - 0x08000000u) * 2u) & (N64_SAVE_MAX - 1u);   /* read-array addresses are halved on the bus */
        for (i = 0; i < len; i++) dst[i] = s->data[(base + i) & (N64_SAVE_MAX - 1u)];
    } else {
        memset(dst, 0, len);
    }
}

void n64_cart2_dma_write(hs_n64_t *n, uint32_t cart, const uint8_t *src, uint32_t len)
{
    n64_save_t *s = &n->save;
    uint32_t i;
    if (s->type == N64_SAVE_SRAM) {
        for (i = 0; i < len; i++) s->data[sram_off(cart + i)] = src[i];
        touched(s);
    } else if (s->type == N64_SAVE_FLASH && s->fl_mode == 4u) {
        for (i = 0; i < len && i < 128u; i++) s->fl_buf[i] = src[i];
    }
}
