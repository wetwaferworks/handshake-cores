/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 * hs_png.c - minimal clean-room PNG writer (RGB8, zlib stored blocks,
 * CRC-32 + Adler-32). Deterministic, no allocation beyond a row buffer.
 */
#include "hs_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t crc_update(uint32_t crc, const uint8_t *p, size_t n)
{
    size_t i;
    int k;
    for (i = 0; i < n; i++) {
        crc ^= p[i];
        for (k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

typedef struct {
    FILE *f;
    uint32_t crc;
} chunk_t;

static void be32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16); b[2] = (uint8_t)(v >> 8); b[3] = (uint8_t)v; }

static void chunk_begin(chunk_t *c, uint32_t len, const char *type)
{
    uint8_t b[4];
    be32(b, len);
    fwrite(b, 1, 4, c->f);
    fwrite(type, 1, 4, c->f);
    c->crc = crc_update(0xFFFFFFFFu, (const uint8_t *)type, 4);
}
static void chunk_data(chunk_t *c, const uint8_t *p, size_t n)
{
    fwrite(p, 1, n, c->f);
    c->crc = crc_update(c->crc, p, n);
}
static void chunk_end(chunk_t *c)
{
    uint8_t b[4];
    be32(b, c->crc ^ 0xFFFFFFFFu);
    fwrite(b, 1, 4, c->f);
}

int hs_png_write_xrgb(const char *path, const uint32_t *px, uint32_t w, uint32_t h, size_t stride)
{
    static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    uint8_t hdr[13];
    uint32_t row_len = w * 3u + 1u;
    uint64_t raw = (uint64_t)row_len * h;
    uint32_t nblocks = (uint32_t)((raw + 65534u) / 65535u);
    uint32_t idat_len = 2u + (uint32_t)raw + nblocks * 5u + 4u;
    uint32_t a1 = 1, a2 = 0, y, x, left_in_block = 0, blocks_done = 0;
    uint64_t remaining = raw;
    uint8_t *row;
    chunk_t c;
    FILE *f = fopen(path, "wb");
    if (!f)
        return -1;
    row = (uint8_t *)malloc(row_len);
    if (!row) { fclose(f); return -1; }
    c.f = f;
    fwrite(sig, 1, 8, f);
    be32(hdr, w); be32(hdr + 4, h);
    hdr[8] = 8; hdr[9] = 2; hdr[10] = 0; hdr[11] = 0; hdr[12] = 0;
    chunk_begin(&c, 13, "IHDR"); chunk_data(&c, hdr, 13); chunk_end(&c);
    chunk_begin(&c, idat_len, "IDAT");
    {
        uint8_t zh[2] = { 0x78, 0x01 };
        chunk_data(&c, zh, 2);
    }
    for (y = 0; y < h; y++) {
        const uint32_t *s = px + (size_t)y * stride;
        uint32_t off = 0;
        row[0] = 0;
        for (x = 0; x < w; x++) {
            row[1 + x * 3] = (uint8_t)(s[x] >> 16);
            row[2 + x * 3] = (uint8_t)(s[x] >> 8);
            row[3 + x * 3] = (uint8_t)s[x];
        }
        for (x = 0; x < row_len; x++) {
            a1 = (a1 + row[x]) % 65521u;
            a2 = (a2 + a1) % 65521u;
        }
        while (off < row_len) {
            uint32_t n;
            if (left_in_block == 0) {
                uint8_t bh[5];
                uint32_t blen = remaining > 65535u ? 65535u : (uint32_t)remaining;
                blocks_done++;
                bh[0] = (uint8_t)(blocks_done == nblocks ? 1 : 0);
                bh[1] = (uint8_t)blen; bh[2] = (uint8_t)(blen >> 8);
                bh[3] = (uint8_t)~blen; bh[4] = (uint8_t)(~blen >> 8);
                chunk_data(&c, bh, 5);
                left_in_block = blen;
            }
            n = row_len - off;
            if (n > left_in_block) n = left_in_block;
            chunk_data(&c, row + off, n);
            off += n;
            left_in_block -= n;
            remaining -= n;
        }
    }
    {
        uint8_t ad[4];
        be32(ad, (a2 << 16) | a1);
        chunk_data(&c, ad, 4);
    }
    chunk_end(&c);
    chunk_begin(&c, 0, "IEND"); chunk_end(&c);
    free(row);
    fclose(f);
    return 0;
}

size_t hs_read_file(const char *path, uint8_t *buf, size_t cap)
{
    size_t n;
    FILE *f = fopen(path, "rb");
    if (!f)
        return 0;
    n = fread(buf, 1, cap, f);
    fclose(f);
    return n;
}
