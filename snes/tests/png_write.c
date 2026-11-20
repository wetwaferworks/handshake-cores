/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 * Minimal dependency-free PNG writer (stored/uncompressed deflate blocks). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "png_write.h"

static uint32_t crc_upd(uint32_t c, const uint8_t *b, size_t n)
{
    size_t i;
    int k;
    for (i = 0; i < n; i++) {
        c ^= b[i];
        for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c;
}
static void be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static int chunk(FILE *f, const char *type, const uint8_t *data, uint32_t len)
{
    uint8_t hdr[8], crcb[4];
    uint32_t c;
    be32(hdr, len); memcpy(hdr + 4, type, 4);
    c = crc_upd(0xFFFFFFFFu, hdr + 4, 4);
    if (len) c = crc_upd(c, data, len);
    be32(crcb, c ^ 0xFFFFFFFFu);
    return fwrite(hdr, 1, 8, f) == 8 && (len == 0 || fwrite(data, 1, len, f) == len) && fwrite(crcb, 1, 4, f) == 4;
}

int hs_png_write_xrgb(const char *path, const uint32_t *px, int w, int h)
{
    static const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    size_t raw_len = (size_t)h * (size_t)(w * 3 + 1);
    size_t nblk = (raw_len + 65534u) / 65535u;
    size_t z_len = 2 + raw_len + nblk * 5 + 4;
    uint8_t *raw = malloc(raw_len), *z = malloc(z_len), ihdr[13];
    size_t i, o = 0, done = 0;
    uint32_t a = 1, b = 0;
    FILE *f;
    int x, y, ok;
    if (!raw || !z) { free(raw); free(z); return -1; }
    for (y = 0; y < h; y++) {
        uint8_t *r = raw + (size_t)y * (size_t)(w * 3 + 1);
        r[0] = 0;
        for (x = 0; x < w; x++) {
            uint32_t p = px[(size_t)y * (size_t)w + (size_t)x];
            r[1 + x * 3] = (uint8_t)(p >> 16); r[2 + x * 3] = (uint8_t)(p >> 8); r[3 + x * 3] = (uint8_t)p;
        }
    }
    z[o++] = 0x78; z[o++] = 0x01;
    while (done < raw_len) {
        size_t n = raw_len - done > 65535u ? 65535u : raw_len - done;
        z[o++] = (uint8_t)(done + n == raw_len);
        z[o++] = (uint8_t)n; z[o++] = (uint8_t)(n >> 8);
        z[o++] = (uint8_t)~n; z[o++] = (uint8_t)((~n) >> 8);
        memcpy(z + o, raw + done, n); o += n; done += n;
    }
    for (i = 0; i < raw_len; i++) { a = (a + raw[i]) % 65521u; b = (b + a) % 65521u; }
    be32(z + o, (b << 16) | a); o += 4;
    be32(ihdr, (uint32_t)w); be32(ihdr + 4, (uint32_t)h);
    ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
    f = fopen(path, "wb");
    if (!f) { free(raw); free(z); return -1; }
    ok = fwrite(sig, 1, 8, f) == 8 && chunk(f, "IHDR", ihdr, 13) && chunk(f, "IDAT", z, (uint32_t)o) && chunk(f, "IEND", 0, 0);
    fclose(f);
    free(raw); free(z);
    return ok ? 0 : -1;
}
