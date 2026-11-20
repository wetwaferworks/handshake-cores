/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_pngread.h - pure C11 PNG reader (own inflate: stored, fixed and dynamic Huffman blocks).
 * Decodes any non-interlaced PNG (grey, grey+alpha, RGB, RGBA, palette; 1/2/4/8/16-bit) to
 * 8-bit RGB, the same conversion Pillow's Image.convert('RGB') performs: alpha is dropped,
 * low-bit grey is scaled to 0..255, 16-bit samples keep their high byte, palette indices
 * are looked up in PLTE. Single header, all functions static. No third-party code. */
#ifndef HS_PNGREAD_H
#define HS_PNGREAD_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const uint8_t *p; size_t n, pos; uint32_t bit, nbit; } hs_inf_in;
typedef struct { uint8_t *b; size_t n, cap; } hs_inf_out;
typedef struct { uint16_t count[16], sym[320]; } hs_huff;

static int hs_inf_bits(hs_inf_in *s, int need, uint32_t *v) {
    while (s->nbit < (uint32_t)need) {
        if (s->pos >= s->n) return -1;
        s->bit |= (uint32_t)s->p[s->pos++] << s->nbit;
        s->nbit += 8;
    }
    *v = s->bit & ((need == 32) ? 0xFFFFFFFFu : ((1u << need) - 1u));
    s->bit >>= need;
    s->nbit -= (uint32_t)need;
    return 0;
}
static int hs_inf_put(hs_inf_out *o, uint8_t c) {
    if (o->n == o->cap) {
        size_t nc = o->cap ? o->cap * 2 : 65536;
        uint8_t *nb = (uint8_t *)realloc(o->b, nc);
        if (!nb) return -1;
        o->b = nb; o->cap = nc;
    }
    o->b[o->n++] = c;
    return 0;
}
static int hs_huff_build(hs_huff *h, const uint8_t *len, int n) {
    uint16_t offs[16];
    int i;
    memset(h->count, 0, sizeof h->count);
    for (i = 0; i < n; i++) h->count[len[i]]++;
    h->count[0] = 0;
    offs[1] = 0;
    for (i = 1; i < 15; i++) offs[i + 1] = (uint16_t)(offs[i] + h->count[i]);
    for (i = 0; i < n; i++) if (len[i]) h->sym[offs[len[i]]++] = (uint16_t)i;
    return 0;
}
static int hs_huff_decode(hs_inf_in *s, const hs_huff *h) {
    int code = 0, first = 0, index = 0, l;
    for (l = 1; l < 16; l++) {
        uint32_t b;
        int cnt;
        if (hs_inf_bits(s, 1, &b)) return -1;
        code |= (int)b;
        cnt = h->count[l];
        if (code - cnt < first) return h->sym[index + (code - first)];
        index += cnt; first += cnt; first <<= 1; code <<= 1;
    }
    return -1;
}
static int hs_inflate_codes(hs_inf_in *s, hs_inf_out *o, const hs_huff *lit, const hs_huff *dst) {
    static const uint16_t lbase[29] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                        35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258 };
    static const uint8_t lext[29] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0 };
    static const uint16_t dbase[30] = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769,
                                        1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577 };
    static const uint8_t dext[30] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };
    for (;;) {
        int sym = hs_huff_decode(s, lit);
        if (sym < 0) return -1;
        if (sym < 256) { if (hs_inf_put(o, (uint8_t)sym)) return -1; }
        else if (sym == 256) return 0;
        else {
            uint32_t e, len, dist;
            int ds;
            sym -= 257;
            if (sym >= 29) return -1;
            if (hs_inf_bits(s, lext[sym], &e)) return -1;
            len = lbase[sym] + e;
            ds = hs_huff_decode(s, dst);
            if (ds < 0 || ds >= 30) return -1;
            if (hs_inf_bits(s, dext[ds], &e)) return -1;
            dist = dbase[ds] + e;
            if (dist > o->n) return -1;
            while (len--) if (hs_inf_put(o, o->b[o->n - dist])) return -1;
        }
    }
}
static int hs_inflate_zlib(const uint8_t *p, size_t n, hs_inf_out *o) {
    hs_inf_in s;
    uint32_t last, type;
    if (n < 2 || ((p[0] << 8) | p[1]) % 31 != 0 || (p[0] & 15) != 8 || (p[1] & 0x20)) return -1;
    s.p = p; s.n = n; s.pos = 2; s.bit = 0; s.nbit = 0;
    do {
        if (hs_inf_bits(&s, 1, &last) || hs_inf_bits(&s, 2, &type)) return -1;
        if (type == 0) {
            uint32_t len;
            s.bit = 0; s.nbit = 0;
            if (s.pos + 4 > s.n) return -1;
            len = (uint32_t)s.p[s.pos] | ((uint32_t)s.p[s.pos + 1] << 8);
            if ((len ^ 0xFFFFu) != ((uint32_t)s.p[s.pos + 2] | ((uint32_t)s.p[s.pos + 3] << 8))) return -1;
            s.pos += 4;
            if (s.pos + len > s.n) return -1;
            while (len--) if (hs_inf_put(o, s.p[s.pos++])) return -1;
        } else if (type == 1) {
            static hs_huff fl, fd;
            static int init;
            if (!init) {
                uint8_t l[288];
                int i;
                for (i = 0; i < 144; i++) l[i] = 8;
                for (; i < 256; i++) l[i] = 9;
                for (; i < 280; i++) l[i] = 7;
                for (; i < 288; i++) l[i] = 8;
                hs_huff_build(&fl, l, 288);
                for (i = 0; i < 30; i++) l[i] = 5;
                hs_huff_build(&fd, l, 30);
                init = 1;
            }
            if (hs_inflate_codes(&s, o, &fl, &fd)) return -1;
        } else if (type == 2) {
            static const uint8_t ord[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
            uint32_t hlit, hdist, hclen, v;
            uint8_t lens[320];
            hs_huff lh, dh, ch;
            int i, nlen;
            if (hs_inf_bits(&s, 5, &hlit) || hs_inf_bits(&s, 5, &hdist) || hs_inf_bits(&s, 4, &hclen)) return -1;
            hlit += 257; hdist += 1; hclen += 4;
            memset(lens, 0, sizeof lens);
            for (i = 0; i < (int)hclen; i++) { if (hs_inf_bits(&s, 3, &v)) return -1; lens[ord[i]] = (uint8_t)v; }
            hs_huff_build(&ch, lens, 19);
            nlen = (int)(hlit + hdist);
            for (i = 0; i < nlen;) {
                int sym = hs_huff_decode(&s, &ch), rep;
                uint8_t val = 0;
                if (sym < 0) return -1;
                if (sym < 16) { lens[i++] = (uint8_t)sym; continue; }
                if (sym == 16) { if (i == 0 || hs_inf_bits(&s, 2, &v)) return -1; val = lens[i - 1]; rep = 3 + (int)v; }
                else if (sym == 17) { if (hs_inf_bits(&s, 3, &v)) return -1; rep = 3 + (int)v; }
                else { if (hs_inf_bits(&s, 7, &v)) return -1; rep = 11 + (int)v; }
                if (i + rep > nlen) return -1;
                while (rep--) lens[i++] = val;
            }
            hs_huff_build(&lh, lens, (int)hlit);
            hs_huff_build(&dh, lens + hlit, (int)hdist);
            if (hs_inflate_codes(&s, o, &lh, &dh)) return -1;
        } else return -1;
    } while (!last);
    return 0;
}

static uint32_t hs_png_be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

/* Returns 0 and a malloc'd w*h*3 RGB buffer, or -1. */
static int hs_png_load_rgb(const char *path, int *pw, int *ph, uint8_t **prgb) {
    FILE *f = fopen(path, "rb");
    uint8_t *file = NULL, *idat = NULL, pal[768];
    size_t fn = 0, fcap = 0, in = 0, pos = 8;
    uint32_t w = 0, h = 0;
    int bd = 0, ct = -1, ilace = 0, npal = 0, rc = -1;
    hs_inf_out raw = { NULL, 0, 0 };
    memset(pal, 0, sizeof pal);
    if (!f) return -1;
    for (;;) {
        size_t r;
        if (fn == fcap) {
            uint8_t *nb = (uint8_t *)realloc(file, fcap ? fcap * 2 : 1 << 20);
            if (!nb) { fclose(f); free(file); return -1; }
            file = nb; fcap = fcap ? fcap * 2 : 1 << 20;
        }
        r = fread(file + fn, 1, fcap - fn, f);
        if (r == 0) break;
        fn += r;
    }
    fclose(f);
    if (fn < 8 || memcmp(file, "\x89PNG\r\n\x1a\n", 8) != 0) goto done;
    idat = (uint8_t *)malloc(fn);
    if (!idat) goto done;
    while (pos + 12 <= fn) {
        uint32_t len = hs_png_be32(file + pos);
        const uint8_t *type = file + pos + 4, *d = file + pos + 8;
        if (pos + 12 + (size_t)len > fn) goto done;
        if (!memcmp(type, "IHDR", 4) && len >= 13) {
            w = hs_png_be32(d); h = hs_png_be32(d + 4); bd = d[8]; ct = d[9]; ilace = d[12];
        } else if (!memcmp(type, "PLTE", 4)) {
            npal = (int)(len / 3); if (npal > 256) npal = 256;
            memcpy(pal, d, (size_t)npal * 3);
        } else if (!memcmp(type, "IDAT", 4)) {
            memcpy(idat + in, d, len); in += len;
        } else if (!memcmp(type, "IEND", 4)) break;
        pos += 12 + (size_t)len;
    }
    if (w == 0 || h == 0 || w > 16384 || h > 16384 || ilace != 0) goto done;
    {
        int ch = ct == 0 ? 1 : ct == 2 ? 3 : ct == 3 ? 1 : ct == 4 ? 2 : ct == 6 ? 4 : 0;
        size_t bpp_bits = (size_t)ch * (size_t)bd, stride = (w * bpp_bits + 7) / 8, bpp = (bpp_bits + 7) / 8, y, x;
        uint8_t *rgb, *prev, *cur;
        if (!ch || !(bd == 1 || bd == 2 || bd == 4 || bd == 8 || bd == 16)) goto done;
        if (hs_inflate_zlib(idat, in, &raw) || raw.n < h * (stride + 1)) goto done;
        rgb = (uint8_t *)malloc((size_t)w * h * 3);
        prev = (uint8_t *)calloc(stride, 1);
        if (!rgb || !prev) { free(rgb); free(prev); goto done; }
        for (y = 0; y < h; y++) {
            int ft = raw.b[y * (stride + 1)];
            cur = raw.b + y * (stride + 1) + 1;
            for (x = 0; x < stride; x++) {
                int a = x >= bpp ? cur[x - bpp] : 0, b = prev[x], c = x >= bpp ? prev[x - bpp] : 0, v = cur[x];
                switch (ft) {
                case 0: break;
                case 1: v += a; break;
                case 2: v += b; break;
                case 3: v += (a + b) / 2; break;
                case 4: { int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
                          v += (pa <= pb && pa <= pc) ? a : (pb <= pc) ? b : c; break; }
                default: free(rgb); free(prev); goto done;
                }
                cur[x] = (uint8_t)v;
            }
            memcpy(prev, cur, stride);
            for (x = 0; x < w; x++) {
                uint8_t *o = rgb + ((size_t)y * w + x) * 3;
                if (bd < 8) {
                    size_t bit = x * (size_t)bd;
                    int v = (cur[bit / 8] >> (8 - bd - (int)(bit % 8))) & ((1 << bd) - 1);
                    if (ct == 3) { o[0] = pal[v * 3]; o[1] = pal[v * 3 + 1]; o[2] = pal[v * 3 + 2]; }
                    else { int s = v * (255 / ((1 << bd) - 1)); o[0] = o[1] = o[2] = (uint8_t)s; }
                } else {
                    const uint8_t *s = cur + x * bpp;
                    int k = bd == 16 ? 2 : 1;
                    if (ct == 3) { o[0] = pal[s[0] * 3]; o[1] = pal[s[0] * 3 + 1]; o[2] = pal[s[0] * 3 + 2]; }
                    else if (ct == 0 || ct == 4) { o[0] = o[1] = o[2] = s[0]; }
                    else { o[0] = s[0]; o[1] = s[k]; o[2] = s[2 * k]; }
                }
            }
        }
        free(prev);
        *pw = (int)w; *ph = (int)h; *prgb = rgb;
        rc = 0;
    }
done:
    free(raw.b); free(idat); free(file);
    return rc;
}
#endif
