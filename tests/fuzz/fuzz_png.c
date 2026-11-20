/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * fuzz_png - deterministic mutation fuzzer for hs_pngread.h (the decoder behind hs-pngcmp and
 * acid2_compare). Pure C11, host sanitizer builds only. usage: fuzz_png SEED.png [iters] */
#include "../hs_pngread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t rs = 0x48535047ull;
static uint64_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }

int main(int argc, char **argv) {
    FILE *f;
    uint8_t *seed, *b;
    long sz;
    unsigned it = argc > 2 ? (unsigned)strtoul(argv[2], NULL, 10) : 2000, i, ok = 0;
    const char *tmp = "fuzz_png.tmp.png";
    if (argc < 2 || !(f = fopen(argv[1], "rb"))) return 2;
    fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);
    seed = (uint8_t *)malloc((size_t)sz); b = (uint8_t *)malloc((size_t)sz + 4096);
    if (!seed || !b || fread(seed, 1, (size_t)sz, f) != (size_t)sz) { fclose(f); return 2; }
    fclose(f);
    for (i = 0; i < it; i++) {
        size_t n = (size_t)sz, k;
        int w, h;
        uint8_t *px;
        memcpy(b, seed, n);
        switch (rnd() % 4) {
        case 0: for (k = 0; k < 1 + rnd() % 32; k++) b[rnd() % n] ^= (uint8_t)(1u << (rnd() % 8)); break;
        case 1: for (k = 0; k < 1 + rnd() % 8; k++) b[8 + rnd() % 33] = (uint8_t)rnd(); break;  /* IHDR */
        case 2: n = 1 + (size_t)(rnd() % n); break;                                           /* truncate */
        default: for (k = 0; k < 4; k++) b[8 + 8 + rnd() % 8] = 0xFF; break;                   /* huge dims */
        }
        if (!(f = fopen(tmp, "wb"))) return 3;
        fwrite(b, 1, n, f); fclose(f);
        if (hs_png_load_rgb(tmp, &w, &h, &px) == 0) { ok++; free(px); }
    }
    remove(tmp);
    printf("FUZZ_PNG cases=%u decoded=%u\n", it, ok);
    free(seed); free(b);
    return 0;
}
