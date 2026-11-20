/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * acid2_compare - pixel-compare hs-gbc acid2 renders with the official reference images
 * (pure C11; PNG decoding by hs_pngread.h). usage: acid2_compare OUTDIR REFDIR */
#include "hs_pngread.h"

static int exists(const char *p) { FILE *f = fopen(p, "rb"); if (f) fclose(f); return f != NULL; }

/* DMG run uses palette 1 (pocket b&w): map its 4 shades to the reference greys */
static int dmg_grey(uint32_t rgb) {
    switch (rgb) {
    case 0xE0E0D8: return 255;
    case 0xA8A8A0: return 170;
    case 0x585850: return 85;
    case 0x101010: return 0;
    default: return -1;
    }
}

int main(int argc, char **argv) {
    static const char *names[2][2] = { { "dmg-acid2.gb_native.png", "dmg-acid2.reference.png" },
                                       { "cgb-acid2.gbc_native.png", "cgb-acid2.reference.png" } };
    int t;
    if (argc != 3) { fprintf(stderr, "usage: acid2_compare OUTDIR REFDIR\n"); return 2; }
    for (t = 0; t < 2; t++) {
        char a[4096], b[4096];
        int aw, ah, bw, bh, x, y, bad = 0, dmg = (t == 0);
        uint8_t *A, *B;
        snprintf(a, sizeof a, "%s/%s", argv[1], names[t][0]);
        snprintf(b, sizeof b, "%s/%s", argv[2], names[t][1]);
        if (!(exists(a) && exists(b))) { printf("ACID2 %s: skipped (missing)\n", names[t][0]); continue; }
        if (hs_png_load_rgb(a, &aw, &ah, &A) || hs_png_load_rgb(b, &bw, &bh, &B) ||
            aw < 160 || ah < 144 || bw < 160 || bh < 144) { printf("ACID2 %s: decode failed\n", names[t][0]); return 1; }
        for (y = 0; y < 144; y++) {
            for (x = 0; x < 160; x++) {
                const uint8_t *pa = A + ((size_t)y * aw + x) * 3, *pb = B + ((size_t)y * bw + x) * 3;
                if (dmg) {
                    int v = dmg_grey(((uint32_t)pa[0] << 16) | ((uint32_t)pa[1] << 8) | pa[2]);
                    bad += abs(v - pb[0]) > 10;
                } else {
                    int k, m = 0;
                    for (k = 0; k < 3; k++) { int d = abs(pa[k] - pb[k]); if (d > m) m = d; }
                    bad += m > 8;
                }
            }
        }
        printf("ACID2 %s/%s: %d/%d pixels match reference -> %s\n", argv[1], names[t][0], 160 * 144 - bad, 160 * 144,
               bad == 0 ? "PASS" : "FAIL");
        free(A); free(B);
    }
    return 0;
}
