/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * gen_demo - builds tests/psx_3d_demo.exe (pure C11): an ORIGINAL, hand-encoded MIPS
 * R3000A PS-EXE (no SDK, no Sony code, no third-party assets). Per frame the
 * program running inside hs-psx's CPU:
 *   * picks a Q12 rotation matrix (64-step table) and loads it into the GTE,
 *   * per octahedron face: LWC2 x6 -> RTPT -> NCLIP (back-face cull) ->
 *     MVMVA (rotate face normal) -> GPF (light colour) -> AVSZ3 -> OT insert,
 *   * DMA6 OTC clears a 16-entry ordering table, background + HUD chains are
 *     linked into it, then DMA2 linked-list mode sends it to GP0,
 *   * a HUD progress bar width is patched from the frame counter,
 *   * once at boot: HLE printf, GP1/GP0 setup, 4-bit texture + CLUT upload
 *     (GP0 A0h), SPU ADPCM upload via DMA4 and a 3-voice C-major chord KON.
 * usage: gen_demo [PSX_ROOT]   (default "."; writes PSX_ROOT/tests/psx_3d_demo.{exe,txt}) */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BASE 0x80010000u
static uint32_t code[8192];
static int ncode;
#define MAXLAB 64
static struct { char name[16]; uint32_t addr; } labels[MAXLAB];
static int nlab;
static struct { int pos; char lab[16]; char kind; } fixups[MAXLAB];
static int nfix;

static void fail(const char *m) { fprintf(stderr, "gen_demo: %s\n", m); exit(1); }
static void emit(uint32_t w) { if (ncode >= (int)(sizeof code / sizeof code[0])) fail("code overflow"); code[ncode++] = w; }
static void label(const char *n) {
    if (nlab >= MAXLAB) fail("too many labels");
    snprintf(labels[nlab].name, sizeof labels[nlab].name, "%s", n);
    labels[nlab++].addr = BASE + (uint32_t)ncode * 4;
}
static uint32_t lookup(const char *n) {
    int i;
    for (i = nlab - 1; i >= 0; i--) if (!strcmp(labels[i].name, n)) return labels[i].addr;
    fail("missing label");
    return 0;
}
static void I(uint32_t op, uint32_t rt, uint32_t rs, int32_t im) { emit((op << 26) | (rs << 21) | (rt << 16) | ((uint32_t)im & 0xFFFF)); }
static void R(uint32_t fn, uint32_t rd, uint32_t rs, uint32_t rt, uint32_t sa) { emit((rs << 21) | (rt << 16) | (rd << 11) | (sa << 6) | fn); }
static void nop(void) { emit(0); }
static void li(uint32_t r, uint32_t v) { I(0x0F, r, 0, (int32_t)((v >> 16) & 0xFFFF)); I(0x0D, r, r, (int32_t)(v & 0xFFFF)); }
static void addiu(uint32_t rt, uint32_t rs, int32_t v) { I(0x09, rt, rs, v); }
static void andi(uint32_t rt, uint32_t rs, int32_t v) { I(0x0C, rt, rs, v); }
static void lui(uint32_t rt, int32_t v) { I(0x0F, rt, 0, v); }
static void lw(uint32_t rt, int32_t off, uint32_t rs) { I(0x23, rt, rs, off); nop(); }   /* load + delay-slot nop */
static void sw(uint32_t rt, int32_t off, uint32_t rs) { I(0x2B, rt, rs, off); }
static void sh(uint32_t rt, int32_t off, uint32_t rs) { I(0x29, rt, rs, off); }
static void sll(uint32_t rd, uint32_t rt, uint32_t sa) { R(0x00, rd, 0, rt, sa); }
static void srl(uint32_t rd, uint32_t rt, uint32_t sa) { R(0x02, rd, 0, rt, sa); }
static void addu(uint32_t rd, uint32_t rs, uint32_t rt) { R(0x21, rd, rs, rt, 0); }
static void subu(uint32_t rd, uint32_t rs, uint32_t rt) { R(0x23, rd, rs, rt, 0); }
static void and_(uint32_t rd, uint32_t rs, uint32_t rt) { R(0x24, rd, rs, rt, 0); }
static void or_(uint32_t rd, uint32_t rs, uint32_t rt) { R(0x25, rd, rs, rt, 0); }
static void slti(uint32_t rt, uint32_t rs, int32_t v) { I(0x0A, rt, rs, v); }
static void jalr(uint32_t rs) { R(0x09, 31, rs, 0, 0); nop(); }
static void fix(const char *l, char kind) {
    if (nfix >= MAXLAB) fail("too many fixups");
    fixups[nfix].pos = ncode; snprintf(fixups[nfix].lab, sizeof fixups[nfix].lab, "%s", l); fixups[nfix].kind = kind; nfix++;
}
static void br(uint32_t op, uint32_t rs, uint32_t rt, const char *l) { fix(l, 'b'); I(op, rt, rs, 0); nop(); }
static void beq(uint32_t rs, uint32_t rt, const char *l) { br(0x04, rs, rt, l); }
static void bne(uint32_t rs, uint32_t rt, const char *l) { br(0x05, rs, rt, l); }
static void blez(uint32_t rs, const char *l) { br(0x06, rs, 0, l); }
static void bgtz(uint32_t rs, const char *l) { br(0x07, rs, 0, l); }
static void j(const char *l) { fix(l, 'j'); emit(0x02u << 26); nop(); }
static void mtc2(uint32_t rt, uint32_t rd) { emit(0x48800000u | (rt << 16) | (rd << 11)); }
static void mfc2(uint32_t rt, uint32_t rd) { emit(0x48000000u | (rt << 16) | (rd << 11)); nop(); }
static void ctc2(uint32_t rt, uint32_t rd) { emit(0x48C00000u | (rt << 16) | (rd << 11)); }
static void cop2(uint32_t cmd) { emit(0x4A000000u | cmd); }
static void lwc2(uint32_t rt, int32_t off, uint32_t rs) { I(0x32, rt, rs, off); }
static void swc2(uint32_t rt, int32_t off, uint32_t rs) { I(0x3A, rt, rs, off); }

#define GTE_RTPT  0x0080030u                                             /* sf=1 */
#define GTE_NCLIP 0x0000006u
#define GTE_MVMVA (0x0080000u | (0u << 17) | (0u << 15) | (3u << 13) | 0x12u) /* sf=1 RT*V0, no translation */
#define GTE_GPF   (0x0080000u | 0x3Du)
#define GTE_AVSZ3 0x000002Du

enum { ZERO, AT, V0, V1, A0, A1, A2, A3, T0, T1, T2, T3, T4, T5, T6, T7,
       S0, S1, S2, S3, S4, S5, S6, S7, T8, T9, RA = 31 };

#define IO    0x1F801000u
#define OT    0x80020000u    /* 16 entries */
#define OTN   16
#define FACEB 0x80021000u    /* per-face packets (10 words each) */
#define BGL   0x80030000u    /* static background chain */
#define HUDL  0x80031000u    /* static HUD chain */
#define MAT   0x80040000u
#define VERT  0x80050000u
#define NORM  0x80050800u
#define TEX   0x80060000u
#define CLUT  0x80060800u
#define SOUND 0x80061000u
#define STR   0x80062000u
#define CNT   0x80070000u

static uint32_t xy(int x, int y) { return ((uint32_t)x & 0xFFFF) | (((uint32_t)y & 0xFFFF) << 16); }

#define R0 150
static const int verts[6][3] = { { 0, -R0, 0 }, { 0, R0, 0 }, { -R0, 0, 0 }, { 0, 0, -R0 }, { R0, 0, 0 }, { 0, 0, R0 } };
static const int faces[8][3] = { { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 5 }, { 0, 5, 2 }, { 1, 3, 2 }, { 1, 4, 3 }, { 1, 5, 4 }, { 1, 2, 5 } };
static const int textured[8] = { 1, 0, 1, 0, 0, 1, 0, 1 };
static const int basecol[8][3] = { { 255, 200, 120 }, { 0, 0, 0 }, { 140, 220, 255 }, { 0, 0, 0 }, { 0, 0, 0 }, { 255, 150, 200 }, { 0, 0, 0 }, { 180, 255, 150 } };
static const uint32_t gcols[8][3] = { { 0 }, { 0xFF4040, 0x40FF40, 0x4040FF }, { 0 }, { 0xFFFF40, 0x40FFFF, 0xFF40FF },
                                      { 0xFF8020, 0x2080FF, 0x80FF20 }, { 0 }, { 0xFFFFFF, 0xFF2080, 0x20FF80 }, { 0 } };

static void normal(int fi, int out[3]) {
    int s[3], k;
    long ss = 0;
    double l;
    for (k = 0; k < 3; k++) s[k] = verts[faces[fi][0]][k] + verts[faces[fi][1]][k] + verts[faces[fi][2]][k];
    for (k = 0; k < 3; k++) ss += (long)s[k] * s[k];
    l = sqrt((double)ss);
    for (k = 0; k < 3; k++) out[k] = (int)rint(s[k] / l * 4096);
}

static void ot_insert(uint32_t node_reg, uint32_t idx_reg, int len_words) {
    /* T6 = &OT[idx]; header = (len<<24) | OT[idx]; OT[idx] = node & 0xFFFFFF */
    sll(T6, idx_reg, 2); addu(T6, T6, S3);
    lw(T7, 0, T6);
    li(T8, 0x00FFFFFF); and_(T7, T7, T8);
    lui(T9, (len_words << 8) & 0xFFFF); or_(T7, T7, T9);
    sw(T7, 0, node_reg);
    and_(T7, node_reg, T8); sw(T7, 0, T6);
}
static void chain_insert(uint32_t first, uint32_t last_hdr, int idx) {
    /* link a static multi-node chain: last header's next = OT[idx]; OT[idx] = first */
    li(T4, first); li(T5, last_hdr); addiu(T3, ZERO, idx);
    sll(T6, T3, 2); addu(T6, T6, S3);
    lw(T7, 0, T6); lw(T9, 0, T5);
    li(T8, 0x00FFFFFF); and_(T7, T7, T8);
    lui(AT, 0xFF00); and_(T9, T9, AT); or_(T9, T9, T7); sw(T9, 0, T5);
    and_(T7, T4, T8); sw(T7, 0, T6);
}

typedef struct { uint32_t w[2048]; int n; } wl;
static void wpush(wl *l, uint32_t v) { if (l->n >= 2048) fail("word list overflow"); l->w[l->n++] = v; }

static uint32_t make_chain(uint32_t base, const wl *words, wl *out) {
    uint32_t addr = base, last = 0;
    int i = 0;
    while (i < words->n) {
        int n = words->n - i < 240 ? words->n - i : 240, k;
        int is_last = i + n >= words->n;
        uint32_t nxt = is_last ? 0x00FFFFFFu : ((addr + 4 + 4 * (uint32_t)n) & 0x1FFFFF);
        last = addr;
        wpush(out, ((uint32_t)n << 24) | nxt);
        for (k = 0; k < n; k++) wpush(out, words->w[i + k]);
        addr += 4 + 4 * (uint32_t)n;
        i += n;
    }
    return last;
}

static void patch_li(int pos, uint32_t reg, uint32_t val) {
    code[pos] = (0x0Fu << 26) | (reg << 16) | ((val >> 16) & 0xFFFF);
    code[pos + 1] = (0x0Du << 26) | (reg << 21) | (reg << 16) | (val & 0xFFFF);
}
static int find_li(int end_pos, uint32_t reg) {
    int p;
    for (p = end_pos - 1; p > 0; p--) if ((code[p] >> 26) == 0x0F && ((code[p] >> 16) & 31) == reg) return p;
    fail("li not found");
    return 0;
}

#define END (CNT + 16)
static unsigned char payload[(END & 0x1FFFFF) - (BASE & 0x1FFFFF)];
static void put(uint32_t addr, const void *data, size_t len) {
    size_t o = (addr & 0x1FFFFF) - (BASE & 0x1FFFFF);
    if (o + len > sizeof payload) fail("payload overflow");
    memcpy(payload + o, data, len);
}
static void put_words(uint32_t addr, const uint32_t *w, int n) {
    int i;
    for (i = 0; i < n; i++) {
        unsigned char b[4] = { (unsigned char)w[i], (unsigned char)(w[i] >> 8), (unsigned char)(w[i] >> 16), (unsigned char)(w[i] >> 24) };
        put(addr + 4 * (uint32_t)i, b, 4);
    }
}
static void put_half(uint32_t addr, const uint16_t *h, int n) {
    int i;
    for (i = 0; i < n; i++) { unsigned char b[2] = { (unsigned char)h[i], (unsigned char)(h[i] >> 8) }; put(addr + 2 * (uint32_t)i, b, 2); }
}
static uint16_t c15(int r, int g, int b) { return (uint16_t)((r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10)); }
static void le32(unsigned char *p, uint32_t v) { p[0] = (unsigned char)v; p[1] = (unsigned char)(v >> 8); p[2] = (unsigned char)(v >> 16); p[3] = (unsigned char)(v >> 24); }

static const char *font_rows(char ch, int y) {
    static const struct { char c; const char *r[5]; } font[] = {
        { 'H', { "101", "101", "111", "101", "101" } }, { 'S', { "111", "100", "111", "001", "111" } },
        { 'P', { "110", "101", "110", "100", "100" } }, { 'X', { "101", "101", "010", "101", "101" } },
        { '-', { "000", "000", "111", "000", "000" } }, { ' ', { "000", "000", "000", "000", "000" } },
        { 'G', { "111", "100", "101", "101", "111" } }, { 'T', { "111", "010", "010", "010", "010" } },
        { 'E', { "111", "100", "110", "100", "111" } }, { 'R', { "110", "101", "110", "101", "101" } },
    };
    size_t i;
    for (i = 0; i < sizeof font / sizeof font[0]; i++) if (font[i].c == ch) return font[i].r[y];
    fail("glyph missing");
    return NULL;
}

int main(int argc, char **argv) {
    const char *root = argc > 1 ? argv[1] : ".";
    static wl bg, hud, bgw, hudw, mat, vd, nd;
    static const uint32_t gp1[] = { 0x00000000, 0x03000000, 0x08000001, 0x04000002, 0x05000000 };
    static const uint32_t gp0[] = { 0xE3000000, 0xE4000000u | 319 | (239 << 10), 0xE5000000, 0xE1000208, 0xE2000000, 0xE6000000 };
    static const int pitches[3] = { 10885, 13714, 16310 };                                /* C4 E4 G4 */
    static const struct { int reg; uint32_t v; } gtec[] = { { 24, 160u << 16 }, { 25, 120u << 16 }, { 26, 300 }, { 5, 0 }, { 6, 0 }, { 7, 600 }, { 29, 1365 }, { 30, 1024 } };
    int bg_patch, hud_patch, bar_patch, bar_word_index, fi, n, k, i, p;
    uint32_t bg_last, hud_last, bar_addr;
    uint16_t tex[64 * 16], clut[16];
    unsigned char snd[16 * 16], hdr[0x800];
    static const char str[] = "hs-psx demo: R3000A+GTE+DMA2/6+SPU, frame %d\n";
    static const char title[40] = "HandshakeOS hs-psx original demo (MIT)";
    char path[4096];
    FILE *f;
    int ntex = 0, ntextured = 0;

    label("start");
    li(S0, IO);
    li(S3, OT);
    li(S7, MAT);
    addiu(S2, ZERO, 0);
    /* HLE BIOS printf via the A0 vector (A(3Fh)) */
    li(A0, STR); addiu(T1, ZERO, 0x3F); addiu(T0, ZERO, 0xA0); jalr(T0);
    /* GPU setup: GP1 reset, display on, 320x240 15bpp NTSC, DMA dir CPU->GPU */
    for (i = 0; i < 5; i++) { li(T0, gp1[i]); sw(T0, 0x814, S0); }
    /* GP0 env: draw area (0,0)-(319,239), offset 0, texpage x=512 4bpp + dither */
    for (i = 0; i < 6; i++) { li(T0, gp0[i]); sw(T0, 0x810, S0); }
    /* 64x64 4-bit texture -> VRAM (512,0) (16 halfwords wide) */
    li(T0, 0xA0000000); sw(T0, 0x810, S0); li(T0, xy(512, 0)); sw(T0, 0x810, S0); li(T0, xy(16, 64)); sw(T0, 0x810, S0);
    li(T2, TEX); addiu(T3, ZERO, 512);
    label("tex");
    lw(T0, 0, T2); addiu(T2, T2, 4); sw(T0, 0x810, S0); addiu(T3, T3, -1); bgtz(T3, "tex");
    /* 16-colour CLUT -> VRAM (512,128) */
    li(T0, 0xA0000000); sw(T0, 0x810, S0); li(T0, xy(512, 128)); sw(T0, 0x810, S0); li(T0, xy(16, 1)); sw(T0, 0x810, S0);
    li(T2, CLUT); addiu(T3, ZERO, 8);
    label("clut");
    lw(T0, 0, T2); addiu(T2, T2, 4); sw(T0, 0x810, S0); addiu(T3, T3, -1); bgtz(T3, "clut");
    /* SPU: enable, DMA-write mode, transfer address 0x1000, DMA4 256 bytes */
    li(T0, 0xC020); sh(T0, 0xDAA, S0);
    addiu(T0, ZERO, 0x1000 / 8); sh(T0, 0xDA6, S0);
    li(T0, 0x0BBBBBBB); sw(T0, 0x0F0, S0);                                               /* DPCR: enable all channels */
    li(T0, SOUND); sw(T0, 0x0C0, S0);
    li(T0, 0x00010040); sw(T0, 0x0C4, S0);
    li(T0, 0x01000201); sw(T0, 0x0C8, S0);
    li(T0, 0xC000); sh(T0, 0xDAA, S0);
    li(T0, 0x3FFF); sh(T0, 0xD80, S0); sh(T0, 0xD82, S0);
    for (i = 0; i < 3; i++) {
        const int offs[7] = { 0, 2, 4, 6, 8, 10, 14 };
        const uint32_t vals[7] = { 0x1400, 0x1400, (uint32_t)pitches[i], 0x1000 / 8, 0x000F, 0x1F00, 0x1000 / 8 };
        for (k = 0; k < 7; k++) { li(T0, vals[k]); sh(T0, 0xC00 + i * 16 + offs[k], S0); }
    }
    addiu(T0, ZERO, 7); sh(T0, 0xD88, S0); sh(ZERO, 0xD8A, S0);
    /* GTE constants: OFX/OFY (160,120), H=300, TR=(0,0,600), ZSF3=1365 */
    for (i = 0; i < 8; i++) { li(T0, gtec[i].v); ctc2(T0, (uint32_t)gtec[i].reg); }

    label("frame");
    /* ---- DMA6 OTC: clear OT (reverse linked) ---- */
    li(T0, OT + (OTN - 1) * 4); sw(T0, 0x0E0, S0);
    addiu(T0, ZERO, OTN); sw(T0, 0x0E4, S0);
    li(T0, 0x11000002); sw(T0, 0x0E8, S0);
    /* ---- rotation matrix for this frame: MAT + (frame & 63) * 20 ---- */
    andi(T0, S2, 63); sll(T1, T0, 2); sll(T2, T0, 4); addu(T0, T1, T2); addu(S1, S7, T0);
    for (n = 0; n < 5; n++) { lw(T0, n * 4, S1); ctc2(T0, (uint32_t)n); }

    /* ---- per-face transform, cull, light, sort ---- */
    for (fi = 0; fi < 8; fi++) {
        uint32_t node = FACEB + (uint32_t)fi * 64, vaddr = VERT + (uint32_t)fi * 24;
        char lc[16], ll[16], lz[16], lz2[16];
        int plen;
        snprintf(lc, sizeof lc, "cull%d", fi); snprintf(ll, sizeof ll, "lit%d", fi);
        snprintf(lz, sizeof lz, "okz%d", fi); snprintf(lz2, sizeof lz2, "okz2%d", fi);
        li(S4, vaddr); li(S5, node);
        for (n = 0; n < 6; n++) lwc2((uint32_t)n, n * 4, S4);
        cop2(GTE_RTPT);
        cop2(GTE_NCLIP);
        mfc2(T0, 24);                                                                      /* MAC0 */
        blez(T0, lc);                                                                      /* back-facing -> skip */
        if (textured[fi]) {
            /* packet: hdr, 0x34|rgb0, xy0, clut|uv0, rgb1, xy1, tpage|uv1, rgb2, xy2, uv2 */
            swc2(12, 8, S5); swc2(13, 20, S5); swc2(14, 32, S5);
            /* light: V0 = face normal, MVMVA -> IR3, intensity = 0x600 + max(0,-IR3)*5/8 */
            li(T0, NORM + (uint32_t)fi * 8); lwc2(0, 0, T0); lwc2(1, 4, T0);
            cop2(GTE_MVMVA);
            mfc2(T1, 11);
            subu(T1, ZERO, T1);
            bgtz(T1, ll);
            addiu(T1, ZERO, 0);
            label(ll);
            srl(T2, T1, 1); srl(T3, T1, 3); addu(T1, T2, T3); addiu(T1, T1, 0x600);
            mtc2(T1, 8);                                                                   /* IR0 */
            for (k = 0; k < 3; k++) { addiu(T0, ZERO, basecol[fi][k] << 4); mtc2(T0, 9 + (uint32_t)k); }
            cop2(GTE_GPF);
            mfc2(T0, 22);                                                                  /* RGB2 (CODE byte 0 from RGBC=0) */
            li(T1, 0x00FFFFFF); and_(T0, T0, T1);
            sw(T0, 16, S5); sw(T0, 28, S5);
            lui(T1, 0x3400); or_(T0, T0, T1); sw(T0, 4, S5);
            plen = 9;
        } else {
            /* packet: hdr, 0x30|rgb0, xy0, rgb1, xy1, rgb2, xy2 */
            swc2(12, 8, S5); swc2(13, 16, S5); swc2(14, 24, S5);
            plen = 6;
        }
        cop2(GTE_AVSZ3);
        mfc2(T0, 7);                                                                       /* OTZ */
        srl(T0, T0, 6);
        slti(T1, T0, OTN - 2); bne(T1, ZERO, lz);
        addiu(T0, ZERO, OTN - 2);
        label(lz);
        bgtz(T0, lz2);
        addiu(T0, ZERO, 1);
        label(lz2);
        ot_insert(S5, T0, plen);
        label(lc);
    }

    /* ---- static chains: background at the far end, HUD at OT[0] ---- */
    chain_insert(BGL, 0, OTN - 1);                                                         /* patched below with real last-header address */
    bg_patch = ncode;
    chain_insert(HUDL, 0, 0);
    hud_patch = ncode;
    /* ---- HUD progress bar width from the frame counter ---- */
    andi(T0, S2, 255); addiu(T0, T0, 32); li(T1, 0); sh(T0, 0, T1);                       /* patched address */
    bar_patch = ncode;
    /* ---- DMA2 linked list from OT[OTN-1] ---- */
    li(T0, OT + (OTN - 1) * 4); sw(T0, 0x0A0, S0);
    sw(ZERO, 0x0A4, S0);
    li(T0, 0x01000401); sw(T0, 0x0A8, S0);
    /* ---- frame counter, ack VBLANK, wait for the next one ---- */
    addiu(S2, S2, 1); li(T0, CNT); sw(S2, 0, T0);
    li(T0, 0xFFFFFFFE); sw(T0, 0x070, S0);
    label("wait");
    lw(T0, 0x070, S0); andi(T0, T0, 1); beq(T0, ZERO, "wait");
    j("frame");

    /* ---------------- GPU packets ---------------- */
    wpush(&bg, 0x02281810); wpush(&bg, xy(0, 0)); wpush(&bg, xy(320, 240));                /* fill (dark blue) */
    wpush(&bg, 0x228040C0); wpush(&bg, xy(-40, 250)); wpush(&bg, xy(150, -20)); wpush(&bg, xy(360, 250)); /* semi-transparent flat tri */
    for (k = -260; k < 340; k += 30) { wpush(&bg, 0x50503010); wpush(&bg, xy(k, 0)); wpush(&bg, 0x00A06020); wpush(&bg, xy(k + 240, 240)); }
    {
        const uint32_t fq[8] = { 0x3820A0FF, xy(-10, 200), 0x00FF6020, xy(330, 200), 0x0020FF60, xy(-10, 240), 0x00FF20A0, xy(330, 240) };
        for (k = 0; k < 8; k++) wpush(&bg, fq[k]);                                         /* Gouraud floor quad */
    }
    wpush(&hud, 0x62102030); wpush(&hud, xy(0, 0)); wpush(&hud, xy(320, 18));              /* semi-transparent top band */
    wpush(&hud, 0x6000F0FF); wpush(&hud, xy(8, 228)); wpush(&hud, xy(32, 6));              /* progress bar (width patched) */
    bar_word_index = hud.n - 1;
    {
        const char *text = "HS-PSX GTE";
        int ci, y, x;
        for (ci = 0; text[ci]; ci++)
            for (y = 0; y < 5; y++)
                for (x = 0; x < 3; x++)
                    if (font_rows(text[ci], y)[x] == '1') { wpush(&hud, 0x60FFFFFF); wpush(&hud, xy(10 + ci * 16 + x * 4, 3 + y * 3 - 1)); wpush(&hud, xy(4, 3)); }
    }
    /* textured sprite: the texture itself as a HUD icon (raw texture, 0x65) */
    wpush(&hud, 0x65808080); wpush(&hud, xy(270, 190)); wpush(&hud, (0x2020u << 16) | 0x0000); wpush(&hud, xy(48, 48));

    bg_last = make_chain(BGL, &bg, &bgw);
    hud_last = make_chain(HUDL, &hud, &hudw);
    if (bg.n > 240) fail("bg chain too long");
    patch_li(find_li(bg_patch, T5), T5, bg_last);
    patch_li(find_li(hud_patch, T5), T5, hud_last);
    /* bar address: offset of bar word in HUD chain (single chunk header + index) */
    bar_addr = HUDL + 4 + (uint32_t)bar_word_index * 4;
    for (p = bar_patch - 1; p > 0; p--)
        if ((code[p] >> 26) == 0x0F && ((code[p] >> 16) & 31) == T1) { patch_li(p, T1, bar_addr); break; }

    for (i = 0; i < nfix; i++) {
        uint32_t a = lookup(fixups[i].lab);
        int pos = fixups[i].pos;
        if (fixups[i].kind == 'b') {
            int64_t off = ((int64_t)a - (int64_t)(BASE + (uint32_t)pos * 4 + 4)) / 4;
            if (off < -32768 || off >= 32768) fail("branch out of range");
            code[pos] = (code[pos] & 0xFFFF0000u) | ((uint32_t)off & 0xFFFF);
        } else {
            code[pos] |= (a >> 2) & 0x3FFFFFF;
        }
    }

    /* ---------------- memory image ---------------- */
    put_words(BASE, code, ncode);
    put_words(BGL, bgw.w, bgw.n);
    put_words(HUDL, hudw.w, hudw.n);
    for (fi = 0; fi < 8; fi++) {                                                           /* per-face packet templates */
        uint32_t node = FACEB + (uint32_t)fi * 64;
        if (textured[fi]) {
            uint32_t pk[10] = { 0, 0x34FFFFFF, 0, (0x2020u << 16) | xy(0, 0), 0xFFFFFF, 0, (0x0208u << 16) | 0x003F, 0xFFFFFF, 0, 0x3F20 };
            pk[6] = (0x0008u << 16) | 0x003F;
            put_words(node, pk, 10);
        } else {
            uint32_t pk[7] = { 0, 0x30000000u | gcols[fi][0], 0, gcols[fi][1], 0, gcols[fi][2], 0 };
            put_words(node, pk, 7);
        }
    }
    for (n = 0; n < 64; n++) {
        double a = n * 2 * M_PI / 64;
        double t = 0.45 + 0.25 * sin(a);
        double ca = cos(a), sa = sin(a), ct = cos(t), st = sin(t);
        double m[9] = { ca, sa * st, sa * ct, 0, ct, -st, -sa, ca * st, ca * ct };
        uint32_t q[9];
        for (k = 0; k < 9; k++) q[k] = (uint32_t)(int64_t)rint(m[k] * 4096) & 0xFFFF;
        wpush(&mat, q[0] | (q[1] << 16)); wpush(&mat, q[2] | (q[3] << 16)); wpush(&mat, q[4] | (q[5] << 16));
        wpush(&mat, q[6] | (q[7] << 16)); wpush(&mat, q[8]);
    }
    put_words(MAT, mat.w, mat.n);
    for (fi = 0; fi < 8; fi++)
        for (k = 0; k < 3; k++) {
            const int *v = verts[faces[fi][k]];
            wpush(&vd, xy(v[0], v[1])); wpush(&vd, (uint32_t)v[2] & 0xFFFF);
        }
    put_words(VERT, vd.w, vd.n);
    for (fi = 0; fi < 8; fi++) {
        int nv[3];
        normal(fi, nv);
        wpush(&nd, xy(nv[0], nv[1])); wpush(&nd, (uint32_t)nv[2] & 0xFFFF);
    }
    put_words(NORM, nd.w, nd.n);
    {
        int y, x;
        for (y = 0; y < 64; y++)
            for (x = 0; x < 64; x += 4) {
                int nn = 0;
                for (k = 0; k < 4; k++) {
                    int cx = (x + k) / 8, cy = y / 8;
                    int idx = 1 + ((cx + cy) & 1) + (((cx ^ cy) & 4) ? 2 : 0);
                    if ((x + k - 32) * (x + k - 32) + (y - 32) * (y - 32) < 64) idx = 5;
                    nn |= idx << (k * 4);
                }
                tex[ntex++] = (uint16_t)nn;
            }
    }
    put_half(TEX, tex, ntex);
    clut[0] = 0; clut[1] = c15(255, 255, 255); clut[2] = c15(70, 90, 200); clut[3] = c15(255, 230, 90);
    clut[4] = c15(200, 60, 60); clut[5] = c15(20, 20, 20);
    for (k = 6; k < 16; k++) clut[k] = c15(255, 0, 255);
    put_half(CLUT, clut, 16);
    /* ADPCM: one 448-sample sine period (16 blocks, filter 0, shift 2), loop start/end+repeat */
    for (i = 0; i < 16; i++) {
        int nib[28];
        for (k = 0; k < 28; k++) {
            int v = (int)rint(sin(2 * M_PI * (i * 28 + k) / 448) * 7.4);
            v = v > 7 ? 7 : v;
            v = v < -8 ? -8 : v;
            nib[k] = v & 15;
        }
        snd[i * 16] = 0x02;
        snd[i * 16 + 1] = (unsigned char)(i == 0 ? 4 : (i == 15 ? 3 : 0));
        for (k = 0; k < 28; k += 2) snd[i * 16 + 2 + k / 2] = (unsigned char)(nib[k] | (nib[k + 1] << 4));
    }
    put(SOUND, snd, sizeof snd);
    put(STR, str, sizeof str);                                                             /* includes the trailing NUL */

    memset(hdr, 0, sizeof hdr);
    memcpy(hdr, "PS-X EXE", 8);
    le32(hdr + 0x10, BASE); le32(hdr + 0x14, 0); le32(hdr + 0x18, BASE); le32(hdr + 0x1C, (uint32_t)sizeof payload); le32(hdr + 0x20, 0);
    le32(hdr + 0x30, 0x801FFF00); le32(hdr + 0x34, 0);
    memcpy(hdr + 0x4C, title, 40);
    snprintf(path, sizeof path, "%s/tests/psx_3d_demo.exe", root);
    f = fopen(path, "wb");
    if (!f || fwrite(hdr, 1, sizeof hdr, f) != sizeof hdr || fwrite(payload, 1, sizeof payload, f) != sizeof payload || fclose(f)) { perror(path); return 1; }
    for (fi = 0; fi < 8; fi++) ntextured += textured[fi];
    snprintf(path, sizeof path, "%s/tests/psx_3d_demo.txt", root);
    f = fopen(path, "w");
    if (!f) { perror(path); return 1; }
    fprintf(f, "tests/psx_3d_demo.exe - generated by tools/gen_demo.c (original, MIT)\n"
               "MIPS R3000A instructions: %d\nbackground chain words: %d\nHUD chain words: %d\n"
               "face packets: %d (textured+lit: %d, Gouraud: %d)\nfile bytes: %d\n",
            ncode, bgw.n, hudw.n, 8, ntextured, 8 - ntextured, (int)(0x800 + sizeof payload));
    if (fclose(f)) return 1;
    printf("wrote %s/tests/psx_3d_demo.exe %d bytes, %d instructions\n", root, (int)(0x800 + sizeof payload), ncode);
    return 0;
}
