/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_testroms.c — synthetic clean-room LoROM test programs for hs-snes.
 * All data is generated with integer arithmetic only (bit-identical ROMs
 * on x86_64 and ARM64).
 */
#include <string.h>
#include "snes_testroms.h"

typedef struct asmb { uint8_t *rom; size_t cap; uint32_t pc; } asmb;

static void e8(asmb *a, uint8_t v) { if (a->pc < a->cap) a->rom[a->pc] = v; a->pc++; }
static void e16(asmb *a, uint16_t v) { e8(a, (uint8_t)v); e8(a, (uint8_t)(v >> 8)); }
static void e24(asmb *a, uint32_t v) { e16(a, (uint16_t)v); e8(a, (uint8_t)(v >> 16)); }
static uint16_t here(const asmb *a) { return (uint16_t)(0x8000u | (a->pc & 0x7FFFu)); }
static uint32_t snes_addr(uint32_t off) { return ((off >> 15) << 16) | 0x8000u | (off & 0x7FFFu); }
static void op1(asmb *a, uint8_t op) { e8(a, op); }
static void op2(asmb *a, uint8_t op, uint8_t b) { e8(a, op); e8(a, b); }
static void op3(asmb *a, uint8_t op, uint16_t w) { e8(a, op); e16(a, w); }
static void op4(asmb *a, uint8_t op, uint32_t l) { e8(a, op); e24(a, l); }
/* branch to a known (backward) target */
static void bto(asmb *a, uint8_t op, uint16_t target) { e8(a, op); e8(a, (uint8_t)(target - (uint16_t)(here(a) + 1u))); }
/* forward branch: returns patch position */
static uint32_t bfwd(asmb *a, uint8_t op) { e8(a, op); e8(a, 0); return a->pc - 1u; }
static void bpatch(asmb *a, uint32_t pos) { a->rom[pos] = (uint8_t)(a->pc - (pos + 1u)); }

/* 8-bit A / 16-bit X helpers */
static void wreg(asmb *a, uint16_t reg, uint8_t v) { if (v) { op2(a, 0xA9, v); op3(a, 0x8D, reg); } else op3(a, 0x9C, reg); }
static void ldx(asmb *a, uint16_t v) { op3(a, 0xA2, v); }
static void stx(asmb *a, uint16_t r) { op3(a, 0x8E, r); }
static void stl(asmb *a, uint32_t l) { op4(a, 0x8F, l); }   /* sta long */
static void dma(asmb *a, int ch, uint8_t mode, uint8_t bbad, uint32_t src24, uint16_t len)
{
    uint16_t b = (uint16_t)(0x4300u + ch * 16);
    wreg(a, b, mode);
    wreg(a, (uint16_t)(b + 1u), bbad);
    ldx(a, (uint16_t)src24); stx(a, (uint16_t)(b + 2u));
    wreg(a, (uint16_t)(b + 4u), (uint8_t)(src24 >> 16));
    ldx(a, len); stx(a, (uint16_t)(b + 5u));
    wreg(a, 0x420B, (uint8_t)(1u << ch));
}
static void dma_vram(asmb *a, uint16_t waddr, uint32_t src24, uint16_t len)
{
    wreg(a, 0x2115, 0x80);
    ldx(a, waddr); stx(a, 0x2116);
    dma(a, 0, 1, 0x18, src24, len);
}

static void header(uint8_t *rom, size_t size, const char *title, uint16_t reset, uint16_t nmi,
                   uint16_t irq, uint16_t cop, uint16_t brk)
{
    uint32_t h = 0x7FC0u, sum = 0, i;
    uint8_t rsz = 0;
    size_t t;
    memset(rom + h, ' ', 21);
    for (i = 0; i < 21 && title[i]; i++) rom[h + i] = (uint8_t)title[i];
    rom[h + 0x15] = 0x20; rom[h + 0x16] = 0x00;
    for (t = 1024; t < size; t <<= 1) rsz++;
    rom[h + 0x17] = rsz; rom[h + 0x18] = 0; rom[h + 0x19] = 1; rom[h + 0x1A] = 0; rom[h + 0x1B] = 0;
    for (i = 0x7FE0u; i < 0x8000u; i++) rom[i] = 0;
    rom[0x7FE4] = (uint8_t)cop; rom[0x7FE5] = (uint8_t)(cop >> 8);
    rom[0x7FE6] = (uint8_t)brk; rom[0x7FE7] = (uint8_t)(brk >> 8);
    rom[0x7FEA] = (uint8_t)nmi; rom[0x7FEB] = (uint8_t)(nmi >> 8);
    rom[0x7FEE] = (uint8_t)irq; rom[0x7FEF] = (uint8_t)(irq >> 8);
    rom[0x7FFA] = (uint8_t)nmi; rom[0x7FFB] = (uint8_t)(nmi >> 8);
    rom[0x7FFC] = (uint8_t)reset; rom[0x7FFD] = (uint8_t)(reset >> 8);
    rom[0x7FFE] = (uint8_t)irq; rom[0x7FFF] = (uint8_t)(irq >> 8);
    rom[h + 0x1C] = 0xFF; rom[h + 0x1D] = 0xFF; rom[h + 0x1E] = 0; rom[h + 0x1F] = 0;
    for (t = 0; t < size; t++) sum += rom[t];
    rom[h + 0x1E] = (uint8_t)sum; rom[h + 0x1F] = (uint8_t)(sum >> 8);
    rom[h + 0x1C] = (uint8_t)~sum; rom[h + 0x1D] = (uint8_t)((~sum) >> 8);
}

/* integer sine (Bhaskara I), angle in 1/256 turns, result * 256 */
static int isin256(int ang)
{
    int d, num, den, v, neg = 0;
    ang &= 255;
    if (ang >= 128) { neg = 1; ang -= 128; }
    d = (ang * 180) / 128;            /* degrees 0..180 */
    num = 4 * d * (180 - d);
    den = 40500 - d * (180 - d);
    v = (num * 256) / den;
    return neg ? -v : v;
}
static int icos256(int ang) { return isin256(ang + 64); }

static void enc_4bpp(const uint8_t pix[64], uint8_t out[32])
{
    int r, x;
    memset(out, 0, 32);
    for (r = 0; r < 8; r++)
        for (x = 0; x < 8; x++) {
            uint8_t c = pix[r * 8 + x], bit = (uint8_t)(0x80u >> x);
            if (c & 1) out[2 * r] |= bit;
            if (c & 2) out[2 * r + 1] |= bit;
            if (c & 4) out[16 + 2 * r] |= bit;
            if (c & 8) out[16 + 2 * r + 1] |= bit;
        }
}
static void enc_2bpp(const uint8_t pix[64], uint8_t out[16])
{
    int r, x;
    memset(out, 0, 16);
    for (r = 0; r < 8; r++)
        for (x = 0; x < 8; x++) {
            uint8_t c = pix[r * 8 + x], bit = (uint8_t)(0x80u >> x);
            if (c & 1) out[2 * r] |= bit;
            if (c & 2) out[2 * r + 1] |= bit;
        }
}
static uint16_t bgr(int r, int g, int b) { return (uint16_t)((r & 31) | ((g & 31) << 5) | ((b & 31) << 10)); }

/* ---------------------------------------------------------------- SPC */
/* builds the SPC700 tone program + directory + BRR sample at ARAM $0200 */
static uint16_t build_spc_blob(uint8_t *b)
{
    static const uint8_t regs[][2] = {
        {0x6C, 0x20}, {0x5D, 0x03}, {0x00, 0x60}, {0x01, 0x50}, {0x02, 0x00}, {0x03, 0x08},
        {0x04, 0x00}, {0x05, 0x8F}, {0x06, 0xE0}, {0x10, 0x30}, {0x11, 0x40}, {0x12, 0x00},
        {0x13, 0x06}, {0x14, 0x01}, {0x15, 0xFF}, {0x16, 0xE0}, {0x0C, 0x7F}, {0x1C, 0x7F},
        {0x2C, 0x20}, {0x3C, 0x20}, {0x0D, 0x30}, {0x6D, 0x60}, {0x7D, 0x01}, {0x0F, 0x7F},
        {0x4D, 0x02}, {0x5C, 0x00}, {0x6C, 0x00}, {0x4C, 0x03}
    };
    uint16_t n = 0;
    unsigned i;
    int k;
    for (i = 0; i < sizeof regs / sizeof regs[0]; i++) {
        b[n++] = 0x8F; b[n++] = regs[i][0]; b[n++] = 0xF2;   /* mov $F2,#reg */
        b[n++] = 0x8F; b[n++] = regs[i][1]; b[n++] = 0xF3;   /* mov $F3,#val */
    }
    b[n++] = 0x8F; b[n++] = 0x5A; b[n++] = 0xF4;             /* mov $F4,#$5A (ready) */
    b[n++] = 0x8F; b[n++] = 0x20; b[n++] = 0xFA;             /* T0 target = 32 (250 Hz) */
    b[n++] = 0x8F; b[n++] = 0x01; b[n++] = 0xF1;             /* enable T0, IPL off */
    /* loop: mov a,$FD ; beq loop ; clrc ; adc a,$10 ; mov $10,a ; mov $F5,a ; bra loop */
    {
        uint16_t loop = n;
        b[n++] = 0xE4; b[n++] = 0xFD;
        b[n] = 0xF0; b[n + 1] = (uint8_t)(loop - (n + 2)); n += 2;
        b[n++] = 0x60;
        b[n++] = 0x84; b[n++] = 0x10;
        b[n++] = 0xC4; b[n++] = 0x10;
        b[n++] = 0xC4; b[n++] = 0xF5;
        b[n] = 0x2F; b[n + 1] = (uint8_t)(loop - (n + 2)); n += 2;
    }
    /* pad to $0300: directory (src 0 and src 1 at $0310 / $0322) */
    while (n < 0x100) b[n++] = 0;
    b[n++] = 0x10; b[n++] = 0x03; b[n++] = 0x10; b[n++] = 0x03;
    b[n++] = 0x22; b[n++] = 0x03; b[n++] = 0x22; b[n++] = 0x03;
    while (n < 0x110) b[n++] = 0;
    /* src 0 @ $0310: two-block looping saw-ish wave, shift 11 */
    b[n++] = 0xB0;
    for (k = 0; k < 8; k++) b[n++] = (uint8_t)((((k * 2) & 15) << 4) | (((k * 2 + 1) & 15)));
    b[n++] = 0xB3;
    for (k = 0; k < 8; k++) b[n++] = (uint8_t)(((((k * 2) ^ 8) & 15) << 4) | ((((k * 2 + 1) ^ 8) & 15)));
    /* src 1 @ $0322: square, filter 1, shift 10, loop */
    b[n++] = 0xA4;
    for (k = 0; k < 8; k++) b[n++] = k < 4 ? 0x77 : 0x99;
    b[n++] = 0xA7;
    for (k = 0; k < 8; k++) b[n++] = k < 4 ? 0x99 : 0x77;
    return n;
}

/* 65C816 upload routine (M=8, X=16). src24 = blob, n bytes, ARAM $0200 */
static uint16_t emit_spc_upload(asmb *a, uint32_t src24, uint16_t n)
{
    uint16_t start = here(a), w0, w1, lp, w2, w3, w4;
    uint8_t kick = (uint8_t)(n + 1u);
    if (!kick) kick = 1;
    w0 = here(a);
    op3(a, 0xAD, 0x2140); op2(a, 0xC9, 0xAA); bto(a, 0xD0, w0);
    op3(a, 0xAD, 0x2141); op2(a, 0xC9, 0xBB); bto(a, 0xD0, w0);
    op2(a, 0xA9, 0xAA); stl(a, TR_RES + 0x100);
    wreg(a, 0x2142, 0x00); wreg(a, 0x2143, 0x02);
    wreg(a, 0x2141, 0x01); wreg(a, 0x2140, 0xCC);
    w1 = here(a); op3(a, 0xCD, 0x2140); bto(a, 0xD0, w1);
    ldx(a, 0);
    lp = here(a);
    op4(a, 0xBF, src24);           /* lda src,x */
    op3(a, 0x8D, 0x2141);
    op1(a, 0x8A);                  /* txa (low byte) */
    op3(a, 0x8D, 0x2140);
    w2 = here(a); op3(a, 0xCD, 0x2140); bto(a, 0xD0, w2);
    op1(a, 0xE8); op3(a, 0xE0, n); bto(a, 0xD0, lp);
    wreg(a, 0x2142, 0x00); wreg(a, 0x2143, 0x02); wreg(a, 0x2141, 0x00);
    op2(a, 0xA9, kick); op3(a, 0x8D, 0x2140);
    w3 = here(a); op3(a, 0xCD, 0x2140); bto(a, 0xD0, w3);
    w4 = here(a); op3(a, 0xAD, 0x2140); op2(a, 0xC9, 0x5A); bto(a, 0xD0, w4);
    stl(a, TR_RES + 0x101);
    op1(a, 0x60);                  /* rts */
    return start;
}

static void emit_init(asmb *a)
{
    op1(a, 0x78); op1(a, 0x18); op1(a, 0xFB);       /* sei clc xce */
    op2(a, 0xC2, 0x30);
    ldx(a, 0x1FFF); op1(a, 0x9A);                   /* ldx #$1fff ; txs */
    op3(a, 0xA9, 0x0000); op1(a, 0x5B);             /* lda #0 ; tcd */
    op2(a, 0xE2, 0x20);                             /* sep #$20 */
    wreg(a, 0x2100, 0x8F);
    wreg(a, 0x4200, 0x00);
    wreg(a, 0x420C, 0x00);
}

/* ============================================================ CPU torture */
size_t tr_build_cpu_rom(uint8_t *rom, size_t cap, tr_info *info)
{
    asmb A = {rom, cap, 0}, *a = &A;
    uint16_t reset, nmi, irq, cop, sub1, sub2, fill, wl;
    uint32_t data_off = 0x8000u, p1, p2, p3, p4;
    size_t size = 0x10000u;
    int i;
    memset(info, 0, sizeof *info);
    /* data bank 1: 32 bytes i*3+1, HDMA table */
    for (i = 0; i < 32; i++) rom[data_off + (uint32_t)i] = (uint8_t)(i * 3 + 1);
    rom[data_off + 0x40] = 0x84; rom[data_off + 0x41] = 0x11; rom[data_off + 0x42] = 0x22;
    rom[data_off + 0x43] = 0x33; rom[data_off + 0x44] = 0x44; rom[data_off + 0x45] = 0x00;

    /* subroutines / handlers first */
    p1 = bfwd(a, 0x80);  /* bra over handlers (placed at reset entry later) */
    sub1 = here(a); op3(a, 0xA9, 0x1111); op1(a, 0x60);
    sub2 = here(a); op3(a, 0xA9, 0x2222); op1(a, 0x6B);
    cop = here(a); op3(a, 0xA9, 0x3333); op1(a, 0x40);
    nmi = here(a);
    op1(a, 0x08); op2(a, 0xE2, 0x20); op1(a, 0x48);
    op3(a, 0xAD, 0x4210);
    op4(a, 0xAF, TR_RES + 0x20); op1(a, 0x1A); stl(a, TR_RES + 0x20);
    op3(a, 0xAD, 0x4218); stl(a, TR_RES + 0x26);
    op3(a, 0xAD, 0x4219); stl(a, TR_RES + 0x27);
    op1(a, 0x68); op1(a, 0x28); op1(a, 0x40);
    irq = here(a);
    op1(a, 0x08); op2(a, 0xE2, 0x20); op1(a, 0x48);
    op3(a, 0xAD, 0x4211);
    op4(a, 0xAF, TR_RES + 0x21); op1(a, 0x1A); stl(a, TR_RES + 0x21);
    op3(a, 0xAD, 0x2137); op3(a, 0xAD, 0x213D); stl(a, TR_RES + 0x22);
    op3(a, 0xAD, 0x213D); op2(a, 0x29, 0x01); stl(a, TR_RES + 0x23);
    op1(a, 0x68); op1(a, 0x28); op1(a, 0x40);
    bpatch(a, p1);
    reset = here(a);

    /* T1: emulation mode ADC + P + S */
    op2(a, 0xA9, 0xFF); op1(a, 0x18); op2(a, 0x69, 0x01);
    op1(a, 0x08); op1(a, 0x68); stl(a, TR_RES + 0x00);            /* expect $37 */
    op1(a, 0xBA); op1(a, 0x8A); stl(a, TR_RES + 0x01);            /* S low = $FF */
    op1(a, 0x18); op1(a, 0xFB);                                   /* clc xce -> native */
    op1(a, 0x08); op1(a, 0x68); stl(a, TR_RES + 0x02);            /* expect $B5 */
    op2(a, 0xC2, 0x30); ldx(a, 0x1FFF); op1(a, 0x9A); op3(a, 0xA9, 0); op1(a, 0x5B);
    stl(a, TR_RES + 0x20);                                        /* zero NMI/IRQ counters */
    /* T3: 16-bit ADC */
    op1(a, 0x18); op3(a, 0xA9, 0x1234); op3(a, 0x69, 0x1111); stl(a, TR_RES + 0x04);
    /* T4: M switch keeps B, XBA */
    op3(a, 0xA9, 0xABCD); op2(a, 0xE2, 0x20); op2(a, 0xA9, 0x12); op1(a, 0xEB);
    stl(a, TR_RES + 0x06); op1(a, 0xEB); stl(a, TR_RES + 0x07); op2(a, 0xC2, 0x20);
    /* T5: direct page offset */
    op3(a, 0xA9, 0x0100); op1(a, 0x5B); op3(a, 0xA9, 0xBEEF); op2(a, 0x85, 0x10);
    op3(a, 0xA9, 0x0000); op1(a, 0x5B); op4(a, 0xAF, 0x7E0110); stl(a, TR_RES + 0x08);
    /* T6: 24-bit long + long,X */
    op3(a, 0xA9, 0x1234); stl(a, 0x7F1234); ldx(a, 2); op3(a, 0xA9, 0x5678); op4(a, 0x9F, 0x7F1232);
    op4(a, 0xAF, 0x7F1234); stl(a, TR_RES + 0x0A);
    /* T7: MVN / MVP */
    ldx(a, 0); fill = here(a);
    op1(a, 0x8A); op4(a, 0x9F, 0x7E2000); op1(a, 0xE8); op1(a, 0xE8); op3(a, 0xE0, 0x0010); bto(a, 0xD0, fill);
    op3(a, 0xA9, 0x000F); ldx(a, 0x2000); op3(a, 0xA0, 0x3000); op1(a, 0x54); e8(a, 0x7E); e8(a, 0x7E);
    op1(a, 0x4B); op1(a, 0xAB);
    op4(a, 0xAF, 0x7E300E); stl(a, TR_RES + 0x0C);
    op3(a, 0xA9, 0x000F); ldx(a, 0x200F); op3(a, 0xA0, 0x400F); op1(a, 0x44); e8(a, 0x7E); e8(a, 0x7E);
    op1(a, 0x4B); op1(a, 0xAB);
    op4(a, 0xAF, 0x7E4002); stl(a, TR_RES + 0x0E);
    op1(a, 0x8A); stl(a, TR_RES + 0x40);                          /* X after MVP = $1FFF */
    op1(a, 0x98); stl(a, TR_RES + 0x42);                          /* Y after MVP = $3FFF */
    /* T8: BCD */
    op1(a, 0xF8); op1(a, 0x18); op3(a, 0xA9, 0x0999); op3(a, 0x69, 0x0001); stl(a, TR_RES + 0x10);
    op1(a, 0x38); op3(a, 0xE9, 0x0001); stl(a, TR_RES + 0x12);
    op2(a, 0xE2, 0x20); op1(a, 0x18); op2(a, 0xA9, 0x99); op2(a, 0x69, 0x01); stl(a, TR_RES + 0x14);
    op2(a, 0xA9, 0x00); op1(a, 0x2A); stl(a, TR_RES + 0x15);
    op1(a, 0x38); op2(a, 0xA9, 0x10); op2(a, 0xE9, 0x01); stl(a, TR_RES + 0x44);  /* $09 */
    op1(a, 0xD8);
    /* T9: hardware multiply / divide */
    wreg(a, 0x4202, 0x12); wreg(a, 0x4203, 0x34);
    for (i = 0; i < 4; i++) op1(a, 0xEA);
    op3(a, 0xAD, 0x4216); stl(a, TR_RES + 0x16); op3(a, 0xAD, 0x4217); stl(a, TR_RES + 0x17);
    wreg(a, 0x4204, 0x34); wreg(a, 0x4205, 0x12); wreg(a, 0x4206, 0x10);
    for (i = 0; i < 8; i++) op1(a, 0xEA);
    op3(a, 0xAD, 0x4214); stl(a, TR_RES + 0x18); op3(a, 0xAD, 0x4215); stl(a, TR_RES + 0x19);
    op3(a, 0xAD, 0x4216); stl(a, TR_RES + 0x1A);
    /* T10: DMA ROM->WRAM ($2180) and ROM->VRAM + $2139/$213A readback */
    wreg(a, 0x2181, 0x00); wreg(a, 0x2182, 0x50); wreg(a, 0x2183, 0x00);
    op2(a, 0xC2, 0x10);
    dma(a, 0, 0x00, 0x80, snes_addr(data_off), 32);
    op4(a, 0xAF, 0x7E501F); stl(a, TR_RES + 0x1C);
    dma_vram(a, 0x0000, snes_addr(data_off), 32);
    wreg(a, 0x2116, 0x05); wreg(a, 0x2117, 0x00);
    op3(a, 0xAD, 0x2139); stl(a, TR_RES + 0x1D);
    op3(a, 0xAD, 0x213A); stl(a, TR_RES + 0x1E);
    /* T12: stack relative, (sr,S),Y, JSR/JSL/COP, PER, (dp),Y, [dp],Y */
    op2(a, 0xC2, 0x30);
    op3(a, 0xF4, 0xABCD); op2(a, 0xA3, 0x01); stl(a, TR_RES + 0x30); op1(a, 0x68);
    op3(a, 0xF4, 0x0110); op3(a, 0xA0, 0x0000); op2(a, 0xB3, 0x01); stl(a, TR_RES + 0x32); op1(a, 0x68);
    op3(a, 0x20, sub1); stl(a, TR_RES + 0x34);
    op4(a, 0x22, sub2); stl(a, TR_RES + 0x36);
    op2(a, 0x02, 0x00); stl(a, TR_RES + 0x38);
    op3(a, 0x62, 0x0000); info->per_addr = here(a); op1(a, 0x68); stl(a, TR_RES + 0x46);
    op3(a, 0xA9, 0x1000); op2(a, 0x85, 0x20);
    op3(a, 0xA9, 0x1000); op2(a, 0x85, 0x22); op3(a, 0xA9, 0x007E); op2(a, 0x85, 0x24);
    op3(a, 0xA0, 0x0004); op2(a, 0xB1, 0x20); stl(a, TR_RES + 0x3A);
    op2(a, 0xB7, 0x22); stl(a, TR_RES + 0x3C);
    /* T11: HDMA -> WRAM, NMI, V-IRQ, WAI, auto-joypad */
    op2(a, 0xE2, 0x20);
    wreg(a, 0x2181, 0x00); wreg(a, 0x2182, 0x60); wreg(a, 0x2183, 0x00);
    wreg(a, 0x4310, 0x00); wreg(a, 0x4311, 0x80);
    ldx(a, (uint16_t)snes_addr(data_off + 0x40)); stx(a, 0x4312);
    wreg(a, 0x4314, 0x01);
    wreg(a, 0x4209, 100); wreg(a, 0x420A, 0x00);
    wreg(a, 0x4200, 0xA1);                                        /* NMI + V-IRQ + autojoy */
    op1(a, 0x58);                                                 /* cli */
    op1(a, 0xCB);                                                 /* wai (IRQ or NMI) */
    wl = here(a);
    op4(a, 0xAF, TR_RES + 0x20); op2(a, 0xC9, 0x01); p2 = bfwd(a, 0xB0);
    op1(a, 0xCB); bto(a, 0x80, wl);
    bpatch(a, p2);
    wreg(a, 0x420C, 0x02);                                        /* HDMA ch1 on */
    wl = here(a);
    op4(a, 0xAF, TR_RES + 0x20); op2(a, 0xC9, 0x02); p3 = bfwd(a, 0xB0);
    op1(a, 0xCB); bto(a, 0x80, wl);
    bpatch(a, p3);
    wreg(a, 0x420C, 0x00);
    wl = here(a);
    op4(a, 0xAF, TR_RES + 0x20); op2(a, 0xC9, 0x04); p4 = bfwd(a, 0xB0);
    op1(a, 0xCB); bto(a, 0x80, wl);
    bpatch(a, p4);
    op2(a, 0xA9, 0xA5); stl(a, TR_RES + 0xFF);
    op1(a, 0xDB);                                                 /* stp */
    header(rom, size, "HS CPU TORTURE", reset, nmi, irq, cop, cop);
    info->size = size;
    return size;
}

/* ============================================================ Mode 1 demo */
size_t tr_build_mode1_rom(uint8_t *rom, size_t cap, tr_info *info)
{
    asmb A = {rom, cap, 0}, *a = &A;
    uint32_t d = 0x2000u;   /* data area inside bank 0 (ROM $00:A000+) */
    uint32_t o_bg1, o_bg2, o_bg3, o_obj, o_m1, o_m2, o_m3, o_pal, o_oam, o_hd0, o_hd1, o_spc;
    uint8_t pix[64], enc[32];
    uint16_t reset, nmi, upload, lp, sl, p;
    int t, x, y, i;
    size_t size = 0x8000u;
    memset(info, 0, sizeof *info);

    /* --- graphics --- */
    o_bg1 = d;
    for (t = 0; t < 16; t++) {
        for (y = 0; y < 8; y++) for (x = 0; x < 8; x++) {
            int ring = (x < y ? x : y), r2 = (7 - x < 7 - y ? 7 - x : 7 - y);
            int m = ring < r2 ? ring : r2;
            pix[y * 8 + x] = (uint8_t)(((m + t) & 7) + ((t & 1) ? 8 : 1));
            if (((x + y + t) & 7) == 0) pix[y * 8 + x] = 0;   /* transparent holes */
        }
        enc_4bpp(pix, enc); memcpy(rom + d, enc, 32); d += 32;
    }
    o_bg2 = d;
    for (t = 0; t < 16; t++) {
        for (y = 0; y < 8; y++) for (x = 0; x < 8; x++)
            pix[y * 8 + x] = (uint8_t)((((x >> 1) ^ (y >> 1) ^ t) & 3) ? 1 + ((x + t) & 7) : 0);
        enc_4bpp(pix, enc); memcpy(rom + d, enc, 32); d += 32;
    }
    o_bg3 = d;
    for (t = 0; t < 8; t++) {
        for (y = 0; y < 8; y++) for (x = 0; x < 8; x++)
            pix[y * 8 + x] = (uint8_t)(((x == y) || (x == 7 - y) || t == 0) ? (1 + (t % 3)) : 0);
        enc_2bpp(pix, enc); memcpy(rom + d, enc, 16); d += 16;
    }
    o_obj = d;   /* 32x32 sprite = tiles 0-3,16-19,32-35,48-51 ; we store 64 tiles */
    for (t = 0; t < 64; t++) {
        int tx = t & 15, ty = t >> 4;
        for (y = 0; y < 8; y++) for (x = 0; x < 8; x++) {
            int px = tx * 8 + x - 16, py = ty * 8 + y - 16;
            int r2 = px * px + py * py;
            uint8_t c = 0;
            if (tx < 4 && r2 < 15 * 15) c = (uint8_t)(1 + ((r2 >> 4) % 15));
            if (tx < 4 && r2 < 15 * 15 && (px + py) > 8 && r2 < 6 * 6 + 60) c = 15;
            pix[y * 8 + x] = c;
        }
        enc_4bpp(pix, enc); memcpy(rom + d, enc, 32); d += 32;
    }
    /* tilemaps 32x32 */
    o_m1 = d;
    for (y = 0; y < 32; y++) for (x = 0; x < 32; x++) {
        uint16_t e = (uint16_t)(((x + y) & 15) | (((x >> 3) & 1) << 10) | ((((x ^ y) >> 2) & 1) << 13) | (((y >> 4) & 1) << 14));
        rom[d++] = (uint8_t)e; rom[d++] = (uint8_t)(e >> 8);
    }
    o_m2 = d;
    for (y = 0; y < 32; y++) for (x = 0; x < 32; x++) {
        uint16_t e = (uint16_t)(((x * 3 + y) & 15) | (2u << 10) | ((((x + y) >> 3) & 1) << 15));
        rom[d++] = (uint8_t)e; rom[d++] = (uint8_t)(e >> 8);
    }
    o_m3 = d;
    for (y = 0; y < 32; y++) for (x = 0; x < 32; x++) {
        uint16_t e = (uint16_t)((((x / 4) + (y / 4)) & 7) | (1u << 10) | ((y > 20) ? 0x2000u : 0u));
        rom[d++] = (uint8_t)e; rom[d++] = (uint8_t)(e >> 8);
    }
    /* palettes */
    o_pal = d;
    for (i = 0; i < 256; i++) {
        uint16_t c;
        if (i < 32) c = bgr(4 + (i & 15), 8 + ((i * 3) & 15), 20 + (i & 7));            /* BG1 pal 0-1 */
        else if (i < 48) c = bgr(31 - (i & 15), 10 + (i & 15), 6);                     /* BG2 pal 2 */
        else if (i < 128) c = bgr((i * 5) & 31, (i * 7) & 31, (i * 11) & 31);
        else c = bgr(31 - ((i & 15) * 2), 12 + (i & 15), ((i >> 4) & 7) * 4);         /* OBJ */
        if (i == 0) c = bgr(2, 4, 12);
        if (i == 4 + 1) c = bgr(31, 31, 8);   /* BG3 pal1 col1 */
        if (i == 4 + 2) c = bgr(8, 31, 31);
        if (i == 4 + 3) c = bgr(31, 8, 31);
        rom[d++] = (uint8_t)c; rom[d++] = (uint8_t)(c >> 8);
    }
    /* OAM: 24 sprites */
    o_oam = d;
    for (i = 0; i < 128; i++) {
        if (i < 24) {
            rom[d++] = (uint8_t)(i * 37);
            rom[d++] = (uint8_t)(16 + (i * 29) % 180);
            rom[d++] = 0x00;
            rom[d++] = (uint8_t)(((i & 7) << 1) | ((i & 3) << 4) | ((i & 8) ? 0x40 : 0));
        } else {
            rom[d++] = 0; rom[d++] = 240; rom[d++] = 0; rom[d++] = 0;
        }
    }
    for (i = 0; i < 32; i++) rom[d++] = (i < 6) ? 0xAA : 0x00;   /* sprites 0..23 large */
    /* HDMA ch0: backdrop gradient via CGADD/CGDATA (mode 3, repeat) */
    o_hd0 = d;
    for (i = 0; i < 2; i++) {
        int l;
        rom[d++] = 0x80 | 112;
        for (l = 0; l < 112; l++) {
            int line = i * 112 + l;
            uint16_t c = bgr(line / 14, 4 + line / 16, 31 - line / 9);
            rom[d++] = 0; rom[d++] = 0; rom[d++] = (uint8_t)c; rom[d++] = (uint8_t)(c >> 8);
        }
    }
    rom[d++] = 0;
    /* HDMA ch1: BG2HOFS wave (mode 2, repeat) */
    o_hd1 = d;
    for (i = 0; i < 2; i++) {
        int l;
        rom[d++] = 0x80 | 112;
        for (l = 0; l < 112; l++) {
            int line = i * 112 + l;
            int v = (isin256(line * 4) * 12) >> 8;
            rom[d++] = (uint8_t)v; rom[d++] = (uint8_t)((v >> 8) & 3);
        }
    }
    rom[d++] = 0;
    o_spc = d;
    info->spc_blob_len = build_spc_blob(rom + d);
    info->spc_blob_off = o_spc;
    info->spc_dest = 0x0200;
    d += info->spc_blob_len;

    /* --- code --- */
    p = (uint16_t)bfwd(a, 0x80);
    upload = emit_spc_upload(a, snes_addr(o_spc), info->spc_blob_len);
    nmi = here(a);
    op1(a, 0x08); op2(a, 0xE2, 0x20); op2(a, 0xC2, 0x10); op1(a, 0x48); op1(a, 0xDA);
    op3(a, 0xAD, 0x4210);
    op2(a, 0xE6, 0x00); { uint32_t q = bfwd(a, 0xD0); op2(a, 0xE6, 0x01); bpatch(a, q); }
    op2(a, 0xA5, 0x00); op3(a, 0x8D, 0x210D); op2(a, 0xA5, 0x01); op2(a, 0x29, 0x03); op3(a, 0x8D, 0x210D);
    op2(a, 0xA5, 0x00); op1(a, 0x4A); op3(a, 0x8D, 0x210E); op3(a, 0x9C, 0x210E);
    op2(a, 0xA9, 0x00); op1(a, 0x38); op2(a, 0xE5, 0x00); op3(a, 0x8D, 0x2110); op3(a, 0x9C, 0x2110);
    op2(a, 0xA5, 0x00); op3(a, 0x8D, 0x2111); op3(a, 0x9C, 0x2111);              /* BG3 HOFS */
    ldx(a, 0); sl = here(a);
    op3(a, 0xFE, 0x0200);                                     /* inc $0200,x (sprite x) */
    op1(a, 0x8A); op2(a, 0x29, 0x04); { uint32_t q = bfwd(a, 0xF0); op3(a, 0xFE, 0x0201); bpatch(a, q); }
    op1(a, 0xE8); op1(a, 0xE8); op1(a, 0xE8); op1(a, 0xE8); op3(a, 0xE0, 24 * 4); bto(a, 0xD0, sl);
    wreg(a, 0x2102, 0); wreg(a, 0x2103, 0);
    dma(a, 7, 0x00, 0x04, 0x000200u, 544);
    op3(a, 0xAD, 0x4218); op2(a, 0x85, 0x02);
    op3(a, 0xAD, 0x2141); stl(a, TR_RES + 0x102);
    op1(a, 0xFA); op1(a, 0x68); op1(a, 0x28); op1(a, 0x40);
    bpatch(a, p);
    reset = here(a);
    emit_init(a);
    op2(a, 0xC2, 0x10);
    op3(a, 0x20, upload);
    /* VRAM / CGRAM / OAM */
    dma_vram(a, 0x0000, snes_addr(o_bg1), 16 * 32);
    dma_vram(a, 0x1000, snes_addr(o_bg2), 16 * 32);
    dma_vram(a, 0x2000, snes_addr(o_bg3), 8 * 16);
    dma_vram(a, 0x4000, snes_addr(o_obj), 64 * 32);
    dma_vram(a, 0x5000, snes_addr(o_m1), 2048);
    dma_vram(a, 0x5400, snes_addr(o_m2), 2048);
    dma_vram(a, 0x5800, snes_addr(o_m3), 2048);
    wreg(a, 0x2121, 0); dma(a, 0, 0x00, 0x22, snes_addr(o_pal), 512);
    /* OAM shadow -> WRAM $0200 via MVN, then DMA to OAM */
    op2(a, 0xC2, 0x30);
    op3(a, 0xA9, 543); ldx(a, (uint16_t)snes_addr(o_oam)); op3(a, 0xA0, 0x0200);
    op1(a, 0x54); e8(a, 0x00); e8(a, 0x00);
    op2(a, 0xE2, 0x20);
    wreg(a, 0x2102, 0); wreg(a, 0x2103, 0);
    dma(a, 7, 0x00, 0x04, 0x000200u, 544);
    /* PPU setup */
    wreg(a, 0x2105, 0x09);           /* mode 1, BG3 high priority */
    wreg(a, 0x2107, 0x50); wreg(a, 0x2108, 0x54); wreg(a, 0x2109, 0x58);
    wreg(a, 0x210B, 0x10); wreg(a, 0x210C, 0x02);
    wreg(a, 0x2101, 0x62);           /* OBJ base $4000, 16/32 */
    wreg(a, 0x212C, 0x13);           /* main: BG1 BG2 OBJ */
    wreg(a, 0x212D, 0x04);           /* sub: BG3 */
    wreg(a, 0x2130, 0x02);           /* add subscreen */
    wreg(a, 0x2131, 0x42);           /* half-add on BG2 */
    wreg(a, 0x2126, 40); wreg(a, 0x2127, 200);
    wreg(a, 0x2123, 0x30);           /* BG2: window 1 enabled + inverted */
    wreg(a, 0x212E, 0x02);           /* mask BG2 on main outside 40..200 */
    /* HDMA */
    wreg(a, 0x4300, 0x03); wreg(a, 0x4301, 0x21);
    ldx(a, (uint16_t)snes_addr(o_hd0)); stx(a, 0x4302); wreg(a, 0x4304, 0x00);
    wreg(a, 0x4310, 0x02); wreg(a, 0x4311, 0x0F);
    ldx(a, (uint16_t)snes_addr(o_hd1)); stx(a, 0x4312); wreg(a, 0x4314, 0x00);
    wreg(a, 0x420C, 0x03);
    wreg(a, 0x2100, 0x0F);
    wreg(a, 0x4200, 0x81);
    lp = here(a);
    op1(a, 0xCB); bto(a, 0x80, lp);
    if (a->pc > 0x2000u) return 0;   /* code overflowed into data */
    header(rom, size, "HS MODE1 DEMO", reset, nmi, nmi, nmi, nmi);
    info->size = size;
    (void)cap;
    return d <= 0x7FC0u ? size : 0;
}

/* ============================================================ Mode 7 demo */
#define M7_ANGLES 32
#define M7_TAB    (1 + 224 * 2 + 1 + 1)   /* two repeat blocks of 112 */

size_t tr_build_mode7_rom(uint8_t *rom, size_t cap, tr_info *info)
{
    asmb A = {rom, cap, 0}, *a = &A;
    uint32_t o_map = 0x8000u, o_chr = 0xC000u, o_tab = 0x10000u, ptr_tab;
    uint16_t reset, nmi, lp, p;
    uint32_t off[M7_ANGLES][4];
    int g, k, x, y, t, l;
    size_t size = 0x20000u;
    memset(info, 0, sizeof *info);
    if (cap < size) return 0;
    /* 128x128 map, 256 tiles 8bpp */
    for (y = 0; y < 128; y++) for (x = 0; x < 128; x++)
        rom[o_map + (uint32_t)(y * 128 + x)] = (uint8_t)((((x >> 2) ^ (y >> 2)) & 1) ? ((x * 5 + y) & 0x3F) : (0x40 | ((x + y * 3) & 0x3F)));
    for (t = 0; t < 256; t++) for (y = 0; y < 8; y++) for (x = 0; x < 8; x++) {
        uint8_t c = (uint8_t)(1 + ((t * 3 + ((x ^ y) >> 1)) & 0x7F));
        if (x == 0 || y == 0) c = (uint8_t)(0x80 | (t & 0x7F));
        rom[o_chr + (uint32_t)(t * 64 + y * 8 + x)] = c;
    }
    /* HDMA matrix tables: A=cos*s B=sin*s C=-sin*s D=cos*s, s = perspective */
    {
        uint32_t cur = o_tab;
        for (g = 0; g < M7_ANGLES; g++) {
            int cs = icos256(g * 8), sn = isin256(g * 8);
            for (k = 0; k < 4; k++) {
                int blk;
                if ((cur & 0x7FFFu) + M7_TAB > 0x8000u) cur = (cur + 0x8000u) & ~0x7FFFu;
                off[g][k] = cur;
                for (blk = 0; blk < 2; blk++) {
                    rom[cur++] = 0x80 | 112;
                    for (l = 0; l < 112; l++) {
                        int line = blk * 112 + l + 1;
                        int s = (256 * 40) / (line + 24);     /* 8.8 scale */
                        int v;
                        switch (k) {
                        case 0: v = (cs * s) >> 8; break;
                        case 1: v = (sn * s) >> 8; break;
                        case 2: v = -((sn * s) >> 8); break;
                        default: v = (cs * s) >> 8; break;
                        }
                        rom[cur++] = (uint8_t)v; rom[cur++] = (uint8_t)(v >> 8);
                    }
                }
                rom[cur++] = 0;
            }
        }
    }
    /* code in bank 0, pointer table after code */
    p = (uint16_t)bfwd(a, 0x80);
    nmi = here(a);
    op1(a, 0x08); op2(a, 0xE2, 0x20); op2(a, 0xC2, 0x10); op1(a, 0x48); op1(a, 0xDA);
    op3(a, 0xAD, 0x4210);
    op2(a, 0xE6, 0x00); { uint32_t q = bfwd(a, 0xD0); op2(a, 0xE6, 0x01); bpatch(a, q); }
    /* X = ((frame >> 2) & 31) * 16 */
    op2(a, 0xC2, 0x20); op2(a, 0xA5, 0x00); op1(a, 0x4A); op1(a, 0x4A); op3(a, 0x29, 0x001F);
    op1(a, 0x0A); op1(a, 0x0A); op1(a, 0x0A); op1(a, 0x0A); op1(a, 0xAA);
    /* VOFS = -(frame*2), M7Y = VOFS + 112 (16-bit math), stored at $04/$06 */
    op2(a, 0xA5, 0x00); op1(a, 0x0A); op3(a, 0x49, 0xFFFF); op1(a, 0x1A); op3(a, 0x29, 0x1FFF); op2(a, 0x85, 0x04);
    op1(a, 0x18); op3(a, 0x69, 112); op3(a, 0x29, 0x1FFF); op2(a, 0x85, 0x06);
    op2(a, 0xE2, 0x20);
    op2(a, 0xA5, 0x04); op3(a, 0x8D, 0x210E); op2(a, 0xA5, 0x05); op3(a, 0x8D, 0x210E);
    op2(a, 0xA5, 0x06); op3(a, 0x8D, 0x2120); op2(a, 0xA5, 0x07); op3(a, 0x8D, 0x2120);
    {
        uint32_t fix[4];
        for (k = 0; k < 4; k++) {
            uint16_t b = (uint16_t)(0x4300u + k * 16);
            fix[k] = a->pc + 1u; op3(a, 0xBD, 0x0000); op3(a, 0x8D, (uint16_t)(b + 2u));
            op3(a, 0xBD, 0x0000); op3(a, 0x8D, (uint16_t)(b + 3u));
            op3(a, 0xBD, 0x0000); op3(a, 0x8D, (uint16_t)(b + 4u));
        }
        op1(a, 0xFA); op1(a, 0x68); op1(a, 0x28); op1(a, 0x40);
        bpatch(a, p);
        reset = here(a);
        emit_init(a);
        op2(a, 0xC2, 0x10);
        /* map -> VRAM low bytes, chr -> VRAM high bytes */
        wreg(a, 0x2115, 0x00); ldx(a, 0); stx(a, 0x2116);
        dma(a, 0, 0x00, 0x18, snes_addr(o_map), 0x4000);
        wreg(a, 0x2115, 0x80); ldx(a, 0); stx(a, 0x2116);
        dma(a, 0, 0x00, 0x19, snes_addr(o_chr), 0x4000);
        /* palette: generated by CPU loop: color = i*0x0421 + (i>>3)... use DMA of computed table */
        wreg(a, 0x2121, 0);
        dma(a, 0, 0x00, 0x22, snes_addr(0x7000u), 512);
        wreg(a, 0x2105, 0x07);
        wreg(a, 0x211A, 0x00);
        wreg(a, 0x212C, 0x01);
        wreg(a, 0x211F, 0x80); wreg(a, 0x211F, 0x00);   /* M7X = 128 */
        wreg(a, 0x2120, 0x70); wreg(a, 0x2120, 0x00);   /* M7Y = 112 */
        wreg(a, 0x210D, 0x00); wreg(a, 0x210D, 0x00);
        for (k = 0; k < 4; k++) {
            uint16_t b = (uint16_t)(0x4300u + k * 16);
            wreg(a, b, 0x02);
            wreg(a, (uint16_t)(b + 1u), (uint8_t)(0x1B + k));
            ldx(a, (uint16_t)snes_addr(off[0][k])); stx(a, (uint16_t)(b + 2u));
            wreg(a, (uint16_t)(b + 4u), (uint8_t)(snes_addr(off[0][k]) >> 16));
        }
        wreg(a, 0x420C, 0x0F);
        wreg(a, 0x2100, 0x0F);
        wreg(a, 0x4200, 0x80);
        lp = here(a); op1(a, 0xCB); bto(a, 0x80, lp);
        /* pointer table */
        while (a->pc & 15u) e8(a, 0);
        ptr_tab = here(a);
        for (g = 0; g < M7_ANGLES; g++) {
            for (k = 0; k < 4; k++) e24(a, snes_addr(off[g][k]));
            e16(a, 0); e16(a, 0);
        }
        for (k = 0; k < 4; k++) {
            uint16_t base = (uint16_t)(ptr_tab + k * 3);
            rom[fix[k]] = (uint8_t)base; rom[fix[k] + 1u] = (uint8_t)(base >> 8);
            rom[fix[k] + 6u] = (uint8_t)(base + 1u); rom[fix[k] + 7u] = (uint8_t)((base + 1u) >> 8);
            rom[fix[k] + 12u] = (uint8_t)(base + 2u); rom[fix[k] + 13u] = (uint8_t)((base + 2u) >> 8);
        }
    }
    /* palette table at bank 0 offset $7000 */
    for (k = 0; k < 256; k++) {
        uint16_t c = bgr(6 + (k & 15), 4 + ((k >> 2) & 15) + ((k & 0x40) ? 10 : 0), 8 + ((k >> 3) & 15));
        if (k & 0x80) c = bgr(31, 28, 10 + (k & 7));
        if (k == 0) c = bgr(10, 14, 28);
        rom[0x7000u + (uint32_t)k * 2u] = (uint8_t)c;
        rom[0x7000u + (uint32_t)k * 2u + 1u] = (uint8_t)(c >> 8);
    }
    if (a->pc > 0x7000u) return 0;
    header(rom, size, "HS MODE7 FLOOR", reset, nmi, nmi, nmi, nmi);
    info->size = size;
    return size;
}
