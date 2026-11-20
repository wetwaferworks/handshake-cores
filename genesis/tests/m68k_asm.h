/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * m68k_asm.h - tiny 68000 assembler used only by the hs-genesis tests to
 * build CPU torture programs and the synthetic Genesis ROM directly in C.
 * Encodings follow the public 68000 programmer's reference manual.
 */
#ifndef HS_M68K_ASM_H
#define HS_M68K_ASM_H

#include <stdint.h>
#include <string.h>

#define ASM_MAX_LABELS 128
#define ASM_MAX_FIX    512

typedef struct asm68 {
    uint8_t *buf;
    uint32_t cap;
    uint32_t pc;
    uint32_t label[ASM_MAX_LABELS];
    uint32_t fix_at[ASM_MAX_FIX];
    uint32_t fix_base[ASM_MAX_FIX];
    uint8_t  fix_lbl[ASM_MAX_FIX];
    uint8_t  fix_kind[ASM_MAX_FIX];   /* 0 = disp16, 1 = abs32 */
    uint32_t nfix;
    int      err;
} asm68_t;

typedef struct ea68 { uint8_t mode, reg, n; uint32_t ext[2]; } ea68_t;

static inline ea68_t EA_D(int n)   { ea68_t e = { 0, (uint8_t)n, 0, { 0, 0 } }; return e; }
static inline ea68_t EA_A(int n)   { ea68_t e = { 1, (uint8_t)n, 0, { 0, 0 } }; return e; }
static inline ea68_t EA_IND(int n) { ea68_t e = { 2, (uint8_t)n, 0, { 0, 0 } }; return e; }
static inline ea68_t EA_PI(int n)  { ea68_t e = { 3, (uint8_t)n, 0, { 0, 0 } }; return e; }
static inline ea68_t EA_PD(int n)  { ea68_t e = { 4, (uint8_t)n, 0, { 0, 0 } }; return e; }
static inline ea68_t EA_DISP(int n, int d) { ea68_t e = { 5, (uint8_t)n, 1, { (uint16_t)d, 0 } }; return e; }
static inline ea68_t EA_IDX(int n, int dn, int d) /* d8(An,Dn.w) */
{ ea68_t e = { 6, (uint8_t)n, 1, { (uint32_t)((dn & 7) << 12) | (uint8_t)d, 0 } }; return e; }
static inline ea68_t EA_AW(uint32_t a) { ea68_t e = { 7, 0, 1, { a & 0xFFFF, 0 } }; return e; }
static inline ea68_t EA_AL(uint32_t a) { ea68_t e = { 7, 1, 2, { a >> 16, a & 0xFFFF } }; return e; }
static inline ea68_t EA_IMM(int sz, uint32_t v)
{
    ea68_t e = { 7, 4, 1, { 0, 0 } };
    if (sz == 4) { e.n = 2; e.ext[0] = v >> 16; e.ext[1] = v & 0xFFFF; }
    else e.ext[0] = sz == 1 ? (v & 0xFF) : (v & 0xFFFF);
    return e;
}

static inline void a_w16(asm68_t *a, uint32_t v)
{
    if (a->pc + 2 > a->cap) { a->err = 1; return; }
    a->buf[a->pc] = (uint8_t)(v >> 8);
    a->buf[a->pc + 1] = (uint8_t)v;
    a->pc += 2;
}
static inline void a_w32(asm68_t *a, uint32_t v) { a_w16(a, v >> 16); a_w16(a, v & 0xFFFF); }
static inline void a_ext(asm68_t *a, ea68_t e) { uint32_t i; for (i = 0; i < e.n; i++) a_w16(a, e.ext[i]); }
static inline uint32_t a_eaf(ea68_t e) { return (uint32_t)(e.mode << 3) | e.reg; }
static inline uint32_t a_sz(int sz) { return sz == 1 ? 0u : (sz == 2 ? 1u : 2u); }

static inline void a_init(asm68_t *a, uint8_t *buf, uint32_t cap, uint32_t org)
{
    memset(a, 0, sizeof(*a));
    a->buf = buf; a->cap = cap; a->pc = org;
}
static inline void a_org(asm68_t *a, uint32_t org) { a->pc = org; }
static inline void a_label(asm68_t *a, int id) { a->label[id] = a->pc; }
static inline void a_fix(asm68_t *a, int id, uint32_t base, int kind)
{
    if (a->nfix >= ASM_MAX_FIX) { a->err = 2; return; }
    a->fix_at[a->nfix] = a->pc; a->fix_base[a->nfix] = base;
    a->fix_lbl[a->nfix] = (uint8_t)id; a->fix_kind[a->nfix] = (uint8_t)kind; a->nfix++;
}
static inline void a_finish(asm68_t *a)
{
    uint32_t i;
    for (i = 0; i < a->nfix; i++) {
        uint32_t at = a->fix_at[i], t = a->label[a->fix_lbl[i]];
        if (a->fix_kind[i] == 0) {
            uint32_t d = t - a->fix_base[i];
            a->buf[at] = (uint8_t)(d >> 8); a->buf[at + 1] = (uint8_t)d;
        } else {
            a->buf[at] = (uint8_t)(t >> 24); a->buf[at + 1] = (uint8_t)(t >> 16);
            a->buf[at + 2] = (uint8_t)(t >> 8); a->buf[at + 3] = (uint8_t)t;
        }
    }
}

/* data movement */
static inline void a_move(asm68_t *a, int sz, ea68_t s, ea68_t d)
{
    uint32_t szc = sz == 1 ? 1u : (sz == 2 ? 3u : 2u);
    a_w16(a, (szc << 12) | ((uint32_t)d.reg << 9) | ((uint32_t)d.mode << 6) | a_eaf(s));
    a_ext(a, s); a_ext(a, d);
}
static inline void a_movea(asm68_t *a, int sz, ea68_t s, int an) { a_move(a, sz, s, EA_A(an)); }
static inline void a_moveq(asm68_t *a, int v, int dn) { a_w16(a, 0x7000u | ((uint32_t)dn << 9) | (uint8_t)v); }
static inline void a_lea(asm68_t *a, ea68_t e, int an) { a_w16(a, 0x41C0u | ((uint32_t)an << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_pea(asm68_t *a, ea68_t e) { a_w16(a, 0x4840u | a_eaf(e)); a_ext(a, e); }
static inline void a_movem(asm68_t *a, int sz, int to_mem, uint16_t mask, ea68_t e)
{
    a_w16(a, 0x4880u | (to_mem ? 0u : 0x400u) | (sz == 4 ? 0x40u : 0u) | a_eaf(e));
    a_w16(a, mask); a_ext(a, e);
}
static inline void a_movep(asm68_t *a, int opmode, int dn, int an, int d)
{ a_w16(a, 0x0108u | ((uint32_t)dn << 9) | ((uint32_t)opmode << 6) | (uint32_t)an); a_w16(a, (uint16_t)d); }

/* <ea>,Dn and Dn,<ea> ALU forms: base = 0xD000 ADD, 0x9000 SUB, 0xB000 CMP, 0xC000 AND, 0x8000 OR */
static inline void a_op_ead(asm68_t *a, uint32_t base, int sz, ea68_t e, int dn)
{ a_w16(a, base | ((uint32_t)dn << 9) | (a_sz(sz) << 6) | a_eaf(e)); a_ext(a, e); }
static inline void a_op_dea(asm68_t *a, uint32_t base, int sz, int dn, ea68_t e)
{ a_w16(a, base | ((uint32_t)dn << 9) | 0x100u | (a_sz(sz) << 6) | a_eaf(e)); a_ext(a, e); }
#define A_ADD 0xD000u
#define A_SUB 0x9000u
#define A_CMP 0xB000u
#define A_AND 0xC000u
#define A_OR  0x8000u
#define A_EOR 0xB000u
/* immediate: 0x0000 ORI, 0x0200 ANDI, 0x0400 SUBI, 0x0600 ADDI, 0x0A00 EORI, 0x0C00 CMPI */
static inline void a_imm(asm68_t *a, uint32_t base, int sz, uint32_t v, ea68_t e)
{
    a_w16(a, base | (a_sz(sz) << 6) | a_eaf(e));
    if (sz == 4) a_w32(a, v); else a_w16(a, sz == 1 ? (v & 0xFF) : (v & 0xFFFF));
    a_ext(a, e);
}
#define A_ORI  0x0000u
#define A_ANDI 0x0200u
#define A_SUBI 0x0400u
#define A_ADDI 0x0600u
#define A_EORI 0x0A00u
#define A_CMPI 0x0C00u
static inline void a_addq(asm68_t *a, int sz, int q, ea68_t e)
{ a_w16(a, 0x5000u | ((uint32_t)(q & 7) << 9) | (a_sz(sz) << 6) | a_eaf(e)); a_ext(a, e); }
static inline void a_subq(asm68_t *a, int sz, int q, ea68_t e)
{ a_w16(a, 0x5100u | ((uint32_t)(q & 7) << 9) | (a_sz(sz) << 6) | a_eaf(e)); a_ext(a, e); }
static inline void a_adda(asm68_t *a, int sz, ea68_t e, int an)
{ a_w16(a, 0xD0C0u | (sz == 4 ? 0x100u : 0u) | ((uint32_t)an << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_suba(asm68_t *a, int sz, ea68_t e, int an)
{ a_w16(a, 0x90C0u | (sz == 4 ? 0x100u : 0u) | ((uint32_t)an << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_cmpa(asm68_t *a, int sz, ea68_t e, int an)
{ a_w16(a, 0xB0C0u | (sz == 4 ? 0x100u : 0u) | ((uint32_t)an << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_addx(asm68_t *a, int sz, int dy, int dx) { a_w16(a, 0xD100u | ((uint32_t)dx << 9) | (a_sz(sz) << 6) | (uint32_t)dy); }
static inline void a_subx(asm68_t *a, int sz, int dy, int dx) { a_w16(a, 0x9100u | ((uint32_t)dx << 9) | (a_sz(sz) << 6) | (uint32_t)dy); }
static inline void a_abcd(asm68_t *a, int dy, int dx) { a_w16(a, 0xC100u | ((uint32_t)dx << 9) | (uint32_t)dy); }
static inline void a_sbcd(asm68_t *a, int dy, int dx) { a_w16(a, 0x8100u | ((uint32_t)dx << 9) | (uint32_t)dy); }
static inline void a_nbcd(asm68_t *a, ea68_t e) { a_w16(a, 0x4800u | a_eaf(e)); a_ext(a, e); }
static inline void a_cmpm(asm68_t *a, int sz, int ay, int ax) { a_w16(a, 0xB108u | ((uint32_t)ax << 9) | (a_sz(sz) << 6) | (uint32_t)ay); }
static inline void a_mulu(asm68_t *a, ea68_t e, int dn) { a_w16(a, 0xC0C0u | ((uint32_t)dn << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_muls(asm68_t *a, ea68_t e, int dn) { a_w16(a, 0xC1C0u | ((uint32_t)dn << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_divu(asm68_t *a, ea68_t e, int dn) { a_w16(a, 0x80C0u | ((uint32_t)dn << 9) | a_eaf(e)); a_ext(a, e); }
static inline void a_divs(asm68_t *a, ea68_t e, int dn) { a_w16(a, 0x81C0u | ((uint32_t)dn << 9) | a_eaf(e)); a_ext(a, e); }
/* single-operand: 0x4000 NEGX, 0x4200 CLR, 0x4400 NEG, 0x4600 NOT, 0x4A00 TST */
static inline void a_unary(asm68_t *a, uint32_t base, int sz, ea68_t e) { a_w16(a, base | (a_sz(sz) << 6) | a_eaf(e)); a_ext(a, e); }
#define A_NEGX 0x4000u
#define A_CLR  0x4200u
#define A_NEG  0x4400u
#define A_NOT  0x4600u
#define A_TST  0x4A00u
static inline void a_tas(asm68_t *a, ea68_t e) { a_w16(a, 0x4AC0u | a_eaf(e)); a_ext(a, e); }
static inline void a_swap(asm68_t *a, int dn) { a_w16(a, 0x4840u | (uint32_t)dn); }
static inline void a_extw(asm68_t *a, int dn) { a_w16(a, 0x4880u | (uint32_t)dn); }
static inline void a_extl(asm68_t *a, int dn) { a_w16(a, 0x48C0u | (uint32_t)dn); }
static inline void a_exg_dd(asm68_t *a, int x, int y) { a_w16(a, 0xC140u | ((uint32_t)x << 9) | (uint32_t)y); }
static inline void a_exg_da(asm68_t *a, int dx, int ay) { a_w16(a, 0xC188u | ((uint32_t)dx << 9) | (uint32_t)ay); }
static inline void a_chk(asm68_t *a, ea68_t e, int dn) { a_w16(a, 0x4180u | ((uint32_t)dn << 9) | a_eaf(e)); a_ext(a, e); }
/* shifts: type 0 AS, 1 LS, 2 ROX, 3 RO */
static inline void a_shi(asm68_t *a, int type, int left, int sz, int cnt, int dn)
{ a_w16(a, 0xE000u | ((uint32_t)(cnt & 7) << 9) | ((uint32_t)left << 8) | (a_sz(sz) << 6) | ((uint32_t)type << 3) | (uint32_t)dn); }
static inline void a_shr(asm68_t *a, int type, int left, int sz, int dx, int dn)
{ a_w16(a, 0xE020u | ((uint32_t)dx << 9) | ((uint32_t)left << 8) | (a_sz(sz) << 6) | ((uint32_t)type << 3) | (uint32_t)dn); }
static inline void a_shm(asm68_t *a, int type, int left, ea68_t e)
{ a_w16(a, 0xE0C0u | ((uint32_t)type << 9) | ((uint32_t)left << 8) | a_eaf(e)); a_ext(a, e); }
/* bit ops: type 0 BTST, 1 BCHG, 2 BCLR, 3 BSET */
static inline void a_bit_i(asm68_t *a, int type, int bit, ea68_t e)
{ a_w16(a, 0x0800u | ((uint32_t)type << 6) | a_eaf(e)); a_w16(a, (uint32_t)bit); a_ext(a, e); }
static inline void a_bit_r(asm68_t *a, int type, int dn, ea68_t e)
{ a_w16(a, 0x0100u | ((uint32_t)dn << 9) | ((uint32_t)type << 6) | a_eaf(e)); a_ext(a, e); }
static inline void a_scc(asm68_t *a, int cc, ea68_t e) { a_w16(a, 0x50C0u | ((uint32_t)cc << 8) | a_eaf(e)); a_ext(a, e); }
/* control flow */
static inline void a_bcc(asm68_t *a, int cc, int lbl)
{ uint32_t base = a->pc + 2; a_w16(a, 0x6000u | ((uint32_t)cc << 8)); a_fix(a, lbl, base, 0); a_w16(a, 0); }
static inline void a_bra(asm68_t *a, int lbl) { a_bcc(a, 0, lbl); }
static inline void a_bsr(asm68_t *a, int lbl) { a_bcc(a, 1, lbl); }
static inline void a_bra_s(asm68_t *a, int8_t d) { a_w16(a, 0x6000u | (uint8_t)d); }
static inline void a_dbcc(asm68_t *a, int cc, int dn, int lbl)
{ uint32_t base = a->pc + 2; a_w16(a, 0x50C8u | ((uint32_t)cc << 8) | (uint32_t)dn); a_fix(a, lbl, base, 0); a_w16(a, 0); }
static inline void a_dbra(asm68_t *a, int dn, int lbl) { a_dbcc(a, 1, dn, lbl); }
static inline void a_jsr(asm68_t *a, ea68_t e) { a_w16(a, 0x4E80u | a_eaf(e)); a_ext(a, e); }
static inline void a_jmp(asm68_t *a, ea68_t e) { a_w16(a, 0x4EC0u | a_eaf(e)); a_ext(a, e); }
static inline void a_lea_lbl(asm68_t *a, int lbl, int an) { a_w16(a, 0x41F9u | ((uint32_t)an << 9)); a_fix(a, lbl, 0, 1); a_w32(a, 0); }
static inline void a_rts(asm68_t *a) { a_w16(a, 0x4E75); }
static inline void a_rte(asm68_t *a) { a_w16(a, 0x4E73); }
static inline void a_rtr(asm68_t *a) { a_w16(a, 0x4E77); }
static inline void a_nop(asm68_t *a) { a_w16(a, 0x4E71); }
static inline void a_trap(asm68_t *a, int n) { a_w16(a, 0x4E40u | (uint32_t)(n & 15)); }
static inline void a_trapv(asm68_t *a) { a_w16(a, 0x4E76); }
static inline void a_stop(asm68_t *a, uint16_t sr) { a_w16(a, 0x4E72); a_w16(a, sr); }
static inline void a_link(asm68_t *a, int an, int d) { a_w16(a, 0x4E50u | (uint32_t)an); a_w16(a, (uint16_t)d); }
static inline void a_unlk(asm68_t *a, int an) { a_w16(a, 0x4E58u | (uint32_t)an); }
static inline void a_move_to_sr(asm68_t *a, ea68_t e) { a_w16(a, 0x46C0u | a_eaf(e)); a_ext(a, e); }
static inline void a_move_from_sr(asm68_t *a, ea68_t e) { a_w16(a, 0x40C0u | a_eaf(e)); a_ext(a, e); }
static inline void a_move_to_ccr(asm68_t *a, ea68_t e) { a_w16(a, 0x44C0u | a_eaf(e)); a_ext(a, e); }
static inline void a_move_usp(asm68_t *a, int to_usp, int an) { a_w16(a, 0x4E60u | (to_usp ? 0u : 8u) | (uint32_t)an); }
static inline void a_ori_ccr(asm68_t *a, int v) { a_w16(a, 0x003C); a_w16(a, (uint32_t)v & 0xFF); }
static inline void a_andi_sr(asm68_t *a, int v) { a_w16(a, 0x027C); a_w16(a, (uint32_t)v & 0xFFFF); }

enum { CC_T = 0, CC_F, CC_HI, CC_LS, CC_CC, CC_CS, CC_NE, CC_EQ, CC_VC, CC_VS, CC_PL, CC_MI, CC_GE, CC_LT, CC_GT, CC_LE };

#endif
