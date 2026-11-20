/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * psx_bus.c - Phase 0 memory map + peripherals: 2 MiB RAM (8 MiB mirror),
 * 1 KiB scratchpad, BIOS window, I_STAT/I_MASK, DMA0..6 (DMA2 GPU block +
 * linked list, DMA4 SPU, DMA6 OTC), root counters 0..2, SIO0 digital /
 * DualShock-digital pad, CD-ROM controller status stub (no disc), MDEC stub,
 * GPU/SPU port dispatch. DMA transfers complete instantly (deterministic).
 */
#include "psx_internal.h"

#include <string.h>

/* ---------------- IRQ ---------------- */
void psx_irq(hs_psx_t *s, uint32_t bit) { s->istat |= 1u << bit; }
int  psx_irq_pending(const hs_psx_t *s) { return (s->istat & s->imask & 0x7FFu) != 0; }

/* ---------------- little-endian helpers ---------------- */
static inline uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint32_t rd16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static inline void wr32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static inline void wr16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static inline uint32_t ram32(const hs_psx_t *s, uint32_t a) { return rd32(s->ram + (a & 0x1FFFFCu)); }
static inline void ram32w(hs_psx_t *s, uint32_t a, uint32_t v) { wr32(s->ram + (a & 0x1FFFFCu), v); }

/* ---------------- timers ---------------- */
static void tmr_event(hs_psx_t *s, uint32_t n, uint32_t irq_en_bit, uint32_t flag_bit)
{
    psx_timer_t *t = &s->tmr[n];
    t->mode |= flag_bit;
    if (t->mode & irq_en_bit) {
        if ((t->mode & 0x40u) || (t->mode & 0x400u)) {   /* repeat, or first one-shot */
            psx_irq(s, 4u + n);
            if (t->mode & 0x80u) t->mode ^= 0x400u;       /* toggle mode */
            else if (!(t->mode & 0x40u)) t->mode &= ~0x400u;
        }
    }
}

static void tmr_advance(hs_psx_t *s, uint32_t n, uint32_t ticks)
{
    psx_timer_t *t = &s->tmr[n];
    uint32_t tgt = t->target & 0xFFFFu, k;
    /* fast path: no target / overflow crossing inside this batch */
    if ((t->count < tgt && t->count + ticks < tgt) || (t->count > tgt && t->count + ticks <= 0xFFFFu)) {
        t->count += ticks;
        return;
    }
    for (k = 0; k < ticks; k++) {          /* bounded by the flush batch */
        uint32_t old = t->count, c = (old + 1u) & 0xFFFFu;
        if (old == 0xFFFFu) tmr_event(s, n, 0x20u, 1u << 12);
        if ((t->mode & 8u) && old == tgt) c = 0;
        t->count = c;
        if (c == tgt) tmr_event(s, n, 0x10u, 1u << 11);
    }
}

static void tmr_flush(hs_psx_t *s)
{
    uint32_t cyc = s->tmr_accum, n;
    s->tmr_accum = 0;
    if (!cyc) return;
    for (n = 0; n < 3; n++) {
        psx_timer_t *t = &s->tmr[n];
        uint32_t src = (t->mode >> 8) & 3u, ticks;
        uint64_t f;
        if (n == 0 && (src & 1u)) {            /* dot clock: 53.693182 MHz / 8 (320 px) */
            f = (uint64_t)t->frac + (uint64_t)cyc * 12985u;   /* 1.58532/8 * 65536 */
            ticks = (uint32_t)(f >> 16); t->frac = (uint32_t)(f & 0xFFFFu);
        } else if (n == 1 && (src & 1u)) {     /* hblank: ~2146 CPU cycles per line */
            f = (uint64_t)t->frac + cyc;
            ticks = (uint32_t)(f / 2146u); t->frac = (uint32_t)(f % 2146u);
        } else if (n == 2 && (src & 2u)) {     /* sysclk / 8 */
            f = (uint64_t)t->frac + cyc;
            ticks = (uint32_t)(f >> 3); t->frac = (uint32_t)(f & 7u);
        } else {
            ticks = cyc;
        }
        if (ticks) tmr_advance(s, n, ticks);
    }
}

void psx_bus_tick(hs_psx_t *s, uint32_t cycles)
{
    s->tmr_accum += cycles;
    s->frame_cyc += cycles;
    if (s->tmr_accum >= 32u)
        tmr_flush(s);
    if (s->sio.ack_pending) {
        s->sio.ack_pending = 0;
        if (s->sio.ctrl & 0x1000u) { s->sio.stat |= 0x200u; psx_irq(s, 7); }
    }
}

/* ---------------- DMA ---------------- */
static void dma_irq_update(hs_psx_t *s, uint32_t old)
{
    uint32_t d = s->dicr & 0x7FFF803Fu;
    uint32_t en = (d >> 16) & 0x7Fu, fl = (d >> 24) & 0x7Fu;
    if ((d & 0x8000u) || ((d & 0x800000u) && (en & fl)))
        d |= 0x80000000u;
    s->dicr = d;
    if ((d & 0x80000000u) && !(old & 0x80000000u))
        psx_irq(s, 3);
}

static void dma_done(hs_psx_t *s, uint32_t ch)
{
    uint32_t old = s->dicr;
    s->dma[ch].chcr &= ~((1u << 24) | (1u << 28));
    if (s->dicr & (1u << (16u + ch)))
        s->dicr |= 1u << (24u + ch);
    dma_irq_update(s, old);
}

static void dma_run(hs_psx_t *s, uint32_t ch)
{
    psx_dma_ch_t *d = &s->dma[ch];
    uint32_t chcr = d->chcr, sync = (chcr >> 9) & 3u;
    uint32_t from_ram = chcr & 1u, step = (chcr & 2u) ? (uint32_t)-4 : 4u;
    uint32_t addr = d->madr & 0x1FFFFCu, n, i;

    if (sync == 0 && !(chcr & (1u << 28)) && ch != 6u)
        return;
    if (ch == 2u && sync == 2u) {        /* GPU linked list */
        uint32_t guard, budget = 0x100000u;          /* words: bounds runaway lists */
        for (guard = 0; guard < 0x40000u && budget; guard++) {
            uint32_t hdr = ram32(s, addr), cnt = hdr >> 24, k;
            for (k = 1; k <= cnt && budget; k++, budget--)
                psx_gp0(s, ram32(s, addr + 4u * k));
            if (hdr & 0x800000u) break;
            addr = hdr & 0x1FFFFCu;
        }
        d->madr = 0x00FFFFFFu;
        dma_done(s, ch);
        return;
    }
    if (sync == 0) n = (d->bcr & 0xFFFFu) ? (d->bcr & 0xFFFFu) : 0x10000u;
    else           n = (d->bcr & 0xFFFFu) * (d->bcr >> 16);
    if (n > 0x80000u) n = 0x80000u;
    if (ch == 6u) {                     /* OTC: reverse-linked clear */
        for (i = 0; i < n; i++) {
            uint32_t v = (i == n - 1u) ? 0x00FFFFFFu : ((addr - 4u) & 0x1FFFFFu);
            ram32w(s, addr, v);
            addr = (addr - 4u) & 0x1FFFFCu;
        }
        dma_done(s, ch);
        return;
    }
    for (i = 0; i < n; i++) {
        if (from_ram) {
            uint32_t w = ram32(s, addr);
            if (ch == 2u) psx_gp0(s, w);
            else if (ch == 4u) psx_spu_dma_write(s, w);
            /* ch0 MDEC-in: not implemented (data discarded) */
        } else {
            uint32_t w = 0;
            if (ch == 2u) w = psx_gpuread(s);
            else if (ch == 4u) w = psx_spu_dma_read(s);
            ram32w(s, addr, w);
        }
        addr = (addr + step) & 0x1FFFFCu;
    }
    d->madr = addr;
    if (sync == 1u) d->bcr &= 0xFFFFu;
    dma_done(s, ch);
}

static uint32_t dma_read(hs_psx_t *s, uint32_t off)
{
    if (off == 0x70u) return s->dpcr;
    if (off == 0x74u) return s->dicr;
    {
        uint32_t ch = off >> 4, r = off & 0xFu;
        if (ch > 6u) return 0;
        if (r == 0) return s->dma[ch].madr;
        if (r == 4) return s->dma[ch].bcr;
        if (r == 8) return s->dma[ch].chcr;
    }
    return 0;
}

static void dma_write(hs_psx_t *s, uint32_t off, uint32_t v)
{
    if (off == 0x70u) { s->dpcr = v; return; }
    if (off == 0x74u) {
        uint32_t old = s->dicr;
        uint32_t ack = v & 0x7F000000u;
        s->dicr = (old & 0x7F000000u & ~ack) | (v & 0x00FF803Fu);
        dma_irq_update(s, old);
        return;
    }
    {
        uint32_t ch = off >> 4, r = off & 0xFu;
        if (ch > 6u) return;
        if (r == 0) s->dma[ch].madr = v & 0xFFFFFFu;
        else if (r == 4) s->dma[ch].bcr = v;
        else if (r == 8) {
            if (ch == 6u) v = (v & 0x51000000u) | 2u;
            s->dma[ch].chcr = v;
            if (v & (1u << 24)) dma_run(s, ch);
        }
    }
}

/* ---------------- SIO0 pad ---------------- */
static uint32_t pad_bits(uint32_t in)
{
    uint32_t b = 0;
    if (in & HS_BTN_SELECT) b |= 1u << 0;
    if (in & HS_BTN_L3)     b |= 1u << 1;
    if (in & HS_BTN_R3)     b |= 1u << 2;
    if (in & HS_BTN_START)  b |= 1u << 3;
    if (in & HS_BTN_UP)     b |= 1u << 4;
    if (in & HS_BTN_RIGHT)  b |= 1u << 5;
    if (in & HS_BTN_DOWN)   b |= 1u << 6;
    if (in & HS_BTN_LEFT)   b |= 1u << 7;
    if (in & HS_BTN_L2)     b |= 1u << 8;
    if (in & HS_BTN_R2)     b |= 1u << 9;
    if (in & HS_BTN_L)      b |= 1u << 10;
    if (in & HS_BTN_R)      b |= 1u << 11;
    if (in & HS_PS1_TRIANGLE) b |= 1u << 12;
    if (in & HS_PS1_CIRCLE)   b |= 1u << 13;
    if (in & HS_PS1_CROSS)    b |= 1u << 14;
    if (in & HS_PS1_SQUARE)   b |= 1u << 15;
    return (~b) & 0xFFFFu;   /* active low */
}

uint32_t psx_pad_word(const hs_psx_t *s);
uint32_t psx_pad_word(const hs_psx_t *s) { return pad_bits(s->input); }

static void sio_tx(hs_psx_t *s, uint32_t v)
{
    psx_sio_t *p = &s->sio;
    uint32_t reply = 0xFFu, ack = 0;
    if ((p->ctrl & 2u) && !(p->ctrl & 0x2000u)) {   /* /JOY1 selected */
        uint32_t pad = pad_bits(s->input);
        switch (p->seq) {
        case 0: if (v == 0x01u) { reply = 0xFFu; ack = 1; p->dev = 1; } else p->dev = 0; break;
        case 1: if (p->dev) { reply = 0x41u; ack = 1; } break;
        case 2: if (p->dev) { reply = 0x5Au; ack = 1; } break;
        case 3: if (p->dev) { reply = pad & 0xFFu; ack = 1; } break;
        case 4: if (p->dev) { reply = pad >> 8; ack = 0; p->dev = 0; } break;
        default: break;
        }
        p->seq++;
    }
    if (p->rx_n < 8u) { p->rx[(p->rx_r + p->rx_n) & 7u] = (uint8_t)reply; p->rx_n++; }
    p->stat |= 0x7u;                  /* TX ready 1/2, RX not empty */
    if (ack) { p->stat |= 0x80u; p->ack_pending = 1; } else p->stat &= ~0x80u;
}

static uint32_t sio_read(hs_psx_t *s, uint32_t off)
{
    psx_sio_t *p = &s->sio;
    switch (off) {
    case 0x0: {
        uint32_t v = 0xFFu;
        if (p->rx_n) { v = p->rx[p->rx_r & 7u]; p->rx_r = (p->rx_r + 1u) & 7u; p->rx_n--; }
        if (!p->rx_n) p->stat &= ~2u;
        return v | (v << 8) | (v << 16) | (v << 24);
    }
    case 0x4: return p->stat | 0x5u | (p->rx_n ? 2u : 0u);
    case 0x8: return p->mode;
    case 0xA: return p->ctrl;
    case 0xE: return p->baud;
    default: return 0;
    }
}

static void sio_write(hs_psx_t *s, uint32_t off, uint32_t v)
{
    psx_sio_t *p = &s->sio;
    switch (off) {
    case 0x0: sio_tx(s, v & 0xFFu); break;
    case 0x8: p->mode = (uint16_t)v; break;
    case 0xA:
        if (v & 0x40u) { memset(p, 0, sizeof(*p)); return; }
        if (v & 0x10u) p->stat &= ~0x238u;
        if (!(v & 2u)) { p->seq = 0; p->dev = 0; }
        p->ctrl = (uint16_t)(v & ~0x50u);
        break;
    case 0xE: p->baud = (uint16_t)v; break;
    default: break;
    }
}

/* ---------------- CD-ROM controller stub (no disc inserted) ---------------- */
static void cd_deliver(hs_psx_t *s, uint8_t intn, const uint8_t *r, uint32_t n)
{
    psx_cdrom_t *c = &s->cd;
    uint32_t i;
    c->ifl = intn;
    c->resp_n = 0; c->resp_r = 0;
    for (i = 0; i < n && i < 16u; i++) c->resp[c->resp_n++] = r[i];
    if (c->ie & c->ifl & 0x1Fu) psx_irq(s, 2);
}

static void cd_command(hs_psx_t *s, uint32_t cmd)
{
    psx_cdrom_t *c = &s->cd;
    uint8_t st = 0x10;               /* shell open / no disc */
    uint8_t r[8];
    c->pending_int = 0;
    switch (cmd) {
    case 0x01: r[0] = st; cd_deliver(s, 3, r, 1); break;                       /* GetStat */
    case 0x19: r[0] = 0x94; r[1] = 0x09; r[2] = 0x19; r[3] = 0xC0;              /* Test 20h */
               cd_deliver(s, 3, r, 4); break;
    case 0x0A: case 0x1A:                                                       /* Init, GetID */
        r[0] = st; cd_deliver(s, 3, r, 1);
        c->pending_int = cmd == 0x0A ? 2 : 5;
        c->pending_resp[0] = cmd == 0x0A ? st : 0x08; c->pending_resp[1] = 0x40;
        c->pending_n = cmd == 0x0A ? 1u : 8u;
        { uint32_t k; for (k = 2; k < 8u; k++) c->pending_resp[k] = 0; }
        break;
    default: r[0] = st | 1u; r[1] = 0x40; cd_deliver(s, 5, r, 2); break;
    }
    c->param_n = 0;
}

static uint32_t cd_read(hs_psx_t *s, uint32_t off)
{
    psx_cdrom_t *c = &s->cd;
    if (off == 0) return (uint32_t)c->index | 0x18u | (c->resp_n > c->resp_r ? 0x20u : 0u);
    if (off == 1) { uint32_t v = 0; if (c->resp_r < c->resp_n) v = c->resp[c->resp_r++]; return v; }
    if (off == 2) return 0;
    if (off == 3) return (c->index & 1u) ? (0xE0u | c->ifl) : (0xE0u | c->ie);
    return 0;
}

static void cd_write(hs_psx_t *s, uint32_t off, uint32_t v)
{
    psx_cdrom_t *c = &s->cd;
    v &= 0xFFu;
    if (off == 0) { c->index = (uint8_t)(v & 3u); return; }
    switch ((c->index << 2) | off) {
    case (0 << 2) | 1: cd_command(s, v); break;
    case (0 << 2) | 2: if (c->param_n < 16u) c->param[c->param_n++] = (uint8_t)v; break;
    case (1 << 2) | 2: c->ie = (uint8_t)(v & 0x1Fu); break;
    case (1 << 2) | 3:
        c->ifl &= (uint8_t)~(v & 0x1Fu);
        if (v & 0x40u) c->param_n = 0;
        if (c->ifl == 0 && c->pending_int) {
            uint8_t n = c->pending_int;
            c->pending_int = 0;
            cd_deliver(s, n, c->pending_resp, c->pending_n);
        }
        break;
    default: break;
    }
}

/* ---------------- IO dispatch ---------------- */
static uint32_t io_read(hs_psx_t *s, uint32_t p, uint32_t size)
{
    uint32_t o = p - 0x1F801000u;
    if (o < 0x24u) return s->memctl[o >> 2];
    if (o == 0x60u) return s->ram_size;
    if (o >= 0x40u && o < 0x50u) return sio_read(s, o - 0x40u);
    if (o >= 0x50u && o < 0x60u) return (o == 0x54u) ? 0x5u : 0u;     /* SIO1 idle */
    if (o == 0x70u) return s->istat;
    if (o == 0x74u) return s->imask;
    if (o >= 0x80u && o < 0x100u) return dma_read(s, o - 0x80u);
    if (o >= 0x100u && o < 0x130u) {
        uint32_t n = (o - 0x100u) >> 4, r = o & 0xFu;
        tmr_flush(s);
        if (r == 0) return s->tmr[n].count;
        if (r == 4) { uint32_t m = s->tmr[n].mode; s->tmr[n].mode &= ~0x1800u; return m; }
        if (r == 8) return s->tmr[n].target;
        return 0;
    }
    if (o >= 0x800u && o < 0x804u) return cd_read(s, o - 0x800u);
    if (o == 0x810u) return psx_gpuread(s);
    if (o == 0x814u) return psx_gpustat(s);
    if (o == 0x820u) return 0;
    if (o == 0x824u) return 0x80040000u;                                 /* MDEC idle */
    if (o >= 0xC00u && o < 0xE80u) {
        if (size == 4u) return psx_spu_read(s, o - 0xC00u) | (psx_spu_read(s, o - 0xC00u + 2u) << 16);
        return psx_spu_read(s, (o - 0xC00u) & ~1u);
    }
    if (o >= 0x1000u) return 0xFFFFFFFFu;                                 /* expansion 2 */
    return 0;
}

static void io_write(hs_psx_t *s, uint32_t p, uint32_t v, uint32_t size)
{
    uint32_t o = p - 0x1F801000u;
    if (o < 0x24u) { s->memctl[o >> 2] = v; return; }
    if (o == 0x60u) { s->ram_size = v; return; }
    if (o >= 0x40u && o < 0x50u) { sio_write(s, o - 0x40u, v); return; }
    if (o == 0x70u) { s->istat &= v & 0x7FFu; return; }
    if (o == 0x74u) { s->imask = v & 0x7FFu; return; }
    if (o >= 0x80u && o < 0x100u) { dma_write(s, o - 0x80u, v); return; }
    if (o >= 0x100u && o < 0x130u) {
        uint32_t n = (o - 0x100u) >> 4, r = o & 0xFu;
        tmr_flush(s);
        if (r == 0) s->tmr[n].count = v & 0xFFFFu;
        else if (r == 4) { s->tmr[n].mode = (v & 0x3FFu) | 0x400u; s->tmr[n].count = 0; }
        else if (r == 8) s->tmr[n].target = v & 0xFFFFu;
        return;
    }
    if (o >= 0x800u && o < 0x804u) { cd_write(s, o - 0x800u, v); return; }
    if (o == 0x810u) { psx_gp0(s, v); return; }
    if (o == 0x814u) { psx_gp1(s, v); return; }
    if (o == 0x820u || o == 0x824u) return;                              /* MDEC: not implemented */
    if (o >= 0xC00u && o < 0xE80u) {
        psx_spu_write(s, (o - 0xC00u) & ~1u, v & 0xFFFFu);
        if (size == 4u) psx_spu_write(s, ((o - 0xC00u) & ~1u) + 2u, v >> 16);
        return;
    }
    if (o == 0x1023u || o == 0x1080u) {                                   /* DUART / debug tty */
        if (s->tty_len < HS_PSX_TTY_SIZE - 1u) { s->tty[s->tty_len++] = (char)v; s->tty[s->tty_len] = 0; }
        return;
    }
}

/* ---------------- public accessors ---------------- */
static inline int in_bios(uint32_t p) { return p >= 0x1FC00000u && p < 0x1FC80000u; }

uint32_t psx_read32(hs_psx_t *s, uint32_t a)
{
    uint32_t p;
    if (a >= 0xFFFE0000u) return s->cache_ctl;
    p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) return rd32(s->ram + (p & 0x1FFFFCu));
    if (p >= 0x1F800000u && p < 0x1F800400u) return rd32(s->scratch + (p & 0x3FCu));
    if (p >= 0x1F801000u && p < 0x1F803000u) return io_read(s, p & ~3u, 4u);
    if (in_bios(p)) return s->bios ? rd32(s->bios + (p & 0x7FFFCu)) : 0u;
    if (p >= 0x1F000000u && p < 0x1F800000u) return 0xFFFFFFFFu;
    s->bad_access++;
    return 0;
}

uint32_t psx_read16(hs_psx_t *s, uint32_t a)
{
    uint32_t p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) return rd16(s->ram + (p & 0x1FFFFEu));
    if (p >= 0x1F800000u && p < 0x1F800400u) return rd16(s->scratch + (p & 0x3FEu));
    if (p >= 0x1F801000u && p < 0x1F803000u) {
        uint32_t o = p - 0x1F801000u;
        if (o >= 0xC00u && o < 0xE80u) return io_read(s, p, 2u) & 0xFFFFu;
        if (o >= 0x40u && o < 0x50u) return io_read(s, p, 2u) & 0xFFFFu;
        return (io_read(s, p & ~3u, 2u) >> ((p & 2u) * 8u)) & 0xFFFFu;
    }
    if (in_bios(p)) return s->bios ? rd16(s->bios + (p & 0x7FFFEu)) : 0u;
    if (p >= 0x1F000000u && p < 0x1F800000u) return 0xFFFFu;
    s->bad_access++;
    return 0;
}

uint32_t psx_read8(hs_psx_t *s, uint32_t a)
{
    uint32_t p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) return s->ram[p & 0x1FFFFFu];
    if (p >= 0x1F800000u && p < 0x1F800400u) return s->scratch[p & 0x3FFu];
    if (p >= 0x1F801000u && p < 0x1F803000u) {
        uint32_t o = p - 0x1F801000u;
        if ((o >= 0x800u && o < 0x804u) || (o >= 0x40u && o < 0x50u)) return io_read(s, p, 1u) & 0xFFu;
        return (io_read(s, p & ~3u, 1u) >> ((p & 3u) * 8u)) & 0xFFu;
    }
    if (in_bios(p)) return s->bios ? s->bios[p & 0x7FFFFu] : 0u;
    if (p >= 0x1F000000u && p < 0x1F800000u) return 0xFFu;
    s->bad_access++;
    return 0;
}

void psx_write32(hs_psx_t *s, uint32_t a, uint32_t v)
{
    uint32_t p;
    if (a >= 0xFFFE0000u) { s->cache_ctl = v; return; }
    p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) { wr32(s->ram + (p & 0x1FFFFCu), v); return; }
    if (p >= 0x1F800000u && p < 0x1F800400u) { wr32(s->scratch + (p & 0x3FCu), v); return; }
    if (p >= 0x1F801000u && p < 0x1F803000u) { io_write(s, p & ~3u, v, 4u); return; }
    if (in_bios(p)) return;
    s->bad_access++;
}

void psx_write16(hs_psx_t *s, uint32_t a, uint32_t v)
{
    uint32_t p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) { wr16(s->ram + (p & 0x1FFFFEu), v); return; }
    if (p >= 0x1F800000u && p < 0x1F800400u) { wr16(s->scratch + (p & 0x3FEu), v); return; }
    if (p >= 0x1F801000u && p < 0x1F803000u) {
        uint32_t o = p - 0x1F801000u;
        if ((o >= 0xC00u && o < 0xE80u) || (o >= 0x40u && o < 0x50u) || (o >= 0x100u && o < 0x130u) ||
            o == 0x70u || o == 0x74u)
            io_write(s, p, v & 0xFFFFu, 2u);
        else
            io_write(s, p & ~3u, (v & 0xFFFFu) << ((p & 2u) * 8u), 2u);
        return;
    }
    if (in_bios(p)) return;
    s->bad_access++;
}

void psx_write8(hs_psx_t *s, uint32_t a, uint32_t v)
{
    uint32_t p = a & 0x1FFFFFFFu;
    if (p < 0x00800000u) { s->ram[p & 0x1FFFFFu] = (uint8_t)v; return; }
    if (p >= 0x1F800000u && p < 0x1F800400u) { s->scratch[p & 0x3FFu] = (uint8_t)v; return; }
    if (p >= 0x1F801000u && p < 0x1F803000u) { io_write(s, p, v & 0xFFu, 1u); return; }
    if (in_bios(p)) return;
    s->bad_access++;
}

void psx_bus_reset(hs_psx_t *s)
{
    uint32_t i;
    memset(s->ram, 0, sizeof(s->ram));
    memset(s->scratch, 0, sizeof(s->scratch));
    memset(s->dma, 0, sizeof(s->dma));
    memset(s->tmr, 0, sizeof(s->tmr));
    memset(&s->sio, 0, sizeof(s->sio));
    memset(&s->cd, 0, sizeof(s->cd));
    for (i = 0; i < 16u; i++) s->memctl[i] = 0;
    s->ram_size = 0x00000B88u;
    s->cache_ctl = 0;
    s->istat = s->imask = 0;
    s->dpcr = 0x07654321u;
    s->dicr = 0;
    s->tmr_accum = 0;
    for (i = 0; i < 3u; i++) s->tmr[i].mode = 0x400u;
}
