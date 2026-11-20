/* SPDX-License-Identifier: MIT
 * gba_bus.c - clean-room GBA memory map, IO registers, DMA0-3, timers 0-3,
 * interrupts, Direct Sound FIFOs (CPU side) and battery saves
 * (SRAM / Flash 64K / Flash 128K / EEPROM 512B-8K).
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include "gba_internal.h"

/* ------------------------------ interrupts ------------------------------ */
void gba_raise_irq(hs_gba *g, uint16_t bits) { g->ifl |= bits; }

/* ------------------------------ audio log ------------------------------- */
void gba_audio_log(hs_gba *g, uint16_t reg, uint16_t val) {
  gba_snapshot *s = &g->snap[g->snap_w];
  if (s->n_ev >= GBA_AUDIO_EV_MAX) { g->ev_dropped++; return; }
  gba_audio_ev *e = &s->ev[s->n_ev++];
  e->t = g->vcount * GBA_CYC_LINE + g->line_cycle;
  e->reg = reg;
  e->val = val;
}

/* --------------------------------- DMA ---------------------------------- */
static void dma_run(hs_gba *g, int ch) {
  gba_dma *d = &g->dma[ch];
  uint16_t ctrl = d->ctrl;
  uint32_t timing = (ctrl >> 12) & 3u;
  int sound = (ch == 1 || ch == 2) && timing == 3u;
  int wide = sound || (ctrl & (1u << 10));
  uint32_t dctl = sound ? 2u : (ctrl >> 5) & 3u, sctl = (ctrl >> 7) & 3u;
  uint32_t count = sound ? 4u : d->cnt;
  uint32_t step = wide ? 4u : 2u;
  int32_t sstep = (sctl == 1u) ? -(int32_t)step : (sctl == 2u) ? 0 : (int32_t)step;
  int32_t dstep = (dctl == 1u) ? -(int32_t)step : (dctl == 2u) ? 0 : (int32_t)step;
  uint32_t src = d->src, dst = d->dst;
  if (g->save_type == HS_GBA_SAVE_EEPROM && ch == 3 && !g->eeprom.addr_bits &&
      (dst >> 24) == 0x0Du) {
    g->eeprom.addr_bits = (count == 9u || count == 73u) ? 6 : 14;
  }
  g->in_dma = 1;
  for (uint32_t i = 0; i < count; i++) {
    if (wide) gba_write32(g, dst & ~3u, gba_read32(g, src & ~3u));
    else gba_write16(g, dst & ~1u, gba_read16(g, src & ~1u));
    src += (uint32_t)sstep;
    dst += (uint32_t)dstep;
  }
  g->in_dma = 0;
  g->dma_stall += count * 2u + 2u;
  d->src = src;
  if (!sound) d->dst = dst;
  uint32_t base = 0xB0u + 12u * (uint32_t)ch;
  if ((ctrl & (1u << 9)) && timing != 0u) {
    uint32_t c = gba_ld16(&g->io[base + 8u]);
    uint32_t cmax = (ch == 3) ? 0x10000u : 0x4000u;
    c &= cmax - 1u;
    d->cnt = c ? c : cmax;
    if (dctl == 3u) d->dst = gba_ld32(&g->io[base + 4u]) & ((ch == 3) ? 0x0FFFFFFFu : 0x07FFFFFFu);
  } else {
    d->ctrl &= (uint16_t)~0x8000u;
    gba_st16(&g->io[base + 10u], d->ctrl);
  }
  if (ctrl & (1u << 14)) gba_raise_irq(g, (uint16_t)(IRQ_DMA0 << ch));
}

void gba_dma_trigger(hs_gba *g, int timing) {
  for (int ch = 0; ch < 4; ch++) {
    gba_dma *d = &g->dma[ch];
    if ((d->ctrl & 0x8000u) && (int)((d->ctrl >> 12) & 3u) == timing) {
      if (timing == 3 && (ch == 1 || ch == 2)) continue; /* sound DMA: FIFO-driven */
      dma_run(g, ch);
    }
  }
}

static void dma_ctrl_write(hs_gba *g, int ch, uint16_t v) {
  gba_dma *d = &g->dma[ch];
  uint16_t old = d->ctrl;
  uint32_t base = 0xB0u + 12u * (uint32_t)ch;
  d->ctrl = v;
  gba_st16(&g->io[base + 10u], v);
  if (!(old & 0x8000u) && (v & 0x8000u)) {
    d->src = gba_ld32(&g->io[base]) & ((ch == 0) ? 0x07FFFFFFu : 0x0FFFFFFFu);
    d->dst = gba_ld32(&g->io[base + 4u]) & ((ch == 3) ? 0x0FFFFFFFu : 0x07FFFFFFu);
    uint32_t cmax = (ch == 3) ? 0x10000u : 0x4000u;
    uint32_t c = gba_ld16(&g->io[base + 8u]) & (cmax - 1u);
    d->cnt = c ? c : cmax;
    if (((v >> 12) & 3u) == 0u && !g->in_dma) dma_run(g, ch);
  }
}

/* ---------------------------- Direct Sound ------------------------------ */
static void fifo_push(hs_gba *g, int ch, uint8_t b) {
  if (g->fifo_n[ch] >= 32u) return;
  g->fifo[ch][g->fifo_w[ch]] = (int8_t)b;
  g->fifo_w[ch] = (uint8_t)((g->fifo_w[ch] + 1u) & 31u);
  g->fifo_n[ch]++;
}

static void fifo_reset(hs_gba *g, int ch) { g->fifo_r[ch] = g->fifo_w[ch] = g->fifo_n[ch] = 0; }

static void fifo_pop(hs_gba *g, int ch) {
  int8_t s = 0;
  if (g->fifo_n[ch]) {
    s = g->fifo[ch][g->fifo_r[ch]];
    g->fifo_r[ch] = (uint8_t)((g->fifo_r[ch] + 1u) & 31u);
    g->fifo_n[ch]--;
  }
  gba_audio_log(g, (uint16_t)(0xF0u + (uint32_t)ch), (uint16_t)(uint8_t)s);
  if (g->fifo_n[ch] <= 16u) {
    uint32_t fa = 0x040000A0u + 4u * (uint32_t)ch;
    for (int d = 1; d <= 2; d++) {
      gba_dma *dm = &g->dma[d];
      if ((dm->ctrl & 0x8000u) && ((dm->ctrl >> 12) & 3u) == 3u && (dm->dst & ~3u) == fa &&
          !g->in_dma) {
        dma_run(g, d);
        break;
      }
    }
  }
}

/* -------------------------------- timers -------------------------------- */
static uint32_t timer_add(hs_gba *g, int i, uint32_t ticks) {
  gba_timer *t = &g->tm[i];
  uint32_t ovf = 0;
  for (uint32_t guard = 0; ticks && guard < 0x20000u; guard++) {
    uint32_t room = 0x10000u - t->counter;
    if (ticks < room) { t->counter = (uint16_t)(t->counter + ticks); break; }
    ticks -= room;
    t->counter = t->reload;
    ovf++;
    if (t->ctrl & 0x40u) gba_raise_irq(g, (uint16_t)(IRQ_TIMER0 << i));
    if (i < 2) {
      uint16_t sh = gba_ld16(&g->io[0x82]);
      if ((sh & 0x0300u) && (int)((sh >> 10) & 1u) == i) fifo_pop(g, 0);
      if ((sh & 0x3000u) && (int)((sh >> 14) & 1u) == i) fifo_pop(g, 1);
    }
  }
  return ovf;
}

void gba_timers_advance(hs_gba *g, uint32_t cycles) {
  static const uint8_t shifts[4] = {0, 6, 8, 10};
  for (int i = 0; i < 4; i++) {
    gba_timer *t = &g->tm[i];
    if (!(t->ctrl & 0x80u)) continue;
    if (i > 0 && (t->ctrl & 4u)) continue;
    uint32_t sh = shifts[t->ctrl & 3u];
    t->presc += cycles;
    uint32_t ticks = t->presc >> sh;
    t->presc &= (1u << sh) - 1u;
    if (!ticks) continue;
    uint32_t ovf = timer_add(g, i, ticks);
    for (int j = i + 1; j < 4 && ovf; j++) {
      gba_timer *n = &g->tm[j];
      if (!(n->ctrl & 0x80u) || !(n->ctrl & 4u)) break;
      ovf = timer_add(g, j, ovf);
    }
  }
}

/* -------------------------------- saves --------------------------------- */
void gba_detect_save(hs_gba *g) {
  static const struct { const char *tag; hs_gba_save_type t; } tags[] = {
    {"EEPROM_V", HS_GBA_SAVE_EEPROM}, {"SRAM_F_V", HS_GBA_SAVE_SRAM},
    {"SRAM_V", HS_GBA_SAVE_SRAM}, {"FLASH1M_V", HS_GBA_SAVE_FLASH128},
    {"FLASH512_V", HS_GBA_SAVE_FLASH64}, {"FLASH_V", HS_GBA_SAVE_FLASH64}};
  g->save_type = HS_GBA_SAVE_NONE;
  for (uint32_t i = 0; i + 12u <= g->rom_size; i += 4u) {
    for (size_t k = 0; k < sizeof tags / sizeof tags[0]; k++) {
      size_t n = strlen(tags[k].tag);
      if (i + n <= g->rom_size && memcmp(g->rom + i, tags[k].tag, n) == 0) {
        g->save_type = tags[k].t;
        return;
      }
    }
  }
}

static int is_eeprom(const hs_gba *g, uint32_t a) {
  if (g->save_type != HS_GBA_SAVE_EEPROM || (a >> 24) != 0x0Du) return 0;
  return g->rom_size <= 0x1000000u || (a & 0x00FFFFFFu) >= 0x00FFFF00u;
}

static uint16_t eeprom_read(hs_gba *g) {
  if (g->eeprom.rbits > 0) {
    int b = g->eeprom.rbits--;
    if (b > 64) return 0;
    return (uint16_t)((g->eeprom.rdata >> (b - 1)) & 1u);
  }
  return 1;
}

static void eeprom_write(hs_gba *g, uint16_t v) {
  uint32_t bit = v & 1u;
  int ab = g->eeprom.addr_bits ? g->eeprom.addr_bits : 6;
  g->eeprom.shift = (g->eeprom.shift << 1) | bit;
  g->eeprom.nbits++;
  int n = g->eeprom.nbits;
  if (g->eeprom.mode == 0) {
    if (n == 2) { g->eeprom.mode = (int)g->eeprom.shift; g->eeprom.shift = 0; g->eeprom.nbits = 0;
      if (g->eeprom.mode != 2 && g->eeprom.mode != 3) g->eeprom.mode = 0; }
    return;
  }
  if (n == ab) { g->eeprom.addr = (uint32_t)g->eeprom.shift & 0x3FFu; g->eeprom.shift = 0; return; }
  if (g->eeprom.mode == 3) { /* read request: addr + stop bit */
    if (n == ab + 1) {
      uint64_t d = 0;
      uint32_t base = (g->eeprom.addr * 8u) & (0x2000u - 8u);
      for (int i = 0; i < 8; i++) d = (d << 8) | g->save[base + (uint32_t)i];
      g->eeprom.rdata = d;
      g->eeprom.rbits = 68;
      g->eeprom.mode = 0; g->eeprom.nbits = 0; g->eeprom.shift = 0;
    }
    return;
  }
  /* write request: addr + 64 data + stop */
  if (n == ab + 64) {
    uint64_t d = g->eeprom.shift;
    uint32_t base = (g->eeprom.addr * 8u) & (0x2000u - 8u);
    for (int i = 7; i >= 0; i--) { g->save[base + (uint32_t)i] = (uint8_t)d; d >>= 8; }
    g->save_dirty = 1;
  } else if (n == ab + 65) {
    g->eeprom.mode = 0; g->eeprom.nbits = 0; g->eeprom.shift = 0; g->eeprom.rbits = 0;
  }
}

static uint8_t backup_read8(hs_gba *g, uint32_t a) {
  uint32_t off = a & 0xFFFFu;
  switch (g->save_type) {
    case HS_GBA_SAVE_SRAM: return g->save[off & 0x7FFFu];
    case HS_GBA_SAVE_FLASH64:
    case HS_GBA_SAVE_FLASH128:
      if (g->flash.id_mode && off < 2u) {
        if (g->save_type == HS_GBA_SAVE_FLASH128) return off ? 0x13 : 0x62; /* Sanyo 128K */
        return off ? 0x1B : 0x32;                                          /* Panasonic 64K */
      }
      return g->save[((uint32_t)g->flash.bank << 16) + off];
    default: return 0xFF;
  }
}

static void backup_write8(hs_gba *g, uint32_t a, uint8_t v) {
  uint32_t off = a & 0xFFFFu;
  if (g->save_type == HS_GBA_SAVE_SRAM) { g->save[off & 0x7FFFu] = v; g->save_dirty = 1; return; }
  if (g->save_type != HS_GBA_SAVE_FLASH64 && g->save_type != HS_GBA_SAVE_FLASH128) return;
  if (g->flash.write_armed) {
    g->save[((uint32_t)g->flash.bank << 16) + off] = v;
    g->flash.write_armed = 0;
    g->save_dirty = 1;
    return;
  }
  if (g->flash.bank_armed) {
    if (off == 0u) g->flash.bank = v & 1u;
    g->flash.bank_armed = 0;
    return;
  }
  switch (g->flash.state) {
    case 0:
      if (off == 0x5555u && v == 0xAAu) g->flash.state = 1;
      else if (v == 0xF0u) g->flash.id_mode = 0;
      break;
    case 1:
      g->flash.state = (off == 0x2AAAu && v == 0x55u) ? 2 : 0;
      break;
    default:
      g->flash.state = 0;
      if (off == 0x5555u) {
        switch (v) {
          case 0x90: g->flash.id_mode = 1; break;
          case 0xF0: g->flash.id_mode = 0; break;
          case 0x80: g->flash.erase_armed = 1; break;
          case 0x10:
            if (g->flash.erase_armed) { memset(g->save, 0xFF, GBA_SAVE_MAX); g->save_dirty = 1; }
            g->flash.erase_armed = 0;
            break;
          case 0xA0: g->flash.write_armed = 1; break;
          case 0xB0: if (g->save_type == HS_GBA_SAVE_FLASH128) g->flash.bank_armed = 1; break;
          default: break;
        }
      } else if (v == 0x30u && g->flash.erase_armed) {
        memset(g->save + ((uint32_t)g->flash.bank << 16) + (off & 0xF000u), 0xFF, 0x1000u);
        g->flash.erase_armed = 0;
        g->save_dirty = 1;
      }
      break;
  }
}

/* ---------------------------------- IO ---------------------------------- */
static uint16_t io_read16(hs_gba *g, uint32_t off) {
  switch (off) {
    case 0x004: {
      uint16_t v = (uint16_t)(gba_ld16(&g->io[4]) & 0xFFF8u);
      if (g->vcount >= 160u && g->vcount < 227u) v |= 1u;
      if (g->line_cycle >= GBA_CYC_HDRAW) v |= 2u;
      if (g->vcount == g->io[5]) v |= 4u;
      return v;
    }
    case 0x006: return (uint16_t)g->vcount;
    case 0x100: case 0x104: case 0x108: case 0x10C: return g->tm[(off - 0x100u) >> 2].counter;
    case 0x102: case 0x106: case 0x10A: case 0x10E: return g->tm[(off - 0x102u) >> 2].ctrl;
    case 0x130: return (uint16_t)(~g->keys & 0x3FFu);
    case 0x200: return g->ie;
    case 0x202: return g->ifl;
    case 0x204: return g->waitcnt;
    case 0x208: return g->ime;
    default:
      if (off >= 0x400u) return 0;
      return gba_ld16(&g->io[off & ~1u]);
  }
}

static void bg_ref_update(hs_gba *g, uint32_t off) {
  if (off >= 0x28u && off < 0x30u) {
    g->bgx[0] = ((int32_t)(gba_ld32(&g->io[0x28]) << 4)) >> 4;
    g->bgy[0] = ((int32_t)(gba_ld32(&g->io[0x2C]) << 4)) >> 4;
  } else if (off >= 0x38u && off < 0x40u) {
    g->bgx[1] = ((int32_t)(gba_ld32(&g->io[0x38]) << 4)) >> 4;
    g->bgy[1] = ((int32_t)(gba_ld32(&g->io[0x3C]) << 4)) >> 4;
  }
}

static void io_write16(hs_gba *g, uint32_t off, uint16_t v) {
  if (off >= 0x400u) return;
  switch (off) {
    case 0x004: gba_st16(&g->io[4], (uint16_t)((gba_ld16(&g->io[4]) & 7u) | (v & ~7u))); return;
    case 0x006: return;
    case 0x082:
      if (v & 0x0800u) fifo_reset(g, 0);
      if (v & 0x8000u) fifo_reset(g, 1);
      v &= (uint16_t)~0x8800u;
      gba_st16(&g->io[off], v);
      gba_audio_log(g, (uint16_t)off, v);
      return;
    case 0x0A0: case 0x0A2: fifo_push(g, 0, (uint8_t)v); fifo_push(g, 0, (uint8_t)(v >> 8)); return;
    case 0x0A4: case 0x0A6: fifo_push(g, 1, (uint8_t)v); fifo_push(g, 1, (uint8_t)(v >> 8)); return;
    case 0x0BA: dma_ctrl_write(g, 0, v); return;
    case 0x0C6: dma_ctrl_write(g, 1, v); return;
    case 0x0D2: dma_ctrl_write(g, 2, v); return;
    case 0x0DE: dma_ctrl_write(g, 3, v); return;
    case 0x100: case 0x104: case 0x108: case 0x10C:
      g->tm[(off - 0x100u) >> 2].reload = v;
      gba_st16(&g->io[off], v);
      return;
    case 0x102: case 0x106: case 0x10A: case 0x10E: {
      gba_timer *t = &g->tm[(off - 0x102u) >> 2];
      if (!(t->ctrl & 0x80u) && (v & 0x80u)) { t->counter = t->reload; t->presc = 0; }
      t->ctrl = v & 0xC7u;
      gba_st16(&g->io[off], t->ctrl);
      return;
    }
    case 0x200: g->ie = v & 0x3FFFu; return;
    case 0x202: g->ifl &= (uint16_t)~v; return;
    case 0x204: g->waitcnt = v; return;
    case 0x208: g->ime = v & 1u; return;
    case 0x300:
      gba_st16(&g->io[off], v);
      if (v & 0xFF00u) g->cpu.halted = 1;
      return;
    default: break;
  }
  gba_st16(&g->io[off], v);
  if (off >= 0x28u && off < 0x40u) bg_ref_update(g, off);
  if (off >= 0x60u && off < 0xA0u) gba_audio_log(g, (uint16_t)off, v);
}

static void io_write8(hs_gba *g, uint32_t off, uint8_t v) {
  if (off >= 0x400u) return;
  if (off == 0x202u) { g->ifl &= (uint16_t)~v; return; }
  if (off == 0x203u) { g->ifl &= (uint16_t)~((uint16_t)v << 8); return; }
  if (off == 0x301u) { g->io[off] = v; g->cpu.halted = 1; return; }
  if (off >= 0xA0u && off < 0xA8u) { fifo_push(g, off < 0xA4u ? 0 : 1, v); return; }
  uint32_t a = off & ~1u;
  uint16_t cur = io_read16(g, a);
  if (a == 0x202u || a == 0x004u) cur = gba_ld16(&g->io[a]);
  uint16_t nv = (off & 1u) ? (uint16_t)((cur & 0x00FFu) | ((uint16_t)v << 8))
                           : (uint16_t)((cur & 0xFF00u) | v);
  io_write16(g, a, nv);
}

/* ---------------------------- memory access ----------------------------- */
static inline uint32_t vram_off(uint32_t a) {
  a &= 0x1FFFFu;
  return a >= 0x18000u ? a - 0x8000u : a;
}

static inline uint32_t rom_open_bus16(uint32_t a) { return (a >> 1) & 0xFFFFu; }

uint8_t gba_read8(hs_gba *g, uint32_t a) {
  switch (a >> 24) {
    case 0x0: return a < 0x4000u ? g->bios[a] : 0;
    case 0x2: return g->ewram[a & 0x3FFFFu];
    case 0x3: return g->iwram[a & 0x7FFFu];
    case 0x4: { uint32_t o = a & 0xFFFFFFu; uint16_t v = io_read16(g, o & ~1u); return (uint8_t)((o & 1u) ? v >> 8 : v); }
    case 0x5: return g->pal[a & 0x3FFu];
    case 0x6: return g->vram[vram_off(a)];
    case 0x7: return g->oam[a & 0x3FFu];
    case 0x8: case 0x9: case 0xA: case 0xB: case 0xC: case 0xD: {
      if (is_eeprom(g, a)) return (uint8_t)eeprom_read(g);
      uint32_t o = a & 0x1FFFFFFu;
      if (o < g->rom_size) return g->rom[o];
      return (uint8_t)(rom_open_bus16(a) >> ((a & 1u) * 8u));
    }
    case 0xE: case 0xF: return backup_read8(g, a);
    default: return 0;
  }
}

uint16_t gba_read16(hs_gba *g, uint32_t a) {
  a &= ~1u;
  switch (a >> 24) {
    case 0x0: return a < 0x4000u ? gba_ld16(&g->bios[a]) : 0;
    case 0x2: return gba_ld16(&g->ewram[a & 0x3FFFFu]);
    case 0x3: return gba_ld16(&g->iwram[a & 0x7FFFu]);
    case 0x4: return io_read16(g, a & 0xFFFFFFu);
    case 0x5: return gba_ld16(&g->pal[a & 0x3FFu]);
    case 0x6: return gba_ld16(&g->vram[vram_off(a)]);
    case 0x7: return gba_ld16(&g->oam[a & 0x3FFu]);
    case 0x8: case 0x9: case 0xA: case 0xB: case 0xC: case 0xD: {
      if (is_eeprom(g, a)) return eeprom_read(g);
      uint32_t o = a & 0x1FFFFFFu;
      if (o + 1u < g->rom_size) return gba_ld16(&g->rom[o]);
      return (uint16_t)rom_open_bus16(a);
    }
    case 0xE: case 0xF: return (uint16_t)(backup_read8(g, a) * 0x0101u);
    default: return 0;
  }
}

uint32_t gba_read32(hs_gba *g, uint32_t a) {
  a &= ~3u;
  switch (a >> 24) {
    case 0x2: return gba_ld32(&g->ewram[a & 0x3FFFFu]);
    case 0x3: return gba_ld32(&g->iwram[a & 0x7FFFu]);
    case 0x8: case 0x9: case 0xA: case 0xB: case 0xC: {
      uint32_t o = a & 0x1FFFFFFu;
      if (o + 3u < g->rom_size) return gba_ld32(&g->rom[o]);
      break;
    }
    case 0xE: case 0xF: return backup_read8(g, a) * 0x01010101u;
    default: break;
  }
  return (uint32_t)gba_read16(g, a) | ((uint32_t)gba_read16(g, a + 2u) << 16);
}

void gba_write8(hs_gba *g, uint32_t a, uint8_t v) {
  switch (a >> 24) {
    case 0x2: g->ewram[a & 0x3FFFFu] = v; return;
    case 0x3: g->iwram[a & 0x7FFFu] = v; return;
    case 0x4: io_write8(g, a & 0xFFFFFFu, v); return;
    case 0x5: gba_st16(&g->pal[a & 0x3FEu], (uint16_t)(v * 0x0101u)); return;
    case 0x6: {
      uint32_t o = vram_off(a);
      uint32_t mode = g->io[0] & 7u;
      if (o < (mode >= 3u ? 0x14000u : 0x10000u)) gba_st16(&g->vram[o & ~1u], (uint16_t)(v * 0x0101u));
      return;
    }
    case 0xE: case 0xF: backup_write8(g, a, v); return;
    default: return; /* OAM byte writes ignored; ROM/BIOS read-only */
  }
}

void gba_write16(hs_gba *g, uint32_t a, uint16_t v) {
  a &= ~1u;
  switch (a >> 24) {
    case 0x2: gba_st16(&g->ewram[a & 0x3FFFFu], v); return;
    case 0x3: gba_st16(&g->iwram[a & 0x7FFFu], v); return;
    case 0x4: io_write16(g, a & 0xFFFFFFu, v); return;
    case 0x5: gba_st16(&g->pal[a & 0x3FFu], v); return;
    case 0x6: gba_st16(&g->vram[vram_off(a)], v); return;
    case 0x7: gba_st16(&g->oam[a & 0x3FFu], v); return;
    case 0xD: if (is_eeprom(g, a)) eeprom_write(g, v); return;
    case 0xE: case 0xF: backup_write8(g, a, (uint8_t)(v >> ((a & 1u) * 8u))); return;
    default: return;
  }
}

void gba_write32(hs_gba *g, uint32_t a, uint32_t v) {
  a &= ~3u;
  switch (a >> 24) {
    case 0x2: gba_st32(&g->ewram[a & 0x3FFFFu], v); return;
    case 0x3: gba_st32(&g->iwram[a & 0x7FFFu], v); return;
    case 0x4: {
      uint32_t o = a & 0xFFFFFFu;
      if (o == 0xA0u || o == 0xA4u) {
        int ch = o == 0xA0u ? 0 : 1;
        for (int i = 0; i < 4; i++) fifo_push(g, ch, (uint8_t)(v >> (8 * i)));
        return;
      }
      io_write16(g, o, (uint16_t)v);
      io_write16(g, o + 2u, (uint16_t)(v >> 16));
      return;
    }
    case 0xE: case 0xF: backup_write8(g, a, (uint8_t)v); return;
    default:
      gba_write16(g, a, (uint16_t)v);
      gba_write16(g, a + 2u, (uint16_t)(v >> 16));
      return;
  }
}

void gba_bus_reset(hs_gba *g) {
  memset(g->ewram, 0, sizeof g->ewram);
  memset(g->iwram, 0, sizeof g->iwram);
  memset(g->io, 0, sizeof g->io);
  memset(g->pal, 0, sizeof g->pal);
  memset(g->vram, 0, sizeof g->vram);
  memset(g->oam, 0, sizeof g->oam);
  memset(g->tm, 0, sizeof g->tm);
  memset(g->dma, 0, sizeof g->dma);
  memset(&g->flash, 0, sizeof g->flash);
  memset(&g->eeprom, 0, sizeof g->eeprom);
  g->ie = g->ifl = g->ime = g->waitcnt = 0;
  g->in_dma = 0;
  g->dma_stall = 0;
  g->vcount = g->line_cycle = 0;
  g->bgx[0] = g->bgx[1] = g->bgy[0] = g->bgy[1] = 0;
  for (int i = 0; i < 2; i++) fifo_reset(g, i);
  /* affine identity defaults */
  gba_st16(&g->io[0x20], 0x100); gba_st16(&g->io[0x26], 0x100);
  gba_st16(&g->io[0x30], 0x100); gba_st16(&g->io[0x36], 0x100);
  gba_st16(&g->io[0x88], 0x200);
}
