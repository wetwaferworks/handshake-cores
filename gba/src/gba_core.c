/* SPDX-License-Identifier: MIT
 * gba_core.c - hs-gba lifecycle + the three-phase zero-mutex frame contract.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include <stdlib.h>
#include "gba_internal.h"

hs_gba *hs_gba_create(void) {
  hs_gba *g = (hs_gba *)calloc(1, sizeof(hs_gba));
  if (!g) return NULL;
  gba_cpu_init_tables(&g->cpu);
  gba_video_init(&g->video);
  hs_gba_reset(g);
  return g;
}

void hs_gba_destroy(hs_gba *g) {
  if (!g) return;
  free(g->rom);
  free(g);
}

int hs_gba_load_bios(hs_gba *g, const uint8_t *bios, size_t len) {
  if (!g || !bios || len != 0x4000u) return -1;
  memcpy(g->bios, bios, 0x4000u);
  g->has_bios = 1;
  hs_gba_reset(g);
  return 0;
}

int hs_gba_load_rom(hs_gba *g, const uint8_t *rom, size_t len) {
  if (!g || !rom || len == 0u || len > GBA_ROM_MAX) return -1;
  uint8_t *copy = (uint8_t *)malloc(len + 4u);
  if (!copy) return -1;
  memcpy(copy, rom, len);
  memset(copy + len, 0, 4u);
  free(g->rom);
  g->rom = copy;
  g->rom_size = (uint32_t)len;
  gba_detect_save(g);
  memset(g->save, 0xFF, GBA_SAVE_MAX);
  if (g->save_type == HS_GBA_SAVE_SRAM) memset(g->save, 0, GBA_SAVE_MAX);
  g->save_dirty = 0;
  hs_gba_reset(g);
  return 0;
}

void hs_gba_reset(hs_gba *g) {
  gba_bus_reset(g);
  if (!g->has_bios) gba_hle_install_stub(g);
  gba_cpu_reset(g, !g->has_bios);
  gba_apu_reset(&g->apu);
  g->snap[0].n_ev = g->snap[1].n_ev = 0;
  g->snap[0].frame_no = g->snap[1].frame_no = 0;
  g->snap_w = 0;
  g->snap_pub = 1;
  g->frame_count = 0;
  g->ev_dropped = 0;
  g->keys = 0;
}

hs_gba_save_type hs_gba_save_kind(const hs_gba *g) { return g->save_type; }

const uint8_t *hs_gba_save_data(const hs_gba *g, size_t *len) {
  size_t n = 0;
  switch (g->save_type) {
    case HS_GBA_SAVE_SRAM: n = 0x8000u; break;
    case HS_GBA_SAVE_FLASH64: n = 0x10000u; break;
    case HS_GBA_SAVE_FLASH128: n = 0x20000u; break;
    case HS_GBA_SAVE_EEPROM: n = (g->eeprom.addr_bits == 6) ? 0x200u : 0x2000u; break;
    default: n = 0; break;
  }
  if (len) *len = n;
  return n ? g->save : NULL;
}

int hs_gba_load_save(hs_gba *g, const uint8_t *data, size_t len) {
  if (!g || !data || len == 0u || len > GBA_SAVE_MAX) return -1;
  memcpy(g->save, data, len);
  if (g->save_type == HS_GBA_SAVE_EEPROM) g->eeprom.addr_bits = (len <= 0x200u) ? 6 : 14;
  g->save_dirty = 0;
  return 0;
}

int hs_gba_save_dirty(const hs_gba *g) { return g->save_dirty; }
void hs_gba_save_clear_dirty(hs_gba *g) { g->save_dirty = 0; }

/* Run the CPU (plus timers / DMA stalls) until line_cycle reaches 'until'. */
static void run_until(hs_gba *g, uint32_t until) {
  for (uint32_t guard = 0; g->line_cycle < until && guard < GBA_CYC_LINE * 2u; guard++) {
    gba_cpu_irq_check(g);
    uint32_t cyc;
    if (g->cpu.halted) {
      cyc = until - g->line_cycle;
      if (cyc > 64u) cyc = 64u;
    } else {
      cyc = gba_cpu_step(g);
    }
    cyc += g->dma_stall;
    g->dma_stall = 0;
    gba_timers_advance(g, cyc);
    g->line_cycle += cyc;
  }
}

static void check_keypad_irq(hs_gba *g) {
  uint16_t kc = gba_ld16(&g->io[0x132]);
  if (!(kc & 0x4000u)) return;
  uint16_t sel = kc & 0x3FFu, hit = g->keys & sel;
  if ((kc & 0x8000u) ? (hit == sel && sel) : (hit != 0u)) gba_raise_irq(g, IRQ_KEYPAD);
}

void hs_gba_step_cpu_phase(hs_gba *g, uint16_t keys_pressed) {
  gba_snapshot *s = &g->snap[g->snap_w];
  g->keys = keys_pressed & 0x3FFu;
  s->n_ev = 0;
  s->frame_no = g->frame_count;
  check_keypad_irq(g);
  for (uint32_t line = 0; line < GBA_LINES; line++) {
    g->vcount = line;
    g->line_cycle = g->line_cycle >= GBA_CYC_LINE ? g->line_cycle - GBA_CYC_LINE : 0u;
    if (g->line_cycle >= GBA_CYC_LINE) g->line_cycle = 0;
    uint16_t dispstat = gba_ld16(&g->io[4]);
    if (line == (uint32_t)(dispstat >> 8) && (dispstat & 0x20u)) gba_raise_irq(g, IRQ_VCOUNT);
    if (line == GBA_VIS_LINES) {
      memcpy(s->pal, g->pal, sizeof s->pal);
      memcpy(s->vram, g->vram, sizeof s->vram);
      memcpy(s->oam, g->oam, sizeof s->oam);
      if (dispstat & 0x08u) gba_raise_irq(g, IRQ_VBLANK);
      gba_dma_trigger(g, 1);
      g->bgx[0] = ((int32_t)(gba_ld32(&g->io[0x28]) << 4)) >> 4;
      g->bgy[0] = ((int32_t)(gba_ld32(&g->io[0x2C]) << 4)) >> 4;
      g->bgx[1] = ((int32_t)(gba_ld32(&g->io[0x38]) << 4)) >> 4;
      g->bgy[1] = ((int32_t)(gba_ld32(&g->io[0x3C]) << 4)) >> 4;
    }
    if (line < GBA_VIS_LINES) {
      gba_line_regs *lr = &s->line[line];
      memcpy(lr->io, g->io, GBA_LINE_IO);
      lr->bgx[0] = g->bgx[0]; lr->bgx[1] = g->bgx[1];
      lr->bgy[0] = g->bgy[0]; lr->bgy[1] = g->bgy[1];
    }
    run_until(g, GBA_CYC_HDRAW);
    if (gba_ld16(&g->io[4]) & 0x10u) gba_raise_irq(g, IRQ_HBLANK);
    if (line < GBA_VIS_LINES) {
      gba_dma_trigger(g, 2);
      g->bgx[0] += (int16_t)gba_ld16(&g->io[0x22]);
      g->bgy[0] += (int16_t)gba_ld16(&g->io[0x26]);
      g->bgx[1] += (int16_t)gba_ld16(&g->io[0x32]);
      g->bgy[1] += (int16_t)gba_ld16(&g->io[0x36]);
    }
    run_until(g, GBA_CYC_LINE);
  }
  g->frame_count++;
  g->snap_pub = g->snap_w; /* publish: consumers only ever read snap[snap_pub] */
  g->snap_w ^= 1;
}

void hs_gba_render_video_phase(hs_gba *g, uint32_t *out640x480) {
  const gba_snapshot *s = &g->snap[g->snap_pub];
  gba_ppu_render(&g->video, s);
  if (out640x480) gba_upscale_640x480(&g->video, out640x480);
}

int hs_gba_synth_audio_phase(hs_gba *g, int16_t *out, int max_frames) {
  return gba_apu_synth(&g->apu, &g->snap[g->snap_pub], out, max_frames);
}

uint64_t hs_gba_frame_hash(const hs_gba *g) { return g->video.hash; }
const uint16_t *hs_gba_native_frame(const hs_gba *g) { return g->video.fb; }
uint64_t hs_gba_frame_count(const hs_gba *g) { return g->frame_count; }
uint32_t hs_gba_audio_events_dropped(const hs_gba *g) { return g->ev_dropped; }
