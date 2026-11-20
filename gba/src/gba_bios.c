/* SPDX-License-Identifier: MIT
 * gba_bios.c - clean-room high-level-emulated GBA BIOS (SWI services) plus a
 * six-instruction IRQ dispatch stub, so games run with or without a BIOS file.
 * Behaviour written from the publicly documented SWI interfaces; all
 * algorithms (CORDIC arctan, integer sqrt, decompressors) are original.
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include "gba_internal.h"

#define BIOS_IF_ADDR 0x03007FF8u
#define HLE_CAP_BYTES 0x40000u

/* sin(2*pi*i/256) in 1.14 fixed point (generated, integer table) */
static const int16_t k_sin256[256] = { 0,402,804,1205,1606,2006,2404,2801,3196,3590,3981,4370,4756,5139,5520,5897,6270,6639,7005,7366,7723,8076,8423,8765,9102,9434,9760,10080,10394,10702,11003,11297,11585,11866,12140,12406,12665,12916,13160,13395,13623,13842,14053,14256,14449,14635,14811,14978,15137,15286,15426,15557,15679,15791,15893,15986,16069,16143,16207,16261,16305,16340,16364,16379,16384,16379,16364,16340,16305,16261,16207,16143,16069,15986,15893,15791,15679,15557,15426,15286,15137,14978,14811,14635,14449,14256,14053,13842,13623,13395,13160,12916,12665,12406,12140,11866,11585,11297,11003,10702,10394,10080,9760,9434,9102,8765,8423,8076,7723,7366,7005,6639,6270,5897,5520,5139,4756,4370,3981,3590,3196,2801,2404,2006,1606,1205,804,402,0,-402,-804,-1205,-1606,-2006,-2404,-2801,-3196,-3590,-3981,-4370,-4756,-5139,-5520,-5897,-6270,-6639,-7005,-7366,-7723,-8076,-8423,-8765,-9102,-9434,-9760,-10080,-10394,-10702,-11003,-11297,-11585,-11866,-12140,-12406,-12665,-12916,-13160,-13395,-13623,-13842,-14053,-14256,-14449,-14635,-14811,-14978,-15137,-15286,-15426,-15557,-15679,-15791,-15893,-15986,-16069,-16143,-16207,-16261,-16305,-16340,-16364,-16379,-16384,-16379,-16364,-16340,-16305,-16261,-16207,-16143,-16069,-15986,-15893,-15791,-15679,-15557,-15426,-15286,-15137,-14978,-14811,-14635,-14449,-14256,-14053,-13842,-13623,-13395,-13160,-12916,-12665,-12406,-12140,-11866,-11585,-11297,-11003,-10702,-10394,-10080,-9760,-9434,-9102,-8765,-8423,-8076,-7723,-7366,-7005,-6639,-6270,-5897,-5520,-5139,-4756,-4370,-3981,-3590,-3196,-2801,-2404,-2006,-1606,-1205,-804,-402 };

/* atan(2^-i) in units of 2*pi/65536 */
static const uint16_t k_cordic[16] = {8192, 4836, 2555, 1297, 651, 326, 163, 81,
                                      41,   20,   10,   5,    3,   1,   1,   0};

void gba_hle_install_stub(hs_gba *g) {
  static const uint32_t stub[] = {
    0xE92D500Fu, /* stmfd sp!, {r0-r3, r12, lr} */
    0xE3A00301u, /* mov   r0, #0x04000000        */
    0xE28FE000u, /* add   lr, pc, #0             */
    0xE510F004u, /* ldr   pc, [r0, #-4]  (user handler @ 0x03FFFFFC) */
    0xE8BD500Fu, /* ldmfd sp!, {r0-r3, r12, lr}  */
    0xE25EF004u  /* subs  pc, lr, #4             */
  };
  memset(g->bios, 0, sizeof g->bios);
  for (uint32_t v = 0; v < 8u; v++) gba_st32(&g->bios[v * 4u], 0xEAFFFFFEu); /* b . */
  gba_st32(&g->bios[0x18], 0xEA000042u); /* b 0x128 */
  for (uint32_t i = 0; i < sizeof stub / sizeof stub[0]; i++) gba_st32(&g->bios[0x128u + i * 4u], stub[i]);
}

static uint32_t isqrt32(uint32_t v) {
  uint32_t res = 0, bit = 1u << 30;
  while (bit > v) bit >>= 2;
  for (int i = 0; i < 16 && bit; i++) {
    if (v >= res + bit) { v -= res + bit; res = (res >> 1) + bit; }
    else res >>= 1;
    bit >>= 2;
  }
  return res;
}

int gba_hle_isqrt_selftest(void) {
  uint32_t bad = 0;
  for (uint32_t v = 0; v < 70000u; v += 7u) {
    uint32_t r = isqrt32(v);
    if (r * r > v || (r + 1u) * (r + 1u) <= v) bad++;
  }
  return bad == 0u;
}

static uint32_t cordic_atan2(int32_t x, int32_t y) {
  int64_t X = (int64_t)x * 65536, Y = (int64_t)y * 65536;
  uint32_t ang = 0;
  if (!x && !y) return 0;
  if (X < 0) { X = -X; Y = -Y; ang = 0x8000u; }
  for (int i = 0; i < 16; i++) {
    int64_t nx;
    if (Y > 0) { nx = X + (Y >> i); Y = Y - (X >> i); ang += k_cordic[i]; }
    else { nx = X - (Y >> i); Y = Y + (X >> i); ang -= k_cordic[i]; }
    X = nx;
  }
  return ang & 0xFFFFu;
}

static void out_bytes(hs_gba *g, uint32_t dst, const uint8_t *buf, uint32_t n, int vram) {
  uint32_t i = 0;
  for (; i + 1u < n; i += 2u) gba_write16(g, dst + i, (uint16_t)(buf[i] | (buf[i + 1u] << 8)));
  if (i < n) {
    if (vram) gba_write16(g, dst + i, buf[i]);
    else gba_write8(g, dst + i, buf[i]);
  }
}

static uint32_t hle_lz77(hs_gba *g, uint32_t src, uint32_t dst, int vram) {
  uint32_t size = gba_read32(g, src) >> 8, o = 0;
  if (size > HLE_CAP_BYTES) size = HLE_CAP_BYTES;
  uint8_t *b = g->scratch;
  src += 4u;
  for (uint32_t guard = 0; o < size && guard < HLE_CAP_BYTES; guard++) {
    uint8_t flags = gba_read8(g, src++);
    for (int k = 0; k < 8 && o < size; k++, flags = (uint8_t)(flags << 1)) {
      if (flags & 0x80u) {
        uint8_t b0 = gba_read8(g, src), b1 = gba_read8(g, src + 1u);
        src += 2u;
        uint32_t len = (uint32_t)(b0 >> 4) + 3u, disp = ((uint32_t)(b0 & 0xFu) << 8 | b1) + 1u;
        for (uint32_t j = 0; j < len && o < size; j++, o++) b[o] = (disp <= o) ? b[o - disp] : 0;
      } else {
        b[o++] = gba_read8(g, src++);
      }
    }
  }
  out_bytes(g, dst, b, size, vram);
  return 20u + size * 2u;
}

static uint32_t hle_rl(hs_gba *g, uint32_t src, uint32_t dst, int vram) {
  uint32_t size = gba_read32(g, src) >> 8, o = 0;
  if (size > HLE_CAP_BYTES) size = HLE_CAP_BYTES;
  uint8_t *b = g->scratch;
  src += 4u;
  for (uint32_t guard = 0; o < size && guard < HLE_CAP_BYTES; guard++) {
    uint8_t f = gba_read8(g, src++);
    if (f & 0x80u) {
      uint32_t len = (f & 0x7Fu) + 3u;
      uint8_t v = gba_read8(g, src++);
      for (uint32_t j = 0; j < len && o < size; j++) b[o++] = v;
    } else {
      uint32_t len = (f & 0x7Fu) + 1u;
      for (uint32_t j = 0; j < len && o < size; j++) b[o++] = gba_read8(g, src++);
    }
  }
  out_bytes(g, dst, b, size, vram);
  return 20u + size * 2u;
}

static uint32_t hle_huff(hs_gba *g, uint32_t src, uint32_t dst) {
  uint32_t hdr = gba_read32(g, src);
  uint32_t bits = hdr & 0xFu, size = hdr >> 8;
  if (bits != 4u && bits != 8u) bits = 8u;
  if (size > HLE_CAP_BYTES) size = HLE_CAP_BYTES;
  uint32_t root = src + 5u;
  uint32_t data = src + 4u + ((uint32_t)gba_read8(g, src + 4u) + 1u) * 2u;
  uint32_t node_addr = root, out = 0, outbits = 0, written = 0;
  uint8_t node = gba_read8(g, root);
  for (uint32_t guard = 0; written < size && guard < HLE_CAP_BYTES; guard++) {
    uint32_t word = gba_read32(g, data);
    data += 4u;
    for (int bit = 31; bit >= 0 && written < size; bit--) {
      uint32_t b = (word >> bit) & 1u;
      uint32_t child = (node_addr & ~1u) + (uint32_t)(node & 0x3Fu) * 2u + 2u + b;
      int leaf = (node & (b ? 0x40u : 0x80u)) != 0;
      node_addr = child;
      node = gba_read8(g, child);
      if (leaf) {
        out |= (uint32_t)(node & ((1u << bits) - 1u)) << outbits;
        outbits += bits;
        if (outbits == 32u) {
          gba_write32(g, dst + written, out);
          written += 4u;
          out = 0;
          outbits = 0;
        }
        node_addr = root;
        node = gba_read8(g, root);
      }
    }
  }
  return 20u + size * 4u;
}

static uint32_t hle_bitunpack(hs_gba *g, uint32_t src, uint32_t dst, uint32_t info) {
  uint32_t len = gba_read16(g, info), sw = gba_read8(g, info + 2u), dw = gba_read8(g, info + 3u);
  uint32_t ofs = gba_read32(g, info + 4u);
  uint32_t zero_flag = ofs >> 31;
  ofs &= 0x7FFFFFFFu;
  if (!sw || sw > 8u || (sw & (sw - 1u))) return 10;
  if (!dw || dw > 32u || (dw & (dw - 1u))) return 10;
  uint32_t smask = (1u << sw) - 1u, out = 0, outbits = 0;
  if (len > HLE_CAP_BYTES) len = HLE_CAP_BYTES;
  for (uint32_t i = 0; i < len; i++) {
    uint8_t byte = gba_read8(g, src + i);
    for (uint32_t pos = 0; pos < 8u; pos += sw) {
      uint32_t v = (byte >> pos) & smask;
      if (v || zero_flag) v += ofs;
      out |= (dw == 32u) ? v : ((v & ((1u << dw) - 1u)) << outbits);
      outbits += dw;
      if (outbits >= 32u) { gba_write32(g, dst, out); dst += 4u; out = 0; outbits = 0; }
    }
  }
  return 20u + len * 8u;
}

static uint32_t hle_cpuset(hs_gba *g, uint32_t src, uint32_t dst, uint32_t ctl, int fast) {
  uint32_t n = ctl & 0x1FFFFFu;
  int fill = (ctl >> 24) & 1u, wide = fast || ((ctl >> 26) & 1u);
  if (fast) n = (n + 7u) & ~7u;
  if (n > 0x10000u) n = 0x10000u;
  if (wide) {
    src &= ~3u; dst &= ~3u;
    uint32_t v = gba_read32(g, src);
    for (uint32_t i = 0; i < n; i++) {
      if (!fill) v = gba_read32(g, src + i * 4u);
      gba_write32(g, dst + i * 4u, v);
    }
  } else {
    src &= ~1u; dst &= ~1u;
    uint16_t v = gba_read16(g, src);
    for (uint32_t i = 0; i < n; i++) {
      if (!fill) v = gba_read16(g, src + i * 2u);
      gba_write16(g, dst + i * 2u, v);
    }
  }
  return 10u + n * (fast ? 1u : 2u);
}

static void hle_bg_affine(hs_gba *g, uint32_t src, uint32_t dst, uint32_t n) {
  if (n > 256u) n = 256u;
  for (uint32_t i = 0; i < n; i++, src += 20u, dst += 16u) {
    int32_t ox = (int32_t)gba_read32(g, src), oy = (int32_t)gba_read32(g, src + 4u);
    int32_t dx = (int16_t)gba_read16(g, src + 8u), dy = (int16_t)gba_read16(g, src + 10u);
    int32_t sx = (int16_t)gba_read16(g, src + 12u), sy = (int16_t)gba_read16(g, src + 14u);
    uint32_t th = gba_read16(g, src + 16u) >> 8;
    int32_t sn = k_sin256[th & 255u], cs = k_sin256[(th + 64u) & 255u];
    int32_t pa = (cs * sx) >> 14, pb = -((sn * sx) >> 14), pc = (sn * sy) >> 14, pd = (cs * sy) >> 14;
    gba_write16(g, dst, (uint16_t)pa);
    gba_write16(g, dst + 2u, (uint16_t)pb);
    gba_write16(g, dst + 4u, (uint16_t)pc);
    gba_write16(g, dst + 6u, (uint16_t)pd);
    gba_write32(g, dst + 8u, (uint32_t)(ox - (pa * dx + pb * dy)));
    gba_write32(g, dst + 12u, (uint32_t)(oy - (pc * dx + pd * dy)));
  }
}

static void hle_obj_affine(hs_gba *g, uint32_t src, uint32_t dst, uint32_t n, uint32_t stride) {
  if (n > 128u) n = 128u;
  for (uint32_t i = 0; i < n; i++, src += 8u, dst += stride * 4u) {
    int32_t sx = (int16_t)gba_read16(g, src), sy = (int16_t)gba_read16(g, src + 2u);
    uint32_t th = gba_read16(g, src + 4u) >> 8;
    int32_t sn = k_sin256[th & 255u], cs = k_sin256[(th + 64u) & 255u];
    gba_write16(g, dst, (uint16_t)((cs * sx) >> 14));
    gba_write16(g, dst + stride, (uint16_t)(-((sn * sx) >> 14)));
    gba_write16(g, dst + stride * 2u, (uint16_t)((sn * sy) >> 14));
    gba_write16(g, dst + stride * 3u, (uint16_t)((cs * sy) >> 14));
  }
}

static void hle_diff(hs_gba *g, uint32_t src, uint32_t dst, int wide, int vram) {
  uint32_t size = gba_read32(g, src) >> 8;
  if (size > HLE_CAP_BYTES) size = HLE_CAP_BYTES;
  src += 4u;
  if (wide) {
    uint16_t acc = 0;
    for (uint32_t i = 0; i + 1u < size; i += 2u) {
      acc = (uint16_t)(acc + gba_read16(g, src + i));
      gba_write16(g, dst + i, acc);
    }
    return;
  }
  uint8_t acc = 0;
  for (uint32_t i = 0; i < size; i++) {
    acc = (uint8_t)(acc + gba_read8(g, src + i));
    g->scratch[i] = acc;
  }
  out_bytes(g, dst, g->scratch, size, vram);
}

static void hle_ram_reset(hs_gba *g, uint32_t f) {
  gba_write16(g, 0x04000000u, 0x0080u);
  if (f & 0x01u) memset(g->ewram, 0, sizeof g->ewram);
  if (f & 0x02u) memset(g->iwram, 0, 0x7E00u);
  if (f & 0x04u) memset(g->pal, 0, sizeof g->pal);
  if (f & 0x08u) memset(g->vram, 0, sizeof g->vram);
  if (f & 0x10u) memset(g->oam, 0, sizeof g->oam);
  if (f & 0x40u) for (uint32_t a = 0x60u; a < 0xA0u; a += 2u) gba_write16(g, 0x04000000u + a, 0);
  if (f & 0x80u) {
    for (uint32_t a = 0x08u; a < 0x60u; a += 2u) gba_write16(g, 0x04000000u + a, 0);
    gba_write16(g, 0x04000020u, 0x100); gba_write16(g, 0x04000026u, 0x100);
    gba_write16(g, 0x04000030u, 0x100); gba_write16(g, 0x04000036u, 0x100);
  }
}

uint32_t gba_hle_swi(hs_gba *g, uint32_t num) {
  gba_cpu *c = &g->cpu;
  uint32_t *r = c->r;
  switch (num) {
    case 0x00: { /* SoftReset */
      uint8_t flag = g->iwram[0x7FFA];
      memset(&g->iwram[0x7E00], 0, 0x200u);
      gba_cpu_reset(g, 1);
      c->pc_next = flag ? 0x02000000u : 0x08000000u;
      c->branched = 1;
      return 20;
    }
    case 0x01: hle_ram_reset(g, r[0]); return 200;
    case 0x02: case 0x03: c->halted = 1; return 4;
    case 0x04: case 0x05: {
      if (num == 0x05u) { r[0] = 1; r[1] = 1; }
      g->ime = 1;
      uint16_t want = (uint16_t)r[1];
      if (!c->intrwait_active) {
        if (r[0] & 1u) gba_write16(g, BIOS_IF_ADDR, (uint16_t)(gba_read16(g, BIOS_IF_ADDR) & ~want));
        c->intrwait_active = 1;
      }
      uint16_t have = gba_read16(g, BIOS_IF_ADDR);
      if (have & want) {
        gba_write16(g, BIOS_IF_ADDR, (uint16_t)(have & ~want));
        c->intrwait_active = 0;
        return 8;
      }
      c->pc_next -= (c->cpsr & CPSR_T) ? 2u : 4u; /* re-issue this SWI after the IRQ */
      c->halted = 1;
      return 4;
    }
    case 0x06: case 0x07: {
      int64_t n = (int32_t)(num == 6u ? r[0] : r[1]), d = (int32_t)(num == 6u ? r[1] : r[0]);
      if (d == 0) { r[0] = n < 0 ? 0xFFFFFFFFu : 1u; r[1] = (uint32_t)n; r[3] = 1; return 40; }
      int64_t q = n / d, m = n % d;
      r[0] = (uint32_t)q;
      r[1] = (uint32_t)m;
      r[3] = (uint32_t)(q < 0 ? -q : q);
      return 40;
    }
    case 0x08: r[0] = isqrt32(r[0]); return 30;
    case 0x09: {
      uint32_t a = cordic_atan2(0x4000, (int16_t)r[0]);
      r[0] = (uint32_t)(int32_t)(int16_t)a;
      return 40;
    }
    case 0x0A: r[0] = cordic_atan2((int16_t)r[0], (int16_t)r[1]); return 40;
    case 0x0B: return hle_cpuset(g, r[0], r[1], r[2], 0);
    case 0x0C: return hle_cpuset(g, r[0], r[1], r[2], 1);
    case 0x0D: r[0] = 0xBAAE187Fu; return 10;
    case 0x0E: hle_bg_affine(g, r[0], r[1], r[2]); return 40u + r[2] * 20u;
    case 0x0F: hle_obj_affine(g, r[0], r[1], r[2], r[3]); return 40u + r[2] * 20u;
    case 0x10: return hle_bitunpack(g, r[0], r[1], r[2]);
    case 0x11: return hle_lz77(g, r[0], r[1], 0);
    case 0x12: return hle_lz77(g, r[0], r[1], 1);
    case 0x13: return hle_huff(g, r[0], r[1]);
    case 0x14: return hle_rl(g, r[0], r[1], 0);
    case 0x15: return hle_rl(g, r[0], r[1], 1);
    case 0x16: hle_diff(g, r[0], r[1], 0, 0); return 40;
    case 0x17: hle_diff(g, r[0], r[1], 0, 1); return 40;
    case 0x18: hle_diff(g, r[0], r[1], 1, 1); return 40;
    default: return 10; /* sound-driver / multiboot SWIs: no-op */
  }
}
