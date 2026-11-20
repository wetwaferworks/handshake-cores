/* SPDX-License-Identifier: MIT
 * gba_ppu.c - clean-room scanline-snapshot GBA PPU (modes 0-5, 128 regular and
 * affine sprites, windows, alpha/brightness blending) rendered read-only from an
 * immutable frame snapshot, plus the integer 240x160 -> 640x480 edge-to-edge
 * sharp-bilinear upsampler (3x exact vertical, 8/3 horizontal, zero floats).
 * Copyright (c) 2026 HandshakeOS contributors.
 */
#include "gba_internal.h"

#define W HS_GBA_SCREEN_W
#define OPAQUE 0x8000u

typedef struct {
  uint16_t bg[4][W];
  uint16_t obj[W];
  uint8_t objp[W];
  uint8_t objsemi[W];
  uint8_t objwin[W];
} line_bufs;

static inline uint16_t pal16(const uint8_t *pal, uint32_t idx) { return gba_ld16(&pal[(idx & 0x1FFu) * 2u]) & 0x7FFFu; }

static void render_text_bg(const gba_snapshot *s, const uint8_t *io, int i, uint32_t l, uint16_t *out) {
  uint16_t cnt = gba_ld16(&io[8 + 2 * i]);
  uint32_t hofs = gba_ld16(&io[0x10 + 4 * i]) & 0x1FFu, vofs = gba_ld16(&io[0x12 + 4 * i]) & 0x1FFu;
  uint32_t cb = ((cnt >> 2) & 3u) * 0x4000u, sb = ((cnt >> 8) & 0x1Fu) * 0x800u, size = (cnt >> 14) & 3u;
  int bpp8 = (cnt & 0x80u) != 0;
  uint32_t wmask = (size & 1u) ? 511u : 255u, hmask = (size & 2u) ? 511u : 255u;
  uint32_t py = (l + vofs) & hmask;
  uint32_t ty = py >> 3, yblk = 0;
  if (ty >= 32u) { ty -= 32u; yblk = (size == 3u) ? 2u : 1u; }
  for (uint32_t x = 0; x < W; x++) {
    uint32_t px = (x + hofs) & wmask, tx = px >> 3, blk = yblk;
    if (tx >= 32u) { tx -= 32u; blk += 1u; }
    uint32_t moff = sb + blk * 0x800u + (ty * 32u + tx) * 2u;
    if (moff + 1u >= 0x10000u) { out[x] = 0; continue; }
    uint16_t e = gba_ld16(&s->vram[moff]);
    uint32_t tile = e & 0x3FFu, ix = px & 7u, iy = py & 7u;
    if (e & 0x400u) ix = 7u - ix;
    if (e & 0x800u) iy = 7u - iy;
    uint32_t c;
    if (bpp8) {
      uint32_t a = cb + tile * 64u + iy * 8u + ix;
      if (a >= 0x10000u) { out[x] = 0; continue; }
      c = s->vram[a];
      out[x] = c ? (uint16_t)(pal16(s->pal, c) | OPAQUE) : 0;
    } else {
      uint32_t a = cb + tile * 32u + iy * 4u + (ix >> 1);
      if (a >= 0x10000u) { out[x] = 0; continue; }
      c = (s->vram[a] >> ((ix & 1u) * 4u)) & 15u;
      out[x] = c ? (uint16_t)(pal16(s->pal, (e >> 12) * 16u + c) | OPAQUE) : 0;
    }
  }
}

static void render_affine_bg(const gba_snapshot *s, const uint8_t *io, int i, uint32_t l, uint16_t *out) {
  int idx = i - 2;
  uint16_t cnt = gba_ld16(&io[8 + 2 * i]);
  uint32_t cb = ((cnt >> 2) & 3u) * 0x4000u, sb = ((cnt >> 8) & 0x1Fu) * 0x800u;
  int32_t size = 128 << ((cnt >> 14) & 3u);
  int wrap = (cnt & 0x2000u) != 0;
  int32_t pa = (int16_t)gba_ld16(&io[0x20 + 0x10 * idx]), pc = (int16_t)gba_ld16(&io[0x24 + 0x10 * idx]);
  int32_t rx = s->line[l].bgx[idx], ry = s->line[l].bgy[idx];
  uint32_t tiles_w = (uint32_t)size >> 3;
  for (uint32_t x = 0; x < W; x++, rx += pa, ry += pc) {
    int32_t px = rx >> 8, py = ry >> 8;
    if (wrap) { px &= size - 1; py &= size - 1; }
    else if (px < 0 || py < 0 || px >= size || py >= size) { out[x] = 0; continue; }
    uint32_t m = sb + ((uint32_t)py >> 3) * tiles_w + ((uint32_t)px >> 3);
    if (m >= 0x10000u) { out[x] = 0; continue; }
    uint32_t a = cb + (uint32_t)s->vram[m] * 64u + ((uint32_t)py & 7u) * 8u + ((uint32_t)px & 7u);
    if (a >= 0x10000u) { out[x] = 0; continue; }
    uint32_t c = s->vram[a];
    out[x] = c ? (uint16_t)(pal16(s->pal, c) | OPAQUE) : 0;
  }
}

static void render_bitmap(const gba_snapshot *s, const uint8_t *io, uint32_t mode, uint32_t l, uint16_t *out) {
  uint16_t dispcnt = gba_ld16(&io[0]);
  int32_t pa = (int16_t)gba_ld16(&io[0x20]), pc = (int16_t)gba_ld16(&io[0x24]);
  int32_t rx = s->line[l].bgx[0], ry = s->line[l].bgy[0];
  uint32_t page = (dispcnt & 0x10u) ? 0xA000u : 0u;
  int32_t bw = (mode == 5u) ? 160 : 240, bh = (mode == 5u) ? 128 : 160;
  for (uint32_t x = 0; x < W; x++, rx += pa, ry += pc) {
    int32_t px = rx >> 8, py = ry >> 8;
    if (px < 0 || py < 0 || px >= bw || py >= bh) { out[x] = 0; continue; }
    uint32_t o = (uint32_t)(py * bw + px);
    if (mode == 3u) out[x] = (uint16_t)((gba_ld16(&s->vram[o * 2u]) & 0x7FFFu) | OPAQUE);
    else if (mode == 5u) out[x] = (uint16_t)((gba_ld16(&s->vram[page + o * 2u]) & 0x7FFFu) | OPAQUE);
    else {
      uint32_t c = s->vram[page + o];
      out[x] = c ? (uint16_t)(pal16(s->pal, c) | OPAQUE) : 0;
    }
  }
}

static const uint8_t k_obj_w[3][4] = {{8, 16, 32, 64}, {16, 32, 32, 64}, {8, 8, 16, 32}};
static const uint8_t k_obj_h[3][4] = {{8, 16, 32, 64}, {8, 8, 16, 32}, {16, 32, 32, 64}};

static void render_objs(const gba_snapshot *s, const uint8_t *io, uint32_t l, line_bufs *lb) {
  uint16_t dispcnt = gba_ld16(&io[0]);
  int map1d = (dispcnt & 0x40u) != 0, bitmap = (dispcnt & 7u) >= 3u;
  for (int i = 0; i < 128; i++) {
    const uint8_t *o = &s->oam[i * 8];
    uint16_t a0 = gba_ld16(o), a1 = gba_ld16(o + 2), a2 = gba_ld16(o + 4);
    int affine = (a0 & 0x100u) != 0;
    if (!affine && (a0 & 0x200u)) continue;
    uint32_t omode = (a0 >> 10) & 3u, shape = (a0 >> 14) & 3u, sz = (a1 >> 14) & 3u;
    if (omode == 3u || shape == 3u) continue;
    int32_t w = k_obj_w[shape][sz], h = k_obj_h[shape][sz];
    int dbl = affine && (a0 & 0x200u);
    int32_t bw = dbl ? 2 * w : w, bh = dbl ? 2 * h : h;
    int32_t y = a0 & 0xFFu, x = a1 & 0x1FFu;
    if (x & 0x100) x -= 512;
    int32_t dy = (int32_t)((l - (uint32_t)y) & 0xFFu);
    if (dy >= bh) continue;
    int bpp8 = (a0 & 0x2000u) != 0;
    uint32_t tile = a2 & 0x3FFu, prio = (a2 >> 10) & 3u, pbank = a2 >> 12;
    if (bitmap && tile < 512u) continue;
    int32_t pa = 0x100, pb = 0, pc = 0, pd = 0x100;
    if (affine) {
      const uint8_t *p = &s->oam[((a1 >> 9) & 0x1Fu) * 32u];
      pa = (int16_t)gba_ld16(p + 6); pb = (int16_t)gba_ld16(p + 14);
      pc = (int16_t)gba_ld16(p + 22); pd = (int16_t)gba_ld16(p + 30);
    }
    uint32_t row_tiles = map1d ? (uint32_t)(w >> 3) * (bpp8 ? 2u : 1u) : 32u;
    for (int32_t sx = 0; sx < bw; sx++) {
      int32_t X = x + sx;
      if (X < 0 || X >= W) continue;
      int32_t tx, ty;
      if (affine) {
        int32_t cx = sx - bw / 2, cy = dy - bh / 2;
        tx = ((pa * cx + pb * cy) >> 8) + w / 2;
        ty = ((pc * cx + pd * cy) >> 8) + h / 2;
        if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;
      } else {
        tx = (a1 & 0x1000u) ? w - 1 - sx : sx;
        ty = (a1 & 0x2000u) ? h - 1 - dy : dy;
      }
      uint32_t c, col;
      if (bpp8) {
        uint32_t tno = tile + ((uint32_t)ty >> 3) * row_tiles + ((uint32_t)tx >> 3) * 2u;
        c = s->vram[0x10000u + (tno & 0x3FFu) * 32u + ((uint32_t)ty & 7u) * 8u + ((uint32_t)tx & 7u)];
        col = pal16(s->pal, 0x100u + c);
      } else {
        uint32_t tno = tile + ((uint32_t)ty >> 3) * row_tiles + ((uint32_t)tx >> 3);
        uint8_t b = s->vram[0x10000u + (tno & 0x3FFu) * 32u + ((uint32_t)ty & 7u) * 4u + (((uint32_t)tx & 7u) >> 1)];
        c = (b >> (((uint32_t)tx & 1u) * 4u)) & 15u;
        col = pal16(s->pal, 0x100u + pbank * 16u + c);
      }
      if (!c) continue;
      if (omode == 2u) { lb->objwin[X] = 1; continue; }
      if ((lb->obj[X] & OPAQUE) && lb->objp[X] <= prio) continue;
      lb->obj[X] = (uint16_t)(col | OPAQUE);
      lb->objp[X] = (uint8_t)prio;
      lb->objsemi[X] = omode == 1u;
    }
  }
}

static inline int in_range(uint32_t v, uint32_t lo, uint32_t hi) {
  return lo <= hi ? (v >= lo && v < hi) : (v >= lo || v < hi);
}

static inline uint16_t blend_alpha(uint16_t a, uint16_t b, uint32_t eva, uint32_t evb) {
  uint32_t r = ((a & 31u) * eva + (b & 31u) * evb) >> 4;
  uint32_t g = (((a >> 5) & 31u) * eva + ((b >> 5) & 31u) * evb) >> 4;
  uint32_t bl = (((a >> 10) & 31u) * eva + ((b >> 10) & 31u) * evb) >> 4;
  if (r > 31u) r = 31u;
  if (g > 31u) g = 31u;
  if (bl > 31u) bl = 31u;
  return (uint16_t)(r | (g << 5) | (bl << 10));
}

static inline uint16_t blend_bright(uint16_t a, uint32_t evy, int up) {
  uint32_t r = a & 31u, g = (a >> 5) & 31u, b = (a >> 10) & 31u;
  if (up) { r += ((31u - r) * evy) >> 4; g += ((31u - g) * evy) >> 4; b += ((31u - b) * evy) >> 4; }
  else { r -= (r * evy) >> 4; g -= (g * evy) >> 4; b -= (b * evy) >> 4; }
  return (uint16_t)(r | (g << 5) | (b << 10));
}

static void render_line(gba_video *v, const gba_snapshot *s, uint32_t l) {
  const uint8_t *io = s->line[l].io;
  uint16_t dispcnt = gba_ld16(&io[0]);
  uint16_t *dst = &v->fb[l * W];
  if (dispcnt & 0x80u) { for (uint32_t x = 0; x < W; x++) dst[x] = 0x7FFF; return; }
  uint32_t mode = dispcnt & 7u;
  line_bufs lb;
  memset(&lb, 0, sizeof lb);
  uint32_t bgmask = 0;
  switch (mode) {
    case 0: bgmask = 0xFu; break;
    case 1: bgmask = 0x7u; break;
    case 2: bgmask = 0xCu; break;
    case 3: case 4: case 5: bgmask = 0x4u; break;
    default: bgmask = 0; break;
  }
  bgmask &= (dispcnt >> 8) & 0xFu;
  for (int i = 0; i < 4; i++) {
    if (!(bgmask & (1u << i))) continue;
    if (mode >= 3u) render_bitmap(s, io, mode, l, lb.bg[i]);
    else if (mode == 0u || (mode == 1u && i < 2)) render_text_bg(s, io, i, l, lb.bg[i]);
    else render_affine_bg(s, io, i, l, lb.bg[i]);
  }
  for (uint32_t x = 0; x < W; x++) lb.objp[x] = 4;
  if (dispcnt & 0x1000u) render_objs(s, io, l, &lb);

  uint32_t bgprio[4];
  for (int i = 0; i < 4; i++) bgprio[i] = gba_ld16(&io[8 + 2 * i]) & 3u;
  uint16_t bldcnt = gba_ld16(&io[0x50]), bldalpha = gba_ld16(&io[0x52]), bldy = gba_ld16(&io[0x54]);
  uint32_t bmode = (bldcnt >> 6) & 3u;
  uint32_t eva = bldalpha & 0x1Fu, evb = (bldalpha >> 8) & 0x1Fu, evy = bldy & 0x1Fu;
  if (eva > 16u) eva = 16u;
  if (evb > 16u) evb = 16u;
  if (evy > 16u) evy = 16u;
  uint16_t backdrop = pal16(s->pal, 0);
  int win_any = (dispcnt & 0xE000u) != 0;
  uint16_t winin = gba_ld16(&io[0x48]), winout = gba_ld16(&io[0x4A]);
  int in_w0y = in_range(l, io[0x45], io[0x44]), in_w1y = in_range(l, io[0x47], io[0x46]);

  for (uint32_t x = 0; x < W; x++) {
    uint32_t mask = 0x3Fu;
    if (win_any) {
      if ((dispcnt & 0x2000u) && in_w0y && in_range(x, io[0x41], io[0x40])) mask = winin & 0x3Fu;
      else if ((dispcnt & 0x4000u) && in_w1y && in_range(x, io[0x43], io[0x42])) mask = (winin >> 8) & 0x3Fu;
      else if ((dispcnt & 0x8000u) && lb.objwin[x]) mask = (winout >> 8) & 0x3Fu;
      else mask = winout & 0x3Fu;
    }
    /* find top two layers: rank = prio*8 + (obj ? 0 : 1 + bg) */
    uint32_t r1 = 0xFFu, r2 = 0xFFu, l1 = 5, l2 = 5;
    uint16_t c1 = backdrop, c2 = backdrop;
    if ((lb.obj[x] & OPAQUE) && (mask & 0x10u)) { r1 = lb.objp[x] * 8u; l1 = 4; c1 = lb.obj[x] & 0x7FFFu; }
    for (uint32_t i = 0; i < 4; i++) {
      if (!(lb.bg[i][x] & OPAQUE) || !(mask & (1u << i))) continue;
      uint32_t rk = bgprio[i] * 8u + 1u + i;
      uint16_t c = lb.bg[i][x] & 0x7FFFu;
      if (rk < r1) { r2 = r1; l2 = l1; c2 = c1; r1 = rk; l1 = i; c1 = c; }
      else if (rk < r2) { r2 = rk; l2 = i; c2 = c; }
    }
    uint16_t out = c1;
    int top1 = (bldcnt >> l1) & 1u, bot2 = (bldcnt >> (8u + l2)) & 1u;
    if (l1 == 4u && lb.objsemi[x] && bot2) out = blend_alpha(c1, c2, eva, evb);
    else if (mask & 0x20u) {
      if (bmode == 1u && top1 && bot2) out = blend_alpha(c1, c2, eva, evb);
      else if (bmode == 2u && top1) out = blend_bright(c1, evy, 1);
      else if (bmode == 3u && top1) out = blend_bright(c1, evy, 0);
    }
    dst[x] = out;
  }
}

void gba_video_init(gba_video *v) {
  memset(v, 0, sizeof *v);
  /* Sharp-bilinear: nearest 2x prescale to 480 virtual columns, then linear
   * 480 -> 640. Positions in 1/256 virtual pixels: u = (2x+1)*96 - 128. */
  for (uint32_t x = 0; x < HS_GBA_OUT_W; x++) {
    int32_t u = (int32_t)(2u * x + 1u) * 96 - 128;
    if (u < 0) u = 0;
    int32_t vi = u >> 8, fr = u & 255;
    int32_t v1 = vi + 1 > 479 ? 479 : vi + 1;
    v->lut_src0[x] = (uint16_t)(vi >> 1);
    v->lut_src1[x] = (uint16_t)(v1 >> 1);
    v->lut_w[x] = (uint16_t)((v->lut_src0[x] == v->lut_src1[x]) ? 0 : fr);
  }
}

void gba_ppu_render(gba_video *v, const gba_snapshot *s) {
  for (uint32_t l = 0; l < GBA_VIS_LINES; l++) render_line(v, s, l);
  uint64_t h = 0xcbf29ce484222325ull;
  for (uint32_t i = 0; i < W * HS_GBA_SCREEN_H; i++) {
    h ^= v->fb[i] & 0xFFu; h *= 0x100000001b3ull;
    h ^= v->fb[i] >> 8; h *= 0x100000001b3ull;
  }
  v->hash = h;
}

void gba_upscale_640x480(const gba_video *v, uint32_t *out) {
  uint32_t rgb[W];
  for (uint32_t y = 0; y < HS_GBA_SCREEN_H; y++) {
    const uint16_t *src = &v->fb[y * W];
    for (uint32_t x = 0; x < W; x++) {
      uint32_t c = src[x], r = c & 31u, g = (c >> 5) & 31u, b = (c >> 10) & 31u;
      rgb[x] = (((r << 3) | (r >> 2)) << 16) | (((g << 3) | (g >> 2)) << 8) | ((b << 3) | (b >> 2));
    }
    uint32_t *row = &out[y * 3u * HS_GBA_OUT_W];
    for (uint32_t x = 0; x < HS_GBA_OUT_W; x++) {
      uint32_t p0 = rgb[v->lut_src0[x]], p1 = rgb[v->lut_src1[x]], w = v->lut_w[x], iw = 256u - w;
      uint32_t rb = (((p0 & 0xFF00FFu) * iw + (p1 & 0xFF00FFu) * w) >> 8) & 0xFF00FFu;
      uint32_t gg = (((p0 & 0x00FF00u) * iw + (p1 & 0x00FF00u) * w) >> 8) & 0x00FF00u;
      row[x] = 0xFF000000u | rb | gg;
    }
    memcpy(row + HS_GBA_OUT_W, row, HS_GBA_OUT_W * sizeof(uint32_t));
    memcpy(row + 2u * HS_GBA_OUT_W, row, HS_GBA_OUT_W * sizeof(uint32_t));
  }
}
