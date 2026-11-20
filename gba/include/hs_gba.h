/* SPDX-License-Identifier: MIT
 * hs_gba.h - HandshakeOS clean-room Game Boy Advance core, public contract.
 * Copyright (c) 2026 HandshakeOS contributors.
 *
 * Zero-mutex three-phase contract:
 *   1. hs_gba_step_cpu_phase()     - CPU/bus/DMA/timers for one frame. Writes
 *                                    only CPU-phase state and the *staging*
 *                                    snapshot, then publishes it (index flip).
 *   2. hs_gba_render_video_phase() - reads the published snapshot read-only,
 *                                    owns only video state + caller buffer.
 *   3. hs_gba_synth_audio_phase()  - reads the published snapshot read-only,
 *                                    owns only APU state + caller buffer.
 * Phases 2 and 3 may run on other cores concurrently with each other and with
 * the NEXT step_cpu_phase (double-buffered snapshots); the caller's handshake
 * barrier orders publish -> consume. No locks, no threads, no clocks.
 */
#ifndef HS_GBA_H
#define HS_GBA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HS_GBA_SCREEN_W 240
#define HS_GBA_SCREEN_H 160
#define HS_GBA_OUT_W 640
#define HS_GBA_OUT_H 480
#define HS_GBA_AUDIO_RATE 48000
#define HS_GBA_MAX_SAMPLES_PER_FRAME 832 /* stereo frames; nominal ~803.6 */
#define HS_GBA_CYCLES_PER_FRAME 280896u  /* 228 lines * 1232 cycles @ 2^24 Hz */

/* Keypad bits (1 = pressed), same order as KEYINPUT. */
enum {
  HS_GBA_KEY_A = 1u << 0, HS_GBA_KEY_B = 1u << 1, HS_GBA_KEY_SELECT = 1u << 2,
  HS_GBA_KEY_START = 1u << 3, HS_GBA_KEY_RIGHT = 1u << 4, HS_GBA_KEY_LEFT = 1u << 5,
  HS_GBA_KEY_UP = 1u << 6, HS_GBA_KEY_DOWN = 1u << 7, HS_GBA_KEY_R = 1u << 8,
  HS_GBA_KEY_L = 1u << 9
};

typedef enum {
  HS_GBA_SAVE_NONE = 0, HS_GBA_SAVE_SRAM, HS_GBA_SAVE_FLASH64,
  HS_GBA_SAVE_FLASH128, HS_GBA_SAVE_EEPROM
} hs_gba_save_type;

typedef struct hs_gba hs_gba;

/* Lifecycle (only place that allocates). */
hs_gba *hs_gba_create(void);
void hs_gba_destroy(hs_gba *g);
/* Optional real BIOS (16 KiB). Without it the clean-room HLE BIOS is used. */
int hs_gba_load_bios(hs_gba *g, const uint8_t *bios, size_t len);
/* Copies ROM (<= 32 MiB), detects save type, resets the machine. */
int hs_gba_load_rom(hs_gba *g, const uint8_t *rom, size_t len);
void hs_gba_reset(hs_gba *g);

/* Battery save image (host owns .sav file I/O). */
hs_gba_save_type hs_gba_save_kind(const hs_gba *g);
const uint8_t *hs_gba_save_data(const hs_gba *g, size_t *len);
int hs_gba_load_save(hs_gba *g, const uint8_t *data, size_t len);
int hs_gba_save_dirty(const hs_gba *g);
void hs_gba_save_clear_dirty(hs_gba *g);

/* ---- The 3-phase zero-mutex contract ---- */
void hs_gba_step_cpu_phase(hs_gba *g, uint16_t keys_pressed);
/* out: 640x480 XRGB8888, edge-to-edge stretch (3x vertical, 8/3 sharp-bilinear
 * horizontal). May be NULL to render only the native 240x160 frame. */
void hs_gba_render_video_phase(hs_gba *g, uint32_t *out640x480);
/* out: interleaved L,R int16 at 48 kHz; returns stereo frames written. */
int hs_gba_synth_audio_phase(hs_gba *g, int16_t *out, int max_frames);
/* FNV-1a 64 of the last rendered native 240x160 RGB555 frame. */
uint64_t hs_gba_frame_hash(const hs_gba *g);

/* Introspection helpers. */
const uint16_t *hs_gba_native_frame(const hs_gba *g); /* 240x160 BGR555 */
uint64_t hs_gba_frame_count(const hs_gba *g);
uint32_t hs_gba_audio_events_dropped(const hs_gba *g);

#ifdef __cplusplus
}
#endif
#endif
