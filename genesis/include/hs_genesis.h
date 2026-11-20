/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_genesis.h - public entry point of the hs-genesis clean-room Sega
 * Genesis / Mega Drive core. Implements the shared 3-phase zero-mutex
 * HandshakeOS core ABI from hs_core.h:
 *   Core 0  step_cpu_phase     : 68000 + bus + VDP ports/DMA, writes BACK slot
 *   Barrier commit_phase       : BACK -> FRONT publish (+ Z80 RAM mirror sync)
 *   Core 1  render_video_phase : VDP planes/window/sprites -> 640x480 stretch
 *   Core 2  synth_audio_phase  : Z80 + YM2612 + SN76489 -> 48 kHz s16 stereo
 */
#ifndef HS_GENESIS_H
#define HS_GENESIS_H

#include "hs_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* hs_core_config_t.flags bits for the Genesis core */
#define HS_GEN_CFG_PAL       0x1u   /* 50 Hz, 313 lines, V30 allowed */
#define HS_GEN_CFG_3BUTTON   0x2u   /* report a 3-button pad (default 6-button) */
#define HS_GEN_CFG_JAPAN     0x4u   /* domestic version register */

extern const hs_core_ops_t hs_genesis_core_ops;

#ifdef __cplusplus
}
#endif
#endif
