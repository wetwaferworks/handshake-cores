# hs-n64 — clean-room Nintendo 64 core for HandshakeOS (REPORT)

Pure C11, MIT, zero mutexes, zero threads, zero mutable globals. All state lives in `hs_n64_t`
(11,778,208 bytes, caller-allocated, 64-byte aligned). The core plugs into the shared, unmodified
`include/hs_core.h` (`hs_core_ops_t`, `HS_SYS_N64`) and `common/hs_video.h`
(`hs_ssaa_render` strip tiling and `hs_fxaa`). Every number below comes from
`build/logs/*.log` produced by `./run-tests.sh` on 2026-11-19 (America/New_York).

## Files (line counts from `wc -l`)
| file | lines | role |
|---|---|---|
| `src/n64_cpu.c` | 658 | VR4300 interpreter: MIPS III 64-bit ISA, branch delay/likely, LWL/LWR/LDL/LDR/SWL/SWR/SDL/SDR, LL/SC, traps, COP0 (32-entry TLB, refill/invalid/mod, Count/Compare IP7, Random, ERET), COP1 FPU (S/D/W/L, FR=0/1, C.cond, BC1/BC1L, FCR31 rounding for conversions, VR4300 canonical qNaN) |
| `src/n64_bus.c` | 433 | physical map: 4/8 MiB RDRAM, DMEM/IMEM, SP regs + SP DMA, DP, MI, VI, AI, PI + PI DMA, RI, SI + PIF joybus, cart ROM; .z64/.v64/.n64 detection (byte-address XOR view, no copy); CIC-6102 CRC; PIF/IPL3 HLE boot |
| `src/n64_rsp.c` | 724 | RSP: OSTask dispatch; HLE gfx (Fast3D + F3DEX encodings, s15.16 integer matrices, lighting, fog, near-plane clipping, culling, viewport); RDP front end that freezes a bounded command list + texture blob into the BACK snapshot; HLE audio "HS-ABI"; scalar-unit LLE |
| `src/n64_rdp.c` | 786 | Phase-1 RDP rasterizer (FRONT-only): edge-function triangles, perspective-correct s/t, TMEM in all formats + TLUT, point / 3-point / bilinear filtering, wrap/mirror/clamp, Z16, 1/2-cycle combiner, blender + fog, alpha compare, fill/texture rectangles, scissor; strip-tiled 1x/FXAA/2x/4x/8x; `hs_n64_walk_backend` Panfrost hook |
| `src/n64_audio.c` | 38 | Phase-2: front AI stream → 48 kHz s16 stereo (16.16 linear resampler) |
| `src/hs_n64.c` | 158 | `hs_n64_core_ops` glue, explicit little-endian FNV-1a hashes |
| `src/n64_internal.h` | 270 | state structs, constants, prototypes |
| `include/hs_n64.h` | 73 | public API: config flags, `n64_rdp_backend_t`, `hs_n64_walk_backend` |
| `tests/test_hs_n64.c` | 1407 | torture suite (282 checks) and demo ROM run |
| `tools/mk_n64_demo.c` | 704 | original MIPS assembler + asset builder (pure C11, built with HC by `build.sh`) → `roms/hs_n64_demo.z64` |
| `build.sh` / `run-tests.sh` | 69 / 27 | HC build + audit / both-arch run + cross-arch compare |

`tests/hs_png.c`, `tests/hs_test.h`, `tests/hs_test_host.c` and `LICENSE` were copied unchanged from
`../tests` and `..`. Nothing outside `hs-cores/n64/` was modified. No git commits or pushes were made.

## Build + audit (`./build.sh`)
* Compiler: `/workspace/hc/build/current/hc` = **hc 0.1.4** (sha256 prefix `ee626d46371d2a5e`).
* Flags: `--profile=component -std=c11 -O3 -Wall -Wextra -Werror -ffp-contract=off` for x86_64, and the same
  plus `--target=aarch64-linux-gnu -mcpu=cortex-a53` for aarch64.
* 9 component TUs × 2 arches cover the 6 core files, `../common/hs_video.c`, `hs_png.c` and `test_hs_n64.c`. The
  only `--allow-runtime` TU is `hs_test_host.c` (the CLOCK_MONOTONIC timer shim, ×2 arches). Total: **20 objects**.
* `hc --audit-elf`: **20/20 PASS**. The `nm` banned-symbol scan (pthread, mutex, futex, `_cond_`, barrier,
  `mtx_`/`cnd_`/`thrd_`, sem_*, time, rand, clock_gettime, rdtsc, getrandom and similar) over the 18 component objects
  found **0 hits**. The ctor/dtor section scan found 0. Writable `.data`/`.bss` object symbols in the core objects: **0**.
* One fix came out of the audit: the regex `mtx_` matched the RSP function `n64_rsp_mtx_mul`, so I renamed it
  `n64_rsp_matmul`.
* Word-hygiene grep over `n64/`: 0 hits.

## Tests (`./run-tests.sh`)
* x86_64 native: **RESULT 282/282**.
* aarch64 (`qemu-aarch64-static -cpu cortex-a53`, statically linked binary): **RESULT 282/282**.
* Cross-arch: all **11 HASH lines are identical**, and **7/7 output files are byte-identical** (5 frame PNGs, the
  compare PNG and the WAV).

Coverage, in 12 groups:
* **CPU**:
  * 32/64-bit ALU and shifts.
  * MULT/MULTU/DIV/DIVU/DMULT/DMULTU (128-bit)/DDIV/DDIVU, including divide by zero and INT_MIN/-1.
  * Delay slots, branch-likely nullification, JAL/JALR/JR, BGEZAL.
  * Big-endian loads and stores, plus the whole unaligned L/R family.
  * AdEL with BadVAddr; Ov traps that leave rd untouched; SYSCALL/TEQ; Cause.BD and EPC on a delay slot.
  * Count==Compare IP7 and its ack; MI-mask VI interrupt on IP2 and its ack.
  * TLB even/odd pages, TLBP/TLBR, refill vector with TLBL, TLB Mod.
* **FPU**:
  * IEEE-exact ADD/SUB/MUL/DIV/SQRT in S and D; ABS/NEG/MOV.
  * Canonical qNaN; denormals are kept.
  * CVT.W.S under all 4 FCR31 modes; ROUND/TRUNC/CEIL/FLOOR; CVT between S/D/W/L.
  * C.cond with BC1T/BC1F/BC1TL; FR=0 register pairing; LDC1/SDC1/LWC1/SWC1.
  * CU1 unusable exception; a 2100-op stress hash.
* **Bus**:
  * The loader handles z64, v64 and n64, verifies the CRC on each, and reads identically across all three. The
    IPL3 HLE copy and the CIC register state are checked, junk images are rejected, and the 8 MiB Expansion Pak works.
  * PI DMA and its interrupt/ack.
  * SP DMA with count/skip in both directions.
  * SP hardware flag.
  * SI joybus controller read (buttons and d-pad stick).
  * AI DMA into the BACK snapshot at 48009 Hz.
  * DP_START/DP_END pass-through with FullSync → DP interrupt.
  * MI mask set/clear and version register; VI_CURRENT.
* **RSP maths**: isqrt64, Mtx load, row-vector multiply order, lighting (facing, back-facing, 45°, renormalised under a
  scaled modelview), ADPCM predictor and saturation.
* **RSP HLE / LLE**:
  * An F3DEX display list exercises segments, viewport, matrix push/pop, cull, near-clip split, trivial reject,
    G_DL return, ENDDL, LoadBlock → snapshot texture blob, SETOTHERMODE, FullSync, and SIG2/halt.
  * The Fast3D encoding is tested separately.
  * The scalar LLE program sums 1..10 = 55.
  * HS-ABI audio covers ADPCM, PAN, RESAMPLE ×2.0, INTERLEAVE, SAVEBUFF, MIXER, CLEARBUFF and state save.
* **RDP**:
  * Fill-mode bounds, scissor, combiner lerp (exact 0x800080), blender 50% (exact 0x7F0080).
  * Z reject/accept; a Z-target fill rect; Gouraud.
  * 9 texel formats via copy-mode texrect: RGBA16, RGBA32, IA8, IA4, I4, I8, IA16, CI8+TLUT, CI4+TLUT.
  * Coverage levels: 1x is hard edges only; 2x gives 3 levels, 4x 15, 8x 63.
  * FXAA touches edges only.
  * Watertight shared edges: an additive-blend quad shows 0 gaps and 0 double hits at 8x and at 1x.
* **Isolation**:
  * Over 20 frames, Phase 1/2 never change CPU, RDRAM or DMEM.
  * Phase 0 never changes the FRONT snapshot.
  * The overlapped schedule (step(f+1) into BACK while render/synth read FRONT(f)) gives video and audio
    bit-identical to the sequential schedule.
* **Demo ROM**: described in the next section.

## Demo ROM `roms/hs_n64_demo.z64`
The ROM is regenerated by `build.sh`. It is 1,052,672 bytes, big-endian .z64, sha256 `3ca4749d…3ca6`.
* Code: 345 original MIPS instructions, and the CIC-6102 CRC is valid.
* Setup: it installs a VI interrupt handler at 0x80000180, PI-DMAs its assets, and runs a 64-bit SD/LD self-check.
  It then programs VI, AI (dacrate 1013) and MI, and reads the joypad through SI/PIF every frame.
* Per frame:
  * Rotates two objects with COP1 math (renormalised sin/cos) and builds Fast3D matrices with CVT.W.S.
  * Submits a gfx OSTask: Z clear and sky fill, then a fogged CI8+TLUT 2-cycle checker ground (9×9 grid), a lit
    RGBA16 textured cube, a copy-mode texrect HUD and an I8 textured Gouraud pyramid.
  * Submits an audio OSTask that ADPCM-decodes, resamples, pans and interleaves, then AI-DMAs 3200 bytes.

90 frames were run. Every frame had 1 gfx task, 1 audio task, ≥64 visible triangles (max 93 after cull/clip), 800 AI
frames, 0 overflow and 0 reserved-instruction exceptions. vicount and the ROM frame counter advanced every frame.
Phase 2 produced 71,987 frames of 48 kHz audio (`n64_audio_48k.wav`). START freezes the scene and release resumes it.
The backend walker saw 93 tris (all textured), 2 fill rects and 1 texrect, and the walk was read-only.

Outputs in `out/x86_64/` and `out/aarch64/`, byte-identical:
* `n64_3d_1x_640x480.png`, `n64_3d_fxaa_640x480.png`, `n64_3d_2x_640x480.png`, `n64_3d_4x_640x480.png` and
  `n64_3d_8x_640x480.png`: each is 640×480 with 0 black edge pixels, filled edge to edge with no bars. The 320×240
  native frame is scaled 2× in both axes inside the rasterizer.
* `n64_ssaa_compare.png` (1600×720): the top row is 4× crops of 1x | FXAA | 2x | 4x | 8x at the strongest-aliasing
  window, found automatically at x=200 y=160. The bottom row is the full 1x and the full 8x frame.
* `n64_audio_48k.wav`.

### Edge quality vs 8x (RMS, 0–255 per channel; edge mask = 34,421 px of strong 8x gradients)
| mode | RMS all | RMS edge |
|---|---|---|
| 1x | 2.423 | 5.870 |
| FXAA | 2.954 | 6.683 |
| 2x | 0.800 | 1.827 |
| 4x | 0.334 | 0.682 |

Honest note: FXAA does **not** move closer to the 8x reference by this metric; its edge RMS is higher than 1x. It
smooths staircases (91.9% of the 32,497 pixels it changes are within 2 px of a real 8x edge), but it also blurs the
checker and texture detail that 8x keeps sharp. SSAA error falls monotonically: 1x > 2x > 4x.

### Frame times (real, `hs_host_now_ns`, single thread)
| phase | x86_64 native (8-vCPU box) | aarch64 under qemu-aarch64-static (cortex-a53, emulated on the x86_64 box) |
|---|---|---|
| Phase 0 CPU+RSP HLE (781,250 instr/frame) | 3.787 ms | 28.689 ms |
| Phase 1 1x | 15.27 ms | 116.0 ms |
| Phase 1 FXAA | 18.45 ms | 138.5 ms |
| Phase 1 2x | 60.4 ms | 466.1 ms |
| Phase 1 4x | 238.0 ms | 1,855.8 ms |
| Phase 1 8x | 959.0 ms | 7,567.7 ms |
| Phase 2 audio | 0.002 ms | 0.011 ms |
| sequential 1x frame | 19.06 ms | 144.7 ms |
| 3-core pipelined bound at 1x (max of phases, **computed**, no threads used) | 15.27 ms | 116.0 ms |

The aarch64 numbers are qemu user-mode emulation on an x86_64 host, not real Cortex-A53 silicon. Real A53 hardware
will differ.

### Cross-arch hashes (identical on both)
```
HASH fpu_stress 46ab9e1e729b89df
HASH demo_video_rolling 62bb682c7e05182a
HASH demo_video_last 1d19341f7c7c4cb4
HASH demo_audio_last 462876b2151a89dd
HASH demo_pcm48k_all bbbabb79b841507f
HASH demo_cpu_state 7bf5474f4b4dc113
HASH demo_fb_1x 48cc0825b4d2c846
HASH demo_fb_fxaa 0df142f8f60deaa3
HASH demo_fb_2x 5afa4531b1ce55f7
HASH demo_fb_4x 4ccc5486c67bd4ff
HASH demo_fb_8x f1b261dc55a015c3
```
The PNG/WAV sha256 values are in `build/logs/sha256_outputs.txt`.

## Determinism design
* Phase 0 writes BACK only, `commit_phase` flips `front ^= 1`, and Phase 1/2 read FRONT plus their own private
  `n64_video_t`/`n64_audio_t`. Texture loads are resolved from RDRAM into the snapshot in Phase 0, so Phase 1 never
  touches RDRAM.
* All RDRAM, ROM and hash I/O uses explicit big- or little-endian byte packing, so host endianness never leaks.
* `-ffp-contract=off`. The FPU uses IEEE single/double with canonical NaNs, and conversions round in software per
  FCR31. RDP setup uses doubles once per primitive and int64 fixed-point stepping per sample. RSP math is integer.
* There are no clocks or RNGs. The RDP "noise" input is a fixed hash of the pixel position.

## Limitations (honest)
* **Interpreter, not a dynarec**: about 3.8 ms/frame on x86_64. Timing is approximate: 1 instruction = 2 PClocks,
  781,250 instructions per frame, and there are no cache, pipeline-stall or RDRAM latency models.
* **Microcode coverage**: RSP gfx is HLE for a Fast3D/F3DEX subset (MTX, VTX, TRI1/TRI2, DL, ENDDL, MOVEMEM viewport
  and lights, MOVEWORD segment/numlight/fog, geometry/other modes, TEXTURE, POPMTX, TEXRECT, RDP pass-through).
  * Not implemented: F3DEX2/S2DEX/Turbo3D, G_CULLDL (no-op), LOD/mipmaps, and point/line microcodes.
  * Commercial ROMs that depend on undocumented microcode behaviour will not render correctly.
* **Audio** uses the HandshakeOS HS-ABI command set (ADPCM/PCM, resample, pan, mixer, interleave), not Nintendo's
  audio microcode ABI. Commercial game audio will not play.
* **RSP LLE** covers the scalar unit only. The vector unit (COP2, LWC2/SWC2) is not modelled.
* **RDP**:
  * Native edge-coefficient triangle commands (0x08–0x0F) from DP_START/DP_END buffers are parsed for length but
    not rasterized. Triangles reach Phase 1 through the private screen-space HS_TRI record.
  * TMEM is linear: no odd-line word swap, no RGBA32 high/low bank split, no LOD/detail/sharpen, no dithering, no
    coverage/AA-blender modes, no YUV.
  * Output is 8888, not 5551 RDRAM writes. Phase 1 re-renders from the frozen command list instead of scanning
    out the VI framebuffer, so CPU-drawn framebuffer effects are not shown.
  * The COP1 FPU does arithmetic in round-to-nearest only (FCR31.RM applies to conversions) and does not raise
    FP exceptions except "unimplemented".
* **IPL3 is HLE**: CIC-6102 register state and PI copy of 1 MiB; other CIC variants are not specially handled.
  SRAM/EEPROM/Flash and Controller Pak are not implemented (`load_sram` returns -1).
* **Input**: the shared ABI has no analog axis, so the d-pad also drives the stick (±80).
* **Software 4x/8x are slow**: 238 ms and 959 ms per frame on x86_64, because they shade 16 and 64 samples per
  output pixel. Real-time 4x/8x is intended for the `n64_rdp_backend_t` hook (Panfrost); the walker is implemented
  and tested, but no Panfrost backend is included or measured here. Software 1x/FXAA are the real-time modes.
* The 3-core figure is a computed bound (the maximum of the measured phase times). This test harness does not spawn
  threads.
