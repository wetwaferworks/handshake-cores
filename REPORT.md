# hs-cores: clean-room HandshakeOS emulator framework + hs-video + hs-gbc

Built 2026-11-19 (America/New_York). Pure C11, MIT, zero mutexes / pthreads / atomics,
no `time()`/`rand()`/`setjmp`/constructors/inline asm, no malloc inside cores.
Compiler: **frozen `hc 0.1.2`** `/workspace/scratch/handshakeos/builds/v2-hc/hc`
(sha256 `48fcf22c…`), `--profile=component` for every core, video and test TU on
x86_64 and `--target=aarch64-linux-gnu -mcpu=cortex-a53 -O3`. The only runtime-profile TU
is `tests/hs_test_host.c` (the monotonic clock used for benchmark timing).

Reproduce: `./run-tests.sh` (build + audit + x86_64 + qemu ARM64 + cross-arch hash diff + acid2).

## Layout
- `include/hs_core.h`: the 7-console ABI (`hs_console_t`, `hs_video_mode_t`, input bitmask,
  `hs_core_ops_t` 3-phase contract + `commit_phase` barrier, FNV-1a helper)
- `common/hs_video.[ch]`: 640x480 LUT stretcher (sharp area-weighted / nearest), integer FXAA,
  strip-tiled 1x/2x/4x/8x SSAA
- `gbc/`: hs-gbc DMG+CGB core (cpu, mmu, ppu, apu, core)
- `tests/`: test suites, PNG writer, host clock shim, acid2 comparer, `ext-roms/` (local-only inputs)
- `build.sh`, `run-tests.sh`, logs in `build/logs/`, artifacts in `out/{x86_64,aarch64}/`

(`gba/`, `snes/`, `genesis/` were created by other workers in parallel. This pass did not touch them.)

## Results
See `build/logs/run-tests.txt`, `build/logs/test_*.log`, and `build/logs/hashes.*.txt`.

### Gate summary (run-tests.sh, 2026-11-19 ~22:00 ET)
- 20/20 objects `hc --audit-elf` PASS, 0 banned symbols from nm (lock/thread/atomic-CAS/setjmp/time/rand), no ctor sections
- test_hs_gbc: 83/83 on x86_64 and 83/83 on ARM64 (qemu-aarch64-static)
- test_hs_video: 55/55 on x86_64 and 55/55 on ARM64
- 40/40 HASH lines are bit-identical between x86_64 and ARM64, and every output PNG is byte-identical
- blargg cpu_instrs (DMG + CGB), instr_timing, mem_timing, halt_bug: PASS
- dmg-acid2 and cgb-acid2: 23040/23040 pixels match the reference on both arches

## Known limitations / next refinements
- PPU: fixed 172-dot mode 3 (no SCX/sprite/window penalties), so rendering is per scanline: mid-scanline register
  writes and VRAM writes during mode 3 take effect on the next line. VRAM/OAM are not blocked during mode 2/3.
  The STAT line-144 OAM quirk, the LY=153 early-zero quirk and the LCD-enable first-line quirk are not modelled.
- APU: the frame sequencer runs on its own 8192-cycle counter (not tied to DIV bit 12/13). The following are not
  modelled: zombie-mode envelope, the extra length clock, wave-RAM read/write while playing, and CGB PCM12/PCM34.
  Point-sampled mixing has no band-limiting, so very high notes alias.
- Mappers: MBC1M multicart, MBC30, HuC1/HuC3, MBC6/7, MMM01 and the camera are not implemented. The RTC sub-second
  counter is not modelled.
- There is no boot ROM (the core starts from the post-boot register state). DMG games on GBC use the selected DMG
  palette, not the CGB boot compatibility palettes. Serial has no link partner.
- The VRAM/OAM write log holds 65,536 entries per frame. Overflow is flagged and was never hit in testing.
- FXAA is an integer approximation of FXAA 3.11 quality-lite: an 8-step bounded end search and a 1-neighbour lerp.
- SSAA uses a box filter. The 3D rasterizer is only a flat-shaded benchmark harness. On the CPU alone, 8x is far from
  real time (53 ms per frame on the host). Usable 4x/8x on the H700 needs the Mali-G31 or a tile-binned rasterizer.
- Parallel safety is proven by memory-domain hashing and by pipelined-order equivalence, not by real threads,
  because pthreads are banned in tests. Wiring Core 0/1/2 into the HandshakeOS runtime is the next step.
- The static aarch64 test binaries carry 3 `pthread_mutex*` symbols from static glibc stdio internals. The 20 hs-cores
  objects carry none.
