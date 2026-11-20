# hs-n64: Super Mario 64 / Ocarina of Time / Majora's Mask compatibility

Status 2026-11-20 (HandshakeOS v5.4 target). No commercial ROM, microcode or BIOS is in this tree,
and none was used to write or test this code. Everything below is checked with synthetic tests.
Real-cartridge status can only be measured with the owner's own dumps, using
`build/<arch>/bench_n64 --rom=/path/to/own.z64 --frames=600 --png=out.png`. That prints
`COMPAT` lines: CIC, header CRC, save type, detected microcode, HLE / skipped / unknown task
counts, unhandled display-list commands, triangles, reserved-instruction count, and save writes.

Native frame rates: SM64 runs its game logic at 30 fps (NTSC). OoT and MM run at **20 fps**
(NTSC, 60 Hz VI). "24-30 fps" for the Zelda games therefore means full speed: 20 game fps
with a 60 Hz VI. The emulator cannot show more distinct Zelda frames than the game draws.

## Closed today (with tests)

| Area | What | Test |
|---|---|---|
| Microcode ID | OSTask `ucode_data` banner -> Fast3D / F3DEX / F3DEX2 (incl. F3DZEX, F3DLX2, F3DLP2) / S2DEX / unknown. HS tasks (`ucode_data == 0`) keep the old path, so all 11 hashes are unchanged | classifier on 6 banners |
| F3DEX2 / F3DZEX HLE | New decoder: VTX, TRI1/TRI2/QUAD, MTX (push/load/projection), POPMTX, GEOMETRYMODE (bit remap), MOVEWORD (segments, num lights, fog, light colour), MOVEMEM (viewport, lights), TEXTURE, DL/ENDDL, BRANCH_Z (always near), OTHERMODE_L/H, RDPHALF_1/2, texture rectangles, RDP pass-through | same scene in F3DEX and F3DZEX encodings gives a byte-identical RDP stream, unlit and lit |
| Fast3D (SM64) | Banner selects the Fast3D vertex/index encoding (was a demo-only flag) | classifier |
| Nintendo audio tasks | Audio tasks with a real microcode are skipped safely (silent, RDRAM untouched, task completes). The HS-ABI mixer would otherwise misread them and write RDRAM | RDRAM hash before/after |
| EEPROM 4K/16K | joybus ch 4: info (00 80 00 / 00 C0 00), read/write 8-byte blocks | PIF frames |
| SRAM 256 Kbit | PI domain 2 DMA both directions, CPU 32-bit access | DMA write/read back |
| FlashRAM 1 Mbit | Command register 0x08010000: ID (11118001 00C2001E), status, sector/chip erase, page-buffer program, read array (bus address x2) | full sequence |
| Save persistence | Save memory survives console reset; `save_sram`/`load_sram` export it (cartridge save + optional 32 KiB pak) | round trip |
| Controller pak | ch 0 cmds 0x02/0x03, 32-byte blocks, data CRC-8 (poly 0x85), status 0x01 when enabled (`HS_N64_CFG_CONTROLLER_PAK`) | write/read/CRC |
| Save type selection | Header ID table: SM -> EEPROM 4K, ZL -> SRAM, ZS -> FlashRAM; `HS_N64_CFG_SAVE(t)` override | synthetic headers |
| Expansion Pak | MM (ZS) gets 8 MiB RDRAM automatically, osMemSize = 8 MiB | synthetic header |
| CIC | CRC-32 of IPL3 (ROM 0x40..0xFFF) -> 6101/6102/6103/6105/6106/7102. Per-CIC s6 seed, entry offset (6103 -1 MiB, 6106 -2 MiB), header-CRC variant, osMemSize also at 0x3F0 for 6105. Unknown IPL3 -> 6102 path (unchanged) | CRC-32 check value, 6102 boot, variants distinct |

The IPL3 CRC-32 values, CIC seeds, save-chip command bytes and the pak CRC are public
interface facts from N64 hardware documentation. They are written from that documentation,
not copied from any emulator. **Each one still needs one check against a real cartridge dump
on the owner's machine.** The FlashRAM read-array address doubling and the 6105 checksum
variant are the least certain.

## Remaining gaps, ranked (biggest blocker first)

1. **CPU throughput on the A53 (speed, all three titles).** The interpreter (decode cache +
   KSEG0/1 fast fetch) runs a fully busy N64 frame (781,250 instructions) in 2.9 ms on x86_64
   and 26.8 ms under qemu-user aarch64 (emulated). Unlike the demo, commercial games keep the
   CPU busy most of the frame. Real-A53 cost is **unmeasured**. qemu-user is not a reliable
   proxy, and the first thing to run on the RG35XX is `bench_n64` (busy line). A JIT needs
   executable memory, which the HC component profile does not allow. The options are a
   runtime-profile JIT TU (needs an owner decision on W^X) or more interpreter work (block
   chaining, register caching).
2. **Audio microcode HLE (sound).** SM64 (ABI 1) and the Zelda audio microcode are not
   implemented, so games run silent. Next step: ABI command set (ADPCM with 8-entry codebooks,
   RESAMPLE, ENVMIXER, INTERLEAVE, LOADBUFF/SAVEBUFF, SETVOL, DMEMMOVE, LOADADPCM, SEGMENT),
   checked against hand-made command lists.
3. **RDRAM framebuffer / depth write-back (Zelda effects).** The renderer draws the frozen RDP
   list straight to the display. It never writes colour or Z back to RDRAM. Effects that read
   the framebuffer or Z buffer on the CPU or as a texture will be wrong or missing: OoT pause-menu
   background, sun/lens-flare depth test, MM pause screen / transitions / Lens of Truth.
   Render-to-texture (SetColorImage to a non-display buffer) needs the same work. It also needs a
   Phase-model decision: Phase 1 is read-only on FRONT today.
4. **S2DEX (OoT pre-rendered backgrounds).** S2DEX tasks are detected and skipped (counted in
   `unknown_gfx_tasks`). OoT's pre-rendered rooms (BG rect copy) need S2DEX `BG_1CYC`/`BG_COPY`
   and `OBJ_RECTANGLE` HLE. `G_LOAD_UCODE` mid-list is counted but not followed.
5. **RDP features.** No LOD / mipmapping (LOD fraction uses the primitive LOD only), no
   SetKey (chroma key), no SetConvert / YUV, no coverage-based AA or dither, and Z decal modes
   are approximate. Zelda uses mip-mapped and 2-cycle textures widely.
6. **F3DEX2 leftovers.** `G_MODIFYVTX`, `G_MV_MATRIX` (forced MVP), exact `G_BRANCH_Z`
   (always takes the high-detail branch now), `G_CULLDL` (no-op), point lighting (F3DZEX), and
   `G_MW_PERSPNORM` / `G_MW_CLIP`. Counted in `unhandled_cmds`.
7. **RSP LLE.** Unknown tasks run on a scalar-only RSP interpreter (no vector unit). Any title
   or task not covered by HLE will misbehave.
8. **CIC-6105 side effects.** OoT/MM boot on 6105. The HLE boot sets seed / osMemSize but does
   not reproduce what the real IPL3 leaves in SP memory or the PIF challenge/response.
   Unverified until a real dump boots.
9. **Input.** The RG35XX 2024 has no analog stick, so the d-pad drives the stick at a fixed 80.
   SM64 / Zelda are playable but walk/run control is coarse. Rumble pak is not emulated.
10. **PAL titles / timing.** NTSC only (VI clock, 60 Hz frame budget).

## How to measure progress on the owner's dumps (not bundled)

```
build/x86_64/bench_n64 --rom=/path/to/sm64.z64 --frames=900 --reps=1 --png=out/sm64.png
qemu-aarch64-static build/aarch64/bench_n64 --rom=... (emulated)    # or natively on the RG35XX
```

Pass criteria per title: `cic` known and `crc_ok 1`, the expected `save_type`, `ucode fast3d`
(SM64) or `f3dex2` (Zelda), `unknown_gfx_tasks` and `reserved_instr` near 0, `tris` > 0
after the intro, and `phase0_cpu_rsp_ms` + max slice ms within the frame budget.
