# Handshake Cores

*Clean-room, pure-C11 game console emulators for HandshakeOS. Zero mutexes, zero threads inside a core, and every frame deterministic and hashable.*

## Credits: the Grok Bot squad (built with Grok)

- **Jaydee Wetwork (JD)**: creator of HandshakeOS, the Handshake Compiler (HC) and the Timing Handshake. Contact: jdwetwork@gmail.com
- **Cortana · Intel & Clarity**: implementation of the zero-mutex design
- **Master Chief · Squad Lead**: implementation (HC lead implementation)
- **Herominde · Fair play and anti-cheat**: implementation (fair-play contract, HC-E001..HC-E026)
- **Mr Data · Training & Evaluation**: implementation (determinism sweeps, benchmarks)
- **CloudAtlas · Technical Art Director**: art (boot art, the animated o.O splash pack, squad ASCII)
- **SpaceXAI**: makers of Grok Bot and the squad

> **Zero mutex in userspace.** The idea and the shape are JD's; the assistants co-implemented. From v5.6 on, the shipped userspace and PID 1 runtime are zero-mutex: PID 1 (`handshake-init`), the emulator cores (the N64 core objects included), the `hs_rt` runtime, the USB modules, and the N64 diagnostic tools `bench_n64` and `hs_pf_probe` (relinked against `hs_rt`) all scan clean, with 0 lock symbols. Evidence is in handshake-builds: `v5/v5.6/image-scan.txt`, `v5/v5.7/image-scan.txt`, and `v5/v5.8/image-scan.txt` each end in `IMAGE_ZERO_MUTEX_CLEAN` (`bench_n64` and `hs_pf_probe` CLEAN). The v5.9.1 run is `v5/v5.9.1/image-scan.txt`, `v5/v5.9.1/nm-locks.txt`, and `v5/v5.9.1/scan-run.txt` (`IMAGE_ZERO_MUTEX_CLEAN`, `NM_SUMMARY objects=186 lock_dirty=0`).
>
> The kernel underneath is mainline Linux 7.3.5 + ROCKNIX, which still has its own kernel mutexes (static count 1,599 mutex call sites; kernel work is next week). v5.8 and later ship kernel `7.3.5-hs-v58` on that base.


---

## What this is, in plain words

These are the emulators HandshakeOS uses to play games on the RG35XX 2024. Each one is written from public hardware documentation, in plain C11, under the MIT license. No code is copied from other emulators, and no commercial games or BIOS files are included: you bring your own.

| Core | Console | v5.5 status | Tests |
|---|---|---|---|
| `gbc/` | Game Boy / Game Boy Color | DMG compatibility palettes + CGB LCD colour curve (new) | **83/83**, no hash changes |
| `gba/` | Game Boy Advance | stable | suite rc=0 |
| `snes/` | Super Nintendo | stable | suite rc=0 |
| `genesis/` | Sega Genesis / Mega Drive | stable | suite rc=0 |
| `psx/` | PlayStation | stable | **108/108** |
| `n64/` | Nintendo 64 | fast paths, F3DEX2, saves (new) | **307/307** x86_64 and aarch64 |
| `common/` | shared video (SSAA strips, FXAA) | stable | **55/55** |

## How a core fits the Timing Handshake

A core is a component in the HandshakeOS Timing Handshake (Jaydee Wetwork and Elfie). All of its state lives in one caller-allocated struct (for example `hs_n64_t`). It has no mutable globals and starts no threads of its own. HandshakeOS calls it at fixed points in each phase (**publish → pull → barrier → commit**). Inputs arrive stamped `(seq, tick)`, and every frame's picture and audio are hashed into the ledger. Running the same game with the same inputs always gives the same hashes, on a PC or on the handheld.

Because nothing is shared outside a phase boundary, a core needs no locks. **Every core object compiles under HC with 0 lock symbols**, the N64 core included (30/30 objects pass `hc --audit-elf`, with 0 lock, thread, clock or RNG symbols in its 26 component objects).

From v5.6 on, those two N64 *diagnostic tools* (`n64/tests/bench_n64.c` and `n64/tools/hs_pf_probe.c`) are relinked against HandshakeOS's own lock-free runtime, `hs_rt`, and scan CLEAN in the shipped images. They are diagnostic tools, not part of the core. The v5.5 image still used the old static-glibc link.

The kernel underneath HandshakeOS is mainline Linux 7.3.5 + ROCKNIX, which still has its own kernel mutexes (static count 1,599 mutex call sites; kernel work is next week).

## What changed for v5.5

### Game Boy: correct colours for original Game Boy games (`gbc/gbc_compat_pal.c`)
On a real Game Boy Color, original Game Boy games are coloured by the boot ROM, which picks a palette from a table keyed on the game title's checksum (and the 4th title letter at `$0137` to break ties). HandshakeOS now does the same, clean-room from Pan Docs. **Pokémon Yellow** gets combination 3: white, yellow, red and black. Colours then pass through the CGB LCD curve, `R' = min(960, 26r + 4g + 2b)` (and the same for G and B), so they look like the real screen instead of oversaturated. GBC suite: 83/83 with no existing hash changes.

### Nintendo 64: faster and more compatible (`n64/`)
- Fast interpreter path: Phase 0 dropped from 52.0 ms to 0.215 ms per frame in emulated aarch64 (`bench_n64`, demo ROM). Final state is identical to the reference interpreter.
- Faster RDP rendering: 1x from 112.3 ms to 51.4 ms emulated, and split into three strip-interleaved slices that can run on three cores (computed bound about 17.9 ms per frame, emulated).
- F3DEX2 / F3DZEX display lists.
- Save memory: EEPROM 4K/16K, SRAM and FlashRAM, persisted. Save type comes from the header ID, and the Expansion Pak is enabled automatically where needed.
- Panfrost GPU path (`n64_pf*`): batch builder validated on a mock device only. The real-device probe is `hs_pf_probe`.
- Tests: **307/307 on x86_64 and 307/307 on aarch64** (the original 282 plus 25 new).
- Full write-up: [`n64/REPORT.md`](n64/REPORT.md), [`n64/docs/N64_COMPAT.md`](n64/docs/N64_COMPAT.md), [`n64/docs/PANFROST_PLAN.md`](n64/docs/PANFROST_PLAN.md).

All of those N64 timings are from emulated or host runs and are labelled as such.

### SNES audio and pacing (after v5.5)

The SNES core now has additional audio/pacing work (`snes/snes_apu.c`, `snes/hs_snes.c`, and `snes/docs/elfie-snes-audio-brief.md`). On the RG35XX 2024, SNES pacing is still the open problem: v5.8 ALttP ran at game_fps 40.6 / draw_fps 10.2 with emu_ms_avg 95 and 209 xruns in 23.5 s. v5.9's catch-up and pad-hold felt better, but the budget is still missed. The golden ledger hash `5f44cefa72e8ac19` is unchanged.

The o.O boot art is CloudAtlas's splash pack, shipped with HandshakeOS. HC v0.1.4 is public in [handshakecompiler](https://github.com/wetwaferworks/handshakecompiler).

## Building and testing

You need Ubuntu 24.04 (WSL works), `gcc`, `aarch64-linux-gnu-gcc`, `qemu-aarch64-static` and the [Handshake Compiler](https://github.com/wetwaferworks/handshakecompiler).

```sh
./build.sh              # build every core with HC, both arches, plus the audit
./run-tests.sh          # all suites, x86_64 and aarch64, with a cross-arch compare
cd n64 && ./run-tests.sh
```

Every generator (demo ROMs, lookup tables) is a C program. There is no Python anywhere in this repo.

## Still open

1. SNES pacing on the H700 (main userspace issue).
2. Real-A53 N64 measurements on the handheld.
3. Audio microcode HLE for SM64 and Zelda (currently silent).
4. RDRAM framebuffer and Z write-back.
5. S2DEX backgrounds.
6. RDP LOD/mipmap, key, YUV, coverage/dither.

## License

MIT, Copyright (c) 2026 Jaydee Wetwork. No commercial ROMs or BIOS images are included or downloaded.

---

## Credits: Muse

- **Elfie (Muse Wonder)**: co-creator of the Timing Handshake with JD; UML design architecture and the HandshakeOS handoff packet.
