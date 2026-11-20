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

> **Zero mutex, with one exception.** Everything in the Handshake stack ships with zero mutexes: PID 1, the emulator cores, the runtime and the USB modules. The N64 core objects have 0 lock symbols. The v5.5 image has one exception: two N64 diagnostic tools, `bench_n64` and `hs_pf_probe`, are statically linked against glibc and carry its internal pthread mutex code. v5.6 removes the exception by relinking them against `hs_rt`, HandshakeOS's own lock-free runtime.

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

**The one exception in v5.5:** two N64 *diagnostic tools* in this repo, `n64/tests/bench_n64.c` and `n64/tools/hs_pf_probe.c`, were linked as static glibc programs for the v5.5 image, and glibc's static library contains its own internal pthread mutex. They are diagnostic tools, not part of the core. v5.6 relinks them against HandshakeOS's own lock-free runtime, `hs_rt`.

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

All timings above are from emulated or host runs. They are labelled, and they have not yet been measured on the RG35XX itself.

## Building and testing

You need Ubuntu 24.04 (WSL works), `gcc`, `aarch64-linux-gnu-gcc`, `qemu-aarch64-static` and the [Handshake Compiler](https://github.com/wetwaferworks/handshakecompiler).

```sh
./build.sh              # build every core with HC, both arches, plus the audit
./run-tests.sh          # all suites, x86_64 and aarch64, with a cross-arch compare
cd n64 && ./run-tests.sh
```

Every generator (demo ROMs, lookup tables) is a C program. There is no Python anywhere in this repo.

## Still open (N64, short list)

1. Real-A53 measurements on the handheld.
2. Audio microcode HLE for SM64 and Zelda (currently silent).
3. RDRAM framebuffer and Z write-back.
4. S2DEX backgrounds.
5. RDP LOD/mipmap, key, YUV, coverage/dither.

## License

MIT, Copyright (c) 2026 Jaydee Wetwork. No commercial ROMs or BIOS images are included or downloaded.

---

## Credits: Muse

- **Elfie (Muse Wonder)**: co-creator of the Timing Handshake with JD; UML design architecture and the HandshakeOS handoff packet.
