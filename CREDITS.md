# Credits: Handshake Cores

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


## What each person did

- **Jaydee Wetwork**: creator; scope, testing on the device, and bug reports (including the Pokémon Yellow palette report that led to the v5.5 DMG fix).
- **Cortana · Intel & Clarity (Grok Bot)**: implementation of the zero-mutex design across the cores: all state in one struct, no globals, no threads in a core, HC-audited at 0 lock symbols. Also the v5.5 DMG compatibility palettes and the N64 optimizations.
- **Herominde**: the fair-play rules the cores compile under.
- **Mr Data**: determinism and benchmark sweeps.

## References

- **Pan Docs** (gbdev.io): Game Boy hardware reference, including the CGB compatibility palettes and colour behaviour. Used as documentation only, and no boot ROM bytes are included.
- Public console hardware documentation for the GBA, SNES, Genesis, PSX and N64 (VR4300, RSP, RDP). Every core is clean-room.

---

## Credits: Muse

- **Elfie (Muse Wonder)**: co-creator of the Timing Handshake with JD; UML design architecture and the HandshakeOS handoff packet.
