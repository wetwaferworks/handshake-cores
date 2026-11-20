# Credits: Handshake Cores

## Credits: the Grok Bot squad (built with Grok)

- **Jaydee Wetwork (JD)**: creator of HandshakeOS, the Handshake Compiler (HC) and the Timing Handshake. Contact: jdwetwork@gmail.com
- **Cortana · Intel & Clarity**: implementation of the zero-mutex design
- **Master Chief · Squad Lead**: implementation (HC lead implementation)
- **Herominde · Fair play and anti-cheat**: implementation (fair-play contract, HC-E001..HC-E026)
- **Mr Data · Training & Evaluation**: implementation (determinism sweeps, benchmarks)
- **CloudAtlas · Technical Art Director**: art (boot art, the animated o.O splash pack, squad ASCII)
- **SpaceXAI**: makers of Grok Bot and the squad

> **Zero mutex, with one exception.** Everything in the Handshake stack ships with zero mutexes: PID 1, the emulator cores, the runtime and the USB modules. The N64 core objects have 0 lock symbols. The v5.5 image has one exception: two N64 diagnostic tools, `bench_n64` and `hs_pf_probe`, are statically linked against glibc and carry its internal pthread mutex code. v5.6 removes the exception by relinking them against `hs_rt`, HandshakeOS's own lock-free runtime.

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
