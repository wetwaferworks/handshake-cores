# External verification ROMs (local test inputs only, NOT shipped)

Downloaded 2026-11-19 for local verification of the clean-room hs-gbc core.
They are inputs to the test suite only; no code from them is used. Keep them
out of any HandshakeOS image or redistribution (blargg's readme has no clear
redistribution license, matching /workspace/handshakeos/roms/manifest.txt).

| file | source | sha256 |
|---|---|---|
| cpu_instrs.gb, instr_timing.gb, mem_timing.gb, halt_bug.gb | blargg (mirror github.com/retrio/gb-test-roms) | see `sha256sum *` |
| dmg-acid2.gb + dmg-acid2.reference.png | github.com/mattcurrie/dmg-acid2 v1.0 (MIT) | |
| cgb-acid2.gbc + cgb-acid2.reference.png | github.com/mattcurrie/cgb-acid2 v1.1 (MIT) | |

The suite skips any file that is missing.
8c5e12f41e0ba5bbca796944f92ffe6de28809198682c4332e38d1b3cf56fcf2  cpu_instrs.gb
464e14b7d42e7feea0b7ede42be7071dc88913f75b9ffa444299424b63d1dff1  dmg-acid2.gb
ddebfb81f992132954a5580e16cfb97346a4e061c762634d7701328e23fea7c6  halt_bug.gb
646067b3d6c79fda810e9c3f1cb7c0efd5abb0a7ac06437c54e65720c15d9925  instr_timing.gb
791cb418fd054b482a6ada5017e2405b96c2373a0f3914c6b23afeee76fa98ce  mem_timing.gb
197fb0bcec544f0400527fc707e0a94f55435974986e6986b424ace5de81720e  cgb-acid2.gbc
9ea9c262c5383353e77d715d021a0f7c5ccbe438f88082cb225756e50c4fdf01  cgb-acid2.reference.png
ca966d50895c7efef05838590d148c2cbfd7fba57dab986f25b35b4da71abb57  dmg-acid2.reference.png
