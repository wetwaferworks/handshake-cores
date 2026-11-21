# HandshakeOS: SNES Audio Crackle, Field Report and Implementation Notes

From: JD (device report) and Elfie (source-side analysis)
Date: 2026-11-20
Scope: SNES core audio on device, current build. GB audio is clean by
comparison. No rush on this; it is written up for whenever the 5.4 / 5.5
line picks it up.

## Field report (JD, on the RG35, 2026-11-20)

- A Link to the Past (SNES): audio audibly messed up, crackling present
  in normal play.
- Pokemon Yellow (GB): audio fine, colors fine.
- Reference point from the v4 device runs: the GB demo held AUDIO XRUNS 0
  and OVERRUNS 0 for a full 90-frame and 128-epoch run. The audio path can
  be clean on this OS; the SNES core is where it breaks.

## How the OS feeds audio (architecture constraint)

HandshakeOS delivers phase-boundary PCM: audio is produced and consumed in
step with the handshake pacing cadence (the pacing dial: 16 Hz silent,
60 Hz DMG at 16.7 ms, 30 Hz, 24 Hz film). Any fix must stay inside that
discipline. Zero mutex stands. Spin locks are kernel level only, cautious,
per JD's ruling of 2026-11-20.

## Suspect 1: rate drift slip (prime suspect)

A real NTSC SNES does not run at exactly 60 Hz. Its frame rate is
60.0988 Hz, and the S-SMP / S-DSP audio chain produces 32,000 samples per
second. 32000 / 60.0988 = 532.4 samples per frame, which is not an integer.

If the core emits a fixed integer number of samples per frame, or the
pacing cadence holds the system at a flat 60.000 Hz while the core
believes it is at 60.0988, the audio buffer slowly drains (or fills) until
it slips a chunk. The listener hears that slip as a crackle or pop at
roughly regular intervals. The GB core never bites here because its rates
divide more kindly, which matches the field report exactly (GB clean,
SNES crackling).

Implementation direction:
- Carry a fractional sample accumulator: distribute 532 and 533 sample
  frames in the correct long-run ratio (Bresenham style), or resample the
  APU output onto the phase cadence explicitly.
- Decide which clock is master (core frame rate vs pacing dial) and make
  the other one follow it in the code, in one visible place, with the
  ratio written down as a named constant.

## Suspect 2: SPC700 burst execution

The SPC700 is its own processor on its own clock (nominally 1.024 MHz),
loosely coupled to the main CPU through the APU IO ports. If the core runs
the SPC700 in one burst per frame instead of interleaving it through the
frame, sample generation timing quantizes to frame boundaries. That reads
as grit or crackle, and it is worst on echo-heavy music because the S-DSP
echo buffer is a feedback path: timing error recirculates. A Link to the
Past's score is echo-heavy, which fits.

Implementation direction:
- Run the APU in slices interleaved with the CPU through the frame (for
  example at scanline granularity), sized so the DSP never waits a whole
  frame for its next input block.
- A/B test with the echo (FIR) path disabled: if the crackle vanishes
  with echo off, the fault is in the feedback path timing, not the mixer.

## Suspect 3: mixer clipping

If the eight S-DSP voices plus echo are summed too hot, loud passages
crackle while quiet ones stay clean. Zelda's fanfares and the title theme
would suffer first.

Implementation direction:
- Check the final mix headroom against the loudest known passage. If
  peaks clip, pull the master gain down (try -6 dB as the test point) and
  confirm the crackle pattern changes before touching anything else.

## Instrumentation plan (isolate by areas, then instrument)

Per JD's diagnostic doctrine: debug builds log, release builds strip.

1. Log AUDIO XRUNS and OVERRUNS per core over a 5-minute Zelda run. GB
   baseline is 0 / 0. If SNES xruns climb, the buffer is starving and
   Suspect 1 leads.
2. Log samples produced per frame for the first 600 frames. A periodic
   one-frame deficit or surplus is the drift slip caught in the act.
3. Log ring buffer fill level at each phase boundary: a slow sawtooth
   trending to empty (or full) confirms drift and gives its rate.
4. Ear test to split the suspects, one minute of play each:
   crackle at regular intervals = drift; constant fuzz = APU timing;
   crackle only when the music swells = clipping.

## Acceptance

- 5 minutes of A Link to the Past on device with no audible crackle in
  quiet and loud passages.
- AUDIO XRUNS 0 and OVERRUNS 0 for the run, same standard the GB demo
  already meets.
- The ledger hash and the run-verify gate stay exactly as green as they
  were before the change; an audio fix must not move the sim.
