#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# hs-gba: compile every .c with hc --profile=component (x86_64 + ARM64 Cortex-A53),
# audit ELF objects (hc --audit-elf + nm), link, run tests natively and under qemu.
set -euo pipefail
cd "$(dirname "$0")"
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
CFLAGS="-std=c11 -Wall -Wextra -Werror -O3 -Iinclude -Isrc -Itests"
SRCS=(src/gba_cpu.c src/gba_bus.c src/gba_bios.c src/gba_ppu.c src/gba_apu.c src/gba_core.c tests/test_hs_gba.c)
LOG=build/logs; mkdir -p build/x86_64 build/aarch64 "$LOG" out/x86_64 out/aarch64
: > "$LOG/compile.txt"; : > "$LOG/audit.txt"; : > "$LOG/nm-locks.txt"
"$HC" --version >> "$LOG/compile.txt" 2>&1
for s in "${SRCS[@]}"; do
  b=$(basename "${s%.c}")
  echo "[x86_64 ] hc --profile=component $s" | tee -a "$LOG/compile.txt"
  "$HC" --profile=component $CFLAGS -c "$s" -o "build/x86_64/$b.o" 2>&1 | tee -a "$LOG/compile.txt"
  echo "[aarch64] hc --profile=component --target=aarch64-linux-gnu -mcpu=cortex-a53 -O3 $s" | tee -a "$LOG/compile.txt"
  "$HC" --profile=component --target=aarch64-linux-gnu $CFLAGS -mcpu=cortex-a53 -c "$s" -o "build/aarch64/$b.o" 2>&1 | tee -a "$LOG/compile.txt"
done
BANNED='pthread|mutex|spin_?lock|rwlock|pthread_cond|barrier|sem_(wait|post|init|timedwait|trywait)|flock|lockf|futex|thrd_|mtx_|cnd_|__atomic|__sync|clock_gettime|gettimeofday|\btime\b|\brand\b|setjmp|longjmp'
fail=0
for o in build/x86_64/*.o build/aarch64/*.o; do
  "$HC" --audit-elf "$o" >> "$LOG/audit.txt" 2>&1 || fail=1
  hits=$(nm -A "$o" 2>/dev/null | grep -Ei "$BANNED" || true)
  if [ -n "$hits" ]; then echo "$hits" >> "$LOG/nm-locks.txt"; fail=1; fi
  # ctor/dtor sections
  if readelf -S "$o" 2>/dev/null | grep -Eq '\.(init_array|fini_array|ctors|dtors|preinit_array)'; then echo "CTOR $o" >> "$LOG/nm-locks.txt"; fail=1; fi
done
echo "audit objects: $(ls build/x86_64/*.o build/aarch64/*.o | wc -l), PASS lines: $(grep -c 'audit PASS' "$LOG/audit.txt")" | tee -a "$LOG/audit.txt"
echo "nm banned-symbol hits: $(grep -c . "$LOG/nm-locks.txt" || true)" | tee -a "$LOG/audit.txt"
[ "$fail" = 0 ] || { echo "AUDIT FAILED"; exit 1; }
gcc -O3 -o build/x86_64/test_hs_gba build/x86_64/*.o
aarch64-linux-gnu-gcc -static -O3 -o build/aarch64/test_hs_gba build/aarch64/*.o
./build/x86_64/test_hs_gba --out out/x86_64 | tee "$LOG/test-x86_64.txt"
qemu-aarch64-static ./build/aarch64/test_hs_gba --out out/aarch64 | tee "$LOG/test-aarch64.txt"
grep -E '^ROM' "$LOG/test-x86_64.txt" > "$LOG/hash-x86_64.txt"
grep -E '^ROM' "$LOG/test-aarch64.txt" > "$LOG/hash-aarch64.txt"
if cmp -s "$LOG/hash-x86_64.txt" "$LOG/hash-aarch64.txt"; then echo "HASHES: x86_64 == aarch64 (bit-identical)"; else echo "HASH MISMATCH"; diff "$LOG/hash-x86_64.txt" "$LOG/hash-aarch64.txt"; exit 1; fi
for f in gba_mode0_640x480.png gba_mode2_affine_640x480.png gba_mode3_bitmap_640x480.png; do
  if cmp -s "out/x86_64/$f" "out/aarch64/$f"; then echo "PNG identical across arches: $f"; else echo "PNG DIFF: $f"; exit 1; fi
done
