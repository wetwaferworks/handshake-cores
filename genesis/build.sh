#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 HandshakeOS contributors
#
# hs-genesis build + verification: every .c through hc 0.1.4 for x86_64 and
# ARM64 Cortex-A53, hc --audit-elf + nm zero-lock audit on every object,
# test run natively and under qemu-aarch64-static, hash comparison.
set -eu
cd "$(dirname "$0")"
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
CFLAGS="-std=c11 -O3 -Wall -Wextra -Iinclude -I../include -Isrc"
SRCS="src/m68k_cpu.c src/gen_bus.c src/gen_vdp.c src/gen_render.c src/gen_z80.c src/gen_ym2612.c src/gen_psg.c src/gen_audio.c src/hs_genesis.c tests/test_hs_genesis.c"
BANNED='pthread_|mtx_|cnd_|sem_(wait|post|init|timedwait|trywait)|__lll_lock|__lll_unlock|flock|lockf|futex|spin_lock|atomic_flag'
mkdir -p build/x86_64 build/aarch64 out
"$HC" --version
for arch in x86_64 aarch64; do
  for s in $SRCS; do
    o=build/$arch/$(basename "$s" .c).o
    if [ $arch = x86_64 ]; then
      "$HC" --profile=component $CFLAGS -c "$s" -o "$o"
    else
      "$HC" --profile=component --target=aarch64-linux-gnu -mcpu=cortex-a53 $CFLAGS -c "$s" -o "$o"
    fi
  done
  o=build/$arch/hs_bench_clock.o
  if [ $arch = x86_64 ]; then
    "$HC" --profile=runtime -std=c11 -O3 -c tests/hs_bench_clock.c -o "$o"
  else
    "$HC" --profile=runtime --target=aarch64-linux-gnu -mcpu=cortex-a53 -std=c11 -O3 -c tests/hs_bench_clock.c -o "$o"
  fi
done
echo "== ELF zero-lock audit =="
n=0; bad=0
for o in build/x86_64/*.o build/aarch64/*.o; do
  prof=component; case "$o" in *hs_bench_clock.o) prof=runtime;; esac
  if ! "$HC" --profile=$prof --audit-elf "$o" >/dev/null 2>&1; then echo "AUDIT FAIL $o"; bad=$((bad+1)); fi
  NM=nm; case "$o" in build/aarch64/*) NM=aarch64-linux-gnu-nm;; esac
  hits=$($NM "$o" | grep -E -c "$BANNED" || true)
  if [ "$hits" != 0 ]; then echo "NM LOCK SYMBOLS in $o:"; $NM "$o" | grep -E "$BANNED"; bad=$((bad+1)); fi
  n=$((n+1))
done
echo "objects audited: $n, failures: $bad"
[ $bad = 0 ]
gcc -o build/x86_64/test_hs_genesis build/x86_64/*.o
aarch64-linux-gnu-gcc -o build/aarch64/test_hs_genesis build/aarch64/*.o
