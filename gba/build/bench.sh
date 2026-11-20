#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# hs-gba benchmark launcher: best-of-3 wall timing of test_hs_gba --bench per phase,
# formatted by tools/bench_report (pure C). Run from hs-cores/gba after build.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
REPORT=build/x86_64/bench_report
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
"$HC" --allow-runtime -std=c11 -O2 -Wall -Wextra -Werror -c tools/bench_report.c -o build/x86_64/bench_report.o
gcc -o "$REPORT" build/x86_64/bench_report.o
BEST=0; OUT=""
run() { # cmd...
  local i s e d; BEST=""
  for i in 1 2 3; do
    s=$(date +%s%N); OUT=$("$@"); e=$(date +%s%N); d=$((e - s))
    if [ -z "$BEST" ] || [ "$d" -lt "$BEST" ]; then BEST=$d; fi
  done
}
for arch in x86_64 aarch64; do
  if [ $arch = x86_64 ]; then pre=(); N=3000; b=build/x86_64/test_hs_gba; label=x86_64
  else pre=(qemu-aarch64-static); N=600; b=build/aarch64/test_hs_gba; label="aarch64(qemu-user)"; fi
  run "${pre[@]}" $b --bench 0 c; z=$BEST
  for wl in rom bus; do
    if [ $wl = rom ]; then name="ROM1 (halting game-like)"; ph=(c cv cva); else name="BUSY (CPU never halts)"; ph=(b bv bva); fi
    run "${pre[@]}" $b --bench $N ${ph[0]}; c0=$BEST
    run "${pre[@]}" $b --bench $N ${ph[1]}; c1=$BEST
    run "${pre[@]}" $b --bench $N ${ph[2]}; c2=$BEST; last=$OUT
    "$REPORT" "$label" "$name" $N $z $c0 $c1 $c2
    echo "    $last"
  done
done
