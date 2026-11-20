#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Full hs-cores gate: build+audit, x86_64 tests, ARM64 (qemu-aarch64-static)
# tests, cross-arch hash comparison, acid2 pixel compare.
set -uo pipefail
cd "$(dirname "$0")"
./build.sh | tee build/logs/build.txt
grep -q "BUILD+AUDIT OK" build/logs/build.txt || { echo "BUILD/AUDIT FAILED"; exit 1; }
mkdir -p out/x86_64 out/aarch64
L=build/logs
./build/x86_64/test_hs_gbc   --out=out/x86_64 > $L/test_hs_gbc.x86_64.log 2>&1;   echo "rc=$?" >> $L/test_hs_gbc.x86_64.log
./build/x86_64/test_hs_video --out=out/x86_64 > $L/test_hs_video.x86_64.log 2>&1; echo "rc=$?" >> $L/test_hs_video.x86_64.log
if [ "${SKIP_QEMU:-0}" != 1 ]; then
  qemu-aarch64-static ./build/aarch64/test_hs_gbc   --out=out/aarch64 > $L/test_hs_gbc.aarch64.log 2>&1;   echo "rc=$?" >> $L/test_hs_gbc.aarch64.log
  qemu-aarch64-static ./build/aarch64/test_hs_video --out=out/aarch64 > $L/test_hs_video.aarch64.log 2>&1; echo "rc=$?" >> $L/test_hs_video.aarch64.log
fi
for f in $L/test_hs_*.log; do echo "$(basename $f): $(grep RESULT $f) $(tail -1 $f)"; done
# cross-arch hash comparison
grep -h '^HASH' $L/test_hs_gbc.x86_64.log $L/test_hs_video.x86_64.log | sort > $L/hashes.x86_64.txt
grep -h '^HASH' $L/test_hs_gbc.aarch64.log $L/test_hs_video.aarch64.log | sort > $L/hashes.aarch64.txt
n=$(wc -l < $L/hashes.x86_64.txt)
if diff -q $L/hashes.x86_64.txt $L/hashes.aarch64.txt >/dev/null; then
  echo "CROSS-ARCH: $n/$n hashes bit-identical (x86_64 == ARM64)"
else
  echo "CROSS-ARCH: MISMATCH"; diff $L/hashes.x86_64.txt $L/hashes.aarch64.txt
fi
for a in x86_64 aarch64; do
  for p in out/$a/*.png; do cmp -s "$p" "out/x86_64/$(basename $p)" || echo "PNG differs: $p"; done
done
# acid2 pixel compare against the official reference images (tests/acid2_compare.c, pure C11, built with HC)
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
mkdir -p build/tools
$HC --allow-runtime -std=c11 -O2 -Wall -Wextra -Werror -c tests/acid2_compare.c -o build/tools/acid2_compare.o
gcc -o build/tools/acid2_compare build/tools/acid2_compare.o
build/tools/acid2_compare out/x86_64 tests/ext-roms
build/tools/acid2_compare out/aarch64 tests/ext-roms
