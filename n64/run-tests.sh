#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Run test_hs_n64 natively (x86_64) and under qemu-aarch64-static (cortex-a53),
# then verify bit-identical state/frame hashes and byte-identical PNG/WAV outputs.
set -euo pipefail
cd "$(dirname "$0")"
LOG=build/logs; mkdir -p "$LOG" out/x86_64 out/aarch64
./build.sh
rc=0
./build/x86_64/test_hs_n64 --out=out/x86_64 > "$LOG/test_x86_64.log" 2>&1 || rc=1
qemu-aarch64-static -cpu cortex-a53 ./build/aarch64/test_hs_n64 --out=out/aarch64 > "$LOG/test_aarch64.log" 2>&1 || rc=1
for a in x86_64 aarch64; do echo "[$a] $(grep '^RESULT' $LOG/test_$a.log || echo 'RESULT missing')"; grep FAIL $LOG/test_$a.log || true; done
grep '^HASH' $LOG/test_x86_64.log > $LOG/hash_x86_64.txt
grep '^HASH' $LOG/test_aarch64.log > $LOG/hash_aarch64.txt
if cmp -s $LOG/hash_x86_64.txt $LOG/hash_aarch64.txt; then
  echo "cross-arch HASH lines: $(wc -l < $LOG/hash_x86_64.txt) identical"
else echo "cross-arch HASH MISMATCH"; diff $LOG/hash_x86_64.txt $LOG/hash_aarch64.txt || true; rc=1; fi
same=0; tot=0
for f in out/x86_64/*.png out/x86_64/*.wav; do
  tot=$((tot+1)); b=$(basename "$f")
  if cmp -s "$f" "out/aarch64/$b"; then same=$((same+1)); else echo "  DIFF $b"; rc=1; fi
done
echo "cross-arch output files byte-identical: $same/$tot"
sha256sum out/x86_64/*.png out/x86_64/*.wav | sed 's#out/x86_64/##' > $LOG/sha256_outputs.txt
cat $LOG/sha256_outputs.txt
for a in x86_64 aarch64; do echo "== $a =="; grep -E '^(TIME|RMS|INFO)' $LOG/test_$a.log; done
exit $rc
