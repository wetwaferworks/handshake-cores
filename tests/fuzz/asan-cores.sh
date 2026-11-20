#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Host ASan+UBSan build and run of every hs-cores test suite plus the PNG-reader fuzzer.
# Review tooling only (plain gcc + sanitizer runtime; nothing here is shipped or HC-audited).
# Usage: tests/fuzz/asan-cores.sh OUTDIR     (HS_SAN_RECOVER=1 keeps going after a UBSan report)
set -uo pipefail
cd "$(dirname "$0")/../.."
ROOT=$PWD
OUT=${1:?OUTDIR}; mkdir -p "$OUT"
REC="-fno-sanitize-recover=undefined"; [ "${HS_SAN_RECOVER:-0}" = 1 ] && REC="-fsanitize-recover=undefined"
SAN="-fsanitize=address,undefined $REC -fno-omit-frame-pointer -g -O1"
CF="-std=c11 -D_GNU_SOURCE -ffp-contract=off -w"
export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=1}
export UBSAN_OPTIONS=${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=1}
fail=0
build() { # name dir "includes" "sources"
  local name=$1 dir=$2 inc=$3 srcs=$4 objs="" s o
  mkdir -p "$OUT/$name"
  for s in $srcs; do
    o="$OUT/$name/$(echo "$s" | tr '/.' '__').o"
    # shellcheck disable=SC2086
    ( cd "$dir" && gcc $CF $SAN $inc -c "$s" -o "$o" ) || { echo "COMPILE FAIL $name $s"; fail=1; return 1; }
    objs="$objs $o"
  done
  # shellcheck disable=SC2086
  gcc $SAN -o "$OUT/$name/$name" $objs -lm -pthread || { echo "LINK FAIL $name"; fail=1; return 1; }
}
run() { # name dir args...
  local name=$1 dir=$2; shift 2
  mkdir -p "$OUT/$name/out"
  ( cd "$dir" && "$OUT/$name/$name" "$@" ) > "$OUT/$name.log" 2>&1
  local rc=$?
  local ub; ub=$(grep -c 'runtime error' "$OUT/$name.log"); local as; as=$(grep -c 'ERROR: AddressSanitizer\|ERROR: LeakSanitizer' "$OUT/$name.log")
  echo "ASAN_RUN $name rc=$rc ubsan_reports=$ub asan_reports=$as"
  [ "$rc" = 0 ] && [ "$as" = 0 ] || fail=1
}
G="common/hs_video.c gbc/gbc_cpu.c gbc/gbc_mmu.c gbc/gbc_ppu.c gbc/gbc_apu.c gbc/gbc_core.c tests/hs_png.c tests/hs_test_host.c"
build test_hs_gbc "$ROOT" "-Iinclude -Icommon -Igbc -Itests" "$G tests/test_hs_gbc.c" && run test_hs_gbc "$ROOT" --out="$OUT/test_hs_gbc/out" --roms="$ROOT/../handshakeos/roms"
build test_hs_video "$ROOT" "-Iinclude -Icommon -Igbc -Itests" "$G tests/test_hs_video.c" && run test_hs_video "$ROOT" --out="$OUT/test_hs_video/out"
build test_hs_gba "$ROOT/gba" "-Iinclude -Isrc -Itests -I../include" "$(cd gba && echo src/*.c) tests/test_hs_gba.c" && run test_hs_gba "$ROOT/gba" --out "$OUT/test_hs_gba/out"
build test_hs_snes "$ROOT/snes" "-I. -Itests -I../include" "$(cd snes && echo hs_snes.c snes_*.c) tests/snes_testroms.c tests/png_write.c tests/test_hs_snes.c" && run test_hs_snes "$ROOT/snes" --out="$OUT/test_hs_snes/out"
build test_hs_genesis "$ROOT/genesis" "-Iinclude -I../include -Isrc -Itests" "$(cd genesis && echo src/*.c) tests/test_hs_genesis.c tests/hs_bench_clock.c" && run test_hs_genesis "$ROOT/genesis" "$OUT/test_hs_genesis/out/genesis_preview.png" 60
build test_hs_psx "$ROOT/psx" "-Iinclude -I../include -I../common -Isrc -Itests" "$(cd psx && echo src/*.c) ../common/hs_video.c tests/hs_png.c tests/hs_test_host.c tests/test_hs_psx.c" && run test_hs_psx "$ROOT/psx" --out="$OUT/test_hs_psx/out"
build test_hs_n64 "$ROOT/n64" "-Iinclude -I../include -I../common -Isrc -Itests" "$(cd n64 && echo src/*.c) ../common/hs_video.c tests/hs_png.c tests/hs_test_host.c tests/test_hs_n64.c" && run test_hs_n64 "$ROOT/n64" --out="$OUT/test_hs_n64/out"
build fuzz_png "$ROOT" "" "tests/fuzz/fuzz_png.c" && run fuzz_png "$OUT" "$ROOT/gba_mode0_640x480.png" 3000
echo "ASAN_CORES fail=$fail"
exit $fail
