#!/bin/bash
# SPDX-License-Identifier: MIT
# hs-snes build: compiles EVERY .c with the frozen hc 0.1.2 (--profile=component)
# for x86_64 (-O3) and ARM64 (-mcpu=cortex-a53 -O3), audits every object with
# `hc --audit-elf` + nm, then links test/bench binaries.
set -u
cd "$(dirname "$0")"
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
CF="-std=c11 -Wall -Wextra -Werror -D_GNU_SOURCE"
CORE="hs_snes.c snes_cpu.c snes_bus.c snes_ppu.c snes_apu.c"
TESTS="tests/snes_testroms.c tests/png_write.c tests/test_hs_snes.c"
RT="tests/bench_hs_snes.c tests/bench_mt_hs_snes.c"   # host timing harnesses: --profile=runtime (CLOCK_MONOTONIC, fork + release/acquire, no locks)
BANNED='pthread_mutex|pthread_rwlock|pthread_spin|pthread_cond|pthread_barrier|sem_wait|sem_post|sem_init|sem_timedwait|sem_trywait|mtx_lock|mtx_unlock|mtx_init|cnd_wait|cnd_signal|futex|flock|lockf|__sync_lock|__atomic_compare_exchange|__aarch64_cas|__aarch64_swp|__aarch64_ldadd|SDL_LockMutex|SDL_UnlockMutex'
mkdir -p logs
rc=0
for arch in x86_64 aarch64; do
  out=build/$arch; mkdir -p $out; log=logs/hc_$arch.log; : > $log
  if [ $arch = aarch64 ]; then T="--target=aarch64-linux-gnu -mcpu=cortex-a53 -O3"; NM=aarch64-linux-gnu-nm; CC=aarch64-linux-gnu-gcc;
  else T="-O3"; NM=nm; CC=gcc; fi
  ok=0; fail=0
  for f in $CORE $TESTS; do
    o=$out/$(basename ${f%.c}).o
    echo "=== $f (component)" >> $log
    if $HC --profile=component $T $CF -c $f -o $o >> $log 2>&1; then ok=$((ok+1)); else fail=$((fail+1)); echo "HC FAIL $arch $f"; rc=1; fi
  done
  for f in $RT; do
    o=$out/$(basename ${f%.c}).o
    echo "=== $f (runtime)" >> $log
    if $HC --profile=runtime $T $CF -c $f -o $o >> $log 2>&1; then ok=$((ok+1)); else fail=$((fail+1)); echo "HC FAIL $arch $f"; rc=1; fi
  done
  objs=$(ls $out/*.o)
  : > logs/audit_$arch.log
  af=0; for o in $objs; do
    prof=component; case $o in *bench_*) prof=runtime;; esac
    $HC --profile=$prof --audit-elf $o >> logs/audit_$arch.log 2>&1 || { af=$((af+1)); echo "AUDIT FAIL $o"; }
  done
  nb=$($NM $objs 2>/dev/null | grep -E -c "$BANNED")
  echo "[$arch] hc compiled_ok=$ok failed=$fail  hc_audit_elf_fail=$af  nm_banned_lock_symbols=$nb  objects=$(echo $objs | wc -w)"
  coreobjs="$out/hs_snes.o $out/snes_cpu.o $out/snes_bus.o $out/snes_ppu.o $out/snes_apu.o"
  # aarch64 binaries are static so qemu-aarch64-static does not need a guest ld.so.
  LDFLAGS=
  if [ $arch = aarch64 ]; then LDFLAGS=-static; fi
  $CC $LDFLAGS -o $out/test_hs_snes $coreobjs $out/snes_testroms.o $out/png_write.o $out/test_hs_snes.o || rc=1
  $CC $LDFLAGS -o $out/bench_hs_snes $coreobjs $out/snes_testroms.o $out/bench_hs_snes.o || rc=1
  $CC $LDFLAGS -o $out/bench_mt_hs_snes $coreobjs $out/snes_testroms.o $out/bench_mt_hs_snes.o || rc=1
done
exit $rc
