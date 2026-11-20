#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Build hs-cores with the FROZEN hc 0.1.2 for x86_64 + ARM64 (cortex-a53),
# then audit every object (hc --audit-elf + nm lock-symbol scan).
set -euo pipefail
cd "$(dirname "$0")"
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
CFLAGS="-std=c11 -Wall -Wextra -Werror -O3 -Iinclude -Icommon -Igbc -Itests"
LOG=build/logs; mkdir -p "$LOG" build/x86_64 build/aarch64
: > "$LOG/compile.log"; : > "$LOG/audit.log"

COMPONENT=(common/hs_video.c gbc/gbc_cpu.c gbc/gbc_mmu.c gbc/gbc_ppu.c gbc/gbc_apu.c gbc/gbc_core.c gbc/gbc_compat_pal.c
           tests/hs_png.c tests/test_hs_gbc.c tests/test_hs_video.c)
RUNTIME=(tests/hs_test_host.c)   # monotonic clock for benchmark timing only

echo "hc: $HC ($($HC --version 2>&1 | head -1)) sha256=$(sha256sum "$HC" | cut -c1-16)"
compile() { # arch profile src
  local arch=$1 prof=$2 src=$3 obj tflags=""
  obj=build/$arch/$(basename "${src%.c}").o
  [ "$arch" = aarch64 ] && tflags="--target=aarch64-linux-gnu -mcpu=cortex-a53"
  echo "[$arch] $prof $src" >> "$LOG/compile.log"
  # shellcheck disable=SC2086
  $HC $tflags $prof $CFLAGS -c "$src" -o "$obj" >> "$LOG/compile.log" 2>&1
}
for arch in x86_64 aarch64; do
  for s in "${COMPONENT[@]}"; do compile $arch --profile=component "$s"; done
  for s in "${RUNTIME[@]}"; do compile $arch --allow-runtime "$s"; done
done
n_obj=$(ls build/x86_64/*.o build/aarch64/*.o | wc -l)
echo "compiled $n_obj objects (component: ${#COMPONENT[@]} TUs x 2 arches, runtime shim: ${#RUNTIME[@]} x 2)"

# ---- audits ----
BANNED='pthread|mutex|rwlock|spin_?lock|_cond_|barrier|sem_(wait|post|init|timedwait|trywait)|mtx_|cnd_|thrd_|futex|flock|lockf|__sync_lock|__atomic_compare_exchange|__aarch64_(cas|swp|ldadd)|setjmp|longjmp|\btime$|\brand$|srand|gettimeofday'
audit_fail=0; nm_hits=0
for o in build/x86_64/*.o build/aarch64/*.o; do
  prof=--profile=component
  case "$o" in *hs_test_host.o) prof=--allow-runtime;; esac
  if ! $HC $prof --audit-elf "$o" >> "$LOG/audit.log" 2>&1; then audit_fail=$((audit_fail+1)); fi
  NM=nm; case "$o" in build/aarch64/*) NM=aarch64-linux-gnu-nm;; esac
  h=$($NM "$o" | awk '{print $NF}' | grep -E -c "$BANNED" || true)
  if [ "$h" != 0 ]; then echo "  nm banned in $o:"; $NM "$o" | grep -E "$BANNED"; fi
  nm_hits=$((nm_hits+h))
  # init_array / ctors sections must not exist
  if objdump -h "$o" 2>/dev/null | grep -E -q '\.(init_array|fini_array|ctors|dtors)'; then echo "  ctor section in $o"; audit_fail=$((audit_fail+1)); fi
done
echo "hc --audit-elf: $((n_obj-audit_fail))/$n_obj PASS; nm banned lock/thread/clock/rng symbols: $nm_hits"

# ---- link ----
gcc -O3 -o build/x86_64/test_hs_gbc build/x86_64/{test_hs_gbc,hs_png,hs_test_host,gbc_cpu,gbc_mmu,gbc_ppu,gbc_apu,gbc_core,gbc_compat_pal,hs_video}.o
gcc -O3 -o build/x86_64/test_hs_video build/x86_64/{test_hs_video,hs_png,hs_test_host,hs_video}.o
aarch64-linux-gnu-gcc -O3 -static -o build/aarch64/test_hs_gbc build/aarch64/{test_hs_gbc,hs_png,hs_test_host,gbc_cpu,gbc_mmu,gbc_ppu,gbc_apu,gbc_core,gbc_compat_pal,hs_video}.o
aarch64-linux-gnu-gcc -O3 -static -o build/aarch64/test_hs_video build/aarch64/{test_hs_video,hs_png,hs_test_host,hs_video}.o
# final binaries: no thread / lock symbols pulled in from our code
for b in build/x86_64/test_hs_gbc build/aarch64/test_hs_gbc; do
  NM=nm; case "$b" in build/aarch64/*) NM=aarch64-linux-gnu-nm;; esac
  echo "$b: pthread_mutex symbols linked = $($NM "$b" 2>/dev/null | grep -c 'pthread_mutex_lock$' || true) (static libc stdio may carry its own internal locks)"
done
[ $audit_fail = 0 ] && [ $nm_hits = 0 ] && echo "BUILD+AUDIT OK"
