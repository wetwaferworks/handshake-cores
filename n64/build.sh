#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# hs-n64 build: every TU through HC v0.1.4 (--profile=component) for x86_64 and
# ARM64 cortex-a53 -O3; only the tiny host timer shim uses --allow-runtime.
# Then hc --audit-elf + nm scan of every object for banned lock/thread/clock/rng
# symbols, a ctor-section scan, a mutable-global scan, and link the test binaries.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build/tmp; export TMPDIR=$PWD/build/tmp   # hard rule: no scratch files outside the tree
HC=${HC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/hc/build/current/hc}   # override with HC=/path/to/hc
CFLAGS="-std=c11 -O3 -Wall -Wextra -Werror -ffp-contract=off -Iinclude -I../include -I../common -Isrc -Itests"
LOG=build/logs; mkdir -p "$LOG" build/x86_64 build/aarch64
rm -f build/x86_64/*.o build/aarch64/*.o
: > "$LOG/compile.log"; : > "$LOG/audit.log"

# regenerate the original demo ROM (deterministic; assembled by tools/mk_n64_demo.c, pure C11, built with HC)
mkdir -p build/tools
$HC --allow-runtime -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c tools/mk_n64_demo.c -o build/tools/mk_n64_demo.o >> "$LOG/compile.log" 2>&1
gcc -o build/tools/mk_n64_demo build/tools/mk_n64_demo.o -lm
build/tools/mk_n64_demo roms/hs_n64_demo.z64 >> "$LOG/compile.log"

COMPONENT=(src/n64_cpu.c src/n64_bus.c src/n64_rsp.c src/n64_rdp.c src/n64_audio.c src/n64_save.c src/hs_n64.c
           src/n64_pf.c ../common/hs_video.c tests/hs_png.c tests/test_hs_n64.c tests/bench_n64.c tools/hs_pf_probe.c)
RUNTIME=(tests/hs_test_host.c    # CLOCK_MONOTONIC for benchmark timing only (never fed to the core)
         src/n64_pf_sys.c)       # open/ioctl/mmap on /dev/dri/renderD128 + WAIT_BO deadline, on-device probe tool only

echo "hc: $HC ($($HC --version 2>&1 | head -1)) sha256=$(sha256sum "$HC" | cut -c1-16)"
compile() { # arch profile src
  local arch=$1 prof=$2 src=$3 obj tflags=""
  obj=build/$arch/$(basename "${src%.c}").o
  [ "$arch" = aarch64 ] && tflags="--target=aarch64-linux-gnu -mcpu=cortex-a53"
  echo "[$arch] $prof $src" >> "$LOG/compile.log"
  # shellcheck disable=SC2086
  $HC $tflags $prof $CFLAGS -c "$src" -o "$obj" >> "$LOG/compile.log" 2>&1 || { echo "COMPILE FAIL [$arch] $src"; tail -30 "$LOG/compile.log"; exit 1; }
}
for arch in x86_64 aarch64; do
  for s in "${COMPONENT[@]}"; do compile $arch --profile=component "$s"; done
  for s in "${RUNTIME[@]}"; do compile $arch --allow-runtime "$s"; done
done
n_obj=$(ls build/x86_64/*.o build/aarch64/*.o | wc -l)
echo "compiled $n_obj objects (component: ${#COMPONENT[@]} TUs x 2 arches, runtime shim: ${#RUNTIME[@]} x 2)"

BANNED='pthread|mutex|rwlock|spin_?lock|_cond_|barrier|sem_(wait|post|init|timedwait|trywait)|mtx_|cnd_|thrd_|futex|flock|lockf|__sync_lock|__atomic_compare_exchange|__aarch64_(cas|swp|ldadd)|setjmp|longjmp|\btime$|\brand$|srand|random|gettimeofday|clock_gettime|rdtsc|getrandom|arc4random'
audit_fail=0; nm_hits=0; n_comp=0
for o in build/x86_64/*.o build/aarch64/*.o; do
  prof=--profile=component; rt=0
  case "$o" in *hs_test_host.o|*n64_pf_sys.o) prof=--allow-runtime; rt=1;; esac
  if ! $HC $prof --audit-elf "$o" >> "$LOG/audit.log" 2>&1; then audit_fail=$((audit_fail+1)); echo "  audit FAIL $o"; fi
  NM=nm; case "$o" in build/aarch64/*) NM=aarch64-linux-gnu-nm;; esac
  if [ $rt = 0 ]; then
    n_comp=$((n_comp+1))
    h=$($NM "$o" | awk '{print $NF}' | grep -E -c "$BANNED" || true)
    if [ "$h" != 0 ]; then echo "  nm banned in $o:"; $NM "$o" | grep -E "$BANNED"; fi
    nm_hits=$((nm_hits+h))
  fi
  if objdump -h "$o" 2>/dev/null | grep -E -q '\.(init_array|fini_array|ctors|dtors)'; then echo "  ctor section in $o"; audit_fail=$((audit_fail+1)); fi
done
# mutable globals in core objects: no object symbols in writable .data/.bss/COMMON
glob=0
for o in build/x86_64/n64_*.o build/x86_64/hs_n64.o build/aarch64/n64_*.o build/aarch64/hs_n64.o; do   # n64_pf*.o included
  g=$(objdump -t "$o" | awk '$3=="O" || $2=="O" {print}' | grep -E '\s(\.data|\.bss|\*COM\*)(\.[A-Za-z0-9_.]*)?\s' | grep -v '\.data\.rel\.ro' | wc -l || true)
  if [ "$g" != 0 ]; then echo "  mutable global in $o:"; objdump -t "$o" | grep -E '\s(\.data|\.bss|\*COM\*)' | grep -v rel.ro; fi
  glob=$((glob+g))
done
echo "hc --audit-elf: $((n_obj-audit_fail))/$n_obj PASS; nm banned lock/thread/clock/rng symbols in $n_comp component objects: $nm_hits; mutable .data/.bss symbols in core objects: $glob"

CORE="n64_cpu n64_bus n64_rsp n64_rdp n64_audio n64_save n64_pf hs_n64 hs_video"
objs() { local a=$1 m=${2:-test_hs_n64} o=""; for n in $m hs_png hs_test_host $CORE; do o="$o build/$a/$n.o"; done; echo $o; }
# shellcheck disable=SC2046
gcc -O3 -o build/x86_64/test_hs_n64 $(objs x86_64) -lm
# shellcheck disable=SC2046
aarch64-linux-gnu-gcc -O3 -static -o build/aarch64/test_hs_n64 $(objs aarch64) -lm
# shellcheck disable=SC2046
gcc -O3 -o build/x86_64/bench_n64 $(objs x86_64 bench_n64) -lm
# shellcheck disable=SC2046
aarch64-linux-gnu-gcc -O3 -static -o build/aarch64/bench_n64 $(objs aarch64 bench_n64) -lm
# on-device Panfrost probe (component main + n64_pf + runtime OS table); runs on the RG35XX
gcc -O2 -o build/x86_64/hs_pf_probe build/x86_64/hs_pf_probe.o build/x86_64/n64_pf.o build/x86_64/n64_pf_sys.o
aarch64-linux-gnu-gcc -O2 -static -o build/aarch64/hs_pf_probe build/aarch64/hs_pf_probe.o build/aarch64/n64_pf.o build/aarch64/n64_pf_sys.o
wc -l src/*.c src/*.h include/*.h tests/test_hs_n64.c tests/bench_n64.c tools/*.c | tail -1 | sed 's/^/source lines (src+include+test+tools): /'
[ $audit_fail = 0 ] && [ $nm_hits = 0 ] && [ $glob = 0 ] && echo "BUILD+AUDIT OK"
