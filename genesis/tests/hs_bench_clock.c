/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_bench_clock.c - benchmark-only monotonic clock for the hs-genesis test
 * harness. Compiled with `hc --profile=runtime` (audited telemetry profile);
 * no emulator code ever calls it, so the cores stay clock-free.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <time.h>

uint64_t hs_bench_now_ns(void);

uint64_t hs_bench_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
