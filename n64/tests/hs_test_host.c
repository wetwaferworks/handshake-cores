/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 * hs_test_host.c - runtime-profile host shim for the test suites: the
 * monotonic clock used ONLY for benchmark timing (never fed into cores).
 */
#define _POSIX_C_SOURCE 199309L
#include <time.h>
#include <stdint.h>

uint64_t hs_host_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}
