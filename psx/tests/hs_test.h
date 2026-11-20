/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 * hs_test.h - tiny test harness shared by the hs-cores test suites.
 */
#ifndef HS_TEST_H
#define HS_TEST_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

/* hs_test_host.c (runtime profile: the only TU allowed to read a clock) */
uint64_t hs_host_now_ns(void);

/* hs_png.c (component) */
int hs_png_write_xrgb(const char *path, const uint32_t *px, uint32_t w, uint32_t h, size_t stride);

/* file helpers (component) */
size_t hs_read_file(const char *path, uint8_t *buf, size_t cap);

typedef struct hs_tstats {
    unsigned pass, fail;
} hs_tstats_t;

#define HS_CHECK(ts, cond, ...) do {                                   \
        if (cond) { (ts)->pass++; }                                     \
        else { (ts)->fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
               printf(__VA_ARGS__); printf("\n"); }                     \
    } while (0)

#endif
