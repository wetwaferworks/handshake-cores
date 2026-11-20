/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_pf_sys.c - runtime-profile OS table for n64_pf.c: open / ioctl /
 * mmap / munmap / close on the Panfrost render node, and CLOCK_MONOTONIC for
 * DRM_IOCTL_PANFROST_WAIT_BO's absolute timeout. Nothing else. Used only by
 * tools/hs_pf_probe.c (the on-device probe); the core and the tests never
 * link it.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include "n64_pf.h"

static int sys_open(void *u, const char *path) { (void)u; return open(path, O_RDWR | O_CLOEXEC); }
static int sys_ioctl(void *u, int fd, unsigned long req, void *arg)
{
    int r;
    (void)u;
    do { r = ioctl(fd, req, arg); } while (r == -1 && (errno == EINTR || errno == EAGAIN));
    return r == 0 ? 0 : -errno;
}
static void *sys_map(void *u, int fd, uint64_t off, size_t len)
{
    void *p;
    (void)u;
    p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)off);
    return p == MAP_FAILED ? NULL : p;
}
static void sys_unmap(void *u, void *p, size_t len) { (void)u; munmap(p, len); }
static void sys_close(void *u, int fd) { (void)u; close(fd); }
static uint64_t sys_now(void *u)
{
    struct timespec t;
    (void)u;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

void hs_pf_sys_os(hs_pf_os_t *os)
{
    os->user = NULL; os->open_dev = sys_open; os->ioctl = sys_ioctl; os->map = sys_map;
    os->unmap = sys_unmap; os->close_dev = sys_close; os->now_ns = sys_now;
}
