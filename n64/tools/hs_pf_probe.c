/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * hs_pf_probe.c - on-device Panfrost probe for the hs-n64 Mali path.
 *   hs_pf_probe [/dev/dri/renderD128]
 * 1. opens the render node, checks the DRM driver is "panfrost", reads the
 *    GPU parameters and reports whether it is a Mali-G31 (0x7093);
 * 2. allocates a 64x64 XRGB image BO and fills it with a WRITE_VALUE job
 *    chain (one job per 8 bytes), waits, checks every job's completion
 *    status and reads back every pixel.
 * Exit 0 only if both steps pass. Prints one "PF ..." line per fact.
 */
#include <stdio.h>
#include <string.h>
#include "n64_pf.h"

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "/dev/dri/renderD128";
    hs_pf_os_t os;
    hs_pf_dev_t d;
    hs_pf_bo_t img;
    hs_pf_fill_result_t fr;
    int r;
    hs_pf_sys_os(&os);
    r = hs_pf_open(&d, &os, path);
    if (r != HS_PF_OK) {
        printf("PF probe FAIL %s: %s\n", path, r == HS_PF_ENODEV ? "cannot open device" :
               (r == HS_PF_ENOTPANFROST ? "DRM driver is not panfrost" : "GET_PARAM failed"));
        return 2;
    }
    printf("PF driver %s %d.%d.%d\n", d.info.driver, d.info.drm_major, d.info.drm_minor, d.info.drm_patch);
    printf("PF gpu_prod_id 0x%04llx (%s) revision 0x%llx\n", (unsigned long long)d.info.prod_id,
           d.info.is_g31 ? "Mali-G31" : "not G31", (unsigned long long)d.info.revision);
    printf("PF shader_present 0x%llx (%u cores) tiler 0x%llx l2 0x%llx js 0x%llx max_threads %llu\n",
           (unsigned long long)d.info.shader_present, d.info.shader_cores, (unsigned long long)d.info.tiler_present,
           (unsigned long long)d.info.l2_present, (unsigned long long)d.info.js_present, (unsigned long long)d.info.max_threads);
    printf("PF core_features 0x%llx thread_features 0x%llx texture_features0 0x%llx mmu_features 0x%llx\n",
           (unsigned long long)d.info.core_features, (unsigned long long)d.info.thread_features,
           (unsigned long long)d.info.texture_features0, (unsigned long long)d.info.mmu_features);
    r = hs_pf_bo_new(&d, 64u * 64u * 4u, HS_PF_BO_NOEXEC, &img);
    if (r != HS_PF_OK) { printf("PF bo FAIL %d\n", r); hs_pf_close(&d); return 3; }
    memset(img.cpu, 0, img.size);
    printf("PF bo handle %u size %u gpu_va 0x%llx\n", img.handle, img.size, (unsigned long long)img.gpu_va);
    r = hs_pf_fill_write_value(&d, &img, 64u, 64u, 64u, 0xFF3C8859u, &fr);
    printf("PF write_value_fill %s r=%d jobs=%u done=%u first_status=0x%x bad_pixels=%u\n",
           r == HS_PF_OK ? "PASS" : "FAIL", r, fr.jobs, fr.done_jobs, fr.first_status, fr.bad_pixels);
    hs_pf_bo_free(&d, &img);
    hs_pf_close(&d);
    return r == HS_PF_OK && d.info.is_g31 ? 0 : 4;
}
