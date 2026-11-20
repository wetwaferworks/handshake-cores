/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * bench_report - formats one hs-gba benchmark line (pure C11). The launcher
 * build/bench.sh times each run (best of 3, nanoseconds) and passes:
 *   bench_report ARCH WORKLOAD N Z_NS CPU_NS VIDEO_NS AUDIO_NS
 * where Z is the --bench 0 startup run and CPU/VIDEO/AUDIO are the c|cv|cva (or b|bv|bva) runs. */
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    double n, z, t[3];
    int i;
    if (argc != 8) { fputs("usage: bench_report ARCH WORKLOAD N Z_NS CPU_NS VIDEO_NS AUDIO_NS\n", stderr); return 2; }
    n = (double)strtol(argv[3], NULL, 10);
    z = (double)strtoll(argv[4], NULL, 10) / 1e9;
    for (i = 0; i < 3; i++) t[i] = (double)strtoll(argv[5 + i], NULL, 10) / 1e9 - z;
    printf("%s %s N=%s: total %.0f fps (%.3f ms/f) | cpu %.3f | video %.3f | audio %.3f ms/f | startup %.1f ms\n",
           argv[1], argv[2], argv[3], n / t[2], t[2] / n * 1e3, t[0] / n * 1e3, (t[1] - t[0]) / n * 1e3,
           (t[2] - t[1]) / n * 1e3, z * 1e3);
    return fflush(stdout) ? 1 : 0;
}
