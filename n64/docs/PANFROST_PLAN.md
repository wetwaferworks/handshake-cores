# hs-n64 on Mali-G31 (Panfrost): feasibility and plan

Status date: 2026-11-20. Target: RG35XX 2024, Allwinner H700, Mali-G31 MP2 (Bifrost v7, GPU_ID 0x7093),
mainline Linux 7.3.5 `panfrost` DRM driver, `/dev/dri/renderD128`. No Mesa, no GLES. Pure C11
talks to the kernel through `DRM_IOCTL_PANFROST_*`.

## 1. What exists now (step 1, in this tree)

| Piece | File | Profile | Validated by |
|---|---|---|---|
| UAPI structs + ioctl numbers (VERSION, GEM_CLOSE, SUBMIT 48 B, WAIT_BO, CREATE_BO, MMAP_BO, GET_PARAM) | `src/n64_pf.h` | header | `_Static_assert` sizes + test vs hand-computed hex |
| Device probe: driver name must be `panfrost`, 11 GET_PARAMs, core count, G31 check | `src/n64_pf.c` | component | mock device test |
| BO create / mmap / close | `src/n64_pf.c` | component | mock device test |
| 64-bit job header + WRITE_VALUE encoders | `src/n64_pf.c` | component | byte-layout test |
| GPU fill: chain of WRITE_VALUE jobs (IMM64, one per 2 pixels), submit, WAIT_BO, check every job's status = DONE, read back every pixel, check nothing outside the rectangle changed | `src/n64_pf.c` | component | mock device test (64x64 in an 80-stride BO, 2048 jobs) |
| Mock Panfrost device that executes WRITE_VALUE chains in an 8 MiB arena | `src/n64_pf.c` | component | used by the tests |
| Kernel OS table (open / ioctl / mmap / munmap / close / CLOCK_MONOTONIC for WAIT_BO's absolute deadline) | `src/n64_pf_sys.c` | runtime | builds; on WSL exits "cannot open device" |
| On-device tool: probe + 64x64 fill, prints `PF ...` lines, exit 0 only if both pass on a G31 | `tools/hs_pf_probe.c` | component | runs on host (no device) |
| RDP -> GPU batch builder over `hs_n64_walk_backend()`: groups primitives with identical RDP state + TMEM into draws, NDC float vertices, TMEM upload set | `src/n64_pf.c` | component | demo frame: 96 prims -> 6 draws, 297 vertices, 4 TMEM images |

The WRITE_VALUE fill is deliberately the first GPU step: it proves device open, the GPU MMU
mapping of our BOs, job submission, the job chain format, completion and CPU read-back, all
without a shader, a framebuffer descriptor or a tiler heap. It is slow (one job per 8 bytes).
It is a correctness probe, not a renderer.

## 2. What needs the real device

Nothing in section 1 has run on a Mali GPU. The mock follows the same rules our encoder
assumes, so a mock pass shows the code is self-consistent, not that the layout is right.
On the RG35XX, run `build/aarch64/hs_pf_probe` (static binary, as root or a `render` group user):

1. `PF driver panfrost ...` and `gpu_prod_id 0x7093 (Mali-G31)`: confirms the probe and param numbering.
2. `PF bo ... gpu_va 0x...`: confirms CREATE_BO / MMAP_BO.
3. `PF write_value_fill PASS ... done=2048 bad_pixels=0`: confirms the SUBMIT struct (48-byte
   jm_ctx form), the 64-bit job header bit layout, the WRITE_VALUE payload and type numbers.
   If SUBMIT returns -EINVAL, try the 40-byte struct (older kernels). If jobs are not DONE,
   `first_status` gives the GPU exception code. Note which one in REPORT.md.

Facts to confirm there: job header bit positions, WRITE_VALUE type 7 = immediate 64,
DONE status = 1, job type 2 = WRITE_VALUE. These are public interface facts about the Mali job
manager as documented by the open Panfrost driver work. They are written here from that
public description, not copied from Mesa source. Before going further, **the owner should
decide whether using descriptor layouts learned from Panfrost reverse engineering is
acceptable under the clean-room rule.** Fragment / tiler / shader descriptors (steps 3-6) need
far more of those layouts than step 1 does.

## 3. Plan after step 1 (each step gated on the previous one passing on hardware)

| Step | Work | Needs | Exit test |
|---|---|---|---|
| 3. Fragment-job clear | Framebuffer descriptor (FBD) + render target descriptor + tiler context with an empty polygon list; one FRAGMENT job clears 640x480 XRGB | Bifrost FBD/RT layouts, tiler heap BO (`PANFROST_BO_HEAP`) | read-back equals clear colour, timed with CLOCK_MONOTONIC around SUBMIT+WAIT |
| 4. Shader tool | `tools/hs_bifrost_asm.c`: pure-C assembler for the small Bifrost ISA subset we need (FAU loads, FMA/ADD clauses, texture fetch, blend, ATEST, store) | Bifrost clause/tuple encoding (largest unknown) | assembled constant-colour fragment shader replaces the clear |
| 5. Triangles | VERTEX/IDVS + TILER jobs from the batch builder's vertex buffer (pre-transformed, so the vertex shader is a pass-through) | attribute/varying descriptors, draw descriptor | demo frame geometry, flat shade |
| 6. RDP pipeline | combiner -> generated fragment shaders (one per distinct combine+othermode key, cached), TMEM -> textures (decode on CPU in Phase 1 for every TMEM hash the builder reports), 3-point filter and blender in shader, depth via Z-buffer | everything above | pixel compare vs software RDP |

Bit exactness: the GPU path cannot be bit-identical to the software RDP (float rasterisation,
different edge rules, float combiner). The plan is: the software path stays the reference and
the default; the GPU path is a separate video mode with its own hash, plus RMS vs the software
8x image in the test suite (the same metric the SSAA modes already report).

Phase fit: Phase 1 (render) would build the batch (CPU, cheap: the batch walk is about the
cost of one replay) and submit; WAIT_BO happens at the start of the next Phase 1, so GPU work
overlaps Phase 0 of the next frame. No threads, no locks: the kernel's job queue is the only
asynchrony, and the core never shares memory with the GPU except BOs owned by Phase 1.

## 4. Feasibility verdict

* Steps 1-2 (probe, BO, job submission, write-value fill): done in code. Validated on the mock only.
* Step 3: feasible. It is a few hundred lines once the FBD layout is confirmed on the device.
* Step 4 (pure-C Bifrost assembler) is the critical path and the main risk. It is weeks of
  work, and without a disassembler it can only be debugged by GPU fault codes and read-back.
* Expected benefit: not measured, so no speedup is claimed. The CPU path after this round
  already meets the frame budget for the demo workload under emulation (see REPORT.md). Commercial titles are CPU-bound and still unmeasured. The GPU path matters
  mainly for 2x/4x SSAA, where the software cost grows with the sample count.

## 5. Alternative

Use Mesa's Panfrost GLES driver (already in mainline Mesa for G31, GLES 3.1) through a thin C
GLES backend over `hs_n64_walk_backend()`. Much less work and no ISA risk, but it adds Mesa to
HandshakeOS and leaves the pure-C / no-third-party-runtime direction. It needs an owner decision.
