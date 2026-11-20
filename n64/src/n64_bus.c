/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 HandshakeOS contributors
 *
 * n64_bus.c - clean-room N64 physical memory map and RCP interface blocks:
 *   RDRAM (4 / 8 MiB), SP DMEM/IMEM + SP regs + SP DMA, DP regs, MI, VI, AI,
 *   PI (+ PI DMA), RI, SI (+ PIF RAM joybus), cartridge ROM, PIF boot HLE,
 *   .z64/.v64/.n64 auto byte-order detection and CIC-6102 header checksum.
 * All accesses are big-endian at the byte level (host endianness never
 * leaks), so behaviour is identical on x86_64 and AArch64.
 */
#include <string.h>
#include "n64_internal.h"

/* ------------------------------------------------------------- ROM */
static uint32_t rom_xor(const hs_n64_t *n) { return n->rom_fmt == 1u ? 1u : (n->rom_fmt == 2u ? 3u : 0u); }

static uint8_t rom_byte(const hs_n64_t *n, uint32_t off)
{
    if ((size_t)off >= n->rom_len) return 0;
    return n->rom[off ^ rom_xor(n)];
}

static uint32_t rom_word(const hs_n64_t *n, uint32_t off)
{
    return ((uint32_t)rom_byte(n, off) << 24) | ((uint32_t)rom_byte(n, off + 1u) << 16) |
           ((uint32_t)rom_byte(n, off + 2u) << 8) | rom_byte(n, off + 3u);
}

int n64_rom_detect_swap(const uint8_t *rom, size_t len)
{
    if (len < 0x40u) return -1;
    if (rom[0] == 0x80 && rom[1] == 0x37 && rom[2] == 0x12 && rom[3] == 0x40) return 0;
    if (rom[0] == 0x37 && rom[1] == 0x80 && rom[2] == 0x40 && rom[3] == 0x12) return 1;
    if (rom[0] == 0x40 && rom[1] == 0x12 && rom[2] == 0x37 && rom[3] == 0x80) return 2;
    return -1;
}

static uint32_t rol32(uint32_t v, uint32_t s) { s &= 31u; return s ? (v << s) | (v >> (32u - s)) : v; }

/* CIC-6102 boot checksum over ROM bytes 0x1000..0x101000 (z64 order). */
void n64_rom_crc(const uint8_t *z, size_t len, uint32_t *crc1, uint32_t *crc2)
{
    uint32_t seed = 0xF8CA4DDCu;
    uint32_t t1 = seed, t2 = seed, t3 = seed, t4 = seed, t5 = seed, t6 = seed;
    uint32_t i;
    for (i = 0x1000u; i < 0x101000u; i += 4u) {
        uint32_t d = ((size_t)i + 4u <= len) ? n64_be32(z + i) : 0u;
        uint32_t r;
        if (t6 + d < t6) t4++;
        t6 += d;
        t3 ^= d;
        r = rol32(d, d & 31u);
        t5 += r;
        if (t2 > d) t2 ^= r; else t2 ^= t6 ^ d;
        t1 += t5 ^ d;
    }
    *crc1 = t6 ^ t4 ^ t3;
    *crc2 = t5 ^ t2 ^ t1;
}

int n64_rom_load(hs_n64_t *n, const uint8_t *rom, size_t len)
{
    int f = n64_rom_detect_swap(rom, len);
    uint32_t c1 = 0x6C078965u, c2 = 0, i, seed;
    uint32_t t1, t2, t3, t4, t5, t6;
    if (f < 0 || len < 0x1000u) return -1;
    n->rom = rom;
    n->rom_len = len;
    n->rom_fmt = (uint32_t)f;
    /* checksum computed through the byte-order view (no copy needed) */
    seed = 0xF8CA4DDCu;
    t1 = t2 = t3 = t4 = t5 = t6 = seed;
    for (i = 0x1000u; i < 0x101000u; i += 4u) {
        uint32_t d = rom_word(n, i), r;
        if (t6 + d < t6) t4++;
        t6 += d;
        t3 ^= d;
        r = rol32(d, d & 31u);
        t5 += r;
        if (t2 > d) t2 ^= r; else t2 ^= t6 ^ d;
        t1 += t5 ^ d;
    }
    c1 = t6 ^ t4 ^ t3;
    c2 = t5 ^ t2 ^ t1;
    n->rom_crc1 = c1;
    n->rom_crc2 = c2;
    n->rom_crc_ok = (c1 == rom_word(n, 0x10u) && c2 == rom_word(n, 0x14u)) ? 1u : 0u;
    return 0;
}

/* ---------------------------------------------------------- interrupts */
void n64_mi_raise(hs_n64_t *n, uint32_t bit) { n->mi_intr |= bit; n64_cpu_update_irq(n); }
void n64_mi_clear(hs_n64_t *n, uint32_t bit) { n->mi_intr &= ~bit; n64_cpu_update_irq(n); }

/* ------------------------------------------------------------- PI DMA */
static void pi_dma_to_rdram(hs_n64_t *n, uint32_t len)
{
    uint32_t dram = n->pi_dram & 0x00FFFFFEu, cart = n->pi_cart & ~1u, i;
    uint32_t xr = rom_xor(n);
    if (len > n->rdram_size) len = n->rdram_size;
    if (cart >= 0x10000000u && cart < 0x1FC00000u) {
        uint32_t off = cart - 0x10000000u;
        for (i = 0; i < len; i++) {
            uint32_t d = dram + i, o = off + i;
            if (d >= n->rdram_size) break;
            n->rdram[d] = ((size_t)o < n->rom_len) ? n->rom[o ^ xr] : 0u;
        }
    }
    n->pi_dram = dram + len;
    n->pi_cart = cart + len;
    n->pi_dma_count++;
    n->pi_status = 0x8u;                    /* idle, interrupt pending */
    n64_mi_raise(n, MI_PI);
}

/* ------------------------------------------------------------- SP DMA */
static void sp_dma(hs_n64_t *n, uint32_t v, int to_sp)
{
    uint32_t len = (v & 0xFFFu) + 1u, count = ((v >> 12) & 0xFFu) + 1u, skip = (v >> 20) & 0xFFFu;
    uint32_t mem = n->rsp.mem_addr & 0x1FFFu, dram = n->rsp.dram_addr & 0x00FFFFF8u, k, i;
    uint8_t *spm = (mem & 0x1000u) ? n->rsp.imem : n->rsp.dmem;
    len = (len + 7u) & ~7u;
    for (k = 0; k < count; k++) {
        for (i = 0; i < len; i++) {
            uint32_t m = (mem + i) & 0xFFFu, d = dram + i;
            if (d >= n->rdram_size) continue;
            if (to_sp) spm[m] = n->rdram[d]; else n->rdram[d] = spm[m];
        }
        mem = (mem & 0x1000u) | ((mem + len) & 0xFFFu);
        dram += len + skip;
    }
    n->rsp.mem_addr = mem;
    n->rsp.dram_addr = dram;
    n->sp_dma_count++;
}

static void sp_status_write(hs_n64_t *n, uint32_t v)
{
    uint32_t s = n->rsp.status, b;
    int was_halted = (s & 1u) != 0;
    if (v & 0x1u) s &= ~1u;
    if (v & 0x2u) s |= 1u;
    if (v & 0x4u) s &= ~2u;
    if (v & 0x8u) n64_mi_clear(n, MI_SP);
    if (v & 0x10u) n64_mi_raise(n, MI_SP);
    if (v & 0x20u) s &= ~0x20u;
    if (v & 0x40u) s |= 0x20u;
    if (v & 0x80u) s &= ~0x40u;
    if (v & 0x100u) s |= 0x40u;
    for (b = 0; b < 8u; b++) {
        if (v & (0x200u << (2u * b))) s &= ~(0x80u << b);
        if (v & (0x400u << (2u * b))) s |= (0x80u << b);
    }
    n->rsp.status = s;
    if (was_halted && !(s & 1u)) n64_rsp_start(n);
}

/* ------------------------------------------------------------- PIF */
static void pif_controller(hs_n64_t *n, uint8_t cmd, uint8_t *res, uint32_t rx)
{
    uint32_t in = n->input, b = 0;
    int8_t sx = 0, sy = 0;
    if (cmd == 0x00 || cmd == 0xFF) {
        if (rx >= 3u) { res[0] = 0x05; res[1] = 0x00; res[2] = 0x02; }
        return;
    }
    if (cmd == 0x01 && rx >= 4u) {
        if (in & HS_N64_A) b |= 0x8000u;
        if (in & HS_N64_B) b |= 0x4000u;
        if (in & HS_BTN_Z) b |= 0x2000u;
        if (in & HS_BTN_START) b |= 0x1000u;
        if (in & HS_BTN_UP) b |= 0x0800u;
        if (in & HS_BTN_DOWN) b |= 0x0400u;
        if (in & HS_BTN_LEFT) b |= 0x0200u;
        if (in & HS_BTN_RIGHT) b |= 0x0100u;
        if (in & HS_BTN_L) b |= 0x0020u;
        if (in & HS_BTN_R) b |= 0x0010u;
        if (in & HS_BTN_C_UP) b |= 0x0008u;
        if (in & HS_BTN_C_DOWN) b |= 0x0004u;
        if (in & HS_BTN_C_LEFT) b |= 0x0002u;
        if (in & HS_BTN_C_RIGHT) b |= 0x0001u;
        /* no analog axis in the shared ABI: the d-pad also drives the stick */
        if (in & HS_BTN_LEFT) sx = -80;
        if (in & HS_BTN_RIGHT) sx = 80;
        if (in & HS_BTN_UP) sy = 80;
        if (in & HS_BTN_DOWN) sy = -80;
        res[0] = (uint8_t)(b >> 8); res[1] = (uint8_t)b;
        res[2] = (uint8_t)sx; res[3] = (uint8_t)sy;
    }
}

void n64_pif_process(hs_n64_t *n)
{
    uint8_t *ram = n->pif_ram;
    uint32_t i = 0, ch = 0, guard;
    for (guard = 0; guard < 64u && i < 63u && ch < 6u; guard++) {
        uint32_t t = ram[i], tx, rx, ci, ri;
        if (t == 0xFEu) break;
        if (t == 0xFFu || t == 0xFDu) { i++; continue; }
        if (t == 0u) { ch++; i++; continue; }
        tx = t & 0x3Fu;
        rx = ram[i + 1u] & 0x3Fu;
        ci = i + 2u;
        ri = ci + tx;
        if (ri + rx > 63u) break;
        if (ch == 0u) pif_controller(n, ram[ci], &ram[ri], rx);
        else ram[i + 1u] |= 0x80u;
        i = ri + rx;
        ch++;
    }
    ram[63] = 0;
}

/* ------------------------------------------------------------- AI */
static void ai_start(hs_n64_t *n, uint32_t len)
{
    n64_snap_t *b = n64_back(n);
    uint32_t addr = n->ai_dram & 0x00FFFFF8u, frames = (len & 0x3FFF8u) / 4u, i;
    for (i = 0; i < frames && b->ai_n < N64_AI_MAX_FRAMES; i++) {
        uint32_t a = addr + i * 4u;
        if (a + 4u > n->rdram_size) break;
        b->ai[b->ai_n * 2u] = (int16_t)n64_be16(n->rdram + a);
        b->ai[b->ai_n * 2u + 1u] = (int16_t)n64_be16(n->rdram + a + 2u);
        b->ai_n++;
    }
    b->ai_freq = n->ai_dacrate ? N64_VI_CLOCK_NTSC / (n->ai_dacrate + 1u) : 0u;
    n->ai_len = len;
    n->ai_dma_count++;
    n64_mi_raise(n, MI_AI);
}

/* ------------------------------------------------------------- VI */
void n64_vi_frame_start(hs_n64_t *n)
{
    n->frame_instr = 0;
    n64_mi_raise(n, MI_VI);
}

static uint32_t vi_current(const hs_n64_t *n)
{
    uint64_t l = (uint64_t)n->frame_instr * 525u / N64_INSTR_PER_FRAME;
    return (uint32_t)(l > 524u ? 524u : l) & ~1u;
}

/* -------------------------------------------------------- dispatch */
uint32_t n64_phys_read32(hs_n64_t *n, uint32_t pa)
{
    uint32_t r = (pa >> 2) & 0x3Fu;
    if (pa < n->rdram_size) return n64_be32(n->rdram + (pa & ~3u));
    if (pa >= 0x04000000u && pa < 0x04002000u)
        return n64_be32(((pa & 0x1000u) ? n->rsp.imem : n->rsp.dmem) + (pa & 0xFFCu));
    switch (pa & 0xFFF00000u) {
    case 0x04000000u:
        if (pa >= 0x04080000u) return n->rsp.pc & 0xFFCu;
        switch (r & 7u) {
        case 0: return n->rsp.mem_addr;
        case 1: return n->rsp.dram_addr;
        case 2: return n->rsp.rd_len;
        case 3: return n->rsp.wr_len;
        case 4: return n->rsp.status;
        case 7: { uint32_t v = n->rsp.hwflag; n->rsp.hwflag = 1; return v; }
        default: return 0;
        }
    case 0x04100000u:
        switch (r & 7u) {
        case 0: return n->dp_start;
        case 1: return n->dp_end;
        case 2: return n->dp_current;
        case 3: return n->dp_status;
        default: return 0;
        }
    case 0x04300000u:
        switch (r & 3u) {
        case 0: return n->mi_mode;
        case 1: return 0x02020102u;
        case 2: return n->mi_intr;
        default: return n->mi_mask;
        }
    case 0x04400000u:
        if (r == 4u) return vi_current(n);
        return r < 14u ? n->vi_reg[r] : 0u;
    case 0x04500000u:
        switch (r & 7u) {
        case 0: return n->ai_dram;
        case 1: return 0;                 /* buffers are consumed by the snapshot instantly */
        case 2: return n->ai_control;
        case 3: return n->ai_status;
        case 4: return n->ai_dacrate;
        case 5: return n->ai_bitrate;
        default: return 0;
        }
    case 0x04600000u:
        switch (r) {
        case 0: return n->pi_dram;
        case 1: return n->pi_cart;
        case 4: return n->pi_status & 0x8u;
        default: return (r >= 5u && r < 13u) ? n->pi_dom[r - 5u] : 0u;
        }
    case 0x04700000u: return n->ri_reg[r & 7u];
    case 0x04800000u:
        if (r == 0u) return n->si_dram;
        if (r == 6u) return n->si_status;
        return 0;
    default: break;
    }
    if (pa >= 0x10000000u && pa < 0x1FC00000u) return rom_word(n, pa - 0x10000000u);
    if (pa >= 0x1FC007C0u && pa < 0x1FC00800u) return n64_be32(n->pif_ram + (pa & 0x3Cu));
    return 0;
}

void n64_phys_write32(hs_n64_t *n, uint32_t pa, uint32_t v, uint32_t mask)
{
    uint32_t r = (pa >> 2) & 0x3Fu;
    if (pa < n->rdram_size) {
        uint8_t *p = n->rdram + (pa & ~3u);
        n64_wbe32(p, (n64_be32(p) & ~mask) | (v & mask));
        return;
    }
    if (pa >= 0x04000000u && pa < 0x04002000u) {
        uint8_t *p = ((pa & 0x1000u) ? n->rsp.imem : n->rsp.dmem) + (pa & 0xFFCu);
        n64_wbe32(p, (n64_be32(p) & ~mask) | (v & mask));
        return;
    }
    switch (pa & 0xFFF00000u) {
    case 0x04000000u:
        if (pa >= 0x04080000u) { n->rsp.pc = v & 0xFFCu; return; }
        switch (r & 7u) {
        case 0: n->rsp.mem_addr = v & 0x1FF8u; return;
        case 1: n->rsp.dram_addr = v & 0xFFFFF8u; return;
        case 2: n->rsp.rd_len = v; sp_dma(n, v, 1); return;
        case 3: n->rsp.wr_len = v; sp_dma(n, v, 0); return;
        case 4: sp_status_write(n, v); return;
        case 7: n->rsp.hwflag = 0; return;
        default: return;
        }
    case 0x04100000u:
        switch (r & 7u) {
        case 0: n->dp_start = v & 0xFFFFF8u; n->dp_current = n->dp_start; return;
        case 1: n->dp_end = v & 0xFFFFF8u; n64_dp_run(n); return;
        case 3: if (v & 1u) n->dp_status &= ~1u; if (v & 2u) n->dp_status |= 1u; return;
        default: return;
        }
    case 0x04300000u:
        if (r == 0u) {
            n->mi_mode = v & 0x7Fu;
            if (v & 0x800u) n64_mi_clear(n, MI_DP);
        } else if (r == 3u) {
            uint32_t b;
            for (b = 0; b < 6u; b++) {
                if (v & (1u << (2u * b))) n->mi_mask &= ~(1u << b);
                if (v & (2u << (2u * b))) n->mi_mask |= (1u << b);
            }
            n64_cpu_update_irq(n);
        }
        return;
    case 0x04400000u:
        if (r == 4u) { n64_mi_clear(n, MI_VI); return; }
        if (r < 14u) n->vi_reg[r] = v;
        return;
    case 0x04500000u:
        switch (r & 7u) {
        case 0: n->ai_dram = v & 0xFFFFF8u; return;
        case 1: ai_start(n, v); return;
        case 2: n->ai_control = v & 1u; return;
        case 3: n64_mi_clear(n, MI_AI); return;
        case 4: n->ai_dacrate = v & 0x3FFFu; return;
        case 5: n->ai_bitrate = v & 0xFu; return;
        default: return;
        }
    case 0x04600000u:
        switch (r) {
        case 0: n->pi_dram = v & 0xFFFFFEu; return;
        case 1: n->pi_cart = v & ~1u; return;
        case 2: n->pi_dma_count++; n64_mi_raise(n, MI_PI); n->pi_status = 0x8u; return; /* RDRAM->cart: no writable cart memory */
        case 3: pi_dma_to_rdram(n, (v & 0xFFFFFFu) + 1u); return;
        case 4: if (v & 2u) { n->pi_status &= ~0x8u; n64_mi_clear(n, MI_PI); } return;
        default: if (r >= 5u && r < 13u) n->pi_dom[r - 5u] = v; return;
        }
    case 0x04700000u: n->ri_reg[r & 7u] = v; return;
    case 0x04800000u:
        if (r == 0u) { n->si_dram = v & 0xFFFFF8u; return; }
        if (r == 1u) {                     /* PIF -> RDRAM */
            uint32_t i;
            n64_pif_process(n);
            for (i = 0; i < 64u; i++) if (n->si_dram + i < n->rdram_size) n->rdram[n->si_dram + i] = n->pif_ram[i];
            n->si_dma_count++; n->si_status |= 0x1000u; n64_mi_raise(n, MI_SI); return;
        }
        if (r == 4u) {                     /* RDRAM -> PIF */
            uint32_t i;
            for (i = 0; i < 64u; i++) n->pif_ram[i] = (n->si_dram + i < n->rdram_size) ? n->rdram[n->si_dram + i] : 0u;
            n->si_dma_count++; n->si_status |= 0x1000u; n64_mi_raise(n, MI_SI); return;
        }
        if (r == 6u) { n->si_status &= ~0x1000u; n64_mi_clear(n, MI_SI); }
        return;
    default: break;
    }
    if (pa >= 0x1FC007C0u && pa < 0x1FC00800u) {
        uint8_t *p = n->pif_ram + (pa & 0x3Cu);
        n64_wbe32(p, (n64_be32(p) & ~mask) | (v & mask));
    }
}

/* -------------------------------------------- PIF / IPL3 boot (HLE) */
void n64_pif_boot(hs_n64_t *n)
{
    n64_cpu_t *c = &n->cpu;
    uint32_t entry = rom_word(n, 0x08u);
    uint32_t len = 0x100000u;
    if ((size_t)len + 0x1000u > n->rom_len) len = (uint32_t)(n->rom_len - 0x1000u);
    n64_cpu_reset(n);
    /* IPL3 behaviour: copy the first MiB after the boot code to the entry
     * point with a real PI DMA, leave the CIC-6102 register state. */
    n->pi_dram = entry & 0x00FFFFFFu;
    n->pi_cart = 0x10001000u;
    pi_dma_to_rdram(n, len);
    n64_mi_clear(n, MI_PI);
    n->pi_status = 0;
    n64_wbe32(n->rdram + 0x318u, n->rdram_size);    /* osMemSize */
    c->gpr[19] = 0;                                  /* s3: ROM boot */
    c->gpr[20] = 1;                                  /* s4: NTSC */
    c->gpr[21] = 0;                                  /* s5: cold reset */
    c->gpr[22] = 0x3Fu;                              /* s6: CIC seed (6102) */
    c->gpr[23] = 0;
    c->gpr[29] = 0xFFFFFFFFA4001FF0ull;              /* sp */
    c->gpr[31] = 0xFFFFFFFFA4001550ull;
    c->cp0[C0_STATUS] = 0x34000000u;
    c->cp0[C0_CONFIG] = 0x0006E463u;
    c->cp0[C0_RANDOM] = 31u;
    c->pc = (uint64_t)(int64_t)(int32_t)entry;
    c->npc = c->pc + 4u;
    n->vi_reg[0] = 0; n->vi_reg[2] = 320u; n->vi_reg[12] = 0x200u; n->vi_reg[13] = 0x400u;
    n->rsp.status = 1u;                              /* RSP halted */
}
