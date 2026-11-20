/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * hs_snes.c — 3-phase zero-mutex driver for the clean-room hs-snes core.
 *
 * Phase 0 runs exactly one NTSC frame (262 lines x 1364 master cycles) of
 * 65C816 + bus + DMA/HDMA + lock-step SPC700 and commits an immutable
 * hs_snes_slot (per-line PPU snapshot, VRAM/OAM/CGRAM versions, APU write
 * log, port-sync log).  Phases 1 and 2 consume committed slots only.
 */
#include <string.h>
#include "snes_internal.h"

size_t hs_snes_sizeof(void) { return sizeof(hs_snes); }

void hs_snes_reset(hs_snes *s)
{
    snes_bus_reset(s);
    snes_ppu_reset(&s->ppu);
    snes_spc_reset(&s->spc);
    s->master = 0;
    s->line_start = 0;
    s->frame_start = 0;
    s->vpos = 0;
    s->vis_lines = 224;
    s->frame_no = 0;
    s->in_vblank = 0;
    s->irq_fired_line = 0;
    s->slot = 0;
    snes_cpu_reset(s);
}

int hs_snes_init(hs_snes *s, const uint8_t *rom, size_t rom_size)
{
    memset(s, 0, sizeof *s);
    if (snes_cart_load(s, rom, rom_size) != 0) return -1;
    memset(s->sram, 0xFF, sizeof s->sram);
    hs_snes_reset(s);
    return 0;
}

int hs_snes_mapper(const hs_snes *s) { return s->mapper; }

const char *hs_snes_title(const hs_snes *s, char out[22])
{
    memcpy(out, s->title, 22);
    out[21] = 0;
    return out;
}

uint8_t *hs_snes_sram(hs_snes *s, size_t *size)
{
    if (size) *size = s->sram_size;
    return s->sram_size ? s->sram : 0;
}

void hs_snes_set_input(hs_snes *s, uint16_t pad1, uint16_t pad2)
{
    s->pad[0] = pad1;
    s->pad[1] = pad2;
}

static void check_irq(hs_snes *s)
{
    uint8_t mode = (uint8_t)((s->nmitimen >> 4) & 3u);
    uint64_t point;
    if (!mode || s->irq_fired_line) return;
    if (mode != 1 && s->vpos != s->vtime) return;
    point = (mode == 2) ? 0u : (uint64_t)s->htime * 4u + 14u;
    if (s->master - s->line_start >= point) {
        s->irq_flag = 1;
        s->cpu.irq_line = 1;
        s->irq_fired_line = 1;
    }
}

void hs_snes_step_cpu_phase(hs_snes *s, hs_snes_slot *out)
{
    unsigned v;
    s->slot = out;
    out->frame_no = s->frame_no;
    out->apu_log_count = 0;
    out->apu_log_dropped = 0;
    out->port_log_count = 0;
    out->port_log_hash = 0xcbf29ce484222325ull;
    out->sample_begin = (uint32_t)(s->spc.cycles >> 5);
    s->vis_lines = (s->ppu.setini & 0x04u) ? 239 : 224;
    out->lines = s->vis_lines;

    for (v = 0; v < SNES_LINES_PER_FRAME; v++) {
        uint64_t line_end;
        s->vpos = (uint16_t)v;
        s->line_start = s->frame_start + (uint64_t)v * SNES_MASTER_PER_LINE;
        line_end = s->line_start + SNES_MASTER_PER_LINE;
        s->irq_fired_line = 0;
        if (v == 0) {
            s->in_vblank = 0;
            s->nmi_flag = 0;
            snes_ppu_snapshot_frame(s);
            snes_hdma_init(s);
        } else if (v <= s->vis_lines) {
            snes_hdma_line(s);
            snes_ppu_snapshot_line(s, (int)v);
        } else if (v == s->vis_lines + 1u) {
            s->in_vblank = 1;
            s->nmi_flag = 1;
            if (s->nmitimen & 0x80u) s->cpu.nmi_pending = 1;
            if (s->nmitimen & 0x01u) { s->autojoy_busy = 1; snes_autojoy(s); }
            if (!(s->ppu.inidisp & 0x80u)) s->ppu.oam_ptr = (uint16_t)((s->ppu.oamaddr_reload & 0x1FFu) << 1);
        } else if (v == s->vis_lines + 4u) {
            s->autojoy_busy = 0;
        }
        /* bounded: every step advances master by >= 6 cycles */
        while (s->master < line_end) {
            check_irq(s);
            snes_cpu_step(s);
        }
        check_irq(s);
        snes_apu_catchup(s);
    }
    s->frame_start += (uint64_t)SNES_LINES_PER_FRAME * SNES_MASTER_PER_LINE;
    if (s->master < s->frame_start) s->master = s->frame_start;
    snes_apu_catchup(s);
    out->sample_end = (uint32_t)(s->spc.cycles >> 5);
    s->frame_no++;
    s->slot = 0;
}

uint64_t hs_snes_frame_hash(uint64_t h, const uint32_t *fb, const int16_t *pcm, int frames)
{
    size_t i;
    if (fb) for (i = 0; i < (size_t)HS_SNES_OUT_W * HS_SNES_OUT_H; i++) h = (h ^ (fb[i] & 0xFFFFFFu)) * 1099511628211ull;
    if (pcm) for (i = 0; i < (size_t)frames * 2u; i++) h = (h ^ (uint16_t)pcm[i]) * 1099511628211ull;
    return h;
}

void hs_snes_cpu_state(const hs_snes *s, hs_snes_cpu_view *v)
{
    v->a = s->cpu.a; v->x = s->cpu.x; v->y = s->cpu.y; v->sp = s->cpu.sp;
    v->dp = s->cpu.dp; v->pc = s->cpu.pc; v->pb = s->cpu.pb; v->db = s->cpu.db;
    v->p = s->cpu.p; v->e = s->cpu.e; v->stopped = s->cpu.stopped; v->waiting = s->cpu.waiting;
    v->master = s->master;
}

uint8_t hs_snes_peek(const hs_snes *s, uint32_t addr24) { return snes_bus_peek(s, addr24 & 0xFFFFFFu); }
uint8_t hs_snes_aram_peek(const hs_snes *s, uint16_t addr) { return s->spc.ram[addr]; }
uint8_t hs_snes_apu_port_out(const hs_snes *s, int port) { return s->spc.out[port & 3]; }
uint64_t hs_snes_spc_cycles(const hs_snes *s) { return s->spc.cycles; }
