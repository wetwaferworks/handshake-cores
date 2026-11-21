/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jaydee Wetwork / HandshakeOS contributors
 *
 * snes_apu.c — clean-room Sony SPC700 + S-DSP for hs-snes.
 *
 * Phase 0 (lock-step with the 65C816, required for $2140-$2143 port
 * causality): SPC700 interpreter (all 256 opcodes), 64 KiB ARAM, three
 * hardware timers, CONTROL/TEST registers, our own protocol-compatible
 * 64-byte IPL boot stub, and a timestamped ARAM/DSP write log that is
 * committed into the frame slot.
 *
 * Phase 2 (own core, reads only the committed slot): S-DSP with 8 voices,
 * BRR ADPCM decode (4 filters), Gaussian-shaped 4-tap interpolation
 * (clean-room integer kernel, sigma 0.62, normalised to 2048), pitch
 * modulation, noise, ADSR + GAIN envelopes, L/R volume, 8-tap echo FIR with
 * feedback into the DSP's private ARAM copy, and a deterministic integer
 * 32 kHz -> 48 kHz stereo resampler.
 */
#include <string.h>
#include "snes_internal.h"

/* Clean-room IPL boot stub (hand-assembled, NOT the Sony mask ROM).
 * Implements the documented CPU<->APU upload protocol:
 *   $FFC0 mov $F4,#$AA / mov $F5,#$BB / wait port0==$CC
 *   cmd:  movw ya,$F6 -> $00/$01 ; y=port0 (kick) ; echo ; a=port1 ; beq exec
 *         y=0 ; wait port0==0
 *   data: a=port1 ; echo y ; mov [$00]+y,a ; inc y (carry -> inc $01)
 *   wait: port0==y -> data ; port0==y-1 -> keep waiting ; else -> cmd
 *   exec: mov x,#0 ; jmp [!$0000+x]
 */
static const uint8_t k_ipl[64] = {
    0x8F, 0xAA, 0xF4, 0x8F, 0xBB, 0xF5, 0x78, 0xCC, 0xF4, 0xD0, 0xFB,       /* FFC0 */
    0xBA, 0xF6, 0xDA, 0x00, 0xEB, 0xF4, 0xCB, 0xF4, 0xE4, 0xF5, 0xF0, 0x20, /* FFCB */
    0x8D, 0x00, 0x7E, 0xF4, 0xD0, 0xFC,                                     /* FFD7 */
    0xE4, 0xF5, 0xCB, 0xF4, 0xD7, 0x00, 0xFC, 0xD0, 0x02, 0xAB, 0x01,       /* FFDD */
    0xCB, 0x02, 0xE4, 0xF4, 0x64, 0x02, 0xF0, 0xED, 0xBC, 0x64, 0x02,       /* FFE8 */
    0xF0, 0xF5, 0x2F, 0xD4,                                                 /* FFF3 */
    0xCD, 0x00, 0x1F, 0x00, 0x00,                                           /* FFF7 */
    0x00, 0x00,                                                             /* FFFC */
    0xC0, 0xFF                                                              /* FFFE reset vector */
};

static const uint8_t k_spc_cycles[256] = {
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,6,8,  2,8,4,5,4,5,5,6,5,5,6,5,2,2,4,6,
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,5,4,  2,8,4,5,4,5,5,6,5,5,6,5,2,2,3,8,
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,6,6,  2,8,4,5,4,5,5,6,5,5,4,5,2,2,4,3,
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,5,5,  2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,6,
    2,8,4,5,3,4,3,6,2,6,5,4,5,2,4,5,  2,8,4,5,4,5,5,6,5,5,5,5,2,2,12,5,
    3,8,4,5,3,4,3,6,2,6,4,4,5,2,4,4,  2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,4,
    3,8,4,5,4,5,4,7,2,5,6,4,5,2,4,9,  2,8,4,5,5,6,6,7,4,5,5,5,2,2,6,3,
    2,8,4,5,3,4,3,6,2,4,5,3,4,3,4,3,  2,8,4,5,4,5,5,6,3,4,5,4,2,2,4,3
};

static const int16_t k_gauss[512] = {
       7,    7,    8,    8,    8,    8,    8,    8,    9,    9,    9,    9,    9,    9,   10,   10,
      10,   10,   10,   11,   11,   11,   11,   11,   12,   12,   12,   12,   13,   13,   13,   13,
      14,   14,   14,   14,   15,   15,   15,   16,   16,   16,   16,   17,   17,   17,   18,   18,
      18,   19,   19,   19,   20,   20,   21,   21,   21,   22,   22,   22,   23,   23,   24,   24,
      25,   25,   25,   26,   26,   27,   27,   28,   28,   29,   29,   30,   30,   31,   31,   32,
      33,   33,   34,   34,   35,   35,   36,   37,   37,   38,   39,   39,   40,   41,   41,   42,
      43,   43,   44,   45,   45,   46,   47,   48,   49,   49,   50,   51,   52,   53,   53,   54,
      55,   56,   57,   58,   59,   60,   61,   62,   63,   64,   65,   66,   67,   68,   69,   70,
      71,   72,   73,   74,   75,   76,   78,   79,   80,   81,   82,   84,   85,   86,   87,   89,
      90,   91,   93,   94,   95,   97,   98,   99,  101,  102,  104,  105,  107,  108,  110,  111,
     113,  115,  116,  118,  119,  121,  123,  124,  126,  128,  130,  131,  133,  135,  137,  139,
     141,  142,  144,  146,  148,  150,  152,  154,  156,  158,  160,  162,  164,  167,  169,  171,
     173,  175,  178,  180,  182,  184,  187,  189,  191,  194,  196,  199,  201,  203,  206,  208,
     211,  214,  216,  219,  221,  224,  227,  229,  232,  235,  238,  240,  243,  246,  249,  252,
     255,  258,  261,  264,  267,  270,  273,  276,  279,  282,  285,  288,  291,  295,  298,  301,
     304,  308,  311,  314,  318,  321,  324,  328,  331,  335,  338,  342,  345,  349,  353,  356,
     360,  363,  367,  371,  375,  378,  382,  386,  390,  394,  397,  401,  405,  409,  413,  417,
     421,  425,  429,  433,  437,  441,  446,  450,  454,  458,  462,  467,  471,  475,  479,  484,
     488,  492,  497,  501,  506,  510,  514,  519,  523,  528,  532,  537,  541,  546,  551,  555,
     560,  564,  569,  574,  578,  583,  588,  593,  597,  602,  607,  612,  616,  621,  626,  631,
     636,  640,  645,  650,  655,  660,  665,  670,  675,  680,  685,  690,  695,  699,  704,  709,
     714,  719,  724,  729,  734,  739,  744,  749,  755,  760,  765,  770,  775,  780,  785,  790,
     795,  800,  805,  810,  815,  820,  825,  830,  835,  840,  845,  850,  855,  860,  865,  870,
     875,  880,  885,  890,  895,  900,  905,  910,  915,  920,  925,  930,  935,  940,  945,  950,
     954,  959,  964,  969,  974,  978,  983,  988,  993,  997, 1002, 1007, 1011, 1016, 1021, 1025,
    1030, 1035, 1039, 1044, 1048, 1053, 1057, 1061, 1066, 1070, 1075, 1079, 1083, 1088, 1092, 1096,
    1100, 1104, 1109, 1113, 1117, 1121, 1125, 1129, 1133, 1137, 1141, 1145, 1148, 1152, 1156, 1160,
    1164, 1167, 1171, 1174, 1178, 1182, 1185, 1189, 1192, 1195, 1199, 1202, 1205, 1209, 1212, 1215,
    1218, 1221, 1224, 1227, 1230, 1233, 1236, 1239, 1241, 1244, 1247, 1250, 1252, 1255, 1257, 1260,
    1262, 1264, 1267, 1269, 1271, 1274, 1276, 1278, 1280, 1282, 1284, 1286, 1288, 1289, 1291, 1293,
    1295, 1296, 1298, 1299, 1301, 1302, 1304, 1305, 1306, 1307, 1309, 1310, 1311, 1312, 1313, 1314,
    1314, 1315, 1316, 1317, 1317, 1318, 1319, 1319, 1319, 1320, 1320, 1320, 1321, 1321, 1321, 1321,
};

enum { SF_C = 0x01, SF_Z = 0x02, SF_I = 0x04, SF_H = 0x08, SF_B = 0x10, SF_P = 0x20, SF_V = 0x40, SF_N = 0x80 };

/* ================================================================ phase 0 */
void snes_spc_reset(snes_spc *p)
{
    memset(p, 0, sizeof *p);
    p->sp = 0xEF;
    p->psw = SF_Z;
    p->ipl_on = 1;
    p->control = 0x80;
    p->pc = (uint16_t)(k_ipl[62] | (k_ipl[63] << 8));
}

static void log_apu(hs_snes *s, uint8_t kind, uint16_t addr, uint8_t val)
{
    hs_snes_slot *o = s->slot;
    hs_snes_apu_event *e;
    if (!o) return;
    if (o->apu_log_count >= HS_SNES_APU_LOG_MAX) { o->apu_log_dropped++; return; }
    e = &o->apu_log[o->apu_log_count++];
    e->sample = (uint32_t)(s->spc.cycles >> 5);
    e->addr = addr;
    e->val = val;
    e->kind = kind;
}

static void log_port(hs_snes *s, uint8_t port, uint8_t val)
{
    hs_snes_slot *o = s->slot;
    uint64_t h;
    uint32_t t;
    if (!o) return;
    t = (uint32_t)(s->master - s->frame_start);
    h = o->port_log_hash;
    h = (h ^ t) * 1099511628211ull;
    h = (h ^ (uint64_t)((port << 8) | val)) * 1099511628211ull;
    o->port_log_hash = h;
    if (o->port_log_count < HS_SNES_PORT_LOG_MAX) {
        hs_snes_port_event *e = &o->port_log[o->port_log_count];
        e->master = t; e->port = port; e->val = val; e->pad = 0;
    }
    o->port_log_count++;
}

static uint8_t spc_read(hs_snes *s, uint16_t a)
{
    snes_spc *p = &s->spc;
    if ((a & 0xFFF0u) == 0x00F0u) {
        switch (a) {
        case 0xF2: return p->dsp_addr;
        case 0xF3: return p->dsp_shadow[p->dsp_addr & 0x7Fu];
        case 0xF4: case 0xF5: case 0xF6: case 0xF7: return p->in[a - 0xF4u];
        case 0xF8: case 0xF9: return p->ram[a];
        case 0xFD: case 0xFE: case 0xFF: {
            uint8_t v = p->t[a - 0xFDu].counter;
            p->t[a - 0xFDu].counter = 0;
            return v;
        }
        default: return 0;
        }
    }
    if (a >= 0xFFC0u && p->ipl_on) return k_ipl[a - 0xFFC0u];
    return p->ram[a];
}

static void spc_write(hs_snes *s, uint16_t a, uint8_t v)
{
    snes_spc *p = &s->spc;
    if ((a & 0xFFF0u) == 0x00F0u) {
        switch (a) {
        case 0xF0: p->test = v; break;
        case 0xF1: {
            int i;
            for (i = 0; i < 3; i++) {
                uint8_t bit = (uint8_t)(1u << i);
                if (!(p->control & bit) && (v & bit)) { p->t[i].stage = 0; p->t[i].counter = 0; }
                p->t[i].enabled = (v & bit) != 0;
            }
            if (v & 0x10u) { p->in[0] = 0; p->in[1] = 0; }
            if (v & 0x20u) { p->in[2] = 0; p->in[3] = 0; }
            p->ipl_on = (v & 0x80u) != 0;
            p->control = v;
            break;
        }
        case 0xF2: p->dsp_addr = v; break;
        case 0xF3:
            if (p->dsp_addr < 0x80u) {
                p->dsp_shadow[p->dsp_addr] = v;
                if (p->dsp_addr == 0x7Cu) p->dsp_shadow[0x7C] = 0;
                log_apu(s, 1, p->dsp_addr, v);
            }
            break;
        case 0xF4: case 0xF5: case 0xF6: case 0xF7:
            p->out[a - 0xF4u] = v;
            log_port(s, (uint8_t)(0x20u | (a - 0xF4u)), v);
            break;
        case 0xFA: case 0xFB: case 0xFC: p->t[a - 0xFAu].target = v; break;
        default: break;
        }
    }
    p->ram[a] = v;
    log_apu(s, 0, a, v);
}

#define SPC (&s->spc)
static inline uint8_t sfetch(hs_snes *s) { uint8_t v = spc_read(s, s->spc.pc); s->spc.pc++; return v; }
static inline uint16_t sfetch16(hs_snes *s) { uint16_t lo = sfetch(s); return (uint16_t)(lo | (sfetch(s) << 8)); }
static inline uint16_t dpa(const snes_spc *p, uint8_t d) { return (uint16_t)(((p->psw & SF_P) ? 0x100u : 0u) | d); }
static inline uint8_t rdp(hs_snes *s, uint8_t d) { return spc_read(s, dpa(SPC, d)); }
static inline void wdp(hs_snes *s, uint8_t d, uint8_t v) { spc_write(s, dpa(SPC, d), v); }
static inline uint16_t rdp16(hs_snes *s, uint8_t d) { uint16_t lo = rdp(s, d); return (uint16_t)(lo | (rdp(s, (uint8_t)(d + 1u)) << 8)); }
static inline void spush(hs_snes *s, uint8_t v) { spc_write(s, (uint16_t)(0x100u | s->spc.sp), v); s->spc.sp--; }
static inline uint8_t spop(hs_snes *s) { s->spc.sp++; return spc_read(s, (uint16_t)(0x100u | s->spc.sp)); }
static inline void snz(snes_spc *p, uint8_t v) { p->psw = (uint8_t)((p->psw & ~(SF_N | SF_Z)) | (v & 0x80u) | (v ? 0u : SF_Z)); }
static inline void sflag(snes_spc *p, uint8_t f, int on) { p->psw = (uint8_t)(on ? (p->psw | f) : (p->psw & ~f)); }

static uint8_t s_adc(snes_spc *p, uint8_t a, uint8_t b)
{
    int r = a + b + (p->psw & SF_C);
    sflag(p, SF_V, (~(a ^ b) & (a ^ r) & 0x80) != 0);
    sflag(p, SF_H, ((a ^ b ^ r) & 0x10) != 0);
    sflag(p, SF_C, r > 0xFF);
    snz(p, (uint8_t)r);
    return (uint8_t)r;
}
static uint8_t s_sbc(snes_spc *p, uint8_t a, uint8_t b) { return s_adc(p, a, (uint8_t)~b); }
static void s_cmp(snes_spc *p, uint8_t a, uint8_t b)
{
    int r = a - b;
    sflag(p, SF_C, r >= 0);
    snz(p, (uint8_t)r);
}
/* g: 0 OR 1 AND 2 EOR 3 CMP 4 ADC 5 SBC; returns result (a unchanged for CMP) */
static uint8_t s_alu(snes_spc *p, int g, uint8_t a, uint8_t b)
{
    uint8_t r;
    switch (g) {
    case 0: r = a | b; snz(p, r); return r;
    case 1: r = a & b; snz(p, r); return r;
    case 2: r = a ^ b; snz(p, r); return r;
    case 3: s_cmp(p, a, b); return a;
    case 4: return s_adc(p, a, b);
    default: return s_sbc(p, a, b);
    }
}
static uint8_t s_shift(snes_spc *p, int kind, uint8_t v)
{
    uint8_t r;
    switch (kind) {
    case 0: sflag(p, SF_C, v & 0x80u); r = (uint8_t)(v << 1); break;                         /* ASL */
    case 1: r = (uint8_t)((v << 1) | (p->psw & SF_C)); sflag(p, SF_C, v & 0x80u); break;     /* ROL */
    case 2: sflag(p, SF_C, v & 1u); r = (uint8_t)(v >> 1); break;                            /* LSR */
    case 3: r = (uint8_t)((v >> 1) | ((p->psw & SF_C) << 7)); sflag(p, SF_C, v & 1u); break; /* ROR */
    case 4: r = (uint8_t)(v - 1u); break;                                                    /* DEC */
    default: r = (uint8_t)(v + 1u); break;                                                   /* INC */
    }
    snz(p, r);
    return r;
}

static int s_branch(hs_snes *s, int cond)
{
    int8_t o = (int8_t)sfetch(s);
    if (cond) { s->spc.pc = (uint16_t)(s->spc.pc + o); return 2; }
    return 0;
}

/* ALU group addressing: returns operand for columns 4..7 of rows 0..B */
static uint16_t s_ea(hs_snes *s, uint8_t op)
{
    snes_spc *p = SPC;
    int odd = (op >> 4) & 1;
    switch (op & 0x0Fu) {
    case 0x4: { uint8_t d = sfetch(s); return dpa(p, odd ? (uint8_t)(d + p->x) : d); }
    case 0x5: { uint16_t a = sfetch16(s); return (uint16_t)(odd ? a + p->x : a); }
    case 0x6: if (!odd) return dpa(p, p->x); return (uint16_t)(sfetch16(s) + p->y);
    default: {
        uint8_t d = sfetch(s);
        if (!odd) return rdp16(s, (uint8_t)(d + p->x));
        return (uint16_t)(rdp16(s, d) + p->y);
    }
    }
}

int snes_spc_step(hs_snes *s)
{
    snes_spc *p = SPC;
    uint8_t op;
    int cyc, extra = 0, i;
    if (p->stopped) {
        cyc = 2;
    } else {
        op = sfetch(s);
        cyc = k_spc_cycles[op];
        switch (op) {
        /* ---- ALU with A: columns 4-8 of rows 0..B ---- */
        case 0x04: case 0x05: case 0x06: case 0x07: case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x24: case 0x25: case 0x26: case 0x27: case 0x34: case 0x35: case 0x36: case 0x37:
        case 0x44: case 0x45: case 0x46: case 0x47: case 0x54: case 0x55: case 0x56: case 0x57:
        case 0x64: case 0x65: case 0x66: case 0x67: case 0x74: case 0x75: case 0x76: case 0x77:
        case 0x84: case 0x85: case 0x86: case 0x87: case 0x94: case 0x95: case 0x96: case 0x97:
        case 0xA4: case 0xA5: case 0xA6: case 0xA7: case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            uint16_t ea = s_ea(s, op);
            p->a = s_alu(p, op >> 5, p->a, spc_read(s, ea));
            break;
        }
        case 0x08: case 0x28: case 0x48: case 0x68: case 0x88: case 0xA8:
            p->a = s_alu(p, op >> 5, p->a, sfetch(s));
            break;
        case 0x18: case 0x38: case 0x58: case 0x78: case 0x98: case 0xB8: {   /* d,#i */
            uint8_t imm = sfetch(s), d = sfetch(s);
            uint8_t r = s_alu(p, op >> 5, rdp(s, d), imm);
            if ((op >> 5) != 3) wdp(s, d, r);
            break;
        }
        case 0x09: case 0x29: case 0x49: case 0x69: case 0x89: case 0xA9: {   /* dd,ds */
            uint8_t ds = sfetch(s), dd = sfetch(s);
            uint8_t src = rdp(s, ds);
            uint8_t r = s_alu(p, op >> 5, rdp(s, dd), src);
            if ((op >> 5) != 3) wdp(s, dd, r);
            break;
        }
        case 0x19: case 0x39: case 0x59: case 0x79: case 0x99: case 0xB9: {   /* (X),(Y) */
            uint8_t src = spc_read(s, dpa(p, p->y));
            uint8_t r = s_alu(p, op >> 5, spc_read(s, dpa(p, p->x)), src);
            if ((op >> 5) != 3) spc_write(s, dpa(p, p->x), r);
            break;
        }
        /* ---- MOV A <- / -> memory ---- */
        case 0xE4: case 0xE5: case 0xE6: case 0xE7: case 0xF4: case 0xF5: case 0xF6: case 0xF7:
            p->a = spc_read(s, s_ea(s, op)); snz(p, p->a); break;
        case 0xC4: case 0xC5: case 0xC6: case 0xC7: case 0xD4: case 0xD5: case 0xD6: case 0xD7:
            spc_write(s, s_ea(s, op), p->a); break;
        case 0xE8: p->a = sfetch(s); snz(p, p->a); break;
        case 0xCD: p->x = sfetch(s); snz(p, p->x); break;
        case 0x8D: p->y = sfetch(s); snz(p, p->y); break;
        case 0xBF: p->a = spc_read(s, dpa(p, p->x)); p->x++; snz(p, p->a); break;
        case 0xAF: spc_write(s, dpa(p, p->x), p->a); p->x++; break;
        case 0xF8: p->x = rdp(s, sfetch(s)); snz(p, p->x); break;
        case 0xF9: p->x = rdp(s, (uint8_t)(sfetch(s) + p->y)); snz(p, p->x); break;
        case 0xE9: p->x = spc_read(s, sfetch16(s)); snz(p, p->x); break;
        case 0xEB: p->y = rdp(s, sfetch(s)); snz(p, p->y); break;
        case 0xFB: p->y = rdp(s, (uint8_t)(sfetch(s) + p->x)); snz(p, p->y); break;
        case 0xEC: p->y = spc_read(s, sfetch16(s)); snz(p, p->y); break;
        case 0xD8: wdp(s, sfetch(s), p->x); break;
        case 0xD9: wdp(s, (uint8_t)(sfetch(s) + p->y), p->x); break;
        case 0xC9: spc_write(s, sfetch16(s), p->x); break;
        case 0xCB: wdp(s, sfetch(s), p->y); break;
        case 0xDB: wdp(s, (uint8_t)(sfetch(s) + p->x), p->y); break;
        case 0xCC: spc_write(s, sfetch16(s), p->y); break;
        case 0xFA: { uint8_t ds = sfetch(s), dd = sfetch(s); uint8_t v = rdp(s, ds); wdp(s, dd, v); break; }
        case 0x8F: { uint8_t imm = sfetch(s), d = sfetch(s); wdp(s, d, imm); break; }
        /* ---- register moves ---- */
        case 0x7D: p->a = p->x; snz(p, p->a); break;
        case 0xDD: p->a = p->y; snz(p, p->a); break;
        case 0x5D: p->x = p->a; snz(p, p->x); break;
        case 0xFD: p->y = p->a; snz(p, p->y); break;
        case 0x9D: p->x = p->sp; snz(p, p->x); break;
        case 0xBD: p->sp = p->x; break;
        /* ---- compares with X / Y ---- */
        case 0xC8: s_cmp(p, p->x, sfetch(s)); break;
        case 0x3E: s_cmp(p, p->x, rdp(s, sfetch(s))); break;
        case 0x1E: s_cmp(p, p->x, spc_read(s, sfetch16(s))); break;
        case 0xAD: s_cmp(p, p->y, sfetch(s)); break;
        case 0x7E: s_cmp(p, p->y, rdp(s, sfetch(s))); break;
        case 0x5E: s_cmp(p, p->y, spc_read(s, sfetch16(s))); break;
        /* ---- shifts / inc / dec on memory ---- */
        case 0x0B: case 0x2B: case 0x4B: case 0x6B: case 0x8B: case 0xAB: {
            uint8_t d = sfetch(s);
            wdp(s, d, s_shift(p, (op >> 5), rdp(s, d)));
            break;
        }
        case 0x1B: case 0x3B: case 0x5B: case 0x7B: case 0x9B: case 0xBB: {
            uint8_t d = (uint8_t)(sfetch(s) + p->x);
            wdp(s, d, s_shift(p, (op >> 5), rdp(s, d)));
            break;
        }
        case 0x0C: case 0x2C: case 0x4C: case 0x6C: case 0x8C: case 0xAC: {
            uint16_t a = sfetch16(s);
            spc_write(s, a, s_shift(p, (op >> 5), spc_read(s, a)));
            break;
        }
        case 0x1C: p->a = s_shift(p, 0, p->a); break;
        case 0x3C: p->a = s_shift(p, 1, p->a); break;
        case 0x5C: p->a = s_shift(p, 2, p->a); break;
        case 0x7C: p->a = s_shift(p, 3, p->a); break;
        case 0x9C: p->a = s_shift(p, 4, p->a); break;
        case 0xBC: p->a = s_shift(p, 5, p->a); break;
        case 0x1D: p->x--; snz(p, p->x); break;
        case 0x3D: p->x++; snz(p, p->x); break;
        case 0xDC: p->y--; snz(p, p->y); break;
        case 0xFC: p->y++; snz(p, p->y); break;
        /* ---- 16-bit ---- */
        case 0x1A: case 0x3A: {   /* DECW / INCW */
            uint8_t d = sfetch(s);
            uint16_t w = rdp16(s, d);
            w = (uint16_t)(op == 0x3A ? w + 1u : w - 1u);
            wdp(s, d, (uint8_t)w);
            wdp(s, (uint8_t)(d + 1u), (uint8_t)(w >> 8));
            sflag(p, SF_N, w & 0x8000u); sflag(p, SF_Z, w == 0);
            break;
        }
        case 0x5A: {  /* CMPW YA,d */
            uint16_t w = rdp16(s, sfetch(s));
            uint16_t ya = (uint16_t)((p->y << 8) | p->a);
            int32_t r = (int32_t)ya - (int32_t)w;
            sflag(p, SF_C, r >= 0); sflag(p, SF_N, r & 0x8000); sflag(p, SF_Z, (uint16_t)r == 0);
            break;
        }
        case 0x7A: case 0x9A: {  /* ADDW / SUBW */
            uint16_t w = rdp16(s, sfetch(s));
            uint16_t ya = (uint16_t)((p->y << 8) | p->a);
            uint32_t b = (op == 0x7A) ? w : (uint16_t)~w;
            uint32_t r = (uint32_t)ya + b + (op == 0x7A ? 0u : 1u);
            sflag(p, SF_C, r > 0xFFFFu);
            sflag(p, SF_V, (~(ya ^ b) & (ya ^ r) & 0x8000u) != 0);
            sflag(p, SF_H, ((ya ^ b ^ r) & 0x1000u) != 0);
            p->a = (uint8_t)r; p->y = (uint8_t)(r >> 8);
            sflag(p, SF_N, r & 0x8000u); sflag(p, SF_Z, (uint16_t)r == 0);
            break;
        }
        case 0xBA: {  /* MOVW YA,d */
            uint16_t w = rdp16(s, sfetch(s));
            p->a = (uint8_t)w; p->y = (uint8_t)(w >> 8);
            sflag(p, SF_N, w & 0x8000u); sflag(p, SF_Z, w == 0);
            break;
        }
        case 0xDA: { uint8_t d = sfetch(s); wdp(s, d, p->a); wdp(s, (uint8_t)(d + 1u), p->y); break; }
        case 0xCF: {  /* MUL YA */
            uint16_t r = (uint16_t)(p->y * p->a);
            p->a = (uint8_t)r; p->y = (uint8_t)(r >> 8);
            snz(p, p->y);
            break;
        }
        case 0x9E: {  /* DIV YA,X */
            uint32_t ya = (uint32_t)((p->y << 8) | p->a), x = p->x;
            sflag(p, SF_V, p->y >= p->x);
            sflag(p, SF_H, (p->y & 15u) >= (p->x & 15u));
            if (p->y < (x << 1)) {
                p->a = (uint8_t)(ya / x);
                p->y = (uint8_t)(ya % x);
            } else {
                p->a = (uint8_t)(255u - (ya - (x << 9)) / (256u - x));
                p->y = (uint8_t)(x + (ya - (x << 9)) % (256u - x));
            }
            snz(p, p->a);
            break;
        }
        case 0xDF:  /* DAA */
            if ((p->psw & SF_C) || p->a > 0x99u) { p->a = (uint8_t)(p->a + 0x60u); p->psw |= SF_C; }
            if ((p->psw & SF_H) || (p->a & 15u) > 9u) p->a = (uint8_t)(p->a + 6u);
            snz(p, p->a);
            break;
        case 0xBE:  /* DAS */
            if (!(p->psw & SF_C) || p->a > 0x99u) { p->a = (uint8_t)(p->a - 0x60u); p->psw &= (uint8_t)~SF_C; }
            if (!(p->psw & SF_H) || (p->a & 15u) > 9u) p->a = (uint8_t)(p->a - 6u);
            snz(p, p->a);
            break;
        case 0x9F: p->a = (uint8_t)((p->a >> 4) | (p->a << 4)); snz(p, p->a); break;  /* XCN */
        /* ---- bit ops ---- */
        case 0x02: case 0x22: case 0x42: case 0x62: case 0x82: case 0xA2: case 0xC2: case 0xE2: {
            uint8_t d = sfetch(s);
            wdp(s, d, (uint8_t)(rdp(s, d) | (1u << (op >> 5))));
            break;
        }
        case 0x12: case 0x32: case 0x52: case 0x72: case 0x92: case 0xB2: case 0xD2: case 0xF2: {
            uint8_t d = sfetch(s);
            wdp(s, d, (uint8_t)(rdp(s, d) & ~(1u << (op >> 5))));
            break;
        }
        case 0x03: case 0x23: case 0x43: case 0x63: case 0x83: case 0xA3: case 0xC3: case 0xE3: {
            uint8_t v = rdp(s, sfetch(s));
            extra = s_branch(s, (v >> (op >> 5)) & 1u);
            break;
        }
        case 0x13: case 0x33: case 0x53: case 0x73: case 0x93: case 0xB3: case 0xD3: case 0xF3: {
            uint8_t v = rdp(s, sfetch(s));
            extra = s_branch(s, !((v >> (op >> 5)) & 1u));
            break;
        }
        case 0x0E: case 0x4E: {  /* TSET1 / TCLR1 !a */
            uint16_t a = sfetch16(s);
            uint8_t v = spc_read(s, a);
            snz(p, (uint8_t)(p->a - v));
            spc_write(s, a, op == 0x0E ? (uint8_t)(v | p->a) : (uint8_t)(v & ~p->a));
            break;
        }
        case 0x0A: case 0x2A: case 0x4A: case 0x6A: case 0x8A: case 0xAA: case 0xCA: case 0xEA: {
            uint16_t mb = sfetch16(s);
            uint16_t a = mb & 0x1FFFu;
            int bit = mb >> 13;
            uint8_t v = spc_read(s, a);
            int m = (v >> bit) & 1;
            int c = p->psw & SF_C;
            switch (op) {
            case 0x0A: sflag(p, SF_C, c | m); break;
            case 0x2A: sflag(p, SF_C, c | !m); break;
            case 0x4A: sflag(p, SF_C, c & m); break;
            case 0x6A: sflag(p, SF_C, c & !m); break;
            case 0x8A: sflag(p, SF_C, c ^ m); break;
            case 0xAA: sflag(p, SF_C, m); break;
            case 0xCA: spc_write(s, a, (uint8_t)(c ? (v | (1u << bit)) : (v & ~(1u << bit)))); break;
            default:   spc_write(s, a, (uint8_t)(v ^ (1u << bit))); break;
            }
            break;
        }
        /* ---- flags ---- */
        case 0x20: p->psw &= (uint8_t)~SF_P; break;
        case 0x40: p->psw |= SF_P; break;
        case 0x60: p->psw &= (uint8_t)~SF_C; break;
        case 0x80: p->psw |= SF_C; break;
        case 0xA0: p->psw |= SF_I; break;
        case 0xC0: p->psw &= (uint8_t)~SF_I; break;
        case 0xE0: p->psw &= (uint8_t)~(SF_V | SF_H); break;
        case 0xED: p->psw ^= SF_C; break;
        /* ---- branches ---- */
        case 0x10: extra = s_branch(s, !(p->psw & SF_N)); break;
        case 0x30: extra = s_branch(s, (p->psw & SF_N) != 0); break;
        case 0x50: extra = s_branch(s, !(p->psw & SF_V)); break;
        case 0x70: extra = s_branch(s, (p->psw & SF_V) != 0); break;
        case 0x90: extra = s_branch(s, !(p->psw & SF_C)); break;
        case 0xB0: extra = s_branch(s, (p->psw & SF_C) != 0); break;
        case 0xD0: extra = s_branch(s, !(p->psw & SF_Z)); break;
        case 0xF0: extra = s_branch(s, (p->psw & SF_Z) != 0); break;
        case 0x2F: { int8_t o = (int8_t)sfetch(s); p->pc = (uint16_t)(p->pc + o); break; }
        case 0x2E: { uint8_t v = rdp(s, sfetch(s)); extra = s_branch(s, p->a != v); break; }
        case 0xDE: { uint8_t v = rdp(s, (uint8_t)(sfetch(s) + p->x)); extra = s_branch(s, p->a != v); break; }
        case 0x6E: { uint8_t d = sfetch(s); uint8_t v = (uint8_t)(rdp(s, d) - 1u); wdp(s, d, v); extra = s_branch(s, v != 0); break; }
        case 0xFE: p->y--; extra = s_branch(s, p->y != 0); break;
        /* ---- jumps / calls ---- */
        case 0x5F: p->pc = sfetch16(s); break;
        case 0x1F: {
            uint16_t a = (uint16_t)(sfetch16(s) + p->x);
            p->pc = (uint16_t)(spc_read(s, a) | (spc_read(s, (uint16_t)(a + 1u)) << 8));
            break;
        }
        case 0x3F: { uint16_t a = sfetch16(s); spush(s, (uint8_t)(p->pc >> 8)); spush(s, (uint8_t)p->pc); p->pc = a; break; }
        case 0x4F: { uint8_t u = sfetch(s); spush(s, (uint8_t)(p->pc >> 8)); spush(s, (uint8_t)p->pc); p->pc = (uint16_t)(0xFF00u | u); break; }
        case 0x01: case 0x11: case 0x21: case 0x31: case 0x41: case 0x51: case 0x61: case 0x71:
        case 0x81: case 0x91: case 0xA1: case 0xB1: case 0xC1: case 0xD1: case 0xE1: case 0xF1: {
            uint16_t va = (uint16_t)(0xFFDEu - ((op >> 4) << 1));
            spush(s, (uint8_t)(p->pc >> 8)); spush(s, (uint8_t)p->pc);
            p->pc = (uint16_t)(spc_read(s, va) | (spc_read(s, (uint16_t)(va + 1u)) << 8));
            break;
        }
        case 0x0F:
            spush(s, (uint8_t)(p->pc >> 8)); spush(s, (uint8_t)p->pc); spush(s, p->psw);
            p->psw = (uint8_t)((p->psw | SF_B) & ~SF_I);
            p->pc = (uint16_t)(spc_read(s, 0xFFDE) | (spc_read(s, 0xFFDF) << 8));
            break;
        case 0x6F: { uint16_t lo = spop(s); p->pc = (uint16_t)(lo | (spop(s) << 8)); break; }
        case 0x7F: { uint16_t lo; p->psw = spop(s); lo = spop(s); p->pc = (uint16_t)(lo | (spop(s) << 8)); break; }
        /* ---- stack ---- */
        case 0x0D: spush(s, p->psw); break;
        case 0x2D: spush(s, p->a); break;
        case 0x4D: spush(s, p->x); break;
        case 0x6D: spush(s, p->y); break;
        case 0x8E: p->psw = spop(s); break;
        case 0xAE: p->a = spop(s); break;
        case 0xCE: p->x = spop(s); break;
        case 0xEE: p->y = spop(s); break;
        /* ---- misc ---- */
        case 0x00: break;
        case 0xEF: case 0xFF: p->stopped = 1; break;
        default: break;   /* unreachable: all 256 opcodes decoded */
        }
    }
    cyc += extra;
    p->cycles += (uint64_t)cyc;
    for (i = 0; i < 3; i++) {
        snes_timer *t = &p->t[i];
        uint16_t per = (i == 2) ? 16u : 128u;
        t->divider = (uint16_t)(t->divider + cyc);
        if (t->divider >= per) {
            t->divider = (uint16_t)(t->divider - per);
            if (t->enabled) {
                t->stage++;
                if (t->stage == t->target) { t->stage = 0; t->counter = (uint8_t)((t->counter + 1u) & 15u); }
            }
        }
    }
    return cyc;
}

/* Run the SPC700 up to the current master-clock time. Called at the end
   of every scanline (and after CPU APU-port I/O), so the SPC and DSP write
   log are interleaved through the frame, not executed as one burst. */
void snes_apu_catchup(hs_snes *s)
{
    uint64_t target = (s->master * SNES_SPC_HZ) / SNES_MASTER_HZ;
    while (s->spc.cycles < target) snes_spc_step(s);
}

uint8_t snes_apu_cpu_read(hs_snes *s, uint8_t port)
{
    snes_apu_catchup(s);
    return s->spc.out[port & 3u];
}

void snes_apu_cpu_write(hs_snes *s, uint8_t port, uint8_t val)
{
    snes_apu_catchup(s);
    s->spc.in[port & 3u] = val;
    log_port(s, (uint8_t)(0x10u | (port & 3u)), val);
}

/* ================================================================ phase 2 */
enum { ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

static const uint16_t k_rate_period[32] = {
    0, 2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80,
    64, 48, 40, 32, 24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1
};

void hs_snes_dsp_init(hs_snes_dsp *d)
{
    memset(d, 0, sizeof *d);
    d->reg[0x6C] = 0xE0;   /* FLG: reset, mute, echo write off */
    d->noise = 0x4000;
}

static inline int clamp16(int v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

static void brr_decode_block(hs_snes_dsp *d, hs_snes_voice *vs, int vi)
{
    uint8_t hdr = d->aram[vs->brr_addr];
    int shift = hdr >> 4, filter = (hdr >> 2) & 3, i;
    for (i = 0; i < 16; i++) {
        uint8_t byte = d->aram[(uint16_t)(vs->brr_addr + 1u + (uint16_t)(i >> 1))];
        int nib = (i & 1) ? (byte & 0x0F) : (byte >> 4);
        int smp = (nib ^ 8) - 8;
        int old = vs->buf[(vs->decoded - 1u) & 31u] >> 1;
        int older = vs->buf[(vs->decoded - 2u) & 31u] >> 1;
        if (shift <= 12) smp = (smp * (1 << shift)) >> 1;
        else smp = smp < 0 ? -2048 : 0;
        switch (filter) {
        case 1: smp += old + ((-old) >> 4); break;
        case 2: smp += 2 * old + ((-3 * old) >> 5) - older + (older >> 4); break;
        case 3: smp += 2 * old + ((-13 * old) >> 6) - older + ((3 * older) >> 4); break;
        default: break;
        }
        smp = clamp16(smp);
        vs->buf[vs->decoded & 31u] = (int16_t)(uint16_t)((uint32_t)smp << 1);
        vs->decoded++;
    }
    if (hdr & 1u) {
        d->endx |= 1u << vi;
        if (hdr & 2u) vs->brr_addr = vs->loop_addr;
        else { vs->stop_at = vs->decoded; vs->brr_addr = (uint16_t)(vs->brr_addr + 9u); }
    } else {
        vs->brr_addr = (uint16_t)(vs->brr_addr + 9u);
    }
}

static void voice_key_on(hs_snes_dsp *d, int vi)
{
    hs_snes_voice *vs = &d->v[vi];
    uint16_t dir = (uint16_t)((d->reg[0x5D] << 8) + d->reg[vi * 16 + 4] * 4u);
    memset(vs->buf, 0, sizeof vs->buf);
    vs->brr_addr = (uint16_t)(d->aram[dir] | (d->aram[(uint16_t)(dir + 1u)] << 8));
    vs->loop_addr = (uint16_t)(d->aram[(uint16_t)(dir + 2u)] | (d->aram[(uint16_t)(dir + 3u)] << 8));
    vs->decoded = 3;
    vs->pos = 0;
    vs->stop_at = 0;
    vs->env = 0;
    vs->env_mode = ENV_ATTACK;
    vs->active = 1;
    d->endx &= ~(1u << vi);
    brr_decode_block(d, vs, vi);
}

static void dsp_write(hs_snes_dsp *d, uint8_t reg, uint8_t val)
{
    int i;
    d->reg[reg & 0x7Fu] = val;
    switch (reg) {
    case 0x4C:
        for (i = 0; i < 8; i++) if (val & (1u << i)) voice_key_on(d, i);
        break;
    case 0x5C:
        for (i = 0; i < 8; i++) if (val & (1u << i)) d->v[i].env_mode = ENV_RELEASE;
        break;
    case 0x7C: d->endx = 0; break;
    case 0x6C:
        if (val & 0x80u) for (i = 0; i < 8; i++) { d->v[i].env_mode = ENV_RELEASE; d->v[i].env = 0; }
        break;
    default: break;
    }
}

static void envelope(hs_snes_dsp *d, hs_snes_voice *vs, const uint8_t *R, const uint8_t *tick)
{
    int env = vs->env;
    if (vs->env_mode == ENV_RELEASE) {
        env -= 8;
        if (env < 0) env = 0;
        vs->env = (int16_t)env;
        return;
    }
    if (R[5] & 0x80u) {
        int sl = R[6] >> 5;
        switch (vs->env_mode) {
        case ENV_ATTACK: {
            int rate = ((R[5] & 0x0F) << 1) + 1;
            if (tick[rate]) env += (rate == 31) ? 1024 : 32;
            if (env >= 0x7E0) { if (env > 0x7FF) env = 0x7FF; vs->env_mode = ENV_DECAY; }
            break;
        }
        case ENV_DECAY: {
            int rate = (((R[5] >> 4) & 7) << 1) + 16;
            if (tick[rate]) env -= ((env - 1) >> 8) + 1;
            if ((env >> 8) <= sl) vs->env_mode = ENV_SUSTAIN;
            break;
        }
        default: {
            int rate = R[6] & 31;
            if (tick[rate]) env -= ((env - 1) >> 8) + 1;
            break;
        }
        }
    } else {
        uint8_t g = R[7];
        if (!(g & 0x80u)) {
            env = (g & 0x7F) << 4;
        } else if (tick[g & 31]) {
            switch ((g >> 5) & 3) {
            case 0: env -= 32; break;
            case 1: env -= ((env - 1) >> 8) + 1; break;
            case 2: env += 32; break;
            default: env += (env < 0x600) ? 32 : 8; break;
            }
        }
    }
    if (env < 0) env = 0;
    if (env > 0x7FF) env = 0x7FF;
    vs->env = (int16_t)env;
    (void)d;
}

static void dsp_sample(hs_snes_dsp *d, int *out_l, int *out_r)
{
    uint8_t tick[32];
    int32_t mainl = 0, mainr = 0, echol = 0, echor = 0;
    int i, r;
    uint8_t flg = d->reg[0x6C], pmon = d->reg[0x2D], non = d->reg[0x3D], eon = d->reg[0x4D], koff = d->reg[0x5C];
    int prev_out = 0;

    d->global_counter++;
    tick[0] = 0;
    for (r = 1; r < 32; r++) {
        if (d->rate_cnt[r] <= 1u) { d->rate_cnt[r] = k_rate_period[r]; tick[r] = 1; }
        else { d->rate_cnt[r]--; tick[r] = 0; }
    }
    if (tick[flg & 31]) {
        int fb = ((d->noise << 13) ^ (d->noise << 14)) & 0x4000;
        d->noise = (uint16_t)(fb | (d->noise >> 1));
    }

    for (i = 0; i < 8; i++) {
        hs_snes_voice *vs = &d->v[i];
        uint8_t *R = &d->reg[i * 16];
        int pitch, smp, out, guard;
        uint32_t idx, f;
        if (!vs->active) { vs->out = 0; prev_out = 0; R[8] = 0; R[9] = 0; continue; }
        if (koff & (1u << i)) vs->env_mode = ENV_RELEASE;
        pitch = (R[2] | (R[3] << 8)) & 0x3FFF;
        if (i > 0 && (pmon & (1u << i))) {
            pitch += ((prev_out >> 5) * pitch) >> 10;
            if (pitch < 0) pitch = 0;
            if (pitch > 0x7FFF) pitch = 0x7FFF;
        }
        idx = vs->pos >> 12;
        f = (vs->pos >> 4) & 0xFFu;
        if (non & (1u << i)) {
            smp = (int16_t)(uint16_t)(d->noise << 1);
        } else {
            int s0 = vs->buf[idx & 31u] >> 1, s1 = vs->buf[(idx + 1u) & 31u] >> 1;
            int s2 = vs->buf[(idx + 2u) & 31u] >> 1, s3 = vs->buf[(idx + 3u) & 31u] >> 1;
            smp = (k_gauss[255 - f] * s0) >> 11;
            smp += (k_gauss[511 - f] * s1) >> 11;
            smp += (k_gauss[256 + f] * s2) >> 11;
            smp += (k_gauss[f] * s3) >> 11;
            smp = clamp16(smp) * 2;
            smp = clamp16(smp);
        }
        envelope(d, vs, R, tick);
        out = (smp * vs->env) >> 11;
        vs->out = (int16_t)out;
        prev_out = out;
        R[8] = (uint8_t)(vs->env >> 4);
        R[9] = (uint8_t)(out >> 8);
        {
            int32_t vl = (out * (int8_t)R[0]) >> 7, vr = (out * (int8_t)R[1]) >> 7;
            /* Delay the clamp until after MVOL + echo so mid-mix overflow
               cannot turn a loud chord into a periodic crackle. */
            mainl += vl; mainr += vr;
            if (eon & (1u << i)) { echol += vl; echor += vr; }
        }
        /* advance pitch counter; keep >= 4 decoded samples ahead */
        vs->pos += (uint32_t)pitch;
        for (guard = 0; guard < 3 && (vs->pos >> 12) + 4u > vs->decoded; guard++) {
            if (vs->stop_at) break;
            brr_decode_block(d, vs, i);
        }
        if (vs->stop_at && (vs->pos >> 12) + 3u >= vs->stop_at) {
            vs->active = 0; vs->env = 0; vs->env_mode = ENV_RELEASE;
        }
        if (vs->decoded > 0x10000u) {       /* rebase: ring index is mod 32 */
            vs->decoded -= 0x8000u;
            vs->pos -= 0x8000u << 12;
            if (vs->stop_at) vs->stop_at -= 0x8000u;
        }
    }

    /* echo + 8-tap FIR */
    {
        uint16_t esa = (uint16_t)(d->reg[0x6D] << 8);
        uint32_t len = (d->reg[0x7D] & 15u) ? (uint32_t)(d->reg[0x7D] & 15u) * 2048u : 4u;
        uint16_t a = (uint16_t)(esa + d->echo_pos);
        int el = (int16_t)(uint16_t)(d->aram[a] | (d->aram[(uint16_t)(a + 1u)] << 8)) >> 1;
        int er = (int16_t)(uint16_t)(d->aram[(uint16_t)(a + 2u)] | (d->aram[(uint16_t)(a + 3u)] << 8)) >> 1;
        int firl = 0, firr = 0, k, eol, eor, inl, inr;
        d->fir_pos = (uint8_t)((d->fir_pos + 1u) & 7u);
        d->fir_l[d->fir_pos] = (int16_t)el;
        d->fir_r[d->fir_pos] = (int16_t)er;
        for (k = 0; k < 8; k++) {
            int c = (int8_t)d->reg[k * 16 + 0x0F];
            int hidx = (d->fir_pos + 1 + k) & 7;
            firl += (d->fir_l[hidx] * c) >> 6;
            firr += (d->fir_r[hidx] * c) >> 6;
        }
        firl = clamp16(firl); firr = clamp16(firr);
        eol = (firl * (int8_t)d->reg[0x2C]) >> 7;
        eor = (firr * (int8_t)d->reg[0x3C]) >> 7;
        inl = clamp16(echol + ((firl * (int8_t)d->reg[0x0D]) >> 7)) & ~1;
        inr = clamp16(echor + ((firr * (int8_t)d->reg[0x0D]) >> 7)) & ~1;
        if (!(flg & 0x20u)) {
            d->aram[a] = (uint8_t)inl; d->aram[(uint16_t)(a + 1u)] = (uint8_t)(inl >> 8);
            d->aram[(uint16_t)(a + 2u)] = (uint8_t)inr; d->aram[(uint16_t)(a + 3u)] = (uint8_t)(inr >> 8);
        }
        d->echo_pos = (uint16_t)(d->echo_pos + 4u);
        if (d->echo_pos >= len) d->echo_pos = 0;
        mainl = ((mainl * (int8_t)d->reg[0x0C]) >> 7) + eol;
        mainr = ((mainr * (int8_t)d->reg[0x1C]) >> 7) + eor;
    }
    if (flg & 0x40u) { mainl = 0; mainr = 0; }
    mainl >>= SNES_MIX_HEADROOM_SHIFT;
    mainr >>= SNES_MIX_HEADROOM_SHIFT;
    {
        int32_t al = mainl < 0 ? -mainl : mainl;
        int32_t ar = mainr < 0 ? -mainr : mainr;
        if ((uint32_t)al > d->peak) d->peak = (uint32_t)al;
        if ((uint32_t)ar > d->peak) d->peak = (uint32_t)ar;
        if (mainl > 32767 || mainl < -32768) d->clips++;
        else if (mainr > 32767 || mainr < -32768) d->clips++;
    }
    d->reg[0x7C] = (uint8_t)d->endx;
    *out_l = clamp16((int)mainl);
    *out_r = clamp16((int)mainr);
}

static void apply_event(hs_snes_dsp *d, const hs_snes_apu_event *e)
{
    if (e->kind == 0) d->aram[e->addr] = e->val;
    else dsp_write(d, (uint8_t)e->addr, e->val);
}

int hs_snes_synth_audio_phase(const hs_snes_slot *slot, hs_snes_dsp *d, int16_t *pcm, int max_frames)
{
    /* 32 kHz -> 48 kHz: 2 input samples produce 3 output samples. */
    static const uint8_t k_w[3] = {0, 85, 171};
    uint32_t ev = 0, n = slot->apu_log_count, smp, produced = 0, want;
    uint32_t begin = slot->sample_begin;
    int out = 0;

    if (n > HS_SNES_APU_LOG_MAX) n = HS_SNES_APU_LOG_MAX;

    /* Fractional 32000/60.0988 samples: Bresenham 532/533, remainder
       9867296 / SNES_MASTER_HZ. The SPC catch-up itself stays per-scanline
       in phase 0 so video and machine-state hashes do not move. */
    want = SNES_SAMPLES_PER_FRAME_BASE;
    d->samp_err += SNES_SAMPLES_BRES_STEP;
    if (d->samp_err >= SNES_SAMPLES_BRES_MOD) {
        d->samp_err -= SNES_SAMPLES_BRES_MOD;
        want++;
    }

    smp = begin;
    while (produced < want) {
        int l, r, ph;
        while (ev < n && (int32_t)(slot->apu_log[ev].sample - smp) <= 0) {
            apply_event(d, &slot->apu_log[ev++]);
        }
        dsp_sample(d, &l, &r);
        for (ph = (int)d->rs_phase; ph < 3; ph += 2) {
            int w = k_w[ph];
            if (out < max_frames) {
                pcm[2 * out] = (int16_t)((d->prev_l * (256 - w) + l * w) >> 8);
                pcm[2 * out + 1] = (int16_t)((d->prev_r * (256 - w) + r * w) >> 8);
                out++;
            }
            d->total_out++;
            d->rs_phase = (uint32_t)(ph + 2);
        }
        d->rs_phase -= 3u;
        d->prev_l = (int16_t)l;
        d->prev_r = (int16_t)r;
        produced++;
        smp++;
    }
    while (ev < n) apply_event(d, &slot->apu_log[ev++]);
    d->sample_now = begin + want;
    d->last_n32 = want;
    d->last_n48 = (uint32_t)out;
#ifdef HS_SNES_AUDIO_DEBUG
    if (d->dbg_n < HS_SNES_DBG_FRAMES) {
        d->dbg_n32[d->dbg_n] = (uint16_t)want;
        d->dbg_n48[d->dbg_n] = (uint16_t)out;
        d->dbg_n++;
    }
#endif
    return out;
}
