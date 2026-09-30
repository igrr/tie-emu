/* OPL3 (YMF262) synthesis: a port of ymf262 from ymfm (https://github.com/aaronsgiles/ymfm,
 * BSD-3-Clause, Copyright (c) 2021 Aaron Giles, see LICENSE.ymfm; the tables are ymfm's) cut down
 * to the OPL3 and to the two outputs the SB16 wires, with what ymfm looks up in the registers
 * every sample worked out when they change. The output is the same as ymfm's, sample for sample.
 *
 * As in ymfm, a register write takes effect at the start of the next sample ("prepare"): the
 * operators' parameters are recomputed from the registers, key on/off applied, and the channels
 * that can be heard found. Prepare also runs every 4097 samples to drop channels that went quiet. */
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "opl3.h"

enum { EG_ATTACK = 1, EG_DECAY, EG_SUSTAIN, EG_RELEASE };
enum { KEYON_NORMAL, KEYON_RHYTHM };

#define EG_QUIET 0x380
#define ENV_OFF 0xffff
#define PHASE_STEP_DYNAMIC 1    /* recomputed every sample: vibrato (LFO PM) is on */

typedef struct {
    uint32_t phase;             /* 10.10 */
    uint32_t phase_step;        /* or PHASE_STEP_DYNAMIC */
    uint16_t att;               /* envelope attenuation, 4.6 */
    uint8_t state;              /* EG_* */
    uint8_t key, keyon_live;    /* key state; live key-on bits by KEYON_* */
    uint8_t eg_rate;            /* rate[state] */
    uint16_t eg_mask;           /* the envelope steps in samples where (env_counter & eg_mask) == 0 */
    uint16_t env;               /* attenuation with tremolo and total level, 4.8; ENV_OFF when quiet */
    /* from the registers, at prepare */
    const uint16_t *wave;
    uint32_t tl;                /* total level * 8 + key scale level */
    uint32_t am_mask;           /* ~0 when tremolo (LFO AM) is on */
    uint32_t sustain;
    uint8_t rate[EG_RELEASE + 1];
    uint32_t block_freq, multiple;  /* for the phase step with vibrato */
    uint16_t opoffs, choffs;    /* register offsets of the operator and of the channel it is in */
} op_t;

typedef struct {
    op_t *op[4];                /* 2 (op[2] NULL) or 4 operators, or none (the second half of a 4-op pair) */
    int16_t fb[2], fb_in;       /* operator 1's feedback */
    uint16_t choffs;
    /* from the registers, at prepare */
    uint8_t fb_shift;           /* 10 - feedback, or 0 for none */
    uint8_t alg;
    bool out_any, out_l, out_r;
} chan_t;

struct opl3 {
    uint8_t regs[0x200];
    uint16_t address;
    /* ymfm's waveforms, attenuation 4.8 with the sign moved from bit 15 to bit 13, and the
     * volume for (waveform + envelope attenuation) that ymfm's attenuation_to_volume gives, signed */
    uint16_t wave[8][1024];
    int16_t volume[0x4000];
    op_t op[36];
    chan_t ch[18];
    uint32_t env_counter;       /* x.2 */
    bool modified;
    uint32_t prepare_count;
    uint32_t active;            /* channels that can be heard, at prepare */
    uint16_t lfo_am_counter, lfo_pm_counter;
    uint8_t lfo_am;
    uint32_t noise;
    /* from the registers, at prepare */
    bool rhythm;
    uint32_t am_depth, pm_depth;
};

/* ---- ymfm's tables ---- */

static const uint16_t s_sin[256] = {
    0x859,0x6c3,0x607,0x58b,0x52e,0x4e4,0x4a6,0x471,0x443,0x41a,0x3f5,0x3d3,0x3b5,0x398,0x37e,0x365,
    0x34e,0x339,0x324,0x311,0x2ff,0x2ed,0x2dc,0x2cd,0x2bd,0x2af,0x2a0,0x293,0x286,0x279,0x26d,0x261,
    0x256,0x24b,0x240,0x236,0x22c,0x222,0x218,0x20f,0x206,0x1fd,0x1f5,0x1ec,0x1e4,0x1dc,0x1d4,0x1cd,
    0x1c5,0x1be,0x1b7,0x1b0,0x1a9,0x1a2,0x19b,0x195,0x18f,0x188,0x182,0x17c,0x177,0x171,0x16b,0x166,
    0x160,0x15b,0x155,0x150,0x14b,0x146,0x141,0x13c,0x137,0x133,0x12e,0x129,0x125,0x121,0x11c,0x118,
    0x114,0x10f,0x10b,0x107,0x103,0x0ff,0x0fb,0x0f8,0x0f4,0x0f0,0x0ec,0x0e9,0x0e5,0x0e2,0x0de,0x0db,
    0x0d7,0x0d4,0x0d1,0x0cd,0x0ca,0x0c7,0x0c4,0x0c1,0x0be,0x0bb,0x0b8,0x0b5,0x0b2,0x0af,0x0ac,0x0a9,
    0x0a7,0x0a4,0x0a1,0x09f,0x09c,0x099,0x097,0x094,0x092,0x08f,0x08d,0x08a,0x088,0x086,0x083,0x081,
    0x07f,0x07d,0x07a,0x078,0x076,0x074,0x072,0x070,0x06e,0x06c,0x06a,0x068,0x066,0x064,0x062,0x060,
    0x05e,0x05c,0x05b,0x059,0x057,0x055,0x053,0x052,0x050,0x04e,0x04d,0x04b,0x04a,0x048,0x046,0x045,
    0x043,0x042,0x040,0x03f,0x03e,0x03c,0x03b,0x039,0x038,0x037,0x035,0x034,0x033,0x031,0x030,0x02f,
    0x02e,0x02d,0x02b,0x02a,0x029,0x028,0x027,0x026,0x025,0x024,0x023,0x022,0x021,0x020,0x01f,0x01e,
    0x01d,0x01c,0x01b,0x01a,0x019,0x018,0x017,0x017,0x016,0x015,0x014,0x014,0x013,0x012,0x011,0x011,
    0x010,0x00f,0x00f,0x00e,0x00d,0x00d,0x00c,0x00c,0x00b,0x00a,0x00a,0x009,0x009,0x008,0x008,0x007,
    0x007,0x007,0x006,0x006,0x005,0x005,0x005,0x004,0x004,0x004,0x003,0x003,0x003,0x002,0x002,0x002,
    0x002,0x001,0x001,0x001,0x001,0x001,0x001,0x001,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,
};

/* 10-bit mantissas with the implicit 0x400, shifted left by 2, in reverse order */
#define X(a) (((a) | 0x400) << 2)
static const uint16_t s_power[256] = {
    X(0x3fa),X(0x3f5),X(0x3ef),X(0x3ea),X(0x3e4),X(0x3df),X(0x3da),X(0x3d4),
    X(0x3cf),X(0x3c9),X(0x3c4),X(0x3bf),X(0x3b9),X(0x3b4),X(0x3ae),X(0x3a9),
    X(0x3a4),X(0x39f),X(0x399),X(0x394),X(0x38f),X(0x38a),X(0x384),X(0x37f),
    X(0x37a),X(0x375),X(0x370),X(0x36a),X(0x365),X(0x360),X(0x35b),X(0x356),
    X(0x351),X(0x34c),X(0x347),X(0x342),X(0x33d),X(0x338),X(0x333),X(0x32e),
    X(0x329),X(0x324),X(0x31f),X(0x31a),X(0x315),X(0x310),X(0x30b),X(0x306),
    X(0x302),X(0x2fd),X(0x2f8),X(0x2f3),X(0x2ee),X(0x2e9),X(0x2e5),X(0x2e0),
    X(0x2db),X(0x2d6),X(0x2d2),X(0x2cd),X(0x2c8),X(0x2c4),X(0x2bf),X(0x2ba),
    X(0x2b5),X(0x2b1),X(0x2ac),X(0x2a8),X(0x2a3),X(0x29e),X(0x29a),X(0x295),
    X(0x291),X(0x28c),X(0x288),X(0x283),X(0x27f),X(0x27a),X(0x276),X(0x271),
    X(0x26d),X(0x268),X(0x264),X(0x25f),X(0x25b),X(0x257),X(0x252),X(0x24e),
    X(0x249),X(0x245),X(0x241),X(0x23c),X(0x238),X(0x234),X(0x230),X(0x22b),
    X(0x227),X(0x223),X(0x21e),X(0x21a),X(0x216),X(0x212),X(0x20e),X(0x209),
    X(0x205),X(0x201),X(0x1fd),X(0x1f9),X(0x1f5),X(0x1f0),X(0x1ec),X(0x1e8),
    X(0x1e4),X(0x1e0),X(0x1dc),X(0x1d8),X(0x1d4),X(0x1d0),X(0x1cc),X(0x1c8),
    X(0x1c4),X(0x1c0),X(0x1bc),X(0x1b8),X(0x1b4),X(0x1b0),X(0x1ac),X(0x1a8),
    X(0x1a4),X(0x1a0),X(0x19c),X(0x199),X(0x195),X(0x191),X(0x18d),X(0x189),
    X(0x185),X(0x181),X(0x17e),X(0x17a),X(0x176),X(0x172),X(0x16f),X(0x16b),
    X(0x167),X(0x163),X(0x160),X(0x15c),X(0x158),X(0x154),X(0x151),X(0x14d),
    X(0x149),X(0x146),X(0x142),X(0x13e),X(0x13b),X(0x137),X(0x134),X(0x130),
    X(0x12c),X(0x129),X(0x125),X(0x122),X(0x11e),X(0x11b),X(0x117),X(0x114),
    X(0x110),X(0x10c),X(0x109),X(0x106),X(0x102),X(0x0ff),X(0x0fb),X(0x0f8),
    X(0x0f4),X(0x0f1),X(0x0ed),X(0x0ea),X(0x0e7),X(0x0e3),X(0x0e0),X(0x0dc),
    X(0x0d9),X(0x0d6),X(0x0d2),X(0x0cf),X(0x0cc),X(0x0c8),X(0x0c5),X(0x0c2),
    X(0x0be),X(0x0bb),X(0x0b8),X(0x0b5),X(0x0b1),X(0x0ae),X(0x0ab),X(0x0a8),
    X(0x0a4),X(0x0a1),X(0x09e),X(0x09b),X(0x098),X(0x094),X(0x091),X(0x08e),
    X(0x08b),X(0x088),X(0x085),X(0x082),X(0x07e),X(0x07b),X(0x078),X(0x075),
    X(0x072),X(0x06f),X(0x06c),X(0x069),X(0x066),X(0x063),X(0x060),X(0x05d),
    X(0x05a),X(0x057),X(0x054),X(0x051),X(0x04e),X(0x04b),X(0x048),X(0x045),
    X(0x042),X(0x03f),X(0x03c),X(0x039),X(0x036),X(0x033),X(0x030),X(0x02d),
    X(0x02a),X(0x028),X(0x025),X(0x022),X(0x01f),X(0x01c),X(0x019),X(0x016),
    X(0x014),X(0x011),X(0x00e),X(0x00b),X(0x008),X(0x006),X(0x003),X(0x000),
};
#undef X

/* by rate: 8 4-bit envelope increments, one per step */
static const uint32_t s_increment[64] = {
    0x00000000, 0x00000000, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x11101110, 0x11101110,
    0x10101010, 0x10111010, 0x11101110, 0x11111110, 0x10101010, 0x10111010, 0x11101110, 0x11111110,
    0x10101010, 0x10111010, 0x11101110, 0x11111110, 0x10101010, 0x10111010, 0x11101110, 0x11111110,
    0x10101010, 0x10111010, 0x11101110, 0x11111110, 0x10101010, 0x10111010, 0x11101110, 0x11111110,
    0x10101010, 0x10111010, 0x11101110, 0x11111110, 0x10101010, 0x10111010, 0x11101110, 0x11111110,
    0x10101010, 0x10111010, 0x11101110, 0x11111110, 0x10101010, 0x10111010, 0x11101110, 0x11111110,
    0x11111111, 0x21112111, 0x21212121, 0x22212221, 0x22222222, 0x42224222, 0x42424242, 0x44424442,
    0x44444444, 0x84448444, 0x84848484, 0x88848884, 0x88888888, 0x88888888, 0x88888888, 0x88888888,
};

/* ---- registers ---- */

static uint32_t op_offset(int opnum)
{
    return (uint32_t)((opnum % 18) + 2 * ((opnum % 18) / 6) + 0x100 * (opnum / 18));
}

static uint32_t ch_offset(int chnum)
{
    return (uint32_t)((chnum % 9) + 0x100 * (chnum / 9));
}

static uint32_t block_freq(const opl3_t *o, uint32_t choffs)
{
    return (uint32_t)(o->regs[0xb0 + choffs] & 0x1f) << 8 | o->regs[0xa0 + choffs];
}

static uint32_t newflag(const opl3_t *o)
{
    return o->regs[0x105] & 1;
}

static uint32_t effective_rate(uint32_t raw, uint32_t ksr)
{
    return raw == 0 ? 0 : raw + ksr < 63 ? raw + ksr : 63;
}

static uint32_t phase_step(uint32_t block_freq, uint32_t multiple, int32_t lfo_raw_pm)
{
    uint32_t fnum = (block_freq & 0x3ff) << 2;
    fnum += (uint32_t)((lfo_raw_pm * (int32_t)((block_freq >> 7) & 7)) >> 1);
    fnum &= 0xfff;
    uint32_t step = (fnum << ((block_freq >> 10) & 7)) >> 2;
    return (step * multiple) >> 1;
}

/* ymfm's cache_operator_data */
static void op_cache(opl3_t *o, op_t *op)
{
    const uint8_t *r = o->regs;
    uint32_t opoffs = op->opoffs;
    op->wave = o->wave[r[0xe0 + opoffs] & (newflag(o) ? 7 : 3)];
    uint32_t bf = op->block_freq = block_freq(o, op->choffs);
    uint32_t keycode = ((bf >> 10) & 7) << 1 | ((bf >> (9 - ((r[0x08] >> 6) & 1))) & 1);
    uint32_t multiple = r[0x20 + opoffs] & 15;
    op->multiple = ((multiple & 0xe) | ((0xc2aa >> multiple) & 1)) * 2;
    if (!op->multiple) {
        op->multiple = 1;
    }
    op->phase_step = r[0x20 + opoffs] & 0x40 ? PHASE_STEP_DYNAMIC : phase_step(bf, op->multiple, 0);
    op->am_mask = r[0x20 + opoffs] & 0x80 ? ~0u : 0;
    op->tl = (uint32_t)(r[0x40 + opoffs] & 0x3f) << 3;
    uint32_t ksl = r[0x40 + opoffs] >> 6;
    ksl = (ksl >> 1) | ((ksl & 1) << 1);
    if (ksl) {
        static const uint8_t fnum_to_atten[16] = { 0, 24, 32, 37, 40, 43, 45, 47, 48, 50, 51, 52, 53, 54, 55, 56 };
        int32_t a = fnum_to_atten[(bf >> 6) & 15] - 8 * (int32_t)(((bf >> 10) & 7) ^ 7);
        op->tl += (uint32_t)(a > 0 ? a : 0) << ksl;
    }
    op->sustain = r[0x80 + opoffs] >> 4;
    op->sustain |= (op->sustain + 1) & 0x10;
    op->sustain <<= 5;
    uint32_t ksr = keycode >> (2 * (((r[0x20 + opoffs] >> 4) & 1) ^ 1));
    uint32_t release = effective_rate((r[0x80 + opoffs] & 15) * 4u, ksr);
    op->rate[EG_ATTACK] = (uint8_t)effective_rate((r[0x60 + opoffs] >> 4) * 4u, ksr);
    op->rate[EG_DECAY] = (uint8_t)effective_rate((r[0x60 + opoffs] & 15) * 4u, ksr);
    op->rate[EG_SUSTAIN] = r[0x20 + opoffs] & 0x20 ? 0 : (uint8_t)release;
    op->rate[EG_RELEASE] = (uint8_t)release;
}

/* What ymfm's clock_envelope checks at the start of every sample. The conditions only change
 * where the envelope steps and at prepare, so this runs there, for the next sample. */
static void op_transitions(op_t *op)
{
    if (op->state == EG_ATTACK && op->att == 0) {
        op->state = EG_DECAY;
    }
    if (op->state == EG_DECAY && op->att >= op->sustain) {
        op->state = EG_SUSTAIN;
    }
    op->eg_rate = op->rate[op->state];
    uint32_t shift = op->eg_rate >> 2u;
    op->eg_mask = (uint16_t)(0x7ffu >> (shift < 11 ? shift : 11));
}

/* ymfm's envelope_attenuation, 4.8, kept for op_volume */
static void op_env(op_t *op, uint32_t am)
{
    if (op->att > EG_QUIET) {
        op->env = ENV_OFF;
        return;
    }
    uint32_t env = op->att + (am & op->am_mask) + op->tl;
    op->env = (uint16_t)((env < 0x3ff ? env : 0x3ff) << 2);
}

static void op_keystate(op_t *op)
{
    uint8_t key = op->keyon_live != 0;
    if (key == op->key) {
        return;
    }
    op->key = key;
    if (key) {
        if (op->state != EG_ATTACK) {
            op->state = EG_ATTACK;
            op->phase = 0;
            if (op->rate[EG_ATTACK] >= 62) {
                op->att = 0;
            }
        }
    } else if (op->state < EG_RELEASE) {
        op->state = EG_RELEASE;
    }
}

static void assign_operators(opl3_t *o)
{
    static const uint8_t pairs[18][2] = {
        { 0, 3 }, { 1, 4 }, { 2, 5 }, { 6, 9 }, { 7, 10 }, { 8, 11 }, { 12, 15 }, { 13, 16 }, { 14, 17 },
        { 18, 21 }, { 19, 22 }, { 20, 23 }, { 24, 27 }, { 25, 28 }, { 26, 29 }, { 30, 33 }, { 31, 34 }, { 32, 35 },
    };
    uint32_t fourop = o->regs[0x104] & 0x3f;
    for (int c = 0; c < 18; c++) {
        chan_t *ch = &o->ch[c];
        int bit = c < 9 ? c : c - 6;    /* channels 0-2 and 9-11 lead a pair: bits 0-2 and 3-5 */
        int lead = c % 9 < 3 && (fourop >> bit & 1);
        int second = c % 9 >= 3 && c % 9 < 6 && (fourop >> (bit - 3) & 1);
        memset(ch->op, 0, sizeof(ch->op));
        if (second) {
            continue;
        }
        ch->op[0] = &o->op[pairs[c][0]];
        ch->op[1] = &o->op[pairs[c][1]];
        if (lead) {
            ch->op[2] = &o->op[pairs[c + 3][0]];
            ch->op[3] = &o->op[pairs[c + 3][1]];
        }
        for (int i = 0; i < 4; i++) {
            if (ch->op[i]) {
                ch->op[i]->choffs = ch->choffs;
            }
        }
    }
}

/* ymfm's clock-time prepare: the new registers take effect */
static void prepare(opl3_t *o)
{
    const uint8_t *r = o->regs;
    assign_operators(o);
    o->active = 0;
    for (int c = 0; c < 18; c++) {
        chan_t *ch = &o->ch[c];
        bool active = false;
        for (int i = 0; i < 4; i++) {
            op_t *op = ch->op[i];
            if (op) {
                op_cache(o, op);
                op_keystate(op);
                active |= op->state != EG_RELEASE || op->att < EG_QUIET;
                op_transitions(op);
                op_env(op, o->lfo_am);
            }
        }
        if (active) {
            o->active |= 1u << c;
        }
        uint32_t fb = (r[0xc0 + ch->choffs] >> 1) & 7;
        ch->fb_shift = fb ? (uint8_t)(10 - fb) : 0;
        ch->alg = (uint8_t)((r[0xc0 + ch->choffs] & 1) | 8 | (r[0xc3 + ch->choffs] & 1) << 1);
        uint8_t outs = newflag(o) ? r[0xc0 + ch->choffs] >> 4 : 3;
        ch->out_any = outs != 0;
        ch->out_l = outs & 1;
        ch->out_r = outs & 2;
    }
    o->rhythm = r[0xbd] & 0x20;
    o->am_depth = r[0xbd] >> 7;
    o->pm_depth = (r[0xbd] >> 6) & 1;
    o->modified = false;
    o->prepare_count = 0;
}

static void keyonoff(opl3_t *o, int c, uint32_t states, int type)
{
    for (int i = 0; i < 4; i++) {
        op_t *op = o->ch[c].op[i];
        if (op) {
            op->keyon_live = (uint8_t)((op->keyon_live & ~(1u << type)) | ((states >> i) & 1) << type);
        }
    }
}

void opl3_write(opl3_t *o, int bank, uint8_t reg, uint8_t v)
{
    uint32_t a = bank ? 0x100u | reg : reg;
    if (bank && !newflag(o) && a != 0x105) {
        a &= 0xff; /* compatibility mode: the upper bank is the lower one, except the mode register */
    }
    o->address = (uint16_t)a;
    o->modified = true;
    if (a == 0x04) {
        if (v & 0x80) {
            o->regs[a] |= 0x80;
        } else {
            o->regs[a] = v;
        }
        return;
    }
    o->regs[a] = v;
    if (a == 0xbd) {
        uint32_t m = v & 0x20 ? v & 0x1f : 0;
        keyonoff(o, 6, (m >> 4) & 1 ? 3 : 0, KEYON_RHYTHM);
        keyonoff(o, 7, (m & 1) | ((m >> 3) & 1) << 1, KEYON_RHYTHM);
        keyonoff(o, 8, ((m >> 2) & 1) | ((m >> 1) & 1) << 1, KEYON_RHYTHM);
    } else if ((a & 0xf0) == 0xb0 && (a & 0x0f) < 9) {
        keyonoff(o, (int)(a & 0x0f) + 9 * (int)(a >> 8), v & 0x20 ? 15 : 0, KEYON_NORMAL);
    }
}

/* ---- per sample ---- */

/* ymfm's clock_envelope in a sample where the envelope steps */
static void op_step(op_t *op, uint32_t eg, uint32_t am)
{
    uint32_t rate = op->eg_rate;
    uint32_t shift = rate >> 2;
    uint32_t c = eg << shift;
    uint32_t inc = (s_increment[rate] >> (4 * ((c >> (shift <= 11 ? 11 : shift)) & 7))) & 15;
    if (op->state == EG_ATTACK) {
        if (rate < 62) {
            op->att = (uint16_t)(op->att + (((uint32_t)~op->att * inc) >> 4));
        }
    } else {
        op->att = (uint16_t)(op->att + inc);
        if (op->att >= 0x400) {
            op->att = 0x3ff;
        }
    }
    op_transitions(op);
    op_env(op, am);
}

static inline void op_clock(op_t *op, uint32_t eg, int32_t lfo_raw_pm, uint32_t am, bool am_moved)
{
    if (!(eg & op->eg_mask)) {
        op_step(op, eg, am);
    } else if (am_moved && op->am_mask) {
        op_env(op, am);
    }
    uint32_t step = op->phase_step;
    if (step == PHASE_STEP_DYNAMIC) {
        step = phase_step(op->block_freq, op->multiple, lfo_raw_pm);
    }
    op->phase += step;
}

static inline int32_t op_volume(const opl3_t *o, const op_t *op, uint32_t phase)
{
    if (op->env == ENV_OFF) {
        return 0;
    }
    return o->volume[op->wave[phase & 0x3ff] + op->env];
}

static inline uint32_t op_phase(const op_t *op)
{
    return op->phase >> 10;
}

static inline int32_t clamp16(int32_t v)
{
    return v < -32768 ? -32768 : v > 32767 ? 32767 : v;
}

static inline void add_out(const chan_t *ch, int32_t out[2], int32_t v)
{
    if (ch->out_l) {
        out[0] += v;
    }
    if (ch->out_r) {
        out[1] += v;
    }
}

/* operator 1, with its feedback */
static inline int32_t op1_volume(const opl3_t *o, chan_t *ch)
{
    int32_t mod = ch->fb_shift ? (ch->fb[0] + ch->fb[1]) >> ch->fb_shift : 0;
    int32_t v = op_volume(o, ch->op[0], op_phase(ch->op[0]) + (uint32_t)mod);
    ch->fb_in = (int16_t)v;
    return v;
}

static void output_2op(const opl3_t *o, chan_t *ch, int32_t out[2])
{
    int32_t v1 = op1_volume(o, ch);
    if (!ch->out_any) {
        return;
    }
    const op_t *op2 = ch->op[1];
    int32_t r;
    if (!(ch->alg & 1)) {
        r = op_volume(o, op2, op_phase(op2) + (uint32_t)(v1 >> 1));
    } else {
        r = clamp16(v1 + op_volume(o, op2, op_phase(op2)));
    }
    add_out(ch, out, r);
}

static void output_4op(const opl3_t *o, chan_t *ch, int32_t out[2])
{
    /* ymfm's algorithms 8-11: op2 input, op3 input, op4 input, then whether O1, O2, O3 are summed */
#define ALG(op2in, op3in, op4in, op1out, op2out, op3out) \
    ((op2in) | ((op3in) << 1) | ((op4in) << 4) | ((op1out) << 7) | ((op2out) << 8) | ((op3out) << 9))
    static const uint16_t s_alg[4] = {
        ALG(1, 2, 3, 0, 0, 0),  /* O1 -> O2 -> O3 -> O4 -> out (O4) */
        ALG(0, 2, 3, 1, 0, 0),  /* (O1 + (O2 -> O3 -> O4)) -> out (O1+O4) */
        ALG(1, 0, 3, 0, 1, 0),  /* ((O1 -> O2) + (O3 -> O4)) -> out (O2+O4) */
        ALG(0, 2, 0, 1, 0, 1),  /* (O1 + (O2 -> O3) + O4) -> out (O1+O3+O4) */
    };
#undef ALG
    int32_t v1 = op1_volume(o, ch);
    if (!ch->out_any) {
        return;
    }
    uint32_t ops = s_alg[ch->alg & 3];
    int16_t opout[8];
    opout[0] = 0;
    opout[1] = (int16_t)v1;
    opout[2] = (int16_t)op_volume(o, ch->op[1], op_phase(ch->op[1]) + (uint32_t)(opout[ops & 1] >> 1));
    opout[5] = (int16_t)(opout[1] + opout[2]);
    opout[3] = (int16_t)op_volume(o, ch->op[2], op_phase(ch->op[2]) + (uint32_t)(opout[(ops >> 1) & 7] >> 1));
    opout[6] = (int16_t)(opout[1] + opout[3]);
    opout[7] = (int16_t)(opout[2] + opout[3]);
    int32_t r = op_volume(o, ch->op[3], op_phase(ch->op[3]) + (uint32_t)(opout[(ops >> 4) & 7] >> 1));
    if (ops & 0x80) {
        r = clamp16(r + opout[1]);
    }
    if (ops & 0x100) {
        r = clamp16(r + opout[2]);
    }
    if (ops & 0x200) {
        r = clamp16(r + opout[3]);
    }
    add_out(ch, out, r);
}

static void output_rhythm(opl3_t *o, int c, int32_t out[2])
{
    chan_t *ch = &o->ch[c];
    uint32_t p13 = op_phase(&o->op[13]), p17 = op_phase(&o->op[17]);
    uint32_t sel = (((p13 >> 2) & 1) ^ ((p13 >> 7) & 1)) | ((p13 >> 3) & 1) | (((p17 >> 5) & 1) ^ ((p17 >> 3) & 1));
    uint32_t noise = (o->noise >> 23) & 1;
    int32_t r;
    if (c == 6) {
        /* bass drum: operators 12 and 15, but with algorithm 1 operator 1 is not added in */
        int32_t v1 = op1_volume(o, ch);
        r = op_volume(o, ch->op[1], op_phase(ch->op[1]) + (uint32_t)((ch->alg & 1) ? 0 : v1 >> 1));
    } else if (c == 7) {
        /* high hat (operator 13) and snare drum (16) */
        uint32_t phase = (sel << 9) | (0xd0u >> (2 * (noise ^ sel)));
        r = op_volume(o, ch->op[0], phase);
        phase = (0x100u << ((op_phase(ch->op[0]) >> 8) & 1)) ^ (noise << 8);
        r = clamp16(r + op_volume(o, ch->op[1], phase));
    } else {
        /* tom tom (operator 14) and top cymbal (17) */
        r = op_volume(o, ch->op[0], op_phase(ch->op[0]));
        r = clamp16(r + op_volume(o, ch->op[1], 0x100 | (sel << 9)));
    }
    add_out(ch, out, r * 2);
}

static int32_t clock_noise_and_lfo(opl3_t *o)
{
    o->noise <<= 1;
    o->noise |= ((o->noise >> 23) ^ (o->noise >> 9) ^ (o->noise >> 8) ^ (o->noise >> 1)) & 1;
    uint32_t am = o->lfo_am_counter++;
    if (am >= 210 * 64 - 1) {
        o->lfo_am_counter = 0;
    }
    int shift = 9 - 2 * (int)o->am_depth;
    o->lfo_am = (uint8_t)(((am < 105 * 64) ? am : (210 * 64 + 63 - am)) >> shift);
    uint32_t pm = o->lfo_pm_counter++;
    static const int8_t pm_scale[8] = { 8, 4, 0, -4, -8, -4, 0, 4 };
    return pm_scale[(pm >> 10) & 7] >> (o->pm_depth ^ 1);
}

void opl3_sample(opl3_t *o, int16_t *l, int16_t *r)
{
    if (o->modified || o->prepare_count++ >= 4096) {
        prepare(o);
    }
    o->env_counter += 4;
    uint32_t am0 = o->lfo_am;
    int32_t pm = clock_noise_and_lfo(o);
    uint32_t am = o->lfo_am, eg = o->env_counter >> 2;
    for (int i = 0; i < 36; i++) {
        op_clock(&o->op[i], eg, pm, am, am != am0);
    }

    int32_t out[2] = { 0, 0 };
    uint32_t active = o->active;
    for (int c = 0; c < 18; c++, active >>= 1) {
        chan_t *ch = &o->ch[c];
        ch->fb[0] = ch->fb[1];
        ch->fb[1] = ch->fb_in;
        if (!(active & 1)) {
            continue;
        }
        if (o->rhythm && c >= 6 && c <= 8) {
            output_rhythm(o, c, out);
        } else if (ch->op[2]) {
            output_4op(o, ch, out);
        } else {
            output_2op(o, ch, out);
        }
    }
    *l = (int16_t)clamp16(out[0]);
    *r = (int16_t)clamp16(out[1]);
}

/* ---- creation ---- */

static void make_waves(opl3_t *o)
{
    uint16_t(*w)[1024] = o->wave;
    for (uint32_t i = 0; i < 1024; i++) {
        uint32_t in = i & 0x100 ? ~i : i;
        w[0][i] = (uint16_t)(s_sin[in & 0xff] | ((i >> 9) & 1) << 15);
    }
    uint16_t zero = w[0][0];
    for (uint32_t i = 0; i < 1024; i++) {
        bool hi = (i >> 9) & 1;
        w[1][i] = hi ? zero : w[0][i];
        w[2][i] = w[0][i] & 0x7fff;
        w[3][i] = (i >> 8) & 1 ? zero : (w[0][i] & 0x7fff);
        w[4][i] = hi ? zero : w[0][(i * 2) & 0x3ff];
        w[5][i] = hi ? zero : w[0][(i * 2) & 0x1ff];
        w[6][i] = (uint16_t)(hi << 15);
        w[7][i] = (uint16_t)((hi ? (i ^ 0x13ff) : i) << 3);
    }
    /* without the sign the waveforms are at most 0xff8 and the envelope 0xffc: the sum is below
     * 0x2000, so the sign can take bit 13, and the shift below is under 32 */
    for (int k = 0; k < 8; k++) {
        for (uint32_t i = 0; i < 1024; i++) {
            w[k][i] = (uint16_t)((w[k][i] & 0x7fff) | (w[k][i] >> 15) << 13);
        }
    }
    for (uint32_t x = 0; x < 0x2000; x++) {
        int16_t v = (int16_t)(s_power[x & 0xff] >> (x >> 8));
        o->volume[x] = v;
        o->volume[x | 0x2000] = (int16_t)-v;
    }
}

opl3_t *opl3_create(void)
{
    /* internal RAM: at 50 KB (mostly the tables) the chip would otherwise go to PSRAM */
    opl3_t *o = heap_caps_calloc(1, sizeof(*o), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!o) {
        o = calloc(1, sizeof(*o));
    }
    if (!o) {
        return NULL;
    }
    make_waves(o);
    o->noise = 1;
    o->modified = true;
    for (int i = 0; i < 36; i++) {
        o->op[i].opoffs = (uint16_t)op_offset(i);
        o->op[i].att = 0x3ff;
        o->op[i].state = EG_RELEASE;
    }
    for (int c = 0; c < 18; c++) {
        o->ch[c].choffs = (uint16_t)ch_offset(c);
    }
    assign_operators(o);
    return o;
}
