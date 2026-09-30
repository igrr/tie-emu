/* Sound Blaster 16 (DSP 4.05, mixer) at 220h / IRQ 7 / DMA 1 and 5, the two 8237 DMA
 * controllers, and the OPL3 register file with its timers at 388h (and 220h..223h).
 *
 * DMA transfers advance with guest time: a running SB transfer's position is a function
 * of the PIT tick count, so the registers the drivers poll and the IRQs they wait for are
 * deterministic. Audio output is rendered along the same clock: before any state change
 * (a DSP command, a DMA or OPL register write, a block end) the output is brought up to
 * the current tick, so what is heard does not depend on how the run is sliced. */
#include <stdlib.h>
#include <string.h>
#include "machine_priv.h"

#define SB_BASE 0x220
#define SB_IRQ 7
#define SB_DMA8 1
#define SB_DMA16 5

/* ---- 8237 DMA ---- */

static const uint16_t s_page_port[8] = { 0x87, 0x83, 0x81, 0x82, 0x8f, 0x8b, 0x89, 0x8a };

static dma_chan_t *dma_chan(machine_t *m, int ch)
{
    return &m->snd.dma[ch & 7];
}

/* Channel position after `units` transfers since programming (units are bytes or words) */
static void dma_advance(machine_t *m, int ch, uint32_t units)
{
    dma_chan_t *d = dma_chan(m, ch);
    uint32_t len = (uint32_t)d->base_count + 1;
    uint32_t done = d->done + units;
    if (done >= len) {
        if (d->mode & 0x10) {
            done %= len;
        } else {
            done = len;
            d->terminal = true;
        }
    }
    d->done = done;
}

static uint8_t dma_in(machine_t *m, uint16_t port)
{
    bool hi = port >= 0xc0;
    int reg = hi ? (port - 0xc0) >> 1 : port;
    int base = hi ? 4 : 0;
    if (reg < 8) {
        dma_chan_t *d = dma_chan(m, base + (reg >> 1));
        uint32_t v = reg & 1 ? (uint32_t)d->base_count - d->done : d->base_addr + d->done;
        bool *ff = &m->snd.dma_ff[hi];
        uint8_t out = *ff ? v >> 8 : v;
        *ff = !*ff;
        return out;
    }
    if (reg == 8) { /* status: terminal count bits */
        uint8_t st = 0;
        for (int i = 0; i < 4; i++) {
            if (dma_chan(m, base + i)->terminal) {
                st |= 1u << i;
                dma_chan(m, base + i)->terminal = false;
            }
        }
        return st;
    }
    return 0xff;
}

static void dma_out(machine_t *m, uint16_t port, uint8_t v)
{
    bool hi = port >= 0xc0;
    int reg = hi ? (port - 0xc0) >> 1 : port;
    int base = hi ? 4 : 0;
    bool *ff = &m->snd.dma_ff[hi];
    if (reg < 8) {
        dma_chan_t *d = dma_chan(m, base + (reg >> 1));
        uint16_t *r = reg & 1 ? &d->base_count : &d->base_addr;
        *r = *ff ? (uint16_t)((*r & 0x00ff) | v << 8) : (uint16_t)((*r & 0xff00) | v);
        *ff = !*ff;
        d->done = 0;
        return;
    }
    switch (reg) {
    case 0x0a:
        dma_chan(m, base + (v & 3))->masked = v & 4;
        return;
    case 0x0b: {
        dma_chan_t *d = dma_chan(m, base + (v & 3));
        d->mode = v;
        d->done = 0;
        return;
    }
    case 0x0c:
        *ff = false;
        return;
    case 0x0d:
        *ff = false;
        for (int i = 0; i < 4; i++) {
            dma_chan(m, base + i)->masked = true;
        }
        return;
    case 0x0e:
        for (int i = 0; i < 4; i++) {
            dma_chan(m, base + i)->masked = false;
        }
        return;
    case 0x0f:
        for (int i = 0; i < 4; i++) {
            dma_chan(m, base + i)->masked = v & (1u << i);
        }
        return;
    }
}

/* ---- SB16 mixer ---- */

/* 5-bit mixer level -> Q15 gain: 31 is 0 dB, 2 dB per step */
static const int32_t s_level_gain[32] = {
    26, 33, 41, 52, 65, 82, 104, 130, 164, 207, 260, 328, 413, 519, 654, 823,
    1036, 1304, 1642, 2067, 2603, 3277, 4125, 5193, 6538, 8231, 10362, 13045, 16422, 20675, 26028, 32767,
};

static void audio_event(machine_t *m, uint32_t data);

/* Event data: an OPL write is bank << 16 | reg << 8 | value; a gain has AUDIO_EV_GAIN, the FM
 * flag and the channel above the Q15 value */
#define AUDIO_EV_OPL 0u
#define AUDIO_EV_GAIN (1u << 31)
#define AUDIO_EV_FM (1u << 29)
#define AUDIO_EV_CHAN(c) ((uint32_t)(c) << 28)
#define AUDIO_EV_VALUE 0x0fffffffu

static void mixer_update_gains(machine_t *m)
{
    const uint8_t *mx = m->snd.sb.mixer;
    audio_out_t *a = &m->snd.out;
    for (int c = 0; c < 2; c++) {
        int32_t master = s_level_gain[mx[0x30 + c] >> 3];
        int shift = mx[0x41 + c] >> 6; /* output gain x1, x2, x4, x8 */
        int32_t pcm = ((master * s_level_gain[mx[0x32 + c] >> 3]) >> 15) << shift;
        int32_t fm = ((master * s_level_gain[mx[0x34 + c] >> 3]) >> 15) << shift;
        if (pcm != a->gain_pcm[c]) {
            a->gain_pcm[c] = pcm;
            audio_event(m, AUDIO_EV_GAIN | AUDIO_EV_CHAN(c) | (uint32_t)pcm);
        }
        if (fm != a->gain_fm[c]) {
            a->gain_fm[c] = fm;
            audio_event(m, AUDIO_EV_GAIN | AUDIO_EV_FM | AUDIO_EV_CHAN(c) | (uint32_t)fm);
        }
    }
}

static void mixer_reset(machine_t *m)
{
    uint8_t *mx = m->snd.sb.mixer;
    memset(mx, 0, 0x80);
    /* Master, voice and FM at 0 dB. The chip resets to -14 dB, but DOS setups ran a mixer
     * utility at boot, and TIE's drivers never touch the mixer. */
    for (int r = 0x30; r <= 0x35; r++) {
        mx[r] = 0xf8;
    }
    mx[0x22] = mx[0x04] = mx[0x26] = 0xff;
    mx[0x44] = mx[0x45] = mx[0x46] = mx[0x47] = 0x80;
    mixer_update_gains(m);
}

static void mixer_write(machine_t *m, uint8_t idx, uint8_t v)
{
    uint8_t *mx = m->snd.sb.mixer;
    /* SB Pro registers (4-bit left/right nibbles) alias the SB16 ones */
    static const uint8_t pro[3][2] = { { 0x22, 0x30 }, { 0x04, 0x32 }, { 0x26, 0x34 } };
    if (idx == 0x00) {
        mixer_reset(m);
        return;
    }
    if (idx >= 0x80 && idx <= 0x82) {
        return; /* IRQ/DMA setup and interrupt status are fixed */
    }
    mx[idx] = v;
    for (int i = 0; i < 3; i++) {
        if (idx == pro[i][0]) {
            mx[pro[i][1]] = (uint8_t)((v & 0xf0) | 0x08);
            mx[pro[i][1] + 1] = (uint8_t)((v & 0x0f) << 4 | 0x08);
        } else if (idx == pro[i][1] || idx == pro[i][1] + 1) {
            mx[pro[i][0]] = (uint8_t)((mx[pro[i][1]] & 0xf0) | mx[pro[i][1] + 1] >> 4);
        }
    }
    mixer_update_gains(m);
}

/* ---- audio output ---- */

/* The running transfer as audio_render reads it: none of this changes during a call. `page` is
 * the host address of the DMA page, or NULL if it is not all in guest memory (then each sample
 * goes through lin_ptr). a and b are the last pair of sample frames read. */
typedef struct {
    int unit;               /* DMA units per sample frame */
    bool bits16, is_signed, stereo;
    int32_t done, len;      /* the channel's position and length in DMA units */
    uint32_t base, page_lin;
    const uint8_t *page;
    uint32_t idx;           /* a and b are sample frames idx and idx + 1 (UINT32_MAX: none yet) */
    int32_t a[2], b[2];
} sb_src_t;

/* The sample `rel` DMA units away from the channel's position */
static inline int32_t sb_sample(machine_t *m, const sb_src_t *src, int32_t rel)
{
    int32_t pos = src->done + rel;
    if (pos < 0 || pos >= src->len) {
        pos %= src->len;
        if (pos < 0) {
            pos += src->len;
        }
    }
    uint32_t off = ((src->base + (uint32_t)pos) & 0xffff) << src->bits16;
    const uint8_t *p = src->page ? src->page + off : lin_ptr(m, src->page_lin + off, src->bits16 ? 2 : 1);
    if (!p) {
        return 0;
    }
    if (src->bits16) {
        uint16_t v = (uint16_t)(p[0] | p[1] << 8);
        return src->is_signed ? (int16_t)v : (int32_t)v - 0x8000;
    }
    return src->is_signed ? (int8_t)p[0] * 256 : ((int32_t)p[0] - 0x80) * 256;
}

/* n / den as a quotient, a 16-bit fraction and the remainder of that: n * 65536 = (q * 65536 + f) * den + r */
static void split(uint64_t n, uint64_t den, uint32_t *q, uint32_t *f, uint64_t *r)
{
    uint64_t qq = n / den, p = (n % den) << 16;
    *q = qq > UINT32_MAX ? UINT32_MAX : (uint32_t)qq;
    *f = (uint32_t)(p / den);
    *r = p % den;
}

/* The running transfer's position at output frame k: sample frame `idx`, and `frac` / 65536 of the
 * way to the next. The cursor steps from one frame to the next; it restarts with a new block or rate. */
static void sb_position(machine_t *m, uint64_t k, uint32_t *idx, uint32_t *frac)
{
    const uint64_t den = (uint64_t)PIT_HZ * MACHINE_AUDIO_RATE;
    sb16_t *sb = &m->snd.sb;
    sb_cursor_t *c = &m->snd.out.cur;
    if (c->k + 1 == k && c->block_start == sb->block_start && c->rate == sb->rate) {
        c->k = k;
        c->rem += c->step_rem;
        uint32_t carry = c->rem >= den;
        if (carry) {
            c->rem -= den;
        }
        c->frac += c->step_frac + carry;
        c->idx += c->step_idx + (c->frac >> 16);
        c->frac &= 0xffff;
    } else {
        /* time since the block start, in 1 / (PIT_HZ * MACHINE_AUDIO_RATE) s */
        uint64_t ft = k * PIT_HZ, bs = sb->block_start * MACHINE_AUDIO_RATE;
        if (ft <= bs) {
            *idx = *frac = 0;
            return;
        }
        split((ft - bs) * sb->rate, den, &c->idx, &c->frac, &c->rem);
        split((uint64_t)PIT_HZ * sb->rate, den, &c->step_idx, &c->step_frac, &c->step_rem);
        /* past 2^32 sample frames the transfer has long ended: do not step from there */
        c->k = c->idx == UINT32_MAX ? UINT64_MAX - 1 : k;
        c->block_start = sb->block_start;
        c->rate = sb->rate;
    }
    *idx = c->idx;
    *frac = c->frac;
}

/* The running transfer's output for output frame k, linearly interpolated between the
 * sample frames around it. Samples are read straight from guest memory, as the DMA would. */
static void sb_frame(machine_t *m, sb_src_t *src, uint64_t k, int16_t out[2])
{
    sb16_t *sb = &m->snd.sb;
    uint32_t idx, f;
    sb_position(m, k, &idx, &f);
    if (idx >= sb->block) {
        out[0] = out[1] = 0;
        return;
    }
    if (idx != src->idx) {
        src->idx = idx;
        for (int n = 0; n < 2; n++) {
            uint32_t i = idx + n < sb->block ? idx + n : idx;
            int32_t rel = ((int32_t)i - (int32_t)sb->frames_done) * src->unit;
            int32_t *s = n ? src->b : src->a;
            s[0] = sb_sample(m, src, rel);
            s[1] = src->stereo ? sb_sample(m, src, rel + 1) : s[0];
        }
    }
    int32_t frac = (int32_t)f;
    for (int c = 0; c < 2; c++) {
        /* between a and b, so within int16 */
        int32_t a = src->a[c], b = src->b[c];
        out[c] = (int16_t)(a + (int32_t)(((int64_t)(b - a) * frac) >> 16));
    }
}

/* Renders the PCM of the output frames that start before PIT tick `upto` into the ring: the
 * frames k with k * PIT_HZ < upto * MACHINE_AUDIO_RATE */
static void audio_render(machine_t *m, uint64_t upto)
{
    audio_out_t *a = &m->snd.out;
    uint64_t end = upto * MACHINE_AUDIO_RATE;
    if (a->frames_at >= end) {
        return;
    }
    sb16_t *sb = &m->snd.sb;
    int ch = sb->bits16 ? SB_DMA16 : SB_DMA8;
    const dma_chan_t *d = dma_chan(m, ch);
    bool playing = sb->active && !sb->paused && sb->rate && !d->masked;
    sb_src_t src = { .unit = sb->stereo ? 2 : 1, .bits16 = sb->bits16, .is_signed = sb->is_signed,
                     .stereo = sb->stereo, .idx = UINT32_MAX };
    if (playing) {
        /* 16-bit channels count words within a 128 KB page */
        src.done = (int32_t)d->done;
        src.len = (int32_t)d->base_count + 1;
        src.base = d->base_addr;
        src.page_lin = ch >= 4 ? (uint32_t)(d->page & 0xfe) << 16 : (uint32_t)d->page << 16;
        src.page = lin_ptr(m, src.page_lin, ch >= 4 ? 0x20000 : 0x10000);
    }
    uint32_t head = atomic_load_explicit(&a->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&a->tail, memory_order_acquire);
    for (; a->frames_at < end; a->frames++, a->frames_at += PIT_HZ) {
        if (head - tail >= AUDIO_RING) {
            tail = atomic_load_explicit(&a->tail, memory_order_acquire);
            if (head - tail >= AUDIO_RING) {
                continue;   /* the reader is behind: this frame is dropped */
            }
        }
        int16_t *dst = &a->ring[(head % AUDIO_RING) * 2];
        if (playing) {
            sb_frame(m, &src, a->frames, dst);
        } else {
            dst[0] = dst[1] = 0;
        }
        head++;
    }
    atomic_store_explicit(&a->head, head, memory_order_release);
}

/* ---- the reader's side: OPL3 synthesis and the mix ---- */

enum { PRODUCER, READER };

static void audio_lock(audio_out_t *a, int me)
{
    atomic_store(&a->want[me], 1);
    atomic_store(&a->turn, !me);
    while (atomic_load(&a->want[!me]) && atomic_load(&a->turn) == !me) {
    }
}

static void audio_unlock(audio_out_t *a, int me)
{
    atomic_store_explicit(&a->want[me], 0, memory_order_release);
}

static void audio_apply(audio_out_t *a, uint32_t data)
{
    if (data & AUDIO_EV_GAIN) {
        int32_t *g = data & AUDIO_EV_FM ? a->mix_fm : a->mix_pcm;
        g[(data >> 28) & 1] = (int32_t)(data & AUDIO_EV_VALUE);
    } else if (a->opl) {
        opl3_write(a->opl, (data >> 16) & 1, (uint8_t)(data >> 8), (uint8_t)data);
    }
}

/* Queues a change for the reader to make at the current ring position. When the queue is full,
 * the reader is far behind or there is none: its state catches up here, and the frames it has
 * not read are dropped without being synthesized, so the emulation never waits for it. */
static void audio_event(machine_t *m, uint32_t data)
{
    audio_out_t *a = &m->snd.out;
    uint32_t h = atomic_load_explicit(&a->ev_head, memory_order_relaxed);
    if (h - atomic_load_explicit(&a->ev_tail, memory_order_acquire) >= AUDIO_EVENTS) {
        audio_lock(a, PRODUCER);
        uint32_t t = atomic_load_explicit(&a->ev_tail, memory_order_relaxed);
        for (; t != h; t++) {
            audio_apply(a, a->ev[t % AUDIO_EVENTS].data);
        }
        atomic_store_explicit(&a->ev_tail, t, memory_order_release);
        uint32_t head = atomic_load_explicit(&a->head, memory_order_relaxed);
        atomic_store_explicit(&a->tail, head, memory_order_release);
        audio_unlock(a, PRODUCER);
    }
    a->ev[h % AUDIO_EVENTS] = (audio_event_t){ atomic_load_explicit(&a->head, memory_order_relaxed), data };
    atomic_store_explicit(&a->ev_head, h + 1, memory_order_release);
}

/* OPL3 output for the next output frame, resampled from the chip's rate */
static void opl_frame(audio_out_t *a, int32_t out[2])
{
    if (!a->opl) {
        out[0] = out[1] = 0;
        return;
    }
    a->opl_rem += OPL3_RATE;
    while (a->opl_rem >= MACHINE_AUDIO_RATE) {
        a->opl_rem -= MACHINE_AUDIO_RATE;
        a->opl_s0[0] = a->opl_s1[0];
        a->opl_s0[1] = a->opl_s1[1];
        opl3_sample(a->opl, &a->opl_s1[0], &a->opl_s1[1]);
    }
    for (int c = 0; c < 2; c++) {
        /* s0 + (s1 - s0) * rem / RATE, rounded toward zero; the product fits 32 bits unsigned */
        int32_t d = a->opl_s1[c] - a->opl_s0[c];
        uint32_t u = (uint32_t)(d < 0 ? -d : d) * (uint32_t)a->opl_rem / MACHINE_AUDIO_RATE;
        out[c] = a->opl_s0[c] + (d < 0 ? -(int32_t)u : (int32_t)u);
    }
}

size_t machine_audio_read(machine_t *m, int16_t *frames, size_t max_frames)
{
    audio_out_t *a = &m->snd.out;
    audio_lock(a, READER);
    uint32_t tail = atomic_load_explicit(&a->tail, memory_order_relaxed);
    uint32_t head = atomic_load_explicit(&a->head, memory_order_acquire);
    uint32_t et = atomic_load_explicit(&a->ev_tail, memory_order_relaxed);
    uint32_t eh = atomic_load_explicit(&a->ev_head, memory_order_acquire);
    size_t n = head - tail;
    if (n > max_frames) {
        n = max_frames;
    }
    for (size_t i = 0; i < n; i++) {
        uint32_t pos = tail + (uint32_t)i;
        while (et != eh && (int32_t)(a->ev[et % AUDIO_EVENTS].pos - pos) <= 0) {
            audio_apply(a, a->ev[et % AUDIO_EVENTS].data);
            et++;
        }
        int32_t fm[2];
        opl_frame(a, fm);
        const int16_t *pcm = &a->ring[(pos % AUDIO_RING) * 2];
        for (int c = 0; c < 2; c++) {
            int32_t v = (pcm[c] * a->mix_pcm[c] + fm[c] * a->mix_fm[c]) >> 15;
            frames[2 * i + c] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        }
    }
    atomic_store_explicit(&a->ev_tail, et, memory_order_release);
    atomic_store_explicit(&a->tail, tail + (uint32_t)n, memory_order_release);
    audio_unlock(a, READER);
    return n;
}

size_t machine_audio_available(machine_t *m)
{
    audio_out_t *a = &m->snd.out;
    return atomic_load_explicit(&a->head, memory_order_acquire) - atomic_load_explicit(&a->tail, memory_order_relaxed);
}

/* ---- SB16 DSP ---- */

static void dsp_put(machine_t *m, uint8_t v)
{
    sb16_t *sb = &m->snd.sb;
    if (sb->out_n < (int)sizeof(sb->out)) {
        sb->out[(sb->out_head + sb->out_n++) % sizeof(sb->out)] = v;
    }
}

static void dsp_reset(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    uint8_t mixer[256];
    memcpy(mixer, sb->mixer, sizeof(mixer));
    memset(sb, 0, sizeof(*sb));
    memcpy(sb->mixer, mixer, sizeof(mixer));
    sb->rate = 22050;
    dsp_put(m, 0xaa);
}

/* Starts a DMA transfer of `samples` sample frames */
static void dsp_start(machine_t *m, bool bits16, bool autoinit, bool stereo, bool is_signed, uint32_t samples)
{
    sb16_t *sb = &m->snd.sb;
    sb->active = true;
    sb->paused = false;
    sb->bits16 = bits16;
    sb->autoinit = autoinit;
    sb->stereo = stereo;
    sb->is_signed = is_signed;
    sb->block = samples ? samples : 1;
    sb->block_start = mach_pit_ticks(m);
    sb->frames_done = 0;
}

static int dsp_args(uint8_t cmd)
{
    if (cmd >= 0xb0 && cmd <= 0xcf) {
        return 3;
    }
    switch (cmd) {
    case 0x10: case 0x40: case 0xe0: case 0xe4: case 0x38: return 1;
    case 0x14: case 0x16: case 0x17: case 0x24: case 0x41: case 0x42: case 0x48: case 0x80: case 0x74: case 0x75:
    case 0x76: case 0x77: return 2;
    default: return 0;
    }
}

static void dsp_command(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    uint8_t c = sb->cmd, *a = sb->args;
    if (c >= 0xb0 && c <= 0xcf) {
        /* the length counts samples; a stereo frame is two of them */
        uint32_t samples = (uint32_t)(a[1] | a[2] << 8) + 1;
        bool stereo = a[0] & 0x20;
        dsp_start(m, c < 0xc0, c & 0x04, stereo, a[0] & 0x10, stereo ? samples / 2 : samples);
        return;
    }
    switch (c) {
    case 0x10: return; /* direct DAC sample */
    case 0x14: case 0x16: case 0x17: case 0x24:
        dsp_start(m, false, false, false, false, (uint32_t)(a[0] | a[1] << 8) + 1);
        return;
    case 0x1c: case 0x1f: case 0x2c:
        dsp_start(m, false, true, false, false, sb->block_size + 1u);
        return;
    case 0x40:
        sb->rate = 1000000 / (256 - a[0]);
        return;
    case 0x41: case 0x42:
        sb->rate = (uint32_t)a[0] << 8 | a[1];
        return;
    case 0x48:
        sb->block_size = (uint16_t)(a[0] | a[1] << 8);
        return;
    case 0x80: /* silence: behaves like a transfer without data */
        dsp_start(m, false, false, false, false, (uint32_t)(a[0] | a[1] << 8) + 1);
        return;
    case 0x90: case 0x91:
        dsp_start(m, false, c == 0x90, false, false, sb->block_size + 1u);
        return;
    case 0xd0: case 0xd5: sb->paused = true; return;
    case 0xd4: case 0xd6: sb->paused = false; sb->block_start = mach_pit_ticks(m); sb->frames_done = 0; return;
    case 0xd1: sb->speaker = true; return;
    case 0xd3: sb->speaker = false; return;
    case 0xd8: dsp_put(m, sb->speaker ? 0xff : 0x00); return;
    case 0xd9: case 0xda: sb->autoinit = false; return;
    case 0xe0: dsp_put(m, ~a[0]); return;
    case 0xe1: dsp_put(m, 4); dsp_put(m, 5); return;
    case 0xe3: {
        static const char cr[] = "COPYRIGHT (C) CREATIVE TECHNOLOGY LTD, 1992.";
        for (size_t i = 0; i < sizeof(cr); i++) {
            dsp_put(m, cr[i]);
        }
        return;
    }
    case 0xe4: sb->test = a[0]; return;
    case 0xe8: dsp_put(m, sb->test); return;
    case 0xf2:
        sb->mixer[0x82] |= 1;
        mach_raise_irq(m, SB_IRQ);
        return;
    case 0xf3:
        sb->mixer[0x82] |= 2;
        mach_raise_irq(m, SB_IRQ);
        return;
    case 0xf8: dsp_put(m, 0); return;
    default:
        return;
    }
}

/* PIT tick at which the running block completes */
static uint64_t sb_block_end(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    if (sb->block != sb->len_block || sb->rate != sb->len_rate) {
        sb->len_block = sb->block;
        sb->len_rate = sb->rate;
        sb->len_ticks = ((uint64_t)sb->block * PIT_HZ + sb->rate - 1) / sb->rate;
    }
    return sb->block_start + sb->len_ticks;
}

/* Advances the transfer to the current time: moves the DMA position, raises IRQs at block ends */
static void sb_update(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    uint64_t now = mach_pit_ticks(m);
    if (!sb->active || sb->paused || !sb->rate) {
        audio_render(m, now);
        return;
    }
    int ch = sb->bits16 ? SB_DMA16 : SB_DMA8;
    uint32_t unit = (sb->stereo ? 2 : 1);
    for (;;) {
        uint64_t end = sb_block_end(m);
        audio_render(m, now < end ? now : end);
        uint64_t frames = mach_div((now - sb->block_start) * sb->rate, PIT_HZ);
        if (frames > sb->block) {
            frames = sb->block;
        }
        if (frames > sb->frames_done) {
            dma_advance(m, ch, (uint32_t)(frames - sb->frames_done) * unit);
            sb->frames_done = (uint32_t)frames;
        }
        if (now < end) {
            break;
        }
        sb->mixer[0x82] |= sb->bits16 ? 2 : 1;
        mach_raise_irq(m, SB_IRQ);
        if (!sb->autoinit) {
            sb->active = false;
            break;
        }
        sb->block_start = end;
        sb->frames_done = 0;
    }
    if (!sb->active) {
        audio_render(m, now);
    }
}

uint64_t sound_next_event(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    uint64_t next = UINT64_MAX;
    if (sb->active && !sb->paused && sb->rate) {
        next = sb_block_end(m);
    }
    for (int i = 0; i < 2; i++) {
        opl_timer_t *t = &m->snd.opl_timer[i];
        if (t->running && !t->expired && t->expire < next) {
            next = t->expire;
        }
    }
    return next;
}

/* ---- OPL3 register file and timers ---- */

static void opl_update_timers(machine_t *m)
{
    uint64_t now = mach_pit_ticks(m);
    for (int i = 0; i < 2; i++) {
        opl_timer_t *t = &m->snd.opl_timer[i];
        if (t->running && !t->expired && now >= t->expire) {
            t->expired = true;
            if (!t->masked) {
                m->snd.opl_status |= 0x80 | (i == 0 ? 0x40 : 0x20);
            }
        }
    }
}

static void opl_write(machine_t *m, int bank, uint8_t reg, uint8_t v)
{
    sb_update(m);
    m->snd.opl_regs[bank][reg] = v;
    if (bank == 0 && reg == 4) {
        if (v & 0x80) {
            m->snd.opl_status = 0;
            return;
        }
        uint64_t now = mach_pit_ticks(m);
        for (int i = 0; i < 2; i++) {
            opl_timer_t *t = &m->snd.opl_timer[i];
            bool start = v & (1u << i);
            t->masked = v & (i == 0 ? 0x40 : 0x20);
            if (start && !t->running) {
                /* timer 1 ticks every 80 us, timer 2 every 320 us */
                uint32_t steps = 256 - m->snd.opl_regs[0][2 + i];
                uint64_t us = (uint64_t)steps * (i == 0 ? 80 : 320);
                t->expire = now + us * PIT_HZ / 1000000;
                t->expired = false;
            }
            t->running = start;
        }
    }
    audio_event(m, AUDIO_EV_OPL | (uint32_t)bank << 16 | (uint32_t)reg << 8 | v);
}

/* ---- ports ---- */

bool sound_port_quiet(const machine_t *m, uint16_t port, bool out)
{
    if (!out) {
        /* DMA address and count registers; the mixer */
        return port <= 0x07 || (port >= 0xc0 && port <= 0xcf) || port == SB_BASE + 4 || port == SB_BASE + 5;
    }
    switch (port) {
    case 0x0c: case 0xd8:       /* DMA flip-flop reset */
    case 0x388: case 0x38a: case SB_BASE: case SB_BASE + 2: case SB_BASE + 8:   /* OPL index */
    case 0x38b: case SB_BASE + 3:
    case SB_BASE + 4:           /* mixer index */
        return true;
    case 0x389: case SB_BASE + 1: case SB_BASE + 9:
        return m->snd.opl_idx[0] != 4;  /* timer control */
    }
    return false;
}

bool sound_in(machine_t *m, uint16_t port, uint8_t *v)
{
    if (port <= 0x0f || (port >= 0xc0 && port <= 0xdf)) {
        sb_update(m);
        *v = dma_in(m, port);
        return true;
    }
    for (int i = 0; i < 8; i++) {
        if (port == s_page_port[i]) {
            *v = dma_chan(m, i)->page;
            return true;
        }
    }
    if (port == 0x388 || port == 0x38a || port == SB_BASE || port == SB_BASE + 2 || port == SB_BASE + 8) {
        opl_update_timers(m);
        *v = m->snd.opl_status;
        return true;
    }
    if (port == 0x389 || port == 0x38b || port == SB_BASE + 1 || port == SB_BASE + 3 || port == SB_BASE + 9) {
        *v = 0xff;
        return true;
    }
    sb16_t *sb = &m->snd.sb;
    switch (port - SB_BASE) {
    case 0x5:
        *v = m->snd.sb.mixer[m->snd.sb.mixer_idx];
        return true;
    case 0x4:
        *v = sb->mixer_idx;
        return true;
    case 0xa:
        if (sb->out_n) {
            *v = sb->out[sb->out_head];
            sb->out_head = (sb->out_head + 1) % sizeof(sb->out);
            sb->out_n--;
        } else {
            *v = 0xff;
        }
        return true;
    case 0xc:
        *v = 0x7f; /* ready for commands */
        return true;
    case 0xe:
        sb_update(m);
        sb->mixer[0x82] &= ~1;
        *v = sb->out_n ? 0xff : 0x7f;
        return true;
    case 0xf:
        sb_update(m);
        sb->mixer[0x82] &= ~2;
        *v = 0xff;
        return true;
    }
    return false;
}

bool sound_out(machine_t *m, uint16_t port, uint8_t v)
{
    if (port <= 0x0f || (port >= 0xc0 && port <= 0xdf)) {
        sb_update(m);
        dma_out(m, port, v);
        return true;
    }
    for (int i = 0; i < 8; i++) {
        if (port == s_page_port[i]) {
            dma_chan(m, i)->page = v;
            return true;
        }
    }
    switch (port) {
    case 0x388: case SB_BASE: case SB_BASE + 8:
        m->snd.opl_idx[0] = v;
        return true;
    case 0x389: case SB_BASE + 1: case SB_BASE + 9:
        opl_write(m, 0, m->snd.opl_idx[0], v);
        return true;
    case 0x38a: case SB_BASE + 2:
        m->snd.opl_idx[1] = v;
        return true;
    case 0x38b: case SB_BASE + 3:
        opl_write(m, 1, m->snd.opl_idx[1], v);
        return true;
    }
    sb16_t *sb = &m->snd.sb;
    switch (port - SB_BASE) {
    case 0x4:
        sb->mixer_idx = v;
        return true;
    case 0x5:
        sb_update(m);
        mixer_write(m, sb->mixer_idx, v);
        return true;
    case 0x6:
        if (v & 1) {
            sb->resetting = true;
        } else if (sb->resetting) {
            dsp_reset(m);
        }
        return true;
    case 0xc:
        if (!sb->in_cmd) {
            sb->cmd = v;
            sb->nargs = dsp_args(v);
            sb->argi = 0;
            sb->in_cmd = sb->nargs > 0;
            if (!sb->in_cmd) {
                sb_update(m);
                dsp_command(m);
            }
        } else {
            sb->args[sb->argi++] = v;
            if (sb->argi == sb->nargs) {
                sb->in_cmd = false;
                sb_update(m);
                dsp_command(m);
            }
        }
        return true;
    }
    return false;
}

void sound_init(machine_t *m)
{
    memset(&m->snd, 0, sizeof(m->snd));
    for (int i = 0; i < 8; i++) {
        m->snd.dma[i].masked = true;
    }
    m->snd.sb.mixer[0x80] = 0x04; /* IRQ 7 */
    m->snd.sb.mixer[0x81] = 1u << SB_DMA8 | 1u << SB_DMA16;
    mixer_reset(m);
    m->snd.sb.rate = 22050;
    m->snd.out.opl_rem = -(int32_t)OPL3_RATE; /* frame 0 is at native sample 0 */
    m->snd.out.opl = opl3_create();
    if (!m->snd.out.opl) {
        mach_log(m, "OPL3: out of memory, FM music disabled");
    }
}

void sound_flush(machine_t *m)
{
    sb16_t *sb = &m->snd.sb;
    uint64_t now = mach_pit_ticks(m);
    if (sb->active && !sb->paused && sb->rate && sb_block_end(m) < now) {
        now = sb_block_end(m); /* the block end is sb_update()'s to process */
    }
    audio_render(m, now);
}

void sound_tick(machine_t *m)
{
    sb_update(m);
    opl_update_timers(m);
}
