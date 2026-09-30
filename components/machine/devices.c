/* Register-level devices: 8259 PIC pair, 8254 PIT, VGA (DAC, CRTC, retrace status) and
 * the 8042 keyboard controller. Time comes from mach_pit_ticks() (executed instructions). */
#include <stdlib.h>
#include <string.h>
#include "machine_priv.h"

/* ---- PIC ---- */

void pic_init(machine_t *m)
{
    m->pic[0] = (pic_t){ .vector_base = 0x08 };
    m->pic[1] = (pic_t){ .vector_base = 0x70 };
}

uint8_t pic_in(machine_t *m, uint16_t port)
{
    pic_t *p = &m->pic[port >= 0xa0];
    if (port & 1) {
        return p->imr;
    }
    return p->read_isr ? p->isr : p->irr;
}

static void eoi(pic_t *p)
{
    for (int i = 0; i < 8; i++) {
        if (p->isr & (1u << i)) {
            p->isr &= ~(1u << i);
            return;
        }
    }
}

void pic_out(machine_t *m, uint16_t port, uint8_t v)
{
    pic_t *p = &m->pic[port >= 0xa0];
    if (!(port & 1)) {
        if (v & 0x10) {             /* ICW1 */
            p->icw_step = (v & 1) ? 3 : 2;
            p->imr = 0;
            p->isr = 0;
        } else if (v & 0x08) {      /* OCW3 */
            if (v & 2) {
                p->read_isr = v & 1;
            }
        } else if ((v & 0xe0) == 0x20) {
            eoi(p);
        } else if ((v & 0xe0) == 0x60) {
            p->isr &= ~(1u << (v & 7));
        }
        pic_update_intr(m);
        return;
    }
    if (p->icw_step) {
        if (p->icw_step >= 2 && p->icw_step < 5) {
            if (p->icw_step == 2 || p->icw_step == 3) {
                p->vector_base = v & 0xf8;
            }
            p->icw_step = p->icw_step == 3 ? 4 : 0; /* ICW3 and ICW4 carry nothing modelled */
        } else {
            p->icw_step = 0;
        }
        return;
    }
    p->imr = v;
    pic_update_intr(m);
}

static int pic_pending(pic_t *p)
{
    uint8_t pend = p->irr & ~p->imr;
    for (int i = 0; i < 8; i++) {
        if (p->isr & (1u << i)) {
            return -1; /* something of higher priority is in service */
        }
        if (pend & (1u << i)) {
            return i;
        }
    }
    return -1;
}

/* Tells the CPU whether an interrupt is waiting, so it stops as soon as IF allows */
void pic_update_intr(machine_t *m)
{
    m->cpu.intr_pending = pic_next_irq(m) >= 0;
}

int pic_next_irq(machine_t *m)
{
    int s = pic_pending(&m->pic[1]);
    if (s >= 0) {
        m->pic[0].irr |= 1u << 2;
    }
    int i = pic_pending(&m->pic[0]);
    if (i < 0) {
        return -1;
    }
    return i == 2 && s >= 0 ? 8 + s : (i == 2 ? -1 : i);
}

void pic_ack(machine_t *m, int irq)
{
    if (irq >= 8) {
        m->pic[1].irr &= ~(1u << (irq - 8));
        m->pic[1].isr |= 1u << (irq - 8);
        irq = 2;
    }
    m->pic[0].irr &= ~(1u << irq);
    m->pic[0].isr |= 1u << irq;
    pic_update_intr(m);
}

/* ---- PIT ---- */

void pit_init(machine_t *m)
{
    for (int i = 0; i < 3; i++) {
        m->pit[i] = (pit_chan_t){ .reload = 0, .mode = 3, .access = 3, .gate = i != 2 };
    }
    m->next_irq0_tick = 65536;
}

static uint32_t reload_of(const pit_chan_t *ch)
{
    return ch->reload ? ch->reload : 65536;
}

static uint16_t pit_count(machine_t *m, int i)
{
    pit_chan_t *ch = &m->pit[i];
    uint32_t r = reload_of(ch);
    if (!ch->gate) {
        return (uint16_t)r;
    }
    uint64_t el = mach_pit_ticks(m) - ch->start_tick;
    switch (ch->mode) {
    case 0:
    case 1:
        return el >= r ? (uint16_t)(0x10000 - (el - r) % 0x10000) : (uint16_t)(r - el);
    case 3: {
        /* square wave: counts down by two, twice per period */
        uint32_t half = r / 2 ? r / 2 : 1;
        return (uint16_t)(r - 2 * (el % half));
    }
    default:
        return (uint16_t)(r - el % r);
    }
}

uint8_t pit_in(machine_t *m, uint16_t port)
{
    if (port == 0x43) {
        return 0xff;
    }
    pit_chan_t *ch = &m->pit[port - 0x40];
    uint16_t v = ch->latched ? ch->latch : pit_count(m, port - 0x40);
    uint8_t out;
    switch (ch->access) {
    case 1: out = v; ch->latched = false; break;
    case 2: out = v >> 8; ch->latched = false; break;
    default:
        out = ch->read_hi_next ? v >> 8 : v;
        if (ch->read_hi_next) {
            ch->latched = false;
        }
        ch->read_hi_next = !ch->read_hi_next;
        break;
    }
    return out;
}

static void pit_loaded(machine_t *m, int i)
{
    pit_chan_t *ch = &m->pit[i];
    ch->start_tick = mach_pit_ticks(m);
    if (i == 0) {
        m->next_irq0_tick = ch->start_tick + reload_of(ch);
    }
}

void pit_out(machine_t *m, uint16_t port, uint8_t v)
{
    if (port == 0x43) {
        int sc = v >> 6;
        if (sc == 3) {
            return; /* read-back: not used */
        }
        pit_chan_t *ch = &m->pit[sc];
        if (((v >> 4) & 3) == 0) {
            ch->latch = pit_count(m, sc);
            ch->latched = true;
            ch->read_hi_next = false;
            return;
        }
        ch->access = (v >> 4) & 3;
        ch->mode = (v >> 1) & 7;
        if (ch->mode > 5) {
            ch->mode -= 4;
        }
        ch->write_hi_next = false;
        ch->read_hi_next = false;
        return;
    }
    int i = port - 0x40;
    pit_chan_t *ch = &m->pit[i];
    switch (ch->access) {
    case 1:
        ch->reload = (ch->reload & 0xff00) | v;
        pit_loaded(m, i);
        break;
    case 2:
        ch->reload = (ch->reload & 0x00ff) | v << 8;
        pit_loaded(m, i);
        break;
    default:
        if (!ch->write_hi_next) {
            ch->reload = (ch->reload & 0xff00) | v;
        } else {
            ch->reload = (ch->reload & 0x00ff) | v << 8;
            pit_loaded(m, i);
        }
        ch->write_hi_next = !ch->write_hi_next;
        break;
    }
}

/* ~70 Hz VGA frames: 449 lines of 38 PIT ticks */
#define LINE_TICKS 38u
#define FRAME_LINES 449u
#define FRAME_TICKS (LINE_TICKS * FRAME_LINES)
#define VRETRACE_LINES 35u

void pit_update(machine_t *m)
{
    uint64_t now = mach_pit_ticks(m);
    if (now >= m->next_irq0_tick) {
        mach_raise_irq(m, 0);
        uint32_t r = reload_of(&m->pit[0]);
        /* coalesce missed ticks, as a PIC does */
        m->next_irq0_tick += (mach_div(now - m->next_irq0_tick, r) + 1) * r;
    }
    m->vga.frames = (uint32_t)mach_div(now, FRAME_TICKS);
}

uint64_t pit_next_event(machine_t *m)
{
    return m->next_irq0_tick;
}

/* ---- VGA ---- */

void vga_init(machine_t *m)
{
    m->vga.vram_size = 1u << 20;
    m->vga.vram = calloc(1, m->vga.vram_size);
    for (int i = 0; i < 256; i++) {
        m->vga.palette[i] = 0;
    }
}

/* In the banked (VESA) modes the CPU sees the current bank of vram at MEM_VGA; the other modes
 * use guest memory there. Switching between the two copies the window, so the guest sees what
 * it would with one memory behind the window. */
static void vga_map(machine_t *m, bool banked)
{
    x86_cpu *c = &m->cpu;
    if (banked && !c->win) {
        memcpy(m->vga.vram, m->mem + MEM_VGA, 0x10000);
    } else if (!banked && c->win) {
        memcpy(m->mem + MEM_VGA, c->win, 0x10000);
    }
    x86_set_window(c, banked ? m->vga.vram + m->vga.bank * 0x10000u : NULL);
}

void vga_set_mode(machine_t *m, int mode)
{
    m->vga.mode = mode;
    m->vga.bank = 0;
    vga_map(m, mode >= 0x100);
    memset(m->vga.crtc, 0, sizeof(m->vga.crtc));
    switch (mode) {
    case 0x13:
        m->vga.width = 320;
        m->vga.height = 200;
        m->vga.pitch = 320;
        memset(m->mem + MEM_VGA, 0, 0x10000);
        break;
    case 0x100:
        m->vga.width = 640;
        m->vga.height = 400;
        m->vga.pitch = 640;
        break;
    case 0x101:
        m->vga.width = 640;
        m->vga.height = 480;
        m->vga.pitch = 640;
        break;
    default: /* text: show nothing */
        m->vga.width = 320;
        m->vga.height = 200;
        m->vga.pitch = 320;
        break;
    }
    m->vga.crtc[0x13] = m->vga.pitch / 8;
    m->mem[MEM_BDA + 0x49] = mode < 0x100 ? mode : 0x13;
}

void vga_set_bank(machine_t *m, uint32_t bank)
{
    uint32_t nbanks = m->vga.vram_size / 0x10000;
    if (bank >= nbanks || bank == m->vga.bank) {
        return;
    }
    m->vga_banks++;
    m->vga.bank = bank;
    vga_map(m, m->vga.mode >= 0x100);
}

/* The first PIT tick after the current one at which the input status bits in mask (vga_in,
 * 3DAh) may change: the start or the end of the vertical retrace, and for bit 0 also the
 * horizontal retrace of each line */
uint64_t vga_status_until(machine_t *m, uint8_t mask)
{
    uint64_t now = mach_pit_ticks(m);
    uint64_t frame = now - mach_mod(now, FRAME_TICKS);
    uint32_t t = (uint32_t)(now - frame);
    uint32_t vr = (FRAME_LINES - VRETRACE_LINES) * LINE_TICKS;
    if (t >= vr || !(mask & 0x09)) {
        return (mask & 0x09) ? frame + FRAME_TICKS : UINT64_MAX;
    }
    if (!(mask & 0x01)) {
        return frame + vr;
    }
    uint32_t line = t - t % LINE_TICKS, hr = LINE_TICKS * 4 / 5;
    return frame + (t % LINE_TICKS < hr ? line + hr : line + LINE_TICKS);
}

uint8_t vga_in(machine_t *m, uint16_t port)
{
    switch (port) {
    case 0x3da: {
        m->vga.attr_flip = false;
        uint32_t t = (uint32_t)mach_mod(mach_pit_ticks(m), FRAME_TICKS);
        uint32_t line = (uint32_t)(t / LINE_TICKS);
        uint8_t v = 0;
        if (line >= FRAME_LINES - VRETRACE_LINES) {
            v |= 0x09;
        } else if (t % LINE_TICKS >= LINE_TICKS * 4 / 5) {
            v |= 0x01;
        }
        return v;
    }
    case 0x3c7:
        return m->vga.dac_comp ? 0x03 : 0x00;
    case 0x3c8:
        return m->vga.dac_write_idx;
    case 0x3c9: {
        uint8_t v = m->vga.dac[m->vga.dac_read_idx][m->vga.dac_comp];
        if (++m->vga.dac_comp == 3) {
            m->vga.dac_comp = 0;
            m->vga.dac_read_idx++;
        }
        return v;
    }
    case 0x3d4: return m->vga.crtc_idx;
    case 0x3d5: return m->vga.crtc[m->vga.crtc_idx & 31];
    case 0x3ce: return m->vga.gc_idx;
    case 0x3cf: return m->vga.gc[m->vga.gc_idx & 15];
    case 0x3c4: return m->vga.seq_idx;
    case 0x3c5: return m->vga.seq[m->vga.seq_idx & 7];
    case 0x3cc: return m->vga.misc;
    case 0x3c1: return m->vga.attr[m->vga.attr_idx & 31];
    }
    return 0xff;
}

void vga_out(machine_t *m, uint16_t port, uint8_t v)
{
    switch (port) {
    case 0x3c7:
        m->vga.dac_read_idx = v;
        m->vga.dac_comp = 0;
        return;
    case 0x3c8:
        m->vga.dac_write_idx = v;
        m->vga.dac_comp = 0;
        return;
    case 0x3c9: {
        uint8_t i = m->vga.dac_write_idx;
        m->vga.dac[i][m->vga.dac_comp] = v & 0x3f;
        if (++m->vga.dac_comp == 3) {
            m->vga.dac_comp = 0;
            uint8_t *c = m->vga.dac[i];
            m->vga.palette[i] = (uint32_t)(c[0] << 2 | c[0] >> 4) << 16 | (uint32_t)(c[1] << 2 | c[1] >> 4) << 8 |
                                (uint32_t)(c[2] << 2 | c[2] >> 4);
            m->vga.dac_write_idx++;
        }
        return;
    }
    case 0x3d4: m->vga.crtc_idx = v; return;
    case 0x3d5:
        m->vga.crtc[m->vga.crtc_idx & 31] = v;
        if ((m->vga.crtc_idx & 31) == 0x13 && m->vga.mode >= 0x100) {
            m->vga.pitch = v * 8u;
        }
        return;
    case 0x3ce: m->vga.gc_idx = v; return;
    case 0x3cf:
        m->vga.gc[m->vga.gc_idx & 15] = v;
        return;
    case 0x3c4: m->vga.seq_idx = v; return;
    case 0x3c5:
        m->vga.seq[m->vga.seq_idx & 7] = v;
        return;
    case 0x3c2: m->vga.misc = v; return;
    case 0x3c0:
        if (!m->vga.attr_flip) {
            m->vga.attr_idx = v;
        } else {
            m->vga.attr[m->vga.attr_idx & 31] = v;
        }
        m->vga.attr_flip = !m->vga.attr_flip;
        return;
    }
}

/* ---- keyboard controller ---- */

void machine_key(machine_t *m, uint8_t scancode)
{
    int next = (m->kbd.tail + 1) % (int)sizeof(m->kbd.buf);
    if (next != m->kbd.head) {
        m->kbd.buf[m->kbd.tail] = scancode;
        m->kbd.tail = next;
    }
}

void machine_mouse(machine_t *m, int dx, int dy, unsigned buttons)
{
    m->mouse.dx += dx;
    m->mouse.dy += dy;
    m->mouse.buttons = buttons;
    m->mouse.x += dx;
    m->mouse.y += dy;
    if (m->mouse.x < m->mouse.minx) m->mouse.x = m->mouse.minx;
    if (m->mouse.x > m->mouse.maxx) m->mouse.x = m->mouse.maxx;
    if (m->mouse.y < m->mouse.miny) m->mouse.y = m->mouse.miny;
    if (m->mouse.y > m->mouse.maxy) m->mouse.y = m->mouse.maxy;
}

/* Presents the next queued byte once the previous one was read */
void kbd_update(machine_t *m)
{
    if (m->kbd.irq_pending || m->kbd.head == m->kbd.tail) {
        return;
    }
    m->kbd.data = m->kbd.buf[m->kbd.head];
    m->kbd.head = (m->kbd.head + 1) % (int)sizeof(m->kbd.buf);
    m->kbd.irq_pending = true;
    mach_raise_irq(m, 1);
}

uint8_t kbd_in(machine_t *m, uint16_t port)
{
    if (port == 0x64) {
        return (m->kbd.irq_pending ? 0x01 : 0x00) | 0x14;
    }
    /* reading the output buffer drops IRQ1: a byte the program read by polling before its
     * interrupt came is not delivered again */
    m->kbd.irq_pending = false;
    mach_lower_irq(m, 1);
    return m->kbd.data;
}

void kbd_out(machine_t *m, uint16_t port, uint8_t v)
{
    if (port == 0x60) {
        /* keyboard commands (LEDs, typematic): acknowledge each byte */
        int prev = (m->kbd.head + (int)sizeof(m->kbd.buf) - 1) % (int)sizeof(m->kbd.buf);
        if (prev != m->kbd.tail) {
            m->kbd.buf[prev] = 0xfa;
            m->kbd.head = prev;
        }
    }
}
