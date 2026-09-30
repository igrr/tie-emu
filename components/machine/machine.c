#include <stdlib.h>
#include <string.h>
#include "machine_priv.h"
#include "aot.h"

/* ---- logging ---- */

void mach_log(machine_t *m, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[%10llu %08" PRIx32 "] ", (unsigned long long)m->cpu.icount, m->cpu.insn_eip);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

void mach_fault(machine_t *m, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m->fault, sizeof(m->fault), fmt, ap);
    va_end(ap);
    m->state = MACH_FAULT;
    x86_request_stop(&m->cpu);
}

/* ---- descriptors ---- */

int desc_alloc(machine_t *m, uint32_t base, uint32_t limit)
{
    for (int i = SEL_FIRST_FREE >> 3; i < NUM_DESC; i++) {
        if (!m->desc[i].used) {
            m->desc[i] = (desc_t){ .base = base, .limit = limit, .used = true };
            return i << 3;
        }
    }
    return -1;
}

static bool env_load_seg(x86_cpu *c, uint16_t sel, uint32_t *base, uint32_t *limit)
{
    machine_t *m = c->user;
    if ((sel & ~3u) == 0) { /* null selector: loadable into data registers */
        *base = 0;
        *limit = 0;
        return true;
    }
    desc_t *d = &m->desc[(sel >> 3) % NUM_DESC];
    if (!d->used) {
        return false;
    }
    *base = d->base;
    *limit = d->limit;
    return true;
}

static bool env_seg_limit(x86_cpu *c, uint16_t sel, uint32_t *limit)
{
    machine_t *m = c->user;
    desc_t *d = &m->desc[(sel >> 3) % NUM_DESC];
    if ((sel & ~3u) == 0 || !d->used) {
        return false;
    }
    *limit = d->limit;
    return true;
}

/* ---- interrupts ---- */

static bool is_default(machine_t *m, int vec)
{
    farptr_t s = stub_vector(vec);
    return m->pmvec[vec].sel == s.sel && m->pmvec[vec].off == s.off;
}

/* The BIOS handlers end with a non-specific EOI, like the real ones: a guest handler that chains to
 * them (Watcom's _chain_intr jumps there instead of acknowledging itself) relies on it. */
static void bios_eoi(machine_t *m, int vec)
{
    if (vec >= 0x70 && vec <= 0x77) {
        pic_out(m, 0xa0, 0x20);
        pic_out(m, 0x20, 0x20);
    } else if (vec >= 0x08 && vec <= 0x0f) {
        pic_out(m, 0x20, 0x20);
    }
}

/* HLE for vector `vec`; returns false if nothing handles it */
static bool hle_int(machine_t *m, int vec)
{
    switch (vec) {
    case 0x08: bios_eoi(m, vec); bios_irq0(m); return true;
    case 0x09: bios_irq1(m); bios_eoi(m, vec); return true;
    case 0x10: bios_int10(m, NULL); return true;
    case 0x16: bios_int16(m); return true;
    case 0x1a: bios_int1a(m); return true;
    case 0x1c: return true;
    case 0x21: dos_int21(m); return true;
    case 0x2f: dpmi_int2f(m); return true;
    case 0x31: dpmi_int31(m); return true;
    case 0x33: bios_int33(m); return true;
    case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
    case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77:
        bios_eoi(m, vec);
        return true;
    case 0x23: case 0x24: case 0x1b:
        return true; /* ctrl-break / critical error: ignore */
    default:
        mach_fault(m, "unhandled INT %02xh (eax=%08" PRIx32 ")", vec, EAX);
        return false;
    }
}

/* Guest instructions a software INT serviced here costs on top of its own: on the real machine
 * the DOS extender reflects BIOS, DOS and mouse driver calls to real mode (a mode switch each way,
 * microseconds on a Pentium) and handles DPMI's itself. Without it a game that polls the mouse or
 * keyboard in a loop (the in-flight keyboard reference) makes many times the calls per guest second
 * a real PC would, each costing the emulator far more than an instruction. Translated code picks
 * the charge up from icount (aot_int). */
#define INT_COST_REFLECTED 500u
#define INT_COST_DPMI 100u

static bool is_irq_vector(int vec)
{
    return (vec >= 0x08 && vec <= 0x0f) || (vec >= 0x70 && vec <= 0x77) || vec == 0x1c;
}

static void env_intr(x86_cpu *c, uint8_t vec)
{
    machine_t *m = c->user;
    if (is_default(m, vec)) {
        hle_int(m, vec);
        if (!is_irq_vector(vec)) {
            c->icount += vec == 0x31 || vec == 0x2f ? INT_COST_DPMI : INT_COST_REFLECTED;
        }
        return;
    }
    x86_interrupt(c, m->pmvec[vec].sel, m->pmvec[vec].off);
}

/* The default handler stubs (0F FF nn / IRETD) reached through a guest far call or IRQ chain.
 * A service's result flags (CF, ZF) must survive the IRETD, so they are copied into the flags
 * image. Not for hardware interrupts: there the image is the interrupted code's flags. */
static void env_hypercall(x86_cpu *c, uint8_t n)
{
    machine_t *m = c->user;
    uint32_t esp = c->r[X86_ESP];
    hle_int(m, n);
    if (is_irq_vector(n) || c->r[X86_ESP] != esp) {
        return; /* an IRQ, or the service chained into a guest handler (INT 1Ch) with its own frame */
    }
    uint32_t lin = c->base[X86_SS] + esp + 8;
    uint8_t *p = lin_ptr(m, lin, 4);
    if (p) {
        uint32_t fl;
        memcpy(&fl, p, 4);
        fl = (fl & ~(X86_CF | X86_ZF)) | (c->eflags & (X86_CF | X86_ZF));
        memcpy(p, &fl, 4);
    }
}

void mach_raise_irq(machine_t *m, int irq)
{
    pic_t *p = &m->pic[irq >> 3];
    p->irr |= 1u << (irq & 7);
    pic_update_intr(m);
}

void mach_lower_irq(machine_t *m, int irq)
{
    pic_t *p = &m->pic[irq >> 3];
    p->irr &= ~(1u << (irq & 7));
    pic_update_intr(m);
}

/* Delivers the highest-priority pending IRQ if the CPU accepts interrupts */
static void deliver_irqs(machine_t *m)
{
    if (!(m->cpu.eflags & X86_IF)) {
        return;
    }
    int irq = pic_next_irq(m);
    if (irq < 0) {
        return;
    }
    m->halted = false;
    pic_ack(m, irq);
    int vec = irq < 8 ? 8 + irq : 0x70 + irq - 8;
    if (is_default(m, vec)) {
        hle_int(m, vec);
        return;
    }
    x86_interrupt(&m->cpu, m->pmvec[vec].sel, m->pmvec[vec].off);
}

/* ---- ports ---- */

/* Device accesses can move the next timer event (PIT reload, SB transfer start, OPL timers) or
 * change what is deliverable (EOI, keyboard read); ending the CPU run after the instruction lets
 * machine_run re-evaluate at a point that depends only on the guest, never on the slice size.
 *
 * Quiet ports are those whose accesses move nothing by themselves: VGA registers, the OPL and SB
 * mixer indexes, OPL registers other than the timer control, DMA address and count reads
 * (sound_port_quiet). What such an access catches up on first (sb_update) may still end a sound
 * block and raise its IRQ, so the run ends when the access changed the next sound event or what
 * is pending, and goes on otherwise: what machine_run would do at the stop (catch the devices up
 * and deliver nothing) changes nothing then. Translated code polls them without leaving. */
static bool quiet_port(machine_t *m, uint16_t port, bool out)
{
    return (port >= 0x3c0 && port <= 0x3df) || port == 0x80 || sound_port_quiet(m, port, out);
}

static uint32_t dev_in(machine_t *m, uint16_t port);

static uint32_t env_in(x86_cpu *c, uint16_t port, int size)
{
    machine_t *m = c->user;
    if (port == 0x3da && size == 1) {
        /* VGA status: reads the time and moves nothing; games poll it millions of times a second */
        return vga_in(m, port);
    }
    if ((port & ~0x81u) == 0x20 && size == 1) {
        /* PIC IMR/IRR/ISR: moves nothing; the iMUSE drivers read the mask ~10k times a second */
        return pic_in(m, port);
    }
    if (size > 1) {
        uint32_t v = 0;
        for (int i = 0; i < size; i++) {
            v |= env_in(c, port + i, 1) << (8 * i);
        }
        return v;
    }
    if (!quiet_port(m, port, false)) {
        x86_request_stop(c);
        return dev_in(m, port);
    }
    uint64_t ev = sound_next_event(m);
    bool pending = c->intr_pending;
    uint32_t v = dev_in(m, port);
    if (c->intr_pending != pending || sound_next_event(m) != ev) {
        x86_request_stop(c);
    }
    return v;
}

/* x86_env.in_until: only the VGA status depends on nothing but the time. The first count whose
 * mach_pit_ticks reaches tick T is ceil(T * ips / PIT_HZ). */
static uint64_t env_in_until(x86_cpu *c, uint16_t port, uint8_t mask)
{
    machine_t *m = c->user;
    if (port != 0x3da) {
        return c->icount + 1;
    }
    uint64_t t = vga_status_until(m, mask);
    if (t == UINT64_MAX) {
        return UINT64_MAX;
    }
    uint32_t ips = m->cfg.ips;
    return t / PIT_HZ * ips + ((t % PIT_HZ) * ips + PIT_HZ - 1) / PIT_HZ;
}

static uint32_t dev_in(machine_t *m, uint16_t port)
{
    switch (port) {
    case 0x20: case 0x21: case 0xa0: case 0xa1:
        return pic_in(m, port);
    case 0x40: case 0x41: case 0x42: case 0x43:
        return pit_in(m, port);
    case 0x61: {
        /* bit 4 toggles with DRAM refresh (every 15 us); bit 5 is PIT channel 2's output */
        pit_update(m);
        uint64_t t = mach_pit_ticks(m);
        uint8_t v = m->port61 & 0x0f;
        if ((t / 18) & 1) {
            v |= 0x10;
        }
        pit_chan_t *ch = &m->pit[2];
        uint32_t reload = ch->reload ? ch->reload : 65536;
        uint64_t el = t - ch->start_tick;
        bool out = ch->mode == 0 ? el >= reload : ((el % reload) < reload / 2);
        if (out) {
            v |= 0x20;
        }
        return v;
    }
    case 0x60: case 0x64:
        return kbd_in(m, port);
    case 0x201:
        return 0xf0; /* joystick: no buttons, axes idle */
    default: {
        if (port >= 0x3c0 && port <= 0x3df) {
            return vga_in(m, port);
        }
        uint8_t v;
        if (sound_in(m, port, &v)) {
            return v;
        }
        return 0xff;
    }
    }
}

static void dev_out(machine_t *m, uint16_t port, uint8_t v);

static void env_out(x86_cpu *c, uint16_t port, uint32_t val, int size)
{
    machine_t *m = c->user;
    if (size > 1) {
        for (int i = 0; i < size; i++) {
            env_out(c, port + i, (val >> (8 * i)) & 0xff, 1);
        }
        return;
    }
    if (!quiet_port(m, port, true)) {
        x86_request_stop(c);
        dev_out(m, port, (uint8_t)val);
        return;
    }
    uint64_t ev = sound_next_event(m);
    bool pending = c->intr_pending;
    dev_out(m, port, (uint8_t)val);
    if (c->intr_pending != pending || sound_next_event(m) != ev) {
        x86_request_stop(c);
    }
}

static void dev_out(machine_t *m, uint16_t port, uint8_t v)
{
    switch (port) {
    case 0x20: case 0x21: case 0xa0: case 0xa1:
        pic_out(m, port, v);
        return;
    case 0x40: case 0x41: case 0x42: case 0x43:
        pit_out(m, port, v);
        return;
    case 0x61:
        pit_update(m);
        if ((v ^ m->port61) & 1) {
            m->pit[2].gate = v & 1;
            if (v & 1) {
                m->pit[2].start_tick = mach_pit_ticks(m);
            }
        }
        m->port61 = v;
        return;
    case 0x60: case 0x64:
        kbd_out(m, port, v);
        return;
    case 0x80:
        return; /* POST code / I/O delay */
    default:
        if (port >= 0x3c0 && port <= 0x3df) {
            vga_out(m, port, v);
            return;
        }
        if (sound_out(m, port, v)) {
            return;
        }
        return;
    }
}

static const x86_env s_env = {
    .in = env_in,
    .out = env_out,
    .intr = env_intr,
    .load_seg = env_load_seg,
    .seg_limit = env_seg_limit,
    .hypercall = env_hypercall,
    .in_until = env_in_until,
};

/* ---- time ---- */

/* floor(icount * PIT_HZ / ips). Called at every port access and device update: from the last
 * result, a small step is exact in 32 bits; otherwise it is recomputed without overflow for
 * any realistic run. The cache is not machine state. */
uint64_t mach_pit_ticks(const machine_t *cm)
{
    machine_t *m = (machine_t *)cm;
    uint64_t ic = m->cpu.icount;
    uint64_t step = ic - m->clk.icount;
    if (step == 0) {
        return m->clk.ticks;
    }
    uint32_t ips = m->cfg.ips;
    if (ic > m->clk.icount && step <= m->clk.max_step) {
        uint32_t f = m->clk.frac + (uint32_t)step * PIT_HZ;
        m->clk.ticks += f / ips;
        m->clk.frac = f % ips;
    } else {
        uint64_t r = (ic % ips) * PIT_HZ;
        m->clk.ticks = (ic / ips) * PIT_HZ + r / ips;
        m->clk.frac = (uint32_t)(r % ips);
    }
    m->clk.icount = ic;
    return m->clk.ticks;
}

uint64_t machine_time_us(const machine_t *m)
{
    uint64_t ic = m->cpu.icount;
    return (ic / m->cfg.ips) * 1000000u + (ic % m->cfg.ips) * 1000000u / m->cfg.ips;
}

static uint64_t ticks_to_icount(machine_t *m, uint64_t ticks)
{
    /* the next event is usually the one of the last call */
    if (ticks == m->clk.ev_tick) {
        return m->clk.ev_icount;
    }
    m->clk.ev_tick = ticks;
    return m->clk.ev_icount = (ticks / PIT_HZ) * m->cfg.ips + ((ticks % PIT_HZ) * m->cfg.ips + PIT_HZ - 1) / PIT_HZ;
}

/* ---- lifecycle ---- */

machine_t *machine_create(const machine_cfg_t *cfg)
{
    machine_t *m = calloc(1, sizeof(*m));
    if (!m) {
        return NULL;
    }
    m->cfg = *cfg;
    if (!m->cfg.ips) {
        m->cfg.ips = 50000000;
    }
    m->mem_size = cfg->mem_size ? cfg->mem_size : 16u << 20;
    m->clk.max_step = m->cfg.ips < UINT32_MAX - PIT_HZ ? (UINT32_MAX - m->cfg.ips) / PIT_HZ : 0;
    m->clk.ev_tick = UINT64_MAX;
    m->mem = calloc(1, m->mem_size);
    if (!m->mem) {
        free(m);
        return NULL;
    }
    x86_init(&m->cpu, m->mem, m->mem_size, &s_env, m);

    m->desc[SEL_CODE >> 3] = (desc_t){ .base = 0, .limit = 0xffffffffu, .used = true, .code = true };
    m->desc[SEL_DATA >> 3] = (desc_t){ .base = 0, .limit = 0xffffffffu, .used = true };
    m->desc[SEL_PSP >> 3] = (desc_t){ .base = MEM_PSP, .limit = 0xff, .used = true };
    m->desc[SEL_ENV >> 3] = (desc_t){ .base = MEM_ENV, .limit = 0x7ff, .used = true };

    for (int v = 0; v < 256; v++) {
        uint8_t *p = m->mem + MEM_STUBS + 4 * v;
        p[0] = 0x0f;
        p[1] = 0xff;
        p[2] = (uint8_t)v;
        p[3] = 0xcf; /* iretd */
        m->pmvec[v] = stub_vector(v);
        m->rmvec[v] = (farptr_t){ 0xf000, 4u * v };
    }

    pic_init(m);
    pit_init(m);
    sound_init(m);
    vga_init(m);
    bios_init(m);
    dos_init(m);
    m->state = MACH_RUNNING;
    return m;
}

void mach_wait_key(machine_t *m)
{
    m->cpu.eip -= 2;
    m->key_wait = true;
    x86_request_stop(&m->cpu);
}

machine_state_t machine_run(machine_t *m, uint64_t insns)
{
    uint64_t end = m->cpu.icount + insns;
    while (m->state == MACH_RUNNING && m->cpu.icount < end) {
        if (m->poll && m->cpu.icount >= m->poll_next) {
            m->poll_next = m->cpu.icount + m->poll_period;
            m->poll(m, m->poll_arg);
        }
        pit_update(m);
        sound_tick(m);
        kbd_update(m);
        deliver_irqs(m);

        /* run up to the next timer event */
        uint64_t next = end;
        uint64_t ev_tick = pit_next_event(m), snd_tick = sound_next_event(m);
        uint64_t ev = ticks_to_icount(m, snd_tick < ev_tick ? snd_tick : ev_tick);
        if (ev > m->cpu.icount && ev < next) {
            next = ev;
        }
        uint64_t n = next > m->cpu.icount ? next - m->cpu.icount : 1;
        if (m->halted) {
            /* nothing runs until an interrupt: let time pass up to the next event */
            m->cpu.icount = next > m->cpu.icount ? next : m->cpu.icount;
            continue;
        }
        x86_stop_t st = m->aot ? aot_run(&m->cpu, n) : x86_run(&m->cpu, n);
        switch (st) {
        case X86_STOP_EXC:
            if (m->state == MACH_RUNNING) {
                mach_fault(m, "CPU exception %d at %04x:%08" PRIx32 " (addr %08" PRIx32 ")", m->cpu.exc, m->cpu.sel[X86_CS],
                           m->cpu.eip, m->cpu.exc_addr);
            }
            break;
        case X86_STOP_HLT:
            m->halted = true;
            break;
        case X86_STOP_REQUEST:
            /* the INT would run again once per instruction until `next`, where the devices are
             * updated; an interrupt already pending would end the loop at the next instruction */
            if (m->key_wait && !(m->cpu.intr_pending && (m->cpu.eflags & X86_IF))) {
                m->cpu.icount = next > m->cpu.icount ? next : m->cpu.icount;
            }
            m->key_wait = false;
            break;
        default:
            break;
        }
    }
    sound_flush(m);
    return m->state;
}

void machine_set_poll(machine_t *m, machine_poll_fn fn, void *arg, uint32_t period_us)
{
    m->poll = fn;
    m->poll_arg = arg;
    m->poll_period = (uint64_t)m->cfg.ips * period_us / 1000000;
    m->poll_next = m->cpu.icount;
}

bool machine_aot(const machine_t *m)
{
    return m->aot;
}

uint32_t machine_vga_banks(const machine_t *m)
{
    return m->vga_banks;
}

void machine_describe_fault(const machine_t *m, char *buf, int len)
{
    const x86_cpu *c = &m->cpu;
    snprintf(buf, len,
             "%s\n  eax=%08" PRIx32 " ebx=%08" PRIx32 " ecx=%08" PRIx32 " edx=%08" PRIx32 " esi=%08" PRIx32 " edi=%08" PRIx32 " ebp=%08" PRIx32 " esp=%08" PRIx32 "\n"
             "  eip=%08" PRIx32 " eflags=%08" PRIx32 " cs=%04x ds=%04x es=%04x fs=%04x gs=%04x ss=%04x icount=%llu\n"
             "  pic irr/isr/imr %02x/%02x/%02x %02x/%02x/%02x",
             m->fault, c->r[X86_EAX], c->r[X86_EBX], c->r[X86_ECX], c->r[X86_EDX], c->r[X86_ESI], c->r[X86_EDI],
             c->r[X86_EBP], c->r[X86_ESP], c->eip, c->eflags, c->sel[X86_CS], c->sel[X86_DS], c->sel[X86_ES],
             c->sel[X86_FS], c->sel[X86_GS], c->sel[X86_SS], (unsigned long long)c->icount, m->pic[0].irr,
             m->pic[0].isr, m->pic[0].imr, m->pic[1].irr, m->pic[1].isr, m->pic[1].imr);
}

/* ---- program load ---- */

int machine_load(machine_t *m, const char *dos_path, const char *args)
{
    char host[512];
    if (!dos_host_path(m, dos_path, host, sizeof(host), true)) {
        mach_fault(m, "%s: not found", dos_path);
        return -1;
    }
    uint32_t eip, esp;
    if (le_load(m, host, MEM_IMAGE_DELTA, &eip, &esp) != 0) {
        return -1;
    }

    /* PSP: INT 20h at 0, top of memory at 2, environment selector at 2Ch, command tail at 80h */
    uint8_t *psp = m->mem + MEM_PSP;
    psp[0] = 0xcd;
    psp[1] = 0x20;
    psp[2] = 0x00;
    psp[3] = 0xa0;
    psp[0x2c] = SEL_ENV & 0xff;
    psp[0x2d] = SEL_ENV >> 8;
    size_t alen = args ? strlen(args) : 0;
    if (alen > 125) {
        alen = 125;
    }
    psp[0x80] = (uint8_t)(alen + (alen ? 1 : 0));
    if (alen) {
        psp[0x81] = ' ';
        memcpy(psp + 0x82, args, alen);
    }
    psp[0x81 + psp[0x80]] = 0x0d;
    m->dta = MEM_PSP + 0x80;

    /* environment: NAME=value strings, an empty string, a word count, then the program path */
    static const char *const envs[] = { "COMSPEC=C:\\COMMAND.COM", "PATH=C:\\", "BLASTER=A220 I7 D1 H5 T6",
                                        "DOS4G=QUIET" };
    char *e = (char *)m->mem + MEM_ENV;
    for (size_t i = 0; i < sizeof(envs) / sizeof(envs[0]); i++) {
        strcpy(e, envs[i]);
        e += strlen(envs[i]) + 1;
    }
    *e++ = 0;
    *e++ = 1;
    *e++ = 0;
    strcpy(e, dos_path);

    x86_cpu *c = &m->cpu;
    x86_set_seg(c, X86_CS, SEL_CODE);
    x86_set_seg(c, X86_DS, SEL_DATA);
    x86_set_seg(c, X86_SS, SEL_DATA);
    x86_set_seg(c, X86_ES, SEL_PSP);
    x86_set_seg(c, X86_FS, 0);
    x86_set_seg(c, X86_GS, 0);
    c->eip = eip;
    c->r[X86_ESP] = esp;
    c->eflags = 0x3202; /* IF, IOPL 3 */
    m->aot = aot_init(m->mem, m->mem_size);
    return 0;
}

/* ---- host input and output ---- */

void machine_frame(machine_t *m, machine_frame_t *out)
{
    const uint8_t *px;
    uint32_t size;
    if (m->vga.mode >= 0x100) {
        px = m->vga.vram; /* banked mode: the window is in vram (vga_map) */
        size = m->vga.vram_size;
    } else {
        px = m->mem + MEM_VGA;
        size = 0x20000; /* A0000h..BFFFFh */
    }
    uint32_t start = ((uint32_t)m->vga.crtc[0x0c] << 8 | m->vga.crtc[0x0d]);
    if (m->vga.mode == 0x13) {
        start = 0; /* chained mode 13h: the start address counts planar words */
    }
    start = start < size ? start : size;
    out->pixels = px + start;
    out->size = size - start;
    out->pitch = m->vga.pitch;
    out->width = m->vga.width;
    out->height = m->vga.height;
    out->palette = m->vga.palette;
}

uint32_t machine_frame_count(const machine_t *m)
{
    return m->vga.frames;
}
