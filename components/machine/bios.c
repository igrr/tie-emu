/* BIOS services: video (VGA + a VBE 1.2 card with banked 8-bit modes), keyboard,
 * timer, mouse driver, and the default IRQ0/IRQ1 handlers. */
#include <string.h>
#include "machine_priv.h"

#define BDA_TICKS       0x46c
#define BIOS_VBE_INFO   0xf1000u    /* OEM string and mode list the VBE info block points at */
#define BIOS_FONT       0xf2000u    /* 8x16 font (blank; the game draws its own) */

static const uint16_t s_vbe_modes[] = { 0x100, 0x101, 0xffff };

static void put16(uint8_t *p, uint16_t v)
{
    memcpy(p, &v, 2);
}

static void put32(uint8_t *p, uint32_t v)
{
    memcpy(p, &v, 4);
}

void bios_init(machine_t *m)
{
    uint8_t *bda = m->mem + MEM_BDA;
    put16(bda + 0x10, 0x0063);  /* equipment: FPU, VGA 80x25 color, 2 floppies */
    put16(bda + 0x13, 640);     /* conventional memory KB */
    bda[0x49] = 3;              /* video mode */
    put16(bda + 0x4a, 80);
    bda[0x84] = 24;

    uint8_t *v = m->mem + BIOS_VBE_INFO;
    strcpy((char *)v, "TIE emulator VBE");
    for (size_t i = 0; i < sizeof(s_vbe_modes) / sizeof(s_vbe_modes[0]); i++) {
        put16(v + 0x40 + 2 * i, s_vbe_modes[i]);
    }
    vga_set_mode(m, 3);
}

/* ---- video ---- */

static void vbe_mode_info(machine_t *m, uint16_t mode, uint8_t *p)
{
    uint16_t w = mode == 0x100 ? 640 : 640, h = mode == 0x100 ? 400 : 480;
    memset(p, 0, 256);
    put16(p + 0x00, 0x001b);    /* supported, optional info, BIOS output, color, graphics */
    p[0x02] = 0x07;             /* window A: exists, readable, writable */
    p[0x03] = 0x00;
    put16(p + 0x04, 64);        /* granularity KB */
    put16(p + 0x06, 64);        /* window size KB */
    put16(p + 0x08, 0xa000);
    put16(p + 0x0a, 0x0000);
    put32(p + 0x0c, 0xf0000000u | 4 * 0x10); /* window function: F000:0040, the INT 10h stub */
    put16(p + 0x10, w);
    put16(p + 0x12, w);
    put16(p + 0x14, h);
    p[0x16] = 8;
    p[0x17] = 16;
    p[0x18] = 1;                /* planes */
    p[0x19] = 8;                /* bpp */
    p[0x1a] = 1;                /* banks */
    p[0x1b] = 4;                /* packed pixel */
    p[0x1c] = 0;
    p[0x1d] = 1;                /* image pages - 1 */
    p[0x1e] = 1;
}

static void vbe(machine_t *m)
{
    uint32_t es = SEG_BASE(X86_ES);
    switch (AL) {
    case 0x00: {
        uint8_t *p = lin_ptr(m, es + DI, 256);
        if (!p) {
            break;
        }
        bool vbe2 = !memcmp(p, "VBE2", 4);
        memset(p, 0, vbe2 ? 512 : 256);
        memcpy(p, "VESA", 4);
        put16(p + 4, 0x0102);
        put32(p + 6, 0xf0000000u | (BIOS_VBE_INFO - 0xf0000));
        put32(p + 0x0a, 0);
        put32(p + 0x0e, 0xf0000000u | (BIOS_VBE_INFO - 0xf0000 + 0x40));
        put16(p + 0x12, 16);    /* 1 MB */
        SET_AX(0x004f);
        return;
    }
    case 0x01: {
        uint8_t *p = lin_ptr(m, es + DI, 256);
        if (!p || (CX != 0x100 && CX != 0x101)) {
            SET_AX(0x014f);
            return;
        }
        vbe_mode_info(m, CX, p);
        SET_AX(0x004f);
        return;
    }
    case 0x02: {
        uint16_t mode = BX & 0x1ff;
        if (mode < 0x100) {
            vga_set_mode(m, mode);
        } else if (mode == 0x100 || mode == 0x101) {
            vga_set_mode(m, mode);
            if (!(BX & 0x8000)) {
                memset(m->vga.vram, 0, m->vga.vram_size);
            }
        } else {
            SET_AX(0x014f);
            return;
        }
        SET_AX(0x004f);
        return;
    }
    case 0x03:
        SET_BX(m->vga.mode);
        SET_AX(0x004f);
        return;
    case 0x05:
        if (BH == 0) {
            if (BL == 0) {
                vga_set_bank(m, DX);
            }
        } else {
            SET_DX(m->vga.bank);
        }
        SET_AX(0x004f);
        return;
    case 0x06:
        if (BL == 1) {
            SET_BX(m->vga.pitch);
            SET_CX(m->vga.width);
            SET_DX(m->vga.vram_size / (m->vga.pitch ? m->vga.pitch : 1));
        }
        SET_AX(0x004f);
        return;
    case 0x07:
        if (BL == 0 || BL == 0x80) {
            uint32_t start = (uint32_t)DX * m->vga.pitch + CX;
            m->vga.crtc[0x0c] = start >> 8;
            m->vga.crtc[0x0d] = start;
        }
        SET_AX(0x004f);
        return;
    case 0x08:
        SET_BX(0x0600);
        SET_AX(0x004f);
        return;
    }
    mach_fault(m, "VBE function %02xh not implemented", AL);
}

void bios_int10(machine_t *m, uint32_t *unused)
{
    (void)unused;
    uint32_t es = SEG_BASE(X86_ES);
    switch (AH) {
    case 0x00:
        vga_set_mode(m, AL & 0x7f);
        return;
    case 0x01: case 0x02: case 0x05: case 0x06: case 0x07: case 0x09: case 0x0a: case 0x0b:
        return;
    case 0x03:
        SET_CX(0x0607);
        SET_DX(0);
        return;
    case 0x08:
        SET_AX(0x0720);
        return;
    case 0x0e:
        fputc(AL, stderr);
        return;
    case 0x0f:
        SET_AL(m->vga.mode < 0x100 ? m->vga.mode : 0x13);
        SET_AH(m->vga.mode == 3 ? 80 : 40);
        EBX &= 0xffff00ffu;
        return;
    case 0x10:
        switch (AL) {
        case 0x10:
            m->vga.dac[BX & 0xff][0] = DH & 0x3f;
            m->vga.dac[BX & 0xff][1] = CH & 0x3f;
            m->vga.dac[BX & 0xff][2] = CL & 0x3f;
            vga_out(m, 0x3c8, BX & 0xff); /* refresh the converted entry through the port model */
            vga_out(m, 0x3c9, DH);
            vga_out(m, 0x3c9, CH);
            vga_out(m, 0x3c9, CL);
            return;
        case 0x12: {
            uint8_t *p = lin_ptr(m, es + DX, CX * 3);
            if (!p) {
                return;
            }
            vga_out(m, 0x3c8, BX & 0xff);
            for (uint32_t i = 0; i < CX * 3; i++) {
                vga_out(m, 0x3c9, p[i]);
            }
            return;
        }
        case 0x15:
            SET_DX(m->vga.dac[BL][0] << 8);
            SET_CX(m->vga.dac[BL][1] << 8 | m->vga.dac[BL][2]);
            return;
        case 0x17: {
            uint8_t *p = lin_ptr(m, es + DX, CX * 3);
            for (uint32_t i = 0; p && i < CX; i++) {
                memcpy(p + 3 * i, m->vga.dac[(BX + i) & 0xff], 3);
            }
            return;
        }
        default:
            return; /* attribute controller / overscan: no visible effect here */
        }
    case 0x11:
        if (AL == 0x30) {
            x86_set_seg(&m->cpu, X86_ES, SEL_DATA);
            m->cpu.r[X86_EBP] = BIOS_FONT;
            SET_CX(16);
            SET_DX(24);
        }
        return;
    case 0x12:
        if (BL == 0x10) {
            SET_BX(0x0003);
            SET_CX(0x0009);
        }
        return;
    case 0x1a:
        if (AL == 0) {
            SET_AL(0x1a);
            SET_BX(0x0008);
        }
        return;
    case 0x4f:
        vbe(m);
        return;
    }
    mach_fault(m, "INT 10h AH=%02xh not implemented", AH);
}

/* ---- keyboard ---- */

/* set 1 make code -> ASCII, unshifted and shifted */
static const char s_ascii[2][0x3a] = {
    { 0, 27, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 8, 9, 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i',
      'o', 'p', '[', ']', 13, 0, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '`', 0, '\\', 'z', 'x', 'c',
      'v', 'b', 'n', 'm', ',', '.', '/', 0, '*', 0, ' ' },
    { 0, 27, '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', 8, 9, 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I',
      'O', 'P', '{', '}', 13, 0, 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '"', '~', 0, '|', 'Z', 'X', 'C',
      'V', 'B', 'N', 'M', '<', '>', '?', 0, '*', 0, ' ' },
};

/* BDA keyboard buffer at 0x41e..0x43d, head/tail at 0x41a/0x41c (offsets from 0x400) */
static void kbuf_put(machine_t *m, uint16_t key)
{
    uint8_t *bda = m->mem + MEM_BDA;
    uint16_t head, tail;
    memcpy(&head, bda + 0x1a, 2);
    memcpy(&tail, bda + 0x1c, 2);
    if (head < 0x1e || head >= 0x3e) {
        head = tail = 0x1e;
    }
    uint16_t next = tail + 2 >= 0x3e ? 0x1e : tail + 2;
    if (next == head) {
        return;
    }
    memcpy(bda + tail, &key, 2);
    memcpy(bda + 0x1a, &head, 2);
    memcpy(bda + 0x1c, &next, 2);
}

bool bios_key_get(machine_t *m, uint16_t *key, bool remove)
{
    uint8_t *bda = m->mem + MEM_BDA;
    uint16_t head, tail;
    memcpy(&head, bda + 0x1a, 2);
    memcpy(&tail, bda + 0x1c, 2);
    if (head == tail || head < 0x1e || head >= 0x3e) {
        return false;
    }
    memcpy(key, bda + head, 2);
    if (remove) {
        head = head + 2 >= 0x3e ? 0x1e : head + 2;
        memcpy(bda + 0x1a, &head, 2);
    }
    return true;
}

/* What INT 16h returns for a make code under the held modifiers, as a PC/AT BIOS's INT 9 puts it
 * in the buffer: scan code << 8 | ASCII. Alt and Ctrl combinations mostly have ASCII 0, and the
 * function keys and some others take other scan codes; the game reads its commands this way
 * (Alt-D, Shift-F9). 0: the combination puts nothing in the buffer. */
static uint16_t bios_key(uint8_t code, uint8_t flags)
{
    bool alt = flags & 0x08, ctrl = flags & 0x04, shift = flags & 0x03;
    if (code >= 0x3b && code <= 0x44) { /* F1..F10 */
        uint8_t n = code - 0x3b;
        return (uint16_t)((alt ? 0x68 + n : ctrl ? 0x5e + n : shift ? 0x54 + n : code) << 8);
    }
    if (code == 0x57 || code == 0x58) { /* F11, F12 */
        uint8_t n = code - 0x57;
        return (uint16_t)((alt ? 0x8b + n : ctrl ? 0x89 + n : shift ? 0x87 + n : 0x85 + n) << 8);
    }
    char ch = code < 0x3a ? s_ascii[shift ? 1 : 0][code] : 0;
    if (alt) {
        if (code >= 0x02 && code <= 0x0d) {
            return (uint16_t)((code + 0x76) << 8); /* digit row: 78h..83h */
        }
        if (code == 0x39) {
            return 0x3920;
        }
        if (code == 0x0f) {
            return 0xa500;
        }
        return ch || code == 0x01 ? (uint16_t)(code << 8) : 0;
    }
    if (ctrl) {
        if (ch >= 'a' && ch <= 'z') {
            return (uint16_t)(code << 8 | (ch - 'a' + 1));
        }
        switch (code) {
        case 0x01: return 0x011b;
        case 0x03: return 0x0300;                   /* Ctrl-2: NUL */
        case 0x07: return 0x071e;                   /* Ctrl-6 */
        case 0x0c: return 0x0c1f;                   /* Ctrl-- */
        case 0x0e: return 0x0e7f;
        case 0x0f: return 0x9400;
        case 0x1a: return 0x1a1b;
        case 0x1b: return 0x1b1d;
        case 0x1c: return 0x1c0a;
        case 0x2b: return 0x2b1c;
        case 0x39: return 0x3920;
        case 0x47: return 0x7700;                   /* Home, End, PgUp, PgDn, left, right */
        case 0x4f: return 0x7500;
        case 0x49: return 0x8400;
        case 0x51: return 0x7600;
        case 0x4b: return 0x7300;
        case 0x4d: return 0x7400;
        case 0x48: return 0x8d00;
        case 0x50: return 0x9100;
        }
        return 0;
    }
    if (shift && code == 0x0f) {
        return 0x0f00; /* back tab */
    }
    if (code == 0x4a || code == 0x4e) {
        return (uint16_t)(code << 8 | (code == 0x4a ? '-' : '+')); /* keypad - and + */
    }
    return (uint16_t)(code << 8 | (uint8_t)ch);
}

void bios_irq1(machine_t *m)
{
    uint8_t sc = kbd_in(m, 0x60);
    uint8_t *flags = m->mem + MEM_BDA + 0x17;
    bool rel = sc & 0x80;
    uint8_t code = sc & 0x7f;
    if (sc == 0xe0) {
        return;
    }
    uint8_t bit = code == 0x2a ? 0x02 : code == 0x36 ? 0x01 : code == 0x1d ? 0x04 : code == 0x38 ? 0x08 : 0;
    if (bit) {
        *flags = rel ? *flags & ~bit : *flags | bit;
        return;
    }
    if (rel) {
        return;
    }
    uint16_t key = bios_key(code, *flags);
    if (key) {
        kbuf_put(m, key);
    }
}

void bios_int16(machine_t *m)
{
    uint16_t key;
    switch (AH & 0xef) {
    case 0x00:
        if (!bios_key_get(m, &key, true)) {
            mach_wait_key(m);
            return;
        }
        SET_AX(key);
        return;
    case 0x01:
        if (bios_key_get(m, &key, false)) {
            SET_AX(key);
            m->cpu.eflags &= ~X86_ZF;
        } else {
            m->cpu.eflags |= X86_ZF;
        }
        return;
    case 0x02:
        SET_AL(m->mem[MEM_BDA + 0x17]);
        return;
    case 0x03:
    case 0x05:
        return;
    }
    mach_fault(m, "INT 16h AH=%02xh not implemented", AH);
}

/* ---- timer ---- */

void bios_irq0(machine_t *m)
{
    uint8_t *p = m->mem + BDA_TICKS;
    uint32_t t;
    memcpy(&t, p, 4);
    t++;
    if (t >= 0x1800b0) {
        t = 0;
        m->mem[MEM_BDA + 0x70] = 1;
    }
    memcpy(p, &t, 4);
    /* chain the user timer tick if someone hooked it */
    farptr_t s = stub_vector(0x1c);
    if (m->pmvec[0x1c].sel != s.sel || m->pmvec[0x1c].off != s.off) {
        x86_interrupt(&m->cpu, m->pmvec[0x1c].sel, m->pmvec[0x1c].off);
    }
}

void bios_int1a(machine_t *m)
{
    uint32_t t;
    memcpy(&t, m->mem + BDA_TICKS, 4);
    switch (AH) {
    case 0x00:
        SET_CX(t >> 16);
        SET_DX(t);
        SET_AL(m->mem[MEM_BDA + 0x70]);
        m->mem[MEM_BDA + 0x70] = 0;
        return;
    case 0x02: {
        uint32_t s = (uint32_t)(machine_time_us(m) / 1000000);
        uint32_t h = 12 + s / 3600, mi = (s / 60) % 60, sec = s % 60;
#define BCD(x) (((x) / 10) << 4 | ((x) % 10))
        SET_CX(BCD(h % 24) << 8 | BCD(mi));
        SET_DX(BCD(sec) << 8);
        return;
    }
    case 0x04:
        SET_CX(0x1995);
        SET_DX(0x0310);
        return;
#undef BCD
    }
    SET_CF();
}

/* ---- mouse (INT 33h) ---- */

void bios_int33(machine_t *m)
{
    switch (AX) {
    case 0x0000:
    case 0x0021:
        SET_AX(0xffff);
        SET_BX(2);
        m->mouse.minx = 0;
        m->mouse.maxx = 639;
        m->mouse.miny = 0;
        m->mouse.maxy = 199;
        m->mouse.x = 320;
        m->mouse.y = 100;
        return;
    case 0x0001:
        m->mouse.shown = true;
        return;
    case 0x0002:
        m->mouse.shown = false;
        return;
    case 0x0003:
        SET_BX(m->mouse.buttons);
        SET_CX(m->mouse.x);
        SET_DX(m->mouse.y);
        return;
    case 0x0004:
        m->mouse.x = (int16_t)CX;
        m->mouse.y = (int16_t)DX;
        return;
    case 0x0005:
    case 0x0006:
        SET_AX(m->mouse.buttons);
        SET_BX(0);
        return;
    case 0x0007:
        m->mouse.minx = (int16_t)CX;
        m->mouse.maxx = (int16_t)DX;
        return;
    case 0x0008:
        m->mouse.miny = (int16_t)CX;
        m->mouse.maxy = (int16_t)DX;
        return;
    case 0x000b:
        SET_CX(m->mouse.dx);
        SET_DX(m->mouse.dy);
        m->mouse.dx = m->mouse.dy = 0;
        return;
    case 0x000c: case 0x000f: case 0x001a: case 0x001d: case 0x0010:
        return;
    case 0x0024:
        SET_BX(0x0626);
        SET_CX(0x0400);
        return;
    }
    mach_fault(m, "INT 33h AX=%04xh not implemented", AX);
}
