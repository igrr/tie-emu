/* DPMI 0.9 host services (INT 31h) as DOS/4GW provides them, and the INT 2Fh multiplex queries. */
#include <string.h>
#include "machine_priv.h"

static void fail(machine_t *m, uint16_t err)
{
    SET_AX(err);
    SET_CF();
}

static desc_t *get_desc(machine_t *m, uint16_t sel)
{
    desc_t *d = &m->desc[(sel >> 3) % NUM_DESC];
    return (sel & ~3u) && d->used ? d : NULL;
}

/* Reload any segment register that holds `sel`, after its descriptor changed */
static void refresh_segs(machine_t *m, uint16_t sel)
{
    for (int s = 0; s < 6; s++) {
        if (m->cpu.sel[s] == sel) {
            x86_set_seg(&m->cpu, s, sel);
        }
    }
}

static int alloc_descs(machine_t *m, int count)
{
    for (int i = SEL_FIRST_FREE >> 3; i + count <= NUM_DESC; i++) {
        int k = 0;
        while (k < count && !m->desc[i + k].used) {
            k++;
        }
        if (k == count) {
            for (k = 0; k < count; k++) {
                m->desc[i + k] = (desc_t){ .base = 0, .limit = 0, .used = true };
            }
            return i << 3;
        }
    }
    return -1;
}

/* ---- extended memory blocks (0501h) ---- */

static memblock_t *xms_find(machine_t *m, uint32_t handle)
{
    return handle >= 1 && handle <= MAX_XMS_BLOCKS && m->xms[handle - 1].used ? &m->xms[handle - 1] : NULL;
}

static uint32_t xms_place(machine_t *m, uint32_t size, int skip)
{
    uint32_t at = m->heap_lo;
    for (;;) {
        bool moved = false;
        for (int i = 0; i < MAX_XMS_BLOCKS; i++) {
            memblock_t *b = &m->xms[i];
            if (i != skip && b->used && at < b->base + b->size && b->base < at + size) {
                at = (b->base + b->size + 0xfff) & ~0xfffu;
                moved = true;
            }
        }
        if (!moved) {
            break;
        }
    }
    return at + size <= m->heap_hi && at + size > at ? at : 0;
}

static uint32_t xms_largest(machine_t *m)
{
    /* coarse: the gap after the highest block */
    uint32_t top = m->heap_lo;
    for (int i = 0; i < MAX_XMS_BLOCKS; i++) {
        if (m->xms[i].used && m->xms[i].base + m->xms[i].size > top) {
            top = (m->xms[i].base + m->xms[i].size + 0xfff) & ~0xfffu;
        }
    }
    return top < m->heap_hi ? m->heap_hi - top : 0;
}

/* ---- real-mode call structure ---- */

typedef struct {
    uint32_t edi, esi, ebp, reserved, ebx, edx, ecx, eax;
    uint16_t flags, es, ds, fs, gs, ip, cs, sp, ss;
} __attribute__((packed)) rmregs_t;

/* Runs an HLE service with the CPU temporarily holding the real-mode register image */
static void simulate_rm_int(machine_t *m, int vec, uint32_t struct_lin)
{
    rmregs_t rm;
    uint8_t *p = lin_ptr(m, struct_lin, sizeof(rm));
    if (!p) {
        fail(m, 0x8021);
        return;
    }
    memcpy(&rm, p, sizeof(rm));
    x86_cpu *c = &m->cpu;
    uint32_t save_r[8], save_base[6], save_fl = c->eflags;
    uint16_t save_sel[6];
    memcpy(save_r, c->r, sizeof(save_r));
    memcpy(save_base, c->base, sizeof(save_base));
    memcpy(save_sel, c->sel, sizeof(save_sel));

    c->r[X86_EAX] = rm.eax;
    c->r[X86_EBX] = rm.ebx;
    c->r[X86_ECX] = rm.ecx;
    c->r[X86_EDX] = rm.edx;
    c->r[X86_ESI] = rm.esi;
    c->r[X86_EDI] = rm.edi;
    c->r[X86_EBP] = rm.ebp;
    c->eflags = (c->eflags & ~0xfffu) | (rm.flags & 0xfff);
    const uint16_t segs[6] = { rm.es, 0, rm.ss, rm.ds, rm.fs, rm.gs };
    for (int s = 0; s < 6; s++) {
        if (s != X86_CS) {
            c->base[s] = (uint32_t)segs[s] << 4;
            c->sel[s] = segs[s];
        }
    }
    switch (vec) {
    case 0x10: bios_int10(m, NULL); break;
    case 0x16: bios_int16(m); break;
    case 0x1a: bios_int1a(m); break;
    case 0x21: dos_int21(m); break;
    case 0x2f: dpmi_int2f(m); break;
    case 0x33: bios_int33(m); break;
    default: mach_fault(m, "DPMI 0300h: real-mode INT %02xh not implemented", vec); break;
    }
    rm.eax = c->r[X86_EAX];
    rm.ebx = c->r[X86_EBX];
    rm.ecx = c->r[X86_ECX];
    rm.edx = c->r[X86_EDX];
    rm.esi = c->r[X86_ESI];
    rm.edi = c->r[X86_EDI];
    rm.ebp = c->r[X86_EBP];
    rm.flags = (uint16_t)(c->eflags & 0xfff);
    rm.es = c->sel[X86_ES] == segs[X86_ES] ? rm.es : (uint16_t)(c->base[X86_ES] >> 4);
    memcpy(p, &rm, sizeof(rm));

    memcpy(c->r, save_r, sizeof(save_r));
    memcpy(c->base, save_base, sizeof(save_base));
    memcpy(c->sel, save_sel, sizeof(save_sel));
    c->eflags = save_fl & ~X86_CF;
}

void dpmi_int31(machine_t *m)
{
    CLR_CF();
    desc_t *d;
    switch (AX) {
    case 0x0000: {
        int sel = alloc_descs(m, CX ? CX : 1);
        if (sel < 0) {
            fail(m, 0x8011);
            return;
        }
        SET_AX(sel);
        return;
    }
    case 0x0001:
        if (!(d = get_desc(m, BX)) || (BX >> 3) < (SEL_FIRST_FREE >> 3)) {
            fail(m, 0x8022);
            return;
        }
        d->used = false;
        return;
    case 0x0002: {
        int sel = desc_alloc(m, (uint32_t)BX << 4, 0xffff);
        if (sel < 0) {
            fail(m, 0x8011);
            return;
        }
        SET_AX(sel);
        return;
    }
    case 0x0003:
        SET_AX(8);
        return;
    case 0x0006:
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        SET_CX(d->base >> 16);
        SET_DX(d->base);
        return;
    case 0x0007:
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        d->base = (uint32_t)CX << 16 | DX;
        refresh_segs(m, BX);
        return;
    case 0x0008:
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        d->limit = (uint32_t)CX << 16 | DX;
        return;
    case 0x0009:
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        d->code = CL & 8;
        return;
    case 0x000a: {
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        int sel = desc_alloc(m, d->base, d->limit);
        if (sel < 0) {
            fail(m, 0x8011);
            return;
        }
        SET_AX(sel);
        return;
    }
    case 0x000b: {
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        uint8_t *p = lin_ptr(m, SEG_BASE(X86_ES) + EDI, 8);
        uint32_t lim = d->limit > 0xfffff ? d->limit >> 12 : d->limit;
        p[0] = lim;
        p[1] = lim >> 8;
        p[2] = d->base;
        p[3] = d->base >> 8;
        p[4] = d->base >> 16;
        p[5] = d->code ? 0xfa : 0xf2;
        p[6] = ((lim >> 16) & 0xf) | 0x40 | (d->limit > 0xfffff ? 0x80 : 0);
        p[7] = d->base >> 24;
        return;
    }
    case 0x000c: {
        if (!(d = get_desc(m, BX))) {
            fail(m, 0x8022);
            return;
        }
        uint8_t *p = lin_ptr(m, SEG_BASE(X86_ES) + EDI, 8);
        d->base = p[2] | p[3] << 8 | p[4] << 16 | (uint32_t)p[7] << 24;
        uint32_t lim = p[0] | p[1] << 8 | (p[6] & 0xf) << 16;
        d->limit = (p[6] & 0x80) ? lim << 12 | 0xfff : lim;
        d->code = p[5] & 8;
        refresh_segs(m, BX);
        return;
    }
    case 0x0100: {
        uint32_t largest = 0;
        uint32_t lin = dos_alloc(m, BX, &largest);
        if (!lin) {
            SET_BX(largest);
            fail(m, 0x0008);
            return;
        }
        int sel = desc_alloc(m, lin, (uint32_t)BX * 16 - 1);
        SET_AX(lin >> 4);
        SET_DX(sel);
        return;
    }
    case 0x0101:
        if (!(d = get_desc(m, DX)) || !dos_free(m, d->base)) {
            fail(m, 0x0009);
            return;
        }
        d->used = false;
        return;
    case 0x0200:
        SET_CX(m->rmvec[BL].sel);
        SET_DX(m->rmvec[BL].off);
        return;
    case 0x0201:
        m->rmvec[BL] = (farptr_t){ CX, DX };
        return;
    case 0x0202:
        SET_CX(SEL_CODE);
        EDX = MEM_STUBS + 4 * 0xff; /* an IRETD */
        return;
    case 0x0203:
        return;
    case 0x0204:
        SET_CX(m->pmvec[BL].sel);
        EDX = m->pmvec[BL].off;
        return;
    case 0x0205:
        m->pmvec[BL] = (farptr_t){ CX, EDX };
        return;
    case 0x0300:
        simulate_rm_int(m, BL, SEG_BASE(X86_ES) + EDI);
        return;
    case 0x0400:
        SET_AX(0x005a);
        SET_BX(0x0005); /* 32-bit host, virtual memory not used */
        SET_CX(4);
        SET_DX(0x0870);
        return;
    case 0x0500: {
        uint8_t *p = lin_ptr(m, SEG_BASE(X86_ES) + EDI, 48);
        uint32_t largest = xms_largest(m), pages = largest / 4096;
        memset(p, 0xff, 48);
        memcpy(p, &largest, 4);
        memcpy(p + 4, &pages, 4);
        memcpy(p + 8, &pages, 4);
        memcpy(p + 0x14, &pages, 4);
        return;
    }
    case 0x0501: {
        uint32_t size = (uint32_t)BX << 16 | CX;
        uint32_t at = size ? xms_place(m, size, -1) : 0;
        int i = 0;
        while (i < MAX_XMS_BLOCKS && m->xms[i].used) {
            i++;
        }
        if (!at || i == MAX_XMS_BLOCKS) {
            fail(m, 0x8013);
            return;
        }
        m->xms[i] = (memblock_t){ .base = at, .size = size, .used = true };
        memset(m->mem + at, 0, size);
        SET_BX(at >> 16);
        SET_CX(at);
        SET_SI((i + 1) >> 16);
        SET_DI(i + 1);
        return;
    }
    case 0x0502: {
        memblock_t *b = xms_find(m, (uint32_t)SI << 16 | DI);
        if (!b) {
            fail(m, 0x8023);
            return;
        }
        b->used = false;
        return;
    }
    case 0x0503: {
        uint32_t h = (uint32_t)SI << 16 | DI;
        memblock_t *b = xms_find(m, h);
        uint32_t size = (uint32_t)BX << 16 | CX;
        if (!b) {
            fail(m, 0x8023);
            return;
        }
        uint32_t at = xms_place(m, size, (int)h - 1);
        if (!at) {
            fail(m, 0x8013);
            return;
        }
        if (at != b->base) {
            memmove(m->mem + at, m->mem + b->base, b->size < size ? b->size : size);
        }
        if (size > b->size) {
            memset(m->mem + at + b->size, 0, size - b->size);
        }
        b->base = at;
        b->size = size;
        SET_BX(at >> 16);
        SET_CX(at);
        return;
    }
    case 0x0600: case 0x0601: case 0x0602: case 0x0603: case 0x0702: case 0x0703:
        return;
    case 0x0604:
        SET_BX(0);
        SET_CX(4096);
        return;
    case 0x0800: {
        /* physical mapping: identity below 1 MB; anything else has no backing here */
        uint32_t phys = (uint32_t)BX << 16 | CX;
        if (phys >= 0x100000) {
            fail(m, 0x8021);
            return;
        }
        return;
    }
    case 0x0801:
        return;
    case 0x0900: {
        bool was = m->cpu.eflags & X86_IF;
        m->cpu.eflags &= ~X86_IF;
        SET_AL(was);
        return;
    }
    case 0x0901: {
        bool was = m->cpu.eflags & X86_IF;
        m->cpu.eflags |= X86_IF;
        SET_AL(was);
        return;
    }
    case 0x0902:
        SET_AL(!!(m->cpu.eflags & X86_IF));
        return;
    case 0x0a00:
        fail(m, 0x8001); /* no vendor extensions */
        return;
    case 0x0e00:
        SET_AX(0x0045); /* coprocessor present and enabled, 80387-class */
        return;
    case 0x0e01:
        return;
    }
    mach_fault(m, "INT 31h AX=%04xh not implemented", AX);
}

void dpmi_int2f(machine_t *m)
{
    switch (AX) {
    case 0x1600: /* Windows enhanced mode: not running */
        SET_AL(0);
        return;
    case 0x1686: /* in protected mode */
        SET_AX(0);
        return;
    case 0x1500: /* MSCDEX: one CD drive, D: */
        SET_BX(m->drive_d[0] ? 1 : 0);
        SET_CX(3);
        return;
    case 0x150b:
        SET_AX(CX == 3 && m->drive_d[0] ? 0x5ad8 : 0);
        SET_BX(0xadad);
        return;
    case 0x150c:
        SET_BX(0x0215);
        return;
    case 0x150d: {
        uint8_t *p = lin_ptr(m, SEG_BASE(X86_ES) + BX, 1);
        if (p) {
            *p = 3;
        }
        return;
    }
    }
    if (AH == 0xfa || AH == 0x43 || AH == 0x16 || AH == 0x11 || AH == 0x4a) {
        return; /* debugger / XMS / other multiplex users: not installed, AL unchanged */
    }
    mach_fault(m, "INT 2Fh AX=%04xh not implemented", AX);
}
