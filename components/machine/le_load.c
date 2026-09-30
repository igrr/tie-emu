/* Linear Executable loader: objects at link address + delta, fixups applied.
 * Handles what Watcom's wlink emits for DOS/4GW: internal-reference fixups of any
 * source type that makes sense in a flat model (32-bit offset, 16:32 pointer, selector). */
#include <stdlib.h>
#include <string.h>
#include "machine_priv.h"

static uint32_t rd32(const uint8_t *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t rd16(const uint8_t *p)
{
    return p[0] | p[1] << 8;
}

int le_load(machine_t *m, const char *host_path, uint32_t delta, uint32_t *eip, uint32_t *esp)
{
    FILE *f = fopen(host_path, "rb");
    if (!f) {
        mach_fault(m, "%s: cannot open", host_path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(len);
    if (!d || fread(d, 1, len, f) != (size_t)len) {
        fclose(f);
        free(d);
        mach_fault(m, "%s: read failed", host_path);
        return -1;
    }
    fclose(f);

    int ret = -1;
    uint32_t base = len > 0x40 ? rd32(d + 0x3c) : 0;
    if (base + 0xb0 > (uint32_t)len || d[base] != 'L' || d[base + 1] != 'E') {
        /* bound to a full DOS/4GW stub: the LE header follows the stub image */
        base = 0;
        for (long i = 0x40; i + 0xb0 < len; i++) {
            if (d[i] == 'L' && d[i + 1] == 'E' && d[i + 2] == 0 && d[i + 3] == 0 && rd32(d + i + 0x44) <= 16) {
                base = i;
                break;
            }
        }
        if (!base) {
            mach_fault(m, "%s: not an LE executable", host_path);
            goto out;
        }
    }
    const uint8_t *h = d + base;
    uint32_t num_pages = rd32(h + 0x14), page_size = rd32(h + 0x28), last_page = rd32(h + 0x2c);
    uint32_t obj_tab = rd32(h + 0x40), num_objs = rd32(h + 0x44), obj_page_tab = rd32(h + 0x48);
    uint32_t fixup_page_tab = rd32(h + 0x68), fixup_rec_tab = rd32(h + 0x6c), data_pages = rd32(h + 0x80);
    uint32_t objbase[17] = { 0 };

    if (num_objs > 16) {
        mach_fault(m, "LE: %" PRIu32 " objects", num_objs);
        goto out;
    }
    for (uint32_t o = 0; o < num_objs; o++) {
        const uint8_t *ot = h + obj_tab + 24 * o;
        uint32_t vsize = rd32(ot), reloc = rd32(ot + 4), pidx = rd32(ot + 12), npages = rd32(ot + 16);
        uint32_t at = reloc + delta;
        objbase[o + 1] = at;
        if (at + vsize > m->mem_size || at + npages * page_size > m->mem_size) {
            mach_fault(m, "LE: object %" PRIu32 " does not fit at %08" PRIx32 "", o + 1, at);
            goto out;
        }
        memset(m->mem + at, 0, vsize);
        for (uint32_t k = 0; k < npages; k++) {
            uint32_t p = pidx - 1 + k;
            const uint8_t *e = h + obj_page_tab + 4 * p;
            uint32_t num = (uint32_t)e[0] << 16 | e[1] << 8 | e[2];
            uint32_t off = data_pages + (num - 1) * page_size;
            uint32_t size = p == num_pages - 1 ? last_page : page_size;
            if (off + size > (uint32_t)len) {
                mach_fault(m, "LE: page %" PRIu32 " outside the file", p);
                goto out;
            }
            memcpy(m->mem + at + k * page_size, d + off, size);
        }
    }

    /* fixups, per page */
    for (uint32_t o = 0; o < num_objs; o++) {
        const uint8_t *ot = h + obj_tab + 24 * o;
        uint32_t pidx = rd32(ot + 12), npages = rd32(ot + 16);
        for (uint32_t k = 0; k < npages; k++) {
            uint32_t p = pidx - 1 + k;
            const uint8_t *r = h + fixup_rec_tab + rd32(h + fixup_page_tab + 4 * p);
            const uint8_t *rend = h + fixup_rec_tab + rd32(h + fixup_page_tab + 4 * (p + 1));
            uint32_t page_lin = objbase[o + 1] + k * page_size;
            while (r < rend) {
                uint8_t src = r[0], flg = r[1];
                r += 2;
                int cnt = 1;
                int16_t single = 0;
                if (src & 0x20) {
                    cnt = *r++;
                } else {
                    single = (int16_t)rd16(r);
                    r += 2;
                }
                if ((flg & 3) != 0) {
                    mach_fault(m, "LE: import fixups are not supported");
                    goto out;
                }
                uint32_t tobj;
                if (flg & 0x40) {
                    tobj = rd16(r);
                    r += 2;
                } else {
                    tobj = *r++;
                }
                uint32_t toff = 0;
                if ((src & 0x0f) != 2) {
                    if (flg & 0x10) {
                        toff = rd32(r);
                        r += 4;
                    } else {
                        toff = rd16(r);
                        r += 2;
                    }
                }
                if (flg & 0x04) {
                    r += (flg & 0x20) ? 4 : 2;
                }
                uint32_t target = (tobj >= 1 && tobj <= num_objs ? objbase[tobj] : 0) + toff;
                for (int i = 0; i < cnt; i++) {
                    int16_t so = (src & 0x20) ? (int16_t)rd16(r + 2 * i) : single;
                    uint32_t site = page_lin + so;
                    uint8_t *sp;
                    switch (src & 0x0f) {
                    case 7: /* 32-bit offset */
                        if ((sp = lin_ptr(m, site, 4))) {
                            memcpy(sp, &target, 4);
                        }
                        break;
                    case 8: { /* 32-bit self-relative */
                        uint32_t rel = target - (site + 4);
                        if ((sp = lin_ptr(m, site, 4))) {
                            memcpy(sp, &rel, 4);
                        }
                        break;
                    }
                    case 2: { /* selector */
                        uint16_t sel = SEL_DATA;
                        if (tobj >= 1 && tobj <= num_objs && (rd32(h + obj_tab + 24 * (tobj - 1) + 8) & 4)) {
                            sel = SEL_CODE;
                        }
                        if ((sp = lin_ptr(m, site, 2))) {
                            memcpy(sp, &sel, 2);
                        }
                        break;
                    }
                    default:
                        mach_fault(m, "LE: fixup source type %d", src & 0x0f);
                        goto out;
                    }
                }
                if (src & 0x20) {
                    r += 2 * cnt;
                }
            }
        }
    }

    uint32_t eip_obj = rd32(h + 0x18), esp_obj = rd32(h + 0x20);
    *eip = objbase[eip_obj] + rd32(h + 0x1c);
    *esp = objbase[esp_obj] + rd32(h + 0x24);

    /* DPMI memory goes above the image */
    uint32_t top = 0;
    for (uint32_t o = 0; o < num_objs; o++) {
        uint32_t end = objbase[o + 1] + rd32(h + obj_tab + 24 * o);
        if (end > top) {
            top = end;
        }
    }
    m->heap_lo = (top + 0xffff) & ~0xffffu;
    m->heap_hi = m->mem_size;
    ret = 0;
out:
    free(d);
    return ret;
}
