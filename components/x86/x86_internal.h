/* Shared between the integer core and the x87 unit */
#pragma once

#include <string.h>
#include "x86.h"

typedef struct {
    uint8_t opsz;       /* 2 or 4 */
    uint8_t adsz;       /* 2 or 4 */
    int8_t seg;         /* segment override, -1 if none */
    uint8_t rep;        /* 0, 0xF2, 0xF3 */
    uint8_t mod, reg, rm;
    uint32_t ea;        /* offset of a memory operand */
    uint32_t lin;       /* its linear address */
} x86_insn;

__attribute__((noreturn)) void x86_raise(x86_cpu *c, int exc, uint32_t addr);

static inline uint8_t *x86_mp(x86_cpu *c, uint32_t lin, uint32_t n)
{
    uint8_t *p = x86_ptr(c, lin, n);
    if (!p) {
        x86_raise(c, X86_EXC_PF, lin);
    }
    return p;
}

static inline uint32_t x86_rd(x86_cpu *c, uint32_t lin, int sz)
{
    uint8_t *p = x86_mp(c, lin, sz);
    if (sz == 1) {
        return *p;
    }
    if (sz == 2) {
        uint16_t v;
        memcpy(&v, p, 2);
        return v;
    }
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

static inline void x86_wr(x86_cpu *c, uint32_t lin, uint32_t v, int sz)
{
    uint8_t *p = x86_mp(c, lin, sz);
    if (sz == 1) {
        *p = (uint8_t)v;
    } else if (sz == 2) {
        uint16_t w = (uint16_t)v;
        memcpy(p, &w, 2);
    } else {
        memcpy(p, &v, 4);
    }
}

void x86_fpu_reset(x86_fpu *f);
/* Executes D8..DF; modrm already decoded into d (d->lin valid when mod != 3) */
void x86_fpu_exec(x86_cpu *c, x86_insn *d, uint8_t op);
