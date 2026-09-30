#include "aot_rt.h"

const aot_entry_t *aot_lookup(uint32_t eip);

static inline void store_state(x86_cpu *c)
{
    c->r[X86_EAX] = EAX;
    c->r[X86_ECX] = ECX;
    c->r[X86_EDX] = EDX;
    c->r[X86_EBX] = EBX;
    c->r[X86_ESP] = ESP;
    c->r[X86_EBP] = EBP;
    c->r[X86_ESI] = ESI;
    c->r[X86_EDI] = EDI;
    aot_budget = aot_bud;
}

void aot_call(aot_fn fn, uint32_t entry)
{
    /* the caller's values of s0..s11: they are callee-saved for it */
    uint32_t sv[12] = { EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI, (uint32_t)aot_bud, (uint32_t)(uintptr_t)aot_m,
                        aot_lo, aot_top };
    x86_cpu *c = aot_cpu;
    EAX = c->r[X86_EAX];
    ECX = c->r[X86_ECX];
    EDX = c->r[X86_EDX];
    EBX = c->r[X86_EBX];
    ESP = c->r[X86_ESP];
    EBP = c->r[X86_EBP];
    ESI = c->r[X86_ESI];
    EDI = c->r[X86_EDI];
    aot_bud = aot_budget;
    aot_m = aot_mem;
    aot_lo = aot_fast_lo;
    aot_top = aot_fast_top;
    uint32_t efl = fn(entry, c->eflags, AOT_NO_RET);
    c = aot_cpu;
    store_state(c);
    c->eflags = efl;
    EAX = sv[0];
    ECX = sv[1];
    EDX = sv[2];
    EBX = sv[3];
    ESP = sv[4];
    EBP = sv[5];
    ESI = sv[6];
    EDI = sv[7];
    aot_bud = (int32_t)sv[8];
    aot_m = (uint8_t *)(uintptr_t)sv[9];
    aot_lo = sv[10];
    aot_top = sv[11];
    aot_cpu->eip = AOT_NO_RET;
}

uint32_t aot_icall(uint32_t target, uint32_t efl, uint32_t next)
{
    const aot_entry_t *e = aot_lookup(target);
    if (!e || !e->start) {
        store_state(aot_cpu);   /* the caller wrote back nothing */
        aot_cpu->eflags = efl;
        aot_exit(target, AOT_EXIT_JUMP);
    }
    return e->fn(0, efl, next);
}
