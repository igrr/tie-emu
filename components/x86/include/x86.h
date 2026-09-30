/* x86-32 user-mode interpreter.
 *
 * Models what a flat-model DOS/4GW program executes: 32-bit protected mode,
 * paging off, segment bases taken from the environment, no privilege checks.
 * Everything outside the CPU (ports, software interrupts, descriptor lookups)
 * goes through x86_env. Guest linear address A is host address mem + A.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <setjmp.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { X86_EAX, X86_ECX, X86_EDX, X86_EBX, X86_ESP, X86_EBP, X86_ESI, X86_EDI };
enum { X86_ES, X86_CS, X86_SS, X86_DS, X86_FS, X86_GS };

#define X86_CF 0x0001u
#define X86_PF 0x0004u
#define X86_AF 0x0010u
#define X86_ZF 0x0040u
#define X86_SF 0x0080u
#define X86_TF 0x0100u
#define X86_IF 0x0200u
#define X86_DF 0x0400u
#define X86_OF 0x0800u
#define X86_AC 0x40000u
#define X86_ID 0x200000u
#define X86_ARITH_FLAGS (X86_CF | X86_PF | X86_AF | X86_ZF | X86_SF | X86_OF)

/* Exceptions raised by the core */
enum {
    X86_EXC_DE = 0,     /* divide error */
    X86_EXC_UD = 6,     /* invalid or unimplemented opcode */
    X86_EXC_NM = 7,
    X86_EXC_GP = 13,    /* bad selector, privileged instruction the env refused */
    X86_EXC_PF = 14,    /* access outside guest memory */
    X86_EXC_MF = 16,
};

typedef enum {
    X86_STOP_BUDGET,    /* executed the requested number of instructions */
    X86_STOP_HLT,
    X86_STOP_EXC,       /* exception; cpu->exc_* describe it, eip is the faulting instruction */
    X86_STOP_REQUEST,   /* the environment called x86_request_stop() */
    X86_STOP_IRQ,       /* intr_pending is set and interrupts are enabled */
    X86_STOP_ENTRY,     /* about to execute an instruction marked in the entry map */
} x86_stop_t;

typedef struct x86_cpu x86_cpu;

typedef struct {
    uint32_t (*in)(x86_cpu *c, uint16_t port, int size);
    void (*out)(x86_cpu *c, uint16_t port, uint32_t val, int size);
    /* INT n / INT3 / INTO. eip already points past the instruction. */
    void (*intr)(x86_cpu *c, uint8_t vec);
    /* Segment register load; false raises #GP. */
    bool (*load_seg)(x86_cpu *c, uint16_t sel, uint32_t *base, uint32_t *limit);
    /* Segment limit for LSL; false clears ZF. May be NULL. */
    bool (*seg_limit)(x86_cpu *c, uint16_t sel, uint32_t *limit);
    /* Emulator call: the reserved encoding 0F FF ib (UD0 with an immediate) that the machine places in
     * guest memory to reach its own services. eip points past the instruction. May be NULL (#UD). */
    void (*hypercall)(x86_cpu *c, uint8_t n);
    /* The first instruction count after icount at which a byte IN of port may read something
     * else in the bits of mask (translated poll loops skip the reads before it); icount + 1 when
     * the device cannot tell. May be NULL. */
    uint64_t (*in_until)(x86_cpu *c, uint16_t port, uint8_t mask);
} x86_env;

typedef struct {
    double st[8];
    uint8_t top;
    uint8_t tag;        /* bit i set: physical register i is empty */
    uint16_t cw;
    uint16_t sw;        /* without TOP; x86_fpu_sw() merges it */
} x86_fpu;

struct x86_cpu {
    uint32_t r[8];
    uint32_t eip;
    uint32_t eflags;
    uint16_t sel[6];
    uint32_t base[6];
    uint32_t cr0;
    x86_fpu fpu;

    uint8_t *mem;
    uint32_t mem_size;
    /* A banked video window: when set, linear X86_WIN_BASE..+X86_WIN_SIZE is this memory
     * instead of mem (an access that crosses the window's edge sees mem) */
    uint8_t *win;
    uint32_t fast_lo, fast_size;    /* [fast_lo, fast_lo + fast_size) is mem, whatever win is */
    const x86_env *env;
    void *user;

    uint64_t icount;

    /* Set by the environment while an interrupt controller has something to deliver; the core
     * stops at the first instruction boundary where IF allows it (after the STI shadow). */
    bool intr_pending;

    /* Mixed mode with translated code (components/aot): stop before an address whose bit is set
     * in the entry map, except at the first instruction of a run and after STI. The map has a
     * bitmap per 4 KB page: bit o & 4095 of entry_pages[o >> 12], o = a - entry_lo (NULL: none
     * in that page). */
    const uint8_t *const *entry_pages;
    uint32_t entry_lo, entry_npages;

    int exc;             /* valid after X86_STOP_EXC */
    uint32_t exc_addr;   /* faulting linear address for #PF */

    /* private */
    uint32_t insn_eip;
    bool stop_req;
    bool sti_shadow;
    jmp_buf *jb;
};

void x86_init(x86_cpu *c, uint8_t *mem, uint32_t mem_size, const x86_env *env, void *user);
x86_stop_t x86_run(x86_cpu *c, uint64_t max_insns);
void x86_request_stop(x86_cpu *c);

/* Helpers for the environment (interrupt delivery, HLE) */
bool x86_set_seg(x86_cpu *c, int sreg, uint16_t sel);
uint32_t x86_fpu_sw(const x86_cpu *c);

/* Deliver an interrupt through a 32-bit gate: push eflags, cs, eip; clear IF/TF */
void x86_interrupt(x86_cpu *c, uint16_t cs, uint32_t eip);

#define X86_WIN_BASE 0xa0000u
#define X86_WIN_SIZE 0x10000u

/* Sets or removes (NULL) the banked video window */
void x86_set_window(x86_cpu *c, uint8_t *win);

static inline uint8_t *x86_ptr_low(x86_cpu *c, uint32_t lin, uint32_t len)
{
    if (c->win && lin - X86_WIN_BASE < X86_WIN_SIZE) {
        return X86_WIN_SIZE - (lin - X86_WIN_BASE) >= len ? c->win + (lin - X86_WIN_BASE) : 0;
    }
    return (lin < c->mem_size && c->mem_size - lin >= len) ? c->mem + lin : 0;
}

/* Guest linear [lin, lin + len) in host memory, or NULL when outside it */
static inline uint8_t *x86_ptr(x86_cpu *c, uint32_t lin, uint32_t len)
{
    if (lin - c->fast_lo < c->fast_size && c->mem_size - lin >= len) {
        return c->mem + lin;
    }
    return x86_ptr_low(c, lin, len);
}

#ifdef __cplusplus
}
#endif
