/* Mixed-mode execution: translated code (aot/aotgen.py) with the interpreter as fallback.
 *
 * aot_run is a drop-in for x86_run. It stops exactly where the interpreter would: translated
 * blocks never run past the end of the slice, port I/O leaves right after the access when the
 * device asks the CPU to stop, and the rest of what can stop the CPU or change what is
 * deliverable (INT, CLI/STI, HLT, most segment loads) is left to the interpreter.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "x86.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Why translated code handed control back to the dispatcher */
enum {
    AOT_EXIT_JUMP,      /* continue at eip: an indirect branch, an untranslated target, a mismatched return */
    AOT_EXIT_BUDGET,    /* the slice ends inside the next block: the interpreter runs the rest */
    AOT_EXIT_PUNT,      /* the instruction at eip is left to the interpreter */
    AOT_EXIT_FAULT,     /* guest memory access out of range */
    AOT_EXIT_STACK,     /* native stack too deep, or ESP not where translated code runs (at a
                         * function start): restart from the dispatcher */
    AOT_EXIT_STOP,      /* a port access asked the CPU to stop (x86_request_stop): eip is after it */
};

/* A ret to an address other than the one its caller pushed: the translated code leaves to the
 * dispatcher, as when the dispatcher's own call returns (aot_enter's -1) */
#define AOT_EXIT_RET (-1)

/* A translated function: entry 0 is a call to its start, entry k > 0 its k-th entry (the
 * dispatcher's way in, aot_entry_t.idx); efl is EFLAGS, and next the return address the caller
 * pushed. It returns EFLAGS when its ret goes to next, and leaves with AOT_EXIT_RET otherwise. */
typedef uint32_t (*aot_fn)(uint32_t entry, uint32_t efl, uint32_t next);

typedef struct {
    uint32_t eip;
    aot_fn fn;          /* the translated function this block is an entry of */
    uint16_t idx;       /* the entry number in fn */
    uint16_t need;      /* instructions the slice must have left to enter here */
    uint8_t start;      /* 1: eip is the function's start (a call target) */
    uint8_t mod;        /* the module (aot_modules) it was translated from */
} aot_entry_t;

/* Guest code the translation was made from: the executable, and modules the program loads */
typedef struct {
    const char *name;
    uint32_t lo, hi;                /* the entries are in [lo, hi) */
    /* constant addresses in [direct_lo, direct_hi) are accessed without a check: the module
     * runs only while that range is in the CPU's fast range */
    uint32_t direct_lo, direct_hi;
    uint32_t hash;                  /* FNV-1a of the bytes in ranges, as translated */
    const uint32_t (*ranges)[2];    /* [start, end) */
    uint32_t nranges;
    /* 0: in memory from the start, checked by aot_init; 1: loaded by the program, checked the
     * first time it is entered after aot_init or aot_invalidate */
    uint32_t loaded;
} aot_module_t;

/* The generated table (aot_table.c) */
extern const aot_entry_t aot_entries[];
extern const uint32_t aot_num_entries;
extern const aot_module_t aot_modules[];
extern const uint32_t aot_num_modules;

/* Checks that guest memory holds the translated executable and sets up the entry map; false when
 * the loaded program is not the one translated */
bool aot_init(const uint8_t *mem, uint32_t mem_size);
/* The program may have loaded code (it opened a file): loaded modules are checked again before
 * they run */
void aot_invalidate(void);
x86_stop_t aot_run(x86_cpu *c, uint64_t max_insns);
/* Runs translated code from entry (0: a call to the function's start) for at most max_insns
 * instructions; returns the AOT_EXIT_* reason it left for, or -1 when it returned (eip = the
 * return address) */
int aot_enter(x86_cpu *c, aot_fn fn, uint32_t entry, int32_t max_insns);

#ifdef __cplusplus
}
#endif
