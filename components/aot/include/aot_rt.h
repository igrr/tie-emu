/* Runtime interface of the translated code (aot/aotgen.py writes C against it).
 *
 * The architectural guest state is the interpreter's x86_cpu, reached through aot_cpu, so control
 * can pass between translated code and the interpreter at any block boundary. Inside translated
 * code the registers and the slice budget are kept in host registers (see below), and EFLAGS in a
 * local of each function. Translated code writes them back to aot_cpu (AOT_SAVE) before it calls
 * anything that uses them (the dispatcher, the string helper) and before every exit, and reads
 * them back (AOT_LOAD) after. Translated functions pass EFLAGS to each other in an argument and
 * return it, with the return address the caller pushed as another argument (aot_fn). Guest
 * linear address A is aot_mem + A. The flag helpers compute exactly what the interpreter
 * (components/x86/x86.c) computes, undefined flags included, so control can pass between the two
 * at any point.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "aot.h"

#ifdef __cplusplus
extern "C" {
#endif


extern x86_cpu *aot_cpu;
extern uint8_t *aot_mem;
extern uint32_t aot_mem_size;
extern uint32_t aot_fast_lo, aot_fast_top;  /* a - aot_fast_lo below aot_fast_top: a 4-byte access at a is in aot_mem */
extern int32_t aot_budget;      /* instructions left in the slice */
extern int32_t aot_budget0;     /* ... when the dispatcher entered translated code */
extern uint32_t aot_esp_lo, aot_esp_size;   /* ESP - aot_esp_lo below aot_esp_size: see AOT_ESP_OK */
extern char *aot_stack_limit;
extern uint8_t *aot_win;       /* aot_cpu->win (aot_p_slowN) */
extern const uint8_t aot_parity[256];

__attribute__((noreturn)) void aot_exit(uint32_t eip, int reason);
__attribute__((noreturn)) void aot_fault(uint32_t addr);
/* Indirect call from translated code (the state saved as for a call): runs the translated
 * function at target, or leaves to the dispatcher */
uint32_t aot_icall(uint32_t target, uint32_t efl, uint32_t next);
/* INT vec, with the guest state saved and `ahead` instructions of the block not executed (this
 * one included), through the machine's handler: returns when the handler leaves the CPU where
 * translated code can go on at next; otherwise leaves to the dispatcher (aot.c) */
void aot_int(uint32_t vec, uint32_t next, int32_t ahead);
/* Runs fn(entry) from code that is not translated, with the guest state from aot_cpu and
 * aot_budget, and writes it back there after, EFLAGS included (aot_glue.c). The function's ret
 * leaves to the dispatcher (AOT_EXIT_RET): no return address is the dispatcher's. */
void aot_call(aot_fn fn, uint32_t entry);
#define AOT_NO_RET 0xffffffffu

#define AOT_INLINE static inline __attribute__((always_inline))
#define AOT_LIKELY(x) __builtin_expect(!!(x), 1)
#define AOT_UNLIKELY(x) __builtin_expect(!!(x), 0)

/* Where translated code keeps the guest state: EAX..EDI, the budget, the memory base and the
 * fast range are global register variables in s1..s11 and s0, so translated functions call each
 * other with the state in registers. These are callee-saved registers: C code that translated
 * code calls preserves them, and code that is not translated enters translated code only through
 * aot_call(), which saves its own values of them and loads the guest state (the dispatcher's
 * call, and aot_icall()). EFLAGS is a local of each function (EFL), from its argument aot_efl.
 *
 * AOT_STATE declares the function's part; AOT_SAVE/AOT_LOAD write back and read everything
 * (exits, the string helper). aot_m, aot_lo and aot_top are aot_mem, aot_fast_lo and
 * aot_fast_top, which stay the same while translated code runs (aot_enter sets them). The
 * runtime (aot.c) defines AOT_RUNTIME and uses none of this. */
#ifndef AOT_RUNTIME
register uint32_t EAX __asm__("s1");
register uint32_t ECX __asm__("s2");
register uint32_t EDX __asm__("s3");
register uint32_t EBX __asm__("s4");
register uint32_t ESP __asm__("s5");
register uint32_t EBP __asm__("s6");
register uint32_t ESI __asm__("s7");
register uint32_t EDI __asm__("s8");
register int32_t aot_bud __asm__("s9");
register uint8_t *aot_m __asm__("s10");
register uint32_t aot_lo __asm__("s11");
register uint32_t aot_top __asm__("s0");
#endif
#define AOT_STATE \
    uint32_t EFL = aot_efl; \
    uint32_t aot_x_eip; \
    int aot_x_reason
#define AOT_SAVE() do { \
        x86_cpu *c_ = aot_cpu; \
        c_->r[X86_EAX] = EAX; c_->r[X86_ECX] = ECX; c_->r[X86_EDX] = EDX; c_->r[X86_EBX] = EBX; \
        c_->r[X86_ESP] = ESP; c_->r[X86_EBP] = EBP; c_->r[X86_ESI] = ESI; c_->r[X86_EDI] = EDI; \
        c_->eflags = EFL; aot_budget = aot_bud; \
    } while (0)
#define AOT_LOAD() do { \
        x86_cpu *c_ = aot_cpu; \
        EAX = c_->r[X86_EAX]; ECX = c_->r[X86_ECX]; EDX = c_->r[X86_EDX]; EBX = c_->r[X86_EBX]; \
        ESP = c_->r[X86_ESP]; EBP = c_->r[X86_EBP]; ESI = c_->r[X86_ESI]; EDI = c_->r[X86_EDI]; \
        EFL = c_->eflags; aot_bud = aot_budget; \
    } while (0)
/* Exits share one copy of AOT_SAVE per function, at AOT_EXIT_PATH (the end of the function) */
#define AOT_EXIT(eip, reason) do { aot_x_eip = (eip); aot_x_reason = (reason); goto aot_x; } while (0)
#define AOT_EXIT_PATH aot_x: AOT_SAVE(); aot_exit(aot_x_eip, aot_x_reason)

/* Calls: to the translated function fn and to the guest address t, pushed return address next.
 * The callee checks where its ret goes (AOT_RET), so a call that returns returned to next. It
 * leaves to the dispatcher with less than `need` instructions left in the slice (the budget check
 * of the block at next, when the call is the only way there: the dispatcher then leaves that
 * block to the interpreter) */
#define AOT_CALL(fn, next, need) do { \
        EFL = fn(0, EFL, (next)); \
        if ((need) && AOT_UNLIKELY(aot_bud < (need))) AOT_EXIT((next), AOT_EXIT_JUMP); \
    } while (0)
#define AOT_ICALL(t, next, need) do { \
        EFL = aot_icall((t), EFL, (next)); \
        if ((need) && AOT_UNLIKELY(aot_bud < (need))) AOT_EXIT((next), AOT_EXIT_JUMP); \
    } while (0)
/* A jump to the translated function fn that leaves this one: it returns where this one would */
#define AOT_TAIL(fn) return fn(0, EFL, aot_next)
/* ... to the guest address t (a ret to an address the function pushed) */
#define AOT_ITAIL(t) return aot_icall((t), EFL, aot_next)
/* INT n (aot_int) */
#define AOT_INT(vec, next, ahead) do { AOT_SAVE(); aot_int((vec), (next), (ahead)); AOT_LOAD(); } while (0)
/* ret to the guest address ret: back to the caller when it is the address the caller pushed and
 * AOT_RET_ROOM instructions are left in the slice, enough for the caller to reach its next budget
 * check (aot/lower.py, Func.budget_checks) */
#define AOT_RET_ROOM 64
#define AOT_RET(ret) do { \
        uint32_t r_ = (ret); \
        if (AOT_UNLIKELY(r_ != aot_next || aot_bud < AOT_RET_ROOM)) AOT_EXIT(r_, AOT_EXIT_RET); \
        return EFL; \
    } while (0)

#define AOT_ARITH (X86_CF | X86_PF | X86_AF | X86_ZF | X86_SF | X86_OF)

/* Slice accounting at the top of a block of n instructions that checks the budget: it leaves
 * unless `need` instructions are left, enough to reach the next check (aot/lower.py,
 * Func.budget_checks); blocks between checks only count. The interpreter runs the rest of the
 * slice after an early exit, so the slice still ends at the same instruction. */
#define AOT_BLOCK(need, n, eip) do { \
        AOT_NEED((need), (eip)); \
        aot_bud -= (n); \
    } while (0)
#define AOT_NEED(need, eip) do { \
        if (AOT_UNLIKELY(aot_bud < (need))) AOT_EXIT((eip), AOT_EXIT_BUDGET); \
    } while (0)
/* Leave at eip with `left` instructions of the current block not executed */
#define AOT_LEAVE(eip, left, reason) do { aot_bud += (left); AOT_EXIT((eip), (reason)); } while (0)
/* At the start of a function called from translated code: room on the native stack, and ESP
 * where translated code runs (AOT_ESP_OK); the dispatcher restarts from eip when either fails */
#define AOT_FUNC(eip) do { \
        register uint32_t bad_ __asm__("t1"); \
        __asm__("call t0, aot_func_check" : "=r"(bad_) : : "t0", "t2"); \
        if (AOT_UNLIKELY(bad_)) AOT_EXIT((eip), AOT_EXIT_STACK); \
    } while (0)

/* The guest stack. Translated code accesses [ESP + d] for |d| < AOT_STACK_MARGIN, and pushes and
 * pops, without a check: while ESP is at least 2 * AOT_STACK_MARGIN inside the fast range, those
 * are in aot_mem. The dispatcher enters translated code only then, and translated code checks it
 * at function starts and after instructions that set ESP other than by a push, pop, call, ret or
 * a small constant add or sub (before `leave`, which pops from the new ESP; aot/lower.py), and
 * leaves when it fails. What is assumed is that between two checks ESP moves less than
 * AOT_STACK_MARGIN: that no stack runs away inside a loop. */
#define AOT_STACK_MARGIN 4096u
#define AOT_ESP_OK(esp) ((esp) - aot_esp_lo < aot_esp_size)

/* IN, with `ahead` instructions of the block not executed yet (this one included) and `bud` the
 * budget left: the device sees the instruction count the interpreter would show it, and may ask
 * the CPU to stop after the instruction (the caller then leaves with AOT_EXIT_STOP) */
static inline uint32_t aot_in(uint32_t port, int size, int32_t bud, int32_t ahead)
{
    x86_cpu *c = aot_cpu;
    uint64_t ic = c->icount;
    c->icount = ic + (uint32_t)(aot_budget0 - bud - ahead);
    uint32_t v = c->env->in(c, (uint16_t)port, size);
    c->icount = ic;
    return v;
}

/* IN AL at the top of a poll loop of len instructions (aot/lower.py, Emitter.poll_loop) that goes
 * on while (AL & mask) != 0 is nz, and reads nothing else: where the device tells when those bits
 * can change next (x86_env.in_until), it skips the iterations that would read the same bits, as
 * far as the budget checks at the loop's top (need) let the loop go on. `rest` instructions of
 * the iteration are counted after this block. Returns AL as the last of them reads it, and in the
 * high word the instructions skipped (the caller takes them off its budget). */
uint64_t aot_in_poll(uint32_t port, int32_t bud, int32_t ahead, uint32_t mask, int32_t nz, int32_t len,
                     int32_t rest, int32_t need);

/* OUT, as aot_in */
static inline void aot_out(uint32_t port, uint32_t v, int size, int32_t bud, int32_t ahead)
{
    x86_cpu *c = aot_cpu;
    uint64_t ic = c->icount;
    c->icount = ic + (uint32_t)(aot_budget0 - bud - ahead);
    c->env->out(c, (uint16_t)port, v, size);
    c->icount = ic;
}

/* ---- memory ---- */

/* The banked video window and what is below it, the last bytes of memory, and faults. A fault
 * leaves to the dispatcher without writing back the locals: the registers are those of the last
 * AOT_SAVE (a fault in translated code stops the machine) */
uint8_t *aot_p_low(uint32_t a, uint32_t n);

#ifndef AOT_RUNTIME
/* A checked access computes the host pointer m + a in t1; outside the fast range aot_p_slow1/2/4
 * replace it with aot_p_low(a, n), back to t0 (a return address register, so the return is
 * predicted), with every other integer register preserved. A C call from each access would make
 * the compiler save its temporaries around it. The check and the call are inline assembly: GCC
 * puts an unlikely path at the end of the function, which in a large function is out of branch
 * range, and every check then branched around a jump to it. Each file of translated code has its
 * own copy of the slow paths (AOT_SLOW_PATHS, once at file scope), near enough for the linker to
 * make the calls single jal instructions. */
#define AOT_P(m, lo, top, a, n) ({ \
        register uint8_t *p_ __asm__("t1"); \
        __asm__("sub t2, %1, %2\n\tadd t1, %3, %1\n\tbltu t2, %4, 1f\n\tcall t0, aot_p_slow" #n "\n1:" \
                : "=&r"(p_) : "r"(a), "r"(lo), "r"(m), "r"(top) : "t0", "t2", \
                "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "ft8", "ft9", "ft10", "ft11", \
                "fa0", "fa1", "fa2", "fa3", "fa4", "fa5", "fa6", "fa7"); \
        p_; \
    })
/* aot_p_slowN: t1 = aot_p_low(t1 - s10, n), back to t0, the other integer registers as they were.
 * What the game reaches there most, the banked video window and the memory below it (a fast range
 * that does not start at 0, s11, means a window: x86_set_window), it finds without the call.
 * aot_func_check: t1 = 0 when AOT_FUNC's checks pass (ESP is s5), back to t0, t2 lost. */
#define AOT_SLOW_PATHS __asm__( \
    "    .pushsection .text.aot_p_slow,\"ax\",@progbits\n" \
    "aot_p_slow1:\n" \
    "    addi sp, sp, -64\n" \
    "    sw a1, 4(sp)\n" \
    "    li a1, 1\n" \
    "    j 1f\n" \
    "aot_p_slow2:\n" \
    "    addi sp, sp, -64\n" \
    "    sw a1, 4(sp)\n" \
    "    li a1, 2\n" \
    "    j 1f\n" \
    "aot_p_slow4:\n" \
    "    addi sp, sp, -64\n" \
    "    sw a1, 4(sp)\n" \
    "    li a1, 4\n" \
    "1:  sw a0, 8(sp)\n" \
    "    beqz s11, 3f\n" \
    "    sub t2, t1, s10\n" \
    "    lui a0, 0xa0\n"                /* X86_WIN_BASE */ \
    "    bltu t2, a0, 4f\n" \
    "    sub t2, t2, a0\n" \
    "    lui a0, 0x10\n"                /* X86_WIN_SIZE */ \
    "    sub a0, a0, a1\n" \
    "    bltu a0, t2, 2f\n" \
    "    lui a0, %hi(aot_win)\n" \
    "    lw a0, %lo(aot_win)(a0)\n" \
    "    add t1, a0, t2\n" \
    "4:  lw a0, 8(sp)\n" \
    "    lw a1, 4(sp)\n" \
    "    addi sp, sp, 64\n" \
    "    jr t0\n" \
    "2:  lw a0, 8(sp)\n" \
    "3:  sw ra, 0(sp)\n" \
    "    sw a2, 12(sp)\n" \
    "    sw a3, 16(sp)\n" \
    "    sw a4, 20(sp)\n" \
    "    sw a5, 24(sp)\n" \
    "    sw a6, 28(sp)\n" \
    "    sw a7, 32(sp)\n" \
    "    sw t0, 36(sp)\n" \
    "    sw t2, 40(sp)\n" \
    "    sw t3, 44(sp)\n" \
    "    sw t4, 48(sp)\n" \
    "    sw t5, 52(sp)\n" \
    "    sw t6, 56(sp)\n" \
    "    sub a0, t1, s10\n" \
    "    call aot_p_low\n" \
    "    mv t1, a0\n" \
    "    lw ra, 0(sp)\n" \
    "    lw a1, 4(sp)\n" \
    "    lw a0, 8(sp)\n" \
    "    lw a2, 12(sp)\n" \
    "    lw a3, 16(sp)\n" \
    "    lw a4, 20(sp)\n" \
    "    lw a5, 24(sp)\n" \
    "    lw a6, 28(sp)\n" \
    "    lw a7, 32(sp)\n" \
    "    lw t0, 36(sp)\n" \
    "    lw t2, 40(sp)\n" \
    "    lw t3, 44(sp)\n" \
    "    lw t4, 48(sp)\n" \
    "    lw t5, 52(sp)\n" \
    "    lw t6, 56(sp)\n" \
    "    addi sp, sp, 64\n" \
    "    jr t0\n" \
    "aot_func_check:\n" \
    "    lui t1, %hi(aot_stack_limit)\n" \
    "    lw t1, %lo(aot_stack_limit)(t1)\n" \
    "    bltu sp, t1, 1f\n" \
    "    lui t1, %hi(aot_esp_lo)\n" \
    "    lw t1, %lo(aot_esp_lo)(t1)\n" \
    "    lui t2, %hi(aot_esp_size)\n" \
    "    lw t2, %lo(aot_esp_size)(t2)\n" \
    "    sub t1, s5, t1\n" \
    "    sltu t1, t1, t2\n" \
    "    xori t1, t1, 1\n" \
    "    jr t0\n" \
    "1:  li t1, 1\n" \
    "    jr t0\n" \
    "    .popsection\n");
#else
#define AOT_P(m, lo, top, a, n) (AOT_UNLIKELY((a) - (lo) >= (top)) ? aot_p_low((a), (n)) : (m) + (a))
#endif

AOT_INLINE uint32_t aot_rd8(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a) { return *AOT_P(m, lo, top, a, 1); }
AOT_INLINE uint32_t aot_rd16(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a)
{
    uint16_t v;
    memcpy(&v, AOT_P(m, lo, top, a, 2), 2);
    return v;
}
AOT_INLINE uint32_t aot_rd32(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a)
{
    uint32_t v;
    memcpy(&v, AOT_P(m, lo, top, a, 4), 4);
    return v;
}
AOT_INLINE void aot_wr8(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a, uint32_t v) { *AOT_P(m, lo, top, a, 1) = (uint8_t)v; }
AOT_INLINE void aot_wr16(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a, uint32_t v)
{
    uint16_t w = (uint16_t)v;
    memcpy(AOT_P(m, lo, top, a, 2), &w, 2);
}
AOT_INLINE void aot_wr32(uint8_t *m, uint32_t lo, uint32_t top, uint32_t a, uint32_t v)
{
    memcpy(AOT_P(m, lo, top, a, 4), &v, 4);
}

/* In translated functions, through the state's copies */
#define RD8(a) aot_rd8(aot_m, aot_lo, aot_top, (a))
#define RD16(a) aot_rd16(aot_m, aot_lo, aot_top, (a))
#define RD32(a) aot_rd32(aot_m, aot_lo, aot_top, (a))
#define WR8(a, v) aot_wr8(aot_m, aot_lo, aot_top, (a), (v))
#define WR16(a, v) aot_wr16(aot_m, aot_lo, aot_top, (a), (v))
#define WR32(a, v) aot_wr32(aot_m, aot_lo, aot_top, (a), (v))
/* Without a check: constant addresses in the module's direct range, and the stack (AOT_ESP_OK) */
#define RDD8(a) ((uint32_t)aot_m[(a)])
#define RDD16(a) ({ uint16_t v_; memcpy(&v_, aot_m + (a), 2); (uint32_t)v_; })
#define RDD32(a) ({ uint32_t v_; memcpy(&v_, aot_m + (a), 4); v_; })
#define WRD8(a, v) (aot_m[(a)] = (uint8_t)(v))
#define WRD16(a, v) do { uint16_t v_ = (uint16_t)(v); memcpy(aot_m + (a), &v_, 2); } while (0)
#define WRD32(a, v) do { uint32_t v_ = (v); memcpy(aot_m + (a), &v_, 4); } while (0)
/* The stack, without a check (AOT_ESP_OK) */
#define PUSH32(v) do { uint32_t pv_ = (v); WRD32(ESP - 4, pv_); ESP -= 4; } while (0)
#define PUSH16(v) do { uint32_t pv_ = (v); WRD16(ESP - 2, pv_); ESP -= 2; } while (0)
#define POP32() ({ uint32_t pv_ = RDD32(ESP); ESP += 4; pv_; })
#define POP16() ({ uint32_t pv_ = RDD16(ESP); ESP += 2; pv_; })

/* ---- flags ---- */

static inline uint32_t aot_mask(int sz) { return sz == 4 ? 0xffffffffu : (1u << (sz * 8)) - 1; }
static inline uint32_t aot_sign(int sz) { return 1u << (sz * 8 - 1); }
static inline int32_t aot_sext(uint32_t v, int sz) { return sz == 1 ? (int8_t)v : sz == 2 ? (int16_t)v : (int32_t)v; }

static inline uint32_t aot_szp(uint32_t r, int sz)
{
    r &= aot_mask(sz);
    return (r == 0 ? X86_ZF : 0) | ((r & aot_sign(sz)) ? X86_SF : 0) | (aot_parity[r & 0xff] ? X86_PF : 0);
}

/* The six arithmetic flags of a, b (masked to sz), r = the masked result, cin = carry in */
static inline uint32_t aot_fl_add(uint32_t a, uint32_t b, uint32_t r, uint32_t cin, int sz)
{
    uint32_t f = ((uint64_t)a + b + cin > aot_mask(sz)) ? X86_CF : 0;
    if ((a ^ r) & (b ^ r) & aot_sign(sz)) {
        f |= X86_OF;
    }
    return f | ((a ^ b ^ r) & X86_AF) | aot_szp(r, sz);
}

static inline uint32_t aot_fl_sub(uint32_t a, uint32_t b, uint32_t r, uint32_t cin, int sz)
{
    uint32_t f = ((uint64_t)b + cin > a) ? X86_CF : 0;
    if ((a ^ b) & (a ^ r) & aot_sign(sz)) {
        f |= X86_OF;
    }
    return f | ((a ^ b ^ r) & X86_AF) | aot_szp(r, sz);
}

static inline uint32_t aot_fl_logic(uint32_t r, int sz)
{
    return aot_szp(r, sz);
}

/* inc/dec: all but CF */
static inline uint32_t aot_fl_inc(uint32_t a, uint32_t r, int sz)
{
    return aot_szp(r, sz) | ((a ^ r) & X86_AF) | (r == aot_sign(sz) ? X86_OF : 0);
}

static inline uint32_t aot_fl_dec(uint32_t a, uint32_t r, int sz)
{
    return aot_szp(r, sz) | ((a ^ r) & X86_AF) | ((a & aot_mask(sz)) == aot_sign(sz) ? X86_OF : 0);
}

/* EFLAGS with the flags of an ALU operation written, out of line: a flag store is some 30
 * instructions, and a call to a function of the same file costs the caller only the registers the
 * function uses (GCC's -fipa-ra) */
#define AOT_FL_STORES(sz) \
    static __attribute__((noinline, unused)) uint32_t aot_st_add##sz(uint32_t efl, uint32_t a, uint32_t b, uint32_t r) \
    { return (efl & ~AOT_ARITH) | aot_fl_add(a, b, r, 0, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_adc##sz(uint32_t efl, uint32_t a, uint32_t b, uint32_t r, uint32_t c) \
    { return (efl & ~AOT_ARITH) | aot_fl_add(a, b, r, c, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_sub##sz(uint32_t efl, uint32_t a, uint32_t b, uint32_t r) \
    { return (efl & ~AOT_ARITH) | aot_fl_sub(a, b, r, 0, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_sbb##sz(uint32_t efl, uint32_t a, uint32_t b, uint32_t r, uint32_t c) \
    { return (efl & ~AOT_ARITH) | aot_fl_sub(a, b, r, c, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_logic##sz(uint32_t efl, uint32_t r) \
    { return (efl & ~AOT_ARITH) | aot_fl_logic(r, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_inc##sz(uint32_t efl, uint32_t a, uint32_t r) \
    { return (efl & ~(AOT_ARITH & ~X86_CF)) | aot_fl_inc(a, r, sz); } \
    static __attribute__((noinline, unused)) uint32_t aot_st_dec##sz(uint32_t efl, uint32_t a, uint32_t r) \
    { return (efl & ~(AOT_ARITH & ~X86_CF)) | aot_fl_dec(a, r, sz); }
AOT_FL_STORES(1)
AOT_FL_STORES(2)
AOT_FL_STORES(4)

AOT_INLINE void aot_set_flags(uint32_t *efl, uint32_t mask, uint32_t f)
{
    *efl = (*efl & ~mask) | (f & mask);
}

/* Condition cc (the low nibble of jcc) on a flags value */
static inline bool aot_cond(int cc, uint32_t f)
{
    bool r;
    switch (cc >> 1) {
    case 0: r = f & X86_OF; break;
    case 1: r = f & X86_CF; break;
    case 2: r = f & X86_ZF; break;
    case 3: r = f & (X86_CF | X86_ZF); break;
    case 4: r = f & X86_SF; break;
    case 5: r = f & X86_PF; break;
    case 6: r = !(f & X86_SF) != !(f & X86_OF); break;
    default: r = (f & X86_ZF) || (!(f & X86_SF) != !(f & X86_OF)); break;
    }
    return r ^ (cc & 1);
}

/* ---- the less common operations, as the interpreter does them; `need` = write the flags ---- */

/* group 2: rol ror rcl rcr shl shr sal sar */
AOT_INLINE uint32_t aot_shift(int op, uint32_t v, unsigned cnt, int sz, bool need, uint32_t *efl)
{
    int bits = sz * 8;
    uint32_t mask = aot_mask(sz), sign = aot_sign(sz);
    uint32_t r;
    bool cf, of;
    v &= mask;
    cnt &= 31;
    if (cnt == 0) {
        return v;
    }
    switch (op) {
    case 0: { /* rol */
        unsigned n = cnt % bits;
        r = n ? ((v << n) | (v >> (bits - n))) & mask : v;
        if (need) {
            cf = r & 1;
            of = !!(r & sign) ^ cf;
            aot_set_flags(efl, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        }
        return r;
    }
    case 1: { /* ror */
        unsigned n = cnt % bits;
        r = n ? ((v >> n) | (v << (bits - n))) & mask : v;
        if (need) {
            cf = r & sign;
            of = !!(r & sign) ^ !!(r & (sign >> 1));
            aot_set_flags(efl, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        }
        return r;
    }
    case 2: { /* rcl */
        unsigned n = sz == 4 ? cnt : cnt % (bits + 1);
        cf = (*efl) & X86_CF;
        r = v;
        for (unsigned i = 0; i < n; i++) {
            bool out = r & sign;
            r = ((r << 1) | cf) & mask;
            cf = out;
        }
        of = !!(r & sign) ^ cf;
        aot_set_flags(efl, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        return r;
    }
    case 3: { /* rcr */
        unsigned n = sz == 4 ? cnt : cnt % (bits + 1);
        cf = (*efl) & X86_CF;
        r = v;
        of = !!(r & sign) ^ cf;
        for (unsigned i = 0; i < n; i++) {
            bool out = r & 1;
            r = (r >> 1) | (cf ? sign : 0);
            cf = out;
        }
        aot_set_flags(efl, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        return r;
    }
    case 4:
    case 6: /* shl */
        r = (uint32_t)((uint64_t)v << cnt) & mask;
        cf = cnt <= (unsigned)bits ? ((uint64_t)v >> (bits - cnt)) & 1 : 0;
        of = !!(r & sign) ^ cf;
        break;
    case 5: /* shr */
        r = (uint32_t)((uint64_t)v >> cnt);
        cf = ((uint64_t)v >> (cnt - 1)) & 1;
        of = v & sign;
        break;
    default: { /* sar */
        int32_t s = aot_sext(v, sz);
        r = (uint32_t)(s >> (cnt > 31 ? 31 : cnt)) & mask;
        cf = (s >> (cnt - 1 > 31 ? 31 : cnt - 1)) & 1;
        of = false;
        break;
    }
    }
    if (need) {
        (*efl) = ((*efl) & ~AOT_ARITH) | aot_szp(r, sz) | (cf ? X86_CF : 0) | (of ? X86_OF : 0);
    }
    return r;
}

AOT_INLINE uint32_t aot_shxd(bool left, uint32_t d, uint32_t s, unsigned cnt, int sz, bool need, uint32_t *efl)
{
    int bits = sz * 8;
    uint32_t mask = aot_mask(sz), sign = aot_sign(sz);
    cnt &= 31;
    if (cnt == 0) {
        return d;
    }
    uint64_t r;
    bool cf;
    d &= mask;
    s &= mask;
    if (sz == 2) {
        uint64_t t = ((uint64_t)d << 32) | ((uint64_t)s << 16) | d;
        if (left) {
            r = (t << cnt) >> 32;
            cf = (t >> (48 - cnt)) & 1;
        } else {
            r = t >> cnt;
            cf = (t >> (cnt - 1)) & 1;
        }
    } else if (left) {
        r = (((uint64_t)d << 32) | s) << cnt >> 32;
        cf = (d >> (bits - cnt)) & 1;
    } else {
        r = (((uint64_t)s << 32) | d) >> cnt;
        cf = (d >> (cnt - 1)) & 1;
    }
    uint32_t res = (uint32_t)r & mask;
    if (need) {
        bool of = !!((res ^ d) & sign);
        (*efl) = ((*efl) & ~AOT_ARITH) | aot_szp(res, sz) | (cf ? X86_CF : 0) | (of ? X86_OF : 0);
    }
    return res;
}

/* imul r, r/m(, imm) */
AOT_INLINE uint32_t aot_imul2(uint32_t a, uint32_t b, int sz, bool need, uint32_t *efl)
{
    int64_t p = (int64_t)aot_sext(a, sz) * aot_sext(b, sz);
    uint32_t lo = (uint32_t)p & aot_mask(sz);
    if (need) {
        bool ovf = p != aot_sext(lo, sz);
        (*efl) = ((*efl) & ~AOT_ARITH) | aot_szp(lo, sz) | (ovf ? X86_CF | X86_OF : 0);
    }
    return lo;
}

AOT_INLINE void aot_setreg(uint32_t *r, uint32_t v, int sz)
{
    if (sz == 4) {
        *r = v;
    } else if (sz == 2) {
        *r = (*r & 0xffff0000u) | (v & 0xffff);
    } else {
        *r = (*r & 0xffffff00u) | (v & 0xff);
    }
}

/* mul/imul r/m: EDX:EAX (DX:AX, AX) = accumulator * src */
AOT_INLINE void aot_mul1(bool sgn, uint32_t src, int sz, bool need, uint32_t *eax, uint32_t *edx, uint32_t *efl)
{
    uint32_t mask = aot_mask(sz);
    int bits = sz * 8;
    uint64_t a = (*eax) & mask, lo, hi;
    bool ovf;
    if (!sgn) {
        uint64_t p = a * (uint64_t)(src & mask);
        lo = p & mask;
        hi = (p >> bits) & mask;
        ovf = hi != 0;
    } else {
        int64_t p = (int64_t)aot_sext((uint32_t)a, sz) * aot_sext(src, sz);
        lo = (uint64_t)p & mask;
        hi = ((uint64_t)p >> bits) & mask;
        ovf = p != aot_sext((uint32_t)lo, sz);
    }
    if (sz == 1) {
        aot_setreg(eax, (uint32_t)(lo | (hi << 8)), 2);
    } else {
        aot_setreg(eax, (uint32_t)lo, sz);
        aot_setreg(edx, (uint32_t)hi, sz);
    }
    if (need) {
        (*efl) = ((*efl) & ~AOT_ARITH) | aot_szp((uint32_t)lo, sz) | (ovf ? X86_CF | X86_OF : 0);
    }
}

/* hi:lo / d for hi < d (the quotient fits 32 bits), with 32-bit divisions: a 64-bit one is a
 * library call on RV32 (Hacker's Delight, divlu: two 16-bit quotient digits, normalized) */
AOT_INLINE uint32_t aot_divlu(uint32_t hi, uint32_t lo, uint32_t d, uint32_t *rem)
{
    if (hi == 0) {
        *rem = lo % d;
        return lo / d;
    }
    int s = __builtin_clz(d);
    d <<= s;
    uint32_t dh = d >> 16, dl = d & 0xffff;
    uint32_t n32 = hi << s | (lo >> 1 >> (31 - s)), n10 = lo << s;
    uint32_t n1 = n10 >> 16, n0 = n10 & 0xffff;
    uint32_t q1 = n32 / dh, r = n32 - q1 * dh;
    while (q1 >> 16 || q1 * dl > (r << 16 | n1)) {
        q1--;
        r += dh;
        if (r >> 16) {
            break;
        }
    }
    uint32_t n21 = (n32 << 16 | n1) - q1 * d;
    uint32_t q0 = n21 / dh;
    r = n21 - q0 * dh;
    while (q0 >> 16 || q0 * dl > (r << 16 | n0)) {
        q0--;
        r += dh;
        if (r >> 16) {
            break;
        }
    }
    *rem = ((n21 << 16 | n0) - q0 * d) >> s;
    return q1 << 16 | q0;
}

/* div/idiv r/m; false = #DE (nothing changed: the interpreter redoes the instruction and raises it) */
AOT_INLINE bool aot_div1(bool sgn, uint32_t src, int sz, uint32_t *eax, uint32_t *edx)
{
    uint32_t mask = aot_mask(sz);
    src &= mask;
    if (src == 0) {
        return false;
    }
    uint32_t q, rem;
    if (sz < 4) {
        /* the dividend fits 32 bits */
        uint32_t num = sz == 1 ? ((*eax) & 0xffff) : ((*edx) & 0xffff) << 16 | ((*eax) & 0xffff);
        if (!sgn) {
            q = num / src;
            rem = num % src;
            if (q > mask) {
                return false;
            }
        } else {
            int32_t n = sz == 1 ? (int16_t)num : (int32_t)num, d = aot_sext(src, sz);
            if (n == INT32_MIN && d == -1) {
                return false;       /* the one quotient C cannot compute */
            }
            int32_t sq = n / d;
            if (sq > (int32_t)(mask >> 1) || sq < -(int32_t)(mask >> 1) - 1) {
                return false;
            }
            q = (uint32_t)sq & mask;
            rem = (uint32_t)(n % d) & mask;
        }
        if (sz == 1) {
            aot_setreg(eax, q | rem << 8, 2);
        } else {
            aot_setreg(eax, q, 2);
            aot_setreg(edx, rem, 2);
        }
        return true;
    }
    uint32_t hi = *edx, lo = *eax;
    if (!sgn) {
        if (hi >= src) {
            return false;
        }
        q = aot_divlu(hi, lo, src, &rem);
    } else {
        /* on magnitudes: the quotient takes the sign of both, the remainder the dividend's */
        bool nneg = hi >> 31, dneg = src >> 31;
        if (nneg) {
            hi = ~hi + (lo == 0);
            lo = -lo;
        }
        uint32_t d = dneg ? -src : src;
        if (hi >= d) {
            return false;
        }
        q = aot_divlu(hi, lo, d, &rem);
        if (nneg != dneg ? q > 0x80000000u : q > 0x7fffffffu) {
            return false;
        }
        if (nneg != dneg) {
            q = -q;
        }
        if (nneg) {
            rem = -rem;
        }
    }
    *eax = q;
    *edx = rem;
    return true;
}

/* ---- string instructions (32-bit addressing, flat segments), on aot_cpu ---- */

/* kind: 0xa4 movs, 0xa6 cmps, 0xaa stos, 0xac lods, 0xae scas; rep: 0, 0xf2, 0xf3 */
void aot_string(int kind, int sz, int rep);

#ifdef __cplusplus
}
#endif
