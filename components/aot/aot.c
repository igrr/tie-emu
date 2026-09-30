/* Dispatcher between translated code and the interpreter */
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#define AOT_RUNTIME
#include "aot_rt.h"

/* Native stack translated code may use below the dispatcher: translated code nests native calls
 * like the guest, and the emulation task's stack (main/main.c) has room for this and 16 KB more.
 * Deeper nesting leaves to the dispatcher (AOT_EXIT_STACK), which starts again from the top. */
#define STACK_ROOM (32 * 1024)
#define CACHE_SLOTS 4096u

x86_cpu *aot_cpu;
uint8_t *aot_mem;
uint32_t aot_mem_size;
uint32_t aot_fast_lo, aot_fast_top;
uint32_t aot_esp_lo, aot_esp_size;
int32_t aot_budget;
int32_t aot_budget0;
char *aot_stack_limit;
uint8_t *aot_win;

const uint8_t aot_parity[256] = {
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
    P6(1), P6(0), P6(0), P6(1)
#undef P2
#undef P4
#undef P6
};

static jmp_buf *s_jb;
static int s_reason;
static uint32_t s_fault_addr;
/* The entry map: a bitmap per 4 KB page from s_lo (s_bits), and the pages of the modules that
 * may run (s_pages, what the interpreter stops at: NULL for pages only in modules that failed
 * their check) */
static uint8_t **s_bits, **s_pages;
static uint32_t s_lo, s_npages;
enum { MOD_UNCHECKED, MOD_OK, MOD_BAD };
static uint8_t *s_state;            /* per module */
static uint32_t s_fast_lo, s_fast_size;     /* the CPU's fast range the states were checked for */
static uint32_t s_invalidations;    /* aot_invalidate calls */
static const aot_entry_t *s_cache[CACHE_SLOTS];
void aot_exit(uint32_t eip, int reason)
{
    aot_cpu->eip = eip;
    s_reason = reason;
    longjmp(*s_jb, 1);
}

void aot_fault(uint32_t addr)
{
    s_fault_addr = addr;
    aot_exit(aot_cpu->eip, AOT_EXIT_FAULT);
}

/* The pages that may hold entries of modules that have not failed their check */
static void update_pages(void)
{
    for (uint32_t p = 0; p < s_npages; p++) {
        uint32_t a = s_lo + (p << 12);
        s_pages[p] = NULL;
        for (uint32_t k = 0; s_bits[p] && k < aot_num_modules; k++) {
            const aot_module_t *md = &aot_modules[k];
            if (s_state[k] != MOD_BAD && md->lo < a + 4096 && a < md->hi) {
                s_pages[p] = s_bits[p];
            }
        }
    }
}

static uint32_t hash(const uint8_t *mem, const aot_module_t *md)
{
    uint32_t h = 0x811c9dc5u;
    for (uint32_t r = 0; r < md->nranges; r++) {
        for (uint32_t a = md->ranges[r][0]; a < md->ranges[r][1]; a++) {
            h = (h ^ mem[a]) * 0x01000193u;
        }
    }
    return h;
}

static bool in_memory(const aot_module_t *md, uint32_t mem_size)
{
    return md->hi <= mem_size && md->direct_hi <= mem_size &&
           (!md->nranges || md->ranges[md->nranges - 1][1] <= mem_size);
}

/* Whether module k can run: its constant addresses are ordinary memory, and a loaded module is
 * in memory as translated */
static bool check(uint32_t k)
{
    const aot_module_t *md = &aot_modules[k];
    bool ok = md->direct_hi <= md->direct_lo ||
              (md->direct_lo >= s_fast_lo && md->direct_hi - s_fast_lo <= s_fast_size);
    if (ok && md->loaded) {
        ok = in_memory(md, aot_mem_size) && hash(aot_mem, md) == md->hash;
    }
    s_state[k] = ok ? MOD_OK : MOD_BAD;
    update_pages();
    return ok;
}

void aot_invalidate(void)
{
    s_invalidations++;
    if (!s_state) {
        return;
    }
    for (uint32_t k = 0; k < aot_num_modules; k++) {
        if (aot_modules[k].loaded) {
            s_state[k] = MOD_UNCHECKED;
        }
    }
    update_pages();
}

const aot_entry_t *aot_lookup(uint32_t eip)
{
    uint32_t o = eip - s_lo;
    const uint8_t *b;
    if ((o >> 12) >= s_npages || !(b = s_pages[o >> 12]) || !(b[(o & 4095) >> 3] >> (o & 7) & 1)) {
        return NULL;
    }
    const aot_entry_t **slot = &s_cache[(eip ^ (eip >> 12)) % CACHE_SLOTS];
    if (*slot && (*slot)->eip == eip) {
        goto found;
    }
    uint32_t lo = 0, hi = aot_num_entries;
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2;
        if (aot_entries[mid].eip < eip) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    *slot = &aot_entries[lo];
found:
    if (AOT_UNLIKELY(s_state[(*slot)->mod] != MOD_OK) && !check((*slot)->mod)) {
        return NULL;
    }
    return *slot;
}

static bool flat(const x86_cpu *c);

void aot_int(uint32_t vec, uint32_t next, int32_t ahead)
{
    x86_cpu *c = aot_cpu;
    uint64_t ic = c->icount;
    uint32_t inv = s_invalidations;
    uint16_t cs = c->sel[X86_CS];
    jmp_buf jb, *prev = c->jb;
    /* the count the interpreter would show the handler: the instructions before this one */
    uint64_t at = ic + (uint32_t)(aot_budget0 - aot_budget - ahead);
    c->icount = at;
    c->insn_eip = next - 2;
    c->eip = next;
    c->jb = &jb;
    if (setjmp(jb)) {
        /* an exception delivering it: the interpreter does the INT again and raises it */
        c->jb = prev;
        c->icount = ic;
        aot_budget += ahead;
        aot_exit(next - 2, AOT_EXIT_PUNT);
    }
    c->env->intr(c, (uint8_t)vec);
    c->jb = prev;
    /* what the machine charges for the service (env_intr): the block after an INT checks the
     * budget (aot/lower.py), so a charge past the end of the slice ends it there, as in the
     * interpreter */
    aot_budget -= (int32_t)(c->icount - at);
    c->icount = ic;
    aot_win = c->win;       /* VESA moves the window with INT 10h */
    /* on only where the handler returned to next and changed nothing translated code assumes */
    if (AOT_LIKELY(!c->stop_req && c->eip == next && c->sel[X86_CS] == cs && flat(c) &&
                   !(c->intr_pending && (c->eflags & X86_IF)) && c->r[X86_ESP] - aot_esp_lo < aot_esp_size &&
                   c->mem == aot_mem && c->fast_lo == aot_fast_lo &&
                   (c->fast_size > 3 ? c->fast_size - 3 : 0) == aot_fast_top && s_invalidations == inv)) {
        return;
    }
    aot_budget += ahead - 1;
    aot_exit(c->eip, c->stop_req ? AOT_EXIT_STOP : AOT_EXIT_JUMP);
}

uint64_t aot_in_poll(uint32_t port, int32_t bud, int32_t ahead, uint32_t mask, int32_t nz, int32_t len,
                     int32_t rest, int32_t need)
{
    x86_cpu *c = aot_cpu;
    uint64_t ic = c->icount;
    uint64_t at = ic + (uint32_t)(aot_budget0 - bud - ahead);
    c->icount = at;
    uint32_t v = c->env->in(c, (uint16_t)port, 1);
    uint32_t skip = 0;
    int32_t top = bud - rest;       /* the budget at the next iteration's check */
    if (((v & mask) != 0) == (nz != 0) && top >= need && !c->stop_req && c->env->in_until) {
        /* iteration j after this one reads at at + j * len */
        uint64_t until = c->env->in_until(c, (uint16_t)port, (uint8_t)mask);
        uint64_t same = until > at ? (until - at - 1) / (uint32_t)len : 0;
        uint32_t fit = (uint32_t)(top - need) / (uint32_t)len + 1;
        uint32_t k = same < fit ? (uint32_t)same : fit;
        if (k) {
            skip = k * (uint32_t)len;
            c->icount = at + skip;
            v = c->env->in(c, (uint16_t)port, 1);
        }
    }
    c->icount = ic;
    return v | (uint64_t)skip << 32;
}

uint8_t *aot_p_low(uint32_t a, uint32_t n)
{
    uint8_t *p = x86_ptr_low(aot_cpu, a, n);
    if (!p) {
        aot_fault(a);
    }
    return p;
}

static uint32_t rd(uint32_t a, int sz)
{
    return sz == 1 ? aot_rd8(aot_mem, aot_fast_lo, aot_fast_top, a)
           : sz == 2 ? aot_rd16(aot_mem, aot_fast_lo, aot_fast_top, a) : aot_rd32(aot_mem, aot_fast_lo, aot_fast_top, a);
}

static void wr(uint32_t a, uint32_t v, int sz)
{
    if (sz == 1) {
        aot_wr8(aot_mem, aot_fast_lo, aot_fast_top, a, v);
    } else if (sz == 2) {
        aot_wr16(aot_mem, aot_fast_lo, aot_fast_top, a, v);
    } else {
        aot_wr32(aot_mem, aot_fast_lo, aot_fast_top, a, v);
    }
}

/* Guest [a, a + len) as one piece of host memory, or NULL (x86_ptr lets an access that starts
 * below the video window run on into memory behind it: element by element, the part in the window
 * would go to the window) */
static uint8_t *span(x86_cpu *c, uint32_t a, uint32_t len)
{
    uint8_t *p = len ? x86_ptr(c, a, len) : NULL;
    return p && x86_ptr(c, a + len - 1, 1) == p + len - 1 ? p : NULL;
}

/* rep movs/stos with both ends in host memory: at once. A copy onto the bytes just ahead of its
 * source (just below it, downwards) repeats them, which memmove would not: that goes element by
 * element. Downwards (DF) the elements run from EDI and ESI to len - sz bytes below them. */
static bool string_fast(x86_cpu *c, int kind, int sz)
{
    uint32_t *r = c->r;
    uint64_t len = (uint64_t)r[X86_ECX] * sz;
    bool down = c->eflags & X86_DF;
    uint32_t back = down && len ? (uint32_t)len - sz : 0;
    uint8_t *d;
    if (len > UINT32_MAX || !(d = span(c, r[X86_EDI] - back, (uint32_t)len))) {
        return false;
    }
    if (kind == 0xa4) {
        uint8_t *s;
        if ((down ? r[X86_ESI] - r[X86_EDI] : r[X86_EDI] - r[X86_ESI]) < len ||
            !(s = span(c, r[X86_ESI] - back, (uint32_t)len))) {
            return false;
        }
        memmove(d, s, len);
        r[X86_ESI] += down ? -(uint32_t)len : (uint32_t)len;
    } else if (sz == 1) {
        memset(d, r[X86_EAX] & 0xff, len);
    } else {
        for (uint32_t i = 0; i < len; i += sz) {
            memcpy(d + i, &r[X86_EAX], sz);
        }
    }
    r[X86_EDI] += down ? -(uint32_t)len : (uint32_t)len;
    r[X86_ECX] = 0;
    return true;
}

void aot_string(int kind, int sz, int rep)
{
    x86_cpu *c = aot_cpu;
    uint32_t *r = c->r;
    if (rep && (kind == 0xa4 || kind == 0xaa) && string_fast(c, kind, sz)) {
        return;
    }
    int step = (c->eflags & X86_DF) ? -sz : sz;
    for (;;) {
        if (rep && r[X86_ECX] == 0) {
            break;
        }
        switch (kind) {
        case 0xa4:
            wr(r[X86_EDI], rd(r[X86_ESI], sz), sz);
            break;
        case 0xa6: {
            uint32_t a = rd(r[X86_ESI], sz), b = rd(r[X86_EDI], sz);
            c->eflags = (c->eflags & ~AOT_ARITH) | aot_fl_sub(a, b, (a - b) & aot_mask(sz), 0, sz);
            break;
        }
        case 0xaa:
            wr(r[X86_EDI], r[X86_EAX], sz);
            break;
        case 0xac:
            aot_setreg(&r[X86_EAX], rd(r[X86_ESI], sz), sz);
            break;
        default: {
            uint32_t a = r[X86_EAX] & aot_mask(sz), b = rd(r[X86_EDI], sz);
            c->eflags = (c->eflags & ~AOT_ARITH) | aot_fl_sub(a, b, (a - b) & aot_mask(sz), 0, sz);
            break;
        }
        }
        if (kind == 0xa4 || kind == 0xa6 || kind == 0xac) {
            r[X86_ESI] += step;
        }
        if (kind != 0xac) {
            r[X86_EDI] += step;
        }
        if (!rep) {
            break;
        }
        r[X86_ECX]--;
        if (kind == 0xa6 || kind == 0xae) {
            bool zf = c->eflags & X86_ZF;
            if ((rep == 0xf3 && !zf) || (rep == 0xf2 && zf)) {
                break;
            }
        }
    }
}

bool aot_init(const uint8_t *mem, uint32_t mem_size)
{
    if (!aot_num_entries) {
        return false;
    }
    for (uint32_t k = 0; k < aot_num_modules; k++) {
        const aot_module_t *md = &aot_modules[k];
        if (!md->loaded && (!in_memory(md, mem_size) || hash(mem, md) != md->hash)) {
            return false;
        }
    }
    if (!s_bits) {
        s_lo = aot_entries[0].eip & ~4095u;
        s_npages = ((aot_entries[aot_num_entries - 1].eip - s_lo) >> 12) + 1;
        s_bits = calloc(s_npages, sizeof(*s_bits));
        s_pages = calloc(s_npages, sizeof(*s_pages));
        s_state = calloc(aot_num_modules, 1);
        if (!s_bits || !s_pages || !s_state) {
            return false;
        }
        /* one block for the bitmaps of the pages with entries */
        uint32_t n = 0, last = ~0u;
        for (uint32_t i = 0; i < aot_num_entries; i++) {
            uint32_t p = (aot_entries[i].eip - s_lo) >> 12;
            n += p != last;
            last = p;
        }
        uint8_t *bits = calloc(n, 512);
        if (!bits) {
            return false;
        }
        for (uint32_t i = 0; i < aot_num_entries; i++) {
            uint32_t o = aot_entries[i].eip - s_lo;
            if (!s_bits[o >> 12]) {
                s_bits[o >> 12] = bits;
                bits += 512;
            }
            s_bits[o >> 12][(o & 4095) >> 3] |= 1u << (o & 7);
        }
    }
    memset(s_state, MOD_UNCHECKED, aot_num_modules);
    s_fast_lo = s_fast_size = ~0u;
    update_pages();
    return true;
}

static bool flat(const x86_cpu *c)
{
    return !(c->base[X86_CS] | c->base[X86_DS] | c->base[X86_ES] | c->base[X86_SS]);
}

/* Where ESP must be for translated code to run (aot_rt.h, AOT_ESP_OK) */
static void esp_range(const x86_cpu *c, uint32_t *lo, uint32_t *size)
{
    *lo = c->fast_lo + 2 * AOT_STACK_MARGIN;
    *size = c->fast_size > 4 * AOT_STACK_MARGIN + 3 ? c->fast_size - 4 * AOT_STACK_MARGIN - 3 : 0;
}

int aot_enter(x86_cpu *c, aot_fn fn, uint32_t entry, int32_t max_insns)
{
    volatile int32_t budget = max_insns;
    jmp_buf jb;
    aot_cpu = c;
    aot_mem = c->mem;
    aot_mem_size = c->mem_size;
    aot_fast_lo = c->fast_lo;
    aot_fast_top = c->fast_size > 3 ? c->fast_size - 3 : 0;
    esp_range(c, &aot_esp_lo, &aot_esp_size);
    aot_win = c->win;
    aot_budget = aot_budget0 = budget;
    c->stop_req = false;
    aot_stack_limit = (char *)__builtin_frame_address(0) - STACK_ROOM;
    s_jb = &jb;
    if (setjmp(jb) == 0) {
        aot_call(fn, entry);
        s_reason = AOT_EXIT_RET;
    }
    c->icount += (uint64_t)(budget - aot_budget);
    if (s_reason == AOT_EXIT_FAULT) {
        c->exc = X86_EXC_PF;
        c->exc_addr = s_fault_addr;
    }
    return s_reason;
}

x86_stop_t aot_run(x86_cpu *c, uint64_t max_insns)
{
    uint64_t end = c->icount + max_insns;
    if (c->fast_lo != s_fast_lo || c->fast_size != s_fast_size) {
        /* checked for another fast range: check again */
        s_fast_lo = c->fast_lo;
        s_fast_size = c->fast_size;
        memset(s_state, MOD_UNCHECKED, aot_num_modules);
        update_pages();
    }
    aot_mem = c->mem;
    aot_mem_size = c->mem_size;
    while (c->icount < end) {
        if (c->intr_pending && (c->eflags & X86_IF) && !c->sti_shadow) {
            return X86_STOP_IRQ;
        }
        const aot_entry_t *e = NULL;
        uint32_t esp_lo, esp_size;
        esp_range(c, &esp_lo, &esp_size);
        if (!c->sti_shadow && flat(c) && c->r[X86_ESP] - esp_lo < esp_size) {
            e = aot_lookup(c->eip);
        }
        /* an entry that does not check the budget needs what it runs to the next check */
        if (e && end - c->icount >= e->need) {
            uint64_t left = end - c->icount;
            int reason = aot_enter(c, e->fn, e->idx, left > INT32_MAX ? INT32_MAX : (int32_t)left);
            if (reason < 0) {
                continue;
            }
            if (reason == AOT_EXIT_FAULT) {
                return X86_STOP_EXC;
            }
            if (reason == AOT_EXIT_STOP) {
                return X86_STOP_REQUEST;
            }
            if (reason != AOT_EXIT_BUDGET && reason != AOT_EXIT_PUNT) {
                continue;
            }
        }
        if (c->icount >= end) {
            break;  /* an INT's charge ran past the end (aot_int) */
        }
        /* the interpreter, up to the next translated entry */
        c->entry_pages = (const uint8_t *const *)s_pages;
        c->entry_lo = s_lo;
        c->entry_npages = s_npages;
        x86_stop_t st = x86_run(c, end - c->icount);
        c->entry_pages = NULL;
        if (st != X86_STOP_ENTRY && st != X86_STOP_BUDGET) {
            return st;
        }
    }
    return X86_STOP_BUDGET;
}
