#include "x86_internal.h"

#define R(n) (c->r[n])

static const uint8_t s_parity[256] = {
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
    P6(1), P6(0), P6(0), P6(1)
#undef P2
#undef P4
#undef P6
};

enum { ALU_ADD, ALU_OR, ALU_ADC, ALU_SBB, ALU_AND, ALU_SUB, ALU_XOR, ALU_CMP };

void x86_raise(x86_cpu *c, int exc, uint32_t addr)
{
    c->exc = exc;
    c->exc_addr = addr;
    longjmp(*c->jb, 1);
}

static inline uint32_t szmask(int sz)
{
    return sz == 4 ? 0xffffffffu : (1u << (sz * 8)) - 1;
}

static inline uint32_t szsign(int sz)
{
    return 1u << (sz * 8 - 1);
}

static inline int32_t sext(uint32_t v, int sz)
{
    return sz == 1 ? (int8_t)v : sz == 2 ? (int16_t)v : (int32_t)v;
}

static inline void set_flags(x86_cpu *c, uint32_t mask, uint32_t val)
{
    c->eflags = (c->eflags & ~mask) | (val & mask);
}

/* ZF, SF, PF of a result */
static inline uint32_t szp(uint32_t r, int sz)
{
    r &= szmask(sz);
    return (r == 0 ? X86_ZF : 0) | ((r & szsign(sz)) ? X86_SF : 0) | (s_parity[r & 0xff] ? X86_PF : 0);
}

static inline bool cond(x86_cpu *c, int cc)
{
    uint32_t f = c->eflags;
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

/* near branch: with a 16-bit operand size EIP wraps to 16 bits */
static inline void jump(x86_cpu *c, uint32_t target, int sz)
{
    c->eip = sz == 2 ? target & 0xffff : target;
}

/* ---- fetch ---- */

static inline uint32_t fetch(x86_cpu *c, int sz)
{
    uint32_t v = x86_rd(c, c->base[X86_CS] + c->eip, sz);
    c->eip += sz;
    return v;
}

#define fetch8(c) fetch(c, 1)
#define fetch16(c) fetch(c, 2)
#define fetch32(c) fetch(c, 4)

/* ---- registers ---- */

static inline uint32_t reg_get(x86_cpu *c, int n, int sz)
{
    if (sz == 4) {
        return R(n);
    }
    if (sz == 2) {
        return R(n) & 0xffff;
    }
    return n < 4 ? R(n) & 0xff : (R(n - 4) >> 8) & 0xff;
}

static inline void reg_set(x86_cpu *c, int n, uint32_t v, int sz)
{
    if (sz == 4) {
        R(n) = v;
    } else if (sz == 2) {
        R(n) = (R(n) & 0xffff0000u) | (v & 0xffff);
    } else if (n < 4) {
        R(n) = (R(n) & 0xffffff00u) | (v & 0xff);
    } else {
        R(n - 4) = (R(n - 4) & 0xffff00ffu) | ((v & 0xff) << 8);
    }
}

/* ---- modrm ---- */

static void modrm(x86_cpu *c, x86_insn *d)
{
    uint8_t m = fetch8(c);
    d->mod = m >> 6;
    d->reg = (m >> 3) & 7;
    d->rm = m & 7;
    if (d->mod == 3) {
        return;
    }
    int seg = X86_DS;
    uint32_t ea;
    if (d->adsz == 4) {
        if (d->rm == 4) {
            uint8_t sib = fetch8(c);
            int idx = (sib >> 3) & 7, base = sib & 7;
            ea = idx == 4 ? 0 : R(idx) << (sib >> 6);
            if (base == 5 && d->mod == 0) {
                ea += fetch32(c);
            } else {
                ea += R(base);
                if (base == X86_ESP || base == X86_EBP) {
                    seg = X86_SS;
                }
            }
        } else if (d->rm == 5 && d->mod == 0) {
            ea = fetch32(c);
        } else {
            ea = R(d->rm);
            if (d->rm == X86_EBP) {
                seg = X86_SS;
            }
        }
        if (d->mod == 1) {
            ea += (int8_t)fetch8(c);
        } else if (d->mod == 2) {
            ea += fetch32(c);
        }
    } else {
        static const int8_t b16[8] = { X86_EBX, X86_EBX, X86_EBP, X86_EBP, -1, -1, X86_EBP, X86_EBX };
        static const int8_t i16[8] = { X86_ESI, X86_EDI, X86_ESI, X86_EDI, X86_ESI, X86_EDI, -1, -1 };
        if (d->rm == 6 && d->mod == 0) {
            ea = fetch16(c);
        } else {
            ea = (b16[d->rm] >= 0 ? R(b16[d->rm]) : 0) + (i16[d->rm] >= 0 ? R(i16[d->rm]) : 0);
            if (b16[d->rm] == X86_EBP) {
                seg = X86_SS;
            }
        }
        if (d->mod == 1) {
            ea += (int8_t)fetch8(c);
        } else if (d->mod == 2) {
            ea += fetch16(c);
        }
        ea &= 0xffff;
    }
    if (d->seg >= 0) {
        seg = d->seg;
    }
    d->ea = ea;
    d->lin = c->base[seg] + ea;
}

static inline uint32_t rm_read(x86_cpu *c, x86_insn *d, int sz)
{
    return d->mod == 3 ? reg_get(c, d->rm, sz) : x86_rd(c, d->lin, sz);
}

static inline void rm_write(x86_cpu *c, x86_insn *d, uint32_t v, int sz)
{
    if (d->mod == 3) {
        reg_set(c, d->rm, v, sz);
    } else {
        x86_wr(c, d->lin, v, sz);
    }
}

/* ---- stack ---- */

static inline void push(x86_cpu *c, uint32_t v, int sz)
{
    uint32_t sp = R(X86_ESP) - sz;
    x86_wr(c, c->base[X86_SS] + sp, v, sz);
    R(X86_ESP) = sp;
}

static inline uint32_t pop(x86_cpu *c, int sz)
{
    uint32_t v = x86_rd(c, c->base[X86_SS] + R(X86_ESP), sz);
    R(X86_ESP) += sz;
    return v;
}

bool x86_set_seg(x86_cpu *c, int sreg, uint16_t sel)
{
    uint32_t base = 0, limit = 0xffffffffu;
    if (c->env && c->env->load_seg && !c->env->load_seg(c, sel, &base, &limit)) {
        return false;
    }
    c->sel[sreg] = sel;
    c->base[sreg] = base;
    return true;
}

static void load_seg(x86_cpu *c, int sreg, uint16_t sel)
{
    if (!x86_set_seg(c, sreg, sel)) {
        x86_raise(c, X86_EXC_GP, sel);
    }
}

void x86_interrupt(x86_cpu *c, uint16_t cs, uint32_t eip)
{
    push(c, c->eflags, 4);
    push(c, c->sel[X86_CS], 4);
    push(c, c->eip, 4);
    c->eflags &= ~(X86_IF | X86_TF);
    if (cs != c->sel[X86_CS]) {
        load_seg(c, X86_CS, cs);
    }
    c->eip = eip;
}

static void set_eflags(x86_cpu *c, uint32_t v, int sz)
{
    const uint32_t writable = X86_ARITH_FLAGS | X86_TF | X86_IF | X86_DF | 0x3000 /* IOPL */ | 0x4000 /* NT */
                              | X86_AC | X86_ID;
    uint32_t m = sz == 2 ? writable & 0xffff : writable;
    c->eflags = (c->eflags & ~m) | (v & m) | 2;
}

/* ---- ALU ---- */

static uint32_t alu(x86_cpu *c, int op, uint32_t a, uint32_t b, int sz)
{
    uint32_t mask = szmask(sz), sign = szsign(sz);
    uint32_t r, f = 0;
    uint32_t cin = c->eflags & X86_CF;
    a &= mask;
    b &= mask;
    switch (op) {
    case ALU_ADD:
    case ALU_ADC: {
        uint64_t full = (uint64_t)a + b + (op == ALU_ADC ? cin : 0);
        r = (uint32_t)full & mask;
        if (full > mask) {
            f |= X86_CF;
        }
        if ((a ^ r) & (b ^ r) & sign) {
            f |= X86_OF;
        }
        f |= (a ^ b ^ r) & X86_AF;
        break;
    }
    case ALU_SUB:
    case ALU_SBB:
    case ALU_CMP: {
        uint64_t sub = (uint64_t)b + (op == ALU_SBB ? cin : 0);
        r = (uint32_t)(a - sub) & mask;
        if (sub > a) {
            f |= X86_CF;
        }
        if ((a ^ b) & (a ^ r) & sign) {
            f |= X86_OF;
        }
        f |= (a ^ b ^ r) & X86_AF;
        break;
    }
    case ALU_OR: r = a | b; break;
    case ALU_AND: r = a & b; break;
    default: r = a ^ b; break;
    }
    c->eflags = (c->eflags & ~X86_ARITH_FLAGS) | f | szp(r, sz);
    return r;
}

static uint32_t incdec(x86_cpu *c, uint32_t a, bool dec, int sz)
{
    uint32_t mask = szmask(sz), sign = szsign(sz);
    uint32_t r = (dec ? a - 1 : a + 1) & mask;
    uint32_t f = szp(r, sz) | ((a ^ r) & X86_AF);
    if (dec ? (a & mask) == sign : r == sign) {
        f |= X86_OF;
    }
    set_flags(c, X86_ARITH_FLAGS & ~X86_CF, f);
    return r;
}

/* group 2: rol ror rcl rcr shl shr sal sar */
static uint32_t shift(x86_cpu *c, int op, uint32_t v, unsigned cnt, int sz)
{
    int bits = sz * 8;
    uint32_t mask = szmask(sz), sign = szsign(sz);
    uint32_t r, f;
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
        cf = r & 1;
        of = !!(r & sign) ^ cf;
        set_flags(c, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        return r;
    }
    case 1: { /* ror */
        unsigned n = cnt % bits;
        r = n ? ((v >> n) | (v << (bits - n))) & mask : v;
        cf = r & sign;
        of = !!(r & sign) ^ !!(r & (sign >> 1));
        set_flags(c, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        return r;
    }
    case 2: { /* rcl */
        unsigned n = sz == 4 ? cnt : cnt % (bits + 1);
        cf = c->eflags & X86_CF;
        r = v;
        for (unsigned i = 0; i < n; i++) {
            bool out = r & sign;
            r = ((r << 1) | cf) & mask;
            cf = out;
        }
        of = !!(r & sign) ^ cf;
        set_flags(c, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
        return r;
    }
    case 3: { /* rcr */
        unsigned n = sz == 4 ? cnt : cnt % (bits + 1);
        cf = c->eflags & X86_CF;
        r = v;
        of = !!(r & sign) ^ cf;
        for (unsigned i = 0; i < n; i++) {
            bool out = r & 1;
            r = (r >> 1) | (cf ? sign : 0);
            cf = out;
        }
        set_flags(c, X86_CF | X86_OF, (cf ? X86_CF : 0) | (of ? X86_OF : 0));
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
        int32_t s = sext(v, sz);
        r = (uint32_t)(s >> (cnt > 31 ? 31 : cnt)) & mask;
        cf = (s >> (cnt - 1 > 31 ? 31 : cnt - 1)) & 1;
        of = false;
        break;
    }
    }
    f = szp(r, sz) | (cf ? X86_CF : 0) | (of ? X86_OF : 0);
    c->eflags = (c->eflags & ~X86_ARITH_FLAGS) | f;
    return r;
}

static uint32_t shxd(x86_cpu *c, bool left, uint32_t d, uint32_t s, unsigned cnt, int sz)
{
    int bits = sz * 8;
    uint32_t mask = szmask(sz), sign = szsign(sz);
    cnt &= 31;
    if (cnt == 0) {
        return d;
    }
    uint64_t r;
    bool cf;
    d &= mask;
    s &= mask;
    if (sz == 2) {
        /* 16-bit: shift through dest:src:dest, which is what Intel parts do for counts above 16 */
        if (left) {
            uint64_t t = ((uint64_t)d << 32) | ((uint64_t)s << 16) | d;
            r = (t << cnt) >> 32;
            cf = (t >> (48 - cnt)) & 1;
        } else {
            uint64_t t = ((uint64_t)d << 32) | ((uint64_t)s << 16) | d;
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
    bool of = !!((res ^ d) & sign);
    c->eflags = (c->eflags & ~X86_ARITH_FLAGS) | szp(res, sz) | (cf ? X86_CF : 0) | (of ? X86_OF : 0);
    return res;
}

/* ---- group 3 multiply / divide ---- */

static void mul_div(x86_cpu *c, int op, uint32_t src, int sz)
{
    uint32_t mask = szmask(sz);
    int bits = sz * 8;
    if (op == 4 || op == 5) {
        uint64_t a = reg_get(c, X86_EAX, sz), lo, hi;
        bool ovf;
        if (op == 4) {
            uint64_t p = a * (uint64_t)(src & mask);
            lo = p & mask;
            hi = (p >> bits) & mask;
            ovf = hi != 0;
        } else {
            int64_t p = (int64_t)sext((uint32_t)a, sz) * sext(src, sz);
            lo = (uint64_t)p & mask;
            hi = ((uint64_t)p >> bits) & mask;
            ovf = p != sext((uint32_t)lo, sz);
        }
        if (sz == 1) {
            reg_set(c, X86_EAX, (uint32_t)(lo | (hi << 8)), 2);
        } else {
            reg_set(c, X86_EAX, (uint32_t)lo, sz);
            reg_set(c, X86_EDX, (uint32_t)hi, sz);
        }
        c->eflags = (c->eflags & ~X86_ARITH_FLAGS) | szp((uint32_t)lo, sz) | (ovf ? X86_CF | X86_OF : 0);
        return;
    }
    src &= mask;
    if (src == 0) {
        x86_raise(c, X86_EXC_DE, 0);
    }
    uint64_t num = sz == 1 ? (R(X86_EAX) & 0xffff)
                   : ((uint64_t)reg_get(c, X86_EDX, sz) << bits) | reg_get(c, X86_EAX, sz);
    uint64_t q, rem;
    if (op == 6) {
        q = num / src;
        rem = num % src;
        if (q > mask) {
            x86_raise(c, X86_EXC_DE, 0);
        }
    } else {
        int64_t n = sz == 1 ? (int16_t)num : sz == 2 ? (int32_t)num : (int64_t)num;
        int64_t s = sext(src, sz);
        if (n == INT64_MIN && s == -1) {
            x86_raise(c, X86_EXC_DE, 0);
        }
        int64_t sq = n / s, sr = n % s;
        if (sq != sext((uint32_t)sq & mask, sz) || sq > (int64_t)(mask >> 1) || sq < -(int64_t)(mask >> 1) - 1) {
            x86_raise(c, X86_EXC_DE, 0);
        }
        q = (uint64_t)sq & mask;
        rem = (uint64_t)sr & mask;
    }
    if (sz == 1) {
        reg_set(c, X86_EAX, (uint32_t)(q | (rem << 8)), 2);
    } else {
        reg_set(c, X86_EAX, (uint32_t)q, sz);
        reg_set(c, X86_EDX, (uint32_t)rem, sz);
    }
}

static uint32_t imul2(x86_cpu *c, uint32_t a, uint32_t b, int sz)
{
    int64_t p = (int64_t)sext(a, sz) * sext(b, sz);
    uint32_t lo = (uint32_t)p & szmask(sz);
    bool ovf = p != sext(lo, sz);
    c->eflags = (c->eflags & ~X86_ARITH_FLAGS) | szp(lo, sz) | (ovf ? X86_CF | X86_OF : 0);
    return lo;
}

/* ---- string instructions ---- */

static void string_op(x86_cpu *c, x86_insn *d, uint8_t op)
{
    int sz = (op & 1) ? d->opsz : 1;
    int step = (c->eflags & X86_DF) ? -sz : sz;
    uint32_t amask = d->adsz == 4 ? 0xffffffffu : 0xffff;
    uint32_t srcbase = c->base[d->seg >= 0 ? d->seg : X86_DS];
    uint32_t esbase = c->base[X86_ES];
    bool rep = d->rep != 0;
    int kind = op & 0xfe;

    for (;;) {
        if (rep && (R(X86_ECX) & amask) == 0) {
            break;
        }
        uint32_t si = R(X86_ESI) & amask, di = R(X86_EDI) & amask;
        switch (kind) {
        case 0xa4: /* movs */
            x86_wr(c, esbase + di, x86_rd(c, srcbase + si, sz), sz);
            break;
        case 0xa6: /* cmps */
            alu(c, ALU_CMP, x86_rd(c, srcbase + si, sz), x86_rd(c, esbase + di, sz), sz);
            break;
        case 0xaa: /* stos */
            x86_wr(c, esbase + di, reg_get(c, X86_EAX, sz), sz);
            break;
        case 0xac: /* lods */
            reg_set(c, X86_EAX, x86_rd(c, srcbase + si, sz), sz);
            break;
        case 0xae: /* scas */
            alu(c, ALU_CMP, reg_get(c, X86_EAX, sz), x86_rd(c, esbase + di, sz), sz);
            break;
        case 0x6c: /* ins */
            x86_wr(c, esbase + di, c->env->in(c, R(X86_EDX) & 0xffff, sz), sz);
            break;
        default: /* 0x6e outs */
            c->env->out(c, R(X86_EDX) & 0xffff, x86_rd(c, srcbase + si, sz), sz);
            break;
        }
        bool uses_si = kind == 0xa4 || kind == 0xa6 || kind == 0xac || kind == 0x6e;
        bool uses_di = kind != 0xac && kind != 0x6e;
        if (uses_si) {
            R(X86_ESI) = (R(X86_ESI) & ~amask) | ((si + step) & amask);
        }
        if (uses_di) {
            R(X86_EDI) = (R(X86_EDI) & ~amask) | ((di + step) & amask);
        }
        if (!rep) {
            break;
        }
        R(X86_ECX) = (R(X86_ECX) & ~amask) | ((R(X86_ECX) - 1) & amask);
        if (kind == 0xa6 || kind == 0xae) {
            bool zf = c->eflags & X86_ZF;
            if ((d->rep == 0xf3 && !zf) || (d->rep == 0xf2 && zf)) {
                break;
            }
        }
    }
}

/* ---- BCD ---- */

static void bcd(x86_cpu *c, uint8_t op, uint8_t imm)
{
    uint32_t al = R(X86_EAX) & 0xff, f = c->eflags;
    bool cf = f & X86_CF, af = f & X86_AF;
    switch (op) {
    case 0x27: /* daa */
    case 0x2f: { /* das */
        uint32_t old = al;
        bool oldcf = cf;
        cf = false;
        if ((al & 0xf) > 9 || af) {
            al = op == 0x27 ? al + 6 : al - 6;
            cf = oldcf || (op == 0x27 ? al > 0xff : old < 6);
            af = true;
        } else {
            af = false;
        }
        if (old > 0x99 || oldcf) {
            al = op == 0x27 ? al + 0x60 : al - 0x60;
            cf = true;
        }
        al &= 0xff;
        reg_set(c, X86_EAX, al, 1);
        c->eflags = (f & ~X86_ARITH_FLAGS) | szp(al, 1) | (cf ? X86_CF : 0) | (af ? X86_AF : 0);
        return;
    }
    case 0x37: /* aaa */
    case 0x3f: { /* aas */
        uint32_t ax = R(X86_EAX) & 0xffff;
        if ((al & 0xf) > 9 || af) {
            ax = op == 0x37 ? ax + 0x106 : ax - 0x106;
            af = cf = true;
        } else {
            af = cf = false;
        }
        ax = (ax & 0xff0f);
        reg_set(c, X86_EAX, ax, 2);
        c->eflags = (f & ~X86_ARITH_FLAGS) | szp(ax & 0xff, 1) | (cf ? X86_CF : 0) | (af ? X86_AF : 0);
        return;
    }
    case 0xd4: /* aam */
        if (imm == 0) {
            x86_raise(c, X86_EXC_DE, 0);
        }
        reg_set(c, X86_EAX, ((al / imm) << 8) | (al % imm), 2);
        c->eflags = (f & ~X86_ARITH_FLAGS) | szp(al % imm, 1);
        return;
    default: { /* aad */
        uint32_t ah = (R(X86_EAX) >> 8) & 0xff;
        al = (al + ah * imm) & 0xff;
        reg_set(c, X86_EAX, al, 2);
        c->eflags = (f & ~X86_ARITH_FLAGS) | szp(al, 1);
        return;
    }
    }
}

/* ---- bit test ---- */

static void bit_op(x86_cpu *c, x86_insn *d, int op, uint32_t bitoff, bool from_reg)
{
    int sz = d->opsz, bits = sz * 8;
    uint32_t v, lin = 0;
    if (d->mod == 3) {
        bitoff &= bits - 1;
        v = reg_get(c, d->rm, sz);
    } else {
        int32_t off = from_reg ? sext(bitoff, sz) : (int32_t)(bitoff & (bits - 1));
        lin = d->lin + (uint32_t)((off >> (sz == 4 ? 5 : 4)) * sz);
        bitoff = (uint32_t)off & (bits - 1);
        v = x86_rd(c, lin, sz);
    }
    uint32_t m = 1u << bitoff;
    set_flags(c, X86_CF, (v & m) ? X86_CF : 0);
    if (op == 0) {
        return;
    }
    v = op == 1 ? v | m : op == 2 ? v & ~m : v ^ m;
    if (d->mod == 3) {
        reg_set(c, d->rm, v, sz);
    } else {
        x86_wr(c, lin, v, sz);
    }
}

/* ---- two-byte opcodes ---- */

static void step_0f(x86_cpu *c, x86_insn *d)
{
    uint8_t op = fetch8(c);
    int sz = d->opsz;
    uint32_t v;

    if (op >= 0x80 && op <= 0x8f) { /* jcc rel16/32 */
        int32_t rel = sz == 4 ? (int32_t)fetch32(c) : (int16_t)fetch16(c);
        if (cond(c, op & 15)) {
            jump(c, c->eip + rel, sz);
        }
        return;
    }
    if (op >= 0x90 && op <= 0x9f) { /* setcc */
        modrm(c, d);
        rm_write(c, d, cond(c, op & 15), 1);
        return;
    }
    if (op >= 0xc8 && op <= 0xcf) { /* bswap; the 16-bit form is undefined, QEMU clears the register */
        if (sz == 2) {
            reg_set(c, op & 7, 0, 2);
            return;
        }
        v = R(op & 7);
        R(op & 7) = (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24);
        return;
    }

    switch (op) {
    case 0x00: /* sldt/str/lldt/ltr/verr/verw */
        modrm(c, d);
        if (d->reg < 2) {
            rm_write(c, d, 0, d->mod == 3 ? sz : 2);
            return;
        }
        if (d->reg >= 4) {
            set_flags(c, X86_ZF, X86_ZF);
            return;
        }
        x86_raise(c, X86_EXC_UD, 0);
    case 0x01: /* sgdt/sidt/lgdt/lidt/smsw/lmsw */
        modrm(c, d);
        switch (d->reg) {
        case 0:
        case 1:
            x86_wr(c, d->lin, 0x7ff, 2);
            x86_wr(c, d->lin + 2, 0, 4);
            return;
        case 4:
            rm_write(c, d, c->cr0, d->mod == 3 ? sz : 2);
            return;
        case 6:
            c->cr0 = (c->cr0 & ~0xfu) | (rm_read(c, d, 2) & 0xf) | (c->cr0 & 1);
            return;
        default:
            x86_raise(c, X86_EXC_UD, 0);
        }
    case 0x02: /* lar */
    case 0x03: { /* lsl */
        modrm(c, d);
        uint32_t lim;
        uint16_t sel = rm_read(c, d, 2);
        if (op == 0x03 && c->env->seg_limit && c->env->seg_limit(c, sel, &lim)) {
            reg_set(c, d->reg, lim, sz);
            set_flags(c, X86_ZF, X86_ZF);
        } else {
            set_flags(c, X86_ZF, 0);
        }
        return;
    }
    case 0x06: /* clts */
        c->cr0 &= ~8u;
        return;
    case 0x20: /* mov r32, crN */
        modrm(c, d);
        R(d->rm) = d->reg == 0 ? c->cr0 : 0;
        return;
    case 0x22: /* mov crN, r32 */
        modrm(c, d);
        if (d->reg == 0) {
            c->cr0 = R(d->rm);
        }
        return;
    case 0x31: /* rdtsc */
        R(X86_EAX) = (uint32_t)c->icount;
        R(X86_EDX) = (uint32_t)(c->icount >> 32);
        return;
    case 0xa2: /* cpuid: an i486DX2 */
        if (R(X86_EAX) == 0) {
            R(X86_EAX) = 1;
            R(X86_EBX) = 0x756e6547; /* GenuineIntel */
            R(X86_EDX) = 0x49656e69;
            R(X86_ECX) = 0x6c65746e;
        } else {
            R(X86_EAX) = 0x435;
            R(X86_EBX) = R(X86_ECX) = 0;
            R(X86_EDX) = 1; /* FPU */
        }
        return;
    case 0xa0: push(c, c->sel[X86_FS], sz); return;
    case 0xa1: load_seg(c, X86_FS, pop(c, sz)); return;
    case 0xa8: push(c, c->sel[X86_GS], sz); return;
    case 0xa9: load_seg(c, X86_GS, pop(c, sz)); return;
    case 0xa3: modrm(c, d); bit_op(c, d, 0, reg_get(c, d->reg, sz), true); return;
    case 0xab: modrm(c, d); bit_op(c, d, 1, reg_get(c, d->reg, sz), true); return;
    case 0xb3: modrm(c, d); bit_op(c, d, 2, reg_get(c, d->reg, sz), true); return;
    case 0xbb: modrm(c, d); bit_op(c, d, 3, reg_get(c, d->reg, sz), true); return;
    case 0xba:
        modrm(c, d);
        if (d->reg < 4) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        bit_op(c, d, d->reg - 4, fetch8(c), false);
        return;
    case 0xa4:
    case 0xa5:
    case 0xac:
    case 0xad: {
        modrm(c, d);
        unsigned cnt = (op & 1) ? R(X86_ECX) & 0xff : fetch8(c);
        v = shxd(c, op < 0xa8, rm_read(c, d, sz), reg_get(c, d->reg, sz), cnt, sz);
        rm_write(c, d, v, sz);
        return;
    }
    case 0xaf:
        modrm(c, d);
        reg_set(c, d->reg, imul2(c, reg_get(c, d->reg, sz), rm_read(c, d, sz), sz), sz);
        return;
    case 0xb0:
    case 0xb1: { /* cmpxchg */
        int s = op == 0xb0 ? 1 : sz;
        modrm(c, d);
        v = rm_read(c, d, s);
        alu(c, ALU_CMP, reg_get(c, X86_EAX, s), v, s);
        if (c->eflags & X86_ZF) {
            rm_write(c, d, reg_get(c, d->reg, s), s);
        } else {
            rm_write(c, d, v, s);
            reg_set(c, X86_EAX, v, s);
        }
        return;
    }
    case 0xc0:
    case 0xc1: { /* xadd */
        int s = op == 0xc0 ? 1 : sz;
        modrm(c, d);
        v = rm_read(c, d, s);
        uint32_t sum = alu(c, ALU_ADD, v, reg_get(c, d->reg, s), s);
        reg_set(c, d->reg, v, s);
        rm_write(c, d, sum, s);
        return;
    }
    case 0xb2:
    case 0xb4:
    case 0xb5: { /* lss/lfs/lgs */
        modrm(c, d);
        if (d->mod == 3) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        uint32_t off = x86_rd(c, d->lin, sz);
        uint16_t sel = x86_rd(c, d->lin + sz, 2);
        load_seg(c, op == 0xb2 ? X86_SS : op == 0xb4 ? X86_FS : X86_GS, sel);
        reg_set(c, d->reg, off, sz);
        return;
    }
    case 0xb6:
    case 0xb7:
    case 0xbe:
    case 0xbf: {
        int s = (op & 1) ? 2 : 1;
        modrm(c, d);
        v = rm_read(c, d, s);
        reg_set(c, d->reg, op >= 0xbe ? (uint32_t)sext(v, s) : v, sz);
        return;
    }
    case 0xff: /* hypercall */
        v = fetch8(c);
        if (!c->env->hypercall) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        c->env->hypercall(c, v);
        return;
    case 0xbc:
    case 0xbd: { /* bsf/bsr */
        modrm(c, d);
        v = rm_read(c, d, sz);
        if (v == 0) {
            set_flags(c, X86_ZF, X86_ZF);
            return;
        }
        int n = op == 0xbc ? __builtin_ctz(v) : 31 - __builtin_clz(v);
        reg_set(c, d->reg, n, sz);
        set_flags(c, X86_ZF, 0);
        return;
    }
    default:
        x86_raise(c, X86_EXC_UD, 0);
    }
}

/* ---- one-byte opcodes ---- */

static void step(x86_cpu *c)
{
    x86_insn d = { .opsz = 4, .adsz = 4, .seg = -1, .rep = 0 };
    uint8_t op;
    uint32_t v, t;
    int sz;

    for (;;) {
        op = fetch8(c);
        switch (op) {
        case 0x26: d.seg = X86_ES; continue;
        case 0x2e: d.seg = X86_CS; continue;
        case 0x36: d.seg = X86_SS; continue;
        case 0x3e: d.seg = X86_DS; continue;
        case 0x64: d.seg = X86_FS; continue;
        case 0x65: d.seg = X86_GS; continue;
        case 0x66: d.opsz = 2; continue;
        case 0x67: d.adsz = 2; continue;
        case 0xf0: continue;
        case 0xf2:
        case 0xf3: d.rep = op; continue;
        }
        break;
    }

    /* ALU block 00..3F: op r/m,r / r,r/m / acc,imm */
    if (op < 0x40 && (op & 7) < 6) {
        int aop = op >> 3;
        sz = (op & 1) ? d.opsz : 1;
        switch (op & 7) {
        case 0:
        case 1:
            modrm(c, &d);
            v = alu(c, aop, rm_read(c, &d, sz), reg_get(c, d.reg, sz), sz);
            if (aop != ALU_CMP) {
                rm_write(c, &d, v, sz);
            }
            return;
        case 2:
        case 3:
            modrm(c, &d);
            v = alu(c, aop, reg_get(c, d.reg, sz), rm_read(c, &d, sz), sz);
            if (aop != ALU_CMP) {
                reg_set(c, d.reg, v, sz);
            }
            return;
        default:
            t = fetch(c, sz);
            v = alu(c, aop, reg_get(c, X86_EAX, sz), t, sz);
            if (aop != ALU_CMP) {
                reg_set(c, X86_EAX, v, sz);
            }
            return;
        }
    }

    sz = d.opsz;
    switch (op) {
    case 0x06: push(c, c->sel[X86_ES], sz); return;
    case 0x07: load_seg(c, X86_ES, pop(c, sz)); return;
    case 0x0e: push(c, c->sel[X86_CS], sz); return;
    case 0x16: push(c, c->sel[X86_SS], sz); return;
    case 0x17: load_seg(c, X86_SS, pop(c, sz)); return;
    case 0x1e: push(c, c->sel[X86_DS], sz); return;
    case 0x1f: load_seg(c, X86_DS, pop(c, sz)); return;
    case 0x0f: step_0f(c, &d); return;
    case 0x27:
    case 0x2f:
    case 0x37:
    case 0x3f:
        bcd(c, op, 0);
        return;
    case 0x40 ... 0x47:
        reg_set(c, op & 7, incdec(c, reg_get(c, op & 7, sz), false, sz), sz);
        return;
    case 0x48 ... 0x4f:
        reg_set(c, op & 7, incdec(c, reg_get(c, op & 7, sz), true, sz), sz);
        return;
    case 0x50 ... 0x57:
        push(c, reg_get(c, op & 7, sz), sz);
        return;
    case 0x58 ... 0x5f:
        v = pop(c, sz);
        reg_set(c, op & 7, v, sz);
        return;
    case 0x60: { /* pusha */
        uint32_t sp = R(X86_ESP);
        for (int i = 0; i < 8; i++) {
            push(c, i == X86_ESP ? sp : R(i), sz);
        }
        return;
    }
    case 0x61: /* popa */
        for (int i = 7; i >= 0; i--) {
            v = pop(c, sz);
            if (i != X86_ESP) {
                reg_set(c, i, v, sz);
            }
        }
        return;
    case 0x68: push(c, fetch(c, sz), sz); return;
    case 0x6a: push(c, (uint32_t)(int8_t)fetch8(c), sz); return;
    case 0x69:
    case 0x6b:
        modrm(c, &d);
        v = rm_read(c, &d, sz);
        t = op == 0x69 ? fetch(c, sz) : (uint32_t)(int8_t)fetch8(c);
        reg_set(c, d.reg, imul2(c, v, t, sz), sz);
        return;
    case 0x6c: case 0x6d: case 0x6e: case 0x6f:
    case 0xa4: case 0xa5: case 0xa6: case 0xa7:
    case 0xaa: case 0xab: case 0xac: case 0xad: case 0xae: case 0xaf:
        string_op(c, &d, op);
        return;
    case 0x70 ... 0x7f: {
        int8_t rel = fetch8(c);
        if (cond(c, op & 15)) {
            jump(c, c->eip + rel, sz);
        }
        return;
    }
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83: {
        int s = (op & 1) ? sz : 1;
        modrm(c, &d);
        v = rm_read(c, &d, s);
        t = op == 0x81 ? fetch(c, s) : (uint32_t)(int8_t)fetch8(c);
        v = alu(c, d.reg, v, t, s);
        if (d.reg != ALU_CMP) {
            rm_write(c, &d, v, s);
        }
        return;
    }
    case 0x84:
    case 0x85: {
        int s = (op & 1) ? sz : 1;
        modrm(c, &d);
        alu(c, ALU_AND, rm_read(c, &d, s), reg_get(c, d.reg, s), s);
        return;
    }
    case 0x86:
    case 0x87: {
        int s = (op & 1) ? sz : 1;
        modrm(c, &d);
        v = rm_read(c, &d, s);
        rm_write(c, &d, reg_get(c, d.reg, s), s);
        reg_set(c, d.reg, v, s);
        return;
    }
    case 0x88: modrm(c, &d); rm_write(c, &d, reg_get(c, d.reg, 1), 1); return;
    case 0x89: modrm(c, &d); rm_write(c, &d, reg_get(c, d.reg, sz), sz); return;
    case 0x8a: modrm(c, &d); reg_set(c, d.reg, rm_read(c, &d, 1), 1); return;
    case 0x8b: modrm(c, &d); reg_set(c, d.reg, rm_read(c, &d, sz), sz); return;
    case 0x8c:
        modrm(c, &d);
        if (d.reg > 5) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        /* to a register: zero-extended to the operand size; to memory: 16 bits */
        rm_write(c, &d, c->sel[d.reg], d.mod == 3 ? sz : 2);
        return;
    case 0x8d:
        modrm(c, &d);
        if (d.mod == 3) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        reg_set(c, d.reg, d.ea, sz);
        return;
    case 0x8e:
        modrm(c, &d);
        if (d.reg == X86_CS || d.reg > 5) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        load_seg(c, d.reg, rm_read(c, &d, 2));
        return;
    case 0x8f:
        /* pop r/m: esp is incremented before the address is computed */
        v = pop(c, sz);
        modrm(c, &d);
        rm_write(c, &d, v, sz);
        return;
    case 0x90: return;
    case 0x91 ... 0x97:
        v = reg_get(c, op & 7, sz);
        reg_set(c, op & 7, reg_get(c, X86_EAX, sz), sz);
        reg_set(c, X86_EAX, v, sz);
        return;
    case 0x98:
        if (sz == 4) {
            R(X86_EAX) = (uint32_t)(int16_t)R(X86_EAX);
        } else {
            reg_set(c, X86_EAX, (uint32_t)(int8_t)R(X86_EAX), 2);
        }
        return;
    case 0x99:
        if (sz == 4) {
            R(X86_EDX) = (int32_t)R(X86_EAX) < 0 ? 0xffffffffu : 0;
        } else {
            reg_set(c, X86_EDX, (R(X86_EAX) & 0x8000) ? 0xffff : 0, 2);
        }
        return;
    case 0x9a: { /* call far */
        uint32_t off = fetch(c, sz);
        uint16_t sel = fetch16(c);
        uint16_t oldcs = c->sel[X86_CS];
        load_seg(c, X86_CS, sel);
        push(c, oldcs, sz);
        push(c, c->eip, sz);
        c->eip = off;
        return;
    }
    case 0x9b: return; /* wait */
    case 0x9c: push(c, c->eflags & 0x00fcffff, sz); return;
    case 0x9d: set_eflags(c, pop(c, sz), sz); return;
    case 0x9e: set_flags(c, X86_CF | X86_PF | X86_AF | X86_ZF | X86_SF, R(X86_EAX) >> 8); return;
    case 0x9f: reg_set(c, 4, (c->eflags & 0xd7) | 2, 1); return;
    case 0xa0:
    case 0xa1:
    case 0xa2:
    case 0xa3: {
        int s = (op & 1) ? sz : 1;
        uint32_t off = d.adsz == 4 ? fetch32(c) : fetch16(c);
        uint32_t lin = c->base[d.seg >= 0 ? d.seg : X86_DS] + off;
        if (op < 0xa2) {
            reg_set(c, X86_EAX, x86_rd(c, lin, s), s);
        } else {
            x86_wr(c, lin, reg_get(c, X86_EAX, s), s);
        }
        return;
    }
    case 0xa8: alu(c, ALU_AND, R(X86_EAX), fetch8(c), 1); return;
    case 0xa9: alu(c, ALU_AND, R(X86_EAX), fetch(c, sz), sz); return;
    case 0xb0 ... 0xb7: reg_set(c, op & 7, fetch8(c), 1); return;
    case 0xb8 ... 0xbf: reg_set(c, op & 7, fetch(c, sz), sz); return;
    case 0xc0:
    case 0xc1:
    case 0xd0:
    case 0xd1:
    case 0xd2:
    case 0xd3: {
        int s = (op & 1) ? sz : 1;
        modrm(c, &d);
        v = rm_read(c, &d, s);
        unsigned cnt = op <= 0xc1 ? fetch8(c) : op <= 0xd1 ? 1 : R(X86_ECX) & 0xff;
        rm_write(c, &d, shift(c, d.reg, v, cnt, s), s);
        return;
    }
    case 0xc2:
        t = fetch16(c);
        c->eip = pop(c, sz);
        R(X86_ESP) += t;
        return;
    case 0xc3:
        c->eip = pop(c, sz);
        return;
    case 0xc4:
    case 0xc5: { /* les/lds */
        modrm(c, &d);
        if (d.mod == 3) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        uint32_t off = x86_rd(c, d.lin, sz);
        uint16_t sel = x86_rd(c, d.lin + sz, 2);
        load_seg(c, op == 0xc4 ? X86_ES : X86_DS, sel);
        reg_set(c, d.reg, off, sz);
        return;
    }
    case 0xc6: modrm(c, &d); rm_write(c, &d, fetch8(c), 1); return;
    case 0xc7: modrm(c, &d); rm_write(c, &d, fetch(c, sz), sz); return;
    case 0xc8: { /* enter */
        uint32_t size = fetch16(c);
        uint8_t level = fetch8(c) & 31;
        push(c, reg_get(c, X86_EBP, sz), sz);
        uint32_t frame = R(X86_ESP);
        for (int i = 1; i < level; i++) {
            uint32_t bp = R(X86_EBP) - i * sz;
            push(c, x86_rd(c, c->base[X86_SS] + bp, sz), sz);
        }
        if (level) {
            push(c, frame, sz);
        }
        reg_set(c, X86_EBP, frame, sz);
        R(X86_ESP) -= size;
        return;
    }
    case 0xc9: /* leave */
        R(X86_ESP) = R(X86_EBP);
        reg_set(c, X86_EBP, pop(c, sz), sz);
        return;
    case 0xca:
    case 0xcb: { /* retf */
        t = op == 0xca ? fetch16(c) : 0;
        uint32_t eip = pop(c, sz);
        uint16_t cs = pop(c, sz);
        load_seg(c, X86_CS, cs);
        c->eip = eip;
        R(X86_ESP) += t;
        return;
    }
    case 0xcc: c->env->intr(c, 3); return;
    case 0xcd: v = fetch8(c); c->env->intr(c, v); return;
    case 0xce:
        if (c->eflags & X86_OF) {
            c->env->intr(c, 4);
        }
        return;
    case 0xcf: { /* iret */
        uint32_t eip = pop(c, sz);
        uint16_t cs = pop(c, sz);
        uint32_t fl = pop(c, sz);
        load_seg(c, X86_CS, cs);
        c->eip = eip;
        set_eflags(c, fl, sz);
        return;
    }
    case 0xd4:
    case 0xd5:
        bcd(c, op, fetch8(c));
        return;
    case 0xd6: reg_set(c, X86_EAX, (c->eflags & X86_CF) ? 0xff : 0, 1); return;
    case 0xd7: { /* xlat */
        uint32_t off = R(X86_EBX) + (R(X86_EAX) & 0xff);
        if (d.adsz == 2) {
            off &= 0xffff;
        }
        reg_set(c, X86_EAX, x86_rd(c, c->base[d.seg >= 0 ? d.seg : X86_DS] + off, 1), 1);
        return;
    }
    case 0xd8 ... 0xdf:
        modrm(c, &d);
        x86_fpu_exec(c, &d, op);
        return;
    case 0xe0:
    case 0xe1:
    case 0xe2:
    case 0xe3: {
        int8_t rel = fetch8(c);
        uint32_t amask = d.adsz == 4 ? 0xffffffffu : 0xffff;
        bool take;
        if (op == 0xe3) {
            take = (R(X86_ECX) & amask) == 0;
        } else {
            R(X86_ECX) = (R(X86_ECX) & ~amask) | ((R(X86_ECX) - 1) & amask);
            take = (R(X86_ECX) & amask) != 0;
            if (op == 0xe0) {
                take = take && !(c->eflags & X86_ZF);
            } else if (op == 0xe1) {
                take = take && (c->eflags & X86_ZF);
            }
        }
        if (take) {
            jump(c, c->eip + rel, sz);
        }
        return;
    }
    case 0xe4: reg_set(c, X86_EAX, c->env->in(c, fetch8(c), 1), 1); return;
    case 0xe5: reg_set(c, X86_EAX, c->env->in(c, fetch8(c), sz), sz); return;
    case 0xe6: v = fetch8(c); c->env->out(c, v, R(X86_EAX) & 0xff, 1); return;
    case 0xe7: v = fetch8(c); c->env->out(c, v, reg_get(c, X86_EAX, sz), sz); return;
    case 0xec: reg_set(c, X86_EAX, c->env->in(c, R(X86_EDX) & 0xffff, 1), 1); return;
    case 0xed: reg_set(c, X86_EAX, c->env->in(c, R(X86_EDX) & 0xffff, sz), sz); return;
    case 0xee: c->env->out(c, R(X86_EDX) & 0xffff, R(X86_EAX) & 0xff, 1); return;
    case 0xef: c->env->out(c, R(X86_EDX) & 0xffff, reg_get(c, X86_EAX, sz), sz); return;
    case 0xe8: {
        int32_t rel = sz == 4 ? (int32_t)fetch32(c) : (int16_t)fetch16(c);
        push(c, c->eip, sz);
        jump(c, c->eip + rel, sz);
        return;
    }
    case 0xe9: {
        int32_t rel = sz == 4 ? (int32_t)fetch32(c) : (int16_t)fetch16(c);
        jump(c, c->eip + rel, sz);
        return;
    }
    case 0xea: {
        uint32_t off = fetch(c, sz);
        load_seg(c, X86_CS, fetch16(c));
        c->eip = off;
        return;
    }
    case 0xeb: {
        int8_t rel = fetch8(c);
        jump(c, c->eip + rel, sz);
        return;
    }
    case 0xf4:
        c->stop_req = true;
        c->exc = -1;
        return;
    case 0xf5: c->eflags ^= X86_CF; return;
    case 0xf6:
    case 0xf7: {
        int s = (op & 1) ? sz : 1;
        modrm(c, &d);
        v = rm_read(c, &d, s);
        switch (d.reg) {
        case 0:
        case 1:
            alu(c, ALU_AND, v, fetch(c, s), s);
            return;
        case 2:
            rm_write(c, &d, ~v, s);
            return;
        case 3:
            rm_write(c, &d, alu(c, ALU_SUB, 0, v, s), s);
            return;
        default:
            mul_div(c, d.reg, v, s);
            return;
        }
    }
    case 0xf8: c->eflags &= ~X86_CF; return;
    case 0xf9: c->eflags |= X86_CF; return;
    case 0xfa: c->eflags &= ~X86_IF; return;
    case 0xfb:
        /* interrupts are recognised only after the next instruction */
        c->sti_shadow = !(c->eflags & X86_IF);
        c->eflags |= X86_IF;
        return;
    case 0xfc: c->eflags &= ~X86_DF; return;
    case 0xfd: c->eflags |= X86_DF; return;
    case 0xfe:
        modrm(c, &d);
        if (d.reg > 1) {
            x86_raise(c, X86_EXC_UD, 0);
        }
        rm_write(c, &d, incdec(c, rm_read(c, &d, 1), d.reg, 1), 1);
        return;
    case 0xff:
        modrm(c, &d);
        switch (d.reg) {
        case 0:
        case 1:
            rm_write(c, &d, incdec(c, rm_read(c, &d, sz), d.reg, sz), sz);
            return;
        case 2:
            v = rm_read(c, &d, sz);
            push(c, c->eip, sz);
            c->eip = v;
            return;
        case 3:
        case 5: {
            if (d.mod == 3) {
                x86_raise(c, X86_EXC_UD, 0);
            }
            uint32_t off = x86_rd(c, d.lin, sz);
            uint16_t sel = x86_rd(c, d.lin + sz, 2);
            uint16_t oldcs = c->sel[X86_CS];
            load_seg(c, X86_CS, sel);
            if (d.reg == 3) {
                push(c, oldcs, sz);
                push(c, c->eip, sz);
            }
            c->eip = off;
            return;
        }
        case 4:
            c->eip = rm_read(c, &d, sz);
            return;
        case 6:
            push(c, rm_read(c, &d, sz), sz);
            return;
        default:
            x86_raise(c, X86_EXC_UD, 0);
        }
    default:
        x86_raise(c, X86_EXC_UD, 0);
    }
}

void x86_set_window(x86_cpu *c, uint8_t *win)
{
    c->win = win;
    c->fast_lo = win ? X86_WIN_BASE + X86_WIN_SIZE : 0;
    c->fast_size = c->mem_size > c->fast_lo ? c->mem_size - c->fast_lo : 0;
}

void x86_init(x86_cpu *c, uint8_t *mem, uint32_t mem_size, const x86_env *env, void *user)
{
    memset(c, 0, sizeof(*c));
    c->mem = mem;
    c->mem_size = mem_size;
    x86_set_window(c, NULL);
    c->env = env;
    c->user = user;
    c->eflags = 0x202;
    c->cr0 = 0x00000011; /* PE, ET */
    x86_fpu_reset(&c->fpu);
}

void x86_request_stop(x86_cpu *c)
{
    c->stop_req = true;
}

x86_stop_t x86_run(x86_cpu *c, uint64_t max_insns)
{
    jmp_buf jb;
    jmp_buf *volatile prev = c->jb;
    uint64_t start = c->icount, end = start + max_insns;
    c->jb = &jb;
    c->stop_req = false;
    c->exc = 0;
    if (setjmp(jb)) {
        c->eip = c->insn_eip;
        c->jb = prev;
        return X86_STOP_EXC;
    }
    while (c->icount < end) {
        if (c->intr_pending && (c->eflags & X86_IF)) {
            if (!c->sti_shadow) {
                c->jb = prev;
                return X86_STOP_IRQ;
            }
        }
        if (c->entry_pages && c->icount != start && !c->sti_shadow) {
            uint32_t o = c->eip - c->entry_lo;
            const uint8_t *b;
            if ((o >> 12) < c->entry_npages && (b = c->entry_pages[o >> 12]) && (b[(o & 4095) >> 3] >> (o & 7) & 1)) {
                c->jb = prev;
                return X86_STOP_ENTRY;
            }
        }
        c->sti_shadow = false;
        c->insn_eip = c->eip;
        step(c);
        c->icount++;
        if (c->stop_req) {
            c->jb = prev;
            if (c->exc == -1) {
                c->exc = 0;
                return X86_STOP_HLT;
            }
            return X86_STOP_REQUEST;
        }
    }
    c->jb = prev;
    return X86_STOP_BUDGET;
}
