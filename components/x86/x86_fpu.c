/* x87 on host doubles. The game uses a handful of x87 instructions (CPU speed calibration, clib), so
 * extended precision is not modelled: values are held as double and converted at the 80-bit load/store
 * boundary. */
#include <math.h>
#include "x86_internal.h"

#define C0 0x0100u
#define C1 0x0200u
#define C2 0x0400u
#define C3 0x4000u

#define ST(i) (f->st[(f->top + (i)) & 7])

void x86_fpu_reset(x86_fpu *f)
{
    memset(f, 0, sizeof(*f));
    f->cw = 0x037f;
    f->tag = 0xff;
}

uint32_t x86_fpu_sw(const x86_cpu *c)
{
    return (c->fpu.sw & ~0x3800u) | ((c->fpu.top & 7u) << 11);
}

static void fpush(x86_fpu *f, double v)
{
    f->top = (f->top - 1) & 7;
    f->st[f->top] = v;
    f->tag &= ~(1u << f->top);
}

static void fpop(x86_fpu *f)
{
    f->tag |= 1u << f->top;
    f->top = (f->top + 1) & 7;
}

static double round_cw(const x86_fpu *f, double v)
{
    switch ((f->cw >> 10) & 3) {
    case 0: return nearbyint(v); /* host default: round to nearest even */
    case 1: return floor(v);
    case 2: return ceil(v);
    default: return trunc(v);
    }
}

static double ld80(const uint8_t *p)
{
    uint64_t mant;
    uint16_t se;
    memcpy(&mant, p, 8);
    memcpy(&se, p + 8, 2);
    int e = se & 0x7fff;
    double v;
    if (e == 0 && mant == 0) {
        v = 0;
    } else if (e == 0x7fff) {
        v = (mant << 1) ? NAN : INFINITY;
    } else {
        v = ldexp((double)mant, e - 16383 - 63);
    }
    return (se & 0x8000) ? -v : v;
}

static void st80(uint8_t *p, double v)
{
    uint64_t mant = 0;
    uint16_t se = signbit(v) ? 0x8000 : 0;
    if (isnan(v)) {
        se |= 0x7fff;
        mant = 0xc000000000000000ull;
    } else if (isinf(v)) {
        se |= 0x7fff;
        mant = 0x8000000000000000ull;
    } else if (v != 0) {
        int e;
        double m = frexp(fabs(v), &e); /* [0.5, 1) */
        mant = (uint64_t)ldexp(m, 64);
        se |= (uint16_t)(e - 1 + 16383);
    }
    memcpy(p, &mant, 8);
    memcpy(p + 8, &se, 2);
}

static void compare(x86_fpu *f, double a, double b)
{
    f->sw &= ~(C0 | C2 | C3);
    if (isnan(a) || isnan(b)) {
        f->sw |= C0 | C2 | C3;
    } else if (a < b) {
        f->sw |= C0;
    } else if (a == b) {
        f->sw |= C3;
    }
}

/* d8/dc/da/de arithmetic: 0 add 1 mul 2 com 3 comp 4 sub 5 subr 6 div 7 divr, as st0 = st0 op src */
static double arith(x86_fpu *f, int op, double a, double b)
{
    switch (op) {
    case 0: return a + b;
    case 1: return a * b;
    case 4: return a - b;
    case 5: return b - a;
    case 6: return a / b;
    case 7: return b / a;
    default: compare(f, a, b); return a;
    }
}

static int64_t to_int(const x86_fpu *f, double v, int bits)
{
    double r = round_cw(f, v);
    double lim = ldexp(1.0, bits - 1);
    if (isnan(r) || r >= lim || r < -lim) {
        return (int64_t)((uint64_t)1 << (bits - 1)) * (bits == 64 ? 1 : -1); /* integer indefinite */
    }
    return (int64_t)r;
}

static double rd_real(x86_cpu *c, uint32_t lin, int bytes)
{
    if (bytes == 4) {
        uint32_t u = x86_rd(c, lin, 4);
        float fl;
        memcpy(&fl, &u, 4);
        return fl;
    }
    uint64_t u = x86_rd(c, lin, 4) | ((uint64_t)x86_rd(c, lin + 4, 4) << 32);
    double d;
    memcpy(&d, &u, 8);
    return d;
}

static void wr_real(x86_cpu *c, uint32_t lin, double v, int bytes)
{
    if (bytes == 4) {
        float fl = (float)v;
        uint32_t u;
        memcpy(&u, &fl, 4);
        x86_wr(c, lin, u, 4);
        return;
    }
    uint64_t u;
    memcpy(&u, &v, 8);
    x86_wr(c, lin, (uint32_t)u, 4);
    x86_wr(c, lin + 4, (uint32_t)(u >> 32), 4);
}

static int64_t rd_int(x86_cpu *c, uint32_t lin, int bytes)
{
    if (bytes == 2) {
        return (int16_t)x86_rd(c, lin, 2);
    }
    if (bytes == 4) {
        return (int32_t)x86_rd(c, lin, 4);
    }
    return (int64_t)(x86_rd(c, lin, 4) | ((uint64_t)x86_rd(c, lin + 4, 4) << 32));
}

static void wr_int(x86_cpu *c, uint32_t lin, int64_t v, int bytes)
{
    if (bytes == 8) {
        x86_wr(c, lin, (uint32_t)v, 4);
        x86_wr(c, lin + 4, (uint32_t)((uint64_t)v >> 32), 4);
    } else {
        x86_wr(c, lin, (uint32_t)v, bytes);
    }
}

/* fstenv/fldenv, 32-bit protected-mode layout (28 bytes) */
static void store_env(x86_cpu *c, uint32_t lin)
{
    x86_fpu *f = &c->fpu;
    uint32_t tagw = 0;
    for (int i = 0; i < 8; i++) {
        tagw |= ((f->tag >> i) & 1 ? 3u : 0u) << (i * 2);
    }
    x86_wr(c, lin, 0xffff0000u | f->cw, 4);
    x86_wr(c, lin + 4, 0xffff0000u | x86_fpu_sw(c), 4);
    x86_wr(c, lin + 8, 0xffff0000u | tagw, 4);
    for (int i = 12; i < 28; i += 4) {
        x86_wr(c, lin + i, 0, 4);
    }
}

static void load_env(x86_cpu *c, uint32_t lin)
{
    x86_fpu *f = &c->fpu;
    f->cw = x86_rd(c, lin, 2);
    uint16_t sw = x86_rd(c, lin + 4, 2);
    f->top = (sw >> 11) & 7;
    f->sw = sw & ~0x3800u;
    uint16_t tagw = x86_rd(c, lin + 8, 2);
    f->tag = 0;
    for (int i = 0; i < 8; i++) {
        if (((tagw >> (i * 2)) & 3) == 3) {
            f->tag |= 1u << i;
        }
    }
}

static void mem_op(x86_cpu *c, x86_insn *d, uint8_t op)
{
    x86_fpu *f = &c->fpu;
    uint32_t lin = d->lin;
    int r = d->reg;
    switch (op) {
    case 0xd8:
        ST(0) = arith(f, r, ST(0), rd_real(c, lin, 4));
        if (r == 3) {
            fpop(f);
        }
        return;
    case 0xdc:
        ST(0) = arith(f, r, ST(0), rd_real(c, lin, 8));
        if (r == 3) {
            fpop(f);
        }
        return;
    case 0xda:
    case 0xde:
        ST(0) = arith(f, r, ST(0), (double)rd_int(c, lin, op == 0xda ? 4 : 2));
        if (r == 3) {
            fpop(f);
        }
        return;
    case 0xd9:
        switch (r) {
        case 0: fpush(f, rd_real(c, lin, 4)); return;
        case 2: wr_real(c, lin, ST(0), 4); return;
        case 3: wr_real(c, lin, ST(0), 4); fpop(f); return;
        case 4: load_env(c, lin); return;
        case 5: f->cw = x86_rd(c, lin, 2); return;
        case 6: store_env(c, lin); return;
        case 7: x86_wr(c, lin, f->cw, 2); return;
        }
        break;
    case 0xdb:
        switch (r) {
        case 0: fpush(f, (double)rd_int(c, lin, 4)); return;
        case 2: wr_int(c, lin, to_int(f, ST(0), 32), 4); return;
        case 3: wr_int(c, lin, to_int(f, ST(0), 32), 4); fpop(f); return;
        case 5: fpush(f, ld80(x86_mp(c, lin, 10))); return;
        case 7: st80(x86_mp(c, lin, 10), ST(0)); fpop(f); return;
        }
        break;
    case 0xdd:
        switch (r) {
        case 0: fpush(f, rd_real(c, lin, 8)); return;
        case 2: wr_real(c, lin, ST(0), 8); return;
        case 3: wr_real(c, lin, ST(0), 8); fpop(f); return;
        case 4: /* frstor */
            load_env(c, lin);
            for (int i = 0; i < 8; i++) {
                ST(i) = ld80(x86_mp(c, lin + 28 + i * 10, 10));
            }
            return;
        case 6: /* fnsave */
            store_env(c, lin);
            for (int i = 0; i < 8; i++) {
                st80(x86_mp(c, lin + 28 + i * 10, 10), ST(i));
            }
            x86_fpu_reset(f);
            return;
        case 7: x86_wr(c, lin, x86_fpu_sw(c), 2); return;
        }
        break;
    case 0xdf:
        switch (r) {
        case 0: fpush(f, (double)rd_int(c, lin, 2)); return;
        case 2: wr_int(c, lin, to_int(f, ST(0), 16), 2); return;
        case 3: wr_int(c, lin, to_int(f, ST(0), 16), 2); fpop(f); return;
        case 5: fpush(f, (double)rd_int(c, lin, 8)); return;
        case 7: wr_int(c, lin, to_int(f, ST(0), 64), 8); fpop(f); return;
        }
        break;
    }
    x86_raise(c, X86_EXC_UD, 0);
}

static void reg_op(x86_cpu *c, x86_insn *d, uint8_t op)
{
    x86_fpu *f = &c->fpu;
    int r = d->reg, i = d->rm;
    double t;
    switch (op) {
    case 0xd8:
        ST(0) = arith(f, r, ST(0), ST(i));
        if (r == 3) {
            fpop(f);
        }
        return;
    case 0xdc:
    case 0xde:
        /* st(i) = st(i) op st0; the sub/div encodings are swapped relative to d8 */
        if (r == 2 || r == 3) {
            compare(f, ST(0), ST(i));
            if (op == 0xde && r == 3 && i == 1) { /* fcompp */
                fpop(f);
                fpop(f);
                return;
            }
            if (r == 3) {
                fpop(f);
            }
            return;
        }
        ST(i) = arith(f, r ^ ((r >= 4) ? 1 : 0), ST(i), ST(0));
        if (op == 0xde) {
            fpop(f);
        }
        return;
    case 0xd9:
        switch (r) {
        case 0: t = ST(i); fpush(f, t); return;
        case 1: t = ST(0); ST(0) = ST(i); ST(i) = t; return;
        case 2: return; /* fnop */
        case 4:
            switch (i) {
            case 0: ST(0) = -ST(0); return;
            case 1: ST(0) = fabs(ST(0)); return;
            case 4: compare(f, ST(0), 0.0); return;
            case 5: { /* fxam */
                double v = ST(0);
                f->sw &= ~(C0 | C1 | C2 | C3);
                if (signbit(v)) {
                    f->sw |= C1;
                }
                if (f->tag & (1u << f->top)) {
                    f->sw |= C3 | C0;
                } else if (isnan(v)) {
                    f->sw |= C0;
                } else if (isinf(v)) {
                    f->sw |= C2 | C0;
                } else if (v == 0) {
                    f->sw |= C3;
                } else {
                    f->sw |= C2;
                }
                return;
            }
            }
            break;
        case 5: {
            static const double k[7] = { 1.0, 3.321928094887362347870, 1.442695040888963407360, 3.141592653589793238463,
                                         0.301029995663981195214, 0.693147180559945309417, 0.0 };
            if (i < 7) {
                fpush(f, k[i]);
                return;
            }
            break;
        }
        case 6:
            switch (i) {
            case 0: ST(0) = exp2(ST(0)) - 1; return;
            case 1: t = ST(1) * log2(ST(0)); fpop(f); ST(0) = t; return;
            case 2: ST(0) = tan(ST(0)); fpush(f, 1.0); f->sw &= ~C2; return;
            case 3: t = atan2(ST(1), ST(0)); fpop(f); ST(0) = t; return;
            case 4: { /* fxtract */
                int e;
                double m = frexp(ST(0), &e);
                ST(0) = e - 1;
                fpush(f, m * 2);
                return;
            }
            case 5: ST(0) = remainder(ST(0), ST(1)); f->sw &= ~C2; return;
            case 6: f->top = (f->top - 1) & 7; return;
            default: f->top = (f->top + 1) & 7; return;
            }
        case 7:
            switch (i) {
            case 0: ST(0) = fmod(ST(0), ST(1)); f->sw &= ~C2; return;
            case 1: t = ST(1) * log2(ST(0) + 1); fpop(f); ST(0) = t; return;
            case 2: ST(0) = sqrt(ST(0)); return;
            case 3: t = ST(0); ST(0) = sin(t); fpush(f, cos(t)); f->sw &= ~C2; return;
            case 4: ST(0) = round_cw(f, ST(0)); return;
            case 5: ST(0) = ldexp(ST(0), (int)trunc(ST(1))); return;
            case 6: ST(0) = sin(ST(0)); f->sw &= ~C2; return;
            default: ST(0) = cos(ST(0)); f->sw &= ~C2; return;
            }
        }
        break;
    case 0xda:
        if (r == 5 && i == 1) { /* fucompp */
            compare(f, ST(0), ST(1));
            fpop(f);
            fpop(f);
            return;
        }
        break;
    case 0xdb:
        if (r == 4) {
            switch (i) {
            case 0: case 1: case 4: return; /* feni/fdisi/fsetpm */
            case 2: f->sw &= 0x7f00; return; /* fnclex */
            case 3: x86_fpu_reset(f); return; /* fninit */
            }
        }
        break;
    case 0xdd:
        switch (r) {
        case 0: f->tag |= 1u << ((f->top + i) & 7); return; /* ffree */
        case 2: ST(i) = ST(0); return;
        case 3: ST(i) = ST(0); fpop(f); return;
        case 4: compare(f, ST(0), ST(i)); return;
        case 5: compare(f, ST(0), ST(i)); fpop(f); return;
        }
        break;
    case 0xdf:
        if (r == 4 && i == 0) { /* fnstsw ax */
            c->r[X86_EAX] = (c->r[X86_EAX] & 0xffff0000u) | x86_fpu_sw(c);
            return;
        }
        break;
    }
    x86_raise(c, X86_EXC_UD, 0);
}

void x86_fpu_exec(x86_cpu *c, x86_insn *d, uint8_t op)
{
    if (d->mod == 3) {
        reg_op(c, d, op);
    } else {
        mem_op(c, d, op);
    }
}
