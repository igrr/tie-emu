"""Lowering of x86 functions to C (against components/aot/include/aot_rt.h).

A function becomes one C function. Blocks end at branches, calls and at instructions left to the
interpreter (punts), so the instruction after any of those starts a block. The blocks where the
dispatcher enters the function are numbered entries of it (the switch at the top, and the entry
table): where it resumes after an exit (Func.own_entries) and the targets of branches from other
functions. Elsewhere the interpreter runs on to the next entry.

Flags are computed where they can be observed. A backward liveness pass over each function finds
the arithmetic flags live after every instruction; exits to the dispatcher count as reading all of
them, a call reads what its callee reads before writing, and a `ret` what is live after the calls
that can return there (aotgen.py, RetLiveness). Inside a block the last flag-setting ALU operation
stays pending (its operands in temporaries): a jcc/setcc/adc/sbb that only needs its flags
evaluates the condition from the operands, and the flags are written to EFL only when they are
live at the end of the block or something reads EFL itself.
"""
from capstone import x86 as X

from discover import JCC

CF, PF, AF, ZF, SF, OF = 0x1, 0x4, 0x10, 0x40, 0x80, 0x800
ALL = CF | PF | AF | ZF | SF | OF
CC_USE = [OF, OF, CF, CF, ZF, ZF, CF | ZF, CF | ZF, SF, SF, PF, PF, SF | OF, SF | OF, ZF | SF | OF, ZF | SF | OF]
SETCC = {'set' + k[1:]: v for k, v in JCC.items()}

NAMES = ['EAX', 'ECX', 'EDX', 'EBX', 'ESP', 'EBP', 'ESI', 'EDI']
REGS = {}
for n, nm in enumerate(NAMES):
    REGS[getattr(X, 'X86_REG_' + nm)] = (n, 4, False)
    REGS[getattr(X, 'X86_REG_' + nm[1:])] = (n, 2, False)
for n, nm in enumerate(['A', 'C', 'D', 'B']):
    REGS[getattr(X, 'X86_REG_%sL' % nm)] = (n, 1, False)
    REGS[getattr(X, 'X86_REG_%sH' % nm)] = (n, 1, True)
FLAT_SEGS = {X.X86_REG_INVALID, X.X86_REG_DS, X.X86_REG_ES, X.X86_REG_SS, X.X86_REG_CS}
# FS and GS: accessed at the offset plus the segment's base, which may be anything
BASED_SEGS = {X.X86_REG_FS: 4, X.X86_REG_GS: 5}
SEGS = {X.X86_REG_ES: 0, X.X86_REG_CS: 1, X.X86_REG_SS: 2, X.X86_REG_DS: 3, X.X86_REG_FS: 4, X.X86_REG_GS: 5}

ALU = {'add': 'add', 'adc': 'add', 'sub': 'sub', 'sbb': 'sub', 'cmp': 'sub', 'and': 'logic', 'or': 'logic',
       'xor': 'logic', 'test': 'logic'}
ALU_OP = {'add': '+', 'adc': '+', 'sub': '-', 'sbb': '-', 'cmp': '-', 'and': '&', 'or': '|', 'xor': '^', 'test': '&'}
SHIFT = {'rol': 0, 'ror': 1, 'rcl': 2, 'rcr': 3, 'shl': 4, 'sal': 4, 'shr': 5, 'sar': 7}
STRING = {'movs': 0xa4, 'cmps': 0xa6, 'stos': 0xaa, 'lods': 0xac, 'scas': 0xae}
PLAIN = {'mov', 'movzx', 'movsx', 'lea', 'push', 'pop', 'xchg', 'not', 'cwde', 'cbw', 'cdq', 'cwd', 'nop',
         'cld', 'std', 'leave', 'pushal', 'popal'}


STACK_MARGIN = 4096     # aot_rt.h, AOT_STACK_MARGIN
SMALL_ESP_STEP = 256    # add/sub esp, imm below this leave ESP unchecked
RET_ROOM = 64           # aot_rt.h, AOT_RET_ROOM
# What EBP holds (Func.frame_states): not known, a frame pointer that may have been changed since
# it was checked (after a call), a frame pointer in the stack range
FP_NONE, FP_UNCHECKED, FP_OK = 0, 1, 2


def signed(v):
    v &= 0xffffffff
    return v - (1 << 32) if v & 0x80000000 else v


def mask(sz):
    return (1 << (8 * sz)) - 1


def hexu(v):
    return '0x%xu' % (v & 0xffffffff)


def mnemonic(i):
    """Mnemonic without the notrack/rep prefixes, and the rep prefix byte"""
    m = i.mnemonic
    if m.startswith('notrack '):
        m = m[8:]
    for p in ('rep ', 'repe ', 'repne '):
        if m.startswith(p):
            m = m[len(p):]
    rep = 0
    for b in prefixes(i):
        if b in (0xf2, 0xf3):
            rep = b
    return m, rep


def prefixes(i):
    """Legacy prefix bytes, from the encoding: capstone's prefix array misses F2 on F2 A5 (it
    reads it as SSE movsd)"""
    n = 0
    while n < len(i.bytes) and i.bytes[n] in (0xf0, 0xf2, 0xf3, 0x26, 0x2e, 0x36, 0x3e, 0x64, 0x65, 0x66, 0x67):
        n += 1
    return i.bytes[:n]


def string_kind(m):
    """(kind, size) of a string instruction mnemonic, or None"""
    k = STRING.get(m[:4])
    if k is None or len(m) != 5 or m[4] not in 'bwd':
        return None
    return k, {'b': 1, 'w': 2, 'd': 4}[m[4]]


def opsize16(i):
    return 0x66 in prefixes(i)


class Insn:
    """What the translator knows about one instruction: its class and flag effect.

    cls: plain, alu (pending-flag setter: add/sub/logic/inc/dec/neg), cond (reads a condition),
    efl (reads or writes EFL through a helper), port (IN, OUT), int (INT n), and the terminators: jcc, jmp, ijmp, table,
    call, icall, ret, punt. uses/defs are arithmetic flag masks; defs only counts flags written
    on every path. pdefs are the flags written on some path (shift by cl, rep cmps)."""

    def __init__(self, prog, i):
        self.i = i
        self.addr = i.address
        self.next = i.address + i.size
        self.m, self.rep = mnemonic(i)
        self.uses = self.defs = self.pdefs = 0
        self.cc = None
        self.target = None
        self.segload = False
        self.cls = self._classify(prog)
        self.pdefs |= self.defs

    def _mem_ok(self):
        i = self.i
        if 0x67 in prefixes(i):
            return False
        for op in i.operands:
            if op.type == X.X86_OP_MEM and op.mem.segment not in FLAT_SEGS and (
                    op.mem.segment not in BASED_SEGS or string_kind(self.m)):
                return False
        return True

    def _classify(self, prog):
        i, m = self.i, self.m
        ops = i.operands
        if not self._mem_ok():
            return 'punt'
        if i.opcode[0] == 0xff and (i.modrm >> 3) & 7 in (3, 5):
            return 'punt'   # far call/jmp through memory: capstone names them call/jmp
        if m in JCC:
            if opsize16(i):
                return 'punt'
            self.cc = JCC[m]
            self.uses = CC_USE[self.cc]
            self.target = ops[0].imm & 0xffffffff
            return 'jcc'
        if m in ('loop', 'loope', 'loopne', 'jecxz'):
            if opsize16(i):
                return 'punt'
            self.target = ops[0].imm & 0xffffffff
            if m in ('loope', 'loopne'):
                self.uses = ZF
            return 'jcc'
        if m == 'jmp':
            if opsize16(i):
                return 'punt'
            if ops[0].type == X.X86_OP_IMM:
                self.target = ops[0].imm & 0xffffffff
                return 'jmp'
            if i.address in prog.tables:
                return 'table'
            return 'ijmp'
        if m == 'call':
            if opsize16(i):
                return 'punt'
            if ops[0].type == X.X86_OP_IMM:
                self.target = ops[0].imm & 0xffffffff
                return 'call'
            return 'icall'
        if m == 'ret':
            if opsize16(i):
                return 'punt'
            return 'ret'
        if m in ALU:
            if m in ('adc', 'sbb'):
                self.uses = CF
            self.defs = ALL
            return 'alu'
        if m in ('inc', 'dec'):
            self.defs = ALL & ~CF
            return 'alu'
        if m == 'neg':
            self.defs = ALL
            return 'alu'
        if m in SETCC:
            self.cc = SETCC[m]
            self.uses = CC_USE[self.cc]
            return 'cond'
        if m in SHIFT:
            op = SHIFT[m]
            pd = CF | OF if op < 4 else ALL
            if op in (2, 3):
                self.uses = CF
            if ops[1].type == X.X86_OP_IMM:
                if ops[1].imm & 31:
                    self.defs = pd
            else:
                self.pdefs = pd
            return 'efl'
        if m in ('shld', 'shrd'):
            if ops[2].type == X.X86_OP_IMM:
                if ops[2].imm & 31:
                    self.defs = ALL
            else:
                self.pdefs = ALL
            return 'efl'
        if m in ('imul', 'mul'):
            self.defs = ALL
            return 'efl'
        if m in ('div', 'idiv'):
            return 'efl'
        if m in ('clc', 'stc'):
            self.defs = CF
            return 'efl'
        if m == 'cmc':
            self.uses = self.defs = CF
            return 'efl'
        if m == 'sahf':
            self.defs = CF | PF | AF | ZF | SF
            return 'efl'
        if m == 'lahf':
            self.uses = CF | PF | AF | ZF | SF
            return 'efl'
        sk = string_kind(m)
        if sk:
            if sk[0] in (0xa6, 0xae):
                if self.rep:
                    self.pdefs = ALL
                else:
                    self.defs = ALL
            return 'efl'
        if m in ('in', 'out'):
            return 'port'
        if m == 'int' and i.bytes[0] == 0xcd:
            return 'int'
        if m in ('push', 'pop', 'mov') and any(o.type == X.X86_OP_REG and o.reg in SEGS for o in ops):
            if m == 'push':
                return 'seg'
            if ops[0].type != X.X86_OP_REG or ops[0].reg not in SEGS:
                return 'seg'        # mov r/m, sreg
            if ops[0].reg == X.X86_REG_CS:
                return 'punt'
            self.segload = True     # pop sreg, mov sreg, r/m: may leave (a bad selector, a segment not flat)
            return 'seg'
        if m in PLAIN:
            if m in ('push', 'pop') and ops[0].type == X.X86_OP_REG and ops[0].reg not in REGS:
                return 'punt'   # segment registers
            if m in ('mov', 'xchg') and any(o.type == X.X86_OP_REG and o.reg not in REGS for o in ops):
                return 'punt'
            if m in ('leave', 'pushal', 'popal') and opsize16(i):
                return 'punt'
            return 'plain'
        return 'punt'


def sets_esp(ins):
    """Whether ins sets ESP in a way translated code checks after (aot_rt.h, AOT_ESP_OK): other
    than by push, pop, call, ret or a small constant add or sub. `leave` checks before."""
    if ins.cls not in ('plain', 'alu', 'efl'):
        return False
    m, ops = ins.m, ins.i.operands
    if m in ('push', 'leave') or not ops:
        return False
    if m == 'xchg':
        return any(o.type == X.X86_OP_REG and o.reg in (X.X86_REG_ESP, X.X86_REG_SP) for o in ops)
    d = ops[0]
    if d.type != X.X86_OP_REG or d.reg not in (X.X86_REG_ESP, X.X86_REG_SP) or m in ('cmp', 'test'):
        return False
    if (m in ('add', 'sub') and d.reg == X.X86_REG_ESP and ops[1].type == X.X86_OP_IMM
            and abs(signed(ops[1].imm)) < SMALL_ESP_STEP):
        return False
    return True


def writes_ebp(ins):
    return any(r in (X.X86_REG_EBP, X.X86_REG_BP) for r in ins.i.regs_access()[1])


def frame_operand(ins):
    """Whether ins accesses memory at [EBP + d] with d small (Func.frame_states)"""
    return ins.m not in ('lea', 'nop') and any(
        o.type == X.X86_OP_MEM and o.mem.base == X.X86_REG_EBP and not o.mem.index and o.mem.segment in FLAT_SEGS
        and abs(signed(o.mem.disp)) < STACK_MARGIN for o in ins.i.operands)


def is_term(ins):
    return ins.cls in ('jcc', 'jmp', 'ijmp', 'table', 'call', 'icall', 'ret', 'punt')


class Func:
    """Blocks of one function and their flag liveness"""

    def __init__(self, prog, start, insns):
        self.prog = prog
        self.start = start
        body = prog.body(start)
        self.ins = {a: insns[a] for a in body}
        leaders = {start}
        for a, ins in self.ins.items():
            if is_term(ins) or ins.cls in ('port', 'int'):
                # after a port access or INT: where translated code resumes when the machine stops the CPU
                leaders.add(ins.next)
            for t in self.targets(ins):
                leaders.add(t)
        self.blocks = {}
        for L in sorted(x for x in leaders if x in self.ins):
            b = []
            a = L
            while True:
                ins = self.ins[a]
                b.append(ins)
                if is_term(ins) or ins.cls == 'punt':
                    break
                a = ins.next
                if a in leaders or a not in self.ins:
                    break
            self.blocks[L] = b
        self.ret_live = ALL     # flags live where its rets go (aotgen.py)
        self.ret_jumps = self.pushed_rets()
        self.budget_checks()
        self.entries = self.own_entries()
        self.fp_in = self.frame_states()

    def budget_checks(self):
        """Blocks that check the slice budget, and how much they need.

        A block checks at the function start and where an edge comes from a block at the same or
        a higher address (loop heads), so the edges between the other blocks go forward and reach
        a check within a bounded number of instructions. A call ends a run: the callee checks at
        its start, and its ret leaves unless RET_ROOM instructions are left (AOT_RET), which is
        enough for what the caller runs after it to its next check, or else the block after the
        call checks too. The block after an INT checks: the machine charges instructions for the
        service (aot_int). A checking block needs its own count plus the longest run of
        instructions through blocks that do not check; the others only count. Entering at a
        block that does not check (from the dispatcher) checks what it needs there."""
        check = {self.start}
        for L in self.blocks:
            for S in self.succs(L):
                if S <= L:
                    check.add(S)
        for L, b in self.blocks.items():
            if b[-1].cls == 'int' and b[-1].next in self.blocks:
                check.add(b[-1].next)
        need = self.needs(check)
        for L, b in self.blocks.items():
            if b[-1].cls in ('call', 'icall') and need.get(b[-1].next, 0) > RET_ROOM:
                check.add(b[-1].next)
        self.check = check
        self.need = need = self.needs(check)
        # checks after a call that the call makes, when the call is the only way into the block but
        # the dispatcher
        preds = {}
        for L in self.blocks:
            for S in self.succs(L):
                preds.setdefault(S, set()).add(L)
        self.call_check = set()
        for L, b in self.blocks.items():
            nxt = b[-1].next
            if b[-1].cls in ('call', 'icall') and nxt in check and nxt != self.start and preds.get(nxt) == {L}:
                self.call_check.add(nxt)

    def poll_loop(self, L):
        """(mask, nz, len) when block L is the top of `L: in al, dx/imm8; test/and al, imm8;
        jz/jnz L`, which reads the port until (AL & mask) != 0 is no longer nz, and only L checks
        the budget in it (aot_rt.h, aot_in_poll); None otherwise"""
        i1 = self.ins[L]
        ops = i1.i.operands
        if (L not in self.check or L in self.call_check or i1.m != 'in' or ops[0].type != X.X86_OP_REG
                or ops[0].reg != X.X86_REG_AL):
            return None
        i2 = self.ins.get(i1.next)
        i3 = i2 and self.ins.get(i2.next)
        if not i3 or i2.m not in ('test', 'and') or i3.m not in ('je', 'jne') or i3.target != L:
            return None
        o = i2.i.operands
        if (o[0].type != X.X86_OP_REG or o[0].reg != X.X86_REG_AL or o[1].type != X.X86_OP_IMM
                or any(a in self.check for a in (i2.addr, i3.addr))):
            return None
        return o[1].imm & 0xff, int(i3.m == 'jne'), 3

    def needs(self, check):
        """The longest run of instructions from each block to a check, a call or a ret"""
        need = {}
        for L in sorted(self.blocks, reverse=True):
            b = self.blocks[L]
            nxt = [] if b[-1].cls in ('call', 'icall') else [need[S] for S in self.succs(L) if S not in check]
            need[L] = len(b) + max(nxt or [0])
        return need

    def own_entries(self):
        """Blocks the dispatcher enters this function at, as far as the function itself tells:
        its start, the blocks that check the budget (where it leaves when the slice runs out),
        where a call returns (a callee that left returns through the dispatcher) and where it
        resumes after an instruction left to the interpreter or a port access. Elsewhere the
        interpreter runs on to one of them. aotgen adds the targets of branches from other
        functions."""
        e = {self.start} | self.check
        for b in self.blocks.values():
            if b[-1].cls in ('call', 'icall') and b[-1].next in self.blocks:
                e.add(b[-1].next)
            for ins in b:
                if (ins.cls in ('punt', 'port', 'int') or ins.segload) and ins.next in self.blocks:
                    e.add(ins.next)
        return e

    def frame_states(self):
        """What EBP holds at the start of each block (FP_*), on every path from the function's
        start. EBP is a frame pointer after `mov ebp, esp`: ESP is in the stack range then, so
        [EBP + d] is in memory as [ESP + d] is (aot_rt.h, AOT_ESP_OK). What a call, an INT or the
        interpreter did to it is checked before the next access (Emitter.block), and an entry from
        the dispatcher checks it too."""
        at = {self.start: FP_NONE}
        work = [self.start]
        while work:
            L = work.pop()
            s = at[L]
            for ins in self.blocks[L]:
                s = self.fp_step(ins, s)
            for S in self.succs(L):
                if S not in at or s < at[S]:
                    at[S] = min(s, at.get(S, s))
                    work.append(S)
        return {L: at.get(L, FP_NONE) for L in self.blocks}

    @staticmethod
    def fp_step(ins, s):
        """What EBP holds after ins, from s before it"""
        if s == FP_OK and ins.cls in ('call', 'icall', 'int', 'punt'):
            s = FP_UNCHECKED
        if frame_operand(ins) and s == FP_UNCHECKED:
            s = FP_OK       # checked before it
        if writes_ebp(ins):
            ops = ins.i.operands
            s = FP_OK if (ins.m == 'mov' and ops[0].type == X.X86_OP_REG and ops[0].reg == X.X86_REG_EBP and
                          ops[1].type == X.X86_OP_REG and ops[1].reg == X.X86_REG_ESP) else FP_NONE
        return s

    def pushed_rets(self):
        """The rets that pop an address the function pushed itself (push target; ret: a jump),
        from ESP relative to the function's start where it is the same on every path to a block"""
        at = {self.start: 0}
        work = [self.start]
        while work:
            L = work.pop()
            d = at[L]
            for ins in self.blocks[L]:
                d = self.esp_step(ins, d)
            for S in self.succs(L):
                if S not in at:
                    at[S] = d
                    work.append(S)
                elif at[S] is not None and at[S] != d:
                    at[S] = None
                    work.append(S)
        out = set()
        for L, b in self.blocks.items():
            if b[-1].cls != 'ret' or at.get(L) is None:
                continue
            d = at[L]
            for ins in b[:-1]:
                d = self.esp_step(ins, d)
            if d is not None and d < 0:
                out.add(b[-1].addr)
        return out

    def esp_step(self, ins, d):
        """ESP relative to the function's start after ins, from d before it (None: not known)"""
        if d is None:
            return None
        m, ops = ins.m, ins.i.operands
        sz = 2 if opsize16(ins.i) else 4
        if ins.cls == 'call':
            # a callee that pops its arguments (ret n) moves ESP
            f = self.prog.ret_pops.get(ins.target)
            return d if f == 0 else None
        if ins.cls in ('jcc', 'jmp', 'table', 'int', 'port', 'cond', 'alu') and not sets_esp(ins):
            if ins.cls == 'alu' and ops and ops[0].type == X.X86_OP_REG and ops[0].reg == X.X86_REG_ESP:
                if m in ('add', 'sub') and ops[1].type == X.X86_OP_IMM:
                    return d + signed(ops[1].imm) * (1 if m == 'add' else -1)
                if m not in ('cmp', 'test'):
                    return None
            return d
        if ins.cls == 'plain' and not sets_esp(ins):
            if m == 'push':
                return d - sz
            if m == 'pop':
                return None if ops[0].type == X.X86_OP_REG and REGS.get(ops[0].reg, (0,))[0] == 4 else d + sz
            if m == 'pushal':
                return d - 32
            if m == 'popal':
                return d + 32
            if m == 'leave':
                return None
            return d
        if ins.cls == 'seg':
            if m == 'push':
                return d - sz
            if m == 'pop':
                return d + sz
            return d
        if ins.cls == 'efl' and not sets_esp(ins):
            return d
        return None

    def leaves_to(self):
        """Code addresses outside this function its branches and fall-throughs leave to"""
        out = set()
        for L, b in self.blocks.items():
            last = b[-1]
            if last.cls in ('jcc', 'jmp') and last.target not in self.ins and not (
                    last.cls == 'jmp' and self.prog.is_tail(self.start, last.i, last.target)
                    and last.target in self.prog.funcs):
                out.add(last.target)
            if last.cls not in ('jmp', 'ijmp', 'table', 'ret') and last.next not in self.ins:
                out.add(last.next)
        return out

    def targets(self, ins):
        """Intra-function branch targets"""
        if ins.cls == 'jcc' or (ins.cls == 'jmp' and not self.prog.is_tail(self.start, ins.i, ins.target)):
            return [ins.target] if ins.target in self.ins else []
        if ins.cls == 'table':
            return [t for t in self.prog.tables[ins.addr] if t in self.ins]
        return []

    def succs(self, L):
        """Successor blocks of block L"""
        last = self.blocks[L][-1]
        s = set(self.targets(last))
        falls = last.cls not in ('jmp', 'ijmp', 'table', 'ret') and not (
            last.cls == 'punt' and last.m in ('hlt', 'iretd', 'iret', 'retf', 'ljmp', 'ud2', 'int3'))
        if falls and last.next in self.blocks:
            s.add(last.next)
        return s

    def exit_uses(self, L, live_in_funcs):
        """Flags read by leaving the function from the end of block L"""
        last = self.blocks[L][-1]
        c = last.cls
        if c == 'ret':
            return ALL if last.addr in self.ret_jumps else self.ret_live
        if c == 'ijmp' or c == 'table':
            return ALL
        if c == 'jmp' and self.prog.is_tail(self.start, last.i, last.target):
            return live_in_funcs.get(last.target, ALL)
        if c in ('jcc', 'jmp') and last.target not in self.ins:
            return ALL
        if last.next not in self.ins and c not in ('jmp', 'ijmp', 'table', 'ret'):
            return ALL      # falls into something that did not decode
        return 0

    def insn_uses(self, ins, live_in_funcs):
        if ins.cls == 'call':
            return live_in_funcs.get(ins.target, ALL)
        if ins.cls in ('icall', 'punt', 'port', 'int') or ins.segload:
            return ALL      # may leave to the dispatcher
        return ins.uses

    def liveness(self, live_in_funcs):
        """Sets self.live_out[L] and self.live_after[addr]; returns the flags live at entry"""
        succs = {L: self.succs(L) for L in self.blocks}
        exit_uses = {L: self.exit_uses(L, live_in_funcs) for L in self.blocks}
        live_in = {L: 0 for L in self.blocks}
        live_out = {}
        changed = True
        while changed:
            changed = False
            for L in sorted(self.blocks, reverse=True):
                out = exit_uses[L]
                for s in succs[L]:
                    out |= live_in[s]
                live_out[L] = out
                live = out
                for ins in reversed(self.blocks[L]):
                    live = (live & ~ins.defs) | self.insn_uses(ins, live_in_funcs)
                if live != live_in[L]:
                    live_in[L] = live
                    changed = True
        self.live_out = live_out
        self.live_after = {}
        for L, b in self.blocks.items():
            live = live_out[L]
            for ins in reversed(b):
                self.live_after[ins.addr] = live
                live = (live & ~ins.defs) | self.insn_uses(ins, live_in_funcs)
        return live_in[self.start]


class Pend:
    """A flag-setting ALU operation whose flags have not been written to EFL"""

    def __init__(self, kind, sz, a, b, r, cin, defs):
        self.kind, self.sz, self.a, self.b, self.r, self.cin, self.defs = kind, sz, a, b, r, cin, defs
        self.stored = False

    def flags_expr(self):
        k, sz, a, b, r, cin = self.kind, self.sz, self.a, self.b, self.r, self.cin
        if k == 'add':
            return 'aot_fl_add(%s, %s, %s, %s, %d)' % (a, b, r, cin, sz)
        if k == 'sub':
            return 'aot_fl_sub(%s, %s, %s, %s, %d)' % (a, b, r, cin, sz)
        if k == 'logic':
            return 'aot_fl_logic(%s, %d)' % (r, sz)
        return 'aot_fl_%s(%s, %s, %d)' % (k, a, r, sz)

    def store(self):
        """EFL = EFL with the operation's flags (aot_rt.h, AOT_FL_STORES)"""
        k, sz, a, b, r, cin = self.kind, self.sz, self.a, self.b, self.r, self.cin
        if k in ('add', 'sub'):
            if cin == '0':
                return 'EFL = aot_st_%s%d(EFL, %s, %s, %s);' % (k, sz, a, b, r)
            return 'EFL = aot_st_%s%d(EFL, %s, %s, %s, %s);' % ('adc' if k == 'add' else 'sbb', sz, a, b, r, cin)
        if k == 'logic':
            return 'EFL = aot_st_logic%d(EFL, %s);' % (sz, r)
        return 'EFL = aot_st_%s%d(EFL, %s, %s);' % (k, sz, a, r)

    def cond(self, cc):
        """C condition for cc from the operands, or None"""
        k, a, b, r = self.kind, self.a, self.b, self.r
        sign = hexu(1 << (8 * self.sz - 1))
        sx = {1: '(int8_t)', 2: '(int16_t)', 4: '(int32_t)'}[self.sz]
        base = cc >> 1
        e = None
        neg = '(%s & %s) != 0' % (r, sign)
        if k == 'sub' and self.cin == '0':
            e = {1: '%s < %s' % (a, b), 2: '%s == 0' % r, 3: '%s <= %s' % (a, b), 4: neg,
                 6: '%s%s < %s%s' % (sx, a, sx, b), 7: '%s%s <= %s%s' % (sx, a, sx, b)}.get(base)
        elif k == 'logic':
            e = {0: '0', 1: '0', 2: '%s == 0' % r, 3: '%s == 0' % r, 4: neg, 6: neg,
                 7: '%s == 0 || %s' % (r, neg)}.get(base)
        elif k == 'add' and self.cin == '0':
            e = {1: '%s < %s' % (r, a), 2: '%s == 0' % r, 4: neg}.get(base)
        elif k in ('inc', 'dec'):
            e = {2: '%s == 0' % r, 4: neg}.get(base)
        if e is None:
            return None
        return '!(%s)' % e if cc & 1 else '(%s)' % e


class Operand:
    """C access to an instruction operand. A memory operand at a constant address in `direct` (the
    image's range, which the dispatcher checks is ordinary memory) is accessed without a check, and
    so is one near ESP (aot_rt.h, AOT_ESP_OK), or near EBP when it is a checked frame pointer
    (Func.frame_states)."""

    def __init__(self, op, sz=None, direct=None, frame=False):
        self.op = op
        self.sz = sz or op.size
        self.addr = None
        self.direct = False
        m = op.mem if op.type == X.X86_OP_MEM else None
        self.seg = BASED_SEGS.get(m.segment) if m else None
        if self.seg is not None:
            return
        if direct and m and not m.base and not m.index:
            a = m.disp & 0xffffffff
            self.direct = direct[0] <= a and a + self.sz <= direct[1]
        if m and (m.base == X.X86_REG_ESP or (frame and m.base == X.X86_REG_EBP)) and not m.index:
            self.direct = abs(signed(m.disp)) < STACK_MARGIN

    def prepare(self, out, tmp):
        """Computes a memory operand's address into a temporary"""
        if self.op.type == X.X86_OP_MEM:
            self.addr = tmp
            a = address(self.op)
            if self.seg is not None:
                a = '%s + aot_cpu->base[%d]' % (a, self.seg)
            out.append('uint32_t %s = %s;' % (tmp, a))

    def read(self, sz=None):
        op = self.op
        sz = sz or self.sz
        if op.type == X.X86_OP_IMM:
            return hexu(op.imm & mask(sz))
        if op.type == X.X86_OP_REG:
            n, rsz, hi = REGS[op.reg]
            if rsz == 4:
                return NAMES[n]
            if rsz == 2:
                return '(%s & 0xffffu)' % NAMES[n]
            return '((%s >> 8) & 0xffu)' % NAMES[n] if hi else '(%s & 0xffu)' % NAMES[n]
        return '%s%d(%s)' % ('RDD' if self.direct else 'RD', 8 * sz, self.addr)

    def write(self, v):
        op = self.op
        if op.type == X.X86_OP_REG:
            n, rsz, hi = REGS[op.reg]
            r = NAMES[n]
            if rsz == 4:
                return '%s = %s;' % (r, v)
            if rsz == 2:
                return '%s = (%s & 0xffff0000u) | ((%s) & 0xffffu);' % (r, r, v)
            if hi:
                return '%s = (%s & 0xffff00ffu) | (((%s) & 0xffu) << 8);' % (r, r, v)
            return '%s = (%s & 0xffffff00u) | ((%s) & 0xffu);' % (r, r, v)
        return '%s%d(%s, %s);' % ('WRD' if self.direct else 'WR', 8 * self.sz, self.addr, v)


def address(op):
    m = op.mem
    parts = []
    if m.base:
        parts.append(NAMES[REGS[m.base][0]])
    if m.index:
        parts.append('%s * %d' % (NAMES[REGS[m.index][0]], m.scale) if m.scale != 1 else NAMES[REGS[m.index][0]])
    if m.disp or not parts:
        parts.append(hexu(m.disp))
    return ' + '.join(parts) if len(parts) > 1 else '(uint32_t)%s' % parts[0]


def fname(a):
    return 'F_%06x' % a


def entry_list(f):
    """The function's entries in the order of their numbers (from 1)"""
    return sorted(e for e in f.entries if e in f.blocks)


def entry_need(f, L):
    """What the dispatcher checks before entering at block L: nothing where the block checks"""
    return 0 if L in f.check and L not in f.call_check else f.need[L]


class Emitter:
    """C for one function"""

    def __init__(self, func, known, live_in_funcs):
        self.f = func
        self.known = known                  # translated function starts
        self.live_in_funcs = live_in_funcs  # flags each function reads before writing
        self.direct = func.prog.img.direct

    def opnd(self, op, sz=None):
        return Operand(op, sz, self.direct, self.fp == FP_OK)

    def emit(self):
        f = self.f
        out = ['uint32_t %s(uint32_t entry, uint32_t aot_efl, uint32_t aot_next)' % fname(f.start), '{',
               '    AOT_STATE;',
               '    if (entry) {', '        switch (entry) {']
        # entry k is the k-th of the sorted entries (aot_entry_t.idx); the dispatcher checks the
        # budget an entry needs (aot_entry_t.need)
        for k, L in enumerate(entry_list(f)):
            if f.fp_in[L] == FP_OK:
                # where the dispatcher enters with EBP a checked frame pointer: EBP is in the stack
                # range, or the interpreter goes on (FP_UNCHECKED: the block checks it)
                out.append('        case %d: if (AOT_UNLIKELY(!AOT_ESP_OK(EBP))) AOT_EXIT(%s, AOT_EXIT_PUNT); goto L_%x;'
                           % (k + 1, hexu(L), L))
                continue
            out.append('        case %d: goto L_%x;' % (k + 1, L))
        out += ['        default: __builtin_unreachable();', '        }', '    }',
                '    AOT_FUNC(%s);' % hexu(f.start)]
        order = sorted(f.blocks)
        if order[0] != f.start:
            out.append('    goto L_%x;' % f.start)
        for k, L in enumerate(order):
            nxt = order[k + 1] if k + 1 < len(order) else None
            out.append('L_%x: {' % L)
            out += ['    ' + s for s in self.block(L, nxt)]
            out.append('}')
        out.append('    AOT_EXIT_PATH;')
        out.append('}')
        return out

    # -- one block --

    def block(self, L, nxt):
        f = self.f
        b = f.blocks[L]
        self.n = len(b)
        if L in f.check and L not in f.call_check:
            self.out = out = ['AOT_BLOCK(%d, %d, %s);' % (f.need[L], self.n, hexu(L))]
        else:
            self.out = out = ['aot_bud -= %d;' % self.n]
        self.pend = None
        self.live_out = f.live_out[L]
        self.fp = f.fp_in[L]
        for k, ins in enumerate(b):
            self.k = k
            self.t = 't%d_' % k
            la = f.live_after[ins.addr]
            lb = (la & ~ins.defs) | f.insn_uses(ins, self.live_in_funcs)
            if self.fp == FP_UNCHECKED and frame_operand(ins):
                # EBP after a call: the frame pointer it was, in the stack range, or leave
                p = self.pend
                out.append('if (AOT_UNLIKELY(!AOT_ESP_OK(EBP))) { %s%s }' % (
                    p.store() + ' ' if p and not p.stored else '', self.leave(ins.addr, self.n - k, 'AOT_EXIT_JUMP')))
                self.fp = FP_OK
            getattr(self, 'do_' + ins.cls)(ins, la, lb)
            self.fp = f.fp_step(ins, self.fp)
            if sets_esp(ins):
                self.flush(ALL)
                out.append('if (AOT_UNLIKELY(!AOT_ESP_OK(ESP))) %s' % self.leave(ins.next, self.n - k - 1,
                                                                              'AOT_EXIT_JUMP'))
        last = b[-1]
        if not is_term(last) or last.cls in ('call', 'icall'):
            self.drop(self.live_out)
            out.append(self.go(last.next, nxt))
        return out

    def go(self, target, nxt=None):
        if target in self.f.blocks:
            return '' if target == nxt else 'goto L_%x;' % target
        return 'AOT_EXIT(%s, AOT_EXIT_JUMP);' % hexu(target)

    def flush(self, live):
        """Writes the pending flags to EFL if any of them is in live"""
        p = self.pend
        if p and not p.stored and p.defs & live:
            self.out.append(p.store())
            p.stored = True

    def drop(self, live):
        self.flush(live)
        self.pend = None

    def cond(self, cc):
        """C expression for condition cc at this point"""
        p = self.pend
        use = CC_USE[cc]
        if p and use & ~p.defs == 0:
            e = p.cond(cc)
            return e if e else 'aot_cond(%d, %s)' % (cc, p.flags_expr())
        if p and p.defs & use:
            self.flush(ALL)
        return 'aot_cond(%d, EFL)' % cc

    def leave(self, eip, left, reason):
        return 'AOT_LEAVE(%s, %d, %s);' % (hexu(eip), left, reason)

    # -- instruction classes --

    def do_punt(self, ins, la, lb):
        self.drop(ALL)
        self.out.append('%s /* %s %s */' % (self.leave(ins.addr, self.n - self.k, 'AOT_EXIT_PUNT'),
                                            ins.i.mnemonic, ins.i.op_str.replace('*/', '')))

    def do_port(self, ins, la, lb):
        """IN and OUT with an immediate port or DX, through the machine's port handlers"""
        self.drop(ALL)
        ops = ins.i.operands
        if ins.i.mnemonic == 'out':
            sz = ops[1].size
            port = hexu(ops[0].imm & 0xff) if ops[0].type == X.X86_OP_IMM else '(EDX & 0xffff)'
            v = {1: 'EAX & 0xffu', 2: 'EAX & 0xffffu', 4: 'EAX'}[sz]
            self.out.append('aot_out(%s, %s, %d, aot_bud, %d);' % (port, v, sz, self.n - self.k))
        else:
            sz = ops[0].size
            port = hexu(ops[1].imm & 0xff) if ops[1].type == X.X86_OP_IMM else '(EDX & 0xffff)'
            v = 'aot_in(%s, %d, aot_bud, %d)' % (port, sz, self.n - self.k)
            poll = self.f.poll_loop(ins.addr) if self.k == 0 else None
            if poll:
                mask, nz, n = poll
                L = ins.addr
                self.out.append('uint64_t %sp = aot_in_poll(%s, aot_bud, %d, 0x%xu, %d, %d, %d, %d);' % (
                    self.t, port, self.n, mask, nz, n, n - self.n, self.f.need[L]))
                self.out.append('aot_bud -= (int32_t)(%sp >> 32);' % self.t)
                v = '(uint32_t)(%sp & 0xffu)' % self.t
            self.out.append({1: 'EAX = (EAX & 0xffffff00u) | %s;', 2: 'EAX = (EAX & 0xffff0000u) | %s;',
                             4: 'EAX = %s;'}[sz] % v)
        self.out.append('if (AOT_UNLIKELY(aot_cpu->stop_req)) %s' % self.leave(ins.next, self.n - self.k - 1,
                                                                            'AOT_EXIT_STOP'))

    def do_int(self, ins, la, lb):
        """INT n through the machine's handler, without leaving (aot_rt.h, AOT_INT)"""
        self.drop(ALL)
        self.out.append('AOT_INT(0x%x, %s, %d);' % (ins.i.operands[0].imm & 0xff, hexu(ins.next), self.n - self.k))

    def do_seg(self, ins, la, lb):
        """Segment register moves: reads of the selector, and loads through x86_set_seg. A load
        the interpreter would fault on is left to it; after one that leaves DS, ES or SS not flat,
        the dispatcher takes over (translated code assumes them flat)."""
        i, out, t = ins.i, self.out, self.t
        ops = i.operands
        if not ins.segload:
            if ins.m == 'push':
                out.append('PUSH%d(aot_cpu->sel[%d]);' % (16 if opsize16(i) else 32, SEGS[ops[0].reg]))
                return
            d = self.opnd(ops[0], 2 if ops[0].type == X.X86_OP_MEM else None)
            d.prepare(out, t + 'd')
            out.append(d.write('aot_cpu->sel[%d]' % SEGS[ops[1].reg]))
            return
        self.drop(ALL)
        s = SEGS[ops[0].reg]
        if ins.m == 'pop':
            sz = 2 if opsize16(i) else 4
            v = 'RD%d(ESP)' % (8 * sz)
        else:
            o = self.opnd(ops[1], 2)
            o.prepare(out, t + 's')
            v = o.read()
        out.append('if (AOT_UNLIKELY(!x86_set_seg(aot_cpu, %d, (%s) & 0xffffu))) %s' % (
            s, v, self.leave(ins.addr, self.n - self.k, 'AOT_EXIT_PUNT')))
        if ins.m == 'pop':
            out.append('ESP += %d;' % sz)
        if s in (0, 2, 3):
            out.append('if (AOT_UNLIKELY(aot_cpu->base[%d])) %s' % (s, self.leave(ins.next, self.n - self.k - 1,
                                                                              'AOT_EXIT_JUMP')))

    def do_plain(self, ins, la, lb):
        i, m, out, t = ins.i, ins.m, self.out, self.t
        ops = i.operands
        if m == 'nop':
            return
        if m == 'cld':
            out.append('EFL &= ~X86_DF;')
            return
        if m == 'std':
            out.append('EFL |= X86_DF;')
            return
        if m == 'leave':
            # it pops from the new ESP: the check comes first
            self.flush(ALL)
            out.append('if (AOT_UNLIKELY(!AOT_ESP_OK(EBP))) %s' % self.leave(ins.addr, self.n - self.k, 'AOT_EXIT_PUNT'))
            out.append('ESP = EBP;')
            out.append('EBP = POP32();')
            return
        if m == 'pushal':
            out.append('{ uint32_t sp = ESP; PUSH32(EAX); PUSH32(ECX); PUSH32(EDX); PUSH32(EBX); PUSH32(sp); '
                       'PUSH32(EBP); PUSH32(ESI); PUSH32(EDI); }')
            return
        if m == 'popal':
            out.append('EDI = POP32(); ESI = POP32(); EBP = POP32(); ESP += 4; EBX = POP32(); EDX = POP32(); '
                       'ECX = POP32(); EAX = POP32();')
            return
        if m in ('cwde', 'cbw', 'cdq', 'cwd'):
            out.append({'cwde': 'EAX = (uint32_t)(int32_t)(int16_t)EAX;',
                        'cbw': 'EAX = (EAX & 0xffff0000u) | ((uint32_t)(int16_t)(int8_t)EAX & 0xffffu);',
                        'cdq': 'EDX = (uint32_t)((int32_t)EAX >> 31);',
                        'cwd': 'EDX = (EDX & 0xffff0000u) | (((int16_t)EAX < 0) ? 0xffffu : 0);'}[m])
            return
        if m == 'push':
            o = self.opnd(ops[0], 2 if opsize16(i) else 4)
            o.prepare(out, t + 'm')
            out.append('PUSH%d(%s);' % (8 * o.sz, o.read()))
            return
        if m == 'pop' and ops[0].type == X.X86_OP_MEM:
            # ESP is incremented before the address is computed
            o = self.opnd(ops[0])
            out.append('uint32_t %sv = POP%d();' % (t, 8 * o.sz))
            o.prepare(out, t + 'm')
            out.append(o.write(t + 'v'))
            return
        if m == 'pop':
            o = self.opnd(ops[0])
            v = 'POP%d()' % (8 * o.sz)
            if ops[0].type == X.X86_OP_REG and REGS[ops[0].reg][0] == 4 and o.sz == 4:
                out.append('ESP = RDD32(ESP);')
            else:
                out.append('{ uint32_t v = %s; %s }' % (v, o.write('v')))
            return
        d = self.opnd(ops[0])
        if m == 'lea':
            out.append(d.write(address(ops[1])))
            return
        s = self.opnd(ops[1]) if len(ops) > 1 else None
        d.prepare(out, t + 'd')
        if s:
            s.prepare(out, t + 's')
        if m == 'mov':
            out.append(d.write(s.read()))
        elif m == 'movzx':
            out.append(d.write(s.read()))
        elif m == 'movsx':
            cast = {1: '(int8_t)', 2: '(int16_t)'}[s.sz]
            out.append(d.write('(uint32_t)(int32_t)%s%s' % (cast, s.read())))
        elif m == 'not':
            out.append(d.write('~%s' % d.read()))
        elif m == 'xchg':
            out.append('{ uint32_t a = %s, b = %s; %s %s }' % (d.read(), s.read(), d.write('b'), s.write('a')))
        else:
            raise NotImplementedError(m)

    def do_alu(self, ins, la, lb):
        i, m, out, t = ins.i, ins.m, self.out, self.t
        ops = i.operands
        d = self.opnd(ops[0])
        sz = d.sz
        M = hexu(mask(sz))
        if self.pend and self.pend.defs & la & ~ins.defs:
            self.flush(ALL)
        cin = '0'
        if m in ('adc', 'sbb'):
            cin = '(uint32_t)' + self.cond(2)
        d.prepare(out, t + 'd')
        a, b, r = t + 'a', t + 'b', t + 'r'
        out.append('uint32_t %s = %s;' % (a, d.read()))
        if m in ALU:
            s = self.opnd(ops[1], sz)
            s.prepare(out, t + 's')
            out.append('uint32_t %s = %s;' % (b, s.read()))
            if cin != '0':
                out.append('uint32_t %sc = %s;' % (t, cin))
                cin = t + 'c'
                expr = '(%s %s %s %s %s)' % (a, ALU_OP[m], b, ALU_OP[m], cin)
            else:
                expr = '(%s %s %s)' % (a, ALU_OP[m], b)
            kind = ALU[m]
        elif m == 'neg':
            out.append('uint32_t %s = 0;' % b)
            # flags of 0 - a: operands (0, a)
            expr = '(0u - %s)' % a
            kind = 'sub'
            a, b = b, a
        else:
            expr = '(%s %s 1u)' % (a, '+' if m == 'inc' else '-')
            kind = m
            b = '1u'
        out.append('uint32_t %s = %s & %s;' % (r, expr, M))
        if m not in ('cmp', 'test'):
            out.append(d.write(r))
        # what the previous pending operation leaves live was flushed above
        self.pend = Pend(kind, sz, a, b, r, cin, ins.defs) if ins.defs & la else None

    def do_cond(self, ins, la, lb):
        d = self.opnd(ins.i.operands[0])
        d.prepare(self.out, self.t + 'd')
        self.out.append(d.write('(uint32_t)%s' % self.cond(ins.cc)))

    def do_efl(self, ins, la, lb):
        i, m, out, t = ins.i, ins.m, self.out, self.t
        ops = i.operands
        self.drop(lb)
        need = 'true' if ins.pdefs & la else 'false'
        if m in SHIFT:
            d = self.opnd(ops[0])
            d.prepare(out, t + 'd')
            cnt = self.opnd(ops[1], 1).read()
            out.append(d.write('aot_shift(%d, %s, %s, %d, %s, &EFL)' % (SHIFT[m], d.read(), cnt, d.sz, need)))
        elif m in ('shld', 'shrd'):
            d = self.opnd(ops[0])
            d.prepare(out, t + 'd')
            s = self.opnd(ops[1])
            cnt = self.opnd(ops[2], 1).read()
            out.append(d.write('aot_shxd(%s, %s, %s, %s, %d, %s, &EFL)' % (
                'true' if m == 'shld' else 'false', d.read(), s.read(), cnt, d.sz, need)))
        elif m in ('imul', 'mul') and len(ops) == 1:
            s = self.opnd(ops[0])
            s.prepare(out, t + 's')
            # through temporaries: EAX and EDX are global register variables (aot_rt.h)
            out.append('{ uint32_t a_ = EAX, d_ = EDX; aot_mul1(%s, %s, %d, %s, &a_, &d_, &EFL); EAX = a_; EDX = d_; }' % (
                'true' if m == 'imul' else 'false', s.read(), s.sz, need))
        elif m == 'imul':
            d = self.opnd(ops[0])
            s1 = self.opnd(ops[1]) if len(ops) == 2 else self.opnd(ops[1])
            s1.prepare(out, t + 's')
            a = d.read() if len(ops) == 2 else s1.read()
            b = s1.read() if len(ops) == 2 else self.opnd(ops[2], d.sz).read()
            out.append(d.write('aot_imul2(%s, %s, %d, %s, &EFL)' % (a, b, d.sz, need)))
        elif m in ('div', 'idiv'):
            s = self.opnd(ops[0])
            s.prepare(out, t + 's')
            out.append('{ uint32_t a_ = EAX, d_ = EDX;')
            out.append('  if (AOT_UNLIKELY(!aot_div1(%s, %s, %d, &a_, &d_))) %s' % (
                'true' if m == 'idiv' else 'false', s.read(), s.sz, self.leave(ins.addr, self.n - self.k,
                                                                               'AOT_EXIT_PUNT')))
            out.append('  EAX = a_; EDX = d_; }')
        elif m == 'clc':
            out.append('EFL &= ~X86_CF;')
        elif m == 'stc':
            out.append('EFL |= X86_CF;')
        elif m == 'cmc':
            out.append('EFL ^= X86_CF;')
        elif m == 'sahf':
            out.append('aot_set_flags(&EFL, X86_CF | X86_PF | X86_AF | X86_ZF | X86_SF, EAX >> 8);')
        elif m == 'lahf':
            out.append('EAX = (EAX & 0xffff00ffu) | (((EFL & 0xd7u) | 2u) << 8);')
        else:
            kind, sz = string_kind(m)
            out.append('AOT_SAVE(); aot_string(0x%x, %d, 0x%x); AOT_LOAD();' % (kind, sz, ins.rep))

    def do_jcc(self, ins, la, lb):
        out = self.out
        m = ins.m
        if m == 'jecxz':
            c = '(ECX == 0)'
        elif m.startswith('loop'):
            out.append('ECX -= 1;')
            c = '(ECX != 0)'
            if m == 'loope':
                c = '(ECX != 0 && %s)' % self.cond(4)
            elif m == 'loopne':
                c = '(ECX != 0 && %s)' % self.cond(5)
        else:
            c = self.cond(ins.cc)
        out.append('bool %sc = %s;' % (self.t, c))
        self.drop(self.live_out)
        out.append('if (%sc) %s' % (self.t, self.go(ins.target) or 'goto L_%x;' % ins.target))
        out.append(self.go(ins.next))

    def do_jmp(self, ins, la, lb):
        f = self.f
        if f.prog.is_tail(f.start, ins.i, ins.target) and ins.target in self.known:
            self.drop(self.live_in_funcs.get(ins.target, ALL))
            self.out.append('AOT_TAIL(%s);' % fname(ins.target))
            return
        self.drop(self.live_out)
        self.out.append(self.go(ins.target))

    def do_ijmp(self, ins, la, lb):
        o = self.opnd(ins.i.operands[0])
        o.prepare(self.out, self.t + 'm')
        self.drop(ALL)
        self.out.append('AOT_EXIT(%s, AOT_EXIT_JUMP);' % o.read())

    def do_table(self, ins, la, lb):
        o = self.opnd(ins.i.operands[0])
        o.prepare(self.out, self.t + 'm')
        self.drop(ALL)
        out = self.out
        out.append('{')
        out.append('    uint32_t t = %s;' % o.read())
        out.append('    switch (t) {')
        for c in sorted(set(self.f.prog.tables[ins.addr])):
            if c in self.f.blocks:
                out.append('    case %s: goto L_%x;' % (hexu(c), c))
        out.append('    default: AOT_EXIT(t, AOT_EXIT_JUMP);')
        out.append('    }')
        out.append('}')

    def do_call(self, ins, la, lb):
        self.drop(lb)
        out = self.out
        out.append('PUSH32(%s);' % hexu(ins.next))
        if ins.target in self.known:
            out.append('AOT_CALL(%s, %s, %d);' % (fname(ins.target), hexu(ins.next), self.call_need(ins)))
        else:
            out.append('AOT_EXIT(%s, AOT_EXIT_JUMP);' % hexu(ins.target))

    def call_need(self, ins):
        """The budget a call checks after it returns (Func.call_check), or 0"""
        return self.f.need[ins.next] if ins.next in self.f.call_check else 0

    def do_icall(self, ins, la, lb):
        o = self.opnd(ins.i.operands[0])
        o.prepare(self.out, self.t + 'm')
        self.drop(ALL)
        out = self.out
        out.append('{ uint32_t t = %s; PUSH32(%s); AOT_ICALL(t, %s, %d); }' % (o.read(), hexu(ins.next), hexu(ins.next),
                                                                          self.call_need(ins)))

    def do_ret(self, ins, la, lb):
        self.drop(ALL)
        ops = ins.i.operands
        if ins.addr in self.f.ret_jumps:
            # to an address this function pushed: a jump there, which returns where this one would
            self.out.append('{ uint32_t t = POP32(); ESP += %s; AOT_ITAIL(t); }' % hexu(ops[0].imm & 0xffff if ops else 0))
        elif ops:
            self.out.append('{ uint32_t r = POP32(); ESP += %s; AOT_RET(r); }' % hexu(ops[0].imm & 0xffff))
        else:
            self.out.append('AOT_RET(POP32());')
