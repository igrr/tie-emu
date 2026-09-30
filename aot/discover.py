"""Code discovery for the ahead-of-time translator."""
import struct

import capstone
from capstone import x86 as X

from le import LE

DELTA = 0x100000

JCC = {'jo': 0, 'jno': 1, 'jb': 2, 'jae': 3, 'je': 4, 'jne': 5, 'jbe': 6, 'ja': 7,
       'js': 8, 'jns': 9, 'jp': 10, 'jnp': 11, 'jl': 12, 'jge': 13, 'jle': 14, 'jg': 15}


class Image:
    """The LE objects as loaded at link address + DELTA, fixups applied."""

    def __init__(self, path, delta=DELTA):
        le = LE(path)
        self.name = le.name()
        fix = {}
        imgs = {o['n']: bytearray(le.obj_image(o)) for o in le.objs}
        for p, src, offs, t in le.fixups():
            o = next(o for o in le.objs if o['pidx'] <= p + 1 < o['pidx'] + o['npages'])
            pb = (p + 1 - o['pidx']) * le.h['page_size']
            tgt = le.objs[t[1] - 1]['base'] + t[2] + delta
            for s in offs:
                off = pb + s
                if 0 <= off <= len(imgs[o['n']]) - 4:
                    struct.pack_into('<I', imgs[o['n']], off, tgt)
                fix[o['base'] + delta + off] = tgt
        self.fix = fix
        code = le.objs[0]
        self.code_lo = code['base'] + delta
        self.code = bytes(imgs[code['n']])
        self.code_hi = self.code_lo + len(self.code)
        self.entry = self.code_lo + le.h['eip']
        # the objects' range: the translation reads and writes constant addresses in it directly
        self.direct = (min(o['base'] for o in le.objs) + delta, max(o['base'] + o['vsize'] for o in le.objs) + delta)

    def in_code(self, a):
        return self.code_lo <= a < self.code_hi


def muse_modules(path):
    """The LX modules (engine and drivers) in the MUSE blob of IMUSE.EXE: {name: LE}. The blob
    starts at the paragraph in the word at MZ offset 0x12."""
    d = open(path, 'rb').read()
    blob = d[struct.unpack_from('<H', d, 0x12)[0] * 16:]
    assert blob[:4] == b'MUSE', 'no MUSE blob in %s' % path
    mods = {}
    at = blob.find(b'MZ')
    while at >= 0:
        if at + 0x40 <= len(blob):
            lfa = at + struct.unpack_from('<I', blob, at + 0x3C)[0]
            if blob[lfa:lfa + 2] == b'LX':
                le = LE(data=blob, at=at)
                mods[le.name()] = le
        at = blob.find(b'MZ', at + 2)
    return mods


class DllImage:
    """A one-object LX module of the MUSE blob, relocated to the address the game's loader puts it
    at (the game allocates it on its heap: aotgen.py --dll).

    Code and data share the object: Watcom puts the code first. The code ends at the first
    variable the code addresses directly that no direct branch before it jumps over (variables
    embedded in the code, like a saved DS, have code after them that is called from before).
    Functions start at the entry (offset 0) and at the targets of fixups below that point that
    are not addressed as variables."""

    def __init__(self, le, base):
        o = le.objs[0]
        assert len(le.objs) == 1 or all(x['vsize'] == 0 for x in le.objs[1:]), 'multi-object module'
        img = bytearray(le.obj_image(o))
        fix = {}
        for p, src, offs, t in le.fixups():
            assert (src & 0xF) == 7 and t[0] == 'int' and t[1] == 1, 'fixup kind'
            for s in offs:
                off = p * le.h['page_size'] + s
                if 0 <= off <= len(img) - 4:
                    struct.pack_into('<I', img, off, base + t[2])
                    fix[base + off] = base + t[2]
        self.name = le.name()
        self.code = bytes(img)
        self.code_lo = self.entry = base
        self.direct = (base, base + len(img))
        self.code_hi, self.data = self._split()
        self.fix = {s: t for s, t in fix.items() if t not in self.data}

    def _split(self):
        md = capstone.Cs(capstone.CS_ARCH_X86, capstone.CS_MODE_32)
        md.detail = True
        lo, hi = self.direct
        variables, branches = set(), []
        # linear sweep from the start: Watcom code is contiguous
        for i in md.disasm(self.code, lo):
            for op in i.operands:
                if op.type == X.X86_OP_MEM and not op.mem.base and not op.mem.index and lo <= op.mem.disp & 0xffffffff < hi:
                    variables.add(op.mem.disp & 0xffffffff)
            if (i.group(capstone.CS_GRP_JUMP) or i.group(capstone.CS_GRP_CALL)) and i.operands[0].type == X.X86_OP_IMM:
                branches.append((i.address, i.operands[0].imm & 0xffffffff))
        for v in sorted(variables):
            if not any(a < v <= t for a, t in branches):
                return v, variables
        return hi, variables

    def in_code(self, a):
        return self.code_lo <= a < self.code_hi


class Program:
    """Decoded instructions and functions of one image"""

    def __init__(self, img):
        self.img = img
        self.md = capstone.Cs(capstone.CS_ARCH_X86, capstone.CS_MODE_32)
        self.md.detail = True
        self.insns = {}
        self.tables = {}        # jmp address -> list of case targets
        self.funcs = set()
        self._discover()

    def decode(self, a):
        i = self.insns.get(a)
        if i is None and self.img.in_code(a):
            off = a - self.img.code_lo
            i = next(self.md.disasm(self.img.code[off:off + 16], a, 1), None)
            if i is not None:
                self.insns[a] = i
        return i

    def succs(self, i):
        """Intra-procedural successors of an instruction, and whether it falls through"""
        m = i.mnemonic
        nxt = i.address + i.size
        if m in JCC or m in ('loop', 'jecxz', 'loope', 'loopne'):
            return [i.operands[0].imm & 0xffffffff], True
        if m == 'jmp':
            op = i.operands[0]
            if op.type == X.X86_OP_IMM:
                return [op.imm & 0xffffffff], False
            return list(self.tables.get(i.address, [])), False
        if m in ('ret', 'iretd', 'iret', 'hlt', 'retf', 'ljmp', 'ud2'):
            return [], False
        return [], True

    def _table(self, i):
        """Switch table cases for `jmp [reg*4 + table]` with the table in the code object"""
        op = i.operands[0]
        if (op.type != X.X86_OP_MEM or not op.mem.index or op.mem.scale != 4 or op.mem.base
                or not self.img.in_code(op.mem.disp & 0xffffffff)):
            return None
        t = op.mem.disp & 0xffffffff
        cases = []
        while t in self.img.fix and self.img.in_code(self.img.fix[t]):
            cases.append(self.img.fix[t])
            t += 4
        return cases

    def _discover(self):
        img = self.img
        data_refs = {t for s, t in img.fix.items() if img.in_code(t) and not img.in_code(s)}
        code_refs = {t for s, t in img.fix.items() if img.in_code(t) and img.in_code(s)}
        seeds = [img.entry] + sorted(data_refs)
        self.funcs = {img.entry} | data_refs
        case_targets = set()
        stack = list(seeds)
        seen = set()

        def walk():
            while stack:
                a = stack.pop()
                while a not in seen:
                    i = self.decode(a)
                    if i is None:
                        break
                    seen.add(a)
                    m = i.mnemonic
                    if m == 'call' and i.operands[0].type == X.X86_OP_IMM:
                        t = i.operands[0].imm & 0xffffffff
                        if img.in_code(t):
                            self.funcs.add(t)
                            stack.append(t)
                    if m == 'jmp' and i.operands[0].type == X.X86_OP_MEM:
                        cases = self._table(i)
                        if cases is not None:
                            self.tables[a] = cases
                            case_targets.update(cases)
                    ts, fall = self.succs(i)
                    stack.extend(t for t in ts if img.in_code(t))
                    if not fall:
                        break
                    a += i.size

        walk()
        # callbacks passed as immediates: code-to-code fixups that are not switch cases
        extra = sorted(t for t in code_refs if t not in case_targets and t not in seen)
        for t in extra:
            if self.decode(t) is not None:
                self.funcs.add(t)
                stack.append(t)
        walk()
        # tables of `int n; ret` stubs (Watcom's int386x computes the address of the one for n
        # from the table's start and returns into it): each stub is a function
        stubs = set()
        for t in sorted(self.funcs):
            while self._int_stub(t) and self._int_stub(t + 3):
                t += 3
                stubs.add(t)
        for t in sorted(stubs - self.funcs):
            self.funcs.add(t)
            stack.append(t)
        walk()
        self.funcs = {f for f in self.funcs if f in self.insns}
        # functions the program may reach other than by a direct call: their rets may go anywhere
        self.addr_taken = {img.entry} | {t for t in img.fix.values() if img.in_code(t)} | stubs
        # what each function's rets pop besides the return address: 0, n (ret n), None (differs)
        self.ret_pops = {}
        for f in self.funcs:
            pops = {self.insns[a].operands[0].imm & 0xffff if self.insns[a].operands else 0
                    for a in self.body(f) if self.insns[a].mnemonic == 'ret'}
            self.ret_pops[f] = pops.pop() if len(pops) == 1 else (0 if not pops else None)

    def _int_stub(self, a):
        off = a - self.img.code_lo
        c = self.img.code
        # INT 3 is CC 90 (int3, nop) in the table
        return (self.img.in_code(a) and off + 3 <= len(c) and c[off + 2] == 0xc3 and
                (c[off] == 0xcd or c[off:off + 2] == b'\xcc\x90'))

    def body(self, start):
        """Instruction addresses of the function at start: reachable without following calls.
        Forward jumps to another function's start are tail calls and end the walk there."""
        seen = set()
        stack = [start]
        while stack:
            a = stack.pop()
            while a not in seen:
                i = self.decode(a)
                if i is None:
                    break
                seen.add(a)
                ts, fall = self.succs(i)
                for t in ts:
                    if not self.is_tail(start, i, t):
                        stack.append(t)
                if not fall:
                    break
                a += i.size
        return seen

    def is_tail(self, start, i, t):
        """A branch to another function's start that leaves this one (forward, or outside it)"""
        return t in self.funcs and t != start and (t > i.address or t < start) and i.mnemonic == 'jmp'
