#!/usr/bin/env python3
"""Ahead-of-time translation of Z_TIE__.EXE (and the iMUSE modules it loads) to C.

    aotgen.py EXE OUTDIR [--files N] [--imuse IMUSE.EXE --dll NAME@BASE ...]

Writes OUTDIR/aot_gen.h (prototypes), OUTDIR/aot_gen_NN.c (the functions, N files) and
OUTDIR/aot_table.c (the entry table the dispatcher in components/aot looks addresses up in, and
the modules: the code each part was translated from).

--dll translates the LX module NAME of the MUSE blob in IMUSE.EXE as the game loads it at BASE.
The game puts the modules on its heap; components/aot/CMakeLists.txt gives the addresses the game
loads them at.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from discover import DllImage, Image, Program, muse_modules  # noqa: E402
from lower import ALL, Emitter, Func, Insn, entry_list, entry_need, fname, hexu  # noqa: E402


def build(prog, log):
    """Functions with liveness; returns (funcs, live_in_funcs)"""
    insns = {a: Insn(prog, i) for a, i in prog.insns.items()}
    funcs = {s: Func(prog, s, insns) for s in sorted(prog.funcs)}
    live_in = {s: 0 for s in funcs}
    ret = RetLiveness(prog, funcs)
    for rnd in range(50):
        changed = 0
        for s, f in funcs.items():
            v = f.liveness(live_in)
            if v != live_in[s]:
                live_in[s] = v
                changed += 1
        changed += ret.update()
        if log:
            log('liveness round %d: %d functions changed' % (rnd, changed))
        if not changed:
            break
    else:
        raise RuntimeError('flag liveness does not converge')
    return funcs, live_in


class RetLiveness:
    """The flags live where each function's rets go (Func.ret_live), starting from none.

    A ret goes to the return address on the stack: after a call to the function, which reads
    what is live after that call. A function the program may reach otherwise (the entry, and
    code addresses stored in data or immediates: function pointers, callbacks) may return
    anywhere, so all flags are live there. A function a jump leaves to returns where the jumping
    function returns. Functions that share a block are one: the dispatcher enters a shared block
    through the function the entry table gives it, whichever one the code was running in."""

    def __init__(self, prog, funcs):
        self.funcs = funcs
        parent = {s: s for s in funcs}

        def find(s):
            while parent[s] != s:
                parent[s] = parent[parent[s]]
                s = parent[s]
            return s
        first = {}
        for s, f in funcs.items():
            for L in f.blocks:
                if L in first:
                    parent[find(s)] = find(first[L])
                else:
                    first[L] = s
        self.cls = {s: find(s) for s in funcs}
        self.calls = []     # (callee, caller, call instruction address)
        self.tails = []     # (target, jumping function)
        for s, f in funcs.items():
            for b in f.blocks.values():
                last = b[-1]
                if last.cls == 'call' and last.target in funcs:
                    self.calls.append((last.target, s, last.addr))
                elif (last.cls == 'jmp' and last.target in funcs and last.target != s
                      and prog.is_tail(s, last.i, last.target)):
                    self.tails.append((last.target, s))
        self.live = {c: 0 for c in set(self.cls.values())}
        for s in funcs:
            if s in prog.addr_taken:
                self.live[self.cls[s]] = ALL
        for s, f in funcs.items():
            f.ret_live = self.live[self.cls[s]]

    def update(self):
        """One round from the functions' current liveness; returns how many functions changed"""
        live = dict(self.live)
        for callee, caller, a in self.calls:
            live[self.cls[callee]] |= self.funcs[caller].live_after.get(a, ALL)
        for t, s in self.tails:
            live[self.cls[t]] |= live[self.cls[s]]
        self.live = live
        changed = 0
        for s, f in self.funcs.items():
            v = live[self.cls[s]]
            if v != f.ret_live:
                f.ret_live = v
                changed += 1
        return changed


def assign_entries(funcs, starts):
    """Gives each entry block one function: the one it starts, else the first that enters there
    by itself (Func.own_entries), else the first that has the block. The targets of branches
    that leave a function are entries too."""
    has = {}
    for s in starts:
        for L in funcs[s].blocks:
            has.setdefault(L, []).append(s)
    want = {}
    for s in starts:
        for L in funcs[s].entries:
            want.setdefault(L, []).append(s)
        for t in funcs[s].leaves_to():
            if t in has:
                want.setdefault(t, [])
    for f in funcs.values():
        f.entries = set()
    for L, ws in want.items():
        s = L if L in funcs else (ws[0] if ws else has[L][0])
        funcs[s].entries.add(L)


def write_file(path, lines):
    text = '\n'.join(lines) + '\n'
    try:
        if open(path).read() == text:
            return
    except OSError:
        pass
    with open(path, 'w') as f:
        f.write(text)


def fnv(data, h=0x811c9dc5):
    for byte in data:
        h = ((h ^ byte) * 0x01000193) & 0xffffffff
    return h


def code_ranges(prog):
    """The bytes a module's check hashes: its whole code for an executable; the instructions
    for a module that keeps variables between them"""
    img = prog.img
    if not isinstance(img, DllImage):
        return [(img.code_lo, img.code_hi)]
    out = []
    for a in sorted(prog.insns):
        e = a + prog.insns[a].size
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], e)
        else:
            out.append([a, e])
    return [tuple(r) for r in out]


def generate(progs, outdir, nfiles=32, log=print):
    """progs: the Program of each module; the first is the executable, in memory from the start"""
    funcs, live_in, mod_of = {}, {}, {}
    for k, prog in enumerate(progs):
        f, li = build(prog, log)
        funcs.update(f)
        live_in.update(li)
        mod_of.update((s, k) for s in f)
    os.makedirs(outdir, exist_ok=True)
    starts = sorted(funcs)
    known = set(starts)
    assign_entries(funcs, starts)
    header = ['/* Generated by aot/aotgen.py: do not edit */', '#pragma once', '#include "aot_rt.h"', '']
    header += ['uint32_t %s(uint32_t entry, uint32_t aot_efl, uint32_t aot_next);' % fname(s) for s in starts]
    write_file(os.path.join(outdir, 'aot_gen.h'), header)
    per_file = (len(starts) + nfiles - 1) // nfiles
    for n in range(nfiles):
        lines = ['/* Generated by aot/aotgen.py: do not edit */', '#include "aot_gen.h"', '', 'AOT_SLOW_PATHS', '']
        for s in starts[n * per_file:(n + 1) * per_file]:
            e = Emitter(funcs[s], known, live_in)
            lines += e.emit()
            lines.append('')
        write_file(os.path.join(outdir, 'aot_gen_%02d.c' % n), lines)
    # entry table: { eip, function, entry number, budget needed, function start, module }
    owner = {}
    for s in starts:
        for k, L in enumerate(entry_list(funcs[s])):
            owner[L] = (s, k + 1)
    table = ['/* Generated by aot/aotgen.py: do not edit */', '#include "aot.h"', '#include "aot_gen.h"', '',
             'const aot_entry_t aot_entries[] = {']
    for L in sorted(owner):
        s, k = owner[L]
        table.append('    { %s, %s, %d, %d, %d, %d },' % (hexu(L), fname(s), k, entry_need(funcs[s], L),
                                                         1 if L in known else 0, mod_of[s]))
    table += ['};', 'const uint32_t aot_num_entries = %d;' % len(owner), '']
    mods = []
    for k, prog in enumerate(progs):
        img = prog.img
        ranges = code_ranges(prog)
        h = 0x811c9dc5
        for a, e in ranges:
            h = fnv(img.code[a - img.code_lo:e - img.code_lo], h)
        table.append('static const uint32_t ranges_%d[][2] = {' % k)
        table += ['    { %s, %s },' % (hexu(a), hexu(e)) for a, e in ranges]
        table.append('};')
        mods.append('    { "%s", %s, %s, %s, %s, %s, ranges_%d, %d, %d },' % (
            img.name, hexu(img.code_lo), hexu(img.code_hi), hexu(img.direct[0]), hexu(img.direct[1]),
            hexu(h), k, len(ranges), 1 if k else 0))
        if log:
            log('module %s at %s: %d functions, %d bytes of code checked' % (
                img.name, hexu(img.code_lo), sum(1 for s in funcs if mod_of[s] == k), sum(e - a for a, e in ranges)))
    table += ['', 'const aot_module_t aot_modules[] = {'] + mods + ['};',
              'const uint32_t aot_num_modules = %d;' % len(progs), '']
    write_file(os.path.join(outdir, 'aot_table.c'), table)
    if log:
        log('%d functions, %d blocks, %d entries, %d files' % (
            len(funcs), sum(len(f.blocks) for f in funcs.values()), len(owner), nfiles))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('exe')
    ap.add_argument('outdir')
    ap.add_argument('--files', type=int, default=32)
    ap.add_argument('--imuse', help='IMUSE.EXE, for --dll')
    ap.add_argument('--dll', action='append', default=[], metavar='NAME@BASE')
    a = ap.parse_args()
    progs = [Program(Image(a.exe))]
    if a.dll:
        mods = muse_modules(a.imuse)
        for spec in a.dll:
            name, base = spec.split('@')
            progs.append(Program(DllImage(mods[name], int(base, 0))))
    generate(progs, a.outdir, a.files)


if __name__ == '__main__':
    main()
