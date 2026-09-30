"""Minimal parser for DOS/4GW Linear Executable (LE) files produced by Watcom, and the LX DLLs
in the iMUSE blob of IMUSE.EXE."""
import struct

HDR = [
    (0x10, 'module_flags'), (0x14, 'num_pages'), (0x18, 'eip_obj'), (0x1C, 'eip'),
    (0x20, 'esp_obj'), (0x24, 'esp'), (0x28, 'page_size'), (0x2C, 'last_page_size'),
    (0x30, 'fixup_size'), (0x38, 'loader_size'), (0x40, 'obj_tab'), (0x44, 'num_objs'),
    (0x48, 'obj_page_tab'), (0x4C, 'iter_pages'), (0x50, 'rsrc_tab'), (0x54, 'num_rsrc'),
    (0x58, 'resname_tab'), (0x5C, 'entry_tab'), (0x68, 'fixup_page_tab'), (0x6C, 'fixup_rec_tab'),
    (0x70, 'imp_mod_tab'), (0x74, 'num_imp_mods'), (0x78, 'imp_proc_tab'), (0x80, 'data_pages'),
    (0x84, 'num_preload'), (0x88, 'nonres_tab'), (0x8C, 'nonres_len'), (0x94, 'auto_ds_obj'),
    (0x98, 'debug_off'), (0x9C, 'debug_len'),
]


class LE:
    """An LE executable (path), or an LX module whose MZ header is at offset `at` of data (its
    offsets are relative to that header)"""

    def __init__(self, path=None, data=None, at=0):
        self.d = d = open(path, 'rb').read() if data is None else data
        self.base = base = at + struct.unpack_from('<I', d, at + 0x3C)[0]
        self.lx = d[base:base + 2] == b'LX'
        if d[base:base + 2] != b'LE' and not self.lx:
            # bound with a full DOS/4GW stub: the LE header follows the stub image
            base = d.find(b'LE\0\0')
            while base >= 0 and struct.unpack_from('<I', d, base + 0x44)[0] > 16:
                base = d.find(b'LE\0\0', base + 1)
            self.base = base
        assert base > 0 and d[base:base + 2] in (b'LE', b'LX'), 'not an LE file'
        self.h = {name: struct.unpack_from('<I', d, base + off)[0] for off, name in HDR}
        h = self.h
        self.objs = []
        for i in range(h['num_objs']):
            vsize, reloc, flags, pidx, npages, _ = struct.unpack_from('<6I', d, base + h['obj_tab'] + 24 * i)
            self.objs.append(dict(n=i + 1, vsize=vsize, base=reloc, flags=flags, pidx=pidx, npages=npages))
        self.pages = []  # per logical page: file offset, size
        for i in range(h['num_pages']):
            if self.lx:
                # offset (shifted by the header's page offset shift, at 0x2C), size, flags
                off, size, flags = struct.unpack_from('<IHH', d, base + h['obj_page_tab'] + 8 * i)
                self.pages.append((at + h['data_pages'] + (off << h['last_page_size']), size, flags))
                continue
            e = d[base + h['obj_page_tab'] + 4 * i: base + h['obj_page_tab'] + 4 * i + 4]
            num = (e[0] << 16) | (e[1] << 8) | e[2]
            size = h['last_page_size'] if i == h['num_pages'] - 1 else h['page_size']
            self.pages.append((h['data_pages'] + (num - 1) * h['page_size'], size, e[3]))
        fpt = base + h['fixup_page_tab']
        self.fixup_offs = struct.unpack_from('<%dI' % (h['num_pages'] + 1), d, fpt)

    def name(self):
        """The module name: the first entry of the resident name table"""
        i = self.base + self.h['resname_tab']
        return self.d[i + 1:i + 1 + self.d[i]].decode('latin-1')

    def obj_image(self, o):
        """Loaded image of one object (zero-filled to vsize)."""
        buf = bytearray(max(o['vsize'], o['npages'] * self.h['page_size']))
        for k in range(o['npages']):
            off, size, flags = self.pages[o['pidx'] - 1 + k]
            assert not (self.lx and flags), 'LX page type %d' % flags
            buf[k * self.h['page_size']:k * self.h['page_size'] + size] = self.d[off:off + size]
        return bytes(buf[:o['vsize']])

    def fixups(self):
        """Yield (page_index, src_type, src_offsets, target) for every fixup record."""
        d, h = self.d, self.h
        rec_base = self.base + h['fixup_rec_tab']
        for p in range(h['num_pages']):
            i, end = rec_base + self.fixup_offs[p], rec_base + self.fixup_offs[p + 1]
            while i < end:
                src, flg = d[i], d[i + 1]
                i += 2
                if src & 0x20:
                    cnt = d[i]
                    i += 1
                else:
                    offs = [struct.unpack_from('<h', d, i)[0]]
                    i += 2
                ttype = flg & 3
                if ttype == 0:  # internal reference
                    if flg & 0x40:
                        obj = struct.unpack_from('<H', d, i)[0]; i += 2
                    else:
                        obj = d[i]; i += 1
                    if (src & 0x0F) == 2:
                        toff = None
                    elif flg & 0x10:
                        toff = struct.unpack_from('<I', d, i)[0]; i += 4
                    else:
                        toff = struct.unpack_from('<H', d, i)[0]; i += 2
                    target = ('int', obj, toff)
                elif ttype == 1:  # import by ordinal
                    mod = d[i] if not flg & 0x40 else struct.unpack_from('<H', d, i)[0]
                    i += 1 if not flg & 0x40 else 2
                    if flg & 0x80:
                        ordn = d[i]; i += 1
                    elif flg & 0x10:
                        ordn = struct.unpack_from('<I', d, i)[0]; i += 4
                    else:
                        ordn = struct.unpack_from('<H', d, i)[0]; i += 2
                    target = ('ord', mod, ordn)
                else:
                    raise NotImplementedError('fixup target type %d' % ttype)
                if flg & 0x04:  # additive
                    i += 4 if flg & 0x20 else 2
                if src & 0x20:
                    offs = list(struct.unpack_from('<%dh' % cnt, d, i))
                    i += 2 * cnt
                yield p, src, offs, target
