#!/usr/bin/env python3
"""Prepares the SD card's files from the GOG release of TIE Fighter.

    sdcard.py SOURCE [OUT]

SOURCE is the GOG installer for Linux (gog_star_wars_tie_fighter_special_edition_*.sh), or a
directory with the game's installed files: Z_TIE__.EXE, the CD image game.gog and the rest.
OUT (default: sdcard/ in the repository) receives

    OUT/tie/c   C:, the game directory, without the CD image
    OUT/tie/d   D:, the files on the CD image

The build reads the game's executables from OUT/tie/c; OUT/tie goes to the root of the card.
"""
import os
import shutil
import struct
import sys
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTALLER_DIR = 'data/noarch/data/'     # the game's files in the installer
CD_IMAGE = 'game.gog'                   # MODE1/2352 track (game.ins is its cue sheet)
SKIP = (CD_IMAGE, 'game.ins')


class CD:
    """The ISO 9660 file system of a MODE1/2352 image"""

    def __init__(self, path):
        self.f = open(path, 'rb')

    def sectors(self, lba, n=1):
        out = b''
        for k in range(n):
            self.f.seek((lba + k) * 2352 + 16)
            out += self.f.read(2048)
        return out

    def walk(self, lba=None, size=None, path=''):
        """(path, first sector, size) of each file"""
        if lba is None:
            root = self.sectors(16)[156:156 + 34]
            lba, size = struct.unpack_from('<I', root, 2)[0], struct.unpack_from('<I', root, 10)[0]
        data = self.sectors(lba, (size + 2047) // 2048)
        i = 0
        while i < len(data):
            ln = data[i]
            if ln == 0:
                i = (i // 2048 + 1) * 2048     # records do not cross sectors
                continue
            rec = data[i:i + ln]
            elba, esize, flags = struct.unpack_from('<I', rec, 2)[0], struct.unpack_from('<I', rec, 10)[0], rec[25]
            name = rec[33:33 + rec[32]]
            i += ln
            if name in (b'\0', b'\1'):
                continue
            full = path + '/' + name.decode('latin1').split(';')[0]
            if flags & 2:
                yield from self.walk(elba, esize, full)
            else:
                yield full, elba, esize

    def extract(self, dest):
        for path, lba, size in self.walk():
            out = os.path.join(dest, path.lstrip('/'))
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, 'wb') as f:
                while size > 0:
                    n = min(size, 2048 * 64)
                    f.write(self.sectors(lba, (n + 2047) // 2048)[:n])
                    lba += (n + 2047) // 2048
                    size -= n


def game_path(rel):
    """Where a file of the game goes on C:. The installer keeps the sample pilot in save/; the
    game looks for pilots in its own directory."""
    return rel[len('save/'):] if rel.lower().startswith('save/') else rel


def from_installer(src, c, tmp):
    """Unpacks the installer's game files into c, and the CD image to tmp"""
    with zipfile.ZipFile(src) as z:
        for info in z.infolist():
            if not info.filename.startswith(INSTALLER_DIR) or info.is_dir():
                continue
            rel = info.filename[len(INSTALLER_DIR):]
            out = tmp if rel == CD_IMAGE else None if rel in SKIP else os.path.join(c, game_path(rel))
            if out:
                os.makedirs(os.path.dirname(out), exist_ok=True)
                with z.open(info) as i, open(out, 'wb') as o:
                    shutil.copyfileobj(i, o, 1 << 20)
    return tmp


def from_directory(src, c):
    """Copies the game files in src into c; returns the CD image's path"""
    for root, _, files in os.walk(src):
        for f in files:
            rel = os.path.relpath(os.path.join(root, f), src).replace(os.sep, '/')
            if rel in SKIP or f.endswith('.conf') or f.startswith('.'):
                continue
            out = os.path.join(c, game_path(rel))
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copyfile(os.path.join(root, f), out)
    return os.path.join(src, CD_IMAGE)


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    src = sys.argv[1]
    out = os.path.join(sys.argv[2] if len(sys.argv) == 3 else os.path.join(REPO, 'sdcard'), 'tie')
    c, d = os.path.join(out, 'c'), os.path.join(out, 'd')
    for p in (c, d):
        if os.path.exists(p):
            sys.exit('%s exists: remove it first' % p)
    tmp = os.path.join(out, CD_IMAGE)
    image = from_directory(src, c) if os.path.isdir(src) else from_installer(src, c, tmp)
    if not os.path.exists(os.path.join(c, 'Z_TIE__.EXE')) or not os.path.exists(image):
        sys.exit('%s: not the GOG release of TIE Fighter (no Z_TIE__.EXE or %s)' % (src, CD_IMAGE))
    CD(image).extract(d)
    if image == tmp:
        os.remove(tmp)
    print('%s: C: and D: for the card' % out)


if __name__ == '__main__':
    main()
