#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check streamed QVision CPU monochrome expansion and transfer state."""

import argparse
from pathlib import Path
import runpy
import tempfile

support = runpy.run_path(
    str(Path(__file__).with_name('jensen-qvision-testlib.py')))


def check(qemu, rom, root):
    bios = root / 'pal.bin'
    bios.write_bytes(support['pal_test_rom']())
    io = support['DeviceIO'](qemu, bios, [
        '-machine', 'qvision=on,qvision-romfile=' + rom.as_posix()])

    def gc(index, value):
        io.outb(0x3ce, index)
        io.outb(0x3cf, value)

    def out(port, value, size):
        io.write(0x300000000 + (port << 7) + ((size - 1) << 5),
                 value << ((port & 3) * 8))

    def window(address, size=4):
        return 0x200000000 + (address << 7) + ((size - 1) << 5)

    def snapshot():
        return b''.join(io.read(window(0x400000 + i)).to_bytes(4, 'little')
                        for i in range(0, 1024, 4))

    def initialize():
        for i in range(0, 1024, 4):
            io.write(window(0x400000 + i), 0xa5a5a5a5)

    def command(width, height, align=0, transparent=False, rop=None):
        io.outb(0x63ca, 0x13)
        gc(0x5a, (0x50 if transparent else 4) |
           (0xc0 if rop is not None and not transparent else 0))
        if rop is not None:
            io.outb(0x33c7, rop)
        gc(0x43, 0x3c)
        gc(0x44, 0x96)
        out(0x63c0, align, 4)
        out(0x63cc, (2 << 16) | 3, 4)
        out(0x23ce, 8, 2)
        out(0x23c2, width, 2)
        out(0x23c4, height, 2)
        io.outb(0x33cf, 0xc0)
        io.outb(0x33ce, 1)

    def data(value, size=4, legacy=False):
        io.write(window(0xa0000 if legacy else 0x400000, size), value)

    try:
        gc(0x0f, 5)
        gc(0x40, 1)
        gc(0x10, 8)
        gc(0x48, 4)
        io.outb(0x3c4, 2)
        io.outb(0x3c5, 0xff)
        gc(8, 0xff)
        initialize()
        command(5, 3)
        assert io.inb(0x63ca) & 0x40, 'transfer is not busy before data'
        assert io.inb(0x33ce) & 1, 'start cleared before host input'
        # Narrow rows are byte-aligned. A bus write can contain several
        # rows; bytes after the command's final row are discarded.
        data(0xf8, size=1)
        gc(0x43, 0x7f)  # The in-flight command retains its colors.
        data(0xffffe888)
        expected = bytearray([0xa5] * 1024)
        for row, bits in enumerate(('11111', '10001', '11101')):
            for x, bit in enumerate(bits):
                expected[(row + 2) * 32 + 3 + x] = 0x3c if bit == '1' else 0x96
        assert snapshot() == expected, 'opaque glyph/padding/bounds'
        assert not io.inb(0x63ca) & 0x40, 'completed transfer remains busy'
        assert not io.inb(0x33ce) & 1, 'completed transfer remains active'
        io.write(window(0x400000), 0x12345678)
        assert io.read(window(0x400000)) == 0x12345678, 'RAM alias not restored'

        initialize()
        command(6, 13)
        # The four doublewords observed in an original Xdec glyph upload.
        for value in (0x50000000, 0xf850f850, 0x00005050, 0):
            data(value)
        expected = bytearray([0xa5] * 1024)
        glyph = ('000000', '000000', '000000', '010100', '010100',
                 '111110', '010100', '111110', '010100', '010100',
                 '000000', '000000', '000000')
        for row, bits in enumerate(glyph):
            for x, bit in enumerate(bits):
                expected[(row + 2) * 32 + 3 + x] = 0x3c if bit == '1' else 0x96
        assert snapshot() == expected, 'packed Xdec glyph replay'
        assert not io.inb(0x63ca) & 0x40, 'packed glyph left engine busy'

        initialize()
        command(9, 2, align=3, transparent=True, rop=6)
        # First row spans two writes; second uses one word through the DOS
        # window. Leading three bits, trailing four bits and upper padding
        # are not part of the nine-pixel destination.
        data(0x15, size=1)
        assert io.inb(0x63ca) & 0x40, 'partial row completed command'
        data(0xa0, size=1)
        data(0xa0d8, size=2, legacy=True)
        expected = bytearray([0xa5] * 1024)
        for row, bits in enumerate(('101011010', '110001010')):
            for x, bit in enumerate(bits):
                if bit == '1':
                    expected[(row + 2) * 32 + 3 + x] = 0x3c ^ 0xa5
        assert snapshot() == expected, 'aligned transparent/XOR mono transfer'
        assert not io.inb(0x63ca) & 0x40, 'mixed-width transfer remains busy'

        initialize()
        io.outb(0x63ca, 3)
        gc(0x5a, 0xc0)
        gc(8, 0x5a)
        io.outb(0x33c7, 6)
        out(0x63c0, 0, 4)
        out(0x63cc, (2 << 16) | 3, 4)
        out(0x23ce, 8, 2)
        out(0x23c2, 3, 2)
        out(0x23c4, 2, 2)
        io.outb(0x33ce, 1)
        data(0x44332211)
        assert io.inb(0x63ca) & 0x40, 'packed upload completed before last row'
        data(0xffff6655)
        expected = bytearray([0xa5] * 1024)
        for row, colors in enumerate(((0x11, 0x22, 0x33), (0x44, 0x55, 0x66))):
            for x, color in enumerate(colors):
                expected[(row + 2) * 32 + 3 + x] = (0xa5 & ~0x5a) | (
                    (0xa5 ^ color) & 0x5a)
        assert snapshot() == expected, 'packed CPU upload row/pitch/ROP/mask'
        assert not io.inb(0x63ca) & 0x40, 'packed upload did not complete'
        gc(8, 0xff)

        command(8, 4)
        io.outb(0x33ce, 0)
        assert not io.inb(0x63ca) & 0x40, 'stop did not abort pending input'
        io.write(window(0x400000), 0x89abcdef)
        assert io.read(window(0x400000)) == 0x89abcdef, 'stop retained FIFO map'
        command(8, 4)
        gc(0x10, 0x48)
        assert not io.inb(0x63ca) & 0x40, 'BLT reset retained pending input'
        gc(0x10, 8)
        command(8, 1)
        io.outb(0x46e8, 6)
        data(0xff)
        io.outb(0x46e8, 14)
        assert io.inb(0x63ca) & 0x40, 'disabled board consumed host input'
        data(0xff)
        assert not io.inb(0x63ca) & 0x40, 're-enabled input did not complete'
        command(8, 4)
        io.vm.command('system_reset')
        assert not io.inb(0x63ca) & 0x40, 'device reset retained pending input'
        print('PASS: QVision mono/packed CPU uploads, byte-aligned rows, '
              'alignment, ROP/masks, mixed bus widths/windows, busy/stop/reset')
    finally:
        io.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--rom', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-qv-host-') as directory:
        check(args.qemu.resolve(), args.rom.resolve(), Path(directory))


if __name__ == '__main__':
    main()
