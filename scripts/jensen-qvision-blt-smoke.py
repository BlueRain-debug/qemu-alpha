#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check QVision's 8-bit solid Triton commands against VRAM contents."""

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

    def mem(offset):
        return 0x200000000 + ((0x400000 + offset) << 7) + 0x60

    def snapshot():
        return b''.join(io.read(mem(i)).to_bytes(4, 'little')
                        for i in range(0, 1024, 4))

    def initialize():
        for i in range(0, 1024, 4):
            io.write(mem(i), 0xa5a5a5a5)

    def color(value):
        for _ in range(2):
            out(0x33ca, value * 0x101, 2)
            out(0x33cc, value * 0x101, 2)

    def command(start, width, height, pitch, xy=False, reverse=False):
        out(0x63cc, start, 4)
        out(0x23c2, width, 2)
        out(0x23c4, height, 2)
        out(0x23ce, pitch // 4, 2)
        io.outb(0x33cf, 0x80 if xy else 0)
        io.outb(0x33ce, 0x41 if reverse else 1)

    try:
        gc(0x0f, 5)
        gc(0x10, 0)
        gc(0x40, 1)
        gc(0x10, 8)
        gc(0x48, 4)
        io.outb(0x3c4, 2)
        io.outb(0x3c5, 0xff)
        assert io.inb(0x3c5) == 0xff, 'AVGA pixel mask truncated to four bits'
        io.outb(0x63ca, 0xc3)
        assert io.inb(0x63ca) == 3, 'busy status must not latch guest writes'
        gc(8, 0xff)
        gc(0x5a, 6)
        initialize()
        color(0x37)
        command((2 << 16) | 3, 5, 3, 16, xy=True)
        expected = bytearray([0xa5] * 1024)
        for y in range(2, 5):
            expected[y * 16 + 3:y * 16 + 8] = bytes([0x37]) * 5
        assert snapshot() == expected, 'XY fill, pitch or adjacent bytes'
        assert not io.inb(0x33ce) & 1, 'completed command remains active'

        # Exhaust all 16 two-input raster operations using an independent
        # table of Boolean functions and a partial plane mask.
        ops = [lambda s, d: 0, lambda s, d: ~(s | d),
               lambda s, d: ~s & d, lambda s, d: ~s,
               lambda s, d: s & ~d, lambda s, d: ~d,
               lambda s, d: s ^ d, lambda s, d: ~(s & d),
               lambda s, d: s & d, lambda s, d: ~(s ^ d),
               lambda s, d: d, lambda s, d: ~s | d,
               lambda s, d: s, lambda s, d: s | ~d,
               lambda s, d: s | d, lambda s, d: 255]
        for rop, operation in enumerate(ops):
            initialize()
            color(0x3c)
            gc(0x5a, 0xc2)
            gc(8, 0x5a)
            io.outb(0x33c7, rop)
            command(100, 7, 2, 16)
            expected = bytearray([0xa5] * 1024)
            pixel = (0xa5 & ~0x5a) | (operation(0x3c, 0xa5) & 0x5a)
            for y in range(2):
                expected[100 + y * 16:107 + y * 16] = bytes([pixel]) * 7
            assert snapshot() == expected, f'ROP {rop:x} or plane mask'

        initialize()
        gc(0x5a, 6)
        color(0x69)
        command(120, 4, 3, 16, reverse=True)
        expected = bytearray([0xa5] * 1024)
        for y in range(3):
            expected[117 - y * 16:121 - y * 16] = bytes([0x69]) * 4
        assert snapshot() == expected, 'reverse fill coordinates'
        io.outb(0x33ce, 0x81)
        assert snapshot() == expected, 'unsupported command flag executed'
        assert io.inb(0x33ce) & 1, 'unexecuted command reported complete'
        command(0, 1024, 0, 1024)
        assert snapshot() == expected, 'zero-height command changed VRAM'
        gc(0x5a, 5)  # Screen source is not yet implemented.
        command(0, 16, 4, 16)
        assert snapshot() == expected, 'unsupported source executed as a fill'
        gc(0x5a, 6)
        out(0x33ca, 0x1111, 2)
        out(0x33cc, 0x1111, 2)
        out(0x33ca, 0x2222, 2)
        out(0x33cc, 0x2222, 2)
        command(0, 16, 4, 16)
        assert snapshot() == expected, 'nonuniform pattern executed'
        print('PASS: QVision solid XY/linear/reverse/pitch, 16 ROPs, '
              'plane masks, busy/completion and unsupported-source checks')
    finally:
        io.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--rom', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-qvision-blt-') as directory:
        check(args.qemu.resolve(), args.rom.resolve(), Path(directory))


if __name__ == '__main__':
    main()
