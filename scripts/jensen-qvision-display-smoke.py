#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check QVision linear 8-bit scanout and the Bt484 X cursor in screen dumps."""

import argparse
from pathlib import Path
import runpy
import tempfile

support = runpy.run_path(str(Path(__file__).with_name('jensen-qvision-testlib.py')))


def test_display(qemu, rom, root, width, height):
    bios = root / 'pal.bin'
    bios.write_bytes(support['pal_test_rom']())
    io = support['DeviceIO'](qemu, bios, [
        '-machine', 'qvision=on,qvision-romfile=' + rom.as_posix()])

    def reg(port, index, value):
        io.outb(port, index)
        io.outb(port + 1, value)

    def memory(offset):
        return 0x200000000 + ((0x400000 + offset) << 7) + 0x60

    def screen():
        image = root / 'screen.ppm'
        io.vm.command('screendump', {'filename': image.as_posix()})
        header, dimensions, maximum, pixels = image.read_bytes().split(b'\n', 3)
        assert (header, dimensions, maximum) == (
            b'P6', f'{width} {height}'.encode(), b'255'), dimensions
        return pixels

    try:
        io.outb(0x3c2, 1)
        for index, value in [(0x11, 0), (1, width // 8 - 1),
                             (0x12, (height - 1) & 255),
                             (7, 0x10 | (((height - 1) & 0x100) >> 7) |
                              (((height - 1) & 0x200) >> 3)),
                             (9, 0x40), (0x13, 128), (0x17, 0xab),
                             (0x18, 255)]:
            reg(0x3d4, index, value)
        for index, value in [(0, 3), (1, 1), (2, 15), (4, 14)]:
            reg(0x3c4, index, value)
        for index, value in [(5, 0), (6, 5), (0x0f, 5), (0x10, 0),
                             (0x40, 1), (0x10, 8), (0x48, 4)]:
            reg(0x3ce, index, value)
        io.outb(0x3c0, 0x10)
        io.outb(0x3c0, 1)
        io.outb(0x3c0, 0x20)
        io.outb(0x83c6, 2)  # Eight bits per palette component.
        io.outb(0x3c8, 0)
        colors = [bytes(rgb) for rgb in ((0, 0, 0), (255, 0, 0),
                                        (0, 255, 0), (0, 0, 255))]
        for rgb in colors:
            for component in rgb:
                io.outb(0x3c9, component)
        expected = bytearray(width * height * 3)
        # Check byte order, a nonzero row, padding at the right edge, and the
        # last visible line. VRAM's pitch is 1024 even in the 640-pixel mode.
        for x, y in [(0, 0), (12, 33), (width - 4, height - 1)]:
            io.write(memory(y * 1024 + x), 0x03020100)
            offset = (y * width + x) * 3
            expected[offset:offset + 12] = b''.join(colors)
        assert screen() == expected, 'wrong 8-bit scanout/stride/palette'
        io.outb(0x3ce, 5)
        assert io.inb(0x3cf) == 0, 'renderer changed guest GR05 readback'

        # Solid foreground X cursor: plane 0 and plane 1 both set. The first
        # pixel must land at (8,16), not at the DAC's coordinate bias (32,32).
        io.outb(0x83c8, 2)
        for component in (255, 255, 255):
            io.outb(0x83c9, component)
        io.outb(0x3c8, 0)
        for _ in range(256):
            io.outb(0x13c7, 255)
        io.outb(0x93c8, 40)
        io.outb(0x93c6, 48)
        io.outb(0x13c9, 3)
        cursor = bytearray(expected)
        for y in range(16, 48):
            offset = (y * width + 8) * 3
            cursor[offset:offset + 32 * 3] = bytes([255]) * 32 * 3
        assert screen() == cursor, 'wrong Bt484 X cursor composition'
        io.outb(0x13c9, 0)
        assert screen() == expected, 'cursor left trails or modified VRAM'
        print(f'PASS: QVision {width}x{height}x256, stride 1024, '
              'palette, X cursor and removal')
    finally:
        io.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--rom', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-qvision-display-') as directory:
        for width, height in [(640, 480), (1024, 768)]:
            test_display(args.qemu.resolve(), args.rom.resolve(),
                         Path(directory), width, height)


if __name__ == '__main__':
    main()
