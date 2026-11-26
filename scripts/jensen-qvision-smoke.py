#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check optional Jensen QVision EISA, ROM, VGA, DAC and memory decoding."""

import argparse
import json
from pathlib import Path
import runpy
import tempfile

support = runpy.run_path(str(Path(__file__).with_name('jensen-qvision-testlib.py')))


def check(qemu, rom, root):
    bios = root / 'pal.bin'
    bios.write_bytes(support['pal_test_rom']())
    io = support['DeviceIO'](qemu, bios, [
        '-machine', 'qvision=on,qvision-romfile=' + rom.as_posix()])
    checks = []

    def expect(condition, name):
        assert condition, name
        checks.append(name)

    def gc(index, value):
        io.outb(0x3ce, index)
        io.outb(0x3cf, value)

    def read_gc(index):
        io.outb(0x3ce, index)
        return io.inb(0x3cf)

    def memory(address):
        return 0x200000000 + (address << 7) + 0x60

    try:
        expect(io.read(0x300000000 + (0x1c80 << 7) + 0x60) == 0x1130110e,
               'CPQ3011 EISA product ID')
        io.outb(0x1c80, 0)
        expect(io.inb(0x1c80) == 0x0e, 'product ID is read-only')
        expect(io.inb(0x1c84) == 1, 'primary board enabled')
        expect(io.inb(0x1c85) == 0x10, 'primary controller and extended ROM')
        io.outb(0x3c2, 1)
        io.outb(0x3d4, 0x0f)
        io.outb(0x3d5, 0x33)
        io.outb(0x46e8, 6)
        for crtc in (0x3d4, 0x3b4):
            io.outb(crtc, 0x0f)
            io.outb(crtc + 1, 0x5a)
            expect(io.inb(crtc + 1) == 0xff,
                   f'ROM other-adapter probe ignores disabled CRTC {crtc:#x}')
        expect(io.inb(0x46e8) == 6, 'disabled video enable remains accessible')
        io.outb(0x46e8, 14)
        expect(io.inb(0x3d5) == 0x33, 'disabled CRTC writes do not alter state')
        io.outb(0x46e8, 16)
        io.outb(0x0102, 1)
        expect(io.inb(0x0102) == 1 and io.inb(0x3ce) == 0xff,
               'setup mode keeps option register accessible and VGA disabled')
        io.outb(0x46e8, 8)
        io.outb(0x0102, 0)
        expect(io.inb(0x3ce) == 0xff, 'option-select command disable')
        io.outb(0x0102, 1)
        expect(io.inb(0x3ce) != 0xff, 'option-select command re-enable')
        gc(0x0f, 5)
        gc(0x10, 0)
        gc(0x40, 1)
        expect(read_gc(0x40) == 1,
               'AVGA enable before BitBLT unlock, as used by Xdec')
        gc(0x40, 0)
        expect(io.read(memory(0xc0000)) == int.from_bytes(rom.read_bytes()[:4],
                                                        'little'),
               'original ROM visible through EISA memory')
        expect(read_gc(0x0c) == 0x37, 'early QVision controller version')
        gc(0x0c, 0)
        expect(read_gc(0x0c) == 0x37, 'controller version is read-only')
        gc(0x48, 4)
        expect(read_gc(0x48) == 0, 'extended graphics register lock')
        gc(0x0f, 5)
        gc(0x10, 8)
        gc(0x48, 4)
        expect(read_gc(0x48) == 4, 'extended register unlock')
        io.write(0x400000, 0x12345678)
        io.write(memory(0x400000), 0x89abcdef)
        expect(io.read(memory(0x400000)) == 0x89abcdef,
               'linear VRAM write/read')
        expect(io.read(0x400000) == 0x12345678,
               'EISA framebuffer does not overwrite Alpha RAM')
        gc(0x48, 5)
        expect(io.read(memory(0x400000)) == 0xffffffff,
               'old high-address window removed')
        expect(io.read(memory(0x500000)) == 0x89abcdef,
               'VRAM preserved when high-address window moves')
        io.outb(0x46e8, 6)
        expect(io.read(memory(0x500000)) == 0xffffffff,
               'video disable removes high-address window')
        io.outb(0x46e8, 14)
        expect(io.read(memory(0x500000)) == 0x89abcdef,
               'video re-enable preserves framebuffer contents')
        io.outb(0x83c4, 2)
        expect(io.inb(0x3ce) == 0xff, 'unselected VGA ports float high')
        expect(io.read(memory(0x500000)) == 0xffffffff,
               'unselected high-address window disabled')
        io.outb(0x83c4, 0)
        expect(io.read(memory(0x500000)) == 0x89abcdef,
               'controller selection restores framebuffer window')
        io.outb(0x1c84, 0)
        expect(io.inb(0x3ce) == 0xff, 'EISA board disable')
        io.outb(0x1c84, 1)
        expect(read_gc(0x0c) == 0x37, 'EISA board re-enable')

        local = 0x1c0000000 + (0x102 << 9)
        expect(io.read(local) == 0x9d, 'local VL82C106 control register')
        io.outb(0x102, 0xa5)
        expect(io.inb(0x102) == 0xa5 and io.read(local) == 0x9d,
               'QVision EISA option register is separate from local chip')
        io.write(local, 0x5c)
        expect(io.inb(0x102) == 0xa5 and io.rtc_read(0x69) == 0x5c,
               'local control write preserves EISA option register')
        io.outb(0x83c6, 2)
        expect(io.inb(0x83c6) == 2, 'Bt484 command register zero')
        expect(io.inb(0x13c6) == 0x40, 'Bt484 status')
        io.outb(0x3c8, 0)
        cursor = bytes((i * 13 + 7) & 255 for i in range(256))
        for value in cursor:
            io.outb(0x13c7, value)
        io.outb(0x3c8, 0)
        expect(bytes(io.inb(0x13c7) for _ in cursor) == cursor,
               'Bt484 cursor RAM and address wrap')
        io.outb(0x83c8, 2)
        for value in (0x12, 0x34, 0x56):
            io.outb(0x83c9, value)
        io.outb(0x83c7, 2)
        expect([io.inb(0x83c9) for _ in range(3)] == [0x12, 0x34, 0x56],
               'Bt484 cursor palette')
        gc(0x40, 1)
        io.outb(0x3c4, 0)
        io.outb(0x3c5, 1)
        expect(read_gc(0x40) == 0 and read_gc(0x48) == 0 and
               read_gc(0x49) == 0,
               'sequencer reset clears AVGA and high-address registers')
        expect(io.read(memory(0x500000)) == 0xffffffff,
               'sequencer reset disables framebuffer window')
        gc(0x48, 5)
        io.vm.command('system_reset')
        expect(read_gc(0x0c) == 0x37 and io.inb(0x1c85) == 0x10,
               'reset restores controller identification and setup')
        expect(io.inb(0x46e8) == 8 and io.inb(0x0102) == 1,
               'reset restores VGA video and option-select enables')
        expect(io.read(memory(0x500000)) == 0xffffffff,
               'reset disables old framebuffer mapping')
        expect(io.inb(0x83c6) == 0 and io.read(local) == 0x9d,
               'reset restores DAC and local chipset control')
    finally:
        io.close()
    print(f'PASS: {len(checks)} QVision EISA/ROM/VGA/Bt484/window/reset checks')
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--rom', type=Path, required=True)
    parser.add_argument('--result', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-qvision-') as directory:
        checks = check(args.qemu.resolve(), args.rom.resolve(), Path(directory))
    if args.result:
        args.result.write_text(json.dumps(checks, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
