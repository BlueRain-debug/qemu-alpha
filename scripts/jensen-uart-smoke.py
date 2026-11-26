#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check Jensen's two VL82C106 16C450 UARTs through both I/O windows."""

import argparse
from pathlib import Path
import runpy
import tempfile

support = runpy.run_path(str(Path(__file__).with_name('jensen-smoke.py')))


def check(qemu, root):
    rom = root / 'pal.bin'
    rom.write_bytes(support['pal_test_rom']())
    io = support['DeviceIO'](qemu, rom, ['-serial', 'null', '-serial', 'null'])
    checks = 0
    try:
        children = io.vm.command('qom-list', {'path': '/machine/unattached'})
        cpu = next(child['name'] for child in children
                   if 'ev4-alpha-cpu' in child['type'])
        io.command(f'irq_intercept_in /machine/unattached/{cpu}')
        for base in (0x3f8, 0x2f8):
            for local in (False, True):
                def read(offset):
                    if local:
                        return io.read(0x1c0000000 + ((base + offset) << 9))
                    return io.inb(base + offset)

                def write(offset, value):
                    if local:
                        io.write(0x1c0000000 + ((base + offset) << 9), value)
                    else:
                        io.outb(base + offset, value)

                write(1, 0)
                write(3, 3)
                for value in (0x01, 0x07, 0xc7):
                    write(2, value)
                    assert read(2) == 1, '16C450 incorrectly reports a FIFO'
                    checks += 1
                for value in (0x55, 0xaa):
                    write(7, value)
                    assert read(7) == value, '16450 scratch register failed'
                    checks += 1
                write(3, 0x83)
                write(0, 12)
                write(1, 0)
                assert read(0) == 12 and read(1) == 0, 'Divisor latch failed'
                checks += 1
                write(3, 3)
                write(4, 0x1b)  # Internal loopback, OUT2, RTS and DTR.
                write(1, 1)
                write(0, 0x5a)
                assert io.irqs.get(5), 'UART receive did not reach IRQ5'
                write(2, 0xc7)  # An absent FCR must not discard the byte.
                assert read(5) & 1, 'Loopback byte not received'
                assert read(2) == 4, 'Receive interrupt identification failed'
                assert read(0) == 0x5a, 'Loopback byte changed'
                assert not io.irqs[5], 'UART receive IRQ5 did not clear'
                assert read(2) == 1, 'Receive interrupt did not clear'
                checks += 6
                write(1, 2)
                assert io.irqs[5], 'THRE did not reach IRQ5'
                assert read(2) == 2, 'THRE interrupt missing'
                assert not io.irqs[5], 'THRE IRQ5 acknowledgement failed'
                assert read(2) == 1, 'THRE acknowledgement failed'
                checks += 4
                write(1, 0)
                write(4, 0)
        io.vm.command('system_reset')
        for base in (0x3f8, 0x2f8):
            io.outb(base + 2, 0xc7)
            assert io.inb(base + 2) == 1, 'Reset enabled FIFO capability'
            checks += 1
    finally:
        io.close()
    print(f'PASS: {checks} VL82C106 UART checks; COM1/COM2, local/sparse I/O, '
          'no FIFO, scratch, divisor, loopback, RX/THRE IRQ5 and reset')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-uart-') as directory:
        check(args.qemu.resolve(), Path(directory))


if __name__ == '__main__':
    main()
