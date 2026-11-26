#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check EV4 single-page invalidation after hardware TLB replacement."""

import argparse
import json
from pathlib import Path
import runpy
import struct
import tempfile
import time

pal = runpy.run_path(str(Path(__file__).with_name('jensen-smoke.py')))
mem, hwmem, ipr, shift = (pal[name] for name in ('mem', 'hwmem', 'ipr', 'shift'))


def constant(reg, value):
    # LDA sign-extends its low half, including the eviction VAs at 0x..8000.
    return [mem(9, reg, 31, (value + 0x8000) >> 16),
            mem(8, reg, reg, value)]


def mapping(va, pa):
    return (constant(2, va) + [ipr(0x1d, 2, 0x20)] +
            constant(1, pa >> 13) + [shift(1, 32),
            mem(8, 1, 1, 0x1100), ipr(0x1d, 1, 0x42)])


def test_rom(replacements, invalidate=True):
    # Entirely original PAL code. No firmware or NT bytes are included.
    code = [mem(8, 10, 31, 0x3000)] + constant(4, 0x400000)
    for offset, value in [(0, 0x1122), (8, 0x6677),
                          (0x2000, 0x3344), (0x2008, 0x7788)]:
        # HW_ST's displacement is 12 bits; use a separate physical base.
        code += constant(5, 0x400000 + offset)
        code += [mem(8, 3, 31, value),
                 hwmem(0x1f, 3, 5, quad=False)]
    code += mapping(0x100000, 0x400000)
    code += [mem(8, 3, 31, 0x1122), mem(0x2c, 3, 2),
             mem(0x28, 3, 2), hwmem(0x1f, 3, 10, 0x10)]
    # Insert entries without accessing those VAs: evict the architectural
    # mapping while retaining QEMU's cached read AND write translations.
    for index in range(replacements):
        code += mapping(0x200000 + index * 0x2000, 0x600000)
    if invalidate:
        code += constant(2, 0x100000) + [ipr(0x1d, 2, 0x48)]  # DTBIS
    code += mapping(0x100000, 0x402000)
    code += [mem(0x28, 3, 2), hwmem(0x1f, 3, 10, 0x18),
             mem(8, 3, 31, 0x5566), mem(0x2c, 3, 2)]
    for index, address in enumerate((0x400000, 0x402000, 0x400008, 0x402008)):
        code += constant(5, address)
        code += [hwmem(0x1b, 3, 5, quad=False),
                 hwmem(0x1f, 3, 10, 0x20 + index * 8)]
    code += [mem(8, 3, 31, 0x600d), hwmem(0x1f, 3, 10, 8), 0xc3ffffff]
    image = bytearray(1024 * 1024)
    struct.pack_into('<' + 'I' * len(code), image, 0x40, *code)
    return image


def check(qemu, root, replacements, invalidate=True):
    stem = ('dtbis' if invalidate else 'eviction') + f'-{replacements}'
    rom = root / (stem + '.bin')
    rom.write_bytes(test_rom(replacements, invalidate))
    vm = pal['VM'](qemu, rom, ['-serial', 'null'])
    try:
        vm.command('cont')
        deadline = time.monotonic() + 10
        while '0x000000000000600d' not in vm.monitor('xp /1gx 0x3008'):
            if time.monotonic() > deadline:
                raise AssertionError('DTBIS test did not complete: ' +
                                     vm.monitor('info registers'))
            time.sleep(.05)
        vm.command('stop')
        dump = root / (stem + '-results.bin')
        vm.monitor(f'pmemsave 0x3010 0x30 "{dump.as_posix()}"')
        values = struct.unpack('<6Q', dump.read_bytes())
    finally:
        vm.close()
    expected = (0x1122, 0x3344, 0x1122, 0x5566, 0x6677, 0x7788)
    failures = sum(a != b for a, b in zip(values, expected))
    print(f'{"FAIL" if failures else "PASS"}: {stem} '
          f'insertions: {[hex(v) for v in values]}')
    return {'insertions': replacements, 'invalidate': invalidate,
            'values': values, 'expected': expected,
            'checks': len(expected), 'failures': failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--result', type=Path, help='optional JSON results')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-tlb-') as directory:
        results = [check(args.qemu.resolve(), Path(directory), count)
                   for count in (0, 127, 128, 256)]
        # Replacing an evicted mapping must work without a later DTBIS.
        results += [check(args.qemu.resolve(), Path(directory), count, False)
                    for count in (128, 256)]
    if args.result is not None:
        args.result.write_text(json.dumps(results, indent=2) + '\n',
                               encoding='utf-8')
    failures = sum(item['failures'] for item in results)
    print(f'{sum(item["checks"] for item in results)} checks; {failures} failures')
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
