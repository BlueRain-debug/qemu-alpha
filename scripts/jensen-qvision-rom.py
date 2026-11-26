#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Prepare a 32 KiB QVision ROM bank from TRW's two EPROM dumps.

No vendor ROM bytes are included. The input archive must contain one
32 KiB U40 dump (even bytes) and one 32 KiB U21 dump (odd bytes).
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import zipfile


def prepare(archive, bank):
    if bank not in (0, 1):
        raise ValueError('ROM bank must be 0 or 1')
    if archive.stat().st_size > 1024 * 1024:
        raise ValueError('EPROM archive is unexpectedly large')
    chips = {}
    hashes = {}
    with zipfile.ZipFile(archive) as package:
        for chip in ('U40', 'U21'):
            suffix = '_' + chip.lower() + '.bin'
            entries = [entry for entry in package.infolist()
                       if not entry.is_dir() and
                       entry.filename.lower().endswith(suffix)]
            if len(entries) != 1 or entries[0].file_size != 32768:
                raise ValueError(f'expected exactly one 32 KiB {chip} dump')
            chips[chip] = package.read(entries[0])
            hashes[chip] = hashlib.sha256(chips[chip]).hexdigest()

    combined = bytearray(65536)
    combined[0::2] = chips['U40']
    combined[1::2] = chips['U21']
    image = bytes(combined[bank * 32768:(bank + 1) * 32768])
    if image[:2] != b'\x55\xaa':
        raise ValueError('interleaved bank has no VGA option ROM signature')
    declared = image[2] * 512
    if not 512 <= declared <= len(image):
        raise ValueError('invalid option ROM declared length')
    if sum(image[:declared]) & 255:
        raise ValueError('option ROM declared-region checksum is not zero')

    info = {'bank': bank, 'size': len(image), 'declared_size': declared,
            'declared_checksum': 0,
            'sha256': hashlib.sha256(image).hexdigest(),
            'eeprom_sha256': hashes}
    if image[3] == 0xe9:
        entry = (6 + struct.unpack_from('<h', image, 4)[0]) & 0xffff
        info['init_entry_offset'] = entry
        info['init_outside_declared_region'] = entry >= declared
    # Retain the whole bank: this BIOS's INIT code is at 0x7000, beyond
    # its advertised checksum region. Trimming to that length loses INIT.
    return image, info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True,
                        help='original qvision1024.zip from TRW')
    parser.add_argument('--output', type=Path, required=True,
                        help='new 32 KiB ROM file; existing files are refused')
    parser.add_argument('--bank', type=int, choices=(0, 1), default=0)
    args = parser.parse_args()
    try:
        image, info = prepare(args.input, args.bank)
        with args.output.open('xb') as stream:
            stream.write(image)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        parser.error(str(error))
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
