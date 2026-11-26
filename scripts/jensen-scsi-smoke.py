#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Check Jensen AHA-1742 CD and hard-disk DMA, sense and interrupts."""
import argparse
import hashlib
from pathlib import Path
import runpy
import struct
import tempfile
import time


def test_scsi(qemu, root):
    support = runpy.run_path(str(Path(__file__).with_name('jensen-smoke.py')))
    rom = root / 'pal.bin'
    rom.write_bytes(support['pal_test_rom']())
    disk = root / 'scsi-cd.iso'
    sector = bytes(range(256)) * 8
    disk.write_bytes(bytes(16 * 2048) + sector + bytes(111 * 2048))
    digest = hashlib.sha256(disk.read_bytes()).digest()
    harddisk = root / 'scsi-hd.raw'
    with harddisk.open('wb') as stream:
        stream.truncate(8 * 1024 * 1024)
    written = bytes((i * 17 + 3) & 255 for i in range(1024))
    scattered = bytes((i * 29 + 7) & 255 for i in range(1024))
    io = support['DeviceIO'](qemu, rom, [
        '-device', 'jensen-aha1742,id=aha,slot=6',
        '-drive', f'if=none,id=cd,media=cdrom,format=raw,readonly=on,'
                  f'file={disk.as_posix()}',
        '-device', 'scsi-cd,drive=cd,bus=aha.0,scsi-id=4',
        '-drive', f'if=none,id=hd,format=raw,file={harddisk.as_posix()}',
        '-device', 'scsi-hd,drive=hd,bus=aha.0,scsi-id=0'])
    io.vm.command('cont')  # Enable virtual timers; qtest executes no guest CPU.
    base, ecb, status = 0x6c80, 0x200000, 0x200100
    sense, buffer = 0x200180, 0x210000

    def write(addr, data):
        io.command(f'write {addr:#x} {len(data):#x} 0x{data.hex()}')

    def read(addr, size):
        return bytes.fromhex(io.command(f'read {addr:#x} {size:#x}')[0][2:])

    def start(cdb, length=0, data=buffer, flags=0x8400, target=4, cmd=1,
              pointer=ecb):
        block = bytearray(48)
        struct.pack_into('<HHH', block, 0, cmd, flags, 0)
        struct.pack_into('<III', block, 8, data, length, status)
        struct.pack_into('<I', block, 28, sense)
        block[32], block[33] = 96, len(cdb)
        block[36:36 + len(cdb)] = cdb
        write(pointer, block)
        write(status, bytes(32))
        io.write(0x300000000 + ((base + 0x50) << 7) + 0x60, pointer)
        io.outb(base + 0x54, 0x40 | target)

    def wait_completion():
        deadline = time.monotonic() + 5
        while not (io.inb(base + 0x57) & 2):
            assert time.monotonic() < deadline, 'SCSI completion timeout'
            io.command('clock_step 1000000')

    def finish(pointer=ecb):
        wait_completion()
        code = io.inb(base + 0x56)
        assert io.read(0x300000000 + ((base + 0x58) << 7) + 0x60) == pointer
        result = read(status, 32)
        io.outb(base + 0x55, 0x40)
        io.outb(0xa0, 0x20)
        io.outb(0x20, 0x20)
        return code, result

    try:
        assert bytes(io.inb(base + i) for i in range(4)) == b'\x04\x90\x00\x01'
        for port, value in [(0x20, 0x11), (0x21, 0x20), (0x21, 4), (0x21, 1),
                            (0xa0, 0x11), (0xa1, 0x28), (0xa1, 2), (0xa1, 1),
                            (0x21, 0xfb), (0xa1, 0xf7)]:
            io.outb(port, value)
        start(b'\x12\0\0\0\x24\0', 36)
        assert not io.inb(base + 0x57) & 2, 'Attention completed SCSI inline'
        assert read(status, 32) == bytes(32), 'Submission wrote completion status'
        assert read(buffer, 36) == bytes(36), 'Submission performed DMA inline'
        wait_completion()
        io.outb(0x20, 0x0a)
        assert io.inb(0x20) & 4, 'SCSI IRQ11 did not reach PIC cascade'
        io.read(0x100000000)  # Lock IRQ11; data is driven on cycle two.
        io.outb(0xa0, 0x0b)
        assert io.inb(0xa0) & 8, 'First INTA did not mark IRQ11 in service'
        io.write(0x100000000, 0)  # Writes must not consume the next cycle.
        assert io.read(0x100000000) == 0x2b, 'Wrong SCSI INTA vector'
        assert io.inb(0xa0) & 8, 'Second INTA acknowledged another IRQ'
        code, result = finish()
        assert code == 0x14 and result[2:4] == b'\0\0', (code, result)
        assert read(buffer, 36)[0] & 31 == 5, 'Not a SCSI CD'
        assert not io.inb(base + 0x57) & 2, 'Mailbox IRQ not acknowledged'
        # SRM polls INTPEND, acknowledges, then reads the completion latches.
        assert io.inb(base + 0x56) == 0x14, 'Acknowledgement lost G2INTST'
        assert io.read(0x300000000 + ((base + 0x58) << 7) + 0x60) == ecb, \
            'Acknowledgement lost mailbox-in'
        io.outb(base + 0x55, 0x40)
        assert io.inb(base + 0x56) == 0x14, 'Empty acknowledgement lost status'
        # Consume any genuine power-on unit attention using autosense.
        for _ in range(3):
            start(bytes(6))
            code, result = finish()
            if code == 0x14:
                break
            assert result[3] == 2 and read(sense, 14)[2] & 15 == 6, result
        else:
            raise AssertionError('CD remained not ready')
        cdb = b'\x28\0\0\0\0\x10\0\0\x01\0'
        start(cdb, 2048)
        assert finish()[0] == 0x14
        assert read(buffer, 2048) == sector, 'Direct CD DMA data mismatch'
        descriptors = struct.pack('<IIII', buffer, 512, buffer + 4096, 1536)
        write(0x200200, descriptors)
        write(buffer, b'\xa5' * 6144)
        start(cdb, len(descriptors), 0x200200, flags=0x9400)
        assert finish()[0] == 0x14
        assert read(buffer, 512) + read(buffer + 4096, 1536) == sector
        assert read(buffer + 512, 512) == b'\xa5' * 512, 'SG overwrote a gap'
        start(b'\xff\0\0\0\0\0')
        code, result = finish()
        assert code == 0xc4 and result[3] == 2, (code, result)
        assert read(sense, 14)[2] & 15 == 5, 'Missing illegal-command autosense'
        # DSB suppresses successful status writes, never error information.
        start(bytes(6), flags=0xc400)
        assert finish()[1] == bytes(32), 'Successful DSB wrote status'
        start(b'\xff\0\0\0\0\0', flags=0xc480)
        code, result = finish()
        assert code == 0xc4 and result[3] == 2, (code, result)
        assert struct.unpack_from('<H', result)[0] & 0x80
        assert read(sense, 14)[2] & 15 == 5, 'DSB/DI hid error sense'
        start(bytes(6), flags=0xc480)
        io.command('clock_step 1000000')
        assert not io.inb(base + 0x57) & 2, 'Successful DI raised completion'
        start(bytes(6), target=3)
        assert finish()[1][2] == 0x11, 'Absent target was reported present'
        # Cancel a queued command before it reaches the SCSI bus.
        start(b'\xff\0\0\0\0\0')
        io.outb(base + 0x54, 0x54)
        assert finish()[1][2] == 4, 'Queued abort did not complete once'
        io.command('clock_step 1000000')
        assert not io.inb(base + 0x57) & 2, 'Canceled timer completed twice'
        # Reset must cancel a queued DMA before its timer fires.
        write(buffer, b'\xa5' * 2048)
        start(cdb, 2048)
        io.outb(base + 0x55, 0x80)
        io.outb(base + 0x55, 0)
        io.command('clock_step 1000000')
        assert not io.inb(base + 0x57) & 2, 'Reset timer posted a stale completion'
        assert read(buffer, 2048) == b'\xa5' * 2048, 'Reset timer performed stale DMA'
        # Enhanced-mode HRST must preserve the ECU's EISA configuration.
        # SRM programs IRQ12 before HRST, then installs an IRQ12 handler.
        config = {4: 1, 0x40: 0x80, 0x42: 3, 0x43: 0xf8,
                  0x44: 2, 0x45: 1}
        for reg, value in config.items():
            io.outb(base + reg, value)
        io.outb(base + 0x55, 0x80)
        for reg, value in config.items():
            assert io.inb(base + reg) == value, (reg, 'HRST lost EISA config')
        assert io.inb(base + 0x57) == 5, 'HRST status missing'
        assert io.inb(base + 0x56) == 0, 'HRST retained completion status'
        assert io.read(0x300000000 + ((base + 0x58) << 7) + 0x60) == 0, \
            'HRST retained mailbox-in'
        io.outb(base + 0x55, 0)
        io.outb(base + 0x42, io.inb(base + 0x42) | 0x10)
        io.outb(0xa1, 0xef)  # IRQ12, rather than the power-on default IRQ11.
        start(b'', cmd=0)
        io.read(0x100000000)
        assert io.read(0x100000000) == 0x2c, 'HRST lost IRQ12 routing'
        assert finish()[0] == 0x14
        io.outb(base + 0x42, 0x12)
        io.outb(0xa1, 0xf7)
        # Multiple deferred commands must retain their distinct ECB pointers.
        start(b'\x12\0\0\0\x24\0', 36, pointer=ecb)
        start(b'\x12\0\0\0\x24\0', 36, pointer=ecb + 0x400)
        assert not io.inb(base + 0x57) & 2, 'Queued commands completed inline'
        assert finish(ecb)[0] == 0x14
        assert finish(ecb + 0x400)[0] == 0x14
        # Two synchronous completions must retain their distinct ECB pointers.
        start(b'', cmd=0, pointer=ecb)
        start(b'', cmd=0, pointer=ecb + 0x400)
        assert finish(ecb)[0] == 0x14
        assert finish(ecb + 0x400)[0] == 0x14
        start(cdb, 7, 0x200200, flags=0x9400)
        assert finish()[1][2] == 0x1a, 'Malformed SG list was accepted'
        io.outb(base + 0x55, 0x80)
        assert io.inb(base + 0x57) & 1
        io.outb(base + 0x55, 0)
        assert io.inb(base + 0x57) == 4
        # A reset between the two INTA cycles must discard the old vector.
        start(b'\x12\0\0\0\x24\0', 36)
        wait_completion()
        io.read(0x100000000)
        io.vm.command('system_reset')
        for port, value in [(0x20, 0x11), (0x21, 0x20), (0x21, 4), (0x21, 1),
                            (0xa0, 0x11), (0xa1, 0x28), (0xa1, 2), (0xa1, 1),
                            (0x21, 0xfb), (0xa1, 0xf7)]:
            io.outb(port, value)
        start(b'\x12\0\0\0\x24\0', 36)
        wait_completion()
        io.read(0x100000000)
        assert io.read(0x100000000) == 0x2b, 'Reset retained old INTA phase'
        assert finish()[0] == 0x14
        # Writable target 0 uses 512-byte blocks, distinct from the CD.
        start(b'\x12\0\0\0\x24\0', 36, target=0)
        assert finish()[0] == 0x10
        assert read(buffer, 36)[0] & 31 == 0, 'Not a SCSI hard disk'
        for _ in range(3):
            start(bytes(6), target=0)
            if finish()[0] == 0x10:
                break
        else:
            raise AssertionError('Hard disk remained not ready')
        start(b'\x25' + bytes(9), 8, target=0)
        assert finish()[0] == 0x10
        assert struct.unpack('>II', read(buffer, 8)) == (16383, 512)
        write(buffer, written)
        start(b'\x2a\0\0\0\0\x40\0\0\x02\0', 1024, target=0)
        assert finish()[0] == 0x10
        write(buffer, bytes(1024))
        start(b'\x28\0\0\0\0\x40\0\0\x02\0', 1024, target=0)
        assert finish()[0] == 0x10
        assert read(buffer, 1024) == written, 'Hard-disk DMA readback failed'
        write(buffer, scattered[:256])
        write(buffer + 4096, scattered[256:])
        write(0x200200, struct.pack('<IIII', buffer, 256, buffer + 4096, 768))
        start(b'\x2a\0\0\0\0\x80\0\0\x02\0', 16, 0x200200,
              flags=0x9400, target=0)
        assert finish()[0] == 0x10
        start(b'\x35' + bytes(9), target=0)
        assert finish()[0] == 0x10
    finally:
        io.close()
    assert hashlib.sha256(disk.read_bytes()).digest() == digest
    with harddisk.open('rb') as stream:
        stream.seek(64 * 512)
        assert stream.read(1024) == written, 'Direct write did not persist'
        stream.seek(128 * 512)
        assert stream.read(1024) == scattered, 'SG write did not persist'
    print('PASS: AHA-1742 CD direct/SG DMA, autosense, absent target, '
          'deferred execution, queued abort/reset, completion queue, '
          'IRQ11/IRQ12/PIC/INTA, HRST configuration and read-only image; '
          'hard-disk capacity, direct/SG writes, readback and persistence')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='jensen-scsi-test-') as directory:
        test_scsi(args.qemu.resolve(), Path(directory))


if __name__ == '__main__':
    main()
