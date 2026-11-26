#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Exercise Jensen's EV4 PAL instructions and, optionally, a local SRM ROM."""

import argparse
import json
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time


def mem(op, ra, rb, disp=0):
    return op << 26 | ra << 21 | rb << 16 | (disp & 0xffff)


def hwmem(op, ra, rb, disp=0, mode=4, quad=True):
    return mem(op, ra, rb, mode << 13 | int(quad) << 12 | (disp & 0xfff))


def ipr(op, reg, index):
    return mem(op, reg, reg, index)


def shift(reg, count):
    return 0x12 << 26 | reg << 21 | count << 13 | 1 << 12 | 0x39 << 5 | reg


def pal_test_rom():
    """Small original PAL program; no DEC firmware is embedded in this test."""
    code = [mem(8, 10, 31, 0x5000), hwmem(0x1f, 20, 10),
            mem(8, 1, 31, 0x1234), hwmem(0x1f, 1, 10, 0x100)]
    for mode, offset in [(6, 0x20), (7, 0x30)]:
        code += [hwmem(0x1b, 1, 10, 0x100, mode),
                 mem(8, 3, 1),
                 hwmem(0x1f, 1, 10, 0x100, mode),
                 hwmem(0x1f, 1, 10, offset),
                 hwmem(0x1f, 3, 10, offset + 8)]
    # Retain PA bit 33, and retain PTE readback across TB invalidation.
    code += [mem(8, 1, 31, 1), shift(1, 52)]
    for pte, zap, temp, offset in [(0x21, 0x26, 0x23, 0x40),
                                   (0x42, 0x46, 0x43, 0x48)]:
        code += [ipr(0x1d, 1, pte), ipr(0x1d, 31, zap)]
        # ITB_PTE reads initiate a new array access. Read its holding
        # register directly to check invalidation preserves existing data.
        if pte != 0x21:
            code += [ipr(0x19, 31, pte)]
        code += [ipr(0x19, 3, temp), hwmem(0x1f, 3, 10, offset)]
    # Put an observable value in the PIC mask through physical sparse I/O.
    code += [mem(8, 4, 31, 3), shift(4, 32), mem(8, 4, 4, 0x1000),
             mem(8, 5, 31, 0xa5), shift(5, 8),
             hwmem(0x1f, 5, 4, 0x80, quad=False)]
    # Map VA 0x20000 to EISA I/O PA 0x300000000, then read through the DTB.
    code += [mem(8, 2, 31, 2), shift(2, 16), ipr(0x1d, 2, 0x20),
             mem(8, 1, 31, 3), shift(1, 51), mem(8, 1, 1, 0x1100),
             ipr(0x1d, 1, 0x42), mem(0x28, 3, 2, 0x1080),
             hwmem(0x1f, 3, 10, 0x50),
             # A write to the read-only INTA window must not crash the host.
             mem(8, 4, 31, 1), shift(4, 32),
             hwmem(0x1f, 31, 4, quad=False),
             # HIRR is read-only state: writes must leave requests unchanged.
             ipr(0x19, 6, 0x2c), ipr(0x1d, 31, 0x2c),
             ipr(0x19, 3, 0x2c),
             0x11 << 26 | 6 << 21 | 3 << 16 | 0x40 << 5 | 3,
             hwmem(0x1f, 3, 10, 0x58),
             mem(8, 1, 31, -1), ipr(0x1d, 1, 0x2c),
             ipr(0x19, 3, 0x2c),
             0x11 << 26 | 6 << 21 | 3 << 16 | 0x40 << 5 | 3,
             hwmem(0x1f, 3, 10, 0x60),
             mem(8, 1, 31, 0x600d), hwmem(0x1f, 1, 10, 8),
             0xc3ffffff]  # br zero, .
    image = bytearray(1024 * 1024)
    struct.pack_into('<' + 'I' * len(code), image, 0x40, *code)
    return image


class VM:
    def __init__(self, qemu, rom, serial, extra=()):
        flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        arguments = [str(qemu), '-machine', 'jensen', '-display', 'none',
                     '-monitor', 'none', '-qmp', 'stdio', '-S']
        if rom is not None:
            arguments += ['-bios', str(rom)]
        self.process = subprocess.Popen(
            arguments + serial + list(extra), stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            creationflags=flags)
        self.replies = queue.Queue()
        threading.Thread(target=self.reader, daemon=True).start()
        self.receive()
        self.command('qmp_capabilities')

    def reader(self):
        try:
            for line in self.process.stdout:
                self.replies.put(json.loads(line))
        finally:
            self.replies.put(None)

    def receive(self):
        while True:
            reply = self.replies.get(timeout=15)
            if reply is None:
                status = self.process.wait(timeout=10)
                raise RuntimeError(f'QEMU exited ({status:#x}): ' +
                                   self.process.stderr.read())
            if 'event' not in reply:
                return reply

    def command(self, name, arguments=None):
        request = {'execute': name}
        if arguments is not None:
            request['arguments'] = arguments
        self.process.stdin.write(json.dumps(request) + '\n')
        self.process.stdin.flush()
        reply = self.receive()
        if 'error' in reply:
            raise RuntimeError(reply['error'])
        return reply['return']

    def monitor(self, command):
        return self.command('human-monitor-command', {'command-line': command})

    def close(self):
        try:
            if self.process.poll() is None:
                self.command('quit')
                self.process.wait(timeout=10)
        except (OSError, RuntimeError, queue.Empty, subprocess.TimeoutExpired):
            pass
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            for stream in [self.process.stdin, self.process.stdout,
                           self.process.stderr]:
                try:
                    stream.close()
                except OSError:
                    pass


def test_pal(qemu, root):
    rom = root / 'pal-test.bin'
    rom.write_bytes(pal_test_rom())
    vm = VM(qemu, rom, ['-serial', 'null'])
    try:
        vm.command('cont')
        deadline = time.monotonic() + 10
        while '0x000000000000600d' not in vm.monitor('xp /1gx 0x5008'):
            if time.monotonic() > deadline:
                raise AssertionError('EV4 PAL test did not finish\n' +
                                     vm.monitor('info registers'))
            time.sleep(.05)
        vm.command('stop')
        dump = (root / 'results.bin').as_posix()
        vm.monitor(f'pmemsave 0x5000 0x68 "{dump}"')
        values = struct.unpack('<13Q', Path(dump).read_bytes())
        assert values[0] == 64, values
        assert values[4:8] == (1, 0x1234, 1, 0x1234), values
        assert values[8:10] == (1 << 33, 1 << 33), values
        assert values[10] == 0xa500, values
        assert values[11:] == (0, 0), values
        print('PASS: EV4 physical LL/SC, 34-bit PTEs, PTE readback, '
              'EISA I/O, HIRR writes ignored')
    finally:
        vm.close()



def test_mmcsr(qemu, root):
    """Faulting HW_LD/HW_ST must report RA in MMCSR, not EXC_SUM."""
    for opcode, ra in [(0x1b, 2), (0x1f, 17)]:
        image = bytearray(1024 * 1024)
        code = [mem(8, 10, 31, 0x6000), mem(8, 4, 31, 2), shift(4, 16),
                hwmem(opcode, ra, 4, mode=0), 0xc3ffffff]
        handler = [ipr(0x19, 3, 0x44), hwmem(0x1f, 3, 10),
                   ipr(0x19, 3, 0x2a), hwmem(0x1f, 3, 10, 8),
                   ipr(0x19, 3, 0x45), hwmem(0x1f, 3, 10, 16),
                   mem(8, 3, 31, 0x600d), hwmem(0x1f, 3, 10, 24),
                   0xc3ffffff]
        struct.pack_into('<' + 'I' * len(code), image, 0x40, *code)
        struct.pack_into('<' + 'I' * len(handler), image, 0x9e0, *handler)
        rom = root / 'mmcsr-test.bin'
        rom.write_bytes(image)
        vm = VM(qemu, rom, ['-serial', 'null'])
        try:
            vm.command('cont')
            deadline = time.monotonic() + 5
            while '0x000000000000600d' not in vm.monitor('xp /1gx 0x6018'):
                assert time.monotonic() < deadline, vm.monitor('info registers')
                time.sleep(.05)
            vm.command('stop')
            dump = (root / 'mmcsr-results.bin').as_posix()
            vm.monitor(f'pmemsave 0x6000 0x20 "{dump}"')
            result = struct.unpack('<4Q', Path(dump).read_bytes())
            expected = (opcode << 9) | (ra << 4) | int(opcode == 0x1f)
            assert result[:3] == (expected, 0, 0x20000), (opcode, result)
        finally:
            vm.close()
    print('PASS: EV4 fault MMCSR opcode/RA/write bit, EXC_SUM and VA')


class DeviceIO:
    def __init__(self, qemu, rom, extra=()):
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        self.vm = VM(qemu, rom, ['-serial', 'null'],
                     ['-accel', 'qtest', '-qtest-log', os.devnull, '-qtest',
                      f'tcp:127.0.0.1:{port},server=on,wait=off'] + list(extra))
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=10)
        self.stream = self.socket.makefile('rwb', buffering=0)
        self.irqs = {}

    def command(self, command):
        self.stream.write((command + '\n').encode('ascii'))
        while True:
            reply = self.stream.readline().decode('ascii').split()
            if reply and reply[0] == 'IRQ':
                self.irqs[int(reply[2])] = reply[1] == 'raise'
                continue
            assert reply and reply[0] == 'OK', reply
            return reply[1:]

    def read(self, addr):
        return int(self.command(f'readl {addr:#x}')[0], 0)

    def write(self, addr, value):
        self.command(f'writel {addr:#x} {value:#x}')

    def inb(self, port):
        return (self.read(0x300000000 + (port << 7)) >> ((port & 3) * 8)) & 255

    def outb(self, port, value):
        self.write(0x300000000 + (port << 7), value << ((port & 3) * 8))

    def rtc_read(self, index):
        self.write(0x1c0000000 + (0x170 << 9), index)
        return self.read(0x1c0000000 + (0x171 << 9))

    def rtc_write(self, index, value):
        self.write(0x1c0000000 + (0x170 << 9), index)
        self.write(0x1c0000000 + (0x171 << 9), value)

    def close(self):
        self.stream.close()
        self.socket.close()
        self.vm.close()


def test_flash_rtc(qemu, root):
    rom = root / 'pal-test.bin'
    rom.write_bytes(pal_test_rom())
    rtc = root / 'rtc-ram.bin'
    nvram = root / 'config-flash.bin'
    nvram.write_bytes(rom.read_bytes())
    extra = ['-machine', f'rtc-ram-file={rtc.as_posix()}', '-drive',
             f'if=pflash,index=1,format=raw,file={nvram.as_posix()}']
    for cycle in range(2):
        # Also exercise the system-ROM pflash input, without -bios.
        source = rom if cycle == 0 else None
        drives = extra if cycle == 0 else extra + ['-drive',
                 f'if=pflash,index=0,format=raw,readonly=on,file={rom.as_posix()}']
        io = DeviceIO(qemu, source, drives)
        try:
            assert io.rtc_read(0x0d) == 0x80
            assert io.read(0x300000000 + (0xc80 << 7) + 0x60) == 0x0024a310
            assert io.read(0x300000000 + (0x2c80 << 7) + 0x60) == 0xffffffff
            if cycle == 0:
                for index, value in [(0x0e, 0x5a), (0x32, 0xa5), (0x4f, 0x33)]:
                    io.rtc_write(index, value)
                base = 0x1a0000000 + (0x1234 << 9)
                for value in [0x40, 0x5a, 0xff]:
                    io.write(base, value)
            assert io.read(0x1a0000000 + (0x1234 << 9)) == 0x5a
            for index, value in [(0x0e, 0x5a), (0x32, 0xa5), (0x4f, 0x33)]:
                assert io.rtc_read(index) == value
            # Sparse access uses the same RTC bridge, including the byte lane.
            io.outb(0x170, 0x32)
            assert io.inb(0x171) == 0xa5
            io.rtc_write(0x50, 0x11)
            assert io.rtc_read(0x50) == 0xff
            io.vm.command('system_reset')
            assert io.rtc_read(0x32) == 0xa5
            assert io.read(0x1a0000000 + (0x1234 << 9)) == 0x5a
        finally:
            io.close()
    assert rtc.stat().st_size == 66
    assert nvram.read_bytes()[0x1234] == 0x5a
    # Read-only Flash must report an error and preserve its backing file.
    before = nvram.read_bytes()
    io = DeviceIO(qemu, rom, ['-drive',
                  f'if=pflash,index=1,format=raw,readonly=on,file={nvram.as_posix()}'])
    try:
        base = 0x1a0000000 + (0x1234 << 9)
        for value in [0x40, 0xa5]:
            io.write(base, value)
        assert io.read(base) == 0x90
        io.write(base, 0xff)
        assert io.read(base) == 0x5a
    finally:
        io.close()
    assert nvram.read_bytes() == before
    assert rom.read_bytes() == pal_test_rom()
    print('PASS: Flash/RTC warm+cold persistence, readonly Flash, pflash ROM')


def test_keyboard_vga(qemu, root):
    rom = root / 'keyboard-test.bin'
    rom.write_bytes(pal_test_rom())
    for locked, vga in [(False, 'cirrus'), (True, 'std')]:
        io = DeviceIO(qemu, rom, ['-machine',
                      f'keyboard-locked={"on" if locked else "off"}',
                      '-vga', vga])
        try:
            io.outb(0x64, 0xc0)
            assert io.inb(0x60) == (0x80 if locked else 0xc0)
            io.outb(0x64, 0xaa)
            assert io.inb(0x60) == 0x55
            io.outb(0x60, 0xff)
            assert [io.inb(0x60), io.inb(0x60)] == [0xfa, 0xaa]
            io.outb(0x60, 0xf2)
            assert [io.inb(0x60) for _ in range(3)] == [0xfa, 0xab, 0x83]
            children = io.vm.command('qom-list', {'path': '/machine/unattached'})
            cpu = next(child['name'] for child in children
                       if 'ev4-alpha-cpu' in child['type'])
            io.command(f'irq_intercept_in /machine/unattached/{cpu}')
            io.outb(0x64, 0x60)
            io.outb(0x60, 3)
            for command, value in [(0xd2, 0x5a), (0xd3, 0x33)]:
                io.outb(0x64, command)
                io.outb(0x60, value)
                assert io.irqs[3]
                assert bool(io.inb(0x64) & 0x20) == (command == 0xd3)
                assert io.inb(0x60) == value
                assert not io.irqs[3]
            # Real host key events produce set-2 make/break scan codes.
            io.vm.command('cont')
            io.vm.command('send-key', {'keys': [{'type': 'qcode', 'data': 'a'}],
                                      'hold-time': 50})
            assert io.inb(0x60) == 0x1c
            io.command('clock_step 100000000')
            assert [io.inb(0x60), io.inb(0x60)] == [0xf0, 0x1c]
            base = 0x200000000 + (0xc0000 << 7) + 0x60
            header = io.read(base)
            assert header & 0xffff == 0xaa55
            io.write(base, 0)
            assert io.read(base) == header
            assert io.inb(0x102) == io.rtc_read(0x69) == 0x9d
            io.outb(0x102, 0x25)
            assert io.rtc_read(0x69) == 0x25
            io.rtc_write(0x69, 0x9d)
            assert io.inb(0x102) == 0x9d
            io.vm.command('system_reset')
            assert io.read(base) == header
        finally:
            io.close()
    print('PASS: key-lock, PS/2 input, IRQ3, VGA ROM, VL82C106 alias')


def test_floppy(qemu, root):
    rom = root / 'pal-test.bin'
    rom.write_bytes(pal_test_rom())
    disk = root / 'floppy.img'
    pattern = bytes(range(256)) * 2
    disk.write_bytes(pattern + bytes(1440 * 1024 - len(pattern)))
    io = DeviceIO(qemu, rom, ['-drive',
                  f'if=floppy,index=0,format=raw,file={disk.as_posix()}'])
    try:
        for value in [0, 0x1c]:
            io.outb(0x3f2, value)
        for _ in range(4):
            io.outb(0x3f5, 8)
            io.inb(0x3f5)
            io.inb(0x3f5)
        for port, value in [(0x20, 0x11), (0x21, 0x20), (0x21, 4),
                            (0x21, 1), (0x21, 0xbf)]:
            io.outb(port, value)
        for value in [3, 0xdf, 2]:  # SPECIFY, DMA enabled
            io.outb(0x3f5, value)
        # DMA channel 2, one sector into system RAM at 2 MiB.
        for port, value in [(0xa, 6), (0xc, 0), (4, 0), (4, 0),
                            (5, 0xff), (5, 1), (0x81, 0x20), (0x481, 0),
                            (0xb, 0x46), (0xa, 2)]:
            io.outb(port, value)
        io.outb(0x20, 0x0a)
        assert io.inb(0x20) == 0
        for value in [0x46, 0, 0, 0, 1, 2, 1, 0x1b, 0xff]:
            io.outb(0x3f5, value)
        io.command('clock_step 500000000')
        assert io.inb(0x3f4) == 0xd0
        assert io.inb(0x20) & 0x40
        io.read(0x100000000)  # First INTA cycle locks the selected IRQ.
        assert io.read(0x100000000) == 0x26
        data = bytes.fromhex(io.command('read 0x200000 0x200')[0][2:])
        assert data == pattern
        result = [io.inb(0x3f5) for _ in range(7)]
        assert result[:3] == [0x20, 0, 0], result
    finally:
        io.close()
    print('PASS: floppy DMA sector read into RAM, IRQ6, PIC/INTA vector')


class Console:
    def __init__(self, qemu, rom, extra=()):
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        self.vm = VM(qemu, rom, ['-chardev',
                     f'socket,id=com,host=127.0.0.1,port={port},'
                     'server=on,wait=off', '-serial', 'chardev:com'], extra)
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=10)
        self.socket.settimeout(.2)
        self.output = bytearray()
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()

    def reader(self):
        while self.vm.process.poll() is None:
            try:
                chunk = self.socket.recv(4096)
                if not chunk:
                    return
                self.output.extend(chunk)
            except socket.timeout:
                continue
            except OSError:
                return

    def wait_for(self, pattern, start=0):
        deadline = time.monotonic() + 30
        while pattern not in self.output[start:]:
            if time.monotonic() > deadline:
                raise AssertionError('Missing SRM output: ' + repr(pattern) +
                                     '\n' + self.output.decode(errors='replace'))
            time.sleep(.05)

    def enter_srm(self, start=0):
        deadline = time.monotonic() + 30
        while (b'>>>' not in self.output[start:] and
               b'Continue NT boot (Y/N)?' not in self.output[start:]):
            if time.monotonic() > deadline:
                raise AssertionError('No SRM prompt: ' +
                                     self.output.decode(errors='replace'))
            time.sleep(.05)
        if b'>>>' not in self.output[start:]:
            self.socket.sendall(b'N')
            self.wait_for(b'>>>', start)
        time.sleep(.3)

    def enter_arc(self, start=0):
        """Allow clean NT POST, while retaining genuine error confirmation."""
        deadline = time.monotonic() + 30
        while True:
            output = self.output[start:]
            if b'Continue NT boot (Y/N)?' in output:
                self.socket.sendall(b'Y')
                return
            if b'System power up OK.' in output:
                # OS_TYPE=NT switches to ARC itself; OSF exposes an SRM prompt.
                time.sleep(.3)
                if b'>>>' not in self.output[start:]:
                    return
            if b'>>>' in self.output[start:]:
                for char in b'runnt\r':
                    self.socket.sendall(bytes([char]))
                    time.sleep(.05)
                return
            if time.monotonic() > deadline:
                raise AssertionError('No POST result or SRM prompt: ' +
                                     self.output.decode(errors='replace'))
            time.sleep(.05)

    def boot(self):
        self.vm.command('cont')
        self.enter_srm()

    def command(self, command):
        start = len(self.output)
        for char in (command + '\r').encode('ascii'):
            self.socket.sendall(bytes([char]))
            time.sleep(.05)
        self.wait_for(b'>>>', start)
        return bytes(self.output[start:])

    def reset(self):
        self.vm.command('stop')
        start = len(self.output)
        self.vm.command('system_reset')
        self.vm.command('cont')
        self.enter_srm(start)

    def close(self):
        self.vm.close()
        self.socket.close()
        self.thread.join(timeout=2)


def test_firmware(qemu, rom, log):
    console = Console(qemu, rom)
    try:
        console.boot()
        result = console.command('show config')
        assert b'64MB' in result and b'CPU         OK EV4' in result, result
        assert b'Alpha AXP SRM Firmware Version' in result, result
        assert b'INIT-S-CPU' in console.command('init')
        # Clean NT POST now enters ARC; this test explicitly requests SRM.
        assert b'OS_TYPE = OSF' in console.command('set os OSF')
        console.reset()
        assert b'64MB' in console.command('show config')
        print('PASS: DEC V2.2 SRM prompt, EV4/64MB, init, system reset')
    finally:
        console.close()
        if log is not None:
            log.write_bytes(console.output)


def invoke_ecu(console):
    """Run the original SRM command; its confirmation needs Y and Return."""
    start = len(console.output)
    for char in b'runecu\r':
        console.socket.sendall(bytes([char]))
        time.sleep(.05)
    console.wait_for(b'Continue (Y/N)?', start)
    console.socket.sendall(b'Y\r')


def switch_to_vga(console):
    """Use the original firmware command; output moves away from serial."""
    for char in b'set console VGA\r':
        console.socket.sendall(bytes([char]))
        time.sleep(.05)
    time.sleep(1)


def type_vga(console, text):
    """Send real PS/2 keys to the firmware's graphical console."""
    for char in text:
        key = {' ': 'spc', '\n': 'ret'}.get(char, char)
        console.vm.command('send-key', {
            'keys': [{'type': 'qcode', 'data': key}], 'hold-time': 30})
        time.sleep(.08)


def test_graphics(qemu, rom, screenshot):
    console = Console(qemu, rom)
    try:
        console.boot()
        switch_to_vga(console)
        type_vga(console, 'show config\n')
        time.sleep(.5)
        console.vm.command('stop')
        # Read actual VGA text cells through the EISA window, not serial echoes.
        chars = bytearray()
        for offset in range(0, 640, 4):
            addr = 0x200000000 + ((0xb8000 + offset) << 7) + 0x60
            result = console.vm.monitor(f'xp /1wx {addr:#x}')
            word = int(result.split(':')[-1].strip(), 16)
            chars.extend((word & 255, (word >> 16) & 255))
        assert b'>>> show config' in chars, chars
        assert b'DEC 2000 Model 300 AXP - ROM Version 2.2' in chars, chars
        if screenshot is not None:
            console.vm.command('screendump', {
                'filename': screenshot.resolve().as_posix(), 'format': 'png'})
        print('PASS: firmware VGA prompt and PS/2-typed show config on screen')
    finally:
        console.close()


def wait_trace(path, pattern, timeout=60):
    deadline = time.monotonic() + timeout
    while pattern not in path.read_text(errors='replace'):
        if time.monotonic() > deadline:
            raise AssertionError('Missing floppy trace: ' + pattern)
        time.sleep(.1)


def test_ecu(qemu, rom, disk, root, log):
    events = root / 'ecu.events'
    events.write_text('fdc_ioport_write\n')
    trace = root / 'ecu-fdc.txt'
    trace.touch()
    console = Console(qemu, rom, ['-drive',
                      'if=floppy,index=0,format=raw,readonly=on,file=' +
                      disk.resolve().as_posix(), '-trace',
                      f'events={events.as_posix()},file={trace.as_posix()}'])
    try:
        console.boot()
        invoke_ecu(console)
        # ARC initializes the floppy before displaying its language menu.
        wait_trace(trace, 'write reg 0x05 val 0xdf')
        time.sleep(2)
        if log is not None:
            console.vm.command('screendump', {
                'filename': log.with_suffix('.arc.png').resolve().as_posix(),
                'format': 'png'})
        type_vga(console, '\n')
        wait_trace(trace, 'write reg 0x05 val 0x4a')
        print('PASS: RUNECU enters ARC; PS/2 Return starts floppy READ ID',
              flush=True)
        time.sleep(2)
        if log is not None:
            console.vm.command('screendump', {
                'filename': log.with_suffix('.ecu.png').resolve().as_posix(),
                'format': 'png'})
        # Issuing a command is not proof that CF.EXE was loaded. Keep the
        # screen and trace for review, and report the remaining limitation.
        reads = ('write reg 0x05 val 0x46', 'write reg 0x05 val 0x66')
        if not any(command in trace.read_text() for command in reads):
            print('KNOWN LIMITATION: ECU has not reached a sector read; '
                  'review the saved screen', flush=True)
    finally:
        console.close()
        if log is not None:
            log.with_suffix('.ecu-serial.txt').write_bytes(console.output)
            log.with_suffix('.ecu-fdc.txt').write_bytes(trace.read_bytes())


def test_persistence(qemu, rom, root, log):
    nvram = root / 'config-flash.bin'
    rtc = root / 'rtc-ram.bin'
    nvram.write_bytes(rom.read_bytes())
    extra = ['-machine', f'rtc-ram-file={rtc.as_posix()}',
             '-drive', f'if=pflash,index=1,format=raw,file={nvram.as_posix()}']
    output = bytearray()
    for cycle in range(2):
        console = Console(qemu, rom, extra)
        try:
            console.boot()
            if cycle == 0:
                assert b'OS_TYPE = OSF' in console.command('set os OSF')
                assert b'AUTO_ACTION = HALT' in console.command(
                    'set auto_action HALT')
                assert b'BOOT_OSFLAGS = QEMU2' in console.command(
                    'set boot_osflags QEMU2')
                console.reset()
            # Completing POST must also restore SRM's environment caches.
            flags = console.command('show boot_osflags')
            action = console.command('show auto_action')
            assert b'BOOT_OSFLAGS = QEMU2' in flags, flags
            assert b'AUTO_ACTION = HALT' in action, action
            assert rtc.read_bytes()[0x26 - 0x0e:0x2b - 0x0e] == b'QEMU2'
            assert b'OS_TYPE = OSF' in console.command('show os')
            config = console.command('show config')
            assert b'NVR         OK' in config
            assert b'ROM Version 2.2' in config and b'Version - 370' in config
        finally:
            console.close()
            output.extend(console.output)
            if log is not None:
                log.write_bytes(output)
    assert rtc.stat().st_size == 66
    assert nvram.read_bytes() != rom.read_bytes()
    print('PASS: SRM OSF, boot flags/action and RTC/Flash survive '
          'warm/cold reset; NVR OK')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qemu', type=Path, required=True)
    parser.add_argument('--firmware', type=Path,
                        help='optional extracted DEC 2000 V2.2 ROM')
    parser.add_argument('--console-log', type=Path)
    parser.add_argument('--ecu-disk', type=Path,
                        help='optional local ECU disk; requires --firmware')
    args = parser.parse_args()
    if args.ecu_disk is not None and args.firmware is None:
        parser.error('--ecu-disk requires --firmware')
    with tempfile.TemporaryDirectory(prefix='jensen-test-') as directory:
        test_pal(args.qemu.resolve(), Path(directory))
        test_mmcsr(args.qemu.resolve(), Path(directory))
        test_flash_rtc(args.qemu.resolve(), Path(directory))
        test_floppy(args.qemu.resolve(), Path(directory))
        test_keyboard_vga(args.qemu.resolve(), Path(directory))
    if args.firmware is not None:
        test_firmware(args.qemu.resolve(), args.firmware.resolve(),
                      args.console_log)
        screenshot = (args.console_log.with_suffix('.vga.png')
                      if args.console_log is not None else None)
        test_graphics(args.qemu.resolve(), args.firmware.resolve(), screenshot)
        with tempfile.TemporaryDirectory(prefix='jensen-nvram-') as directory:
            persist_log = (args.console_log.with_suffix('.persistent.txt')
                           if args.console_log is not None else None)
            test_persistence(args.qemu.resolve(), args.firmware.resolve(),
                             Path(directory), persist_log)
        if args.ecu_disk is not None:
            with tempfile.TemporaryDirectory(prefix='jensen-ecu-') as directory:
                test_ecu(args.qemu.resolve(), args.firmware.resolve(),
                         args.ecu_disk, Path(directory), args.console_log)


if __name__ == '__main__':
    main()