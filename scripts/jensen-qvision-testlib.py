# SPDX-License-Identifier: GPL-2.0-or-later
"""QMP/qtest transport for the standalone Jensen QVision local tests.

Adapted from the local Jensen smoke harness. No vendor firmware is included.
The synthetic ROM is never executed: these device tests use the qtest accelerator.
"""

import json
import os
import queue
import socket
import struct
import subprocess
import threading


def pal_test_rom():
    image = bytearray(1024 * 1024)
    struct.pack_into('<I', image, 0x40, 0xc3ffffff)
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
