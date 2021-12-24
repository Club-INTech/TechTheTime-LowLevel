"""
Shell interface
"""

import argparse
import cmd
import itertools as it
import random as random
import sys
import time as tm
from enum import Enum

import controller_order as order
import matplotlib.pyplot as plt
import numpy as np
from encoder_tracker import EncoderTracker
from remote_stream import Command as RemoteCommand
from remote_stream import Order, RemoteStream

j = 0
t = 0
noise1 = 0
noise2 = 0


class Stream:
    def read(self):
        global j, t, noise1, noise2

        if j == 0:
            t = int(tm.time() * 1e3)
            noise1 += int(10 * random.random() - 5)
            noise2 += int(10 * random.random() - 5)
            noise1 = min(max(noise1, 0), 30)
            noise2 = min(max(noise2, 0), 30)

        L = [
            0,
            0,
            t & 0xFF,
            (t >> 8) & 0xFF,
            0,
            0,
            int(100 * (np.sin(t * 1e-3) + 1)) + noise1,
            0,
            0,
            0,
            int(10 * (np.cos(t * 1e-3) + 1)) + noise2,
            0,
            0,
            0,
        ]
        x = L[j]
        j += 1
        j %= len(L)
        return x

    def write(self, x):
        pass

    def skip_to_frame(self):
        return True


class Shell(cmd.Cmd):
    prompt = "[shell] -- "

    def __init__(self):
        super().__init__()
        self._mode = ShellMode.BASE
        self._tracker = EncoderTracker()
        self._remote = RemoteStream(
            port="/dev/ttyUSB0", tracker_pipe=self._tracker.pipe
        )

    def do_dump(self, _):
        with DumpModeGuard(self):
            print(
                "Input from serial will be dump in the terminal between each user input"
            )
            print("Type 'quit' to disable serial dumping")
            run_shell(self)

    def do_sendraw(self, line):
        parser = Parser(description="Send a raw input to remote")
        parser.add_argument("input", nargs="+")
        args = parser.parse_args(line)
        self._remote.pipe.send(bytes(map(lambda x: int("0x" + x, 16), args.input)))

    def do_track(self, _):
        print("Arming the tracker...")
        with TrackerModeGuard(self), self._tracker:
            print("Tracker ready")
            print("Tracker will be disarmed when tracking is over or by typing 'quit'")
            run_shell(self)

        print("Tracker is disarmed")

    def do_translate(self, _):
        print("Commanding remote to start a translation...")
        self._remote.pipe.send(Order(order.translate, 0xFFFFFFFF))
        # self._remote.pipe.recv()
        return True if self._mode is ShellMode.TRACKER else False

    def do_quit(self, _):
        return True


class ShellMode(Enum):
    BASE = 0
    TRACKER = 1
    DUMP = 2


class ShellModeGuard:
    def __init__(self, shell, mode):
        self._shell = shell
        self._mode = mode

    def __enter__(self):
        if self._shell._mode is not ShellMode.BASE:
            raise ShellException(
                "Could not enter in mode "
                + str(self._mode)
                + " : shell is already in mode "
                + str(self._shell._mode)
            )
        self._shell._mode = self._mode
        Shell.prompt = {
            ShellMode.TRACKER: "[shell > tracker] -- ",
            ShellMode.DUMP: "[shell > dump] -- ",
        }.get(self._mode)
        self._set()

    def __exit__(self, *_):
        self._restore()
        self._shell._mode = ShellMode.BASE
        Shell.prompt = "[shell] -- "


class TrackerModeGuard(ShellModeGuard):
    def __init__(self, *args, **kwargs):
        super().__init__(mode=ShellMode.TRACKER, *args, **kwargs)

    def _set(self):
        pass

    def _restore(self):
        pass


class DumpModeGuard(ShellModeGuard):
    def __init__(self, *args, **kwargs):
        super().__init__(mode=ShellMode.DUMP, *args, **kwargs)

    def _set(self):
        self._shell._remote.pipe.send(RemoteCommand.START_DUMP)
        self._shell.postcmd = self._dump_serial

    def _restore(self):
        self._shell._remote.pipe.send(RemoteCommand.STOP_DUMP)
        self._shell.postcmd = lambda *_: None

    def _dump_serial(self, stop, _):
        tm.sleep(500e-3)
        if self._shell._remote.pipe.poll(0):
            data = bytearray()
            while self._shell._remote.pipe.poll(0):
                data += self._shell._remote.pipe.recv()

            hline = "-" * (6 + 3 * 16)
            print(" " * 3 + "| " + bytes(range(16)).hex(" ") + " |")
            print(hline)
            for i, row in enumerate(it.zip_longest(*([iter(data)] * 16), fillvalue=0)):
                print("{:02x} | ".format(i << 4) + bytes(row).hex(" ") + " |")
            print(hline)

        return stop


class ShellException(BaseException):
    def __init__(self, message=None):
        self.message = message


class Parser(argparse.ArgumentParser):
    def __init__(self, *args, **kwargs):
        fname = sys._getframe(1).f_code.co_name[3:]
        super().__init__(prog=fname, *args, **kwargs)

    def parse_args(self, line):
        try:
            return super().parse_args(line.split())
        except SystemExit as e:
            raise ShellException()


def run_shell(shell):
    is_running = True
    while is_running:
        try:
            shell.cmdloop()
            is_running = False
        except ShellException as e:
            if e.message is not None:
                print(e.message)


if __name__ == "__main__":
    run_shell(Shell())
