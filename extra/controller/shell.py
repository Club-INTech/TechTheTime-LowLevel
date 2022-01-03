"""
Shell interface
"""

import argparse
import cmd
import itertools as it
import sys
import textwrap
from enum import Enum

import controller_rpc as rpc
import matplotlib.pyplot as plt
import numpy as np
import remote
from tracker import Tracker
from utility.match import Match


class MetaShell(type):
    def __new__(meta, name, bases, attrs):
        new_attributes = map(
            lambda fname: (
                "help_" + fname[3:],
                lambda self: getattr(self, fname)("-h"),
            ),
            filter(lambda name: name[:3] == "do_", attrs.keys()),
        )
        attrs.update({*new_attributes})
        return super(MetaShell, meta).__new__(meta, name, bases, attrs)


class Shell(cmd.Cmd, metaclass=MetaShell):
    """
    Execute commands received from a specified input stream and output to a specified output stream
    """

    def __init__(self):
        """
        Open a serial port from communication with a remote device and initialize the tracker context manager
        """
        super().__init__()
        self.prompt = "[shell] -- "
        self._mode = ShellMode.BASE
        self._tracker = Tracker()
        self._remote = remote.Stream(
            port="/dev/ttyUSB0", tracker_pipe=self._tracker.pipe
        )

    def do_dump(self, line):
        """
        Enable dump mode
        In dump mode, every byte received from serial is dumped to the specified output stream after each command.
        """
        Parser().parse_args(line)
        with DumpModeGuard(self):
            print(
                "Input from serial will be dump in the terminal between each user input"
            )
            print("Type 'quit' to disable serial dumping")
            run_shell(self)

    def do_sendraw(self, line):
        """
        Send a raw byte sequence to the remote device
        """
        parser = Parser()
        parser.add_argument("input", nargs="+", help="Byte sequence to send")
        args = parser.parse_args(line)
        self._remote.pipe.send(bytes(map(lambda x: int("0x" + x, 16), args.input)))

    def do_track(self, line):
        """
        Arm the tracker
        When a position measure will be received from the remote device, a pyplot display will appear ploting the position data.
        """
        Parser().parse_args(line)
        print("Arming the tracker...")
        with TrackerModeGuard(self), self._tracker:
            print("Tracker ready")
            print("Tracker will be disarmed when tracking is over or by typing 'quit'")
            run_shell(self)

        print("Tracker is disarmed")

    def do_translate(self, line):
        """
        Command the remote device to perform a translation
        """
        units_scales = {"tick": 1}

        parser = Parser()
        parser.add_argument("distance", type=int, help="Length of the translation")
        parser.add_argument(
            "--unit",
            "-u",
            nargs="?",
            choices=units_scales.keys(),
            default="tick",
            help="Unit in which 'distance' is given",
        )
        parser.add_argument(
            "--timeout",
            "-t",
            nargs="?",
            default=500e-3,
            help="Maximum delay between the reception of two measures from the remote device",
        )
        args = parser.parse_args(line)

        distance = (Match(args.unit) & units_scales) * args.distance

        while self._remote.pipe.poll():
            self._remote.pipe.recv()
        print("Commanding remote to start a translation...")
        self._remote.pipe.send(remote.Order(rpc.translate, distance))

        self._tracker.reset_timeout_counter()
        while self._tracker.timeout_counter_s < args.timeout:
            pass

        return True if self._mode is ShellMode.TRACKER else False

    def do_quit(self, line):
        """
        Quit the current mode
        If the shell is not in any mode, the shell will stop after this command.
        """
        Parser().parse_args(line)
        return True


class ShellMode(Enum):
    """
    Modes for the shell
    """

    BASE = 0
    TRACKER = 1
    DUMP = 2


class ShellModeGuard:
    """
    Context manager to handle shell mode
    When entering a context with a ShellModeGuard instance, the shell is configured for the specified mode.
    When exiting the context, the base configuration of the shell is restored.
    This is an abstract class, whose children must implement the _set and _restore methods.
    """

    def __init__(self, shell, mode):
        """
        Hold a reference to a shell and a mode
        """
        self._shell = shell
        self._mode = mode

    def __enter__(self):
        """
        Configure the shell for the specified mode
        The _set abstract method is call after configuration.
        """
        if self._shell._mode is not ShellMode.BASE:
            raise ShellException(
                "Could not enter in mode "
                + str(self._mode)
                + " : shell is already in mode "
                + str(self._shell._mode)
            )
        self._shell._mode = self._mode
        self._shell.prompt = Match(self._mode) & {
            ShellMode.TRACKER: "[shell > tracker] -- ",
            ShellMode.DUMP: "[shell > dump] -- ",
        }
        self._set()

    def __exit__(self, *_):
        """
        Restore the shell configuration
        The _restore abstract method is call before restoration.
        """
        self._restore()
        self._shell._mode = ShellMode.BASE
        self._shell.prompt = "[shell] -- "


class TrackerModeGuard(ShellModeGuard):
    """
    Configures the remote device interface for position tracking
    Within the current context, the remote device interface will be sending every measure received from the remote device through its tracker pipe.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(mode=ShellMode.TRACKER, *args, **kwargs)

    def _set(self):
        self._shell._remote.pipe.send(remote.Command.START_MEASURE_FORWARDING)

    def _restore(self):
        self._shell._remote.pipe.send(remote.Command.STOP_MEASURE_FORWARDING)


class DumpModeGuard(ShellModeGuard):
    """
    Configures the remote device interface for dumping received data and hook a post-command callback to the shell to dump the data to the specified output stream
    The byte sequences received from the remote device will be received through its control pipe.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(mode=ShellMode.DUMP, *args, **kwargs)

    def _set(self):
        self._shell._remote.pipe.send(remote.Command.START_DUMP)
        self._shell.precmd = self._empty_remote_pipe
        self._shell.postcmd = self._dump_remote

    def _restore(self):
        self._shell._remote.pipe.send(remote.Command.STOP_DUMP)
        self._shell.precmd = lambda line: line
        self._shell.postcmd = lambda stop, _: stop

    def _empty_remote_pipe(self, line):
        """
        Completely empty the remote to pipe
        """
        while self._shell._remote.pipe.poll():
            self._shell._remote.pipe.recv()

        return line

    def _dump_remote(self, stop, _):
        """
        Output the byte sequence dumped by the remote device interface to the specified output stream
        The shell will capture the data dumped by the interface until a given keepalive timeout is reached.
        Any data received during the capture period will be displayed.
        """

        data = bytearray()
        while self._shell._remote.pipe.poll(500e-3):
            data += self._shell._remote.pipe.recv()

        if data != b"":
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
    """
    Parse the command line arguments
    This class derivated from argparse.ArgumentParser is adapted for parsing command line arguments passed to the shell.
    """

    def __init__(self, *args, **kwargs):
        fname = sys._getframe(1).f_code.co_name[3:]
        function = Shell.__dict__["do_" + fname]
        super().__init__(
            prog=fname,
            description=textwrap.dedent(function.__doc__),
            formatter_class=argparse.RawDescriptionHelpFormatter,
            *args,
            **kwargs
        )

    def parse_args(self, line):
        try:
            return super().parse_args(line.split())
        except SystemExit as e:
            raise ShellException()


def run_shell(shell):
    """
    Run a shell session
    ShellException instances does not terminate the shell, but any other exception does.
    """

    is_running = True
    while is_running:
        try:
            shell.cmdloop()
            is_running = False
        except ShellException as e:
            if e.message is not None:
                print(e.message)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    run_shell(Shell(args.port))
