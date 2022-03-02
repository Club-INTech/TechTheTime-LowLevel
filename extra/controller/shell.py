"""
Shell interface
"""

import argparse
import cmd
import itertools as it
import multiprocessing as mp
import sys
import textwrap
import time as tm
from ast import literal_eval
from enum import Enum
from os import path
from pprint import pprint
from sys import stdin, stdout
from termios import TCIFLUSH, tcflush

import controller_rpc as rpc
import matplotlib.pyplot as plt
import numpy as np
import remote
import tracker as trk
from keyboard import block_key, is_pressed, send, unhook_all
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

    def __init__(self, port):
        """
        Open a serial port from communication with a remote device and initialize the tracker context manager
        """
        remote_tracker_pipe = mp.Pipe()

        super().__init__()
        self.prompt = "[shell] -- "
        self._mode = ShellMode.BASE
        self._tracker = Tracker(remote_pipe=remote_tracker_pipe[0])
        self._remote = remote.Stream(port=port, tracker_pipe=remote_tracker_pipe[1])
        self._pid_path = path.dirname(__file__) + "/data/pid"

        if path.exists(self._pid_path):
            print("Loading the stored PID parameters into remote...")
            with open(self._pid_path) as f:
                pid = literal_eval(f.read())
                self._remote.pipe.send(
                    remote.Order(rpc.set_left_pid, *map(float, pid["left"].values()))
                )
                self._remote.pipe.send(
                    remote.Order(rpc.set_right_pid, *map(float, pid["right"].values()))
                )
                self._remote.pipe.send(
                    remote.Order(
                        rpc.set_translation_pid,
                        *map(float, pid["translation"].values())
                    )
                )
                self._remote.pipe.send(
                    remote.Order(
                        rpc.set_rotation_pid, *map(float, pid["rotation"].values())
                    )
                )
        else:
            with open(self._pid_path, "w") as f:
                f.write(
                    repr(
                        {
                            "left": {"kp": 0, "ki": 0, "kd": 0},
                            "right": {"kp": 0, "ki": 0, "kd": 0},
                            "translation": {"kp": 0, "ki": 0, "kd": 0},
                            "rotation": {"kp": 0, "ki": 0, "kd": 0},
                        }
                    )
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
        parser = Parser(
            epilog="Example : 'sendraw aa bb cc' -> send the sequence '0xaa 0xbb 0xcc' to the remote device"
        )
        parser.add_argument("input", nargs="+", help="Byte sequence to send")
        args = parser.parse_args(line)

        if not all(map(lambda x: 0 <= int(x, base=16) < 256, args.input)):
            raise ShellException(
                "The byte sequence must be given in the following format: 'sendraw xx xx xx xx...' Where x are hexadecimal digits"
            )
        self._remote.pipe.send(bytes(map(lambda x: int("0x" + x, 16), args.input)))

    def do_track(self, line):
        """
        Arm the tracker
        When a position measure will be received from the remote device, a pyplot display will appear ploting the position data.
        """
        parser = Parser()
        parser.add_argument(
            "--show-record",
            "-sr",
            action="store_true",
            help="Enables data display after tracking",
        )
        parser.add_argument(
            "--exec",
            "-e",
            nargs="*",
            help="Execute that command after arming the tracker",
        )
        args = parser.parse_args(line)

        print("Arming the tracker...")
        self._tracker.shows_record = args.show_record
        with self._tracker, TrackerModeGuard(self):
            print("Tracker ready")
            print("Tracker will be disarmed when tracking is over or by typing 'quit'")
            self.onecmd(" ".join(args.exec)) if args.exec else run_shell(self)

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
            type=float,
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

        self._tracker.pipe.send(trk.Setpoint(distance))
        return True if self._mode is ShellMode.TRACKER else False

    def do_rotate(self, line):
        """
        Command the remote device to perform a rotation
        """
        units_scales = {"tick": 1}

        parser = Parser()
        parser.add_argument("angle", type=int, help="Angle of the rotation")
        parser.add_argument(
            "--unit",
            "-u",
            nargs="?",
            choices=units_scales.keys(),
            default="tick",
            help="Unit in which 'angle' is given",
        )
        parser.add_argument(
            "--timeout",
            "-t",
            type=float,
            nargs="?",
            default=500e-3,
            help="Maximum delay between the reception of two measures from the remote device",
        )
        args = parser.parse_args(line)

        angle = (Match(args.unit) & units_scales) * args.angle

        while self._remote.pipe.poll():
            self._remote.pipe.recv()
        print("Commanding remote to start a rotation...")
        self._remote.pipe.send(remote.Order(rpc.rotate, angle))

        self._tracker.reset_timeout_counter()
        while self._tracker.timeout_counter_s < args.timeout:
            pass

        self._tracker.pipe.send(trk.Setpoint(angle))
        return True if self._mode is ShellMode.TRACKER else False

    def do_free(self, line):
        """
        Command the remote device for an infinite translation until space is pressed
        """

        parser = Parser()
        parser.add_argument("pwm", type=int, help="Change pwm parameter")
        args = parser.parse_args(line)

        pwm = args.pwm

        print("Commanding remote to start a free movement...")
        print("Press SPACE to stop")
        self._remote.pipe.send(remote.Order(rpc.set_free_movement, pwm))

        while (
            self._mode != ShellMode.TRACKER
            or self._tracker.get_status() == trk.Status.READY
        ) and not is_pressed(" "):
            tm.sleep(50e-3)

        self._remote.pipe.send(remote.Order(rpc.release_motor))

        return True if self._mode is ShellMode.TRACKER else False

    def do_joystick(self, line):
        """
        Command the remote device according to the joystick
        """
        parser = Parser()
        parser.add_argument(
            "distance_step", type=int, help="Minimal distance traveled in one step"
        )
        parser.add_argument("angle_step", type=int, help="Minimal angle in one step")
        args = parser.parse_args(line)

        print("Commanding remote to start a free movement...")

        with JoystickModeGuard(self):
            while True:
                if is_pressed("z"):
                    distance = args.distance_step
                elif is_pressed("s"):
                    distance = -args.distance_step
                else:
                    distance = 0

                if is_pressed("d"):
                    angle = args.angle_step
                elif is_pressed("q"):
                    angle = -args.angle_step
                else:
                    angle = 0

                if is_pressed(" "):
                    # Clear stdin because it is filled wih the irrevelant keyboard input and hide the space input in the terminal
                    tcflush(stdin, TCIFLUSH)
                    stdout.write("\r")
                    return

                # Hide the keyboard input in the terminal
                stdout.write("\r" + 4 * " " + "\r")

                self._remote.pipe.send(remote.Order(rpc.set_joystick, distance, angle))
                tm.sleep(100e-3)

    def do_pid(self, line):
        """
        Change the PID parameters of the remote device and store them locally
        """
        parser = Parser()
        parser.add_argument(
            "--left-kp",
            "-lkp",
            type=float,
            default=None,
            help="Change left kp parameters",
        )
        parser.add_argument(
            "--left-ki",
            "-lki",
            type=float,
            default=None,
            help="Change left ki parameters",
        )
        parser.add_argument(
            "--left-kd",
            "-lkd",
            type=float,
            default=None,
            help="Change left kd parameters",
        )
        parser.add_argument(
            "--right-kp",
            "-rkp",
            type=float,
            default=None,
            help="Change right kp parameters",
        )
        parser.add_argument(
            "--right-ki",
            "-rki",
            type=float,
            default=None,
            help="Change right ki parameters",
        )
        parser.add_argument(
            "--right-kd",
            "-rkd",
            type=float,
            default=None,
            help="Change right kd parameters",
        )
        parser.add_argument(
            "--translation-kp",
            "-trkp",
            type=float,
            default=None,
            help="Change translation kp parameters",
        )
        parser.add_argument(
            "--translation-ki",
            "-trki",
            type=float,
            default=None,
            help="Change translation ki parameters",
        )
        parser.add_argument(
            "--translation-kd",
            "-trkd",
            type=float,
            default=None,
            help="Change translation kd parameters",
        )
        parser.add_argument(
            "--rotation-kp",
            "-rtkp",
            type=float,
            default=None,
            help="Change rotation kp parameters",
        )
        parser.add_argument(
            "--rotation-ki",
            "-rtki",
            type=float,
            default=None,
            help="Change rotation ki parameters",
        )
        parser.add_argument(
            "--rotation-kd",
            "-rtkd",
            type=float,
            default=None,
            help="Change rotation kd parameters",
        )
        args = parser.parse_args(line)

        pid = {}
        with open(self._pid_path) as f:
            pid = literal_eval(f.read())

        if not any(map(lambda x: x is not None, vars(args).values())):
            pprint(pid)
            return

        if args.left_kp is not None:
            pid["left"]["kp"] = args.left_kp
        if args.left_ki is not None:
            pid["left"]["ki"] = args.left_ki
        if args.left_kd is not None:
            pid["left"]["kd"] = args.left_kd
        if args.right_kp is not None:
            pid["right"]["kp"] = args.right_kp
        if args.right_ki is not None:
            pid["right"]["ki"] = args.right_ki
        if args.right_kd is not None:
            pid["right"]["kd"] = args.right_kd
        if args.translation_kp is not None:
            pid["translation"]["kp"] = args.translation_kp
        if args.translation_ki is not None:
            pid["translation"]["ki"] = args.translation_ki
        if args.translation_kd is not None:
            pid["translation"]["kd"] = args.translation_kd
        if args.rotation_kp is not None:
            pid["rotation"]["kp"] = args.rotation_kp
        if args.rotation_ki is not None:
            pid["rotation"]["ki"] = args.rotation_ki
        if args.rotation_kd is not None:
            pid["rotation"]["kd"] = args.rotation_kd

        if (
            args.left_kp is not None
            or args.left_ki is not None
            or args.left_kd is not None
        ):
            print("Changing left PID parameters...")
            self._remote.pipe.send(
                remote.Order(rpc.set_left_pid, *pid["left"].values())
            )
        if (
            args.right_kp is not None
            or args.right_ki is not None
            or args.right_kd is not None is not None
        ):
            print("Changing right PID parameters...")
            self._remote.pipe.send(
                remote.Order(rpc.set_right_pid, *pid["right"].values())
            )
        if (
            args.translation_kp is not None
            or args.translation_ki is not None
            or args.translation_kd is not None
        ):
            print("Changing translation PID parameters...")
            self._remote.pipe.send(
                remote.Order(rpc.set_translation_pid, *pid["translation"].values())
            )
        if (
            args.rotation_kp is not None
            or args.rotation_ki is not None
            or args.rotation_kd is not None
        ):
            print("Changing rotation PID parameters...")
            self._remote.pipe.send(
                remote.Order(rpc.set_rotation_pid, *pid["rotation"].values())
            )

        with open(self._pid_path, "w") as f:
            f.write(repr(pid))

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
    JOYSTICK = 3


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
            ShellMode.JOYSTICK: "[shell > joystick] -- ",
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
        self._shell._remote.pipe.send(remote.Command.CLEAR_INPUT_BUFFER)
        self._shell._remote.pipe.send(remote.Command.START_MEASURE_FORWARDING)
        self._shell._remote.pipe.send(remote.Order(rpc.set_mode, rpc.HubMode.TRACKER))

    def _restore(self):
        self._shell._remote.pipe.send(remote.Command.STOP_MEASURE_FORWARDING)
        self._shell._remote.pipe.send(remote.Order(rpc.release_motor))
        self._shell._remote.pipe.send(remote.Order(rpc.set_mode, rpc.HubMode.BASE))


class JoystickModeGuard(ShellModeGuard):
    """
    Configures the remote device interface for free movement with the joystick
    """

    def __init__(self, *args, **kwargs):
        super().__init__(mode=ShellMode.JOYSTICK, *args, **kwargs)

    def _set(self):
        self._shell._remote.pipe.send(remote.Order(rpc.start_joystick))

    def _restore(self):
        self._shell._remote.pipe.send(remote.Order(rpc.release_motor))


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
    args = parser.parse_args()
    run_shell(Shell(port=args.port))
