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
from itertools import product
from os import listdir, path
from shutil import copyfile
from sys import stdin, stdout
from termios import TCIFLUSH, tcflush

import colorama
import controller_rpc as rpc
import matplotlib.pyplot as plt
import numpy as np
import remote
import tracker as trk
from colorama import Fore, Style
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

    def __init__(self, port, out_stream=stdout, in_stream=stdin):
        """
        Open a serial port from communication with a remote device and initialize the tracker context manager
        """
        remote_tracker_pipe = mp.Pipe()

        super().__init__()
        self.prompt = "[shell] -- "
        self._mode = ShellMode.BASE
        self._tracker = Tracker(remote_pipe=remote_tracker_pipe[0])
        self._remote = remote.Stream(port=port, tracker_pipe=remote_tracker_pipe[1])
        self._out = out_stream
        self._in = in_stream

        if path.exists(self._get_pid_path()):
            with self._log_attempt(
                "Loading the stored PID parameters into remote"
            ), open(self._get_pid_path()) as f:
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
                        *map(float, pid["translation"].values()),
                    )
                )
                self._remote.pipe.send(
                    remote.Order(
                        rpc.set_rotation_pid, *map(float, pid["rotation"].values())
                    )
                )
        else:
            with open(self._get_pid_path(), "w") as f:
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
        Parser(self).parse_args(line)
        with DumpModeGuard(self):
            self._log_status(
                "Input from serial will be dump in the terminal between each user input"
            )
            self._log_status("Type 'quit' to disable serial dumping")
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
        with self._log_attempt(f"Sending '{args.input}' to remote"):
            self._remote.pipe.send(bytes(map(lambda x: int("0x" + x, 16), args.input)))

    def do_track(self, line):
        """
        Arm the tracker
        When a position measure will be received from the remote device, a pyplot display will appear ploting the position data.
        """
        parser = Parser(self)
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

        self._tracker.shows_record = args.show_record

        self._log_status("Arming the tracker...")
        with self._tracker, TrackerModeGuard(self):
            self._log_status("Tracker ready")
            self._log_status(
                "Tracker will be disarmed when tracking is over or by typing 'quit'"
            )
            self.onecmd(" ".join(args.exec)) if args.exec else run_shell(self)
        self._log_status("Tracker is disarmed")

    def do_translate(self, line):
        """
        Command the remote device to perform a translation
        """
        units_scales = {"tick": 1}

        parser = Parser(self)
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
        with self._log_attempt("Commanding remote to start a translation"):
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

        parser = Parser(self)
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
        with self._log_attempt("Commanding remote to start a rotation"):
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

        parser = Parser(self)
        parser.add_argument("pwm", type=int, help="Change pwm parameter")
        args = parser.parse_args(line)

        pwm = args.pwm

        self._log_status("Commanding remote to start a free movement...")
        self._log_status("Press SPACE to stop")
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
        parser = Parser(self)
        parser.add_argument(
            "distance_step", type=int, help="Minimal distance traveled in one step"
        )
        parser.add_argument("angle_step", type=int, help="Minimal angle in one step")
        args = parser.parse_args(line)

        self._log_status("Commanding remote in joystick mode...")
        self._log_status("Press SPACE to stop")
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
                    self._out.write("\r")
                    return

                # Hide the keyboard input in the terminal
                self._out.write("\r" + 4 * " " + "\r")

                self._remote.pipe.send(remote.Order(rpc.set_joystick, distance, angle))
                tm.sleep(100e-3)

    def do_pid(self, line):
        """
        Forward arguments to a 'pid' subcommand
        """

        parser = Parser(self, add_help=False, prefix_chars="??")
        parser.add_argument("mode", choices=["show", "list", "set", "load", "save"])
        parser.add_argument("args", nargs="*")
        args = parser.parse_args(line)
        return (
            Match(args.mode)
            & {
                "show": lambda: self.do_pid_show,
                "list": lambda: self.do_pid_list,
                "set": lambda: self.do_pid_set,
                "load": lambda: self.do_pid_load,
                "save": lambda: self.do_pid_save,
            }
        )(" ".join(args.args))

    def do_pid_show(self, line):
        """
        Display the current PID profile
        """
        targets = [
            ("left", "l"),
            ("right", "r"),
            ("translation", "tr"),
            ("rotation", "rt"),
        ]
        coefficients = ["kp", "ki", "kd"]

        parser = Parser(self)
        parser.add_argument("profile", help="Target profile", nargs="?", default="")
        args = parser.parse_args(line)

        pid = {}
        try:
            with self._log_attempt(f"Loading {args.profile}"), open(
                self._get_pid_path(args.profile)
            ) as f:
                pid = literal_eval(f.read())
        except FileNotFoundError:
            raise ShellException(f"No such profile {args.profile}")

        profile_name = args.profile if args.profile != "" else "(current profile)"
        self._out.write(
            Fore.BLUE + Style.BRIGHT + f"{profile_name}\n" + Style.RESET_ALL
        )
        for (target, _) in targets:
            self._out.write(
                Fore.YELLOW + Style.BRIGHT + f"- {target}\n" + Style.RESET_ALL
            )
            for coefficient in coefficients:
                self._out.write(
                    Fore.YELLOW
                    + Style.BRIGHT
                    + f"-- {coefficient}{Style.RESET_ALL}: {pid[target][coefficient]:e}\n"
                )

    def do_pid_list(self, line):
        """
        List all the available PID profiles
        """

        parser = Parser(self)
        parser.add_argument(
            "--full", "-f", action="store_true", help="Show the PID parameters as well"
        )
        args = parser.parse_args(line)

        profile_names = list(
            map(lambda x: x[:-4], listdir(path.dirname(self._get_pid_path())))
        )
        if args.full:
            for name in profile_names:
                self.do_pid_show(name)
        else:
            for name in profile_names[1:]:
                self._out.write(
                    Fore.BLUE + Style.BRIGHT + name + "\n" + Style.RESET_ALL
                )

    def do_pid_set(self, line):
        """
        Change the PID parameters of the remote device and store them locally
        """
        targets = [
            ("left", "l"),
            ("right", "r"),
            ("translation", "tr"),
            ("rotation", "rt"),
        ]
        coefficients = ["kp", "ki", "kd"]

        parser = Parser(self)
        for ((target, abreviation), coefficient) in product(targets, coefficients):
            parser.add_argument(
                f"--{target}-{coefficient}",
                f"-{abreviation}{coefficient}",
                type=float,
                default=None,
                help=f"Change {target} {coefficient} parameters",
            )
        args = parser.parse_args(line)

        pid = {}
        with open(self._get_pid_path()) as f:
            pid = literal_eval(f.read())

        for ((target, _), coefficient) in product(targets, coefficients):
            if vars(args)[f"{target}_{coefficient}"] is not None:
                pid[target][coefficient] = vars(args)[f"{target}_{coefficient}"]

        for (target, _) in targets:
            if any(map(lambda x: vars(args)[f"{target}_{x}"], coefficients)):
                with self._log_attempt(f"Changing {target} PID parameters on remote"):
                    self._remote.pipe.send(
                        remote.Order(
                            vars(rpc)[f"set_{target}_pid"], *pid[target].values()
                        )
                    )

        with open(self._get_pid_path(), "w") as f:
            f.write(repr(pid))

    def do_pid_load(self, line):
        """
        Load a PID profile and make it the current one
        """
        parser = Parser(self)
        parser.add_argument("profile", help="Target profile")
        parser.add_argument("--save", "-s", help="Save the current PID profile")
        parser.add_argument(
            "--discard", action="store_true", help="Discard the current PID profile"
        )
        args = parser.parse_args(line)

        if args.save is not None:
            with self._log_attempt(f"Saving profile {args.save}"):
                copyfile(self._get_pid_path(), self._get_pid_path(args.save))
        elif not args.discard:
            raise ShellException(
                "Attempted to load a new profile without saving the current one (to discard it, use the --discard flag)"
            )

        try:
            with self._log_attempt(f"Loading profile {args.profile}"):
                copyfile(self._get_pid_path(args.profile), self._get_pid_path())
        except FileNotFoundError:
            raise ShellException(f"No such profile '{args.profile}'")

        with self._log_attempt(f"Changing PID parameters on remote"):
            pid = {}
            with open(self._get_pid_path()) as f:
                pid = literal_eval(f.read())

            for target in ["left", "right", "translation", "rotation"]:
                self._remote.pipe.send(
                    remote.Order(vars(rpc)[f"set_{target}_pid"], *pid[target].values())
                )

    def do_pid_save(self, line):
        """
        Save the current PID profile
        """
        parser = Parser(self)
        parser.add_argument("profile", help="Profile to save to")
        args = parser.parse_args(line)

        with self._log_attempt(f"Saving profile {args.profile}"):
            copyfile(self._get_pid_path(), self._get_pid_path(args.profile))

    def do_pump(self, line):
        """
        Enable / disable a pump
        """
        parser = Parser(self)
        parser.add_argument("device", type=int, help="ID of the target pump")
        parser.add_argument(
            "state",
            choices=["on", "off"],
            help="Whether the pump will be enabled or disabled",
        )
        args = parser.parse_args(line)
        self._remote.pipe.send(
            remote.Order(rpc.set_pump, args.device, args.state == "on")
        )

    def do_valve(self, line):
        """
        Enable / disable a valve
        """
        parser = Parser(self)
        parser.add_argument("device", type=int, help="ID of the target valve")
        parser.add_argument(
            "state",
            choices=["on", "off"],
            help="Whether the valve will be enabled or disabled",
        )
        args = parser.parse_args(line)
        self._remote.pipe.send(
            remote.Order(rpc.set_valve, args.device, args.state == "on")
        )

    def do_servo(self, line):
        """
        Move a servomotor
        """
        parser = Parser(self)
        parser.add_argument("device", type=int, help="ID of the target valve")
        parser.add_argument(
            "value", type=int, value="Position for the servo (from 0 to 65535)"
        )

        args = parser.parse_args(line)
        self._remote.pipe.send(remote.Order(rpc.set_servo, args.device, args.value))

    def do_quit(self, line):
        """
        Quit the current mode
        If the shell is not in any mode, the shell will stop after this command.
        """
        Parser(self).parse_args(line)
        return True

    def _log_status(self, message):
        self._out.write(message + "\n")

    def _log_attempt(self, message):
        return AttemptLogger(self, message)

    def _get_pid_path(self, profile=""):
        return path.dirname(__file__) + f"/data/{profile}.pid"

    def do_dxl_pos(self, line):
        """
        Change position of the dynamixel with a input in tick
        """
        parser = Parser(self)
        parser.add_argument("dxl_id", type=int, help="Dynamixel id")
        parser.add_argument(
            "position_tick", type=int, help="Position in tick for the dynamixel"
        )
        args = parser.parse_args(line)

        id = args.dxl_id
        position = args.position_tick

        with self._log_attempt("Moving the position of the dynamixel"):
            self._remote.pipe.send(remote.Order(rpc.dxl_position, id, position))

    def do_dxl_pos_a(self, line):
        """
        Change position of the dynamixel with a input in degree
        """
        parser = Parser(self)
        parser.add_argument("dxl_id", type=int, help="Dynamixel id")
        parser.add_argument(
            "position_angle", type=int, help="Position in degree for the dynamixel"
        )
        args = parser.parse_args(line)

        id = args.dxl_id
        position = args.position_angle

        with self._log_attempt("Moving the position of the dynamixel"):
            self._remote.pipe.send(remote.Order(rpc.dxl_position_angle, id, position))


class AttemptLogger:
    def __init__(self, shell, message):
        self._shell = shell
        self._message = message

    def __enter__(self):
        self._shell._out.write(self._message + "... ")

    def __exit__(self, exception_type, *_):
        if exception_type is None:
            self._shell._out.write(
                Fore.GREEN + Style.BRIGHT + "Success" + Style.RESET_ALL + "\n"
            )
        else:
            self._shell._out.write(
                Fore.RED + Style.BRIGHT + "Failure" + Style.RESET_ALL + "\n"
            )


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
            ShellMode.TRACKER: Fore.CYAN
            + Style.BRIGHT
            + f"[shell > tracker] {Style.RESET_ALL}-- ",
            ShellMode.DUMP: Fore.YELLOW
            + Style.BRIGHT
            + f"[shell > dump] {Style.RESET_ALL}-- ",
            ShellMode.JOYSTICK: Fore.MAGENTA
            + Style.BRIGHT
            + f"[shell > joystick] {Style.RESET_ALL}-- ",
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
            self._out.write(" " * 3 + "| " + bytes(range(16)).hex(" ") + " |\n")
            self._out.write(hline + "\n")
            for i, row in enumerate(it.zip_longest(*([iter(data)] * 16), fillvalue=0)):
                self._out.write(
                    "{:02x} | ".format(i << 4) + bytes(row).hex(" ") + " |\n"
                )
            self._out.write(hline) + "\n"

        return stop


class ShellException(BaseException):
    def __init__(self, message=None):
        self.message = message


class Parser(argparse.ArgumentParser):
    """
    Parse the command line arguments
    This class derivated from argparse.ArgumentParser is adapted for parsing command line arguments passed to the shell.
    """

    def __init__(self, shell, *args, **kwargs):
        self._shell = shell
        fname = sys._getframe(1).f_code.co_name[3:]
        function = Shell.__dict__["do_" + fname]
        super().__init__(
            prog=fname,
            description=Fore.GREEN + Style.NORMAL + textwrap.dedent(function.__doc__),
            epilog=Style.RESET_ALL,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            *args,
            **kwargs,
        )

    def parse_args(self, line):
        self._shell._out.write(Fore.RED + Style.BRIGHT)
        try:
            return super().parse_args(line.split())
        except SystemExit as e:
            raise ShellException()
        finally:
            self._shell._out.write(Style.RESET_ALL)


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
                shell._out.write(
                    Fore.RED + Style.BRIGHT + e.message + Style.RESET_ALL + "\n"
                )


if __name__ == "__main__":
    colorama.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    args = parser.parse_args()
    try:
        run_shell(Shell(port=args.port))

    finally:
        colorama.deinit()
        self._shell._remote.pipe.send(remote.Order(rpc.release_motor))
