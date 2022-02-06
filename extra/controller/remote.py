"""
Remote communication interface
"""

import multiprocessing as mp
import time as tm
from collections import deque
from enum import Enum

import controller_rpc as rpc
import serial as sr
from utility.match import Match

REFRESH_DELAY_S = 1e-3
SERIAL_TIMEOUT_S = 1e-3
KEEPALIVE_DELAY_S = 500e-3


class Stream:
    """
    Handles a serial communication stream with a remote device
    """

    def __init__(self, port, tracker_pipe):
        """
        Start listening to a serial port and hold a pipe to a tracker process
        """
        assert all(
            map(lambda x: rpc.HEADER[0], rpc.HEADER)
        ), "Stream can only work with header consisting in a reapeated byte"
        self.pipe, remote_pipe = mp.Pipe()
        self._process = mp.Process(
            target=_StreamProcess(
                port=port, pipe=remote_pipe, tracker_pipe=tracker_pipe
            ),
            daemon=True,
        )
        self._process.start()


class Command(Enum):
    """
    Command for controlling remote handler
    """

    START_DUMP = 0
    STOP_DUMP = 1
    START_MEASURE_FORWARDING = 2
    STOP_MEASURE_FORWARDING = 3


class Order:
    """
    Command to call a remote order
    """

    def __init__(self, function, *args):
        """
        Store the order to be called and its parameters
        """
        self._fname = function.__name__
        self._args = args

    def __call__(self, output_stream):
        """
        Effectively call the order on remote device
        """
        # Arbitrary code execution
        # Evil as hell, but there is nothing to haxx to it's fine :)
        # I shall confess my sin tomorrow
        # By the way, it wouldn't have been necessary if pybind11 pickled C++ functions correctly
        function = eval("rpc." + self._fname)
        function(*self._args, output_stream)


class _StreamProcess:
    """
    Manages frame sending to the remote device and resolve request from the remote device
    """

    def __init__(self, port, pipe, tracker_pipe):
        """
        Start listening to a serial port and hold a pipe to a tracker process and a control pipe
        """
        self._pipe = pipe
        self._tracker_pipe = tracker_pipe
        self._serial = sr.Serial(
            port=port,
            baudrate=115200,
            bytesize=sr.EIGHTBITS,
            parity=sr.PARITY_NONE,
            stopbits=sr.STOPBITS_ONE,
            timeout=SERIAL_TIMEOUT_S,
        )
        self._read_pattern_counter = 0
        self._write_pattern_counter = 0
        self._header_sentinel = 0
        self._must_dump_serial = False
        self._must_forward_measure = False
        self._latest_keepalive_date_s = 0
        self._serial_history = deque(50 * [0])
        self._serial_history_index = 0

    def __call__(self):
        """
        Receive request from the remote device and handle command from the control pipe
        """
        while True:
            tm.sleep(REFRESH_DELAY_S)
            try:
                if self._find_header():
                    Match(rpc.FrameType(self._read_and_unstuff())) & {
                        rpc.FrameType.REQUEST: self._handle_request,
                        rpc.FrameType.RESPONSE: lambda: None,
                    }
            except Exception as e:
                self._serial_history.rotate(-self._serial_history_index - 1)
                self._serial_history_index = 0
                print(e)
                print(
                    "Serial history : {}".format(bytearray(self._serial_history).hex())
                )

            if self._pipe.poll():
                Match(self._pipe.recv()) & {
                    Command: Match()
                    & {
                        Command.START_DUMP: self._enable_dumping,
                        Command.STOP_DUMP: self._disable_dumping,
                        Command.START_MEASURE_FORWARDING: self._enable_measure_forwarding,
                        Command.STOP_MEASURE_FORWARDING: self._disable_measure_forwarding,
                    },
                    bytes: self._serial.write,
                    Order: self._start_frame,
                }

    def _find_header(self):
        """
        Drop bytes from the remote input until a frame header is found or there is no more bytes
        If a header is found, it is dropped from the input.
        """
        while self._serial.in_waiting > 0:
            byte = self._serial.read()
            self._header_sentinel = (
                self._header_sentinel + 1
                if int.from_bytes(byte, "little") == rpc.HEADER[0]
                else 0
            )
            if self._must_dump_serial:
                self._pipe.send(byte)
            if self._header_sentinel == len(rpc.HEADER):
                self._header_sentinel = 0
                return True

        return False

    def _handle_request(self):
        """
        Handle an incoming request from the remote device
        """
        Match(rpc.execute(self._read_and_unstuff, self._write_and_stuff)) & {
            rpc.Measure: self._forward_measure
        }

    def _start_frame(self, order):
        """
        Send a frame to call an order on the remote device
        """
        self._serial.write(rpc.HEADER)
        self._write_and_stuff(rpc.FrameType.REQUEST)
        order(self._write_and_stuff)
        self._write_and_stuff(0x0)

    def _read_and_unstuff(self):
        """
        Read the input from the remote device while byte unstuffing
        """
        byte = self._serial.read()

        self._serial_history[self._serial_history_index] = int.from_bytes(
            byte, "little"
        )
        self._serial_history_index = (self._serial_history_index + 1) % len(
            self._serial_history
        )

        byte_int = int.from_bytes(byte, "little")
        if self._must_dump_serial:
            self._pipe.send(byte)
        self._read_pattern_counter = (
            self._read_pattern_counter + 1 if byte_int == rpc.HEADER[0] else 0
        )
        if self._read_pattern_counter == len(rpc.HEADER) - 1:
            self._serial.read()
            self._read_pattern_counter = 0
        return byte_int

    def _write_and_stuff(self, byte_int):
        """
        Write to the remote device output while byte stuffing
        """
        self._serial.write(int(byte_int).to_bytes(1, "little"))
        self._write_pattern_counter = (
            self._write_pattern_counter + 1 if byte_int == rpc.HEADER[0] else 0
        )
        if self._write_pattern_counter == len(rpc.HEADER) - 1:
            self._serial.write(b"\x00")
            self._write_pattern_counter = 0

    def _forward_measure(self, measure):
        """
        Forward a received measure to the tracker if needed
        """
        if self._must_forward_measure:
            self._tracker_pipe.send(measure)

    def _enable_dumping(self):
        self._must_dump_serial = True

    def _disable_dumping(self):
        self._must_dump_serial = False

    def _enable_measure_forwarding(self):
        self._must_forward_measure = True

    def _disable_measure_forwarding(self):
        self._must_forward_measure = False
