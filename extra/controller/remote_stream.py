"""
Remote communication interface
"""

import multiprocessing as mp
from enum import Enum

import controller_order as order
import serial as sr


class RemoteStream:
    def __init__(self, port, tracker_pipe):
        assert all(
            map(lambda x: order.HEADER[0], order.HEADER)
        ), "RemoteStream can only work with header consisting in a reapeated byte"
        self.pipe, remote_pipe = mp.Pipe()
        self._process = mp.Process(
            target=_RemoteStreamProcess(
                port=port, pipe=remote_pipe, tracker_pipe=tracker_pipe
            ),
            daemon=True,
        )
        self._process.start()


class Command(Enum):
    START_DUMP = 0
    STOP_DUMP = 1


class Order:
    def __init__(self, function, *args):
        self._fname = function.__name__
        self._args = args

    def __call__(self, output_stream):
        # Arbitrary code execution
        # Evil as hell, but there is nothing to haxx to it's fine :)
        # I shall confess my sin tomorrow
        # By the way, it wouldn't have been necessary if pybind11 pickled C++ functions correctly
        function = eval("order." + self._fname)
        function(*self._args, output_stream)


class _RemoteStreamProcess:
    def __init__(self, port, pipe, tracker_pipe):
        self._pipe = pipe
        self._tracker_pipe = tracker_pipe
        self._serial = sr.Serial(
            port=port,
            baudrate=115200,
            bytesize=sr.EIGHTBITS,
            parity=sr.PARITY_NONE,
            stopbits=sr.STOPBITS_ONE,
            timeout=0,
        )
        self._read_pattern_counter = 0
        self._write_pattern_counter = 0
        self._inbuf = []
        self._dump_mode = False

    def __call__(self):
        while True:
            serial_input = self._serial.read_until(expected=order.HEADER)

            if serial_input != b"":
                self._inbuf.append(serial_input)
                if self._dump_mode:
                    self._pipe.send(serial_input)

            if self._inbuf[-len(order.HEADER) :] == order.HEADER:
                self._serial.write(order.HEADER)
                obj = order.execute(self._read_and_unstuff, self._write_and_stuff)
                {order.Measure: self._tracker_pipe.send}.get(type(obj))(obj)
                self._inbuf = []
            if self._pipe.poll(0):
                obj = self._pipe.recv()
                {
                    Command: lambda obj: {
                        Command.START_DUMP: self._enable_dumping,
                        Command.STOP_DUMP: self._disable_dumping,
                    }.get(obj)(),
                    bytes: self._serial.write,
                    Order: self._start_frame,
                }.get(type(obj))(obj)

    def _start_frame(self, obj):
        for byte in order.HEADER:
            self._write_and_stuff(byte)
        obj(self._write_and_stuff)

    def _read_and_unstuff(self):
        byte = self._serial.read()
        self._read_pattern_counter = (
            self._read_pattern_counter + 1 if byte == order.HEADER[0] else 0
        )
        if self._read_pattern_counter == len(order.HEADER) - 1:
            self._serial.read()
            self._read_pattern_counter = 0
        return byte

    def _write_and_stuff(self, byte):
        print(byte)
        byte = self._serial.write(byte)
        self._write_pattern_counter = (
            self._write_pattern_counter + 1 if byte == order.HEADER[0] else 0
        )
        if self._write_pattern_counter == len(order.HEADER) - 1:
            self._serial.write(0x00)
            self._write_pattern_counter = 0
        return byte

    def _enable_dumping(self):
        self._dump_mode = True

    def _disable_dumping(self):
        self._dump_mode = False
