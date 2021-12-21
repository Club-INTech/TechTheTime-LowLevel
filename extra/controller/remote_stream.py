"""
Remote communication interface
"""

import serial as sr
import controller_order as order
import multiprocessing as mp


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

    def __call__(self):
        while True:
            self._inbuf.append(self._serial.read_until(expected=order.HEADER))
            if self._inbuf[-len(order.HEADER) :] == order.HEADER:
                self._serial.write(order.HEADER)
                obj = order.execute(self._read_and_unstuff, self._write_and_stuff)
                {order.Measure: self._tracker_pipe.send}.get(type(obj))(obj)
                self._inbuf = []
            if self._pipe.poll(0):
                self._serial.write(self._pipe.recv())

    def _read_and_unstuff(self):
        byte = self._serial.read()
        self._read_pattern_counter = (
            self._read_pattern_counter + 1 if byte == order.HEADER[0] else 0
        )
        if self._read_pattern_counter == len(order.HEADER[0]) - 1:
            self._serial.read()
            self._read_pattern_counter = 0
        return byte

    def _write_and_stuff(self, byte):
        byte = self._serial.write(0x00)
        self._write_pattern_counter = (
            self._write_pattern_counter + 1 if byte == order.HEADER[0] else 0
        )
        if self._write_pattern_counter == len(order.HEADER[0]) - 1:
            self._serial.write(0x00)
            self._write_pattern_counter = 0
        return byte
