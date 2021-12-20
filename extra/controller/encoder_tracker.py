"""
Encoder state tracking
"""

import matplotlib.pyplot as plt
import matplotlib.transforms as tsf
import numpy as np
import controller_order as order
import time as tm
import multiprocessing as mp
import operator as op

from enum import Enum

TRACKER_SETUP_TIMEOUT_S = 3


class EncoderTracker:
    """
    Handles a pyplot display that tracks encoder measures
    """

    def __init__(self):
        """
        Creates pipes to feed the tracker data
        """
        self.pipe, self._remote_pipe = mp.Pipe()
        self._process = mp.Process(
            target=_EncoderTrackerProcess(pipe=self._remote_pipe), daemon=True
        )

    def __enter__(self):
        """
        Start the tracking
        """
        self._process.start()
        status = (
            self.pipe.recv()
            if self.pipe.poll(TRACKER_SETUP_TIMEOUT_S)
            else Status.TIMEOUT
        )
        if status != Status.READY:
            raise RuntimeError(
                "Tracker failed initialization with status code " + str(status)
            )

    def __exit__(self, *_):
        """
        End the tracking
        """
        self.pipe.send(Command.STOP)
        self._process.join()
        obj = self.pipe.recv()
        {tuple: lambda x: self._set_measures(*x)}.get(type(obj))(obj)

    def _set_measures(self, time_us, left_ticks, right_ticks):
        self.time_us = time_us
        self.left_ticks = left_ticks
        self.right_ticks = right_ticks


class Command(Enum):
    """
    Control commands for encoder tracking
    """

    STOP = 0


class Status(Enum):
    """
    Status code for encoder tracking
    """

    READY = 0
    TIMEOUT = 1


class _EncoderTrackerProcess:
    """
    Plots encoder measures received from a given stream on a pyplot display
    The display will consists the plots of the encoder measurers and a real-time cursor indicating the current time.
    """

    def __init__(self, pipe):
        """
        Hold a stream from which measures will be received and a pipe to control the current instance
        """
        self._pipe = pipe

    def __call__(self):
        """
        Start the tracking
        The function will keep running until any data is received through the pipe
        """
        self._setup()
        self._pipe.send(Status.READY)
        while not self._pipe.poll():
            pass

        self._begin_time = tm.time()
        self._remote_begin_time = None
        self._is_running = True

        while self._is_running:
            plt.show()
            plt.pause(0.001)

            while self._pipe.poll():
                obj = self._pipe.recv()
                {
                    order.Measure: lambda x: self._plot_measure(x),
                    Command: lambda x: self._resolve_command(x),
                }.get(type(obj))(obj)
            self._update_cursor()

        # Get the lists of points for left and right ticks
        left_ticks_points = list(
            zip(self._left_ticks_plot.get_ydata(), self._left_ticks_plot.get_xdata())
        )
        right_ticks_points = list(
            zip(self._right_ticks_plot.get_ydata(), self._right_ticks_plot.get_xdata())
        )

        # Sort them to correct any lag
        left_ticks_points.sort()
        right_ticks_points.sort()

        # Make tuples of list out of these lists of tuples
        left_ticks_axis = tuple(map(list, zip(*left_ticks_points)))
        right_ticks_axis = tuple(map(list, zip(*right_ticks_points)))

        self._pipe.send((left_ticks_axis[0], left_ticks_axis[1], right_ticks_axis[1]))

    def _setup(self):
        """
        Setup the pyplot display
        """
        plt.ion()

        self._fig = plt.figure()

        self._delta_ax = self._fig.add_subplot(1, 3, 2)
        self._delta_ax.set_xlim(-1, 1)
        self._delta_ax.set_ylim(0, 7)
        self._delta_ax.set_xlabel("Tick delta")
        self._delta_ax.set_ylabel("Time (s)")
        self._delta_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._left_ax = self._fig.add_subplot(1, 3, 1)
        self._left_ax.set_xlim(0, 1)
        self._left_ax.set_ylim(0, 7)
        self._left_ax.set_xlabel("Left ticks")
        self._left_ax.set_ylabel("Time (s)")
        self._left_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._right_ax = self._fig.add_subplot(1, 3, 3)
        self._right_ax.set_xlim(0, 1)
        self._right_ax.set_ylim(0, 7)
        self._right_ax.set_xlabel("Right ticks")
        self._right_ax.set_ylabel("Time (s)")
        self._right_ax.grid(color="xkcd:gunmetal", linestyle=":")
        self._axes = [self._delta_ax, self._left_ax, self._right_ax]

        self._cursors = list(
            map(lambda x: x.axhline(y=0, color="r", linestyle="--"), self._axes)
        )

        self._timer_texts = list(
            map(lambda x: x.text(s="", x=102, y=0, color="r"), self._axes)
        )

        self._delta_plot = self._delta_ax.plot(
            [], [], color="b", marker="s", linestyle="none"
        )[0]
        self._left_ref_ticks_plot = self._left_ax.plot(
            [], [], color="xkcd:lavender", marker="s", linestyle="none"
        )[0]
        self._right_ref_ticks_plot = self._right_ax.plot(
            [], [], color="xkcd:lavender", marker="s", linestyle="none"
        )[0]
        self._left_ticks_plot = self._left_ax.plot(
            [], [], color="g", marker="s", linestyle="none"
        )[0]
        self._right_ticks_plot = self._right_ax.plot(
            [], [], color="g", marker="s", linestyle="none"
        )[0]
        self._plots = [
            self._delta_plot,
            self._left_ticks_plot,
            self._right_ticks_plot,
        ]

    def _plot_measure(self, measure):
        """
        Plot a new measure
        """
        measure_data = [
            measure.left_encoder_ticks() - measure.right_encoder_ticks(),
            measure.left_encoder_ticks(),
            measure.right_encoder_ticks(),
        ]

        if self._remote_begin_time is None:
            self._remote_begin_time = measure.time_us() * 1e-3
        measure_timestamp = measure.time_us() * 1e-3 - self._remote_begin_time

        for plot in self._plots:
            plot.set_ydata(np.append(plot.get_ydata(), measure_timestamp))
        for plot, datum in zip(self._plots, measure_data):
            plot.set_xdata(np.append(plot.get_xdata(), datum))

        self._left_ref_ticks_plot.set_xdata(self._right_ticks_plot.get_xdata())
        self._left_ref_ticks_plot.set_ydata(self._right_ticks_plot.get_ydata())
        self._right_ref_ticks_plot.set_xdata(self._left_ticks_plot.get_xdata())
        self._right_ref_ticks_plot.set_ydata(self._left_ticks_plot.get_ydata())

    def _update_cursor(self):
        """
        Update the real-time cursor
        """
        time = tm.time() - self._begin_time

        for ax, plot in zip(self._axes, self._plots):
            ax.set_ylim(max(time - 6, 0), max(time + 1, 7))
            ax.set_xlim(
                min(plot.get_xdata()[-1], ax.get_xlim()[0]),
                max(plot.get_xdata()[-1], ax.get_xlim()[1]),
            )

        for cursor in self._cursors:
            cursor.set_ydata([time, time])

        for text in self._timer_texts:
            text.set_y(time)
            text.set_text(str("%.2f" % time) + "s")

    def _resolve_command(self, command):
        """
        Apply the effect of a received command
        """
        {
            Command.STOP: self._stop,
        }.get(command)()

    def _stop(self):
        self._is_running = False
