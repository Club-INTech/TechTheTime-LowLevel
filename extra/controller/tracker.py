"""
Encoder state tracking
"""

import multiprocessing as mp
import time as tm
from ctypes import c_double
from enum import Enum

import controller_rpc as rpc
import matplotlib.pyplot as plt
import matplotlib.transforms as tsf
import numpy as np
from utility.match import Match

TRACKER_SETUP_TIMEOUT_S = 3
REFRESH_DELAY_S = 0.001


class Tracker:
    """
    Handles a pyplot display that tracks encoder measures
    The instances of this class expose a pipe to send measures to the process handling the pyplot display.
    The process can be made ready and available to plot incoming measure by using the instances as context managers.
    """

    def __init__(self):
        """
        Creates pipes to feed the tracker data
        """
        self.pipe, self._remote_pipe = mp.Pipe()
        self._latest_measure_date_s = mp.Value(c_double, 0)

    def __enter__(self):
        """
        Start the tracking
        """
        self._process = mp.Process(
            target=_TrackerProcess(
                pipe=self._remote_pipe,
                latest_measure_date_s=self._latest_measure_date_s,
            ),
            daemon=True,
        )
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
        Match(self.pipe.recv()) & {tuple: lambda t: self._set_measures(*t)}

    @property
    def timeout_counter_s(self):
        """
        Time since the last received measure
        """
        return tm.time() - self._latest_measure_date_s.value

    def reset_timeout_counter(self):
        """
        Reinitialize the timeout counter back to zero
        """
        self._latest_measure_date_s.value = tm.time()

    def _set_measures(self, time_us, left_ticks, right_ticks):
        """
        Save the measures sent by the process
        """
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


class _TrackerProcess:
    """
    Plots encoder measures received from a given stream on a pyplot display
    The display will consists the plots of the encoder measurers and a real-time cursor indicating the current time.
    """

    def __init__(self, pipe, latest_measure_date_s):
        """
        Hold a pipe to control the current instance and a floating point reference to indicate the last time a measure was received
        """
        self._pipe = pipe
        self._latest_measure_date_s = latest_measure_date_s

    def __call__(self):
        """
        Start the tracking
        The function will keep running until any data is received through the pipe
        """

        # Setup the pyplot display, notify the other end of the pipe and wait for a command or a measure
        self._setup()
        self._pipe.send(Status.READY)
        while not self._pipe.poll():
            pass

        self._begin_time = tm.time()
        self._remote_begin_time_s = None
        self._is_running = True

        while self._is_running:
            # Update the display with data from pipe
            while self._pipe.poll():
                Match(self._pipe.recv()) & {
                    rpc.Measure: self._plot_measure,
                    Command: Match() & {Command.STOP: self._stop},
                }
            self._update_cursor()

            # Refresh the display
            plt.show()
            plt.pause(REFRESH_DELAY_S)

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

        # Send back the measures after checking if anything has been plot
        self._pipe.send(
            (left_ticks_axis[0], left_ticks_axis[1], right_ticks_axis[1])
            if len(left_ticks_axis) == 2
            else ([], [], [])
        )

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
        self._latest_measure_date_s.value = tm.time()

        measure_data = [
            measure.left_encoder_ticks - measure.right_encoder_ticks,
            measure.left_encoder_ticks,
            measure.right_encoder_ticks,
        ]

        if self._remote_begin_time_s is None:
            self._remote_begin_time_s = measure.time_us * 1e-6
        measure_timestamp = measure.time_us * 1e-6 - self._remote_begin_time_s

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
            if plot.get_xdata().size > 0:
                ax.set_xlim(
                    min(plot.get_xdata()[-1], ax.get_xlim()[0]),
                    max(plot.get_xdata()[-1], ax.get_xlim()[1]),
                )

        for cursor in self._cursors:
            cursor.set_ydata([time, time])

        for text in self._timer_texts:
            text.set_y(time)
            text.set_text(str("%.2f" % time) + "s")

    def _stop(self):
        """
        Stop the display
        """
        self._is_running = False
