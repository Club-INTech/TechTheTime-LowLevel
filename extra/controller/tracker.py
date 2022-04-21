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
REFRESH_DELAY_S = 1e-3


class Tracker:
    """
    Handles a pyplot display that tracks encoder measures
    The instances of this class expose a pipe to send measures to the process handling the pyplot display.
    The process can be made ready and available to plot incoming measure by using the instances as context managers.
    """

    def __init__(self, remote_pipe):
        """
        Creates pipes to feed the tracker data
        """
        self.pipe, self._pipe = mp.Pipe()
        self._remote_pipe = remote_pipe
        self.shows_record = False
        self._status = Status.STOPPED
        self._latest_measure_date_s = mp.Value(c_double, 0)

    def __enter__(self):
        """
        Start the tracking
        """
        self._process = mp.Process(
            target=_TrackerProcess(
                pipe=self._pipe,
                remote_pipe=self._remote_pipe,
                latest_measure_date_s=self._latest_measure_date_s,
                shows_record=self.shows_record,
            ),
            daemon=True,
        )
        self._process.start()
        if not self.wait_status(Status.READY, timeout=TRACKER_SETUP_TIMEOUT_S):
            raise RuntimeError(
                "Tracker failed initialization with status code " + str(self._status)
            )

    def __exit__(self, *_):
        """
        End the tracking
        """
        if self.get_status() == Status.READY:
            self.pipe.send(Command.STOP)
        self._process.join()

    def get_status(self):
        timestamp = tm.time()
        while self.pipe.poll():
            self._status = self.pipe.recv()
        return self._status

    def wait_status(self, status, timeout):
        timestamp = tm.time()
        while tm.time() - timestamp < timeout:
            if self.pipe.poll():
                self._status = self.pipe.recv()
                if self._status == status:
                    return True
        return False

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
    SHOWING_RECORD = 1
    STOPPED = 2


class Setpoint:
    def __init__(self, value):
        self.value = value


class _TrackerProcess:
    """
    Plots encoder measures received from a given stream on a pyplot display
    The display will consists the plots of the encoder measurers and a real-time cursor indicating the current time.
    """

    def __init__(self, pipe, remote_pipe, latest_measure_date_s, shows_record):
        """
        Hold a pipe to control the current instance and a floating point reference to indicate the last time a measure was received
        """
        self._pipe = pipe
        self._remote_pipe = remote_pipe
        self._latest_measure_date_s = latest_measure_date_s
        self._sanity_epsilon = 1000
        self._trigger_threshold = 10
        self._time_window_s = 3
        self._shows_record = shows_record
        self._setpoint = None
        self._is_running = False
        self._bias_sum = 0

    def __call__(self):
        """
        Start the tracking
        The function will keep running until any data is received through the pipe
        """

        self._setup_display()

        # Purge the remote pipe from remaining measures from last tracking
        while self._remote_pipe.poll():
            self._remote_pipe.recv()

        self._pipe.send(Status.READY)
        while not self._is_running:
            # Start tracking when detecting a measure beyond the detection threshold
            if self._remote_pipe.poll():
                obj = self._remote_pipe.recv()
                if type(obj) is rpc.Measure and (
                    obj.left_encoder_ticks > self._trigger_threshold
                    or obj.right_encoder_ticks > self._trigger_threshold
                ):
                    self._is_running = True

            # Abort the tracking if the main thread decides so
            if self._pipe.poll() and self._pipe.recv() == Command.STOP:
                return
            tm.sleep(REFRESH_DELAY_S)

        self._begin_time = tm.time()
        self._remote_begin_time_s = None

        while self._is_running:
            # Poll the main thread pipe for commands
            if self._pipe.poll():
                Match(self._pipe.recv()) & {
                    Command: Match() & {Command.STOP: self._stop},
                    Setpoint: self._set_setpoint,
                }
            # Empty the remote pipe completely, as measures will be send at high pace
            while self._remote_pipe.poll():
                Match(self._remote_pipe.recv()) & {
                    rpc.Measure: self._plot_measure,
                }
            self._update_cursor()

            # Refresh the display
            plt.show()
            plt.pause(REFRESH_DELAY_S)

        plt.close(self._fig)

        if self._shows_record and len(self._left_ticks_plot.get_xdata()) > 0:
            self._pipe.send(Status.SHOWING_RECORD)
            self._show_record()

        self._pipe.send(Status.STOPPED)

    def _setup_display(self):
        """
        Setup the pyplot display
        """
        plt.ion()

        self._fig = plt.figure()

        self._delta_ax = self._fig.add_subplot(1, 3, 2)
        self._delta_ax.set_xlim(-1, 1)
        self._delta_ax.set_ylim(0, self._time_window_s)
        self._delta_ax.set_xlabel("Tick delta")
        self._delta_ax.set_ylabel("Time (s)")
        self._delta_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._left_ax = self._fig.add_subplot(1, 3, 1)
        self._left_ax.set_xlim(0, 1)
        self._left_ax.set_ylim(0, self._time_window_s)
        self._left_ax.set_xlabel("Left ticks")
        self._left_ax.set_ylabel("Time (s)")
        self._left_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._right_ax = self._fig.add_subplot(1, 3, 3)
        self._right_ax.set_xlim(0, 1)
        self._right_ax.set_ylim(0, self._time_window_s)
        self._right_ax.set_xlabel("Right ticks")
        self._right_ax.set_ylabel("Time (s)")
        self._right_ax.grid(color="xkcd:gunmetal", linestyle=":")
        self._axes = [self._delta_ax, self._left_ax, self._right_ax]

        self._cursors = list(
            map(lambda x: x.axhline(y=0, color="r", linestyle="--"), self._axes)
        )

        self._middle_line = self._delta_ax.axvline(
            x=0, color="xkcd:gunmetal", marker=".", linestyle="-"
        )

        self._timer_texts = list(
            map(lambda x: x.text(s="", x=102, y=0, color="r"), self._axes)
        )

        self._bias_text = self._fig.text(s="", x=0.05, y=0.95)

        self._delta_plot = self._delta_ax.plot(
            [], [], color="b", marker=".", linestyle="none"
        )[0]
        self._left_ref_ticks_plot = self._left_ax.plot(
            [], [], color="xkcd:lavender", marker=".", linestyle="none"
        )[0]
        self._right_ref_ticks_plot = self._right_ax.plot(
            [], [], color="xkcd:lavender", marker=".", linestyle="none"
        )[0]
        self._left_ticks_plot = self._left_ax.plot(
            [], [], color="g", marker=".", linestyle="none"
        )[0]
        self._right_ticks_plot = self._right_ax.plot(
            [], [], color="g", marker=".", linestyle="none"
        )[0]
        self._plots = [
            self._delta_plot,
            self._left_ticks_plot,
            self._right_ticks_plot,
        ]

        self._fig.canvas.mpl_connect("close_event", self._stop)

    def _plot_measure(self, measure):
        """
        Plot a new measure
        """
        self._latest_measure_date_s.value = tm.time()

        # Data sanity check
        if self._remote_begin_time_s is not None and (
            abs(measure.left_encoder_ticks - self._left_ticks_plot.get_xdata()[-1])
            > self._sanity_epsilon
            or abs(measure.right_encoder_ticks - self._right_ticks_plot.get_xdata()[-1])
            > self._sanity_epsilon
        ):
            return

        measure_data = [
            measure.right_encoder_ticks - measure.left_encoder_ticks,
            measure.left_encoder_ticks,
            measure.right_encoder_ticks,
        ]

        if self._remote_begin_time_s is None:
            self._remote_begin_time_s = measure.time_us * 1e-3
        measure_timestamp = measure.time_us * 1e-3 - self._remote_begin_time_s

        for plot in self._plots:
            plot.set_ydata(np.append(plot.get_ydata(), measure_timestamp))
        for plot, datum in zip(self._plots, measure_data):
            plot.set_xdata(np.append(plot.get_xdata(), datum))

        self._left_ref_ticks_plot.set_xdata(self._right_ticks_plot.get_xdata())
        self._left_ref_ticks_plot.set_ydata(self._right_ticks_plot.get_ydata())
        self._right_ref_ticks_plot.set_xdata(self._left_ticks_plot.get_xdata())
        self._right_ref_ticks_plot.set_ydata(self._left_ticks_plot.get_ydata())

        self._bias_sum += measure_data[0]

        self._bias_text.set_text(
            f"Mean bias {self._bias_sum / len(self._left_ticks_plot.get_xdata())}"
        )

    def _set_setpoint(self, setpoint):
        """
        Set the setpoint of the current movement
        """
        self._setpoint = setpoint

    def _update_cursor(self):
        """
        Update the real-time cursor
        """
        time = tm.time() - self._begin_time

        self._left_ax.invert_xaxis()
        for ax, plot in zip(self._axes, self._plots):
            ax.set_ylim(
                max(time - self._time_window_s + 1, 0),
                max(time + 1, self._time_window_s),
            )
            if plot.get_xdata().size > 0:
                ax.set_xlim(
                    min(plot.get_xdata()[-1], ax.get_xlim()[0]),
                    max(plot.get_xdata()[-1], ax.get_xlim()[1]),
                )
        self._left_ax.invert_xaxis()

        for cursor in self._cursors:
            cursor.set_ydata([time, time])

        for text in self._timer_texts:
            text.set_y(time)
            text.set_text(str("%.2f" % time) + "s")

    def _stop(self, evt=None):
        """
        Stop the display
        """
        self._is_running = False

    def _show_record(self):
        """
        Show the tracking result in a non-interactive figure
        """

        # Get the lists of points for left and right ticks
        left_ticks_points = list(
            zip(self._left_ticks_plot.get_ydata(), self._left_ticks_plot.get_xdata())
        )
        right_ticks_points = list(
            zip(
                self._right_ticks_plot.get_ydata(),
                self._right_ticks_plot.get_xdata(),
            )
        )

        # Sort them to correct any lag
        left_ticks_points.sort()
        right_ticks_points.sort()

        # Make tuples of list out of these lists of tuples
        left_ticks_axis = tuple(map(list, zip(*left_ticks_points)))
        right_ticks_axis = tuple(map(list, zip(*right_ticks_points)))

        # Make derivatives of these plots
        left_ticks_dt_axis, right_ticks_dt_axis = (left_ticks_axis[0][1:], []), (
            right_ticks_axis[0][1:],
            [],
        )
        (last_t, last_x) = left_ticks_points[0]
        for (t, x) in left_ticks_points[1:]:
            left_ticks_dt_axis[1].append((x - last_x) / (t - last_t))
            last_t = t
            last_x = x
        (last_t, last_x) = right_ticks_points[0]
        for (t, x) in right_ticks_points[1:]:
            right_ticks_dt_axis[1].append((x - last_x) / (t - last_t))
            last_t = t
            last_x = x

        plt.ioff()
        fig, ax = plt.subplots()
        ax.grid(color="xkcd:gunmetal", linestyle=":")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Ticks")
        ax.set_xlim(0, left_ticks_axis[0][-1])
        ax.plot(*left_ticks_axis, color="r", label="Left")
        ax.plot(*right_ticks_axis, color="b", label="Right")

        dt_ax = ax.twinx()
        dt_ax.set_ylabel("Ticks derivative (s-1)")
        dt_ax.plot(
            *left_ticks_dt_axis,
            color="tab:red",
            linestyle="dotted",
            label="Left derivative",
        )
        dt_ax.plot(
            *right_ticks_dt_axis,
            color="tab:blue",
            linestyle="dotted",
            label="Right derivative",
        )

        fig.text(
            s=f"Mean bias: {self._bias_sum / len(left_ticks_axis[0])}", x=0.05, y=0.95
        )

        if self._setpoint:
            ax.axhline(y=self._setpoint.value, color="g", linestyle="--")
        plt.legend(loc="best")
        plt.show()
