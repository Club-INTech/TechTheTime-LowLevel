"""
Encoder state tracking
"""

import matplotlib.pyplot as plt
import matplotlib.transforms as tsf
import numpy as np
import controller_order as order
import random as rnd
import time as tm
import math
import multiprocessing as mp


class EncoderTracker:
    def __init__(self, stream):
        self._stream = stream

    def __enter__(self):
        self._pipe, remote_pipe = mp.Pipe()
        self._process = mp.Process(
            target=EncoderTrackerProcess(self._stream, remote_pipe), daemon=True
        )
        self._process.start()

    def __exit__(self, *_):
        self._pipe.send(None)
        self._process.join()


class EncoderTrackerProcess:
    def __init__(self, stream, pipe):
        self._stream = stream
        self._pipe = pipe

    def __call__(self):
        plt.ion()

        self._setup()

        self._begin_time = tm.time()
        self._remote_begin_time = None

        while not self._pipe.poll():
            if self._stream.skip_to_frame():
                self._plot_measure(order.execute(self._stream.read, self._stream.write))
            self._update_cursor()
            plt.show()
            plt.pause(0.01)

    def _setup(self):
        self._fig = plt.figure()

        self._delta_ax = self._fig.add_subplot(1, 3, 2)
        self._delta_ax.set_xlim(-200, 200)
        self._delta_ax.set_ylim(0, 7)
        self._delta_ax.set_xlabel("Tick delta")
        self._delta_ax.set_ylabel("Time (s)")
        self._delta_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._left_ax = self._fig.add_subplot(1, 3, 1)
        self._left_ax.set_xlim(-200, 200)
        self._left_ax.set_ylim(0, 7)
        self._left_ax.set_xlabel("Left ticks")
        self._left_ax.set_ylabel("Time (s)")
        self._left_ax.grid(color="xkcd:gunmetal", linestyle=":")

        self._right_ax = self._fig.add_subplot(1, 3, 3)
        self._right_ax.set_xlim(-200, 200)
        self._right_ax.set_ylim(0, 7)
        self._right_ax.set_xlabel("Right ticks")
        self._right_ax.set_ylabel("Time (s)")
        self._right_ax.grid(color="xkcd:gunmetal", linestyle=":")
        self._axes = [self._delta_ax, self._right_ax, self._left_ax]

        self._cursors = list(
            map(lambda x: x.axhline(y=0, color="r", linestyle="--"), self._axes)
        )

        self._timer_texts = list(
            map(lambda x: x.text(s="", x=102, y=0, color="r"), self._axes)
        )

        self._delta_plot = self._delta_ax.plot(
            [], [], color="b", marker=".", linestyle="none"
        )[0]
        self._left_ref_ticks_plot = self._left_ax.plot(
            [], [], color="xkcd:lavender", marker=".", linestyle="none"
        )[0]
        self._left_ticks_plot = self._left_ax.plot(
            [], [], color="g", marker=".", linestyle="none"
        )[0]
        self._right_ref_ticks_plot = self._right_ax.plot(
            [], [], color="xkcd:lavender", marker=".", linestyle="none"
        )[0]
        self._right_ticks_plot = self._right_ax.plot(
            [], [], color="g", marker=".", linestyle="none"
        )[0]
        self._plots = [
            self._delta_plot,
            self._left_ticks_plot,
            self._left_ref_ticks_plot,
            self._right_ticks_plot,
            self._right_ref_ticks_plot,
        ]

    def _plot_measure(self, measure):
        measure_data = [
            measure.left_encoder_ticks() - measure.right_encoder_ticks(),
            measure.left_encoder_ticks(),
            measure.right_encoder_ticks(),
            measure.right_encoder_ticks(),
            measure.left_encoder_ticks(),
        ]

        if self._remote_begin_time is None:
            self._remote_begin_time = measure.time_us() * 1e-3
        measure_timestamp = measure.time_us() * 1e-3 - self._remote_begin_time

        for plot in self._plots:
            plot.set_ydata(np.append(plot.get_ydata(), measure_timestamp))
        for plot, datum in zip(self._plots, measure_data):
            plot.set_xdata(np.append(plot.get_xdata(), datum))

    def _update_cursor(self):
        time = tm.time() - self._begin_time

        self._delta_ax.set_ylim(max(time - 6, 0), max(time + 1, 7))
        self._left_ax.set_ylim(max(time - 6, 0), max(time + 1, 7))
        self._right_ax.set_ylim(max(time - 6, 0), max(time + 1, 7))

        for cursor in self._cursors:
            cursor.set_ydata([time, time])

        for text in self._timer_texts:
            text.set_y(time)
            text.set_text(str("%.2f" % time) + "s")
