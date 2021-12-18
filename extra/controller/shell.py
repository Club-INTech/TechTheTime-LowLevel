"""
Shell interface
"""

import matplotlib.pyplot as plt
import matplotlib.transforms as tsf
import numpy as np
import controller_order as order
import random as rnd
import time as tm
import math

time = 0
time0 = 0
j = 0
t = 0


def input_stream():
    global time, j, t

    if j == 0:
        t = int((time - time0) * 1e3)

    L = [
        0,
        0,
        t & 0xFF,
        (t >> 8) & 0xFF,
        int(100 * (np.sin(t * 1e-3) + 1)),
        0,
        int(100 * (np.cos(t * 1e-3) + 1)),
        0,
    ]
    x = L[j]
    j += 1
    j %= len(L)
    return x


def output_stream(x):
    None


plt.ion()
fig = plt.figure()

delta_ax = fig.add_subplot(1, 3, 2)
delta_ax.set_xlim(-200, 200)
delta_ax.set_ylim(0, 7)
delta_ax.set_xlabel("Tick delta")
delta_ax.set_ylabel("Time (s)")
delta_ax.grid(color="xkcd:gunmetal", linestyle=":")

left_ax = fig.add_subplot(1, 3, 1)
left_ax.set_xlim(-200, 200)
left_ax.set_ylim(0, 7)
left_ax.set_xlabel("Left ticks")
left_ax.set_ylabel("Time (s)")
left_ax.grid(color="xkcd:gunmetal", linestyle=":")

right_ax = fig.add_subplot(1, 3, 3)
right_ax.set_xlim(-200, 200)
right_ax.set_ylim(0, 7)
right_ax.set_xlabel("Right ticks")
right_ax.set_ylabel("Time (s)")
right_ax.grid(color="xkcd:gunmetal", linestyle=":")
axes = [delta_ax, right_ax, left_ax]

delta_cursor = delta_ax.axhline(y=time, color="r", linestyle="--")
left_cursor = left_ax.axhline(y=time, color="r", linestyle="--")
right_cursor = right_ax.axhline(y=time, color="r", linestyle="--")
cursors = [delta_cursor, left_cursor, right_cursor]

delta_timer_text = delta_ax.text(s=str(time / 100) + "s", x=102, y=0, color="r")
left_timer_text = left_ax.text(s=str(time / 100) + "s", x=102, y=0, color="r")
right_timer_text = right_ax.text(s=str(time / 100) + "s", x=102, y=0, color="r")
texts = [delta_timer_text, left_timer_text, right_timer_text]

delta_plot = delta_ax.plot([], [], color="b", marker=".", linestyle="none")[0]
left_ticks_plot = left_ax.plot([], [], color="g", marker=".", linestyle="none")[0]
left_ref_ticks_plot = left_ax.plot([], [], color="g", marker=".", linestyle="none")[0]
right_ticks_plot = right_ax.plot([], [], color="g", marker=".", linestyle="none")[0]
right_ref_ticks_plot = right_ax.plot([], [], color="g", marker=".", linestyle="none")[0]
plots = [
    delta_plot,
    left_ticks_plot,
    left_ref_ticks_plot,
    right_ticks_plot,
    right_ref_ticks_plot,
]

time0 = tm.time()
for _ in range(1000):
    time = tm.time()
    measure = order.execute(input_stream, output_stream)
    measure_timestamp = measure.time_us()
    measure_data = [
        measure.left_encoder_ticks() - measure.right_encoder_ticks(),
        measure.left_encoder_ticks(),
        measure.right_encoder_ticks(),
        measure.right_encoder_ticks(),
        measure.left_encoder_ticks(),
    ]

    for plot in plots:
        plot.set_ydata(np.append(plot.get_ydata(), measure.time_us() * 1e-3))
    for plot, datum in zip(plots, measure_data):
        plot.set_xdata(np.append(plot.get_xdata(), datum))

    delta_ax.set_ylim(max(time - time0 - 6, 0), max(time - time0 + 1, 7))
    left_ax.set_ylim(max(time - time0 - 6, 0), max(time - time0 + 1, 7))
    right_ax.set_ylim(max(time - time0 - 6, 0), max(time - time0 + 1, 7))

    for cursor in cursors:
        cursor.set_ydata([time - time0, time - time0])

    for text in texts:
        text.set_y(time - time0)
        text.set_text(str("%.2f" % (time - time0)) + "s")

    plt.show()
    plt.pause(0.01)
    time += 1
