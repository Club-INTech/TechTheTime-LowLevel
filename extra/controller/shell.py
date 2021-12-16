"""
Shell interface
"""

import matplotlib.pyplot as plt
import numpy as np
import controller_order as order
import random as rnd

j = 0


def input_stream():
    global j
    L = [
        0,
        0,
        int(rnd.random() * 100),
        0,
        int(rnd.random() * 100),
        0,
        int(rnd.random() * 100),
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
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)

left_ticks_plot = ax.plot([], [], color="b", marker="x", linestyle="none")[0]
right_ticks_plot = ax.plot([], [], color="r", marker="x", linestyle="none")[0]

for i in range(1, 100):
    measure = order.execute(input_stream, output_stream)

    left_ticks_plot.set_xdata(np.append(left_ticks_plot.get_xdata(), measure.time_us()))
    right_ticks_plot.set_xdata(
        np.append(right_ticks_plot.get_xdata(), measure.time_us())
    )
    left_ticks_plot.set_ydata(
        np.append(left_ticks_plot.get_ydata(), measure.left_encoder_ticks())
    )
    right_ticks_plot.set_ydata(
        np.append(right_ticks_plot.get_ydata(), measure.right_encoder_ticks())
    )
    plt.show()
    plt.pause(0.07)
