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
    L = [0, 0, int(rnd.random() * 10), 0, 0, 0]
    x = L[j]
    j += 1
    j %= len(L)
    return x


def output_stream(x):
    print(x)


plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, 10)

line = ax.plot([], [])[0]

L = []

for i in range(1, 100):
    L.append(order.execute(input_stream, output_stream)[0])

    line.set_xdata(range(0, i))
    line.set_ydata(np.array(L))
    plt.show()
    plt.pause(0.07)
