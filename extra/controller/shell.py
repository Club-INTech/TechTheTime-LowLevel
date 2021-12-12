"""
Shell interface
"""

import matplotlib.pyplot as plt
import numpy as np


plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, np.log(100))

line = ax.plot([], [])[0]

for i in range(1, 100):
    line.set_xdata(range(1, i))
    line.set_ydata(np.log(range(1, i)))
    plt.show()
    plt.pause(0.07)
