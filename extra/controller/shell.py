"""
Shell interface
"""

import time as tm
import numpy as np
import controller_order as order
import random as random
import matplotlib.pyplot as plt

from encoder_tracker import EncoderTracker

j = 0
t = 0
noise1 = 0
noise2 = 0


class Stream:
    def read(self):
        global j, t, noise1, noise2

        if j == 0:
            t = int(tm.time() * 1e3)
            noise1 += int(10 * random.random() - 5)
            noise2 += int(10 * random.random() - 5)
            noise1 = min(max(noise1, 0), 30)
            noise2 = min(max(noise2, 0), 30)

        L = [
            0,
            0,
            t & 0xFF,
            (t >> 8) & 0xFF,
            int(100 * (np.sin(t * 1e-3) + 1)) + noise1,
            0,
            int(10 * (np.cos(t * 1e-3) + 1)) + noise2,
            0,
        ]
        x = L[j]
        j += 1
        j %= len(L)
        return x

    def write(self, x):
        pass

    def skip_to_frame(self):
        return True


encoder_tracker = EncoderTracker()
stream = Stream()
with encoder_tracker:
    time = tm.time()
    while tm.time() < time + 15:
        encoder_tracker.pipe.send(order.execute(stream.read, stream.write))
        tm.sleep(1e-2)

plt.plot(encoder_tracker.time_us, encoder_tracker.left_ticks)
plt.plot(encoder_tracker.time_us, encoder_tracker.right_ticks)
plt.show()
