"""
Shell interface
"""

import time as tm
import numpy as np

from encoder_tracker import EncoderTracker

j = 0
t = 0


class Stream:
    def read(self):
        global j, t

        if j == 0:
            t = int(tm.time() * 1e3)

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

    def write(self, x):
        pass

    def skip_to_frame(self):
        return True


with EncoderTracker(Stream()) as encoder_tracker:
    time = tm.time()
    while tm.time() < time + 5:
        pass
