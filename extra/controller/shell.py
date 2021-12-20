"""
Shell interface
"""

import time as tm
import numpy as np
import controller_order as order
import random as random
import matplotlib.pyplot as plt
import cmd

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
            0,
            0,
            int(100 * (np.sin(t * 1e-3) + 1)) + noise1,
            0,
            0,
            0,
            int(10 * (np.cos(t * 1e-3) + 1)) + noise2,
            0,
            0,
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


class Shell(cmd.Cmd):
    prompt = "[shell] -- "

    def __init__(self):
        super().__init__()
        self._encoder_tracker = None

    def do_track(self, _):
        print("Arming the tracker...")
        self._encoder_tracker = EncoderTracker()
        old_prompt = self.prompt
        self.prompt = "[shell > tracker] -- "

        with self._encoder_tracker:
            print("Tracker ready")
            print("Tracker will be disarmed when tracking is over or by typing 'quit'")
            self.cmdloop()

        print("Tracker is disarmed")
        self.prompt = old_prompt
        self._encoder_tracker = None

    def do_translate(self, _):
        print("Commanding remote to start a translation...")

        time = tm.time()
        stream = Stream()
        if self._encoder_tracker is not None:
            while tm.time() < time + 3:
                self._encoder_tracker.pipe.send(
                    order.execute(stream.read, stream.write)
                )
                tm.sleep(1e-2)

        return self._encoder_tracker is not None

    def do_quit(self, _):
        return True


Shell().cmdloop()
