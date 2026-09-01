"""Synthetic S/T streams so the GUI works without hardware."""

import math
import random
import threading
import time

from . import config


class DemoSource(threading.Thread):
    """Generates plausible S/T streams so the GUI works without the rig."""

    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.tof_seq = 0

    def run(self):
        rng = random.Random(7)
        vel = 0.0
        enc = 0.0
        next_state = 0.0
        next_tof = 0.0
        start = time.monotonic()
        while not self.state.quit:
            now = time.monotonic()
            t = now - start
            if self.state.manual_mode:
                target = self.state.manual_tgt
            else:
                target = 1000.0 if (t % 2.0) < 1.0 else -1000.0
            vel += (target - vel) * min(1.0, 0.02 / 0.18)  # first-order lag
            vel += rng.uniform(-8.0, 8.0)
            pwm = max(-1023.0, min(1023.0, target * 0.22 + (target - vel) * 0.45))
            enc = (enc + vel / 60.0 * 4096.0 * 0.02) % 4096.0
            yaw = 90.0 + 25.0 * math.sin(0.35 * t)
            pitch = 18.0 * math.sin(0.6 * t)
            roll = 12.0 * math.sin(0.45 * t + 1.0)

            if now >= next_state:
                next_state = now + 0.04
                self.state.update_state(
                    int(t * 1000), yaw, pitch, roll, pwm, vel, target, int(enc), 1, self.tof_seq
                )
            if now >= next_tof:
                next_tof = now + 1.0 / 15.0
                self.tof_seq += 1
                self.state.update_tof(self.tof_seq, self._demo_tof(t, rng))
            time.sleep(0.004)

    @staticmethod
    def _demo_tof(t, rng):
        ball_x = 3.5 + 2.5 * math.sin(0.4 * t)
        ball_y = 3.5 + 2.0 * math.cos(0.3 * t)
        frame = []
        for row in range(config.TOF_SIDE):
            for col in range(config.TOF_SIDE):
                gap = math.hypot(row - ball_y, col - ball_x)
                if gap < 1.4:
                    frame.append(int(220 + 60 * gap + rng.uniform(-15, 15)))
                else:
                    frame.append(int(950 + rng.uniform(-35, 35)))
        return frame
