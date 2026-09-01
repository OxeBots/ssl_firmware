"""Synthetic debug telemetry so the GUI works without hardware."""

import math
import random
import threading
import time

from . import config
from . import protocol as proto


class DemoSource(threading.Thread):
    """Generates plausible debug telemetry so the GUI works without the rig."""

    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.tof_seq = 0
        self.imu_seq = 0
        self.health_seq = 0
        self.vel = [0.0] * proto.WHEEL_COUNT
        self.enc = 0.0

    def run(self):
        rng = random.Random(7)
        next_state = 0.0
        next_imu = 0.0
        next_tof = 0.0
        next_health = 0.0
        start = time.monotonic()
        while not self.state.quit:
            now = time.monotonic()
            t = now - start
            targets = list(self.state.manual_targets)
            if self.state.sweep:
                targets[self.state.selected_wheel] = (
                    config.SWEEP_RPM if (t % 2.0) < 1.0 else -config.SWEEP_RPM
                )
            pwm = [0.0] * proto.WHEEL_COUNT
            for i in range(proto.WHEEL_COUNT):
                self.vel[i] += (targets[i] - self.vel[i]) * min(1.0, 0.02 / 0.18)
                self.vel[i] += rng.uniform(-8.0, 8.0)
                pwm[i] = max(-1023.0, min(1023.0, targets[i] * 0.22 + (targets[i] - self.vel[i]) * 0.45))
            self.enc = (self.enc + self.vel[0] / 60.0 * 4096.0 * 0.02) % 4096.0
            yaw = 90.0 + 25.0 * math.sin(0.35 * t)
            pitch = 18.0 * math.sin(0.6 * t)
            roll = 12.0 * math.sin(0.45 * t + 1.0)

            if now >= next_state:
                next_state = now + 0.04
                wheels = [
                    {"pwm": pwm[i], "vel_rpm": self.vel[i], "tgt_rpm": targets[i]}
                    for i in range(proto.WHEEL_COUNT)
                ]
                self.state.update_state({
                    "uptime_ms": int(t * 1000),
                    "wheels": wheels,
                    "yaw": yaw,
                    "pitch": pitch,
                    "roll": roll,
                    "valid_mask": 0xF,
                    "sequence": int(t * 25),
                })
            if now >= next_imu:
                next_imu = now + 0.1
                self.imu_seq += 1
                self.state.update_imu({
                    "accel": [
                        0.05 * math.sin(0.7 * t),
                        0.04 * math.cos(0.5 * t),
                        0.98 + 0.02 * math.sin(0.9 * t),
                    ],
                    "gyro": [3.0 * math.sin(0.8 * t), 2.0 * math.cos(0.6 * t), 1.5 * math.sin(0.4 * t)],
                    "mag": [220 + 40 * math.sin(0.3 * t), 180 + 30 * math.cos(0.35 * t), 410],
                    "sequence": self.imu_seq,
                })
            if now >= next_tof:
                next_tof = now + 1.0 / 15.0
                self.tof_seq += 1
                self.state.update_tof(
                    {"sequence": self.tof_seq, "distances": self._demo_tof(t, rng)}
                )
            if now >= next_health:
                next_health = now + 0.5
                self.health_seq += 1
                self.state.update_health({
                    "robot_id": 3,
                    "error_flags": 0,
                    "lifecycle": 3,
                    "active_faults": 0,
                    "latched_faults": 0,
                    "motion_inhibited": 0,
                    "kick_inhibited": 1,
                    "imu_cal_flags": 7,
                    "tof_available": 1,
                    "supply_voltage": 14.6 + 0.05 * math.sin(0.2 * t),
                    "supply_flags": 1,
                    "free_heap": 178000 + int(2000 * math.sin(0.1 * t)),
                    "min_free_heap": 165000,
                    "sequence": self.health_seq,
                })
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
