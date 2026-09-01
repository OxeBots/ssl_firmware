"""Thread-shared snapshot of the latest bench telemetry."""

import collections
import threading
import time

from . import config


class BenchState:
    """Thread-shared snapshot of the latest bench telemetry."""

    def __init__(self):
        self.demo = False
        self.quit = False
        self.connected = None  # port name, or None while disconnected
        # State line fields
        self.ms = 0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.pwm = 0.0
        self.vel_rpm = 0.0
        self.tgt_rpm = 0.0
        self.enc_raw = 0
        self.enc_ok = 0
        self.tof_seq = 0
        # ToF frame
        self.tof = [0] * (config.TOF_SIDE * config.TOF_SIDE)
        self.tof_seq_seen = -1
        # Bookkeeping
        self.last_state_t = 0.0
        self.last_tof_t = 0.0
        self.state_count = 0
        self.tof_count = 0
        self.console = collections.deque(maxlen=5)
        self.history = collections.deque(maxlen=2000)  # (t, vel, tgt) for the strip chart
        self.history_lock = threading.Lock()
        # Command routing (real serial via reader, emulated in demo mode)
        self.reader = None
        self.scanner = None
        self.manual_mode = False
        self.manual_tgt = 0.0

    def update_state(self, ms, yaw, pitch, roll, pwm, vel_rpm, tgt_rpm, enc_raw, enc_ok, tof_seq):
        now = time.monotonic()
        self.ms = ms
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.pwm = pwm
        self.vel_rpm = vel_rpm
        self.tgt_rpm = tgt_rpm
        self.enc_raw = enc_raw
        self.enc_ok = enc_ok
        self.tof_seq = tof_seq
        self.last_state_t = now
        self.state_count += 1
        with self.history_lock:
            self.history.append((now, vel_rpm, tgt_rpm))

    def update_tof(self, seq, distances):
        self.tof = distances
        self.tof_seq_seen = seq
        self.last_tof_t = time.monotonic()
        self.tof_count += 1

    def send_command(self, text):
        """Forward a command to the firmware (or emulate it in demo mode)."""
        self.console.append("> " + text)
        if self.demo:
            if text.startswith("T"):
                try:
                    self.manual_tgt = float(text[1:])
                    self.manual_mode = True
                except ValueError:
                    pass
            elif text == "0":
                self.manual_mode = True
                self.manual_tgt = 0.0
            elif text == "W":
                self.manual_mode = False
        elif self.reader is not None:
            self.reader.send(text)
