"""Thread-shared snapshot of the latest serial debug telemetry."""

import collections
import threading
import time

from . import config
from . import protocol as proto

WHEEL_NAMES = ("M1 FL", "M2 BL", "M3 BR", "M4 FR")


class BenchState:
    """Thread-shared snapshot of the latest serial debug telemetry."""

    def __init__(self):
        self.demo = False
        self.quit = False
        self.connected = None  # port name, or None while disconnected
        self.streaming = False
        # Per-wheel control data
        self.wheels = [
            {"pwm": 0.0, "vel": 0.0, "tgt": 0.0} for _ in range(proto.WHEEL_COUNT)
        ]
        self.valid_mask = 0
        # Attitude (degrees)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        # Raw IMU sensors (g, deg/s, mG)
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        # Health / system status
        self.health = {}
        self.uptime_ms = 0
        # ToF frame
        self.tof = [0] * (config.TOF_SIDE * config.TOF_SIDE)
        self.tof_seq_seen = -1
        # Bookkeeping
        self.last_state_t = 0.0
        self.last_imu_t = 0.0
        self.last_tof_t = 0.0
        self.last_health_t = 0.0
        self.state_count = 0
        self.imu_count = 0
        self.tof_count = 0
        self.health_count = 0
        self.rx_bytes = 0  # raw serial bytes received since connect
        self.console = collections.deque(maxlen=500)
        # Console view: 0 = follow the latest lines, >0 = lines scrolled back
        self.console_offset = 0
        # Console text selection: {"anchor": (line, char), "focus": (line, char)}
        # with line = index into the full console history
        self.console_sel = None
        # Set by the reader when the requested port is gone and the GUI
        # should go back to the port picker for a rescan.
        self.picker_requested = False
        # Per-wheel (t, vel, tgt) strip-chart history
        self.history = [
            collections.deque(maxlen=600) for _ in range(proto.WHEEL_COUNT)
        ]
        self.history_lock = threading.Lock()
        # Command routing (real serial via reader, emulated in demo mode)
        self.reader = None
        self.scanner = None
        # Dashboard-side controls
        self.active_tab = config.TAB_MOTORS
        self.selected_wheel = 0
        self.sweep = False
        self.sweep_high = False
        self.last_sweep_t = 0.0
        self.manual_targets = [0.0] * proto.WHEEL_COUNT
        self.control_sequence = 0

    # ------------------------------------------------------------------
    # Frame updates (reader thread / demo source)
    # ------------------------------------------------------------------

    def update_state(self, data):
        now = time.monotonic()
        self.uptime_ms = data["uptime_ms"]
        for i, wheel in enumerate(data["wheels"]):
            self.wheels[i]["pwm"] = wheel["pwm"]
            self.wheels[i]["vel"] = wheel["vel_rpm"]
            self.wheels[i]["tgt"] = wheel["tgt_rpm"]
        self.valid_mask = data["valid_mask"]
        self.yaw = data["yaw"]
        self.pitch = data["pitch"]
        self.roll = data["roll"]
        self.last_state_t = now
        self.state_count += 1
        with self.history_lock:
            for i, wheel in enumerate(self.wheels):
                self.history[i].append((now, wheel["vel"], wheel["tgt"]))

    def update_imu(self, data):
        self.accel = data["accel"]
        self.gyro = data["gyro"]
        self.mag = data["mag"]
        self.last_imu_t = time.monotonic()
        self.imu_count += 1

    def update_tof(self, data):
        self.tof = data["distances"]
        self.tof_seq_seen = data["sequence"]
        self.last_tof_t = time.monotonic()
        self.tof_count += 1

    def update_health(self, data):
        self.health = data
        self.last_health_t = time.monotonic()
        self.health_count += 1

    def add_console(self, line):
        was_full = len(self.console) == self.console.maxlen
        self.console.append(line)
        if self.console_offset > 0 and not was_full:
            # Keep a scrolled-back view anchored while history grows.
            self.console_offset += 1
        # Echo every console line to the terminal, like a serial monitor.
        print(line, flush=True)

    # ------------------------------------------------------------------
    # Commands (GUI thread)
    # ------------------------------------------------------------------

    def set_targets(self, targets_rpm):
        """Set all four wheel targets in RPM (or emulate in demo mode)."""
        self.add_console("> targets " + " ".join(f"{t:.0f}" for t in targets_rpm) + " RPM")
        if self.demo:
            self.manual_targets = [float(t) for t in targets_rpm]
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        message = proto.encode_control(
            proto.OP_SET_TARGETS, self.control_sequence, [t * 10 for t in targets_rpm]
        )
        self.reader.send_frame(proto.MSG_CONTROL, message)

    def stop_motors(self):
        """Zero every wheel target (or emulate in demo mode)."""
        self.add_console("> stop motors")
        if self.demo:
            self.manual_targets = [0.0] * proto.WHEEL_COUNT
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_MOTORS, self.control_sequence)
        )

    def set_selected_target(self, rpm):
        """Set the selected wheel target, keeping the other wheels as they are."""
        targets = list(self.manual_targets) if self.demo else [w["tgt"] for w in self.wheels]
        targets[self.selected_wheel] = rpm
        self.set_targets(targets)

    def update_sweep(self, now):
        """Dashboard-driven +/-1000 RPM square sweep on the selected wheel."""
        if not self.sweep:
            return
        if now - self.last_sweep_t < 1.0:
            return
        self.last_sweep_t = now
        self.sweep_high = not self.sweep_high
        rpm = config.SWEEP_RPM if self.sweep_high else -config.SWEEP_RPM
        self.set_selected_target(rpm)
