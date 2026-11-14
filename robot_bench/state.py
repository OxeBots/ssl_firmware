"""Thread-shared snapshot of the latest serial debug telemetry."""

import collections
import threading
import time

from . import config
from . import protocol as proto

WHEEL_NAMES = ("M1 · Dianteira esq.", "M2 · Traseira esq.", "M3 · Traseira dir.", "M4 · Dianteira dir.")


class BenchState:
    """Thread-shared snapshot of the latest serial debug telemetry."""

    def __init__(self):
        self.demo = False
        self.quit = False
        self.connected = None  # port name, or None while disconnected
        self.streaming = False
        # Per-wheel control data
        self.wheels = [
            {"pwm": 0.0, "vel": 0.0, "tgt": 0.0, "angle": 0.0} for _ in range(proto.WHEEL_COUNT)
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
        self.tof_status = [0] * (config.TOF_SIDE * config.TOF_SIDE)
        self.tof_seq_seen = -1
        # Motor/encoder wiring calibration frame
        self.calibration = None
        self.calibration_confirm = False
        # Wheel control calibration frame (PID + feedforward per wheel)
        self.tuning = None
        self.tuning_confirm = False
        # Wheel position control frame and the latest move request
        self.position = None
        self.position_request = None
        # Demo source: start time of the simulated runs, None while idle
        self.demo_calibration_start = None
        self.demo_tuning_start = None
        self.demo_position = None
        # Bookkeeping
        self.last_state_t = 0.0
        self.last_imu_t = 0.0
        self.last_tof_t = 0.0
        self.last_health_t = 0.0
        self.last_calibration_t = 0.0
        self.last_tuning_t = 0.0
        self.last_position_t = 0.0
        self.state_count = 0
        self.imu_count = 0
        self.tof_count = 0
        self.health_count = 0
        self.calibration_count = 0
        self.tuning_count = 0
        self.position_count = 0
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
        self.history = [collections.deque(maxlen=600) for _ in range(proto.WHEEL_COUNT)]
        self.orientation_history = collections.deque(maxlen=600)
        self.imu_history = collections.deque(maxlen=600)
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
        # Bench override: let wheel targets through the health motion gate
        self.motion_override = False

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
            self.wheels[i]["angle"] = wheel["angle_deg"]
        self.valid_mask = data["valid_mask"]
        self.yaw = data["yaw"]
        self.pitch = data["pitch"]
        self.roll = data["roll"]
        self.last_state_t = now
        self.state_count += 1
        with self.history_lock:
            for i, wheel in enumerate(self.wheels):
                self.history[i].append((now, wheel["vel"], wheel["tgt"]))
            self.orientation_history.append((now, self.yaw, self.pitch, self.roll))

    def update_imu(self, data):
        now = time.monotonic()
        self.accel = data["accel"]
        self.gyro = data["gyro"]
        self.mag = data["mag"]
        self.last_imu_t = now
        self.imu_count += 1
        with self.history_lock:
            self.imu_history.append((now, tuple(self.accel), tuple(self.gyro), tuple(self.mag)))

    def update_tof(self, data):
        with self.history_lock:
            self.tof = list(data["distances"])
            self.tof_status = list(data["statuses"])
            self.tof_seq_seen = data["sequence"]
            self.last_tof_t = time.monotonic()
            self.tof_count += 1

    def clear_freshness(self):
        """A closed or rebooted link cannot supply a current robot state."""
        self.streaming = False
        if self.motion_override:
            # The firmware drops the override when the streaming session ends.
            self.motion_override = False
            self.add_console("liberação de movimento da bancada removida ao encerrar a conexão")
        with self.history_lock:
            self.last_state_t = 0.0
            self.last_imu_t = 0.0
            self.last_tof_t = 0.0
            self.last_health_t = 0.0
            self.last_calibration_t = 0.0
            self.last_tuning_t = 0.0
            self.last_position_t = 0.0
            self.health = {}
            self.calibration = None
            self.tuning = None
            self.position = None

    def update_health(self, data):
        self.health = data
        self.last_health_t = time.monotonic()
        self.health_count += 1

    def update_calibration(self, data):
        self.calibration = data
        self.last_calibration_t = time.monotonic()
        self.calibration_count += 1

    def update_tuning(self, data):
        self.tuning = data
        self.last_tuning_t = time.monotonic()
        self.tuning_count += 1

    def update_position(self, data):
        self.position = data
        self.last_position_t = time.monotonic()
        self.position_count += 1

    def state_age(self):
        """Seconds since the last state frame (infinite when none arrived)."""
        if not self.last_state_t:
            return float("inf")
        return time.monotonic() - self.last_state_t

    def add_console(self, line):
        was_full = len(self.console) == self.console.maxlen
        self.console.append(line)
        if self.console_offset > 0:
            # Keep a scrolled-back view anchored as messages arrive.
            self.console_offset += 1
        if was_full and self.console_sel:
            if min(self.console_sel["anchor"][0], self.console_sel["focus"][0]) == 0:
                self.console_sel = None
            else:
                for end in ("anchor", "focus"):
                    line_index, char_index = self.console_sel[end]
                    self.console_sel[end] = (line_index - 1, char_index)
        # Echo every console line to the terminal, like a serial monitor.
        print(line, flush=True)

    # ------------------------------------------------------------------
    # Commands (GUI thread)
    # ------------------------------------------------------------------

    def set_targets(self, targets_rpm):
        """Set all four wheel targets in RPM (or emulate in demo mode)."""
        if self.calibration_running():
            self.add_console("> alvos ignorados: calibração dos motores em andamento")
            return
        if self.tuning_running():
            self.add_console("> alvos ignorados: calibração do controle em andamento")
            return
        self._clear_demo_position()
        targets_rpm = [max(-proto.WHEEL_RPM_MAX, min(proto.WHEEL_RPM_MAX, t)) for t in targets_rpm]
        self.add_console("> alvos " + " ".join(f"{t:.0f}" for t in targets_rpm) + " RPM")
        if self.demo:
            self.manual_targets = [float(t) for t in targets_rpm]
            return
        if self.reader is None:
            return
        if self.motion_override:
            # Re-arm: the firmware clears the override when a session ends.
            self._send_motion_override()
        self.control_sequence += 1
        message = proto.encode_control(proto.OP_SET_TARGETS, self.control_sequence, [t * 10 for t in targets_rpm])
        self.reader.send_frame(proto.MSG_CONTROL, message)

    def stop_motors(self):
        """Zero every wheel target (or emulate in demo mode)."""
        self.sweep = False
        self._clear_demo_position()
        self.add_console("> parar motores")
        if self.demo:
            self.manual_targets = [0.0] * proto.WHEEL_COUNT
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        self.reader.send_frame(proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_MOTORS, self.control_sequence))

    def set_motion_override(self, enabled):
        """Let dashboard wheel targets through the health motion gate."""
        enabled = bool(enabled)
        if enabled == self.motion_override:
            return
        self.motion_override = enabled
        self.add_console("> liberação de movimento da bancada " + ("ativada" if enabled else "desativada"))
        if self.demo:
            return
        self._send_motion_override()
        if not enabled:
            self.stop_motors()

    def _send_motion_override(self):
        """Re-arm the firmware override; it is dropped when the session ends."""
        if self.reader is None:
            return
        self.control_sequence += 1
        if self.motion_override:
            op = proto.OP_ENABLE_MOTION_OVERRIDE
        else:
            op = proto.OP_DISABLE_MOTION_OVERRIDE
        self.reader.send_frame(proto.MSG_CONTROL, proto.encode_control(op, self.control_sequence))

    def set_calibration_confirm(self, value):
        """Arm the safety confirmation required before starting a calibration."""
        self.calibration_confirm = bool(value)

    def calibration_live(self):
        """True while the latest calibration frame is recent enough to use."""
        return (
            bool(self.last_calibration_t)
            and time.monotonic() - self.last_calibration_t <= config.CALIBRATION_STALE_S
        )

    def calibration_running(self):
        """True while a fresh calibration frame reports a running routine."""
        return self.calibration_live() and self.calibration["state"] == proto.CALIBRATION_RUNNING

    def start_calibration(self):
        """Start the motor wiring calibration; returns False when not allowed."""
        if not self.calibration_confirm:
            self.add_console("> calibração não iniciada: confirme que o robô está levantado")
            return False
        live_link = (
            self.connected
            and self.streaming
            and self.last_state_t
            and time.monotonic() - self.last_state_t <= config.SENSOR_STALE_S
        )
        if not self.demo and not live_link:
            self.add_console("> calibração não iniciada: link serial sem telemetria")
            return False
        if self.calibration_running():
            return False
        self.sweep = False
        self.add_console("> iniciar calibração dos motores")
        if self.demo:
            self.demo_calibration_start = time.monotonic()
            return True
        if self.reader is None:
            return False
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_CONTROL,
            proto.encode_control(proto.OP_START_CALIBRATION, self.control_sequence),
        )
        return True

    def stop_calibration(self):
        """Abort the motor wiring calibration."""
        self.add_console("> cancelar calibração dos motores")
        if self.demo:
            self.demo_calibration_start = None
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_CONTROL,
            proto.encode_control(proto.OP_STOP_CALIBRATION, self.control_sequence),
        )

    def _live_link(self):
        """True when the dashboard has a live telemetry link (or demo source)."""
        if self.demo:
            return True
        return bool(
            self.connected
            and self.streaming
            and self.last_state_t
            and time.monotonic() - self.last_state_t <= config.SENSOR_STALE_S
        )

    def _clear_demo_position(self):
        """Cancel the simulated position move (the firmware cancels its own)."""
        if self.demo_position is not None:
            self.demo_position = None
            self.add_console("> movimento de posição cancelado")

    def tuning_live(self):
        """True while the latest control-calibration frame is recent enough to use."""
        return (
            bool(self.last_tuning_t)
            and time.monotonic() - self.last_tuning_t <= config.TUNING_STALE_S
        )

    def tuning_running(self):
        """True while a fresh control-calibration frame reports a running routine."""
        return self.tuning_live() and self.tuning["state"] == proto.TUNING_RUNNING

    def set_tuning_confirm(self, value):
        """Arm the safety confirmation required before starting the control calibration."""
        self.tuning_confirm = bool(value)

    def start_tuning(self):
        """Start the wheel control calibration; returns False when not allowed."""
        if not self.tuning_confirm:
            self.add_console("> calibração do controle não iniciada: confirme que o robô está levantado")
            return False
        if not self._live_link():
            self.add_console("> calibração do controle não iniciada: link serial sem telemetria")
            return False
        if self.tuning_running():
            return False
        if self.calibration_running():
            self.add_console("> calibração do controle não iniciada: calibração dos motores em andamento")
            return False
        self.sweep = False
        self._clear_demo_position()
        self.add_console("> iniciar calibração do controle")
        if self.demo:
            self.demo_tuning_start = time.monotonic()
            return True
        if self.reader is None:
            return False
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_CONTROL,
            proto.encode_control(proto.OP_START_TUNING, self.control_sequence),
        )
        return True

    def stop_tuning(self):
        """Abort the wheel control calibration."""
        self.add_console("> cancelar calibração do controle")
        if self.demo:
            self.demo_tuning_start = None
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_CONTROL,
            proto.encode_control(proto.OP_STOP_TUNING, self.control_sequence),
        )

    def position_live(self):
        """True while the latest position frame is recent enough to use."""
        return (
            bool(self.last_position_t)
            and time.monotonic() - self.last_position_t <= config.POSITION_STALE_S
        )

    def start_position_move(self, angles_deg, wheel_mask, absolute, max_rpm=0):
        """Request a wheel position move; returns False when not allowed."""
        if not self._live_link():
            self.add_console("> movimento de posição não iniciado: link serial sem telemetria")
            return False
        if self.calibration_running() or self.tuning_running():
            self.add_console("> movimento de posição não iniciado: calibração em andamento")
            return False
        if not wheel_mask:
            return False
        if not self.demo and (self.valid_mask & wheel_mask) != wheel_mask:
            self.add_console("> movimento de posição não iniciado: encoder de alguma roda não está calibrado")
            return False
        angles = [float(angle) for angle in angles_deg]
        self.sweep = False
        self.position_request = {
            "angles_deg": angles,
            "wheel_mask": int(wheel_mask),
            "absolute": bool(absolute),
            "max_rpm": int(max_rpm),
        }
        mode = proto.POSITION_MODE_ABSOLUTE if absolute else proto.POSITION_MODE_RELATIVE
        mode_text = "absoluto" if absolute else "relativo"
        self.add_console(
            "> posição "
            + " ".join(
                f"M{i + 1}={angles[i]:+.1f}°" for i in range(proto.WHEEL_COUNT) if wheel_mask & (1 << i)
            )
            + f" ({mode_text})"
        )
        if self.demo:
            self.manual_targets = [0.0] * proto.WHEEL_COUNT
            self.demo_position = dict(self.position_request)
            return True
        if self.reader is None:
            return False
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_POSITION_CONTROL,
            proto.encode_position_control(
                proto.POSITION_OP_MOVE, mode, wheel_mask, angles, max_rpm, self.control_sequence
            ),
        )
        return True

    def stop_position(self):
        """Cancel the wheel position moves and zero the wheels."""
        self.add_console("> parar posição")
        self._clear_demo_position()
        if self.demo:
            self.manual_targets = [0.0] * proto.WHEEL_COUNT
            return
        if self.reader is None:
            return
        self.control_sequence += 1
        self.reader.send_frame(
            proto.MSG_POSITION_CONTROL,
            proto.encode_position_control(
                proto.POSITION_OP_STOP, 0, 0, [0.0] * proto.WHEEL_COUNT, 0, self.control_sequence
            ),
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
