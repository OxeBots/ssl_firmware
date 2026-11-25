"""Thread-shared snapshot of the latest serial debug telemetry."""

import collections
import math
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
            {"pwm": 0.0, "vel": 0.0, "tgt": 0.0, "angle": 0.0, "pulse_rpm": 0.0, "pulse_count": 0}
            for _ in range(proto.WHEEL_COUNT)
        ]
        self.valid_mask = 0
        # Bit i: wheel i has a baseline for the digital FG pulse speed.
        self.pulse_valid_mask = 0
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
        self.state_uptime_ms = 0
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
        self.wheel_tuning_reply = None
        # Calibrated wheel-speed limit in RPM (from the control calibration).
        # Before the first tuning frame the old constant is the fallback.
        self.max_rpm = proto.WHEEL_RPM_MAX
        self.max_rpm_calibrated = False
        self.wheel_max_rpm = [0.0] * proto.WHEEL_COUNT
        # Wheel position control frame and the latest move request
        self.position = None
        self.position_request = None
        # Raw encoder/motor snapshot (~10 Hz) and per-wheel plot history
        self.raw = None
        self.raw_history = [collections.deque(maxlen=600) for _ in range(proto.WHEEL_COUNT)]
        # Wheel whose encoder the mux is focused on for the raw I2C read.
        self.raw_encoder_wheel = None
        self._raw_encoder_sent = None
        # Demo source: start time of the simulated runs, None while idle
        self.demo_calibration_start = None
        self.demo_tuning_start = None
        self.demo_wheel_tuning = {}
        self.demo_position = None
        # Bookkeeping
        self.last_link_t = 0.0
        self.last_state_t = 0.0
        self.last_imu_t = 0.0
        self.last_tof_t = 0.0
        self.last_health_t = 0.0
        self.last_calibration_t = 0.0
        self.last_tuning_t = 0.0
        self.last_position_t = 0.0
        self.last_raw_t = 0.0
        self.state_count = 0
        self.imu_count = 0
        self.tof_count = 0
        self.health_count = 0
        self.calibration_count = 0
        self.tuning_count = 0
        self.position_count = 0
        self.raw_count = 0
        self.rx_bytes = 0  # raw serial bytes received since connect
        self.console = collections.deque(maxlen=500)
        # Absolute index of the next console line; lets readers pick up only
        # the lines that arrived since their last poll.
        self.console_total = 0
        # Console view: 0 = follow the latest lines, >0 = lines scrolled back
        self.console_offset = 0
        # Console text selection: {"anchor": (line, char), "focus": (line, char)}
        # with line = index into the full console history
        self.console_sel = None
        # Set by the reader when the requested port is gone and the GUI
        # should go back to the port picker for a rescan.
        self.picker_requested = False
        # Per-wheel (t, vel, tgt) strip-chart history
        self.history = [collections.deque(maxlen=int(config.CHART_WINDOW_S * 100)) for _ in range(proto.WHEEL_COUNT)]
        self.orientation_history = collections.deque(maxlen=600)
        self.imu_history = collections.deque(maxlen=600)
        self.history_lock = threading.Lock()
        # Command routing (real serial via reader, emulated in demo mode)
        self.reader = None
        self.scanner = None
        # Dashboard-side controls
        self.active_tab = config.TAB_MOTORS
        self.stream_mask = 0
        self.selected_wheel = 0
        # Arrow-key target step in RPM (configurable in the Controle panel).
        self.arrow_step_rpm = 10.0
        self.sweep = False
        self.sweep_high = False
        self.last_sweep_t = 0.0
        self.manual_targets = [0.0] * proto.WHEEL_COUNT
        self.control_sequence = 0
        # Bench override: let wheel targets through the health motion gate

    # ------------------------------------------------------------------
    # Frame updates (reader thread / demo source)
    # ------------------------------------------------------------------

    def update_state(self, data):
        now = time.monotonic()
        self.state_uptime_ms = self.uptime_ms = data["uptime_ms"]
        for i, wheel in enumerate(data["wheels"]):
            self.wheels[i]["pwm"] = wheel["pwm"]
            self.wheels[i]["vel"] = wheel["vel_rpm"]
            self.wheels[i]["tgt"] = wheel["tgt_rpm"]
            self.wheels[i]["angle"] = wheel["angle_deg"]
            self.wheels[i]["pulse_rpm"] = wheel["pulse_rpm"]
            self.wheels[i]["pulse_count"] = wheel["pulse_count"]
        self.valid_mask = data["valid_mask"]
        self.pulse_valid_mask = data["pulse_valid_mask"]
        self.last_state_t = now
        self.state_count += 1
        with self.history_lock:
            for i, wheel in enumerate(self.wheels):
                # A wheel without a pulse baseline plots a gap instead of a stale speed.
                pulse = wheel["pulse_rpm"] if self.pulse_valid_mask & (1 << i) else float("nan")
                self.history[i].append((now, wheel["vel"], wheel["tgt"], pulse))

    def update_link(self, data):
        self.uptime_ms = data["uptime_ms"]
        self.last_link_t = time.monotonic()

    def update_imu(self, data):
        now = time.monotonic()
        self.accel = data["accel"]
        self.gyro = data["gyro"]
        self.mag = data["mag"]
        self.yaw = data["yaw"]
        self.pitch = data["pitch"]
        self.roll = data["roll"]
        self.last_imu_t = now
        self.imu_count += 1
        with self.history_lock:
            self.imu_history.append((now, tuple(self.accel), tuple(self.gyro), tuple(self.mag)))
            self.orientation_history.append((now, self.yaw, self.pitch, self.roll))

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
        with self.history_lock:
            self.last_link_t = 0.0
            self.last_state_t = 0.0
            self.last_imu_t = 0.0
            self.last_tof_t = 0.0
            self.last_health_t = 0.0
            self.last_calibration_t = 0.0
            self.last_tuning_t = 0.0
            self.last_position_t = 0.0
            self.last_raw_t = 0.0
            self.health = {}
            self.calibration = None
            self.tuning = None
            self.wheel_tuning_reply = None
            self.position = None
            self.pulse_valid_mask = 0
            for wheel in self.wheels:
                wheel["pulse_rpm"] = 0.0
                wheel["pulse_count"] = 0
            self.max_rpm = proto.WHEEL_RPM_MAX
            self.max_rpm_calibrated = False
            self.wheel_max_rpm = [0.0] * proto.WHEEL_COUNT
            self.raw = None
            self.raw_encoder_wheel = None
            self._raw_encoder_sent = None
            for history in self.raw_history:
                history.clear()

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
        # An uncalibrated firmware reports the drivetrain fallback as its live
        # limit (it never sends 0 for the limit itself), so the fallback value
        # also means "not calibrated".
        if 0.0 < data["max_rpm"] < proto.WHEEL_RPM_MAX - 0.05:
            self.max_rpm = data["max_rpm"]
            self.max_rpm_calibrated = True
            self.wheel_max_rpm = list(data["wheel_max_rpm"])
        else:
            # No stored limit reported: keep the fallback until a calibration runs.
            self.max_rpm = proto.WHEEL_RPM_MAX
            self.max_rpm_calibrated = False
            self.wheel_max_rpm = [0.0] * proto.WHEEL_COUNT

    def update_position(self, data):
        self.position = data
        self.last_position_t = time.monotonic()
        self.position_count += 1

    def update_raw(self, data):
        now = time.monotonic()
        self.raw = data
        self.last_raw_t = now
        self.raw_count += 1
        with self.history_lock:
            for index in range(proto.WHEEL_COUNT):
                i2c = data["i2c_raw"][index]
                # The 16-bit sentinel means "no I2C reading": plot a gap instead.
                i2c_value = float("nan") if i2c == 0xFFFF else float(i2c)
                self.raw_history[index].append(
                    (
                        now,
                        data["analog_raw"][index],
                        i2c_value,
                        data["pulse_delta"][index],
                        data["ekf_rpm"][index],
                        data["applied_pwm"][index],
                    )
                )

    def raw_live(self):
        """True while the latest raw frame is recent enough to display."""
        return (
            bool(self.last_raw_t)
            and time.monotonic() - self.last_raw_t <= config.RAW_STALE_S
        )

    def state_age(self):
        """Seconds since the last state frame (infinite when none arrived)."""
        if not self.last_state_t:
            return float("inf")
        return time.monotonic() - self.last_state_t

    def add_console(self, line):
        self.console_total += 1
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
        """Set all four wheel targets in RPM (or emulate in demo mode); returns False when ignored."""
        if self.calibration_running():
            self.add_console("> alvos ignorados: calibração dos motores em andamento")
            return False
        if self.tuning_running():
            self.add_console("> alvos ignorados: calibração do controle em andamento")
            return False
        self._clear_demo_position()
        limit = self.max_rpm
        clamped = [max(-limit, min(limit, float(t))) for t in targets_rpm]
        if any(abs(float(t) - c) > 0.05 for t, c in zip(targets_rpm, clamped)):
            self.add_console(f"> alvos limitados ao máximo calibrado ±{limit:.0f} RPM")
        targets_rpm = clamped
        self.add_console("> alvos " + " ".join(f"{t:.0f}" for t in targets_rpm) + " RPM")
        if self.demo:
            self.manual_targets = [float(t) for t in targets_rpm]
            return True
        if self.reader is None:
            self.add_console("> alvos não enviados: sem link serial")
            return False
        self.control_sequence += 1
        message = proto.encode_control(proto.OP_SET_TARGETS, self.control_sequence, [t * 10 for t in targets_rpm])
        self.reader.send_frame(proto.MSG_CONTROL, message)
        return True

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

    def select_raw_encoder(self, wheel):
        """Focus the raw I2C read on one wheel's encoder (None releases it).

        The firmware only reads the focused mux channel; the selection is
        re-sent whenever the link comes back or the raw view is reopened.
        """
        if wheel is not None:
            try:
                wheel = int(wheel)
            except (TypeError, ValueError):
                wheel = None
            if wheel is not None and not 0 <= wheel < proto.WHEEL_COUNT:
                wheel = None
        self.raw_encoder_wheel = wheel
        if self.demo or self.reader is None or not self._live_link():
            self._raw_encoder_sent = None
            return
        if wheel == self._raw_encoder_sent:
            return
        self.control_sequence += 1
        selector = -1 if wheel is None else wheel
        sent = self.reader.send_frame(
            proto.MSG_CONTROL,
            proto.encode_control(
                proto.OP_SELECT_RAW_ENCODER, self.control_sequence, [selector, 0, 0, 0]
            ),
        )
        if sent:
            self._raw_encoder_sent = wheel

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
        if not self._live_link():
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

    def link_fresh(self, now=None):
        """The heartbeat or a motor sample is recent."""
        last_frame_t = max(self.last_link_t, self.last_state_t)
        if now is None:
            now = time.monotonic()
        return bool(last_frame_t and now - last_frame_t <= config.SENSOR_STALE_S)

    def _live_link(self, now=None):
        """True when the dashboard has a live telemetry link (or demo source)."""
        return self.demo or bool(self.connected and self.streaming and self.link_fresh(now))

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

    def request_wheel_tuning(self, wheel, sequence, fields=0, values=None):
        """Read or patch the selected wheel's current controller values."""
        if not self._live_link() or (self.reader is None and not self.demo):
            return False
        try:
            payload = proto.encode_wheel_tuning(wheel, sequence, fields, values)
        except OverflowError:
            return False
        if self.demo:
            profile = self.demo_wheel_tuning.setdefault(
                wheel, [1.0, 12.0, 0.01] + [0.0, 30.0, 175.0, 8.0, 240.0, 0.0, 0.0, 0.1, 0.02] * 2
                + [0.0, 0.0],
            )
            for index in range(proto.WHEEL_TUNING_FIELDS):
                if fields & (1 << index):
                    profile[index] = values[index]
            rpm = self.manual_targets[wheel]
            gate = profile[6 if rpm >= 0.0 else 15] or 12.0
            mode = 0 if not rpm else (2 if abs(rpm) <= gate * 30.0 / math.pi else 1)
            self.wheel_tuning_reply = {
                "wheel": wheel,
                "status": proto.WHEEL_TUNING_OK,
                "mode": mode,
                "sequence": sequence,
                "fields": fields,
                "values": profile.copy(),
            }
        else:
            self.reader.send_frame(proto.MSG_WHEEL_TUNING, payload)
        return True

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

    def adjust_selected_target(self, fraction):
        """Move the selected wheel target by a fraction of the calibrated limit."""
        current = (
            self.manual_targets[self.selected_wheel] if self.demo else self.wheels[self.selected_wheel]["tgt"]
        )
        self.set_selected_target(current + fraction * self.max_rpm)

    def adjust_selected_target_by_rpm(self, delta_rpm):
        """Move the selected wheel target by an absolute RPM step."""
        current = (
            self.manual_targets[self.selected_wheel] if self.demo else self.wheels[self.selected_wheel]["tgt"]
        )
        self.set_selected_target(current + float(delta_rpm))

    def update_sweep(self, now):
        """Dashboard-driven square sweep between +/- the calibrated limit."""
        if not self.sweep:
            return
        if now - self.last_sweep_t < 1.0:
            return
        self.last_sweep_t = now
        self.sweep_high = not self.sweep_high
        rpm = self.max_rpm if self.sweep_high else -self.max_rpm
        self.set_selected_target(rpm)
