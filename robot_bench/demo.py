"""Synthetic debug telemetry so the GUI works without hardware."""

import math
import random
import threading
import time

from . import config
from . import protocol as proto

# Simulated wiring calibration: one wheel per 1.8 s, 7.2 s for the whole routine.
DEMO_CALIBRATION_WHEEL_S = 1.8
DEMO_CALIBRATION_TOTAL_S = 4 * DEMO_CALIBRATION_WHEEL_S
# Report DONE for a few seconds so the dashboard can show the stored result.
DEMO_CALIBRATION_DONE_S = 4.0
# Detected wiring, non-identity on purpose: wheel -> analog slot and mux channel.
DEMO_CALIBRATION_ANALOG_MAP = (2, 0, 3, 1)
DEMO_CALIBRATION_MUX_MAP = (1, 3, 0, 2)
# Level magnitudes in the visible band (ADC counts / encoder angle counts).
DEMO_CALIBRATION_LEVEL = (600, 1500)


def calibration_frame(sequence, **fields):
    """DebugCalibration dict with idle defaults, overridden by fields."""
    frame = {
        "state": proto.CALIBRATION_IDLE,
        "wheel": 4,
        "progress": 0,
        "result_mask": 0,
        "i2c_ready": 0,
        "mapping_valid": 0,
        "analog_map": [0, 1, 2, 3],
        "mux_map": [0, 1, 2, 3],
        "analog_level": [0, 0, 0, 0],
        "i2c_level": [0, 0, 0, 0],
        "range_mask": 0,
        "error_code": proto.CALIBRATION_ERROR_NONE,
        "sequence": sequence,
    }
    frame.update(fields)
    return frame


def idle_calibration_frame(sequence, calibrated=False):
    """Idle frame; a finished demo run keeps reporting its stored mapping."""
    if not calibrated:
        return calibration_frame(sequence)
    return calibration_frame(
        sequence,
        mapping_valid=1,
        analog_map=list(DEMO_CALIBRATION_ANALOG_MAP),
        mux_map=list(DEMO_CALIBRATION_MUX_MAP),
        range_mask=0xF,
    )


def done_calibration_frame(sequence):
    """DONE frame carrying the simulated non-identity mapping."""
    return calibration_frame(
        sequence,
        state=proto.CALIBRATION_DONE,
        progress=100,
        result_mask=0xF,
        i2c_ready=0xF,
        mapping_valid=1,
        analog_map=list(DEMO_CALIBRATION_ANALOG_MAP),
        mux_map=list(DEMO_CALIBRATION_MUX_MAP),
        range_mask=0xF,
    )


def running_calibration_frame(elapsed, sequence):
    """One frame of the simulated run; DONE once every wheel was calibrated."""
    if elapsed >= DEMO_CALIBRATION_TOTAL_S:
        return done_calibration_frame(sequence)
    measured = int(elapsed / DEMO_CALIBRATION_WHEEL_S)
    wheel = min(proto.WHEEL_COUNT - 1, measured)
    analog_map = list(range(proto.WHEEL_COUNT))
    mux_map = list(range(proto.WHEEL_COUNT))
    for done_wheel in range(measured):
        analog_map[done_wheel] = DEMO_CALIBRATION_ANALOG_MAP[done_wheel]
        mux_map[done_wheel] = DEMO_CALIBRATION_MUX_MAP[done_wheel]
    analog_level = [0] * proto.WHEEL_COUNT
    i2c_level = [0] * proto.WHEEL_COUNT
    analog_level[DEMO_CALIBRATION_ANALOG_MAP[wheel]] = demo_level(sequence)
    i2c_level[DEMO_CALIBRATION_MUX_MAP[wheel]] = demo_level(sequence + 1)
    completed = (1 << measured) - 1
    return calibration_frame(
        sequence,
        state=proto.CALIBRATION_RUNNING,
        wheel=wheel,
        progress=int(elapsed / DEMO_CALIBRATION_TOTAL_S * 100),
        result_mask=completed,
        i2c_ready=completed,
        analog_map=analog_map,
        mux_map=mux_map,
        analog_level=analog_level,
        i2c_level=i2c_level,
    )


def demo_level(sequence):
    """Detected movement in the visible 600..1500 count band."""
    minimum, maximum = DEMO_CALIBRATION_LEVEL
    return minimum + sequence * 97 % (maximum - minimum)


# Simulated control calibration: one wheel per 2.6 s, 10.4 s for the whole run.
DEMO_TUNING_WHEEL_S = 2.6
DEMO_TUNING_TOTAL_S = 4 * DEMO_TUNING_WHEEL_S
DEMO_TUNING_DONE_S = 4.0
DEMO_TUNING_RPM = 287.0
# Parameters the simulated run converges to, one entry per wheel.
DEMO_TUNED = (
    {"kp": 58.0, "ki": 6.0, "kd": 0.0, "ff": 176.0, "velocity_ff": 54.0, "rise": 280, "overshoot": 2, "ss": 1},
    {"kp": 64.0, "ki": 6.5, "kd": 0.0, "ff": 182.0, "velocity_ff": 56.0, "rise": 240, "overshoot": 3, "ss": 1},
    {"kp": 61.0, "ki": 6.2, "kd": 0.0, "ff": 179.0, "velocity_ff": 55.0, "rise": 260, "overshoot": 2, "ss": 2},
    {"kp": 67.0, "ki": 6.8, "kd": 0.0, "ff": 184.0, "velocity_ff": 57.0, "rise": 230, "overshoot": 4, "ss": 1},
)
DEMO_TUNING_PHASES = (
    proto.TUNING_PHASE_FRICTION,
    proto.TUNING_PHASE_IDENTIFY,
    proto.TUNING_PHASE_VALIDATE,
    proto.TUNING_PHASE_REFINE,
    proto.TUNING_PHASE_STORE,
)


def tuning_parameters(index):
    """One wheel's demo tuning parameters as the GUI stores them."""
    tuned = DEMO_TUNED[index]
    return {
        "kp": tuned["kp"],
        "ki": tuned["ki"],
        "kd": tuned["kd"],
        "feedforward": tuned["ff"],
        "velocity_ff": tuned["velocity_ff"],
        "rise_ms": tuned["rise"],
        "overshoot_pct": tuned["overshoot"],
        "ss_error_pct": tuned["ss"],
    }


def tuning_arrays(**overrides):
    """Per-wheel parameter arrays filled with zero or with demo values."""
    arrays = {
        "kp": [0.0] * proto.WHEEL_COUNT,
        "ki": [0.0] * proto.WHEEL_COUNT,
        "kd": [0.0] * proto.WHEEL_COUNT,
        "feedforward": [0.0] * proto.WHEEL_COUNT,
        "velocity_ff": [0.0] * proto.WHEEL_COUNT,
        "rise_ms": [0] * proto.WHEEL_COUNT,
        "overshoot_pct": [0] * proto.WHEEL_COUNT,
        "ss_error_pct": [0] * proto.WHEEL_COUNT,
    }
    for wheel in range(proto.WHEEL_COUNT):
        for key, value in tuning_parameters(wheel).items():
            arrays[key][wheel] = value
    arrays.update(overrides)
    return arrays


def tuning_frame(sequence, **fields):
    """DebugTuning dict with idle defaults, overridden by fields."""
    frame = {
        "state": proto.TUNING_IDLE,
        "wheel": 4,
        "phase": proto.TUNING_PHASE_IDLE,
        "progress": 0,
        "result_mask": 0,
        "error_code": proto.TUNING_ERROR_NONE,
        **tuning_arrays(
            kp=[0.0] * proto.WHEEL_COUNT,
            ki=[0.0] * proto.WHEEL_COUNT,
            kd=[0.0] * proto.WHEEL_COUNT,
            feedforward=[0.0] * proto.WHEEL_COUNT,
            velocity_ff=[0.0] * proto.WHEEL_COUNT,
            rise_ms=[0] * proto.WHEEL_COUNT,
            overshoot_pct=[0] * proto.WHEEL_COUNT,
            ss_error_pct=[0] * proto.WHEEL_COUNT,
        ),
        "sequence": sequence,
    }
    frame.update(fields)
    return frame


def idle_tuning_frame(sequence, tuned=False):
    """Idle frame; a finished demo run keeps reporting its stored parameters."""
    if not tuned:
        return tuning_frame(sequence)
    return tuning_frame(
        sequence,
        result_mask=0xF,
        **tuning_arrays(),
    )


def done_tuning_frame(sequence):
    """DONE frame carrying the simulated per-wheel parameters."""
    return tuning_frame(
        sequence,
        state=proto.TUNING_DONE,
        progress=100,
        result_mask=0xF,
        **tuning_arrays(),
    )


def running_tuning_frame(elapsed, sequence):
    """One frame of the simulated control calibration run."""
    if elapsed >= DEMO_TUNING_TOTAL_S:
        return done_tuning_frame(sequence)
    measured = int(elapsed / DEMO_TUNING_WHEEL_S)
    wheel = min(proto.WHEEL_COUNT - 1, measured)
    inside = (elapsed - measured * DEMO_TUNING_WHEEL_S) / DEMO_TUNING_WHEEL_S
    phase = DEMO_TUNING_PHASES[min(len(DEMO_TUNING_PHASES) - 1, int(inside * len(DEMO_TUNING_PHASES)))]
    completed = (1 << measured) - 1
    arrays = tuning_arrays()
    for done_wheel in range(measured):
        for key, value in tuning_parameters(done_wheel).items():
            arrays[key][done_wheel] = value
    current = tuning_parameters(wheel)
    # The wheel under calibration shows the candidate values it is validating.
    for key in ("kp", "ki", "kd", "feedforward", "velocity_ff"):
        arrays[key][wheel] = current[key]
    return tuning_frame(
        sequence,
        state=proto.TUNING_RUNNING,
        wheel=wheel,
        phase=phase,
        progress=int(elapsed / DEMO_TUNING_TOTAL_S * 100),
        result_mask=completed,
        **arrays,
    )


# Simulated position-control loop: proportional approach with a small lag.
DEMO_POSITION_GAIN = 4.0
DEMO_POSITION_VELOCITY_LAG_S = 0.08


def position_frame(sequence, **fields):
    """DebugPosition dict with idle defaults, overridden by fields."""
    frame = {
        "active_mask": 0,
        "done_mask": 0,
        "error_code": proto.POSITION_ERROR_NONE,
        "angle_deg": [0.0] * proto.WHEEL_COUNT,
        "target_deg": [0.0] * proto.WHEEL_COUNT,
        "sequence": sequence,
    }
    frame.update(fields)
    return frame


class DemoSource(threading.Thread):
    """Generates plausible debug telemetry so the GUI works without the rig."""

    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.tof_seq = 0
        self.imu_seq = 0
        self.health_seq = 0
        self.calibration_seq = 0
        self.calibration_done = False
        self.calibration_done_until = 0.0
        self.tuning_seq = 0
        self.tuning_done = False
        self.tuning_done_until = 0.0
        self.position_seq = 0
        self.position_request_seen = None
        self.position_active_mask = 0
        self.position_done_mask = 0
        self.position_targets = [0.0] * proto.WHEEL_COUNT
        self.vel = [0.0] * proto.WHEEL_COUNT
        self.angle = [0.0] * proto.WHEEL_COUNT
        self.angle_cont = [0.0] * proto.WHEEL_COUNT

    def run(self):
        rng = random.Random(7)
        next_state = 0.0
        next_imu = 0.0
        next_tof = 0.0
        next_health = 0.0
        next_calibration = 0.0
        next_tuning = 0.0
        next_position = 0.0
        start = time.monotonic()
        last_t = start
        while not self.state.quit:
            now = time.monotonic()
            t = now - start
            dt = now - last_t
            last_t = now
            request = self.state.demo_position
            if request is not None and request is not self.position_request_seen:
                self.position_request_seen = request
                for i in range(proto.WHEEL_COUNT):
                    if request["wheel_mask"] & (1 << i):
                        if request["absolute"]:
                            self.position_targets[i] = request["angles_deg"][i]
                        else:
                            self.position_targets[i] = self.angle_cont[i] + request["angles_deg"][i]
                self.position_active_mask = int(request["wheel_mask"])
                self.position_done_mask = 0
            pwm = [0.0] * proto.WHEEL_COUNT
            if request is not None and self.position_active_mask:
                max_rpm = request["max_rpm"] or config.POSITION_MAX_RPM
                targets = [0.0] * proto.WHEEL_COUNT
                for i in range(proto.WHEEL_COUNT):
                    if not self.position_active_mask & (1 << i):
                        self.vel[i] += (0.0 - self.vel[i]) * min(1.0, dt / 0.1)
                        continue
                    error = self.position_targets[i] - self.angle_cont[i]
                    velocity_target = max(-max_rpm, min(max_rpm, DEMO_POSITION_GAIN * error))
                    self.vel[i] += (velocity_target - self.vel[i]) * min(
                        1.0, dt / DEMO_POSITION_VELOCITY_LAG_S
                    )
                    targets[i] = self.vel[i]
                    bit = 1 << i
                    if abs(error) < 0.5 and abs(self.vel[i]) < 5.0:
                        self.position_done_mask |= bit
                    else:
                        self.position_done_mask &= ~bit
            else:
                self.position_active_mask = 0
                self.position_done_mask = 0
                targets = list(self.state.manual_targets)
                for i in range(proto.WHEEL_COUNT):
                    self.vel[i] += (targets[i] - self.vel[i]) * min(1.0, 0.02 / 0.18)
                    self.vel[i] += rng.uniform(-8.0, 8.0)
            for i in range(proto.WHEEL_COUNT):
                pwm[i] = max(-1023.0, min(1023.0, targets[i] * 0.22 + (targets[i] - self.vel[i]) * 0.45))
                # Wheel angle in degrees: 1 RPM is 6 deg/s at the shaft.
                self.angle_cont[i] += self.vel[i] * 6.0 * dt
                self.angle[i] = self.angle_cont[i] % 360.0
            yaw = 90.0 + 25.0 * math.sin(0.35 * t)
            pitch = 18.0 * math.sin(0.6 * t)
            roll = 12.0 * math.sin(0.45 * t + 1.0)

            if now >= next_state:
                next_state = now + 0.04
                wheels = [
                    {
                        "pwm": pwm[i],
                        "vel_rpm": self.vel[i],
                        "tgt_rpm": targets[i],
                        "angle_deg": self.angle[i],
                    }
                    for i in range(proto.WHEEL_COUNT)
                ]
                self.state.update_state(
                    {
                        "uptime_ms": int(t * 1000),
                        "wheels": wheels,
                        "yaw": yaw,
                        "pitch": pitch,
                        "roll": roll,
                        "valid_mask": 0xF,
                        "sequence": int(t * 25),
                    }
                )
            if now >= next_imu:
                next_imu = now + 0.1
                self.imu_seq += 1
                self.state.update_imu(
                    {
                        "accel": [
                            0.05 * math.sin(0.7 * t),
                            0.04 * math.cos(0.5 * t),
                            0.98 + 0.02 * math.sin(0.9 * t),
                        ],
                        "gyro": [3.0 * math.sin(0.8 * t), 2.0 * math.cos(0.6 * t), 1.5 * math.sin(0.4 * t)],
                        "mag": [220 + 40 * math.sin(0.3 * t), 180 + 30 * math.cos(0.35 * t), 410],
                        "sequence": self.imu_seq,
                    }
                )
            if now >= next_tof:
                next_tof = now + 1.0 / 15.0
                self.tof_seq += 1
                self.state.update_tof(
                    {"sequence": self.tof_seq, "distances": self._demo_tof(t, rng), "statuses": [5] * 64}
                )
            if now >= next_health:
                next_health = now + 0.5
                self.health_seq += 1
                self.state.update_health(
                    {
                        "robot_id": 3,
                        "error_flags": 0,
                        "lifecycle": 3,
                        "active_faults": 0,
                        "latched_faults": 0,
                        "motion_inhibited": 0,
                        "kick_inhibited": 1,
                        "imu_cal_flags": 7,
                        "tof_available": 1,
                        "radio_ready": 1,
                        "base_station_connected": 1,
                        "supply_voltage": 14.6 + 0.05 * math.sin(0.2 * t),
                        "supply_flags": 1,
                        "free_heap": 178000 + int(2000 * math.sin(0.1 * t)),
                        "min_free_heap": 165000,
                        "sequence": self.health_seq,
                    }
                )
            if now >= next_calibration:
                running = self.state.demo_calibration_start is not None
                next_calibration = now + (0.2 if running else 0.5)
                self.calibration_seq += 1
                if running:
                    elapsed = now - self.state.demo_calibration_start
                    self.state.update_calibration(running_calibration_frame(elapsed, self.calibration_seq))
                    if elapsed >= DEMO_CALIBRATION_TOTAL_S:
                        self.calibration_done = True
                        self.calibration_done_until = now + DEMO_CALIBRATION_DONE_S
                        self.state.demo_calibration_start = None
                elif now < self.calibration_done_until:
                    self.state.update_calibration(done_calibration_frame(self.calibration_seq))
                else:
                    self.state.update_calibration(
                        idle_calibration_frame(self.calibration_seq, self.calibration_done)
                    )
            if now >= next_tuning:
                running = self.state.demo_tuning_start is not None
                next_tuning = now + (0.2 if running else 0.5)
                self.tuning_seq += 1
                if running:
                    elapsed = now - self.state.demo_tuning_start
                    self.state.update_tuning(running_tuning_frame(elapsed, self.tuning_seq))
                    if elapsed >= DEMO_TUNING_TOTAL_S:
                        self.tuning_done = True
                        self.tuning_done_until = now + DEMO_TUNING_DONE_S
                        self.state.demo_tuning_start = None
                elif now < self.tuning_done_until:
                    self.state.update_tuning(done_tuning_frame(self.tuning_seq))
                else:
                    self.state.update_tuning(
                        idle_tuning_frame(self.tuning_seq, self.tuning_done)
                    )
            if now >= next_position:
                next_position = now + (0.1 if self.position_active_mask else 0.5)
                self.position_seq += 1
                self.state.update_position(
                    position_frame(
                        self.position_seq,
                        active_mask=self.position_active_mask,
                        done_mask=self.position_done_mask,
                        angle_deg=list(self.angle_cont),
                        target_deg=list(self.position_targets),
                    )
                )
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
