"""Binary frame codec and parsers for the serial debug protocol.

Wire format (see proto/serial_debug_protocol.bitproto): each frame is
COBS(msg_type | bitproto message | crc8) followed by a 0x00 delimiter.
Messages are little-endian, bit-packed contiguously by bitproto.
"""

import struct

# Message type ids (first byte of every frame payload)
MSG_STATE = 1
MSG_IMU = 2
MSG_TOF = 3
MSG_HEALTH = 4
MSG_TEXT = 5
MSG_CONTROL = 6
MSG_CALIBRATION = 7
MSG_TUNING = 8
MSG_POSITION = 9
MSG_POSITION_CONTROL = 10

# Bitproto serialized sizes (BYTES_LENGTH_* in the generated C header)
STATE_SIZE = 47
IMU_SIZE = 22
TOF_SIZE = 196
HEALTH_SIZE = 29
TEXT_SIZE = 97
CONTROL_SIZE = 13
CALIBRATION_SIZE = 36
TUNING_SIZE = 66
POSITION_SIZE = 39
POSITION_CONTROL_SIZE = 25

# DebugControl operation ids
OP_STOP_STREAM = 0
OP_START_STREAM = 1
OP_KEEPALIVE = 2
OP_SET_TARGETS = 3
OP_STOP_MOTORS = 4
# Bench override: wheel targets pass through while the health monitor
# inhibits motion (bare bench without supply sense or encoder calibration).
OP_ENABLE_MOTION_OVERRIDE = 5
OP_DISABLE_MOTION_OVERRIDE = 6

# Motor/encoder wiring calibration: the robot spins one motor at a time in
# open loop, detects the encoder behind each wheel and stores the map in NVS.
OP_START_CALIBRATION = 7
OP_STOP_CALIBRATION = 8

# Wheel control calibration: per-wheel friction/plant identification, gain
# synthesis, closed-loop validation and NVS persistence.
OP_START_TUNING = 9
OP_STOP_TUNING = 10

# Wheel position control operations / modes (DebugPositionControl message)
POSITION_OP_MOVE = 0
POSITION_OP_STOP = 1
POSITION_MODE_RELATIVE = 0
POSITION_MODE_ABSOLUTE = 1

# DebugPosition failure reasons
POSITION_ERROR_NONE = 0
POSITION_ERROR_TIMEOUT = 1
POSITION_ERROR_ABORTED = 2
POSITION_ERROR_ENCODER = 3

# DebugCalibration routine states
CALIBRATION_IDLE = 0
CALIBRATION_RUNNING = 1
CALIBRATION_DONE = 2
CALIBRATION_FAILED = 3

# DebugCalibration failure reasons
CALIBRATION_ERROR_NONE = 0
CALIBRATION_ERROR_NO_MOVEMENT = 1
CALIBRATION_ERROR_AMBIGUOUS_ANALOG = 2
CALIBRATION_ERROR_AMBIGUOUS_I2C = 3
CALIBRATION_ERROR_BAD_MAPPING = 4
CALIBRATION_ERROR_ABORTED = 5
CALIBRATION_ERROR_I2C = 6

# DebugTuning routine states
TUNING_IDLE = 0
TUNING_RUNNING = 1
TUNING_DONE = 2
TUNING_FAILED = 3

# DebugTuning phases
TUNING_PHASE_IDLE = 0
TUNING_PHASE_FRICTION = 1
TUNING_PHASE_IDENTIFY = 2
TUNING_PHASE_VALIDATE = 3
TUNING_PHASE_REFINE = 4
TUNING_PHASE_STORE = 5
TUNING_PHASE_DONE = 6

# DebugTuning failure reasons
TUNING_ERROR_NONE = 0
TUNING_ERROR_NO_MOVEMENT = 1
TUNING_ERROR_UNSTABLE = 2
TUNING_ERROR_TIMEOUT = 3
TUNING_ERROR_ABORTED = 4

TOF_ZONES = 64
WHEEL_COUNT = 4

# Wheel-shaft speed limit: 4000 RPM motor through the 3:1 reduction. The
# int16 0.1 RPM wire range (+/-3276.7 RPM) sits far above this.
WHEEL_RPM_MAX = 4000.0 / 3.0

# Bound partial frames using the largest current message and framing overhead.
_MAX_CHUNK = TOF_SIZE + 4

def crc8(data):
    """CRC-8 (poly 0x07, init 0x00, no reflection) over data."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def cobs_encode(payload):
    """COBS-encode payload (output never contains 0x00)."""
    out = bytearray()
    code_index = 0
    code = 1
    out.append(0)  # placeholder for the first code byte
    for byte in payload:
        if byte:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = 0xFF
                code = 1
                code_index = len(out)
                out.append(0)
        else:
            out[code_index] = code
            code = 1
            code_index = len(out)
            out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    """COBS-decode one frame; returns bytes or None when malformed."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            return None
        if i + code - 1 > n:
            return None
        out += data[i : i + code - 1]
        i += code - 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def encode_frame(msg_type, message):
    """Build one wire frame: COBS(type | message | crc8) + 0x00."""
    payload = bytes([msg_type]) + bytes(message)
    payload += bytes([crc8(payload)])
    return cobs_encode(payload) + b"\x00"


def encode_control(op, sequence=0, wheel_targets=(0, 0, 0, 0)):
    """Build a DebugControl message (dashboard -> firmware)."""
    return struct.pack("<BI4h", op, sequence, *[int(t) for t in wheel_targets])


class FrameDecoder:
    """Streaming decoder: feed raw serial bytes, receive decoded frames."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        """Feed bytes; return a list of (msg_type, message) frames."""
        self._buffer += data
        frames = []
        while True:
            index = self._buffer.find(0)
            if index < 0:
                if len(self._buffer) > _MAX_CHUNK:
                    self._buffer.clear()
                break
            chunk = bytes(self._buffer[:index])
            del self._buffer[: index + 1]
            if chunk:
                frame = _decode_frame(chunk)
                if frame is not None:
                    frames.append(frame)
        return frames


def _decode_frame(chunk):
    """Decode one COBS chunk; return (msg_type, message) or None."""
    payload = cobs_decode(chunk)
    if payload is None or len(payload) < 3:
        return None
    if crc8(payload[:-1]) != payload[-1]:
        return None
    return payload[0], payload[1:-1]


def parse_state(message):
    """DebugState -> dict: wheels, attitude, uptime, encoder mask, sequence."""
    if len(message) != STATE_SIZE:
        return None
    uptime_ms = struct.unpack_from("<I", message, 0)[0]
    wheels = [struct.unpack_from("<3hH", message, 4 + i * 8) for i in range(WHEEL_COUNT)]
    yaw, pitch, roll = struct.unpack_from("<3h", message, 36)
    valid_mask = message[42]
    sequence = struct.unpack_from("<I", message, 43)[0]
    return {
        "uptime_ms": uptime_ms,
        "wheels": [
            {
                "pwm": w[0],
                "vel_rpm": w[1] / 10.0,
                "tgt_rpm": w[2] / 10.0,
                "angle_deg": w[3] / 100.0,
            }
            for w in wheels
        ],
        "yaw": yaw / 100.0,
        "pitch": pitch / 100.0,
        "roll": roll / 100.0,
        "valid_mask": valid_mask,
        "sequence": sequence,
    }


def parse_imu(message):
    """DebugImu -> dict: acceleration in g, rotation in deg/s, field in mG."""
    if len(message) != IMU_SIZE:
        return None
    values = struct.unpack("<9hI", message)
    return {
        "accel": [values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0],
        "gyro": [values[3] / 10.0, values[4] / 10.0, values[5] / 10.0],
        "mag": [float(values[6]), float(values[7]), float(values[8])],
        "sequence": values[9],
    }


def parse_tof(message):
    """DebugTof -> dict: unfiltered distances and their sensor status codes."""
    if len(message) != TOF_SIZE:
        return None
    sequence = struct.unpack_from("<I", message, 0)[0]
    distances = list(struct.unpack_from("<64h", message, 4))
    statuses = list(message[132:196])
    return {"sequence": sequence, "distances": distances, "statuses": statuses}


def parse_health(message):
    """DebugHealth -> dict of safety, power and system status."""
    if len(message) != HEALTH_SIZE:
        return None
    (robot_id, error_flags, lifecycle, active_faults, latched_faults, motion, kick,
     imu_cal, tof, radio_ready, base_station_connected, supply, supply_flags,
     free_heap, min_free_heap, sequence) = struct.unpack(
        "<BHBHHBBBBBBHBIII", message
    )
    return {
        "robot_id": robot_id,
        "error_flags": error_flags,
        "lifecycle": lifecycle,
        "active_faults": active_faults,
        "latched_faults": latched_faults,
        "motion_inhibited": motion,
        "kick_inhibited": kick,
        "imu_cal_flags": imu_cal,
        "tof_available": tof,
        "radio_ready": radio_ready,
        "base_station_connected": base_station_connected,
        "supply_voltage": supply / 100.0,
        "supply_flags": supply_flags,
        "free_heap": free_heap,
        "min_free_heap": min_free_heap,
        "sequence": sequence,
    }


def parse_text(message):
    """DebugText -> one ASCII log line."""
    if len(message) != TEXT_SIZE:
        return None
    # Byte 0 is the text length; the ASCII text follows it.
    return message[1 : 1 + message[0]].decode("ascii", errors="replace")


def parse_calibration(message):
    """DebugCalibration -> dict: routine state, detected wiring and live levels."""
    if len(message) != CALIBRATION_SIZE:
        return None
    values = struct.unpack("<6B4B4B4h4hBBI", message)
    return {
        "state": values[0],
        "wheel": values[1],
        "progress": values[2],
        "result_mask": values[3],
        "i2c_ready": values[4],
        "mapping_valid": values[5],
        "analog_map": list(values[6:10]),
        "mux_map": list(values[10:14]),
        "analog_level": list(values[14:18]),
        "i2c_level": list(values[18:22]),
        "range_mask": values[22],
        "error_code": values[23],
        "sequence": values[24],
    }


def parse_tuning(message):
    """DebugTuning -> dict: routine state, per-wheel parameters and step metrics."""
    if len(message) != TUNING_SIZE:
        return None
    values = struct.unpack("<6B20h4H4B4BI", message)
    return {
        "state": values[0],
        "wheel": values[1],
        "phase": values[2],
        "progress": values[3],
        "result_mask": values[4],
        "error_code": values[5],
        "kp": [value / 100.0 for value in values[6:10]],
        "ki": [value / 100.0 for value in values[10:14]],
        "kd": [value / 10000.0 for value in values[14:18]],
        "feedforward": [float(value) for value in values[18:22]],
        "velocity_ff": [value / 100.0 for value in values[22:26]],
        "rise_ms": list(values[26:30]),
        "overshoot_pct": list(values[30:34]),
        "ss_error_pct": list(values[34:38]),
        "sequence": values[38],
    }


def parse_position(message):
    """DebugPosition -> dict: position-control masks and continuous wheel angles in degrees."""
    if len(message) != POSITION_SIZE:
        return None
    values = struct.unpack("<3B4i4iI", message)
    return {
        "active_mask": values[0],
        "done_mask": values[1],
        "error_code": values[2],
        "angle_deg": [value / 1000.0 for value in values[3:7]],
        "target_deg": [value / 1000.0 for value in values[7:11]],
        "sequence": values[11],
    }


def encode_position_control(op, mode, wheel_mask, angles_deg, max_rpm=0, sequence=0):
    """Build a DebugPositionControl message (dashboard -> firmware)."""
    angles = [int(round(angle * 1000.0)) for angle in angles_deg]
    return struct.pack(
        "<BBB4iHI", op, mode, wheel_mask, *angles, int(max_rpm), int(sequence)
    )


def parse_frame(msg_type, message):
    """Map one decoded frame to its parser; None for unknown types."""
    parsers = {
        MSG_STATE: parse_state,
        MSG_IMU: parse_imu,
        MSG_TOF: parse_tof,
        MSG_HEALTH: parse_health,
        MSG_TEXT: parse_text,
        MSG_CALIBRATION: parse_calibration,
        MSG_TUNING: parse_tuning,
        MSG_POSITION: parse_position,
    }
    parser = parsers.get(msg_type)
    return parser(message) if parser is not None else None
