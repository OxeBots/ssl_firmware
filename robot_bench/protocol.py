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
MSG_RAW = 11
MSG_SUBSCRIBE = 12
MSG_LINK = 13
MSG_WHEEL_TUNING = 14

WHEEL_TUNING_REQUEST = 0
WHEEL_TUNING_OK = 1
WHEEL_TUNING_BUSY = 2
WHEEL_TUNING_INVALID = 3
WHEEL_TUNING_FIELDS = 23
_WHEEL_TUNING = struct.Struct(f"<BBBII{WHEEL_TUNING_FIELDS}f")

STREAM_STATE = 1
STREAM_IMU = 2
STREAM_TOF = 4
STREAM_HEALTH = 8
STREAM_CALIBRATION = 16
STREAM_TUNING = 32
STREAM_POSITION = 64
STREAM_RAW = 128
STREAM_TEXT = 256
STREAM_ENCODER_STAGES = 512

# Bitproto serialized sizes (BYTES_LENGTH_* in the generated C header)
STATE_SIZE = 61
IMU_SIZE = 28
TOF_SIZE = 196
HEALTH_SIZE = 29
TEXT_SIZE = 97
CONTROL_SIZE = 13
CALIBRATION_SIZE = 70
TUNING_SIZE = 108
POSITION_SIZE = 39
POSITION_CONTROL_SIZE = 25
RAW_SIZE = 44

# DebugControl operation ids
OP_STOP_STREAM = 0
OP_SET_TARGETS = 3
OP_STOP_MOTORS = 4
# Bench override: wheel targets pass through while the health monitor
# inhibits motion (bare bench without supply sense or encoder calibration).

# Motor/encoder wiring calibration: the robot spins one motor at a time in
# open loop, detects the encoder behind each wheel and stores the map in NVS.
OP_START_CALIBRATION = 7
OP_STOP_CALIBRATION = 8

# Wheel control calibration: per-wheel friction/plant identification, gain
# synthesis, closed-loop validation and NVS persistence.
OP_START_TUNING = 9
OP_STOP_TUNING = 10

# Raw view: focus the 10 Hz raw I2C read on one wheel's encoder through the
# CD4052B mux (wheel_targets[0] = 0..3 selects M1..M4, -1 releases the focus).
OP_SELECT_RAW_ENCODER = 11

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
# Every wheel was visited but at least one was not matched; the stored map is untouched.
CALIBRATION_PARTIAL = 4

# DebugCalibration failure reasons
CALIBRATION_ERROR_NONE = 0
CALIBRATION_ERROR_NO_MOVEMENT = 1
CALIBRATION_ERROR_AMBIGUOUS_ANALOG = 2
CALIBRATION_ERROR_AMBIGUOUS_I2C = 3
CALIBRATION_ERROR_BAD_MAPPING = 4
CALIBRATION_ERROR_ABORTED = 5
CALIBRATION_ERROR_I2C = 6
CALIBRATION_ERROR_MAGNET_TOO_CLOSE = 7
CALIBRATION_ERROR_MAGNET_TOO_FAR = 8

# DebugCalibration encoder operation modes (AS5600 output stage). REDUCED keeps
# the analog output inside the ESP32 ADC linear range and is the required mode.
ENCODER_STAGE_FULL = 0
ENCODER_STAGE_REDUCED = 1
ENCODER_STAGE_PWM = 2
ENCODER_STAGE_UNKNOWN = 3

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
TUNING_PHASE_MAX_SPEED = 7

# DebugTuning failure reasons
TUNING_ERROR_NONE = 0
TUNING_ERROR_NO_MOVEMENT = 1
TUNING_ERROR_UNSTABLE = 2
TUNING_ERROR_TIMEOUT = 3
TUNING_ERROR_ABORTED = 4
TUNING_ERROR_SPEED_MISMATCH = 5
TUNING_ERROR_SAVE_FAILED = 6

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


def encode_subscription(streams):
    return struct.pack("<H", streams)


# The board prints plain text on the serial port before the firmware starts
# framing its own logs (ROM bootloader, ESP-IDF startup). Such a line is
# printable ASCII up to a newline; a binary frame carries its message type
# right after the COBS code byte, so it never looks printable.
_CONSOLE_LINE_MAX = 512
# Bound the lines kept while nobody drains them (radio discovery also decodes).
_PENDING_LINES_MAX = 256


class FrameDecoder:
    """Streaming decoder: feed raw serial bytes, receive decoded frames.

    Text lines printed directly on the port are collected as console lines;
    drain_lines() returns the lines that arrived since the previous call.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._lines = []

    def feed(self, data):
        """Feed bytes; return a list of (msg_type, message) frames."""
        self._buffer += data
        frames = []
        while True:
            # Text printed directly on the port may sit between frames too.
            self._split_text_lines()
            index = self._buffer.find(0)
            if index < 0:
                if len(self._buffer) > _MAX_CHUNK and not _is_plain_text(self._buffer):
                    self._buffer.clear()
                break
            chunk = bytes(self._buffer[:index])
            del self._buffer[: index + 1]
            if chunk:
                frame = _decode_frame(chunk)
                if frame is not None:
                    frames.append(frame)
        return frames

    def drain_lines(self):
        """Console lines printed directly on the port since the last call."""
        lines = self._lines
        self._lines = []
        return lines

    def restart(self):
        """Begin a new session: drop a dead session's half frame, keep text."""
        if not _is_plain_text(self._buffer):
            self._buffer.clear()

    def _split_text_lines(self):
        """Peel complete plain text lines off the head of the buffer."""
        while True:
            end = self._buffer.find(b"\n")
            if end < 0 or not _is_plain_text(self._buffer[: end + 1]):
                return
            raw = bytes(self._buffer[: end + 1])
            del self._buffer[: end + 1]
            line = _strip_escapes(raw.rstrip(b"\r\n").decode("utf-8", "replace")).rstrip()
            if line:
                self._lines.append(line)
                if len(self._lines) > _PENDING_LINES_MAX:
                    del self._lines[0]


def _is_plain_text(raw):
    """True when the bytes are the printable text the board prints directly.

    A binary frame always carries its message type (a control character) right
    after the COBS code byte, so it can never pass this check.
    """
    if len(raw) > _CONSOLE_LINE_MAX:
        return False
    # ESC starts an ANSI colour sequence that the console strips for display.
    return all(0x20 <= byte <= 0x7E or byte in (0x09, 0x0A, 0x0D, 0x1B) for byte in raw)


def _strip_escapes(line):
    """Drop ANSI colour sequences so the console shows plain text."""
    out = []
    index = 0
    length = len(line)
    while index < length:
        if line[index] == "\x1b":
            # ESC [ ... letter is the colour sequence; any other escape is
            # dropped too, so no control byte reaches the console.
            index += 1
            if line[index : index + 1] == "[":
                index += 1
                while index < length and not ("@" <= line[index] <= "~"):
                    index += 1
                index += 1
            continue
        out.append(line[index])
        index += 1
    return "".join(out)


def _decode_frame(chunk):
    """Decode one COBS chunk; return (msg_type, message) or None."""
    payload = cobs_decode(chunk)
    if payload is None or len(payload) < 3:
        return None
    if crc8(payload[:-1]) != payload[-1]:
        return None
    return payload[0], payload[1:-1]


def parse_state(message):
    """Decode motor telemetry, including the two packed 12-bit fields."""
    if len(message) != STATE_SIZE:
        return None
    uptime_ms = struct.unpack_from("<I", message, 0)[0]
    wheels = []
    for i in range(WHEEL_COUNT):
        pwm, velocity, target, packed, count = struct.unpack_from("<3h3sI", message, 4 + i * 13)
        angle_and_pulse = int.from_bytes(packed, "little", signed=True)
        wheels.append({
            "pwm": pwm,
            "vel_rpm": velocity / 10.0,
            "tgt_rpm": target / 10.0,
            "angle_deg": (angle_and_pulse & 0xFFF) / 10.0,
            "pulse_rpm": float(angle_and_pulse >> 12),
            "pulse_count": count,
        })
    sequence, valid = struct.unpack_from("<IB", message, 56)
    return {
        "uptime_ms": uptime_ms,
        "wheels": wheels,
        "valid_mask": valid & 0xF,
        "pulse_valid_mask": valid >> 4,
        "sequence": sequence,
    }


def parse_imu(message):
    """DebugImu -> dict: acceleration in g, rotation in deg/s, field in mG."""
    if len(message) != IMU_SIZE:
        return None
    values = struct.unpack("<9hI3h", message)
    return {
        "accel": [values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0],
        "gyro": [values[3] / 10.0, values[4] / 10.0, values[5] / 10.0],
        "mag": [float(values[6]), float(values[7]), float(values[8])],
        "sequence": values[9],
        "yaw": values[10] / 100.0,
        "pitch": values[11] / 100.0,
        "roll": values[12] / 100.0,
    }


def parse_link(message):
    if len(message) != 4:
        return None
    return {"uptime_ms": struct.unpack("<I", message)[0]}


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
    (robot_id, error_flags, lifecycle, active_faults, latched_faults, _, kick,
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
    """DebugCalibration -> dict: routine state, detected wiring, levels and stages.

    encoder_stage[i] is the AS5600 output stage of wheel i (0 FULL, 1 REDUCED,
    2 PWM, 3 unknown) and stage_mask bit i means that value was read/verified.
    adc_min_raw/adc_max_raw are the stored raw ADC endpoints (0 and 360 degree)
    of wheel i's encoder; both are 0 when range_mask bit i is clear.
    magnet_warning[i] is the last magnetic field warning on mux input i.
    detected_mux_map[i] is the mux input detected for wheel i when its i2c_ready bit is set.
    """
    if len(message) != CALIBRATION_SIZE:
        return None
    values = struct.unpack("<6B4B4B4h4hBBB4BI4BB4H4H4B4B", message)
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
        "analog_ready": values[24],
        "wheel_error": list(values[25:29]),
        "sequence": values[29],
        "encoder_stage": list(values[30:34]),
        "stage_mask": values[34],
        "adc_min_raw": list(values[35:39]),
        "adc_max_raw": list(values[39:43]),
        "magnet_warning": list(values[43:47]),
        "detected_mux_map": list(values[47:51]),
    }


def parse_tuning(message):
    """DebugTuning -> dict: routine state, per-wheel parameters and step metrics.

    The first gain, feedforward and metric arrays describe the forward (positive
    target) direction; the *_rev arrays describe the reverse direction. Sequence
    is field 15, the reverse fields 16..20 and the calibrated speed limit plus
    the per-wheel measured maxima are fields 21 and 22.
    """
    if len(message) != TUNING_SIZE:
        return None
    values = struct.unpack("<6B20h4H4B4BI4h4h4H4B4BH4H", message)
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
        "feedforward_rev": [float(value) for value in values[39:43]],
        "velocity_ff_rev": [value / 100.0 for value in values[43:47]],
        "rise_ms_rev": list(values[47:51]),
        "overshoot_pct_rev": list(values[51:55]),
        "ss_error_pct_rev": list(values[55:59]),
        "max_rpm": values[59] / 10.0,
        "wheel_max_rpm": [value / 10.0 for value in values[60:64]],
    }


def parse_raw(message):
    """DebugRaw -> dict: raw encoder values and motor feedback per wheel.

    i2c_raw keeps the 0xFFFF sentinel when the AS5600 angle could not be read;
    ekf_rpm is in the motor frame (positive = CW) before the control smoothing.
    """
    if len(message) != RAW_SIZE:
        return None
    values = struct.unpack("<I4H4H4H4h4h", message)
    return {
        "sequence": values[0],
        "analog_raw": list(values[1:5]),
        "i2c_raw": list(values[5:9]),
        "pulse_delta": list(values[9:13]),
        "ekf_rpm": [value / 10.0 for value in values[13:17]],
        "applied_pwm": list(values[17:21]),
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


def encode_wheel_tuning(wheel, sequence, fields=0, values=None):
    if values is None:
        values = (0.0,) * WHEEL_TUNING_FIELDS
    return _WHEEL_TUNING.pack(wheel, WHEEL_TUNING_REQUEST, 0, sequence, fields, *values)


def parse_wheel_tuning(message):
    if len(message) != _WHEEL_TUNING.size:
        return None
    wheel, status, mode, sequence, fields, *values = _WHEEL_TUNING.unpack(message)
    if status not in (WHEEL_TUNING_OK, WHEEL_TUNING_BUSY, WHEEL_TUNING_INVALID):
        return None
    return {
        "wheel": wheel,
        "status": status,
        "mode": mode,
        "sequence": sequence,
        "fields": fields,
        "values": values,
    }


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
        MSG_WHEEL_TUNING: parse_wheel_tuning,
        MSG_POSITION: parse_position,
        MSG_RAW: parse_raw,
        MSG_LINK: parse_link,
    }
    parser = parsers.get(msg_type)
    return parser(message) if parser is not None else None
