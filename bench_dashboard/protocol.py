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

# Bitproto serialized sizes (BYTES_LENGTH_* in the generated C header)
STATE_SIZE = 39
IMU_SIZE = 22
TOF_SIZE = 132
HEALTH_SIZE = 27
TEXT_SIZE = 97
CONTROL_SIZE = 13

# DebugControl operation ids
OP_STOP_STREAM = 0
OP_START_STREAM = 1
OP_KEEPALIVE = 2
OP_SET_TARGETS = 3
OP_STOP_MOTORS = 4

TOF_ZONES = 64
WHEEL_COUNT = 4

# Sanity limit for garbage chunks between frames (max valid frame is ~136 B)
_MAX_CHUNK = 170

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
    wheels = [struct.unpack_from("<3h", message, 4 + i * 6) for i in range(WHEEL_COUNT)]
    yaw, pitch, roll = struct.unpack_from("<3h", message, 28)
    valid_mask = message[34]
    sequence = struct.unpack_from("<I", message, 35)[0]
    return {
        "uptime_ms": uptime_ms,
        "wheels": [
            {"pwm": w[0], "vel_rpm": w[1] / 10.0, "tgt_rpm": w[2] / 10.0} for w in wheels
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
    """DebugTof -> dict: 64 distances in mm (0 = no valid target)."""
    if len(message) != TOF_SIZE:
        return None
    sequence = struct.unpack_from("<I", message, 0)[0]
    distances = list(struct.unpack_from("<64h", message, 4))
    return {"sequence": sequence, "distances": distances}


def parse_health(message):
    """DebugHealth -> dict of safety, power and system status."""
    if len(message) != HEALTH_SIZE:
        return None
    (robot_id, error_flags, lifecycle, active_faults, latched_faults, motion, kick,
     imu_cal, tof, supply, supply_flags, free_heap, min_free_heap, sequence) = struct.unpack(
        "<BHBHHBBBBHBIII", message
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


def parse_frame(msg_type, message):
    """Map one decoded frame to its parser; None for unknown types."""
    parsers = {
        MSG_STATE: parse_state,
        MSG_IMU: parse_imu,
        MSG_TOF: parse_tof,
        MSG_HEALTH: parse_health,
        MSG_TEXT: parse_text,
    }
    parser = parsers.get(msg_type)
    return parser(message) if parser is not None else None
