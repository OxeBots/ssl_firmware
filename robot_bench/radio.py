"""nRF24L01 USB adapter link and the robot radio protocol codec.

The bench reaches a robot either through the USB debug serial link
(serial_io.BenchReader) or through this radio link. The adapter is a USB
serial bridge (CH340, 1a86:7523) that forwards every UART write as one radio
packet and writes received radio payloads back to the serial port, so the
over-the-air protocol is exactly the one in proto/ssl_robot_protocol.bitproto
(the same one scripts/nrf24_usb.py speaks). A session alternates commands
with unicast telemetry requests. The firmware stops the wheels 250 ms after
the last motion command. The dashboard encodes and decodes the bitproto
frame layout with struct.
"""

import collections
import math
import struct
import threading
import time

import serial
import serial.tools.list_ports

# nRF24L01 USB adapter identity (CH340 bridge).
ADAPTER_VID = 0x1A86
ADAPTER_PID = 0x7523

# Radio protocol constants (proto/ssl_robot_protocol.bitproto).
MSG_TYPE_COMMAND = 0
MSG_TYPE_CONFIG = 1
MSG_TYPE_TELEMETRY = 2
MSG_TYPE_TELEMETRY_REQ = 3
ROBOT_ID_MAX = 10
ROBOT_ID_BROADCAST = 15
CONFIG_FLAG_SET_ID = 1
CONFIG_FLAG_RUN_CALIBRATION = 2
CONFIG_FLAG_CALIBRATE_MAG = 4
CONFIG_FLAG_FACTORY_RESET = 16

# Bitproto message sizes: Header + Payload (see nrf24_bench.c).
COMMAND_SIZE = 30
CONFIG_SIZE = 8
TELEMETRY_SIZE = 22

# Radio bit meanings of RobotTelemetry.error_flags.
ERROR_FLAGS = (
    (1, "Comunicação"),
    (2, "IMU"),
    (4, "ToF"),
    (8, "Bateria baixa"),
    (16, "Bateria alta"),
    (32, "Kicker"),
    (64, "Encoder"),
    (128, "Temperatura"),
    (256, "Motor 0"),
    (512, "Motor 1"),
    (1024, "Motor 2"),
    (2048, "Motor 3"),
)

# The bridge writes this status code on the serial stream after each completed
# transmission; strip it before looking for telemetry frames.
_BRIDGE_STATUS = b"\x02\x00\x00\x00\x00\x00"

# Adapter radio setup: receive on the address the robot transmits to ("ADMIN")
# and transmit to the address the robot listens on ("ESP32"), channel 76
# (2.476 GHz) and 2 Mbps, matching sdkconfig.defaults.
_ADAPTER_RX_ADDRESS = (0x41, 0x44, 0x4D, 0x49, 0x4E)
_ADAPTER_TX_ADDRESS = (0x45, 0x53, 0x50, 0x33, 0x32)
_ADAPTER_FREQUENCY_GHZ = 2.476
_ADAPTER_DATA_RATE = 3

# Adapter UART.
BAUD = 115200
AT_TIMEOUT_S = 2.0
# The bridge frames UART bursts as radio packets. Leave an idle gap so it
# sends each write as a separate packet.
FRAME_GAP_S = 0.05
# Serial wait for the reply to one telemetry request.
REPLY_TIMEOUT_S = 0.08
# Keyboard motion expires if the window stops refreshing it or replies stop.
KEYBOARD_LEASE_S = 0.20
KEYBOARD_LINK_TIMEOUT_S = 0.25
KEYBOARD_MIN_RPM = 50
# Geometry shared with lib/kinematics/omni_robot.h, in millimetres.
WHEEL_RADIUS_MM = 32.5
WHEEL_DISTANCE_MM = 96.0
# The firmware caps body rotation at 8 rad/s.
KEYBOARD_MAX_RPM = int(8.0 * WHEEL_DISTANCE_MM * 60 / (math.tau * WHEEL_RADIUS_MM))
# A telemetry frame older than this means the robot is not answering.
LIVE_STALE_S = 1.0
RECONNECT_DELAY_S = 1.0
# Give up reconnecting and hand control back to the connection picker.
PICKER_FALLBACK_S = 8.0
# Auto-discovery waits this long for the telemetry reply of each robot id.
DISCOVERY_TIMEOUT_S = 0.12


def _timestamp_ms():
    return int(time.time() * 1000) & 0xFFFFFFFF


def robot_id_valid(robot_id):
    """True for a unicast robot id the protocol accepts as a command target."""
    return 0 <= robot_id <= ROBOT_ID_MAX


def find_adapters():
    """Attached nRF24L01 USB adapters as (device, description) pairs."""
    adapters = []
    for port in serial.tools.list_ports.comports():
        if port.vid == ADAPTER_VID and port.pid == ADAPTER_PID:
            adapters.append((port.device, port.description or "Adaptador nRF24L01"))
    return sorted(adapters)


def error_flag_names(flags):
    """Readable names of the RobotTelemetry error bits that are set."""
    return [name for bit, name in ERROR_FLAGS if flags & bit]


def keyboard_velocity(x, y, turn, rpm):
    """Convert directional keys to body velocity at the requested moving-wheel RPM."""
    rpm = min(KEYBOARD_MAX_RPM, max(KEYBOARD_MIN_RPM, rpm))
    wheel_mm_s = math.tau * rpm / 60 * WHEEL_RADIUS_MM
    if turn and not (x or y):
        return 0, 0, turn * math.ceil(100 * wheel_mm_s / WHEEL_DISTANCE_MM)
    axes = abs(x) + abs(y)
    if axes and not turn:
        # Wheels sit at 45 degrees. On a diagonal only two wheels move.
        speed = math.ceil(math.sqrt(2) * wheel_mm_s / axes)
        return x * speed, y * speed, 0
    return 0, 0, 0


def encode_command(robot_id, x=0, y=0, angle=0, vx=0, vy=0, w=0, kick=0, timestamp=None):
    """Build one RobotCommand message (30 bytes) for the radio link.

    Units match the protocol: position in mm, angle in 0.01 degree, linear
    velocity in mm/s, angular velocity in 0.01 rad/s and kick in 1..255.
    """
    if not 0 <= robot_id <= 0x0F:
        raise ValueError(f"robot id fora da faixa: {robot_id}")
    stamp = _timestamp_ms() if timestamp is None else int(timestamp) & 0xFFFFFFFF
    header = bytes([(robot_id << 4) | MSG_TYPE_COMMAND]) + struct.pack("<I", stamp)
    pose = struct.pack("<6h", int(x), int(y), int(angle), int(vx), int(vy), int(w))
    return header + pose + bytes(12) + bytes([int(kick) & 0xFF])


def encode_config(robot_id, config_flags, param=0, timestamp=None):
    """Build one RobotConfig message (8 bytes) for the radio link."""
    if not 0 <= robot_id <= 0x0F:
        raise ValueError(f"robot id fora da faixa: {robot_id}")
    stamp = _timestamp_ms() if timestamp is None else int(timestamp) & 0xFFFFFFFF
    return (
        bytes([(robot_id << 4) | MSG_TYPE_CONFIG])
        + struct.pack("<I", stamp)
        + bytes([int(config_flags) & 0xFF])
        + struct.pack("<H", int(param) & 0xFFFF)
    )


def encode_telemetry_request(robot_id, timestamp):
    """Build a unicast RobotTelemetryRequest containing only a header."""
    if not robot_id_valid(robot_id):
        raise ValueError(f"robot id fora da faixa: {robot_id}")
    return bytes([(robot_id << 4) | MSG_TYPE_TELEMETRY_REQ]) + struct.pack("<I", timestamp)


def decode_telemetry(frame):
    """Decode one RobotTelemetry frame into a dict, or None when malformed."""
    if len(frame) < TELEMETRY_SIZE or (frame[0] & 0x0F) != MSG_TYPE_TELEMETRY:
        return None
    timestamp = struct.unpack_from("<I", frame, 1)[0]
    x, y, angle, vx, vy, w = struct.unpack_from("<6h", frame, 5)
    battery = frame[17]
    kicker_voltage, error_flags = struct.unpack_from("<2H", frame, 18)
    return {
        "robot_id": frame[0] >> 4,
        "timestamp": timestamp,
        "x_mm": x,
        "y_mm": y,
        "angle_deg": angle / 100.0,
        "vx_mm_s": vx,
        "vy_mm_s": vy,
        "w_rad_s": w / 100.0,
        "battery_pct": battery,
        "kicker_voltage_v": kicker_voltage / 100.0,
        "error_flags": error_flags,
    }


class TelemetryStream:
    """Incremental decoder for the bytes the adapter writes back to the host."""

    def __init__(self):
        self._buffer = bytearray()

    def reset(self):
        self._buffer.clear()

    def feed(self, data):
        """Feed bytes; return the telemetry frames that completed."""
        self._buffer += data
        frames = []
        while True:
            if self._buffer.startswith(_BRIDGE_STATUS):
                del self._buffer[: len(_BRIDGE_STATUS)]
                continue
            if len(self._buffer) < TELEMETRY_SIZE:
                break
            frame = decode_telemetry(self._buffer[:TELEMETRY_SIZE])
            if frame is None:
                # Not the start of a telemetry frame: resynchronise one byte.
                del self._buffer[:1]
                continue
            frames.append(frame)
            del self._buffer[:TELEMETRY_SIZE]
        # Garbage between frames must not grow the buffer without bound.
        if len(self._buffer) > 4 * TELEMETRY_SIZE:
            del self._buffer[:-TELEMETRY_SIZE]
        return frames


class RadioError(Exception):
    """The adapter could not be opened or configured."""


def _at_command(ser, command, timeout=AT_TIMEOUT_S):
    """Send one AT command and return its decoded response text."""
    ser.reset_input_buffer()
    ser.write((command + "\r\n").encode("ascii"))
    ser.flush()
    response = b""
    deadline = time.monotonic() + timeout
    quiet_deadline = None
    while time.monotonic() < deadline:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting)
            quiet_deadline = None
        elif response:
            if quiet_deadline is None:
                quiet_deadline = time.monotonic() + 0.35
            elif time.monotonic() >= quiet_deadline:
                break
        time.sleep(0.01)
    return response.decode("gb2312", errors="replace")


def _address_text(address):
    return ",".join(f"0x{byte:02X}" for byte in address)


class RadioSession(threading.Thread):
    """Streams robot commands over the adapter and collects telemetry replies."""

    def __init__(self, port, robot_id=0):
        super().__init__(daemon=True, name="radio-link")
        self.port = port
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._ser = None
        self._decoder = TelemetryStream()
        self._next_tx_at = 0.0
        self._robot_id = int(robot_id) if robot_id_valid(int(robot_id)) else 0
        self._command = {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0}
        self._keyboard_velocity = (0, 0, 0)
        self._keyboard_until = 0.0
        self._pending_kick = 0
        self._configs = collections.deque()
        self._telemetry = None
        self._last_rx_t = 0.0
        self._last_rtt_ms = 0.0
        self._tx_count = 0
        self._rx_count = 0
        self._connected = False
        self._error = ""
        self._request_picker = False
        self._messages = collections.deque(maxlen=60)

    @classmethod
    def connect(cls, port, robot_id=0):
        """Open and configure the adapter; raises RadioError when unusable."""
        session = cls(port, robot_id)
        try:
            session._open()
        except (serial.SerialException, OSError, RadioError) as exc:
            session._close_serial()
            raise RadioError(str(exc)) from exc
        session._log(f"rádio conectado em {port} · robô {session._robot_id}")
        return session

    # ------------------------------------------------------------------
    # Adapter setup
    # ------------------------------------------------------------------

    def _open(self):
        ser = serial.Serial(self.port, BAUD, timeout=0.01, write_timeout=0.2, exclusive=True)
        self._ser = ser
        try:
            ser.reset_input_buffer()
            self._configure_adapter(ser)
            ser.reset_input_buffer()
        except (serial.SerialException, OSError, RadioError):
            self._close_serial()
            raise
        self._connected = True

    def _configure_adapter(self, ser):
        commands = (
            f"AT+RXA={_address_text(_ADAPTER_RX_ADDRESS)}",
            f"AT+TXA={_address_text(_ADAPTER_TX_ADDRESS)}",
            f"AT+FREQ={_ADAPTER_FREQUENCY_GHZ:.3f}",
            f"AT+RATE={_ADAPTER_DATA_RATE}",
        )
        for command in commands:
            response = _at_command(ser, command)
            if "成功" not in response and "successful" not in response.lower():
                raise RadioError(f"adaptador não aceitou {command}: {response.strip() or 'sem resposta'}")

    def _close_serial(self):
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass

    # ------------------------------------------------------------------
    # Command API (GUI thread)
    # ------------------------------------------------------------------

    def _log(self, message):
        self._messages.append(message)
        print(message, flush=True)

    def set_motion(self, x=None, y=None, angle=None, vx=None, vy=None, w=None):
        """Replace the streamed command fields; None keeps the current value."""
        values = {"x": x, "y": y, "angle": angle, "vx": vx, "vy": vy, "w": w}
        with self._lock:
            self._keyboard_until = 0.0
            for field, value in values.items():
                if value is not None:
                    self._command[field] = int(value)

    def keyboard_link_ready(self):
        """Require a recent reply from the selected robot before taking the wheels."""
        with self._lock:
            return self._connected and 0 < self._last_rx_t and (
                time.monotonic() - self._last_rx_t < KEYBOARD_LINK_TIMEOUT_S
            )

    def set_keyboard_velocity(self, velocity):
        """Refresh a short-lived velocity command from the dashboard input loop."""
        with self._lock:
            self._keyboard_velocity = velocity
            if any(velocity):
                if not self._keyboard_until:
                    self._command = {field: 0 for field in self._command}
                    self._pending_kick = 0
                self._keyboard_until = time.monotonic() + KEYBOARD_LEASE_S
            else:
                self._keyboard_until = 0.0

    def stop_motion(self):
        """Stream a zero velocity command (the wheels stop) and drop pending kicks."""
        with self._lock:
            self._command = {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0}
            self._keyboard_until = 0.0
            self._pending_kick = 0

    def kick(self, strength):
        """Send one kick with the next command; strength 1..255."""
        with self._lock:
            self._pending_kick = max(0, min(255, int(strength)))

    def set_robot_id(self, robot_id):
        """Change the target robot the streamed command is addressed to."""
        robot_id = int(robot_id)
        if not robot_id_valid(robot_id):
            return False
        with self._lock:
            if robot_id != self._robot_id:
                self._robot_id = robot_id
                self._keyboard_until = 0.0
                self._last_rx_t = 0.0
                self._telemetry = None
        self._log(f"rádio: robô alvo alterado para {robot_id}")
        return True

    def send_config(self, config_flags, param=0):
        """Queue one RobotConfig command; it is sent before the next poll."""
        with self._lock:
            self._configs.append((int(config_flags), int(param)))
        self._log(f"rádio: config 0x{int(config_flags):02X} param {int(param)}")

    # ------------------------------------------------------------------
    # Robot discovery (picker)
    # ------------------------------------------------------------------

    def probe(self, robot_id, timeout_s=DISCOVERY_TIMEOUT_S):
        """Request telemetry from robot_id and return the matching reply."""
        expected_timestamp = _timestamp_ms()
        frame = encode_telemetry_request(robot_id, expected_timestamp)
        if not self._write(frame):
            return None
        sent = time.monotonic()
        with self._lock:
            self._tx_count += 1
        for telemetry in self._read_reply(sent + timeout_s):
            if telemetry["robot_id"] == robot_id and telemetry["timestamp"] == expected_timestamp:
                telemetry["rtt_ms"] = (time.monotonic() - sent) * 1000.0
                return telemetry
        return None

    def discover(self, timeout_s=DISCOVERY_TIMEOUT_S):
        """Probe every robot id and return the robots that answer, in id order."""
        found = []
        for robot_id in range(ROBOT_ID_MAX + 1):
            telemetry = self.probe(robot_id, timeout_s)
            if telemetry is not None:
                found.append(telemetry)
        return found

    # ------------------------------------------------------------------
    # Snapshot for the panel (GUI thread)
    # ------------------------------------------------------------------

    def snapshot(self):
        """Thread-shared copy of the link state for the radio panel."""
        with self._lock:
            command = dict(self._command)
            configs = len(self._configs)
            last_rx_t = self._last_rx_t
            return {
                "port": self.port,
                "connected": self._connected,
                "robot_id": self._robot_id,
                "telemetry": dict(self._telemetry) if self._telemetry else None,
                "rx_age_s": (time.monotonic() - last_rx_t) if last_rx_t else None,
                "rtt_ms": self._last_rtt_ms,
                "tx_count": self._tx_count,
                "rx_count": self._rx_count,
                "command": command,
                "pending_configs": configs,
                "error": self._error,
                "messages": list(self._messages),
                "live": bool(last_rx_t) and time.monotonic() - last_rx_t <= LIVE_STALE_S,
            }

    def request_picker(self):
        """True when the adapter is gone and the dashboard should rescan."""
        return self._request_picker

    def close(self):
        """Stop the link thread and close the adapter port."""
        self._closing.set()
        if self.ident is not None:
            self.join(timeout=2.0)
        self._close_serial()

    # ------------------------------------------------------------------
    # Link loop
    # ------------------------------------------------------------------

    def _next_config(self):
        with self._lock:
            if self._configs:
                return self._configs.popleft()
            return None

    def _next_frame(self):
        """Frame to transmit next: a queued config or the streamed command."""
        config = self._next_config()
        if config is not None:
            flags, param = config
            with self._lock:
                robot_id = self._robot_id
            return encode_config(robot_id, flags, param)
        with self._lock:
            robot_id = self._robot_id
            command = dict(self._command)
            now = time.monotonic()
            if self._keyboard_until:
                if (
                    now < self._keyboard_until
                    and self._connected
                    and 0 < self._last_rx_t
                    and now - self._last_rx_t < KEYBOARD_LINK_TIMEOUT_S
                ):
                    command["vx"], command["vy"], command["w"] = self._keyboard_velocity
                else:
                    self._keyboard_until = 0.0
            kick, self._pending_kick = self._pending_kick, 0
        return encode_command(robot_id, kick=kick, **command)

    def _write(self, frame):
        with self._lock:
            ser = self._ser
        if ser is None:
            return False
        wait_s = self._next_tx_at - time.monotonic()
        if wait_s > 0.0:
            time.sleep(wait_s)
        try:
            ser.write(frame)
            ser.flush()
        except (serial.SerialException, OSError):
            self._link_lost()
            return False
        self._next_tx_at = time.monotonic() + FRAME_GAP_S
        return True

    def _read_reply(self, deadline):
        """Yield decoded replies as they arrive before the deadline."""
        while time.monotonic() < deadline:
            with self._lock:
                ser = self._ser
            if ser is None:
                return
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting) if waiting else b""
            except (serial.SerialException, OSError):
                self._link_lost()
                return
            if data:
                yield from self._decoder.feed(data)
            else:
                time.sleep(0.002)

    def _link_lost(self):
        with self._lock:
            self._connected = False
            self._keyboard_until = 0.0
            self._last_rx_t = 0.0
        if not self._error:
            self._log("rádio: conexão com o adaptador perdida, reconectando...")
        self._error = "conexão perdida"
        self._close_serial()

    def _reconnect(self):
        try:
            self._open()
        except (serial.SerialException, OSError, RadioError):
            self._close_serial()
            return False
        self._error = ""
        self._decoder.reset()
        self._log(f"rádio reconectado em {self.port}")
        return True

    def run(self):
        first_failure = None
        request_telemetry = True
        while not self._closing.is_set():
            if self._ser is None:
                now = time.monotonic()
                if first_failure is None:
                    first_failure = now
                elif now - first_failure >= PICKER_FALLBACK_S:
                    self._request_picker = True
                    self._log("rádio: adaptador indisponível - voltando para a seleção de conexão")
                    return
                if not self._reconnect():
                    time.sleep(RECONNECT_DELAY_S)
                    continue
            first_failure = None
            if request_telemetry:
                with self._lock:
                    robot_id = self._robot_id
                telemetry = self.probe(robot_id, REPLY_TIMEOUT_S)
                with self._lock:
                    if telemetry is not None and telemetry["robot_id"] == self._robot_id:
                        self._telemetry = telemetry
                        self._last_rx_t = time.monotonic()
                        self._rx_count += 1
                        self._last_rtt_ms = telemetry["rtt_ms"]
            elif self._write(self._next_frame()):
                with self._lock:
                    self._tx_count += 1
            request_telemetry = not request_telemetry
            self._closing.wait(max(0.0, self._next_tx_at - time.monotonic()))
        with self._lock:
            robot_id = self._robot_id
        self._write(encode_command(robot_id))

def discover_robots(port, timeout_s=DISCOVERY_TIMEOUT_S):
    """Open the adapter and list robots that answer a telemetry request."""
    session = RadioSession.connect(port)
    try:
        return session.discover(timeout_s)
    finally:
        session.close()
