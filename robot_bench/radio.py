"""ESP-NOW gateway transport and the SSL robot Bitproto codec."""

import collections
import math
import os
import select
import struct
import threading
import time

import serial

from . import protocol

ESP_LINK_PORT = os.environ.get("ESP_LINK_PORT", "/dev/ttyUSB0")
ESP_LINK_BAUD = int(os.environ.get("ESP_LINK_BAUD", "921600"))
ESP_LINK_FRAME_TYPE = 0xA5
ESP_LINK_LOOPBACK_TYPE = 0xA6
ESP_LINK_BROADCAST_TYPE = 0xA7
GATEWAY_READY_LINE = "ESP_LINK ready"

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

# Bitproto message sizes.
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

# Space synchronous discovery requests.
FRAME_GAP_S = 0.05
COMMAND_PERIOD_S = 1 / 60
TELEMETRY_PERIOD_S = 1 / 20
IDLE_TELEMETRY_PERIOD_S = 1.0
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
    """Return the configured gateway port when its serial device exists."""
    return [(ESP_LINK_PORT, "Ponte ESP-NOW")] if os.path.exists(ESP_LINK_PORT) else []


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
    if (
        len(frame) != TELEMETRY_SIZE
        or (frame[0] & 0x0F) != MSG_TYPE_TELEMETRY
        or not robot_id_valid(frame[0] >> 4)
    ):
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


def encode_link_frame(payload, frame_type=ESP_LINK_FRAME_TYPE):
    """Wrap one Bitproto message for the gateway's UART."""
    return b"\x00" + protocol.encode_frame(frame_type, payload)


class TelemetryStream:
    """Incremental decoder for the gateway's UART telemetry stream."""

    def __init__(self, frame_type=ESP_LINK_FRAME_TYPE):
        self._frame_type = frame_type
        self._decoder = protocol.FrameDecoder()

    def reset(self):
        self._decoder = protocol.FrameDecoder()

    def drain_lines(self):
        return self._decoder.drain_lines()

    def feed(self, data):
        """Feed bytes; return the telemetry frames that completed."""
        frames = []
        for kind, payload in self._decoder.feed(data):
            if kind == self._frame_type:
                frame = decode_telemetry(payload)
                if frame is not None:
                    frames.append(frame)
        return frames


class RadioError(Exception):
    """The gateway is unavailable or the request could not be started."""


def open_gateway(port, baud=ESP_LINK_BAUD):
    """Open the ESP-NOW UART exclusively with control lines deasserted."""
    ser = serial.Serial(baudrate=baud, timeout=0.01, write_timeout=0.2, exclusive=True)
    ser.port = port
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
        ser.reset_input_buffer()
    except (serial.SerialException, OSError):
        ser.close()
        raise
    return ser


def wait_for_gateway(port, cancelled):
    """Confirm the gateway has finished booting by matching a UART echo."""
    decoder = TelemetryStream(ESP_LINK_LOOPBACK_TYPE)
    stamp = _timestamp_ms()
    frame = encode_link_frame(encode_telemetry_request(0, stamp), ESP_LINK_LOOPBACK_TYPE)
    deadline = time.monotonic() + 3
    while not cancelled() and time.monotonic() < deadline:
        port.write(frame)
        next_probe = min(deadline, time.monotonic() + 0.1)
        while not cancelled() and (wait := next_probe - time.monotonic()) > 0:
            ready, _, _ = select.select([port], [], [], wait)
            if not ready:
                break
            for echo in decoder.feed(port.read(port.in_waiting or 1)):
                if echo["robot_id"] == 0 and echo["timestamp"] == stamp:
                    return True
    if cancelled():
        return False
    raise RadioError("Sem eco UART da ponte. Confira a porta, a taxa UART e o firmware do ESP Link.")


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
        self._benchmark = None

    @classmethod
    def connect(cls, port, robot_id=0):
        """Open the gateway's serial port; raises RadioError when unusable."""
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
        self._ser = open_gateway(self.port)
        self._connected = True

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
            if self._benchmark is not None:
                return
            self._keyboard_until = 0.0
            for field, value in values.items():
                if value is not None:
                    self._command[field] = int(value)

    def keyboard_link_ready(self):
        """Require a recent reply from the selected robot before taking the wheels."""
        with self._lock:
            return self._benchmark is None and self._connected and 0 < self._last_rx_t and (
                time.monotonic() - self._last_rx_t < KEYBOARD_LINK_TIMEOUT_S
            )

    def set_keyboard_velocity(self, velocity):
        """Refresh a short-lived velocity command from the dashboard input loop."""
        with self._lock:
            if self._benchmark is not None:
                return
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
            self._clear_motion()

    def _clear_motion(self):
        self._command = dict.fromkeys(self._command, 0)
        self._keyboard_until = 0.0
        self._pending_kick = 0

    def kick(self, strength):
        """Send one kick with the next command; strength 1..255."""
        with self._lock:
            if self._benchmark is None:
                self._pending_kick = max(0, min(255, int(strength)))

    def set_robot_id(self, robot_id):
        """Change the target robot the streamed command is addressed to."""
        robot_id = int(robot_id)
        if not robot_id_valid(robot_id):
            return False
        with self._lock:
            if self._benchmark is not None:
                return False
            if robot_id != self._robot_id:
                self._robot_id = robot_id
                self._clear_motion()
                self._last_rx_t = 0.0
                self._telemetry = None
        self._log(f"rádio: robô alvo alterado para {robot_id}")
        return True

    def send_config(self, config_flags, param=0):
        """Queue one RobotConfig command; it is sent before the next poll."""
        with self._lock:
            if self._benchmark is not None:
                return
            self._configs.append((int(config_flags), int(param)))
        self._log(f"rádio: config 0x{int(config_flags):02X} param {int(param)}")

    def start_benchmark(self, benchmark):
        """Give the benchmark exclusive use of the running session's serial link."""
        with self._lock:
            if not self._connected or self._ser is None or self._closing.is_set():
                raise RadioError("Conecte a ponte ESP-NOW antes de iniciar o teste.")
            if self._benchmark is not None or self._configs:
                raise RadioError("Aguarde o fim da operação de rádio atual.")
            self._clear_motion()
            self._benchmark = benchmark

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
                "benchmarking": self._benchmark is not None,
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
        with self._lock:
            if self._benchmark is not None:
                self._benchmark.cancel()
        if self.ident is not None:
            self.join(timeout=2.0)
        self._close_serial()

    # ------------------------------------------------------------------
    # Link loop
    # ------------------------------------------------------------------

    def _next_frame(self):
        """Send queued configuration or stream motion to a responding robot."""
        with self._lock:
            robot_id = self._robot_id
            if self._configs:
                flags, param = self._configs.popleft()
                return encode_config(robot_id, flags, param)
            now = time.monotonic()
            if not self._last_rx_t or now - self._last_rx_t > LIVE_STALE_S:
                self._clear_motion()
                return None
            command = dict(self._command)
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
        wait_s = self._next_tx_at - time.monotonic()
        if wait_s > 0.0:
            time.sleep(wait_s)
        if not self._send_frame(frame, flush=True):
            return False
        self._next_tx_at = time.monotonic() + FRAME_GAP_S
        return True

    def _send_frame(self, frame, *, flush=False):
        with self._lock:
            ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(encode_link_frame(frame))
            if flush:
                ser.flush()
        except (serial.SerialException, OSError):
            self._link_lost()
            return False
        return True

    def _read_available(self):
        with self._lock:
            ser = self._ser
        if ser is None:
            return []
        try:
            waiting = ser.in_waiting
            return self._decoder.feed(ser.read(waiting)) if waiting else []
        except (serial.SerialException, OSError):
            self._link_lost()
            return []

    def _read_reply(self, deadline):
        """Yield decoded replies as they arrive before the deadline."""
        while self._ser is not None and time.monotonic() < deadline:
            frames = self._read_available()
            if frames:
                yield from frames
            else:
                time.sleep(0.002)

    def _link_lost(self):
        with self._lock:
            self._connected = False
            self._clear_motion()
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
        next_command = next_poll = 0.0
        pending = None
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
                    self._closing.wait(RECONNECT_DELAY_S)
                    continue
                next_command = next_poll = 0.0
                pending = None
            first_failure = None
            with self._lock:
                benchmark, robot_id = self._benchmark, self._robot_id
                last_rx = self._last_rx_t
            if benchmark is not None:
                if not last_rx or self._send_frame(encode_command(robot_id)):
                    try:
                        benchmark.run(self._ser, self._closing)
                    except (OSError, ValueError):
                        self._link_lost()
                else:
                    benchmark.fail("A ponte ESP-NOW ficou indisponível.")
                self._decoder.reset()
                with self._lock:
                    self._last_rx_t = 0.0
                    self._benchmark = None
                next_command = next_poll = 0.0
                pending = None
                continue

            now = time.monotonic()
            if now >= next_command:
                next_command += COMMAND_PERIOD_S
                if next_command <= now:
                    next_command = now + COMMAND_PERIOD_S
                frame = self._next_frame()
                if frame is not None:
                    if not self._send_frame(frame):
                        continue
                    with self._lock:
                        self._tx_count += 1

            if pending is not None and (pending[0] != robot_id or now - pending[2] >= REPLY_TIMEOUT_S):
                pending = None
            if pending is None and now >= next_poll:
                stamp = _timestamp_ms()
                sent = time.monotonic()
                if not self._send_frame(encode_telemetry_request(robot_id, stamp)):
                    continue
                pending = robot_id, stamp, sent
                live = last_rx and now - last_rx <= LIVE_STALE_S
                next_poll = sent + (TELEMETRY_PERIOD_S if live else IDLE_TELEMETRY_PERIOD_S)
                with self._lock:
                    self._tx_count += 1

            for telemetry in self._read_available():
                if pending is None or (telemetry["robot_id"], telemetry["timestamp"]) != pending[:2]:
                    continue
                received = time.monotonic()
                sent = pending[2]
                rtt_s = received - sent
                pending = None
                with self._lock:
                    if rtt_s <= REPLY_TIMEOUT_S and telemetry["robot_id"] == self._robot_id:
                        telemetry["rtt_ms"] = rtt_s * 1000
                        self._telemetry = telemetry
                        self._last_rx_t = received
                        self._rx_count += 1
                        self._last_rtt_ms = telemetry["rtt_ms"]
                        next_poll = sent + TELEMETRY_PERIOD_S
            self._closing.wait(max(0, min(0.002, next_command - time.monotonic())))
        with self._lock:
            robot_id = self._robot_id
            had_link = bool(self._last_rx_t)
        if had_link:
            self._send_frame(encode_command(robot_id), flush=True)


def discover_robots(port, timeout_s=DISCOVERY_TIMEOUT_S):
    """Open the adapter and list robots that answer a telemetry request."""
    session = RadioSession.connect(port)
    try:
        wait_for_gateway(session._ser, session._closing.is_set)
        found = session.discover(timeout_s)
        if session._ser is None:
            raise RadioError("A conexão USB com a ponte foi perdida durante a busca.")
        return found
    except OSError as error:
        raise RadioError(str(error)) from error
    finally:
        session.close()
