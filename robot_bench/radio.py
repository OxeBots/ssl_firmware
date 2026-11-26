"""ESP32 Base Station radio link and robot radio protocol codec.

The bench reaches robots over radio through the USB-connected ESP32 Base Station
(proto/base_station_protocol.bitproto, COBS+CRC8 @ 921600 baud). Each robot uses
its own hardware unicast RF address with Auto-ACK, and broadcast frames use
W_TX_PAYLOAD_NO_ACK.
"""

import collections
import math
import struct
import threading
import time

import serial
import serial.tools.list_ports

from . import base_station

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
CONFIG_FLAG_RESET_NVS = 8
CONFIG_FLAG_FACTORY_RESET = 16
CONFIG_FLAG_SET_CHANNEL = 32

COMMAND_SIZE = 30
CONFIG_SIZE = 8
TELEMETRY_SIZE = 22

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

BAUD = base_station.DEFAULT_BAUD
FRAME_GAP_S = 0.015
REPLY_TIMEOUT_S = 0.030
DISCOVERY_TIMEOUT_S = 0.045
KEYBOARD_LEASE_S = 0.20
KEYBOARD_LINK_TIMEOUT_S = 0.25
KEYBOARD_MIN_RPM = 50
WHEEL_RADIUS_MM = 32.5
WHEEL_DISTANCE_MM = 96.0
KEYBOARD_MAX_RPM = int(8.0 * WHEEL_DISTANCE_MM * 60 / (math.tau * WHEEL_RADIUS_MM))
LIVE_STALE_S = 1.0
RECONNECT_DELAY_S = 1.0
PICKER_FALLBACK_S = 8.0


def _timestamp_ms():
    return int(time.time() * 1000) & 0xFFFFFFFF


def robot_id_valid(robot_id):
    """True for a unicast robot id the protocol accepts as a command target."""
    return 0 <= robot_id <= ROBOT_ID_MAX


def find_adapters(baud=BAUD):
    """Probe USB serial ports and return attached ESP32 Base Stations as (device, detail)."""
    adapters = []
    for port in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
        dev = port.device
        if not any(tok in dev for tok in ("ttyUSB", "ttyACM", "usbserial", "usbmodem", "COM")):
            continue
        try:
            info = base_station.probe_base_station(dev, baud=baud)
        except (serial.SerialException, OSError):
            continue
        if info is not None:
            rate = base_station.RATE_NAMES.get(info["data_rate"], "?")
            adapters.append((dev, f"ESP32 Base Station · ch {info['channel']} · {rate}"))
    return adapters


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
        speed = math.ceil(math.sqrt(2) * wheel_mm_s / axes)
        return x * speed, y * speed, 0
    return 0, 0, 0


def encode_command(robot_id, x=0, y=0, angle=0, vx=0, vy=0, w=0, kick=0, timestamp=None):
    """Build one RobotCommand message (30 bytes) for the radio link."""
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


class RadioError(Exception):
    """The ESP32 Base Station could not be opened or configured."""


class RadioSession(threading.Thread):
    """Streams robot commands over the ESP32 Base Station and collects telemetry replies."""

    def __init__(self, port, robot_id=0, baud=BAUD):
        super().__init__(daemon=True, name="radio-link")
        self.port = port
        self.baud = baud
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._link = None
        self._base_info = None
        self._scan_result = None
        self._pending_scan = False
        self._pending_channel_switch = None
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
        self._tx_ack_ok = 0
        self._tx_retries_sum = 0
        self._rx_count = 0
        self._connected = False
        self._error = ""
        self._request_picker = False
        self._messages = collections.deque(maxlen=60)

    @classmethod
    def connect(cls, port, robot_id=0, baud=BAUD):
        """Open and handshake the ESP32 Base Station; raises RadioError on failure."""
        session = cls(port, robot_id, baud=baud)
        try:
            session._open()
        except (serial.SerialException, OSError, RadioError, base_station.BaseStationError) as exc:
            session._close_serial()
            raise RadioError(str(exc)) from exc
        ch = session._base_info["channel"] if session._base_info else "?"
        session._log(f"base station conectada em {port} · ch {ch} · robô {session._robot_id}")
        return session

    def _open(self):
        link = base_station.BaseStationLink(self.port, baud=self.baud)
        info = link.open()
        self._link = link
        self._base_info = info
        self._connected = True

    def _close_serial(self):
        with self._lock:
            link, self._link = self._link, None
        if link is not None:
            link.close()

    def _log(self, message):
        self._messages.append(message)
        print(message, flush=True)

    def set_motion(self, x=None, y=None, angle=None, vx=None, vy=None, w=None):
        values = {"x": x, "y": y, "angle": angle, "vx": vx, "vy": vy, "w": w}
        with self._lock:
            self._keyboard_until = 0.0
            for field, value in values.items():
                if value is not None:
                    self._command[field] = int(value)

    def keyboard_link_ready(self):
        with self._lock:
            return self._connected and 0 < self._last_rx_t and (
                time.monotonic() - self._last_rx_t < KEYBOARD_LINK_TIMEOUT_S
            )

    def set_keyboard_velocity(self, velocity):
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
        with self._lock:
            self._command = {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0}
            self._keyboard_until = 0.0
            self._pending_kick = 0

    def kick(self, strength):
        with self._lock:
            self._pending_kick = max(0, min(255, int(strength)))

    def set_robot_id(self, robot_id):
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
        with self._lock:
            self._configs.append((int(config_flags), int(param)))
        self._log(f"rádio: config 0x{int(config_flags):02X} param {int(param)}")

    def request_scan(self):
        """Queue an RF spectrum RPD scan (0..125) on the Base Station thread."""
        with self._lock:
            self._pending_scan = True
        self._log("base: varredura de espectro solicitada...")

    def switch_channel(self, channel, notify_robots=True):
        """Queue a coordinated RF channel change (robots + Base Station NVS)."""
        ch = max(0, min(125, int(channel)))
        with self._lock:
            self._pending_channel_switch = (ch, bool(notify_robots))
        self._log(f"base: migração para canal {ch} solicitada...")

    def probe(self, robot_id, timeout_s=DISCOVERY_TIMEOUT_S):
        """Request telemetry from robot_id via the Base Station and return reply dict."""
        expected_timestamp = _timestamp_ms()
        frame = encode_telemetry_request(robot_id, expected_timestamp)
        with self._lock:
            link = self._link
        if link is None:
            return None
        try:
            link.clear_rx()
            sent = time.monotonic()
            tx_st = link.send_rf(
                robot_id,
                frame,
                tx_flags=base_station.BS_TX_FLAG_EXPECT_REPLY,
                timeout_s=min(timeout_s, 0.025),
            )
            with self._lock:
                self._tx_count += 1
                if tx_st is not None and tx_st["status"] == base_station.BS_TX_STATUS_OK:
                    self._tx_ack_ok += 1
                    self._tx_retries_sum += tx_st["retries"]
            if tx_st is None or tx_st["status"] != base_station.BS_TX_STATUS_OK:
                return None

            deadline = sent + timeout_s
            while time.monotonic() < deadline:
                rx = link.wait_rf_rx(timeout_s=max(0.002, deadline - time.monotonic()))
                if rx is None:
                    break
                telemetry = decode_telemetry(rx["payload"])
                if (
                    telemetry is not None
                    and telemetry["robot_id"] == robot_id
                    and telemetry["timestamp"] == expected_timestamp
                ):
                    telemetry["rtt_ms"] = (time.monotonic() - sent) * 1000.0
                    return telemetry
            return None
        except (serial.SerialException, OSError, base_station.BaseStationError):
            self._link_lost()
            return None

    def discover(self, timeout_s=DISCOVERY_TIMEOUT_S):
        """Probe every robot id (0..ROBOT_ID_MAX) and return answering robots."""
        found = []
        for robot_id in range(ROBOT_ID_MAX + 1):
            telemetry = self.probe(robot_id, timeout_s)
            if telemetry is not None:
                found.append(telemetry)
        return found

    def snapshot(self):
        with self._lock:
            command = dict(self._command)
            configs = len(self._configs)
            last_rx_t = self._last_rx_t
            ack_rate = (100.0 * self._tx_ack_ok / self._tx_count) if self._tx_count else 0.0
            avg_retries = (self._tx_retries_sum / self._tx_ack_ok) if self._tx_ack_ok else 0.0
            return {
                "port": self.port,
                "connected": self._connected,
                "robot_id": self._robot_id,
                "telemetry": dict(self._telemetry) if self._telemetry else None,
                "rx_age_s": (time.monotonic() - last_rx_t) if last_rx_t else None,
                "rtt_ms": self._last_rtt_ms,
                "tx_count": self._tx_count,
                "tx_ack_rate": ack_rate,
                "avg_retries": avg_retries,
                "rx_count": self._rx_count,
                "base_info": dict(self._base_info) if self._base_info else None,
                "scan_result": dict(self._scan_result) if self._scan_result else None,
                "command": command,
                "pending_configs": configs,
                "error": self._error,
                "messages": list(self._messages),
                "live": bool(last_rx_t) and time.monotonic() - last_rx_t <= LIVE_STALE_S,
            }

    def request_picker(self):
        return self._request_picker

    def close(self):
        self._closing.set()
        if self.ident is not None:
            self.join(timeout=2.0)
        self._close_serial()

    def _next_config(self):
        with self._lock:
            if self._configs:
                return self._configs.popleft()
            return None

    def _next_frame(self):
        config = self._next_config()
        if config is not None:
            flags, param = config
            with self._lock:
                robot_id = self._robot_id
            return robot_id, encode_config(robot_id, flags, param)
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
        return robot_id, encode_command(robot_id, kick=kick, **command)

    def _send_rf(self, target_id, frame, no_ack=False):
        with self._lock:
            link = self._link
        if link is None:
            return False
        wait_s = self._next_tx_at - time.monotonic()
        if wait_s > 0.0:
            time.sleep(wait_s)
        flags = base_station.BS_TX_FLAG_NO_ACK if (no_ack or target_id == ROBOT_ID_BROADCAST) else 0
        try:
            tx_st = link.send_rf(target_id, frame, tx_flags=flags, timeout_s=0.03)
        except (serial.SerialException, OSError, base_station.BaseStationError):
            self._link_lost()
            return False
        self._next_tx_at = time.monotonic() + FRAME_GAP_S
        with self._lock:
            self._tx_count += 1
            if tx_st is not None and tx_st["status"] in (
                base_station.BS_TX_STATUS_OK,
                base_station.BS_TX_STATUS_NO_ACK_SENT,
            ):
                self._tx_ack_ok += 1
                self._tx_retries_sum += tx_st["retries"]
        return tx_st is not None

    def _handle_base_tasks(self):
        with self._lock:
            do_scan = self._pending_scan
            self._pending_scan = False
            ch_switch = self._pending_channel_switch
            self._pending_channel_switch = None
            link = self._link
            info = self._base_info
        if link is None:
            return

        if do_scan:
            try:
                res = link.scan_channels(0, 125, 24, timeout_s=5.0)
                if res is not None:
                    with self._lock:
                        self._scan_result = res
                    self._log(
                        f"base: scan ok · melhor canal = {res['best_channel']} ({2400 + res['best_channel']} MHz)"
                    )
            except (serial.SerialException, OSError, base_station.BaseStationError):
                self._link_lost()
                return

        if ch_switch is not None:
            new_ch, notify = ch_switch
            try:
                if notify:
                    for rid in range(ROBOT_ID_MAX + 1):
                        cfg = encode_config(rid, CONFIG_FLAG_SET_CHANNEL, new_ch)
                        link.send_rf(rid, cfg, timeout_s=0.025)
                    bcast = encode_config(ROBOT_ID_BROADCAST, CONFIG_FLAG_SET_CHANNEL, new_ch)
                    for _ in range(3):
                        link.send_rf(
                            ROBOT_ID_BROADCAST,
                            bcast,
                            tx_flags=base_station.BS_TX_FLAG_NO_ACK,
                            timeout_s=0.025,
                        )
                updated = link.set_config(
                    channel=new_ch,
                    data_rate=info["data_rate"] if info else base_station.BS_RATE_2MBPS,
                    pa_level=info["pa_level"] if info else 3,
                    retransmit_delay=info["retransmit_delay"] if info else 1,
                    retransmit_count=info["retransmit_count"] if info else 3,
                    persist_nvs=True,
                )
                if updated is not None:
                    with self._lock:
                        self._base_info = updated
                    self._log(f"base: canal aplicado = {updated['channel']} ({2400 + updated['channel']} MHz)")
            except (serial.SerialException, OSError, base_station.BaseStationError):
                self._link_lost()

    def _link_lost(self):
        with self._lock:
            self._connected = False
            self._keyboard_until = 0.0
            self._last_rx_t = 0.0
        if not self._error:
            self._log("rádio: conexão com a ESP32 Base Station perdida, reconectando...")
        self._error = "conexão perdida"
        self._close_serial()

    def _reconnect(self):
        try:
            self._open()
        except (serial.SerialException, OSError, RadioError, base_station.BaseStationError):
            self._close_serial()
            return False
        self._error = ""
        self._log(f"base station reconectada em {self.port}")
        return True

    def run(self):
        first_failure = None
        request_telemetry = True
        while not self._closing.is_set():
            if self._link is None:
                now = time.monotonic()
                if first_failure is None:
                    first_failure = now
                elif now - first_failure >= PICKER_FALLBACK_S:
                    self._request_picker = True
                    self._log("rádio: base station indisponível - voltando para a seleção de conexão")
                    return
                if not self._reconnect():
                    time.sleep(RECONNECT_DELAY_S)
                    continue
            first_failure = None
            self._handle_base_tasks()
            if self._link is None:
                continue
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
            else:
                target_id, frame = self._next_frame()
                self._send_rf(target_id, frame)
            request_telemetry = not request_telemetry
            self._closing.wait(max(0.0, self._next_tx_at - time.monotonic()))
        with self._lock:
            robot_id = self._robot_id
        self._send_rf(robot_id, encode_command(robot_id))


def discover_robots(port, timeout_s=DISCOVERY_TIMEOUT_S):
    """Open the ESP32 Base Station and list robots that answer a telemetry request."""
    session = RadioSession.connect(port)
    try:
        return session.discover(timeout_s)
    finally:
        session.close()
