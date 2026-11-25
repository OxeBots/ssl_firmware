"""Local HTTP API exposing the running dashboard to other programs and agents.

The dashboard owns the serial port, so it is the only process that talks to the
robot. This module serves its shared BenchState over HTTP: reads return the
latest telemetry, writes call the same commands the GUI controls call, so a
human can keep using the window while an agent drives the robot in parallel.

Two local listeners are enabled by default: loopback TCP for processes in the
same machine or container, and a Unix socket inside the repository (shared by
the devcontainer bind mount), so a host-side agent reaches a dashboard running
inside the container. There is no authentication; keep the listeners local.
"""

import ipaddress
import json
import math
import os
import socket
import socketserver
import stat
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import config
from . import protocol as proto

API_VERSION = 1

# Repository root: the workspace is bind-mounted into the devcontainer, so a
# Unix socket below it is reachable from the host and from the container.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_SOCKET_PATH = os.path.join(REPO_ROOT, "tmp", "robot_bench_api.sock")

# Command bodies are small JSON objects; anything larger is a client bug.
MAX_BODY_BYTES = 64 * 1024
DEFAULT_CONSOLE_LINES = 100
MAX_CONSOLE_LINES = 1000

# The API answers this machine and the local network only; the web is refused.
LOCAL_NETWORKS = tuple(
    ipaddress.ip_network(cidr)
    for cidr in (
        "127.0.0.0/8",
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
        "169.254.0.0/16",
        "::1/128",
        "fc00::/7",
        "fe80::/10",
    )
)

CALIBRATION_STATE_NAMES = ("idle", "running", "done", "failed", "partial")
CALIBRATION_ERROR_NAMES = (
    "none",
    "no_movement",
    "ambiguous_analog",
    "ambiguous_i2c",
    "bad_mapping",
    "aborted",
    "i2c_error",
)
ENCODER_STAGE_NAMES = ("full", "reduced", "pwm", "unknown")
TUNING_STATE_NAMES = ("idle", "running", "done", "failed")
TUNING_PHASE_NAMES = (
    "idle",
    "friction",
    "identify",
    "validate",
    "refine",
    "store",
    "done",
    "max_speed",
)
TUNING_ERROR_NAMES = (
    "none", "no_movement", "unstable", "timeout", "aborted", "speed_mismatch", "save_failed",
)
POSITION_ERROR_NAMES = ("none", "timeout", "aborted", "encoder")

_ENDPOINTS = (
    {
        "method": "GET",
        "path": "/api/status",
        "description": "link, counters, dashboard state and the calibrated RPM limit",
    },
    {
        "method": "GET",
        "path": "/api/state",
        "description": "wheel PWM/RPM/target/angle and FG pulse feedback",
    },
    {"method": "GET", "path": "/api/health", "description": "safety lifecycle, faults, supply and heap"},
    {"method": "GET", "path": "/api/tof", "description": "8x8 distance grid in millimetres and statuses"},
    {"method": "GET", "path": "/api/imu", "description": "acceleration (g), rotation (deg/s) and field (mG)"},
    {
        "method": "GET",
        "path": "/api/calibration",
        "description": "wiring calibration frame with maps, encoder operation modes and stored ADC endpoints",
    },
    {
        "method": "GET",
        "path": "/api/tuning",
        "description": "control calibration frame, per-direction parameters/metrics and the RPM limit",
    },
    {"method": "GET", "path": "/api/position", "description": "wheel position control frame"},
    {
        "method": "GET",
        "path": "/api/raw",
        "description": "raw ADC/I2C encoder values, FG pulse deltas, EKF RPM and applied PWM",
    },
    {
        "method": "GET",
        "path": "/api/console?limit=100&since=0",
        "description": "console lines; since returns only the newer ones",
    },
    {"method": "GET", "path": "/api/snapshot?console=100", "description": "status plus every frame in one call"},
    {
        "method": "POST",
        "path": "/api/targets",
        "body": '{"targets_rpm": [0, 0, 0, 0]} or {"wheel": 0, "rpm": 500}',
        "description": "set the wheel targets (RPM), clamped to the calibrated limit",
    },
    {"method": "POST", "path": "/api/stop", "body": "{}", "description": "zero every wheel target"},
    {
        "method": "POST",
        "path": "/api/sweep",
        "body": '{"enabled": true}',
        "description": "dashboard square sweep on the selected wheel",
    },
    {
        "method": "POST",
        "path": "/api/raw_encoder",
        "body": '{"wheel": 0} or {"wheel": null}',
        "description": "focus the raw I2C read on one wheel's encoder through the mux",
    },
    {
        "method": "POST",
        "path": "/api/calibration",
        "body": '{"action": "start", "confirm": true} or {"action": "stop"}',
        "description": "motor/encoder wiring calibration",
    },
    {
        "method": "POST",
        "path": "/api/tuning",
        "body": '{"action": "start", "confirm": true} or {"action": "stop"}',
        "description": "wheel control calibration",
    },
    {
        "method": "POST",
        "path": "/api/position",
        "body": (
            '{"action": "move", "moves": [{"wheel": 0, "angle_deg": 45}], '
            '"absolute": false, "max_rpm": 300} or {"action": "stop"}'
        ),
        "description": "wheel position moves",
    },
)


class _ApiError(Exception):
    """Request failure that carries its HTTP status code."""

    def __init__(self, status, error):
        super().__init__(error)
        self.status = status


def client_allowed(client_address):
    """True for this machine and local-network peers only."""
    if not client_address:
        return False
    host = client_address[0] or ""
    if host == "unix":
        return True  # the socket file mode already limits this to the dashboard user
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return False
    return any(address in network for network in LOCAL_NETWORKS)


def _host_is_local(host):
    """True when the listener is bound to this machine only."""
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# Payload builders
# ---------------------------------------------------------------------------


def _name(names, code):
    """Label for one wire enum code; unknown codes stay explicit."""
    if 0 <= code < len(names):
        return names[code]
    return f"unknown({code})"


def _frame(timestamp, data, stale_s):
    """Wrap one telemetry frame with its freshness window (data is None until seen)."""
    if not timestamp:
        return {"live": False, "age_s": None, "data": None}
    age = time.monotonic() - timestamp
    return {"live": age <= stale_s, "age_s": round(age, 3), "data": data}


def _tab_title(index):
    if 0 <= index < len(config.TAB_TITLES):
        return config.TAB_TITLES[index]
    return None


def _status(state):
    """Link, counters and dashboard options."""
    age = time.monotonic() - state.last_state_t if state.last_state_t else None
    position = state.position if state.position_live() else None
    return {
        "demo": state.demo,
        "connected": state.connected,
        "streaming": state.streaming,
        "live": state.link_fresh(),
        "state_age_s": None if age is None else round(age, 3),
        "uptime_ms": state.uptime_ms,
        "rx_bytes": state.rx_bytes,
        "counters": {
            "state": state.state_count,
            "imu": state.imu_count,
            "tof": state.tof_count,
            "health": state.health_count,
            "calibration": state.calibration_count,
            "tuning": state.tuning_count,
            "position": state.position_count,
            "raw": state.raw_count,
            "console": state.console_total,
        },
        "active_tab": state.active_tab,
        "active_tab_title": _tab_title(state.active_tab),
        "selected_wheel": state.selected_wheel,
        "max_rpm": state.max_rpm,
        "max_rpm_calibrated": state.max_rpm_calibrated,
        "wheel_max_rpm": list(state.wheel_max_rpm),
        "raw_encoder_wheel": state.raw_encoder_wheel,
        "sweep": state.sweep,
        "calibration": {"live": state.calibration_live(), "running": state.calibration_running()},
        "tuning": {"live": state.tuning_live(), "running": state.tuning_running()},
        "position": {"live": position is not None, "moving": bool(position and position["active_mask"])},
    }


def _state_data(state):
    """Latest wheel telemetry."""
    return {
        "uptime_ms": state.state_uptime_ms,
        "valid_mask": state.valid_mask,
        "pulse_valid_mask": state.pulse_valid_mask,
        "wheels": [
            {
                "pwm": wheel["pwm"],
                "rpm": wheel["vel"],
                "target_rpm": wheel["tgt"],
                "angle_deg": wheel["angle"],
                "pulse_rpm": wheel["pulse_rpm"],
                "pulse_count": wheel["pulse_count"],
                "calibrated": bool(state.valid_mask & (1 << index)),
            }
            for index, wheel in enumerate(state.wheels)
        ],
    }


def _raw_data(state):
    """Raw encoder and motor-feedback frame as parsed."""
    return dict(state.raw) if state.raw else None


def _tof_data(state):
    """Wheel-free 8x8 grid of distances and statuses."""
    side = config.TOF_SIDE
    return {
        "side": side,
        "distances_mm": [state.tof[row * side : (row + 1) * side] for row in range(side)],
        "statuses": [state.tof_status[row * side : (row + 1) * side] for row in range(side)],
    }


def _imu_data(state):
    return {
        "accel_g": list(state.accel),
        "gyro_dps": list(state.gyro),
        "mag_mg": list(state.mag),
        "yaw": state.yaw,
        "pitch": state.pitch,
        "roll": state.roll,
    }


def _calibration_data(frame):
    if frame is None:
        return None
    data = dict(frame)
    data["state_name"] = _name(CALIBRATION_STATE_NAMES, frame["state"])
    data["error_name"] = _name(CALIBRATION_ERROR_NAMES, frame["error_code"])
    data["wheel_error_names"] = [_name(CALIBRATION_ERROR_NAMES, code) for code in frame["wheel_error"]]
    data["stage_names"] = [_name(ENCODER_STAGE_NAMES, stage) for stage in frame["encoder_stage"]]
    return data


def _tuning_data(frame):
    if frame is None:
        return None
    data = dict(frame)
    data["state_name"] = _name(TUNING_STATE_NAMES, frame["state"])
    data["phase_name"] = _name(TUNING_PHASE_NAMES, frame["phase"])
    data["error_name"] = _name(TUNING_ERROR_NAMES, frame["error_code"])
    return data


def _position_data(frame):
    if frame is None:
        return None
    data = dict(frame)
    data["error_name"] = _name(POSITION_ERROR_NAMES, frame["error_code"])
    return data


def _console_snapshot(state, limit=DEFAULT_CONSOLE_LINES, since=None):
    """Console tail, or the lines after an absolute index, plus the next cursor."""
    total = state.console_total
    lines = list(state.console)
    first = max(0, total - len(lines))
    if since is None:
        start = max(first, total - limit)
    else:
        start = min(max(since, first), total)
    selected = lines[start - first : start - first + limit]
    return {"since": start, "next_since": start + len(selected), "total": total, "lines": selected}


def _console_since(state, since):
    """Console lines appended after an absolute index."""
    lines = list(state.console)
    first = max(0, state.console_total - len(lines))
    start = max(0, min(since - first, len(lines)))
    return lines[start:]


def _last_message(messages):
    """Last dashboard console line, without the command prefix."""
    for line in reversed(messages):
        text = line[2:] if line.startswith("> ") else line
        if text:
            return text
    return ""


def _snapshot(state, limit):
    """Every frame and the console tail in one response."""
    return {
        "status": _status(state),
        "state": _frame(state.last_state_t, _state_data(state), config.SENSOR_STALE_S),
        "health": _frame(
            state.last_health_t, dict(state.health) if state.health else None, config.HEALTH_STALE_S
        ),
        "tof": _frame(state.last_tof_t, _tof_data(state), config.SENSOR_STALE_S),
        "imu": _frame(state.last_imu_t, _imu_data(state), config.SENSOR_STALE_S),
        "calibration": _frame(
            state.last_calibration_t, _calibration_data(state.calibration), config.CALIBRATION_STALE_S
        ),
        "tuning": _frame(state.last_tuning_t, _tuning_data(state.tuning), config.TUNING_STALE_S),
        "position": _frame(state.last_position_t, _position_data(state.position), config.POSITION_STALE_S),
        "raw": _frame(state.last_raw_t, _raw_data(state), config.RAW_STALE_S),
        "console": _console_snapshot(state, limit),
    }


def _index():
    return {
        "api": "robot_bench",
        "version": API_VERSION,
        "description": (
            "Local API of the running Robot Bench dashboard. The dashboard owns the serial port; "
            "these endpoints read its telemetry and run the same commands as the GUI."
        ),
        "endpoints": _ENDPOINTS,
        "notes": [
            "Read endpoints answer {ok, live, age_s, data}; data is null before the first frame.",
            "Errors answer {ok: false, error} with HTTP 400, 403, 404, 405, 409, 413 or 500.",
            "Only this machine and the local network are accepted; web clients get HTTP 403.",
            "A calibration start needs {action: start, confirm: true} because the wheels spin.",
            "Wheel targets are refused while a calibration runs; follow GET /api/console.",
        ],
    }


# ---------------------------------------------------------------------------
# Command bodies
# ---------------------------------------------------------------------------


def _is_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except OverflowError:
        return False


def _require_bool(body, key):
    value = body.get(key)
    if not isinstance(value, bool):
        raise _ApiError(400, f"'{key}' must be true or false")
    return value


def _execute(state, method, *args):
    """Run one dashboard command; a False return means the dashboard refused it."""
    since = state.console_total
    accepted = method(*args)
    messages = _console_since(state, since)
    if accepted is False:
        raise _ApiError(409, _last_message(messages) or "command rejected by the dashboard")
    return {"messages": messages}


def _targets_command(state, body):
    if "targets_rpm" in body:
        values = body["targets_rpm"]
        if not isinstance(values, list) or len(values) != proto.WHEEL_COUNT:
            raise _ApiError(400, f"'targets_rpm' must be a list of {proto.WHEEL_COUNT} numbers")
        if not all(_is_number(value) for value in values):
            raise _ApiError(400, "'targets_rpm' must contain numbers only")
        targets = [float(value) for value in values]
    elif "wheel" in body or "rpm" in body:
        wheel = body.get("wheel")
        rpm = body.get("rpm")
        if not isinstance(wheel, int) or isinstance(wheel, bool) or not 0 <= wheel < proto.WHEEL_COUNT:
            raise _ApiError(400, f"'wheel' must be an integer between 0 and {proto.WHEEL_COUNT - 1}")
        if not _is_number(rpm):
            raise _ApiError(400, "'rpm' must be a number")
        current = state.manual_targets if state.demo else [item["tgt"] for item in state.wheels]
        targets = [float(value) for value in current]
        targets[wheel] = float(rpm)
    else:
        raise _ApiError(400, "send 'targets_rpm' or 'wheel' and 'rpm'")
    result = _execute(state, state.set_targets, targets)
    result["max_rpm"] = state.max_rpm
    result["targets_rpm"] = [max(-state.max_rpm, min(state.max_rpm, value)) for value in targets]
    return result


def _raw_encoder_command(state, body):
    if "wheel" not in body:
        raise _ApiError(400, "send 'wheel' (0..3) or 'wheel': null")
    wheel = body.get("wheel")
    if wheel is not None:
        if not isinstance(wheel, int) or isinstance(wheel, bool) or not 0 <= wheel < proto.WHEEL_COUNT:
            raise _ApiError(400, f"'wheel' must be an integer between 0 and {proto.WHEEL_COUNT - 1}, or null")
    state.select_raw_encoder(wheel)
    return {"raw_encoder_wheel": state.raw_encoder_wheel}


def _sweep_command(state, body):
    enabled = _require_bool(body, "enabled")
    if enabled != state.sweep:
        state.sweep = enabled
        state.add_console("Varredura " + ("ligada" if enabled else "desligada"))
    return {"sweep": state.sweep}


def _calibration_command(state, body):
    action = body.get("action")
    if action == "start":
        if body.get("confirm") is not True:
            raise _ApiError(409, "confirm the robot is lifted with 'confirm': true")
        state.set_calibration_confirm(True)
        return _execute(state, state.start_calibration)
    if action == "stop":
        return _execute(state, state.stop_calibration)
    raise _ApiError(400, "'action' must be 'start' or 'stop'")


def _tuning_command(state, body):
    action = body.get("action")
    if action == "start":
        if body.get("confirm") is not True:
            raise _ApiError(409, "confirm the robot is lifted with 'confirm': true")
        state.set_tuning_confirm(True)
        return _execute(state, state.start_tuning)
    if action == "stop":
        return _execute(state, state.stop_tuning)
    raise _ApiError(400, "'action' must be 'start' or 'stop'")


def _position_command(state, body):
    action = body.get("action")
    if action == "stop":
        return _execute(state, state.stop_position)
    if action != "move":
        raise _ApiError(400, "'action' must be 'move' or 'stop'")
    moves = body.get("moves")
    if not isinstance(moves, list) or not moves:
        raise _ApiError(400, "'moves' must be a non-empty list")
    angles = [0.0] * proto.WHEEL_COUNT
    wheel_mask = 0
    for move in moves:
        if not isinstance(move, dict):
            raise _ApiError(400, "each 'moves' item must be {wheel, angle_deg}")
        wheel = move.get("wheel")
        angle = move.get("angle_deg")
        if not isinstance(wheel, int) or isinstance(wheel, bool) or not 0 <= wheel < proto.WHEEL_COUNT:
            raise _ApiError(400, f"'wheel' must be an integer between 0 and {proto.WHEEL_COUNT - 1}")
        if not _is_number(angle) or abs(angle) > config.POSITION_LIMIT_DEG:
            limit = config.POSITION_LIMIT_DEG
            raise _ApiError(400, f"'angle_deg' must be a number between -{limit:.0f} and {limit:.0f}")
        angles[wheel] = float(angle)
        wheel_mask |= 1 << wheel
    absolute = body.get("absolute", False)
    if not isinstance(absolute, bool):
        raise _ApiError(400, "'absolute' must be true or false")
    max_rpm = body.get("max_rpm", 0)
    if not _is_number(max_rpm) or not 0 <= max_rpm <= 65535:
        raise _ApiError(400, "'max_rpm' must be a number between 0 and 65535")
    result = _execute(state, state.start_position_move, angles, wheel_mask, absolute, int(max_rpm))
    result["moves"] = [
        {"wheel": wheel, "angle_deg": angles[wheel]}
        for wheel in range(proto.WHEEL_COUNT)
        if wheel_mask & (1 << wheel)
    ]
    result["absolute"] = absolute
    result["max_rpm"] = int(max_rpm)
    return result


# ---------------------------------------------------------------------------
# HTTP layer
# ---------------------------------------------------------------------------


def _query_int(query, name, default, minimum, maximum):
    values = query.get(name)
    if not values:
        return default
    try:
        value = int(values[0])
    except ValueError:
        raise _ApiError(400, f"'{name}' must be an integer")
    if not minimum <= value <= maximum:
        raise _ApiError(400, f"'{name}' must be between {minimum} and {maximum}")
    return value


class _Handler(BaseHTTPRequestHandler):
    """Serves one request against the shared dashboard state."""

    protocol_version = "HTTP/1.1"
    server_version = "RobotBenchAPI"

    def __init__(self, state, request, client_address, server):
        self.state = state
        super().__init__(request, client_address, server)

    def log_message(self, format, *args):
        """Keep the dashboard console free of one line per API request."""

    def _send(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        if self.close_connection:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _allowed_client(self):
        """Refuse web clients: browsers, reverse proxies and public peers cannot use the API."""
        browser = self.headers.get("Origin") or self.headers.get("Sec-Fetch-Site")
        proxy = self.headers.get("X-Forwarded-For") or self.headers.get("Forwarded")
        if client_allowed(self.client_address) and not browser and not proxy:
            return True
        self.close_connection = True
        self._error(403, "local clients only: the API is not reachable from the web")
        return False

    def _ok(self, payload):
        self._send(200, {"ok": True, **payload})

    def _error(self, status, error):
        self._send(status, {"ok": False, "error": error})

    def _read_json(self):
        length = self.headers.get("Content-Length")
        if not length:
            return {}
        try:
            size = int(length)
        except ValueError:
            self.close_connection = True
            raise _ApiError(400, "invalid Content-Length")
        if size > MAX_BODY_BYTES:
            self.close_connection = True
            raise _ApiError(413, f"body larger than {MAX_BODY_BYTES} bytes")
        raw = self.rfile.read(size)
        if not raw:
            return {}
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise _ApiError(400, "body must be UTF-8 JSON")
        if not isinstance(body, dict):
            raise _ApiError(400, "body must be a JSON object")
        return body

    def do_GET(self):
        if not self._allowed_client():
            return
        state = self.state
        try:
            url = urlparse(self.path)
            path = url.path.rstrip("/") or "/"
            query = parse_qs(url.query)
            if path == "/":
                self._ok(_index())
            elif path == "/api/status":
                self._ok(_status(state))
            elif path == "/api/snapshot":
                self._ok(_snapshot(state, _query_int(query, "console", DEFAULT_CONSOLE_LINES, 0, MAX_CONSOLE_LINES)))
            elif path == "/api/console":
                limit = _query_int(query, "limit", DEFAULT_CONSOLE_LINES, 0, MAX_CONSOLE_LINES)
                since = _query_int(query, "since", None, 0, 1 << 30)
                self._ok(_console_snapshot(state, limit, since))
            elif path == "/api/state":
                self._ok(_frame(state.last_state_t, _state_data(state), config.SENSOR_STALE_S))
            elif path == "/api/health":
                health = dict(state.health) if state.health else None
                self._ok(_frame(state.last_health_t, health, config.HEALTH_STALE_S))
            elif path == "/api/tof":
                self._ok(_frame(state.last_tof_t, _tof_data(state), config.SENSOR_STALE_S))
            elif path == "/api/imu":
                self._ok(_frame(state.last_imu_t, _imu_data(state), config.SENSOR_STALE_S))
            elif path == "/api/calibration":
                self._ok(
                    _frame(state.last_calibration_t, _calibration_data(state.calibration), config.CALIBRATION_STALE_S)
                )
            elif path == "/api/tuning":
                self._ok(_frame(state.last_tuning_t, _tuning_data(state.tuning), config.TUNING_STALE_S))
            elif path == "/api/position":
                self._ok(_frame(state.last_position_t, _position_data(state.position), config.POSITION_STALE_S))
            elif path == "/api/raw":
                self._ok(_frame(state.last_raw_t, _raw_data(state), config.RAW_STALE_S))
            else:
                self._error(404, f"unknown endpoint: {path}; see GET /")
        except _ApiError as error:
            self._error(error.status, str(error))
        except Exception as error:  # never leave a client without a JSON answer
            self.state.add_console(f"API: falha interna em {self.path}: {error}")
            self._error(500, f"internal error: {error}")

    def do_POST(self):
        if not self._allowed_client():
            return
        state = self.state
        try:
            body = self._read_json()
            path = urlparse(self.path).path.rstrip("/")
            if path == "/api/targets":
                self._ok(_targets_command(state, body))
            elif path == "/api/stop":
                self._ok(_execute(state, state.stop_motors))
            elif path == "/api/sweep":
                self._ok(_sweep_command(state, body))
            elif path == "/api/raw_encoder":
                self._ok(_raw_encoder_command(state, body))
            elif path == "/api/calibration":
                self._ok(_calibration_command(state, body))
            elif path == "/api/tuning":
                self._ok(_tuning_command(state, body))
            elif path == "/api/position":
                self._ok(_position_command(state, body))
            else:
                self._error(404, f"unknown endpoint: {path}; see GET /")
        except _ApiError as error:
            self._error(error.status, str(error))
        except Exception as error:  # never leave a client without a JSON answer
            self.state.add_console(f"API: falha interna em {self.path}: {error}")
            self._error(500, f"internal error: {error}")

    def do_PUT(self):
        if not self._allowed_client():
            return
        # The body of an unsupported method is not read; do not reuse the socket.
        self.close_connection = True
        self._error(405, "use GET or POST; see GET /")

    do_DELETE = do_PUT
    do_PATCH = do_PUT


def _socket_is_live(path):
    """True when another process answers on this socket path."""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        client.settimeout(0.2)
        client.connect(path)
    except OSError:
        return False
    finally:
        client.close()
    return True


class _UnixHTTPServer(socketserver.ThreadingUnixStreamServer):
    """HTTP server over an AF_UNIX socket; BaseHTTPRequestHandler needs an address tuple."""

    daemon_threads = True

    def get_request(self):
        request, _address = super().get_request()
        return request, ("unix", 0)


class ApiServer:
    """HTTP listeners that expose one running dashboard to local clients."""

    def __init__(self, state):
        self.state = state
        self.port = 0
        self.socket_path = None
        self._servers = []

    def start(self, host=DEFAULT_HOST, port=DEFAULT_PORT, socket_path=DEFAULT_SOCKET_PATH):
        """Bind the listeners; a busy or invalid address only disables that listener."""
        if port is not None:
            try:
                server = ThreadingHTTPServer((host, port), partial(_Handler, self.state))
            except OSError as error:
                self.state.add_console(f"API HTTP indisponível em {host}:{port}: {error}")
            else:
                self.port = server.server_address[1]
                self._servers.append(server)
                shown_host = "*" if host in ("0.0.0.0", "::") else host
                scope = "" if _host_is_local(host) else " (rede local apenas; web bloqueada)"
                self.state.add_console(f"API local: http://{shown_host}:{self.port}{scope}")
        if socket_path:
            path = os.path.abspath(socket_path)
            if os.path.lexists(path) and _socket_is_live(path):
                # Never steal the socket of a dashboard that is still running.
                self.state.add_console(f"API local (socket) em uso por outra instância: {path}")
            else:
                try:
                    directory = os.path.dirname(path)
                    if directory:
                        os.makedirs(directory, exist_ok=True)
                    try:
                        stale = stat.S_ISSOCK(os.lstat(path).st_mode)
                    except FileNotFoundError:
                        stale = False
                    if stale:
                        os.unlink(path)  # socket left behind by a crashed session
                    server = _UnixHTTPServer(path, partial(_Handler, self.state))
                    try:
                        os.chmod(path, 0o600)  # only the dashboard user may connect
                    except OSError:
                        pass  # without the write bit for others, the umask still gates it
                except OSError as error:
                    self.state.add_console(f"API local indisponível no socket {path}: {error}")
                else:
                    self._servers.append(server)
                    self.socket_path = path
                    self.state.add_console(f"API local (socket): {path}")
        for server in self._servers:
            # A short poll interval only makes stop() responsive; requests are
            # handled as soon as they arrive.
            thread = threading.Thread(
                target=server.serve_forever, kwargs={"poll_interval": 0.1}, name="bench-api", daemon=True
            )
            thread.start()
        return self

    def stop(self):
        """Stop both listeners and remove the socket file."""
        for server in self._servers:
            server.shutdown()
            server.server_close()
        self._servers = []
        if self.socket_path:
            try:
                os.unlink(self.socket_path)
            except OSError:
                pass
            self.socket_path = None
