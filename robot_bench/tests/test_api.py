"""Regression checks for the local HTTP API over the dashboard state."""

import http.client
import json
import os
import socket
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from unittest.mock import Mock, patch

from robot_bench import api, config, demo, protocol
from robot_bench.state import BenchState


def state_frame():
    """DebugState payload as the reader parses it."""
    return {
        "uptime_ms": 5000,
        "wheels": [
            {
                "pwm": 100 + index,
                "vel_rpm": 10.0 * index,
                "tgt_rpm": 20.0 * index,
                "angle_deg": 1.5 * index,
                "pulse_rpm": 9.5 * index,
                "pulse_count": 500 * index,
            }
            for index in range(4)
        ],
        "yaw": 1.0,
        "pitch": 2.0,
        "roll": 3.0,
        "valid_mask": 0b0101,
        "pulse_valid_mask": 0b1010,
        "sequence": 1,
    }


def raw_frame(**overrides):
    """DebugRaw payload as the reader parses it."""
    frame = {
        "sequence": 7,
        "analog_raw": [100, 200, 300, 4095],
        "i2c_raw": [10, 20, 0xFFFF, 4095],
        "pulse_delta": [0, 1, 2, 3],
        "ekf_rpm": [-12.5, 0.0, 7.3, 100.0],
        "applied_pwm": [-1023, 0, 512, 1023],
    }
    frame.update(overrides)
    return frame


class _UnixConnection(http.client.HTTPConnection):
    """HTTP connection over an AF_UNIX socket, as an agent on the host uses it."""

    def __init__(self, path):
        super().__init__("localhost")
        self.socket_path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(self.socket_path)


class ApiTestCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.state = BenchState()
        self.state.demo = True
        self.server = api.ApiServer(self.state)
        self.server.start("127.0.0.1", 0, os.path.join(self.directory.name, "api.sock"))
        self.base = f"http://127.0.0.1:{self.server.port}"
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def tearDown(self):
        self.server.stop()
        self.directory.cleanup()

    def request(self, method, path, data=None, headers=None):
        request = urllib.request.Request(self.base + path, data=data, method=method, headers=headers or {})
        try:
            with self.opener.open(request, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def get(self, path):
        return self.request("GET", path)

    def post(self, path, body=None):
        data = None if body is None else json.dumps(body).encode("utf-8")
        return self.request("POST", path, data)

    def live_state(self):
        """Dashboard on a live serial link (commands are not emulated)."""
        self.state.demo = False
        self.state.reader = Mock()
        self.state.connected = "/dev/ttyUSB0"
        self.state.streaming = True
        self.state.last_state_t = time.monotonic()
        return self.state


class IndexAndFrameTests(ApiTestCase):
    def test_index_lists_every_endpoint_and_both_listeners_start(self):
        status, payload = self.get("/")
        self.assertEqual(status, 200)
        self.assertEqual(payload["api"], "robot_bench")
        self.assertEqual(payload["version"], api.API_VERSION)
        paths = {entry["path"].split("?")[0] for entry in payload["endpoints"]}
        for path in (
            "/api/state",
            "/api/health",
            "/api/calibration",
            "/api/targets",
            "/api/position",
            "/api/raw",
            "/api/raw_encoder",
        ):
            self.assertIn(path, paths)
        self.assertNotEqual(self.server.port, 0)
        self.assertTrue(os.path.exists(self.server.socket_path))
        self.assertTrue(any("API local:" in line for line in self.state.console))

    def test_status_reports_the_link_counters_and_dashboard_options(self):
        status, payload = self.get("/api/status")
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertTrue(payload["demo"])
        self.assertFalse(payload["live"])
        self.assertIsNone(payload["state_age_s"])
        self.assertIn("counters", payload)
        self.assertIn("calibration", payload)
        self.assertIn("tuning", payload)
        self.assertIn("position", payload)
        self.assertEqual(payload["max_rpm"], protocol.WHEEL_RPM_MAX)
        self.assertFalse(payload["max_rpm_calibrated"])
        self.assertEqual(payload["wheel_max_rpm"], [0.0] * 4)
        self.state.update_tuning(demo.done_tuning_frame(1))
        status, payload = self.get("/api/status")
        self.assertEqual(payload["max_rpm"], demo.DEMO_MAX_RPM)
        self.assertTrue(payload["max_rpm_calibrated"])
        self.assertEqual(payload["wheel_max_rpm"], list(demo.DEMO_WHEEL_MAX_RPM))

    def test_state_reads_report_wheels_calibration_and_freshness(self):
        self.state.update_state(state_frame())
        status, payload = self.get("/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(payload["live"])
        self.assertLess(payload["age_s"], config.SENSOR_STALE_S)
        data = payload["data"]
        self.assertEqual(data["valid_mask"], 0b0101)
        self.assertEqual(data["wheels"][3]["target_rpm"], 60.0)
        self.assertEqual(data["wheels"][3]["angle_deg"], 4.5)
        self.assertEqual(data["wheels"][3]["pulse_rpm"], 28.5)
        self.assertEqual(data["wheels"][3]["pulse_count"], 1500)
        self.assertEqual(data["pulse_valid_mask"], 0b1010)
        self.assertEqual([wheel["calibrated"] for wheel in data["wheels"]], [True, False, True, False])
        self.state.last_state_t = time.monotonic() - config.SENSOR_STALE_S - 1
        status, payload = self.get("/api/state")
        self.assertFalse(payload["live"])
        self.assertIsNotNone(payload["data"])
        self.state.clear_freshness()
        status, payload = self.get("/api/state")
        self.assertFalse(payload["live"])
        self.assertIsNone(payload["data"])

    def test_raw_reads_report_encoder_and_motor_feedback(self):
        self.state.update_raw(raw_frame())
        status, payload = self.get("/api/raw")
        self.assertEqual(status, 200)
        self.assertTrue(payload["live"])
        self.assertLess(payload["age_s"], config.RAW_STALE_S)
        data = payload["data"]
        self.assertEqual(data["sequence"], 7)
        self.assertEqual(data["analog_raw"], [100, 200, 300, 4095])
        self.assertEqual(data["i2c_raw"][2], 0xFFFF)
        self.assertEqual(data["pulse_delta"], [0, 1, 2, 3])
        self.assertEqual(data["ekf_rpm"], [-12.5, 0.0, 7.3, 100.0])
        self.assertEqual(data["applied_pwm"], [-1023, 0, 512, 1023])
        self.assertEqual(self.state.raw_count, 1)
        self.state.last_raw_t = time.monotonic() - config.RAW_STALE_S - 1
        status, payload = self.get("/api/raw")
        self.assertFalse(payload["live"])
        self.assertIsNotNone(payload["data"])
        self.state.clear_freshness()
        status, payload = self.get("/api/raw")
        self.assertFalse(payload["live"])
        self.assertIsNone(payload["data"])

    def test_missing_frames_answer_null_data(self):
        for path in (
            "/api/state",
            "/api/health",
            "/api/tof",
            "/api/imu",
            "/api/calibration",
            "/api/tuning",
            "/api/position",
            "/api/raw",
        ):
            with self.subTest(path=path):
                status, payload = self.get(path)
                self.assertEqual(status, 200)
                self.assertTrue(payload["ok"])
                self.assertFalse(payload["live"])
                self.assertIsNone(payload["data"])

    def test_snapshot_combines_every_frame_and_the_console(self):
        self.state.update_state(state_frame())
        self.state.update_health({"robot_id": 1, "supply_voltage": 12.0})
        self.state.update_tof({"distances": [100] * 64, "statuses": [5] * 64, "sequence": 1})
        self.state.update_imu({
            "accel": [0.0, 0.0, 1.0],
            "gyro": [0.0, 0.0, 0.0],
            "mag": [1.0, 2.0, 3.0],
            "yaw": 12.0,
            "pitch": 0.0,
            "roll": 0.0,
        })
        self.state.update_calibration(demo.idle_calibration_frame(1))
        self.state.update_tuning(demo.idle_tuning_frame(1))
        self.state.update_position(demo.position_frame(1))
        self.state.update_raw(raw_frame())
        self.state.add_console("linha de teste")
        status, payload = self.get("/api/snapshot?console=5")
        self.assertEqual(status, 200)
        for key in ("status", "state", "health", "tof", "imu", "calibration", "tuning", "position", "raw"):
            self.assertIn(key, payload)
        for key in ("state", "health", "tof", "imu", "calibration", "tuning", "position", "raw"):
            self.assertTrue(payload[key]["live"], key)
            self.assertIsNotNone(payload[key]["data"], key)
        self.assertEqual(payload["calibration"]["data"]["state_name"], "idle")
        self.assertEqual(payload["calibration"]["data"]["stage_names"], ["reduced"] * 4)
        self.assertEqual(payload["tuning"]["data"]["phase_name"], "idle")
        self.assertEqual(payload["position"]["data"]["error_name"], "none")
        self.assertEqual(payload["raw"]["data"]["applied_pwm"][3], 1023)
        self.assertEqual(payload["tof"]["data"]["side"], config.TOF_SIDE)
        self.assertEqual(payload["imu"]["data"]["yaw"], 12.0)
        self.assertEqual(payload["tof"]["data"]["distances_mm"][0][0], 100)
        self.assertEqual(len(payload["tof"]["data"]["statuses"]), config.TOF_SIDE)
        self.assertIn("linha de teste", payload["console"]["lines"])

    def test_calibration_payload_includes_stage_and_adc_endpoints(self):
        self.state.update_calibration(demo.done_calibration_frame(1))
        status, payload = self.get("/api/calibration")
        self.assertEqual(status, 200)
        data = payload["data"]
        self.assertTrue(payload["live"])
        self.assertEqual(data["stage_names"], ["reduced"] * 4)
        self.assertEqual(data["adc_min_raw"], [minimum for minimum, _maximum in demo.DEMO_ADC_RANGE])
        self.assertEqual(data["adc_max_raw"], [maximum for _minimum, maximum in demo.DEMO_ADC_RANGE])

    def test_tuning_payload_exposes_both_rotation_directions(self):
        self.state.update_tuning(demo.done_tuning_frame(1))
        status, payload = self.get("/api/tuning")
        self.assertEqual(status, 200)
        data = payload["data"]
        self.assertTrue(payload["live"])
        for key in (
            "feedforward",
            "feedforward_rev",
            "velocity_ff",
            "velocity_ff_rev",
            "rise_ms",
            "rise_ms_rev",
            "overshoot_pct",
            "overshoot_pct_rev",
            "ss_error_pct",
            "ss_error_pct_rev",
        ):
            self.assertIn(key, data)
        self.assertEqual(data["feedforward_rev"][0], -198.0)
        self.assertEqual(data["velocity_ff_rev"][0], 57.0)
        self.assertEqual(data["rise_ms_rev"][0], 310)
        self.assertEqual(data["max_rpm"], demo.DEMO_MAX_RPM)
        self.assertEqual(data["wheel_max_rpm"], list(demo.DEMO_WHEEL_MAX_RPM))

    def test_console_cursor_returns_only_new_lines(self):
        self.state.add_console("um")
        self.state.add_console("dois")
        status, payload = self.get("/api/console?limit=10")
        self.assertEqual(payload["lines"][-2:], ["um", "dois"])
        cursor = payload["next_since"]
        self.assertEqual(payload["total"], cursor)
        self.state.add_console("tres")
        status, payload = self.get(f"/api/console?since={cursor}")
        self.assertEqual(payload["lines"], ["tres"])
        self.assertEqual(payload["next_since"], cursor + 1)
        status, payload = self.get("/api/console?limit=1")
        self.assertEqual(payload["lines"], ["tres"])
        status, payload = self.get("/api/console?limit=abc")
        self.assertEqual(status, 400)


class CommandTests(ApiTestCase):
    def test_raw_encoder_focus_command_and_status(self):
        status, payload = self.post("/api/raw_encoder", {"wheel": 2})
        self.assertEqual(status, 200)
        self.assertEqual(payload["raw_encoder_wheel"], 2)
        self.assertEqual(self.state.raw_encoder_wheel, 2)
        status, payload = self.get("/api/status")
        self.assertEqual(payload["raw_encoder_wheel"], 2)
        status, payload = self.post("/api/raw_encoder", {"wheel": None})
        self.assertEqual(status, 200)
        self.assertIsNone(payload["raw_encoder_wheel"])
        self.assertIsNone(self.state.raw_encoder_wheel)
        for body in ({}, {"wheel": 4}, {"wheel": -1}, {"wheel": "0"}, {"wheel": True}):
            with self.subTest(body=body):
                status, payload = self.post("/api/raw_encoder", body)
                self.assertEqual(status, 400)
                self.assertFalse(payload["ok"])

    def test_targets_commands_route_to_the_dashboard(self):
        status, payload = self.post("/api/targets", {"wheel": 2, "rpm": 500})
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertEqual(self.state.manual_targets[2], 500)
        self.assertEqual(payload["targets_rpm"][2], 500)
        status, payload = self.post("/api/targets", {"targets_rpm": [100, -200, 300, 0]})
        self.assertEqual(status, 200)
        self.assertEqual(self.state.manual_targets, [100, -200, 300, 0])
        self.assertTrue(any("alvos" in line for line in payload["messages"]))

    def test_targets_bodies_are_validated(self):
        for body in (
            {},
            {"targets_rpm": [1, 2, 3]},
            {"targets_rpm": [1, 2, 3, True]},
            {"wheel": 4, "rpm": 100},
            {"wheel": 0},
            {"rpm": 100},
        ):
            with self.subTest(body=body):
                status, payload = self.post("/api/targets", body)
                self.assertEqual(status, 400)
                self.assertFalse(payload["ok"])

    def test_targets_are_clamped_to_the_calibrated_limit(self):
        self.state.update_tuning(demo.done_tuning_frame(1))
        status, payload = self.post("/api/targets", {"targets_rpm": [500, -500, 62, 0]})
        self.assertEqual(status, 200)
        limit = demo.DEMO_MAX_RPM
        self.assertEqual(payload["max_rpm"], limit)
        self.assertEqual(payload["targets_rpm"], [limit, -limit, 62.0, 0.0])
        self.assertEqual(self.state.manual_targets, [limit, -limit, 62.0, 0.0])
        status, payload = self.post("/api/targets", {"wheel": 0, "rpm": -900})
        self.assertEqual(payload["targets_rpm"][0], -limit)
        self.assertEqual(self.state.manual_targets[0], -limit)

    def test_targets_are_refused_while_a_calibration_runs(self):
        state = self.live_state()
        state.update_calibration(demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING))
        status, payload = self.post("/api/targets", {"targets_rpm": [100, 0, 0, 0]})
        self.assertEqual(status, 409)
        self.assertIn("calibração", payload["error"])
        state.reader.send_frame.assert_not_called()

    def test_targets_without_a_link_are_refused(self):
        self.state.demo = False
        status, payload = self.post("/api/targets", {"targets_rpm": [100, 0, 0, 0]})
        self.assertEqual(status, 409)
        self.assertIn("sem link serial", payload["error"])

    def test_stop_override_and_sweep_commands(self):
        status, payload = self.post("/api/stop")
        self.assertEqual(status, 200)
        self.assertEqual(self.state.manual_targets, [0.0] * 4)
        self.assertTrue(any("parar motores" in line for line in payload["messages"]))
        status, payload = self.post("/api/sweep", {"enabled": True})
        self.assertEqual(status, 200)
        self.assertTrue(self.state.sweep)
        status, payload = self.post("/api/sweep", {"enabled": False})
        self.assertFalse(self.state.sweep)

    def test_calibration_needs_confirmation_and_stop_clears_the_demo_run(self):
        self.state.update_calibration(demo.idle_calibration_frame(1))
        status, payload = self.post("/api/calibration", {"action": "start"})
        self.assertEqual(status, 409)
        self.assertIn("confirm", payload["error"])
        status, payload = self.post("/api/calibration", {"action": "start", "confirm": True})
        self.assertEqual(status, 200)
        self.assertTrue(self.state.calibration_confirm)
        self.assertIsNotNone(self.state.demo_calibration_start)
        self.assertTrue(any("calibração" in line for line in payload["messages"]))
        status, payload = self.post("/api/calibration", {"action": "stop"})
        self.assertEqual(status, 200)
        self.assertIsNone(self.state.demo_calibration_start)
        status, payload = self.post("/api/calibration", {"action": "bogus"})
        self.assertEqual(status, 400)

    def test_tuning_needs_confirmation(self):
        status, payload = self.post("/api/tuning", {"action": "start"})
        self.assertEqual(status, 409)
        status, payload = self.post("/api/tuning", {"action": "start", "confirm": True})
        self.assertEqual(status, 200)
        self.assertTrue(self.state.tuning_confirm)
        self.assertIsNotNone(self.state.demo_tuning_start)
        status, payload = self.post("/api/tuning", {"action": "stop"})
        self.assertEqual(status, 200)
        self.assertIsNone(self.state.demo_tuning_start)

    def test_position_move_and_stop_route_to_the_dashboard(self):
        status, payload = self.post(
            "/api/position",
            {"action": "move", "moves": [{"wheel": 1, "angle_deg": 90}], "absolute": True, "max_rpm": 250},
        )
        self.assertEqual(status, 200)
        requested = self.state.demo_position
        self.assertEqual(requested["wheel_mask"], 0b0010)
        self.assertEqual(requested["angles_deg"][1], 90.0)
        self.assertTrue(requested["absolute"])
        self.assertEqual(requested["max_rpm"], 250)
        self.assertEqual(payload["moves"], [{"wheel": 1, "angle_deg": 90.0}])
        status, payload = self.post("/api/position", {"action": "stop"})
        self.assertEqual(status, 200)
        self.assertIsNone(self.state.demo_position)

    def test_position_body_is_validated(self):
        for body in (
            {},
            {"action": "move"},
            {"action": "move", "moves": []},
            {"action": "move", "moves": [{"wheel": 4, "angle_deg": 10}]},
            {"action": "move", "moves": [{"wheel": 0, "angle_deg": "10"}]},
            {"action": "move", "moves": [{"wheel": 0, "angle_deg": 10}], "absolute": "yes"},
            {"action": "move", "moves": [{"wheel": 0, "angle_deg": 10}], "max_rpm": -1},
        ):
            with self.subTest(body=body):
                status, payload = self.post("/api/position", body)
                self.assertEqual(status, 400)
                self.assertFalse(payload["ok"])

    def test_unknown_endpoints_and_invalid_requests_answer_json(self):
        status, payload = self.get("/api/nope")
        self.assertEqual(status, 404)
        self.assertFalse(payload["ok"])
        status, payload = self.request("PUT", "/api/state")
        self.assertEqual(status, 405)
        status, payload = self.request("POST", "/api/targets", b"{not json")
        self.assertEqual(status, 400)
        status, payload = self.request("POST", "/api/nope", b"{}")
        self.assertEqual(status, 404)

    def test_commands_are_refused_when_the_api_has_no_listeners(self):
        server = api.ApiServer(self.state)
        server.start("127.0.0.1", None, "")
        self.assertEqual(server.port, 0)
        self.assertIsNone(server.socket_path)
        server.stop()


class UnixSocketTests(ApiTestCase):
    def test_unix_socket_serves_the_same_api(self):
        connection = _UnixConnection(self.server.socket_path)
        try:
            connection.request("GET", "/api/status")
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            payload = json.loads(response.read())
            self.assertTrue(payload["ok"])
            self.assertTrue(payload["demo"])
        finally:
            connection.close()

    def test_unix_socket_accepts_commands(self):
        connection = _UnixConnection(self.server.socket_path)
        body = json.dumps({"wheel": 0, "rpm": 300}).encode("utf-8")
        try:
            connection.request("POST", "/api/targets", body, {"Content-Type": "application/json"})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertTrue(json.loads(response.read())["ok"])
        finally:
            connection.close()
        self.assertEqual(self.state.manual_targets[0], 300)


class AccessControlTests(ApiTestCase):
    def test_client_allowlist_covers_this_machine_and_local_networks(self):
        for address in (
            "127.0.0.1",
            "127.1.2.3",
            "::1",
            "192.168.1.10",
            "10.1.2.3",
            "172.16.0.9",
            "169.254.10.1",
            "fd00::1",
            "fe80::1",
        ):
            self.assertTrue(api.client_allowed((address, 1234)), address)
        for address in ("8.8.8.8", "203.0.113.5", "100.64.0.1", "198.51.100.9", "2001:db8::1"):
            self.assertFalse(api.client_allowed((address, 1234)), address)
        self.assertTrue(api.client_allowed(("unix", 0)))
        self.assertFalse(api.client_allowed(("", 0)))
        self.assertFalse(api.client_allowed(None))

    def test_web_clients_get_http_403(self):
        with patch("robot_bench.api.client_allowed", return_value=False):
            status, payload = self.get("/api/state")
            self.assertEqual(status, 403)
            self.assertFalse(payload["ok"])
            status, payload = self.post("/api/targets", {"wheel": 0, "rpm": 100})
            self.assertEqual(status, 403)
            self.assertEqual(self.state.manual_targets[0], 0)

    def test_browser_and_proxy_headers_get_http_403(self):
        for headers in (
            {"Origin": "https://evil.example"},
            {"Sec-Fetch-Site": "cross-site"},
            {"X-Forwarded-For": "8.8.8.8"},
            {"Forwarded": "for=203.0.113.5"},
        ):
            with self.subTest(headers=headers):
                status, payload = self.request("GET", "/api/status", headers=headers)
                self.assertEqual(status, 403)
                self.assertFalse(payload["ok"])

    def test_socket_is_owner_only(self):
        self.assertEqual(os.stat(self.server.socket_path).st_mode & 0o777, 0o600)


class ListenerConflictTests(ApiTestCase):
    def test_second_instance_does_not_steal_a_running_socket(self):
        other = api.ApiServer(self.state)
        other.start("127.0.0.1", None, self.server.socket_path)
        try:
            self.assertIsNone(other.socket_path)
            connection = _UnixConnection(self.server.socket_path)
            try:
                connection.request("GET", "/api/status")
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                response.read()
            finally:
                connection.close()
        finally:
            other.stop()

    def test_second_instance_reports_a_busy_tcp_port(self):
        other = api.ApiServer(self.state)
        other.start("127.0.0.1", self.server.port, "")
        try:
            self.assertEqual(other.port, 0)
        finally:
            other.stop()


if __name__ == "__main__":
    unittest.main()
