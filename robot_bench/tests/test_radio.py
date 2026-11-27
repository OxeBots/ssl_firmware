"""Regression checks for the ESP-NOW gateway codec, stream and session."""

import socket
import struct
import unittest
from unittest.mock import Mock, patch

from robot_bench import config, discovery, picker, protocol, radio, radio_panel
from scripts.ssl_robot_protocol_bp import (
    CONFIG_FLAG_CALIBRATE_MAG,
    CONFIG_FLAG_RUN_CALIBRATION,
    CONFIG_FLAG_SET_ID,
    MSG_TYPE_COMMAND,
    MSG_TYPE_CONFIG,
    MSG_TYPE_TELEMETRY,
    MSG_TYPE_TELEMETRY_REQ,
    RobotCommand,
    RobotConfig,
    RobotTelemetry,
    RobotTelemetryRequest,
)


class FakePort:
    """Stand-in for serial.tools.list_ports_common.ListPortInfo."""

    def __init__(self, device, vid=None, pid=None, description=None):
        self.device = device
        self.vid = vid
        self.pid = pid
        self.description = description or "USB Serial"


def make_telemetry(robot_id=2, timestamp=0x01020304, **pose):
    message = RobotTelemetry()
    message.header.msg_type = MSG_TYPE_TELEMETRY
    message.header.robot_id = robot_id
    message.header.timestamp = timestamp
    message.robot_pose.x = pose.get("x", -500)
    message.robot_pose.y = pose.get("y", 500)
    message.robot_pose.angle = pose.get("angle", 9000)
    message.robot_pose.x_v = pose.get("vx", -100)
    message.robot_pose.y_v = pose.get("vy", 200)
    message.robot_pose.angular_vel = pose.get("w", -50)
    message.battery_percentage = pose.get("battery", 87)
    message.kicker_voltage = pose.get("kicker", 1650)
    message.error_flags = pose.get("errors", 0x0021)
    return message.encode()


class CommandEncodingTests(unittest.TestCase):
    def test_command_matches_the_bitproto_encoder(self):
        message = RobotCommand()
        message.header.msg_type = MSG_TYPE_COMMAND
        message.header.robot_id = 3
        message.header.timestamp = 0x12345678
        message.target_pose.x = -1234
        message.target_pose.y = 5678
        message.target_pose.angle = -90
        message.target_pose.x_v = 1000
        message.target_pose.y_v = -2000
        message.target_pose.angular_vel = 150
        message.kick_velocity = 42
        encoded = radio.encode_command(
            3,
            x=-1234,
            y=5678,
            angle=-90,
            vx=1000,
            vy=-2000,
            w=150,
            kick=42,
            timestamp=0x12345678,
        )
        self.assertEqual(len(encoded), radio.COMMAND_SIZE)
        self.assertEqual(encoded, message.encode())

    def test_command_field_layout(self):
        encoded = radio.encode_command(5, x=7, timestamp=0x0A0B0C0D)
        self.assertEqual(encoded[0], (5 << 4) | MSG_TYPE_COMMAND)
        self.assertEqual(encoded[1:5], bytes([0x0D, 0x0C, 0x0B, 0x0A]))
        self.assertEqual(encoded[5:7], bytes([7, 0]))
        self.assertEqual(encoded[17:29], bytes(12))
        self.assertEqual(encoded[29], 0)

    def test_command_rejects_an_id_that_does_not_fit_the_header(self):
        with self.assertRaises(ValueError):
            radio.encode_command(16)
        self.assertEqual(radio.encode_command(radio.ROBOT_ID_BROADCAST)[0] >> 4, 15)

    def test_config_matches_the_bitproto_encoder(self):
        message = RobotConfig()
        message.header.msg_type = MSG_TYPE_CONFIG
        message.header.robot_id = 5
        message.header.timestamp = 0xAABBCCDD
        message.config_flags = CONFIG_FLAG_RUN_CALIBRATION | CONFIG_FLAG_CALIBRATE_MAG
        message.param = 1234
        encoded = radio.encode_config(
            message.header.robot_id, message.config_flags, message.param, timestamp=message.header.timestamp
        )
        self.assertEqual(len(encoded), radio.CONFIG_SIZE)
        self.assertEqual(encoded, message.encode())

    def test_config_rejects_an_id_that_does_not_fit_the_header(self):
        with self.assertRaises(ValueError):
            radio.encode_config(16, CONFIG_FLAG_SET_ID)

    def test_telemetry_request_matches_the_bitproto_encoder(self):
        message = RobotTelemetryRequest()
        message.header.msg_type = MSG_TYPE_TELEMETRY_REQ
        message.header.robot_id = 4
        message.header.timestamp = 0x0A0B0C0D
        self.assertEqual(radio.encode_telemetry_request(4, message.header.timestamp), message.encode())

    def test_telemetry_request_rejects_broadcast(self):
        with self.assertRaises(ValueError):
            radio.encode_telemetry_request(radio.ROBOT_ID_BROADCAST, 0)


class TelemetryDecodingTests(unittest.TestCase):
    def test_decodes_every_telemetry_field(self):
        decoded = radio.decode_telemetry(make_telemetry())
        self.assertEqual(
            decoded,
            {
                "robot_id": 2,
                "timestamp": 0x01020304,
                "x_mm": -500,
                "y_mm": 500,
                "angle_deg": 90.0,
                "vx_mm_s": -100,
                "vy_mm_s": 200,
                "w_rad_s": -0.5,
                "battery_pct": 87,
                "kicker_voltage_v": 16.5,
                "error_flags": 0x21,
            },
        )

    def test_rejects_short_and_empty_frames(self):
        frame = make_telemetry()
        self.assertIsNone(radio.decode_telemetry(frame[: radio.TELEMETRY_SIZE - 1]))
        self.assertIsNone(radio.decode_telemetry(b""))

    def test_rejects_other_message_types(self):
        frame = bytearray(make_telemetry())
        frame[0] = (2 << 4) | MSG_TYPE_COMMAND
        self.assertIsNone(radio.decode_telemetry(bytes(frame)))

    def test_error_flag_names(self):
        self.assertEqual(radio.error_flag_names(0), [])
        self.assertEqual(
            radio.error_flag_names(1 | 64 | 2048),
            ["Comunicação", "Encoder", "Motor 3"],
        )


class TelemetryStreamTests(unittest.TestCase):
    def setUp(self):
        self.stream = radio.TelemetryStream()

    def test_fragmented_frame_is_assembled(self):
        frame = radio.encode_link_frame(make_telemetry())
        for byte in frame[:-1]:
            self.assertEqual(self.stream.feed(bytes([byte])), [])
        frames = self.stream.feed(frame[-1:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["robot_id"], 2)

    def test_foreign_frame_type_and_corrupted_checksum_are_rejected(self):
        foreign = protocol.encode_frame(protocol.MSG_LINK, make_telemetry())
        corrupted = bytearray(radio.encode_link_frame(make_telemetry()))
        corrupted[2] ^= 1
        self.assertEqual(self.stream.feed(foreign + corrupted), [])

    def test_two_frames_in_one_read(self):
        data = radio.encode_link_frame(make_telemetry(robot_id=2)) + radio.encode_link_frame(
            make_telemetry(robot_id=4, battery=40)
        )
        frames = self.stream.feed(data)
        self.assertEqual([frame["robot_id"] for frame in frames], [2, 4])
        self.assertEqual(frames[1]["battery_pct"], 40)

    def test_garbage_before_a_frame_is_skipped(self):
        garbage = b"boot log without a trailing newline"
        frames = self.stream.feed(garbage + radio.encode_link_frame(make_telemetry()))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["timestamp"], 0x01020304)

    def test_partial_garbage_keeps_the_frame(self):
        frame = radio.encode_link_frame(make_telemetry())
        self.assertEqual(self.stream.feed(frame[:20]), [])
        self.assertEqual(len(self.stream.feed(frame[20:])), 1)


class AdapterDiscoveryTests(unittest.TestCase):
    def test_configured_gateway_port_is_listed_when_present(self):
        with (
            patch.object(radio, "ESP_LINK_PORT", "/dev/ttyUSB0"),
            patch.object(radio.os.path, "exists", return_value=True) as exists,
        ):
            self.assertEqual(radio.find_adapters(), [("/dev/ttyUSB0", "Ponte ESP-NOW")])
            exists.assert_called_once_with("/dev/ttyUSB0")


class DiscoveryFilterTests(unittest.TestCase):
    def test_radio_adapter_is_not_a_robot_port(self):
        ports = [
            FakePort("/dev/ttyUSB0", vid=0x10C4, pid=0xEA60),
            FakePort("/dev/ttyUSB1", vid=0x10C4, pid=0xEA60),
        ]
        with (
            patch.object(radio, "ESP_LINK_PORT", "/dev/ttyUSB0"),
            patch.object(discovery.serial.tools.list_ports, "comports", return_value=ports),
        ):
            candidates = discovery.list_candidate_ports()
        self.assertEqual([device for device, _description in candidates], ["/dev/ttyUSB1"])

    def test_is_radio_adapter(self):
        with patch.object(radio, "ESP_LINK_PORT", "/dev/ttyUSB0"):
            self.assertTrue(discovery.is_radio_adapter(FakePort("/dev/ttyUSB0")))
            self.assertFalse(discovery.is_radio_adapter(FakePort("/dev/ttyUSB1")))
            self.assertFalse(discovery.is_radio_adapter(FakePort("/dev/ttyACM0")))


class PickerRadioTests(unittest.TestCase):
    def test_radio_robot_id_is_clamped_to_the_protocol_range(self):
        with patch.object(picker.dpg, "get_value", return_value=15):
            self.assertEqual(picker.radio_robot_id(), picker.RADIO_ROBOT_MAX)
        with patch.object(picker.dpg, "get_value", return_value=-4):
            self.assertEqual(picker.radio_robot_id(), 0)
        with patch.object(picker.dpg, "get_value", return_value=3):
            self.assertEqual(picker.radio_robot_id(), 3)

    def test_radio_connect_button_forwards_the_adapter_device(self):
        seen = []
        original = picker._on_connect_radio
        picker._on_connect_radio = seen.append
        try:
            picker._connect_radio(None, None, "/dev/ttyUSB1")
        finally:
            picker._on_connect_radio = original
        self.assertEqual(seen, ["/dev/ttyUSB1"])


def make_snapshot(
    connected=True,
    live=False,
    rx_count=0,
    telemetry=None,
    rtt_ms=0.0,
    tx_count=0,
    port="/dev/ttyUSB1",
    messages=(),
):
    return {
        "port": port,
        "connected": connected,
        "robot_id": 2,
        "telemetry": telemetry,
        "rx_age_s": None,
        "rtt_ms": rtt_ms,
        "tx_count": tx_count,
        "rx_count": rx_count,
        "command": {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0},
        "pending_configs": 0,
        "error": "",
        "messages": list(messages),
        "live": live,
    }


TELEMETRY = {
    "robot_id": 2,
    "timestamp": 0x01020304,
    "x_mm": -500,
    "y_mm": 500,
    "angle_deg": 90.0,
    "vx_mm_s": -100,
    "vy_mm_s": 200,
    "w_rad_s": -0.5,
    "battery_pct": 87,
    "kicker_voltage_v": 16.5,
    "error_flags": 1,
}


class RadioPanelTests(unittest.TestCase):
    def test_status_text_covers_every_link_state(self):
        self.assertEqual(
            radio_panel.status_text(make_snapshot(live=True)),
            ("Ao vivo · /dev/ttyUSB1", config.OK_COLOR),
        )
        self.assertEqual(
            radio_panel.status_text(make_snapshot(live=False, rx_count=4)),
            ("Conectado · sem resposta", config.BAD_COLOR),
        )
        self.assertEqual(
            radio_panel.status_text(make_snapshot(live=False, rx_count=0)),
            ("Conectado · aguardando telemetria", config.TEXT_DIM),
        )
        self.assertEqual(
            radio_panel.status_text(make_snapshot(connected=False)),
            ("Desconectado · reconectando", config.BAD_COLOR),
        )

    def test_telemetry_rows_are_placeholders_without_a_frame(self):
        rows = radio_panel.telemetry_rows(make_snapshot())
        self.assertEqual([label for label, _value in rows], list(radio_panel.TELEMETRY_LABELS))
        self.assertEqual([value for _label, value in rows], ["—"] * len(radio_panel.TELEMETRY_LABELS))

    def test_telemetry_rows_format_the_radio_frame(self):
        snapshot = make_snapshot(telemetry=TELEMETRY, rtt_ms=21.34, rx_count=12, tx_count=15)
        rows = dict(radio_panel.telemetry_rows(snapshot))
        self.assertEqual(rows["Robô"], "2")
        self.assertEqual(rows["Bateria"], "87 %")
        self.assertEqual(rows["Kicker"], "16.50 V")
        self.assertEqual(rows["Yaw"], "90.00°")
        self.assertEqual(rows["Posição"], "x -500 mm · y 500 mm")
        self.assertEqual(rows["Velocidade"], "vx -100 · vy 200 mm/s · w -0.50 rad/s")
        self.assertEqual(rows["Avisos"], "Comunicação")
        self.assertEqual(rows["RTT"], "21.3 ms")
        self.assertEqual(rows["Quadros"], "RX 12 · TX 15")

    def test_telemetry_rows_report_no_errors(self):
        telemetry = dict(TELEMETRY, error_flags=0)
        rows = dict(radio_panel.telemetry_rows(make_snapshot(telemetry=telemetry)))
        self.assertEqual(rows["Avisos"], "nenhum")


class FocusTrackingTests(unittest.TestCase):
    def test_viewport_focus_assumes_true_without_the_glfw_symbols(self):
        with patch.object(radio_panel, "_focus_checked", True), patch.object(radio_panel, "_focus_library", None):
            self.assertTrue(radio_panel.viewport_focused())

    def test_viewport_focus_reads_the_glfw_flag(self):
        library = Mock()
        library.glfwGetCurrentContext.return_value = 0x1234
        with patch.object(radio_panel, "_focus_checked", True), patch.object(radio_panel, "_focus_library", library):
            library.glfwGetWindowAttrib.return_value = 0
            self.assertFalse(radio_panel.viewport_focused())
            library.glfwGetWindowAttrib.return_value = 1
            self.assertTrue(radio_panel.viewport_focused())
            library.glfwGetCurrentContext.return_value = 0
            self.assertTrue(radio_panel.viewport_focused())


class DiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.session = radio.RadioSession("/dev/ttyUSB1", robot_id=0)

    def tearDown(self):
        self.session.close()

    def _install_fakes(self, answering):
        """Make the session answer discovery probes for the given robot ids."""
        written = []

        def write(frame):
            written.append(frame)
            return True

        def read_reply(_deadline):
            frame = written[-1]
            self.assertEqual(frame[0] & 0x0F, MSG_TYPE_TELEMETRY_REQ)
            robot_id = frame[0] >> 4
            if robot_id not in answering:
                return []
            expected_timestamp = struct.unpack_from("<I", frame, 1)[0]
            payload = bytearray(make_telemetry(robot_id=robot_id))
            struct.pack_into("<I", payload, 1, expected_timestamp)
            return [radio.decode_telemetry(bytes(payload))]

        self.session._write = write
        self.session._read_reply = read_reply

    def test_discover_lists_only_the_robots_that_answer(self):
        self._install_fakes({2, 5})
        found = self.session.discover(timeout_s=0.01)
        self.assertEqual([telemetry["robot_id"] for telemetry in found], [2, 5])
        for telemetry in found:
            self.assertIn("rtt_ms", telemetry)

    def test_auto_discovery_waits_for_a_gateway_that_ignores_the_first_echo_request(self):
        reader, writer = socket.socketpair()
        with reader, writer:
            port = Mock(in_waiting=4096)
            port.fileno.side_effect = reader.fileno
            port.read.side_effect = reader.recv

            def answer(frame):
                kind, payload = protocol.FrameDecoder().feed(frame)[0]
                self.assertEqual(kind, radio.ESP_LINK_LOOPBACK_TYPE)
                self.assertEqual(payload[0], MSG_TYPE_TELEMETRY_REQ)
                if port.write.call_count == 1:
                    writer.sendall(b"rst:0x1 (POWERON_RESET)\r\n")
                else:
                    stamp = struct.unpack_from("<I", payload, 1)[0]
                    reply = make_telemetry(robot_id=0, timestamp=stamp)
                    writer.sendall(radio.encode_link_frame(reply, kind))

            def probe(robot_id, _timeout):
                self.assertEqual(port.write.call_count, 2)
                return {"robot_id": 0} if robot_id == 0 else None

            port.write.side_effect = answer
            with (
                patch.object(radio, "open_gateway", return_value=port),
                patch.object(radio.RadioSession, "probe", side_effect=probe),
            ):
                found = radio.discover_robots("/dev/ttyUSB0")

        self.assertEqual(found, [{"robot_id": 0}])
        port.close.assert_called_once()

    def test_probe_spaces_requests_even_when_replies_arrive_immediately(self):
        self.session._ser = Mock()
        sent = []
        now = 1.0

        def write(frame):
            kind, payload = protocol.FrameDecoder().feed(frame)[0]
            self.assertEqual(kind, radio.ESP_LINK_FRAME_TYPE)
            sent.append((now, payload))

        def sleep(delay):
            nonlocal now
            now += delay

        def read_reply(_deadline):
            frame = sent[-1][1]
            return [radio.decode_telemetry(make_telemetry(
                robot_id=frame[0] >> 4, timestamp=struct.unpack_from("<I", frame, 1)[0]
            ))]

        self.session._ser.write.side_effect = write
        with (
            patch.object(self.session, "_read_reply", side_effect=read_reply),
            patch.object(radio.time, "monotonic", side_effect=lambda: now),
            patch.object(radio.time, "sleep", side_effect=sleep),
        ):
            first = self.session.probe(2)
            second = self.session.probe(5)

        self.assertEqual([first["robot_id"], second["robot_id"]], [2, 5])
        self.assertEqual([frame[0] & 0x0F for _, frame in sent], [MSG_TYPE_TELEMETRY_REQ] * 2)
        self.assertAlmostEqual(sent[1][0] - sent[0][0], radio.FRAME_GAP_S)
        self.assertAlmostEqual(second["rtt_ms"], 0.0)

    def test_probe_returns_none_without_a_reply(self):
        self._install_fakes(set())
        self.assertIsNone(self.session.probe(2, timeout_s=0.01))

    def test_probe_ignores_a_reply_with_a_foreign_timestamp(self):
        def write(_frame):
            return True

        def read_reply(_deadline):
            return [radio.decode_telemetry(make_telemetry(robot_id=2))]

        self.session._write = write
        self.session._read_reply = read_reply
        self.assertIsNone(self.session.probe(2, timeout_s=0.01))


class PickerDiscoveryTests(unittest.TestCase):
    def test_discovered_label_and_detail(self):
        telemetry = {"robot_id": 7, "rtt_ms": 80.4, "battery_pct": 42, "error_flags": 1}
        self.assertEqual(picker.discovered_label(telemetry), "Robô 7")
        detail = picker.discovered_detail(telemetry)
        self.assertIn("RTT 80 ms", detail)
        self.assertIn("Bateria 42 %", detail)
        self.assertIn("Erros: Comunicação", detail)

    def test_discovered_detail_without_errors(self):
        telemetry = {"robot_id": 2, "rtt_ms": 21.0, "battery_pct": 95, "error_flags": 0}
        self.assertIn("sem erros", picker.discovered_detail(telemetry))


class RadioSessionTests(unittest.TestCase):
    def setUp(self):
        self.session = radio.RadioSession("/dev/ttyUSB1", robot_id=2)

    def tearDown(self):
        self.session.close()

    def test_initial_snapshot_is_disconnected(self):
        snapshot = self.session.snapshot()
        self.assertEqual(snapshot["port"], "/dev/ttyUSB1")
        self.assertEqual(snapshot["robot_id"], 2)
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["live"])
        self.assertIsNone(snapshot["telemetry"])
        self.assertIsNone(snapshot["rx_age_s"])
        self.assertEqual(snapshot["command"], {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0})

    def test_out_of_range_robot_id_falls_back_to_zero(self):
        self.assertEqual(radio.RadioSession("/dev/ttyUSB1", robot_id=11).snapshot()["robot_id"], 0)

    def test_set_motion_streams_the_requested_command(self):
        self.session._last_rx_t = radio.time.monotonic()
        self.session.set_motion(x=100, vy=-200, w=30)
        self.session.set_motion(vx=50)
        frame = self.session._next_frame()
        self.assertEqual(len(frame), radio.COMMAND_SIZE)
        self.assertEqual(frame[0] >> 4, 2)
        # Pose starts at byte 5: x, y, angle, vx, vy, w (int16 little-endian).
        self.assertEqual(frame[5:7], bytes([100, 0]))
        self.assertEqual(frame[7:9], bytes([0, 0]))
        self.assertEqual(frame[9:11], bytes([0, 0]))
        self.assertEqual(frame[11:13], bytes([50, 0]))
        self.assertEqual(frame[13:15], bytes([0x38, 0xFF]))
        self.assertEqual(frame[15:17], bytes([30, 0]))

    def test_stop_motion_zeroes_the_streamed_command(self):
        self.session._last_rx_t = radio.time.monotonic()
        self.session.set_motion(vx=500, w=100)
        self.session.kick(200)
        self.session.stop_motion()
        self.session.kick(0)
        frame = self.session._next_frame()
        self.assertEqual(frame[5:17], bytes(12))
        self.assertEqual(frame[29], 0)

    def test_keyboard_speed_uses_a_wheel_rpm_floor(self):
        self.assertEqual(radio.keyboard_velocity(1, 0, 0, 10), (241, 0, 0))
        self.assertEqual(radio.keyboard_velocity(1, -1, 0, 50), (121, -121, 0))
        self.assertEqual(radio.keyboard_velocity(0, 0, -1, 50), (0, 0, -178))
        self.assertEqual(radio.keyboard_velocity(1, 0, 1, 50), (0, 0, 0))

    def test_keyboard_motion_expires_and_clears_a_previous_manual_setpoint(self):
        self.session.set_motion(x=100, vx=500)
        self.session._connected = True
        self.session._last_rx_t = 10.0
        with patch.object(radio.time, "monotonic", return_value=10.05):
            self.session.set_keyboard_velocity((193, 0, 0))
            self.assertEqual(self.session._next_frame()[5:17], struct.pack("<6h", 0, 0, 0, 193, 0, 0))
        with patch.object(radio.time, "monotonic", return_value=10.26):
            self.session._last_rx_t = 10.24
            self.assertEqual(self.session._next_frame()[5:17], bytes(12))

    def test_keyboard_motion_requires_fresh_telemetry_from_the_target(self):
        self.session._connected = True
        self.session._last_rx_t = 10.0
        with patch.object(radio.time, "monotonic", return_value=10.3):
            self.session.set_keyboard_velocity((193, 0, 0))
            self.assertFalse(self.session.keyboard_link_ready())
            self.assertEqual(self.session._next_frame()[5:17], bytes(12))

    def test_kick_is_sent_with_one_command_only(self):
        self.session._last_rx_t = radio.time.monotonic()
        self.session.kick(120)
        first = self.session._next_frame()
        second = self.session._next_frame()
        self.assertEqual(first[29], 120)
        self.assertEqual(second[29], 0)

    def test_set_robot_id_validates_and_retargets(self):
        self.assertFalse(self.session.set_robot_id(11))
        self.assertEqual(self.session.snapshot()["robot_id"], 2)
        self.session.set_motion(vx=250)
        self.assertTrue(self.session.set_robot_id(7))
        self.assertIsNone(self.session._next_frame())
        self.session._last_rx_t = radio.time.monotonic()
        self.assertEqual(self.session._next_frame()[0] >> 4, 7)
        self.assertEqual(self.session.snapshot()["command"]["vx"], 0)

    def test_config_command_precedes_the_next_poll(self):
        self.session.send_config(CONFIG_FLAG_SET_ID, param=4)
        config_frame = self.session._next_frame()
        self.assertEqual(len(config_frame), radio.CONFIG_SIZE)
        self.assertEqual(config_frame[0] >> 4, 2)
        self.assertEqual(config_frame[5], CONFIG_FLAG_SET_ID)
        self.assertEqual(config_frame[6:8], bytes([4, 0]))
        self.session._last_rx_t = radio.time.monotonic()
        self.assertEqual(len(self.session._next_frame()), radio.COMMAND_SIZE)
        self.assertEqual(self.session.snapshot()["messages"][-1], "rádio: config 0x01 param 4")

    def test_command_cadence_continues_through_a_telemetry_timeout(self):
        sent = []
        now = 1.0

        def wait(delay):
            nonlocal now
            now += max(delay, 0.0001)
            if now >= 1.1:
                self.session._closing.set()

        def send(frame, **_kwargs):
            sent.append((now, frame))
            return True

        def read_available():
            if now < 1.09:
                return []
            second_request = next(frame for sent_at, frame in sent if sent_at > 1.08 and frame[0] == 0x23)
            stamp = struct.unpack_from("<I", second_request, 1)[0]
            return [radio.decode_telemetry(make_telemetry(robot_id, timestamp)) for robot_id, timestamp in (
                (1, stamp), (2, stamp - 1), (2, stamp), (2, stamp),
            )]

        self.session._ser = Mock()
        self.session._last_rx_t = 0.99
        self.session.set_motion(vx=250)
        self.session.send_config(CONFIG_FLAG_CALIBRATE_MAG)
        with (
            patch.object(radio.time, "monotonic", side_effect=lambda: now),
            patch.object(radio, "_timestamp_ms", side_effect=lambda: int(now * 1000)),
            patch.object(self.session, "_send_frame", side_effect=send),
            patch.object(self.session, "_read_available", side_effect=read_available),
            patch.object(self.session._closing, "wait", side_effect=wait),
        ):
            self.session.run()

        self.assertEqual(sent[0][1], radio.encode_config(2, CONFIG_FLAG_CALIBRATE_MAG, timestamp=1000))
        requests = [t for t, frame in sent if frame[0] & 0x0F == MSG_TYPE_TELEMETRY_REQ]
        self.assertEqual(len(requests), 2)
        self.assertAlmostEqual(requests[1] - requests[0], radio.REPLY_TIMEOUT_S, delta=0.002)
        commands = [t for t, frame in sent if frame[0] & 0x0F == MSG_TYPE_COMMAND and t < requests[1]]
        self.assertEqual(len(commands), 4)
        for first, second in zip(commands, commands[1:], strict=False):
            self.assertAlmostEqual(second - first, radio.COMMAND_PERIOD_S, delta=0.001)
        self.assertEqual(sent[-1][1][5:], bytes(25))
        self.assertEqual(self.session.snapshot()["rx_count"], 1)

    def test_session_only_probes_an_unconfirmed_target(self):
        now = 1.0
        sent = []

        def wait(delay):
            nonlocal now
            now += max(delay, 0.0001)
            if now > 3.05:
                self.session._closing.set()

        self.session._ser = Mock()
        self.session.set_motion(vx=250)
        self.session.kick(120)
        with (
            patch.object(radio.time, "monotonic", side_effect=lambda: now),
            patch.object(self.session, "_send_frame", side_effect=lambda frame: sent.append((now, frame)) or True),
            patch.object(self.session, "_read_available", return_value=[]),
            patch.object(self.session._closing, "wait", side_effect=wait),
        ):
            self.session.run()

        self.assertEqual([frame[0] & 0x0F for _, frame in sent], [MSG_TYPE_TELEMETRY_REQ] * 3)
        for (first, _), (second, _) in zip(sent, sent[1:], strict=False):
            self.assertAlmostEqual(second - first, radio.IDLE_TELEMETRY_PERIOD_S, delta=0.003)
        self.session._last_rx_t = radio.time.monotonic()
        self.assertEqual(self.session._next_frame()[5:], bytes(25))

    def test_connect_to_a_missing_adapter_raises_radio_error(self):
        with self.assertRaises(radio.RadioError):
            radio.RadioSession.connect("/dev/ttyUSB99", robot_id=0)


if __name__ == "__main__":
    unittest.main()
