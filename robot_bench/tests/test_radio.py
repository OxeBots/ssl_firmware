"""Regression checks for the ESP32 Base Station radio link, codec, discovery and session."""

import struct
import unittest
from unittest.mock import Mock, patch

from robot_bench import base_station, config, discovery, picker, protocol as proto, radio, radio_panel
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


def make_base_info(channel=76, rx_channel=90, data_rate=base_station.BS_RATE_2MBPS, reinits=0, mask=3):
    return {
        "protocol_version": base_station.PROTO_VERSION,
        "radio_ready": bool(mask & 0x01),
        "radio_ready_mask": mask,
        "dual_radio": (mask & 0x03) == 0x03,
        "channel": channel,
        "rx_channel": rx_channel,
        "data_rate": data_rate,
        "pa_level": 3,
        "retransmit_delay": 1,
        "retransmit_count": 3,
        "tx_ok_count": 100,
        "tx_fail_count": 2,
        "rx_count": 25,
        "radio_reinit_count": reinits,
        "addr_suffix": base_station.DEFAULT_ADDR_SUFFIX,
        "base_addr_lsb": base_station.DEFAULT_BASE_ADDR_LSB,
    }


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

    def test_config_matches_the_bitproto_encoder(self):
        message = RobotConfig()
        message.header.msg_type = MSG_TYPE_CONFIG
        message.header.robot_id = 5
        message.header.timestamp = 0xAABBCCDD
        message.config_flags = CONFIG_FLAG_RUN_CALIBRATION | CONFIG_FLAG_CALIBRATE_MAG
        message.param = 1234
        encoded = radio.encode_config(
            message.header.robot_id,
            message.config_flags,
            message.param,
            timestamp=message.header.timestamp,
        )
        self.assertEqual(len(encoded), radio.CONFIG_SIZE)
        self.assertEqual(encoded, message.encode())

    def test_set_channel_config_encoding(self):
        encoded = radio.encode_config(2, radio.CONFIG_FLAG_SET_CHANNEL, 114, timestamp=100)
        self.assertEqual(encoded[5], 32)
        self.assertEqual(struct.unpack_from("<H", encoded, 6)[0], 114)

    def test_telemetry_request_matches_the_bitproto_encoder(self):
        message = RobotTelemetryRequest()
        message.header.msg_type = MSG_TYPE_TELEMETRY_REQ
        message.header.robot_id = 4
        message.header.timestamp = 0x0A0B0C0D
        self.assertEqual(
            radio.encode_telemetry_request(4, message.header.timestamp), message.encode()
        )

    def test_telemetry_request_rejects_broadcast(self):
        with self.assertRaises(ValueError):
            radio.encode_telemetry_request(radio.ROBOT_ID_BROADCAST, 0)


class BaseStationCodecTests(unittest.TestCase):
    def test_base_info_and_scan_result_roundtrip_via_cobs(self):
        raw_info = base_station._INFO_STRUCT.pack(
            base_station.PROTO_VERSION,
            3,
            104,
            1,
            3,
            1,
            3,
            500,
            3,
            120,
            2,
            base_station.DEFAULT_ADDR_SUFFIX,
            0xB4,
            118,
        )
        frame = proto.encode_frame(base_station.BS_MSG_INFO, raw_info)
        dec = proto.FrameDecoder()
        frames = dec.feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], base_station.BS_MSG_INFO)
        parsed = base_station.parse_info(frames[0][1])
        self.assertEqual(parsed["channel"], 104)
        self.assertEqual(parsed["rx_channel"], 118)
        self.assertTrue(parsed["dual_radio"])
        self.assertEqual(parsed["radio_ready_mask"], 3)
        self.assertEqual(parsed["tx_ok_count"], 500)
        self.assertEqual(parsed["radio_reinit_count"], 2)

        busy = bytes((i % 101 for i in range(126)))
        raw_scan = base_station._SCAN_RES_STRUCT.pack(0, 126, 104, busy, 118)
        scan_frame = proto.encode_frame(base_station.BS_MSG_SCAN_RESULT, raw_scan)
        scan_frames = dec.feed(scan_frame)
        self.assertEqual(len(scan_frames), 1)
        scan = base_station.parse_scan_result(scan_frames[0][1])
        self.assertEqual(scan["best_channel"], 104)
        self.assertEqual(scan["best_rx_channel"], 118)
        self.assertEqual(len(scan["busy_pct"]), 126)

    def test_rf_tx_and_status_framing(self):
        cmd = radio.encode_command(1, vx=300, timestamp=55)
        frame = base_station.encode_rf_tx(9, 1, cmd)
        dec = proto.FrameDecoder()
        out = dec.feed(frame)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0][0], base_station.BS_MSG_RF_TX)

        frame_ch = base_station.encode_rf_tx_on_channel(10, 1, 76, cmd)
        out_ch = dec.feed(frame_ch)
        self.assertEqual(len(out_ch), 1)
        self.assertEqual(out_ch[0][0], base_station.BS_MSG_RF_TX_CH)
        self.assertEqual(out_ch[0][1][4], 76)

        self.assertEqual(base_station.encode_channel_pair_param(104, 104), 104)
        self.assertEqual(base_station.encode_channel_pair_param(104, 118), 0x8000 | (118 << 8) | 104)
        self.assertEqual(base_station.encode_channel_pair_param(104, 0), 0x8000 | 104)

        raw_st = base_station._TX_STATUS_STRUCT.pack(9, 1, base_station.BS_TX_STATUS_OK, 1, 410)
        parsed = base_station.parse_tx_status(raw_st)
        self.assertEqual(parsed["seq"], 9)
        self.assertEqual(parsed["retries"], 1)
        self.assertEqual(parsed["tx_time_us"], 410)

    def test_base_station_link_open_handshake_and_close(self):
        raw_info = base_station._INFO_STRUCT.pack(
            base_station.PROTO_VERSION,
            3,
            76,
            1,
            3,
            1,
            3,
            10,
            0,
            5,
            0,
            base_station.DEFAULT_ADDR_SUFFIX,
            0xB4,
            90,
        )
        reply = proto.encode_frame(base_station.BS_MSG_INFO, raw_info)

        class FakeSerial:
            def __init__(self):
                self.is_open = True
                self.rx = bytearray(reply)
                self.written = bytearray()

            @property
            def in_waiting(self):
                return len(self.rx)

            def reset_input_buffer(self):
                pass

            def write(self, data):
                self.written += data
                return len(data)

            def flush(self):
                pass

            def read(self, n):
                chunk = bytes(self.rx[:n])
                del self.rx[:n]
                return chunk

            def close(self):
                self.is_open = False

        fake_ser = FakeSerial()
        with patch.object(base_station, "open_serial_no_reset", return_value=fake_ser):
            link = base_station.BaseStationLink("/dev/ttyUSB1")
            info = link.open()
            self.assertEqual(info["channel"], 76)
            self.assertTrue(info["radio_ready"])
            self.assertTrue(link.is_open)
            link.close()
            self.assertFalse(link.is_open)


class TelemetryDecodingTests(unittest.TestCase):
    def test_decodes_every_telemetry_field(self):
        decoded = radio.decode_telemetry(make_telemetry())
        self.assertEqual(decoded["robot_id"], 2)
        self.assertEqual(decoded["battery_pct"], 87)
        self.assertAlmostEqual(decoded["kicker_voltage_v"], 16.5)

    def test_rejects_short_and_wrong_type(self):
        self.assertIsNone(radio.decode_telemetry(b""))
        frame = bytearray(make_telemetry())
        frame[0] = (2 << 4) | MSG_TYPE_COMMAND
        self.assertIsNone(radio.decode_telemetry(bytes(frame)))

    def test_error_flag_names(self):
        self.assertEqual(radio.error_flag_names(0), [])
        self.assertEqual(
            radio.error_flag_names(1 | 64 | 2048),
            ["Comunicação", "Encoder", "Motor 3"],
        )


class AdapterDiscoveryTests(unittest.TestCase):
    def test_find_adapters_probes_serial_ports_for_base_station(self):
        ports = [
            FakePort("/dev/ttyUSB0", vid=0x10C4, pid=0xEA60),
            FakePort("/dev/ttyUSB1", vid=0x1A86, pid=0x7523),
        ]

        def fake_probe(dev, baud=radio.BAUD):
            return make_base_info(channel=114) if dev == "/dev/ttyUSB1" else None

        with (
            patch.object(radio.serial.tools.list_ports, "comports", return_value=ports),
            patch.object(base_station, "probe_base_station", side_effect=fake_probe),
        ):
            adapters = radio.find_adapters()
        self.assertEqual(len(adapters), 1)
        self.assertEqual(adapters[0][0], "/dev/ttyUSB1")
        self.assertIn("ch 114", adapters[0][1])

    def test_discovery_lists_all_usb_serial_candidates(self):
        ports = [
            FakePort("/dev/ttyUSB0", vid=0x10C4, pid=0xEA60),
            FakePort("/dev/ttyUSB1", vid=0x1A86, pid=0x7523),
        ]
        with patch.object(discovery.serial.tools.list_ports, "comports", return_value=ports):
            candidates = discovery.list_candidate_ports()
        self.assertEqual([d for d, _ in candidates], ["/dev/ttyUSB0", "/dev/ttyUSB1"])


class PickerRadioTests(unittest.TestCase):
    def test_radio_robot_id_is_clamped_to_the_protocol_range(self):
        with patch.object(picker.dpg, "get_value", return_value=15):
            self.assertEqual(picker.radio_robot_id(), picker.RADIO_ROBOT_MAX)
        with patch.object(picker.dpg, "get_value", return_value=-4):
            self.assertEqual(picker.radio_robot_id(), 0)
        with patch.object(picker.dpg, "get_value", return_value=3):
            self.assertEqual(picker.radio_robot_id(), 3)


class RadioSessionTests(unittest.TestCase):
    def setUp(self):
        self.session = radio.RadioSession("/dev/ttyUSB1", robot_id=2)

    def test_keyboard_velocity_bounds(self):
        self.assertEqual(radio.keyboard_velocity(1, 0, 0, 10), (241, 0, 0))
        self.assertEqual(radio.keyboard_velocity(1, -1, 0, 50), (121, -121, 0))
        self.assertEqual(radio.keyboard_velocity(0, 0, -1, 50), (0, 0, -178))
        self.assertEqual(radio.keyboard_velocity(1, 0, 1, 50), (0, 0, 0))

    def test_next_frame_prioritizes_queued_config(self):
        self.session.send_config(CONFIG_FLAG_SET_ID, param=4)
        rid, frame = self.session._next_frame()
        self.assertEqual(rid, 2)
        self.assertEqual(len(frame), radio.CONFIG_SIZE)
        rid2, frame2 = self.session._next_frame()
        self.assertEqual(rid2, 2)
        self.assertEqual(len(frame2), radio.COMMAND_SIZE)

    def test_probe_matches_reply_from_base_station(self):
        fake_link = Mock()
        fake_link.send_rf.return_value = {
            "seq": 1,
            "target_robot_id": 2,
            "status": base_station.BS_TX_STATUS_OK,
            "retries": 0,
            "tx_time_us": 400,
        }
        fake_link.wait_rf_rx.return_value = {
            "rx_timestamp_ms": 10,
            "pipe": 1,
            "payload_len": radio.TELEMETRY_SIZE,
            "payload": make_telemetry(robot_id=2, timestamp=0x11223344),
        }
        self.session._link = fake_link
        with patch.object(radio, "_timestamp_ms", return_value=0x11223344):
            tel = self.session.probe(2, timeout_s=0.05)
        self.assertIsNotNone(tel)
        self.assertEqual(tel["robot_id"], 2)
        self.assertEqual(self.session.snapshot()["tx_count"], 1)

    def test_base_tasks_scan_and_channel_switch(self):
        fake_link = Mock()
        fake_link.scan_channels.return_value = {
            "start_channel": 0,
            "count": 126,
            "best_channel": 114,
            "busy_pct": [0] * 126,
        }
        fake_link.send_rf.return_value = {
            "seq": 1,
            "target_robot_id": 0,
            "status": base_station.BS_TX_STATUS_OK,
            "retries": 0,
            "tx_time_us": 380,
        }
        fake_link.set_config.return_value = make_base_info(channel=114)
        self.session._link = fake_link
        self.session._base_info = make_base_info(channel=76)

        self.session.request_scan()
        self.session.switch_channel(114, notify_robots=True)
        self.session._handle_base_tasks()

        snap = self.session.snapshot()
        self.assertEqual(snap["scan_result"]["best_channel"], 114)
        self.assertEqual(snap["base_info"]["channel"], 114)
        fake_link.set_config.assert_called_once()

    def test_connect_raises_radio_error_on_missing_port(self):
        with self.assertRaises(radio.RadioError):
            radio.RadioSession.connect("/dev/ttyUSB99", robot_id=0)


if __name__ == "__main__":
    unittest.main()
