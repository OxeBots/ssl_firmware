"""Check benchmark framing, reply matching, and radio session ownership."""

import socket
import threading
import time
import unittest
from unittest.mock import Mock, patch

import dearpygui.dearpygui as dpg

from robot_bench import bench_panel
from robot_bench import communication_bench as bench
from robot_bench import protocol, radio
from scripts import ssl_robot_protocol_bp as bp


class BenchTests(unittest.TestCase):
    def test_panel_starts_the_selected_radio_or_uart_workload(self):
        dpg.create_context()
        self.addCleanup(dpg.destroy_context)
        panel = bench_panel.BenchPanel()
        with dpg.window():
            panel.build()

        session = Mock()
        panel.update(session, {"connected": True, "benchmarking": False, "robot_id": 3}, {"body_h": 400})
        self.assertEqual(dpg.get_value(bench_panel.TRAFFIC), bench_panel.TELEMETRY)
        self.assertEqual(dpg.get_value(bench_panel.COMMAND_FORMAT), "Individual")
        self.assertFalse(dpg.get_item_configuration(bench_panel.OUTPUT)["show"])

        for traffic, command_format, button, expected in (
            (bench_panel.TELEMETRY, "Equipe", bench_panel.START, "telemetry"),
            (bench_panel.WITH_COMMANDS, "Individual", bench_panel.START, "unicast"),
            (bench_panel.WITH_COMMANDS, "Equipe", bench_panel.START, "team"),
            (bench_panel.WITH_COMMANDS, "Equipe", bench_panel.UART_TEST, "loopback"),
        ):
            with self.subTest(mode=expected):
                dpg.set_value(bench_panel.TRAFFIC, traffic)
                dpg.set_value(bench_panel.COMMAND_FORMAT, command_format)
                dpg.get_item_callback(button)(button, None, dpg.get_item_user_data(button))
                run = session.start_benchmark.call_args.args[0]
                self.assertEqual(run.options.mode, expected)
                self.assertEqual(run.options.robot_ids, (3,))

    def test_gateway_echo_is_separate_from_radio_telemetry(self):
        reply = bp.RobotTelemetry()
        reply.header.msg_type = bp.MSG_TYPE_TELEMETRY
        reply.header.timestamp = 345
        regular_payload = reply.encode()
        regular = radio.encode_link_frame(regular_payload)
        reply.header.timestamp = 678
        echo = radio.encode_link_frame(reply.encode(), radio.ESP_LINK_LOOPBACK_TYPE)
        self.assertEqual(radio.TelemetryStream().feed(echo + regular), [radio.decode_telemetry(regular_payload)])
        loopback = radio.TelemetryStream(radio.ESP_LINK_LOOPBACK_TYPE)
        self.assertEqual(loopback.feed(echo + regular), [radio.decode_telemetry(reply.encode())])

    def test_team_stop_matches_generated_bitproto_and_splits_large_groups(self):
        run = bench.Benchmark(bench.Options(robot_ids=(5, 1, 6, 9), mode="team"))
        frames = list(run._commands(123))
        self.assertEqual(len(frames), 2)
        for frame, ids in zip(frames, ((5, 1, 6), (9,)), strict=True):
            header = bp.Header()
            header.msg_type = bp.MSG_TYPE_TEAM_VEL_CMD
            header.robot_id = len(ids)
            header.timestamp = 123
            expected = header.encode()
            for robot_id in ids:
                slot = bp.SingleRobotVelCmd()
                slot.robot_id = robot_id
                expected += slot.encode()
            self.assertEqual(frame, expected)

    def test_interval_and_robot_timestamp_matching_under_load(self):
        run = bench.Benchmark(bench.Options((0, 1), 10, 1, "unicast", 2, 75))
        frames = []
        reader, writer = socket.socketpair()
        with reader, writer:
            port = Mock(in_waiting=4096)
            port.fileno.side_effect = reader.fileno
            port.read.side_effect = reader.recv

            def answer(frame):
                kind, payload = protocol.FrameDecoder().feed(frame)[0]
                if kind == radio.ESP_LINK_LOOPBACK_TYPE:
                    echo = bp.RobotTelemetry()
                    echo.header.msg_type = bp.MSG_TYPE_TELEMETRY
                    echo.header.timestamp = int.from_bytes(payload[1:5], "little")
                    writer.sendall(radio.encode_link_frame(echo.encode(), kind))
                    return len(frame)
                self.assertEqual(kind, radio.ESP_LINK_FRAME_TYPE)
                if not frames:
                    writer.sendall(b"\xff\xe0\x00ESP_LINK ready\n")
                frames.append(payload)
                if payload[0] & 0x0F == bp.MSG_TYPE_TELEMETRY_REQ:
                    req = bp.RobotTelemetryRequest()
                    req.decode(payload)
                    reply = bp.RobotTelemetry()
                    reply.header.msg_type = bp.MSG_TYPE_TELEMETRY
                    # Only robot 0 answers with the correct ID and timestamp.
                    reply.header.robot_id = 0
                    reply.header.timestamp = req.header.timestamp
                    framed = radio.encode_link_frame(reply.encode())
                    writer.sendall(framed + framed)
                return len(frame)

            port.write.side_effect = answer
            run.run(port, threading.Event())

        result = run.result
        self.assertEqual((result.state, result.cycles, result.skipped), ("complete", 10, 0))
        self.assertEqual(result.gateway_restarts, 1)
        self.assertEqual(sum(p[0] & 0x0F == bp.MSG_TYPE_COMMAND for p in frames), 20)
        self.assertTrue(all(p[5:] == bytes(25) for p in frames if p[0] & 0x0F == bp.MSG_TYPE_COMMAND))
        robot0, robot1 = result.robots
        self.assertEqual((robot0.requests, robot0.replies, robot0.on_time, robot0.timed_out), (5, 5, 5, 0))
        self.assertEqual((robot1.requests, robot1.replies, robot1.timed_out), (5, 0, 5))
        self.assertLess(robot0.max_gap_ms, robot1.max_gap_ms)
        self.assertGreater(robot1.max_gap_ms, 850)

    def test_benchmark_suspends_controls_and_resumes_telemetry_on_same_session(self):
        session = radio.RadioSession("/dev/ttyUSB0", 3)
        session._ser = Mock()
        session._connected = True
        session._last_rx_t = time.monotonic()
        session.set_motion(vx=750)
        session.kick(100)
        run = Mock()
        session.start_benchmark(run)
        session.set_motion(vx=400)
        session.kick(200)
        session.send_config(radio.CONFIG_FLAG_CALIBRATE_MAG)
        self.assertFalse(session.set_robot_id(2))

        def stop_after_resume():
            session._closing.set()
            return []

        with (
            patch.object(session, "_send_frame", return_value=True) as send,
            patch.object(session, "_read_available", side_effect=stop_after_resume),
        ):
            session.run()

        run.run.assert_called_once_with(session._ser, session._closing)
        frames = [call.args[0] for call in send.call_args_list]
        self.assertEqual([frame[0] >> 4 for frame in frames], [3] * 2)
        self.assertEqual([frame[0] & 0x0F for frame in frames], [
            radio.MSG_TYPE_COMMAND, radio.MSG_TYPE_TELEMETRY_REQ,
        ])
        self.assertEqual(frames[0][5:], bytes(25))
        self.assertFalse(session.snapshot()["benchmarking"])
        self.assertFalse(session.snapshot()["pending_configs"])


if __name__ == "__main__":
    unittest.main()
