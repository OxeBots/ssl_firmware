"""Regression checks for the control-calibration and wheel-position tab."""

import struct
import time
import unittest
from unittest.mock import Mock

import dearpygui.dearpygui as dpg

from robot_bench import app, config, demo, protocol, tuning
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import (
    DebugPosition,
    DebugPositionControl,
    DebugTuning,
)


def live_state(*, confirm=False, demo_mode=False):
    """Bench state on a live link with an idle control-calibration frame."""
    state = BenchState()
    state.reader = Mock()
    state.connected = "/dev/ttyUSB0"
    state.streaming = True
    state.last_state_t = time.monotonic()
    state.valid_mask = 0xF
    state.demo = demo_mode
    state.set_tuning_confirm(confirm)
    state.update_tuning(demo.idle_tuning_frame(1))
    return state


class TuningWireFormatTests(unittest.TestCase):
    def test_tuning_parser_matches_generated_bitproto_layout(self):
        generated = DebugTuning(
            state=protocol.TUNING_RUNNING,
            wheel=2,
            phase=protocol.TUNING_PHASE_VALIDATE,
            progress=57,
            result_mask=0b0011,
            error_code=protocol.TUNING_ERROR_NONE,
            kp=[1500, 0, 250, -100],
            ki=[50, 0, 0, 0],
            kd=[25, 0, 0, 0],
            feedforward=[176, 0, 200, 1023],
            velocity_ff=[5400, 0, 0, 100],
            rise_ms=[200, 210, 220, 230],
            overshoot_pct=[0, 1, 2, 3],
            ss_error_pct=[0, 2, 4, 6],
            sequence=999,
        )
        message = bytes(generated.encode())
        self.assertEqual(len(message), protocol.TUNING_SIZE)
        expected = struct.pack(
            "<6B20h4H4B4BI",
            1,
            2,
            3,
            57,
            0b0011,
            0,
            1500,
            0,
            250,
            -100,
            50,
            0,
            0,
            0,
            25,
            0,
            0,
            0,
            176,
            0,
            200,
            1023,
            5400,
            0,
            0,
            100,
            200,
            210,
            220,
            230,
            0,
            1,
            2,
            3,
            0,
            2,
            4,
            6,
            999,
        )
        self.assertEqual(message, expected)
        parsed = protocol.parse_tuning(message)
        self.assertEqual(parsed["state"], protocol.TUNING_RUNNING)
        self.assertEqual(parsed["wheel"], 2)
        self.assertEqual(parsed["phase"], protocol.TUNING_PHASE_VALIDATE)
        self.assertEqual(parsed["progress"], 57)
        self.assertEqual(parsed["result_mask"], 0b0011)
        self.assertEqual(parsed["kp"], [15.0, 0.0, 2.5, -1.0])
        self.assertEqual(parsed["ki"], [0.5, 0.0, 0.0, 0.0])
        self.assertEqual(parsed["kd"], [0.0025, 0.0, 0.0, 0.0])
        self.assertEqual(parsed["feedforward"], [176.0, 0.0, 200.0, 1023.0])
        self.assertEqual(parsed["velocity_ff"], [54.0, 0.0, 0.0, 1.0])
        self.assertEqual(parsed["rise_ms"], [200, 210, 220, 230])
        self.assertEqual(parsed["overshoot_pct"], [0, 1, 2, 3])
        self.assertEqual(parsed["ss_error_pct"], [0, 2, 4, 6])
        self.assertEqual(parsed["sequence"], 999)
        self.assertIsNone(protocol.parse_tuning(message[:-1]))

    def test_position_parser_matches_generated_bitproto_layout(self):
        generated = DebugPosition(
            active_mask=0b0101,
            done_mask=0b0001,
            error_code=protocol.POSITION_ERROR_TIMEOUT,
            angle_mdeg=[-90500, 0, 180000, 3609000],
            target_mdeg=[90500, 0, -180000, 7200000],
            sequence=42,
        )
        message = bytes(generated.encode())
        self.assertEqual(len(message), protocol.POSITION_SIZE)
        self.assertEqual(
            message,
            struct.pack(
                "<3B4i4iI",
                0b0101,
                0b0001,
                protocol.POSITION_ERROR_TIMEOUT,
                -90500,
                0,
                180000,
                3609000,
                90500,
                0,
                -180000,
                7200000,
                42,
            ),
        )
        parsed = protocol.parse_position(message)
        self.assertEqual(parsed["active_mask"], 0b0101)
        self.assertEqual(parsed["done_mask"], 0b0001)
        self.assertEqual(parsed["error_code"], protocol.POSITION_ERROR_TIMEOUT)
        self.assertEqual(parsed["angle_deg"], [-90.5, 0.0, 180.0, 3609.0])
        self.assertEqual(parsed["target_deg"], [90.5, 0.0, -180.0, 7200.0])
        self.assertEqual(parsed["sequence"], 42)
        self.assertIsNone(protocol.parse_position(message[:-1]))

    def test_position_control_encoder_matches_generated_bitproto_layout(self):
        angles = [-90.125, 0.0, 12.5, 359.999]
        generated = DebugPositionControl(
            op=protocol.POSITION_OP_MOVE,
            mode=protocol.POSITION_MODE_ABSOLUTE,
            wheel_mask=0b1111,
            angle_mdeg=[int(round(angle * 1000.0)) for angle in angles],
            max_rpm=300,
            sequence=7,
        )
        expected = bytes(generated.encode())
        self.assertEqual(len(expected), protocol.POSITION_CONTROL_SIZE)
        self.assertEqual(
            protocol.encode_position_control(
                protocol.POSITION_OP_MOVE,
                protocol.POSITION_MODE_ABSOLUTE,
                0b1111,
                angles,
                300,
                7,
            ),
            expected,
        )


class TuningHelperTests(unittest.TestCase):
    def test_freshness_and_running_track_the_stale_window(self):
        state = BenchState()
        now = 100.0
        self.assertFalse(tuning.tuning_fresh(state, now))
        state.update_tuning(demo.idle_tuning_frame(1))
        state.last_tuning_t = now
        self.assertTrue(tuning.tuning_fresh(state, now))
        self.assertFalse(tuning.tuning_running(state, now))
        state.tuning = demo.tuning_frame(2, state=protocol.TUNING_RUNNING, wheel=1)
        self.assertTrue(tuning.tuning_running(state, now))
        state.last_tuning_t = now - config.TUNING_STALE_S - 0.1
        self.assertFalse(tuning.tuning_fresh(state, now))
        self.assertFalse(tuning.tuning_running(state, now))

    def test_start_button_needs_confirmation_live_link_and_idle_routine(self):
        now = 100.0
        state = BenchState()
        state.update_tuning(demo.idle_tuning_frame(1))
        state.last_tuning_t = now
        self.assertFalse(tuning.tuning_start_enabled(state, now))
        state.set_tuning_confirm(True)
        self.assertFalse(tuning.tuning_start_enabled(state, now))
        state.reader = Mock()
        state.connected = "/dev/ttyUSB0"
        state.streaming = True
        state.last_state_t = now
        self.assertTrue(tuning.tuning_start_enabled(state, now))
        state.tuning = demo.tuning_frame(2, state=protocol.TUNING_RUNNING, wheel=1)
        self.assertFalse(tuning.tuning_start_enabled(state, now))
        state.tuning = demo.idle_tuning_frame(3)
        state.last_state_t = now - config.SENSOR_STALE_S - 0.1
        self.assertFalse(tuning.tuning_start_enabled(state, now))
        state.last_state_t = now
        # The wiring calibration blocks the control calibration.
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        state.last_calibration_t = time.monotonic()
        self.assertFalse(tuning.tuning_start_enabled(state, now))

    def test_start_button_works_in_demo_without_a_link(self):
        now = 100.0
        state = live_state(demo_mode=True)
        state.connected = None
        state.streaming = False
        state.last_tuning_t = now
        state.set_tuning_confirm(True)
        self.assertTrue(tuning.tuning_start_enabled(state, now))
        state.demo_tuning_start = now
        self.assertFalse(tuning.tuning_start_enabled(state, now))

    def test_stop_button_follows_running_and_demo_runs(self):
        now = 100.0
        state = live_state()
        state.last_tuning_t = now
        self.assertFalse(tuning.tuning_stop_enabled(state, now))
        state.tuning = demo.tuning_frame(2, state=protocol.TUNING_RUNNING, wheel=1)
        self.assertTrue(tuning.tuning_stop_enabled(state, now))
        state.tuning = demo.idle_tuning_frame(3)
        state.demo = True
        state.demo_tuning_start = now
        self.assertTrue(tuning.tuning_stop_enabled(state, now))

    def test_status_text_covers_every_routine_state(self):
        now = 100.0
        state = BenchState()
        self.assertEqual(
            tuning.tuning_status_text(state, now),
            (tuning.STATUS_WAITING, config.TEXT_DIM),
        )
        state.last_tuning_t = now
        state.tuning = demo.idle_tuning_frame(1)
        self.assertEqual(tuning.tuning_status_text(state, now), (tuning.STATUS_IDLE, config.TEXT_DIM))
        state.tuning = demo.tuning_frame(2, state=protocol.TUNING_RUNNING, progress=40)
        self.assertEqual(
            tuning.tuning_status_text(state, now), (tuning.STATUS_RUNNING.format(40), config.ACCENT)
        )
        state.tuning = demo.done_tuning_frame(3)
        self.assertEqual(tuning.tuning_status_text(state, now), (tuning.STATUS_DONE, config.OK_COLOR))
        state.tuning = demo.tuning_frame(
            4, state=protocol.TUNING_FAILED, error_code=protocol.TUNING_ERROR_NO_MOVEMENT
        )
        self.assertEqual(
            tuning.tuning_status_text(state, now),
            (tuning.STATUS_FAILED.format(tuning.TUNING_ERROR_NAMES[1]), config.BAD_COLOR),
        )
        state.last_tuning_t = now - config.TUNING_STALE_S - 1.0
        self.assertEqual(tuning.tuning_status_text(state, now), (tuning.STATUS_STALE, config.BAD_COLOR))

    def test_phase_and_result_text(self):
        self.assertEqual(tuning.phase_text(None), "Fase: —")
        frame = demo.tuning_frame(
            1, state=protocol.TUNING_RUNNING, wheel=2, phase=protocol.TUNING_PHASE_VALIDATE
        )
        self.assertEqual(tuning.phase_text(frame), "Roda M3 · Validação")
        self.assertEqual(tuning.result_state_text(None, 0), tuning.RESULT_PENDING)
        self.assertEqual(tuning.result_state_text(frame, 2), tuning.RESULT_RUNNING)
        self.assertEqual(tuning.result_state_text(frame, 0), tuning.RESULT_PENDING)
        frame["result_mask"] = 0b0010
        self.assertEqual(tuning.result_state_text(frame, 1), tuning.RESULT_OK)
        self.assertEqual(tuning.result_state_color(frame, 1), config.OK_COLOR)
        done = demo.done_tuning_frame(2)
        self.assertEqual(tuning.result_state_text(done, 0), tuning.RESULT_OK)
        failed = demo.tuning_frame(3, state=protocol.TUNING_FAILED, result_mask=0)
        self.assertEqual(tuning.result_state_text(failed, 0), tuning.RESULT_FAILED)
        self.assertEqual(tuning.result_state_color(failed, 0), config.BAD_COLOR)

    def test_metric_text_formats(self):
        frame = demo.tuning_frame(
            1,
            rise_ms=[300, 0, 0, 0],
            overshoot_pct=[3, 0, 0, 0],
            ss_error_pct=[2, 0, 0, 0],
        )
        self.assertEqual(tuning.gain_text(58.0), "58.00")
        self.assertEqual(tuning.gain_text(0.0, 3), "0.000")
        self.assertEqual(tuning.rise_text(frame, 0), "300 ms")
        self.assertEqual(tuning.rise_text(frame, 1), "—")
        self.assertEqual(tuning.rise_text(None, 0), "—")
        self.assertEqual(tuning.percent_text(frame["overshoot_pct"], 0), "3 %")
        self.assertEqual(tuning.percent_text(frame["overshoot_pct"], 1), "—")
        self.assertEqual(tuning.percent_text(None, 0), "—")

    def test_display_smoothing_suppresses_single_spikes(self):
        samples = [(0.0, 100.0, 100.0), (0.1, 100.0, 100.0), (0.2, 140.0, 100.0), (0.3, 100.0, 100.0)]
        series = tuning.smooth_history(samples, 1, window=3)
        self.assertEqual(len(series), len(samples))
        self.assertEqual([round(item[1], 1) for item in series], [100.0, 100.0, 100.0, 100.0])
        self.assertEqual(tuning.smooth_history([], 1), [])
        flat = tuning.smooth_history([(index * 0.1, 50.0, 50.0) for index in range(5)], 1)
        self.assertEqual([round(item[1], 1) for item in flat], [50.0] * 5)

    def test_tuning_errors_have_readable_text(self):
        for code, text in tuning.TUNING_ERROR_NAMES.items():
            self.assertEqual(tuning.tuning_error_text(code), text)
        self.assertEqual(tuning.tuning_error_text(99), "Código 99")


class PositionHelperTests(unittest.TestCase):
    def test_status_text_covers_position_states(self):
        now = 100.0
        state = BenchState()
        self.assertEqual(tuning.position_status_text(state, now), (tuning.POSITION_IDLE, config.TEXT_DIM))
        state.last_position_t = now
        state.position = demo.position_frame(
            1, active_mask=0b0011, done_mask=0b0001, angle_deg=[0, 90, 0, 0], target_deg=[0, 90, 0, 0]
        )
        self.assertEqual(tuning.position_status_text(state, now), (tuning.POSITION_ACTIVE, config.ACCENT))
        state.position = demo.position_frame(
            2, active_mask=0b0011, done_mask=0b0011, angle_deg=[0, 90, 0, 0], target_deg=[0, 90, 0, 0]
        )
        self.assertEqual(tuning.position_status_text(state, now), (tuning.POSITION_DONE, config.OK_COLOR))
        state.position = demo.position_frame(3, error_code=protocol.POSITION_ERROR_TIMEOUT)
        self.assertEqual(
            tuning.position_status_text(state, now),
            (f"Falha · {tuning.POSITION_ERROR_NAMES[1]}", config.BAD_COLOR),
        )
        state.last_position_t = now - config.POSITION_STALE_S - 1.0
        self.assertEqual(tuning.position_status_text(state, now), (tuning.POSITION_STALE, config.BAD_COLOR))

    def test_position_rows_show_angles_errors_and_status(self):
        pose = demo.position_frame(
            1,
            active_mask=0b0101,
            done_mask=0b0001,
            angle_deg=[10.0, 0.0, -45.0, 0.0],
            target_deg=[10.0, 0.0, -40.0, 0.0],
        )
        first = tuning.position_row(pose, 0)
        self.assertEqual(first["angle"], "10.0°")
        self.assertEqual(first["status"], "Alvo atingido")
        self.assertEqual(first["color"], config.OK_COLOR)
        third = tuning.position_row(pose, 2)
        self.assertEqual(third["error"], "+5.0°")
        self.assertEqual(third["status"], "Movendo")
        self.assertEqual(third["color"], config.ACCENT)
        second = tuning.position_row(pose, 1)
        self.assertEqual(second["status"], "Parado")
        self.assertEqual(second["color"], config.TEXT_DIM)
        idle = tuning.position_row(None, 0)
        self.assertEqual(idle["angle"], "—")

    def test_move_buttons_need_link_routines_and_calibrated_encoder(self):
        now = 100.0
        state = BenchState()
        self.assertFalse(tuning.move_enabled(state, now, 0))
        state.demo = True
        self.assertTrue(tuning.move_enabled(state, now, 0))
        state.demo = False
        state.reader = Mock()
        state.connected = "/dev/ttyUSB0"
        state.streaming = True
        state.last_state_t = now
        state.valid_mask = 0b0011
        self.assertTrue(tuning.move_enabled(state, now, 1))
        self.assertFalse(tuning.move_enabled(state, now, 2))
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        state.last_calibration_t = time.monotonic()
        self.assertFalse(tuning.move_enabled(state, now, 1))

    def test_stop_position_enabled_only_with_active_move(self):
        now = 100.0
        state = BenchState()
        self.assertFalse(tuning.position_stop_enabled(state, now))
        state.last_position_t = now
        state.position = demo.position_frame(1, active_mask=0b0001)
        self.assertTrue(tuning.position_stop_enabled(state, now))
        state.position = demo.position_frame(2)
        self.assertFalse(tuning.position_stop_enabled(state, now))
        state.demo = True
        state.demo_position = {"wheel_mask": 1}
        self.assertTrue(tuning.position_stop_enabled(state, now))


class CommandTests(unittest.TestCase):
    def test_start_and_stop_tuning_send_control_ops(self):
        state = live_state(confirm=True)
        self.assertTrue(state.start_tuning())
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_START_TUNING, 1)
        )
        state.stop_tuning()
        self.assertEqual(state.control_sequence, 2)
        state.reader.send_frame.assert_called_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_TUNING, 2)
        )

    def test_start_tuning_refuses_without_confirmation_or_link(self):
        state = live_state(confirm=False)
        self.assertFalse(state.start_tuning())
        state.set_tuning_confirm(True)
        state.last_state_t = 0.0
        self.assertFalse(state.start_tuning())
        state.reader.send_frame.assert_not_called()

    def test_demo_tuning_run_starts_and_stops_without_frames(self):
        state = live_state(confirm=True, demo_mode=True)
        self.assertTrue(state.start_tuning())
        self.assertIsNotNone(state.demo_tuning_start)
        state.reader.send_frame.assert_not_called()
        state.stop_tuning()
        self.assertIsNone(state.demo_tuning_start)

    def test_position_move_sends_relative_or_absolute_frame(self):
        state = live_state()
        angles = [0.0, 0.0, 45.0, 0.0]
        self.assertTrue(state.start_position_move(angles, 1 << 2, False, 250))
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_POSITION_CONTROL,
            protocol.encode_position_control(
                protocol.POSITION_OP_MOVE, protocol.POSITION_MODE_RELATIVE, 0b0100, angles, 250, 1
            ),
        )
        self.assertEqual(state.position_request["wheel_mask"], 0b0100)
        state.stop_position()
        self.assertEqual(state.control_sequence, 2)
        state.reader.send_frame.assert_called_with(
            protocol.MSG_POSITION_CONTROL,
            protocol.encode_position_control(
                protocol.POSITION_OP_STOP, protocol.POSITION_MODE_RELATIVE, 0, [0.0] * 4, 0, 2
            ),
        )

    def test_position_move_requires_calibrated_encoders(self):
        state = live_state()
        state.valid_mask = 0b0011
        self.assertFalse(state.start_position_move([10.0] * 4, 1 << 3, False, 0))
        state.reader.send_frame.assert_not_called()
        self.assertTrue(state.start_position_move([10.0] * 4, 1 << 1, False, 0))

    def test_position_move_cancelled_by_velocity_targets_and_stop(self):
        state = live_state(confirm=True, demo_mode=True)
        self.assertTrue(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))
        self.assertIsNotNone(state.demo_position)
        state.set_targets([100.0, 0.0, 0.0, 0.0])
        self.assertIsNone(state.demo_position)
        self.assertTrue(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, True))
        state.stop_motors()
        self.assertIsNone(state.demo_position)

    def test_position_move_refused_during_calibration(self):
        state = live_state()
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        state.last_calibration_t = time.monotonic()
        self.assertFalse(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))
        state.calibration = None
        state.last_calibration_t = 0.0
        state.update_tuning(demo.tuning_frame(2, state=protocol.TUNING_RUNNING))
        self.assertFalse(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))


class DemoFrameTests(unittest.TestCase):
    def test_tuning_frames_progress_and_finish(self):
        idle = demo.idle_tuning_frame(1)
        self.assertEqual(idle["state"], protocol.TUNING_IDLE)
        self.assertEqual(idle["feedforward"], [0.0] * 4)
        start = demo.running_tuning_frame(0.0, 1)
        self.assertEqual(start["state"], protocol.TUNING_RUNNING)
        self.assertEqual(start["wheel"], 0)
        self.assertEqual(start["progress"], 0)
        middle = demo.running_tuning_frame(demo.DEMO_TUNING_WHEEL_S * 2.5, 2)
        self.assertEqual(middle["wheel"], 2)
        self.assertEqual(middle["result_mask"], 0b0011)
        self.assertGreater(middle["progress"], 50)
        self.assertGreater(middle["kp"][2], 0.0)
        end = demo.running_tuning_frame(demo.DEMO_TUNING_TOTAL_S, 3)
        self.assertEqual(end["state"], protocol.TUNING_DONE)
        self.assertEqual(end["progress"], 100)
        self.assertEqual(end["result_mask"], 0xF)
        self.assertEqual(len(end["kp"]), 4)
        self.assertTrue(all(value > 0 for value in end["feedforward"]))

    def test_idle_frame_keeps_stored_parameters(self):
        tuned = demo.idle_tuning_frame(1, tuned=True)
        self.assertEqual(tuned["state"], protocol.TUNING_IDLE)
        self.assertEqual(tuned["result_mask"], 0xF)
        self.assertTrue(all(value > 0 for value in tuned["kp"]))
        untuned = demo.idle_tuning_frame(2, tuned=False)
        self.assertEqual(untuned["result_mask"], 0)
        self.assertTrue(all(value == 0 for value in untuned["kp"]))

    def test_position_frame_defaults_are_idle(self):
        frame = demo.position_frame(1)
        self.assertEqual(frame["active_mask"], 0)
        self.assertEqual(frame["done_mask"], 0)
        self.assertEqual(frame["error_code"], protocol.POSITION_ERROR_NONE)
        self.assertEqual(frame["angle_deg"], [0.0] * 4)
        self.assertEqual(frame["target_deg"], [0.0] * 4)


class TabLayoutTests(unittest.TestCase):
    def test_tuning_tab_is_appended_and_bound_to_f7(self):
        self.assertEqual(config.TAB_TITLES[config.TAB_TUNING], "Controle")
        self.assertEqual(len(app.TAB_TAGS), len(config.TAB_TITLES))
        self.assertEqual(app.KEY_ACTIONS[dpg.mvKey_F7], ("tab", config.TAB_TUNING))
        self.assertEqual(app.TAB_TAGS[config.TAB_TUNING], "tab_tuning")


if __name__ == "__main__":
    unittest.main()
