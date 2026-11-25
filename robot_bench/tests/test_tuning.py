"""Regression checks for the control-calibration content (Calibração > Controlador)."""

import struct
import time
import unittest
from unittest.mock import Mock, patch

from robot_bench import app, calibration, config, demo, protocol, tuning
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import DebugPosition, DebugPositionControl


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


class PositionWireFormatTests(unittest.TestCase):
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
    def test_only_encoder_calibration_requests_output_stage_readback(self):
        state = live_state()
        layout = app.compute_layout(1200, 800)
        with (
            patch.object(app.dpg, "configure_item"),
            patch.object(app.rendering, "set_text"),
            patch.object(app, "header_spacer_width", return_value=20),
            patch.object(app.motors, "leave"),
            patch.object(app.motors, "update"),
            patch.object(app.motors, "active_view", return_value=app.motors.MOTORS_VIEW_OVERVIEW),
            patch.object(app.motors, "sidebar_tab", return_value=app.motors.SIDEBAR_TUNING),
            patch.object(calibration, "update"),
            patch.object(calibration, "active_subtab", return_value=calibration.SUBTAB_ENCODERS),
        ):
            for tab, expect_readback in ((config.TAB_MOTORS, False), (config.TAB_CALIBRATION, True)):
                state.active_tab = tab
                app.refresh_dashboard(state, None, layout, time.monotonic(), 0, 0, 60)
                self.assertTrue(state.stream_mask & protocol.STREAM_CALIBRATION)
                self.assertEqual(bool(state.stream_mask & protocol.STREAM_ENCODER_STAGES), expect_readback)

    def test_control_calibration_subscribes_to_wheel_history_and_tuning(self):
        state = live_state()
        state.active_tab = config.TAB_CALIBRATION
        layout = app.compute_layout(1200, 800)
        with (
            patch.object(app.dpg, "configure_item"),
            patch.object(app.rendering, "set_text"),
            patch.object(app, "header_spacer_width", return_value=20),
            patch.object(app.motors, "leave"),
            patch.object(calibration, "update"),
            patch.object(calibration, "active_subtab", return_value=calibration.SUBTAB_CONTROL),
        ):
            app.refresh_dashboard(state, None, layout, time.monotonic(), 0, 0, 60)

        self.assertEqual(
            state.stream_mask,
            protocol.STREAM_CALIBRATION | protocol.STREAM_STATE | protocol.STREAM_TUNING,
        )

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
        self.assertEqual(
            tuning.tuning_status_text(state, now),
            (tuning.STATUS_DONE.format(demo.DEMO_MAX_RPM), config.OK_COLOR),
        )
        self.assertIn("138 RPM", tuning.STATUS_DONE.format(demo.DEMO_MAX_RPM))
        state.tuning = demo.tuning_frame(
            4, state=protocol.TUNING_FAILED, error_code=protocol.TUNING_ERROR_NO_MOVEMENT
        )
        self.assertEqual(
            tuning.tuning_status_text(state, now),
            (tuning.STATUS_FAILED.format(tuning.TUNING_ERROR_NAMES[1]), config.BAD_COLOR),
        )
        self.assertIn("dois sentidos", tuning.STATUS_DONE)
        self.assertIn("dois sentidos", tuning.STATUS_RUNNING)
        self.assertIn("dois sentidos", tuning.STATUS_IDLE)
        state.last_tuning_t = now - config.TUNING_STALE_S - 1.0
        self.assertEqual(tuning.tuning_status_text(state, now), (tuning.STATUS_STALE, config.BAD_COLOR))

    def test_phase_and_result_text(self):
        self.assertEqual(tuning.phase_text(None), "Fase: —")
        frame = demo.tuning_frame(
            1, state=protocol.TUNING_RUNNING, wheel=2, phase=protocol.TUNING_PHASE_VALIDATE
        )
        self.assertEqual(tuning.phase_text(frame), "Roda M3 · Validação")
        max_speed = demo.tuning_frame(
            1, state=protocol.TUNING_RUNNING, wheel=2, phase=protocol.TUNING_PHASE_MAX_SPEED
        )
        self.assertEqual(tuning.PHASE_NAMES[protocol.TUNING_PHASE_MAX_SPEED], "Velocidade máxima")
        self.assertEqual(tuning.phase_text(max_speed), "Roda M3 · Velocidade máxima")
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

    def test_partial_results_distinguish_saved_wheels_from_failures(self):
        now = 100.0
        state = BenchState()
        state.last_tuning_t = now
        state.tuning = demo.tuning_frame(
            1, state=protocol.TUNING_FAILED, error_code=protocol.TUNING_ERROR_UNSTABLE,
            result_mask=0b0101, max_rpm=479.8,
        )
        summary, color = tuning.tuning_status_text(state, now)
        self.assertIn("2/4 rodas salvas", summary)
        self.assertEqual(color, config.PWM_COLOR)
        self.assertEqual(tuning.result_state_text(state.tuning, 0), tuning.RESULT_OK)
        self.assertEqual(tuning.result_state_text(state.tuning, 1), tuning.RESULT_FAILED)
        self.assertEqual(tuning.max_speed_texts(state.tuning)[1], "Limite comum: 479.8 RPM")

    def test_speed_warning_preserves_saved_status(self):
        now = 100.0
        state = BenchState()
        state.last_tuning_t = now
        state.tuning = demo.done_tuning_frame(1)
        state.tuning["error_code"] = protocol.TUNING_ERROR_SPEED_MISMATCH
        self.assertEqual(
            tuning.tuning_status_text(state, now),
            (tuning.STATUS_SPEED_MISMATCH, config.PWM_COLOR),
        )
        self.assertEqual(tuning.result_state_text(state.tuning, 3), tuning.RESULT_OK)

    def test_metric_text_formats(self):
        frame = demo.tuning_frame(
            1,
            rise_ms=[300, 0, 0, 0],
            overshoot_pct=[3, 0, 0, 0],
            ss_error_pct=[2, 0, 0, 0],
            rise_ms_rev=[350, 0, 0, 0],
            overshoot_pct_rev=[4, 0, 0, 0],
            ss_error_pct_rev=[3, 0, 0, 0],
        )
        self.assertEqual(tuning.gain_text(58.0), "58.00")
        self.assertEqual(tuning.gain_text(0.0, 3), "0.000")
        self.assertEqual(tuning.rise_text(frame, 0), "300 ms")
        self.assertEqual(tuning.rise_text(frame, 1), "—")
        self.assertEqual(tuning.rise_text(None, 0), "—")
        self.assertEqual(tuning.rise_text(frame, 0, reverse=True), "350 ms")
        self.assertEqual(tuning.rise_text(frame, 1, reverse=True), "—")
        self.assertEqual(tuning.percent_text(frame["overshoot_pct"], 0), "3 %")
        self.assertEqual(tuning.percent_text(frame["overshoot_pct"], 1), "—")
        self.assertEqual(tuning.percent_text(frame["overshoot_pct_rev"], 0), "4 %")
        self.assertEqual(tuning.percent_text(None, 0), "—")

    def test_result_rows_split_forward_and_reverse(self):
        rows = tuning.result_rows(demo.done_tuning_frame(1))
        self.assertEqual(len(rows), 2 * protocol.WHEEL_COUNT)
        self.assertEqual([row["label"] for row in rows], list(tuning.DIRECTION_LABELS))
        self.assertEqual(rows[0]["label"], "M1 +")
        self.assertEqual(rows[1]["label"], "M1 −")
        self.assertEqual(rows[0]["status"], tuning.RESULT_OK)
        self.assertEqual(rows[0]["color"], config.OK_COLOR)
        self.assertEqual(rows[0]["rise"], "280 ms")
        self.assertEqual(rows[0]["overshoot"], "2 %")
        self.assertEqual(rows[0]["ss_error"], "1 %")
        self.assertEqual(rows[1]["rise"], "310 ms")
        self.assertEqual(rows[1]["overshoot"], "3 %")
        self.assertEqual(rows[1]["ss_error"], "2 %")
        pending = tuning.result_rows(None)
        self.assertEqual(pending[0]["status"], tuning.RESULT_PENDING)
        self.assertEqual((pending[0]["rise"], pending[1]["rise"]), ("—", "—"))

    def test_param_rows_repeat_gains_and_split_feedforward(self):
        rows = tuning.param_rows(demo.done_tuning_frame(1))
        self.assertEqual(len(rows), 2 * protocol.WHEEL_COUNT)
        self.assertEqual([row["label"] for row in rows], list(tuning.DIRECTION_LABELS))
        forward, reverse = rows[0], rows[1]
        self.assertEqual(forward["values"][:3], reverse["values"][:3])
        self.assertEqual(forward["values"][3], "176")
        self.assertEqual(reverse["values"][3], "-198")
        self.assertEqual(forward["values"][4], "54.00")
        self.assertEqual(reverse["values"][4], "57.00")
        self.assertEqual(tuning.param_rows(None)[0]["values"], ["—"] * 5)

    def test_max_speed_texts_show_measured_maxima_and_limit(self):
        self.assertEqual(
            tuning.max_speed_texts(None),
            ("Máx. medido: M1 — · M2 — · M3 — · M4 —", "Limite comum: —"),
        )
        measured, limit = tuning.max_speed_texts(demo.done_tuning_frame(1))
        self.assertIn("M1 175", measured)
        self.assertIn("M4 138", measured)
        self.assertEqual(limit, "Limite comum: 138.0 RPM")
        running = demo.running_tuning_frame(demo.DEMO_TUNING_WHEEL_S * 2.2, 2)
        measured, limit = tuning.max_speed_texts(running)
        self.assertIn("M1 175", measured)
        self.assertIn("M3 —", measured)
        self.assertEqual(limit, "Limite comum: —")

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

class DemoFrameTests(unittest.TestCase):
    def test_tuning_frames_progress_and_finish(self):
        idle = demo.idle_tuning_frame(1)
        self.assertEqual(idle["state"], protocol.TUNING_IDLE)
        self.assertEqual(idle["feedforward"], [0.0] * 4)
        self.assertEqual(idle["feedforward_rev"], [0.0] * 4)
        self.assertEqual(demo.DEMO_TUNING_TOTAL_S, 20.0)
        self.assertEqual(demo.DEMO_MAX_RPM, 138)
        self.assertIn(protocol.TUNING_PHASE_MAX_SPEED, demo.DEMO_TUNING_PHASES)
        start = demo.running_tuning_frame(0.0, 1)
        self.assertEqual(start["state"], protocol.TUNING_RUNNING)
        self.assertEqual(start["wheel"], 0)
        self.assertEqual(start["progress"], 0)
        self.assertEqual(start["max_rpm"], 0.0)
        middle = demo.running_tuning_frame(demo.DEMO_TUNING_WHEEL_S * 2.5, 2)
        self.assertEqual(middle["wheel"], 2)
        self.assertEqual(middle["result_mask"], 0b0011)
        self.assertGreater(middle["progress"], 50)
        self.assertGreater(middle["kp"][2], 0.0)
        self.assertLess(middle["feedforward_rev"][2], 0.0)
        self.assertEqual(middle["wheel_max_rpm"][0], 175.0)
        self.assertEqual(middle["wheel_max_rpm"][3], 0.0)
        self.assertEqual(middle["max_rpm"], 0.0)
        end = demo.running_tuning_frame(demo.DEMO_TUNING_TOTAL_S, 3)
        self.assertEqual(end["state"], protocol.TUNING_DONE)
        self.assertEqual(end["progress"], 100)
        self.assertEqual(end["result_mask"], 0xF)
        self.assertEqual(len(end["kp"]), 4)
        self.assertTrue(all(value > 0 for value in end["feedforward"]))
        self.assertTrue(all(value < 0 for value in end["feedforward_rev"]))
        self.assertTrue(all(value > 0 for value in end["velocity_ff_rev"]))
        self.assertTrue(all(value > 0 for value in end["rise_ms_rev"]))
        self.assertEqual(end["max_rpm"], 138.0)
        self.assertEqual(end["wheel_max_rpm"], list(demo.DEMO_WHEEL_MAX_RPM))

    def test_idle_frame_keeps_stored_parameters(self):
        tuned = demo.idle_tuning_frame(1, tuned=True)
        self.assertEqual(tuned["state"], protocol.TUNING_IDLE)
        self.assertEqual(tuned["result_mask"], 0xF)
        self.assertTrue(all(value > 0 for value in tuned["kp"]))
        self.assertTrue(all(value < 0 for value in tuned["feedforward_rev"]))
        self.assertTrue(all(value > 0 for value in tuned["velocity_ff_rev"]))
        self.assertEqual(tuned["max_rpm"], 138.0)
        self.assertEqual(tuned["wheel_max_rpm"], list(demo.DEMO_WHEEL_MAX_RPM))
        untuned = demo.idle_tuning_frame(2, tuned=False)
        self.assertEqual(untuned["result_mask"], 0)
        self.assertTrue(all(value == 0 for value in untuned["kp"]))
        self.assertTrue(all(value == 0 for value in untuned["feedforward_rev"]))
        self.assertEqual(untuned["max_rpm"], 0.0)
        self.assertEqual(untuned["wheel_max_rpm"], [0.0] * 4)

if __name__ == "__main__":
    unittest.main()
