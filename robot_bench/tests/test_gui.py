"""Regression checks for commands, layout, status text and shared helpers."""

import time
import unittest
from unittest.mock import Mock, patch

from robot_bench import config, console, health, motors, picker, protocol, rendering
from robot_bench.app import compute_layout, connection_status, perform_action, status_line
from robot_bench.state import BenchState


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.state = BenchState()

    def test_repeated_demo_adjustments_accumulate_before_telemetry_arrives(self):
        self.state.demo = True
        perform_action(self.state, "wheel", 2)
        perform_action(self.state, "adjust", 100)
        perform_action(self.state, "adjust", 100)
        self.assertEqual(self.state.manual_targets, [0, 0, 200, 0])

    def test_stop_cancels_sweep_and_cannot_restart_motors(self):
        for demo in (False, True):
            with self.subTest(demo=demo):
                state = BenchState()
                state.demo = demo
                state.reader = Mock()
                state.sweep = True
                state.manual_targets = [500] * 4
                perform_action(state, "stop")
                state.update_sweep(time.monotonic() + 10)
                self.assertFalse(state.sweep)
                if demo:
                    self.assertEqual(state.manual_targets, [0] * 4)
                    state.reader.send_frame.assert_not_called()
                else:
                    state.reader.send_frame.assert_called_once_with(
                        protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_MOTORS, 1)
                    )

    def test_tab_and_wheel_actions_route_to_dashboard_state(self):
        for tab in range(len(config.TAB_TITLES)):
            perform_action(self.state, "tab", tab)
            self.assertEqual(self.state.active_tab, tab)
        perform_action(self.state, "wheel", 3)
        self.assertEqual(self.state.selected_wheel, 3)

    def test_sweep_action_toggles_the_dashboard_sweep(self):
        perform_action(self.state, "sweep")
        self.assertTrue(self.state.sweep)
        perform_action(self.state, "sweep")
        self.assertFalse(self.state.sweep)

    def test_quit_action_stops_the_motors_and_flags_the_session(self):
        self.state.reader = Mock()
        perform_action(self.state, "quit")
        self.assertTrue(self.state.quit)
        self.assertFalse(self.state.sweep)
        self.state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_MOTORS, 1)
        )

    def test_motion_override_arms_the_firmware_before_sending_targets(self):
        self.state.reader = Mock()
        self.state.set_motion_override(True)
        self.assertTrue(self.state.motion_override)
        self.state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_ENABLE_MOTION_OVERRIDE, 1)
        )
        self.state.reader.reset_mock()
        self.state.set_targets([100, 0, 0, 0])
        self.assertEqual(
            [call.args for call in self.state.reader.send_frame.call_args_list],
            [
                (protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_ENABLE_MOTION_OVERRIDE, 2)),
                (
                    protocol.MSG_CONTROL,
                    protocol.encode_control(protocol.OP_SET_TARGETS, 3, [1000, 0, 0, 0]),
                ),
            ],
        )

    def test_motion_override_off_stops_motors_and_resets_with_the_link(self):
        self.state.reader = Mock()
        self.state.set_motion_override(True)
        self.state.set_motion_override(False)
        self.assertFalse(self.state.motion_override)
        self.assertEqual(
            [call.args for call in self.state.reader.send_frame.call_args_list][-2:],
            [
                (protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_DISABLE_MOTION_OVERRIDE, 2)),
                (protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_MOTORS, 3)),
            ],
        )
        self.state.set_motion_override(True)
        self.state.clear_freshness()
        self.assertFalse(self.state.motion_override)

    def test_motion_override_is_local_in_demo_mode(self):
        self.state.demo = True
        self.state.reader = Mock()
        self.state.set_motion_override(True)
        self.assertTrue(self.state.motion_override)
        self.state.reader.send_frame.assert_not_called()


class WindowChromeTests(unittest.TestCase):
    def test_compute_layout_reserves_space_above_and_below_the_tab(self):
        for width, height in (config.MIN_WINDOW, config.DEFAULT_WINDOW, (1920, 1080)):
            with self.subTest(size=(width, height)):
                layout = compute_layout(width, height)
                self.assertLessEqual(layout["body_w"], width)
                self.assertLess(layout["body_h"], height)
                self.assertGreater(layout["body_w"], layout["body_h"] - height + 200)
                self.assertEqual(layout, compute_layout(width, height))
        self.assertLessEqual(compute_layout(400, 300)["body_w"], 400)

    def test_connection_status_reports_demo_live_stale_and_disconnected(self):
        state = BenchState()
        state.demo = True
        self.assertIn("Demo", connection_status(state)[0])
        state.demo = False
        self.assertIn("Desconectado", connection_status(state)[0])
        state.connected = "/dev/ttyUSB0"
        self.assertIn("aguardando", connection_status(state)[0])
        state.streaming = True
        state.last_state_t = time.monotonic()
        self.assertIn("Ao vivo", connection_status(state)[0])
        state.last_state_t = time.monotonic() - config.SENSOR_STALE_S - 1
        self.assertIn("desatualizada", connection_status(state)[0])

    def test_status_line_reports_rates_bytes_and_a_dash_for_stale_uptime(self):
        state = BenchState()
        state.rx_bytes = 1234
        line = status_line(state, 25.0, 15.0, 40.0)
        self.assertIn("Estado 25 Hz", line)
        self.assertIn("Distância 15 Hz", line)
        self.assertIn("Recebido 1,234 B", line)
        self.assertIn("Tempo ativo -", line)
        self.assertIn("Exibição 40 fps", line)
        state.uptime_ms = 42000
        state.last_state_t = time.monotonic()
        self.assertIn("Tempo ativo 42s", status_line(state, 0.0, 0.0, 0.0))


class PickerTests(unittest.TestCase):
    def test_ports_are_ordered_by_probe_readiness_then_name(self):
        results = {
            "/dev/ttyUSB2": {"desc": "b", "status": "empty", "detail": ""},
            "/dev/ttyUSB1": {"desc": "a", "status": "firmware", "detail": ""},
            "/dev/ttyUSB0": {"desc": "c", "status": "probing", "detail": ""},
            "/dev/ttyUSB3": {"desc": "d", "status": "busy", "detail": ""},
            "/dev/ttyUSB4": {"desc": "e", "status": "firmware", "detail": ""},
        }
        self.assertEqual(
            [device for device, _info in picker.sorted_entries(results)],
            ["/dev/ttyUSB1", "/dev/ttyUSB4", "/dev/ttyUSB0", "/dev/ttyUSB2", "/dev/ttyUSB3"],
        )


class PlotHelperTests(unittest.TestCase):
    def test_history_series_filters_old_and_future_samples_and_breaks_gaps(self):
        now = 100.0
        samples = [
            (80.0, 1.0, 0.0),
            (95.0, 20.0, 0.0),
            (95.1, 40.0, 0.0),
            (99.0, 60.0, 0.0),
            (102.0, 80.0, 0.0),
        ]
        xs, ys = rendering.history_series(samples, 1, now, window_s=12.0)
        self.assertEqual(len(xs), len(ys))
        self.assertEqual([value for value in ys if value == value], [20.0, 40.0, 60.0])
        self.assertGreaterEqual(sum(1 for value in xs if value != value), 1)
        self.assertEqual(ys[-1], 60.0)
        self.assertAlmostEqual(xs[-1], -1.0)

    def test_history_series_breaks_angle_wraparound_only_when_requested(self):
        samples = [(99.0, 179.0, 0.0), (99.04, -179.0, 0.0), (99.08, -178.0, 0.0)]
        wrapped_xs, _wrapped_ys = rendering.history_series(samples, 1, 100.0, wrap=True)
        plain_xs, _plain_ys = rendering.history_series(samples, 1, 100.0, wrap=False)
        self.assertEqual(sum(1 for value in wrapped_xs if value != value), 1)
        self.assertEqual(sum(1 for value in plain_xs if value != value), 0)

    def test_history_peak_keeps_a_minimum_scale_and_adds_headroom(self):
        samples = [(99.0, 500.0, 0.0), (99.5, -200.0, 0.0)]
        self.assertEqual(rendering.history_peak(samples, 1, 100.0, 12.0, 600.0), 600.0)
        self.assertAlmostEqual(rendering.history_peak(samples, 1, 100.0, 12.0, 10.0), 575.0)

    def test_distance_color_maps_invalid_and_far_ranges(self):
        self.assertEqual(rendering.distance_color(0), (44, 48, 58))
        self.assertEqual(rendering.distance_color(-5), (44, 48, 58))
        self.assertEqual(rendering.distance_color(config.TOF_MAX_MM), rendering.TOF_COLOR_STOPS[-1][1])
        self.assertEqual(rendering.distance_color(4000.0), rendering.TOF_COLOR_STOPS[-1][1])
        near = rendering.distance_color(100)
        far = rendering.distance_color(3000)
        self.assertNotEqual(near, far)

    def test_yaw_rotation_turns_x_into_y(self):
        matrix = rendering.euler_to_matrix(90.0, 0.0, 0.0)
        x, y, z = rendering.rotate(matrix, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(z, 0.0)


class WheelCardTests(unittest.TestCase):
    def test_encoder_status_tracks_freshness_and_the_valid_mask(self):
        state = BenchState()
        now = 100.0
        self.assertEqual(motors.encoder_status(state, 0, now), ("Aguardando", config.TEXT_DIM))
        state.last_state_t = now
        state.valid_mask = 0b0010
        self.assertEqual(motors.encoder_status(state, 1, now)[0], "Encoder pronto")
        self.assertEqual(motors.encoder_status(state, 0, now)[0], "Não calibrado")
        self.assertEqual(motors.encoder_status(state, 1, now + config.SENSOR_STALE_S + 0.1)[0], "Desatualizada")

    def test_direction_label_has_a_deadband(self):
        self.assertEqual(motors.direction_label(0.0), "Parado")
        self.assertEqual(motors.direction_label(5.0), "Parado")
        self.assertEqual(motors.direction_label(6.0), "Horário")
        self.assertEqual(motors.direction_label(-6.0), "Anti-horário")

    def test_card_hit_test_uses_screen_rectangles(self):
        rects = [((0, 0), (100, 100)), ((110, 0), (200, 100))]
        self.assertEqual(motors.card_index_at((50, 50), rects), 0)
        self.assertEqual(motors.card_index_at((150, 50), rects), 1)
        self.assertIsNone(motors.card_index_at((105, 50), rects))
        self.assertIsNone(motors.card_index_at((50, 120), rects))


class HealthPanelTests(unittest.TestCase):
    def test_live_health_expires_after_the_stale_window(self):
        state = BenchState()
        self.assertEqual(health.live_health(state), {})
        state.health = {"robot_id": 3}
        state.last_health_t = time.monotonic()
        self.assertEqual(health.live_health(state), {"robot_id": 3})
        state.last_health_t = time.monotonic() - config.HEALTH_STALE_S - 0.1
        self.assertEqual(health.live_health(state), {})
        self.assertEqual(health.missing_health_text(state), "Dados de saúde desatualizados")
        state.last_health_t = 0.0
        self.assertEqual(health.missing_health_text(state), "Aguardando dados de saúde")

    def test_lifecycle_names_and_colors(self):
        self.assertEqual(health.lifecycle_text(None), "—")
        self.assertEqual(health.lifecycle_text(3), "Ativo")
        self.assertEqual(health.lifecycle_text(99), "Desconhecido (99)")
        self.assertEqual(health.lifecycle_color(3), config.OK_COLOR)
        self.assertEqual(health.lifecycle_color(5), config.BAD_COLOR)
        self.assertEqual(health.lifecycle_color(None), config.TEXT_DIM)

    def test_error_flags_render_one_row_per_set_bit(self):
        self.assertEqual(health.error_flag_names(0), [])
        self.assertEqual(health.error_flag_names(0b101), ["0: Comunicação perdida", "2: Falha do sensor de distância"])
        self.assertEqual(health.error_flag_names(1 << 11), ["11: Falha do motor 3"])

    def test_inhibit_rows_distinguish_unknown_inhibited_and_allowed(self):
        self.assertEqual(health.inhibit_text(None), "—")
        self.assertEqual(health.inhibit_text(1), "Inibido")
        self.assertEqual(health.inhibit_text(0), "Permitido")
        self.assertEqual(health.inhibit_color(None), config.TEXT_DIM)
        self.assertEqual(health.inhibit_color(1), config.BAD_COLOR)
        self.assertEqual(health.inhibit_color(0), config.OK_COLOR)

    def test_supply_status_prioritises_faults_over_the_normal_flag(self):
        self.assertEqual(health.supply_text(None), "—")
        self.assertEqual(health.supply_text(14.6), "14.60 V")
        self.assertEqual(health.supply_status(BenchState(), 0b011)[0], "Subtensão")
        self.assertEqual(health.supply_status(BenchState(), 0b101)[0], "Sobretensão")
        self.assertEqual(health.supply_status(BenchState(), 0b001)[0], "Alimentação normal")
        self.assertEqual(health.supply_status(BenchState(), 0b000)[0], "Monitor de tensão indisponível")
        self.assertEqual(health.supply_color(0b010), config.BAD_COLOR)
        self.assertEqual(health.supply_color(0b001), config.OK_COLOR)

    def test_calibration_and_connection_helpers(self):
        self.assertEqual(health.imu_cal_text(0b111), "A G M")
        self.assertEqual(health.imu_cal_text(0b101), "A — M")
        self.assertEqual(health.imu_cal_text(None), "—")
        self.assertEqual(health.imu_cal_color(7), config.OK_COLOR)
        self.assertEqual(health.imu_cal_color(3), config.TEXT_DIM)
        self.assertEqual(health.connection_text(1), "Conectado")
        self.assertEqual(health.connection_text(0), "Desconectado")
        self.assertEqual(health.connection_text(None), "—")
        self.assertEqual(health.connection_color(1), config.OK_COLOR)
        self.assertEqual(health.connection_color(0), config.BAD_COLOR)

    def test_uptime_formatting_rolls_over_hours(self):
        self.assertEqual(health.format_uptime(0), "00:00:00")
        self.assertEqual(health.format_uptime(3_723_000), "01:02:03")


class ConsoleTests(unittest.TestCase):
    def test_visible_lines_clamp_the_offset_to_a_full_oldest_window(self):
        state = BenchState()
        state.console.extend(str(index) for index in range(50))
        height = 36 + 21 * 5  # capacity = (height - 36) // 21
        lines, start = console.console_visible_lines(state, 800, height)
        self.assertEqual((len(lines), start, lines[-1]), (5, 45, "49"))
        state.console_offset = 999
        lines, start = console.console_visible_lines(state, 800, height)
        self.assertEqual((len(lines), start, lines[0]), (5, 0, "0"))
        state.console.clear()
        self.assertEqual(console.console_visible_lines(state, 800, height), ([], 0))

    def test_history_stays_anchored_as_messages_arrive(self):
        state = BenchState()
        state.console.extend(str(index) for index in range(500))
        height = 36 + 21 * 22
        state.console_offset = 3
        before = console.console_visible_lines(state, 800, height)[0]
        state.add_console("new message")
        self.assertEqual(console.console_visible_lines(state, 800, height)[0], before)
        state.console.clear()
        state.console.extend(str(index) for index in range(50))
        state.console_offset = 3
        before = console.console_visible_lines(state, 800, height)[0]
        state.add_console("new message")
        self.assertEqual(console.console_visible_lines(state, 800, height)[0], before)

    def test_selection_text_spans_lines_and_orders_the_anchor(self):
        state = BenchState()
        state.console.extend(("alpha", "beta", "gamma"))
        state.console_sel = {"anchor": (0, 2), "focus": (2, 3)}
        self.assertEqual(console.console_selection_text(state), "pha\nbeta\ngam")
        state.console_sel = {"anchor": (2, 3), "focus": (0, 2)}
        self.assertEqual(console.console_selection_text(state), "pha\nbeta\ngam")
        state.console_sel = {"anchor": (1, 0), "focus": (1, 4)}
        self.assertEqual(console.console_selection_text(state), "beta")
        state.console_sel = None
        self.assertEqual(console.console_selection_text(state), "")

    def test_hit_test_maps_rows_and_characters_with_the_fallback_advance(self):
        state = BenchState()
        state.console.extend(f"line {index}" for index in range(10))
        height = 36 + 21 * 3  # three visible rows
        lines, start = console.console_visible_lines(state, 800, height)
        last_top = height - 28
        row, char = console.console_hit_test(state, 800, height, (12, last_top))
        self.assertEqual((row, char), (start + len(lines) - 1, 0))
        target = "line 9"
        x = 12 + 7.25 * 3
        row, char = console.console_hit_test(state, 800, height, (x, last_top))
        self.assertEqual(char, 3)
        row, char = console.console_hit_test(state, 800, height, (10_000, last_top))
        self.assertEqual(char, len(target))

    def test_press_and_drag_track_the_pointer_as_a_selection(self):
        state = BenchState()
        state.console.append("hello world")
        height = 36 + 21 * 3
        with (
            patch.object(console.dpg, "get_item_rect_size", return_value=(800, height)),
            patch.object(console.dpg, "get_item_rect_min", return_value=(0, 0)),
        ):
            console.press(state, (12, height - 28))
            console.drag(state, (12 + 7.25 * 5, height - 28))
        self.assertEqual(console.console_selection_text(state), "hello")

    def test_wheel_scrolls_towards_history_and_ignores_the_pointer_outside(self):
        state = BenchState()
        state.console.extend(str(index) for index in range(100))
        with (
            patch.object(console.dpg, "get_item_rect_size", return_value=(800, 400)),
            patch.object(console.dpg, "get_item_rect_min", return_value=(10, 10)),
        ):
            console.wheel(state, 1.0, (500, 300))
            self.assertEqual(state.console_offset, 3)
            console.wheel(state, -1.0, (500, 300))
            self.assertEqual(state.console_offset, 0)
            console.wheel(state, 1.0, (5, 300))
            self.assertEqual(state.console_offset, 0)

    def test_release_copies_the_selection_and_logs_the_copy_once(self):
        state = BenchState()
        state.console.extend(("first", "second"))
        state.console_sel = {"anchor": (0, 0), "focus": (0, 5)}
        with patch.object(console.dpg, "set_clipboard_text") as clipboard:
            console.release(state, (0, 0))
            console.release(state, (0, 0))
        clipboard.assert_called_with("first")
        copies = [line for line in state.console if "caracteres copiados" in line]
        self.assertEqual(len(copies), 1)

    def test_repeated_clipboard_failures_report_the_reason_once(self):
        state = BenchState()
        state.console.extend(("first", "second"))
        state.console_sel = {"anchor": (0, 0), "focus": (0, 5)}
        with patch.object(console.dpg, "set_clipboard_text", side_effect=RuntimeError("display unavailable")):
            console.release(state, (0, 0))
            console.release(state, (0, 0))
        failures = [line for line in state.console if "área de transferência indisponível" in line]
        self.assertEqual(failures, ["> área de transferência indisponível: display unavailable"])


if __name__ == "__main__":
    unittest.main()
