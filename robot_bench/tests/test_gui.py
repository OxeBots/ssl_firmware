"""Regression checks for commands, layout, status text and shared helpers."""

import math
import time
import unittest
from unittest.mock import Mock, patch

import dearpygui.dearpygui as dpg

from robot_bench import app, config, console, demo, health, motors, picker, protocol, radio_panel, raw, rendering
from robot_bench.app import compute_layout, connection_status, perform_action, status_line
from robot_bench.state import BenchState


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


def live_state():
    """Bench state on a live serial link, ready to command motion."""
    state = BenchState()
    state.reader = Mock()
    state.connected = "/dev/ttyUSB0"
    state.streaming = True
    state.last_state_t = time.monotonic()
    state.valid_mask = 0xF
    return state


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.state = BenchState()

    def test_repeated_demo_adjustments_accumulate_after_a_calibrated_limit(self):
        self.state.demo = True
        self.state.update_tuning(demo.done_tuning_frame(1))
        perform_action(self.state, "wheel", 2)
        perform_action(self.state, "adjust", 0.25)
        perform_action(self.state, "adjust", 0.25)
        self.assertEqual(self.state.manual_targets, [0.0, 0.0, 0.5 * demo.DEMO_MAX_RPM, 0.0])

    def test_arrow_rpm_step_is_configurable(self):
        self.state.demo = True
        self.state.update_tuning(demo.done_tuning_frame(1))
        perform_action(self.state, "wheel", 2)
        self.state.arrow_step_rpm = 20.0
        perform_action(self.state, "adjust_rpm", 1.0)
        perform_action(self.state, "adjust_rpm", 1.0)
        self.assertEqual(self.state.manual_targets, [0.0, 0.0, 40.0, 0.0])

    def test_editing_arrow_step_blocks_drive_keys_and_keeps_stop(self):
        keys = app.KeyRepeater()
        down = {dpg.mvKey_1, dpg.mvKey_Right, dpg.mvKey_Up, dpg.mvKey_Spacebar}
        with (
            patch.object(dpg, "is_key_pressed", side_effect=lambda key: key in down),
            patch.object(dpg, "is_key_down", side_effect=lambda key: key in down),
        ):
            self.assertEqual(keys.poll(1.0, editing_step=True), [("stop", None)])
            down.remove(dpg.mvKey_Spacebar)
            self.assertEqual(keys.poll(2.0), [])
            down.clear()
            keys.poll(2.1)
            down.add(dpg.mvKey_Up)
            self.assertEqual(keys.poll(2.2), [("adjust_rpm", 1.0)])

    def test_stop_cancels_sweep_and_cannot_restart_motors(self):
        for demo_mode in (False, True):
            with self.subTest(demo=demo_mode):
                state = BenchState()
                state.demo = demo_mode
                state.reader = Mock()
                state.sweep = True
                state.manual_targets = [500] * 4
                perform_action(state, "stop")
                state.update_sweep(time.monotonic() + 10)
                self.assertFalse(state.sweep)
                if demo_mode:
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

    def test_wheel_targets_are_sent_directly_with_health_warnings(self):
        self.state.reader = Mock()
        self.state.health = {"error_flags": 0b1001000}
        self.assertTrue(self.state.set_targets([100, 0, 0, 0]))
        self.state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL,
            protocol.encode_control(protocol.OP_SET_TARGETS, 1, [1000, 0, 0, 0]),
        )


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

    def test_resolve_viewport_adapts_to_hidpi_fhd_and_small_screens(self):
        hidpi = {
            "screen_w": 2880,
            "screen_h": 1800,
            "work_x": 0,
            "work_y": 0,
            "work_w": 2880,
            "work_h": 1800,
            "content_scale": 1.0,
        }
        vp = app.resolve_viewport(monitor=hidpi)
        self.assertAlmostEqual(vp["scale"], 1.75)
        self.assertGreater(vp["width"], 2200)
        self.assertGreater(vp["height"], 1400)
        self.assertLessEqual(vp["width"], hidpi["work_w"])
        self.assertLessEqual(vp["height"], hidpi["work_h"])
        self.assertEqual(vp["x_pos"], (hidpi["work_w"] - vp["width"]) // 2)
        self.assertLessEqual(vp["min_width"], hidpi["work_w"] // 2)

        fhd = {
            "screen_w": 1920,
            "screen_h": 1080,
            "work_x": 0,
            "work_y": 0,
            "work_w": 1920,
            "work_h": 1080,
            "content_scale": 1.0,
        }
        vp_fhd = app.resolve_viewport(monitor=fhd)
        self.assertEqual(vp_fhd["scale"], 1.0)
        self.assertLessEqual(vp_fhd["width"], fhd["work_w"])
        self.assertLessEqual(vp_fhd["height"], fhd["work_h"])

        small = {
            "screen_w": 1366,
            "screen_h": 768,
            "work_x": 0,
            "work_y": 0,
            "work_w": 1366,
            "work_h": 740,
            "content_scale": 1.0,
        }
        vp_small = app.resolve_viewport(monitor=small)
        self.assertEqual(vp_small["scale"], 1.0)
        self.assertLessEqual(vp_small["height"], small["work_h"])
        self.assertLessEqual(vp_small["min_height"], vp_small["height"])

        override = app.resolve_viewport(width=1600, height=900, scale=1.5, monitor=hidpi)
        self.assertEqual((override["width"], override["height"], override["scale"]), (1600, 900, 1.5))

        env_override = app.resolve_viewport(monitor=fhd, env_scale="1.25")
        self.assertAlmostEqual(env_override["scale"], 1.25)

        headless = app.resolve_viewport(monitor=None)
        self.assertEqual(headless["scale"], 1.0)
        self.assertEqual((headless["width"], headless["height"]), config.DEFAULT_WINDOW)

    def test_scaled_layout_keeps_proportional_chrome_and_restores_default(self):
        try:
            rendering.set_ui_scale(1.75)
            self.assertEqual(rendering.font_size("label"), int(round(rendering.FONT_SIZES["label"] * 1.75)))
            layout = compute_layout(2476, 1548)
            self.assertGreater(layout["body_w"], 2300)
            self.assertGreater(layout["body_h"], 1200)
        finally:
            rendering.set_ui_scale(1.0)

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

    def test_history_peak_ignores_missing_pulse_samples(self):
        samples = [(99.0, float("nan"), 0.0), (99.5, 200.0, 0.0)]
        self.assertEqual(rendering.history_peak(samples, 1, 100.0, 12.0, 600.0), 600.0)
        self.assertAlmostEqual(rendering.history_peak(samples, 1, 100.0, 12.0, 10.0), 230.0)

    def test_series_bounds_fits_the_data_and_keeps_a_minimum_span(self):
        low, high = rendering.series_bounds(([100.0, 2000.0],), 512.0)
        self.assertAlmostEqual(low, 100.0 - 0.08 * 1900.0)
        self.assertAlmostEqual(high, 2000.0 + 0.08 * 1900.0)
        low, high = rendering.series_bounds(([2000.0, 2002.0, float("nan")],), 512.0)
        self.assertAlmostEqual((low + high) / 2.0, 2001.0)
        self.assertAlmostEqual(high - low, 512.0 * 1.16)
        self.assertEqual(rendering.series_bounds(([],), 512.0), (0.0, 512.0))

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

    def test_pulse_texts_need_a_live_link_and_the_wheel_valid_bit(self):
        state = BenchState()
        state.wheels[0]["pulse_rpm"] = 123.4
        state.wheels[0]["pulse_count"] = 987654
        now = 100.0
        self.assertEqual(motors.pulse_texts(state, 0, now), ("—", "—"))
        state.last_state_t = now
        self.assertEqual(motors.pulse_texts(state, 0, now), ("—", "—"))
        state.pulse_valid_mask = 0b0001
        self.assertEqual(motors.pulse_texts(state, 0, now), ("+123.4 RPM", "987,654"))
        self.assertEqual(motors.pulse_texts(state, 1, now), ("—", "—"))
        state.last_state_t = now - config.SENSOR_STALE_S - 1
        self.assertEqual(motors.pulse_texts(state, 0, now), ("—", "—"))

    def test_sidebar_height_leaves_room_for_the_tab_bar(self):
        self.assertEqual(motors.sidebar_height(492), 492 - motors.SIDEBAR_TAB_HEIGHT)
        self.assertEqual(motors.sidebar_height(492, 40), 452)
        self.assertEqual(motors.sidebar_height(100), motors.MIN_SIDEBAR_HEIGHT)

    def test_card_plot_height_takes_what_the_text_rows_leave(self):
        # Rows measured on screen: 84 px above the chart, 12 px below it.
        top_h, footer_h = 84.0, 12.0
        plot_h = motors.card_plot_height(300, top_h, footer_h)
        self.assertGreater(plot_h, top_h)
        self.assertEqual(motors.card_plot_height(340, top_h, footer_h) - plot_h, 40)
        self.assertEqual(motors.card_plot_height(60, top_h, footer_h), motors.MIN_PLOT_HEIGHT)

    def test_card_chart_stays_the_largest_row_at_the_supported_windows(self):
        for size in (config.MIN_WINDOW, config.DEFAULT_WINDOW, (1920, 1080)):
            layout = app.compute_layout(*size)
            card_h = (motors.view_height(layout["body_h"]) - motors.GAP) // 2
            plot_h = motors.card_plot_height(card_h, 84.0, 12.0)
            self.assertGreater(plot_h, 84.0, msg=f"size={size}")

    def test_adjust_labels_and_limit_texts_scale_with_the_calibration(self):
        self.assertEqual(motors.adjust_label(1.0, 124.0), "+100% · 124")
        self.assertEqual(motors.adjust_label(-0.25, 124.0), "-25% · 31")
        self.assertEqual(motors.adjust_label(0.25, 1333.0), "+25% · 333")
        state = BenchState()
        fallback_text, fallback_color = motors.max_rpm_text(state)
        self.assertIn("não calibrado", fallback_text)
        self.assertIn("1333", fallback_text)
        self.assertEqual(fallback_color, config.DEMO_COLOR)
        state.update_tuning(demo.done_tuning_frame(1))
        text, color = motors.max_rpm_text(state)
        self.assertIn("Máximo calibrado", text)
        self.assertIn("138", text)
        self.assertEqual(color, config.OK_COLOR)
        self.assertIn("138", motors.sweep_hint_text(state))


class RawViewTests(unittest.TestCase):
    def test_state_keeps_raw_history_and_clears_it_on_disconnect(self):
        state = BenchState()
        self.assertFalse(state.raw_live())
        state.update_raw(raw_frame())
        self.assertEqual(state.raw_count, 1)
        self.assertTrue(state.raw_live())
        with state.history_lock:
            first = list(state.raw_history[0])[-1]
            third = list(state.raw_history[2])[-1]
        self.assertEqual(first[1], 100)
        self.assertEqual(first[2], 10.0)
        self.assertEqual(first[3], 0)
        self.assertEqual(first[4], -12.5)
        self.assertEqual(first[5], -1023)
        self.assertTrue(math.isnan(third[2]))
        state.last_raw_t = time.monotonic() - config.RAW_STALE_S - 0.1
        self.assertFalse(state.raw_live())
        state.last_raw_t = time.monotonic()
        state.clear_freshness()
        self.assertIsNone(state.raw)
        self.assertEqual(state.last_raw_t, 0.0)
        with state.history_lock:
            self.assertEqual(list(state.raw_history[0]), [])

    def test_card_status_tracks_i2c_validity_and_freshness(self):
        now = 100.0
        state = BenchState()
        self.assertEqual(raw.card_status(state, 0, now), (raw.STATUS_WAITING, config.TEXT_DIM))
        state.update_raw(raw_frame())
        state.last_raw_t = now
        self.assertEqual(raw.card_status(state, 0, now), (raw.STATUS_RAW, config.OK_COLOR))
        self.assertEqual(raw.card_status(state, 2, now), (raw.STATUS_I2C, config.DEMO_COLOR))
        self.assertEqual(
            raw.card_status(state, 0, now + config.RAW_STALE_S + 0.1),
            (raw.STATUS_STALE, config.BAD_COLOR),
        )
        self.assertIsNone(raw.i2c_degrees(0xFFFF))
        self.assertEqual(raw.i2c_degrees(2048), 180.0)

    def test_raw_values_map_wheel_data_and_units(self):
        state = BenchState()
        state.update_state(
            {
                "uptime_ms": 1,
                "wheels": [
                    {
                        "pwm": 0,
                        "vel_rpm": 2.0,
                        "tgt_rpm": 0.0,
                        "angle_deg": 0.0,
                        "pulse_rpm": -5.0 * index,
                        "pulse_count": 100 * index,
                    }
                    for index in range(4)
                ],
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "valid_mask": 0xF,
                "pulse_valid_mask": 0b0011,
                "sequence": 1,
            }
        )
        state.update_raw(raw_frame())
        now = time.monotonic()
        values = raw.raw_values(state, 2, now)
        self.assertEqual(values[0][0][1], "300")
        self.assertEqual(values[0][1][1], "—")
        self.assertEqual(values[0][2][1], "—")
        self.assertEqual(values[0][3][1], "0.0°")
        self.assertEqual(values[0][4][1], "200")
        self.assertEqual(values[0][5][1], "2")
        self.assertEqual(values[1][0][1], "+7.3")
        self.assertEqual(values[1][1][1], "—")
        self.assertEqual(values[1][2][1], "+2.0")
        self.assertEqual(values[1][3][1], "+512")
        self.assertEqual(values[1][4][1], "Horário")
        self.assertEqual(values[1][5][1], raw.STATUS_I2C)
        empty = raw.raw_values(BenchState(), 0, 0.0)
        self.assertEqual(empty[0][0][1], "—")
        self.assertEqual(empty[0][3][1], "—")
        self.assertEqual(empty[1][5][1], raw.STATUS_WAITING)
        self.assertEqual(raw.wheel_label(0), "M1 · Dianteira esq.")
        self.assertEqual(raw.wheel_label(3), "M4 · Dianteira dir.")
        self.assertEqual(
            (raw.pwm_direction(0), raw.pwm_direction(10), raw.pwm_direction(-10)),
            ("Parado", "Horário", "Anti-horário"),
        )

    def test_demo_raw_i2c_honours_the_mux_focus(self):
        self.assertEqual(demo.raw_i2c_values([90.0, 180.0, 270.0, 0.0], None), [0xFFFF] * 4)
        values = demo.raw_i2c_values([90.0, 180.0, 270.0, 0.0], 2)
        self.assertEqual(values[:2], [0xFFFF, 0xFFFF])
        self.assertEqual(values[2], 3072)
        self.assertEqual(values[3], 0xFFFF)

    def test_adc_angle_conversion_uses_the_calibration_endpoints(self):
        calibration = demo.done_calibration_frame(1)
        minimum, maximum = demo.DEMO_ADC_RANGE[0]
        endpoints = raw.adc_endpoints(calibration, 0)
        self.assertEqual(endpoints, (minimum, maximum))
        self.assertEqual(raw.adc_angle_counts(minimum, endpoints), 0.0)
        self.assertEqual(raw.adc_angle_counts(maximum, endpoints), raw.ADC_MAX)
        self.assertAlmostEqual(
            raw.adc_angle_counts((minimum + maximum) / 2.0, endpoints), raw.ADC_MAX / 2.0
        )
        self.assertEqual(raw.adc_angle_counts(0, endpoints), 0.0)
        self.assertEqual(raw.adc_angle_counts(4095, endpoints), raw.ADC_MAX)
        self.assertIsNone(raw.adc_angle_counts(100, None))
        self.assertEqual(raw.adc_plot_label(calibration, 0), "ADC (ângulo)")
        no_range = demo.calibration_frame(2)
        self.assertIsNone(raw.adc_endpoints(no_range, 0))
        self.assertEqual(raw.adc_plot_label(no_range, 0), "ADC bruto (sem faixa)")
        rejected = demo.calibration_frame(
            3, range_mask=0xF, adc_min_raw=[0, 0, 0, 0], adc_max_raw=[0, 0, 0, 0]
        )
        self.assertIsNone(raw.adc_endpoints(rejected, 0))

    def test_selector_labels_widths_and_plot_colours(self):
        self.assertEqual(raw.wheel_label(1), "M2 · Traseira esq.")
        self.assertEqual(raw.selector_label(1, False), "M2")
        self.assertEqual(raw.selector_label(1, True), "M2 · Traseira esq.")
        widths = raw.selector_widths(900, 2)
        self.assertEqual(len(widths), protocol.WHEEL_COUNT)
        self.assertEqual(sum(widths) + 3 * raw.GAP, 900)
        self.assertEqual(widths[2], 900 - 3 * raw.COMPACT_BUTTON_WIDTH - 3 * raw.GAP)
        self.assertEqual((widths[0], widths[1], widths[3]), (raw.COMPACT_BUTTON_WIDTH,) * 3)
        self.assertGreater(widths[2], raw.COMPACT_BUTTON_WIDTH)
        self.assertEqual(raw.PLOT_ADC_COLOR, config.ACCENT)
        self.assertEqual(raw.PLOT_I2C_COLOR, config.PWM_COLOR)
        self.assertNotEqual(raw.PLOT_ADC_COLOR, raw.PLOT_I2C_COLOR)

    def test_plot_height_uses_the_free_tab_space(self):
        self.assertEqual(raw.plot_height(458), (458 - raw.PLOT_RESERVED_HEIGHT) // 3)
        self.assertEqual(raw.plot_height(100), raw.MIN_PLOT_HEIGHT)

    def test_view_height_leaves_room_for_the_internal_tab_bar(self):
        self.assertEqual(motors.view_height(492), 492 - motors.MOTORS_VIEW_TAB_HEIGHT)
        self.assertEqual(motors.view_height(492, 40), 452)
        self.assertEqual(motors.view_height(100), motors.MIN_VIEW_HEIGHT)


class RawEncoderFocusTests(unittest.TestCase):
    def test_focus_chosen_before_link_is_sent_once_after_link_becomes_live(self):
        state = BenchState()
        state.reader = Mock()
        state.selected_wheel = 1
        raw.on_show(state)
        self.assertEqual(state.raw_encoder_wheel, 1)
        state.reader.send_frame.assert_not_called()

        state.connected = "/dev/ttyUSB0"
        state.streaming = True
        state.last_link_t = time.monotonic()
        raw.sync_selection(state)
        raw.sync_selection(state)
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL,
            protocol.encode_control(protocol.OP_SELECT_RAW_ENCODER, 1, [1, 0, 0, 0]),
        )

    def test_selection_sends_the_control_op_and_clears_on_disconnect(self):
        state = BenchState()
        state.reader = Mock()
        state.connected = "/dev/ttyUSB0"
        state.streaming = True
        state.last_state_t = time.monotonic()
        state.select_raw_encoder(2)
        self.assertEqual(state.raw_encoder_wheel, 2)
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL,
            protocol.encode_control(protocol.OP_SELECT_RAW_ENCODER, 1, [2, 0, 0, 0]),
        )
        state.select_raw_encoder(2)
        self.assertEqual(state.reader.send_frame.call_count, 1)
        state.select_raw_encoder(None)
        state.reader.send_frame.assert_called_with(
            protocol.MSG_CONTROL,
            protocol.encode_control(protocol.OP_SELECT_RAW_ENCODER, 2, [-1, 0, 0, 0]),
        )
        state.select_raw_encoder(1)
        state.clear_freshness()
        self.assertIsNone(state.raw_encoder_wheel)

    def test_invalid_choices_normalise_and_no_link_stores_without_sending(self):
        state = BenchState()
        state.select_raw_encoder(7)
        self.assertIsNone(state.raw_encoder_wheel)
        state.select_raw_encoder(1)
        self.assertEqual(state.raw_encoder_wheel, 1)
        state.reader = Mock()
        state.demo = True
        state.select_raw_encoder(3)
        self.assertEqual(state.raw_encoder_wheel, 3)
        state.reader.send_frame.assert_not_called()

    def test_show_hide_and_selection_sync_send_the_focus(self):
        state = BenchState()
        state.demo = True
        state.selected_wheel = 2
        raw.sync_selection(state)
        self.assertEqual(state.raw_encoder_wheel, 2)
        state.selected_wheel = 3
        raw.sync_selection(state)
        self.assertEqual(state.raw_encoder_wheel, 3)
        raw.on_show(state)
        self.assertEqual(state.raw_encoder_wheel, 3)
        raw.on_hide(state)
        self.assertIsNone(state.raw_encoder_wheel)


class MaxRpmLimitTests(unittest.TestCase):
    def test_state_tracks_the_calibrated_limit_and_falls_back(self):
        state = BenchState()
        self.assertEqual(state.max_rpm, protocol.WHEEL_RPM_MAX)
        self.assertFalse(state.max_rpm_calibrated)
        state.update_tuning(demo.done_tuning_frame(1))
        self.assertEqual(state.max_rpm, demo.DEMO_MAX_RPM)
        self.assertTrue(state.max_rpm_calibrated)
        self.assertEqual(state.wheel_max_rpm, list(demo.DEMO_WHEEL_MAX_RPM))
        # A fresh frame without a stored limit restores the fallback.
        state.update_tuning(demo.idle_tuning_frame(1))
        self.assertEqual(state.max_rpm, protocol.WHEEL_RPM_MAX)
        self.assertFalse(state.max_rpm_calibrated)
        # A live uncalibrated robot reports the drivetrain fallback as its limit;
        # that sentinel must still read as "not calibrated".
        uncalibrated = demo.done_tuning_frame(3)
        uncalibrated["max_rpm"] = protocol.WHEEL_RPM_MAX
        state.update_tuning(uncalibrated)
        self.assertEqual(state.max_rpm, protocol.WHEEL_RPM_MAX)
        self.assertFalse(state.max_rpm_calibrated)
        state.update_tuning(demo.done_tuning_frame(4))
        state.clear_freshness()
        self.assertEqual(state.max_rpm, protocol.WHEEL_RPM_MAX)
        self.assertEqual(state.wheel_max_rpm, [0.0] * 4)

    def test_targets_and_sweep_stay_inside_the_calibrated_limit(self):
        state = BenchState()
        state.demo = True
        state.update_tuning(demo.done_tuning_frame(1))
        self.assertTrue(state.set_targets([500.0, -500.0, 100.0, 0.0]))
        limit = demo.DEMO_MAX_RPM
        self.assertEqual(state.manual_targets, [limit, -limit, 100.0, 0.0])
        self.assertTrue(any("limitados ao máximo calibrado" in line for line in state.console))
        state.selected_wheel = 0
        state.sweep = True
        state.sweep_high = False
        state.last_sweep_t = 0.0
        state.update_sweep(10.0)
        self.assertEqual(state.manual_targets[0], limit)
        state.update_sweep(20.0)
        self.assertEqual(state.manual_targets[0], -limit)

    def test_adjusted_target_moves_by_a_fraction_of_the_limit(self):
        state = BenchState()
        state.demo = True
        state.update_tuning(demo.done_tuning_frame(1))
        state.selected_wheel = 1
        state.adjust_selected_target(1.0)
        self.assertEqual(state.manual_targets[1], demo.DEMO_MAX_RPM)
        state.adjust_selected_target(-0.25)
        self.assertEqual(state.manual_targets[1], 0.75 * demo.DEMO_MAX_RPM)


class DigitalFeedbackTests(unittest.TestCase):
    def state_frame(self, pulse_mask):
        return {
            "uptime_ms": 1000,
            "wheels": [
                {
                    "pwm": index,
                    "vel_rpm": 10.0 * index,
                    "tgt_rpm": 0.0,
                    "angle_deg": 0.0,
                    "pulse_rpm": -5.0 * index,
                    "pulse_count": 100 * index,
                }
                for index in range(4)
            ],
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "valid_mask": 0b0101,
            "pulse_valid_mask": pulse_mask,
            "sequence": 1,
        }

    def test_state_keeps_pulse_feedback_and_plots_gaps_when_invalid(self):
        state = BenchState()
        state.update_state(self.state_frame(0b0011))
        self.assertEqual(state.pulse_valid_mask, 0b0011)
        self.assertEqual(state.wheels[2]["pulse_rpm"], -10.0)
        self.assertEqual(state.wheels[2]["pulse_count"], 200)
        with state.history_lock:
            first = list(state.history[0])[-1]
            third = list(state.history[2])[-1]
        self.assertEqual(first[3], 0.0)
        self.assertTrue(math.isnan(third[3]))

    def test_clear_freshness_drops_the_pulse_feedback(self):
        state = BenchState()
        state.update_state(self.state_frame(0xF))
        state.clear_freshness()
        self.assertEqual(state.pulse_valid_mask, 0)
        self.assertEqual([wheel["pulse_rpm"] for wheel in state.wheels], [0.0] * 4)
        self.assertEqual([wheel["pulse_count"] for wheel in state.wheels], [0] * 4)


class PositionPanelTests(unittest.TestCase):
    def test_status_text_covers_position_states(self):
        now = 100.0
        state = BenchState()
        self.assertEqual(motors.position_status_text(state, now), (motors.POSITION_IDLE, config.TEXT_DIM))
        state.last_position_t = now
        state.position = demo.position_frame(
            1, active_mask=0b0011, done_mask=0b0001, angle_deg=[0, 90, 0, 0], target_deg=[0, 90, 0, 0]
        )
        self.assertEqual(motors.position_status_text(state, now), (motors.POSITION_ACTIVE, config.ACCENT))
        state.position = demo.position_frame(
            2, active_mask=0b0011, done_mask=0b0011, angle_deg=[0, 90, 0, 0], target_deg=[0, 90, 0, 0]
        )
        self.assertEqual(motors.position_status_text(state, now), (motors.POSITION_DONE, config.OK_COLOR))
        state.position = demo.position_frame(3, error_code=protocol.POSITION_ERROR_TIMEOUT)
        self.assertEqual(
            motors.position_status_text(state, now),
            (f"Falha · {motors.POSITION_ERROR_NAMES[1]}", config.BAD_COLOR),
        )
        state.last_position_t = now - config.POSITION_STALE_S - 1.0
        self.assertEqual(motors.position_status_text(state, now), (motors.POSITION_STALE, config.BAD_COLOR))

    def test_position_rows_show_angles_errors_and_status(self):
        pose = demo.position_frame(
            1,
            active_mask=0b0101,
            done_mask=0b0001,
            angle_deg=[10.0, 0.0, -45.0, 0.0],
            target_deg=[10.0, 0.0, -40.0, 0.0],
        )
        first = motors.position_row(pose, 0)
        self.assertEqual(first["angle"], "10.0°")
        self.assertEqual(first["status"], "Alvo atingido")
        self.assertEqual(first["color"], config.OK_COLOR)
        third = motors.position_row(pose, 2)
        self.assertEqual(third["error"], "+5.0°")
        self.assertEqual(third["status"], "Movendo")
        self.assertEqual(third["color"], config.ACCENT)
        second = motors.position_row(pose, 1)
        self.assertEqual(second["status"], "Parado")
        self.assertEqual(second["color"], config.TEXT_DIM)
        idle = motors.position_row(None, 0)
        self.assertEqual(idle["angle"], "—")

    def test_move_buttons_need_link_routines_and_calibrated_encoder(self):
        now = 100.0
        state = BenchState()
        self.assertFalse(motors.move_enabled(state, now, 0))
        state.demo = True
        self.assertTrue(motors.move_enabled(state, now, 0))
        state.demo = False
        state = live_state()
        state.last_state_t = now
        state.valid_mask = 0b0011
        self.assertTrue(motors.move_enabled(state, now, 1))
        self.assertFalse(motors.move_enabled(state, now, 2))
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        state.last_calibration_t = time.monotonic()
        self.assertFalse(motors.move_enabled(state, now, 1))

    def test_stop_position_enabled_only_with_active_move(self):
        now = 100.0
        state = BenchState()
        self.assertFalse(motors.position_stop_enabled(state, now))
        state.last_position_t = now
        state.position = demo.position_frame(1, active_mask=0b0001)
        self.assertTrue(motors.position_stop_enabled(state, now))
        state.position = demo.position_frame(2)
        self.assertFalse(motors.position_stop_enabled(state, now))
        state.demo = True
        state.demo_position = {"wheel_mask": 1}
        self.assertTrue(motors.position_stop_enabled(state, now))

    def test_demo_position_frame_defaults_are_idle(self):
        frame = demo.position_frame(1)
        self.assertEqual(frame["active_mask"], 0)
        self.assertEqual(frame["done_mask"], 0)
        self.assertEqual(frame["error_code"], protocol.POSITION_ERROR_NONE)
        self.assertEqual(frame["angle_deg"], [0.0] * 4)
        self.assertEqual(frame["target_deg"], [0.0] * 4)


class PositionCommandTests(unittest.TestCase):
    def test_move_sends_relative_or_absolute_frame(self):
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

    def test_move_requires_calibrated_encoders(self):
        state = live_state()
        state.valid_mask = 0b0011
        self.assertFalse(state.start_position_move([10.0] * 4, 1 << 3, False, 0))
        state.reader.send_frame.assert_not_called()
        self.assertTrue(state.start_position_move([10.0] * 4, 1 << 1, False, 0))

    def test_move_cancelled_by_velocity_targets_and_stop(self):
        state = live_state()
        state.demo = True
        self.assertTrue(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))
        self.assertIsNotNone(state.demo_position)
        state.set_targets([100.0, 0.0, 0.0, 0.0])
        self.assertIsNone(state.demo_position)
        self.assertTrue(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, True))
        state.stop_motors()
        self.assertIsNone(state.demo_position)

    def test_move_refused_during_calibration(self):
        state = live_state()
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        state.last_calibration_t = time.monotonic()
        self.assertFalse(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))
        state.calibration = None
        state.last_calibration_t = 0.0
        state.update_tuning(demo.tuning_frame(2, state=protocol.TUNING_RUNNING))
        self.assertFalse(state.start_position_move([90.0, 0.0, 0.0, 0.0], 0b0001, False))


class TabLayoutTests(unittest.TestCase):
    def test_usb_tabs_plus_the_radio_tab(self):
        self.assertEqual(len(config.TAB_TITLES), 7)
        self.assertEqual(config.TAB_TITLES[-1], "Rádio")
        self.assertEqual(
            app.TAB_TAGS,
            (
                "tab_motors",
                "tab_calibration",
                "tab_health",
                "tab_tof",
                "tab_imu",
                "tab_console",
                "tab_radio",
            ),
        )
        self.assertEqual(len(app.TAB_TAGS), len(config.TAB_TITLES))
        for index in range(len(config.TAB_TITLES)):
            self.assertEqual(app.KEY_ACTIONS[getattr(dpg, f"mvKey_F{index + 1}")], ("tab", index))
        self.assertEqual(app.KEY_ACTIONS[dpg.mvKey_Up], ("adjust_rpm", 1.0))
        self.assertEqual(app.KEY_ACTIONS[dpg.mvKey_Down], ("adjust_rpm", -1.0))
        self.assertEqual(app.KEY_ACTIONS[dpg.mvKey_Right], ("adjust", 1.0))
        self.assertEqual(app.KEY_ACTIONS[dpg.mvKey_Left], ("adjust", -1.0))


class DualConnectionTests(unittest.TestCase):
    def test_tab_visibility_needs_the_matching_link(self):
        self.assertFalse(app.tab_visible(config.TAB_MOTORS, radio_active=False, usb_active=False))
        self.assertTrue(app.tab_visible(config.TAB_MOTORS, radio_active=False, usb_active=True))
        self.assertFalse(app.tab_visible(config.TAB_RADIO, radio_active=False, usb_active=True))
        self.assertTrue(app.tab_visible(config.TAB_RADIO, radio_active=True, usb_active=False))
        self.assertTrue(app.tab_visible(config.TAB_RADIO, radio_active=True, usb_active=True))

    def test_radio_connection_status_covers_the_link_states(self):
        self.assertEqual(
            app.radio_connection_status({"live": True, "connected": True, "rx_count": 5}),
            ("Rádio ao vivo", config.OK_COLOR),
        )
        self.assertEqual(
            app.radio_connection_status({"live": False, "connected": True, "rx_count": 5}),
            ("Rádio sem resposta", config.BAD_COLOR),
        )
        self.assertEqual(
            app.radio_connection_status({"live": False, "connected": True, "rx_count": 0}),
            ("Rádio aguardando", config.TEXT_DIM),
        )
        self.assertEqual(
            app.radio_connection_status({"live": False, "connected": False, "rx_count": 0}),
            ("Rádio desconectado", config.BAD_COLOR),
        )

    def test_header_combines_both_links(self):
        state = BenchState()
        state.connected = "/dev/ttyUSB1"
        state.streaming = True
        state.last_state_t = time.monotonic()
        radio_snapshot = {"live": True, "connected": True, "rx_count": 5}
        text, color = app.connection_status(state, radio_snapshot)
        self.assertIn("Ao vivo | /dev/ttyUSB1", text)
        self.assertIn("Rádio ao vivo", text)
        self.assertEqual(color, config.OK_COLOR)
        text, _color = app.connection_status(state)
        self.assertEqual(text, "Ao vivo | /dev/ttyUSB1")

    def test_radio_only_header_shows_the_active_link(self):
        snapshot = {"live": True, "connected": True, "rx_count": 5}
        self.assertEqual(app.connection_status(BenchState(), snapshot), ("Rádio ao vivo", config.OK_COLOR))

    def test_radio_panel_placeholder_snapshot_is_disconnected(self):
        snapshot = radio_panel._empty_snapshot()
        self.assertFalse(snapshot["live"])
        self.assertFalse(snapshot["connected"])
        self.assertIsNone(snapshot["telemetry"])
        self.assertEqual(snapshot["messages"], [])
        text, color = radio_panel.status_text(snapshot)
        self.assertEqual(text, "Desconectado · reconectando")
        self.assertEqual(color, config.BAD_COLOR)


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
        self.assertEqual(health.supply_color(0b010), config.DEMO_COLOR)
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
        self.assertEqual(health.connection_color(0), config.DEMO_COLOR)

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

    def test_clear_button_drops_the_history_and_returns_the_view_to_live(self):
        state = BenchState()
        state.console.extend(("first", "second"))
        state.console_total = 9
        state.console_offset = 3
        state.console_sel = {"anchor": (0, 0), "focus": (0, 5)}
        console.clear(state)
        self.assertEqual(list(state.console), [])
        self.assertEqual(state.console_offset, 0)
        self.assertIsNone(state.console_sel)
        # External cursors keep their meaning: the absolute counter does not go back.
        self.assertEqual(state.console_total, 9)

    def test_click_on_the_clear_button_drops_the_history(self):
        state = BenchState()
        state.console.extend(("first", "second"))
        width, height = 800, 36 + 21 * 3
        left, top, right, bottom = console.clear_button_rect(width)
        self.assertTrue(0 < left < right <= width)
        self.assertLess(bottom, console.CONSOLE_HEADER_H)
        with (
            patch.object(console.dpg, "get_item_rect_size", return_value=(width, height)),
            patch.object(console.dpg, "get_item_rect_min", return_value=(0, 0)),
        ):
            console.press(state, ((left + right) / 2.0, (top + bottom) / 2.0))
        self.assertEqual(list(state.console), [])
        self.assertIsNone(state.console_sel)

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
