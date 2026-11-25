"""Regression checks for the motor wiring calibration tab and its commands."""

import time
import unittest
from unittest.mock import Mock, patch

from robot_bench import calibration, config, demo, protocol
from robot_bench.serial_io import BenchReader
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import DebugCalibration


def calibration_state(*, confirm=False, live=True):
    """Bench state on a live link with an idle calibration frame."""
    state = BenchState()
    state.reader = Mock()
    state.connected = "/dev/ttyUSB0"
    state.streaming = live
    if live:
        state.last_state_t = time.monotonic()
    state.set_calibration_confirm(confirm)
    state.update_calibration(demo.idle_calibration_frame(1))
    return state


class CalibrationHelperTests(unittest.TestCase):
    def test_subtab_content_height_leaves_room_for_the_tab_bar(self):
        self.assertEqual(calibration.content_height(492), 492 - calibration.SUBTAB_BAR_HEIGHT)
        self.assertEqual(calibration.content_height(492, 40), 452)
        self.assertEqual(calibration.content_height(100), calibration.MIN_CONTENT_HEIGHT)

    def test_active_subtab_defaults_to_encoders(self):
        with patch.object(calibration.dpg, "does_item_exist", return_value=False):
            self.assertEqual(calibration.active_subtab(), calibration.SUBTAB_ENCODERS)
        with (
            patch.object(calibration.dpg, "does_item_exist", return_value=True),
            patch.object(calibration.dpg, "get_value", return_value="motors_sidebar_tabs"),
        ):
            self.assertEqual(calibration.active_subtab(), calibration.SUBTAB_ENCODERS)
        with (
            patch.object(calibration.dpg, "does_item_exist", return_value=True),
            patch.object(calibration.dpg, "get_value", return_value=calibration.SUBTAB_CONTROL),
        ):
            self.assertEqual(calibration.active_subtab(), calibration.SUBTAB_CONTROL)

    def test_freshness_helpers_track_the_stale_window(self):
        state = BenchState()
        now = 100.0
        self.assertFalse(calibration.calibration_fresh(state, now))
        state.calibration = demo.idle_calibration_frame(1)
        state.last_calibration_t = now
        self.assertTrue(calibration.calibration_fresh(state, now))
        self.assertFalse(calibration.calibration_running(state, now))
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        self.assertTrue(calibration.calibration_running(state, now))
        state.last_calibration_t = now - config.CALIBRATION_STALE_S - 0.1
        self.assertFalse(calibration.calibration_fresh(state, now))
        self.assertFalse(calibration.calibration_running(state, now))

    def test_start_button_needs_confirmation_live_link_and_idle_routine(self):
        now = 100.0
        state = BenchState()
        state.update_calibration(demo.idle_calibration_frame(1))
        state.last_calibration_t = now
        self.assertFalse(calibration.calibration_start_enabled(state, now))
        state.set_calibration_confirm(True)
        self.assertFalse(calibration.calibration_start_enabled(state, now))
        state.connected = "/dev/ttyUSB0"
        state.streaming = True
        state.last_state_t = now
        self.assertTrue(calibration.calibration_start_enabled(state, now))
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        self.assertFalse(calibration.calibration_start_enabled(state, now))
        state.calibration = demo.idle_calibration_frame(1)
        state.last_state_t = now - config.SENSOR_STALE_S - 0.1
        self.assertFalse(calibration.calibration_start_enabled(state, now))
        state.last_state_t = now
        state.calibration_confirm = False
        self.assertFalse(calibration.calibration_start_enabled(state, now))

    def test_start_button_works_in_demo_without_a_link(self):
        now = 100.0
        state = BenchState()
        state.demo = True
        state.set_calibration_confirm(True)
        state.update_calibration(demo.idle_calibration_frame(1))
        state.last_calibration_t = now
        self.assertTrue(calibration.calibration_start_enabled(state, now))
        state.demo_calibration_start = now
        self.assertFalse(calibration.calibration_start_enabled(state, now))

    def test_stop_button_follows_running_and_demo_runs(self):
        now = 100.0
        state = BenchState()
        state.update_calibration(demo.idle_calibration_frame(1))
        state.last_calibration_t = now
        self.assertFalse(calibration.calibration_stop_enabled(state, now))
        state.calibration = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING)
        self.assertTrue(calibration.calibration_stop_enabled(state, now))
        state.calibration = demo.idle_calibration_frame(1)
        state.demo = True
        state.demo_calibration_start = now
        self.assertTrue(calibration.calibration_stop_enabled(state, now))

    def test_level_ratios_normalize_each_column_and_keep_noise_empty(self):
        self.assertEqual(calibration.level_ratios([0, 0, 0, 0]), [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(calibration.level_ratios([600, 0, 0, 0]), [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(calibration.level_ratios([600, 300, 150, 0]), [1.0, 0.5, 0.25, 0.0])
        noise = calibration.level_ratios([50, 100, 25, 0])
        self.assertLessEqual(max(noise), 0.5)
        self.assertEqual(noise[0], 0.25)

    def test_error_text_covers_every_failure_reason(self):
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_NONE), "Sem erro")
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_NO_MOVEMENT), "Sem movimento detectado")
        self.assertEqual(
            calibration.error_text(protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG), "Sinal analógico ambíguo"
        )
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_AMBIGUOUS_I2C), "Sinal I2C ambíguo")
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_BAD_MAPPING), "Mapeamento inválido")
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_ABORTED), "Cancelada")
        self.assertEqual(calibration.error_text(protocol.CALIBRATION_ERROR_I2C), "Falha de I2C")
        self.assertEqual(calibration.error_text(9), "Código 9")

    def test_magnet_warning_uses_the_saved_motor_without_replacing_calibration_status(self):
        state = calibration_state()
        self.assertEqual(calibration.magnet_warning_text(state.calibration), "")
        frame = demo.calibration_frame(
            2,
            state=protocol.CALIBRATION_RUNNING,
            progress=30,
            mapping_valid=1,
            mux_map=[1, 3, 0, 2],
            magnet_warning=[
                0,
                protocol.CALIBRATION_ERROR_MAGNET_TOO_CLOSE,
                0,
                protocol.CALIBRATION_ERROR_MAGNET_TOO_FAR,
            ],
        )
        state.update_calibration(frame)
        self.assertEqual(
            calibration.magnet_warning_text(frame),
            "Aviso · M1 · Ímã perto demais, M2 · Ímã longe demais ou ausente",
        )
        self.assertEqual(
            calibration.status_text(state, state.last_calibration_t),
            ("Calibrando · 30% concluído", config.ACCENT),
        )
        self.assertEqual(calibration.magnet_warning_text(None), "")

    def test_magnet_warning_prefers_current_i2c_match_over_saved_motor(self):
        frame = demo.calibration_frame(
            2,
            state=protocol.CALIBRATION_RUNNING,
            mapping_valid=1,
            mux_map=[2, 3, 1, 0],
            i2c_ready=0b0010,
            detected_mux_map=[0, 2, 2, 3],
            magnet_warning=[
                0, 0, protocol.CALIBRATION_ERROR_MAGNET_TOO_FAR, protocol.CALIBRATION_ERROR_MAGNET_TOO_CLOSE,
            ],
        )
        self.assertEqual(
            calibration.magnet_warning_text(frame),
            "Aviso · M2 · Ímã longe demais ou ausente, mux 4 (motor não identificado) · Ímã perto demais",
        )

    def test_magnet_warning_leaves_unknown_and_conflicting_assignments_unidentified(self):
        warning = [0, 0, protocol.CALIBRATION_ERROR_MAGNET_TOO_FAR, 0]
        for frame in (
            demo.calibration_frame(1, magnet_warning=warning),
            demo.calibration_frame(
                2, mapping_valid=1, i2c_ready=0b0011, detected_mux_map=[2, 2, 0, 3], magnet_warning=warning
            ),
        ):
            with self.subTest(frame=frame):
                self.assertEqual(
                    calibration.magnet_warning_text(frame),
                    "Aviso · mux 3 (motor não identificado) · Ímã longe demais ou ausente",
                )

    def test_failure_text_names_the_failed_path(self):
        i2c_failed = demo.calibration_frame(
            1, analog_ready=0b0001, wheel_error=[protocol.CALIBRATION_ERROR_NO_MOVEMENT, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(i2c_failed, 0), "sem movimento no I2C")
        ambiguous_i2c = demo.calibration_frame(
            1, analog_ready=0b0001, wheel_error=[protocol.CALIBRATION_ERROR_AMBIGUOUS_I2C, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(ambiguous_i2c, 0), "sinal I2C ambíguo")
        i2c_bus = demo.calibration_frame(
            1, analog_ready=0b0001, wheel_error=[protocol.CALIBRATION_ERROR_I2C, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(i2c_bus, 0), "falha de I2C")
        analog_failed = demo.calibration_frame(
            1, i2c_ready=0b0001, wheel_error=[protocol.CALIBRATION_ERROR_NO_MOVEMENT, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(analog_failed, 0), "sem movimento no analógico")
        ambiguous_analog = demo.calibration_frame(
            1, i2c_ready=0b0001, wheel_error=[protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(ambiguous_analog, 0), "sinal analógico ambíguo")
        both_failed = demo.calibration_frame(
            1, wheel_error=[protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG, 0, 0, 0]
        )
        self.assertEqual(calibration.failure_text(both_failed, 0), "sinal analógico ambíguo")

    def test_mapping_text_names_the_analog_input_and_mux_channel(self):
        calib = demo.calibration_frame(1, analog_map=[2, 0, 3, 1], mux_map=[1, 3, 0, 2], range_mask=0b0101)
        self.assertEqual(calibration.analog_text(calib, 0), "ENC3")
        self.assertEqual(calibration.mux_text(calib, 0), "mux 2")
        self.assertEqual(calibration.mapping_text(calib, 0), "ENC3 · mux 2")
        self.assertEqual(calibration.mapping_text(calib, 3), "ENC2 · mux 3")
        self.assertEqual(calibration.range_text(calib, 0), "pronto")
        self.assertEqual(calibration.range_text(calib, 1), "sem faixa")
        self.assertEqual(calibration.analog_text(None, 0), "—")
        self.assertEqual(calibration.mux_text(None, 0), "—")
        self.assertEqual(calibration.mapping_text(None, 0), "—")
        self.assertEqual(calibration.range_text(None, 0), "—")
        self.assertEqual(
            calibration.mapping_header(None),
            ("Aguardando quadro de calibração", config.TEXT_DIM),
        )
        self.assertEqual(
            calibration.mapping_header(calib),
            ("Mapa padrão · não calibrado", config.BAD_COLOR),
        )
        self.assertEqual(
            calibration.mapping_header(demo.done_calibration_frame(1)),
            ("Mapa calibrado · gravado na NVS", config.OK_COLOR),
        )

    def test_adc_range_text_shows_the_stored_endpoints(self):
        self.assertEqual(calibration.ADC_RANGE_HEADER, "ADC mín–máx")
        done = demo.done_calibration_frame(1)
        minimum, maximum = demo.DEMO_ADC_RANGE[0]
        self.assertEqual(calibration.adc_range_text(done, 0), (f"{minimum}–{maximum}", config.TEXT))
        no_range = demo.calibration_frame(2)
        self.assertEqual(calibration.adc_range_text(no_range, 0), ("—", config.TEXT_DIM))
        self.assertEqual(calibration.adc_range_text(None, 0), ("—", config.TEXT_DIM))
        partial = demo.calibration_frame(
            3, range_mask=0b0101, adc_min_raw=[341, 0, 2965, 0], adc_max_raw=[2965, 0, 3270, 0]
        )
        self.assertEqual(calibration.adc_range_text(partial, 0), ("341–2965", config.TEXT))
        self.assertEqual(calibration.adc_range_text(partial, 1), ("—", config.TEXT_DIM))
        # A rejected raw range keeps the mask bit via the mV fallback but has no endpoints.
        rejected = demo.calibration_frame(
            4, range_mask=0xF, adc_min_raw=[0, 0, 0, 0], adc_max_raw=[0, 0, 0, 0]
        )
        self.assertEqual(calibration.adc_range_text(rejected, 2), ("—", config.TEXT_DIM))
        equal = demo.calibration_frame(
            5, range_mask=0b0001, adc_min_raw=[900, 0, 0, 0], adc_max_raw=[900, 0, 0, 0]
        )
        self.assertEqual(calibration.adc_range_text(equal, 0), ("—", config.TEXT_DIM))

    def test_stage_text_and_summary_flag_non_reduced_encoders(self):
        reduced = demo.calibration_frame(1)
        self.assertEqual(calibration.stage_text(reduced, 0), ("REDUCED", config.OK_COLOR))
        self.assertEqual(calibration.stage_summary(reduced), ("Encoders em modo REDUCED", config.OK_COLOR))
        self.assertEqual(calibration.stage_text(None, 0), ("—", config.TEXT_DIM))
        self.assertIn("aguardando", calibration.stage_summary(None)[0])
        full = demo.calibration_frame(2, encoder_stage=[1, 0, 1, 1])
        self.assertEqual(calibration.stage_text(full, 1), ("FULL", config.DEMO_COLOR))
        text, color = calibration.stage_summary(full)
        self.assertIn("M2 FULL", text)
        self.assertEqual(color, config.DEMO_COLOR)
        pwm = demo.calibration_frame(3, encoder_stage=[1, 1, 2, 1])
        self.assertEqual(calibration.stage_text(pwm, 2), ("PWM", config.BAD_COLOR))
        text, color = calibration.stage_summary(pwm)
        self.assertIn("M3 PWM", text)
        self.assertEqual(color, config.BAD_COLOR)
        unknown = demo.calibration_frame(4, encoder_stage=[1, 1, 1, 3])
        self.assertEqual(calibration.stage_text(unknown, 3), ("—", config.TEXT_DIM))
        self.assertIn("M4 desconhecido", calibration.stage_summary(unknown)[0])
        unverified = demo.calibration_frame(5, stage_mask=0b0111)
        self.assertEqual(calibration.stage_text(unverified, 3), ("—", config.TEXT_DIM))
        text, color = calibration.stage_summary(unverified)
        self.assertIn("M4 não verificado", text)
        self.assertEqual(color, config.DEMO_COLOR)

    def test_result_text_distinguishes_analog_only_both_failed_and_pending(self):
        calib = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING, result_mask=0b0011, i2c_ready=0b0001)
        self.assertEqual(calibration.result_text(calib, 0), calibration.RESULT_BOTH)
        self.assertEqual(calibration.result_text(calib, 1), calibration.RESULT_ANALOG)
        self.assertEqual(calibration.result_text(calib, 2), calibration.RESULT_PENDING)
        self.assertEqual(calibration.result_text(None, 0), "Aguardando")
        failed = demo.calibration_frame(
            1,
            state=protocol.CALIBRATION_FAILED,
            result_mask=0b0001,
            i2c_ready=0b0001,
            error_code=protocol.CALIBRATION_ERROR_NO_MOVEMENT,
        )
        self.assertEqual(calibration.result_text(failed, 1), calibration.RESULT_FAILED)
        stored = demo.idle_calibration_frame(1, calibrated=True)
        self.assertTrue(
            all(calibration.result_text(stored, wheel) == calibration.RESULT_STORED for wheel in range(4))
        )
        self.assertEqual(calibration.result_color(stored, 0), config.OK_COLOR)
        done = demo.done_calibration_frame(1)
        self.assertTrue(all(calibration.result_text(done, wheel) == calibration.RESULT_BOTH for wheel in range(4)))
        self.assertEqual(calibration.result_color(done, 0), config.OK_COLOR)
        self.assertEqual(calibration.result_color(calib, 1), config.DEMO_COLOR)
        self.assertEqual(calibration.result_color(failed, 1), config.BAD_COLOR)
        self.assertEqual(calibration.result_color(calib, 2), config.TEXT_DIM)
        partial = demo.calibration_frame(
            1,
            state=protocol.CALIBRATION_PARTIAL,
            result_mask=0b0011,
            i2c_ready=0b0001,
            analog_ready=0b0011,
            wheel_error=[
                0,
                0,
                protocol.CALIBRATION_ERROR_NO_MOVEMENT,
                protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG,
            ],
        )
        self.assertEqual(calibration.result_text(partial, 0), calibration.RESULT_BOTH)
        self.assertEqual(calibration.result_text(partial, 1), calibration.RESULT_ANALOG)
        self.assertEqual(calibration.result_text(partial, 2), calibration.RESULT_FAILED)
        self.assertEqual(calibration.result_text(partial, 3), calibration.RESULT_FAILED)
        self.assertEqual(calibration.result_color(partial, 2), config.BAD_COLOR)
        self.assertEqual(calibration.result_color(partial, 3), config.BAD_COLOR)

    def test_wheel_rows_build_the_result_table(self):
        rows = calibration.wheel_rows(demo.done_calibration_frame(1))
        self.assertEqual(len(rows), 4)
        self.assertEqual(rows[0]["mapping"], "ENC3 · mux 2")
        self.assertEqual(rows[0]["result"], calibration.RESULT_BOTH)
        empty = calibration.wheel_rows(None)
        self.assertEqual(empty[0]["mapping"], "—")
        self.assertEqual(empty[0]["result"], "Aguardando")

    def test_status_line_reports_waiting_stale_running_done_and_failure(self):
        now = 100.0
        state = BenchState()
        self.assertEqual(calibration.status_text(state, now)[0], "Aguardando quadro de calibração")
        state.update_calibration(demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING, progress=42))
        self.assertEqual(calibration.status_text(state, now)[0], "Calibrando · 42% concluído")
        state.last_calibration_t = now - config.CALIBRATION_STALE_S - 0.1
        self.assertEqual(calibration.status_text(state, now)[0], "Dados de calibração desatualizados")
        state.last_calibration_t = now
        state.calibration = demo.done_calibration_frame(1)
        self.assertEqual(calibration.status_text(state, now)[0], "Calibração concluída · mapa salvo na NVS")
        state.calibration = demo.calibration_frame(
            1, state=protocol.CALIBRATION_FAILED, error_code=protocol.CALIBRATION_ERROR_AMBIGUOUS_I2C
        )
        self.assertEqual(calibration.status_text(state, now)[0], "Falha · Sinal I2C ambíguo")
        state.calibration = demo.calibration_frame(
            1,
            state=protocol.CALIBRATION_PARTIAL,
            wheel_error=[
                0,
                protocol.CALIBRATION_ERROR_NO_MOVEMENT,
                0,
                protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG,
            ],
        )
        self.assertEqual(
            calibration.status_text(state, now),
            (
                "Calibração incompleta · M2 sem movimento detectado, M4 sinal analógico ambíguo",
                config.BAD_COLOR,
            ),
        )
        state.calibration = demo.idle_calibration_frame(1)
        self.assertEqual(
            calibration.status_text(state, now),
            ("Confirme o aviso de segurança para calibrar", config.TEXT_DIM),
        )

    def test_status_line_names_the_failed_path(self):
        now = 100.0
        state = BenchState()
        state.last_calibration_t = now
        state.calibration = demo.calibration_frame(
            1,
            state=protocol.CALIBRATION_PARTIAL,
            result_mask=0b0001,
            i2c_ready=0b0101,
            analog_ready=0b1001,
            wheel_error=[
                0,
                protocol.CALIBRATION_ERROR_AMBIGUOUS_I2C,
                protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG,
                protocol.CALIBRATION_ERROR_NO_MOVEMENT,
            ],
        )
        self.assertEqual(
            calibration.status_text(state, now),
            (
                "Calibração incompleta · M2 sinal I2C ambíguo, M3 sinal analógico ambíguo, "
                "M4 sem movimento no I2C",
                config.BAD_COLOR,
            ),
        )

    def test_spinning_text_names_the_wheel_and_the_outcome(self):
        running = demo.calibration_frame(1, state=protocol.CALIBRATION_RUNNING, wheel=2)
        self.assertEqual(calibration.spinning_text(running), "Girando M3 · Traseira dir.")
        done = demo.done_calibration_frame(1)
        self.assertEqual(calibration.spinning_text(done), "Rotina concluída · motores parados")
        self.assertEqual(
            calibration.spinning_text(
                demo.calibration_frame(1, state=protocol.CALIBRATION_PARTIAL)
            ),
            "Rotina concluída com falhas · motores parados",
        )
        self.assertEqual(
            calibration.spinning_text(
                demo.calibration_frame(1, state=protocol.CALIBRATION_FAILED)
            ),
            "Rotina interrompida · motores parados",
        )
        self.assertEqual(calibration.spinning_text(demo.idle_calibration_frame(1)), "Nenhum motor girando")
        self.assertEqual(calibration.spinning_text(None), "Nenhum motor girando")


class CalibrationCommandTests(unittest.TestCase):
    def test_start_requires_the_safety_confirmation(self):
        state = calibration_state()
        self.assertFalse(state.start_calibration())
        state.reader.send_frame.assert_not_called()
        self.assertTrue(any("confirme que o robô está levantado" in line for line in state.console))

    def test_start_requires_a_live_telemetry_link(self):
        state = calibration_state(confirm=True)
        state.streaming = False
        self.assertFalse(state.start_calibration())
        state.streaming = True
        state.last_state_t = time.monotonic() - config.SENSOR_STALE_S - 0.1
        self.assertFalse(state.start_calibration())
        state.connected = None
        self.assertFalse(state.start_calibration())
        state.reader.send_frame.assert_not_called()
        self.assertTrue(any("link serial sem telemetria" in line for line in state.console))

    def test_start_sends_the_control_op_and_cancels_the_sweep(self):
        state = calibration_state(confirm=True)
        state.sweep = True
        self.assertTrue(state.start_calibration())
        self.assertFalse(state.sweep)
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_START_CALIBRATION, 1)
        )

    def test_start_is_ignored_while_the_routine_runs(self):
        state = calibration_state(confirm=True)
        state.update_calibration(demo.calibration_frame(2, state=protocol.CALIBRATION_RUNNING))
        self.assertFalse(state.start_calibration())
        state.reader.send_frame.assert_not_called()

    def test_stop_sends_the_abort_op(self):
        state = calibration_state(confirm=True)
        state.stop_calibration()
        state.reader.send_frame.assert_called_once_with(
            protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_CALIBRATION, 1)
        )

    def test_demo_start_and_stop_are_local(self):
        state = BenchState()
        state.demo = True
        state.reader = Mock()
        state.set_calibration_confirm(True)
        state.update_calibration(demo.idle_calibration_frame(1))
        self.assertTrue(state.start_calibration())
        self.assertIsNotNone(state.demo_calibration_start)
        state.reader.send_frame.assert_not_called()
        state.stop_calibration()
        self.assertIsNone(state.demo_calibration_start)

    def test_set_targets_is_ignored_while_the_routine_runs(self):
        state = calibration_state(confirm=True)
        state.update_calibration(demo.calibration_frame(2, state=protocol.CALIBRATION_RUNNING))
        state.set_targets([100, 0, 0, 0])
        state.reader.send_frame.assert_not_called()
        self.assertTrue(any("alvos ignorados" in line for line in state.console))

    def test_set_targets_is_ignored_in_demo_while_the_routine_runs(self):
        state = BenchState()
        state.demo = True
        state.update_calibration(demo.calibration_frame(2, state=protocol.CALIBRATION_RUNNING))
        state.set_targets([100, 0, 0, 0])
        self.assertEqual(state.manual_targets, [0.0, 0.0, 0.0, 0.0])

    def test_serial_reader_routes_calibration_frames_to_the_state(self):
        state = BenchState()
        reader = BenchReader(state, None, 115200)
        frame = demo.idle_calibration_frame(4)
        reader._dispatch(protocol.MSG_CALIBRATION, frame)
        self.assertIs(state.calibration, frame)
        self.assertEqual(state.calibration_count, 1)

    def test_calibration_update_records_the_frame_and_count(self):
        state = BenchState()
        frame = demo.idle_calibration_frame(3)
        state.update_calibration(frame)
        self.assertIs(state.calibration, frame)
        self.assertEqual(state.calibration_count, 1)
        self.assertTrue(state.calibration_live())
        state.update_calibration(demo.idle_calibration_frame(4))
        self.assertEqual(state.calibration_count, 2)

    def test_clear_freshness_drops_the_frame_and_keeps_the_confirmation(self):
        state = calibration_state(confirm=True)
        state.clear_freshness()
        self.assertIsNone(state.calibration)
        self.assertEqual(state.last_calibration_t, 0.0)
        self.assertTrue(state.calibration_confirm)
        self.assertFalse(state.calibration_running())


class DemoCalibrationTests(unittest.TestCase):
    def test_idle_frame_starts_with_the_default_identity_map(self):
        frame = demo.idle_calibration_frame(7)
        self.assertEqual(frame["state"], protocol.CALIBRATION_IDLE)
        self.assertEqual(frame["wheel"], 4)
        self.assertEqual(frame["mapping_valid"], 0)
        self.assertEqual(frame["analog_map"], [0, 1, 2, 3])
        self.assertEqual(frame["mux_map"], [0, 1, 2, 3])
        self.assertEqual(frame["range_mask"], 0)
        self.assertEqual(frame["analog_ready"], 0)
        self.assertEqual(frame["wheel_error"], [0, 0, 0, 0])
        self.assertEqual(frame["encoder_stage"], [protocol.ENCODER_STAGE_REDUCED] * 4)
        self.assertEqual(frame["stage_mask"], 0xF)
        self.assertEqual(frame["adc_min_raw"], [0, 0, 0, 0])
        self.assertEqual(frame["adc_max_raw"], [0, 0, 0, 0])
        self.assertEqual(frame["sequence"], 7)

    def test_calibrated_idle_frame_keeps_the_detected_mapping(self):
        frame = demo.idle_calibration_frame(8, calibrated=True)
        self.assertEqual(frame["state"], protocol.CALIBRATION_IDLE)
        self.assertEqual(frame["mapping_valid"], 1)
        self.assertEqual(frame["analog_map"], list(demo.DEMO_CALIBRATION_ANALOG_MAP))
        self.assertEqual(frame["mux_map"], list(demo.DEMO_CALIBRATION_MUX_MAP))
        self.assertEqual(frame["range_mask"], 0xF)
        self.assertEqual(
            frame["adc_min_raw"], [minimum for minimum, _maximum in demo.DEMO_ADC_RANGE]
        )
        self.assertEqual(
            frame["adc_max_raw"], [maximum for _minimum, maximum in demo.DEMO_ADC_RANGE]
        )

    def test_running_frame_advances_per_wheel_and_spikes_the_mapped_inputs(self):
        frame = demo.running_calibration_frame(0.0, 1)
        self.assertEqual(frame["state"], protocol.CALIBRATION_RUNNING)
        self.assertEqual(frame["wheel"], 0)
        self.assertEqual(frame["progress"], 0)
        self.assertEqual(frame["result_mask"], 0)
        analog_slot = demo.DEMO_CALIBRATION_ANALOG_MAP[0]
        mux_channel = demo.DEMO_CALIBRATION_MUX_MAP[0]
        self.assertGreaterEqual(frame["analog_level"][analog_slot], demo.DEMO_CALIBRATION_LEVEL[0])
        self.assertGreaterEqual(frame["i2c_level"][mux_channel], demo.DEMO_CALIBRATION_LEVEL[0])
        self.assertEqual(sum(frame["analog_level"]) - frame["analog_level"][analog_slot], 0)
        later = demo.running_calibration_frame(1.9, 5)
        self.assertEqual(later["wheel"], 1)
        self.assertEqual(later["result_mask"], 0b0001)
        self.assertEqual(later["i2c_ready"], 0b0001)
        self.assertEqual(later["analog_ready"], 0b0001)
        self.assertEqual(later["progress"], 26)
        self.assertEqual(later["analog_map"][0], demo.DEMO_CALIBRATION_ANALOG_MAP[0])
        self.assertEqual(later["analog_map"][1], 1)
        self.assertEqual(later["mux_map"][0], demo.DEMO_CALIBRATION_MUX_MAP[0])

    def test_running_frame_becomes_done_with_the_stored_non_identity_map(self):
        frame = demo.running_calibration_frame(demo.DEMO_CALIBRATION_TOTAL_S, 5)
        self.assertEqual(frame, demo.done_calibration_frame(5))
        self.assertEqual(
            frame["adc_min_raw"], [minimum for minimum, _maximum in demo.DEMO_ADC_RANGE]
        )
        self.assertEqual(
            frame["adc_max_raw"], [maximum for _minimum, maximum in demo.DEMO_ADC_RANGE]
        )
        self.assertEqual(frame["state"], protocol.CALIBRATION_DONE)
        self.assertEqual(frame["wheel"], 4)
        self.assertEqual(frame["progress"], 100)
        self.assertEqual(frame["result_mask"], 0xF)
        self.assertEqual(frame["i2c_ready"], 0xF)
        self.assertEqual(frame["analog_ready"], 0xF)
        self.assertEqual(frame["range_mask"], 0xF)
        self.assertEqual(frame["mapping_valid"], 1)
        self.assertEqual(frame["analog_map"], list(demo.DEMO_CALIBRATION_ANALOG_MAP))
        self.assertEqual(frame["mux_map"], list(demo.DEMO_CALIBRATION_MUX_MAP))
        self.assertNotEqual(frame["analog_map"], list(range(4)))

    def test_simulated_frames_round_trip_through_the_wire_parser(self):
        for frame in (
            demo.idle_calibration_frame(1),
            demo.idle_calibration_frame(2, calibrated=True),
            demo.running_calibration_frame(2.5, 3),
            demo.done_calibration_frame(4),
            demo.calibration_frame(
                5,
                state=protocol.CALIBRATION_PARTIAL,
                wheel=4,
                progress=100,
                result_mask=0b0011,
                i2c_ready=0b0001,
                analog_ready=0b0011,
                wheel_error=[
                    0,
                    0,
                    protocol.CALIBRATION_ERROR_NO_MOVEMENT,
                    protocol.CALIBRATION_ERROR_I2C,
                ],
            ),
        ):
            with self.subTest(state=frame["state"]):
                message = DebugCalibration(**frame).encode()
                self.assertEqual(len(message), protocol.CALIBRATION_SIZE)
                self.assertEqual(protocol.parse_calibration(message), frame)


if __name__ == "__main__":
    unittest.main()
