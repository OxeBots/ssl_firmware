"""Raw range transport, measurement quality and session freshness checks."""

import struct
import time
import unittest
from unittest.mock import Mock, patch

from robot_bench import config, protocol, rendering, sensing
from robot_bench.serial_io import BenchReader
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import DebugTof


class RawTofTests(unittest.TestCase):
    def test_generated_wire_frame_survives_serial_fragmentation_without_filtering(self):
        distances = [-12, 0, 100, 2000, 4100, 32767, -32768, 815] * 8
        statuses = [5, 6, 9, 10, 3, 12, 0, 255] * 8
        generated = DebugTof(sequence=0xFFFF00FF, distance_mm=distances, target_status=statuses)
        message = generated.encode()
        self.assertEqual(len(message), protocol.TOF_SIZE)
        self.assertEqual(message, struct.pack("<I64h64B", generated.sequence, *distances, *statuses))
        wire = protocol.encode_frame(protocol.MSG_TOF, message)
        for chunk_size in (1, 17, 64, 198, 256):
            with self.subTest(chunk_size=chunk_size):
                decoder = protocol.FrameDecoder()
                frames = []
                for offset in range(0, len(wire), chunk_size):
                    frames.extend(decoder.feed(wire[offset:offset + chunk_size]))
                self.assertEqual(len(frames), 1)
                result = protocol.parse_frame(*frames[0])
                self.assertEqual(result, {
                    "sequence": generated.sequence, "distances": distances, "statuses": statuses,
                })
        self.assertIsNone(protocol.parse_tof(message[:132]))

    def test_quality_uses_status_instead_of_distance_cutoff(self):
        for distance, status, expected in (
            (4100, 5, "valid"), (2000, 6, "uncertain"), (2000, 9, "uncertain"),
            (100, 10, "uncertain"), (100, 12, "invalid"), (100, 3, "invalid"),
            (100, 42, "invalid"), (0, 5, "invalid"), (-12, 5, "invalid"),
            (815, 255, "missing"), (100, 0, "missing"),
        ):
            with self.subTest(distance=distance, status=status):
                self.assertEqual(sensing.tof_zone_quality(distance, status), expected)

    def test_frame_copies_both_arrays_and_disconnect_invalidates_every_stream(self):
        state = BenchState()
        frame = {"sequence": 3, "distances": [815] * 64, "statuses": [255] * 64}
        state.update_tof(frame)
        frame["distances"][0], frame["statuses"][0] = 0, 5
        self.assertEqual((state.tof[0], state.tof_status[0]), (815, 255))
        state.last_state_t = state.last_imu_t = state.last_health_t = time.monotonic()
        state.health = {"motion_inhibited": 0}
        state.connected, state.streaming = "test", True
        reader = BenchReader(state, "test", 115200)
        reader.ser = Mock()
        port = reader.ser
        reader._close()
        self.assertIsNone(state.connected)
        self.assertFalse(state.streaming)
        self.assertEqual((state.last_state_t, state.last_imu_t, state.last_tof_t, state.last_health_t), (0,) * 4)
        self.assertEqual(state.health, {})
        port.close.assert_called_once()
        self.assertEqual(
            port.write.call_args.args[0],
            protocol.encode_frame(protocol.MSG_CONTROL, protocol.encode_control(protocol.OP_STOP_STREAM)),
        )

    def test_boot_text_invalidates_previous_readings(self):
        state = BenchState()
        state.streaming = True
        state.last_state_t = state.last_imu_t = state.last_tof_t = state.last_health_t = time.monotonic()
        state.health = {"motion_inhibited": 0}
        reader = BenchReader(state, "test", 115200)
        reader._session_start = time.monotonic()
        reader.ser = Mock()

        def reboot(_size):
            state.quit = True
            return b"rst:0x1 boot: ESP-IDF"

        reader.ser.read.side_effect = reboot
        with patch("builtins.print"):
            reader._read_loop(reader.ser)
        self.assertFalse(state.streaming)
        self.assertEqual(state.last_tof_t, 0)
        self.assertEqual(state.health, {})


class TofModelTests(unittest.TestCase):
    def setUp(self):
        self.print_patch = patch("builtins.print")
        self.print_patch.start()
        self.addCleanup(self.print_patch.stop)

    def test_cell_texts_distinguish_uncertain_missing_and_failed_readings(self):
        for distance, status, show_label, expected in (
            (2000, 9, False, ("~2000", "S9")),
            (100, 12, False, ("100?", "S12")),
            (815, 255, False, ("—", "S255")),
            (4100, 5, False, ("4100", "S5")),
            (100, 5, True, ("100", "S5 Válido")),
            (0, 5, True, ("—", "S5 Sem distância")),
        ):
            with self.subTest(distance=distance, status=status):
                self.assertEqual(sensing.tof_cell_texts(distance, status, "live", show_label), expected)
        self.assertEqual(sensing.tof_cell_texts(2000, 9, "stale", False), ("—", "Obsoleto"))
        self.assertEqual(sensing.tof_cell_texts(2000, 9, "waiting", False), ("—", "Sem dados"))

    def test_every_status_code_has_a_label(self):
        for status in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 255):
            with self.subTest(status=status):
                self.assertIn(status, sensing.TOF_STATUS_LABELS)
        self.assertEqual(sensing.tof_status_label(100, 42), "Desconhecido")
        self.assertEqual(sensing.tof_status_label(815, 255), "Sem alvo")
        self.assertEqual(sensing.tof_status_label(0, 5), "Sem distância")

    def test_zone_counts_and_summary_line_match_the_frame(self):
        state = BenchState()
        state.tof = [100, 2000, 100, 815, 0, -12, 100, 4100] * 8
        state.tof_status = [5, 9, 12, 255, 0, 3, 10, 5] * 8
        state.tof_seq_seen = 7
        state.last_tof_t = time.monotonic()
        self.assertEqual(
            sensing.tof_zone_counts(state.tof, state.tof_status),
            {"valid": 16, "uncertain": 16, "invalid": 16, "missing": 16},
        )
        summary = sensing.tof_summary_text(state, time.monotonic())
        self.assertIn("Quadro 7", summary)
        self.assertIn("Válidos 16 · Incertos 16 · Rejeitados 16 · Ausentes 16", summary)

    def test_stream_mode_switches_from_waiting_to_live_to_stale(self):
        state = BenchState()
        now = time.monotonic()
        self.assertEqual(sensing.tof_stream_mode(state, now), "waiting")
        self.assertEqual(sensing.tof_summary_text(state, now), "Aguardando dados de distância")
        state.update_tof({"sequence": 7, "distances": [100] * 64, "statuses": [5] * 64})
        self.assertEqual(sensing.tof_stream_mode(state, now), "live")
        stale_time = state.last_tof_t + config.SENSOR_STALE_S + 0.1
        self.assertEqual(sensing.tof_stream_mode(state, stale_time), "stale")
        self.assertEqual(
            sensing.tof_summary_text(state, stale_time), "Dados de distância desatualizados · último quadro 7"
        )

    def test_hover_text_exposes_raw_values_even_without_a_target(self):
        self.assertEqual(
            sensing.tof_hover_text(2, 7, 815, 255),
            "Zona 3,8 · Distância bruta 815 mm · Sem alvo (status 255)",
        )
        self.assertEqual(
            sensing.tof_hover_text(0, 0, 2000, 9),
            "Zona 1,1 · Distância bruta 2000 mm · Incerto (status 9)",
        )

    def test_cell_style_marks_uncertainty_rejection_and_dark_text(self):
        fill, border, thickness, text_color = sensing.tof_cell_style(800, 5, "live")
        self.assertEqual((border, text_color), (config.BACKGROUND, (20, 22, 26)))
        self.assertEqual(fill, rendering.distance_color(800))
        self.assertEqual(sensing.tof_cell_style(2000, 9, "live")[1], config.DEMO_COLOR)
        self.assertEqual(sensing.tof_cell_style(2000, 9, "live")[2], 2.0)
        self.assertEqual(sensing.tof_cell_style(100, 12, "live")[1], config.BAD_COLOR)
        self.assertEqual(sensing.tof_cell_style(0, 0, "live")[:2], (config.INSET, config.PANEL_BORDER))
        self.assertEqual(sensing.tof_cell_style(2000, 9, "waiting")[:2], (config.INSET, config.PANEL_BORDER))

    def test_layout_keeps_a_square_grid_and_the_legend_inside_the_tab(self):
        for body_w, body_h in ((992, 580), (1248, 652), (1888, 932), (992, 480)):
            with self.subTest(size=(body_w, body_h)):
                header_h = 60
                geometry = sensing.tof_layout(body_w, body_h, header_h)
                grid = geometry["grid_side"]
                self.assertGreaterEqual(grid, sensing.TOF_MIN_GRID)
                self.assertLessEqual(grid, body_h - header_h)
                self.assertAlmostEqual(geometry["cell_side"], grid / 8.0)
                self.assertGreaterEqual(geometry["left_pad"], 0)
                self.assertLessEqual(
                    geometry["left_pad"] + grid + sensing.TOF_GRID_GAP + sensing.TOF_LEGEND_W, body_w
                )

    def test_legend_geometry_and_colors_cover_the_range(self):
        geometry = sensing.tof_legend_geometry(300)
        self.assertEqual(len(geometry["ticks"]), 5)
        self.assertLess(geometry["ticks"][0], geometry["ticks"][-1])
        self.assertEqual(len(sensing.tof_legend_colors()), sensing.TOF_LEGEND_SEGMENTS * sensing.TOF_LEGEND_SLICES)
        colors = sensing.tof_legend_colors()
        first_stop = rendering.TOF_COLOR_STOPS[0][1]
        last_stop = rendering.TOF_COLOR_STOPS[-1][1]
        self.assertLessEqual(sum(abs(a - b) for a, b in zip(colors[0], first_stop)), 12)
        self.assertLessEqual(sum(abs(a - b) for a, b in zip(colors[-1], last_stop)), 12)
        self.assertNotEqual(colors[0], colors[-1])


class ImuModelTests(unittest.TestCase):
    def test_attitude_and_sensor_freshness(self):
        state = BenchState()
        now = time.monotonic()
        self.assertFalse(sensing.imu_attitude_fresh(state, now))
        self.assertFalse(sensing.imu_sensor_fresh(state, now))
        self.assertEqual(sensing.imu_attitude_status(state, now), "Aguardando atitude")
        state.last_state_t = state.last_imu_t = now
        self.assertTrue(sensing.imu_attitude_fresh(state, now))
        self.assertTrue(sensing.imu_sensor_fresh(state, now))
        self.assertEqual(sensing.imu_attitude_status(state, now), "Atitude atual")
        stale = now + config.SENSOR_STALE_S + 0.1
        self.assertFalse(sensing.imu_attitude_fresh(state, stale))
        self.assertEqual(sensing.imu_attitude_status(state, stale), "Atitude desatualizada")

    def test_current_reading_and_scale_formatting(self):
        self.assertEqual(sensing.format_angle(12.5), "+12.5°")
        self.assertEqual(sensing.format_angle(0.0), "+0.0°")
        self.assertEqual(sensing.format_reading(0), "+0.000")
        self.assertEqual(sensing.format_reading(-0.034), "-0.034")
        self.assertEqual(sensing.format_scale(600), "±600")
        self.assertEqual(sensing.format_scale(0.1), "±0.1")
        self.assertEqual(sensing.history_span_label("°"), "Histórico de 12 s · °")
        self.assertEqual(sensing.history_span_label("mG"), "Histórico de 12 s · mG")

    def test_recent_samples_drop_old_values_and_keep_the_newest(self):
        samples = [(8.0, 1), (90.0, 2), (99.5, 3), (100.0, 4)]
        self.assertEqual(sensing.recent_samples(samples, 100.0), [(90.0, 2), (99.5, 3), (100.0, 4)])
        self.assertEqual(sensing.recent_samples(samples, 80.0), [(90.0, 2), (99.5, 3), (100.0, 4)])
        self.assertEqual(sensing.recent_samples([], 100.0), [])

    def test_orientation_and_sensor_samples_project_one_axis(self):
        orientation = [(1.0, 10.0, 20.0, 30.0)]
        self.assertEqual(sensing.orientation_samples(orientation, 0), [(1.0, 10.0)])
        self.assertEqual(sensing.orientation_samples(orientation, 2), [(1.0, 30.0)])
        imu = [(1.0, (0.1, 0.2, 0.3), (1.0, 2.0, 3.0), (100.0, 200.0, 300.0))]
        self.assertEqual(sensing.sensor_samples(imu, 1, 1), [(1.0, 0.2)])
        self.assertEqual(sensing.sensor_samples(imu, 2, 1), [(1.0, 2.0)])
        self.assertEqual(sensing.sensor_samples(imu, 3, 0), [(1.0, 100.0)])

    def test_imu_layout_keeps_twelve_readable_plots(self):
        for body_w, body_h in ((992, 580), (1248, 652), (1888, 932)):
            with self.subTest(size=(body_w, body_h)):
                layout = sensing.imu_layout(body_w, body_h, 16, 90)
                self.assertGreaterEqual(layout["attitude_w"], 260)
                self.assertGreaterEqual(layout["plot_w"], 80)
                self.assertGreaterEqual(layout["plot_h"], 40)
                right_w = body_w - layout["attitude_w"] - sensing.STACK_GAP
                used = 3 * layout["plot_w"] + layout["label_w"] + 3 * sensing.STACK_GAP
                self.assertLessEqual(used, right_w)
                self.assertLessEqual(4 * layout["row_h"] + 3 * sensing.STACK_GAP, body_h)


if __name__ == "__main__":
    unittest.main()
