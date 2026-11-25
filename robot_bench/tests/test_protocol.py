"""Wire-format checks tying the generated bitproto classes to the parsers."""

import struct
import time
import unittest
from unittest.mock import Mock, patch

from robot_bench import protocol, serial_io
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import (
    DBG_STREAM_ENCODER_STAGES,
    DebugCalibration,
    DebugRaw,
    DebugState,
    DebugSubscribe,
    DebugTuning,
    DebugWheel,
)


class StateWireFormatTests(unittest.TestCase):
    def test_state_parser_matches_generated_bitproto_layout(self):
        wheels = [
            DebugWheel(
                pwm=-500 + index,
                velocity=1000 - index * 100,
                target=-200 + index * 50,
                angle=(0, 1111, 2250, 3599)[index],
                pulse_velocity=(-2048, -375, 0, 2047)[index],
                pulse_count=0x01000000 + index,
            )
            for index in range(4)
        ]
        generated = DebugState(
            uptime_ms=123456,
            wheels=wheels,
            encoder_valid_mask=0b0101,
            sequence=98765,
            pulse_valid_mask=0b1010,
        )
        message = generated.encode()
        self.assertEqual(len(message), protocol.STATE_SIZE)
        expected = bytearray(struct.pack("<I", 123456))
        for wheel in wheels:
            angle_and_speed = wheel.angle | ((wheel.pulse_velocity & 0xFFF) << 12)
            expected += struct.pack("<3h", wheel.pwm, wheel.velocity, wheel.target)
            expected += angle_and_speed.to_bytes(3, "little")
            expected += struct.pack("<I", wheel.pulse_count)
        expected += struct.pack("<IB", 98765, 0xA5)
        self.assertEqual(message, expected)
        parsed = protocol.parse_state(message)
        self.assertEqual(parsed["uptime_ms"], 123456)
        self.assertEqual(parsed["valid_mask"], 0b0101)
        self.assertEqual(parsed["pulse_valid_mask"], 0b1010)
        self.assertEqual(parsed["sequence"], 98765)
        for index, wheel in enumerate(parsed["wheels"]):
            self.assertEqual(wheel["pwm"], -500 + index)
            self.assertEqual(wheel["vel_rpm"], 100.0 - index * 10.0)
            self.assertEqual(wheel["tgt_rpm"], -20.0 + index * 5.0)
            self.assertAlmostEqual(wheel["angle_deg"], wheels[index].angle / 10)
            self.assertEqual(wheel["pulse_rpm"], wheels[index].pulse_velocity)
            self.assertEqual(wheel["pulse_count"], 0x01000000 + index)
        self.assertIsNone(protocol.parse_state(message[:-1]))

    def test_subscription_matches_generated_schema(self):
        streams = protocol.STREAM_STATE | protocol.STREAM_TEXT
        payload = protocol.encode_subscription(streams)
        self.assertEqual(payload, DebugSubscribe(streams=streams).encode())
        self.assertEqual(len(protocol.encode_frame(protocol.MSG_SUBSCRIBE, payload)), 6)


class ConsoleTextTests(unittest.TestCase):
    """Plain text printed directly on the port (bootloader, early boot)."""

    def test_boot_text_is_peeled_and_frames_still_decode(self):
        decoder = protocol.FrameDecoder()
        boot = (
            b"ets Jun  8 2016 00:22:57\r\n\r\n"
            b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n"
            b"mode:DIO, clock div:2\r\n"
        )
        frame = protocol.encode_frame(protocol.MSG_LINK, struct.pack("<I", 1234))
        frames = decoder.feed(boot + frame)
        self.assertEqual([msg_type for msg_type, _message in frames], [protocol.MSG_LINK])
        self.assertEqual(
            decoder.drain_lines(),
            [
                "ets Jun  8 2016 00:22:57",
                "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)",
                "mode:DIO, clock div:2",
            ],
        )
        self.assertEqual(decoder.drain_lines(), [])

    def test_a_line_split_across_reads_is_joined(self):
        decoder = protocol.FrameDecoder()
        self.assertEqual(decoder.feed(b"I (27) boot: ESP-IDF v5.4\r\nI (32) boot.esp"), [])
        self.assertEqual(decoder.drain_lines(), ["I (27) boot: ESP-IDF v5.4"])
        decoder.feed(b"32: SPI Speed      : 40MHz\r\n")
        self.assertEqual(decoder.drain_lines(), ["I (32) boot.esp32: SPI Speed      : 40MHz"])

    def test_ansi_colours_never_reach_the_console(self):
        decoder = protocol.FrameDecoder()
        decoder.feed(b"\x1b[0;32mI (27) boot: ESP-IDF v5.4.2\x1b[0m\r\n")
        self.assertEqual(decoder.drain_lines(), ["I (27) boot: ESP-IDF v5.4.2"])

    def test_text_between_frames_is_kept(self):
        decoder = protocol.FrameDecoder()
        first = protocol.encode_frame(protocol.MSG_LINK, struct.pack("<I", 1))
        second = protocol.encode_frame(protocol.MSG_LINK, struct.pack("<I", 2))
        frames = decoder.feed(first + b"boot text line\r\n" + second)
        self.assertEqual(len(frames), 2)
        self.assertEqual(decoder.drain_lines(), ["boot text line"])

    def test_restart_keeps_a_pending_text_line_and_drops_half_frames(self):
        decoder = protocol.FrameDecoder()
        decoder.feed(b"e")
        decoder.restart()
        decoder.feed(b"ts Jun  8 2016 00:22:57\r\n")
        self.assertEqual(decoder.drain_lines(), ["ets Jun  8 2016 00:22:57"])
        decoder.feed(protocol.encode_frame(protocol.MSG_LINK, struct.pack("<I", 7))[:3])
        decoder.restart()
        frames = decoder.feed(protocol.encode_frame(protocol.MSG_LINK, struct.pack("<I", 9)))
        self.assertEqual([msg_type for msg_type, _message in frames], [protocol.MSG_LINK])
        self.assertEqual(decoder.drain_lines(), [])

    def test_binary_frames_are_never_read_as_text(self):
        decoder = protocol.FrameDecoder()
        # A frame whose payload is printable text and contains newlines: the
        # message type byte after the COBS code makes it non-printable.
        frame = protocol.encode_frame(protocol.MSG_TEXT, b"I (27) boot: ESP-IDF\nline two")
        frames = decoder.feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.drain_lines(), [])


class SerialReaderConsoleTests(unittest.TestCase):
    def test_serial_open_keeps_reset_lines_inactive_except_for_stalled_link_recovery(self):
        reader = serial_io.BenchReader(BenchState(), "fake", 115200)
        for reset_on_open in (False, True):
            reader._reset_on_open = reset_on_open
            port = Mock(port=None, dtr=True, rts=True)
            with patch.object(serial_io.serial, "Serial", return_value=port) as factory:
                port.open.side_effect = lambda: self.assertEqual(
                    (port.port, port.dtr, port.rts), ("fake", reset_on_open, reset_on_open)
                )
                self.assertIs(reader._connect(), port)
                factory.assert_called_once_with(
                    baudrate=115200, timeout=0.05, write_timeout=0.5, exclusive=True
                )
                port.open.assert_called_once_with()
                self.assertEqual((port.dtr, port.rts, reader._reset_on_open), (False, False, False))

    def test_boot_text_reaches_the_console_from_the_serial_reader(self):
        class FakeSerial:
            def __init__(self, state, data):
                self.state = state
                self.data = bytearray(data)
                self.port = "fake"
                self.writes = []

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, _size):
                if not self.data:
                    self.state.quit = True
                    return b""
                chunk = bytes(self.data[:64])
                del self.data[:64]
                return chunk

            def write(self, frame):
                self.writes.append(frame)

        state = BenchState()
        wire = b"ets Jun  8 2016 00:22:57\r\nI (27) boot: ESP-IDF v5.4.2\r\n"
        serial_port = FakeSerial(state, wire)
        reader = serial_io.BenchReader(state, "fake", 115200)
        reader.ser = serial_port
        reader._session_start = time.monotonic()
        reader._read_loop(serial_port)
        state.quit = False
        lines = list(state.console)
        self.assertIn("ets Jun  8 2016 00:22:57", lines)
        self.assertIn("I (27) boot: ESP-IDF v5.4.2", lines)


class CalibrationWireFormatTests(unittest.TestCase):
    def test_encoder_stage_subscription_matches_bitproto(self):
        self.assertEqual(protocol.STREAM_ENCODER_STAGES, DBG_STREAM_ENCODER_STAGES)

    def test_calibration_frame_can_start_with_a_newline_byte(self):
        message = DebugCalibration(
            state=protocol.CALIBRATION_DONE,
            wheel=4,
            progress=100,
            result_mask=15,
            i2c_ready=15,
            mapping_valid=1,
            analog_map=[2, 3, 0, 1],
        ).encode()
        frame = protocol.encode_frame(protocol.MSG_CALIBRATION, message)
        self.assertEqual(frame[:1], b"\n")
        expected = [(protocol.MSG_CALIBRATION, message)]

        self.assertEqual(protocol.FrameDecoder().feed(frame), expected)

        decoder = protocol.FrameDecoder()
        self.assertEqual(decoder.feed(b"\n" + frame[:1]), [])
        self.assertEqual(decoder.feed(frame[1:]), expected)
        self.assertEqual(decoder.drain_lines(), [])

    def test_calibration_parser_matches_generated_bitproto_layout(self):
        generated = DebugCalibration(
            state=protocol.CALIBRATION_PARTIAL,
            wheel=4,
            progress=100,
            result_mask=0b1011,
            i2c_ready=0b1111,
            mapping_valid=1,
            analog_map=[2, 0, 3, 1],
            mux_map=[1, 3, 0, 2],
            analog_level=[-120, 800, 0, 1500],
            i2c_level=[12, -40, 900, 0],
            range_mask=0b1010,
            error_code=protocol.CALIBRATION_ERROR_NONE,
            analog_ready=0b1011,
            wheel_error=[0, 0, protocol.CALIBRATION_ERROR_AMBIGUOUS_ANALOG, 0],
            sequence=123456,
            encoder_stage=[
                protocol.ENCODER_STAGE_FULL,
                protocol.ENCODER_STAGE_REDUCED,
                protocol.ENCODER_STAGE_PWM,
                protocol.ENCODER_STAGE_UNKNOWN,
            ],
            stage_mask=0b1011,
            adc_min_raw=[341, 0, 2965, 500],
            adc_max_raw=[3680, 0, 3270, 4000],
            magnet_warning=[
                0,
                protocol.CALIBRATION_ERROR_MAGNET_TOO_CLOSE,
                0,
                protocol.CALIBRATION_ERROR_MAGNET_TOO_FAR,
            ],
            detected_mux_map=[3, 1, 2, 0],
        )
        message = generated.encode()
        self.assertEqual(len(message), protocol.CALIBRATION_SIZE)
        self.assertEqual(
            message,
            struct.pack(
                "<6B4B4B4h4hBBB4BI4BB4H4H4B4B",
                4,
                4,
                100,
                0b1011,
                0b1111,
                1,
                2,
                0,
                3,
                1,
                1,
                3,
                0,
                2,
                -120,
                800,
                0,
                1500,
                12,
                -40,
                900,
                0,
                0b1010,
                0,
                0b1011,
                0,
                0,
                2,
                0,
                123456,
                0,
                1,
                2,
                3,
                0b1011,
                341,
                0,
                2965,
                500,
                3680,
                0,
                3270,
                4000,
                0,
                7,
                0,
                8,
                3,
                1,
                2,
                0,
            ),
        )
        parsed = protocol.parse_calibration(message)
        self.assertEqual(
            parsed,
            {
                "state": 4,
                "wheel": 4,
                "progress": 100,
                "result_mask": 0b1011,
                "i2c_ready": 0b1111,
                "mapping_valid": 1,
                "analog_map": [2, 0, 3, 1],
                "mux_map": [1, 3, 0, 2],
                "analog_level": [-120, 800, 0, 1500],
                "i2c_level": [12, -40, 900, 0],
                "range_mask": 0b1010,
                "error_code": 0,
                "analog_ready": 0b1011,
                "wheel_error": [0, 0, 2, 0],
                "sequence": 123456,
                "encoder_stage": [0, 1, 2, 3],
                "stage_mask": 0b1011,
                "adc_min_raw": [341, 0, 2965, 500],
                "adc_max_raw": [3680, 0, 3270, 4000],
                "magnet_warning": [0, 7, 0, 8],
                "detected_mux_map": [3, 1, 2, 0],
            },
        )
        self.assertEqual(protocol.ENCODER_STAGE_FULL, 0)
        self.assertEqual(protocol.ENCODER_STAGE_REDUCED, 1)
        self.assertEqual(protocol.ENCODER_STAGE_PWM, 2)
        self.assertEqual(protocol.ENCODER_STAGE_UNKNOWN, 3)
        self.assertIsNone(protocol.parse_calibration(message[:-1]))
        self.assertEqual(
            protocol.parse_frame(protocol.MSG_CALIBRATION, message),
            parsed,
        )

    def test_generated_class_decodes_the_packed_layout(self):
        message = struct.pack(
            "<6B4B4B4h4hBBB4BI4BB4H4H4B4B",
            4,
            4,
            100,
            0b0100,
            0b0110,
            1,
            3,
            0,
            2,
            1,
            2,
            0,
            3,
            1,
            -50,
            640,
            0,
            1200,
            8,
            -24,
            750,
            0,
            0b0101,
            0,
            0b0101,
            1,
            2,
            0,
            3,
            987,
            2,
            1,
            0,
            3,
            0b1101,
            410,
            0,
            3650,
            500,
            3680,
            0,
            3900,
            4000,
            8,
            0,
            7,
            0,
            0,
            3,
            1,
            2,
        )
        generated = DebugCalibration()
        generated.decode(bytearray(message))
        self.assertEqual(generated.state, protocol.CALIBRATION_PARTIAL)
        self.assertEqual(generated.analog_ready, 0b0101)
        self.assertEqual(generated.wheel_error, [1, 2, 0, 3])
        self.assertEqual(generated.error_code, protocol.CALIBRATION_ERROR_NONE)
        self.assertEqual(generated.sequence, 987)
        self.assertEqual(generated.encoder_stage, [2, 1, 0, 3])
        self.assertEqual(generated.stage_mask, 0b1101)
        self.assertEqual(generated.adc_min_raw, [410, 0, 3650, 500])
        self.assertEqual(generated.adc_max_raw, [3680, 0, 3900, 4000])
        self.assertEqual(generated.magnet_warning, [8, 0, 7, 0])
        self.assertEqual(generated.detected_mux_map, [0, 3, 1, 2])
        parsed = protocol.parse_calibration(message)
        self.assertEqual(parsed["analog_ready"], 0b0101)
        self.assertEqual(parsed["wheel_error"], [1, 2, 0, 3])
        self.assertEqual(parsed["encoder_stage"], [2, 1, 0, 3])
        self.assertEqual(parsed["stage_mask"], 0b1101)
        self.assertEqual(parsed["magnet_warning"], [8, 0, 7, 0])
        self.assertEqual(parsed["detected_mux_map"], [0, 3, 1, 2])

class TuningWireFormatTests(unittest.TestCase):
    def test_tuning_parser_matches_generated_bitproto_layout(self):
        generated = DebugTuning(
            state=protocol.TUNING_RUNNING,
            wheel=2,
            phase=protocol.TUNING_PHASE_MAX_SPEED,
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
            feedforward_rev=[-500, 0, -200, -1],
            velocity_ff_rev=[5100, 1, 2, 3],
            rise_ms_rev=[300, 301, 302, 303],
            overshoot_pct_rev=[1, 2, 3, 4],
            ss_error_pct_rev=[5, 6, 7, 8],
            max_rpm=1240,
            wheel_max_rpm=[1750, 1700, 1650, 1380],
        )
        message = bytes(generated.encode())
        self.assertEqual(len(message), protocol.TUNING_SIZE)
        # Sequence (field 15), reverse fields (16..20), then the speed limit
        # (field 21) and the per-wheel maxima (field 22).
        expected = struct.pack(
            "<6B20h4H4B4BI4h4h4H4B4BH4H",
            protocol.TUNING_RUNNING,
            2,
            protocol.TUNING_PHASE_MAX_SPEED,
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
            -500,
            0,
            -200,
            -1,
            5100,
            1,
            2,
            3,
            300,
            301,
            302,
            303,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            1240,
            1750,
            1700,
            1650,
            1380,
        )
        self.assertEqual(message, expected)
        parsed = protocol.parse_tuning(message)
        self.assertEqual(parsed["state"], protocol.TUNING_RUNNING)
        self.assertEqual(parsed["wheel"], 2)
        self.assertEqual(parsed["phase"], protocol.TUNING_PHASE_MAX_SPEED)
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
        self.assertEqual(parsed["feedforward_rev"], [-500.0, 0.0, -200.0, -1.0])
        self.assertEqual(parsed["velocity_ff_rev"], [51.0, 0.01, 0.02, 0.03])
        self.assertEqual(parsed["rise_ms_rev"], [300, 301, 302, 303])
        self.assertEqual(parsed["overshoot_pct_rev"], [1, 2, 3, 4])
        self.assertEqual(parsed["ss_error_pct_rev"], [5, 6, 7, 8])
        self.assertEqual(parsed["max_rpm"], 124.0)
        self.assertEqual(parsed["wheel_max_rpm"], [175.0, 170.0, 165.0, 138.0])
        self.assertIsNone(protocol.parse_tuning(message[:-1]))


class ControlOpTests(unittest.TestCase):
    def test_raw_encoder_selector_uses_the_signed_first_target(self):
        self.assertEqual(protocol.OP_SELECT_RAW_ENCODER, 11)
        self.assertEqual(
            protocol.encode_control(protocol.OP_SELECT_RAW_ENCODER, 3, [-1, 0, 0, 0]),
            struct.pack("<BI4h", 11, 3, -1, 0, 0, 0),
        )
        self.assertEqual(
            protocol.encode_control(protocol.OP_SELECT_RAW_ENCODER, 4, [2, 0, 0, 0]),
            struct.pack("<BI4h", 11, 4, 2, 0, 0, 0),
        )


class RawWireFormatTests(unittest.TestCase):
    def test_raw_parser_matches_generated_bitproto_layout(self):
        generated = DebugRaw(
            sequence=123456,
            analog_raw=[0, 4095, 2048, 1234],
            i2c_raw=[4095, 0, 0xFFFF, 2048],
            pulse_delta=[0, 65535, 12, 34],
            ekf_velocity=[-1234, 0, 5678, -1],
            applied_pwm=[-1023, 0, 512, 1023],
        )
        message = bytes(generated.encode())
        self.assertEqual(len(message), protocol.RAW_SIZE)
        expected = struct.pack(
            "<I4H4H4H4h4h",
            123456,
            0,
            4095,
            2048,
            1234,
            4095,
            0,
            0xFFFF,
            2048,
            0,
            65535,
            12,
            34,
            -1234,
            0,
            5678,
            -1,
            -1023,
            0,
            512,
            1023,
        )
        self.assertEqual(message, expected)
        parsed = protocol.parse_raw(message)
        self.assertEqual(parsed["sequence"], 123456)
        self.assertEqual(parsed["analog_raw"], [0, 4095, 2048, 1234])
        self.assertEqual(parsed["i2c_raw"], [4095, 0, 0xFFFF, 2048])
        self.assertEqual(parsed["pulse_delta"], [0, 65535, 12, 34])
        self.assertEqual(parsed["ekf_rpm"], [-123.4, 0.0, 567.8, -0.1])
        self.assertEqual(parsed["applied_pwm"], [-1023, 0, 512, 1023])
        self.assertEqual(protocol.parse_frame(protocol.MSG_RAW, message), parsed)
        self.assertIsNone(protocol.parse_raw(message[:-1]))


if __name__ == "__main__":
    unittest.main()
