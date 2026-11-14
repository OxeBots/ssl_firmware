"""Wire-format checks tying the generated bitproto classes to the parsers."""

import struct
import unittest

from robot_bench import protocol
from scripts.serial_debug_protocol_bp import DebugCalibration, DebugState, DebugWheel


class StateWireFormatTests(unittest.TestCase):
    def test_state_parser_matches_generated_bitproto_layout(self):
        wheels = [
            DebugWheel(
                pwm=-500 + index,
                velocity=1000 - index * 100,
                target=-200 + index * 50,
                angle=index * 1111,
            )
            for index in range(4)
        ]
        generated = DebugState(
            uptime_ms=123456,
            wheels=wheels,
            yaw=1250,
            pitch=-700,
            roll=180,
            encoder_valid_mask=0b0101,
            sequence=98765,
        )
        message = generated.encode()
        self.assertEqual(len(message), protocol.STATE_SIZE)
        wheel_values = [
            value
            for wheel in wheels
            for value in (wheel.pwm, wheel.velocity, wheel.target, wheel.angle)
        ]
        self.assertEqual(
            message,
            struct.pack(
                "<I" + "3hH" * 4 + "3hBI",
                123456,
                *wheel_values,
                1250,
                -700,
                180,
                0b0101,
                98765,
            ),
        )
        parsed = protocol.parse_state(message)
        self.assertEqual(parsed["uptime_ms"], 123456)
        self.assertEqual(parsed["valid_mask"], 0b0101)
        self.assertEqual(parsed["sequence"], 98765)
        self.assertEqual((parsed["yaw"], parsed["pitch"], parsed["roll"]), (12.5, -7.0, 1.8))
        for index, wheel in enumerate(parsed["wheels"]):
            self.assertEqual(wheel["pwm"], -500 + index)
            self.assertEqual(wheel["vel_rpm"], 100.0 - index * 10.0)
            self.assertEqual(wheel["tgt_rpm"], -20.0 + index * 5.0)
            self.assertAlmostEqual(wheel["angle_deg"], index * 11.11)


class CalibrationWireFormatTests(unittest.TestCase):
    def test_calibration_parser_matches_generated_bitproto_layout(self):
        generated = DebugCalibration(
            state=1,
            wheel=2,
            progress=42,
            result_mask=0b0101,
            i2c_ready=0b0001,
            mapping_valid=1,
            analog_map=[2, 0, 3, 1],
            mux_map=[1, 3, 0, 2],
            analog_level=[-120, 800, 0, 1500],
            i2c_level=[12, -40, 900, 0],
            range_mask=0b1010,
            error_code=3,
            sequence=123456,
        )
        message = generated.encode()
        self.assertEqual(len(message), protocol.CALIBRATION_SIZE)
        self.assertEqual(
            message,
            struct.pack(
                "<6B4B4B4h4hBBI",
                1,
                2,
                42,
                0b0101,
                0b0001,
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
                3,
                123456,
            ),
        )
        parsed = protocol.parse_calibration(message)
        self.assertEqual(
            parsed,
            {
                "state": 1,
                "wheel": 2,
                "progress": 42,
                "result_mask": 0b0101,
                "i2c_ready": 0b0001,
                "mapping_valid": 1,
                "analog_map": [2, 0, 3, 1],
                "mux_map": [1, 3, 0, 2],
                "analog_level": [-120, 800, 0, 1500],
                "i2c_level": [12, -40, 900, 0],
                "range_mask": 0b1010,
                "error_code": 3,
                "sequence": 123456,
            },
        )
        self.assertIsNone(protocol.parse_calibration(message[:-1]))
        self.assertEqual(
            protocol.parse_frame(protocol.MSG_CALIBRATION, message),
            parsed,
        )


if __name__ == "__main__":
    unittest.main()
