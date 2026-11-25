"""Regression checks for the live wheel tuning commands and editor."""

import math
import struct
import unittest

from robot_bench import protocol, wheel_tuning
from robot_bench.state import BenchState
from scripts.serial_debug_protocol_bp import DebugWheelTuning


class WheelTuningTests(unittest.TestCase):
    def test_wire_format_matches_bitproto(self):
        values = [0.0] * protocol.WHEEL_TUNING_FIELDS
        values[2], values[12], values[-1] = 0.03125, -123.5, 1.0
        fields = (1 << 2) | (1 << 12) | (1 << 22)
        generated = DebugWheelTuning(
            wheel=3,
            status=protocol.WHEEL_TUNING_REQUEST,
            sequence=42,
            fields=fields,
            values=struct.unpack("<23I", struct.pack("<23f", *values)),
        )
        request = protocol.encode_wheel_tuning(3, 42, fields, values)
        self.assertEqual(request, bytes(generated.encode()))
        self.assertIsNone(protocol.parse_wheel_tuning(request))
        generated.mode = 2
        generated.status = protocol.WHEEL_TUNING_OK
        self.assertEqual(
            protocol.parse_frame(protocol.MSG_WHEEL_TUNING, bytes(generated.encode())),
            {
                "wheel": 3,
                "status": protocol.WHEEL_TUNING_OK,
                "mode": 2,
                "sequence": 42,
                "fields": fields,
                "values": values,
            },
        )

    def test_draft_converts_only_edited_fields(self):
        values = [0.000123456789] * protocol.WHEEL_TUNING_FIELDS
        baseline = [wheel_tuning._display(i, value) for i, value in enumerate(values)]
        draft = baseline.copy()
        draft[6], draft[19], draft[22] = "100,5", "85", False

        fields, proposed = wheel_tuning._changes(values, draft, baseline)

        self.assertEqual(len(baseline), len(wheel_tuning.PARAMETERS))
        self.assertEqual(fields, (1 << 6) | (1 << 19) | (1 << 22))
        self.assertAlmostEqual(proposed[6], 100.5 * math.pi / 30)
        self.assertAlmostEqual(proposed[19], 0.085)
        self.assertEqual(proposed[22], 0.0)
        self.assertEqual(proposed[2], values[2])
        draft[6] = "nan"
        with self.assertRaisesRegex(ValueError, "Transição"):
            wheel_tuning._changes(values, draft, baseline)

    def test_live_edit_leaves_demo_target_and_other_wheels_intact(self):
        state = BenchState()
        state.demo = True
        state.selected_wheel = 2
        state.set_selected_target(-120.0)
        self.assertTrue(state.request_wheel_tuning(2, 1))
        values = state.wheel_tuning_reply["values"].copy()
        values[0] = 34.0
        self.assertTrue(state.request_wheel_tuning(2, 2, 1, values))
        self.assertEqual(state.wheel_tuning_reply["values"][0], 34.0)
        self.assertEqual(state.manual_targets, [0.0, 0.0, -120.0, 0.0])
        self.assertTrue(state.request_wheel_tuning(1, 3))
        self.assertEqual(state.wheel_tuning_reply["values"][0], 1.0)
        self.assertTrue(state.request_wheel_tuning(2, 4))
        self.assertEqual(state.wheel_tuning_reply["values"][0], 34.0)
