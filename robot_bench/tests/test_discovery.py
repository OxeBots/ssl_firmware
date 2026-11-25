"""USB picker regression checks."""

import errno
import threading
import unittest
from unittest.mock import MagicMock, patch

from robot_bench import discovery, protocol


class UsbDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.device = "/dev/ttyUSB0"
        self.results = {self.device: {"status": "probing"}}
        active = threading.Event()
        active.set()
        self.scanner = discovery.PortScanner(self.results, threading.Lock(), active)

    def test_distinguishes_busy_ports_from_disconnections(self):
        for error, expected in ((errno.EAGAIN, "busy"), (errno.ENODEV, "unavailable")):
            with self.subTest(error=error), patch.object(
                discovery.serial, "Serial", side_effect=OSError(error, "USB probe failed")
            ):
                self.scanner._probe(self.device)
                self.assertEqual(self.results[self.device]["status"], expected)

    def test_sets_modem_lines_before_opening_the_probe(self):
        port = MagicMock()

        def enter():
            self.assertEqual(port.port, self.device)
            self.assertFalse(port.dtr)
            self.assertFalse(port.rts)
            return port

        port.__enter__.side_effect = enter
        port.read.return_value = protocol.encode_frame(protocol.MSG_LINK, b"valid")
        with (
            patch.object(discovery.serial, "Serial", return_value=port) as factory,
            patch.object(protocol, "parse_link", return_value={}),
        ):
            self.scanner._probe(self.device)
        self.assertTrue(factory.call_args.kwargs["exclusive"])
        self.assertEqual(self.results[self.device]["status"], "firmware")
