"""USB serial device discovery and debug-firmware detection."""

import re
import threading
import time

import serial
import serial.tools.list_ports

from . import protocol as proto

# Opening the port may reset the ESP32; give it time to boot before probing.
PROBE_WINDOW_S = 4.5
# START_STREAM re-send interval: covers a lost first frame when the board
# reboots as the port opens, or a firmware that is still booting.
PROBE_RESEND_S = 0.9
# Pause between full scan passes while the picker is open.
SCAN_RETRY_S = 4.0


def list_candidate_ports():
    """All USB serial devices as (device, description) tuples."""
    candidates = []
    for port in serial.tools.list_ports.comports():
        device = port.device
        if re.search(r"ttyUSB\d+|ttyACM\d+|usbserial|usbmodem", device) or re.match(r"COM\d+$", device):
            if "Bluetooth" not in (port.description or ""):
                candidates.append((device, port.description or "USB serial device"))
    return sorted(candidates)


class PortScanner(threading.Thread):
    """Probes USB serial ports in the background for the debug firmware."""

    def __init__(self, results, lock, active):
        super().__init__(daemon=True)
        self.results = results  # device -> {desc, status, detail}; guarded by lock
        self.lock = lock
        self.active = active  # threading.Event: keep scanning while set

    def run(self):
        while self.active.is_set():
            candidates = list_candidate_ports()
            with self.lock:
                known = {device for device, _desc in candidates}
                for device in [d for d in list(self.results) if d not in known]:
                    del self.results[device]
                for device, desc in candidates:
                    self.results.setdefault(
                        device, {"desc": desc, "status": "probing", "detail": ""}
                    )
            for device, _desc in candidates:
                if not self.active.is_set():
                    return
                try:
                    self._probe(device)
                except (serial.SerialException, OSError):
                    # A flapping or contended port must not kill the scanner.
                    with self.lock:
                        entry = self.results.get(device)
                        if entry is not None:
                            entry["status"] = "busy"
                            entry["detail"] = "in use or unavailable"
            deadline = time.monotonic() + SCAN_RETRY_S
            while self.active.is_set() and time.monotonic() < deadline:
                time.sleep(0.2)

    def _probe(self, device):
        """Open a port briefly and flag it when the debug firmware answers."""
        status, detail = "empty", "no firmware response (is another monitor using the port?)"
        try:
            ser = serial.Serial(device, 115200, timeout=0.2)
        except (serial.SerialException, OSError, ValueError):
            status, detail = "busy", "in use or unavailable"
        else:
            try:
                try:
                    ser.dtr = False
                    ser.rts = False
                except (serial.SerialException, OSError):
                    pass  # modem-control ioctls unsupported on this device
                decoder = proto.FrameDecoder()
                # The firmware streams only for subscribed clients. Re-send
                # START_STREAM periodically: the board may reboot as the port
                # opens or may still be booting when the first one is sent.
                ser.write(
                    proto.encode_frame(
                        proto.MSG_CONTROL, proto.encode_control(proto.OP_START_STREAM)
                    )
                )
                deadline = time.monotonic() + PROBE_WINDOW_S
                next_send = time.monotonic() + PROBE_RESEND_S
                while time.monotonic() < deadline and status == "empty":
                    if not self.active.is_set():
                        break
                    if time.monotonic() >= next_send:
                        next_send = time.monotonic() + PROBE_RESEND_S
                        ser.write(
                            proto.encode_frame(
                                proto.MSG_CONTROL, proto.encode_control(proto.OP_START_STREAM)
                            )
                        )
                    chunk = ser.read(128)
                    if chunk and decoder.feed(chunk):
                        status, detail = "firmware", "Serial debug firmware"
                ser.write(
                    proto.encode_frame(
                        proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_STREAM)
                    )
                )
            except (serial.SerialException, OSError):
                status, detail = "busy", "in use or unavailable"
            finally:
                ser.close()
        with self.lock:
            entry = self.results.get(device)
            if entry is not None:
                entry["status"] = status
                entry["detail"] = detail
