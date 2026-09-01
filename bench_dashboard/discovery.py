"""USB serial device discovery and BenchDashboard firmware detection."""

import re
import threading
import time

import serial
import serial.tools.list_ports

# Lines that identify the BenchDashboard firmware on a port.
FIRMWARE_LINE_PREFIXES = ("S,", "T,", "INIT", "ACK", "Protoboard", "UNITY")
# Covers the firmware boot: opening the port may reset the ESP32.
PROBE_WINDOW_S = 4.5
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
    """Probes USB serial ports in the background for the bench firmware."""

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
                self._probe(device)
            deadline = time.monotonic() + SCAN_RETRY_S
            while self.active.is_set() and time.monotonic() < deadline:
                time.sleep(0.2)

    def _probe(self, device):
        """Open a port briefly and flag it when the bench firmware answers."""
        status, detail = "empty", "no firmware response"
        try:
            ser = serial.Serial(device, 115200, timeout=0.2)
        except (serial.SerialException, OSError, ValueError):
            status, detail = "busy", "in use or unavailable"
        else:
            try:
                ser.dtr = False
                ser.rts = False
                deadline = time.monotonic() + PROBE_WINDOW_S
                buffer = b""
                while time.monotonic() < deadline and status == "empty":
                    buffer += ser.read(128)
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        line = raw.decode("ascii", errors="replace").strip("\r")
                        if line.startswith(FIRMWARE_LINE_PREFIXES):
                            status, detail = "firmware", "BenchDashboard firmware"
                            break
            finally:
                ser.close()
        with self.lock:
            entry = self.results.get(device)
            if entry is not None:
                entry["status"] = status
                entry["detail"] = detail
