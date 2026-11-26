"""USB serial device discovery for robot debug ports and ESP32 Base Stations."""

import errno
import re
import threading
import time

import serial
import serial.tools.list_ports

from . import base_station
from . import protocol as proto

# Give a board that is already rebooting time to answer the robot debug probe.
PROBE_WINDOW_S = 4.5
PROBE_RESEND_S = 0.9
SCAN_RETRY_S = 4.0


def list_candidate_ports():
    """All USB serial devices as (device, description) tuples."""
    candidates = []
    for port in serial.tools.list_ports.comports():
        device = port.device
        if re.search(r"ttyUSB\d+|ttyACM\d+|usbserial|usbmodem", device) or re.match(r"COM\d+$", device):
            if "Bluetooth" not in (port.description or ""):
                candidates.append((device, port.description or "Dispositivo serial USB"))
    return sorted(candidates)


class PortScanner(threading.Thread):
    """Probes USB serial ports in the background for Base Station and robot debug firmware."""

    def __init__(self, results, lock, active, keep=()):
        super().__init__(daemon=True)
        self.results = results  # device -> {desc, status, detail}; guarded by lock
        self.lock = lock
        self.active = active
        self.keep = set(keep)

    def run(self):
        while self.active.is_set():
            candidates = [
                (device, desc) for device, desc in list_candidate_ports() if device not in self.keep
            ]
            with self.lock:
                known = {device for device, _desc in candidates} | self.keep
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
        """Identify whether a port hosts an ESP32 Base Station or robot debug firmware."""
        status, detail = "empty", "no firmware response"

        # 1. Fast Base Station handshake (921600 baud, COBS+CRC8 BS_MSG_PING).
        try:
            info = base_station.probe_base_station(device, timeout_s=0.55)
            if info is not None:
                rate = base_station.RATE_NAMES.get(info["data_rate"], "?")
                status = "base_station"
                detail = f"Base Station · ch {info['channel']} · {rate}"
                with self.lock:
                    entry = self.results.get(device)
                    if self.active.is_set() and entry is not None:
                        entry["status"] = status
                        entry["detail"] = detail
                        entry["base_info"] = info
                return
        except (serial.SerialException, OSError) as error:
            if getattr(error, "errno", None) in (errno.EBUSY, errno.EAGAIN):
                status, detail = "busy", str(error)
                with self.lock:
                    entry = self.results.get(device)
                    if self.active.is_set() and entry is not None:
                        entry["status"] = status
                        entry["detail"] = detail
                return

        # 2. Fall back to robot SerialDebug probe (115200 baud).
        try:
            ser = serial.Serial(baudrate=115200, timeout=0.2, write_timeout=0.5, exclusive=True)
            ser.dtr = False
            ser.rts = False
            ser.port = device
            with ser:
                decoder = proto.FrameDecoder()
                probe = proto.encode_frame(proto.MSG_SUBSCRIBE, proto.encode_subscription(0))
                ser.write(probe)
                deadline = time.monotonic() + PROBE_WINDOW_S
                next_send = time.monotonic() + PROBE_RESEND_S
                while time.monotonic() < deadline and status == "empty":
                    if not self.active.is_set():
                        break
                    if time.monotonic() >= next_send:
                        next_send = time.monotonic() + PROBE_RESEND_S
                        ser.write(probe)
                    chunk = ser.read(128)
                    if any(
                        kind == proto.MSG_LINK and proto.parse_link(payload) is not None
                        for kind, payload in decoder.feed(chunk)
                    ):
                        status, detail = "firmware", "Serial debug firmware"
                if status == "firmware":
                    ser.write(
                        proto.encode_frame(
                            proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_STREAM)
                        )
                    )
        except (serial.SerialException, OSError, ValueError) as error:
            status = "busy" if getattr(error, "errno", None) in (errno.EBUSY, errno.EAGAIN) else "unavailable"
            detail = str(error)

        with self.lock:
            entry = self.results.get(device)
            if self.active.is_set() and entry is not None:
                entry["status"] = status
                entry["detail"] = detail
