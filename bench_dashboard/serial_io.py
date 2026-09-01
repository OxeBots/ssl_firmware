"""Background serial reader connecting to the bench firmware."""

import threading
import time

import serial

from .protocol import parse_line


class BenchReader(threading.Thread):
    """Connects to the bench serial port, reconnects forever, feeds parse_line."""

    def __init__(self, state, port, baud):
        super().__init__(daemon=True)
        self.state = state
        self.requested_port = port
        self.baud = baud
        self.ser = None
        self.send_lock = threading.Lock()

    def run(self):
        while not self.state.quit:
            ser = self._connect()
            if ser is None:
                time.sleep(1.0)
                continue
            self.ser = ser
            self.state.connected = ser.port
            self.state.console.append(f"connected to {ser.port} @ {self.baud}")
            self._read_loop(ser)
            self._close()

    def _connect(self):
        if self.requested_port:
            candidates = [self.requested_port]
        else:
            candidates = []
        for device in candidates:
            try:
                ser = serial.Serial(device, self.baud, timeout=0.05)
                ser.dtr = False  # avoid the auto-reset pulse on open
                ser.rts = False
                return ser
            except (serial.SerialException, OSError, ValueError):
                continue
        return None

    def _read_loop(self, ser):
        buffer = b""
        while not self.state.quit:
            try:
                chunk = ser.read(256)
            except (serial.SerialException, OSError):
                self.state.console.append("serial link lost, reconnecting...")
                return
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                parse_line(self.state, raw.decode("ascii", errors="replace").strip("\r"))

    def _close(self):
        with self.send_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.ser = None
        self.state.connected = None

    def send(self, text):
        with self.send_lock:
            if self.ser is None:
                return
            try:
                self.ser.write((text + "\n").encode("ascii"))
            except (serial.SerialException, OSError):
                pass
