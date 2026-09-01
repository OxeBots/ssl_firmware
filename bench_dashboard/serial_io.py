"""Background serial reader connecting to the debug firmware."""

import threading
import time

import serial

from . import protocol as proto

# START_STREAM / keepalive interval: well under the 1.5 s firmware watchdog.
# START_STREAM is re-sent until the first frame arrives: the board may reboot
# as the port opens or may still be booting when the first one is sent.
SEND_PERIOD_S = 0.9
RECONNECT_DELAY_S = 0.5
# Boot signatures that reveal the firmware rebooted mid-session.
_BOOT_SIGNATURES = (b"rst:0x", b"boot: ESP-IDF", b"load:0x")
# A streaming link with no state frame for this long has lost its
# subscription (board reset, frozen firmware): re-subscribe.
STREAM_STALL_S = 3.0
# When the requested port cannot be reopened for this long (cable pulled,
# board back on another port), hand control back to the port picker.
PICKER_FALLBACK_S = 5.0
# A connected session that produces no state frame for this long is wedged
# (firmware RX deaf after a power cycle, hung stream): reopen the port -
# the DTR/RTS edge at open hardware-resets the board for a clean session.
REOPEN_STALL_S = 10.0


class BenchReader(threading.Thread):
    """Connects to the debug serial port, reconnects forever, feeds parsers."""

    def __init__(self, state, port, baud):
        super().__init__(daemon=True)
        self.state = state
        self.requested_port = port
        self.baud = baud
        self.ser = None
        self.send_lock = threading.Lock()
        self.decoder = proto.FrameDecoder()

    def run(self):
        first_open_fail = None
        while not self.state.quit:
            ser = self._connect()
            if ser is None:
                now = time.monotonic()
                if first_open_fail is None:
                    first_open_fail = now
                elif now - first_open_fail >= PICKER_FALLBACK_S:
                    # The port is gone for good (cable pulled, board back on
                    # another port): rescan via the port picker.
                    self.state.connected = None
                    self.state.add_console("port unavailable - returning to port selection")
                    self.state.picker_requested = True
                    return
                time.sleep(1.0)
                continue
            first_open_fail = None
            self.ser = ser
            self.state.connected = ser.port
            self.decoder = proto.FrameDecoder()
            self.state.add_console(f"connected to {ser.port} @ {self.baud}")
            self._session_start = time.monotonic()
            self._read_loop(ser)
            self._close()
            time.sleep(RECONNECT_DELAY_S)

    def _connect(self):
        if self.requested_port:
            candidates = [self.requested_port]
        else:
            candidates = []
        for device in candidates:
            try:
                ser = serial.Serial(device, self.baud, timeout=0.05)
            except (serial.SerialException, OSError, ValueError):
                continue
            try:
                # Best effort: avoid the auto-reset pulse on open. Some
                # devices (and ptys) reject the modem ioctls; that only
                # means the reset trick does not apply.
                ser.dtr = False
                ser.rts = False
            except (serial.SerialException, OSError):
                pass
            return ser
        return None

    def _read_loop(self, ser):
        # Subscribe and keep the subscription alive. Until the first frame
        # arrives we re-send START_STREAM (a board that reboots on port open
        # loses the first one); afterwards a keepalive keeps the stream up.
        last_send = 0.0
        bytes_seen = 0
        reboot_hint_shown = False
        contention_hint_shown = False
        hint_deadline = time.monotonic() + 6.0
        while not self.state.quit:
            now = time.monotonic()
            if (
                self.state.streaming
                and self.state.last_state_t
                and now - self.state.last_state_t > STREAM_STALL_S
            ):
                # No state frames for a while: the subscription is gone
                # (board reset without boot text reaching us, or a frozen
                # stream). Fall back to START_STREAM until they return.
                self.state.streaming = False
            if now - max(self.state.last_state_t, self._session_start) > REOPEN_STALL_S:
                # Connected but this session never produced (or long stopped
                # producing) state frames and resubscribing did not help:
                # reopen the port. Its DTR/RTS edge hardware-resets the board
                # for a clean session.
                self.state.add_console("no frames - reopening port to recover the link")
                return
            if now - last_send >= SEND_PERIOD_S:
                last_send = now
                if self.state.streaming:
                    self.send_frame(
                        proto.MSG_CONTROL,
                        proto.encode_control(proto.OP_KEEPALIVE, self.state.control_sequence),
                    )
                else:
                    self.send_frame(
                        proto.MSG_CONTROL, proto.encode_control(proto.OP_START_STREAM)
                    )
            try:
                chunk = ser.read(256)
            except (serial.SerialException, OSError):
                self.state.add_console("serial link lost, reconnecting...")
                return
            if not chunk:
                continue
            bytes_seen += len(chunk)
            self.state.rx_bytes = bytes_seen
            if any(sig in chunk for sig in _BOOT_SIGNATURES):
                if not reboot_hint_shown:
                    reboot_hint_shown = True
                    self.state.add_console("firmware rebooted mid-session - resubscribing")
                # The rebooted firmware dropped its stream state: fall back to
                # START_STREAM until frames arrive again.
                self.state.streaming = False
            for msg_type, message in self.decoder.feed(chunk):
                data = proto.parse_frame(msg_type, message)
                if data is None:
                    continue
                self._dispatch(msg_type, data)
            # Bytes flow but no state frame ever decodes: another reader
            # (serial monitor) on the same port is eating the stream.
            if (
                not contention_hint_shown
                and not reboot_hint_shown
                and not self.state.streaming
                and bytes_seen > 1500
                and now > hint_deadline
            ):
                contention_hint_shown = True
                self.state.add_console(
                    "data is arriving but no frames decode - close other monitors on this port"
                )

    def _dispatch(self, msg_type, data):
        if msg_type == proto.MSG_STATE:
            self.state.streaming = True
            self.state.update_state(data)
        elif msg_type == proto.MSG_IMU:
            self.state.update_imu(data)
        elif msg_type == proto.MSG_TOF:
            self.state.update_tof(data)
        elif msg_type == proto.MSG_HEALTH:
            self.state.update_health(data)
        elif msg_type == proto.MSG_TEXT:
            self.state.add_console(data)

    def _close(self):
        self._stop_stream()
        with self.send_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.ser = None
        self.state.connected = None
        self.state.streaming = False

    def _stop_stream(self):
        self.send_frame(proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_STREAM))

    def send_frame(self, msg_type, message):
        """Transmit one control frame (best effort, thread-safe)."""
        with self.send_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(proto.encode_frame(msg_type, message))
            except (serial.SerialException, OSError):
                pass
