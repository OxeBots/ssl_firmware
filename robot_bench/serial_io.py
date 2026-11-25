"""Background serial reader connecting to the debug firmware."""

import threading
import time

import serial

from . import protocol as proto

# Repeat the subscription well inside the 1.5 s firmware timeout.
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
        self._reset_on_open = False

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
                    self.state.add_console("porta indisponível - voltando para a seleção de portas")
                    self.state.picker_requested = True
                    return
                time.sleep(1.0)
                continue
            first_open_fail = None
            self.ser = ser
            self.state.clear_freshness()
            self.state.connected = ser.port
            self.decoder = proto.FrameDecoder()
            self.state.add_console(f"conectado a {ser.port} · {self.baud} baud")
            self._session_start = time.monotonic()
            self._read_loop(ser)
            self._close()
            time.sleep(RECONNECT_DELAY_S)

    def _connect(self):
        if not self.requested_port:
            return None
        try:
            ser = serial.Serial(baudrate=self.baud, timeout=0.05, write_timeout=0.5, exclusive=True)
            ser.port = self.requested_port
            # Keep the auto-reset lines inactive when opening a healthy link.
            if not self._reset_on_open:
                ser.dtr = False
                ser.rts = False
            ser.open()
        except (serial.SerialException, OSError, ValueError):
            return None
        if self._reset_on_open:
            self._reset_on_open = False
            try:
                ser.dtr = False
                ser.rts = False
            except (serial.SerialException, OSError):
                pass
        return ser

    def _read_loop(self, ser):
        # Send changes immediately and renew the subscription periodically.
        last_send = 0.0
        last_mask = None
        bytes_seen = 0
        reboot_hint_shown = False
        contention_hint_shown = False
        hint_deadline = time.monotonic() + 6.0
        while not self.state.quit:
            now = time.monotonic()
            last_link_t = self.state.last_link_t
            if self.state.streaming and last_link_t and now - last_link_t > STREAM_STALL_S:
                self.state.streaming = False
            if now - max(last_link_t, self._session_start) > REOPEN_STALL_S:
                # Connected but this session never produced (or long stopped
                # producing) state frames and resubscribing did not help:
                # reopen the port. Its DTR/RTS edge hardware-resets the board
                # for a clean session.
                self.state.add_console("sem quadros - reabrindo a porta para recuperar o link")
                self._reset_on_open = True
                return
            mask = self.state.stream_mask
            if mask != last_mask or now - last_send >= SEND_PERIOD_S:
                last_send = now
                last_mask = mask
                self.send_frame(proto.MSG_SUBSCRIBE, proto.encode_subscription(mask))
            try:
                chunk = ser.read(min(ser.in_waiting or 1, 1024))
            except (serial.SerialException, OSError):
                self.state.add_console("conexão serial perdida, reconectando...")
                return
            if not chunk:
                continue
            bytes_seen += len(chunk)
            self.state.rx_bytes = bytes_seen
            if any(sig in chunk for sig in _BOOT_SIGNATURES):
                if not reboot_hint_shown:
                    reboot_hint_shown = True
                    self.state.add_console("firmware reiniciou no meio da sessão - reativando a transmissão")
                self.state.clear_freshness()
                # Drop the half frame of the dead session; the bootloader lines
                # that follow are decoded as plain console text.
                self.decoder.restart()
                last_mask = None
            frames = self.decoder.feed(chunk)
            for line in self.decoder.drain_lines():
                self.state.add_console(line)
            for msg_type, message in frames:
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
                    "dados estão chegando, mas nenhum quadro é decodificado - feche outros monitores nesta porta"
                )

    def _dispatch(self, msg_type, data):
        if msg_type == proto.MSG_LINK:
            self.state.streaming = True
            self.state.update_link(data)
        elif msg_type == proto.MSG_STATE:
            self.state.update_state(data)
        elif msg_type == proto.MSG_IMU:
            self.state.update_imu(data)
        elif msg_type == proto.MSG_TOF:
            self.state.update_tof(data)
        elif msg_type == proto.MSG_HEALTH:
            self.state.update_health(data)
        elif msg_type == proto.MSG_TEXT:
            self.state.add_console(data)
        elif msg_type == proto.MSG_CALIBRATION:
            self.state.update_calibration(data)
        elif msg_type == proto.MSG_TUNING:
            self.state.update_tuning(data)
        elif msg_type == proto.MSG_WHEEL_TUNING:
            self.state.wheel_tuning_reply = data
        elif msg_type == proto.MSG_POSITION:
            self.state.update_position(data)
        elif msg_type == proto.MSG_RAW:
            self.state.update_raw(data)

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
        self.state.clear_freshness()

    def _stop_stream(self):
        self.send_frame(proto.MSG_CONTROL, proto.encode_control(proto.OP_STOP_STREAM))

    def send_frame(self, msg_type, message):
        """Transmit one control frame and report whether it was written."""
        with self.send_lock:
            if self.ser is None:
                return False
            try:
                frame = proto.encode_frame(msg_type, message)
                return self.ser.write(frame) == len(frame)
            except (serial.SerialException, OSError):
                return False
