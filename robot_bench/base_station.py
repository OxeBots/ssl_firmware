"""COBS+CRC8 Bitproto transport client for the ESP32 DevKit Base Station.

Wire framing matches proto/base_station_protocol.bitproto and reuses
protocol.encode_frame / protocol.FrameDecoder (COBS + CRC-8 poly 0x07 + 0x00).
"""

import collections
import errno
import struct
import time

try:
    import fcntl
    import termios
except ImportError:  # Non-POSIX (Windows)
    fcntl = None
    termios = None

import serial
import serial.tools.list_ports

from . import protocol as proto

PROTO_VERSION = 2
DEFAULT_BAUD = 921600
RX_CHANNEL_FOLLOW_TX = 0xFF

# Host PC -> Base Station message IDs (0x80..0x87)
BS_MSG_PING = 0x80
BS_MSG_GET_INFO = 0x81
BS_MSG_SET_CONFIG = 0x82
BS_MSG_SCAN_REQ = 0x83
BS_MSG_RF_TX = 0x84
BS_MSG_RF_TX_CH = 0x85

# Base Station -> Host PC message IDs (0x88..0x8F)
BS_MSG_INFO = 0x88
BS_MSG_RF_TX_STATUS = 0x89
BS_MSG_RF_RX = 0x8A
BS_MSG_SCAN_RESULT = 0x8B
BS_MSG_LOG = 0x8C

BS_TX_FLAG_NO_ACK = 1
BS_TX_FLAG_EXPECT_REPLY = 2

BS_TX_STATUS_OK = 0
BS_TX_STATUS_MAX_RT = 1
BS_TX_STATUS_NO_ACK_SENT = 2
BS_TX_STATUS_RADIO_ERR = 3
BS_TX_STATUS_QUEUE_FULL = 4

BS_RATE_1MBPS = 0
BS_RATE_2MBPS = 1
BS_RATE_250KBPS = 2

RATE_NAMES = {
    BS_RATE_1MBPS: "1 Mbps",
    BS_RATE_2MBPS: "2 Mbps",
    BS_RATE_250KBPS: "250 Kbps",
}

DEFAULT_ADDR_SUFFIX = bytes((0xC6, 0x9A, 0xD3, 0xE5))
DEFAULT_BASE_ADDR_LSB = 0xB4

_PING_STRUCT = struct.Struct("<I")
_EMPTY_STRUCT = struct.Struct("<B")
_CONFIG_STRUCT = struct.Struct("<BBBBBB4sBB")
_SCAN_REQ_STRUCT = struct.Struct("<BBB")
_RF_TX_STRUCT = struct.Struct("<HBBB31s")
_RF_TX_CH_STRUCT = struct.Struct("<HBBBB31s")

_INFO_STRUCT = struct.Struct("<BBBBBBBIIIH4sBB")
_TX_STATUS_STRUCT = struct.Struct("<HBBBH")
_RF_RX_STRUCT = struct.Struct("<IBB31s")
_SCAN_RES_STRUCT = struct.Struct("<BBB126sB")
_LOG_STRUCT = struct.Struct("<BB96s")


def encode_channel_pair_param(tx_channel, rx_channel=RX_CHANNEL_FOLLOW_TX):
    """Encode (base_tx_ch, base_rx_ch) into RobotConfig.param (uint16)."""
    tx_ch = int(tx_channel) & 0x7F
    if rx_channel is None or rx_channel == RX_CHANNEL_FOLLOW_TX or int(rx_channel) == tx_ch:
        return tx_ch
    rx_ch = int(rx_channel) & 0x7F
    return 0x8000 | (rx_ch << 8) | tx_ch


class BaseStationError(Exception):
    """Raised when the ESP32 Base Station cannot be opened or does not respond."""


def encode_ping(seq=1):
    return proto.encode_frame(BS_MSG_PING, _PING_STRUCT.pack(int(seq) & 0xFFFFFFFF))


def encode_get_info():
    return proto.encode_frame(BS_MSG_GET_INFO, _EMPTY_STRUCT.pack(0))


def encode_set_config(
    channel,
    data_rate=BS_RATE_2MBPS,
    pa_level=3,
    retransmit_delay=1,
    retransmit_count=3,
    persist_nvs=True,
    addr_suffix=DEFAULT_ADDR_SUFFIX,
    base_addr_lsb=DEFAULT_BASE_ADDR_LSB,
    rx_channel=RX_CHANNEL_FOLLOW_TX,
):
    suffix = bytes(addr_suffix)[:4].ljust(4, b"\x00")
    rx_val = RX_CHANNEL_FOLLOW_TX if rx_channel is None else (int(rx_channel) & 0xFF)
    payload = _CONFIG_STRUCT.pack(
        int(channel) & 0xFF,
        int(data_rate) & 0xFF,
        int(pa_level) & 0xFF,
        int(retransmit_delay) & 0xFF,
        int(retransmit_count) & 0xFF,
        1 if persist_nvs else 0,
        suffix,
        int(base_addr_lsb) & 0xFF,
        rx_val,
    )
    return proto.encode_frame(BS_MSG_SET_CONFIG, payload)


def encode_scan_request(start_channel=0, end_channel=125, samples_per_channel=32):
    payload = _SCAN_REQ_STRUCT.pack(
        int(start_channel) & 0xFF,
        int(end_channel) & 0xFF,
        int(samples_per_channel) & 0xFF,
    )
    return proto.encode_frame(BS_MSG_SCAN_REQ, payload)


def encode_rf_tx(seq, target_robot_id, payload, tx_flags=0):
    raw = bytes(payload)
    if not 1 <= len(raw) <= 31:
        raise ValueError(f"RF payload length out of range: {len(raw)}")
    padded = raw.ljust(31, b"\x00")
    packed = _RF_TX_STRUCT.pack(
        int(seq) & 0xFFFF,
        int(target_robot_id) & 0xFF,
        int(tx_flags) & 0xFF,
        len(raw),
        padded,
    )
    return proto.encode_frame(BS_MSG_RF_TX, packed)


def encode_rf_tx_on_channel(seq, target_robot_id, rf_channel, payload, tx_flags=0):
    raw = bytes(payload)
    if not 1 <= len(raw) <= 31:
        raise ValueError(f"RF payload length out of range: {len(raw)}")
    padded = raw.ljust(31, b"\x00")
    packed = _RF_TX_CH_STRUCT.pack(
        int(seq) & 0xFFFF,
        int(target_robot_id) & 0xFF,
        int(tx_flags) & 0xFF,
        int(rf_channel) & 0xFF,
        len(raw),
        padded,
    )
    return proto.encode_frame(BS_MSG_RF_TX_CH, packed)


def parse_info(payload):
    if len(payload) != _INFO_STRUCT.size:
        return None
    fields = _INFO_STRUCT.unpack(payload)
    mask = fields[1]
    return {
        "protocol_version": fields[0],
        "radio_ready": bool(mask & 0x01),
        "radio_ready_mask": mask,
        "dual_radio": (mask & 0x03) == 0x03,
        "channel": fields[2],
        "rx_channel": fields[13],
        "data_rate": fields[3],
        "pa_level": fields[4],
        "retransmit_delay": fields[5],
        "retransmit_count": fields[6],
        "tx_ok_count": fields[7],
        "tx_fail_count": fields[8],
        "rx_count": fields[9],
        "radio_reinit_count": fields[10],
        "addr_suffix": fields[11],
        "base_addr_lsb": fields[12],
    }


def parse_tx_status(payload):
    if len(payload) != _TX_STATUS_STRUCT.size:
        return None
    seq, robot_id, status, retries, tx_time_us = _TX_STATUS_STRUCT.unpack(payload)
    return {
        "seq": seq,
        "target_robot_id": robot_id,
        "status": status,
        "retries": retries,
        "tx_time_us": tx_time_us,
    }


def parse_rf_rx(payload):
    if len(payload) != _RF_RX_STRUCT.size:
        return None
    rx_ts_ms, pipe, length, data = _RF_RX_STRUCT.unpack(payload)
    if not 1 <= length <= 31:
        return None
    return {
        "rx_timestamp_ms": rx_ts_ms,
        "pipe": pipe,
        "payload_len": length,
        "payload": data[:length],
    }


def parse_scan_result(payload):
    if len(payload) != _SCAN_RES_STRUCT.size:
        return None
    start_ch, count, best_ch, busy_raw, best_rx_ch = _SCAN_RES_STRUCT.unpack(payload)
    count = min(count, 126)
    return {
        "start_channel": start_ch,
        "count": count,
        "best_channel": best_ch,
        "best_rx_channel": best_rx_ch,
        "busy_pct": list(busy_raw[:count]),
    }


def parse_log(payload):
    if len(payload) != _LOG_STRUCT.size:
        return None
    level, length, text_raw = _LOG_STRUCT.unpack(payload)
    length = min(length, 96)
    return {
        "level": level,
        "text": text_raw[:length].decode("utf-8", errors="replace"),
    }


def _clear_modem_lines_atomic(ser):
    """Deassert DTR and RTS together without hitting the ESP32 reset state (DTR=0, RTS=1)."""
    if fcntl is not None and termios is not None:
        try:
            fd = ser.fileno()
            # Clear HUPCL so closing the port also avoids an unintended reset pulse.
            attrs = termios.tcgetattr(fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
            TIOCMSET = getattr(termios, "TIOCMSET", 0x5418)
            TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
            TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)
            bits = struct.unpack("I", fcntl.ioctl(fd, TIOCMGET, struct.pack("I", 0)))[0]
            bits &= ~(TIOCM_DTR | TIOCM_RTS)
            fcntl.ioctl(fd, TIOCMSET, struct.pack("I", bits))
            return
        except (OSError, ValueError, AttributeError):
            pass
    # Fallback: clear RTS before DTR so EN (DTR=0, RTS=1) is never asserted.
    try:
        ser.rts = False
        ser.dtr = False
    except (serial.SerialException, OSError):
        pass


def open_serial_no_reset(port, baud=DEFAULT_BAUD, timeout=0.01, write_timeout=0.2):
    """Open a serial port without triggering the ESP32 DevKit DTR/RTS auto-reset circuit."""
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout,
        write_timeout=write_timeout,
        rtscts=False,
        dsrdtr=False,
        exclusive=True,
    )
    _clear_modem_lines_atomic(ser)
    return ser


def probe_base_station(port, baud=DEFAULT_BAUD, timeout_s=0.75, retry_interval_s=0.15):
    """Probe one serial port with BS_MSG_PING; return BaseInfo dict or None."""
    ser = None
    try:
        ser = open_serial_no_reset(port, baud=baud, timeout=0.02, write_timeout=0.1)
        ser.reset_input_buffer()
        decoder = proto.FrameDecoder()
        ping_seq = 1
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                ser.write(encode_ping(ping_seq))
                ser.flush()
                ping_seq += 1
                next_send = now + retry_interval_s
            waiting = ser.in_waiting
            chunk = ser.read(waiting if waiting else 64)
            if chunk:
                for kind, payload in decoder.feed(chunk):
                    if kind == BS_MSG_INFO:
                        info = parse_info(payload)
                        if info is not None and info["protocol_version"] == PROTO_VERSION:
                            return info
        return None
    except (serial.SerialException, OSError, ValueError) as exc:
        if getattr(exc, "errno", None) in (errno.EBUSY, errno.EAGAIN):
            raise
        return None
    finally:
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass


class BaseStationLink:
    """Synchronous framed UART0 client for one ESP32 Base Station."""

    def __init__(self, port, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self._ser = None
        self._decoder = proto.FrameDecoder()
        self._seq = 1
        self._ping_seq = 1
        self._info_queue = collections.deque()
        self._tx_status_queue = collections.deque()
        self._rf_rx_queue = collections.deque(maxlen=64)
        self._scan_queue = collections.deque()
        self._logs = collections.deque(maxlen=32)

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def open(self):
        self._ser = open_serial_no_reset(self.port, baud=self.baud, timeout=0.005, write_timeout=0.2)
        self._ser.reset_input_buffer()
        self._decoder = proto.FrameDecoder()
        info = self.ping(timeout_s=0.75)
        if info is None:
            self.close()
            raise BaseStationError(f"ESP32 Base Station não respondeu ao handshake em {self.port}")
        return info

    def close(self):
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        self._decoder = proto.FrameDecoder()
        self._info_queue.clear()
        self._tx_status_queue.clear()
        self._rf_rx_queue.clear()
        self._scan_queue.clear()

    def _write(self, frame):
        if self._ser is None:
            raise BaseStationError("porta fechada")
        self._ser.write(frame)
        self._ser.flush()

    def _poll(self):
        if self._ser is None:
            return False
        waiting = self._ser.in_waiting
        data = self._ser.read(waiting if waiting else 1)
        if not data:
            return False
        for kind, payload in self._decoder.feed(data):
            if kind == BS_MSG_INFO:
                info = parse_info(payload)
                if info is not None:
                    self._info_queue.append(info)
            elif kind == BS_MSG_RF_TX_STATUS:
                status = parse_tx_status(payload)
                if status is not None:
                    self._tx_status_queue.append(status)
            elif kind == BS_MSG_RF_RX:
                rx = parse_rf_rx(payload)
                if rx is not None:
                    self._rf_rx_queue.append(rx)
            elif kind == BS_MSG_SCAN_RESULT:
                scan = parse_scan_result(payload)
                if scan is not None:
                    self._scan_queue.append(scan)
            elif kind == BS_MSG_LOG:
                log = parse_log(payload)
                if log is not None:
                    self._logs.append(log)
        return True

    def ping(self, timeout_s=0.40, retry_interval_s=0.15):
        self._info_queue.clear()
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                seq = self._ping_seq
                self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
                self._write(encode_ping(seq))
                next_send = now + retry_interval_s
            if self._info_queue:
                return self._info_queue.popleft()
            self._poll()
        return self._info_queue.popleft() if self._info_queue else None

    def get_info(self, timeout_s=0.25):
        self._info_queue.clear()
        self._write(encode_get_info())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._info_queue:
                return self._info_queue.popleft()
            self._poll()
        return None

    def set_config(self, **kwargs):
        timeout_s = kwargs.pop("timeout_s", 0.5)
        self._info_queue.clear()
        self._write(encode_set_config(**kwargs))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._info_queue:
                return self._info_queue.popleft()
            self._poll()
        return None

    def scan_channels(self, start_channel=0, end_channel=125, samples_per_channel=32, timeout_s=5.0):
        self._scan_queue.clear()
        self._write(encode_scan_request(start_channel, end_channel, samples_per_channel))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._scan_queue:
                return self._scan_queue.popleft()
            self._poll()
        return None

    def send_rf(self, target_robot_id, payload, tx_flags=0, timeout_s=0.05):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        self._write(encode_rf_tx(seq, target_robot_id, payload, tx_flags=tx_flags))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            while self._tx_status_queue:
                status = self._tx_status_queue.popleft()
                if status["seq"] == seq:
                    return status
            self._poll()
        return None

    def send_rf_on_channel(self, target_robot_id, rf_channel, payload, tx_flags=0, timeout_s=0.05):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        self._write(
            encode_rf_tx_on_channel(seq, target_robot_id, rf_channel, payload, tx_flags=tx_flags)
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            while self._tx_status_queue:
                status = self._tx_status_queue.popleft()
                if status["seq"] == seq:
                    return status
            self._poll()
        return None

    def clear_rx(self):
        self._rf_rx_queue.clear()

    def wait_rf_rx(self, timeout_s=0.05):
        if self._rf_rx_queue:
            return self._rf_rx_queue.popleft()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._rf_rx_queue:
                return self._rf_rx_queue.popleft()
            self._poll()
        return self._rf_rx_queue.popleft() if self._rf_rx_queue else None

    def drain_logs(self):
        logs = list(self._logs)
        self._logs.clear()
        return logs
