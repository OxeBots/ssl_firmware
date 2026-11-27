"""Fixed-rate ESP-NOW gateway benchmark shared by the dashboard and CLI."""

import itertools
import math
import select
import statistics
import struct
import threading
import time
from dataclasses import dataclass, replace

import serial

from . import radio

MODES = ("telemetry", "unicast", "team", "loopback")
MAX_RATE_HZ = 500
MAX_DURATION_S = 120
MAX_TIMEOUT_MS = 2000
MAX_TELEMETRY_INTERVAL = 100
TEAM_MESSAGE_TYPE = 4
TEAM_MAX_ROBOTS = 3


def parse_robot_ids(text):
    try:
        return tuple(int(value.strip()) for value in text.split(","))
    except ValueError as error:
        raise ValueError("Informe IDs separados por vírgulas.") from error


@dataclass(frozen=True)
class Options:
    robot_ids: tuple[int, ...] = (0,)
    rate_hz: int = 60
    duration_s: int = 10
    mode: str = "telemetry"
    telemetry_every: int = 1
    timeout_ms: int = 150
    force_broadcast: bool = False

    def __post_init__(self):
        if not self.robot_ids or any(not radio.robot_id_valid(robot_id) for robot_id in self.robot_ids):
            raise ValueError(f"Os IDs devem estar entre 0 e {radio.ROBOT_ID_MAX}.")
        if len(set(self.robot_ids)) != len(self.robot_ids):
            raise ValueError("Informe cada robô uma vez.")
        if not 1 <= self.rate_hz <= MAX_RATE_HZ or not 1 <= self.duration_s <= MAX_DURATION_S:
            raise ValueError(f"Use 1–{MAX_RATE_HZ} Hz e 1–{MAX_DURATION_S} s.")
        if self.mode not in MODES:
            raise ValueError("Modo de tráfego inválido.")
        if not 1 <= self.telemetry_every <= MAX_TELEMETRY_INTERVAL or not 1 <= self.timeout_ms <= MAX_TIMEOUT_MS:
            raise ValueError(
                f"Use telemetria a cada 1–{MAX_TELEMETRY_INTERVAL} ciclos e prazo de 1–{MAX_TIMEOUT_MS} ms."
            )


@dataclass(frozen=True)
class RobotResult:
    robot_id: int
    requests: int = 0
    replies: int = 0
    timed_out: int = 0
    on_time: int = 0
    median_ms: float | None = None
    p95_ms: float | None = None
    p99_ms: float | None = None
    max_ms: float | None = None
    max_gap_ms: float | None = None


@dataclass(frozen=True)
class Result:
    planned_cycles: int
    robots: tuple[RobotResult, ...]
    state: str = "waiting"
    cycles: int = 0
    skipped: int = 0
    elapsed_s: float = 0.0
    tx_frames: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    gateway_restarts: int = 0
    error: str = ""


def encode_team_stop(robot_ids, timestamp):
    """Pack a team velocity message with zero motion and kick in each slot."""
    header = struct.pack("<BI", (len(robot_ids) << 4) | TEAM_MESSAGE_TYPE, timestamp)
    return header + b"".join(struct.pack("<B5x", robot_id) for robot_id in robot_ids)


class Benchmark:
    """A cancellable benchmark. The caller owns the gateway serial port."""

    def __init__(self, options):
        self.options = options
        self._cancel = threading.Event()
        self.result = Result(options.rate_hz * options.duration_s, tuple(RobotResult(i) for i in options.robot_ids))

    def cancel(self):
        self._cancel.set()
        if self.result.state == "waiting":
            self.result = replace(self.result, state="cancelled")

    def fail(self, error):
        self.result = replace(self.result, state="error", error=str(error))

    def run(self, port, closing):
        """Send at the requested cadence and match replies by robot ID and timestamp."""

        def cancelled():
            return self._cancel.is_set() or closing.is_set()

        try:
            if not radio.wait_for_gateway(port, cancelled):
                self.cancel()
                return
        except (radio.RadioError, serial.SerialTimeoutException) as error:
            self.fail(error)
            return
        except (OSError, ValueError) as error:
            self.fail(error)
            raise

        options = self.options
        period = 1.0 / options.rate_hz
        timeout = options.timeout_ms / 1000
        reply_type = radio.ESP_LINK_LOOPBACK_TYPE if options.mode == "loopback" else radio.ESP_LINK_FRAME_TYPE
        frame_type = (
            radio.ESP_LINK_BROADCAST_TYPE if options.force_broadcast and options.mode != "loopback" else reply_type
        )
        decoder = radio.TelemetryStream(reply_type)
        requests = dict.fromkeys(options.robot_ids, 0)
        misses = dict.fromkeys(options.robot_ids, 0)
        on_time = dict.fromkeys(options.robot_ids, 0)
        latencies = {robot_id: [] for robot_id in options.robot_ids}
        first_request = {}
        last_reply = {}
        longest_gap = dict.fromkeys(options.robot_ids, 0.0)
        pending = {}
        # Increment timestamps even when several requests leave in the same millisecond.
        stamps = itertools.count(time.time_ns() // 1_000_000 & 0xFFFFFFFF)
        start = time.perf_counter()
        next_snapshot = start
        cycles = skipped = tx_frames = tx_bytes = rx_bytes = gateway_restarts = 0
        finished_tx = None

        def expire(now):
            while pending:
                stamp = next(iter(pending))
                robot_id, sent = pending[stamp]
                if now < sent + timeout:
                    break
                misses[robot_id] += 1
                del pending[stamp]

        def read_until(deadline):
            nonlocal rx_bytes, gateway_restarts
            while not cancelled():
                wait = max(0, deadline - time.perf_counter())
                ready, _, _ = select.select([port], [], [], min(wait, 0.02))
                if not ready:
                    if not wait:
                        break
                    continue
                data = port.read(port.in_waiting or 1)
                received = time.perf_counter()
                rx_bytes += len(data)
                expire(received)
                for telemetry in decoder.feed(data):
                    stamp = telemetry["timestamp"]
                    robot_id = telemetry["robot_id"]
                    if stamp not in pending or pending[stamp][0] != robot_id:
                        continue
                    _, sent = pending.pop(stamp)
                    rtt = received - sent
                    latencies[robot_id].append(rtt * 1000)
                    on_time[robot_id] += int(rtt <= period)
                    previous = last_reply.get(robot_id, first_request[robot_id])
                    longest_gap[robot_id] = max(longest_gap[robot_id], received - previous)
                    last_reply[robot_id] = received
                gateway_restarts += decoder.drain_lines().count(radio.GATEWAY_READY_LINE)
                if not wait:
                    break
            expire(time.perf_counter())

        def write(payload, robot_id=None, stamp=None):
            nonlocal tx_frames, tx_bytes
            frame = radio.encode_link_frame(payload, frame_type)
            sent = time.perf_counter()
            if port.write(frame) != len(frame):
                raise serial.SerialTimeoutException("Escrita UART incompleta")
            tx_frames += 1
            tx_bytes += len(frame)
            if robot_id is not None:
                requests[robot_id] += 1
                first_request.setdefault(robot_id, sent)
                pending[stamp] = robot_id, sent

        def publish(state, error=""):
            robots = []
            observed_until = finished_tx or time.perf_counter()
            for robot_id, samples in latencies.items():
                ordered = sorted(samples)
                count = len(ordered)
                gap_ms = None
                if robot_id in first_request:
                    last = last_reply.get(robot_id, first_request[robot_id])
                    gap_ms = max(longest_gap[robot_id], observed_until - last) * 1000
                robots.append(
                    RobotResult(
                        robot_id,
                        requests[robot_id],
                        count,
                        misses[robot_id],
                        on_time[robot_id],
                        statistics.median(ordered) if count else None,
                        ordered[math.ceil(0.95 * count) - 1] if count else None,
                        ordered[math.ceil(0.99 * count) - 1] if count else None,
                        ordered[-1] if count else None,
                        gap_ms,
                    )
                )
            self.result = Result(
                self.result.planned_cycles,
                tuple(robots),
                state,
                cycles,
                skipped,
                observed_until - start,
                tx_frames,
                tx_bytes,
                rx_bytes,
                gateway_restarts,
                error,
            )

        try:
            publish("running")
            for index in range(self.result.planned_cycles):
                read_until(start + index * period)
                if cancelled():
                    break
                now = time.perf_counter()
                if now >= start + (index + 1) * period:
                    # A stalled host should not replay missed cycles in a burst.
                    skipped += 1
                    continue
                for payload in self._commands(next(stamps) & 0xFFFFFFFF):
                    if cancelled():
                        break
                    write(payload)
                if index % options.telemetry_every == 0:
                    for robot_id in options.robot_ids:
                        if cancelled():
                            break
                        stamp = next(stamps) & 0xFFFFFFFF
                        write(radio.encode_telemetry_request(robot_id, stamp), robot_id, stamp)
                cycles += 1
                if now >= next_snapshot:
                    publish("running")
                    next_snapshot = now + 0.25

            read_until(start + self.result.planned_cycles * period)
            finished_tx = time.perf_counter()
            if pending:
                read_until(next(reversed(pending.values()))[1] + timeout)
            publish("cancelled" if cancelled() else "complete")
        except serial.SerialTimeoutException as error:
            publish("error", str(error))
        except (OSError, ValueError) as error:
            publish("error", str(error))
            raise

    def _commands(self, timestamp):
        options = self.options
        if options.mode == "unicast":
            for robot_id in options.robot_ids:
                yield radio.encode_command(robot_id, timestamp=timestamp)
        elif options.mode == "team":
            for index in range(0, len(options.robot_ids), TEAM_MAX_ROBOTS):
                yield encode_team_stop(options.robot_ids[index : index + TEAM_MAX_ROBOTS], timestamp)
