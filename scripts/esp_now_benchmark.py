"""Run the Robot Bench ESP-NOW stress test from the command line."""

import argparse
import sys
import threading
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_bench import communication_bench as bench  # noqa: E402
from robot_bench import radio  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--port", default=radio.ESP_LINK_PORT, help="gateway serial port")
    parser.add_argument("--baud", type=int, default=radio.ESP_LINK_BAUD, help="gateway UART baud rate")
    parser.add_argument("--robots", default="0", help="comma-separated robot IDs")
    parser.add_argument("--hz", type=int, default=60, help="scheduled cycles per second per robot")
    parser.add_argument("--duration", type=int, default=10, help="test duration in seconds")
    parser.add_argument("--mode", choices=bench.MODES, default="telemetry", help="telemetry with optional stop frames")
    parser.add_argument("--telemetry-every", type=int, default=1, help="send telemetry every N cycles")
    parser.add_argument("--timeout-ms", type=int, default=150, help="how long to count a matching reply")
    parser.add_argument("--broadcast", action="store_true", help="force broadcast downlinks for radio comparisons")
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    try:
        options = bench.Options(
            bench.parse_robot_ids(args.robots),
            args.hz,
            args.duration,
            args.mode,
            args.telemetry_every,
            args.timeout_ms,
            args.broadcast,
        )
    except ValueError as error:
        parser.error(str(error))

    run = bench.Benchmark(options)
    try:
        with radio.open_gateway(args.port, args.baud) as port:
            run.run(port, threading.Event())
    except (OSError, ValueError) as error:
        run.fail(error)
    except KeyboardInterrupt:
        return 130

    result = run.result
    route = "UART only" if options.mode == "loopback" else ("broadcast" if options.force_broadcast else "auto routing")
    print(f"ESP-NOW {args.port} · {args.baud} baud · {options.mode} · {route} · {options.rate_hz} Hz per robot")
    if result.error:
        print(result.error, file=sys.stderr)
    actual_rate = result.cycles / result.elapsed_s if result.elapsed_s else 0
    print(
        f"{result.cycles}/{result.planned_cycles} cycles at {actual_rate:.1f} Hz, {result.skipped} skipped, "
        f"{result.tx_frames} frames TX, {result.tx_bytes} B TX, {result.rx_bytes} B RX, "
        f"{result.gateway_restarts} gateway boot reports"
    )
    print(f"On-time limit: {1000 / options.rate_hz:.2f} ms")
    for robot in result.robots:
        print(
            f"Robot {robot.robot_id}: {robot.replies}/{robot.requests} replies, "
            f"{robot.timed_out} expired, {robot.on_time}/{robot.requests} on time"
        )
        if robot.median_ms is not None:
            print(
                f"  RTT median {robot.median_ms:.2f} ms, p95 {robot.p95_ms:.2f} ms, "
                f"p99 {robot.p99_ms:.2f} ms, max {robot.max_ms:.2f} ms"
            )
        if robot.max_gap_ms is not None:
            print(f"  Longest gap without telemetry {robot.max_gap_ms:.2f} ms")
    return int(result.state == "error" or not any(robot.replies for robot in result.robots))


if __name__ == "__main__":
    sys.exit(main())
