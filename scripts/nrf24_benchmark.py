"""
nrf24_benchmark.py — NRF24L01 multi-robot round-robin benchmark.

Sends fire-and-forget RobotCommand packets to two configurable robots in a
round-robin fashion, followed by explicit RobotTelemetryRequest packets every
N commands (--tel-interval) to measure telemetry RTT and packet delivery rate.

Usage:
    python scripts/nrf24_benchmark.py                          # robots 0 & 1, tel every 1 cmd
    python scripts/nrf24_benchmark.py --robot0 2 --robot1 5
    python scripts/nrf24_benchmark.py -n 200 -t 0.025 --tel-interval 5 --cmd-guard 0.004
"""

import time
import argparse
import sys

try:
    import ssl_robot_protocol_bp
except ImportError:
    import scripts.ssl_robot_protocol_bp as ssl_robot_protocol_bp

import nrf24_usb

MSG_TYPE_TELEMETRY_REQ = getattr(ssl_robot_protocol_bp, "MSG_TYPE_TELEMETRY_REQ", 3)


def encode_telemetry_request(robot_id: int, timestamp_ms: int) -> bytearray:
    """
    Encode a 5-byte RobotTelemetryRequest packet.
    Uses ssl_robot_protocol_bp.RobotTelemetryRequest if regenerated, or falls
    back to encoding the 5-byte Header directly (identical 40-bit wire layout).
    """
    hdr = nrf24_usb._make_header(MSG_TYPE_TELEMETRY_REQ, robot_id, timestamp_ms)
    if hasattr(ssl_robot_protocol_bp, "RobotTelemetryRequest"):
        req = ssl_robot_protocol_bp.RobotTelemetryRequest()
        req.header = hdr
        return req.encode()
    return hdr.encode()


class RobotStats:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.cmds_sent = 0
        self.sent = 0       # telemetry requests sent
        self.received = 0   # telemetry replies received
        self.rtt_list = []

    def record_success(self, rtt_ms):
        self.received += 1
        self.rtt_list.append(rtt_ms)

    def success_rate(self):
        return (self.received / self.sent * 100) if self.sent > 0 else 0.0

    def print_report(self, total_time_sec):
        c = nrf24_usb.Colors
        rate = self.success_rate()
        color = c.GREEN if rate >= 95 else (c.YELLOW if rate >= 80 else c.RED)

        print(f"\n  Robot ID:        {self.robot_id}")
        print(f"  Commands Sent:   {self.cmds_sent} (fire-and-forget)")
        print(f"  Tel Reqs Sent:   {self.sent}")
        print(f"  Tel Replies Rx:  {self.received}")
        print(f"  Tel Success:     {color}{rate:.2f}%{c.RESET}")
        if self.rtt_list:
            print(f"  Min Tel RTT:     {min(self.rtt_list):.2f} ms")
            print(f"  Max Tel RTT:     {max(self.rtt_list):.2f} ms")
            print(f"  Avg Tel RTT:     {sum(self.rtt_list) / len(self.rtt_list):.2f} ms")
        if total_time_sec > 0:
            print(f"  Cmd Frequency:   {self.cmds_sent / total_time_sec:.2f} cmds/sec")
            print(f"  Tel Throughput:  {self.received / total_time_sec:.2f} replies/sec")


# After this many consecutive misses a robot is marked unreachable and its
# receive window is skipped for the rest of the benchmark run, preventing
# cascading buffer pollution that corrupts the other robot's results.
UNREACHABLE_THRESHOLD = 10


def run_benchmark(device, robot0_id, robot1_id, num_packets, timeout_sec, tel_interval=1, cmd_guard_sec=0.004):
    c = nrf24_usb.Colors
    print(f"\n{c.BOLD}=== STARTING ROUND-ROBIN BENCHMARK (CMD + TEL_REQ) ==={c.RESET}")
    print(f"Robots:        {robot0_id}  ↔  {robot1_id}")
    print(f"Total cmds:    {num_packets}  ({num_packets // 2} per robot)")
    print(f"Tel interval:  every {tel_interval} command(s) per robot")
    print(f"Cmd guard:     {cmd_guard_sec * 1000:.1f} ms")
    print(f"RX timeout:    {timeout_sec * 1000:.0f} ms\n")

    stats = {robot0_id: RobotStats(robot0_id), robot1_id: RobotStats(robot1_id)}
    consecutive = {robot0_id: 0, robot1_id: 0}  # consecutive telemetry miss counter
    unreachable = {robot0_id: False, robot1_id: False}
    robot_ids = [robot0_id, robot1_id]

    test_start = time.time()

    for i in range(num_packets):
        target_id = robot_ids[i % 2]
        stat = stats[target_id]
        stat.cmds_sent += 1

        # 1. Send fire-and-forget RobotCommand (no telemetry reply expected)
        cmd_ms = int(time.time() * 1000) & 0xFFFFFFFF
        cmd = ssl_robot_protocol_bp.RobotCommand()
        cmd.header = nrf24_usb._make_header(nrf24_usb.MSG_TYPE_COMMAND, target_id, cmd_ms)
        cmd.target_pose.x = i % 1000
        cmd.target_pose.y = 0
        cmd.kick_velocity = 0

        device.send_data(cmd.encode())
        time.sleep(cmd_guard_sec)
        device.ser.reset_input_buffer()

        # Check if we should request telemetry on this cycle for target_id
        should_request_tel = (tel_interval > 0) and (stat.cmds_sent % tel_interval == 0)

        r0_label = (
            f"{c.RED}UNREACHABLE{c.RESET}"
            if unreachable[robot0_id]
            else f"{stats[robot0_id].received}/{stats[robot0_id].sent}"
        )
        r1_label = (
            f"{c.RED}UNREACHABLE{c.RESET}"
            if unreachable[robot1_id]
            else f"{stats[robot1_id].received}/{stats[robot1_id].sent}"
        )
        sys.stdout.write(
            f"\r[{i + 1}/{num_packets}] → Robot {target_id}  (Tel R{robot0_id}: {r0_label}  R{robot1_id}: {r1_label})"
        )
        sys.stdout.flush()

        if not should_request_tel:
            continue

        if unreachable[target_id]:
            continue

        # 2. Send explicit RobotTelemetryRequest (MSG_TYPE_TELEMETRY_REQ) and await reply
        stat.sent += 1
        req_ms = int(time.time() * 1000) & 0xFFFFFFFF
        req_bytes = encode_telemetry_request(target_id, req_ms)

        t0 = time.perf_counter()
        device.send_data(req_bytes)

        telemetry = device.receive_data(timeout=timeout_sec)
        t1 = time.perf_counter()

        # Always flush after a receive window — clears any partial bytes or
        # late-arriving dongle status codes before the next send.
        device.ser.reset_input_buffer()

        if telemetry is not None and telemetry.header.timestamp == req_ms and telemetry.header.robot_id == target_id:
            stat.record_success((t1 - t0) * 1000)
            consecutive[target_id] = 0
            time.sleep(cmd_guard_sec)
        else:
            consecutive[target_id] += 1
            if not unreachable[target_id] and consecutive[target_id] >= UNREACHABLE_THRESHOLD:
                unreachable[target_id] = True
                print(
                    f"\n{c.YELLOW}[WARN] Robot {target_id} marked UNREACHABLE "
                    f"after {UNREACHABLE_THRESHOLD} consecutive telemetry misses. "
                    f"Skipping its telemetry requests for remaining packets.{c.RESET}"
                )

    test_end = time.time()
    total_time = test_end - test_start

    # Print final report
    c = nrf24_usb.Colors
    total_tel_sent = sum(s.sent for s in stats.values())
    combined_recv = sum(s.received for s in stats.values())
    combined_rate = (combined_recv / total_tel_sent * 100) if total_tel_sent > 0 else 0.0

    print("\n\n" + "=" * 50)
    print(f"{c.BOLD}BENCHMARK RESULTS{c.RESET}")
    print("=" * 50)
    print(f"Total time:      {total_time:.2f} s")
    print(f"Total cmds sent: {num_packets}  ({num_packets // 2} per robot)")
    print(f"Total tel reqs:  {total_tel_sent}")
    if total_time > 0:
        print(f"Total cmd rate:  {num_packets / total_time:.2f} cmds/sec")

    oc = c.GREEN if combined_rate >= 95 else (c.YELLOW if combined_rate >= 80 else c.RED)
    print(f"Tel Overall:     {oc}{combined_recv}/{total_tel_sent}  ({combined_rate:.2f}%){c.RESET}")

    print(f"\n{'─' * 50}")
    print(f"{c.BOLD}Per-Robot Breakdown{c.RESET}")
    print(f"{'─' * 50}")
    for rid in [robot0_id, robot1_id]:
        stats[rid].print_report(total_time)

    print("=" * 50 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="NRF24L01 Multi-Robot Round-Robin Benchmark (RobotCommand + RobotTelemetryRequest)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", type=str, default="auto", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "-n",
        "--packets",
        type=int,
        default=500,
        help="Total command packets (split evenly between the two robots)",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=0.050,
        help="RX timeout per telemetry request in seconds",
    )
    parser.add_argument(
        "--tel-interval",
        type=int,
        default=1,
        help="Request telemetry every N commands per robot (0 = commands only, no telemetry)",
    )
    parser.add_argument(
        "--cmd-guard",
        type=float,
        default=0.004,
        help="Guard time in seconds after sending a fire-and-forget RobotCommand",
    )
    parser.add_argument(
        "-r",
        "--rate",
        type=int,
        default=3,
        choices=[1, 2, 3],
        help="1=250K, 2=1M, 3=2M",
    )
    parser.add_argument("--robot0", type=int, default=0, help="First robot ID  (0-10)")
    parser.add_argument("--robot1", type=int, default=1, help="Second robot ID (0-10)")
    args = parser.parse_args()

    # Validate robot IDs
    for rid, label in [(args.robot0, "--robot0"), (args.robot1, "--robot1")]:
        if rid > nrf24_usb.ROBOT_ID_MAX or rid == nrf24_usb.ROBOT_ID_BROADCAST:
            print(
                f"Error: {label}={rid} is out of range. "
                f"Valid IDs are 0–{nrf24_usb.ROBOT_ID_MAX} "
                f"(broadcast {nrf24_usb.ROBOT_ID_BROADCAST} is reserved)."
            )
            sys.exit(1)

    if args.robot0 == args.robot1:
        print("Error: --robot0 and --robot1 must be different robot IDs.")
        sys.exit(1)

    # Ensure even split
    if args.packets % 2 != 0:
        args.packets += 1
        print(f"{nrf24_usb.Colors.YELLOW}Note: packet count rounded up to {args.packets}.{nrf24_usb.Colors.RESET}")

    # Connect
    device = (
        nrf24_usb.nRF24L01_Controller.auto_connect(baudrate=args.baud)
        if args.port.lower() == "auto"
        else nrf24_usb.nRF24L01_Controller(port=args.port, baudrate=args.baud)
    )

    if not device:
        sys.exit(1)

    print(f"\n{nrf24_usb.Colors.CYAN}Applying Benchmark Configurations...{nrf24_usb.Colors.RESET}")
    device.set_data_rate(args.rate)
    time.sleep(1)

    run_benchmark(
        device,
        args.robot0,
        args.robot1,
        args.packets,
        args.timeout,
        tel_interval=args.tel_interval,
        cmd_guard_sec=args.cmd_guard,
    )
    device.close()


if __name__ == "__main__":
    main()
