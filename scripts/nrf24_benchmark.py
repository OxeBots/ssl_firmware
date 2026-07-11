"""
nrf24_benchmark.py — NRF24L01 multi-robot round-robin benchmark.

Sends RobotCommand packets to two configurable robots in a round-robin
fashion, waits for each telemetry reply, and reports per-robot statistics.

Usage:
    python nrf24_benchmark.py                          # robots 0 & 1 (defaults)
    python nrf24_benchmark.py --robot0 2 --robot1 5
    python nrf24_benchmark.py -n 200 -t 0.1
"""

import time
import argparse
import sys
import ssl_robot_protocol_bp
import nrf24_usb


class RobotStats:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.sent = 0
        self.received = 0
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
        print(f"  Packets Sent:    {self.sent}")
        print(f"  Packets Recv:    {self.received}")
        print(f"  Success Rate:    {color}{rate:.2f}%{c.RESET}")
        if self.rtt_list:
            print(f"  Min RTT:         {min(self.rtt_list):.2f} ms")
            print(f"  Max RTT:         {max(self.rtt_list):.2f} ms")
            print(f"  Avg RTT:         {sum(self.rtt_list) / len(self.rtt_list):.2f} ms")
        if total_time_sec > 0:
            print(f"  Throughput:      {self.received / total_time_sec:.2f} packets/sec")


# After this many consecutive misses a robot is marked unreachable and its
# receive window is skipped for the rest of the benchmark run, preventing
# cascading buffer pollution that corrupts the other robot's results.
UNREACHABLE_THRESHOLD = 10


def run_benchmark(device, robot0_id, robot1_id, num_packets, timeout_sec):
    c = nrf24_usb.Colors
    print(f"\n{c.BOLD}=== STARTING ROUND-ROBIN BENCHMARK ==={c.RESET}")
    print(f"Robots:      {robot0_id}  ↔  {robot1_id}")
    print(f"Total pkts:  {num_packets}  ({num_packets // 2} per robot)")
    print(f"RX timeout:  {timeout_sec * 1000:.0f} ms\n")

    stats = {robot0_id: RobotStats(robot0_id), robot1_id: RobotStats(robot1_id)}
    consecutive = {robot0_id: 0, robot1_id: 0}  # consecutive miss counter
    unreachable = {robot0_id: False, robot1_id: False}
    robot_ids = [robot0_id, robot1_id]

    test_start = time.time()

    for i in range(num_packets):
        target_id = robot_ids[i % 2]
        stat = stats[target_id]
        stat.sent += 1

        # Use uint32-safe timestamp
        send_ms = int(time.time() * 1000) & 0xFFFFFFFF

        cmd = ssl_robot_protocol_bp.RobotCommand()
        cmd.header = nrf24_usb._make_header(nrf24_usb.MSG_TYPE_COMMAND, target_id, send_ms)
        cmd.target_pose.x = i % 1000
        cmd.target_pose.y = 0
        cmd.kick_velocity = 0

        t0 = time.perf_counter()
        device.send_data(cmd.encode())

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
            f"\r[{i + 1}/{num_packets}] → Robot {target_id}  (R{robot0_id}: {r0_label}  R{robot1_id}: {r1_label})"
        )
        sys.stdout.flush()

        if unreachable[target_id]:
            # Wait for the dongle's TX cycle to complete before the next send.
            # The NRF24L01 is half-duplex: sending the next packet before the
            # previous TX is fully settled corrupts the radio state.
            time.sleep(timeout_sec)
            # Clear any unexpected bytes (e.g. dongle status code) so they
            # don't pollute the next receive window.
            device.ser.reset_input_buffer()
            continue

        telemetry = device.receive_data(timeout=timeout_sec)
        t1 = time.perf_counter()

        # Always flush after a receive window — clears any partial bytes or
        # late-arriving dongle status codes before the next send.
        device.ser.reset_input_buffer()

        if telemetry is not None and telemetry.header.timestamp == send_ms and telemetry.header.robot_id == target_id:
            stat.record_success((t1 - t0) * 1000)
            consecutive[target_id] = 0
        else:
            consecutive[target_id] += 1
            if not unreachable[target_id] and consecutive[target_id] >= UNREACHABLE_THRESHOLD:
                unreachable[target_id] = True
                print(
                    f"\n{c.YELLOW}[WARN] Robot {target_id} marked UNREACHABLE "
                    f"after {UNREACHABLE_THRESHOLD} consecutive misses. "
                    f"Skipping its receive window for remaining packets.{c.RESET}"
                )

    test_end = time.time()
    total_time = test_end - test_start

    # Print final report
    c = nrf24_usb.Colors
    print("\n\n" + "=" * 50)
    print(f"{c.BOLD}BENCHMARK RESULTS{c.RESET}")
    print("=" * 50)
    print(f"Total time:  {total_time:.2f} s")
    print(f"Total sent:  {num_packets}  ({num_packets // 2} per robot)")

    combined_recv = sum(s.received for s in stats.values())
    combined_rate = combined_recv / num_packets * 100
    oc = c.GREEN if combined_rate >= 95 else (c.YELLOW if combined_rate >= 80 else c.RED)
    print(f"Overall:     {oc}{combined_recv}/{num_packets}  ({combined_rate:.2f}%){c.RESET}")

    print(f"\n{'─' * 50}")
    print(f"{c.BOLD}Per-Robot Breakdown{c.RESET}")
    print(f"{'─' * 50}")
    for rid in [robot0_id, robot1_id]:
        stats[rid].print_report(total_time)

    print("=" * 50 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="NRF24L01 Multi-Robot Round-Robin Benchmark",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", type=str, default="auto", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "-n",
        "--packets",
        type=int,
        default=500,
        help="Total packets (split evenly between the two robots)",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=0.050,
        help="RX timeout per packet in seconds",
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

    run_benchmark(device, args.robot0, args.robot1, args.packets, args.timeout)
    device.close()


if __name__ == "__main__":
    main()
