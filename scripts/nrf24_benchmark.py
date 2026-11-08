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

import argparse
import glob
import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
for _pio_sp in glob.glob(os.path.expanduser("~/.platformio/penv/lib/python*/site-packages")):
    if _pio_sp not in sys.path:
        sys.path.append(_pio_sp)

import ssl_robot_protocol_bp
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
# telemetry requests are throttled to 1 Hz (hot-plug probe), preventing
# 35-50ms timeouts on every cycle while still automatically recovering as
# soon as the robot is powered on!
UNREACHABLE_THRESHOLD = 5
OFFLINE_PROBE_INTERVAL_SEC = 1.0


def run_benchmark(device, robot0_id, robot1_id, num_packets, timeout_sec, tel_interval=5, cmd_guard_sec=0.015):
    c = nrf24_usb.Colors
    unique_ids = [robot0_id] if robot0_id == robot1_id else [robot0_id, robot1_id]
    print(f"\n{c.BOLD}=== STARTING ROUND-ROBIN BENCHMARK (CMD + TEL_REQ) ==={c.RESET}")
    print(f"Robots:        {'  ↔  '.join(str(r) for r in unique_ids)}")
    print(f"Total cmds:    {num_packets}  ({num_packets // len(unique_ids)} per robot)")
    print(f"Tel interval:  every {tel_interval} command(s) per robot (1 Hz hot-plug probe when offline)")
    print(f"Cmd TX sync:   Dongle 6B ACK wait (timeout {cmd_guard_sec * 1000:.1f} ms)")
    print(f"RX timeout:    {timeout_sec * 1000:.0f} ms\n")

    stats = {rid: RobotStats(rid) for rid in unique_ids}
    consecutive = {rid: 0 for rid in unique_ids}  # consecutive telemetry miss counter
    unreachable = {rid: False for rid in unique_ids}
    last_probe_time = {rid: 0.0 for rid in unique_ids}
    robot_ids = unique_ids

    test_start = time.time()

    for i in range(num_packets):
        target_id = robot_ids[i % len(robot_ids)]
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
        # Wait for the USB dongle to finish RF TX and emit its 6-byte status (02 00 00 00 00 00)
        device.wait_dongle_tx_complete(timeout=max(0.018, cmd_guard_sec), settle_sec=0.0015)

        # Check if we should request telemetry on this cycle for target_id:
        # - Online robots: every `tel_interval` commands
        # - Offline (UNREACHABLE) robots: probed once every 1.0s (1 Hz) for hot-plug recovery
        now_perf = time.perf_counter()
        if tel_interval <= 0:
            should_request_tel = False
        elif not unreachable[target_id]:
            should_request_tel = (stat.cmds_sent % tel_interval == 0)
        else:
            should_request_tel = ((now_perf - last_probe_time[target_id]) >= OFFLINE_PROBE_INTERVAL_SEC)

        status_labels = []
        for rid in unique_ids:
            lbl = (
                f"{c.RED}OFFLINE(1Hz){c.RESET}"
                if unreachable[rid]
                else f"{stats[rid].received}/{stats[rid].sent}"
            )
            status_labels.append(f"R{rid}: {lbl}")
        sys.stdout.write(
            f"\r[{i + 1}/{num_packets}] → Robot {target_id}  (Tel {'  '.join(status_labels)})"
        )
        sys.stdout.flush()

        if not should_request_tel:
            continue

        # 2. Send explicit RobotTelemetryRequest (MSG_TYPE_TELEMETRY_REQ) and await reply
        was_unreachable = unreachable[target_id]
        last_probe_time[target_id] = now_perf
        req_ms = int(time.time() * 1000) & 0xFFFFFFFF
        req_bytes = encode_telemetry_request(target_id, req_ms)

        t0 = time.perf_counter()
        device.send_data(req_bytes)

        telemetry = device.receive_data(timeout=timeout_sec)
        t1 = time.perf_counter()

        # Always flush after a receive window — clears any partial bytes or
        # late-arriving dongle status codes before the next send.
        device.ser.reset_input_buffer()

        if telemetry is not None and telemetry.header.robot_id == target_id:
            stat.sent += 1
            stat.record_success((t1 - t0) * 1000)
            consecutive[target_id] = 0
            unreachable[target_id] = False
            if was_unreachable:
                print(
                    f"\n{c.GREEN}✓ [HOT-PLUG] Robot {target_id} is now ONLINE! "
                    f"Resuming normal telemetry requests every {tel_interval} commands.{c.RESET}"
                )
            time.sleep(0.0015)
        else:
            if not was_unreachable:
                stat.sent += 1
                consecutive[target_id] += 1
                if consecutive[target_id] >= UNREACHABLE_THRESHOLD:
                    unreachable[target_id] = True
                    print(
                        f"\n{c.YELLOW}[WARN] Robot {target_id} marked OFFLINE "
                        f"after {UNREACHABLE_THRESHOLD} consecutive misses. "
                        f"Probing at 1 Hz until it comes online.{c.RESET}"
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
    print(f"Total cmds sent: {num_packets}  ({num_packets // len(unique_ids)} per robot)")
    print(f"Total tel reqs:  {total_tel_sent}")
    if total_time > 0:
        print(f"Total cmd rate:  {num_packets / total_time:.2f} cmds/sec")

    oc = c.GREEN if combined_rate >= 95 else (c.YELLOW if combined_rate >= 80 else c.RED)
    print(f"Tel Overall:     {oc}{combined_recv}/{total_tel_sent}  ({combined_rate:.2f}%){c.RESET}")

    print(f"\n{'─' * 50}")
    print(f"{c.BOLD}Per-Robot Breakdown{c.RESET}")
    print(f"{'─' * 50}")
    for rid in unique_ids:
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
        help="Total command packets",
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
        default=5,
        help="Request telemetry every N commands per robot (0 = commands only, no telemetry)",
    )
    parser.add_argument(
        "--cmd-guard",
        type=float,
        default=0.018,
        help="Max timeout in seconds waiting for dongle TX-complete status after RobotCommand",
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
    parser.add_argument("--robot1", type=int, default=1, help="Second robot ID (0-10, set same as --robot0 for single robot)")
    parser.add_argument("--skip-config", action="store_true", help="Skip AT configuration of the USB dongle")
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

    # Ensure even split when testing 2 robots
    if args.robot0 != args.robot1 and args.packets % 2 != 0:
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

    if not args.skip_config:
        print(f"\n{nrf24_usb.Colors.CYAN}Applying Benchmark Configurations...{nrf24_usb.Colors.RESET}")
        rx_address_bytes = [ord(c) for c in "ADMIN"]
        tx_address_bytes = [ord(c) for c in "ESP32"]
        device.set_addresses(rx_address_bytes, tx_address_bytes)
        device.set_frequency(2.476)
        device.set_data_rate(args.rate)
        time.sleep(0.5)
        device.ser.reset_input_buffer()

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
