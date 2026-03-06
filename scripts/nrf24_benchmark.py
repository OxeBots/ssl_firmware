import time
import argparse
import sys
import importlib.util
import ssl_robot_protocol_bp

# Dynamically import your existing nrf24-usb.py despite the hyphen in the filename
try:
    spec = importlib.util.spec_from_file_location("nrf24_module", "nrf24-usb.py")
    nrf = importlib.util.module_from_spec(spec)
    sys.modules["nrf24_module"] = nrf
    spec.loader.exec_module(nrf)
except FileNotFoundError:
    print("Error: Ensure nrf24-usb.py is in the same directory as this script.")
    sys.exit(1)


def run_benchmark(device, num_packets, timeout_sec):
    print(f"\n{nrf.Colors.BOLD}=== STARTING BENCHMARK ==={nrf.Colors.RESET}")
    print(
        f"Sending {num_packets} packets with a {timeout_sec * 1000:.0f}ms RX timeout...\n"
    )

    success_count = 0
    rtt_list = []

    # Silence the continuous print statements from the original receive_data method
    # by temporarily suppressing stdout if desired, but we will rely on capturing the output.

    test_start_time = time.time()

    for i in range(num_packets):
        cmd = ssl_robot_protocol_bp.RobotCommand()
        send_ms = int(time.time() * 1000) % (2**31 - 1)
        cmd.timestamp = send_ms
        cmd.target_pose.x = i % 1000
        cmd.target_pose.y = 0
        cmd.kick_velocity = 0

        encoded = cmd.encode()

        t0 = time.perf_counter()
        device.send_data(encoded)

        # Override the original print function inside the loop to avoid terminal spam
        # You can monitor progress with a simple counter
        sys.stdout.write(f"\rProgress: {i + 1}/{num_packets}")
        sys.stdout.flush()

        telemetry = device.receive_data(timeout=timeout_sec)
        t1 = time.perf_counter()

        if telemetry and telemetry.timestamp == send_ms:
            success_count += 1
            rtt_list.append((t1 - t0) * 1000)  # Convert to ms

    test_end_time = time.time()
    total_time = test_end_time - test_start_time

    print("\n\n" + "=" * 40)
    print(f"{nrf.Colors.BOLD}BENCHMARK RESULTS{nrf.Colors.RESET}")
    print("=" * 40)

    success_rate = (success_count / num_packets) * 100
    print(f"Packets Sent:      {num_packets}")
    print(f"Packets Received:  {success_count}")

    if success_rate >= 95:
        color = nrf.Colors.GREEN
    elif success_rate >= 80:
        color = nrf.Colors.YELLOW
    else:
        color = nrf.Colors.RED

    print(f"Success Rate:      {color}{success_rate:.2f}%{nrf.Colors.RESET}")

    if success_count > 0:
        print(f"Min Latency (RTT): {min(rtt_list):.2f} ms")
        print(f"Max Latency (RTT): {max(rtt_list):.2f} ms")
        print(f"Avg Latency (RTT): {sum(rtt_list) / len(rtt_list):.2f} ms")

    print(f"Total Time Taken:  {total_time:.2f} seconds")
    print(f"Throughput:        {success_count / total_time:.2f} packets/sec")
    print("=" * 40 + "\n")


def main():
    parser = argparse.ArgumentParser(description="NRF24L01 USB Adapter Benchmark")
    parser.add_argument("-p", "--port", type=str, default="auto", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "-n", "--packets", type=int, default=500, help="Number of packets to test"
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

    args = parser.parse_args()

    if args.port.lower() == "auto":
        device = nrf.nRF24L01_Controller.auto_connect(baudrate=args.baud)
    else:
        device = nrf.nRF24L01_Controller(port=args.port, baudrate=args.baud)

    if not device:
        sys.exit(1)

    # Configure Dongle
    print(f"\n{nrf.Colors.CYAN}Applying Benchmark Configurations...{nrf.Colors.RESET}")
    device.set_data_rate(args.rate)

    time.sleep(1)  # Let adapter settle
    run_benchmark(device, args.packets, args.timeout)
    device.close()


if __name__ == "__main__":
    main()
