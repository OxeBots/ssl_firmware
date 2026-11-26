"""Multi-robot round-robin benchmark via the ESP32 Base Station.

Measures real hardware TX Auto-ACK rate, average RF retries, per-packet RF TX
latency (us), and unicast Telemetry RTT (ms) across one or more SSL robots.
"""

import argparse
import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from robot_bench import base_station, radio  # noqa: E402
from scripts import nrf24_usb  # noqa: E402


class RobotStats:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.cmds_sent = 0
        self.cmd_ack_ok = 0
        self.retries_sum = 0
        self.tx_us_sum = 0.0
        self.tel_sent = 0
        self.tel_rx = 0
        self.rtt_list = []

    def print_report(self, elapsed_s):
        c = nrf24_usb.Colors
        ack_rate = (100.0 * self.cmd_ack_ok / self.cmds_sent) if self.cmds_sent else 0.0
        tel_rate = (100.0 * self.tel_rx / self.tel_sent) if self.tel_sent else 0.0
        avg_ret = (self.retries_sum / self.cmd_ack_ok) if self.cmd_ack_ok else 0.0
        avg_tx = (self.tx_us_sum / self.cmd_ack_ok) if self.cmd_ack_ok else 0.0
        color = c.GREEN if ack_rate >= 95 else (c.YELLOW if ack_rate >= 80 else c.RED)

        print(f"\n  Robot ID:        {self.robot_id}")
        print(f"  Commands Sent:   {self.cmds_sent} ({self.cmds_sent / max(elapsed_s, 1e-6):.1f} Hz)")
        print(f"  HW TX Auto-ACK:  {color}{ack_rate:.2f}%{c.RESET} (avg retries={avg_ret:.2f}, avg tx={avg_tx:.0f} us)")
        print(f"  Telemetry RX:    {self.tel_rx}/{self.tel_sent} ({tel_rate:.2f}%)")
        if self.rtt_list:
            print(
                f"  Telemetry RTT:   avg={sum(self.rtt_list)/len(self.rtt_list):.2f} ms "
                f"(min={min(self.rtt_list):.2f}, max={max(self.rtt_list):.2f})"
            )


def main():
    parser = argparse.ArgumentParser(description="Benchmark multi-robô na ESP32 Base Station")
    parser.add_argument("-p", "--port", default=None)
    parser.add_argument("-b", "--baud", type=int, default=base_station.DEFAULT_BAUD)
    parser.add_argument("--robots", default="0,1,2", help="IDs separados por vírgula (ex: 0 ou 0,1,2)")
    parser.add_argument("-d", "--duration", type=float, default=10.0, help="Duração em segundos")
    parser.add_argument("--tel-interval", type=int, default=5, help="Telemetria a cada N cmds (0=off)")
    args = parser.parse_args()

    robot_ids = [int(tok.strip()) for tok in args.robots.split(",") if tok.strip()]
    link, info = nrf24_usb.connect_base(args.port, args.baud)
    nrf24_usb.print_info(info, link.port, link.baud)

    stats = {rid: RobotStats(rid) for rid in robot_ids}
    start = time.monotonic()
    try:
        while (time.monotonic() - start) < args.duration:
            for idx, rid in enumerate(robot_ids):
                st = stats[rid]
                cmd = radio.encode_command(rid)
                tx_st = link.send_rf(rid, cmd, timeout_s=0.025)
                st.cmds_sent += 1
                if tx_st is not None and tx_st["status"] == base_station.BS_TX_STATUS_OK:
                    st.cmd_ack_ok += 1
                    st.retries_sum += tx_st["retries"]
                    st.tx_us_sum += tx_st["tx_time_us"]

                if args.tel_interval > 0 and ((st.cmds_sent + idx) % args.tel_interval == 0):
                    st.tel_sent += 1
                    tel, _, rtt = nrf24_usb.send_telemetry_request(link, rid, timeout_s=0.025)
                    if tel is not None:
                        st.tel_rx += 1
                        st.rtt_list.append(rtt)
    finally:
        elapsed = time.monotonic() - start
        info_after = link.get_info()
        link.close()

    total_cmds = sum(s.cmds_sent for s in stats.values())
    print(f"\n=== RESULTADO ({elapsed:.2f}s | {total_cmds / max(elapsed, 1e-6):.1f} cmds/s total) ===")
    for rid in robot_ids:
        stats[rid].print_report(elapsed)
    if info_after:
        print(f"\n  Base Watchdog Reinits: {info_after['radio_reinit_count']}")


if __name__ == "__main__":
    main()
