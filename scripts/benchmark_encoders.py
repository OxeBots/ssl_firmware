#!/usr/bin/env python3
"""Benchmark the wheel encoder acquisition path through the dashboard API.

Captures the firmware's ENCSTAT console lines (per-wheel publish rate, gap
jitter, processing time and overruns), standstill velocity noise and analog
raw noise while the robot runs normally. Use it before and after a firmware
optimization or a sampling-rate change; it never opens the serial port and
never moves a wheel (keep the wheels stopped for the noise metrics).

Example (dashboard already running):

    python3 scripts/benchmark_encoders.py --label baseline --duration 30 \
        --output tmp/verify/encoder_bench/baseline

The script writes results.json and report.md in the output directory.
"""

import argparse
import json
import re
import statistics
import time
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8765"
LOG_PREFIX = re.compile(r"^[IWEDV] \(\d+\) ")
# The firmware console frame carries at most 96 bytes, so longer ESP_LOG lines
# arrive as consecutive fragments (first one with the level/timestamp prefix).
# Reassemble them before parsing.
ENCSTAT = re.compile(
    r"ENCSTAT M(\d) n=(\d+) gap_avg_us=([\d.]+) gap_max_us=([\d.]+) "
    r"proc_avg_us=([\d.]+) proc_max_us=([\d.]+) overruns=(\d+)"
)
ENCSTAT_COMPACT = re.compile(
    r"ENC(?:STAT)? M(\d) n=(\d+) gap=(\d+)/(\d+) proc=([\d.]+)/([\d.]+) ovr=(\d+)"
)


def join_log_lines(lines):
    """Rejoins fragmented console lines (96-byte DebugText frames)."""
    joined = []
    for line in lines:
        if not joined or LOG_PREFIX.match(line):
            joined.append(line)
        else:
            joined[-1] += line
    return joined


def request(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as response:
        return json.load(response)


def capture(duration_s):
    """Polls the console cursor and the raw frame while the window runs."""
    since = request("/api/console?limit=1")["next_since"]
    lines = []
    raw_samples = []
    deadline = time.monotonic() + duration_s
    next_console = time.monotonic()
    while time.monotonic() < deadline:
        raw_samples.append(request("/api/raw")["data"])
        now = time.monotonic()
        if now >= next_console:
            payload = request(f"/api/console?limit=1000&since={since}")
            lines.extend(payload.get("lines", []))
            since = payload["next_since"]
            next_console = now + 0.5
        time.sleep(0.1)
    return lines, raw_samples


def parse_stats(lines):
    windows = {}
    for line in join_log_lines(lines):
        match = ENCSTAT.search(line)
        if match:
            values = (match.group(1), match.group(2), match.group(3), match.group(4),
                      match.group(5), match.group(6), match.group(7))
        else:
            compact = ENCSTAT_COMPACT.search(line)
            if not compact:
                continue
            values = (compact.group(1), compact.group(2), compact.group(3), compact.group(4),
                      compact.group(5), compact.group(6), compact.group(7))
        wheel = int(values[0]) - 1
        windows.setdefault(wheel, []).append(
            {
                "samples": int(values[1]),
                "gap_avg_us": float(values[2]),
                "gap_max_us": float(values[3]),
                "proc_avg_us": float(values[4]),
                "proc_max_us": float(values[5]),
                "overruns": int(values[6]),
            }
        )
    return windows


def noise(raw_samples):
    wheels = {}
    for wheel in range(4):
        rpm = [sample["ekf_rpm"][wheel] for sample in raw_samples]
        analog = [sample["analog_raw"][wheel] for sample in raw_samples]
        analog_mean = statistics.fmean(analog) if analog else 0.0
        wheels[f"M{wheel + 1}"] = {
            "rpm_mean": statistics.fmean(rpm) if rpm else 0.0,
            "rpm_std": statistics.pstdev(rpm) if len(rpm) > 1 else 0.0,
            "analog_mean": analog_mean,
            "analog_std": statistics.pstdev(analog) if len(analog) > 1 else 0.0,
            # A clamped analog input (raw 0 or >= 4095) has no useful noise
            # metric; the firmware also logs a stuck-input warning for it.
            "analog_invalid": analog_mean <= 1.0 or analog_mean >= 4090.0,
        }
    return wheels


def summarize(label, windows, raw_samples, duration_s):
    wheels = {}
    for wheel in sorted(windows):
        records = windows[wheel]
        noise_item = noise(raw_samples)[f"M{wheel + 1}"]
        wheels[f"M{wheel + 1}"] = {
            "windows": len(records),
            "samples_per_window": statistics.fmean(r["samples"] for r in records),
            "gap_avg_us": statistics.fmean(r["gap_avg_us"] for r in records),
            "gap_max_us": max(r["gap_max_us"] for r in records),
            "proc_avg_us": statistics.fmean(r["proc_avg_us"] for r in records),
            "proc_max_us": max(r["proc_max_us"] for r in records),
            "overruns": max(r["overruns"] for r in records),
            "rpm_std": noise_item["rpm_std"],
            "analog_std": noise_item["analog_std"],
            "analog_invalid": noise_item["analog_invalid"],
        }
    return {"label": label, "duration_s": round(duration_s, 1), "wheels": wheels}


def markdown(summary):
    lines = [f"# Encoder acquisition benchmark — {summary['label']}", ""]
    lines.append(f"Capture: {summary['duration_s']} s, wheels stopped.")
    lines.append("")
    lines.append(
        "| wheel | windows | samples/window | gap avg us | gap max us | proc avg us | "
        "proc max us | overruns | rpm std | analog std |"
    )
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for name, item in summary["wheels"].items():
        lines.append(
            f"| {name} | {item['windows']} | {item['samples_per_window']:.0f} | "
            f"{item['gap_avg_us']:.0f} | {item['gap_max_us']:.0f} | "
            f"{item['proc_avg_us']:.2f} | {item['proc_max_us']:.2f} | {item['overruns']} | "
            f"{item['rpm_std']:.3f} | {'invalid' if item['analog_invalid'] else format(item['analog_std'], '.2f')} |"
        )
    if not summary["wheels"]:
        lines.append("")
        lines.append("No ENCSTAT lines captured. Is CONFIG_ENCODER_DIAG enabled and the")
        lines.append("benchmark running for at least one 10 s ENCSTAT window?")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", default="capture", help="name of this capture (baseline, optimized, 160k...)")
    parser.add_argument("--duration", type=float, default=30.0, help="capture seconds (>= 10 s window)")
    parser.add_argument("--output", required=True, help="output directory for results.json and report.md")
    args = parser.parse_args()

    status = request("/api/status")
    if not status.get("live"):
        raise SystemExit("dashboard is not live (no robot streaming)")
    lines, raw_samples = capture(args.duration)
    summary = summarize(args.label, parse_stats(lines), raw_samples, args.duration)
    summary["date"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    summary["frame_counters"] = status["counters"]

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "results.json").write_text(json.dumps(summary, indent=1) + "\n")
    report = markdown(summary)
    (output / "report.md").write_text(report)
    print(report)


if __name__ == "__main__":
    main()
