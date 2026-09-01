"""Command line entry point for the bench dashboard."""

import argparse

from .app import run_gui
from .demo import DemoSource
from .state import BenchState


def main(argv=None):
    parser = argparse.ArgumentParser(description="SSL bench dashboard (BenchDashboard firmware test)")
    parser.add_argument("--port", help="serial port (default: port picker screen on startup)")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default 115200)")
    parser.add_argument("--demo", action="store_true", help="run on synthetic data without hardware")
    parser.add_argument("--width", type=int, default=1280, help="window width")
    parser.add_argument("--height", type=int, default=800, help="window height")
    parser.add_argument("--screenshot-dir", help="save a PNG of the window every --screenshot-period seconds")
    parser.add_argument("--screenshot-period", type=float, default=2.0, help="seconds between screenshots")
    args = parser.parse_args(argv)

    state = BenchState()
    if args.demo:
        state.demo = True
        state.connected = "demo"
        DemoSource(state).start()
    run_gui(state, args)
