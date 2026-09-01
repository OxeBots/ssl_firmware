"""Command line entry point for the bench dashboard."""

import argparse
import os

from .app import run_gui
from .demo import DemoSource
from .state import BenchState


def main(argv=None):
    # The dashboard never plays audio; the dummy driver silences ALSA errors
    # on hosts without a sound card. SDL reads this at pygame.init().
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    parser = argparse.ArgumentParser(description="SSL bench dashboard (serial debug firmware)")
    parser.add_argument("--port", help="serial port (default: port picker screen on startup)")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default 115200)")
    parser.add_argument("--demo", action="store_true", help="run on synthetic data without hardware")
    parser.add_argument("--width", type=int, default=1280, help="initial window width (resizable)")
    parser.add_argument("--height", type=int, default=800, help="initial window height (resizable)")
    parser.add_argument(
        "--tab", type=int, choices=(0, 1, 2), default=0,
        help="tab to open: 0 motors, 1 health, 2 sensing",
    )
    parser.add_argument("--screenshot-dir", help="save a PNG of the window every --screenshot-period seconds")
    parser.add_argument("--screenshot-period", type=float, default=2.0, help="seconds between screenshots")
    args = parser.parse_args(argv)

    state = BenchState()
    state.active_tab = args.tab
    if args.demo:
        state.demo = True
        state.connected = "demo"
        state.streaming = True
        DemoSource(state).start()
    run_gui(state, args)
