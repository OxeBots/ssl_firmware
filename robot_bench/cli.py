"""Command line entry point for the bench dashboard."""

import argparse

from . import config
from .app import run_gui
from .demo import DemoSource
from .state import BenchState


def main(argv=None):
    parser = argparse.ArgumentParser(description=f"{config.APP_NAME} - diagnóstico do robô SSL")
    parser.add_argument("--port", help="porta serial (padrão: tela de escolha de porta na inicialização)")
    parser.add_argument("--baud", type=int, default=115200, help="taxa de transmissão serial (padrão 115200)")
    parser.add_argument("--demo", action="store_true", help="executar com dados sintéticos, sem hardware")
    parser.add_argument(
        "--width", type=int, default=config.DEFAULT_WINDOW[0], help="largura inicial da janela (redimensionável)"
    )
    parser.add_argument(
        "--height", type=int, default=config.DEFAULT_WINDOW[1], help="altura inicial da janela (redimensionável)"
    )
    parser.add_argument(
        "--tab",
        type=int,
        choices=range(len(config.TAB_TITLES)),
        default=0,
        help="aba inicial: 0 motores, 1 calibração, 2 saúde, 3 ToF, 4 IMU, 5 console, 6 rádio (se conectado)",
    )
    parser.add_argument("--screenshot-dir", help="salvar um PNG da janela a cada --screenshot-period segundos")
    parser.add_argument("--screenshot-period", type=float, default=2.0, help="segundos entre capturas de tela")
    args = parser.parse_args(argv)

    state = BenchState()
    state.active_tab = args.tab
    if args.demo:
        state.demo = True
        state.connected = "demo"
        state.streaming = True
        DemoSource(state).start()
    run_gui(state, args)
