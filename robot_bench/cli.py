"""Command line entry point for the bench dashboard."""

import argparse

from . import api, config
from .app import run_gui
from .demo import DemoSource
from .state import BenchState


def main(argv=None):
    parser = argparse.ArgumentParser(description=f"{config.APP_NAME} - diagnóstico do robô SSL")
    parser.add_argument("--port", help="porta serial (padrão: tela de escolha de porta na inicialização)")
    parser.add_argument("--baud", type=int, default=115200, help="taxa de transmissão serial (padrão 115200)")
    parser.add_argument("--demo", action="store_true", help="executar com dados sintéticos, sem hardware")
    parser.add_argument(
        "--width", type=int, default=None, help="largura inicial da janela (padrão: ajustada à tela)"
    )
    parser.add_argument(
        "--height", type=int, default=None, help="altura inicial da janela (padrão: ajustada à tela)"
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="fator de escala da interface, ex.: 1.5 ou 2.0 (padrão: automático pela tela)",
    )
    parser.add_argument(
        "--tab",
        type=int,
        choices=range(len(config.TAB_TITLES)),
        default=0,
        help="aba inicial: 0 motores, 1 calibração, 2 saúde, 3 ToF, 4 IMU, 5 console, 6 rádio, 7 bench",
    )
    parser.add_argument("--screenshot-dir", help="salvar um PNG da janela a cada --screenshot-period segundos")
    parser.add_argument("--screenshot-period", type=float, default=2.0, help="segundos entre capturas de tela")
    parser.add_argument(
        "--api-host",
        default=api.DEFAULT_HOST,
        help="interface da API local; 0.0.0.0 aceita a rede local, a web é sempre bloqueada (padrão 127.0.0.1)",
    )
    parser.add_argument(
        "--api-port", type=int, default=api.DEFAULT_PORT, help="porta HTTP da API local, 0 desativa (padrão 8765)"
    )
    parser.add_argument(
        "--api-socket",
        default=api.DEFAULT_SOCKET_PATH,
        help='socket Unix da API compartilhado com o host; "" desativa (padrão tmp/robot_bench_api.sock)',
    )
    args = parser.parse_args(argv)
    if not 0 <= args.api_port <= 65535:
        parser.error("--api-port deve estar entre 0 e 65535")

    state = BenchState()
    state.active_tab = args.tab
    if args.demo:
        state.demo = True
        state.connected = "demo"
        state.streaming = True
        DemoSource(state).start()
    server = api.ApiServer(state)
    server.start(args.api_host, args.api_port or None, args.api_socket)
    try:
        run_gui(state, args)
    finally:
        server.stop()
