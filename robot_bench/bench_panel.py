"""Controls and results for the ESP-NOW stress test."""

import dearpygui.dearpygui as dpg

from . import communication_bench as bench
from . import config, radio, rendering

PANEL = "bench_communication"
IDS = "bench_robot_ids"
RATE = "bench_rate"
DURATION = "bench_duration"
TRAFFIC = "bench_traffic"
COMMAND_FORMAT = "bench_command_format"
INTERVAL = "bench_telemetry_interval"
TIMEOUT = "bench_timeout"
BROADCAST = "bench_force_broadcast"
START = "bench_start"
STOP = "bench_stop"
UART_TEST = "bench_uart_test"
STATUS = "bench_status"
PROGRESS = "bench_progress"
SUMMARY = "bench_summary"
OUTPUT = "bench_output"
RESULTS = "bench_results"
DETAILS = "bench_details"
COUNTERS = "bench_counters"
DEADLINE = "bench_deadline"
TELEMETRY = "Só telemetria"
WITH_COMMANDS = "Telemetria com comandos"
COMMAND_MODES = {"Individual": "unicast", "Equipe": "team"}
MODE_LABELS = {
    "telemetry": TELEMETRY,
    "unicast": "Telemetria com comandos individuais",
    "team": "Telemetria com comandos de equipe",
    "loopback": "Loopback UART",
}


def _format_ms(value):
    return "—" if value is None else f"{value:.2f}"


class BenchPanel:
    def __init__(self):
        self._session = None
        self._run = None
        self._shown_result = None
        self._error = ""

    def build(self):
        with dpg.child_window(tag=PANEL, width=-1, height=rendering.px(420)):
            rendering.add_text("Teste de comunicação ESP-NOW", "title")
            dpg.add_radio_button((TELEMETRY, WITH_COMMANDS), tag=TRAFFIC, default_value=TELEMETRY, horizontal=True)
            with dpg.tooltip(TRAFFIC):
                rendering.add_text("Os comandos do teste usam movimento e chute zerados.", "small")
            with dpg.group(horizontal=True):
                with dpg.group():
                    rendering.add_text("IDs dos robôs", "label")
                    dpg.add_input_text(tag=IDS, default_value="0", width=rendering.px(160))
                for label, tag, value, limit in (
                    ("Frequência (Hz/robô)", RATE, 60, bench.MAX_RATE_HZ),
                    ("Duração (s)", DURATION, 10, bench.MAX_DURATION_S),
                ):
                    with dpg.group():
                        rendering.add_text(label, "label")
                        dpg.add_input_int(
                            tag=tag,
                            default_value=value,
                            width=rendering.px(140),
                            step=0,
                            min_value=1,
                            max_value=limit,
                            min_clamped=True,
                            max_clamped=True,
                        )
            with dpg.collapsing_header(label="Opções avançadas"):
                with dpg.group(horizontal=True):
                    dpg.add_combo(
                        tuple(COMMAND_MODES),
                        label="Formato dos comandos zerados",
                        tag=COMMAND_FORMAT,
                        default_value="Individual",
                        width=rendering.px(140),
                        show=False,
                    )
                    dpg.add_checkbox(label="Forçar broadcast", tag=BROADCAST)
                with dpg.group(horizontal=True):
                    for label, tag, value, limit in (
                        ("Intervalo da telemetria (ciclos)", INTERVAL, 1, bench.MAX_TELEMETRY_INTERVAL),
                        ("Tempo limite da resposta (ms)", TIMEOUT, 150, bench.MAX_TIMEOUT_MS),
                    ):
                        dpg.add_input_int(
                            label=label,
                            tag=tag,
                            default_value=value,
                            width=rendering.px(70),
                            step=0,
                            min_value=1,
                            max_value=limit,
                            min_clamped=True,
                            max_clamped=True,
                        )
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Testar UART", tag=UART_TEST, enabled=False, callback=self._start, user_data="loopback"
                    )
                    rendering.add_text("Mede o trecho entre o computador e a ponte.", "small", config.TEXT_DIM)
            with dpg.group(horizontal=True):
                dpg.add_button(label="Iniciar teste", tag=START, enabled=False, callback=self._start)
                dpg.add_button(label="Parar", tag=STOP, enabled=False, callback=self._stop)
                rendering.add_text("", "small", tag=STATUS)
            with dpg.group(tag=OUTPUT, show=False):
                dpg.add_separator()
                rendering.add_text("", "small", tag=SUMMARY)
                dpg.add_progress_bar(tag=PROGRESS, width=-1, default_value=0.0)
                self._build_table(RESULTS, ("ID", "Respostas", "No ciclo", "Latência p95 (ms)", "Maior lacuna (ms)"))
                rendering.add_text("", "tiny", config.TEXT_DIM, tag=DEADLINE)
                with dpg.collapsing_header(label="Detalhes do resultado"):
                    rendering.add_text("", "small", tag=COUNTERS)
                    self._build_table(DETAILS, ("ID", "Expiradas", "Mediana (ms)", "p99 (ms)", "Máxima (ms)"))

    @staticmethod
    def _build_table(tag, labels):
        with dpg.table(
            tag=tag,
            header_row=True,
            policy=dpg.mvTable_SizingStretchProp,
            row_background=True,
            borders_innerH=True,
        ):
            for label in labels:
                dpg.add_table_column(label=label)

    def _start(self, _sender, _app_data, user_data):
        if self._session is None:
            return
        mode = "telemetry"
        if user_data == "loopback":
            mode = "loopback"
        elif dpg.get_value(TRAFFIC) == WITH_COMMANDS:
            mode = COMMAND_MODES[dpg.get_value(COMMAND_FORMAT)]
        try:
            options = bench.Options(
                robot_ids=bench.parse_robot_ids(dpg.get_value(IDS)),
                rate_hz=dpg.get_value(RATE),
                duration_s=dpg.get_value(DURATION),
                mode=mode,
                telemetry_every=dpg.get_value(INTERVAL),
                timeout_ms=dpg.get_value(TIMEOUT),
                force_broadcast=dpg.get_value(BROADCAST),
            )
            run = bench.Benchmark(options)
            self._session.start_benchmark(run)
        except (ValueError, radio.RadioError) as error:
            self._error = str(error)
        else:
            self._run = run
            self._error = ""

    def _stop(self, _sender, _app_data, _user_data):
        if self._run is not None:
            self._run.cancel()

    def update(self, session, snapshot, layout):
        if session is not self._session:
            self._session = session
            self._error = ""
            if session is not None:
                dpg.set_value(IDS, str(snapshot["robot_id"]))

        dpg.configure_item(PANEL, height=layout["body_h"])
        active = bool(snapshot and snapshot["benchmarking"])
        connected = bool(snapshot and snapshot["connected"])
        for tag in (START, UART_TEST):
            dpg.configure_item(tag, enabled=connected and not active)
        dpg.configure_item(STOP, enabled=active)
        for tag in (IDS, RATE, DURATION, TRAFFIC, INTERVAL, TIMEOUT, BROADCAST):
            dpg.configure_item(tag, enabled=not active)
        dpg.configure_item(COMMAND_FORMAT, enabled=not active, show=dpg.get_value(TRAFFIC) == WITH_COMMANDS)

        run = self._run
        result = run.result if run is not None else None
        if self._error:
            status, color = self._error, config.BAD_COLOR
        elif active:
            status, color = "Teste em andamento · controle por rádio pausado", config.ACCENT
        elif result is not None and result.state == "error":
            status, color = result.error, config.BAD_COLOR
        elif not connected:
            status, color = "Conecte a ponte ESP-NOW em Conexões.", config.TEXT_DIM
        elif result is not None:
            status = "Teste interrompido" if result.state == "cancelled" else "Teste concluído"
            color = config.TEXT
        else:
            status, color = "", config.TEXT_DIM
        rendering.set_text(STATUS, status, color)

        if result is not None and result is not self._shown_result:
            self._shown_result = result
            dpg.configure_item(OUTPUT, show=True)
            done = result.cycles + result.skipped
            dpg.set_value(PROGRESS, done / result.planned_cycles)
            dpg.configure_item(PROGRESS, overlay=f"{done}/{result.planned_cycles} ciclos")
            rate = result.cycles / result.elapsed_s if result.elapsed_s else 0
            summary = f"{MODE_LABELS[run.options.mode]} · {rate:.1f} Hz efetivos"
            if run.options.force_broadcast and run.options.mode != "loopback":
                summary += " · Broadcast forçado"
            rendering.set_text(SUMMARY, summary)
            rendering.set_text(
                COUNTERS,
                f"{result.skipped} ciclos pulados · "
                f"{result.tx_frames} quadros TX · {result.tx_bytes:,} B TX · {result.rx_bytes:,} B RX · "
                f"{result.gateway_restarts} reinícios da ponte",
            )
            rendering.set_text(DEADLINE, f"No ciclo = resposta em até {1000 / run.options.rate_hz:.2f} ms.")
            dpg.delete_item(RESULTS, children_only=True, slot=1)
            dpg.delete_item(DETAILS, children_only=True, slot=1)
            for robot in result.robots:
                on_time = f"{robot.on_time / robot.requests:.1%}" if robot.requests else "—"
                with dpg.table_row(parent=RESULTS):
                    dpg.add_text(str(robot.robot_id))
                    dpg.add_text(f"{robot.replies}/{robot.requests}")
                    dpg.add_text(on_time)
                    dpg.add_text(_format_ms(robot.p95_ms))
                    dpg.add_text(_format_ms(robot.max_gap_ms))
                with dpg.table_row(parent=DETAILS):
                    dpg.add_text(str(robot.robot_id))
                    dpg.add_text(str(robot.timed_out))
                    for ms in (robot.median_ms, robot.p99_ms, robot.max_ms):
                        dpg.add_text(_format_ms(ms))
