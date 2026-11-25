"""Wheel control calibration content: PID auto-tuning controls, results and step plot.

Built by calibration.py inside the Calibração tab's Controlador sub-tab.
"""

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto

GAP = 8
RIGHT_WIDTH = 450
PANEL_MARGIN = 34
BUTTON_HEIGHT = 32
# Vertical space the controls panel spends before the step plot; the plot
# takes the rest of the panel, between the minimum and maximum heights.
PLOT_RESERVED_HEIGHT = 356
MIN_PLOT_HEIGHT = 80
MAX_PLOT_HEIGHT = 200

CONFIRM_LABEL = "Robô levantado · rodas livres"
START_LABEL = "Calibrar controle"
STOP_LABEL = "Parar calibração"

PANEL = "tuning_panel"
PARAMS_PANEL = "tuning_params_panel"
CONFIRM_CHECK = "tuning_confirm"
START_BUTTON = "tuning_start"
STOP_BUTTON = "tuning_stop"
STATUS_TEXT = "tuning_status"
PROGRESS_BAR = "tuning_progress"
PHASE_TEXT = "tuning_phase"
PLOT = "tuning_plot"
RESULTS_TABLE = "tuning_results_table"
PARAMS_TABLE = "tuning_params_table"
MAX_MEASURED_TEXT = "tuning_max_measured"
MAX_LIMIT_TEXT = "tuning_max_limit"
# One display row per wheel and rotation direction: forward (+) then reverse (-).
DIRECTION_LABELS = tuple(
    f"M{wheel + 1} {side}" for wheel in range(proto.WHEEL_COUNT) for side in ("+", "−")
)
DIRECTION_ROWS = len(DIRECTION_LABELS)
RESULT_STATUS = tuple(f"tuning_result_status_{row}" for row in range(DIRECTION_ROWS))
RESULT_RISE = tuple(f"tuning_result_rise_{row}" for row in range(DIRECTION_ROWS))
RESULT_OVERSHOOT = tuple(f"tuning_result_overshoot_{row}" for row in range(DIRECTION_ROWS))
RESULT_SS_ERROR = tuple(f"tuning_result_ss_{row}" for row in range(DIRECTION_ROWS))
PARAM_KP = tuple(f"tuning_param_kp_{row}" for row in range(DIRECTION_ROWS))
PARAM_KI = tuple(f"tuning_param_ki_{row}" for row in range(DIRECTION_ROWS))
PARAM_KD = tuple(f"tuning_param_kd_{row}" for row in range(DIRECTION_ROWS))
PARAM_FEEDFORWARD = tuple(f"tuning_param_ff_{row}" for row in range(DIRECTION_ROWS))
PARAM_VELOCITY_FF = tuple(f"tuning_param_vff_{row}" for row in range(DIRECTION_ROWS))

STATUS_WAITING = "Aguardando quadro da calibração do controle"
STATUS_STALE = "Dados da calibração desatualizados"
STATUS_RUNNING = "Calibrando os dois sentidos · {}% concluído"
STATUS_DONE = "Calibração concluída nos dois sentidos · limite de {:.0f} RPM salvo na NVS"
STATUS_FAILED = "Falha · {}"
STATUS_PARTIAL = "Calibração parcial · {}/{} rodas salvas · {}"
STATUS_SPEED_MISMATCH = "Parâmetros salvos · velocidades médias divergentes"
STATUS_IDLE = "Ociosa · confirme o aviso de segurança para calibrar os dois sentidos"
SPINNING_IDLE = "Nenhuma roda girando"
SPINNING_DONE = "Rotina concluída · motores parados"

RESULT_OK = "Salvo"
RESULT_RUNNING = "Em curso"
RESULT_FAILED = "Falhou"
RESULT_PENDING = "Pendente"

PHASE_NAMES = {
    proto.TUNING_PHASE_IDLE: "—",
    proto.TUNING_PHASE_FRICTION: "Atrito",
    proto.TUNING_PHASE_IDENTIFY: "Identificação",
    proto.TUNING_PHASE_VALIDATE: "Validação",
    proto.TUNING_PHASE_REFINE: "Refino",
    proto.TUNING_PHASE_STORE: "Gravação",
    proto.TUNING_PHASE_DONE: "Concluída",
    proto.TUNING_PHASE_MAX_SPEED: "Velocidade máxima",
}

TUNING_ERROR_NAMES = {
    proto.TUNING_ERROR_NONE: "Sem erro",
    proto.TUNING_ERROR_NO_MOVEMENT: "Roda não se moveu",
    proto.TUNING_ERROR_UNSTABLE: "Sem resposta válida no passo",
    proto.TUNING_ERROR_TIMEOUT: "Tempo esgotado",
    proto.TUNING_ERROR_ABORTED: "Cancelada",
    proto.TUNING_ERROR_SPEED_MISMATCH: "Velocidades médias divergentes",
    proto.TUNING_ERROR_SAVE_FAILED: "Falha ao salvar os parâmetros",
}

_TABLE_THEME = None
_PROGRESS_THEME = None
_START_THEME = None
_STOP_THEME = None


# ---------------------------------------------------------------------------
# Pure helpers: freshness, button rules, texts and gains
# ---------------------------------------------------------------------------


def tuning_fresh(state, now):
    """True while the latest control-calibration frame is recent enough to use."""
    return bool(state.last_tuning_t) and now - state.last_tuning_t <= config.TUNING_STALE_S


def tuning_running(state, now):
    """True while a fresh calibration frame reports a running routine."""
    return tuning_fresh(state, now) and state.tuning["state"] == proto.TUNING_RUNNING


def demo_tuning_active(state):
    """True while a demo run is active before its first running frame arrives."""
    return bool(state.demo and state.demo_tuning_start is not None)


def tuning_start_enabled(state, now):
    """Calibrar button: confirmed, live link, fresh frame, routine idle and motors stopped."""
    if not state.tuning_confirm or not tuning_fresh(state, now):
        return False
    if tuning_running(state, now) or demo_tuning_active(state):
        return False
    if state.calibration_running():
        return False
    return state._live_link(now)


def tuning_stop_enabled(state, now):
    """Parar button: only while the routine (or the demo run) is active."""
    return tuning_running(state, now) or demo_tuning_active(state)


def phase_text(tuning):
    """Current routine phase as short Portuguese text."""
    if tuning is None:
        return "Fase: —"
    phase = PHASE_NAMES.get(tuning["phase"], f"Fase {tuning['phase']}")
    if tuning["wheel"] < proto.WHEEL_COUNT:
        return f"Roda M{tuning['wheel'] + 1} · {phase}"
    return f"Fase: {phase}"


def tuning_error_text(code):
    """Portuguese description of a calibration failure reason."""
    return TUNING_ERROR_NAMES.get(code, f"Código {code}")


def tuning_status_text(state, now):
    """Status line and color for the control calibration routine."""
    tuning = state.tuning
    if not state.last_tuning_t:
        return STATUS_WAITING, config.TEXT_DIM
    if not tuning_fresh(state, now):
        return STATUS_STALE, config.BAD_COLOR
    if tuning["state"] == proto.TUNING_RUNNING:
        return STATUS_RUNNING.format(tuning["progress"]), config.ACCENT
    if tuning["state"] == proto.TUNING_DONE:
        if tuning["error_code"] == proto.TUNING_ERROR_SPEED_MISMATCH:
            return STATUS_SPEED_MISMATCH, config.PWM_COLOR
        limit = tuning["max_rpm"] if tuning["max_rpm"] > 0 else state.max_rpm
        return STATUS_DONE.format(limit), config.OK_COLOR
    if tuning["state"] == proto.TUNING_FAILED:
        error = tuning_error_text(tuning["error_code"])
        saved = tuning["result_mask"].bit_count()
        if saved:
            return STATUS_PARTIAL.format(saved, proto.WHEEL_COUNT, error), config.PWM_COLOR
        return STATUS_FAILED.format(error), config.BAD_COLOR
    return STATUS_IDLE, config.TEXT_DIM


def spinning_text(tuning):
    """What the routine is doing right now."""
    if tuning is None:
        return SPINNING_IDLE
    if tuning["state"] == proto.TUNING_RUNNING:
        return phase_text(tuning)
    if tuning["state"] in (proto.TUNING_DONE, proto.TUNING_FAILED):
        return SPINNING_DONE
    return SPINNING_IDLE


def result_state_text(tuning, wheel):
    """Per-wheel calibration result text."""
    if tuning is None:
        return RESULT_PENDING
    bit = 1 << wheel
    if tuning["result_mask"] & bit:
        return RESULT_OK
    if tuning["state"] == proto.TUNING_RUNNING and tuning["wheel"] == wheel:
        return RESULT_RUNNING
    if tuning["state"] in (proto.TUNING_DONE, proto.TUNING_FAILED):
        return RESULT_FAILED
    return RESULT_PENDING


def result_state_color(tuning, wheel):
    """Palette color matching one per-wheel result."""
    text = result_state_text(tuning, wheel)
    if text == RESULT_OK:
        return config.OK_COLOR
    if text == RESULT_RUNNING:
        return config.ACCENT
    if text == RESULT_FAILED:
        return config.BAD_COLOR
    return config.TEXT_DIM


def smooth_history(samples, value_index, window=5):
    """Median + moving-average display series (the encoder velocity ripple is
    a measurement artefact; the plot shows the underlying trend)."""
    if not samples:
        return []
    half = max(0, window // 2)
    medians = []
    for index, sample in enumerate(samples):
        begin = max(0, index - half)
        end = min(len(samples), index + half + 1)
        values = sorted(item[value_index] for item in samples[begin:end])
        # Lower median for even-sized edge windows (keeps single spikes out).
        medians.append((sample[0], values[(len(values) - 1) // 2]))
    series = []
    for index in range(len(medians)):
        begin = max(0, index - half)
        end = min(len(medians), index + half + 1)
        average = sum(item[1] for item in medians[begin:end]) / (end - begin)
        series.append((medians[index][0], average))
    return series


def gain_text(value, digits=2):
    """Format one controller gain with a fixed number of decimals."""
    return f"{value:.{digits}f}"


def rise_text(tuning, wheel, reverse=False):
    """Validated step rise time of one wheel in one rotation direction."""
    if tuning is None:
        return "—"
    values = tuning["rise_ms_rev"] if reverse else tuning["rise_ms"]
    if values[wheel] <= 0:
        return "—"
    return f"{values[wheel]} ms"


def percent_text(values, wheel):
    """Percent metric of one wheel, or a dash when it was never validated."""
    if values is None or values[wheel] <= 0:
        return "—"
    return f"{values[wheel]:.0f} %"


def result_rows(tuning):
    """Per-direction result rows: forward (+) then reverse (-) for each wheel."""
    rows = []
    for wheel in range(proto.WHEEL_COUNT):
        status = result_state_text(tuning, wheel)
        color = result_state_color(tuning, wheel)
        for reverse, suffix in ((False, ""), (True, "_rev")):
            rows.append(
                {
                    "label": DIRECTION_LABELS[2 * wheel + reverse],
                    "status": status,
                    "color": color,
                    "rise": rise_text(tuning, wheel, reverse),
                    "overshoot": percent_text(tuning[f"overshoot_pct{suffix}"] if tuning else None, wheel),
                    "ss_error": percent_text(tuning[f"ss_error_pct{suffix}"] if tuning else None, wheel),
                }
            )
    return rows


def max_speed_texts(tuning):
    """Per-wheel measured maxima and the common calibrated limit for the panel."""
    if tuning is None:
        return "Máx. medido: M1 — · M2 — · M3 — · M4 —", "Limite comum: —"
    measured = " · ".join(
        f"M{wheel + 1} {tuning['wheel_max_rpm'][wheel]:.1f}"
        if tuning["wheel_max_rpm"][wheel] > 0
        else f"M{wheel + 1} —"
        for wheel in range(proto.WHEEL_COUNT)
    )
    rpm = tuning["max_rpm"]
    limit = (
        f"{rpm:.1f} RPM"
        if rpm > 0 and (rpm < proto.WHEEL_RPM_MAX - 0.05 or tuning["state"] == proto.TUNING_DONE)
        else "—"
    )
    return f"Máx. medido: {measured} RPM", f"Limite comum: {limit}"


def param_rows(tuning):
    """Per-direction NVS parameter rows: Kp/Ki/Kd repeat, feedforward is per sense."""
    rows = []
    for wheel in range(proto.WHEEL_COUNT):
        if tuning is None:
            rows.append({"label": f"M{wheel + 1} +", "values": ["—"] * 5})
            rows.append({"label": f"M{wheel + 1} −", "values": ["—"] * 5})
            continue
        common = [
            gain_text(tuning["kp"][wheel]),
            gain_text(tuning["ki"][wheel]),
            gain_text(tuning["kd"][wheel], 3),
        ]
        rows.append(
            {
                "label": f"M{wheel + 1} +",
                "values": common + [f"{tuning['feedforward'][wheel]:.0f}", gain_text(tuning["velocity_ff"][wheel])],
            }
        )
        rows.append(
            {
                "label": f"M{wheel + 1} −",
                "values": common
                + [f"{tuning['feedforward_rev'][wheel]:.0f}", gain_text(tuning["velocity_ff_rev"][wheel])],
            }
        )
    return rows


# ---------------------------------------------------------------------------
# Widget construction
# ---------------------------------------------------------------------------


def _create_themes():
    global _TABLE_THEME, _PROGRESS_THEME, _START_THEME, _STOP_THEME
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (float(rendering.px(2)), 0.0)},
        component=dpg.mvTable,
    )
    _PROGRESS_THEME = rendering.create_theme(
        colors={dpg.mvThemeCol_PlotHistogram: config.PWM_COLOR, dpg.mvThemeCol_FrameBg: config.INSET},
        component=dpg.mvProgressBar,
    )
    _START_THEME = rendering.create_theme(
        colors={
            dpg.mvThemeCol_Button: (28, 70, 84),
            dpg.mvThemeCol_ButtonHovered: (38, 92, 108),
            dpg.mvThemeCol_ButtonActive: config.ACCENT,
        },
        component=dpg.mvButton,
    )
    _STOP_THEME = rendering.create_theme(
        colors={
            dpg.mvThemeCol_Button: (74, 30, 30),
            dpg.mvThemeCol_ButtonHovered: (120, 42, 38),
            dpg.mvThemeCol_ButtonActive: config.BAD_COLOR,
        },
        component=dpg.mvButton,
    )


def _build_tuning_panel(state):
    def on_confirm(_sender, app_data, _user_data=None):
        state.set_tuning_confirm(bool(app_data))

    def on_start(_sender, _app_data, _user_data=None):
        state.start_tuning()

    def on_stop(_sender, _app_data, _user_data=None):
        state.stop_tuning()

    with rendering.panel("Calibração do controle", PANEL, width=1, height=1):
        rendering.add_text(
            "Mede cada roda nos dois sentidos e salva os ajustes que passam no teste.",
            "small",
            config.TEXT_DIM,
        )
        rendering.add_text(
            "Compara as velocidades médias com as quatro rodas em movimento.",
            "small",
            config.TEXT_DIM,
        )
        rendering.add_text("PERIGO: levante o robô e mantenha as rodas livres.", "small", config.BAD_COLOR)
        dpg.add_checkbox(
            tag=CONFIRM_CHECK,
            label=CONFIRM_LABEL,
            default_value=False,
            callback=on_confirm,
        )
        with dpg.group(horizontal=True):
            dpg.add_button(
                tag=START_BUTTON,
                label=START_LABEL,
                width=rendering.px(200),
                height=rendering.px(BUTTON_HEIGHT),
                callback=on_start,
            )
            dpg.bind_item_theme(START_BUTTON, _START_THEME)
            dpg.add_button(
                tag=STOP_BUTTON,
                label=STOP_LABEL,
                width=rendering.px(200),
                height=rendering.px(BUTTON_HEIGHT),
                callback=on_stop,
            )
            dpg.bind_item_theme(STOP_BUTTON, _STOP_THEME)
        rendering.add_text(STATUS_WAITING, "small", config.TEXT_DIM, tag=STATUS_TEXT)
        dpg.add_progress_bar(
            tag=PROGRESS_BAR, default_value=0.0, width=rendering.px(200), height=rendering.px(8)
        )
        dpg.bind_item_theme(PROGRESS_BAR, _PROGRESS_THEME)
        rendering.add_text(SPINNING_IDLE, "small", config.TEXT_DIM, tag=PHASE_TEXT)
        dpg.add_separator()
        rendering.add_text("Resposta do passo · 12 s · média 200 ms", "small", config.TEXT_DIM)
        rendering.add_plot(PLOT, height=rendering.px(120), width=1)
        rendering.add_series(PLOT, 0, config.VELOCITY_COLOR)
        rendering.add_series(PLOT, 1, config.TARGET_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-config.CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=PLOT,
            tag=PLOT + "_placeholder",
        )


def _build_results():
    with rendering.panel("Resultado e parâmetros", PARAMS_PANEL, width=rendering.px(RIGHT_WIDTH), height=1):
        with dpg.group(horizontal=True):
            with dpg.group():
                rendering.add_text("Resultado por sentido", "small", config.TEXT_DIM)
                with dpg.table(tag=RESULTS_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
                    dpg.add_table_column(init_width_or_weight=0.14)
                    dpg.add_table_column(init_width_or_weight=0.26)
                    dpg.add_table_column(init_width_or_weight=0.22)
                    dpg.add_table_column(init_width_or_weight=0.19)
                    dpg.add_table_column(init_width_or_weight=0.19)
                    with dpg.table_row():
                        for label in ("Roda", "Estado", "Subida", "Over", "Regime"):
                            rendering.add_text(label, "micro", config.TEXT_DIM)
                    for index, label in enumerate(DIRECTION_LABELS):
                        with dpg.table_row():
                            rendering.add_text(label, "tiny", config.TEXT)
                            rendering.add_text(
                                RESULT_PENDING, "tiny", config.TEXT_DIM, tag=RESULT_STATUS[index]
                            )
                            rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_RISE[index])
                            rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_OVERSHOOT[index])
                            rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_SS_ERROR[index])
                dpg.bind_item_theme(RESULTS_TABLE, _TABLE_THEME)
                rendering.add_text("+ horário · − anti-horário", "micro", config.TEXT_DIM)
            with dpg.group():
                rendering.add_text("Parâmetros atuais", "small", config.TEXT_DIM)
                with dpg.table(tag=PARAMS_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
                    dpg.add_table_column(init_width_or_weight=0.13)
                    dpg.add_table_column(init_width_or_weight=0.17)
                    dpg.add_table_column(init_width_or_weight=0.16)
                    dpg.add_table_column(init_width_or_weight=0.17)
                    dpg.add_table_column(init_width_or_weight=0.18)
                    dpg.add_table_column(init_width_or_weight=0.19)
                    with dpg.table_row():
                        for label in ("Roda", "Kp", "Ki", "Kd", "Avanço", "v_ff"):
                            rendering.add_text(label, "micro", config.TEXT_DIM)
                    for index, label in enumerate(DIRECTION_LABELS):
                        with dpg.table_row():
                            rendering.add_text(label, "tiny", config.TEXT)
                            for tag in (
                                PARAM_KP[index],
                                PARAM_KI[index],
                                PARAM_KD[index],
                                PARAM_FEEDFORWARD[index],
                                PARAM_VELOCITY_FF[index],
                            ):
                                rendering.add_text("—", "tiny", config.TEXT_DIM, tag=tag)
                dpg.bind_item_theme(PARAMS_TABLE, _TABLE_THEME)
                rendering.add_text(
                    "Avanço: atrito do sentido · v_ff: duty por rad/s · NVS.",
                    "micro",
                    config.TEXT_DIM,
                )
        dpg.add_separator()
        rendering.add_text("", "micro", config.TEXT_DIM, tag=MAX_MEASURED_TEXT)
        rendering.add_text("", "small", config.TEXT_DIM, tag=MAX_LIMIT_TEXT)


def build(state):
    """Create the Controlador sub-tab contents once."""
    _create_themes()
    with dpg.group(horizontal=True):
        _build_tuning_panel(state)
        _build_results()


# ---------------------------------------------------------------------------
# Per-frame updates
# ---------------------------------------------------------------------------


def _update_tuning_controls(state, now, inner_w, body_h):
    gap = rendering.px(GAP)
    dpg.configure_item(
        START_BUTTON,
        width=(inner_w - gap) // 2,
        enabled=tuning_start_enabled(state, now),
    )
    dpg.configure_item(
        STOP_BUTTON,
        width=(inner_w - gap) // 2,
        enabled=tuning_stop_enabled(state, now),
    )
    dpg.configure_item(PROGRESS_BAR, width=inner_w)
    if dpg.get_value(CONFIRM_CHECK) != state.tuning_confirm:
        dpg.set_value(CONFIRM_CHECK, state.tuning_confirm)
    status, color = tuning_status_text(state, now)
    rendering.set_text(STATUS_TEXT, status, color)
    tuning = state.tuning if tuning_fresh(state, now) else None
    dpg.set_value(PROGRESS_BAR, tuning["progress"] / 100.0 if tuning is not None else 0.0)
    running = tuning_running(state, now) or demo_tuning_active(state)
    rendering.set_text(
        PHASE_TEXT,
        spinning_text(tuning) if tuning is not None or not running else "Iniciando calibração",
        config.ACCENT if running else config.TEXT_DIM,
    )
    wheel = tuning["wheel"] if tuning is not None and tuning["wheel"] < proto.WHEEL_COUNT else 0
    with state.history_lock:
        samples = list(state.history[wheel])
    # Display smoothing only: the control metrics in the firmware use their own
    # median + 50 ms average over the 100 Hz samples.
    display = [
        (velocity[0], velocity[1], target[1])
        for velocity, target in zip(smooth_history(samples, 1), smooth_history(samples, 2))
    ]
    xs_velocity, ys_velocity = rendering.history_series(display, 1, now)
    xs_target, ys_target = rendering.history_series(display, 2, now)
    rendering.set_series(PLOT, 0, xs_velocity, ys_velocity)
    rendering.set_series(PLOT, 1, xs_target, ys_target)
    peak = max(
        100.0,
        rendering.history_peak(display, 1, now, config.CHART_WINDOW_S, 100.0),
        rendering.history_peak(display, 2, now, config.CHART_WINDOW_S, 100.0),
    )
    rendering.set_y_limits(PLOT, -peak, peak)
    plot_h = min(
        rendering.px(MAX_PLOT_HEIGHT),
        max(rendering.px(MIN_PLOT_HEIGHT), int(body_h) - rendering.px(PLOT_RESERVED_HEIGHT)),
    )
    dpg.configure_item(PLOT, width=inner_w, height=plot_h)
    dpg.configure_item(PLOT + "_placeholder", show=not xs_velocity)


def _update_results(state, now):
    tuning = state.tuning if tuning_fresh(state, now) else None
    tone = config.TEXT if tuning else config.TEXT_DIM
    right_inner = rendering.px(RIGHT_WIDTH) - rendering.px(PANEL_MARGIN) - rendering.px(GAP)
    results_w = right_inner * 48 // 100
    params_w = right_inner - results_w
    dpg.configure_item(RESULTS_TABLE, width=results_w)
    dpg.configure_item(PARAMS_TABLE, width=params_w)
    for row, status_tag, rise_tag, overshoot_tag, ss_tag in zip(
        result_rows(tuning), RESULT_STATUS, RESULT_RISE, RESULT_OVERSHOOT, RESULT_SS_ERROR
    ):
        rendering.set_text(status_tag, row["status"], row["color"])
        rendering.set_text(rise_tag, row["rise"], tone)
        rendering.set_text(overshoot_tag, row["overshoot"], tone)
        rendering.set_text(ss_tag, row["ss_error"], tone)
    for row, kp_tag, ki_tag, kd_tag, ff_tag, vff_tag in zip(
        param_rows(tuning), PARAM_KP, PARAM_KI, PARAM_KD, PARAM_FEEDFORWARD, PARAM_VELOCITY_FF
    ):
        for tag, value in zip((kp_tag, ki_tag, kd_tag, ff_tag, vff_tag), row["values"]):
            rendering.set_text(tag, value, tone)
    measured, limit = max_speed_texts(tuning)
    rendering.set_text(MAX_MEASURED_TEXT, measured, tone)
    rendering.set_text(
        MAX_LIMIT_TEXT,
        limit,
        config.OK_COLOR if limit != "Limite comum: —" else config.TEXT_DIM,
    )


def update(state, layout, now):
    """Refresh the Controlador sub-tab contents."""
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    right_w = rendering.px(RIGHT_WIDTH)
    gap = rendering.px(GAP)
    main_w = max(rendering.px(320), body_w - right_w - gap)
    dpg.configure_item(PANEL, width=main_w, height=body_h)
    dpg.configure_item(PARAMS_PANEL, width=right_w, height=body_h)
    inner_w = main_w - rendering.px(PANEL_MARGIN)
    _update_tuning_controls(state, now, inner_w, body_h)
    _update_results(state, now)
