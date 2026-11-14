"""Wheel control calibration tab (PID auto-tuning) and wheel position control."""

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto

GAP = 8
RIGHT_WIDTH = 330
PANEL_MARGIN = 34
BUTTON_HEIGHT = 32
# Share of the tab height taken by the calibration row, the rest goes to position.
TUNING_ROW_RATIO = 50
MIN_PLOT_HEIGHT = 80
POSITION_INPUT_WIDTH = 100
POSITION_BUTTON_WIDTH = 64
POSITION_ROW_HEIGHT = 24

CONFIRM_LABEL = "Robô levantado · rodas livres"
START_LABEL = "Calibrar controle"
STOP_LABEL = "Parar calibração"
STOP_POSITION_LABEL = "Parar posição"

PANEL = "tuning_panel"
PARAMS_PANEL = "tuning_params_panel"
POSITION_PANEL = "tuning_position_panel"
CONFIRM_CHECK = "tuning_confirm"
START_BUTTON = "tuning_start"
STOP_BUTTON = "tuning_stop"
STATUS_TEXT = "tuning_status"
PROGRESS_BAR = "tuning_progress"
PHASE_TEXT = "tuning_phase"
PLOT = "tuning_plot"
RESULTS_TABLE = "tuning_results_table"
PARAMS_TABLE = "tuning_params_table"
RESULT_STATUS = tuple(f"tuning_result_status_{wheel}" for wheel in range(proto.WHEEL_COUNT))
RESULT_RISE = tuple(f"tuning_result_rise_{wheel}" for wheel in range(proto.WHEEL_COUNT))
RESULT_OVERSHOOT = tuple(f"tuning_result_overshoot_{wheel}" for wheel in range(proto.WHEEL_COUNT))
RESULT_SS_ERROR = tuple(f"tuning_result_ss_{wheel}" for wheel in range(proto.WHEEL_COUNT))
PARAM_KP = tuple(f"tuning_param_kp_{wheel}" for wheel in range(proto.WHEEL_COUNT))
PARAM_KI = tuple(f"tuning_param_ki_{wheel}" for wheel in range(proto.WHEEL_COUNT))
PARAM_KD = tuple(f"tuning_param_kd_{wheel}" for wheel in range(proto.WHEEL_COUNT))
PARAM_FEEDFORWARD = tuple(f"tuning_param_ff_{wheel}" for wheel in range(proto.WHEEL_COUNT))
PARAM_VELOCITY_FF = tuple(f"tuning_param_vff_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_ABSOLUTE_CHECK = "position_absolute"
POSITION_MAX_RPM_INPUT = "position_max_rpm"
POSITION_STOP_BUTTON = "position_stop"
POSITION_STATUS = "position_status"
POSITION_ANGLE_INPUTS = tuple(f"position_angle_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_MOVE_BUTTONS = tuple(f"position_move_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_READ_ANGLE = tuple(f"position_read_angle_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_READ_ERROR = tuple(f"position_read_error_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_READ_STATUS = tuple(f"position_read_status_{wheel}" for wheel in range(proto.WHEEL_COUNT))

STATUS_WAITING = "Aguardando quadro da calibração do controle"
STATUS_STALE = "Dados da calibração desatualizados"
STATUS_RUNNING = "Calibrando · {}% concluído"
STATUS_DONE = "Calibração concluída · parâmetros salvos na NVS"
STATUS_FAILED = "Falha · {}"
STATUS_IDLE = "Ociosa · confirme o aviso de segurança para calibrar"
SPINNING_IDLE = "Nenhuma roda girando"
SPINNING_DONE = "Rotina concluída · motores parados"
SPINNING_FAILED = "Rotina interrompida · motores parados"
POSITION_IDLE = "Posição ociosa · motores livres"
POSITION_ACTIVE = "Movimento de posição em andamento"
POSITION_DONE = "Alvo(s) atingido(s) · segurando posição"
POSITION_STALE = "Dados de posição desatualizados"

RESULT_OK = "OK"
RESULT_RUNNING = "Em curso"
RESULT_FAILED = "Falhou"
RESULT_PENDING = "Pendente"
RESULT_STORED = "Salvo na NVS"

PHASE_NAMES = {
    proto.TUNING_PHASE_IDLE: "—",
    proto.TUNING_PHASE_FRICTION: "Atrito",
    proto.TUNING_PHASE_IDENTIFY: "Identificação",
    proto.TUNING_PHASE_VALIDATE: "Validação",
    proto.TUNING_PHASE_REFINE: "Refino",
    proto.TUNING_PHASE_STORE: "Gravação",
    proto.TUNING_PHASE_DONE: "Concluída",
}

TUNING_ERROR_NAMES = {
    proto.TUNING_ERROR_NONE: "Sem erro",
    proto.TUNING_ERROR_NO_MOVEMENT: "Roda não se moveu",
    proto.TUNING_ERROR_UNSTABLE: "Sem resposta válida no passo",
    proto.TUNING_ERROR_TIMEOUT: "Tempo esgotado",
    proto.TUNING_ERROR_ABORTED: "Cancelada",
}

POSITION_ERROR_NAMES = {
    proto.POSITION_ERROR_NONE: "Sem erro",
    proto.POSITION_ERROR_TIMEOUT: "Tempo esgotado",
    proto.POSITION_ERROR_ABORTED: "Cancelada",
    proto.POSITION_ERROR_ENCODER: "Encoder não calibrado",
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
    if state.demo:
        return True
    return bool(
        state.connected
        and state.streaming
        and state.last_state_t
        and now - state.last_state_t <= config.SENSOR_STALE_S
    )


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
        return STATUS_DONE, config.OK_COLOR
    if tuning["state"] == proto.TUNING_FAILED:
        return STATUS_FAILED.format(tuning_error_text(tuning["error_code"])), config.BAD_COLOR
    return STATUS_IDLE, config.TEXT_DIM


def spinning_text(tuning):
    """What the routine is doing right now."""
    if tuning is None:
        return SPINNING_IDLE
    if tuning["state"] == proto.TUNING_RUNNING:
        return phase_text(tuning)
    if tuning["state"] == proto.TUNING_DONE:
        return SPINNING_DONE
    if tuning["state"] == proto.TUNING_FAILED:
        return SPINNING_FAILED
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
    if tuning["state"] == proto.TUNING_DONE:
        return RESULT_FAILED
    if tuning["state"] == proto.TUNING_FAILED:
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


def rise_text(tuning, wheel):
    """Validated step rise time of one wheel."""
    if tuning is None or tuning["rise_ms"][wheel] <= 0:
        return "—"
    return f"{tuning['rise_ms'][wheel]} ms"


def percent_text(values, wheel):
    """Percent metric of one wheel, or a dash when it was never validated."""
    if values is None or values[wheel] <= 0:
        return "—"
    return f"{values[wheel]:.0f} %"


def position_error_text(code):
    """Portuguese description of a position-control failure reason."""
    return POSITION_ERROR_NAMES.get(code, f"Código {code}")


def position_live(state, now):
    """True while the latest position frame is recent enough to use."""
    return bool(state.last_position_t) and now - state.last_position_t <= config.POSITION_STALE_S


def demo_position_active(state):
    """True while a demo position move is in progress."""
    return bool(state.demo and state.demo_position is not None)


def position_status_text(state, now):
    """Status line and color for the position control panel."""
    if state.demo and state.demo_position is not None:
        pose = state.position if position_live(state, now) else None
        if pose is not None and pose["active_mask"] and pose["done_mask"] == pose["active_mask"]:
            return POSITION_DONE, config.OK_COLOR
        return POSITION_ACTIVE, config.ACCENT
    if not state.last_position_t:
        return POSITION_IDLE, config.TEXT_DIM
    if not position_live(state, now):
        return POSITION_STALE, config.BAD_COLOR
    pose = state.position
    if pose is None:
        return POSITION_IDLE, config.TEXT_DIM
    if pose["active_mask"] and pose["done_mask"] == pose["active_mask"]:
        return POSITION_DONE, config.OK_COLOR
    if pose["active_mask"]:
        return POSITION_ACTIVE, config.ACCENT
    if pose["error_code"] != proto.POSITION_ERROR_NONE:
        return f"Falha · {position_error_text(pose['error_code'])}", config.BAD_COLOR
    return POSITION_IDLE, config.TEXT_DIM


def position_row(pose, wheel):
    """Measured angle, error and status of one wheel inside the position panel."""
    if pose is None:
        return {"angle": "—", "error": "—", "status": "Parado", "color": config.TEXT_DIM}
    angle = pose["angle_deg"][wheel]
    target = pose["target_deg"][wheel]
    bit = 1 << wheel
    if not pose["active_mask"] & bit:
        return {"angle": f"{angle:.1f}°", "error": "—", "status": "Parado", "color": config.TEXT_DIM}
    error = target - angle
    status = "Alvo atingido" if pose["done_mask"] & bit else "Movendo"
    color = config.OK_COLOR if pose["done_mask"] & bit else config.ACCENT
    return {"angle": f"{angle:.1f}°", "error": f"{error:+.1f}°", "status": status, "color": color}


def move_enabled(state, now, wheel):
    """Girar button: live link, idle routines and a calibrated encoder."""
    if state.calibration_running() or tuning_running(state, now):
        return False
    if state.demo:
        return True
    if not wheel_live_link(state, now):
        return False
    return bool(state.valid_mask & (1 << wheel))


def wheel_live_link(state, now):
    """True when telemetry is fresh enough to command motion."""
    return bool(
        state.connected
        and state.streaming
        and state.last_state_t
        and now - state.last_state_t <= config.SENSOR_STALE_S
    )


def position_stop_enabled(state, now):
    """Parar posição button: any active move (or demo move)."""
    if demo_position_active(state):
        return True
    return position_live(state, now) and bool(state.position and state.position["active_mask"])


# ---------------------------------------------------------------------------
# Widget construction
# ---------------------------------------------------------------------------


def _create_themes():
    global _TABLE_THEME, _PROGRESS_THEME, _START_THEME, _STOP_THEME
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (2.0, 1.0)},
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
            "Mede atrito/ganho de cada roda, calcula PID + avanço e valida o passo; salva na NVS.",
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
                width=200,
                height=BUTTON_HEIGHT,
                callback=on_start,
            )
            dpg.bind_item_theme(START_BUTTON, _START_THEME)
            dpg.add_button(
                tag=STOP_BUTTON,
                label=STOP_LABEL,
                width=200,
                height=BUTTON_HEIGHT,
                callback=on_stop,
            )
            dpg.bind_item_theme(STOP_BUTTON, _STOP_THEME)
        rendering.add_text(STATUS_WAITING, "small", config.TEXT_DIM, tag=STATUS_TEXT)
        dpg.add_progress_bar(tag=PROGRESS_BAR, default_value=0.0, width=200, height=8)
        dpg.bind_item_theme(PROGRESS_BAR, _PROGRESS_THEME)
        rendering.add_text(SPINNING_IDLE, "small", config.TEXT_DIM, tag=PHASE_TEXT)


def _build_results(state):
    with rendering.panel("Resultado por roda", PARAMS_PANEL, width=RIGHT_WIDTH, height=1):
        with dpg.table(tag=RESULTS_TABLE + "_grid", header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.12)
            dpg.add_table_column(init_width_or_weight=0.28)
            dpg.add_table_column(init_width_or_weight=0.20)
            dpg.add_table_column(init_width_or_weight=0.20)
            dpg.add_table_column(init_width_or_weight=0.20)
            with dpg.table_row():
                for label in ("Roda", "Estado", "Subida", "Over", "Regime"):
                    rendering.add_text(label, "micro", config.TEXT_DIM)
            for wheel in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                    rendering.add_text(RESULT_PENDING, "tiny", config.TEXT_DIM, tag=RESULT_STATUS[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_RISE[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_OVERSHOOT[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_SS_ERROR[wheel])
        dpg.bind_item_theme(RESULTS_TABLE + "_grid", _TABLE_THEME)
        dpg.add_separator()
        rendering.add_text("Parâmetros atuais (NVS)", "small", config.TEXT_DIM)
        with dpg.table(tag=PARAMS_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.12)
            dpg.add_table_column(init_width_or_weight=0.17)
            dpg.add_table_column(init_width_or_weight=0.17)
            dpg.add_table_column(init_width_or_weight=0.17)
            dpg.add_table_column(init_width_or_weight=0.19)
            dpg.add_table_column(init_width_or_weight=0.18)
            with dpg.table_row():
                for label in ("Roda", "Kp", "Ki", "Kd", "Avanço", "v_ff"):
                    rendering.add_text(label, "micro", config.TEXT_DIM)
            for wheel in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=PARAM_KP[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=PARAM_KI[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=PARAM_KD[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=PARAM_FEEDFORWARD[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=PARAM_VELOCITY_FF[wheel])
        dpg.bind_item_theme(PARAMS_TABLE, _TABLE_THEME)
        dpg.add_separator()
        rendering.add_text(
            "Avanço: duty de atrito · v_ff: duty por rad/s · gravados na NVS.",
            "micro",
            config.TEXT_DIM,
        )
        dpg.add_separator()
        rendering.add_text("Resposta do passo · 12 s · média 200 ms", "small", config.TEXT_DIM)
        rendering.add_plot(PLOT, height=120, width=1)
        rendering.add_series(PLOT, 0, config.VELOCITY_COLOR)
        rendering.add_series(PLOT, 1, config.TARGET_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-config.CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=PLOT,
            tag=PLOT + "_placeholder",
        )


def _build_position_panel(state):
    def on_move(wheel):
        def callback(_sender, _app_data, _user_data=None):
            angle = float(dpg.get_value(POSITION_ANGLE_INPUTS[wheel]))
            absolute = bool(dpg.get_value(POSITION_ABSOLUTE_CHECK))
            max_rpm = float(dpg.get_value(POSITION_MAX_RPM_INPUT))
            angles = [0.0] * proto.WHEEL_COUNT
            angles[wheel] = angle
            state.start_position_move(angles, 1 << wheel, absolute, max(int(max_rpm), 0))

        return callback

    def on_stop(_sender, _app_data, _user_data=None):
        state.stop_position()

    with rendering.panel("Posição das rodas", POSITION_PANEL, width=1, height=1):
        rendering.add_text(
            "Gira a roda até o ângulo e mantém. Relativo = delta; absoluto = ângulo desde o boot.",
            "small",
            config.TEXT_DIM,
        )
        with dpg.group(horizontal=True):
            dpg.add_checkbox(
                tag=POSITION_ABSOLUTE_CHECK,
                label="Ângulo absoluto (senão relativo)",
                default_value=False,
            )
            dpg.add_input_float(
                tag=POSITION_MAX_RPM_INPUT,
                default_value=config.POSITION_MAX_RPM,
                width=POSITION_INPUT_WIDTH,
                step=10.0,
                format="%.0f",
            )
            rendering.add_text("RPM máx (0 = padrão)", "tiny", config.TEXT_DIM)
            dpg.add_button(
                tag=POSITION_STOP_BUTTON,
                label=STOP_POSITION_LABEL,
                width=140,
                height=BUTTON_HEIGHT,
                callback=on_stop,
            )
            dpg.bind_item_theme(POSITION_STOP_BUTTON, _STOP_THEME)
            rendering.add_text(POSITION_IDLE, "small", config.TEXT_DIM, tag=POSITION_STATUS)
        with dpg.table(tag=POSITION_PANEL + "_grid", header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.07)
            dpg.add_table_column(init_width_or_weight=0.17)
            dpg.add_table_column(init_width_or_weight=0.11)
            dpg.add_table_column(init_width_or_weight=0.13)
            dpg.add_table_column(init_width_or_weight=0.13)
            dpg.add_table_column(init_width_or_weight=0.39)
            with dpg.table_row():
                for label in ("Roda", "Alvo (°)", "", "Atual", "Erro", "Estado"):
                    rendering.add_text(label, "micro", config.TEXT_DIM)
            for wheel in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                    dpg.add_input_float(
                        tag=POSITION_ANGLE_INPUTS[wheel],
                        default_value=90.0,
                        width=POSITION_INPUT_WIDTH,
                        step=1.0,
                        format="%.1f",
                    )
                    dpg.add_button(
                        tag=POSITION_MOVE_BUTTONS[wheel],
                        label="Girar",
                        width=POSITION_BUTTON_WIDTH,
                        callback=on_move(wheel),
                    )
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=POSITION_READ_ANGLE[wheel])
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=POSITION_READ_ERROR[wheel])
                    rendering.add_text("Parado", "tiny", config.TEXT_DIM, tag=POSITION_READ_STATUS[wheel])
        dpg.bind_item_theme(POSITION_PANEL + "_grid", _TABLE_THEME)


def build(state):
    """Create the Controle tab contents once."""
    _create_themes()
    with dpg.group(horizontal=True):
        with dpg.group():
            _build_tuning_panel(state)
            _build_position_panel(state)
        _build_results(state)


# ---------------------------------------------------------------------------
# Per-frame updates
# ---------------------------------------------------------------------------


def _update_tuning_controls(state, now, inner_w):
    dpg.configure_item(
        START_BUTTON,
        width=(inner_w - GAP) // 2,
        enabled=tuning_start_enabled(state, now),
    )
    dpg.configure_item(
        STOP_BUTTON,
        width=(inner_w - GAP) // 2,
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
    dpg.configure_item(PLOT, width=RIGHT_WIDTH - PANEL_MARGIN)
    dpg.configure_item(PLOT + "_placeholder", show=not xs_velocity)


def _update_results(state, now):
    tuning = state.tuning if tuning_fresh(state, now) else None
    for wheel in range(proto.WHEEL_COUNT):
        rendering.set_text(
            RESULT_STATUS[wheel],
            result_state_text(tuning, wheel),
            result_state_color(tuning, wheel),
        )
        rendering.set_text(RESULT_RISE[wheel], rise_text(tuning, wheel), config.TEXT if tuning else config.TEXT_DIM)
        rendering.set_text(
            RESULT_OVERSHOOT[wheel],
            percent_text(tuning["overshoot_pct"] if tuning else None, wheel),
            config.TEXT if tuning else config.TEXT_DIM,
        )
        rendering.set_text(
            RESULT_SS_ERROR[wheel],
            percent_text(tuning["ss_error_pct"] if tuning else None, wheel),
            config.TEXT if tuning else config.TEXT_DIM,
        )
        if tuning is None:
            values = ["—"] * 5
        else:
            values = [
                gain_text(tuning["kp"][wheel]),
                gain_text(tuning["ki"][wheel]),
                gain_text(tuning["kd"][wheel], 3),
                f"{tuning['feedforward'][wheel]:.0f}",
                gain_text(tuning["velocity_ff"][wheel]),
            ]
        tags = (PARAM_KP[wheel], PARAM_KI[wheel], PARAM_KD[wheel], PARAM_FEEDFORWARD[wheel], PARAM_VELOCITY_FF[wheel])
        for tag, value in zip(tags, values):
            rendering.set_text(tag, value, config.TEXT if tuning else config.TEXT_DIM)


def _update_position(state, now, inner_w):
    pose = state.position if position_live(state, now) else None
    if state.demo and state.demo_position is not None and pose is not None:
        pose = dict(pose)
    status, color = position_status_text(state, now)
    rendering.set_text(POSITION_STATUS, status, color)
    for wheel in range(proto.WHEEL_COUNT):
        row = position_row(pose, wheel)
        rendering.set_text(POSITION_READ_ANGLE[wheel], row["angle"], config.TEXT if pose else config.TEXT_DIM)
        rendering.set_text(POSITION_READ_ERROR[wheel], row["error"], row["color"])
        rendering.set_text(POSITION_READ_STATUS[wheel], row["status"], row["color"])
        dpg.configure_item(POSITION_MOVE_BUTTONS[wheel], enabled=move_enabled(state, now, wheel))
    dpg.configure_item(
        POSITION_ABSOLUTE_CHECK,
        enabled=not (tuning_running(state, now) or state.calibration_running()),
    )
    dpg.configure_item(
        POSITION_MAX_RPM_INPUT,
        width=POSITION_INPUT_WIDTH,
        enabled=not (tuning_running(state, now) or state.calibration_running()),
    )
    dpg.configure_item(
        POSITION_STOP_BUTTON,
        enabled=position_stop_enabled(state, now),
    )
    dpg.configure_item(POSITION_PANEL + "_grid", width=inner_w)


def update(state, layout, now):
    """Refresh the Controle tab contents."""
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    main_w = max(320, body_w - RIGHT_WIDTH - GAP)
    tuning_h = max(240, body_h * TUNING_ROW_RATIO // 100)
    position_h = max(200, body_h - tuning_h - GAP)
    dpg.configure_item(PANEL, width=main_w, height=tuning_h)
    dpg.configure_item(POSITION_PANEL, width=main_w, height=position_h)
    dpg.configure_item(PARAMS_PANEL, width=RIGHT_WIDTH, height=body_h)
    inner_w = main_w - PANEL_MARGIN
    _update_tuning_controls(state, now, inner_w)
    _update_results(state, now)
    _update_position(state, now, inner_w)
