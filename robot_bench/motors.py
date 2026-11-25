"""Selectable wheel telemetry cards, speed plots, motor control and wheel position."""

import dearpygui.dearpygui as dpg

from . import config, raw, rendering
from . import protocol as proto
from .state import WHEEL_NAMES

GAP = 8
CONTROL_WIDTH = 280
CONTROL_INNER = CONTROL_WIDTH - 34
BUTTON_HEIGHT = 28
ARROW_STEP_WIDTH = 132
CARD_PADDING_X = 8
CARD_PADDING_Y = 3
CARD_ITEM_GAP = 2
CARD_INNER_MARGIN = 18
# The speed charts follow the trace, but never zoom in tighter than this, so
# encoder noise at standstill does not fill the plot.
MIN_SCALE_RPM = 50.0
PWM_MAX = 1023.0
DIRECTION_DEADBAND = 5.0
# Rows around the speed plot: the plot takes whatever card height is left
# over, so it stays the largest element at any window size.
CARD_BORDER = 1
MIN_PLOT_HEIGHT = 40
BAR_HEIGHT = 6
# Measured values as three rows of two label/value pairs; the column weights
# follow the width each column needs, so the rows fit the narrowest card.
DETAIL_ROWS = (
    (("angle", "ÂNGULO"), ("target", "Alvo")),
    (("pwm", "PWM"), ("direction", "Sentido")),
    (("pulse", "PULSO"), ("count", "Contagem")),
)
DETAIL_WEIGHTS = (0.17, 0.33, 0.17, 0.33)
# Space the sidebar tab bar takes above the Controle / Posição panels.
SIDEBAR_TAB_HEIGHT = 34
MIN_SIDEBAR_HEIGHT = 180

CARD_TAGS = tuple(f"motors_card_{index}" for index in range(proto.WHEEL_COUNT))
CONTROL_PANEL = "motors_control"
CONTROL_TARGET = "motors_control_target"
STOP_BUTTON = "motors_stop"
SWEEP_BUTTON = "motors_sweep"
ARROW_STEP_INPUT = "motors_arrow_step"
MAX_RPM_TEXT = "motors_max_rpm"
SWEEP_HINT = "motors_sweep_hint"
# Target buttons move the selected wheel by a fraction of the calibrated limit.
ADJUST_FRACTIONS = (-1.0, -0.25, 0.25, 1.0)
ADJUST_BUTTONS = {
    fraction: f"motors_adjust_{'m' if fraction < 0 else 'p'}{abs(int(fraction * 100))}"
    for fraction in ADJUST_FRACTIONS
}
SIDEBAR_TABS = "motors_sidebar_tabs"
SIDEBAR_CONTROL = "motors_sidebar_control"
SIDEBAR_POSITION = "motors_sidebar_position"
# Internal Motors tab bar: overview cards + sidebar, or the raw data view.
MOTORS_VIEW_TABS = "motors_view_tabs"
MOTORS_VIEW_OVERVIEW = "motors_view_overview"
MOTORS_VIEW_RAW = "motors_view_raw"
MOTORS_VIEW_TAB_HEIGHT = 34
MIN_VIEW_HEIGHT = 160

POSITION_PANEL = "motors_position"
POSITION_GRID = "motors_position_grid"
POSITION_ABSOLUTE_CHECK = "position_absolute"
POSITION_MAX_RPM_INPUT = "position_max_rpm"
POSITION_STOP_BUTTON = "position_stop"
POSITION_STATUS = "position_status"
POSITION_ANGLE_INPUTS = tuple(f"position_angle_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_MOVE_BUTTONS = tuple(f"position_move_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_READ_ANGLE = tuple(f"position_read_angle_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_READ_STATUS = tuple(f"position_read_status_{wheel}" for wheel in range(proto.WHEEL_COUNT))
POSITION_INPUT_WIDTH = 82
POSITION_BUTTON_WIDTH = 54

POSITION_IDLE = "Posição ociosa · motores livres"
POSITION_ACTIVE = "Movimento de posição em andamento"
POSITION_DONE = "Alvo(s) atingido(s) · segurando posição"
POSITION_STALE = "Dados de posição desatualizados"
STOP_POSITION_LABEL = "Parar posição"

POSITION_ERROR_NAMES = {
    proto.POSITION_ERROR_NONE: "Sem erro",
    proto.POSITION_ERROR_TIMEOUT: "Tempo esgotado",
    proto.POSITION_ERROR_ABORTED: "Cancelada",
    proto.POSITION_ERROR_ENCODER: "Encoder não calibrado",
}

_CARD_THEMES = []
_TABLE_THEME = None
_BAR_THEME = None
_STOP_THEME = None
_SIDEBAR_THEME = None
_SWEEP_THEMES = []
_BOUND_SELECTED = -1
_BOUND_SWEEP = None
_RAW_VIEW_ACTIVE = False


def _part(index, name):
    return f"{CARD_TAGS[index]}_{name}"


def wheel_live(state, now):
    """True while the newest state frame is fresh enough to display."""
    return bool(state.last_state_t) and now - state.last_state_t <= config.SENSOR_STALE_S


def encoder_status(state, index, now):
    """Encoder status text and color for one wheel card."""
    if not state.last_state_t:
        return "Aguardando", config.TEXT_DIM
    if not wheel_live(state, now):
        return "Desatualizada", config.BAD_COLOR
    if state.valid_mask & (1 << index):
        return "Encoder pronto", config.OK_COLOR
    return "Não calibrado", config.BAD_COLOR


def pulse_texts(state, index, now):
    """Digital FG pulse speed and count of one wheel, or dashes while invalid."""
    if not wheel_live(state, now) or not state.pulse_valid_mask & (1 << index):
        return "—", "—"
    wheel = state.wheels[index]
    return f"{wheel['pulse_rpm']:+.1f} RPM", f"{wheel['pulse_count']:,}"


def direction_label(pwm):
    """Rotation direction implied by the applied motor PWM."""
    if pwm > DIRECTION_DEADBAND:
        return "Horário"
    if pwm < -DIRECTION_DEADBAND:
        return "Anti-horário"
    return "Parado"


def direction_color(label):
    if label == "Parado":
        return config.TEXT_DIM
    return config.ACCENT if label == "Horário" else config.PWM_COLOR


def card_index_at(pos, rects):
    """Index of the card rectangle containing a screen position, or None."""
    x, y = pos
    for index, (minimum, maximum) in enumerate(rects):
        if minimum[0] <= x <= maximum[0] and minimum[1] <= y <= maximum[1]:
            return index
    return None


def adjust_label(fraction, limit):
    """Target button label: fraction of the calibrated limit and its RPM value."""
    return f"{fraction * 100:+.0f}% · {abs(fraction) * limit:.0f}"


def max_rpm_text(state):
    """Sidebar line with the calibrated limit, or the fallback when not calibrated."""
    if state.max_rpm_calibrated:
        return f"Máximo calibrado: ±{state.max_rpm:.0f} RPM", config.OK_COLOR
    return f"Máximo não calibrado · limite ±{state.max_rpm:.0f} RPM", config.DEMO_COLOR


def sweep_hint_text(state):
    """Sweep description with the current calibrated limit."""
    return f"Varredura ±{state.max_rpm:.0f} RPM · a cada 1 s"


def sidebar_height(body_h, tab_bar_h=SIDEBAR_TAB_HEIGHT):
    """Panel height left under the sidebar tab bar."""
    return max(MIN_SIDEBAR_HEIGHT, int(body_h) - int(tab_bar_h))


def view_height(body_h, tab_bar_h=MOTORS_VIEW_TAB_HEIGHT):
    """Content height left under the Motors internal tab bar."""
    return max(MIN_VIEW_HEIGHT, int(body_h) - int(tab_bar_h))


def card_plot_height(card_h, top_h, footer_h):
    """Height of the speed chart: the card height left by the rows around it.

    top_h runs from the card top to the chart top; footer_h is the encoder
    status and legend row under the chart.
    """
    chrome = CARD_ITEM_GAP + CARD_PADDING_Y + CARD_BORDER
    return max(MIN_PLOT_HEIGHT, int(card_h) - int(top_h) - int(footer_h) - chrome)


def active_view():
    """Alias of the selected Motors sub-tab, defaulting to the overview."""
    if not dpg.does_item_exist(MOTORS_VIEW_TABS):
        return MOTORS_VIEW_OVERVIEW
    selected = dpg.get_value(MOTORS_VIEW_TABS)
    if isinstance(selected, int):
        selected = dpg.get_item_alias(selected)
    return selected if selected == MOTORS_VIEW_RAW else MOTORS_VIEW_OVERVIEW


def sidebar_tab():
    """Alias of the selected sidebar sub-tab, defaulting to Controle."""
    if not dpg.does_item_exist(SIDEBAR_TABS):
        return SIDEBAR_CONTROL
    selected = dpg.get_value(SIDEBAR_TABS)
    if isinstance(selected, int):
        selected = dpg.get_item_alias(selected)
    return selected if selected == SIDEBAR_POSITION else SIDEBAR_CONTROL


# ---------------------------------------------------------------------------
# Pure position-control helpers (panel moved here from the old Controle tab)
# ---------------------------------------------------------------------------


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


def wheel_live_link(state, now):
    """True when telemetry is fresh enough to command motion."""
    return bool(
        state.connected
        and state.streaming
        and state.last_state_t
        and now - state.last_state_t <= config.SENSOR_STALE_S
    )


def move_enabled(state, now, wheel):
    """Girar button: live link, idle routines and a calibrated encoder."""
    if state.calibration_running() or state.tuning_running():
        return False
    if state.demo:
        return True
    if not wheel_live_link(state, now):
        return False
    return bool(state.valid_mask & (1 << wheel))


def position_stop_enabled(state, now):
    """Parar posição button: any active move (or demo move)."""
    if demo_position_active(state):
        return True
    return position_live(state, now) and bool(state.position and state.position["active_mask"])


def _create_themes():
    global _BAR_THEME, _STOP_THEME, _TABLE_THEME, _SIDEBAR_THEME
    for background, border, border_size in (
        (config.PANEL, config.PANEL_BORDER, 1.0),
        ((31, 46, 62), config.ACCENT, 2.0),
    ):
        _CARD_THEMES.append(
            rendering.create_theme(
                colors={
                    dpg.mvThemeCol_ChildBg: background,
                    dpg.mvThemeCol_Border: border,
                    dpg.mvThemeCol_Text: config.TEXT,
                },
                styles={
                    dpg.mvStyleVar_WindowPadding: (float(CARD_PADDING_X), float(CARD_PADDING_Y)),
                    # The internal Motors tab bar costs each card two rows of
                    # height; tighter item gaps keep the overview unclipped.
                    dpg.mvStyleVar_ItemSpacing: (8.0, float(CARD_ITEM_GAP)),
                    # Text rows are padded by the frame padding; the card keeps
                    # it small so the speed chart gets the height instead.
                    dpg.mvStyleVar_FramePadding: (10.0, 1.0),
                    dpg.mvStyleVar_ChildBorderSize: (border_size,),
                    dpg.mvStyleVar_ChildRounding: (8.0,),
                },
            )
        )
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (2.0, 0.0)},
        component=dpg.mvTable,
    )
    _BAR_THEME = rendering.create_theme(
        colors={dpg.mvThemeCol_PlotHistogram: config.PWM_COLOR, dpg.mvThemeCol_FrameBg: config.INSET},
        component=dpg.mvProgressBar,
    )
    _STOP_THEME = rendering.create_theme(
        colors={
            dpg.mvThemeCol_Button: (74, 30, 30),
            dpg.mvThemeCol_ButtonHovered: (120, 42, 38),
            dpg.mvThemeCol_ButtonActive: config.BAD_COLOR,
        },
        component=dpg.mvButton,
    )
    # The Motors internal tab bar costs the sidebar height; a tighter item
    # spacing keeps the Controle/Posição panels unclipped at 1024x640.
    _SIDEBAR_THEME = rendering.create_theme(
        colors={
            dpg.mvThemeCol_ChildBg: config.PANEL,
            dpg.mvThemeCol_Border: config.PANEL_BORDER,
            dpg.mvThemeCol_Text: config.TEXT,
        },
        styles={
            dpg.mvStyleVar_WindowPadding: (16.0, 10.0),
            dpg.mvStyleVar_ItemSpacing: (8.0, 2.0),
            dpg.mvStyleVar_ChildBorderSize: (1.0,),
            dpg.mvStyleVar_ChildRounding: (8.0,),
        },
    )
    _SWEEP_THEMES.extend(
        (
            rendering.create_theme(
                colors={
                    dpg.mvThemeCol_Button: config.INSET,
                    dpg.mvThemeCol_ButtonHovered: config.TAB_ACTIVE,
                    dpg.mvThemeCol_ButtonActive: config.ACCENT,
                },
                component=dpg.mvButton,
            ),
            rendering.create_theme(
                colors={
                    dpg.mvThemeCol_Button: (28, 70, 84),
                    dpg.mvThemeCol_ButtonHovered: (38, 92, 108),
                    dpg.mvThemeCol_ButtonActive: config.ACCENT,
                },
                component=dpg.mvButton,
            ),
        )
    )


def _build_card(index):
    card = CARD_TAGS[index]
    with dpg.child_window(tag=card, width=1, height=1, no_scrollbar=True):
        # Wheel name and speed share one row so the card height goes to the chart.
        with dpg.table(
            tag=_part(index, "header"), header_row=False, policy=dpg.mvTable_SizingStretchProp
        ):
            dpg.add_table_column(init_width_or_weight=0.55)
            dpg.add_table_column(init_width_or_weight=0.45)
            with dpg.table_row():
                rendering.add_text(
                    WHEEL_NAMES[index], "title", config.TEXT, tag=_part(index, "name")
                )
                with dpg.group(horizontal=True):
                    rendering.add_text(
                        "—", "big", config.VELOCITY_COLOR, tag=_part(index, "velocity")
                    )
                    rendering.add_text("RPM", "small", config.TEXT_DIM)
        # Measured values as label/value pairs, two per row.
        with dpg.table(
            tag=_part(index, "details"), header_row=False, policy=dpg.mvTable_SizingStretchProp
        ):
            for weight in DETAIL_WEIGHTS:
                dpg.add_table_column(init_width_or_weight=weight)
            for row in DETAIL_ROWS:
                with dpg.table_row():
                    for part, label in row:
                        rendering.add_text(label, "micro", config.TEXT_DIM)
                        rendering.add_text("—", "tiny", config.TEXT, tag=_part(index, part))
        dpg.bind_item_theme(_part(index, "header"), _TABLE_THEME)
        dpg.bind_item_theme(_part(index, "details"), _TABLE_THEME)
        dpg.add_progress_bar(tag=_part(index, "bar"), width=1, height=BAR_HEIGHT, default_value=0.0)
        dpg.bind_item_theme(_part(index, "bar"), _BAR_THEME)
        rendering.add_plot(_part(index, "plot"), height=MIN_PLOT_HEIGHT, width=1)
        rendering.add_series(_part(index, "plot"), 0, config.VELOCITY_COLOR)
        rendering.add_series(_part(index, "plot"), 1, config.TARGET_COLOR)
        rendering.add_series(_part(index, "plot"), 2, config.PULSE_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-config.CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=_part(index, "plot"),
            tag=_part(index, "placeholder"),
        )
        # Encoder status and chart legend live under the plot; the card keeps
        # every spare pixel for the chart itself.
        with dpg.group(tag=_part(index, "footer"), horizontal=True):
            rendering.add_text("Aguardando", "micro", config.TEXT_DIM, tag=_part(index, "encoder"))
            rendering.add_text("·", "micro", config.TEXT_DIM)
            rendering.add_text("Vel", "micro", config.VELOCITY_COLOR)
            rendering.add_text("/ alvo", "micro", config.TARGET_COLOR)
            rendering.add_text("/ pulso", "micro", config.PULSE_COLOR)
            rendering.add_text("· 12 s", "micro", config.TEXT_DIM)
            rendering.add_text("±600", "micro", config.TEXT_DIM, tag=_part(index, "scale"))


def _build_control(state):
    def on_adjust(_sender, _app_data, user_data):
        state.adjust_selected_target(float(user_data))

    def on_stop(_sender, _app_data, _user_data=None):
        state.stop_motors()

    def on_sweep(_sender, _app_data, _user_data=None):
        state.sweep = not state.sweep
        state.add_console("Varredura " + ("ligada" if state.sweep else "desligada"))

    def on_arrow_step(_sender, app_data, _user_data=None):
        state.arrow_step_rpm = float(max(1, int(app_data)))

    with rendering.panel(f"Controle · M{state.selected_wheel + 1}", CONTROL_PANEL, width=CONTROL_WIDTH, height=1):
        rendering.add_text("Alvo da roda selecionada", "small", config.TEXT_DIM)
        rendering.add_text("— RPM", "big", config.ACCENT, tag=CONTROL_TARGET)
        rendering.add_text(max_rpm_text(state)[0], "tiny", max_rpm_text(state)[1], tag=MAX_RPM_TEXT)
        rendering.add_text("Clique em uma roda ou pressione 1–4", "tiny", config.TEXT_DIM)
        dpg.add_separator()
        with dpg.group(horizontal=True):
            for fraction in ADJUST_FRACTIONS[:2]:
                button = dpg.add_button(
                    tag=ADJUST_BUTTONS[fraction],
                    label=adjust_label(fraction, state.max_rpm),
                    width=1,
                    height=BUTTON_HEIGHT,
                    callback=on_adjust,
                    user_data=fraction,
                )
                rendering.bind_font(button, "tiny")
        with dpg.group(horizontal=True):
            for fraction in ADJUST_FRACTIONS[2:]:
                button = dpg.add_button(
                    tag=ADJUST_BUTTONS[fraction],
                    label=adjust_label(fraction, state.max_rpm),
                    width=1,
                    height=BUTTON_HEIGHT,
                    callback=on_adjust,
                    user_data=fraction,
                )
                rendering.bind_font(button, "tiny")
        dpg.add_button(
            tag=STOP_BUTTON,
            label="Parar motores · Espaço",
            width=1,
            height=BUTTON_HEIGHT,
            callback=on_stop,
        )
        dpg.add_button(
            tag=SWEEP_BUTTON,
            label="Varredura desligada · W",
            width=1,
            height=BUTTON_HEIGHT,
            callback=on_sweep,
        )
        rendering.add_text(sweep_hint_text(state), "tiny", config.TEXT_DIM, tag=SWEEP_HINT)
        with dpg.group(horizontal=True):
            rendering.add_text("Passo ↑/↓ (RPM)", "tiny", config.TEXT_DIM)
            dpg.add_input_int(
                tag=ARROW_STEP_INPUT,
                default_value=int(state.arrow_step_rpm),
                width=ARROW_STEP_WIDTH,
                min_value=1,
                max_value=500,
                min_clamped=True,
                max_clamped=True,
                step=1,
                callback=on_arrow_step,
            )
            rendering.bind_font(ARROW_STEP_INPUT, "small")
        rendering.add_text("←/→ 100% do limite", "tiny", config.TEXT_DIM)


def _build_position(state):
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

    with rendering.panel("Posição das rodas", POSITION_PANEL, width=CONTROL_WIDTH, height=1):
        rendering.add_text("Gira a roda até o ângulo e mantém.", "small", config.TEXT_DIM)
        rendering.add_text("Relativo = delta · absoluto = desde o boot.", "tiny", config.TEXT_DIM)
        dpg.add_checkbox(
            tag=POSITION_ABSOLUTE_CHECK,
            label="Ângulo absoluto (senão relativo)",
            default_value=False,
        )
        rendering.bind_font(POSITION_ABSOLUTE_CHECK, "micro")
        with dpg.group(horizontal=True):
            rendering.add_text("RPM máx (0 = padrão)", "tiny", config.TEXT_DIM)
            dpg.add_input_float(
                tag=POSITION_MAX_RPM_INPUT,
                default_value=config.POSITION_MAX_RPM,
                width=POSITION_INPUT_WIDTH,
                step=0.0,
                format="%.0f",
            )
        with dpg.group(horizontal=True):
            dpg.add_button(
                tag=POSITION_STOP_BUTTON,
                label=STOP_POSITION_LABEL,
                width=140,
                height=BUTTON_HEIGHT,
                callback=on_stop,
            )
            dpg.bind_item_theme(POSITION_STOP_BUTTON, _STOP_THEME)
        rendering.add_text(POSITION_IDLE, "micro", config.TEXT_DIM, tag=POSITION_STATUS)
        with dpg.table(tag=POSITION_GRID, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.10)
            dpg.add_table_column(init_width_or_weight=0.34)
            dpg.add_table_column(init_width_or_weight=0.22)
            dpg.add_table_column(init_width_or_weight=0.34)
            with dpg.table_row():
                for label in ("Roda", "Alvo (°)", "", "Atual"):
                    rendering.add_text(label, "micro", config.TEXT_DIM)
            for wheel in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                    dpg.add_input_float(
                        tag=POSITION_ANGLE_INPUTS[wheel],
                        default_value=90.0,
                        width=POSITION_INPUT_WIDTH,
                        step=0.0,
                        format="%.1f",
                    )
                    move_button = dpg.add_button(
                        tag=POSITION_MOVE_BUTTONS[wheel],
                        label="Girar",
                        width=POSITION_BUTTON_WIDTH,
                        callback=on_move(wheel),
                    )
                    rendering.bind_font(move_button, "tiny")
                    with dpg.group():
                        rendering.add_text("—", "micro", config.TEXT_DIM, tag=POSITION_READ_ANGLE[wheel])
                        rendering.add_text("Parado", "micro", config.TEXT_DIM, tag=POSITION_READ_STATUS[wheel])
        dpg.bind_item_theme(POSITION_GRID, _TABLE_THEME)


def _build_overview(state):
    with dpg.group(horizontal=True):
        with dpg.group():
            for row in range(2):
                with dpg.group(horizontal=True):
                    _build_card(row * 2)
                    _build_card(row * 2 + 1)
        with dpg.group():
            with dpg.tab_bar(tag=SIDEBAR_TABS):
                with dpg.tab(label="Controle", tag=SIDEBAR_CONTROL):
                    _build_control(state)
                with dpg.tab(label="Posição", tag=SIDEBAR_POSITION):
                    _build_position(state)
            dpg.bind_item_theme(CONTROL_PANEL, _SIDEBAR_THEME)
            dpg.bind_item_theme(POSITION_PANEL, _SIDEBAR_THEME)


def build(state):
    """Create the Motors tab contents once: overview and raw data views."""
    _create_themes()
    with dpg.tab_bar(tag=MOTORS_VIEW_TABS):
        with dpg.tab(label="Visão geral", tag=MOTORS_VIEW_OVERVIEW):
            _build_overview(state)
        with dpg.tab(label="Dados brutos", tag=MOTORS_VIEW_RAW):
            raw.build(state)


def _apply_selection(selected):
    global _BOUND_SELECTED
    if selected == _BOUND_SELECTED or not _CARD_THEMES:
        return
    for index, card in enumerate(CARD_TAGS):
        dpg.bind_item_theme(card, _CARD_THEMES[1 if index == selected else 0])
    _BOUND_SELECTED = selected


def _update_card(state, index, now):
    wheel = state.wheels[index]
    live = wheel_live(state, now)
    status, status_color = encoder_status(state, index, now)
    rendering.set_text(_part(index, "encoder"), status, status_color)
    name_color = config.ACCENT if index == state.selected_wheel else config.TEXT
    rendering.set_text(_part(index, "name"), WHEEL_NAMES[index], name_color)
    if live:
        rendering.set_text(_part(index, "velocity"), f"{wheel['vel']:+.1f}", config.VELOCITY_COLOR)
        rendering.set_text(_part(index, "angle"), f"{wheel['angle']:.1f}°", config.TEXT)
        rendering.set_text(_part(index, "target"), f"{wheel['tgt']:+.1f} RPM", config.TEXT)
        duty = wheel["pwm"] / PWM_MAX * 100.0
        rendering.set_text(_part(index, "pwm"), f"{wheel['pwm']:+.0f} · {duty:+.0f}%", config.TEXT)
        dpg.set_value(_part(index, "bar"), min(1.0, abs(wheel["pwm"]) / PWM_MAX))
        label = direction_label(wheel["pwm"])
        rendering.set_text(_part(index, "direction"), label, direction_color(label))
    else:
        for part in ("velocity", "angle", "target", "pwm"):
            rendering.set_text(_part(index, part), "—", config.TEXT_DIM)
        dpg.set_value(_part(index, "bar"), 0.0)
        rendering.set_text(_part(index, "direction"), "Desconhecido", config.TEXT_DIM)
    pulse_rpm, pulse_count = pulse_texts(state, index, now)
    rendering.set_text(_part(index, "pulse"), pulse_rpm, config.PULSE_COLOR)
    rendering.set_text(_part(index, "count"), pulse_count, config.TEXT if live else config.TEXT_DIM)

    plot = _part(index, "plot")
    with state.history_lock:
        samples = list(state.history[index])
    xs_velocity, ys_velocity = rendering.history_series(samples, 1, now)
    xs_target, ys_target = rendering.history_series(samples, 2, now)
    xs_pulse, ys_pulse = rendering.history_series(samples, 3, now)
    rendering.set_series(plot, 0, xs_velocity, ys_velocity)
    rendering.set_series(plot, 1, xs_target, ys_target)
    rendering.set_series(plot, 2, xs_pulse, ys_pulse)
    peak = max(
        MIN_SCALE_RPM,
        rendering.history_peak(samples, 1, now, config.CHART_WINDOW_S, MIN_SCALE_RPM),
        rendering.history_peak(samples, 2, now, config.CHART_WINDOW_S, MIN_SCALE_RPM),
        rendering.history_peak(samples, 3, now, config.CHART_WINDOW_S, MIN_SCALE_RPM),
    )
    rendering.set_y_limits(plot, -peak, peak)
    rendering.set_text(_part(index, "scale"), f"±{peak:.0f}")
    dpg.configure_item(_part(index, "placeholder"), show=not xs_velocity)


def _update_control(state, now):
    global _BOUND_SWEEP
    selected = state.selected_wheel
    rendering.set_text(CONTROL_PANEL + "_title", f"Controle · M{selected + 1}")
    live = wheel_live(state, now)
    target = state.manual_targets[selected] if state.demo else state.wheels[selected]["tgt"]
    target_text = f"{target:+.0f} RPM" if live else "— RPM"
    rendering.set_text(CONTROL_TARGET, target_text, config.ACCENT if live else config.TEXT_DIM)
    for fraction, tag in ADJUST_BUTTONS.items():
        dpg.configure_item(
            tag,
            width=(CONTROL_INNER - GAP) // 2,
            label=adjust_label(fraction, state.max_rpm),
        )
    limit_text, limit_color = max_rpm_text(state)
    rendering.set_text(MAX_RPM_TEXT, limit_text, limit_color)
    rendering.set_text(SWEEP_HINT, sweep_hint_text(state))
    dpg.configure_item(STOP_BUTTON, width=CONTROL_INNER)
    dpg.configure_item(
        SWEEP_BUTTON,
        width=CONTROL_INNER,
        label=f"Varredura {'ligada' if state.sweep else 'desligada'} · W",
    )
    if state.sweep != _BOUND_SWEEP and _SWEEP_THEMES:
        dpg.bind_item_theme(SWEEP_BUTTON, _SWEEP_THEMES[1 if state.sweep else 0])
        _BOUND_SWEEP = state.sweep


def _update_position(state, now, inner_w):
    pose = state.position if position_live(state, now) else None
    status, color = position_status_text(state, now)
    rendering.set_text(POSITION_STATUS, status, color)
    for wheel in range(proto.WHEEL_COUNT):
        row = position_row(pose, wheel)
        feedback = row["angle"] if row["error"] == "—" else f"{row['angle']} · {row['error']}"
        rendering.set_text(POSITION_READ_ANGLE[wheel], feedback, config.TEXT if pose else config.TEXT_DIM)
        rendering.set_text(POSITION_READ_STATUS[wheel], row["status"], row["color"])
        dpg.configure_item(POSITION_MOVE_BUTTONS[wheel], enabled=move_enabled(state, now, wheel))
    busy = state.calibration_running() or state.tuning_running()
    dpg.configure_item(POSITION_ABSOLUTE_CHECK, enabled=not busy)
    dpg.configure_item(POSITION_MAX_RPM_INPUT, width=POSITION_INPUT_WIDTH, enabled=not busy)
    dpg.configure_item(POSITION_STOP_BUTTON, enabled=position_stop_enabled(state, now))
    dpg.configure_item(POSITION_GRID, width=inner_w)


def update(state, layout, now):
    """Refresh the active Motors sub-tab contents."""
    global _RAW_VIEW_ACTIVE
    body_h = view_height(
        layout["body_h"], rendering.item_height(MOTORS_VIEW_TABS) or MOTORS_VIEW_TAB_HEIGHT
    )
    if active_view() == MOTORS_VIEW_RAW:
        if not _RAW_VIEW_ACTIVE:
            _RAW_VIEW_ACTIVE = True
            raw.on_show(state)
        raw.update(state, {"body_w": layout["body_w"], "body_h": body_h}, now)
        return
    if _RAW_VIEW_ACTIVE:
        _RAW_VIEW_ACTIVE = False
        raw.on_hide(state)
    _apply_selection(state.selected_wheel)
    card_w = max(250, (layout["body_w"] - CONTROL_WIDTH - 2 * GAP) // 2)
    card_h = max(180, (body_h - GAP) // 2)
    inner_w = card_w - CARD_INNER_MARGIN
    for index, card in enumerate(CARD_TAGS):
        plot = _part(index, "plot")
        dpg.configure_item(card, width=card_w, height=card_h)
        dpg.configure_item(_part(index, "header"), width=inner_w)
        dpg.configure_item(_part(index, "details"), width=inner_w)
        dpg.configure_item(_part(index, "bar"), width=inner_w)
        # Measure the rows above the chart instead of assuming a row height, so
        # the chart always receives every pixel those rows do not use.
        top_h = max(0.0, dpg.get_item_pos(plot)[1])
        dpg.configure_item(
            plot,
            width=inner_w,
            height=card_plot_height(
                card_h, top_h, rendering.item_height(_part(index, "footer"))
            ),
        )
        _update_card(state, index, now)
    panel_h = sidebar_height(body_h, rendering.item_height(SIDEBAR_TABS) or SIDEBAR_TAB_HEIGHT)
    dpg.configure_item(CONTROL_PANEL, width=CONTROL_WIDTH, height=panel_h)
    dpg.configure_item(POSITION_PANEL, width=CONTROL_WIDTH, height=panel_h)
    if sidebar_tab() == SIDEBAR_POSITION:
        _update_position(state, now, CONTROL_INNER)
    else:
        _update_control(state, now)


def leave(state):
    """Release the raw mux focus when the Motors tab is no longer visible."""
    global _RAW_VIEW_ACTIVE
    if _RAW_VIEW_ACTIVE:
        _RAW_VIEW_ACTIVE = False
        raw.on_hide(state)


def select_wheel_at(state, pos):
    """Select the wheel card under a screen position; returns True on a hit."""
    if active_view() != MOTORS_VIEW_OVERVIEW:
        return False
    rects = []
    for card in CARD_TAGS:
        if not dpg.does_item_exist(card):
            continue
        position = dpg.get_item_pos(card)
        size = dpg.get_item_rect_size(card)
        rects.append((position, (position[0] + size[0], position[1] + size[1])))
    index = card_index_at(pos, rects)
    if index is None:
        return False
    state.selected_wheel = index
    return True
