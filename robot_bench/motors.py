"""Selectable wheel telemetry cards, speed plots and the motor control panel."""

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto
from .state import WHEEL_NAMES

GAP = 8
CONTROL_WIDTH = 280
CONTROL_INNER = CONTROL_WIDTH - 34
BUTTON_HEIGHT = 32
CARD_PADDING_X = 8
CARD_INNER_MARGIN = 18
MIN_SCALE_RPM = 600.0
PWM_MAX = 1023.0
DIRECTION_DEADBAND = 5.0
# Vertical space a card spends on everything except the speed plot
CARD_FIXED_HEIGHT = 194

CARD_TAGS = tuple(f"motors_card_{index}" for index in range(proto.WHEEL_COUNT))
CONTROL_PANEL = "motors_control"
CONTROL_TARGET = "motors_control_target"
STOP_BUTTON = "motors_stop"
SWEEP_BUTTON = "motors_sweep"
OVERRIDE_CHECK = "motors_override"
OVERRIDE_HINT = "motors_override_hint"
ADJUST_BUTTONS = {
    -500: "motors_adjust_m500",
    -100: "motors_adjust_m100",
    100: "motors_adjust_p100",
    500: "motors_adjust_p500",
}

_CARD_THEMES = []
_TABLE_THEME = None
_BAR_THEME = None
_STOP_THEME = None
_SWEEP_THEMES = []
_BOUND_SELECTED = -1
_BOUND_SWEEP = None


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


def _create_themes():
    global _BAR_THEME, _STOP_THEME, _TABLE_THEME
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
                    dpg.mvStyleVar_WindowPadding: (CARD_PADDING_X, 6.0),
                    dpg.mvStyleVar_ChildBorderSize: (border_size,),
                    dpg.mvStyleVar_ChildRounding: (8.0,),
                },
            )
        )
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (2.0, 1.0)},
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
        with dpg.table(tag=_part(index, "header"), header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.55)
            dpg.add_table_column(init_width_or_weight=0.45)
            with dpg.table_row():
                rendering.add_text(WHEEL_NAMES[index], "title", config.TEXT, tag=_part(index, "name"))
                rendering.add_text("Aguardando", "small", config.TEXT_DIM, tag=_part(index, "encoder"))
        with dpg.group(horizontal=True):
            rendering.add_text("—", "big", config.VELOCITY_COLOR, tag=_part(index, "velocity"))
            rendering.add_text("RPM", "small", config.TEXT_DIM)
        with dpg.table(tag=_part(index, "details"), header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.16)
            dpg.add_table_column(init_width_or_weight=0.34)
            dpg.add_table_column(init_width_or_weight=0.22)
            dpg.add_table_column(init_width_or_weight=0.28)
            with dpg.table_row():
                rendering.add_text("ÂNGULO", "micro", config.TEXT_DIM)
                rendering.add_text("—", "small", config.TEXT, tag=_part(index, "angle"))
                rendering.add_text("Alvo", "micro", config.TEXT_DIM)
                rendering.add_text("—", "small", config.TEXT, tag=_part(index, "target"))
            with dpg.table_row():
                rendering.add_text("PWM", "micro", config.TEXT_DIM)
                rendering.add_text("—", "small", config.TEXT, tag=_part(index, "pwm"))
                rendering.add_text("Sentido", "micro", config.TEXT_DIM)
                rendering.add_text("", "small", config.TEXT_DIM, tag=_part(index, "direction"))
        dpg.bind_item_theme(_part(index, "header"), _TABLE_THEME)
        dpg.bind_item_theme(_part(index, "details"), _TABLE_THEME)
        dpg.add_progress_bar(tag=_part(index, "bar"), width=1, height=8, default_value=0.0)
        dpg.bind_item_theme(_part(index, "bar"), _BAR_THEME)
        rendering.add_plot(_part(index, "plot"), height=60, width=1)
        rendering.add_series(_part(index, "plot"), 0, config.VELOCITY_COLOR)
        rendering.add_series(_part(index, "plot"), 1, config.TARGET_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-config.CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=_part(index, "plot"),
            tag=_part(index, "placeholder"),
        )
        with dpg.group(horizontal=True):
            rendering.add_text("Velocidade / alvo · 12 s", "micro", config.TEXT_DIM)
            rendering.add_text("±600", "micro", config.TEXT_DIM, tag=_part(index, "scale"))


def _build_control(state):
    def on_adjust(_sender, _app_data, user_data):
        target = (
            state.manual_targets[state.selected_wheel] if state.demo else state.wheels[state.selected_wheel]["tgt"]
        )
        state.set_selected_target(target + float(user_data))

    def on_stop(_sender, _app_data, _user_data=None):
        state.stop_motors()

    def on_sweep(_sender, _app_data, _user_data=None):
        state.sweep = not state.sweep
        state.add_console("Varredura " + ("ligada" if state.sweep else "desligada"))

    def on_override(_sender, app_data, _user_data=None):
        state.set_motion_override(bool(app_data))

    with rendering.panel(f"Controle · M{state.selected_wheel + 1}", CONTROL_PANEL, width=CONTROL_WIDTH, height=1):
        rendering.add_text("Alvo da roda selecionada", "small", config.TEXT_DIM)
        rendering.add_text("— RPM", "big", config.ACCENT, tag=CONTROL_TARGET)
        rendering.add_text("Clique em uma roda ou pressione 1–4", "tiny", config.TEXT_DIM)
        dpg.add_separator()
        with dpg.group(horizontal=True):
            for delta in (-500, -100):
                dpg.add_button(
                    tag=ADJUST_BUTTONS[delta],
                    label=f"{delta:+d} RPM",
                    width=1,
                    height=BUTTON_HEIGHT,
                    callback=on_adjust,
                    user_data=delta,
                )
        with dpg.group(horizontal=True):
            for delta in (100, 500):
                dpg.add_button(
                    tag=ADJUST_BUTTONS[delta],
                    label=f"{delta:+d} RPM",
                    width=1,
                    height=BUTTON_HEIGHT,
                    callback=on_adjust,
                    user_data=delta,
                )
        dpg.add_separator()
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
        rendering.add_text(f"Varredura ±{config.SWEEP_RPM:.0f} RPM · a cada 1 s", "tiny", config.TEXT_DIM)
        rendering.add_text("↑/↓ ±100 · ←/→ ±500", "tiny", config.TEXT_DIM)
        dpg.add_separator()
        dpg.add_checkbox(
            tag=OVERRIDE_CHECK,
            label="Liberar movimento da bancada",
            default_value=False,
            callback=on_override,
        )
        rendering.add_text("Ignorar a inibição de movimento", "tiny", config.TEXT_DIM, tag=OVERRIDE_HINT)


def build(state):
    """Create the Motors tab contents once."""
    _create_themes()
    with dpg.group(horizontal=True):
        with dpg.group():
            for row in range(2):
                with dpg.group(horizontal=True):
                    _build_card(row * 2)
                    _build_card(row * 2 + 1)
        _build_control(state)


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

    plot = _part(index, "plot")
    with state.history_lock:
        samples = list(state.history[index])
    xs_velocity, ys_velocity = rendering.history_series(samples, 1, now)
    xs_target, ys_target = rendering.history_series(samples, 2, now)
    rendering.set_series(plot, 0, xs_velocity, ys_velocity)
    rendering.set_series(plot, 1, xs_target, ys_target)
    peak = max(
        MIN_SCALE_RPM,
        rendering.history_peak(samples, 1, now, config.CHART_WINDOW_S, MIN_SCALE_RPM),
        rendering.history_peak(samples, 2, now, config.CHART_WINDOW_S, MIN_SCALE_RPM),
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
    for tag in ADJUST_BUTTONS.values():
        dpg.configure_item(tag, width=(CONTROL_INNER - GAP) // 2)
    dpg.configure_item(STOP_BUTTON, width=CONTROL_INNER)
    dpg.configure_item(
        SWEEP_BUTTON,
        width=CONTROL_INNER,
        label=f"Varredura {'ligada' if state.sweep else 'desligada'} · W",
    )
    if state.sweep != _BOUND_SWEEP and _SWEEP_THEMES:
        dpg.bind_item_theme(SWEEP_BUTTON, _SWEEP_THEMES[1 if state.sweep else 0])
        _BOUND_SWEEP = state.sweep
    if dpg.get_value(OVERRIDE_CHECK) != state.motion_override:
        dpg.set_value(OVERRIDE_CHECK, state.motion_override)
    rendering.set_text(
        OVERRIDE_HINT,
        "Ativo · ignora a inibição de movimento"
        if state.motion_override
        else "Ignorar a inibição de movimento",
        config.DEMO_COLOR if state.motion_override else config.TEXT_DIM,
    )


def update(state, layout, now):
    """Refresh the Motors tab contents."""
    _apply_selection(state.selected_wheel)
    card_w = max(250, (layout["body_w"] - CONTROL_WIDTH - 2 * GAP) // 2)
    card_h = max(180, (layout["body_h"] - GAP) // 2)
    inner_w = card_w - CARD_INNER_MARGIN
    plot_h = max(40, card_h - CARD_FIXED_HEIGHT)
    for index, card in enumerate(CARD_TAGS):
        dpg.configure_item(card, width=card_w, height=card_h)
        dpg.configure_item(_part(index, "header"), width=inner_w)
        dpg.configure_item(_part(index, "details"), width=inner_w)
        dpg.configure_item(_part(index, "bar"), width=inner_w)
        dpg.configure_item(_part(index, "plot"), width=inner_w, height=plot_h)
        _update_card(state, index, now)
    dpg.configure_item(CONTROL_PANEL, width=CONTROL_WIDTH, height=layout["body_h"])
    _update_control(state, now)


def select_wheel_at(state, pos):
    """Select the wheel card under a screen position; returns True on a hit."""
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
