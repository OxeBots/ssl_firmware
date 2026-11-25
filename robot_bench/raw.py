"""Motors tab "Dados brutos" sub-tab: raw signals of one selected wheel."""

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto
from .state import WHEEL_NAMES

GAP = 8
PANEL_MARGIN = 34
MIN_PLOT_HEIGHT = 60
# Vertical space the raw view spends outside its three plots (selector row,
# value header, three captions, item gaps and panel padding). Measured at
# 238 px; 240 keeps the bottom plot clear of the panel at 1024x640 and 1366.
PLOT_RESERVED_HEIGHT = 240
ADC_MAX = 4095.0
# Smallest vertical span of the angle plot, in 12-bit counts, so a stationary
# wheel keeps a readable scale instead of zooming into the ADC noise.
MIN_ADC_SPAN = 512.0
I2C_INVALID = 0xFFFF
CHART_WINDOW_S = config.CHART_WINDOW_S

PANEL = "raw_panel"
SELECTOR_TAGS = tuple(f"raw_selector_{index}" for index in range(proto.WHEEL_COUNT))
# The three unselected wheels stay compact; the selected button takes the rest.
COMPACT_BUTTON_WIDTH = 92
VALUE_TAGS = tuple(
    tuple(f"raw_value_{row}_{column}" for column in range(6)) for row in range(2)
)
PLOT_ADC = "raw_plot_adc"
PLOT_FG = "raw_plot_fg"
PLOT_SPEED = "raw_plot_speed"
ADC_LEGEND = "raw_adc_legend"

STATUS_WAITING = "Aguardando dados"
STATUS_STALE = "Desatualizado"
STATUS_I2C = "I2C indisponível"
STATUS_RAW = "Bruto"

PLOT_ADC_COLOR = config.ACCENT
# The I2C raw angle must stand apart from the analog ADC trace on the shared plot.
PLOT_I2C_COLOR = config.PWM_COLOR
PLOT_FG_COLOR = config.PULSE_COLOR
PLOT_EKF_COLOR = config.TARGET_COLOR
PLOT_PULSE_COLOR = config.PULSE_COLOR
PLOT_CONTROL_COLOR = config.VELOCITY_COLOR

VALUE_LABELS = (
    ("ADC", "I2C", "I2C °", "Ângulo", "FG total", "ΔFG"),
    ("EKF RPM", "Pulso RPM", "Ctrl RPM", "PWM", "Sentido", "Status"),
)

_PANEL_THEME = None
_SELECTOR_THEMES = []
_BOUND_SELECTED = -1


def _panel_theme():
    global _PANEL_THEME
    if _PANEL_THEME is None:
        _PANEL_THEME = rendering.create_theme(
            colors={
                dpg.mvThemeCol_ChildBg: config.PANEL,
                dpg.mvThemeCol_Border: config.PANEL_BORDER,
                dpg.mvThemeCol_Text: config.TEXT,
            },
            styles={
                dpg.mvStyleVar_WindowPadding: (16.0, 10.0),
                dpg.mvStyleVar_ItemSpacing: (8.0, 6.0),
                dpg.mvStyleVar_ChildBorderSize: (1.0,),
                dpg.mvStyleVar_ChildRounding: (8.0,),
            },
        )
    return _PANEL_THEME


def wheel_label(index):
    """Full display name of one wheel."""
    return WHEEL_NAMES[index]


def selector_label(index, selected):
    """Wide full name for the selected wheel, short M-labels for the others."""
    return wheel_label(index) if selected else f"M{index + 1}"


def selector_widths(inner_w, selected):
    """Selector button widths: the selected wheel widens to fill the row."""
    compact = COMPACT_BUTTON_WIDTH
    wide = max(compact, int(inner_w) - (proto.WHEEL_COUNT - 1) * compact - 3 * GAP)
    return [wide if index == selected else compact for index in range(proto.WHEEL_COUNT)]


def _create_selector_themes():
    global _SELECTOR_THEMES
    if _SELECTOR_THEMES:
        return
    for background, border, text in (
        (config.INSET, config.PANEL_BORDER, config.TEXT_DIM),
        (config.TAB_ACTIVE, config.ACCENT, config.TEXT),
    ):
        _SELECTOR_THEMES.append(
            rendering.create_theme(
                colors={
                    dpg.mvThemeCol_Button: background,
                    dpg.mvThemeCol_ButtonHovered: config.TAB_ACTIVE,
                    dpg.mvThemeCol_ButtonActive: config.ACCENT,
                    dpg.mvThemeCol_Border: border,
                    dpg.mvThemeCol_Text: text,
                },
                styles={
                    dpg.mvStyleVar_FramePadding: (8.0, 4.0),
                    dpg.mvStyleVar_FrameBorderSize: (1.0,),
                    dpg.mvStyleVar_FrameRounding: (4.0,),
                },
                component=dpg.mvButton,
            )
        )


def sync_selection(state):
    """Sync the mux focus once the serial link can accept it."""
    state.select_raw_encoder(state.selected_wheel)


def on_show(state):
    """Focus the mux on the selected wheel when the raw view becomes active."""
    state.select_raw_encoder(state.selected_wheel)


def on_hide(state):
    """Release the mux focus when the raw view stops being visible."""
    state.select_raw_encoder(None)


def adc_endpoints(calibration, wheel):
    """Usable stored raw ADC endpoints (min, max) of one wheel, or None."""
    if calibration is None or not calibration["range_mask"] & (1 << wheel):
        return None
    minimum = calibration["adc_min_raw"][wheel]
    maximum = calibration["adc_max_raw"][wheel]
    if maximum <= minimum:
        return None
    return minimum, maximum


def adc_angle_counts(adc_raw, endpoints):
    """Map raw ADC counts to equivalent 12-bit encoder-angle counts, clamped."""
    if endpoints is None:
        return None
    minimum, maximum = endpoints
    ratio = (adc_raw - minimum) / (maximum - minimum)
    return min(1.0, max(0.0, ratio)) * ADC_MAX


def adc_plot_label(calibration, wheel):
    """Legend of the ADC trace: angle-equivalent, or raw while no range is stored."""
    if adc_endpoints(calibration, wheel) is None:
        return "ADC bruto (sem faixa)"
    return "ADC (ângulo)"


def raw_live(state, now):
    """True while the latest raw frame is recent enough to display."""
    return bool(state.last_raw_t) and now - state.last_raw_t <= config.RAW_STALE_S


def card_status(state, index, now):
    """Status text and color of the selected wheel."""
    if not state.last_raw_t:
        return STATUS_WAITING, config.TEXT_DIM
    if not raw_live(state, now):
        return STATUS_STALE, config.BAD_COLOR
    if state.raw is not None and state.raw["i2c_raw"][index] == I2C_INVALID:
        return STATUS_I2C, config.DEMO_COLOR
    return STATUS_RAW, config.OK_COLOR


def i2c_degrees(value):
    """AS5600 raw angle counts (0..4095) to degrees, or None when unreadable."""
    if value == I2C_INVALID:
        return None
    return value * 360.0 / 4096.0


def pwm_direction(pwm):
    """Motor direction implied by the signed applied duty."""
    if pwm > 0:
        return "Horário"
    if pwm < 0:
        return "Anti-horário"
    return "Parado"


def angle_text(state, index, now):
    """Analog encoder angle of one wheel, or a dash while the state is stale."""
    if not state.last_state_t or now - state.last_state_t > config.SENSOR_STALE_S:
        return "—"
    return f"{state.wheels[index]['angle']:.1f}°"


def raw_values(state, index, now):
    """Wide header cells of one wheel as two rows of six (label, value, color)."""
    raw = state.raw if raw_live(state, now) else None
    wheel = state.wheels[index]
    status, status_color = card_status(state, index, now)
    if raw is None:
        values = [
            [(label, "—", config.TEXT_DIM) for label in VALUE_LABELS[0]],
            [(label, "—", config.TEXT_DIM) for label in VALUE_LABELS[1][:5]]
            + [(VALUE_LABELS[1][5], status, status_color)],
        ]
        return values
    analog = raw["analog_raw"][index]
    i2c = raw["i2c_raw"][index]
    degrees = i2c_degrees(i2c)
    i2c_color = config.TEXT_DIM if degrees is None else PLOT_I2C_COLOR
    pulse_ok = bool(state.pulse_valid_mask & (1 << index))
    pwm = raw["applied_pwm"][index]
    return [
        [
            ("ADC", f"{analog}", config.ACCENT),
            ("I2C", "—" if degrees is None else f"{i2c}", i2c_color),
            ("I2C °", "—" if degrees is None else f"{degrees:.1f}°", i2c_color),
            ("Ângulo", angle_text(state, index, now), config.TEXT),
            ("FG total", f"{wheel['pulse_count']:,}", PLOT_PULSE_COLOR),
            ("ΔFG", f"{raw['pulse_delta'][index]}", PLOT_FG_COLOR),
        ],
        [
            ("EKF RPM", f"{raw['ekf_rpm'][index]:+.1f}", PLOT_EKF_COLOR),
            ("Pulso RPM", f"{wheel['pulse_rpm']:+.1f}" if pulse_ok else "—", PLOT_PULSE_COLOR),
            ("Ctrl RPM", f"{wheel['vel']:+.1f}", PLOT_CONTROL_COLOR),
            ("PWM", f"{pwm:+d}", config.PWM_COLOR if pwm else config.TEXT_DIM),
            ("Sentido", pwm_direction(pwm), config.PWM_COLOR if pwm else config.TEXT_DIM),
            ("Status", status, status_color),
        ],
    ]


def plot_height(body_h):
    """Height of each of the three raw plots for the given content height."""
    return max(MIN_PLOT_HEIGHT, (int(body_h) - PLOT_RESERVED_HEIGHT) // 3)


def _build_value_table():
    with dpg.table(header_row=False, policy=dpg.mvTable_SizingStretchProp):
        for _ in range(6):
            dpg.add_table_column(init_width_or_weight=1.0)
        for row in range(2):
            with dpg.table_row():
                for column in range(6):
                    with dpg.group():
                        rendering.add_text(VALUE_LABELS[row][column], "micro", config.TEXT_DIM)
                        rendering.add_text(
                            "—", "tiny", config.TEXT_DIM, tag=VALUE_TAGS[row][column]
                        )


def _apply_selector(state, inner_w):
    """Update the selector labels, widths and themes for the current wheel."""
    global _BOUND_SELECTED
    selected = state.selected_wheel
    if selected != _BOUND_SELECTED and len(_SELECTOR_THEMES) == 2:
        for index, tag in enumerate(SELECTOR_TAGS):
            dpg.bind_item_theme(tag, _SELECTOR_THEMES[1 if index == selected else 0])
            dpg.configure_item(tag, label=selector_label(index, index == selected))
        _BOUND_SELECTED = selected
    for tag, width in zip(SELECTOR_TAGS, selector_widths(inner_w, selected)):
        dpg.configure_item(tag, width=width)


def build(state):
    """Create the Dados brutos sub-tab contents once."""

    def on_select(_sender, _app_data, user_data=None):
        index = int(user_data)
        state.selected_wheel = index
        state.select_raw_encoder(index)

    _create_selector_themes()
    with dpg.child_window(tag=PANEL, width=1, height=1, no_scrollbar=True):
        dpg.bind_item_theme(PANEL, _panel_theme())
        # Segmented selector: the selected motor gets the wide, accent button.
        with dpg.group(horizontal=True):
            for index in range(proto.WHEEL_COUNT):
                button = dpg.add_button(
                    tag=SELECTOR_TAGS[index],
                    label=selector_label(index, index == 0),
                    width=COMPACT_BUTTON_WIDTH,
                    callback=on_select,
                    user_data=index,
                )
                rendering.bind_font(button, "small")
        _build_value_table()
        with dpg.group(horizontal=True):
            rendering.add_text(
                adc_plot_label(state.calibration, 0), "micro", PLOT_ADC_COLOR, tag=ADC_LEGEND
            )
            rendering.add_text("/ I2C (ângulo)", "micro", PLOT_I2C_COLOR)
        rendering.add_plot(PLOT_ADC, height=80, width=1)
        rendering.add_series(PLOT_ADC, 0, PLOT_ADC_COLOR)
        rendering.add_series(PLOT_ADC, 1, PLOT_I2C_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=PLOT_ADC,
            tag=PLOT_ADC + "_placeholder",
        )
        rendering.add_text("Δ FG · contagens/100 ms", "micro", PLOT_FG_COLOR)
        rendering.add_plot(PLOT_FG, height=80, width=1)
        rendering.add_series(PLOT_FG, 0, PLOT_FG_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=PLOT_FG,
            tag=PLOT_FG + "_placeholder",
        )
        with dpg.group(horizontal=True):
            rendering.add_text("Velocidade · EKF", "micro", PLOT_EKF_COLOR)
            rendering.add_text("/ pulso", "micro", PLOT_PULSE_COLOR)
            rendering.add_text("/ controle", "micro", PLOT_CONTROL_COLOR)
        rendering.add_plot(PLOT_SPEED, height=80, width=1)
        rendering.add_series(PLOT_SPEED, 0, PLOT_EKF_COLOR)
        rendering.add_series(PLOT_SPEED, 1, PLOT_PULSE_COLOR)
        rendering.add_series(PLOT_SPEED, 2, PLOT_CONTROL_COLOR)
        dpg.add_plot_annotation(
            label="Aguardando dados",
            default_value=(-CHART_WINDOW_S / 2.0, 0.0),
            color=rendering.rgba(config.TEXT_DIM),
            parent=PLOT_SPEED,
            tag=PLOT_SPEED + "_placeholder",
        )


def update(state, layout, now):
    """Refresh the raw view of the selected wheel."""
    sync_selection(state)
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    inner_w = body_w - PANEL_MARGIN
    dpg.configure_item(PANEL, width=body_w, height=body_h)
    _apply_selector(state, inner_w)
    index = state.selected_wheel
    values = raw_values(state, index, now)
    for row in range(2):
        for column in range(6):
            _label, value, color = values[row][column]
            rendering.set_text(VALUE_TAGS[row][column], value, color)
    with state.history_lock:
        raw_samples = list(state.raw_history[index])
        state_samples = list(state.history[index])
    plot_h = plot_height(body_h)
    dpg.configure_item(PLOT_ADC, width=inner_w, height=plot_h)
    dpg.configure_item(PLOT_FG, width=inner_w, height=plot_h)
    dpg.configure_item(PLOT_SPEED, width=inner_w, height=plot_h)
    endpoints = adc_endpoints(state.calibration, index)
    rendering.set_text(ADC_LEGEND, adc_plot_label(state.calibration, index))
    if endpoints is None:
        adc_samples = raw_samples
    else:
        adc_samples = [
            (
                sample[0],
                adc_angle_counts(sample[1], endpoints),
                sample[2],
                sample[3],
                sample[4],
                sample[5],
            )
            for sample in raw_samples
        ]
    xs_adc, ys_adc = rendering.history_series(adc_samples, 1, now, CHART_WINDOW_S)
    xs_i2c, ys_i2c = rendering.history_series(raw_samples, 2, now, CHART_WINDOW_S)
    rendering.set_series(PLOT_ADC, 0, xs_adc, ys_adc)
    rendering.set_series(PLOT_ADC, 1, xs_i2c, ys_i2c)
    adc_low, adc_high = rendering.series_bounds((ys_adc, ys_i2c), MIN_ADC_SPAN)
    rendering.set_y_limits(PLOT_ADC, adc_low, adc_high)
    dpg.configure_item(PLOT_ADC + "_placeholder", show=not xs_adc)
    xs_fg, ys_fg = rendering.history_series(raw_samples, 3, now, CHART_WINDOW_S)
    rendering.set_series(PLOT_FG, 0, xs_fg, ys_fg)
    fg_peak = max(1.0, rendering.history_peak(raw_samples, 3, now, CHART_WINDOW_S, 1.0))
    rendering.set_y_limits(PLOT_FG, 0.0, fg_peak)
    dpg.configure_item(PLOT_FG + "_placeholder", show=not xs_fg)
    xs_ekf, ys_ekf = rendering.history_series(raw_samples, 4, now, CHART_WINDOW_S)
    xs_pulse, ys_pulse = rendering.history_series(state_samples, 3, now, CHART_WINDOW_S)
    xs_control, ys_control = rendering.history_series(state_samples, 1, now, CHART_WINDOW_S)
    rendering.set_series(PLOT_SPEED, 0, xs_ekf, ys_ekf)
    rendering.set_series(PLOT_SPEED, 1, xs_pulse, ys_pulse)
    rendering.set_series(PLOT_SPEED, 2, xs_control, ys_control)
    speed_peak = max(
        10.0,
        rendering.history_peak(raw_samples, 4, now, CHART_WINDOW_S, 10.0),
        rendering.history_peak(state_samples, 3, now, CHART_WINDOW_S, 10.0),
        rendering.history_peak(state_samples, 1, now, CHART_WINDOW_S, 10.0),
    )
    rendering.set_y_limits(PLOT_SPEED, -speed_peak, speed_peak)
    dpg.configure_item(PLOT_SPEED + "_placeholder", show=not xs_ekf)
