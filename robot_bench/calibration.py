"""Motor/encoder wiring calibration tab and the stored wiring map."""

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto
from .state import WHEEL_NAMES

GAP = 8
RIGHT_WIDTH = 330
PANEL_MARGIN = 34
BUTTON_HEIGHT = 32
BAR_HEIGHT = 10
# Level peak floor (ADC / encoder angle counts): sensor noise must not fill the bars.
LEVEL_FLOOR = 200.0
CONFIRM_LABEL = "Robô levantado · rodas livres"
START_LABEL = "Calibrar motores"
STOP_LABEL = "Parar calibração"

PANEL = "calibration_panel"
MAPPING_PANEL = "calibration_mapping_panel"
CONFIRM_CHECK = "calibration_confirm"
START_BUTTON = "calibration_start"
STOP_BUTTON = "calibration_stop"
STATUS_TEXT = "calibration_status"
PROGRESS_BAR = "calibration_progress"
SPINNING_TEXT = "calibration_spinning"
LEVELS_TABLE = "calibration_levels_table"
RESULTS_TABLE = "calibration_results_table"
MAPPING_HEADER = "calibration_mapping_header"
MAPPING_TABLE = "calibration_mapping_table"
ANALOG_BARS = tuple(f"calibration_analog_bar_{slot}" for slot in range(proto.WHEEL_COUNT))
I2C_BARS = tuple(f"calibration_i2c_bar_{slot}" for slot in range(proto.WHEEL_COUNT))
RESULT_MAPPING = tuple(f"calibration_result_mapping_{wheel}" for wheel in range(proto.WHEEL_COUNT))
RESULT_STATUS = tuple(f"calibration_result_status_{wheel}" for wheel in range(proto.WHEEL_COUNT))
MAPPING_ANALOG = tuple(f"calibration_map_analog_{wheel}" for wheel in range(proto.WHEEL_COUNT))
MAPPING_MUX = tuple(f"calibration_map_mux_{wheel}" for wheel in range(proto.WHEEL_COUNT))
MAPPING_RANGE = tuple(f"calibration_map_range_{wheel}" for wheel in range(proto.WHEEL_COUNT))

RESULT_BOTH = "Ambos os caminhos"
RESULT_ANALOG = "Só analógico"
RESULT_FAILED = "Falhou"
RESULT_PENDING = "Pendente"
RESULT_STORED = "Salvo na NVS"

CALIBRATION_ERROR_NAMES = {
    proto.CALIBRATION_ERROR_NONE: "Sem erro",
    proto.CALIBRATION_ERROR_NO_MOVEMENT: "Sem movimento detectado",
    proto.CALIBRATION_ERROR_AMBIGUOUS_ANALOG: "Sinal analógico ambíguo",
    proto.CALIBRATION_ERROR_AMBIGUOUS_I2C: "Sinal I2C ambíguo",
    proto.CALIBRATION_ERROR_BAD_MAPPING: "Mapeamento inválido",
    proto.CALIBRATION_ERROR_ABORTED: "Cancelada",
    proto.CALIBRATION_ERROR_I2C: "Falha de I2C",
}

_TABLE_THEME = None
_ANALOG_BAR_THEME = None
_I2C_BAR_THEME = None
_PROGRESS_THEME = None
_START_THEME = None
_STOP_THEME = None


# ---------------------------------------------------------------------------
# Pure helpers: freshness, button rules, text and bar fractions
# ---------------------------------------------------------------------------


def calibration_fresh(state, now):
    """True while the latest calibration frame is recent enough to use."""
    return bool(state.last_calibration_t) and now - state.last_calibration_t <= config.CALIBRATION_STALE_S


def calibration_running(state, now):
    """True while a fresh calibration frame reports a running routine."""
    return calibration_fresh(state, now) and state.calibration["state"] == proto.CALIBRATION_RUNNING


def demo_calibration_active(state):
    """True while a demo run is active before its first running frame arrives."""
    return bool(state.demo and state.demo_calibration_start is not None)


def calibration_start_enabled(state, now):
    """Calibrar button: confirmed, live link, fresh frame and routine idle."""
    if not state.calibration_confirm or not calibration_fresh(state, now):
        return False
    if calibration_running(state, now) or demo_calibration_active(state):
        return False
    if state.demo:
        return True
    return bool(
        state.connected
        and state.streaming
        and state.last_state_t
        and now - state.last_state_t <= config.SENSOR_STALE_S
    )


def calibration_stop_enabled(state, now):
    """Parar button: only while the routine (or the demo run) is active."""
    return calibration_running(state, now) or demo_calibration_active(state)


def level_ratios(levels):
    """Bar fractions for one level column, scaled to the peak with a noise floor."""
    peak = max([LEVEL_FLOOR] + [abs(level) for level in levels])
    return [min(1.0, abs(level) / peak) for level in levels]


def analog_text(calib, wheel):
    """Analog input slot of one wheel, named like the connector (ENC1..ENC4)."""
    if calib is None:
        return "—"
    return f"ENC{calib['analog_map'][wheel] + 1}"


def mux_text(calib, wheel):
    """CD4052B mux channel of one wheel (encoder pair M1..M4)."""
    if calib is None:
        return "—"
    return f"mux {calib['mux_map'][wheel] + 1}"


def mapping_text(calib, wheel):
    """Combined analog and I2C mapping of one wheel."""
    return f"{analog_text(calib, wheel)} · {mux_text(calib, wheel)}" if calib is not None else "—"


def range_text(calib, wheel):
    """Stored analog range status of the encoder mapped to one wheel."""
    if calib is None:
        return "—"
    return "pronto" if calib["range_mask"] & (1 << wheel) else "sem faixa"


def mapping_header(calib):
    """Header wording and color for the stored map panel."""
    if calib is None:
        return "Aguardando quadro de calibração", config.TEXT_DIM
    if calib["mapping_valid"]:
        return "Mapa calibrado · gravado na NVS", config.OK_COLOR
    return "Mapa padrão · não calibrado", config.BAD_COLOR


def error_text(code):
    """Portuguese description of a DebugCalibration failure reason."""
    return CALIBRATION_ERROR_NAMES.get(code, f"Código {code}")


def result_text(calib, wheel):
    """Detection result for one wheel."""
    if calib is None:
        return "Aguardando"
    if calib["state"] == proto.CALIBRATION_IDLE and calib["mapping_valid"]:
        # A previous run stored the map in NVS: every wheel is already mapped.
        return RESULT_STORED
    bit = 1 << wheel
    if calib["result_mask"] & bit:
        return RESULT_BOTH if calib["i2c_ready"] & bit else RESULT_ANALOG
    if calib["state"] in (proto.CALIBRATION_DONE, proto.CALIBRATION_FAILED):
        return RESULT_FAILED
    return RESULT_PENDING


def result_color(calib, wheel):
    """Palette color matching one detection result."""
    text = result_text(calib, wheel)
    if text == RESULT_BOTH:
        return config.OK_COLOR
    if text == RESULT_ANALOG:
        return config.DEMO_COLOR
    if text == RESULT_FAILED:
        return config.BAD_COLOR
    if text == RESULT_STORED:
        return config.OK_COLOR
    return config.TEXT_DIM


def wheel_rows(calib):
    """Per-wheel display rows for the result table."""
    return [
        {
            "mapping": mapping_text(calib, wheel),
            "result": result_text(calib, wheel),
            "color": result_color(calib, wheel),
        }
        for wheel in range(proto.WHEEL_COUNT)
    ]


def status_text(state, now):
    """Status line and color for the calibration routine."""
    calib = state.calibration
    if not state.last_calibration_t:
        return "Aguardando quadro de calibração", config.TEXT_DIM
    if not calibration_fresh(state, now):
        return "Dados de calibração desatualizados", config.BAD_COLOR
    if calib["state"] == proto.CALIBRATION_RUNNING:
        return f"Calibrando · {calib['progress']}% concluído", config.ACCENT
    if calib["state"] == proto.CALIBRATION_DONE:
        return "Calibração concluída · mapa salvo na NVS", config.OK_COLOR
    if calib["state"] == proto.CALIBRATION_FAILED:
        return f"Falha · {error_text(calib['error_code'])}", config.BAD_COLOR
    return "Ociosa · confirme o aviso de segurança para calibrar", config.TEXT_DIM


def spinning_text(calib):
    """Which wheel is spinning, or the routine outcome."""
    if calib is None:
        return "Nenhum motor girando"
    if calib["state"] == proto.CALIBRATION_RUNNING:
        return f"Girando {WHEEL_NAMES[min(calib['wheel'], proto.WHEEL_COUNT - 1)]}"
    if calib["state"] == proto.CALIBRATION_DONE:
        return "Rotina concluída · motores parados"
    if calib["state"] == proto.CALIBRATION_FAILED:
        return "Rotina interrompida · motores parados"
    return "Nenhum motor girando"


# ---------------------------------------------------------------------------
# Widget construction
# ---------------------------------------------------------------------------


def _create_themes():
    global _TABLE_THEME, _ANALOG_BAR_THEME, _I2C_BAR_THEME, _PROGRESS_THEME, _START_THEME, _STOP_THEME
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (2.0, 1.0)},
        component=dpg.mvTable,
    )
    _ANALOG_BAR_THEME = rendering.create_theme(
        colors={dpg.mvThemeCol_PlotHistogram: config.ACCENT, dpg.mvThemeCol_FrameBg: config.INSET},
        component=dpg.mvProgressBar,
    )
    _I2C_BAR_THEME = rendering.create_theme(
        colors={dpg.mvThemeCol_PlotHistogram: config.VELOCITY_COLOR, dpg.mvThemeCol_FrameBg: config.INSET},
        component=dpg.mvProgressBar,
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


def _build_levels_table():
    """Analog and I2C movement per encoder input slot."""
    with dpg.group():
        rendering.add_text("Detecção ao vivo", "small", config.TEXT_DIM)
        with dpg.table(tag=LEVELS_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.16)
            dpg.add_table_column(init_width_or_weight=0.42)
            dpg.add_table_column(init_width_or_weight=0.42)
            with dpg.table_row():
                rendering.add_text("Entrada", "micro", config.TEXT_DIM)
                rendering.add_text("Analógico", "micro", config.TEXT_DIM)
                rendering.add_text("I2C (mux)", "micro", config.TEXT_DIM)
            for slot in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"ENC{slot + 1}", "tiny", config.TEXT_DIM)
                    dpg.add_progress_bar(tag=ANALOG_BARS[slot], default_value=0.0, width=100, height=BAR_HEIGHT)
                    dpg.add_progress_bar(tag=I2C_BARS[slot], default_value=0.0, width=100, height=BAR_HEIGHT)
    dpg.bind_item_theme(LEVELS_TABLE, _TABLE_THEME)
    for tag in ANALOG_BARS:
        dpg.bind_item_theme(tag, _ANALOG_BAR_THEME)
    for tag in I2C_BARS:
        dpg.bind_item_theme(tag, _I2C_BAR_THEME)


def _build_results_table():
    """Detected mapping and detection outcome per wheel."""
    with dpg.group():
        rendering.add_text("Resultado por roda", "small", config.TEXT_DIM)
        with dpg.table(tag=RESULTS_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.14)
            dpg.add_table_column(init_width_or_weight=0.40)
            dpg.add_table_column(init_width_or_weight=0.46)
            with dpg.table_row():
                rendering.add_text("Roda", "micro", config.TEXT_DIM)
                rendering.add_text("Mapeamento", "micro", config.TEXT_DIM)
                rendering.add_text("Resultado", "micro", config.TEXT_DIM)
            for wheel in range(proto.WHEEL_COUNT):
                with dpg.table_row():
                    rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=RESULT_MAPPING[wheel])
                    rendering.add_text("Pendente", "tiny", config.TEXT_DIM, tag=RESULT_STATUS[wheel])
    dpg.bind_item_theme(RESULTS_TABLE, _TABLE_THEME)


def _build_mapping_table():
    """Stored wheel -> analog input / I2C mux channel mapping."""
    with dpg.table(tag=MAPPING_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
        dpg.add_table_column(init_width_or_weight=0.16)
        dpg.add_table_column(init_width_or_weight=0.28)
        dpg.add_table_column(init_width_or_weight=0.26)
        dpg.add_table_column(init_width_or_weight=0.30)
        with dpg.table_row():
            for label in ("Roda", "Analógico", "I2C", "Faixa"):
                rendering.add_text(label, "micro", config.TEXT_DIM)
        for wheel in range(proto.WHEEL_COUNT):
            with dpg.table_row():
                rendering.add_text(f"M{wheel + 1}", "tiny", config.TEXT)
                rendering.add_text("—", "tiny", config.TEXT_DIM, tag=MAPPING_ANALOG[wheel])
                rendering.add_text("—", "tiny", config.TEXT_DIM, tag=MAPPING_MUX[wheel])
                rendering.add_text("—", "tiny", config.TEXT_DIM, tag=MAPPING_RANGE[wheel])
    dpg.bind_item_theme(MAPPING_TABLE, _TABLE_THEME)


def build(state):
    """Create the Calibração tab contents once."""
    _create_themes()

    def on_confirm(_sender, app_data, _user_data=None):
        state.set_calibration_confirm(bool(app_data))

    def on_start(_sender, _app_data, _user_data=None):
        state.start_calibration()

    def on_stop(_sender, _app_data, _user_data=None):
        state.stop_calibration()

    with dpg.group(horizontal=True):
        with rendering.panel("Calibração dos motores", PANEL, width=1, height=1):
            rendering.add_text(
                "Gira um motor por vez em malha aberta e mede cada entrada de encoder",
                "small",
                config.TEXT_DIM,
            )
            rendering.add_text(
                "(analógica e I2C). O mapa detectado vai para a NVS e é aplicado na hora.",
                "small",
                config.TEXT_DIM,
            )
            rendering.add_text("PERIGO: levante o robô e mantenha as rodas livres.", "small", config.BAD_COLOR)
            rendering.add_text(
                "As rodas vão girar; não toque no robô durante a rotina.", "small", config.BAD_COLOR
            )
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
            rendering.add_text("Aguardando quadro de calibração", "small", config.TEXT_DIM, tag=STATUS_TEXT)
            dpg.add_progress_bar(tag=PROGRESS_BAR, default_value=0.0, width=200, height=BAR_HEIGHT)
            dpg.bind_item_theme(PROGRESS_BAR, _PROGRESS_THEME)
            rendering.add_text("Nenhum motor girando", "small", config.TEXT_DIM, tag=SPINNING_TEXT)
            with dpg.group(horizontal=True):
                _build_levels_table()
                _build_results_table()
        with rendering.panel("Mapeamento atual", MAPPING_PANEL, width=RIGHT_WIDTH, height=1):
            rendering.add_text("Aguardando quadro de calibração", "small", config.TEXT_DIM, tag=MAPPING_HEADER)
            rendering.add_text("ENCn: entrada analógica (caminho de controle)", "micro", config.TEXT_DIM)
            rendering.add_text("mux n: par I2C atrás do CD4052B", "micro", config.TEXT_DIM)
            dpg.add_spacer(height=4)
            _build_mapping_table()
            dpg.add_separator()
            rendering.add_text("O mapa é gravado na NVS e sobrevive a reinicializações.", "micro", config.TEXT_DIM)


# ---------------------------------------------------------------------------
# Per-frame updates
# ---------------------------------------------------------------------------


def _update_controls(state, now, inner_w):
    dpg.configure_item(
        START_BUTTON,
        width=(inner_w - GAP) // 2,
        enabled=calibration_start_enabled(state, now),
    )
    dpg.configure_item(
        STOP_BUTTON,
        width=(inner_w - GAP) // 2,
        enabled=calibration_stop_enabled(state, now),
    )
    dpg.configure_item(PROGRESS_BAR, width=inner_w)
    if dpg.get_value(CONFIRM_CHECK) != state.calibration_confirm:
        dpg.set_value(CONFIRM_CHECK, state.calibration_confirm)
    status, color = status_text(state, now)
    rendering.set_text(STATUS_TEXT, status, color)
    calib = state.calibration if calibration_fresh(state, now) else None
    dpg.set_value(PROGRESS_BAR, calib["progress"] / 100.0 if calib is not None else 0.0)
    running = calibration_running(state, now) or demo_calibration_active(state)
    rendering.set_text(
        SPINNING_TEXT,
        spinning_text(calib),
        config.ACCENT if running else config.TEXT_DIM,
    )


def _update_levels(state, now, inner_w):
    levels_w = (inner_w - GAP) * 55 // 100
    results_w = inner_w - GAP - levels_w
    dpg.configure_item(LEVELS_TABLE, width=levels_w)
    dpg.configure_item(RESULTS_TABLE, width=results_w)
    bar_w = max(40, levels_w * 42 // 100 - 14)
    calib = state.calibration
    if calibration_running(state, now):
        analog_ratios = level_ratios(calib["analog_level"])
        i2c_ratios = level_ratios(calib["i2c_level"])
    else:
        analog_ratios = i2c_ratios = [0.0] * proto.WHEEL_COUNT
    for slot in range(proto.WHEEL_COUNT):
        dpg.configure_item(ANALOG_BARS[slot], width=bar_w)
        dpg.set_value(ANALOG_BARS[slot], analog_ratios[slot])
        dpg.configure_item(I2C_BARS[slot], width=bar_w)
        dpg.set_value(I2C_BARS[slot], i2c_ratios[slot])
    for wheel, row in enumerate(wheel_rows(calib)):
        rendering.set_text(
            RESULT_MAPPING[wheel],
            row["mapping"],
            config.TEXT if calib is not None else config.TEXT_DIM,
        )
        rendering.set_text(RESULT_STATUS[wheel], row["result"], row["color"])


def _update_mapping(state):
    calib = state.calibration
    header, color = mapping_header(calib)
    rendering.set_text(MAPPING_HEADER, header, color)
    dpg.configure_item(MAPPING_TABLE, width=RIGHT_WIDTH - PANEL_MARGIN)
    for wheel in range(proto.WHEEL_COUNT):
        text_color = config.TEXT if calib is not None else config.TEXT_DIM
        rendering.set_text(MAPPING_ANALOG[wheel], analog_text(calib, wheel), text_color)
        rendering.set_text(MAPPING_MUX[wheel], mux_text(calib, wheel), text_color)
        stored_range = range_text(calib, wheel)
        if stored_range == "pronto":
            range_color = config.OK_COLOR
        elif stored_range == "sem faixa":
            range_color = config.DEMO_COLOR
        else:
            range_color = config.TEXT_DIM
        rendering.set_text(MAPPING_RANGE[wheel], stored_range, range_color)


def update(state, layout, now):
    """Refresh the Calibração tab from the latest calibration frame."""
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    main_w = max(320, body_w - RIGHT_WIDTH - GAP)
    dpg.configure_item(PANEL, width=main_w, height=body_h)
    dpg.configure_item(MAPPING_PANEL, width=RIGHT_WIDTH, height=body_h)
    inner_w = main_w - PANEL_MARGIN
    _update_controls(state, now, inner_w)
    _update_levels(state, now, inner_w)
    _update_mapping(state)
