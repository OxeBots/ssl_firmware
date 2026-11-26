"""Radio tab: telemetry and commands over the nRF24L01 adapter."""

import ctypes
import time

import dearpygui.dearpygui as dpg
import dearpygui._dearpygui as dpg_core

from . import config, radio, rendering

GAP = 8
HEADER_HEIGHT = 30
BUTTON_HEIGHT = 30
PANEL_MARGIN = 32
MESSAGES_LINES = 10
INT16_MIN = -32768
INT16_MAX = 32767
DISCONNECT_WIDTH = 130
KICK_INPUT_WIDTH = 70
NEW_ID_INPUT_WIDTH = 70
CONFIRM_LABEL = "Robô levantado · rodas livres"
RESET_CONFIRM_LABEL = "Confirmar reset de fábrica"
TELEMETRY_LABELS = (
    "Robô",
    "Bateria",
    "Kicker",
    "Yaw",
    "Posição",
    "Velocidade",
    "Avisos",
    "RTT",
    "Quadros",
)
MOTION_LABELS = ("x (mm)", "y (mm)", "ângulo", "vx", "vy", "w")

RADIO_TITLE = "radio_title"
RADIO_STATUS = "radio_status"
RADIO_HEADER_SPACER = "radio_header_spacer"
RADIO_DISCONNECT_BUTTON = "radio_disconnect"

RADIO_TELEMETRY_PANEL = "radio_telemetry_panel"
RADIO_TELEMETRY_TABLE = "radio_telemetry_table"
RADIO_TELEMETRY_VALUES = tuple(f"radio_telemetry_value_{index}" for index in range(len(TELEMETRY_LABELS)))

RADIO_COMMAND_PANEL = "radio_command_panel"
RADIO_DRIVE_PANEL = "radio_drive_panel"
RADIO_DRIVE_TABS = "radio_drive_tabs"
RADIO_KEYBOARD_TAB = "radio_keyboard_tab"
RADIO_KEYBOARD_ENABLE = "radio_keyboard_enable"
RADIO_KEYBOARD_SPEED = "radio_keyboard_speed"
RADIO_KEYBOARD_STATUS = "radio_keyboard_status"
RADIO_TARGET_INPUT = "radio_target_input"
RADIO_COMMAND_UNITS = "radio_command_units"
RADIO_MOTION_TABLE = "radio_motion_table"
RADIO_MOTION_INPUTS = tuple(f"radio_motion_input_{index}" for index in range(len(MOTION_LABELS)))
RADIO_SEND_BUTTON = "radio_send_motion"
RADIO_STOP_BUTTON = "radio_stop"
RADIO_KICK_INPUT = "radio_kick_input"
RADIO_KICK_RANGE = "radio_kick_range"
RADIO_KICK_BUTTON = "radio_kick"

RADIO_CONFIG_PANEL = "radio_config_panel"
RADIO_CONFIG_WARNING = "radio_config_warning"
RADIO_SAFETY_CHECK = "radio_safety_check"
RADIO_CONTROL_CAL_BUTTON = "radio_control_calibration"
RADIO_MAG_CAL_BUTTON = "radio_mag_calibration"
RADIO_NEW_ID_INPUT = "radio_new_id_input"
RADIO_SET_ID_BUTTON = "radio_set_id"
RADIO_RESET_CHECK = "radio_reset_check"
RADIO_RESET_BUTTON = "radio_reset"
RADIO_CHANNEL_INPUT = "radio_channel_input"
RADIO_SET_CHANNEL_BUTTON = "radio_set_channel"
RADIO_SCAN_BUTTON = "radio_scan_spectrum"
RADIO_BASE_STATUS = "radio_base_status"

RADIO_MESSAGES_PANEL = "radio_messages_panel"
RADIO_MESSAGES_TABLE = "radio_messages_table"
RADIO_MESSAGES_EMPTY = "radio_messages_empty"
RADIO_MESSAGE_LINES = tuple(f"radio_message_{index}" for index in range(MESSAGES_LINES))

_TABLE_THEME = None
_TELEMETRY_THEME = None
_MESSAGES_THEME = None
_ACTION_THEME = None
_STOP_THEME = None
_TEXT_WIDTHS = {}
# Session shown by the panel; only the target-id callback needs it outside
# update(), because build() runs once at startup (before the first connection).
_session = None
_keyboard_armed = False
_keyboard_neutral = False
_keyboard_focus_lost = False
# A refocused window keeps the drive blocked until the key state stays empty
# for this long; a key held through the focus change repeats within this window.
FOCUS_REGAIN_GRACE_S = 0.25
_focus_empty_since = 0.0
_DRIVE_KEYS = (
    dpg.mvKey_W, dpg.mvKey_S, dpg.mvKey_A, dpg.mvKey_D,
    dpg.mvKey_Up, dpg.mvKey_Down, dpg.mvKey_Left, dpg.mvKey_Right,
    dpg.mvKey_Q, dpg.mvKey_E, dpg.mvKey_Spacebar,
)
# Dear PyGui draws through GLFW. Its focus flag is the only signal that the OS
# window lost focus: without it, keys still held when the window comes back
# would resume the radio drive without a new key press.
GLFW_FOCUSED = 0x00020001
_focus_library = None
_focus_checked = False


# ---------------------------------------------------------------------------
# Pure helpers: link status and telemetry rows
# ---------------------------------------------------------------------------


def status_text(snapshot):
    """Link state line and color for the header."""
    if snapshot["live"]:
        return f"Ao vivo · {snapshot['port']}", config.OK_COLOR
    if snapshot["connected"] and snapshot["rx_count"] > 0:
        return "Conectado · sem resposta", config.BAD_COLOR
    if snapshot["connected"]:
        return "Conectado · aguardando telemetria", config.TEXT_DIM
    return "Desconectado · reconectando", config.BAD_COLOR


def telemetry_rows(snapshot):
    """Portuguese label/value rows for the telemetry panel."""
    telemetry = snapshot["telemetry"]
    if telemetry is None:
        return [(label, "—") for label in TELEMETRY_LABELS]
    errors = radio.error_flag_names(telemetry["error_flags"])
    rtt = f"{snapshot['rtt_ms']:.1f} ms" if snapshot["rx_count"] else "—"
    velocity = (
        f"vx {telemetry['vx_mm_s']} · vy {telemetry['vy_mm_s']} mm/s"
        f" · w {telemetry['w_rad_s']:.2f} rad/s"
    )
    return [
        ("Robô", str(telemetry["robot_id"])),
        ("Bateria", f"{telemetry['battery_pct']} %"),
        ("Kicker", f"{telemetry['kicker_voltage_v']:.2f} V"),
        ("Yaw", f"{telemetry['angle_deg']:.2f}°"),
        ("Posição", f"x {telemetry['x_mm']} mm · y {telemetry['y_mm']} mm"),
        ("Velocidade", velocity),
        ("Avisos", ", ".join(errors) if errors else "nenhum"),
        ("RTT", rtt),
        ("Quadros", f"RX {snapshot['rx_count']} · TX {snapshot['tx_count']}"),
    ]


# ---------------------------------------------------------------------------
# Widget construction
# ---------------------------------------------------------------------------


def _text_width(text, font_name):
    """Rendered width of one interface text, measured once per scale."""
    key = (text, font_name, rendering.ui_scale())
    width = _TEXT_WIDTHS.get(key)
    if width is not None:
        return width
    font = rendering.font(font_name)
    measured = dpg.get_text_size(text, font=font) if font is not None else dpg.get_text_size(text)
    if not measured:
        return len(text) * rendering.pxf(7.0)
    _TEXT_WIDTHS[key] = width = float(measured[0])
    return width


def _create_themes():
    global _TABLE_THEME, _TELEMETRY_THEME, _MESSAGES_THEME, _ACTION_THEME, _STOP_THEME
    if _TABLE_THEME is not None:
        return
    _TABLE_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (float(rendering.px(2)), float(rendering.px(1)))},
        component=dpg.mvTable,
    )
    _TELEMETRY_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (float(rendering.px(2)), float(rendering.px(2)))},
        component=dpg.mvTable,
    )
    _MESSAGES_THEME = rendering.create_theme(
        styles={dpg.mvStyleVar_CellPadding: (float(rendering.px(2)), 0.0)},
        component=dpg.mvTable,
    )
    _ACTION_THEME = rendering.create_theme(
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


def _build_header(on_disconnect):
    with dpg.group(horizontal=True):
        rendering.add_text("Rádio nRF24L01", "title", config.TEXT, tag=RADIO_TITLE)
        dpg.add_spacer(tag=RADIO_HEADER_SPACER, width=rendering.px(8))
        rendering.add_text("", "small", config.TEXT_DIM, tag=RADIO_STATUS)
        dpg.add_button(
            tag=RADIO_DISCONNECT_BUTTON,
            label="Desconectar",
            width=rendering.px(DISCONNECT_WIDTH),
            height=rendering.px(BUTTON_HEIGHT),
            callback=on_disconnect,
        )


def _build_telemetry_panel():
    with rendering.panel("Telemetria", RADIO_TELEMETRY_PANEL, width=1, height=1):
        with dpg.table(tag=RADIO_TELEMETRY_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=0.22)
            dpg.add_table_column(init_width_or_weight=0.78)
            for index, label in enumerate(TELEMETRY_LABELS):
                with dpg.table_row():
                    rendering.add_text(label, "tiny", config.TEXT_DIM)
                    rendering.add_text("—", "small", config.TEXT_DIM, tag=RADIO_TELEMETRY_VALUES[index])
        dpg.bind_item_theme(RADIO_TELEMETRY_TABLE, _TELEMETRY_THEME)


def _build_target_row(on_target_changed, on_stop):
    with dpg.group(horizontal=True):
        rendering.add_text("Robô alvo", "small", config.TEXT_DIM)
        dpg.add_input_int(
            tag=RADIO_TARGET_INPUT,
            default_value=0,
            width=rendering.px(80),
            min_value=0,
            max_value=radio.ROBOT_ID_MAX,
            min_clamped=True,
            max_clamped=True,
            step=0,
            callback=on_target_changed,
        )
        dpg.add_button(
            tag=RADIO_STOP_BUTTON,
            label="Parar [Espaço]",
            width=rendering.px(150),
            height=rendering.px(BUTTON_HEIGHT),
            callback=on_stop,
        )
        dpg.bind_item_theme(RADIO_STOP_BUTTON, _STOP_THEME)


def _build_keyboard_tab():
    with dpg.tab(label="Teclado", tag=RADIO_KEYBOARD_TAB):
        dpg.add_checkbox(tag=RADIO_KEYBOARD_ENABLE, label="Ativar teclado")
        rendering.bind_font(RADIO_KEYBOARD_ENABLE, "small")
        with dpg.group(horizontal=True):
            rendering.add_text("Velocidade das rodas", "small", config.TEXT_DIM)
            dpg.add_input_int(
                tag=RADIO_KEYBOARD_SPEED,
                default_value=radio.KEYBOARD_MIN_RPM,
                width=rendering.px(100),
                min_value=radio.KEYBOARD_MIN_RPM,
                max_value=radio.KEYBOARD_MAX_RPM,
                min_clamped=True,
                max_clamped=True,
                step=5,
            )
            rendering.add_text("RPM", "small", config.TEXT_DIM)
        rendering.add_text("", "small", config.TEXT_DIM, tag=RADIO_KEYBOARD_STATUS)
        dpg.add_separator()
        with dpg.table(header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column(init_width_or_weight=1)
            dpg.add_table_column(init_width_or_weight=2)
            for key, action in (
                ("W A S D  /  Setas", "Mover"),
                ("Q  /  E", "Girar à esquerda / direita"),
                ("Espaço", "Parar"),
            ):
                with dpg.table_row():
                    rendering.add_text(key, "mono", config.TEXT)
                    rendering.add_text(action, "small", config.TEXT_DIM)
        rendering.add_text("Mantenha as teclas pressionadas para mover.", "small", config.TEXT_DIM)


def _build_command_panel(on_send_motion, on_kick):
    button_h = rendering.px(BUTTON_HEIGHT)
    with dpg.tab(label="Manual", tag=RADIO_COMMAND_PANEL):
        rendering.add_text(
            "x, y (mm) · ângulo (0,01°) · vx, vy (mm/s) · w (0,01 rad/s)",
            "tiny",
            config.TEXT_DIM,
            tag=RADIO_COMMAND_UNITS,
        )
        with dpg.table(tag=RADIO_MOTION_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            for _column in range(4):
                dpg.add_table_column(init_width_or_weight=0.25)
            for row in range(3):
                with dpg.table_row():
                    for column in range(2):
                        index = row * 2 + column
                        rendering.add_text(MOTION_LABELS[index], "tiny", config.TEXT_DIM)
                        dpg.add_input_int(
                            tag=RADIO_MOTION_INPUTS[index],
                            width=rendering.px(80),
                            min_value=INT16_MIN,
                            max_value=INT16_MAX,
                            min_clamped=True,
                            max_clamped=True,
                            step=0,
                        )
        dpg.bind_item_theme(RADIO_MOTION_TABLE, _TABLE_THEME)
        dpg.add_button(
            tag=RADIO_SEND_BUTTON,
            label="Enviar movimento contínuo",
            width=1,
            height=button_h,
            callback=on_send_motion,
        )
        dpg.bind_item_theme(RADIO_SEND_BUTTON, _ACTION_THEME)
        with dpg.group(horizontal=True):
            rendering.add_text("Chute", "small", config.TEXT_DIM)
            dpg.add_input_int(
                tag=RADIO_KICK_INPUT,
                width=rendering.px(KICK_INPUT_WIDTH),
                min_value=0,
                max_value=255,
                min_clamped=True,
                max_clamped=True,
                step=0,
            )
            rendering.add_text("0–255", "tiny", config.TEXT_DIM, tag=RADIO_KICK_RANGE)
            dpg.add_button(
                tag=RADIO_KICK_BUTTON,
                label="Chutar",
                width=1,
                height=button_h,
                callback=on_kick,
            )


def _build_config_panel(
    on_control_calibration,
    on_mag_calibration,
    on_set_id,
    on_factory_reset,
    on_scan_spectrum,
    on_set_channel,
):
    button_h = rendering.px(BUTTON_HEIGHT)
    with dpg.tab(label="Configuração", tag=RADIO_CONFIG_PANEL):
        rendering.add_text("Base Station & Canal RF", "label", config.TEXT)
        rendering.add_text("Canal: —", "tiny", config.TEXT_DIM, tag=RADIO_BASE_STATUS)
        with dpg.group(horizontal=True):
            rendering.add_text("Canal", "small", config.TEXT_DIM)
            dpg.add_input_int(
                tag=RADIO_CHANNEL_INPUT,
                default_value=76,
                width=rendering.px(NEW_ID_INPUT_WIDTH),
                min_value=0,
                max_value=125,
                min_clamped=True,
                max_clamped=True,
                step=1,
            )
            dpg.add_button(
                tag=RADIO_SET_CHANNEL_BUTTON,
                label="Aplicar canal",
                width=rendering.px(130),
                height=button_h,
                callback=on_set_channel,
            )
            dpg.add_button(
                tag=RADIO_SCAN_BUTTON,
                label="Escanear",
                width=rendering.px(100),
                height=button_h,
                callback=on_scan_spectrum,
            )
        dpg.add_separator()
        rendering.add_text(
            "PERIGO: levante o robô e mantenha as rodas livres.",
            "small",
            config.BAD_COLOR,
            tag=RADIO_CONFIG_WARNING,
        )
        dpg.add_checkbox(tag=RADIO_SAFETY_CHECK, label=CONFIRM_LABEL, default_value=False)
        rendering.bind_font(RADIO_SAFETY_CHECK, "small")
        dpg.add_button(
            tag=RADIO_CONTROL_CAL_BUTTON,
            label="Calibração de controle",
            width=1,
            height=button_h,
            callback=on_control_calibration,
        )
        dpg.bind_item_theme(RADIO_CONTROL_CAL_BUTTON, _ACTION_THEME)
        dpg.add_button(
            tag=RADIO_MAG_CAL_BUTTON,
            label="Calibrar magnetômetro",
            width=1,
            height=button_h,
            callback=on_mag_calibration,
        )
        with dpg.group(horizontal=True):
            rendering.add_text("Novo ID", "small", config.TEXT_DIM)
            dpg.add_input_int(
                tag=RADIO_NEW_ID_INPUT,
                width=rendering.px(NEW_ID_INPUT_WIDTH),
                min_value=0,
                max_value=radio.ROBOT_ID_MAX,
                min_clamped=True,
                max_clamped=True,
                step=0,
            )
            dpg.add_button(
                tag=RADIO_SET_ID_BUTTON,
                label="Definir ID",
                width=1,
                height=button_h,
                callback=on_set_id,
            )
        dpg.add_separator()
        dpg.add_checkbox(tag=RADIO_RESET_CHECK, label=RESET_CONFIRM_LABEL, default_value=False)
        rendering.bind_font(RADIO_RESET_CHECK, "small")
        dpg.add_button(
            tag=RADIO_RESET_BUTTON,
            label="Reset de fábrica",
            width=1,
            height=button_h,
            callback=on_factory_reset,
        )
        dpg.bind_item_theme(RADIO_RESET_BUTTON, _STOP_THEME)


def _build_messages_panel():
    with dpg.child_window(tag=RADIO_MESSAGES_PANEL, width=1, height=1, no_scrollbar=False):
        rendering.add_text("Mensagens", "title", config.TEXT, tag=RADIO_MESSAGES_PANEL + "_title")
        rendering.add_text(
            "Sem mensagens do link de rádio",
            "small",
            config.TEXT_DIM,
            tag=RADIO_MESSAGES_EMPTY,
        )
        with dpg.table(tag=RADIO_MESSAGES_TABLE, header_row=False, policy=dpg.mvTable_SizingStretchProp):
            dpg.add_table_column()
            for index in range(MESSAGES_LINES):
                with dpg.table_row():
                    rendering.add_text("", "mono", config.TEXT_DIM, tag=RADIO_MESSAGE_LINES[index])
        dpg.bind_item_theme(RADIO_MESSAGES_TABLE, _MESSAGES_THEME)


def build(on_disconnect, stop_usb):
    """Create the radio tab contents; the dashboard tab owns the layout."""
    _create_themes()

    def on_disconnect_clicked(_sender, _app_data, _user_data=None):
        on_disconnect()

    def on_target_changed(_sender, app_data, _user_data=None):
        if _session is not None and int(app_data) != _session.snapshot()["robot_id"]:
            on_stop(None, None)
            _session.set_robot_id(int(app_data))

    def on_send_motion(_sender, _app_data, _user_data=None):
        if _session is None:
            return
        dpg.set_value(RADIO_KEYBOARD_ENABLE, False)
        _session.set_motion(
            x=int(dpg.get_value(RADIO_MOTION_INPUTS[0])),
            y=int(dpg.get_value(RADIO_MOTION_INPUTS[1])),
            angle=int(dpg.get_value(RADIO_MOTION_INPUTS[2])),
            vx=int(dpg.get_value(RADIO_MOTION_INPUTS[3])),
            vy=int(dpg.get_value(RADIO_MOTION_INPUTS[4])),
            w=int(dpg.get_value(RADIO_MOTION_INPUTS[5])),
        )

    def on_stop(_sender, _app_data, _user_data=None):
        dpg.set_value(RADIO_KEYBOARD_ENABLE, False)
        if _session is not None:
            _session.stop_motion()
        stop_usb()

    def on_kick(_sender, _app_data, _user_data=None):
        if _session is not None:
            _session.kick(int(dpg.get_value(RADIO_KICK_INPUT)))

    def on_control_calibration(_sender, _app_data, _user_data=None):
        if _session is None or not dpg.get_value(RADIO_SAFETY_CHECK):
            return
        _session.send_config(radio.CONFIG_FLAG_RUN_CALIBRATION)

    def on_mag_calibration(_sender, _app_data, _user_data=None):
        if _session is not None:
            _session.send_config(radio.CONFIG_FLAG_CALIBRATE_MAG)

    def on_set_id(_sender, _app_data, _user_data=None):
        if _session is None or not dpg.get_value(RADIO_SAFETY_CHECK):
            return
        _session.send_config(radio.CONFIG_FLAG_SET_ID, param=int(dpg.get_value(RADIO_NEW_ID_INPUT)))

    def on_factory_reset(_sender, _app_data, _user_data=None):
        if _session is None or not dpg.get_value(RADIO_RESET_CHECK):
            return
        _session.send_config(radio.CONFIG_FLAG_FACTORY_RESET)

    def on_scan_spectrum(_sender, _app_data, _user_data=None):
        if _session is not None:
            _session.request_scan()

    def on_set_channel(_sender, _app_data, _user_data=None):
        if _session is not None:
            _session.switch_channel(int(dpg.get_value(RADIO_CHANNEL_INPUT)), notify_robots=True)

    _build_header(on_disconnect_clicked)
    with dpg.group(horizontal=True):
        with rendering.panel("Controle por rádio", RADIO_DRIVE_PANEL, width=1, height=1):
            _build_target_row(on_target_changed, on_stop)
            with dpg.tab_bar(tag=RADIO_DRIVE_TABS):
                _build_keyboard_tab()
                _build_command_panel(on_send_motion, on_kick)
                _build_config_panel(
                    on_control_calibration,
                    on_mag_calibration,
                    on_set_id,
                    on_factory_reset,
                    on_scan_spectrum,
                    on_set_channel,
                )
        with dpg.group():
            _build_telemetry_panel()
            _build_messages_panel()


# ---------------------------------------------------------------------------
# Per-frame updates
# ---------------------------------------------------------------------------


def _apply_layout(body_w, body_h):
    gap = rendering.px(GAP)
    panel_margin = rendering.px(PANEL_MARGIN)
    content_h = max(rendering.px(390), body_h - rendering.px(HEADER_HEIGHT) - gap)
    telemetry_w = body_w * 45 // 100
    drive_w = body_w - telemetry_w - gap
    telemetry_h = rendering.px(312)
    dpg.configure_item(RADIO_DRIVE_PANEL, width=drive_w, height=content_h)
    dpg.configure_item(RADIO_TELEMETRY_PANEL, width=telemetry_w, height=telemetry_h)
    dpg.configure_item(RADIO_MESSAGES_PANEL, width=telemetry_w, height=content_h - telemetry_h - gap)

    telemetry_inner = telemetry_w - panel_margin
    command_inner = drive_w - panel_margin
    dpg.configure_item(RADIO_TELEMETRY_TABLE, width=telemetry_inner)
    value_w = max(rendering.px(80), telemetry_inner * 78 // 100 - rendering.px(6))
    for tag in RADIO_TELEMETRY_VALUES:
        dpg.configure_item(tag, wrap=value_w)
    dpg.configure_item(RADIO_COMMAND_UNITS, wrap=command_inner)
    motion_w = max(rendering.px(60), command_inner // 4 - rendering.px(10))
    for tag in RADIO_MOTION_INPUTS:
        dpg.configure_item(tag, width=motion_w)
    dpg.configure_item(RADIO_SEND_BUTTON, width=command_inner)
    kick_fixed = (
        int(_text_width("Chute", "small"))
        + rendering.px(KICK_INPUT_WIDTH)
        + int(_text_width("0–255", "tiny"))
        + 3 * gap
    )
    dpg.configure_item(RADIO_KICK_BUTTON, width=max(rendering.px(80), command_inner - kick_fixed))
    dpg.configure_item(RADIO_CONFIG_WARNING, wrap=command_inner)
    for tag in (RADIO_CONTROL_CAL_BUTTON, RADIO_MAG_CAL_BUTTON, RADIO_RESET_BUTTON):
        dpg.configure_item(tag, width=command_inner)
    set_id_fixed = int(_text_width("Novo ID", "small")) + rendering.px(NEW_ID_INPUT_WIDTH) + 2 * gap
    dpg.configure_item(RADIO_SET_ID_BUTTON, width=max(rendering.px(80), command_inner - set_id_fixed))


def _update_header(snapshot, body_w):
    status, color = status_text(snapshot)
    rendering.set_text(RADIO_STATUS, status, color)
    gap = rendering.px(GAP)
    fixed = (
        _text_width("Rádio nRF24L01", "title")
        + _text_width(status, "small")
        + rendering.px(DISCONNECT_WIDTH)
        + 3 * gap
    )
    dpg.configure_item(RADIO_HEADER_SPACER, width=max(gap, int(body_w - fixed)))


def _update_telemetry(snapshot):
    telemetry = snapshot["telemetry"]
    for index, (label, value) in enumerate(telemetry_rows(snapshot)):
        if telemetry is None:
            color = config.TEXT_DIM
        elif label == "Avisos":
            color = config.OK_COLOR if value == "nenhum" else config.DEMO_COLOR
        elif label == "RTT":
            color = config.ACCENT if value != "—" else config.TEXT_DIM
        elif label == "Quadros":
            color = config.TEXT_DIM
        else:
            color = config.TEXT
        rendering.set_text(RADIO_TELEMETRY_VALUES[index], value, color)


def _update_messages(snapshot):
    messages = snapshot["messages"][-MESSAGES_LINES:]
    for index, tag in enumerate(RADIO_MESSAGE_LINES):
        text = messages[index] if index < len(messages) else ""
        color = config.TEXT if index == len(messages) - 1 else config.TEXT_DIM
        rendering.set_text(tag, text, color)
    dpg.configure_item(RADIO_MESSAGES_EMPTY, show=not messages)


def _update_buttons(active):
    """Enable the radio commands only while a session is connected."""
    safety = active and bool(dpg.get_value(RADIO_SAFETY_CHECK))
    reset = active and bool(dpg.get_value(RADIO_RESET_CHECK))
    for tag in (
        RADIO_SEND_BUTTON,
        RADIO_STOP_BUTTON,
        RADIO_KICK_BUTTON,
        RADIO_MAG_CAL_BUTTON,
        RADIO_DISCONNECT_BUTTON,
        RADIO_SET_CHANNEL_BUTTON,
        RADIO_SCAN_BUTTON,
    ):
        dpg.configure_item(tag, enabled=active)
    dpg.configure_item(RADIO_CONTROL_CAL_BUTTON, enabled=safety)
    dpg.configure_item(RADIO_SET_ID_BUTTON, enabled=safety)
    dpg.configure_item(RADIO_RESET_BUTTON, enabled=reset)


def set_target(robot_id):
    """Show the robot id a new session is streaming commands to."""
    dpg.set_value(RADIO_TARGET_INPUT, int(robot_id))
    dpg.set_value(RADIO_KEYBOARD_ENABLE, False)


def _resolve_focus_library():
    """Load the GLFW symbols exported by the Dear PyGui binary, once."""
    global _focus_library, _focus_checked
    _focus_checked = True
    try:
        library = ctypes.CDLL(dpg_core.__file__)
        library.glfwGetCurrentContext.restype = ctypes.c_void_p
        library.glfwGetWindowAttrib.argtypes = [ctypes.c_void_p, ctypes.c_int]
        library.glfwGetWindowAttrib.restype = ctypes.c_int
    except (AttributeError, OSError):
        return
    _focus_library = library


def viewport_focused():
    """True while the OS window has focus, and when the check is unavailable."""
    if not _focus_checked:
        _resolve_focus_library()
    if _focus_library is None:
        return True
    window = _focus_library.glfwGetCurrentContext()
    if not window:
        return True
    return bool(_focus_library.glfwGetWindowAttrib(window, GLFW_FOCUSED))


def poll_keyboard(session, *, visible, stop_usb):
    """Update radio motion only while its drive tab is active and keys are held."""
    global _keyboard_armed, _keyboard_neutral, _keyboard_focus_lost, _focus_empty_since
    if session is None:
        _keyboard_armed = _keyboard_neutral = False
        return
    if not visible and not _keyboard_armed:
        return

    selected = dpg.get_value(RADIO_DRIVE_TABS)
    if isinstance(selected, int):
        selected = dpg.get_item_alias(selected)
    keyboard_visible = visible and selected == RADIO_KEYBOARD_TAB
    if not keyboard_visible and _keyboard_armed:
        dpg.set_value(RADIO_KEYBOARD_ENABLE, False)
    armed = keyboard_visible and dpg.get_value(RADIO_KEYBOARD_ENABLE)
    if (armed and not _keyboard_armed) or (visible and dpg.is_key_pressed(dpg.mvKey_Spacebar)):
        session.stop_motion()
        stop_usb()
    _keyboard_armed = armed

    keys = {key for key in _DRIVE_KEYS if visible and dpg.is_key_down(key)}
    focused = viewport_focused()
    now = time.monotonic()
    if not focused:
        _keyboard_focus_lost = True
        _focus_empty_since = 0.0
    elif _keyboard_focus_lost:
        if keys:
            _focus_empty_since = 0.0
        elif not _focus_empty_since:
            _focus_empty_since = now
        elif now - _focus_empty_since >= FOCUS_REGAIN_GRACE_S:
            _keyboard_focus_lost = False
            _focus_empty_since = 0.0

    velocity = (0, 0, 0)
    status, color = "Teclado desativado", config.TEXT_DIM
    if not armed:
        _keyboard_neutral = False
    elif not session.keyboard_link_ready():
        _keyboard_neutral = False
        status, color = "Aguardando resposta do robô", config.BAD_COLOR
    elif not focused:
        # Keys held outside the window stop the wheels; coming back needs a
        # release so a key resting on the keyboard cannot restart the drive.
        _keyboard_neutral = False
        status, color = "Janela sem foco", config.BAD_COLOR
    elif _keyboard_focus_lost:
        _keyboard_neutral = False
        status = "Solte as teclas para iniciar" if keys else "Aguardando teclado"
    elif (
        dpg.is_item_active(RADIO_TARGET_INPUT)
        or dpg.is_item_active(RADIO_KEYBOARD_SPEED)
        or dpg.mvKey_Spacebar in keys
    ):
        _keyboard_neutral = False
        status = "Parado"
    elif not _keyboard_neutral and keys:
        status = "Solte as teclas para iniciar"
    else:
        _keyboard_neutral = True
        x = bool(keys & {dpg.mvKey_W, dpg.mvKey_Up}) - bool(keys & {dpg.mvKey_S, dpg.mvKey_Down})
        y = bool(keys & {dpg.mvKey_A, dpg.mvKey_Left}) - bool(keys & {dpg.mvKey_D, dpg.mvKey_Right})
        turn = (dpg.mvKey_Q in keys) - (dpg.mvKey_E in keys)
        velocity = radio.keyboard_velocity(x, y, turn, dpg.get_value(RADIO_KEYBOARD_SPEED))
        status, color = ("Enviando movimento", config.ACCENT) if any(velocity) else ("Pronto", config.OK_COLOR)

    session.set_keyboard_velocity(velocity)
    rendering.set_text(RADIO_KEYBOARD_STATUS, status, color)


def _empty_snapshot():
    """Snapshot-like state shown while no radio session is connected."""
    return {
        "port": "",
        "connected": False,
        "robot_id": 0,
        "telemetry": None,
        "rx_age_s": None,
        "rtt_ms": 0.0,
        "tx_count": 0,
        "tx_ack_rate": 0.0,
        "avg_retries": 0.0,
        "rx_count": 0,
        "base_info": None,
        "scan_result": None,
        "command": {"x": 0, "y": 0, "angle": 0, "vx": 0, "vy": 0, "w": 0},
        "pending_configs": 0,
        "error": "",
        "messages": [],
        "live": False,
    }


def _update_base_status(snapshot):
    info = snapshot.get("base_info")
    if not info:
        rendering.set_text(RADIO_BASE_STATUS, "Base Station: desconectada", config.TEXT_DIM)
        return
    ch = info["channel"]
    ack = snapshot.get("tx_ack_rate", 0.0)
    reinits = info.get("radio_reinit_count", 0)
    text = f"Canal {ch} ({2400 + ch} MHz) · ACK {ack:.0f}% · Reinits {reinits}"
    scan = snapshot.get("scan_result")
    if scan:
        best = scan["best_channel"]
        text += f" · Scan: melhor ch {best}"
    rendering.set_text(RADIO_BASE_STATUS, text, config.ACCENT)


def update(session, layout):
    """Refresh the radio tab; session is None while the radio is disconnected."""
    global _session
    _session = session
    snapshot = session.snapshot() if session is not None else _empty_snapshot()
    body_w = max(200, int(layout["body_w"]))
    body_h = max(200, int(layout["body_h"]))
    _apply_layout(body_w, body_h)
    _update_header(snapshot, body_w)
    _update_telemetry(snapshot)
    _update_base_status(snapshot)
    _update_messages(snapshot)
    _update_buttons(session is not None)
