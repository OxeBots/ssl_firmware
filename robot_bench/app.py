"""Window lifecycle, main loop and input routing for Robot Bench."""

import os
import threading
import time

import dearpygui.dearpygui as dpg

from . import calibration, config, console, health, motors, picker, radio, radio_panel, rendering, sensing
from . import protocol as proto
from .discovery import PortScanner
from .serial_io import BenchReader

MAIN_WINDOW = "main_window"
TAB_BAR = "tab_bar"
TAB_TAGS = (
    "tab_motors",
    "tab_calibration",
    "tab_health",
    "tab_tof",
    "tab_imu",
    "tab_console",
    "tab_radio",
)
HEADER_SPACER = "header_spacer"
CONNECTION_STATUS = "connection_status"
CONNECTIONS_BUTTON = "connections_button"
CONNECTIONS_BUTTON_LABEL = "Conexões"
CONNECTIONS_BUTTON_WIDTH = 110
STATUS_BAR = "status_bar"
HEADER_SUBTITLE = "Diagnóstico do robô SSL"

# Vertical space reserved around the tab content inside the window.
MARGIN = 16
HEADER_HEIGHT = 38
GAP = 12
TAB_STRIP_HEIGHT = 30
STATUS_HEIGHT = 24
# Gap between header items, matching the shared theme item spacing.
HEADER_ITEM_GAP = 8

_TEXT_WIDTHS = {}

# Keyboard auto-repeat while an arrow key or wheel shortcut is held down.
REPEAT_DELAY_S = 0.35
REPEAT_PERIOD_S = 0.06

# The nRF24L01 USB adapters are re-detected at this period while the picker is
# open, so plugging the radio bridge in shows up without pressing "Buscar portas".
RADIO_SCAN_PERIOD_S = 2.0
RADIO_DISCOVERY_RETRY_S = 4.0

KEY_ACTIONS = {
    dpg.mvKey_F1: ("tab", config.TAB_MOTORS),
    dpg.mvKey_F2: ("tab", config.TAB_CALIBRATION),
    dpg.mvKey_F3: ("tab", config.TAB_HEALTH),
    dpg.mvKey_F4: ("tab", config.TAB_TOF),
    dpg.mvKey_F5: ("tab", config.TAB_IMU),
    dpg.mvKey_F6: ("tab", config.TAB_CONSOLE),
    dpg.mvKey_F7: ("tab", config.TAB_RADIO),
    dpg.mvKey_1: ("wheel", 0),
    dpg.mvKey_2: ("wheel", 1),
    dpg.mvKey_3: ("wheel", 2),
    dpg.mvKey_4: ("wheel", 3),
    dpg.mvKey_Up: ("adjust_rpm", 1.0),
    dpg.mvKey_Down: ("adjust_rpm", -1.0),
    dpg.mvKey_Right: ("adjust", 1.0),
    dpg.mvKey_Left: ("adjust", -1.0),
    dpg.mvKey_Spacebar: ("stop", None),
    dpg.mvKey_W: ("sweep", None),
    dpg.mvKey_Q: ("quit", "q"),
    dpg.mvKey_Escape: ("quit", None),
}


def perform_action(state, action, value=None):
    """Dashboard shortcuts all go through this single command path."""
    if action == "tab":
        state.active_tab = value
    elif action == "wheel":
        state.selected_wheel = value
    elif action == "adjust":
        # Left/right move the selected target by a fraction of the calibrated limit.
        state.adjust_selected_target(value)
    elif action == "adjust_rpm":
        # Up/down move the selected target by the configurable RPM step.
        state.adjust_selected_target_by_rpm(value * state.arrow_step_rpm)
    elif action == "stop":
        state.stop_motors()
    elif action == "sweep":
        state.sweep = not state.sweep
        state.add_console("Varredura " + ("ligada" if state.sweep else "desligada"))
    elif action == "quit":
        stop_and_exit(state)


def stop_and_exit(state):
    if state.quit:
        return
    if state.scanner is not None:
        state.scanner.active.clear()
    state.stop_motors()
    state.quit = True


def compute_layout(width, height):
    """Window space left for the active tab after header, tabs and status bar."""
    body_width = max(200, width - 2 * MARGIN)
    body_height = max(120, height - 2 * MARGIN - HEADER_HEIGHT - GAP - TAB_STRIP_HEIGHT - GAP - STATUS_HEIGHT)
    return {"body_w": body_width, "body_h": body_height}


def text_width(text, font_name):
    """Rendered width of one fixed interface text, measured once."""
    key = (text, font_name)
    width = _TEXT_WIDTHS.get(key)
    if width is not None:
        return width
    font = rendering.font(font_name)
    measured = dpg.get_text_size(text, font=font) if font is not None else dpg.get_text_size(text)
    if not measured:
        return len(text) * 7.0
    _TEXT_WIDTHS[key] = width = float(measured[0])
    return width


def header_spacer_width(status, body_w):
    """Spacer that keeps the connection badge and buttons flush with the right edge."""
    # Five item gaps sit between the title and the badge; the 14 px spacer
    # separates the title from the subtitle.
    prefix = (
        text_width(config.APP_NAME, "big")
        + 14.0
        + text_width(HEADER_SUBTITLE, "small")
        + 5 * HEADER_ITEM_GAP
    )
    return max(20, int(body_w - prefix - text_width(status, "small") - CONNECTIONS_BUTTON_WIDTH - 2))


def usb_connection_status(state):
    """USB debug link badge text and color for the header."""
    if state.demo:
        return "Demo | dados simulados", config.DEMO_COLOR
    if state._live_link():
        return f"Ao vivo | {state.connected}", config.OK_COLOR
    if state.connected and (state.last_link_t or state.last_state_t):
        return "Conectado | telemetria desatualizada", config.BAD_COLOR
    if state.connected:
        return "Conectado | aguardando dados", config.TEXT_DIM
    return "Desconectado | reconectando", config.BAD_COLOR


def radio_connection_status(snapshot):
    """Radio link badge text and color for the header."""
    if snapshot["live"]:
        return "Rádio ao vivo", config.OK_COLOR
    if snapshot["connected"] and snapshot["rx_count"]:
        return "Rádio sem resposta", config.BAD_COLOR
    if snapshot["connected"]:
        return "Rádio aguardando", config.TEXT_DIM
    return "Rádio desconectado", config.BAD_COLOR


def radio_discovery_due(radio_state, now):
    """Start a scan for a new adapter or retry an empty scan after its delay."""
    if radio_state["session"] is not None or radio_state["discovering"] or not radio_state["devices"]:
        return False
    device = radio_state["devices"][0][0]
    discovered = radio_state["discovered"]
    return (
        radio_state["discovered_device"] != device
        or discovered is None
        or (not discovered and now >= radio_state["discovery_retry_at"])
    )


def connection_status(state, radio_snapshot=None):
    """Connection badge text and color; combines the USB and radio links."""
    usb_text, usb_color = usb_connection_status(state)
    if radio_snapshot is None:
        return usb_text, usb_color
    radio_text, radio_color = radio_connection_status(radio_snapshot)
    if not state.connected and not state.demo:
        return radio_text, radio_color
    if usb_color == config.OK_COLOR and radio_color == config.OK_COLOR:
        color = config.OK_COLOR
    elif usb_color == config.BAD_COLOR or radio_color == config.BAD_COLOR:
        color = config.BAD_COLOR
    else:
        color = config.TEXT_DIM
    return f"{usb_text} · {radio_text}", color


def status_line(state, hz_state, hz_tof, fps):
    """Stream rates and uptime shown under the tabs."""
    live = state.link_fresh()
    uptime = f"{state.uptime_ms // 1000}s" if live else "-"
    return (
        f"Estado {hz_state:.0f} Hz   |   Distância {hz_tof:.0f} Hz   |   Recebido {state.rx_bytes:,} B"
        f"   |   Tempo ativo {uptime}   |   Exibição {fps:.0f} fps"
    )


class KeyRepeater:
    """Turns held dashboard keys into one action plus auto-repeat actions."""

    def __init__(self):
        self._next_repeat = {}

    def poll(self, now, *, drive_shortcuts=True, editing_step=False):
        actions = []
        for key, action in KEY_ACTIONS.items():
            pressed = dpg.is_key_pressed(key)
            down = dpg.is_key_down(key)
            blocked = not drive_shortcuts and (key == dpg.mvKey_Q or action[0] not in ("tab", "quit"))
            if editing_step and action[0] in ("wheel", "adjust", "adjust_rpm"):
                blocked = True
            if blocked:
                if down:
                    self._next_repeat[key] = float("inf")
                else:
                    self._next_repeat.pop(key, None)
            elif pressed and key not in self._next_repeat:
                actions.append(action)
                self._next_repeat[key] = now + REPEAT_DELAY_S
            elif down and key in self._next_repeat and now >= self._next_repeat[key]:
                actions.append(action)
                self._next_repeat[key] = now + REPEAT_PERIOD_S
            elif not down:
                self._next_repeat.pop(key, None)
        return actions


def tab_visible(index, radio_active, usb_active):
    """True when a tab can be selected: the radio tab needs the radio link."""
    return radio_active if index == config.TAB_RADIO else usb_active


def select_tab(state, index):
    state.active_tab = index
    if dpg.does_item_exist(TAB_BAR):
        dpg.set_value(TAB_BAR, TAB_TAGS[index])


def build_dashboard(state, on_open_connections, on_disconnect_radio):
    """Create the primary dashboard window and its tab contents.

    The radio tab is created hidden and only shown while a radio session is
    connected, so the USB dashboard and the radio link can be used together.
    """
    with dpg.window(
        tag=MAIN_WINDOW,
        no_title_bar=True,
        no_move=True,
        no_resize=True,
        no_scrollbar=True,
        no_collapse=True,
        no_scroll_with_mouse=True,
    ):
        with dpg.group(horizontal=True):
            rendering.add_text(config.APP_NAME, "big", tag="header_title")
            dpg.add_spacer(width=14)
            rendering.add_text(HEADER_SUBTITLE, "small", config.TEXT_DIM)
            dpg.add_spacer(tag=HEADER_SPACER, width=200)
            rendering.add_text("", "small", config.TEXT_DIM, tag=CONNECTION_STATUS)
            dpg.add_button(
                tag=CONNECTIONS_BUTTON,
                label=CONNECTIONS_BUTTON_LABEL,
                width=CONNECTIONS_BUTTON_WIDTH,
                callback=on_open_connections,
            )
        with dpg.tab_bar(tag=TAB_BAR):
            with dpg.tab(label="Motores   F1", tag=TAB_TAGS[config.TAB_MOTORS]):
                motors.build(state)
            with dpg.tab(label="Calibração   F2", tag=TAB_TAGS[config.TAB_CALIBRATION]):
                calibration.build(state)
            with dpg.tab(label="Saúde   F3", tag=TAB_TAGS[config.TAB_HEALTH]):
                health.build(state)
            with dpg.tab(label="ToF   F4", tag=TAB_TAGS[config.TAB_TOF]):
                sensing.build_tof(state)
            with dpg.tab(label="IMU   F5", tag=TAB_TAGS[config.TAB_IMU]):
                sensing.build_imu(state)
            with dpg.tab(label="Console   F6", tag=TAB_TAGS[config.TAB_CONSOLE]):
                console.build(state)
            with dpg.tab(label="Rádio   F7", tag=TAB_TAGS[config.TAB_RADIO], show=False):
                radio_panel.build(on_disconnect_radio, state.stop_motors)
        rendering.add_text("", "tiny", config.TEXT_DIM, tag=STATUS_BAR)


def refresh_dashboard(state, radio_session, layout, now, hz_state, hz_tof, fps):
    """Push the latest telemetry into the visible tab and window chrome."""
    radio_snapshot = radio_session.snapshot() if radio_session is not None else None
    status, color = connection_status(state, radio_snapshot)
    rendering.set_text(CONNECTION_STATUS, status, color)
    footer = status_line(state, hz_state, hz_tof, fps)
    if state.active_tab == config.TAB_RADIO and radio_snapshot is not None:
        age = radio_snapshot["rx_age_s"]
        footer = f"Última resposta há {age * 1000:.0f} ms" if age is not None else "Aguardando resposta"
    rendering.set_text(STATUS_BAR, footer)
    dpg.configure_item(HEADER_SPACER, width=header_spacer_width(status, layout["body_w"]))
    if state.active_tab != config.TAB_MOTORS:
        # The raw encoder mux focus only makes sense while the Motors tab is open.
        motors.leave(state)
    if state.active_tab == config.TAB_MOTORS:
        state.stream_mask = proto.STREAM_STATE | proto.STREAM_CALIBRATION
        if motors.active_view() == motors.MOTORS_VIEW_RAW:
            state.stream_mask |= proto.STREAM_RAW
        else:
            state.stream_mask |= proto.STREAM_TUNING
            if motors.sidebar_tab() == motors.SIDEBAR_POSITION:
                state.stream_mask |= proto.STREAM_POSITION
        motors.update(state, layout, now)
    elif state.active_tab == config.TAB_CALIBRATION:
        state.stream_mask = proto.STREAM_CALIBRATION
        if calibration.active_subtab() == calibration.SUBTAB_CONTROL:
            state.stream_mask |= proto.STREAM_STATE | proto.STREAM_TUNING
        calibration.update(state, layout, now)
    elif state.active_tab == config.TAB_HEALTH:
        state.stream_mask = proto.STREAM_HEALTH
        health.update(state, layout, now)
    elif state.active_tab == config.TAB_TOF:
        state.stream_mask = proto.STREAM_TOF
        sensing.update_tof(state, layout, now)
    elif state.active_tab == config.TAB_IMU:
        state.stream_mask = proto.STREAM_IMU
        sensing.update_imu(state, layout, now)
    elif state.active_tab == config.TAB_CONSOLE:
        state.stream_mask = proto.STREAM_TEXT
        console.update(state, layout)
    elif state.active_tab == config.TAB_RADIO:
        state.stream_mask = 0
        radio_panel.update(radio_session, layout)


def run_gui(state, args):
    dpg.create_context()
    rendering.load_fonts()
    rendering.apply_theme()

    picking_port = args.port is None and not args.demo
    scan_results = {}
    scan_lock = threading.Lock()
    scan_active = threading.Event()
    wheel_delta = {"value": 0.0}
    # nRF24L01 adapters, live radio session and robot discovery results.
    radio_state = {
        "session": None,
        "devices": [],
        "discovered": None,
        "discovering": False,
        "discovery_error": "",
        "discovered_device": None,
        "discovery_retry_at": 0.0,
    }
    width = max(args.width, config.MIN_WINDOW[0])
    height = max(args.height, config.MIN_WINDOW[1])

    def start_session(device):
        nonlocal picking_port
        scan_active.clear()
        if state.scanner is not None:
            state.scanner.join(timeout=2.0)
            state.scanner = None
        state.reader = BenchReader(state, device, args.baud)
        state.reader.start()
        picking_port = False

    def rescan_ports():
        nonlocal scan_active
        scan_active.clear()
        if state.scanner is not None:
            state.scanner.join(timeout=2.0)
        # The port used by the running USB session must not be probed: the probe
        # opens it and toggles the modem lines, which resets the robot.
        busy_port = state.reader.requested_port if state.reader is not None else None
        with scan_lock:
            scan_results.clear()
            if busy_port:
                scan_results[busy_port] = {
                    "desc": "Em uso por esta bancada",
                    "status": "firmware",
                    "detail": "",
                }
        scan_active = threading.Event()
        scan_active.set()
        state.scanner = PortScanner(
            scan_results, scan_lock, scan_active, keep={busy_port} if busy_port else ()
        )
        state.scanner.start()

    def scan_radio_devices():
        radio_state["devices"] = radio.find_adapters()
        if not radio_state["devices"]:
            radio_state["discovered_device"] = None

    def open_connections(_sender=None, _app_data=None, _user_data=None):
        nonlocal picking_port
        picking_port = True
        if state.scanner is None:
            rescan_ports()
        else:
            state.scanner.active.set()
        scan_radio_devices()
        if radio_state["session"] is None:
            # Let the picker rediscover the robots that are powered on.
            radio_state["discovered"] = None
            radio_state["discovery_error"] = ""

    def on_back(_sender=None, _app_data=None, _user_data=None):
        nonlocal picking_port
        if state.reader is None and radio_state["session"] is None and not state.demo:
            return
        picking_port = False

    def start_radio_discovery(device):
        if radio_state["discovering"]:
            return
        radio_state["discovering"] = True
        radio_state["discovered"] = None
        radio_state["discovery_error"] = ""
        radio_state["discovered_device"] = device

        def worker():
            # Let a just-closed radio session finish on the bridge before the
            # discovery opens the adapter again.
            time.sleep(0.25)
            try:
                radio_state["discovered"] = radio.discover_robots(device)
            except radio.RadioError as error:
                radio_state["discovered"] = []
                radio_state["discovery_error"] = str(error)
            finally:
                radio_state["discovery_retry_at"] = time.monotonic() + RADIO_DISCOVERY_RETRY_S
                radio_state["discovering"] = False

        threading.Thread(target=worker, daemon=True).start()

    def discover_radio_robots():
        if not radio_state["devices"]:
            return
        start_radio_discovery(radio_state["devices"][0][0])

    def close_radio_session():
        nonlocal picking_port
        session = radio_state["session"]
        radio_state["session"] = None
        if session is not None:
            session.close()
        # Returning to the connection screen rediscovers the robots that are on.
        radio_state["discovered"] = None
        radio_state["discovery_error"] = ""
        if state.active_tab == config.TAB_RADIO:
            state.active_tab = config.TAB_MOTORS
        if state.reader is None and not state.demo:
            picking_port = True

    def start_radio_session(device):
        nonlocal picking_port
        if radio_state["session"] is not None:
            picker.set_radio_status("O rádio já está conectado.", config.TEXT_DIM)
            return
        if radio_state["discovering"]:
            picker.set_radio_status("Busca de robôs em andamento; aguarde terminar.", config.TEXT_DIM)
            return
        try:
            session = radio.RadioSession.connect(device, picker.radio_robot_id())
        except radio.RadioError as error:
            picker.set_radio_status(f"Falha ao abrir o rádio: {error}", config.BAD_COLOR)
            return
        picker.set_radio_status("")
        scan_active.clear()
        if state.scanner is not None:
            state.scanner.join(timeout=2.0)
            state.scanner = None
        radio_state["session"] = session
        radio_panel.set_target(picker.radio_robot_id())
        session.start()
        # Show the new link right away; the radio tab becomes visible this frame.
        dpg.configure_item(TAB_TAGS[config.TAB_RADIO], show=True)
        select_tab(state, config.TAB_RADIO)
        picking_port = False

    def on_connect_radio(device):
        start_radio_session(device)

    def on_discover_radio():
        discover_radio_robots()

    def on_disconnect_radio(_sender=None, _app_data=None, _user_data=None):
        close_radio_session()
        scan_radio_devices()
        if picking_port:
            rescan_ports()

    def on_connect(_sender, _app_data, user_data=None):
        device = picker.device_for_row(user_data)
        if device is not None:
            start_session(device)

    def on_rescan(_sender, _app_data, _user_data=None):
        scan_radio_devices()
        radio_state["discovered"] = None
        rescan_ports()

    def on_quit(_sender=None, _app_data=None, _user_data=None):
        stop_and_exit(state)

    def on_wheel(_sender, app_data, _user_data=None):
        wheel_delta["value"] += float(app_data)

    if picking_port:
        scan_active.set()
        state.scanner = PortScanner(scan_results, scan_lock, scan_active)
        state.scanner.start()
        scan_radio_devices()
    elif not state.demo:
        state.reader = BenchReader(state, args.port, args.baud)
        state.reader.start()

    build_dashboard(state, open_connections, on_disconnect_radio)
    picker.build(on_connect, on_rescan, on_quit, on_connect_radio, on_discover_radio, on_back)
    with dpg.handler_registry():
        dpg.add_mouse_wheel_handler(callback=on_wheel)
    dpg.create_viewport(
        title=config.APP_NAME,
        width=width,
        height=height,
        min_width=config.MIN_WINDOW[0],
        min_height=config.MIN_WINDOW[1],
        clear_color=rendering.rgba(config.BACKGROUND),
    )
    dpg.setup_dearpygui()
    dpg.set_primary_window(MAIN_WINDOW, True)
    dpg.show_viewport()

    def on_exit():
        stop_and_exit(state)

    dpg.set_exit_callback(on_exit)
    if args.screenshot_dir:
        os.makedirs(args.screenshot_dir, exist_ok=True)

    keys = KeyRepeater()
    last_rate_time = time.monotonic()
    last_state_count = last_tof_count = 0
    hz_state = hz_tof = 0.0
    frame_count = 0
    fps = 0.0
    last_frame_time = time.monotonic()
    screenshot_index = 0
    next_screenshot = 0.0
    last_radio_scan = 0.0
    dpg.set_value(TAB_BAR, TAB_TAGS[state.active_tab])
    dpg.render_dearpygui_frame()

    try:
        while not state.quit and dpg.is_dearpygui_running():
            now = time.monotonic()
            session = radio_state["session"]
            if session is not None and session.request_picker():
                state.add_console("rádio: link perdido, desconectando")
                on_disconnect_radio()
                session = radio_state["session"]
            radio_active = session is not None
            usb_active = state.reader is not None or state.demo

            if not picking_port:
                selected = dpg.get_value(TAB_BAR)
                if isinstance(selected, int):
                    selected = dpg.get_item_alias(selected)
                if selected in TAB_TAGS:
                    state.active_tab = TAB_TAGS.index(selected)
            actions = keys.poll(
                now,
                drive_shortcuts=not picking_port and state.active_tab != config.TAB_RADIO,
                editing_step=dpg.is_item_active(motors.ARROW_STEP_INPUT),
            )
            if not picking_port:
                for action, value in actions:
                    if action == "tab":
                        if tab_visible(value, radio_active, usb_active):
                            select_tab(state, value)
                    elif state.active_tab != config.TAB_RADIO or (action == "quit" and value is None):
                        if action == "stop" and session is not None:
                            session.stop_motion()
                        perform_action(state, action, value)
            if state.quit:
                break

            if state.picker_requested:
                state.picker_requested = False
                if state.reader is not None:
                    state.reader.join(timeout=2.0)
                    state.reader = None
                state.sweep = False
                usb_active = state.reader is not None or state.demo
                if not radio_active:
                    rescan_ports()
                    picking_port = True

            if not picking_port and not usb_active and not radio_active:
                # No link left: back to the connection screen.
                rescan_ports()
                picking_port = True

            # USB tabs need the debug link, the radio tab needs the radio
            # session; both sets are visible when both links are connected.
            for index in range(len(TAB_TAGS) - 1):
                dpg.configure_item(TAB_TAGS[index], show=usb_active)
            dpg.configure_item(TAB_TAGS[config.TAB_RADIO], show=radio_active)
            if state.active_tab == config.TAB_RADIO and not radio_active:
                state.active_tab = config.TAB_MOTORS
            if radio_active and not usb_active and state.active_tab != config.TAB_RADIO:
                state.active_tab = config.TAB_RADIO
            current_tab = dpg.get_value(TAB_BAR)
            if isinstance(current_tab, int):
                current_tab = dpg.get_item_alias(current_tab)
            if current_tab != TAB_TAGS[state.active_tab]:
                dpg.set_value(TAB_BAR, TAB_TAGS[state.active_tab])

            dpg.configure_item(MAIN_WINDOW, show=not picking_port)
            dpg.configure_item(picker.WINDOW, show=picking_port)
            radio_panel.poll_keyboard(
                session,
                visible=not picking_port and state.active_tab == config.TAB_RADIO,
                stop_usb=state.stop_motors,
            )

            elapsed = now - last_rate_time
            if elapsed >= 1.0:
                hz_state = (state.state_count - last_state_count) / elapsed
                hz_tof = (state.tof_count - last_tof_count) / elapsed
                last_state_count, last_tof_count = state.state_count, state.tof_count
                last_rate_time = now
            frame_count += 1
            frame_elapsed = now - last_frame_time
            if frame_elapsed >= 1.0:
                fps = frame_count / frame_elapsed
                frame_count = 0
                last_frame_time = now

            view_width = dpg.get_viewport_client_width()
            view_height = dpg.get_viewport_client_height()
            if not picking_port:
                state.update_sweep(now)
                layout = compute_layout(view_width, view_height)
                refresh_dashboard(state, radio_state["session"], layout, now, hz_state, hz_tof, fps)
                mouse = dpg.get_mouse_pos(local=False)
                left_clicked = dpg.is_mouse_button_clicked(dpg.mvMouseButton_Left)
                left_down = dpg.is_mouse_button_down(dpg.mvMouseButton_Left)
                left_released = dpg.is_mouse_button_released(dpg.mvMouseButton_Left)
                if wheel_delta["value"]:
                    if state.active_tab == config.TAB_CONSOLE:
                        console.wheel(state, wheel_delta["value"], mouse)
                    wheel_delta["value"] = 0.0
                if left_clicked:
                    if state.active_tab == config.TAB_CONSOLE:
                        console.press(state, mouse)
                    elif state.active_tab == config.TAB_MOTORS:
                        motors.select_wheel_at(state, mouse)
                elif left_down and state.active_tab == config.TAB_CONSOLE:
                    console.drag(state, mouse)
                elif left_released and state.active_tab == config.TAB_CONSOLE:
                    console.release(state, mouse)
            else:
                state.stream_mask = 0
                if now - last_radio_scan >= RADIO_SCAN_PERIOD_S:
                    last_radio_scan = now
                    scan_radio_devices()
                if radio_discovery_due(radio_state, now):
                    discover_radio_robots()
                picker.update(
                    scan_results,
                    scan_lock,
                    radio_state["devices"],
                    radio_state["discovered"],
                    radio_state["discovering"],
                    radio_state["discovery_error"],
                    can_back=usb_active or radio_active,
                    usb_connected=usb_active,
                    radio_connected=radio_active,
                )
                picker_width, picker_height = dpg.get_item_rect_size(picker.WINDOW)
                if picker_width and picker_height:
                    dpg.configure_item(
                        picker.WINDOW,
                        pos=[(view_width - picker_width) // 2, (view_height - picker_height) // 2],
                    )

            if args.screenshot_dir and now >= next_screenshot:
                next_screenshot = now + args.screenshot_period
                path = os.path.join(args.screenshot_dir, f"bench_gui_{screenshot_index:03d}.png")
                dpg.output_frame_buffer(path)
                screenshot_index += 1
            dpg.render_dearpygui_frame()
    finally:
        stop_and_exit(state)
        if radio_state["session"] is not None:
            radio_state["session"].close()
            radio_state["session"] = None
        for worker in (state.reader, state.scanner):
            if worker is not None:
                worker.join(timeout=2.0)
        dpg.destroy_context()
