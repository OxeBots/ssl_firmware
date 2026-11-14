"""Window lifecycle, main loop and input routing for Robot Bench."""

import os
import threading
import time

import dearpygui.dearpygui as dpg

from . import calibration, config, console, health, motors, picker, rendering, sensing, tuning
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
    "tab_tuning",
)
HEADER_SPACER = "header_spacer"
CONNECTION_STATUS = "connection_status"
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

KEY_ACTIONS = {
    dpg.mvKey_F1: ("tab", config.TAB_MOTORS),
    dpg.mvKey_F2: ("tab", config.TAB_CALIBRATION),
    dpg.mvKey_F3: ("tab", config.TAB_HEALTH),
    dpg.mvKey_F4: ("tab", config.TAB_TOF),
    dpg.mvKey_F5: ("tab", config.TAB_IMU),
    dpg.mvKey_F6: ("tab", config.TAB_CONSOLE),
    dpg.mvKey_F7: ("tab", config.TAB_TUNING),
    dpg.mvKey_1: ("wheel", 0),
    dpg.mvKey_2: ("wheel", 1),
    dpg.mvKey_3: ("wheel", 2),
    dpg.mvKey_4: ("wheel", 3),
    dpg.mvKey_Up: ("adjust", 100),
    dpg.mvKey_Down: ("adjust", -100),
    dpg.mvKey_Right: ("adjust", 500),
    dpg.mvKey_Left: ("adjust", -500),
    dpg.mvKey_Spacebar: ("stop", None),
    dpg.mvKey_W: ("sweep", None),
    dpg.mvKey_Q: ("quit", None),
    dpg.mvKey_Escape: ("quit", None),
}


def perform_action(state, action, value=None):
    """Dashboard shortcuts all go through this single command path."""
    if action == "tab":
        state.active_tab = value
    elif action == "wheel":
        state.selected_wheel = value
    elif action == "adjust":
        target = state.manual_targets[state.selected_wheel] if state.demo else state.wheels[state.selected_wheel]["tgt"]
        state.set_selected_target(target + value)
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
    """Spacer that keeps the connection badge flush with the right edge."""
    # Four item gaps sit between the title and the badge; the 14 px spacer
    # separates the title from the subtitle.
    prefix = (
        text_width(config.APP_NAME, "big")
        + 14.0
        + text_width(HEADER_SUBTITLE, "small")
        + 4 * HEADER_ITEM_GAP
    )
    return max(20, int(body_w - prefix - text_width(status, "small") - 2))


def connection_status(state):
    """Connection badge text and color for the header."""
    if state.demo:
        return "Demo | dados simulados", config.DEMO_COLOR
    if (
        state.connected and state.streaming and state.last_state_t
        and time.monotonic() - state.last_state_t <= config.SENSOR_STALE_S
    ):
        return f"Ao vivo | {state.connected}", config.OK_COLOR
    if state.connected and state.last_state_t:
        return "Conectado | telemetria desatualizada", config.BAD_COLOR
    if state.connected:
        return "Conectado | aguardando dados", config.TEXT_DIM
    return "Desconectado | reconectando", config.BAD_COLOR


def status_line(state, hz_state, hz_tof, fps):
    """Stream rates and uptime shown under the tabs."""
    live = state.last_state_t and time.monotonic() - state.last_state_t <= config.SENSOR_STALE_S
    uptime = f"{state.uptime_ms // 1000}s" if live else "-"
    return (
        f"Estado {hz_state:.0f} Hz   |   Distância {hz_tof:.0f} Hz   |   Recebido {state.rx_bytes:,} B"
        f"   |   Tempo ativo {uptime}   |   Exibição {fps:.0f} fps"
    )


class KeyRepeater:
    """Turns held dashboard keys into one action plus auto-repeat actions."""

    def __init__(self):
        self._next_repeat = {}

    def poll(self, now):
        actions = []
        for key, action in KEY_ACTIONS.items():
            pressed = dpg.is_key_pressed(key)
            down = dpg.is_key_down(key)
            if pressed and key not in self._next_repeat:
                actions.append(action)
                self._next_repeat[key] = now + REPEAT_DELAY_S
            elif down and key in self._next_repeat and now >= self._next_repeat[key]:
                actions.append(action)
                self._next_repeat[key] = now + REPEAT_PERIOD_S
            elif not down:
                self._next_repeat.pop(key, None)
        return actions


def select_tab(state, index):
    state.active_tab = index
    if dpg.does_item_exist(TAB_BAR):
        dpg.set_value(TAB_BAR, TAB_TAGS[index])


def build_dashboard(state):
    """Create the primary dashboard window and all seven tab contents."""
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
            with dpg.tab(label="Controle   F7", tag=TAB_TAGS[config.TAB_TUNING]):
                tuning.build(state)
        rendering.add_text("", "tiny", config.TEXT_DIM, tag=STATUS_BAR)


def refresh_dashboard(state, layout, now, hz_state, hz_tof, fps):
    """Push the latest telemetry into the visible tab and window chrome."""
    status, color = connection_status(state)
    rendering.set_text(CONNECTION_STATUS, status, color)
    rendering.set_text(STATUS_BAR, status_line(state, hz_state, hz_tof, fps))
    dpg.configure_item(HEADER_SPACER, width=header_spacer_width(status, layout["body_w"]))
    if state.active_tab == config.TAB_MOTORS:
        motors.update(state, layout, now)
    elif state.active_tab == config.TAB_CALIBRATION:
        calibration.update(state, layout, now)
    elif state.active_tab == config.TAB_HEALTH:
        health.update(state, layout, now)
    elif state.active_tab == config.TAB_TOF:
        sensing.update_tof(state, layout, now)
    elif state.active_tab == config.TAB_IMU:
        sensing.update_imu(state, layout, now)
    elif state.active_tab == config.TAB_CONSOLE:
        console.update(state, layout)
    elif state.active_tab == config.TAB_TUNING:
        tuning.update(state, layout, now)


def run_gui(state, args):
    dpg.create_context()
    rendering.load_fonts()
    rendering.apply_theme()

    picking_port = args.port is None and not args.demo
    scan_results = {}
    scan_lock = threading.Lock()
    scan_active = threading.Event()
    wheel_delta = {"value": 0.0}
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
        if state.scanner is not None:
            state.scanner.join(timeout=2.0)
        with scan_lock:
            scan_results.clear()
        scan_active.set()
        state.scanner = PortScanner(scan_results, scan_lock, scan_active)
        state.scanner.start()

    def on_connect(_sender, _app_data, user_data=None):
        device = picker.device_for_row(user_data)
        if device is not None:
            start_session(device)

    def on_rescan(_sender, _app_data, _user_data=None):
        rescan_ports()

    def on_quit(_sender, _app_data, _user_data=None):
        stop_and_exit(state)

    def on_wheel(_sender, app_data, _user_data=None):
        wheel_delta["value"] += float(app_data)

    if picking_port:
        scan_active.set()
        state.scanner = PortScanner(scan_results, scan_lock, scan_active)
        state.scanner.start()
    elif not state.demo:
        state.reader = BenchReader(state, args.port, args.baud)
        state.reader.start()

    build_dashboard(state)
    picker.build(on_connect, on_rescan, on_quit)
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
    dpg.set_value(TAB_BAR, TAB_TAGS[state.active_tab])
    dpg.render_dearpygui_frame()

    try:
        while not state.quit and dpg.is_dearpygui_running():
            now = time.monotonic()
            for action, value in keys.poll(now):
                perform_action(state, action, value)
                if action == "tab" and not state.quit:
                    select_tab(state, value)
            if state.quit:
                break
            selected = dpg.get_value(TAB_BAR)
            if isinstance(selected, int):
                selected = dpg.get_item_alias(selected)
            if selected in TAB_TAGS:
                state.active_tab = TAB_TAGS.index(selected)

            if not picking_port and state.picker_requested:
                state.picker_requested = False
                if state.reader is not None:
                    state.reader.join(timeout=2.0)
                    state.reader = None
                state.sweep = False
                rescan_ports()
                picking_port = True
            dpg.configure_item(MAIN_WINDOW, show=not picking_port)
            dpg.configure_item(picker.WINDOW, show=picking_port)

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

            if not picking_port:
                state.update_sweep(now)
                layout = compute_layout(dpg.get_viewport_client_width(), dpg.get_viewport_client_height())
                refresh_dashboard(state, layout, now, hz_state, hz_tof, fps)
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
                picker.update(scan_results, scan_lock)
                view_width = dpg.get_viewport_client_width()
                view_height = dpg.get_viewport_client_height()
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
        for worker in (state.reader, state.scanner):
            if worker is not None:
                worker.join(timeout=2.0)
        dpg.destroy_context()
