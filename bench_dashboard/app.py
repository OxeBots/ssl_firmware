"""Main GUI loop: resizable window, tabs, port picker and input handling."""

import os
import threading
import time

import pygame

from . import config
from .discovery import PortScanner
from .panels import (
    console_hit_test,
    console_selection_text,
    draw_console_panel,
    draw_health_tab,
    draw_motors_tab,
    draw_sensing_tab,
    draw_status_bar,
    draw_tab_bar,
)
from .picker import draw_port_select, select_click_target
from .serial_io import BenchReader

TAB_KEYS = {pygame.K_F1: 0, pygame.K_F2: 1, pygame.K_F3: 2}
WHEEL_KEYS = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2, pygame.K_4: 3}


def handle_key(event, state):
    if event.key in (pygame.K_q, pygame.K_ESCAPE):
        stop_and_exit(state)
    elif event.key in TAB_KEYS:
        state.active_tab = TAB_KEYS[event.key]
    elif event.key in WHEEL_KEYS:
        state.selected_wheel = WHEEL_KEYS[event.key]
    elif event.key == pygame.K_UP:
        state.set_selected_target(state.wheels[state.selected_wheel]["tgt"] + 100)
    elif event.key == pygame.K_DOWN:
        state.set_selected_target(state.wheels[state.selected_wheel]["tgt"] - 100)
    elif event.key == pygame.K_RIGHT:
        state.set_selected_target(state.wheels[state.selected_wheel]["tgt"] + 500)
    elif event.key == pygame.K_LEFT:
        state.set_selected_target(state.wheels[state.selected_wheel]["tgt"] - 500)
    elif event.key == pygame.K_SPACE:
        state.stop_motors()
    elif event.key == pygame.K_w:
        state.sweep = not state.sweep
        state.console.append("sweep " + ("ON" if state.sweep else "OFF"))


def stop_and_exit(state):
    if state.scanner is not None:
        state.scanner.active.clear()
    if not state.demo:
        state.stop_motors()  # safety stop; the reader also closes the stream
    state.quit = True


def compute_layout(width, height):
    """Panel rectangles for the current window size."""
    margin = 10
    tab_h = 46
    bottom_h = 116
    gap = 8
    tab_rect = pygame.Rect(margin, margin, width - margin * 2, tab_h)
    body_y = margin + tab_h + gap
    body_h = height - body_y - bottom_h - gap - margin
    body_rect = pygame.Rect(margin, body_y, width - margin * 2, body_h)
    bottom_y = body_y + body_h + gap
    console_w = int(width * 0.42)
    console_rect = pygame.Rect(margin, bottom_y, console_w, bottom_h)
    status_rect = pygame.Rect(
        console_rect.right + gap, bottom_y, width - console_w - gap * 2 - margin * 2, bottom_h,
    )
    return tab_rect, body_rect, console_rect, status_rect


def run_gui(state, args):
    pygame.init()
    try:
        pygame.scrap.init()
    except Exception:
        pass  # copy falls back to a console note when the clipboard is unavailable
    pygame.key.set_repeat(350, 60)
    width = max(args.width, config.MIN_WINDOW[0])
    height = max(args.height, config.MIN_WINDOW[1])
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    pygame.display.set_caption("SSL Bench Dashboard")
    clock = pygame.time.Clock()
    fonts = {
        "title": pygame.font.SysFont(config.FONT_NAMES, 22, bold=True),
        "big": pygame.font.SysFont(config.FONT_NAMES, 30, bold=True),
        "label": pygame.font.SysFont(config.FONT_NAMES, 20),
        "small": pygame.font.SysFont(config.FONT_NAMES, 17),
        "tiny": pygame.font.SysFont(config.FONT_NAMES, 14),
    }

    screenshot_dir = None
    screenshot_index = 0
    next_screenshot_t = 0.0
    if args.screenshot_dir:
        screenshot_dir = args.screenshot_dir
        os.makedirs(screenshot_dir, exist_ok=True)

    # Port picker phase runs until a port is chosen (--port/--demo skip it).
    phase = "select" if (args.port is None and not args.demo) else "dashboard"
    scan_results = {}
    scan_lock = threading.Lock()
    scan_active = threading.Event()
    row_rects = []
    buttons = {}
    if phase == "select":
        scan_active.set()
        state.scanner = PortScanner(scan_results, scan_lock, scan_active)
        state.scanner.start()
    elif not state.demo and args.port:
        reader = BenchReader(state, args.port, args.baud)
        state.reader = reader
        reader.start()

    hz_state, hz_tof, fps = 0.0, 0.0, 0.0
    last_hz_t = time.monotonic()
    last_state_count = last_tof_count = 0

    while not state.quit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_and_exit(state)
            elif event.type == pygame.VIDEORESIZE:
                width = max(event.w, config.MIN_WINDOW[0])
                height = max(event.h, config.MIN_WINDOW[1])
                screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if phase == "select":
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        stop_and_exit(state)
                else:
                    handle_key(event, state)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if phase == "select":
                    chosen = select_click_target(event.pos, row_rects, buttons, scan_results, scan_lock)
                    if chosen == "__quit__":
                        stop_and_exit(state)
                    elif chosen is not None:
                        scan_active.clear()
                        args.port = chosen
                        # Let an in-flight probe release the port before the
                        # reader opens it (two open handles corrupt the reads).
                        if state.scanner is not None:
                            state.scanner.join(timeout=2.0)
                        reader = BenchReader(state, chosen, args.baud)
                        state.reader = reader
                        reader.start()
                        phase = "dashboard"
                else:
                    tab_rect, _body, console_rect, _status = compute_layout(width, height)
                    if event.button == 1 and console_rect.collidepoint(event.pos):
                        # Terminal-style text selection in the console.
                        hit = console_hit_test(state, console_rect, fonts, event.pos)
                        state.console_sel = {"anchor": hit, "focus": hit} if hit else None
                    else:
                        if state.console_sel:
                            state.console_sel = None
                        for rect, index in draw_tab_bar(screen, tab_rect, state.active_tab, fonts):
                            if rect.collidepoint(event.pos):
                                state.active_tab = index
            elif event.type == pygame.MOUSEBUTTONUP:
                if phase == "dashboard" and event.button == 1 and state.console_sel:
                    text = console_selection_text(state)
                    if text:
                        try:
                            pygame.scrap.put_text(text)
                            state.add_console(f"> copied {len(text)} chars to clipboard")
                        except Exception:
                            state.add_console("> clipboard unavailable")
            elif event.type == pygame.MOUSEMOTION:
                if phase == "dashboard" and state.console_sel and event.buttons[0]:
                    _t, _b, console_rect, _s = compute_layout(width, height)
                    hit = console_hit_test(state, console_rect, fonts, event.pos)
                    if hit:
                        state.console_sel["focus"] = hit
            elif event.type == pygame.MOUSEWHEEL:
                if phase == "dashboard":
                    _t, _b, console_rect, _s = compute_layout(width, height)
                    if console_rect.collidepoint(pygame.mouse.get_pos()):
                        # Wheel up scrolls into history, wheel down towards live.
                        state.console_offset = max(0, state.console_offset - event.y * 3)

        if phase == "dashboard" and state.picker_requested:
            # The reader gave up on the requested port (cable pulled, board
            # back on another port): rescan and let the user pick again.
            state.picker_requested = False
            if state.reader is not None:
                state.reader.join(timeout=2.0)
                state.reader = None
            scan_results.clear()
            scan_active.set()
            state.scanner = PortScanner(scan_results, scan_lock, scan_active)
            state.scanner.start()
            phase = "select"

        now = time.monotonic()
        if now - last_hz_t >= 1.0:
            hz_state = state.state_count - last_state_count
            hz_tof = state.tof_count - last_tof_count
            last_state_count, last_tof_count = state.state_count, state.tof_count
            last_hz_t = now
        fps = clock.get_fps()

        state.update_sweep(now)

        screen.fill(config.BACKGROUND)
        if phase == "select":
            draw_port_select(screen, scan_results, scan_lock, fonts, row_rects, buttons)
        else:
            tab_rect, body_rect, console_rect, status_rect = compute_layout(width, height)
            draw_tab_bar(screen, tab_rect, state.active_tab, fonts)
            if state.active_tab == 0:
                draw_motors_tab(screen, body_rect, state, fonts, now)
            elif state.active_tab == 1:
                draw_health_tab(screen, body_rect, state, fonts)
            else:
                draw_sensing_tab(screen, body_rect, state, fonts, now)
            draw_console_panel(screen, console_rect, state, fonts)
            draw_status_bar(screen, status_rect, state, fonts, hz_state, hz_tof, fps)
        pygame.display.flip()
        if screenshot_dir is not None and now >= next_screenshot_t:
            next_screenshot_t = now + args.screenshot_period
            pygame.image.save(
                screen, os.path.join(screenshot_dir, f"bench_gui_{screenshot_index:03d}.png")
            )
            screenshot_index += 1
        clock.tick(40)

    pygame.quit()
