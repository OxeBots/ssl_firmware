"""Main GUI loop: window, port-picker/dashboard phases and input handling."""

import os
import threading
import time

import pygame

from . import config
from .discovery import PortScanner
from .panels import (
    draw_console_panel,
    draw_motor_panel,
    draw_orientation_panel,
    draw_status_bar,
    draw_tof_panel,
)
from .picker import draw_port_select, select_click_target
from .serial_io import BenchReader


def handle_key(event, state):
    if event.key in (pygame.K_q, pygame.K_ESCAPE):
        stop_and_exit(state)
    elif event.key == pygame.K_UP:
        state.send_command(f"T{state.tgt_rpm + 100:.0f}")
    elif event.key == pygame.K_DOWN:
        state.send_command(f"T{state.tgt_rpm - 100:.0f}")
    elif event.key == pygame.K_RIGHT:
        state.send_command(f"T{state.tgt_rpm + 500:.0f}")
    elif event.key == pygame.K_LEFT:
        state.send_command(f"T{state.tgt_rpm - 500:.0f}")
    elif event.key == pygame.K_SPACE:
        state.send_command("0")
    elif event.key == pygame.K_w:
        state.send_command("W")
    elif event.key == pygame.K_x:
        state.send_command("Q")  # firmware stops the motor and ends the test
        state.console.append("firmware test exit requested (reset board to rerun)")


def stop_and_exit(state):
    if state.scanner is not None:
        state.scanner.active.clear()
    if not state.demo:
        state.send_command("0")  # safety stop; the firmware keeps streaming
    state.quit = True


def run_gui(state, args):
    pygame.init()
    pygame.key.set_repeat(350, 60)
    screen = pygame.display.set_mode((args.width, args.height))
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

    margin = 10
    status_h = 36
    console_h = 158
    left_w = int(args.width * 0.37)
    body_bottom = args.height - status_h - margin * 2
    tof_rect = pygame.Rect(margin, margin, left_w, body_bottom - console_h - margin)
    console_rect = pygame.Rect(margin, tof_rect.bottom + margin, left_w, console_h)
    right_x = margin + left_w + margin
    right_w = args.width - right_x - margin
    orientation_h = int(body_bottom * 0.44)
    orientation_rect = pygame.Rect(right_x, margin, right_w, orientation_h)
    motor_rect = pygame.Rect(right_x, orientation_rect.bottom + margin, right_w, body_bottom - orientation_h - margin)
    status_rect = pygame.Rect(margin, args.height - status_h - margin, args.width - margin * 2, status_h)

    hz_state, hz_tof, fps = 0.0, 0.0, 0.0
    last_hz_t = time.monotonic()
    last_state_count = last_tof_count = 0

    while not state.quit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_and_exit(state)
            elif event.type == pygame.KEYDOWN:
                if phase == "select":
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        stop_and_exit(state)
                else:
                    handle_key(event, state)
            elif event.type == pygame.MOUSEBUTTONDOWN and phase == "select":
                chosen = select_click_target(event.pos, row_rects, buttons, scan_results, scan_lock)
                if chosen == "__quit__":
                    stop_and_exit(state)
                elif chosen is not None:
                    scan_active.clear()
                    args.port = chosen
                    reader = BenchReader(state, chosen, args.baud)
                    state.reader = reader
                    reader.start()
                    phase = "dashboard"

        now = time.monotonic()
        if now - last_hz_t >= 1.0:
            hz_state = state.state_count - last_state_count
            hz_tof = state.tof_count - last_tof_count
            last_state_count, last_tof_count = state.state_count, state.tof_count
            last_hz_t = now
        fps = clock.get_fps()

        screen.fill(config.BACKGROUND)
        if phase == "select":
            draw_port_select(screen, scan_results, scan_lock, fonts, row_rects, buttons)
        else:
            draw_tof_panel(screen, tof_rect, state, fonts, now)
            draw_console_panel(screen, console_rect, state, fonts)
            draw_orientation_panel(screen, orientation_rect, state, fonts)
            draw_motor_panel(screen, motor_rect, state, fonts, now)
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
