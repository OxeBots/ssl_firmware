"""Dashboard panels: tab bar, motors, health, sensing, console and status."""

import pygame

from . import config
from .rendering import (
    TOF_COLOR_STOPS,
    blit_text,
    fit_text,
    center_bar,
    distance_color,
    draw_cube,
    panel,
)
from .state import WHEEL_NAMES

ERROR_FLAG_NAMES = [
    "comm loss",
    "IMU fail",
    "ToF fail",
    "batt undervolt",
    "batt overvolt",
    "kicker no charge",
    "encoder invalid",
    "temp warning",
    "M0 fault",
    "M1 fault",
    "M2 fault",
    "M3 fault",
]

LIFECYCLE_NAMES = ["BOOT", "SELF_TEST", "DISARMED", "ACTIVE", "DEGRADED", "SAFE_STOP", "CALIBRATION"]
LIFECYCLE_COLORS = {
    3: config.OK_COLOR,
    4: (255, 200, 80),
    5: config.BAD_COLOR,
    6: config.ACCENT,
}


# ---------------------------------------------------------------------------
# Tab bar
# ---------------------------------------------------------------------------

def draw_tab_bar(surface, rect, active_tab, fonts):
    """Draw the three tab buttons; returns [(rect, tab_index), ...]."""
    tab_w = 240
    rects = []
    for index, title in enumerate(config.TAB_TITLES):
        tab_rect = pygame.Rect(rect.x + index * (tab_w + 8), rect.y, tab_w, rect.h)
        active = index == active_tab
        pygame.draw.rect(surface, config.TAB_ACTIVE if active else (36, 40, 50), tab_rect, border_radius=8)
        pygame.draw.rect(
            surface,
            config.ACCENT if active else config.PANEL_BORDER,
            tab_rect,
            2 if active else 1,
            border_radius=8,
        )
        blit_text(
            surface,
            title,
            fonts["title" if active else "label"],
            config.TEXT if active else config.TEXT_DIM,
            center=tab_rect.center,
        )
        rects.append((tab_rect, index))
    return rects


# ---------------------------------------------------------------------------
# Motors & control tab
# ---------------------------------------------------------------------------

def draw_wheel_card(surface, rect, index, state, fonts, now):
    wheel = state.wheels[index]
    selected = index == state.selected_wheel
    border = config.ACCENT if selected else config.PANEL_BORDER
    pygame.draw.rect(surface, config.PANEL, rect, border_radius=8)
    pygame.draw.rect(surface, border, rect, 2 if selected else 1, border_radius=8)
    valid = bool(state.valid_mask & (1 << index))
    title_color = config.OK_COLOR if valid else config.BAD_COLOR
    blit_text(
        surface,
        f"{WHEEL_NAMES[index]}" + ("" if valid else "  (enc n/cal)"),
        fonts["title"],
        title_color,
        topleft=(rect.x + 14, rect.y + 10),
    )
    inner = pygame.Rect(rect.x + 16, rect.y + 38, rect.w - 32, rect.h - 48)
    duty = wheel["pwm"] / 1023.0 * 100.0
    blit_text(
        surface,
        f"PWM {wheel['pwm']:+5.0f} / 1023  ({duty:+4.0f}%)",
        fonts["small"],
        config.TEXT,
        topleft=(inner.x, inner.y),
    )
    center_bar(
        surface, pygame.Rect(inner.x, inner.y + 20, inner.w, 18), wheel["pwm"], 1023, config.PWM_COLOR
    )
    if wheel["pwm"] > 5:
        direction, dir_color = "CW ->", config.OK_COLOR
    elif wheel["pwm"] < -5:
        direction, dir_color = "<- CCW", config.OK_COLOR
    else:
        direction, dir_color = "-- STOP", config.TEXT_DIM
    span = max(600.0, abs(wheel["tgt"]), abs(wheel["vel"])) * 1.15
    blit_text(
        surface,
        f"target {wheel['tgt']:+8.1f} RPM",
        fonts["small"],
        config.TARGET_COLOR,
        topleft=(inner.x, inner.y + 44),
    )
    center_bar(
        surface, pygame.Rect(inner.x, inner.y + 62, inner.w, 14), wheel["tgt"], span, config.TARGET_COLOR
    )
    blit_text(
        surface,
        f"speed  {wheel['vel']:+8.1f} RPM",
        fonts["small"],
        config.VELOCITY_COLOR,
        topleft=(inner.x, inner.y + 80),
    )
    center_bar(
        surface, pygame.Rect(inner.x, inner.y + 98, inner.w, 14), wheel["vel"], span, config.VELOCITY_COLOR
    )
    blit_text(surface, direction, fonts["small"], dir_color, topright=(rect.right - 14, rect.y + 12))
    # RPM strip chart inside the motor card, below the readouts.
    chart = pygame.Rect(rect.x + 14, rect.y + 152, rect.w - 28, rect.h - 162)
    draw_speed_chart(surface, chart, state, fonts, now, index, title="speed (RPM)")


def draw_speed_chart(surface, area, state, fonts, now, index, title=None):
    """Strip chart of target vs measured speed for one wheel."""
    with state.history_lock:
        history = tuple(state.history[index])
        while history and history[0][0] < now - config.CHART_WINDOW_S:
            history = history[1:]
    pygame.draw.rect(surface, (24, 27, 34), area, border_radius=6)
    pygame.draw.rect(surface, config.PANEL_BORDER, area, 1, border_radius=6)
    blit_text(
        surface,
        title if title is not None else f"{WHEEL_NAMES[index]} speed (RPM)",
        fonts["small"],
        config.TEXT_DIM,
        topleft=(area.x + 10, area.y + 6),
    )
    if len(history) < 2:
        blit_text(
            surface, "waiting for state frames...", fonts["small"], config.TEXT_DIM, center=area.center
        )
        return
    peak = max(600.0, max(max(abs(vel), abs(tgt)) for _t, vel, tgt in history) * 1.15)

    # Keep the plot between the title band and the bottom peak label.
    inset = (area.h - 48) / 2

    def y_of(value):
        return area.centery - value / peak * inset

    for fraction, color in (
        (1.0, config.PANEL_BORDER),
        (0.5, (45, 50, 62)),
        (0.0, (70, 76, 92)),
        (-0.5, (45, 50, 62)),
        (-1.0, config.PANEL_BORDER),
    ):
        y = area.centery - fraction * inset
        pygame.draw.line(surface, color, (area.x + 4, y), (area.right - 4, y), 1)
    blit_text(surface, f"+{peak:.0f}", fonts["tiny"], config.TEXT_DIM, topright=(area.right - 6, area.y + 8))
    blit_text(
        surface, f"-{peak:.0f}", fonts["tiny"], config.TEXT_DIM, topright=(area.right - 6, area.bottom - 22)
    )

    t0 = now - config.CHART_WINDOW_S

    def x_of(t):
        return area.x + (t - t0) / config.CHART_WINDOW_S * area.w

    points_t = [(x_of(t), y_of(tgt)) for t, _vel, tgt in history]
    points_v = [(x_of(t), y_of(vel)) for t, vel, _tgt in history]
    pygame.draw.lines(surface, config.TARGET_COLOR, False, points_t, 1)
    pygame.draw.lines(surface, config.VELOCITY_COLOR, False, points_v, 2)
    # Compact one-row legend under the title, only when the chart is tall
    # enough to fit it between the gridlines.
    if area.h >= 150 and area.w >= 200:
        legend_y = area.y + 30
        pygame.draw.line(surface, config.TARGET_COLOR, (area.x + 10, legend_y + 6), (area.x + 28, legend_y + 6), 1)
        blit_text(surface, "target", fonts["tiny"], config.TARGET_COLOR, topleft=(area.x + 32, legend_y))
        pygame.draw.line(surface, config.VELOCITY_COLOR, (area.x + 95, legend_y + 6), (area.x + 113, legend_y + 6), 2)
        blit_text(surface, "speed", fonts["tiny"], config.VELOCITY_COLOR, topleft=(area.x + 117, legend_y))


def draw_control_panel(surface, rect, state, fonts):
    panel(surface, rect, f"Control - {WHEEL_NAMES[state.selected_wheel]}", fonts)
    x = rect.x + 16
    y = rect.y + 48
    blit_text(
        surface,
        f"target: {state.wheels[state.selected_wheel]['tgt']:+8.1f} RPM",
        fonts["big"],
        config.ACCENT,
        topleft=(x, y),
    )
    blit_text(
        surface,
        f"sweep: {'ON (+/-1000 RPM)' if state.sweep else 'off'}",
        fonts["label"],
        config.OK_COLOR if state.sweep else config.TEXT_DIM,
        topleft=(x, y + 48),
    )
    help_y = y + 92
    for line in (
        "[1-4] wheel   [UP/DOWN] +/-100",
        "[LEFT/RIGHT] +/-500",
        "[SPACE] stop all   [W] sweep",
        "[Q] quit dashboard",
    ):
        blit_text(surface, line, fonts["small"], config.TEXT_DIM, topleft=(x, help_y))
        help_y += 26


def draw_motors_tab(surface, rect, state, fonts, now):
    # Each wheel card embeds its RPM strip chart; the control panel keeps the
    # full-height column on the right.
    control_w = max(400, min(470, int(rect.w * 0.31)))
    grid_w = rect.w - control_w - 16
    card_w = (grid_w - 12) // 2
    card_h = (rect.h - 12) // 2
    for index in range(4):
        col, row = index % 2, index // 2
        card = pygame.Rect(
            rect.x + col * (card_w + 12), rect.y + row * (card_h + 12), card_w, card_h
        )
        draw_wheel_card(surface, card, index, state, fonts, now)
    draw_control_panel(
        surface,
        pygame.Rect(rect.right - control_w, rect.y, control_w, rect.h),
        state,
        fonts,
    )


# ---------------------------------------------------------------------------
# Health & state tab
# ---------------------------------------------------------------------------

def _status_row(surface, x, y, label, value, color=None, fonts=None):
    blit_text(surface, label, fonts["small"], config.TEXT_DIM, topleft=(x, y))
    blit_text(
        surface,
        str(value),
        fonts["label"],
        color if color is not None else config.TEXT,
        topleft=(x + 170, y - 2),
    )

def draw_system_panel(surface, rect, state, fonts):
    panel(surface, rect, "System", fonts)
    health = state.health
    x = rect.x + 16
    y = rect.y + 44
    _status_row(surface, x, y, "robot ID", health.get("robot_id", "-"), fonts=fonts)
    y += 34
    uptime_s = state.uptime_ms // 1000
    _status_row(
        surface, x, y, "uptime", f"{uptime_s // 3600:02d}:{uptime_s % 3600 // 60:02d}:{uptime_s % 60:02d}", fonts=fonts
    )
    y += 34
    free = health.get("free_heap")
    _status_row(
        surface,
        x,
        y,
        "free heap",
        f"{free / 1024:.0f} KB" if free is not None else "-",
        fonts=fonts,
    )
    y += 34
    minimum = health.get("min_free_heap")
    _status_row(
        surface,
        x,
        y,
        "min heap",
        f"{minimum / 1024:.0f} KB" if minimum is not None else "-",
        fonts=fonts,
    )
    y += 34
    _status_row(surface, x, y, "state frames", state.state_count, fonts=fonts)
    y += 34
    _status_row(surface, x, y, "ToF frames", state.tof_count, fonts=fonts)

def draw_safety_panel(surface, rect, state, fonts):
    panel(surface, rect, "Safety", fonts)
    health = state.health
    x = rect.x + 16
    y = rect.y + 44
    lifecycle = health.get("lifecycle")
    if lifecycle is None:
        name, color = "-", config.TEXT_DIM
    else:
        name = LIFECYCLE_NAMES[lifecycle] if lifecycle < len(LIFECYCLE_NAMES) else f"?{lifecycle}"
        color = LIFECYCLE_COLORS.get(lifecycle, config.TEXT)
    _status_row(surface, x, y, "lifecycle", name, color=color, fonts=fonts)
    y += 36
    motion = health.get("motion_inhibited")
    kick = health.get("kick_inhibited")
    _status_row(
        surface,
        x,
        y,
        "motion inhibited",
        "yes" if motion else "no",
        color=config.BAD_COLOR if motion else config.OK_COLOR,
        fonts=fonts,
    )
    y += 34
    _status_row(
        surface,
        x,
        y,
        "kick inhibited",
        "yes" if kick else "no",
        color=config.BAD_COLOR if kick else config.OK_COLOR,
        fonts=fonts,
    )
    y += 38
    blit_text(surface, "active error flags", fonts["small"], config.TEXT_DIM, topleft=(x, y))
    error_flags = health.get("error_flags")
    y += 24
    if error_flags is None:
        blit_text(surface, "-", fonts["label"], config.TEXT_DIM, topleft=(x, y))
    elif error_flags == 0:
        blit_text(surface, "none", fonts["label"], config.OK_COLOR, topleft=(x, y))
    else:
        for bit, name in enumerate(ERROR_FLAG_NAMES):
            if not error_flags & (1 << bit):
                continue
            blit_text(
                surface, f"bit {bit}: {name}", fonts["small"], config.BAD_COLOR, topleft=(x, y)
            )
            y += 22

def draw_power_panel(surface, rect, state, fonts):
    panel(surface, rect, "Power & Sensors", fonts)
    health = state.health
    x = rect.x + 16
    y = rect.y + 48
    supply = health.get("supply_voltage")
    flags = health.get("supply_flags")
    if supply is None:
        supply_text, color = "-", config.TEXT_DIM
    else:
        supply_text = f"{supply:5.2f} V"
        color = config.TEXT
        if flags is not None and flags & 2:
            color = config.BAD_COLOR
        elif flags is not None and flags & 4:
            color = config.BAD_COLOR
    blit_text(surface, "supply", fonts["small"], config.TEXT_DIM, topleft=(x, y))
    blit_text(surface, supply_text, fonts["big"], color, topleft=(x + 110, y - 8))
    y += 48
    if flags is not None and flags & 2:
        blit_text(surface, "UNDERVOLTAGE", fonts["label"], config.BAD_COLOR, topleft=(x, y))
    elif flags is not None and flags & 4:
        blit_text(surface, "OVERVOLTAGE", fonts["label"], config.BAD_COLOR, topleft=(x, y))
    elif flags is not None and flags & 1:
        blit_text(surface, "supply OK", fonts["label"], config.OK_COLOR, topleft=(x, y))
    else:
        blit_text(surface, "VMON not available", fonts["label"], config.TEXT_DIM, topleft=(x, y))
    y += 44
    cal = health.get("imu_cal_flags")
    if cal is None:
        cal_text, cal_color = "-", config.TEXT_DIM
    else:
        cal_text = (
            f"{'A' if cal & 1 else '-'} {'G' if cal & 2 else '-'} {'M' if cal & 4 else '-'}"
        )
        cal_color = config.OK_COLOR if cal == 7 else config.TEXT
    _status_row(surface, x, y, "IMU calibrated", cal_text, color=cal_color, fonts=fonts)
    y += 34
    tof = health.get("tof_available")
    _status_row(
        surface,
        x,
        y,
        "ToF sensor",
        "online" if tof else "offline",
        color=config.OK_COLOR if tof else config.BAD_COLOR,
        fonts=fonts,
    )

def draw_health_tab(surface, rect, state, fonts):
    col_w = (rect.w - 24) // 3
    draw_system_panel(surface, pygame.Rect(rect.x, rect.y, col_w, rect.h), state, fonts)
    draw_safety_panel(
        surface, pygame.Rect(rect.x + col_w + 12, rect.y, col_w, rect.h), state, fonts
    )
    draw_power_panel(
        surface,
        pygame.Rect(rect.x + 2 * (col_w + 12), rect.y, rect.w - 2 * (col_w + 12), rect.h),
        state,
        fonts,
    )


# ---------------------------------------------------------------------------
# Sensing tab
# ---------------------------------------------------------------------------

def draw_tof_panel(surface, rect, state, fonts, now):
    panel(surface, rect, "VL53L5CX ToF - 8x8 distances (mm)", fonts)
    stale = now - state.last_tof_t > 1.0
    frame = [0] * (config.TOF_SIDE * config.TOF_SIDE) if stale else state.tof
    grid_area = pygame.Rect(rect.x + 14, rect.y + 38, rect.w - 28, rect.h - 38 - 64)
    cell = max(10, min(grid_area.w, grid_area.h) // config.TOF_SIDE)
    origin_x = rect.centerx - cell * config.TOF_SIDE // 2
    origin_y = grid_area.y + max(0, (grid_area.h - cell * config.TOF_SIDE) // 2)
    value_font = fonts["small"] if cell >= 52 else fonts["tiny"]
    for row in range(config.TOF_SIDE):
        for col in range(config.TOF_SIDE):
            distance = frame[row * config.TOF_SIDE + col]
            cell_rect = pygame.Rect(
                origin_x + col * cell + 1, origin_y + row * cell + 1, cell - 2, cell - 2
            )
            pygame.draw.rect(surface, distance_color(distance), cell_rect, border_radius=3)
            if distance > 0 and cell >= 30:
                color = (20, 22, 26) if distance < 1400 else (235, 238, 245)
                blit_text(surface, str(distance), value_font, color, center=cell_rect.center)
    legend_y = rect.bottom - 46
    blit_text(surface, "near", fonts["small"], config.TEXT_DIM, topleft=(rect.x + 14, legend_y))
    x = rect.x + 60
    for mm, _color in TOF_COLOR_STOPS:
        pygame.draw.rect(surface, distance_color(mm), (x, legend_y + 2, 26, 16), border_radius=3)
        blit_text(surface, str(int(mm)), fonts["tiny"], config.TEXT_DIM, center=(x + 13, legend_y + 34))
        x += 34
    blit_text(surface, "4 m", fonts["small"], config.TEXT_DIM, topleft=(x + 6, legend_y))
    blit_text(
        surface,
        f"frame #{state.tof_seq_seen}",
        fonts["small"],
        config.TEXT_DIM,
        topright=(rect.right - 14, legend_y),
    )
    if stale:
        blit_text(
            surface,
            "TOF OFFLINE - waiting for frames",
            fonts["label"],
            config.BAD_COLOR,
            center=grid_area.center,
        )

def draw_orientation_panel(surface, rect, state, fonts):
    panel(surface, rect, "Orientation - GY-85 AHRS", fonts)
    cube_size = min(rect.h - 52, rect.w // 2)
    cube_area = pygame.Rect(rect.x + 14, rect.y + 40, cube_size, cube_size)
    draw_cube(surface, cube_area, state.yaw, state.pitch, state.roll, fonts)
    read_x = cube_area.right + 24
    # Scale the row spacing so the readouts never collide with the status
    # line at the bottom of the panel.
    step = max(56, (rect.h - 40 - 42) // 3)
    y = rect.y + 46
    for label, value in (("YAW", state.yaw), ("PITCH", state.pitch), ("ROLL", state.roll)):
        blit_text(surface, label, fonts["label"], config.TEXT_DIM, topleft=(read_x, y))
        blit_text(surface, f"{value:+7.1f}", fonts["big"], config.ACCENT, topleft=(read_x, y + 22))
        y += step
    imu_live = abs(state.yaw) + abs(state.pitch) + abs(state.roll) > 0.01
    if imu_live:
        blit_text(surface, "IMU link OK", fonts["small"], config.OK_COLOR, topleft=(read_x, rect.bottom - 34))
    else:
        blit_text(
            surface,
            "IMU attitude all zero - check GY-85",
            fonts["small"],
            config.BAD_COLOR,
            topleft=(read_x, rect.bottom - 34),
        )

def draw_raw_sensors_panel(surface, rect, state, fonts):
    panel(surface, rect, "Raw sensors", fonts)
    rows = [
        ("accel (g)", state.accel, config.ACCENT),
        ("gyro (deg/s)", state.gyro, config.OK_COLOR),
        ("mag (mG)", state.mag, config.PWM_COLOR),
    ]
    x = rect.x + 16
    y = rect.y + 44
    # Narrow panels drop to the smaller font so the values stay inside.
    value_font = fonts["big"] if rect.w >= 680 else fonts["label"]
    value_x = x + 150 if rect.w >= 680 else x + 130
    for label, values, color in rows:
        blit_text(surface, label, fonts["label"], config.TEXT_DIM, topleft=(x, y))
        blit_text(
            surface,
            f"{values[0]:+8.3f}  {values[1]:+8.3f}  {values[2]:+8.3f}",
            value_font,
            color,
            topleft=(value_x, y - 6 if value_font is fonts["big"] else y + 1),
        )
        y += 44

def draw_sensing_tab(surface, rect, state, fonts, now):
    left_w = int(rect.w * 0.42)
    draw_tof_panel(surface, pygame.Rect(rect.x, rect.y, left_w, rect.h), state, fonts, now)
    right_x = rect.x + left_w + 12
    right_w = rect.right - right_x
    cube_h = int(rect.h * 0.55)
    draw_orientation_panel(
        surface, pygame.Rect(right_x, rect.y, right_w, cube_h), state, fonts
    )
    draw_raw_sensors_panel(
        surface,
        pygame.Rect(right_x, rect.y + cube_h + 12, right_w, rect.h - cube_h - 12),
        state,
        fonts,
    )


# ---------------------------------------------------------------------------
# Shared bottom strip
# ---------------------------------------------------------------------------

CONSOLE_LINE_H = 21
CONSOLE_SELECT_COLOR = (56, 90, 148)


def console_visible_lines(state, rect):
    """Console lines drawn in the panel and the first line's history index."""
    max_lines = max(1, (rect.h - 36) // CONSOLE_LINE_H)
    lines = list(state.console)
    if not lines:
        return [], 0
    end = max(1, min(len(lines), len(lines) - state.console_offset))
    start = max(0, end - max_lines)
    return lines[start:end], start


def console_hit_test(state, rect, fonts, pos):
    """(history_line_index, char_index) under pos, or None when empty."""
    lines, start = console_visible_lines(state, rect)
    if not lines:
        return None
    # The last visible line sits at rect.bottom - 28, older ones above it.
    row = len(lines) - 1 - int(round((rect.bottom - 28 - pos[1]) / CONSOLE_LINE_H))
    row = max(0, min(len(lines) - 1, row))
    text = lines[row]
    x = pos[0] - (rect.x + 12)
    char = len(text)
    for i in range(len(text) + 1):
        if fonts["small"].size(text[:i])[0] >= x:
            char = i
            break
    return start + row, char


def console_selection_text(state):
    """Clipboard text for the active selection ('' when empty)."""
    if not state.console_sel:
        return ""
    a_line, a_ch = state.console_sel["anchor"]
    b_line, b_ch = state.console_sel["focus"]
    if (a_line, a_ch) > (b_line, b_ch):
        (a_line, a_ch), (b_line, b_ch) = (b_line, b_ch), (a_line, a_ch)
    lines = list(state.console)
    a_line = max(0, a_line)
    b_line = min(len(lines) - 1, b_line)
    if not lines or a_line > b_line:
        return ""
    if a_line == b_line:
        return lines[a_line][a_ch:b_ch]
    parts = [lines[a_line][a_ch:]]
    parts.extend(lines[i] for i in range(a_line + 1, b_line))
    parts.append(lines[b_line][:b_ch])
    return "\n".join(parts)


def draw_console_panel(surface, rect, state, fonts):
    title = "Serial console"
    if state.console_offset > 0:
        title += "  (history - wheel down for live)"
    panel(surface, rect, title, fonts)
    lines, start = console_visible_lines(state, rect)
    if not lines:
        return
    a_line = b_line = a_ch = b_ch = 0
    if state.console_sel:
        a_line, a_ch = state.console_sel["anchor"]
        b_line, b_ch = state.console_sel["focus"]
        if (a_line, a_ch) > (b_line, b_ch):
            (a_line, a_ch), (b_line, b_ch) = (b_line, b_ch), (a_line, a_ch)
    y = rect.bottom - 28 - (len(lines) - 1) * CONSOLE_LINE_H
    x0 = rect.x + 12
    for offset_idx, raw in enumerate(lines):
        abs_i = start + offset_idx
        color = (
            config.BAD_COLOR if raw.startswith("ERR") else (config.TEXT if raw.startswith(">") else config.TEXT_DIM)
        )
        text = fit_text(raw, fonts["small"], rect.w - 24)
        if state.console_sel and a_line <= abs_i <= b_line:
            left_ch = 0 if abs_i > a_line else min(a_ch, len(text))
            right_ch = len(text) if abs_i < b_line else min(b_ch, len(text))
            if right_ch > left_ch:
                xa = x0 + fonts["small"].size(text[:left_ch])[0]
                xb = x0 + fonts["small"].size(text[:right_ch])[0]
                pygame.draw.rect(
                    surface, CONSOLE_SELECT_COLOR, (xa, y, xb - xa, CONSOLE_LINE_H - 2)
                )
        blit_text(surface, text, fonts["small"], color, topleft=(x0, y))
        y += CONSOLE_LINE_H

def draw_status_bar(surface, rect, state, fonts, hz_state, hz_tof, fps):
    """Bottom bar: link status, stream rates, FPS."""
    panel(surface, rect, "", fonts)
    if state.demo:
        link, color = "DEMO MODE - synthetic data", config.DEMO_COLOR
    elif state.connected and state.streaming:
        link = (
            f"{state.connected}  -  state {hz_state:.0f} Hz, tof {hz_tof:.0f} Hz"
            f"  -  RX {state.rx_bytes} B  -  {fps:.0f} fps"
        )
        color = config.OK_COLOR
    elif state.connected:
        link = f"{state.connected}  -  waiting for stream... (RX {state.rx_bytes} B)"
        color = config.TEXT_DIM
    else:
        link, color = "no serial port found - retrying...", config.BAD_COLOR
    uptime = state.uptime_ms
    uptime_text = f"fw uptime {uptime // 1000}s" if uptime else ""
    max_link_w = rect.w - 24
    if uptime_text:
        max_link_w -= fonts["tiny"].size(uptime_text)[0] + 20
    blit_text(
        surface,
        fit_text(link, fonts["label"], max_link_w),
        fonts["label"],
        color,
        topleft=(rect.x + 12, rect.y + 8),
    )
    if uptime_text:
        blit_text(
            surface,
            uptime_text,
            fonts["tiny"],
            config.TEXT_DIM,
            topright=(rect.right - 12, rect.y + 12),
        )
