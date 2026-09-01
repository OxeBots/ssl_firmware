"""Dashboard panels: ToF heatmap, orientation, motor gauges, console, status."""

import math

import pygame

from . import config
from .rendering import TOF_COLOR_STOPS, blit_text, center_bar, distance_color, draw_cube, panel


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
            cell_rect = pygame.Rect(origin_x + col * cell + 1, origin_y + row * cell + 1, cell - 2, cell - 2)
            pygame.draw.rect(surface, distance_color(distance), cell_rect, border_radius=3)
            if distance > 0 and cell >= 30:
                color = (20, 22, 26) if distance < 1400 else (235, 238, 245)
                blit_text(surface, str(distance), value_font, color, center=cell_rect.center)
    # Legend
    legend_y = rect.bottom - 46
    blit_text(surface, "near", fonts["small"], config.TEXT_DIM, topleft=(rect.x + 14, legend_y))
    x = rect.x + 60
    for mm, _ in TOF_COLOR_STOPS:
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
            surface, "TOF OFFLINE - waiting for frames", fonts["label"], config.BAD_COLOR, center=grid_area.center
        )


def draw_orientation_panel(surface, rect, state, fonts):
    panel(surface, rect, "Orientation - GY-85 AHRS", fonts)
    cube_size = min(rect.h - 52, rect.w // 2)
    cube_area = pygame.Rect(rect.x + 14, rect.y + 40, cube_size, cube_size)
    draw_cube(surface, cube_area, state.yaw, state.pitch, state.roll, fonts)
    read_x = cube_area.right + 24
    y = rect.y + 66
    for label, value in (("YAW", state.yaw), ("PITCH", state.pitch), ("ROLL", state.roll)):
        blit_text(surface, label, fonts["label"], config.TEXT_DIM, topleft=(read_x, y))
        blit_text(surface, f"{value:+7.1f}", fonts["big"], config.ACCENT, topleft=(read_x, y + 24))
        y += 78
    # The S line carries no IMU health flag; treat a non-zero AHRS output as
    # a live sensor (the attitude never sits exactly at zero in flight).
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


def draw_encoder_dial(surface, center, radius, state, fonts):
    pygame.draw.circle(surface, (38, 42, 52), center, radius)
    pygame.draw.circle(surface, config.PANEL_BORDER, center, radius, 1)
    for angle_deg in (0, 90, 180, 270):
        angle = math.radians(angle_deg - 90)
        outer = (center[0] + 0.85 * radius * math.cos(angle), center[1] + 0.85 * radius * math.sin(angle))
        inner = (center[0] + 0.68 * radius * math.cos(angle), center[1] + 0.68 * radius * math.sin(angle))
        pygame.draw.line(surface, config.PANEL_BORDER, inner, outer, 1)
    angle = math.radians(state.enc_raw / 4096.0 * 360.0 - 90.0)
    color = config.ACCENT if state.enc_ok else config.BAD_COLOR
    needle = (center[0] + 0.8 * radius * math.cos(angle), center[1] + 0.8 * radius * math.sin(angle))
    pygame.draw.line(surface, color, center, needle, 2)
    pygame.draw.circle(surface, color, center, 3)
    text_x = center[0] + radius + 10
    if state.enc_ok:
        degrees = state.enc_raw * 360.0 / 4096.0
        blit_text(
            surface,
            f"AS5600 {state.enc_raw:4d}",
            fonts["tiny"],
            config.TEXT_DIM,
            topleft=(text_x, center[1] - 18),
        )
        blit_text(
            surface,
            f"{degrees:5.1f} deg",
            fonts["tiny"],
            config.TEXT_DIM,
            topleft=(text_x, center[1] - 2),
        )
    else:
        blit_text(surface, "AS5600", fonts["tiny"], config.BAD_COLOR, topleft=(text_x, center[1] - 16))
        blit_text(surface, "no I2C link", fonts["tiny"], config.BAD_COLOR, topleft=(text_x, center[1]))


def draw_speed_chart(surface, area, state, fonts, now):
    # The reader thread appends concurrently: prune and copy under the lock.
    with state.history_lock:
        while state.history and state.history[0][0] < now - config.CHART_WINDOW_S:
            state.history.popleft()
        snapshot = tuple(state.history)
    pygame.draw.rect(surface, (24, 27, 34), area, border_radius=6)
    pygame.draw.rect(surface, config.PANEL_BORDER, area, 1, border_radius=6)
    blit_text(surface, "speed (RPM), last 12 s", fonts["small"], config.TEXT_DIM, topleft=(area.x + 10, area.y + 6))
    if len(snapshot) < 2:
        blit_text(
            surface, "waiting for state frames...", fonts["small"], config.TEXT_DIM, center=area.center
        )
        return
    peak = max(600.0, max(max(abs(vel), abs(tgt)) for _t, vel, tgt in snapshot) * 1.15)

    def y_of(value):
        return area.centery - value / peak * (area.h / 2 - 16)

    for fraction, color in (
        (1.0, config.PANEL_BORDER),
        (0.5, (45, 50, 62)),
        (0.0, (70, 76, 92)),
        (-0.5, (45, 50, 62)),
        (-1.0, config.PANEL_BORDER),
    ):
        y = area.centery - fraction * (area.h / 2 - 16)
        pygame.draw.line(surface, color, (area.x + 4, y), (area.right - 4, y), 1)
    blit_text(surface, f"+{peak:.0f}", fonts["tiny"], config.TEXT_DIM, topright=(area.right - 6, area.y + 8))
    blit_text(surface, f"-{peak:.0f}", fonts["tiny"], config.TEXT_DIM, topright=(area.right - 6, area.bottom - 22))

    t0 = now - config.CHART_WINDOW_S

    def x_of(t):
        return area.x + (t - t0) / config.CHART_WINDOW_S * area.w

    points_t = [(x_of(t), y_of(tgt)) for t, _vel, tgt in snapshot]
    points_v = [(x_of(t), y_of(vel)) for t, vel, _tgt in snapshot]
    pygame.draw.lines(surface, config.TARGET_COLOR, False, points_t, 1)
    pygame.draw.lines(surface, config.VELOCITY_COLOR, False, points_v, 2)
    legend_x = area.right - 150
    pygame.draw.line(surface, config.TARGET_COLOR, (legend_x, area.y + 14), (legend_x + 22, area.y + 14), 1)
    blit_text(surface, "target", fonts["tiny"], config.TARGET_COLOR, topleft=(legend_x + 28, area.y + 7))
    pygame.draw.line(surface, config.VELOCITY_COLOR, (legend_x, area.y + 30), (legend_x + 22, area.y + 30), 2)
    blit_text(surface, "speed", fonts["tiny"], config.VELOCITY_COLOR, topleft=(legend_x + 28, area.y + 23))


def draw_motor_panel(surface, rect, state, fonts, now):
    panel(surface, rect, "Motor 1 (M1 / FL) - PWM / direction / speed", fonts)
    gauge_w = 330
    gauges = pygame.Rect(rect.x + 16, rect.y + 42, gauge_w, rect.h - 56)
    chart_area = pygame.Rect(gauges.right + 16, rect.y + 44, rect.right - gauges.right - 30, rect.h - 60)

    # PWM bar with duty percentage
    duty_pct = state.pwm / 1023.0 * 100.0
    blit_text(
        surface,
        f"PWM {state.pwm:+5.0f} / 1023  ({duty_pct:+4.0f}%)",
        fonts["label"],
        config.TEXT,
        topleft=(gauges.x, gauges.y),
    )
    center_bar(surface, pygame.Rect(gauges.x, gauges.y + 26, gauges.w, 26), state.pwm, 1023, config.PWM_COLOR)

    # Direction from the sign of the PID output (it drives the DIR bit)
    if state.pwm > 5:
        direction, dir_color = "CW ->", config.OK_COLOR
    elif state.pwm < -5:
        direction, dir_color = "<- CCW", config.OK_COLOR
    else:
        direction, dir_color = "-- STOP", config.TEXT_DIM
    blit_text(surface, direction, fonts["big"], dir_color, topleft=(gauges.x, gauges.y + 62))

    # Speed bars: target and measured on a shared scale
    speed_y = gauges.y + 116
    span = max(600.0, abs(state.tgt_rpm), abs(state.vel_rpm)) * 1.15
    blit_text(
        surface,
        f"target {state.tgt_rpm:+7.1f} RPM",
        fonts["small"],
        config.TARGET_COLOR,
        topleft=(gauges.x, speed_y),
    )
    center_bar(surface, pygame.Rect(gauges.x, speed_y + 22, gauges.w, 18), state.tgt_rpm, span, config.TARGET_COLOR)
    blit_text(
        surface,
        f"speed  {state.vel_rpm:+7.1f} RPM",
        fonts["small"],
        config.VELOCITY_COLOR,
        topleft=(gauges.x, speed_y + 48),
    )
    center_bar(surface, pygame.Rect(gauges.x, speed_y + 70, gauges.w, 18), state.vel_rpm, span, config.VELOCITY_COLOR)

    # Raw AS5600 angle dial (ground truth from the I2C path)
    draw_encoder_dial(surface, (gauges.x + 60, speed_y + 150), 42, state, fonts)

    draw_speed_chart(surface, chart_area, state, fonts, now)


def draw_console_panel(surface, rect, state, fonts):
    panel(surface, rect, "Serial console", fonts)
    y = rect.bottom - 30
    for line in reversed(state.console):
        color = (
            config.BAD_COLOR if line.startswith("ERR") else (config.TEXT if line.startswith(">") else config.TEXT_DIM)
        )
        blit_text(surface, line, fonts["small"], color, topleft=(rect.x + 12, y))
        y -= 22


def draw_status_bar(surface, rect, state, fonts, hz_state, hz_tof, fps):
    """Bottom bar: link status, stream rates, FPS and key help."""
    panel(surface, rect, "", fonts)
    if state.demo:
        link, color = "DEMO MODE - synthetic data", config.DEMO_COLOR
    elif state.connected:
        link = f"{state.connected}  -  state {hz_state:.0f} Hz, tof {hz_tof:.0f} Hz  -  {fps:.0f} fps"
        color = config.OK_COLOR
    else:
        link, color = "no serial port found - retrying...", config.BAD_COLOR
    blit_text(surface, link, fonts["label"], color, topleft=(rect.x + 12, rect.y + 8))
    help_text = "[UP/DOWN +/-100] [LEFT/RIGHT +/-500] [SPACE stop] [W sweep] [X exit fw] [Q quit]"
    blit_text(surface, help_text, fonts["tiny"], config.TEXT_DIM, topright=(rect.right - 12, rect.y + 12))
