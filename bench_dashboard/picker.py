"""First-screen port picker: scan result rendering and click resolution."""

import pygame

from . import config
from .rendering import blit_text

STATUS_ORDER = {"firmware": 0, "probing": 1, "empty": 2, "busy": 3}
STATUS_TEXT = {
    "firmware": ("firmware found", config.OK_COLOR),
    "probing": ("scanning...", config.TEXT_DIM),
    "empty": ("no response (other monitor on port?)", config.TEXT_DIM),
    "busy": ("in use", config.BAD_COLOR),
}


def select_click_target(pos, row_rects, buttons, scan_results, scan_lock):
    """Resolve a picker click: a port name, "__quit__", or None.

    Rows are only clickable when the probe flagged the firmware; a port with
    no response cannot open a working dashboard session.
    """
    for rect, device in row_rects:
        if rect.collidepoint(pos):
            with scan_lock:
                info = scan_results.get(device)
            if info is not None and info["status"] == "firmware":
                return device
            return None
    if buttons.get("rescan") is not None and buttons["rescan"].collidepoint(pos):
        with scan_lock:
            scan_results.clear()
    elif buttons.get("quit") is not None and buttons["quit"].collidepoint(pos):
        return "__quit__"
    return None


def draw_port_select(surface, scan_results, scan_lock, fonts, row_rects, buttons):
    """Port picker: lists USB serial devices and flags the one with the firmware."""
    surface.fill(config.BACKGROUND)
    panel = pygame.Rect(0, 0, 700, 480)
    panel.center = surface.get_rect().center
    pygame.draw.rect(surface, config.PANEL, panel, border_radius=10)
    pygame.draw.rect(surface, config.PANEL_BORDER, panel, 1, border_radius=10)
    blit_text(surface, "SSL Bench Dashboard", fonts["big"], config.TEXT, center=(panel.centerx, panel.y + 46))
    blit_text(
        surface,
        "Pick the serial port running the BenchDashboard firmware:",
        fonts["small"],
        config.TEXT_DIM,
        center=(panel.centerx, panel.y + 86),
    )

    with scan_lock:
        entries = sorted(
            scan_results.items(), key=lambda item: (STATUS_ORDER.get(item[1]["status"], 9), item[0])
        )
    row_rects.clear()
    row_h, gap, list_top = 62, 8, panel.y + 112
    for index, (device, info) in enumerate(entries[:5]):
        row = pygame.Rect(panel.x + 24, list_top + index * (row_h + gap), panel.w - 48, row_h)
        found = info["status"] == "firmware"
        pygame.draw.rect(surface, (36, 46, 42) if found else (38, 42, 52), row, border_radius=6)
        pygame.draw.rect(surface, config.OK_COLOR if found else config.PANEL_BORDER, row, 1, border_radius=6)
        blit_text(surface, device, fonts["label"], config.TEXT, topleft=(row.x + 14, row.y + 8))
        blit_text(surface, info["desc"], fonts["tiny"], config.TEXT_DIM, topleft=(row.x + 14, row.y + 36))
        status_text, status_color = STATUS_TEXT.get(info["status"], ("...", config.TEXT_DIM))
        blit_text(surface, status_text, fonts["small"], status_color, topright=(row.right - 122, row.y + 12))
        if found:
            chip = pygame.Rect(row.right - 106, row.y + 14, 92, 34)
            pygame.draw.rect(surface, (52, 96, 66), chip, border_radius=6)
            blit_text(surface, "Connect", fonts["small"], config.TEXT, center=chip.center)
        row_rects.append((row, device))
    if not entries:
        blit_text(
            surface,
            "No USB serial devices found - plug in the rig and wait for the rescan",
            fonts["small"],
            config.TEXT_DIM,
            center=(panel.centerx, list_top + 70),
        )
    elif len(entries) > 5:
        blit_text(
            surface,
            f"+{len(entries) - 5} more devices",
            fonts["tiny"],
            config.TEXT_DIM,
            center=(panel.centerx, list_top + 5 * (row_h + gap)),
        )

    buttons.clear()
    rescan = pygame.Rect(panel.x + 24, panel.bottom - 58, 130, 38)
    quit_button = pygame.Rect(panel.right - 154, panel.bottom - 58, 130, 38)
    for rect, label in ((rescan, "Rescan"), (quit_button, "Quit")):
        pygame.draw.rect(surface, (52, 58, 72), rect, border_radius=6)
        blit_text(surface, label, fonts["label"], config.TEXT, center=rect.center)
    buttons["rescan"] = rescan
    buttons["quit"] = quit_button
