"""Shared constants: window sizing, timing windows, fonts and color palette."""

# Window size limits: resizable between MIN_WINDOW and the full screen
MIN_WINDOW = (1024, 640)
DEFAULT_WINDOW = (1280, 800)

# Dashboard tabs, in display order
TAB_MOTORS = 0
TAB_HEALTH = 1
TAB_SENSING = 2
TAB_TITLES = ("Motors & Control", "Health & State", "Sensing")

# Speed strip chart time span
CHART_WINDOW_S = 12.0
# Wheel target square sweep amplitude (dashboard-driven)
SWEEP_RPM = 1000.0

TOF_SIDE = 8
TOF_MAX_MM = 4000.0

FONT_NAMES = "consolas,dejavusansmono,menlo,monospace"

# Color palette
BACKGROUND = (16, 18, 24)
PANEL = (28, 31, 40)
PANEL_BORDER = (62, 68, 84)
TEXT = (222, 226, 235)
TEXT_DIM = (140, 146, 160)
ACCENT = (90, 200, 255)
TARGET_COLOR = (185, 185, 195)
VELOCITY_COLOR = (90, 200, 255)
PWM_COLOR = (255, 176, 64)
OK_COLOR = (110, 220, 130)
BAD_COLOR = (235, 90, 80)
DEMO_COLOR = (255, 200, 80)
TAB_ACTIVE = (52, 58, 72)
