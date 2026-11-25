"""Shared constants: window sizing, timing windows, font files and color palette."""

APP_NAME = "Robot Bench"

# Window size limits: resizable between MIN_WINDOW and the full screen
MIN_WINDOW = (1024, 640)
DEFAULT_WINDOW = (1280, 800)
# Automatic HiDPI and screen-fit defaults
UI_SCALE_MIN = 0.75
UI_SCALE_MAX = 3.0
WINDOW_WORKAREA_RATIO = 0.86
HIDPI_REF_SIZE = (1600, 1000)

# Dashboard tabs, in display order. The radio tab only shows while the radio
# link is connected, so USB and radio sessions can run at the same time.
TAB_MOTORS = 0
TAB_CALIBRATION = 1
TAB_HEALTH = 2
TAB_TOF = 3
TAB_IMU = 4
TAB_CONSOLE = 5
TAB_RADIO = 6
TAB_TITLES = ("Motores", "Calibração", "Saúde", "ToF", "IMU", "Console", "Rádio")

# Speed strip chart time span
CHART_WINDOW_S = 12.0

TOF_SIDE = 8
TOF_MAX_MM = 4000.0
SENSOR_STALE_S = 1.0
HEALTH_STALE_S = 1.5
# Calibration frames arrive at ~5 Hz; a live link keeps them well under this.
CALIBRATION_STALE_S = 1.5
# Control-calibration frames arrive at ~5 Hz, position frames at ~10 Hz.
TUNING_STALE_S = 1.5
POSITION_STALE_S = 1.0
# Raw encoder/motor snapshots arrive at ~10 Hz.
RAW_STALE_S = 0.5
# Default speed cap of a wheel position move (wheel-shaft RPM).
POSITION_MAX_RPM = 300.0
# Acceptable wheel position inputs (degrees; relative moves accumulate).
POSITION_LIMIT_DEG = 36000.0

# TrueType files for the interface and the serial console; the first existing
# path wins, and the Dear PyGui default font is used when none is found.
FONT_FILES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf",
    "C:/Windows/Fonts/segoeui.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
)
MONO_FONT_FILES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
    "C:/Windows/Fonts/consola.ttf",
    "/System/Library/Fonts/Menlo.ttc",
)

# Color palette (RGB; the rendering helpers convert to Dear PyGui RGBA)
BACKGROUND = (15, 21, 30)
PANEL = (23, 32, 44)
PANEL_BORDER = (48, 63, 80)
TEXT = (231, 238, 246)
TEXT_DIM = (157, 173, 191)
ACCENT = (91, 198, 232)
INSET = (17, 25, 36)
TARGET_COLOR = (185, 185, 195)
VELOCITY_COLOR = (90, 200, 255)
PULSE_COLOR = (196, 140, 255)
PWM_COLOR = (255, 176, 64)
OK_COLOR = (110, 220, 130)
BAD_COLOR = (235, 90, 80)
DEMO_COLOR = (255, 200, 80)
TAB_ACTIVE = (34, 62, 79)
