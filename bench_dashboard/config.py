"""Shared constants: protocol framing, timing windows, fonts and color palette."""

# S,<ms>,<yaw>,<pitch>,<roll>,<pwm>,<vel>,<tgt>,<enc>,<enc_ok>,<tof_seq>
STATE_FIELD_COUNT = 11
# T,<seq>,<d0>..<d63>
TOF_FIELD_COUNT = 66
TOF_SIDE = 8
TOF_MAX_MM = 4000.0

# Speed strip chart time span
CHART_WINDOW_S = 12.0

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
