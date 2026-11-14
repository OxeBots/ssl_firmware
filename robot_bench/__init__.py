"""Robot Bench: live diagnostics for the SSL robot firmware (see README.md)."""

import sys

try:
    import dearpygui.dearpygui as dpg  # noqa: F401  # imported early to fail with a friendly message
except ImportError:
    sys.exit("dearpygui é obrigatório: pip install dearpygui pyserial")
