"""Live GUI dashboard for the SSL firmware protoboard bench (see README.md)."""

import sys

try:
    import pygame  # noqa: F401  # imported early to fail with a friendly message
except ImportError:
    sys.exit("pygame is required: pip install pygame pyserial")
