#!/bin/sh
set -e

# Ensure host bind-mount dirs exist before the container starts
mkdir -p "$HOME/.claude"
mkdir -p "$HOME/.gemini"
mkdir -p "$HOME/.qwen"
mkdir -p "$HOME/.codex"
mkdir -p "$HOME/.pi"
mkdir -p "$HOME/.config/opencode"
mkdir -p "$HOME/.local/share/opencode"
mkdir -p "$HOME/.local/state/opencode"
mkdir -p "$HOME/.ssh"

# Grant same-UID container processes access to the host X server so GUI apps
# (robot_bench package) can open a window through the X11 socket mount.
if [ -n "$DISPLAY" ]; then
  xhost +si:localuser:"$USER" >/dev/null 2>&1 || true
fi
