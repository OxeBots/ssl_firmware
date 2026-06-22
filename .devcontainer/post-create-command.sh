#!/bin/sh
set -e

cd "$WORKSPACE_FOLDER"

# Ensure the workspace is owned by the current user (avoids permission issues)
sudo chown -R "$(whoami)" "$WORKSPACE_FOLDER"

# Reload udev rules (PlatformIO USB devices)
sudo udevadm control --reload 2>/dev/null || true
