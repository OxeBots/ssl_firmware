#!/bin/sh
set -e

cd "$WORKSPACE_FOLDER"

# Reload udev rules (PlatformIO USB devices) every attach
sudo udevadm control --reload 2>/dev/null || true

# Regenerate compile_commands.json for clangd IntelliSense.
# Runs every attach so clangd is always in sync with the current build.
./scripts/regenerate_compiledb.sh || echo "*** compiledb failed (may need manual fix)"
