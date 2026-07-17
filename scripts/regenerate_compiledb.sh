#!/bin/sh
# Generate compile_commands.json for clangd/clang-tidy.
#
# `pio run -t compiledb` covers test/ via a post extra-script, so a bare run
# suffices. This script wraps it and idempotently re-appends test entries.
set -e

cd "$(dirname "$0")/.."

# Ensure pio and xtensa toolchain are on PATH so clangd resolves the real cross-compiler.
export PATH="$HOME/.platformio/penv/bin:$HOME/.platformio/packages/toolchain-xtensa-esp-elf/bin:$PATH"

echo "==> Generating compile_commands.json via PlatformIO..."
pio run -t compiledb

echo "==> Appending test file entries..."
python3 scripts/generate_test_compile_commands.py

echo "==> Done. compile_commands.json is up to date."
