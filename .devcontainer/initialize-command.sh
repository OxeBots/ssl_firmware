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
