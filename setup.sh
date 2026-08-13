#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYENV_BIN="${PYENV_BIN:-$HOME/.pyenv/bin/pyenv}"
MICROSCOPE_PHOTO="${MICROSCOPE_PHOTO:-python -m tools.microscope.capture}"
START_STREAM="${SDL_SETUP_START_STREAM:-1}"

cd "$ROOT_DIR"

echo "Installing dependencies into pyenv main..."
# The repo itself is not installed -- the phase scripts in scripts/ import
# tools/ directly. Only the third-party packages are needed.
"$PYENV_BIN" exec python -m pip install -r requirements.txt

case "${START_STREAM,,}" in
  0|false|no|off)
    echo "Skipping microscope live-session startup because SDL_SETUP_START_STREAM=$START_STREAM"
    exit 0
    ;;
esac

echo "Checking microscope live-session status..."
status="$("$PYENV_BIN" exec $MICROSCOPE_PHOTO live-session status 2>/dev/null || true)"
if printf '%s\n' "$status" | grep -Eq '"running"[[:space:]]*:[[:space:]]*true'; then
  echo "Microscope live-session is already running."
else
  echo "Starting microscope live-session monitor with LAN binding..."
  "$PYENV_BIN" exec $MICROSCOPE_PHOTO live-session start --lan
fi

echo "Microscope monitor: http://127.0.0.1:8766/"
echo "LAN candidate URLs:"
"$PYENV_BIN" exec $MICROSCOPE_PHOTO lan-info --lan --port 8766 || true
echo "User-saved folder: /home/lamp/camera_captures/user-saved"
