#!/usr/bin/env bash
# main.sh — control the microscope live monitor (:8766) and the lab dashboard
# (:8770, robot-run ops feed + camera tabs).
#
#   ./main.sh            start monitor + dashboard (no-op if already running)
#   ./main.sh restart    reload both (e.g. after a code/UI change)
#   ./main.sh stop       stop both
#   ./main.sh status     show status of both
#   ./main.sh run [WELL] run the full plate-imaging loop live (default well C9)
#
# On start/restart it also opens the monitor AND the lab dashboard in the Windows
# default browser. Set NO_BROWSER=1 to skip that. Run from an interactive WSL
# login so the WSL->Windows camera interop (and the browser launch) is valid.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYENV_BIN="${PYENV_BIN:-$HOME/.pyenv/bin/pyenv}"
# No install needed: call the module directly. There is no
# microscope-photo command any more (pyproject.toml was retired).
MP="${MICROSCOPE_PHOTO:-python -m tools.microscope.capture}"
URL="http://127.0.0.1:8766/"
LAB_URL="http://127.0.0.1:8770/"   # lab dashboard: robot-run ops feed + camera tabs
LAB_LOG="/tmp/sdl_lab_display.log"
cd "$ROOT_DIR"

is_running() {
  "$PYENV_BIN" exec $MP live-session status 2>/dev/null \
    | grep -Eq '"running"[[:space:]]*:[[:space:]]*true'
}
print_urls() {
  echo
  echo "Live monitor: $URL"
  echo "LAN candidate URLs:"
  "$PYENV_BIN" exec $MP lan-info --lan --port 8766 || true
}
open_url_in_browser() {
  local url="$1"
  [ "${NO_BROWSER:-0}" = "1" ] && return 0
  echo "Opening $url in the default browser..."
  powershell.exe -NoProfile -Command "Start-Process '$url'" >/dev/null 2>&1 \
    || explorer.exe "$url" >/dev/null 2>&1 \
    || cmd.exe /c start "" "$url" >/dev/null 2>&1 \
    || echo "(could not auto-open a browser; open $url manually)"
}
open_browser() {
  sleep 1  # let the web server bind first
  open_url_in_browser "$URL"
}

lab_is_running() {
  "$PYENV_BIN" exec python -c \
    "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8770/api/status', timeout=2)" \
    >/dev/null 2>&1
}
start_lab() {
  if lab_is_running; then
    echo "Lab dashboard already running on :8770."
  else
    echo "Starting lab dashboard on :8770 (log: $LAB_LOG)..."
    nohup setsid "$PYENV_BIN" exec python -m dashboard --port 8770 \
      >"$LAB_LOG" 2>&1 </dev/null &
    local i
    for i in $(seq 1 20); do
      if lab_is_running; then break; fi
      sleep 0.5
    done
  fi
  echo "Lab dashboard: $LAB_URL"
}
stop_lab() {
  pkill -f "python -m dashboard" 2>/dev/null || true
}
open_lab_browser() {
  sleep 1
  open_url_in_browser "$LAB_URL"
}

case "${1:-start}" in
  start)
    if is_running; then
      echo "Live monitor is already running."
    else
      echo "Starting live monitor with LAN binding..."
      "$PYENV_BIN" exec $MP live-session start --lan
    fi
    print_urls
    start_lab
    open_browser
    open_lab_browser
    ;;
  restart)
    echo "Restarting live monitor..."
    "$PYENV_BIN" exec $MP live-session stop || true
    sleep 1
    "$PYENV_BIN" exec $MP live-session start --lan
    print_urls
    stop_lab
    sleep 1
    start_lab
    open_browser
    open_lab_browser
    ;;
  stop)
    "$PYENV_BIN" exec $MP live-session stop || true
    stop_lab
    ;;
  status)
    "$PYENV_BIN" exec $MP live-session status || true
    if lab_is_running; then echo "Lab dashboard: running ($LAB_URL)"; else echo "Lab dashboard: stopped"; fi
    ;;
  run)
    # Run the full plate-imaging loop live, end to end, no Claude needed.
    #   ./main.sh run         # default well C9
    #   ./main.sh run E7      # any well A1..H12
    WELL="${2:-C9}"
    if is_running; then
      echo "Live monitor already running."
    else
      echo "Starting live monitor (needed for live-frame capture)..."
      "$PYENV_BIN" exec $MP live-session start --lan
      sleep 2
    fi
    echo "Running plate-imaging loop (well $WELL) ..."
    # plate_imaging starts/raises the dashboard, opens the browser, and streams
    # its ops to :8770 on its own; the monitor above provides the live frames.
    "$PYENV_BIN" exec python scripts/tool_building/plate_imaging.py --execute run --well "$WELL"
    ;;
  *)
    echo "usage: main.sh [start|restart|stop|status|run [WELL]]" >&2
    exit 2
    ;;
esac
