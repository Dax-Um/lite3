#!/usr/bin/env bash
set -e

ENV_FILE="${LITE3_PERCEPTION_ENV:-/home/ysc/.ros_version.sh}"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi
set -u

MODE="execute"
for arg in "$@"; do
  case "$arg" in
    --dry-run) MODE="dry-run" ;;
    --execute) MODE="execute" ;;
    -h|--help) echo "usage: $0 [--dry-run|--execute]"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

ROOT="${LITE3_PERCEPTION_APP_ROOT:-/home/ysc/lite3}"
WATCHDOG="${LITE3_NAV_WATCHDOG_SCRIPT:-$ROOT/scripts/run_nav_watchdog.py}"
LOG_FILE="${LITE3_NAV_WATCHDOG_LOG:-/tmp/lite3_nav_watchdog.log}"

if [ "$MODE" = "dry-run" ]; then
  echo "stop existing process: run_nav_watchdog.py"
  echo "start: python3 $WATCHDOG"
  exit 0
fi

test -f "$WATCHDOG"
if pgrep -af "[r]un_nav_watchdog.py" >/dev/null; then
  echo "stopping existing navigation watchdog"
  pkill -TERM -f "[r]un_nav_watchdog.py" || true
  for _ in $(seq 1 30); do
    if ! pgrep -af "[r]un_nav_watchdog.py" >/dev/null; then
      break
    fi
    sleep 0.1
  done
  if pgrep -af "[r]un_nav_watchdog.py" >/dev/null; then
    echo "existing navigation watchdog did not stop" >&2
    exit 1
  fi
fi

nohup python3 "$WATCHDOG" >"$LOG_FILE" 2>&1 &
for _ in $(seq 1 20); do
  if pgrep -af "[r]un_nav_watchdog.py" >/dev/null; then
    echo "navigation watchdog started"
    exit 0
  fi
  sleep 0.1
done
echo "navigation watchdog failed to start; log=$LOG_FILE" >&2
exit 1
