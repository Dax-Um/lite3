#!/usr/bin/env bash
set -e

ENV_FILE="${LITE3_PERCEPTION_ENV:-/home/ysc/.ros_version.sh}"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi
set -u

MODE="dry-run"
for arg in "$@"; do
  case "$arg" in
    --dry-run) MODE="dry-run" ;;
    --execute) MODE="execute" ;;
    -h|--help)
      echo "usage: $0 [--dry-run|--execute]"
      exit 0
      ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "stop navigation processes matching start_nav.sh, controller_server, planner_server, waypoint_follower"
  exit 0
fi

pkill -f "start_nav.sh|controller_server|planner_server|waypoint_follower" 2>/dev/null || true
echo "navigation stop requested"
