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

PATTERN="start_nav.sh|hdl_localization|nav2_map_server/map_server|nav2_lifecycle_manager/lifecycle_manager|nav2_controller/controller_server|nav2_planner/planner_server|nav2_recoveries/recoveries_server|nav2_bt_navigator/bt_navigator|nav2_waypoint_follower/waypoint_follower|global_map_server|motion_sender"
pkill -f "$PATTERN" 2>/dev/null || true
for _ in $(seq 1 30); do
  if ! pgrep -af "$PATTERN" >/dev/null; then
    echo "navigation stopped"
    exit 0
  fi
  sleep 0.1
done
echo "navigation processes did not stop cleanly" >&2
pgrep -af "$PATTERN" >&2 || true
exit 1
