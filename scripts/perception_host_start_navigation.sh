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

COG_ROOT="${LITE3_COG_ROOT:-/home/ysc/lite_cog_ros2}"
MAP_DIR="${LITE3_MAP_DIR:-$COG_ROOT/system/map}"
NAV_DIR="${LITE3_NAV_SCRIPT_DIR:-$COG_ROOT/system/scripts/nav}"
NAV_SCRIPT="${LITE3_NAV_SCRIPT:-$NAV_DIR/start_nav.sh}"
STATUS_SCRIPT="${LITE3_NAV_STATUS_SCRIPT:-$(cd "$(dirname "$0")" && pwd)/perception_host_nav_status.sh}"

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "check lite3.pcd exists: $MAP_DIR/lite3.pcd"
  echo "check lite3.yaml exists: $MAP_DIR/lite3.yaml"
  echo "check LiDAR topic: /rslidar_points"
  echo "check no duplicate start_nav.sh process"
  echo "cd $NAV_DIR"
  echo "bash $NAV_SCRIPT"
  echo "ros2 action list | grep /FollowWaypoints"
  echo "ros2 topic list | grep -E '/odom|/status|/local_costmap/costmap|/cmd_vel'"
  exit 0
fi

test -f "$MAP_DIR/lite3.pcd"
test -f "$MAP_DIR/lite3.yaml"
ros2 topic list | grep /rslidar_points
if pgrep -af "start_nav.sh|controller_server|planner_server|waypoint_follower" >/dev/null; then
  if "$STATUS_SCRIPT" --execute; then
    echo "navigation already running and ready"
    exit 0
  fi
  echo "partial navigation stack detected; refusing duplicate start" >&2
  exit 1
fi

cd "$NAV_DIR"
bash "$NAV_SCRIPT"
"$STATUS_SCRIPT" --execute
