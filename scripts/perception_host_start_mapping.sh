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
SLAM_DIR="${LITE3_SLAM_SCRIPT_DIR:-$COG_ROOT/system/scripts/slam}"
SLAM_SCRIPT="${LITE3_SLAM_SCRIPT:-$SLAM_DIR/start_slam.sh}"

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "check no start_nav.sh process is running"
  echo "check LiDAR topic: /rslidar_points"
  echo "cd $SLAM_DIR"
  echo "bash $SLAM_SCRIPT"
  echo "operator joystick mapping required"
  exit 0
fi

if pgrep -af "start_nav.sh|controller_server|planner_server|waypoint_follower" >/dev/null; then
  echo "navigation is running; stop navigation before mapping" >&2
  exit 3
fi

ros2 topic list | grep /rslidar_points
cd "$SLAM_DIR"
bash "$SLAM_SCRIPT"
echo "operator joystick mapping required"
