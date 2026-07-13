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
LIDAR_DIR="${LITE3_LIDAR_SCRIPT_DIR:-$COG_ROOT/system/scripts/lidar}"
LIDAR_SCRIPT="${LITE3_LIDAR_SCRIPT:-$LIDAR_DIR/start_rslidar.sh}"

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "check process: start_rslidar.sh or rslidar"
  echo "cd $LIDAR_DIR"
  echo "bash $LIDAR_SCRIPT"
  echo "ros2 topic list | grep /rslidar_points"
  exit 0
fi

if pgrep -af "start_rslidar.sh|rslidar" >/dev/null; then
  if ros2 topic list | grep -Fxq /rslidar_points; then
    echo "rslidar already running and publishing /rslidar_points"
    exit 0
  fi
  echo "partial LiDAR process detected without /rslidar_points" >&2
  exit 1
fi

cd "$LIDAR_DIR"
bash "$LIDAR_SCRIPT"
ros2 topic list | grep -Fxq /rslidar_points
