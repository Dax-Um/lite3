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
SLAM_DIR="${LITE3_SLAM_SCRIPT_DIR:-$COG_ROOT/system/scripts/slam}"
PCD2GRID_CMD="${LITE3_PCD2GRID_CMD:-pcd2grid}"
SAVE_MAP_SCRIPT="${LITE3_SAVE_MAP_SCRIPT:-$SLAM_DIR/save_map.sh}"

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "check file: $MAP_DIR/lite3.pcd"
  echo "stop existing pcd2grid process if needed"
  echo "run $PCD2GRID_CMD"
  echo "bash $SAVE_MAP_SCRIPT # save_map"
  echo "check file: $MAP_DIR/lite3.pgm"
  echo "check file: $MAP_DIR/lite3.yaml"
  exit 0
fi

if [ ! -f "$MAP_DIR/lite3.pcd" ]; then
  echo "missing: $MAP_DIR/lite3.pcd" >&2
  exit 4
fi

pkill -f pcd2grid 2>/dev/null || true
$PCD2GRID_CMD
bash "$SAVE_MAP_SCRIPT"
test -f "$MAP_DIR/lite3.pgm"
test -f "$MAP_DIR/lite3.yaml"
