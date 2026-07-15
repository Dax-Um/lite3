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
STOP_SCRIPT="${LITE3_NAV_STOP_SCRIPT:-$(cd "$(dirname "$0")" && pwd)/perception_host_stop_navigation.sh}"
CONFIG_SCRIPT="${LITE3_NAV_CONFIG_SCRIPT:-$(cd "$(dirname "$0")" && pwd)/perception_host_prepare_nav_config.py}"
LIDAR_PROBE="${LITE3_LIDAR_PROBE:-$(cd "$(dirname "$0")" && pwd)/perception_host_probe_lidar.py}"
LIDAR_EXPECTED_FRAME="${LITE3_LIDAR_EXPECTED_FRAME:-rslidar}"
LIDAR_MAX_AGE_SEC="${LITE3_LIDAR_MAX_AGE_SEC:-5}"
LOG_FILE="${LITE3_NAV_LOG:-/tmp/lite3_navigation.log}"
PID_FILE="${LITE3_NAV_PID_FILE:-/tmp/lite3_navigation.pid}"

lidar_is_fresh() {
  python3 "$LIDAR_PROBE" \
    --timeout-sec 15 \
    --max-age-sec "$LIDAR_MAX_AGE_SEC" \
    --expected-frame "$LIDAR_EXPECTED_FRAME"
}

nav_process_exists() {
  local pattern
  pattern='start_nav.sh|hdl_localization|nav2_map_server/map_server|nav2_lifecycle_manager/lifecycle_manager|nav2_controller/controller_server|nav2_planner/planner_server|nav2_recoveries/recoveries_server|nav2_bt_navigator/bt_navigator|nav2_waypoint_follower/waypoint_follower|global_map_server|motion_sender'
  pgrep -af "$pattern" >/dev/null
}

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "check lite3.pcd exists: $MAP_DIR/lite3.pcd"
  echo "check lite3.yaml exists: $MAP_DIR/lite3.yaml"
  echo "check fresh LiDAR samples: /rslidar_points"
  echo "power-on check: verify actual /rslidar_points header.frame_id and header.stamp"
  echo "check ready stack; stop a partial stack before restart"
  echo "python3 $CONFIG_SCRIPT --apply"
  echo "cd $NAV_DIR"
  echo "nohup bash $NAV_SCRIPT > $LOG_FILE 2>&1 &"
  echo "ros2 action list | grep /navigate_to_pose"
  echo "ros2 topic list | grep -E '/odom|/status|/local_costmap/costmap|/cmd_vel'"
  exit 0
fi

test -f "$MAP_DIR/lite3.pcd"
test -f "$MAP_DIR/lite3.yaml"
python3 "$CONFIG_SCRIPT" --apply
if ! lidar_is_fresh; then
  echo "navigation start refused: /rslidar_points has no fresh samples" >&2
  exit 1
fi
if "$STATUS_SCRIPT" --execute; then
  echo "navigation already running and ready"
  exit 0
fi
if nav_process_exists; then
  echo "partial navigation stack detected; stopping it before restart" >&2
  "$STOP_SCRIPT" --execute
fi

cd "$NAV_DIR"
nohup bash "$NAV_SCRIPT" >"$LOG_FILE" 2>&1 </dev/null &
echo "$!" >"$PID_FILE"

for _ in $(seq 1 20); do
  if nav_process_exists; then
    echo "navigation launch requested; readiness will be polled by IQ9"
    exit 0
  fi
  if ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "navigation launch exited before nodes appeared; log=$LOG_FILE" >&2
    tail -60 "$LOG_FILE" >&2 || true
    exit 1
  fi
  sleep 0.5
done

echo "navigation launch is running; readiness will be polled by IQ9"
