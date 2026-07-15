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
    -h|--help)
      echo "usage: $0 [--dry-run|--execute]"
      exit 0
      ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

COG_ROOT="${LITE3_COG_ROOT:-/home/ysc/lite_cog_ros2}"
MAP_DIR="${LITE3_MAP_DIR:-$COG_ROOT/system/map}"
NAV_SCRIPT="${LITE3_NAV_SCRIPT:-$COG_ROOT/system/scripts/nav/start_nav.sh}"
CONFIG_SCRIPT="${LITE3_NAV_CONFIG_SCRIPT:-$(cd "$(dirname "$0")" && pwd)/perception_host_prepare_nav_config.py}"
LIDAR_PROBE="${LITE3_LIDAR_PROBE:-$(cd "$(dirname "$0")" && pwd)/perception_host_probe_lidar.py}"
LIDAR_EXPECTED_FRAME="${LITE3_LIDAR_EXPECTED_FRAME:-rslidar}"
LIDAR_MAX_AGE_SEC="${LITE3_LIDAR_MAX_AGE_SEC:-5}"
LIVE_CONFIG_TIMEOUT_SEC="${LITE3_LIVE_CONFIG_TIMEOUT_SEC:-25}"

lidar_is_fresh() {
  python3 "$LIDAR_PROBE" \
    --timeout-sec 15 \
    --max-age-sec "$LIDAR_MAX_AGE_SEC" \
    --expected-frame "$LIDAR_EXPECTED_FRAME"
}

print_checks() {
  echo "mode=$MODE"
  echo "check file: $MAP_DIR/lite3.pcd"
  echo "check file: $MAP_DIR/lite3.pgm"
  echo "check file: $MAP_DIR/lite3.yaml"
  echo "check process: start_nav.sh ($NAV_SCRIPT)"
  echo "python3 $CONFIG_SCRIPT"
  echo "power-on check: verify actual /rslidar_points header.frame_id and header.stamp"
  echo "ros2 node list | grep -E 'hdl_localization|controller_server|planner_server|bt_navigator'"
  echo "ros2 topic list | grep -E '/odom|/status|/map|/local_costmap/costmap|/global_costmap/costmap|/cmd_vel'"
  echo "ros2 action list | grep -E '/navigate_to_pose|/compute_path_to_pose'"
  echo "ros2 topic info -v /cmd_vel"
}

if [ "$MODE" = "dry-run" ]; then
  print_checks
  exit 0
fi

missing=0
if ! python3 "$CONFIG_SCRIPT"; then
  echo "unsafe persisted Nav2 configuration" >&2
  missing=1
fi
for file in "$MAP_DIR/lite3.pcd" "$MAP_DIR/lite3.pgm" "$MAP_DIR/lite3.yaml"; do
  if [ ! -f "$file" ]; then
    echo "missing: $file" >&2
    missing=1
  fi
done

pgrep -af "start_nav.sh|nav2|controller_server|planner_server" || true

if ! nodes="$(ros2 node list 2>&1)"; then
  echo "failed: ros2 node list: $nodes" >&2
  nodes=""
  missing=1
fi
for expected in \
  /hdl_localization \
  /planner_server \
  /controller_server \
  /global_costmap/global_costmap \
  /local_costmap/local_costmap \
  /bt_navigator \
  /motion_sender; do
  if ! grep -Fxq "$expected" <<<"$nodes"; then
    echo "missing node: $expected" >&2
    missing=1
  fi
done

if ! topics="$(ros2 topic list 2>&1)"; then
  echo "failed: ros2 topic list: $topics" >&2
  topics=""
  missing=1
fi
for expected in \
  /rslidar_points \
  /odom \
  /status \
  /map \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /cmd_vel; do
  if ! grep -Fxq "$expected" <<<"$topics"; then
    echo "missing topic: $expected" >&2
    missing=1
  fi
done

if ! lidar_is_fresh; then
  echo "stale topic: /rslidar_points has no fresh samples" >&2
  missing=1
fi

if ! actions="$(ros2 action list 2>&1)"; then
  echo "failed: ros2 action list: $actions" >&2
  actions=""
  missing=1
fi
for expected in /navigate_to_pose /compute_path_to_pose; do
  if ! grep -Fxq "$expected" <<<"$actions"; then
    echo "missing action: $expected" >&2
    missing=1
  fi
done

if ! cmd_info="$(ros2 topic info -v /cmd_vel 2>&1)"; then
  echo "failed: /cmd_vel endpoint inspection: $cmd_info" >&2
  cmd_info=""
  missing=1
fi
controller_publishers="$(grep -Ec '^Node name: controller_server$' <<<"$cmd_info" || true)"
if [ "$controller_publishers" -ne 1 ]; then
  echo "missing /cmd_vel publisher: controller_server" >&2
  missing=1
fi
motion_subscribers="$(grep -Ec '^Node name: motion_sender$' <<<"$cmd_info" || true)"
if [ "$motion_subscribers" -ne 1 ]; then
  echo "missing /cmd_vel subscriber: motion_sender" >&2
  missing=1
fi

if ! timeout "$LIVE_CONFIG_TIMEOUT_SEC" python3 "$CONFIG_SCRIPT" --live; then
  echo "unsafe live Nav2 configuration" >&2
  missing=1
fi
exit "$missing"
