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

print_checks() {
  echo "mode=$MODE"
  echo "check file: $MAP_DIR/lite3.pcd"
  echo "check file: $MAP_DIR/lite3.pgm"
  echo "check file: $MAP_DIR/lite3.yaml"
  echo "check process: start_nav.sh ($NAV_SCRIPT)"
  echo "ros2 node list | grep -E 'hdl_localization|controller_server|planner_server|waypoint_follower'"
  echo "ros2 topic list | grep -E '/odom|/status|/map|/local_costmap/costmap|/global_costmap/costmap|/cmd_vel'"
  echo "ros2 action list | grep -E '/FollowWaypoints|/navigate_to_pose'"
  echo "ros2 topic info -v /cmd_vel"
}

if [ "$MODE" = "dry-run" ]; then
  print_checks
  exit 0
fi

missing=0
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
  /waypoint_follower \
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

if ! actions="$(ros2 action list 2>&1)"; then
  echo "failed: ros2 action list: $actions" >&2
  actions=""
  missing=1
fi
for expected in /FollowWaypoints /navigate_to_pose /compute_path_to_pose; do
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
if ! grep -Eq '^Node name: controller_server$' <<<"$cmd_info"; then
  echo "missing /cmd_vel publisher: controller_server" >&2
  missing=1
fi
if ! grep -Eq '^Node name: motion_sender$' <<<"$cmd_info"; then
  echo "missing /cmd_vel subscriber: motion_sender" >&2
  missing=1
fi
exit "$missing"
