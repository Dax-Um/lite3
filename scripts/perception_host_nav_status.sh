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
ros2 node list | grep -E 'hdl_localization|controller_server|planner_server|waypoint_follower'
ros2 topic list | grep -E '/odom|/status|/map|/local_costmap/costmap|/global_costmap/costmap|/cmd_vel'
ros2 action list | grep -E '/FollowWaypoints|/navigate_to_pose'
ros2 topic info -v /cmd_vel
exit "$missing"
