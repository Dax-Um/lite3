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
LOG_FILE="${LITE3_LIDAR_LOG:-/tmp/lite3_rslidar.log}"
PID_FILE="${LITE3_LIDAR_PID_FILE:-/tmp/lite3_rslidar.pid}"
READY_TIMEOUT_SEC="${LITE3_LIDAR_READY_TIMEOUT_SEC:-20}"
LIDAR_PROBE="${LITE3_LIDAR_PROBE:-$(cd "$(dirname "$0")" && pwd)/perception_host_probe_lidar.py}"
LIDAR_EXPECTED_FRAME="${LITE3_LIDAR_EXPECTED_FRAME:-rslidar}"
LIDAR_MAX_AGE_SEC="${LITE3_LIDAR_MAX_AGE_SEC:-5}"

lidar_is_fresh() {
  local timeout_sec="${1:-15}"
  python3 "$LIDAR_PROBE" \
    --timeout-sec "$timeout_sec" \
    --max-age-sec "$LIDAR_MAX_AGE_SEC" \
    --expected-frame "$LIDAR_EXPECTED_FRAME"
}

lidar_process_exists() {
  if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    return 0
  fi
  pgrep -af '/system/scripts/lidar/start_rslidar.sh([[:space:]]|$)|/rslidar_sdk_node([[:space:]]|$)|/rs_driver_node([[:space:]]|$)' >/dev/null
}

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "python3 $LIDAR_PROBE --timeout-sec 15 --max-age-sec $LIDAR_MAX_AGE_SEC --expected-frame $LIDAR_EXPECTED_FRAME"
  echo "power-on check: verify actual /rslidar_points header.frame_id and header.stamp"
  echo "check exact driver executable: rslidar_sdk_node or rs_driver_node"
  echo "cd $LIDAR_DIR"
  echo "nohup bash $LIDAR_SCRIPT > $LOG_FILE 2>&1 &"
  echo "wait up to ${READY_TIMEOUT_SEC}s for fresh /rslidar_points samples"
  exit 0
fi

if lidar_process_exists; then
  if lidar_is_fresh 15; then
    echo "rslidar is publishing fresh /rslidar_points samples"
    exit 0
  fi
  echo "LiDAR driver process exists but /rslidar_points is stale" >&2
  exit 1
fi

# A publisher may be running outside the vendor process names (for example in
# a container).  Keep this discovery probe short so a cold start retains most
# of READY_TIMEOUT_SEC for the newly launched driver.
if lidar_is_fresh 2; then
  echo "rslidar is publishing fresh /rslidar_points samples"
  exit 0
fi

cd "$LIDAR_DIR"
nohup bash "$LIDAR_SCRIPT" >"$LOG_FILE" 2>&1 </dev/null &
echo "$!" >"$PID_FILE"

if lidar_is_fresh "$READY_TIMEOUT_SEC"; then
  echo "rslidar started and fresh /rslidar_points samples were received"
  exit 0
fi

echo "timed out waiting for fresh /rslidar_points; log=$LOG_FILE" >&2
tail -40 "$LOG_FILE" >&2 || true
exit 1
