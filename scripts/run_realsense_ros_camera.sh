#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${REALSENSE_OUTPUT_DIR:-/home/ubuntu/iq9_coyote/outputs/realsense}"
STALE_AFTER_SEC="${REALSENSE_STALE_AFTER_SEC:-5}"
STARTUP_GRACE_SEC="${REALSENSE_STARTUP_GRACE_SEC:-12}"
mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}/latest.jpg" "${OUTPUT_DIR}/latest_depth.npy" "${OUTPUT_DIR}/latest_meta.json"
CAMERA_PID=""
BRIDGE_PID=""
cleanup() { kill "${CAMERA_PID:-}" "${BRIDGE_PID:-}" 2>/dev/null || true; }
trap cleanup EXIT
python3 /workspace/lite3/scripts/run_realsense_ros_image_bridge.py --output-dir "${OUTPUT_DIR}" &
BRIDGE_PID=$!

realsense_usb_present() {
  for device in /sys/bus/usb/devices/*; do
    [[ -r "${device}/idVendor" && -r "${device}/idProduct" ]] || continue
    [[ "$(<"${device}/idVendor")" == "8086" && "$(<"${device}/idProduct")" == "0b3a" ]] && return 0
  done
  return 1
}

output_is_stale() {
  [[ -f "${OUTPUT_DIR}/latest_meta.json" ]] || return 0
  local now modified
  now="$(date +%s)"
  modified="$(stat -c %Y "${OUTPUT_DIR}/latest_meta.json")"
  (( now - modified > STALE_AFTER_SEC ))
}

while true; do
  ros2 launch realsense2_camera rs_launch.py camera_name:=realsense enable_color:=true enable_depth:=true enable_infra:=false align_depth.enable:=true &
  CAMERA_PID=$!
  CAMERA_STARTED_AT="$(date +%s)"
  while kill -0 "${CAMERA_PID}" 2>/dev/null; do
    sleep 2
    # A hot-plugged D435i can be visible to USB while the old librealsense
    # context remains unable to acquire its UVC power state. Restart only this
    # camera launch in that exact condition; no Docker/system restart needed.
    if realsense_usb_present && output_is_stale \
      && (( $(date +%s) - CAMERA_STARTED_AT >= STARTUP_GRACE_SEC )); then
      echo "[lite3-realsense] USB device present but RGB/depth output is stale; restarting camera node" >&2
      kill "${CAMERA_PID}" 2>/dev/null || true
      break
    fi
  done
  wait "${CAMERA_PID}" 2>/dev/null || true
  CAMERA_PID=""
  sleep 1
done
