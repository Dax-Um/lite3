#!/usr/bin/env bash
set -e

if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
  echo "[mqtt-docker] RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp" >&2
  exit 2
fi

# The generated setup file also chains the ROS Foxy underlay used at build time.
source /opt/lite3_overlay/setup.bash
mkdir -p /tmp/ros/launch /tmp/ros/mqtt /tmp/ros/coyote
exec "$@"
