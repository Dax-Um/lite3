#!/usr/bin/env bash
set -e

# The generated setup file also chains the ROS Foxy underlay used at build time.
source /opt/lite3_overlay/setup.bash
exec "$@"
