#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DOCKER_ARGS=(
  --rm
  --name lite3-mqtt-runtime
  --network host
  --env PYTHONDONTWRITEBYTECODE=1
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  --env ROS_LOCALHOST_ONLY=0
  --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  --volume "${ROOT}:/workspace/lite3"
  --workdir /workspace/lite3
)

if [ -d "${HOME}/.ssh" ]; then
  DOCKER_ARGS+=(--volume "${HOME}/.ssh:/root/.ssh:ro")
fi

exec docker run "${DOCKER_ARGS[@]}" \
  iq9-lite3-mqtt-foxy:latest \
  python3 scripts/run_mqtt_runtime.py "$@"
