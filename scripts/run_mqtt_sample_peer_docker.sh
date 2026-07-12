#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec docker run --rm \
  --network host \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --volume "${ROOT}:/workspace/lite3" \
  --workdir /workspace/lite3 \
  iq9-lite3-mqtt-foxy:latest \
  python3 scripts/run_mqtt_sample_peer.py "$@"
