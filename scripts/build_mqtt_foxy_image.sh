#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec docker build \
  --file "${ROOT}/deploy/mqtt/Dockerfile.foxy" \
  --tag iq9-lite3-mqtt-foxy:latest \
  "${ROOT}/deploy/mqtt"
