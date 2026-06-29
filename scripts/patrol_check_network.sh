#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-${LITE3_MOTION_HOST:-}}"
if [[ -z "${HOST}" ]]; then
  echo "[patrol] missing motion host. Pass HOST as argv[1] or set LITE3_MOTION_HOST." >&2
  exit 2
fi
echo "[patrol] ping motion host: ${HOST}"
ping -c 3 "$HOST"
echo "[patrol] local addresses"
ip -br addr
