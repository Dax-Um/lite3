#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-${LITE3_MOTION_HOST:-192.168.1.120}}"
EXPECTED_IQ9_ROBOT_SIDE_IP="${LITE3_IQ9_ROBOT_SIDE_IP:-192.168.1.215}"
if [[ -z "${HOST}" ]]; then
  echo "[patrol] missing motion host. Pass HOST as argv[1] or set LITE3_MOTION_HOST." >&2
  exit 2
fi
echo "[patrol] ping motion host: ${HOST}"
ping -c 3 "$HOST"
echo "[patrol] local addresses"
ip -br addr
if ! ip -br addr | grep -q "${EXPECTED_IQ9_ROBOT_SIDE_IP}"; then
  echo "[patrol] missing IQ9 robot-side static IP: ${EXPECTED_IQ9_ROBOT_SIDE_IP}" >&2
  exit 3
fi
echo "[patrol] IQ9 robot-side static IP OK: ${EXPECTED_IQ9_ROBOT_SIDE_IP}"
