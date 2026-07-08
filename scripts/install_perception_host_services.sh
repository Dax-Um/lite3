#!/usr/bin/env bash
set -eu

MODE="dry-run"
HOST=""
REMOTE_ROOT="/home/ysc/lite3"
SYSTEMD_DIR="/etc/systemd/system"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run) MODE="dry-run"; shift ;;
    --execute) MODE="execute"; shift ;;
    --host)
      HOST="${2:-}"
      shift 2
      ;;
    --remote-root)
      REMOTE_ROOT="${2:-}"
      shift 2
      ;;
    -h|--help)
      echo "usage: $0 --host user@host [--dry-run|--execute] [--remote-root /home/ysc/lite3]"
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$HOST" ]; then
  echo "--host is required" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_SRC="$REPO_ROOT/deploy/systemd/perception-host"

WRAPPERS="
perception_host_nav_status.sh
perception_host_start_lidar.sh
perception_host_start_mapping.sh
perception_host_finish_mapping.sh
perception_host_start_navigation.sh
perception_host_stop_navigation.sh
"

SERVICES="
lite3-lidar.service
lite3-navigation.service
lite3-mapping.service
"

if [ "$MODE" = "dry-run" ]; then
  echo "mode=dry-run"
  echo "target=$HOST"
  echo "remote root=$REMOTE_ROOT"
  echo "create directory: $REMOTE_ROOT/scripts"
  for wrapper in $WRAPPERS; do
    echo "copy: scripts/$wrapper -> $REMOTE_ROOT/scripts/$wrapper"
  done
  for service in $SERVICES; do
    echo "copy: deploy/systemd/perception-host/$service -> $SYSTEMD_DIR/$service"
  done
  echo "remote command: chmod +x $REMOTE_ROOT/scripts/perception_host_*.sh"
  echo "remote command: sudo systemctl daemon-reload"
  echo "not enabling or starting services automatically"
  exit 0
fi

ssh "$HOST" "mkdir -p '$REMOTE_ROOT/scripts'"
for wrapper in $WRAPPERS; do
  scp "$SCRIPT_DIR/$wrapper" "$HOST:$REMOTE_ROOT/scripts/$wrapper"
done
ssh "$HOST" "chmod +x '$REMOTE_ROOT'/scripts/perception_host_*.sh"

for service in $SERVICES; do
  scp "$SERVICE_SRC/$service" "$HOST:/tmp/$service"
  ssh "$HOST" "sudo mv '/tmp/$service' '$SYSTEMD_DIR/$service'"
done

ssh "$HOST" "sudo systemctl daemon-reload"
echo "installed perception host wrappers and service templates"
echo "services were not enabled or started"
