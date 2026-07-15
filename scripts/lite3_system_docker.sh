#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMAND="${1:-start}"
NAME="lite3-system"
IMAGE="${LITE3_SYSTEM_IMAGE:-iq9-lite3-mqtt-foxy:latest}"
BROKER_HOST="${MQTT_HOST:-127.0.0.1}"
BROKER_PORT="${MQTT_PORT:-1883}"
SPOOL_DIR="${COYOTE_SPOOL_DIR:-/home/ubuntu/iq9_coyote/outputs/spool}"
PATROL_CONFIG="${LITE3_PATROL_CONFIG:-/workspace/lite3/configs/routes/mqtt_triangle_patrol.yaml}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
NAV_NETWORK_INTERFACE="${LITE3_NAV_NETWORK_INTERFACE:-end0}"
COYOTE_PERCEPTION_SCRIPT="${COYOTE_PERCEPTION_SCRIPT:-/home/ubuntu/iq9_coyote/run_perception_node.sh}"
COYOTE_PERCEPTION_LOG="${COYOTE_PERCEPTION_LOG:-/home/ubuntu/iq9_coyote/outputs/perception_launcher.log}"
STARTUP_TIMEOUT_SEC=15
LEGACY_CONTAINERS=(lite3-mqtt-runtime lite3-coyote-mqtt-bridge)

container_exists() {
  docker inspect "$1" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null || true)" = "true" ]
}

current_generation_has_log() {
  local container="$1"
  local marker="$2"
  local started_at
  started_at="$(docker inspect -f '{{.State.StartedAt}}' "${container}")"
  docker logs --since "${started_at}" "${container}" 2>&1 | grep -F "${marker}" >/dev/null
}

stop_and_remove() {
  local container="$1"
  if ! container_exists "${container}"; then
    return
  fi
  if container_running "${container}"; then
    docker update --restart no "${container}" >/dev/null
    docker kill --signal SIGINT "${container}" >/dev/null 2>&1 || true
    shutdown_deadline=$((SECONDS + 20))
    while container_running "${container}" && (( SECONDS < shutdown_deadline )); do
      sleep 1
    done
    if container_running "${container}"; then
      docker stop --timeout 5 "${container}" >/dev/null
    fi
  fi
  docker rm "${container}" >/dev/null
}

verify_runtime() {
  local rmw
  rmw="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" \
    | sed -n 's/^RMW_IMPLEMENTATION=//p' | tail -n 1)"
  if [ "${rmw}" != "rmw_cyclonedds_cpp" ]; then
    echo "[lite3-system] invalid RMW_IMPLEMENTATION: ${rmw:-unset}" >&2
    return 1
  fi
  docker exec "${NAME}" /usr/local/bin/lite3-mqtt-entrypoint \
    ros2 pkg prefix lite3_bringup >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_mqtt_runtime.py' >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_nav_watchdog.py' >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_coyote_mqtt_bridge.py' >/dev/null
}

start_system() {
  if container_exists lite3-test-broker && ! container_running lite3-test-broker; then
    docker start lite3-test-broker >/dev/null
  fi

  if ! pgrep -af '/home/ubuntu/iq9_coyote/perception_node.py' >/dev/null; then
    mkdir -p "$(dirname "${COYOTE_PERCEPTION_LOG}")"
    nohup bash "${COYOTE_PERCEPTION_SCRIPT}" \
      >"${COYOTE_PERCEPTION_LOG}" 2>&1 </dev/null &
  fi

  for legacy in "${LEGACY_CONTAINERS[@]}"; do
    stop_and_remove "${legacy}"
  done

  if container_running "${NAME}"; then
    if ! verify_runtime \
      || ! current_generation_has_log "${NAME}" "runtime ready"; then
      echo "[lite3-system] existing container is not healthy; run restart" >&2
      return 1
    fi
    echo "[lite3-system] already running: ${NAME}"
    return
  fi

  stop_and_remove "${NAME}"

  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[lite3-system] missing image ${IMAGE}; run scripts/build_mqtt_foxy_image.sh" >&2
    return 1
  fi
  mkdir -p "${SPOOL_DIR}"

  docker_args=(
    --detach
    --name "${NAME}"
    --restart no
    --network host
    --env PYTHONDONTWRITEBYTECODE=1
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    --env ROS_LOCALHOST_ONLY=0
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    --env ROS_LOG_DIR=/tmp/ros/launch
    --env "MQTT_HOST=${BROKER_HOST}"
    --env "MQTT_PORT=${BROKER_PORT}"
    --env "MQTT_USER=${MQTT_USER:-}"
    --env "MQTT_PASS=${MQTT_PASS:-}"
    --volume "${ROOT}:/workspace/lite3:ro"
    --volume "${SPOOL_DIR}:${SPOOL_DIR}"
    --workdir /workspace/lite3
  )
  if [ -d "${HOME}/.ssh" ]; then
    docker_args+=(--volume "${HOME}/.ssh:/root/.ssh:ro")
  fi

  docker run "${docker_args[@]}" "${IMAGE}" \
    ros2 launch lite3_bringup lite3_system.launch.py \
    "broker_host:=${BROKER_HOST}" \
    "broker_port:=${BROKER_PORT}" \
    "patrol_config:=${PATROL_CONFIG}" \
    "coyote_spool_dir:=${SPOOL_DIR}" \
    "nav_network_interface:=${NAV_NETWORK_INTERFACE}" >/dev/null

  startup_deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  while container_running "${NAME}" && (( SECONDS < startup_deadline )); do
    if current_generation_has_log "${NAME}" "runtime ready" \
      && current_generation_has_log "${NAME}" "coyote bridge ready"; then
      break
    fi
    sleep 1
  done
  if ! container_running "${NAME}"; then
    docker logs --tail 100 "${NAME}" >&2 || true
    stop_and_remove "${NAME}"
    echo "[lite3-system] container exited during startup" >&2
    return 1
  fi
  if ! verify_runtime; then
    docker logs --tail 100 "${NAME}" >&2 || true
    stop_and_remove "${NAME}"
    echo "[lite3-system] Foxy/Cyclone runtime verification failed" >&2
    return 1
  fi
  if ! current_generation_has_log "${NAME}" "runtime ready" \
    || ! current_generation_has_log "${NAME}" "coyote bridge ready"; then
    docker logs --tail 100 "${NAME}" >&2 || true
    stop_and_remove "${NAME}"
    echo "[lite3-system] required processes did not become ready within ${STARTUP_TIMEOUT_SEC}s" >&2
    return 1
  fi
  docker update --restart unless-stopped "${NAME}" >/dev/null
  echo "[lite3-system] started: ${NAME} broker=${BROKER_HOST}:${BROKER_PORT}"
}

case "${COMMAND}" in
  start)
    start_system
    ;;
  stop)
    stop_and_remove "${NAME}"
    echo "[lite3-system] stopped"
    ;;
  restart)
    stop_and_remove "${NAME}"
    start_system
    ;;
  status)
    if container_running "${NAME}"; then
      verify_runtime
      docker ps --filter "name=^/${NAME}$" --format '{{.Names}} {{.Status}} {{.Image}}'
    else
      echo "[lite3-system] stopped"
      exit 1
    fi
    ;;
  logs)
    exec docker logs --follow --tail 100 "${NAME}"
    ;;
  *)
    echo "usage: $0 {start|stop|restart|status|logs}" >&2
    exit 2
    ;;
esac
