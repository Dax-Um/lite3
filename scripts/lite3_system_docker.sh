#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMAND="start"
BROKER_MODE="${LITE3_BROKER_MODE:-hl}"
NAME="lite3-system"
IMAGE="${LITE3_SYSTEM_IMAGE:-iq9-lite3-mqtt-foxy:latest}"
HL_BROKER_HOST="${MQTT_HOST:-192.168.26.19}"
HL_BROKER_PORT="${MQTT_PORT:-1883}"
TEST_BROKER_HOST="127.0.0.1"
TEST_BROKER_PORT="1883"
SPOOL_DIR="${COYOTE_SPOOL_DIR:-/home/ubuntu/iq9_coyote/outputs/spool}"
REALSENSE_OUTPUT_DIR="${REALSENSE_OUTPUT_DIR:-/home/ubuntu/iq9_coyote/outputs/realsense}"
REALSENSE_REQUEST_DIR="${REALSENSE_REQUEST_DIR:-/home/ubuntu/iq9_coyote/outputs/realsense_requests}"
AUDIO_REQUEST_DIR="${AUDIO_REQUEST_DIR:-/home/ubuntu/iq9_coyote/audio_requests}"
PATROL_CONFIG="${LITE3_PATROL_CONFIG:-/workspace/lite3/configs/routes/mqtt_triangle_patrol.yaml}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
NAV_NETWORK_INTERFACE="${LITE3_NAV_NETWORK_INTERFACE:-end0}"
CYCLONEDDS_CONFIG="${LITE3_CYCLONEDDS_CONFIG:-/workspace/lite3/configs/lite3/cyclonedds_iq9_perception.xml}"
COYOTE_PERCEPTION_SCRIPT="${COYOTE_PERCEPTION_SCRIPT:-/home/ubuntu/iq9_coyote/run_perception_node.sh}"
COYOTE_PERCEPTION_LOG="${COYOTE_PERCEPTION_LOG:-/home/ubuntu/iq9_coyote/outputs/perception_launcher.log}"
AUDIO_SERVICE_SCRIPT="${AUDIO_SERVICE_SCRIPT:-/home/ubuntu/iq9_coyote/run_audio_cue_service.py}"
AUDIO_SERVICE_LOG="${AUDIO_SERVICE_LOG:-/home/ubuntu/iq9_coyote/outputs/audio_service.log}"
AUDIO_SERVICE_PYTHON="${AUDIO_SERVICE_PYTHON:-/home/ubuntu/iq9_coyote/tts_venv/bin/python}"
VOICE_ASR_ENABLED="${LITE3_VOICE_ASR_ENABLED:-1}"
VOICE_ASR_LOG="${LITE3_VOICE_ASR_LOG:-/home/ubuntu/iq9_coyote/outputs/lite3_voice_asr.log}"
VOICE_ASR_PID_FILE="${LITE3_VOICE_ASR_PID_FILE:-/tmp/lite3_voice_asr.pid}"
VOICE_ASR_ASSET_ROOT="${LITE3_VOICE_ASR_ASSET_ROOT:-${ROOT}/assets/asr}"
VOICE_ASR_CAPTURE_BACKEND="${LITE3_VOICE_ASR_CAPTURE_BACKEND:-pipewire}"
VOICE_ASR_DEVICE="${LITE3_VOICE_ASR_DEVICE:-alsa_input.usb-Shenzhen_jiayz_photo_industrial_ltd_BOYALINK_112004030501001585002101FFFFFFfg-01.analog-stereo}"
VOICE_ASR_LANGUAGE="${LITE3_VOICE_ASR_LANGUAGE:-en}"
VOICE_ASR_ONSET_RMS="${LITE3_VOICE_ASR_ONSET_RMS:-50}"
VOICE_ASR_RELEASE_RMS="${LITE3_VOICE_ASR_RELEASE_RMS:-40}"
VOICE_ASR_SILENCE_CHUNKS="${LITE3_VOICE_ASR_SILENCE_CHUNKS:-5}"
VOICE_RUNTIME_DIR="${LITE3_VOICE_RUNTIME_DIR:-/home/ubuntu/iq9_coyote/outputs/voice_control}"
VOICE_ASR_EVENTS="${LITE3_VOICE_ASR_EVENTS:-${VOICE_RUNTIME_DIR}/asr_events.jsonl}"
VOICE_ACTION_EVENTS="${LITE3_VOICE_ACTION_EVENTS:-${VOICE_RUNTIME_DIR}/action_events.jsonl}"
VOICE_RAG_ENABLED="${LITE3_VOICE_RAG_ENABLED:-1}"
VOICE_RAG_LOG="${LITE3_VOICE_RAG_LOG:-/home/ubuntu/iq9_coyote/outputs/lite3_voice_rag.log}"
VOICE_RAG_PID_FILE="${LITE3_VOICE_RAG_PID_FILE:-/tmp/lite3_voice_rag.pid}"
VOICE_RAG_PYTHON="${LITE3_VOICE_RAG_PYTHON:-${ROOT}/.venv-voice/bin/python}"
VOICE_EXECUTE_ENABLED="${LITE3_VOICE_EXECUTE:-1}"
VOICE_EXECUTOR_LOG="${LITE3_VOICE_EXECUTOR_LOG:-/home/ubuntu/iq9_coyote/outputs/lite3_voice_executor.log}"
VOICE_EXECUTOR_PID_FILE="${LITE3_VOICE_EXECUTOR_PID_FILE:-/tmp/lite3_voice_executor.pid}"
VOICE_TTS_ENABLED="${LITE3_VOICE_TTS_ENABLED:-1}"
VOICE_TTS_LOG="${LITE3_VOICE_TTS_LOG:-/home/ubuntu/iq9_coyote/outputs/lite3_voice_tts.log}"
VOICE_TTS_PID_FILE="${LITE3_VOICE_TTS_PID_FILE:-/tmp/lite3_voice_tts.pid}"
VOICE_MOTION_HOST="${LITE3_VOICE_MOTION_HOST:-192.168.1.120}"
VOICE_MOTION_PORT="${LITE3_VOICE_MOTION_PORT:-43893}"
VOICE_STATE_FILE="${LITE3_VOICE_STATE_FILE:-${VOICE_RUNTIME_DIR}/motion_state.json}"
STARTUP_TIMEOUT_SEC=15
LEGACY_CONTAINERS=(lite3-mqtt-runtime lite3-coyote-mqtt-bridge)

usage() {
  echo "usage: $0 [start|stop|restart|status|logs] [--test|--hl]" >&2
  echo "  --test  use/start IQ9 local lite3-test-broker (127.0.0.1:1883)" >&2
  echo "  --hl    use Home Hub broker (${HL_BROKER_HOST}:${HL_BROKER_PORT})" >&2
}

for argument in "$@"; do
  case "${argument}" in
    start|stop|restart|status|logs)
      COMMAND="${argument}"
      ;;
    --test)
      BROKER_MODE="test"
      ;;
    --hl)
      BROKER_MODE="hl"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

case "${BROKER_MODE}" in
  test)
    BROKER_HOST="${TEST_BROKER_HOST}"
    BROKER_PORT="${TEST_BROKER_PORT}"
    ;;
  hl)
    BROKER_HOST="${HL_BROKER_HOST}"
    BROKER_PORT="${HL_BROKER_PORT}"
    ;;
  *)
    echo "[lite3-system] unsupported broker mode: ${BROKER_MODE}" >&2
    exit 2
    ;;
esac

container_exists() {
  docker inspect "$1" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null || true)" = "true" ]
}

ensure_test_broker() {
  if container_exists lite3-test-broker; then
    if ! container_running lite3-test-broker; then
      docker start lite3-test-broker >/dev/null
    fi
    return
  fi
  docker run --detach --name lite3-test-broker --restart unless-stopped \
    --network host eclipse-mosquitto:2 mosquitto -p 1883 >/dev/null
}

stop_test_broker() {
  if container_running lite3-test-broker; then
    docker stop --timeout 5 lite3-test-broker >/dev/null
  fi
}

voice_asr_running() {
  [ -f "${VOICE_ASR_PID_FILE}" ] && kill -0 "$(cat "${VOICE_ASR_PID_FILE}")" 2>/dev/null
}

start_voice_asr() {
  if [ "${VOICE_ASR_ENABLED}" != "1" ]; then
    return
  fi
  if voice_asr_running; then
    return
  fi
  mkdir -p "$(dirname "${VOICE_ASR_LOG}")" "${VOICE_RUNTIME_DIR}"
  nohup python3 "${ROOT}/scripts/run_asr_microphone_test.py" \
    --asset-root "${VOICE_ASR_ASSET_ROOT}" \
    --capture-backend "${VOICE_ASR_CAPTURE_BACKEND}" \
    --target "${VOICE_ASR_DEVICE}" \
    --language "${VOICE_ASR_LANGUAGE}" \
    --vad-onset-rms "${VOICE_ASR_ONSET_RMS}" \
    --vad-release-rms "${VOICE_ASR_RELEASE_RMS}" \
    --silence-chunks "${VOICE_ASR_SILENCE_CHUNKS}" \
    --result-jsonl "${VOICE_ASR_EVENTS}" \
    >"${VOICE_ASR_LOG}" 2>&1 </dev/null &
  echo $! >"${VOICE_ASR_PID_FILE}"
}

stop_voice_asr() {
  if ! voice_asr_running; then
    rm -f "${VOICE_ASR_PID_FILE}"
    return
  fi
  kill -TERM "$(cat "${VOICE_ASR_PID_FILE}")" 2>/dev/null || true
  rm -f "${VOICE_ASR_PID_FILE}"
}

voice_rag_running() {
  [ -f "${VOICE_RAG_PID_FILE}" ] && kill -0 "$(cat "${VOICE_RAG_PID_FILE}")" 2>/dev/null
}

start_voice_rag() {
  if [ "${VOICE_RAG_ENABLED}" != "1" ] || voice_rag_running; then
    return
  fi
  if [ ! -x "${VOICE_RAG_PYTHON}" ]; then
    echo "[lite3-system] voice RAG venv missing: ${VOICE_RAG_PYTHON}" >&2
    return 1
  fi
  mkdir -p "${VOICE_RUNTIME_DIR}" "$(dirname "${VOICE_RAG_LOG}")"
  touch "${VOICE_ASR_EVENTS}"
  nohup env PYTHONPATH="${ROOT}/src" "${VOICE_RAG_PYTHON}" -m lite3_voice.voice_rag follow \
    --events "${VOICE_ASR_EVENTS}" >"${VOICE_RAG_LOG}" 2>&1 </dev/null &
  echo $! >"${VOICE_RAG_PID_FILE}"
}

voice_executor_running() {
  [ -f "${VOICE_EXECUTOR_PID_FILE}" ] && kill -0 "$(cat "${VOICE_EXECUTOR_PID_FILE}")" 2>/dev/null
}

start_voice_executor() {
  if [ "${VOICE_EXECUTE_ENABLED}" != "1" ] || voice_executor_running; then return; fi
  touch "${VOICE_ACTION_EVENTS}"
  nohup env PYTHONPATH="${ROOT}/src" VOICE_STATE_FILE="${VOICE_STATE_FILE}" VOICE_MOTION_HOST="${VOICE_MOTION_HOST}" VOICE_MOTION_PORT="${VOICE_MOTION_PORT}" VOICE_ACTION_EVENTS="${VOICE_ACTION_EVENTS}" python3 -c '
from lite3_voice.executor import VoiceActionExecutor, run
import os
executor = VoiceActionExecutor(os.environ["VOICE_STATE_FILE"], os.environ["VOICE_MOTION_HOST"], int(os.environ["VOICE_MOTION_PORT"]))
try: raise SystemExit(run(executor, os.environ["VOICE_ACTION_EVENTS"]))
finally: executor.close()
' >"${VOICE_EXECUTOR_LOG}" 2>&1 </dev/null &
  echo $! >"${VOICE_EXECUTOR_PID_FILE}"
}

stop_voice_executor() {
  if voice_executor_running; then kill -TERM "$(cat "${VOICE_EXECUTOR_PID_FILE}")" 2>/dev/null || true; fi
  rm -f "${VOICE_EXECUTOR_PID_FILE}"
}

voice_tts_running() {
  [ -f "${VOICE_TTS_PID_FILE}" ] && kill -0 "$(cat "${VOICE_TTS_PID_FILE}")" 2>/dev/null
}

start_voice_tts() {
  if [ "${VOICE_TTS_ENABLED}" != "1" ] || voice_tts_running; then return; fi
  touch "${VOICE_ACTION_EVENTS}"
  nohup env PYTHONPATH="${ROOT}/src" python3 -m lite3_voice.voice_tts \
    --events "${VOICE_ACTION_EVENTS}" --request-dir "${AUDIO_REQUEST_DIR}" \
    >"${VOICE_TTS_LOG}" 2>&1 </dev/null &
  echo $! >"${VOICE_TTS_PID_FILE}"
}

stop_voice_tts() {
  if voice_tts_running; then kill -TERM "$(cat "${VOICE_TTS_PID_FILE}")" 2>/dev/null || true; fi
  rm -f "${VOICE_TTS_PID_FILE}"
}

stop_voice_rag() {
  if voice_rag_running; then
    kill -TERM "$(cat "${VOICE_RAG_PID_FILE}")" 2>/dev/null || true
  fi
  rm -f "${VOICE_RAG_PID_FILE}"
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
  local rmw configured_host configured_port
  rmw="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" \
    | sed -n 's/^RMW_IMPLEMENTATION=//p' | tail -n 1)"
  if [ "${rmw}" != "rmw_cyclonedds_cpp" ]; then
    echo "[lite3-system] invalid RMW_IMPLEMENTATION: ${rmw:-unset}" >&2
    return 1
  fi
  configured_host="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" | sed -n 's/^MQTT_HOST=//p' | tail -n 1)"
  configured_port="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" | sed -n 's/^MQTT_PORT=//p' | tail -n 1)"
  if [ "${configured_host}" != "${BROKER_HOST}" ] || [ "${configured_port}" != "${BROKER_PORT}" ]; then
    echo "[lite3-system] broker profile differs from running container" >&2
    return 1
  fi
  docker exec "${NAME}" /usr/local/bin/lite3-mqtt-entrypoint \
    ros2 pkg prefix lite3_bringup >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_motion_state_receiver.py' >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_coyote_mqtt_bridge.py' >/dev/null
  docker exec "${NAME}" pgrep -f \
    '/workspace/lite3/scripts/run_realsense_ros_camera.sh' >/dev/null
}

start_system() {
  if [ "${BROKER_MODE}" = "test" ]; then
    ensure_test_broker
  else
    stop_test_broker
  fi

  if ! pgrep -af '/home/ubuntu/iq9_coyote/perception_node.py' >/dev/null; then
    mkdir -p "$(dirname "${COYOTE_PERCEPTION_LOG}")"
    nohup bash "${COYOTE_PERCEPTION_SCRIPT}" \
      >"${COYOTE_PERCEPTION_LOG}" 2>&1 </dev/null &
  fi
  if ! pgrep -af '[r]un_audio_cue_service.py' >/dev/null; then
    mkdir -p "${AUDIO_REQUEST_DIR}" "$(dirname "${AUDIO_SERVICE_LOG}")"
    if [ ! -x "${AUDIO_SERVICE_PYTHON}" ]; then
      AUDIO_SERVICE_PYTHON=python3
    fi
    nohup "${AUDIO_SERVICE_PYTHON}" "${AUDIO_SERVICE_SCRIPT}" \
      --request-dir "${AUDIO_REQUEST_DIR}" \
      >"${AUDIO_SERVICE_LOG}" 2>&1 </dev/null &
  fi
  start_voice_asr
  start_voice_rag
  start_voice_executor
  start_voice_tts

  for legacy in "${LEGACY_CONTAINERS[@]}"; do
    stop_and_remove "${legacy}"
  done

  if container_running "${NAME}"; then
    if verify_runtime \
      && current_generation_has_log "${NAME}" "MOTION_STATE receiver listening"; then
      echo "[lite3-system] already running: ${NAME} broker=${BROKER_HOST}:${BROKER_PORT} mode=${BROKER_MODE}"
      return
    fi
    echo "[lite3-system] replacing existing container for selected broker profile" >&2
    stop_and_remove "${NAME}"
  fi

  stop_and_remove "${NAME}"

  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[lite3-system] missing image ${IMAGE}; run scripts/build_mqtt_foxy_image.sh" >&2
    return 1
  fi
  mkdir -p "${SPOOL_DIR}"
  mkdir -p "${REALSENSE_OUTPUT_DIR}"
  mkdir -p "${REALSENSE_REQUEST_DIR}"
  mkdir -p "${AUDIO_REQUEST_DIR}"
  mkdir -p "${VOICE_RUNTIME_DIR}"

  docker_args=(
    --detach
    --name "${NAME}"
    --restart no
    --network host
    --device /dev/bus/usb:/dev/bus/usb
    --env PYTHONDONTWRITEBYTECODE=1
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    # IQ9 must receive the perception host's PointCloud2 over end0.  Pin
    # Cyclone to that interface instead of restricting DDS to loopback.
    --env ROS_LOCALHOST_ONLY=0
    --env "CYCLONEDDS_URI=file://${CYCLONEDDS_CONFIG}"
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    --env ROS_LOG_DIR=/tmp/ros/launch
    --env "MQTT_HOST=${BROKER_HOST}"
    --env "MQTT_PORT=${BROKER_PORT}"
    --env "MQTT_USER=${MQTT_USER:-}"
    --env "MQTT_PASS=${MQTT_PASS:-}"
    --volume "${ROOT}:/workspace/lite3:ro"
    --volume "${SPOOL_DIR}:${SPOOL_DIR}"
    --volume "${REALSENSE_OUTPUT_DIR}:${REALSENSE_OUTPUT_DIR}"
    --volume "${REALSENSE_REQUEST_DIR}:${REALSENSE_REQUEST_DIR}"
    --volume "${AUDIO_REQUEST_DIR}:${AUDIO_REQUEST_DIR}"
    --volume "${VOICE_RUNTIME_DIR}:${VOICE_RUNTIME_DIR}"
    --workdir /workspace/lite3
  )
  if [ -d "${HOME}/.ssh" ]; then
    docker_args+=(--volume "${HOME}/.ssh:/root/.ssh:ro")
  fi

  # Run the mounted workspace launch file directly so the system uses the
  # current IQ9 source rather than the launch copy baked into the image.
  docker run "${docker_args[@]}" "${IMAGE}" \
    ros2 launch /workspace/lite3/deploy/mqtt/lite3_bringup/launch/lite3_system.launch.py \
    "broker_host:=${BROKER_HOST}" \
    "broker_port:=${BROKER_PORT}" \
    "patrol_config:=${PATROL_CONFIG}" \
    "coyote_spool_dir:=${SPOOL_DIR}" \
    "voice_runtime_dir:=${VOICE_RUNTIME_DIR}" \
    "nav_network_interface:=${NAV_NETWORK_INTERFACE}" >/dev/null

  startup_deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  while container_running "${NAME}" && (( SECONDS < startup_deadline )); do
    if current_generation_has_log "${NAME}" "MOTION_STATE receiver listening" \
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
  if ! current_generation_has_log "${NAME}" "MOTION_STATE receiver listening" \
    || ! current_generation_has_log "${NAME}" "coyote bridge ready"; then
    docker logs --tail 100 "${NAME}" >&2 || true
    stop_and_remove "${NAME}"
    echo "[lite3-system] required processes did not become ready within ${STARTUP_TIMEOUT_SEC}s" >&2
    return 1
  fi
  docker update --restart unless-stopped "${NAME}" >/dev/null
  echo "[lite3-system] started: ${NAME} broker=${BROKER_HOST}:${BROKER_PORT} mode=${BROKER_MODE}"
}

case "${COMMAND}" in
  start)
    start_system
    ;;
  stop)
    stop_and_remove "${NAME}"
    stop_voice_asr
    stop_voice_rag
    stop_voice_executor
    stop_voice_tts
    if [ "${BROKER_MODE}" = "test" ]; then
      stop_test_broker
    fi
    echo "[lite3-system] stopped"
    ;;
  restart)
    stop_and_remove "${NAME}"
    start_system
    ;;
  status)
    if container_running "${NAME}"; then
      verify_runtime
      voice_asr_running || { echo "[lite3-system] voice ASR is not running" >&2; exit 1; }
      voice_rag_running || { echo "[lite3-system] voice RAG is not running" >&2; exit 1; }
      if [ "${VOICE_EXECUTE_ENABLED}" = "1" ]; then voice_executor_running || { echo "[lite3-system] voice executor is not running" >&2; exit 1; }; fi
      if [ "${VOICE_TTS_ENABLED}" = "1" ]; then voice_tts_running || { echo "[lite3-system] voice TTS is not running" >&2; exit 1; }; fi
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
    usage
    exit 2
    ;;
esac
