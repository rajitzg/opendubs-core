#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <target_node> <params_file> [expected_param ...]" >&2
  exit 2
fi

TARGET_NODE="$1"
PARAM_FILE="$2"
shift 2
EXPECTED_PARAMS=("$@")

echo "Waiting for node ${TARGET_NODE}..."
until ros2 node list 2>/dev/null | grep -Fx -- "${TARGET_NODE}" >/dev/null; do
  sleep 1
done

if [[ "${TARGET_NODE}" == "/mavros/param" ]]; then
  ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}" >/dev/null || true
fi

if [[ ${#EXPECTED_PARAMS[@]} -gt 0 ]]; then
  echo "Waiting for expected params on ${TARGET_NODE}..."
  timeout_s=60
  elapsed=0

  while true; do
    listed="$(ros2 param list "${TARGET_NODE}" 2>/dev/null)"
    missing=0

    for param_name in "${EXPECTED_PARAMS[@]}"; do
      echo "${listed}" | grep -F -- "${param_name}" >/dev/null || {
        echo "[DEBUG] Still waiting for param ${param_name} on ${TARGET_NODE}..."
        missing=1
        break
      }
    done

    if [[ ${missing} -eq 0 ]]; then
      break
    fi

    if [[ ${elapsed} -ge ${timeout_s} ]]; then
      echo "[WARN] Timeout waiting for params on ${TARGET_NODE}; proceeding with load"
      break
    fi

    sleep 1
    elapsed=$((elapsed + 1))
  done
fi

echo "Loading params into ${TARGET_NODE} from ${PARAM_FILE}"
ros2 param load "${TARGET_NODE}" "${PARAM_FILE}"
