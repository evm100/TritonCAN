#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${PROJECT_DIR}/config/vcan_blink_demo.yaml"
DEVICE_SCRIPT="${SCRIPT_DIR}/vcan_blink_device.py"
SESSION_NAME="vcan_blink_demo"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config not found at ${CONFIG_FILE}." >&2
  exit 1
fi

if [[ ! -x "${DEVICE_SCRIPT}" ]]; then
  echo "Device script ${DEVICE_SCRIPT} is not executable." >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  cat <<EOM
[!] tmux is not installed. Launch the endpoints manually with:
    python3 ${DEVICE_SCRIPT} --device device_a
    python3 ${DEVICE_SCRIPT} --device device_b
EOM
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Session ${SESSION_NAME} already exists. Attach with 'tmux attach -t ${SESSION_NAME}' or kill it first." >&2
  exit 1
fi

TMUX_BASE_CMD=("python3" "${DEVICE_SCRIPT}" "--config" "${CONFIG_FILE}")
DEVICE_A_CMD=("${TMUX_BASE_CMD[@]}" "--device" "device_a")
DEVICE_B_CMD=("${TMUX_BASE_CMD[@]}" "--device" "device_b")

tmux new-session -d -s "${SESSION_NAME}" "${DEVICE_A_CMD[@]}"

tmux split-window -h -t "${SESSION_NAME}:0" "${DEVICE_B_CMD[@]}"

tmux select-layout -t "${SESSION_NAME}:0" even-horizontal

echo "Attached tmux session '${SESSION_NAME}'. Use Ctrl+B then D to detach."
exec tmux attach-session -t "${SESSION_NAME}"
