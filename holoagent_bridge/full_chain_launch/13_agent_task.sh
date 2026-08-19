#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

if (( $# == 0 )); then
    echo "用法: $0 \"13号机器人导航到黄色塑料料箱附近\"" >&2
    exit 2
fi
if [[ -z "${QWEN_API_KEY:-}" ]]; then
    echo "请先 export QWEN_API_KEY=..." >&2
    exit 2
fi
if ! robot_bridge_is_ready; then
    echo "robot_bridge 尚未在 http://127.0.0.1:8000 就绪" >&2
    exit 1
fi

mkdir -p "${AGENT_OUTPUT_ROOT}"
export DEFAULT_ROBOT_ID="${ROBOT_ID}"
robot_url_variable="ROBOT_${ROBOT_ID}_URL"
printf -v "${robot_url_variable}" '%s' "${!robot_url_variable:-http://127.0.0.1:8000}"
export "${robot_url_variable}"
export QWEN_MODEL="${QWEN_MODEL:-qwen3.7-plus}"
export QWEN_BASE_URL="${QWEN_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"

cd "${REPO_ROOT}/HoloAgent"
exec "${AGENT_PYTHON}" agentic_robot/agentOS/holoagent_agent.py \
    --mode single_robot \
    --output-root "${AGENT_OUTPUT_ROOT}" \
    --task "$*"
