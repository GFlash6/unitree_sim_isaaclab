#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
require_dir "${HMSG_GRAPH_PATH}"
require_dir "${SIGLIP_SNAPSHOT}"
require_file "${SEMANTIC_SCENE_ROOT}/sim_to_map.json"
mkdir -p "$(dirname "${HMSG_ANCHOR_PATH}")"
cd "${REPO_ROOT}/HoloAgent/agentic_robot/fsr_vln"

exec env CUDA_VISIBLE_DEVICES="${HMSG_GPU}" PYTHONUNBUFFERED=1 \
    "${SEMANTIC_PYTHON}" scripts/hmsg_query_server.py \
    --scene-root "${SEMANTIC_SCENE_ROOT}" \
    --graph-path "${HMSG_GRAPH_PATH}" \
    --siglip-snapshot "${SIGLIP_SNAPSHOT}" \
    --sim-to-map "${SEMANTIC_SCENE_ROOT}/sim_to_map.json" \
    --anchor-path "${HMSG_ANCHOR_PATH}" \
    --room-name warehouse \
    --host 127.0.0.1 \
    --port 8120
