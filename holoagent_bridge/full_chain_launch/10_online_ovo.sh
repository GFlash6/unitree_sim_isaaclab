#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
require_file "${OVO_CONFIG}"
require_file "${SAM3_CHECKPOINT}"
source_ros_core

mkdir -p "${OVO_DATA_DIR}"
experiment_name="${OVO_EXPERIMENT_PREFIX}_$(date +%Y%m%d_%H%M%S)"
cd "${REPO_ROOT}/HoloAgent/agentic_robot/fsr_vln"
exec env CUDA_VISIBLE_DEVICES="${OVO_GPU}" DISABLE_WANDB=true PYTHONUNBUFFERED=1 \
    "${SEMANTIC_PYTHON}" run_stream_mapping.py \
    --data_dir "${OVO_DATA_DIR}" \
    --scene_name live_session \
    --dataset_name IsaacG1 \
    --experiment_name "${experiment_name}" \
    --config_path "${OVO_CONFIG}"
