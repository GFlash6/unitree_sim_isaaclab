#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

cd "${REPO_ROOT}"
set +u
# shellcheck disable=SC1091
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate unitree_sim_env
set -u

app_args=(
    --device "cuda:${ISAAC_GPU}"
    --livestream 0
    --enable_cameras
    --task Isaac-Move-Cylinder-G129-Dex1-Wholebody
    --action_source dds_wholebody
    --robot_type g129
    --enable_dex1_dds
    --enable_wholebody_dds
)

if [[ "${ISAAC_GUI}" == "1" ]]; then
    [[ -n "${DISPLAY:-}" ]] || { echo 'ISAAC_GUI=1 需要可用的 DISPLAY' >&2; exit 1; }
    exec env PYTHONUNBUFFERED=1 python sim_main.py "${app_args[@]}"
fi

exec env -u DISPLAY PYTHONUNBUFFERED=1 python sim_main.py --headless "${app_args[@]}"
