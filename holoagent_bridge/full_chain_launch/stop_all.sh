#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

components=(robot_bridge task_plane online_ovo hmsg nav2 localization cmd_vel_dds rgbd imu_clock mid360 isaac)
for component in "${components[@]}"; do
    stop_component "${component}"
done
echo "本启动器记录的进程已全部停止。"
