#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

failures=0
check_file() { require_file "$1" || failures=$((failures + 1)); }
check_dir() { require_dir "$1" || failures=$((failures + 1)); }
check_command() {
    command -v "$1" >/dev/null || { echo "缺少命令: $1" >&2; failures=$((failures + 1)); }
}

for command_name in bash curl nvidia-smi ros2 timeout; do
    check_command "${command_name}"
done

check_file "${REPO_ROOT}/sim_main.py"
check_file "${REPO_ROOT}/holoagent_bridge/mid360_to_ros2_topic.py"
check_file "${REPO_ROOT}/holoagent_bridge/imu_to_ros2_topic.py"
check_file "${REPO_ROOT}/holoagent_bridge/isaac_rgbd_pose_bridge.py"
check_file "${REPO_ROOT}/holoagent_bridge/cmd_vel_to_unitree_dds.py"
check_file "${NAV2_MAP}"
check_file "${PRIOR_MAP}/cloudGlobal.pcd"
check_file "${PRIOR_MAP}/singlesession_posegraph.g2o"
check_dir "${HMSG_GRAPH_PATH}"
check_file "${SEMANTIC_SCENE_ROOT}/runtime_evidence.json"
check_file "${SEMANTIC_SCENE_ROOT}/sim_to_map.json"
check_dir "${SIGLIP_SNAPSHOT}"
check_file "${SAM3_CHECKPOINT}"
check_file "${OVO_CONFIG}"
check_file "${SEMANTIC_PYTHON}"
check_file "${AGENT_PYTHON}"

if [[ -f "${NAV2_MAP}" ]]; then
    nav_image="$(awk '$1 == "image:" {print $2}' "${NAV2_MAP}")"
    check_file "$(dirname "${NAV2_MAP}")/${nav_image}"
fi

gpu_count="$(nvidia-smi -L 2>/dev/null | wc -l)"
for gpu_index in "${ISAAC_GPU}" "${OVO_GPU}" "${HMSG_GPU}"; do
    if [[ ! "${gpu_index}" =~ ^[0-9]+$ ]] || (( gpu_index >= gpu_count )); then
        echo "GPU ${gpu_index} 不存在" >&2
        failures=$((failures + 1))
    fi
done

source_ros_services
for package in fast_livo localization_monitor nav_bringup semantic_map_bridge \
               semantic_goal nav_executor manipulation robot_bridge; do
    if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
        echo "ROS 包未安装或未构建: ${package}" >&2
        failures=$((failures + 1))
    fi
done

if ! /usr/bin/python3 -c 'import rclpy, unitree_sdk2py, numpy, cv2' 2>/dev/null; then
    echo "系统 Python 缺少 ROS/Unitree bridge 依赖" >&2
    failures=$((failures + 1))
fi
if ! "${SEMANTIC_PYTHON}" -c 'import torch, rclpy, sam3, open_clip, fastapi, uvicorn' 2>/dev/null; then
    echo "语义 Python 环境缺少 OVO/SAM3/SigLIP 依赖" >&2
    failures=$((failures + 1))
fi
if ! "${AGENT_PYTHON}" -c 'import openai, requests, yaml' 2>/dev/null; then
    echo "Agent Python 环境缺少 openai/requests/yaml" >&2
    failures=$((failures + 1))
fi

for port_component in '8000 robot_bridge' '8120 hmsg' '8121 online_ovo'; do
    read -r port component <<<"${port_component}"
    if ss -ltnH "sport = :${port}" | grep -q . && ! managed_pid "${component}" >/dev/null; then
        echo "端口 ${port} 已被本启动器之外的进程占用" >&2
        failures=$((failures + 1))
    fi
done

mapfile -t sim_pids < <(pgrep -f 'python[^ ]* .*[/]sim_main.py|python[^ ]* sim_main.py' || true)
if (( ${#sim_pids[@]} > 0 )); then
    owned_sim_pid="$(managed_pid isaac 2>/dev/null || true)"
    owned_sim_pgid="$(ps -o pgid= -p "${owned_sim_pid}" 2>/dev/null | tr -d ' ')"
    for sim_pid in "${sim_pids[@]}"; do
        sim_pgid="$(ps -o pgid= -p "${sim_pid}" 2>/dev/null | tr -d ' ')"
        if [[ -z "${owned_sim_pgid}" || "${sim_pgid}" != "${owned_sim_pgid}" ]]; then
            echo "检测到启动器之外的 sim_main.py pid=${sim_pid}；拒绝再启动第二个仿真" >&2
            failures=$((failures + 1))
        fi
    done
fi

if [[ -z "${QWEN_API_KEY:-}" ]]; then
    echo "提示: QWEN_API_KEY 未设置；服务链可启动，但 13_agent_task.sh 暂不可用。"
fi

if (( failures > 0 )); then
    echo "预检失败: ${failures} 项" >&2
    exit 1
fi
echo "预检通过：地图、模型、Python 环境、ROS 包、GPU 和端口均可用。"
