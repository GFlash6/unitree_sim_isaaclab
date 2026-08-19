#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

"${SCRIPT_DIR}/00_preflight.sh"
session_name="$(date +%Y%m%d_%H%M%S)"
FULL_CHAIN_SESSION_DIR="${RUNS_ROOT}/${session_name}"
export FULL_CHAIN_SESSION_DIR
mkdir -p "${FULL_CHAIN_SESSION_DIR}/logs" "${RUN_STATE_DIR}"
ln -sfn "${session_name}" "${RUNS_ROOT}/latest"
printf '%s\n' "${FULL_CHAIN_SESSION_DIR}" > "${RUN_STATE_DIR}/session"

cleanup_on_error() {
    local code=$?
    (( code == 0 )) && code=130
    trap - ERR INT TERM
    echo "启动中断，停止本启动器已经拉起的进程。" >&2
    "${SCRIPT_DIR}/stop_all.sh" || true
    exit "${code}"
}
trap cleanup_on_error ERR INT TERM

start_background isaac "${SCRIPT_DIR}/01_isaaclab.sh"
"${SCRIPT_DIR}/02_wait_sim_ready.sh"

start_background mid360 "${SCRIPT_DIR}/03_mid360_bridge.sh"
start_background imu_clock "${SCRIPT_DIR}/04_imu_clock_bridge.sh"
start_background rgbd "${SCRIPT_DIR}/05_rgbd_bridge.sh"
start_background cmd_vel_dds "${SCRIPT_DIR}/06_cmd_vel_dds_bridge.sh"

source_ros_services
wait_until 60 'MID360 ROS topic' topic_has_message /sensors/lidar/points
wait_until 60 'IMU ROS topic' topic_has_message /sensors/imu/data
wait_until 60 'RGB ROS topic' topic_has_message /sensors/front/rgb

start_background localization "${SCRIPT_DIR}/07_localization.sh"
wait_until "${LOCALIZATION_READY_TIMEOUT}" 'localization TRACKING' localization_is_tracking

start_background nav2 "${SCRIPT_DIR}/08_nav2.sh"
wait_until 180 'Nav2 controller active' nav2_is_active

start_background hmsg "${SCRIPT_DIR}/09_hmsg_server.sh"
wait_until 180 'HMSG HTTP server' hmsg_is_ready

start_background online_ovo "${SCRIPT_DIR}/10_online_ovo.sh"
wait_until 180 'online OVO TCP server' tcp_port_is_open 8121
wait_until "${OVO_READY_TIMEOUT}" "online OVO real query: ${OVO_READY_QUERY}" ovo_has_real_result

start_background task_plane "${SCRIPT_DIR}/11_task_plane.sh"
wait_until 90 'task ROS actions' task_actions_are_ready

start_background robot_bridge "${SCRIPT_DIR}/12_robot_bridge.sh"
wait_until 60 'robot bridge HTTP API' robot_bridge_is_ready

trap - ERR INT TERM
echo
echo "完整链路已就绪（未自动下发运动任务）。"
echo "日志目录: ${FULL_CHAIN_SESSION_DIR}/logs"
echo "状态检查: ${SCRIPT_DIR}/status.sh"
echo "提交任务: ${SCRIPT_DIR}/13_agent_task.sh \"13号机器人导航到黄色塑料料箱附近\""
echo "停止全部: ${SCRIPT_DIR}/stop_all.sh"
