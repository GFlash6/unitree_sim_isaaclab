#!/usr/bin/env bash

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
set +e

components=(isaac mid360 imu_clock rgbd cmd_vel_dds localization nav2 hmsg online_ovo task_plane robot_bridge)
for component in "${components[@]}"; do
    if pid="$(managed_pid "${component}")"; then
        printf '%-16s RUNNING pid=%s\n' "${component}" "${pid}"
    else
        printf '%-16s STOPPED\n' "${component}"
    fi
done

if [[ -r "${RUN_STATE_DIR}/session" ]]; then
    echo "session: $(<"${RUN_STATE_DIR}/session")"
fi

source_ros_services
topics="$(timeout 5 ros2 topic list 2>/dev/null)"
for topic in /sensors/lidar/points /sensors/imu/data /sensors/front/rgb /localization/status; do
    if grep -qx "${topic}" <<<"${topics}"; then
        echo "ROS topic ${topic}: PRESENT"
    else
        echo "ROS topic ${topic}: MISSING"
    fi
done

nav2_is_active && echo 'Nav2: ACTIVE' || echo 'Nav2: NOT ACTIVE'
hmsg_is_ready && echo 'HMSG 8120: READY' || echo 'HMSG 8120: DOWN'
tcp_port_is_open 8121 && echo 'OVO 8121: LISTENING' || echo 'OVO 8121: DOWN'
robot_bridge_is_ready && echo 'robot_bridge 8000: READY' || echo 'robot_bridge 8000: DOWN'
