#!/usr/bin/env bash

set -euo pipefail

LAUNCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${LAUNCH_DIR}/../.." && pwd)"
RUN_STATE_DIR="${LAUNCH_DIR}/.run"
RUNS_ROOT="${LAUNCH_DIR}/runs"
export LAUNCH_DIR REPO_ROOT RUN_STATE_DIR RUNS_ROOT

# shellcheck source=config.sh
source "${LAUNCH_DIR}/config.sh"

require_file() {
    [[ -f "$1" ]] || { echo "缺少文件: $1" >&2; return 1; }
}

require_dir() {
    [[ -d "$1" ]] || { echo "缺少目录: $1" >&2; return 1; }
}

source_ros_core() {
    local nounset_was_on=0
    [[ $- == *u* ]] && nounset_was_on=1
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/HoloAgent/agentic_robot/thirdparty/install/setup.bash"
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/HoloAgent/agentic_robot/core/install/setup.bash"
    [[ ${nounset_was_on} -eq 1 ]] && set -u
}

source_ros_services() {
    source_ros_core
    local nounset_was_on=0
    [[ $- == *u* ]] && nounset_was_on=1
    set +u
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/HoloAgent/agentic_robot/services/install/setup.bash"
    [[ ${nounset_was_on} -eq 1 ]] && set -u
}

pid_file() {
    printf '%s/%s.pid\n' "${RUN_STATE_DIR}" "$1"
}

managed_pid() {
    local component=$1 file pid expected_ticks current_ticks
    file="$(pid_file "${component}")"
    [[ -r "${file}" ]] || return 1
    read -r pid expected_ticks < "${file}" || return 1
    [[ "${pid}" =~ ^[0-9]+$ && "${expected_ticks}" =~ ^[0-9]+$ ]] || return 1
    [[ -r "/proc/${pid}/stat" ]] || return 1
    current_ticks="$(awk '{print $22}' "/proc/${pid}/stat")"
    [[ "${current_ticks}" == "${expected_ticks}" ]] || return 1
    printf '%s\n' "${pid}"
}

start_background() {
    local component=$1 script=$2 pid ticks log_dir log_file
    if pid="$(managed_pid "${component}")"; then
        echo "[已运行] ${component} pid=${pid}"
        return 0
    fi

    mkdir -p "${RUN_STATE_DIR}"
    log_dir="${FULL_CHAIN_SESSION_DIR:?FULL_CHAIN_SESSION_DIR 未设置}/logs"
    mkdir -p "${log_dir}"
    log_file="${log_dir}/${component}.log"
    /usr/bin/python3 "${LAUNCH_DIR}/_process_group_exec.py" "${script}" \
        >"${log_file}" 2>&1 </dev/null &
    pid=$!
    sleep 1
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[失败] ${component}，日志: ${log_file}" >&2
        tail -n 30 "${log_file}" >&2 || true
        return 1
    fi
    ticks="$(awk '{print $22}' "/proc/${pid}/stat")"
    printf '%s %s\n' "${pid}" "${ticks}" > "$(pid_file "${component}")"
    echo "[已启动] ${component} pid=${pid} log=${log_file}"
}

process_group_alive() {
    kill -0 -- "-$1" 2>/dev/null
}

stop_component() {
    local component=$1 pid pgid file deadline
    file="$(pid_file "${component}")"
    if ! pid="$(managed_pid "${component}")"; then
        rm -f "${file}"
        echo "[未运行] ${component}"
        return 0
    fi
    pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ')"
    if [[ "${pgid}" != "${pid}" ]]; then
        echo "拒绝停止 ${component}: pid=${pid} 不是独立进程组组长" >&2
        return 1
    fi

    echo "[停止中] ${component} pid=${pid}"
    kill -INT -- "-${pgid}" 2>/dev/null || true
    deadline=$((SECONDS + 10))
    while process_group_alive "${pgid}" && (( SECONDS < deadline )); do sleep 0.2; done
    if process_group_alive "${pgid}"; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
        deadline=$((SECONDS + 5))
        while process_group_alive "${pgid}" && (( SECONDS < deadline )); do sleep 0.2; done
    fi
    if process_group_alive "${pgid}"; then
        echo "[强制停止] ${component}"
        kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi
    rm -f "${file}"
}

wait_until() {
    local seconds=$1 description=$2
    shift 2
    local deadline=$((SECONDS + seconds))
    until "$@"; do
        if (( SECONDS >= deadline )); then
            echo "等待超时: ${description} (${seconds}s)" >&2
            return 1
        fi
        sleep 2
    done
    echo "[已就绪] ${description}"
}

topic_has_message() {
    timeout 5 ros2 topic echo --once "$1" >/dev/null 2>&1
}

localization_is_tracking() {
    timeout 5 ros2 topic echo --once /localization/status 2>/dev/null \
        | grep -q 'state: TRACKING'
}

nav2_is_active() {
    timeout 5 ros2 lifecycle get /controller_server 2>/dev/null \
        | grep -q 'active'
}

tcp_port_is_open() {
    timeout 3 bash -c "exec 3<>/dev/tcp/127.0.0.1/$1" 2>/dev/null
}

hmsg_is_ready() {
    curl -fsS --max-time 3 http://127.0.0.1:8120/health >/dev/null 2>&1
}

ovo_has_real_result() {
    local payload
    payload="$(printf '{\"object_query\":\"%s\"}' "${OVO_READY_QUERY}")"
    curl -fsS --max-time 10 -H 'Content-Type: application/json' \
        -d "${payload}" http://127.0.0.1:8121/query >/dev/null
}

task_actions_are_ready() {
    local actions
    actions="$(timeout 5 ros2 action list 2>/dev/null)" || return 1
    grep -qx '/navigation/navigate_to_object' <<<"${actions}" \
        && grep -qx '/manipulation/execute_skill' <<<"${actions}"
}

robot_bridge_is_ready() {
    curl -fsS --max-time 3 http://127.0.0.1:8000/health >/dev/null 2>&1
}
