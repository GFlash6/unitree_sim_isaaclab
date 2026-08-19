#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
source_ros_core
cd "${REPO_ROOT}"
exec ros2 launch nav_bringup holoagent_tasks.launch.py
