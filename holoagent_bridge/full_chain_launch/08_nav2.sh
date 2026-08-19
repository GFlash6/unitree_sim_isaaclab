#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
require_file "${NAV2_MAP}"
source_ros_core
cd "${REPO_ROOT}"
exec ros2 launch nav_bringup g1_navigation2_launch.py \
    map:="${NAV2_MAP}" use_sim_time:=true
