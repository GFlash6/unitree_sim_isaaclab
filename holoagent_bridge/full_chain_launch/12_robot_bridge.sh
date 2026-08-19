#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
source_ros_services
cd "${REPO_ROOT}"
export ROBOT_ID CONTROL_URL
exec ros2 run robot_bridge robot_bridge_node
