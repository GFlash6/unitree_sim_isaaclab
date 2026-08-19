#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"
require_dir "${PRIOR_MAP}"
source_ros_core
cd "${REPO_ROOT}"
exec ros2 launch fast_livo isaac_localization.launch.py prior_map:="${PRIOR_MAP}"
