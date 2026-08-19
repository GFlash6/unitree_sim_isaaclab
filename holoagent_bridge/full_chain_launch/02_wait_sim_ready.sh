#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_lib.sh
source "${SCRIPT_DIR}/_lib.sh"

exec /usr/bin/python3 "${SCRIPT_DIR}/wait_sim_ready.py" --timeout "${SIM_READY_TIMEOUT}"
