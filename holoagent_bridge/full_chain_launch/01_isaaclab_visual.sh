#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ISAAC_GUI=1
exec "${SCRIPT_DIR}/01_isaaclab.sh"
