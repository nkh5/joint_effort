#!/usr/bin/env bash
# Read-only servo check, no ROS:  ./scripts/run_preflight.sh [--readonly] [--port /dev/serial/by-id/...]
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 "$workspace_dir/tools/sts_preflight.py" "$@"
