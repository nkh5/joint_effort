#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$bundle_dir/ros2_ws"
exec /usr/bin/python3 tools/identify_motors.py "$@"
