#!/usr/bin/env bash
set -euo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$workspace_dir"
exec "$workspace_dir/scripts/with_ros.sh" ros2 run joint_effort_hardware sts_commission "$@"
