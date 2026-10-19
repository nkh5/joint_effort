#!/usr/bin/env bash
set -euo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=89
exec "$workspace_dir/scripts/with_ros.sh" ros2 launch joint_effort_hardware input.launch.py "$@"
