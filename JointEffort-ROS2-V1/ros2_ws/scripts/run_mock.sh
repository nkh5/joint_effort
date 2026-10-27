#!/usr/bin/env bash
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$workspace_dir/scripts/with_ros.sh" ros2 launch joint_effort_robot mock.launch.py "$@"
