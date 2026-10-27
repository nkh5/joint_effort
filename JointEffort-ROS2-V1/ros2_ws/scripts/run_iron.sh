#!/usr/bin/env bash
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=88
exec "$workspace_dir/scripts/with_ros.sh" ros2 launch joint_effort_iron mock.launch.py "$@"
