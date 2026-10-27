#!/usr/bin/env bash
# Datum measurement (visual only, no motors): RViz model + one slider per joint.
# Drag each link onto your physical 0 deg mark; close the window to print the values.
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=88
exec "$workspace_dir/scripts/with_ros.sh" ros2 launch joint_effort_iron datum.launch.py "$@"
