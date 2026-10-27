#!/usr/bin/env bash
# Move one joint a few degrees:  ./scripts/run_jog.sh joint_2 +3   (needs run_real.sh teleop:=false running)
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=88
exec "$workspace_dir/scripts/with_ros.sh" ros2 run joint_effort_iron iron_jog "$@"
