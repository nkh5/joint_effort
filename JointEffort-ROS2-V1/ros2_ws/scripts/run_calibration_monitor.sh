#!/usr/bin/env bash
# Live joint angles + raw servo steps while scripts/run_real.sh is running.
#   ./scripts/run_calibration_monitor.sh [path/to/sts_calibration.yaml]
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=88
cal="${1:-$workspace_dir/src/joint_effort_iron/config/sts_calibration.yaml}"
exec "$workspace_dir/scripts/with_ros.sh" python3 "$workspace_dir/tools/sts_calibration_monitor.py" "$cal"
