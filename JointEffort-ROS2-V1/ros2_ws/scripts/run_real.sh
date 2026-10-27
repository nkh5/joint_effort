#!/usr/bin/env bash
# Real iron arm on the STS servos (ROS domain 88, same as scripts/run_input.sh).
#   ./scripts/run_real.sh readonly:=true    torque OFF, joint states only (calibration / checking)
#   ./scripts/run_real.sh teleop:=false     torque ON, controllers only (for scripts/run_jog.sh)
#   ./scripts/run_real.sh                   torque ON, full stack for joystick driving
# Runs tools/sts_preflight.py first (read-only serial check); refuses to launch if it fails.
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cal="$workspace_dir/src/joint_effort_iron/config/sts_calibration.yaml"
preflight_args=(--cal "$cal" --urdf "$workspace_dir/src/joint_effort_iron/urdf/iron.urdf")
for arg in "$@"; do
  case "$arg" in
    readonly:=true|readonly:=True|readonly:=1) preflight_args+=(--readonly) ;;
    port:=*) preflight_args+=(--port "${arg#port:=}") ;;
  esac
done
echo "== preflight (read-only serial check) =="
if ! python3 "$workspace_dir/tools/sts_preflight.py" "${preflight_args[@]}"; then
  echo "Preflight failed - NOT launching. Fix the problem above and run again."
  exit 1
fi
export JE_ROS_DOMAIN_ID=88
# Reads the calibration straight from the source tree, so edits need no rebuild (override with calibration:=...)
exec "$workspace_dir/scripts/with_ros.sh" ros2 launch joint_effort_iron real.launch.py \
  calibration:="$cal" "$@"
