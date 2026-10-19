#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir="$bundle_dir/ros2_ws"
export PYTHONPATH="$workspace_dir/src/joint_effort_hardware:$workspace_dir/src/joint_effort_input:$workspace_dir/src/amw_pi_bridge${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$workspace_dir/work/test-tmp"
export TMPDIR="$workspace_dir/work/test-tmp"
exec /usr/bin/python3 -m unittest discover -s "$workspace_dir/tests" -v
