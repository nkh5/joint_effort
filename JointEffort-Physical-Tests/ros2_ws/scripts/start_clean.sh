#!/usr/bin/env bash
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1
export ROS_DOMAIN_ID="${JE_ROS_DOMAIN_ID:-89}"
export XDG_CACHE_HOME="$workspace_dir/work/cache"
export XDG_CONFIG_HOME="$workspace_dir/work/config"
export ROS_HOME="$workspace_dir/work/ros_home"
export ROS_LOG_DIR="$workspace_dir/work/ros_logs"
if [ -f "$workspace_dir/install/setup.bash" ]; then source "$workspace_dir/install/setup.bash"; fi
exec "$@"
