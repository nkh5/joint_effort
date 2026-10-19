#!/usr/bin/env bash
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# All commands use one clean Jazzy environment, even if .bashrc sources Kilted.
exec env -i HOME="$HOME" PATH=/usr/bin:/bin DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" JE_ROS_DOMAIN_ID="${JE_ROS_DOMAIN_ID:-89}" bash --noprofile --norc "$workspace_dir/scripts/start_clean.sh" "$@"
