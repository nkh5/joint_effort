#!/usr/bin/env bash
# Preview the AMW status screen in a desktop window on this PC (same domain as run_input.sh).
# On the Pi use the framebuffer instead - see LCD-SETUP.md.
#   ./scripts/run_lcd.sh            live, reads /amw/display_state
#   ./scripts/run_lcd.sh --demo     no ROS; keys: space m c + - f w j u n q
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export JE_ROS_DOMAIN_ID=88
exec "$workspace_dir/scripts/with_ros.sh" ros2 run joint_effort_input amw_lcd "$@"
