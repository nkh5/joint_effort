#!/usr/bin/env bash
# Fetch adityakamath/sts_hardware_interface into src/ at the commit this workspace was tested with,
# then apply the JointEffort goal-speed patch (scripts/sts_goal_speed.patch; enables run_real.sh
# servo_speed:=...; with that option unset the driver behaves exactly as upstream).
# Rebuild afterwards:  ./scripts/with_ros.sh colcon build --symlink-install --base-paths src
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
commit=eeb0ad0
dest="$workspace_dir/src/sts_hardware_interface"
patch="$workspace_dir/scripts/sts_goal_speed.patch"
if [ ! -d "$dest/.git" ]; then
  git clone https://github.com/adityakamath/sts_hardware_interface.git "$dest"
fi
git -C "$dest" fetch --quiet origin
if git -C "$dest" apply --reverse --check "$patch" 2>/dev/null; then
  git -C "$dest" apply --reverse "$patch"          # take the old copy off so the checkout is clean
fi
git -C "$dest" checkout --quiet "$commit"
git -C "$dest" apply "$patch"
echo "sts_hardware_interface at $(git -C "$dest" log --oneline -1) + goal-speed patch"
echo "Now rebuild: ./scripts/with_ros.sh colcon build --symlink-install --base-paths src"
