#!/usr/bin/env bash
# Fetch adityakamath/sts_hardware_interface into src/ at the commit this workspace was tested with.
set -e
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
commit=eeb0ad0
dest="$workspace_dir/src/sts_hardware_interface"
if [ ! -d "$dest/.git" ]; then
  git clone https://github.com/adityakamath/sts_hardware_interface.git "$dest"
fi
git -C "$dest" fetch --quiet origin
git -C "$dest" checkout --quiet "$commit"
echo "sts_hardware_interface at $(git -C "$dest" log --oneline -1)"
