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
patch_file="$workspace_dir/patches/sts-r02.patch"
if git -C "$dest" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo "R02 driver patch already applied"
else
  git -C "$dest" apply --check "$patch_file"
  git -C "$dest" apply "$patch_file"
fi
echo "sts_hardware_interface at $(git -C "$dest" log --oneline -1)"
