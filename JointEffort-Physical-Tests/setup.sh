#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y python3 python3-serial
printf '%s\n' 'Installed Python dependencies. Run ./check_setup.sh next.'
