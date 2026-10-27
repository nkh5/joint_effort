#!/usr/bin/env bash
# Can this shell (e.g. inside distrobox) open the servo adapter and the Pico?
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
port=$(python3 -c "import yaml,sys;print(yaml.safe_load(open(sys.argv[1]))['serial_port'])" \
  "$workspace_dir/src/joint_effort_iron/config/sts_calibration.yaml" 2>/dev/null)
echo "User: $(id -un)   groups: $(id -Gn)"
echo; echo "Serial devices by id:"; ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none - is the adapter plugged in?)"
echo; echo "Servo adapter from sts_calibration.yaml: $port"
if [ -e "$port" ]; then
  real=$(readlink -f "$port"); ls -l "$real"
  if [ -r "$real" ] && [ -w "$real" ]; then echo "  readable/writable: yes"; else echo "  readable/writable: NO (see REAL-ARM.md 0.3)"; fi
else
  echo "  NOT FOUND"
fi
echo; echo "Joysticks:"; ls -l /dev/input/js* 2>/dev/null || echo "  (no /dev/input/js* - is the Pico plugged in?)"
echo; python3 -c "import serial, yaml; print('python3 serial', serial.VERSION, '+ yaml: ok')" 2>&1
