#!/usr/bin/env bash
# One-command bench test of the loose STS servos. Run INSIDE the distrobox, from anywhere:
#   ./bench/run_bench.sh                 full run (~12 min)
#   ./bench/run_bench.sh --quick         shorter (~5 min)
#   ./bench/run_bench.sh --manual        also asks you for protractor readings (accuracy + backlash)
#   ./bench/run_bench.sh --ros-only      only the ROS part (G), e.g. after a run where it failed
# Options: --port /dev/serial/by-id/...  --ids 1,2,3,4,5  --rates "50 100 200"  --skip-ros  --yes
# Results: bench/results/<date-time>/report.md (+ plots/, raw json, launch logs)
set -euo pipefail
bench_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ws=$(dirname "$bench_dir")
cal="$ws/src/joint_effort_iron/config/sts_calibration.yaml"
port=$(python3 -c "import yaml,sys;print(yaml.safe_load(open(sys.argv[1]))['serial_port'])" "$cal" 2>/dev/null || true)
ids="1,2,3,4,5"; rates="50 100 200"; quick=""; manual=""; yes=""; skip_ros=""; ros_only=""
while [ $# -gt 0 ]; do
  case "$1" in
    --port) port="$2"; shift 2 ;;
    --ids) ids="$2"; shift 2 ;;
    --rates) rates="$2"; shift 2 ;;
    --quick) quick="--quick"; rates="50"; shift ;;
    --manual) manual="--manual"; shift ;;
    --yes) yes="--yes"; shift ;;
    --skip-ros) skip_ros=1; shift ;;
    --ros-only) ros_only=1; shift ;;
    -h|--help) sed -n 2,10p "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done
[ -n "$port" ] || { echo "No serial port: pass --port /dev/serial/by-id/..."; exit 2; }
ros_nodes='^[^ ]*(ros2_control_node|robot_state_publisher|spawner)( |$)'
if pgrep -f "$ros_nodes" >/dev/null; then
  echo "A ros2_control_node is running (arm stack, or a leftover from an earlier run)."
  echo "Stop it first - only one program may use the adapter:  pkill -INT -f ros2_control_node; sleep 3; pkill -9 -f ros2_control_node"; exit 1
fi
out="$bench_dir/results/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out"
echo "Results: $out"

torque_off() {   # final safety net: whatever happened, leave every motor limp
  python3 -c "import sys; sys.path.insert(0, '$bench_dir'); import stsbus; b = stsbus.Bus('$port'); [b.torque(int(i), False) for i in '$ids'.split(',')]" 2>/dev/null || true
}

if [ -z "$ros_only" ]; then
  echo; echo "################ Part 1: raw serial tests ################"
  python3 "$bench_dir/bench_serial.py" --port "$port" --ids "$ids" --out "$out" $quick $manual $yes
fi

launch_pid=""
stop_launch() {   # SIGINT the whole launch process group (driver turns torque off), wait, then force
  [ -n "$launch_pid" ] || return 0
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do pgrep -f "$ros_nodes" >/dev/null || break; sleep 0.5; done
  if pgrep -f "$ros_nodes" >/dev/null; then
    echo "  (ROS processes still running after 15 s - killing them)"
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    pkill -KILL -f "$ros_nodes" 2>/dev/null || true
    sleep 1
  fi
  wait "$launch_pid" 2>/dev/null || true
  launch_pid=""
}
cleanup() { stop_launch; torque_off; }
trap cleanup EXIT INT TERM

if [ -z "$skip_ros" ]; then
  [ -f "$ws/install/setup.bash" ] || { echo "Workspace not built (install/ missing) - skipping ROS part"; skip_ros=1; }
fi
if [ -z "$skip_ros" ]; then
  export JE_ROS_DOMAIN_ID=86
  for rate in $rates; do
    echo; echo "################ Part 2: sts_hardware_interface at $rate Hz ################"
    if pgrep -f "$ros_nodes" >/dev/null; then
      echo "ROS processes from the previous rate are still running - skipping $rate Hz"; continue
    fi
    setsid "$ws/scripts/with_ros.sh" ros2 launch "$bench_dir/bench.launch.py" port:="$port" rate:="$rate" ids:="$ids" \
      > "$out/launch_$rate.log" 2>&1 < /dev/null &
    launch_pid=$!
    ok=""
    for _ in $(seq 1 45); do
      if timeout 10 "$ws/scripts/with_ros.sh" ros2 control list_controllers 2>/dev/null | grep -q "bench_position_controller.*active"; then ok=1; break; fi
      kill -0 "$launch_pid" 2>/dev/null || break
      if grep -qE "Could not load and initialize hardware|Failed to 'configure'|Failed to 'activate'|process has died" "$out/launch_$rate.log"; then break; fi
      sleep 1
    done
    if [ -z "$ok" ]; then
      echo "Controllers did not come up at $rate Hz - see $out/launch_$rate.log"; tail -15 "$out/launch_$rate.log"
    else
      "$ws/scripts/with_ros.sh" python3 "$bench_dir/bench_bus.py" --rate "$rate" --out "$out/bus_$rate.json" $quick || true
    fi
    stop_launch
    sleep 1
  done
fi

echo; echo "################ Report ################"
python3 "$bench_dir/bench_report.py" "$out"
