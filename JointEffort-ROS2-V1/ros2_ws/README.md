# ROS workspace (V1)

Start with [the V1 README](../README.md), then [REAL-ARM.md](REAL-ARM.md) for the real arm.

- `src`: ROS packages, robot descriptions, meshes, and settings. `src/sts_hardware_interface` is
  fetched by `scripts/get_driver.sh` (not committed).
- `scripts`: startup commands and the Jazzy environment (`with_ros.sh`).
- `tools`: calibration, preflight, demos, planning and integration checks.
- `calibration`: measurements (`Calibration.csv`) and the bring-up log.
- `tests`: unit tests and the software servo emulator.
- `reports`: model data and test results.

```bash
./scripts/get_driver.sh
./scripts/with_ros.sh colcon build --symlink-install --base-paths src
./scripts/with_ros.sh python3 -m pytest -q tests
```

| command | what |
| --- | --- |
| `./scripts/run_iron.sh` | mock arm (RViz) |
| `./scripts/run_input.sh mapping_confirmed:=true` | Pico input (works with mock or real) |
| `./scripts/run_datum.sh` | RViz sliders to measure your 0° marks |
| `./scripts/run_preflight.sh [--readonly]` | read-only servo check, no ROS |
| `./scripts/run_real.sh readonly:=true` | real arm, torque off |
| `./scripts/run_calibration_monitor.sh` | live angles while read-only |
| `./scripts/run_real.sh teleop:=false` + `./scripts/run_jog.sh joint_2 +3` | powered, single-joint jogs |
| `./scripts/run_real.sh` | powered, full stack for the joystick |

Do not run `tools/generate_iron.py`: it rebuilds `iron.urdf` and would undo the V1 axis flips,
elbow limit and home pose.
