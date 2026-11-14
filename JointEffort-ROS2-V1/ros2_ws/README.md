# Joint Effort ROS workspace

The current mock and real launches use `src/joint_effort_iron/urdf/iron.urdf`.
Its visual meshes come from `joint_effort_arm_draft_description`; the current motor
settings are in `src/joint_effort_iron/config/sts_calibration.yaml`.
Follow [REAL-ARM.md](REAL-ARM.md) for the physical-arm workflow.

The R02 source and migration tools remain available, but the current runtime is
not the R02 revision recorded in [R02-INTEGRATION.md](R02-INTEGRATION.md).
Running `tools/generate_iron.py` replaces the runtime model, SRDF and collision
meshes; it is a model migration, not a normal startup or cleanup step.

- `src`: arm control, Pico input, shared kinematics, visual models and dependencies.
- `scripts`: startup commands and the clean Jazzy environment.
- `tools`: calibration, motion tools, model generation and software checks.
- `calibration`: current measurement inputs and records from earlier measurements.
- `bench`: serial-bus and motor-gain diagnostics used by the physical tools.
- `patches`: retained R02 driver patch; the current fetch script applies its speed patch.
- `tests`: regression tests and the software servo emulator.
- `reports`: saved results labeled by model revision; see [reports/README.md](reports/README.md).
- `results`: distinct physical joint-tracking captures and plots.

`joint_effort_robot` now supplies the shared kinematics library. The earlier
two-arm mock launch and its task bridge have been removed. Use
`./scripts/run_iron.sh` for the current iron-arm mock.

Requires Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2, ros2_control, RViz, joy, colcon,
and the Python dependencies listed in the packages.

Build and test from this folder:

```bash
./scripts/get_driver.sh
mkdir -p work/tmp
./scripts/with_ros.sh env CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --parallel-workers 1
./scripts/with_ros.sh env TMPDIR="$PWD/work/tmp" python3 -m unittest discover -s tests -v
```

See [REAL-ARM.md](REAL-ARM.md) before connecting to the assembled arm.
