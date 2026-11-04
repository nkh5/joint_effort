# Joint Effort ROS workspace — R02

The iron arm now uses the R02 CAD geometry. Start with [R02-INTEGRATION.md](R02-INTEGRATION.md)
for the changes, file map, test results and mock demo commands.

The software has been tested with mock motors. The assembled arm still needs R02 encoder
measurements, direction checks, travel checks and a measured tip offset before powered use.
Old V1 calibration settings are saved in `calibration/sts_calibration_legacy_v1.yaml`.
They are not the calibration for this model.

- `src`: ROS packages, geometry, motor driver interfaces and settings.
- `scripts`: startup commands and the clean Jazzy environment.
- `tools`: calibration, model generation, motion tools and software checks.
- `calibration`: measurement sheets and earlier measurements kept for reference.
- `patches`: the changes applied to the pinned STS driver.
- `tests`: unit tests and the software servo emulator.
- `reports/r02`: results for this model. Older reports describe earlier models.

Requires Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2, ros2_control, RViz, joy, colcon,
and the Python dependencies listed in the packages.

Build from this folder:

```bash
./scripts/get_driver.sh
mkdir -p work/tmp
./scripts/with_ros.sh env CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --parallel-workers 1
./scripts/with_ros.sh env TMPDIR="$PWD/work/tmp" python3 -m unittest discover -s tests -v
```

See [REAL-ARM.md](REAL-ARM.md) before connecting this version to the assembled arm.
