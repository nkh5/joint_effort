# ROS workspace

Start with [the project README](../README.md).

- `src`: ROS packages, robot descriptions, meshes, and settings.
- `scripts`: startup commands and the Jazzy environment.
- `tools`: demos, planning, calibration, and integration checks.
- `tests`: unit tests.
- `reports`: model data and test results.

Build with `./scripts/with_ros.sh colcon build --symlink-install --base-paths src`. Start the current arm with `./scripts/run_iron.sh`.

The current arm uses mock hardware. 
