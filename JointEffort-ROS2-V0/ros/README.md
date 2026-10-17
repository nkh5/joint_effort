# Joint Effort ROS

This workspace runs our iron-arm model in RViz. It includes mock motors, collision checks, joystick input, and planned movements. The current launch is for our iron arm prototype only. 

All of the arm movement is simulated, we still need to test real motor driver and physical calibration. 

## Start

Use Ubuntu 24.04 and ROS 2 Jazzy. Install the dependencies in [Setup](docs/SETUP.md), then open a terminal in `ros2_ws`:

```bash
chmod +x scripts/*.sh
./scripts/with_ros.sh colcon build --symlink-install --base-paths src
./scripts/run_iron.sh
```

In another terminal, open the same folder:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_status.py
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_demo.py sequence --seconds 3
```

Run one movement tool at a time. See [the iron-arm guide](iron_prototype/README.md) for planning, the practice run, and stopping.

## Read next

- [Controller guide](iron_prototype/PICO-CONTROLLER-GUIDE.md): real input checks and the simulated joystick demo.
- [Code guide](docs/CODE.md): how the programs fits together.
- [Calibration](iron_prototype/calibration/README.md): collect and fit real measurements.
- [File list](docs/FILES.md): every file in this bundle and its purpose.

The CAD model uses a urdf export I did on October 10th. The most recent CAD changes have not been updated to this model. 
