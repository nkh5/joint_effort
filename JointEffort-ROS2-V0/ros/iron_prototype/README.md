# Iron-arm mock

Open a terminal in `ros2_ws`. Start the model:

```bash
./scripts/run_iron.sh
```

Leave this terminal open. In a second terminal, open `ros2_ws` and check the system:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_status.py
```

## Simple movement

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_demo.py sequence --seconds 3
```

The arm turns, raises its tip, and lowers it. The command prints the measured mock movement. Joint 1 is the base, joints 2 and 3 are the main arm joints, and joints 4 and 5 are the wrist joints.

## Planned movement

Preview a route to 10 degrees at joint 1:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_plan.py --joint joint_1 --degrees 10
```

Add `--execute` to move the mock. To move to a checked hover above the board:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_plan.py --hover --execute
```

These tools check the model, hardware type, feedback, trajectory timing, and sampled collisions before execution. Use them instead of RViz's ordinary execution controls, which are disabled in this launch.

## Cold cycle

Preview the paths:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_cycle.py
```

Run two cycles:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_cycle.py --execute --cycles 2
```

The tip hovers 20 mm above the provisional board, lowers 10 mm, waits one second, and retreats. The lowest position remains about 10 mm above the board. This routine uses no heat or board contact.

## Stop

From another terminal in `ros2_ws`:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/iron_stop.py
```

Check the movement terminal for its stop result. Ctrl+C in a plan or cycle terminal also requests cancellation. Issue a new command to restart.

To close the system, stop the input launch first, then press Ctrl+C in the model launch terminal. Restarting the mock restores its saved CAD pose; it does not home a physical arm.

