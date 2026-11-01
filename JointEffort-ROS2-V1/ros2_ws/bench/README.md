# Motor test bench

One command tests all five STS3215s off the arm (no load) and writes a report.

## Setup

- All five motors daisy-chained on the Waveshare adapter, 12 V on, IDs 1-5.
- Sticks / protractor rig attached are fine: each motor swings **up to ±15°** around where it
  starts. Make sure nothing is in the way, and start each motor away from the ends of its encoder
  range (the test refuses a motor within 15° of the 0/4095 wrap).
- Nothing else may use the adapter: no `run_real.sh`, no `watch ... run_preflight.sh`.
- Workspace built once (`./scripts/with_ros.sh colcon build --symlink-install --base-paths src`).

## Run (inside the distrobox, from ros2_ws)

```bash
chmod +x bench/*.sh bench/*.py
./bench/run_bench.sh            # full run, ~12 min
./bench/run_bench.sh --quick    # ~5 min: fewer repeats, ROS test at 50 Hz only
./bench/run_bench.sh --manual   # full run + protractor prompts (true output angle, backlash)
./bench/run_bench.sh --ros-only # only part G (the ROS driver test), ~3 min
```

It asks once for Enter before the motors get torque. Torque is turned off at the end (and on Ctrl-C).
Other options: `--port /dev/serial/by-id/...` (default: the port in `sts_calibration.yaml`),
`--ids 1,2,3`, `--rates "50 100"`, `--skip-ros`, `--yes` (no Enter prompt).

Output: `bench/results/<date-time>/report.md` - summary of PASS / CHECK lines at the top, tables
below, plots in `plots/` (needs `python3-matplotlib`). Raw data: `serial.json`, `serial_traces.json`,
`bus_<rate>.json`, `launch_<rate>.log`.

## What it tests

| part | what | how | good result |
| --- | --- | --- | --- |
| A | settings of every servo | reads all EEPROM/RAM registers, flags any that differ between motors, mode != 0, angle limits, supply | identical settings, mode 0, ~12.6 V |
| B | raw bus, no ROS | exactly the driver's packets (one SyncWrite to all + one SyncRead) - flat out, then at 50 Hz with all five moving ±15° | 0 missed replies; tracking ~0.1° rms |
| C | step response | 0.5, 1, 3, 10° both ways, ACC 100 (driver default) and ACC 20; sampled as fast as the bus allows | final error ≤ 1-2 counts, moved/asked ≈ 1.00 |
| D | deadband | steps of 1-8 counts: smallest step that moves at all, and how far short it stops | moves for <= 4 counts, stops <= 2 short |
| E | slow ramp | 3° over 3 s with a new goal every 20 ms (= the jog test) for ACC 100/50/20/10 and a speed-limited variant | moves of ~1 count, pauses near the ideal ~88 ms, max error < 0.3° |
| G | through ROS | the real `sts_hardware_interface` streaming a new goal to all five every cycle (forward position controller) at 50, 100, 200 Hz: slow and fast sines | 0 read failures/recoveries, 0 stalls (position frozen while the goal keeps moving; pauses at turnarounds are the deadband, not stalls), joint_states at the set rate |
| F | (`--manual`) | commands 0, ±10, ±30° and asks for your protractor reading; then backlash (push the stick each way) | protractor matches encoder; backlash < 0.5° |

How to read the two questions you care about:

- **"Does the board talk to all 5 at 50 Hz?"** - parts B and G. B isolates the bus/adapter/power;
  G adds the driver and ros2_control. Read failures, recoveries or stalls in G but not in B point at
  the driver/timing; failures in both point at wiring, power or the adapter.
- **"Does it move exactly what I ask with no load?"** - part C. Then E tells you whether the
  stair-stepping exists without load: if the driver default already moves in jumps of several counts
  here, the jaggedness is control (fixable with ACC/speed/deadband); if it is smooth here, the
  jaggedness on the arm comes from load, friction and backlash.

The STS3215 encoder is on the output shaft, so encoder-based results cannot show play in the horn or
stick - that is what `--manual` (protractor) is for.

Not automated (needs weights / a stopwatch): holding stiffness - hang a known weight at a known
radius, note the encoder deflection and current with `./scripts/run_preflight.sh --readonly`.
