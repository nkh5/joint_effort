# Calibration

Everything lives in `src/amw_bringup/config/arm.yaml`. Each joint has:

| key | meaning |
| --- | --- |
| `motor_id` | servo ID on the bus |
| `zero` | raw encoder count (0..4095) when the joint is at angle 0 |
| `dir` | +1 or -1: which way the motor turns relative to the joint |
| `lower_deg`, `upper_deg` | software travel limits |
| `reference_deg` | the joint's angle when its link is on its physical 0-degree mark |
| `acceleration` | servo ACC register, 1..254 (100 = default; lower = gentler) |

The joint angle everywhere (ROS, RViz, `amw status`, the motor tools) is

```
angle = dir * (raw - zero) * 360 / 4096      degrees
```

That is the whole calibration model. The URDF never changes for calibration: its joint axes and
directions are fixed.

**ROS must be stopped** for every `amw calibrate` and `amw motors` command (they talk to the servos
directly, and the adapter can only have one user). They never turn torque on. Support the arm.

## The reference pose

The reference pose is the pose `Calibration.csv` was measured from: **every link on its physical
0-degree mark** (link parallel to its motor's long side). In joint angles (`reference_deg`):

| joint | at its mark |
| --- | --- |
| joint_1 base | 0.0 |
| joint_2 shoulder | +23.8 |
| joint_3 elbow | +48.2 (arm straight) |
| joint_4 wrist 1 | 0.0 |
| joint_5 wrist 2 | +88.6 |

The elbow's mark (arm straight) is **outside** its travel limits (-84.8..0). That is fine for
calibrating with torque off, but move the elbow back inside its limits before `amw start real`;
the motor check refuses to start otherwise.

## Set the zeros: `amw calibrate zero [joint ...]`

Use this after a motor swap, a re-centre, or if a joint reads wrong at its mark.

1. Power on, ROS stopped. Put the joints on their 0-degree marks.
2. `amw calibrate zero` (all joints) or `amw calibrate zero joint_3` (one joint).
3. It reads each motor, shows old and new `zero`, and asks before writing.

## Check directions and wiring: `amw calibrate check [joint ...]`

For each joint it tells you which way the iron tip should move for a + turn (from the arm model,
e.g. "the iron tip moves 17 mm forward, 14 mm up"). Move only that joint by hand 10-20 degrees that
way and press Enter. It reports:

- `OK`
- `REVERSED - set dir: X`: flip `dir` for that joint (it offers to). Then rerun `amw calibrate zero`
  for that joint, because its zero was computed with the old direction.
- `NO MOVE SEEN - but joint_N moved`: the motor IDs of two joints are swapped. Fix `motor_id`.

You can also check visually: `amw start real --readonly` (RViz needs a display), then move a joint
by hand and watch the model.

## Measure travel: `amw calibrate limits [joint ...]`

Live readout of every joint with the min/max seen. Move each joint slowly to both ends of its safe
travel (stop before collisions, cable strain or hard stops), then Ctrl-C. It proposes limits 2 degrees
inside what you reached (`--margin` to change) and asks before writing. You can also edit
`lower_deg`/`upper_deg` by hand. The mock start pose (SRDF `home`) must stay inside the limits.

## Servo gains

`src/amw_bringup/config/gains.yaml` holds P/I/D/deadband (and optionally punch) per joint. They live
in the servos' own memory and are **not** written at startup any more:

```bash
amw motors gains     # what the servos have now
amw motors tune      # write gains.yaml (shows the changes, asks first)
```

## If a joint's travel crosses the encoder wrap

`arm.yaml` is rejected with "crosses the encoder wrap at 0/4095" when a joint's travel would need raw
counts outside 0..4095. Put the joint in the middle of its travel and run
`amw motors recenter joint_N` (the motor then reads 2048 there), then `amw calibrate zero joint_N`.

## History

`archive/` keeps `Calibration.csv`, the bring-up log, the Oct 31 bench results, and
`sts_calibration_v1_before_cleanup.yaml`, the file `arm.yaml` was migrated from on 2026-11-13
(`zero` = old `center_steps`, `dir` = -1 = the old driver's fixed direction,
`reference_deg` = old `datum_offset_rad`). `tests/test_arm_config.py` checks that the migration gives
exactly the same angles.
