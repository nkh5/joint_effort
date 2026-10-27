# Real arm bring-up log

Fill in while following [REAL-ARM.md](../REAL-ARM.md). One line per check; write what you
actually saw, including "NO" and odd numbers. Commit this file with the calibration.

Date: ________   Who: ________   Laptop/Pi: ________   Supply voltage (multimeter): ______ V

## 0. Setup

| check | result |
| --- | --- |
| check_serial.sh: dialout, adapter rw, /dev/input/js*, serial+yaml ok | |
| get_driver.sh commit | |
| colcon: packages finished | |
| pytest: passed / failed | |

## 1. Mock regression

| check | result |
| --- | --- |
| check_iron_live.py: three PASS lines | |
| RViz home pose correct (elbow bent, iron above PCB) | |
| stick +X / +Y / twist+ direction | / / |
| mode pause, panel 2 resume, rotation yaw/pitch, panel 1 OFF | |
| joy device name (if changed) / stick_signs (if changed) | |

## 2. Datum

| joint | datum_offset_rad | deg | center_steps | encoder_shift_counts (2b) |
| --- | --- | --- | --- | --- |
| joint_1 base | | | | |
| joint_2 shoulder | | | | |
| joint_3 elbow | | | | |
| joint_4 wrist 1 | | | | |
| joint_5 wrist 2 | | | | |

fit_calibration.py last line: ____________________

## 3. Bus check (no ROS), each joint on its 0° mark

| joint | answers | raw at mark | csv_zero (+shift) | angle rad at mark | ≈ datum? | volt | temp |
| --- | --- | --- | --- | --- | --- | --- | --- |
| joint_1 | | | 2064 | | | | |
| joint_2 | | | 2035 | | | | |
| joint_3 | | | 3338 | | | | |
| joint_4 | | | 2004 | | | | |
| joint_5 | | | 2041 | | | | |

## 4-5. Read-only ROS

| joint | only its row moves | RViz same direction | seen min rad | seen max rad | URDF limit | limit changed to |
| --- | --- | --- | --- | --- | --- | --- |
| joint_1 | | | | | -0.524..0.524 | |
| joint_2 | | | | | -1.5..1.5 | |
| joint_3 | | | | | 0.7..2.3 | |
| joint_4 | | | | | -0.524..0.524 | |
| joint_5 | | | | | -0.524..0.524 | |

Axis flips / csv_angle_sign changes made: ____________________

## 6. Powered hold

| check | result |
| --- | --- |
| preflight OK | |
| no jump at torque on | |
| both controllers active | |
| /joint_states rate (Hz) | |
| /emergency_stop: torque off on all five | |

## 7. Jogs (±3°)

| joint | +3 got | -3 got | other joints still | real direction = RViz |
| --- | --- | --- | --- | --- |
| joint_1 | | | | |
| joint_2 | | | | |
| joint_3 | | | | |
| joint_4 | | | | |
| joint_5 | | | | |

## 8. Joystick on the real arm

| check | result |
| --- | --- |
| speed level used | |
| stick +X / +Y / twist+ (vs RViz world axes) | / / |
| mode pause / resume / rotation yaw / pitch | |
| panel 1 OFF stops | |
| anything odd (noise, jerk, stalls, heat) | |

## Notes
