# Calibration data

## Current model: R02

- `r02-measurements.csv`: fill in measured R02 angles and raw encoder counts for each joint.
- `../src/joint_effort_iron/config/sts_calibration.yaml`: active R02 motor map. Empty values
  and false verification flags are intentional until the assembled arm is measured.
- `../tools/fit_r02_calibration.py`: checks the measurements; `--write` saves the fitted map.

Use at least three points per joint: zero and a negative/positive angle, spanning at least
10 degrees. Five or more points give a better consistency check. Angles are relative to the
R02 link references, not the motor case. Use the column names already in the worksheet.

```bash
python3 tools/fit_r02_calibration.py
python3 tools/fit_r02_calibration.py --write
```

Run these from `ros2_ws`. The tool does not connect to motors. Each write resets the physical
verification flags. Confirm direction, usable travel and tip position separately.
See [REAL-ARM.md](../REAL-ARM.md) for the physical checks.

## Earlier measurements

`Calibration.csv`, `bench-2026-10-31.*`, the bring-up notes and
`sts_calibration_legacy_v1.yaml` record earlier measurements and settings. Their zero references
are different from R02. Keep them as records; they are not the active R02 calibration.
The older `fit_calibration.py` tool is retained for those data and rejects the R02 settings.
