# Calibration data

- `Calibration.csv`: hand measurements, rows `joint_N,angle_deg,encoder_counts`. `angle_deg` is
  measured from the physical 0° mark of each joint (link parallel to the motor's long side),
  positive = the direction the V0 URDF (all axes `0 0 1`) called positive. Encoder counts were
  taken after every motor was set to read 2048 at a mid position. Columns G-M (an older table)
  are ignored by the tools.
- `bringup-log.md`: fill in while following `../REAL-ARM.md`.

`../tools/fit_calibration.py` turns these into `../src/joint_effort_iron/config/sts_calibration.yaml`.
To re-measure a joint, replace its rows here and rerun `python3 tools/fit_calibration.py --write`.
