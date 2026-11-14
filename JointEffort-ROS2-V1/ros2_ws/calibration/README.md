# Calibration data

## Current physical arm

The current launch reads `../src/joint_effort_iron/config/sts_calibration.yaml`.
It contains populated V1-style centers, datum offsets and motor tuning. The
current fitting workflow uses `Calibration.csv` and `../tools/fit_calibration.py`.
Follow [REAL-ARM.md](../REAL-ARM.md) to check the selected model, direction and
usable travel. A populated file alone does not establish physical verification.

`bench-2026-10-31.*` and `bringup-log.md` retain measurement context.
`sts_calibration_legacy_v1.yaml` is a separate historical snapshot, not a duplicate
of the current settings. Frozen test fixtures remain under `../tests/fixtures`
so regression tests do not depend on changing physical measurements.

## R02 reference and migration

`r02-measurements.csv` and `original-r02-measurements.csv` retain R02 inputs.
They differ: the later sheet includes a separate Motor reference table.
`../tools/fit_r02_calibration.py` fits R02 angles and encoder counts. Use it only
with the matching R02 model and calibration workflow. The saved fit and software
results in `../reports/r02` refer to an earlier integration revision.

R02 angles use the R02 link references, which differ from the current physical
setup. A model migration requires new direction, travel and tip checks. See
[R02-INTEGRATION.md](../R02-INTEGRATION.md) for the reference procedure.

The R02 fitter reports every joint, including rejected data. Its default maximum
error is 0.5 degrees. Rejected fits return exit code 1 and do not overwrite settings.
The optional Motor table is analyzed separately and does not set R02 zeros.
