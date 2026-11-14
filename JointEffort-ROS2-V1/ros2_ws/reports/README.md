# Saved model data and software results

These files record earlier project revisions. They are retained for traceability; a saved
PASS result does not validate the current checkout. Compare the tested source fingerprints
and model hash with the files being submitted, then rerun the applicable checks.

- `iron/`: earlier iron-arm software checks, collision samples,
  and the workspace study used by the hover tools. These results predate the current
  physical-arm geometry and calibration edits.
- `r02/`: saved R02 integration checks. `verification.json` records the tested revision,
  source fingerprints and URDF hash. The current physical-arm URDF differs from that
  tested model. See [r02/README.md](r02/README.md) for the original result descriptions.
- Physical jog measurements and plots are in `../results/`; calibration measurements are
  in `../calibration/`. They are retained separately from software simulation results.

`iron/workspace-study.json` contains proposed hover endpoints and modeled collision
results. Its `urdf_sha256` belongs to the model studied, not necessarily the active model.
Regenerate the study and repeat collision checks after changing geometry or joint limits.
The obsolete October placeholder motor map and old hull report were removed after the
history review. The November 13 calibration supersedes that proposed map, and all six
hashes in the removed hull report differ from the current mesh files. The retained
`r02/collision-hulls.json` matches the current hull meshes.

The obsolete two-arm prototype reports (`workspace-study.json`, `joint-reference.csv`,
and `collision-endpoints.json` at this folder's root) were removed with their prototype
workflows. They do not describe the current five-joint iron arm.

Mock checks do not establish physical accuracy, load capacity, stopping distance, or
clearance throughout every trajectory.
