# Task Plan: Fix Part 3 angle units + legend

**Purpose:** Make the Part 3 orientation-error plots show degrees (as labelled), restore the missing legend, and refresh the notebook, figures and README.

## Objective
In `src/part3_functions.py` the orientation-error curves are plotted in **radians** under a "RMSE Angle (deg)" axis label (only the legend RMSE values are converted with `np.rad2deg`). In `plot_two_robots_vio` the orientation subplot also has no legend (`ax3[0].legend()` is called twice, `ax3[1].legend()` never). End state: curves in degrees, both legends present, `part_3.ipynb` re-executed, Part 3 figures re-exported, README "Known limitations" item removed.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme` (tasks 1–4 committed and pushed). Use `.venv/bin/...`. In this shell `grep` is a wrapper function, so use `command grep`.
- Affected code (current state):
  - `plot_single_robot_vio` (~lines 80–99): `ax3[1].plot(t, angle_error_1, ...)` with `angle_error_1` in rad.
  - `plot_two_robots_vio` (~lines 188–216): `ax3[1].plot(t, angle_error_1 / angle_error_2 / angle_error, ...)` in rad; the line after `ax3[1].set_title('Error on orientation')` is `ax3[0].legend()` and should be `ax3[1].legend()`.
- Notebooks run with cwd `notebooks/`: `cd notebooks && ../.venv/bin/jupyter nbconvert --to notebook --execute --inplace --ExecutePreprocessor.timeout=1800 part_3.ipynb`.
- Figure export: `.venv/bin/python scripts/export_notebook_figures.py` (rewrites all `docs/images/part_N_fig_K.png`; only the part_3 files should change in content).
- README line ~192 under `## Known limitations`: "- The Part 3 angle-error plots are labelled "deg", but the plotted values are in radians."

## Tasks

### Task 5: Fix Part 3 angle units + legend
- **Files:** `src/part3_functions.py` (modify), `notebooks/part_3.ipynb` (re-execute), `docs/images/part_3_fig_*.png` (regenerate), `README.md` (modify)
- **Do:**
  1. In both functions, plot `np.rad2deg(<angle_error array>)` on `ax3[1]`; keep RMSE legend values as they are (already in deg). Do not change the error computations or the return values.
  2. In `plot_two_robots_vio`, replace the duplicated `ax3[0].legend()` after the orientation title with `ax3[1].legend()`.
  3. Also check `src/part1_functions.py` and `src/part2_functions.py` for the same mismatch (angle plotted in rad under a deg label). **Only report** findings; do not change them.
  4. Re-execute `part_3.ipynb` only; re-run the export script.
  5. Look at `docs/images/part_3_fig_2.png` and `part_3_fig_4.png` with your image-reading tool: the angle y-axis values must now be in a degree range consistent with the legend RMSE values, and fig_4's lower subplot must have a legend.
  6. README: remove the Part 3 deg/rad bullet from "Known limitations"; keep the other bullets. If a README caption mentions the unit issue, adjust it.
  7. `git status` must show no changes to `docs/images/part_1_*`, `part_2_*`, `part_4_*` byte-wise (if the export rewrites them identically, fine; if they differ, `git checkout` is NOT allowed — report it instead).
- **Acceptance:**
  1. `command grep -n "ax3\[1\].plot" src/part3_functions.py` → every match wraps the angle array in `np.rad2deg(`.
  2. In `plot_two_robots_vio` there is exactly one `ax3[0].legend()` and one `ax3[1].legend()`.
  3. `part_3.ipynb` executes with exit 0 and no `error` outputs.
  4. Visual check in step 5 passes (describe what you saw in the report).
  5. `command grep -c "radians" README.md` → 0 in Known limitations (the bullet is gone).
  6. `.venv/bin/python -m pytest -q` passes.
  7. Commit `OC: Plot Part 3 orientation errors in degrees and restore legend` (trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`), including the result report, pushed.

## Out of Scope
- Changing Part 1/2/4 code, error definitions ("RMSE" naming of mean absolute error), or parameters.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_5_part3_angle_units.md`.
- Never run destructive git commands.
