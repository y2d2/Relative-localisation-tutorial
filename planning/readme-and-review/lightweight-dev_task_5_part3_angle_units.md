# lightweight-dev Output — Task: Fix Part 3 angle units + legend

**Task:** #5    **Session:** readme-and-review    **Date:** 2026-11-23

## Summary
Part 3 orientation-error curves are now plotted in degrees (`np.rad2deg(...)` on every `ax3[1].plot`) in both `plot_single_robot_vio` and `plot_two_robots_vio`. The error computations and return values are unchanged. In `plot_two_robots_vio`, the duplicated `ax3[0].legend()` after the orientation title is now `ax3[1].legend()`, so the orientation subplot has its legend back. `part_3.ipynb` was re-executed, the figures were re-exported (only `part_3_fig_2.png` and `part_3_fig_4.png` changed), and the Part 3 deg/rad bullet was removed from the README "Known limitations" section.

Part 1/2 findings (report only, no changes made):
- `src/part1_functions.py` (~lines 213-217): the angle curves are plotted in radians, and the y-label is `'RMSE Angle (rad)'`, so the axis is correct. The legend RMSE values are in deg (`np.rad2deg`), so the legend and axis units differ, but nothing is mislabelled.
- `src/part2_functions.py`: there are no angle-error plots. All y-labels are positions, GDOP or position error in m. Nothing to report.

## Files
| File | Action | Why |
|---|---|---|
| `src/part3_functions.py` | modify | Plot angle errors in deg; fix the orientation legend call |
| `notebooks/part_3.ipynb` | modify | Re-executed with the new plots |
| `docs/images/part_3_fig_2.png`, `docs/images/part_3_fig_4.png` | modify | Re-exported figures |
| `README.md` | modify | Removed the resolved Known-limitations bullet |

## Build / Tests
- nbconvert execute `part_3.ipynb`: PASS (written OK, 0 error outputs)
- `scripts/export_notebook_figures.py`: PASS (only the part_3 fig_2/fig_4 PNGs differ in git status)
- `.venv/bin/python -m pytest -q`: PASS (2 passed)

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | `command grep -n "ax3\[1\].plot" src/part3_functions.py` → every match wraps the angle array in `np.rad2deg(` | PASS | Lines 94, 209, 210, 211 |
| 2. | In `plot_two_robots_vio` there is exactly one `ax3[0].legend()` and one `ax3[1].legend()` | PASS | Lines 206 and 215 |
| 3. | `part_3.ipynb` executes with exit 0 and no `error` outputs | PASS | 0 error outputs |
| 4. | Visual check in step 5 passes | PASS | fig_2: angle axis 0 to ~11 deg; legend RMSE 6.32 deg, which fits. fig_4: angle axis 0 to ~42 deg; the lower subplot has a legend with 3 entries (RMSE 9.79 / 12.64 / 21.05 deg), which fits the curves |
| 5. | `command grep -c "radians" README.md` → 0 | PASS | 0 |
| 6. | `.venv/bin/python -m pytest -q` passes | PASS | 2 passed |
| 7. | Commit `OC: Plot Part 3 orientation errors in degrees and restore legend` ... pushed | PASS | This report is included in that commit |

## Blockers
None. Part 1/2/4 images were rewritten byte-identically, so there are no diffs.

## Next Steps
- Optional: in Part 1, make the legend and axis units match (plot in deg and relabel the axis). This is out of scope here.
