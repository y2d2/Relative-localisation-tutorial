# lightweight-dev Output — Task: Execute notebooks + export figures

**Task:** #3    **Session:** readme-and-review    **Date:** 2026-11-22

## Summary
I ran all four notebooks headless in place, with cwd `notebooks/`, `--ExecutePreprocessor.timeout=1800` and the `python3` kernel. That kernel resolves to `.venv/share/jupyter/kernels/python3`, so the venv is used and no ipykernel reinstall was needed. All four runs exited 0, and no code fixes were needed. I added `scripts/export_notebook_figures.py`, which uses only the standard library and `nbformat`. It writes every `image/png` output to `docs/images/part_<N>_fig_<K>.png`. I checked every exported PNG by eye: all are real, non-blank plots.

Runtime per notebook (wall clock, including kernel start):

| Notebook | Runtime | Code cells | Images |
|---|---|---|---|
| part_1.ipynb | 9 s | 3 | 3 |
| part_2.ipynb | 10 s | 4 | 5 |
| part_3.ipynb | 9 s | 3 | 4 |
| part_4.ipynb | 63 s | 4 | 3 |

## Exported figures

Cell index = 0-based index in `nb.cells`, counting markdown cells too.

| File | Source cell | Description |
|---|---|---|
| `docs/images/part_1_fig_1.png` | part_1 cell 3 | "Robot: Perfect vs Noisy Trajectory": one robot's ground-truth path (blue) against the measured pose (red dashed), with heading arrows, at low noise (sigma_x=0.01 m, sigma_theta=0.1 deg). The two curves almost overlap. |
| `docs/images/part_1_fig_2.png` | part_1 cell 5 | "Effect of uncertainty of global pose estimation on relative pose estimation": ground truth for Robot 1 (blue) and Robot 2 (green), plus Robot 2's estimate (red dashed). The estimate is widely scattered at high noise (0.1 m, 10 deg). |
| `docs/images/part_1_fig_3.png` | part_1 cell 5 | Two panels: "RMSE on Relative Pose Distance" and "RMSE on Relative Pose Angle" over time. Relative pose RMSE_pos = 0.93 m vs about 0.14 m for each robot; RMSE_ang = 14.56 deg vs about 10 deg. Shows that errors compound in the relative pose. |
| `docs/images/part_2_fig_1.png` | part_2 cell 3 | "Robot 1 Perfect Trajectory with UWB Anchors": the true path (blue), UWB-trilaterated position estimates (red dotted) and 3 anchors A1–A3 (red triangles). |
| `docs/images/part_2_fig_2.png` | part_2 cell 3 | Scatter of the per-sample position error of the UWB estimate, RMSE = 0.21 m. No title and no axis labels; the x-axis is the sample index. |
| `docs/images/part_2_fig_3.png` | part_2 cell 5 | "GDOP Heatmap with Robot Trajectory Overlay": a GDOP contour map (colorbar about 1.14–2.3) with the anchors and the robot trajectory (white) overlaid. |
| `docs/images/part_2_fig_4.png` | part_2 cell 5 | Two panels: "GDOP Along Trajectory" (about 1.15–1.23) and "Position Error Estimate from GDOP + Range Noise" (about 0.231–0.246 m) over time. |
| `docs/images/part_2_fig_5.png` | part_2 cell 7 | Two panels. Left, "Position Estimates: Perfect vs Mis-calibrated Anchors": the true trajectory, estimates from perfect and from mis-calibrated anchors, and true vs mis-calibrated anchor positions. Right, "Position Estimation Error Over Time": MRSE 0.21 m (perfect) vs 0.60 m (mis-calibrated). |
| `docs/images/part_3_fig_1.png` | part_3 cell 3 | "Robot: Perfect vs VIO Estimated Trajectory with Drift": the true path (blue) against the VIO estimate (red dashed), with heading arrows. The estimate drifts progressively. |
| `docs/images/part_3_fig_2.png` | part_3 cell 3 | Two panels showing robot 1's VIO error over time. Position RMSE = 0.49 m, growing with drift. Angle RMSE = 6.32 deg. The angle axis label reads "(deg)", but the plotted values look like radians. |
| `docs/images/part_3_fig_3.png` | part_3 cell 5 | "Two Robots: Perfect vs VIO Estimated Trajectories with Drift": the perfect and VIO paths for Robots 1 and 2, plus Robot 2's estimated relative trajectory (red dashed). |
| `docs/images/part_3_fig_4.png` | part_3 cell 5 | Two panels: "Error on position" and "Error on orientation" over time. Robot 1 RMSE = 0.94 m and Robot 2 RMSE = 1.01 m, against a relative-pose RMSE of 2.83 m. Shows VIO drift compounding in the relative estimate. |
| `docs/images/part_4_fig_1.png` | part_4 cell 3 | `track_robot_location_with_beacon` (known start pose, one UWB anchor at (5,5)): the true trajectory (blue) and the SLAM/UPF estimate (red dashed) closely follow each other, with active estimations in green. Plot has a legend and no title. |
| `docs/images/part_4_fig_2.png` | part_4 cell 5 | `estimate_robot_location_with_beacon` (unknown start pose, 8x8 hypotheses): a ring of killed hypotheses (light green) and surviving active hypotheses (green) arranged as rotationally symmetric "flower" trajectories around the anchor (black). Illustrates the rotational ambiguity of single-anchor ranging. No title. |
| `docs/images/part_4_fig_3.png` | part_4 cell 7 | `estimate_anchor_from_robot` (the inverse problem, 4x4 hypotheses): the robot/anchor trajectory (black) and the anchor's position estimates. The active hypothesis converges near the true anchor at (5,5) (blue square), and the others are killed (light green). No title. |

Total size of `docs/images/`: 1,290,528 bytes (about 1.3 MB) for 15 files. The smallest file is part_2_fig_2.png at 25 KB.

## Files
| File | Action | Why |
|---|---|---|
| `notebooks/part_1.ipynb` … `part_4.ipynb` | modify | Saved executed outputs, including inline plots. |
| `scripts/export_notebook_figures.py` | create | Reusable CLI that exports `image/png` outputs to `docs/images/`. |
| `docs/images/part_<N>_fig_<K>.png` (15 files) | create | Figures for the README (Task 4). |
| `planning/readme-and-review/lightweight-dev_task_3_execute_and_export.md` | create | This report. |

## Build / Tests
- nbconvert part_1..part_4 (`--execute --inplace`): PASS, all exit 0.
- Notebook check (every code cell has `execution_count`, 0 `error` outputs, at least 1 `image/png` each): PASS.
- Export script run: PASS, exit 0, 15 files.
- Export script negative case (run before Part 4 had finished, so `part_4.ipynb` had no images): PASS. It exited 1 with `ERROR: part_4.ipynb contains no image/png outputs`.
- Idempotency (second run, file list diffed): PASS, the lists are identical.
- PNG validity (magic bytes, header dimensions, >10 KB): PASS for all 15 files.
- Lint: not run, because `ruff` is not installed in the venv.

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | All four nbconvert runs exit 0; every code cell in every notebook has `execution_count` set and no `error` outputs. | PASS | |
| 2. | Each notebook contains ≥ 1 `image/png` output. | PASS | 3 / 5 / 4 / 3 |
| 3. | `docs/images/` contains ≥ 1 `part_N_fig_K.png` per part; each file > 10 KB and opens as a valid PNG. | PASS | Smallest file is 25 KB. |
| 4. | Re-running the export script produces the same file list (idempotent). | PASS | |
| 5. | Result report contains the filename → cell → description table. | PASS | See "Exported figures". |
| 6. | Commit `OC: Execute notebooks with outputs and export figures to docs/images` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed. Total size of `docs/images/` reported. | PASS | Size is about 1.3 MB. The commit is made together with this report. |

## Blockers
None.

## Next Steps
- Task 4 caption notes. Part 2 fig 2 and all three Part 4 figures have no plot title, so their README captions need to carry the context.
- Possible plot bug in `src/`, out of scope here. The angle axis of Part 3 figs 2 and 4 is labelled "(deg)", but the values look like radians.
