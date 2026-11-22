# lightweight-dev Output — Task: Part 4 notebook text + seeds

**Task:** #2    **Session:** readme-and-review    **Date:** 2026-11-22

## Summary
- `notebooks/part_4.ipynb`, intro cell `part4-intro`: replaced "using an SE(2) Extended Kalman Filter (EKF)" with "using an Unscented Particle Filter (UPF), in which each particle runs an Unscented Kalman Filter (UKF)". The rest of the sentence is unchanged.
- `notebooks/part_4.ipynb`: added `seed=42` to the three filter calls (`track_robot_location_with_beacon`, `estimate_robot_location_with_beacon`, `estimate_anchor_from_robot`). All three functions accept `seed=None` in `src/part4_functions.py`.
- Typo pass over the markdown cells of all four notebooks:
  - part_1: "0.002m en 0.1 deg" -> "0.002 m and 0.1 deg"; "UWB system are" -> "UWB systems are"; "viceversa" -> "vice versa".
  - part_3: removed a double space in "Is the  evolution".
  - part_4: "estimation converges?" -> "estimation converge?"; "estimations fo the anchors position? Could this be usefull?" -> "estimations of the anchor's position? Could this be useful?".
  - part_2: nothing found.
- I edited the raw JSON with exact string replacements. Cell ids, the 1-space indent, outputs and metadata are unchanged. I did not execute any notebook.

## Files
| File | Action | Why |
|---|---|---|
| `notebooks/part_4.ipynb` | modify | Intro now says UPF instead of EKF, filter calls are seeded, typos fixed |
| `notebooks/part_1.ipynb` | modify | Markdown typo fixes |
| `notebooks/part_3.ipynb` | modify | Removed a double space in a markdown cell |
| `planning/readme-and-review/lightweight-dev_task_2_part4_notebook.md` | create | This result report |

## Build / Tests
- nbformat validation of all 4 notebooks: PASS
- grep checks for the EKF wording and for the seeds: PASS

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | `command grep -c "EKF\|Extended Kalman" notebooks/part_4.ipynb` → 0. | PASS | Output 0 |
| 2. | `command grep -c "seed=42" notebooks/part_4.ipynb` → ≥ 3 (one per filter call). | PASS | Output 6: 3 filter calls plus the 3 existing `TrajectoryConfig(..., seed=42)` lines |
| 3. | All four notebooks still load: `.venv/bin/python -c "import nbformat,glob; [nbformat.validate(nbformat.read(f,4)) for f in glob.glob('notebooks/*.ipynb')]"` exits 0. | PASS | Exit 0 |
| 4. | Commit `OC: Fix Part 4 method description (UPF) and seed filter runs` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed. | PASS | Committed together with this report and pushed to origin/docs/readme |

## Blockers
None

## Next Steps
- Task 3 (execute and export) will regenerate the Part 4 outputs with the new seeds. The stored outputs still come from the earlier unseeded runs.
