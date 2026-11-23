# Task Plan: Repo review, fixes, README + notebook figures

**Purpose:** Fix the review findings, execute the four notebooks (keeping outputs), export their figures as PNGs and write a complete README.

## Objective
The repository currently has a title-only README, dead/broken code paths (7 of 9 tests fail), stale metadata and a wrong method description in Part 4. End state: dead code removed, all tests pass, notebooks executed with outputs committed, figures exported to `docs/images/`, and a README that explains purpose, setup, the four parts (with figures), tests and layout.

## Context
- Repo: `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme` (branched from `main` @ 575e8b4, already pushed with upstream).
- Python env: `.venv` (Python 3.11) in the repo root. Installed: `manifpy`, `arp_utility_code`, `upf_rte`, `nbconvert 7.17`, `nbclient`, `ipykernel`, `pytest`. NOT installed: `ipympl`.
- Notebooks (`notebooks/part_1..4.ipynb`) do `sys.path.insert(0, '../src')` → must be executed with cwd = `notebooks/`. Plots use `plt.show()` with the default inline backend (no `%matplotlib widget`).
- Commits: `OC:` prefix, push after each task, never destructive git ops. End commit messages with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- AGENTS.md rules: `src/` layout, tests in `tests/`, notebooks only parameters + one-shot calls.

## Review findings (planner, on `main` @ 575e8b4)
| # | Finding | Action |
|---|---|---|
| R1 | `README.md` only contains a title | Task 4 |
| R2 | `pyproject.toml` description says "5 progressive parts" (there are 4) | Task 1 |
| R3 | `manifpy`, `upf_rte` imported but not declared as deps; `ipympl` declared but unused/not installed | Task 1 |
| R4 | `src/ekf.py` + `EKFConfig` unused by any notebook/test | Task 1 (delete) |
| R5 | `src/ukf_helpers.py`, `src/ukf_wrappers.py` unused by notebooks; their tests (`tests/test_2d_upf.py`, `tests/test_2d_upf_connected_agent.py`) fail with `TypeError` — wrappers out of sync with the `upf_rte` API (`set_initial_state` signature) | Task 1 (delete) |
| R6 | `src/_2D_UPF.py`: non-PEP8 module name (uppercase, leading underscore) | Task 1 (rename to `upf_2d.py`) |
| R7 | `src/part4_functions.py` mixes `from src.x import` and `from x import` (path-fragile) | Task 1 |
| R8 | Part 4 notebook intro says "SE(2) Extended Kalman Filter (EKF)"; code uses the UPF (`UPF_2D`) | Task 2 |
| R9 | Part 4 notebook calls pass no `seed` → filter runs not reproducible | Task 2 |
| R10 | AGENTS.md references `instructions/part_*.md` specs that do not exist | Reported only (out of scope) |
| R11 | `Transformation_Matrix_Fucntions` typo lives in upstream libs (`upf_rte`, `arp_utility_code`) | Reported only (external) |

## Tasks (all consecutive — each commits to the same branch)

### Task 1: Code cleanup & metadata → `work_plan_task_1_code_cleanup.md`
### Task 2: Part 4 notebook text + seeds (depends on #1) → `work_plan_task_2_part4_notebook.md`
### Task 3: Execute notebooks + export figures (depends on #1, #2) → `work_plan_task_3_execute_and_export.md`
### Task 4: Write README (depends on #3) → `work_plan_task_4_readme.md`

Results: `lightweight-dev_task_<N>_<desc>.md` in this folder.

## Out of Scope
- Changing simulation/filter algorithms or default parameters (other than adding `seed=42` in Part 4 calls).
- Fixing upstream libs (`libs/`, `upf_rte`, `arp_utility_code`, manif submodule).
- Editing `AGENTS.md`, `install.sh`, `planning/eurobin-deliverable/`.
- Opening a PR (user can ask afterwards).

## Notes
- [Assumption] Deleting `ukf_wrappers.py` + its two failing tests is the intended "remove dead code" fix (user chose "Report + all fixes"); the live UPF is `UPF_2D` in `_2D_UPF.py`.
- [Assumption] Figures are exported from executed-notebook outputs (`image/png`), not by re-running code separately.
- [Risk] Part 4 UPF runs may take minutes; use a generous nbconvert timeout (≥1800 s/cell).
- [Assumption] euROBIN is mentioned in the README acknowledgements without a grant number (user to add if needed).

## Follow-ups (round 2, requested 2026-11-23)
| # | Finding | Action |
|---|---|---|
| R12 | Part 3 orientation-error curves plotted in rad under a "deg" label; `plot_two_robots_vio` orientation subplot has no legend (`ax3[0].legend()` called twice) | Task 5 |
| R10 | AGENTS.md references missing `instructions/part_*.md` | Task 6 |
| R13 | No test exercises the notebooks end-to-end | Task 7 |

Tasks 5 → 6 → 7 run one after another (same branch, each commits; Task 5 and Task 7 both edit README.md).
- Task 5: `work_plan_task_5_part3_angle_units.md`
- Task 6: `work_plan_task_6_agents_md.md`
- Task 7: `work_plan_task_7_notebook_smoke_test.md`
