# Task Plan: Part 4 notebook text + seeds

**Purpose:** Correct the Part 4 method description and make its filter runs reproducible.

## Objective
`notebooks/part_4.ipynb` says it uses an "SE(2) Extended Kalman Filter (EKF)", but the code (`src/part4_functions.py` → `UPF_2D` in `src/upf_2d.py`) uses an Unscented Particle Filter (UPF: a particle filter whose particles each run a UKF). Its three filter calls also pass no `seed`, so results change on every run. Fix both.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme`. Task 1 already ran (module renamed to `upf_2d.py`, dead EKF code removed).
- `track_robot_location_with_beacon`, `estimate_robot_location_with_beacon`, `estimate_anchor_from_robot` in `src/part4_functions.py` all accept `seed=None` and call `np.random.seed(seed)` when it is given.
- Edit `.ipynb` JSON carefully (e.g. with a small Python script using `json`/`nbformat`, preserving cell ids and the 1-space indent format). Do not execute the notebook in this task, and leave existing outputs/metadata untouched apart from the edited sources.

## Tasks

### Task 2: Part 4 notebook text + seeds
- **Files:** `notebooks/part_4.ipynb` (modify)
- **Do:**
  1. In the intro markdown cell (id `part4-intro`) replace "using an SE(2) Extended Kalman Filter (EKF)" with wording that says the fusion uses an Unscented Particle Filter (UPF), where each particle runs an Unscented Kalman Filter. Keep the rest of the sentence.
  2. In each of the three code cells calling the Part 4 functions, add `seed=42` as a keyword argument to the call (e.g. `n_azimuth=4, n_heading=4, seed=42`).
  3. Scan all markdown cells of all four notebooks for remaining obvious typos (e.g. "bellow", "arround", "reltave", "singel", "intersted", "en" used for "and"); fix any found. Do not rewrite the teaching text otherwise.
- **Acceptance:**
  1. `command grep -c "EKF\|Extended Kalman" notebooks/part_4.ipynb` → 0.
  2. `command grep -c "seed=42" notebooks/part_4.ipynb` → ≥ 3 (one per filter call).
  3. All four notebooks still load: `.venv/bin/python -c "import nbformat,glob; [nbformat.validate(nbformat.read(f,4)) for f in glob.glob('notebooks/*.ipynb')]"` exits 0.
  4. Commit `OC: Fix Part 4 method description (UPF) and seed filter runs` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed.

## Out of Scope
- Executing notebooks, changing parameters other than adding `seed`, src changes.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_2_part4_notebook.md` (include in the task commit).
- Never run destructive git commands.
