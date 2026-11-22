# Task Plan: Code cleanup & metadata

**Purpose:** Remove dead/broken modules, rename the UPF module, normalise imports and fix `pyproject.toml`.

## Objective
Make the codebase consistent: only code used by the notebooks remains, all tests pass, imports are uniform, and project metadata is correct.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme` (already tracking origin). Use `.venv/bin/python`.
- Notebooks import from `src/` via `sys.path.insert(0, '../src')`; pytest uses `pythonpath = ["src"]`. So imports inside `src/` must be flat (`from config import ...`), never `from src.x import ...`.
- Current baseline: `.venv/bin/python -m pytest -q` → 7 failed, 2 passed. Failures are all in `tests/test_2d_upf.py` and `tests/test_2d_upf_connected_agent.py` (`TypeError: TargetTrackingUKF.set_initial_state() missing ... 'ca_heading', 'ca_sigma_heading', 'sigma_uwb'`). `tests/test_part_4_upf.py` passes.
- No notebook uses `ekf.py`, `ukf_helpers.py` or `ukf_wrappers.py` (verify with grep before deleting).
- Note: the shell has a `grep` function wrapper that may misbehave; use `command grep`.

## Tasks

### Task 1: Code cleanup & metadata
- **Files:**
  - `src/ekf.py` (delete), `src/ukf_helpers.py` (delete), `src/ukf_wrappers.py` (delete)
  - `tests/test_2d_upf.py` (delete), `tests/test_2d_upf_connected_agent.py` (delete)
  - `src/config.py` (modify: remove `EKFConfig` dataclass)
  - `src/_2D_UPF.py` → `src/upf_2d.py` (rename with `git mv`)
  - `src/part4_functions.py` (modify)
  - `pyproject.toml` (modify)
- **Do:**
  1. Before deleting, `command grep -rn` for `ekf`, `EKFConfig`, `ukf_helpers`, `ukf_wrappers` across `src/ tests/ notebooks/` (exclude `__pycache__`). Only delete if the only references are within the files being deleted. If something else references them, stop and report.
  2. Delete the five files with `git rm`; remove `EKFConfig` from `src/config.py`.
  3. `git mv src/_2D_UPF.py src/upf_2d.py`; update every import of `_2D_UPF` (at least `src/part4_functions.py` line `from _2D_UPF import UPF_2D` → `from upf_2d import UPF_2D`; grep tests/notebooks too).
  4. In `src/part4_functions.py` replace `from src.vio_sensor`, `from src.part1_functions`, `from src.part3_functions`, `from src.part2_functions` with flat imports (`from vio_sensor ...` etc.). Do not change function behaviour.
  5. `pyproject.toml`: description → `"Relative localisation tutorial — 4 progressive parts"`; dependencies: add `"manifpy"` and `"upf_rte"` (installed from `libs/` by `install.sh`; add a TOML comment saying so), remove `"ipympl"`.
  6. Remove stale `src/__pycache__` is NOT required (untracked, ignored) — leave it.
- **Notes:** Do not touch `libs/`, notebooks, `install.sh`, `AGENTS.md`. Keep diffs minimal.
- **Acceptance:**
  1. `.venv/bin/python -m pytest -q` → all tests pass (expected 2 passed, 0 failed).
  2. `command grep -rnE "from src\.|_2D_UPF|ukf_wrappers|ukf_helpers|EKFConfig|import ekf|from ekf" src tests notebooks --include=*.py --include=*.ipynb` → no matches.
  3. `cd notebooks && ../.venv/bin/python -c "import sys; sys.path.insert(0,'../src'); import part1_functions, part2_functions, part3_functions, part4_functions"` exits 0.
  4. `pyproject.toml` description says 4 parts, lists `manifpy` and `upf_rte`, no `ipympl`.
  5. One commit `OC: Remove dead EKF/UKF-wrapper code, rename UPF module, fix pyproject metadata` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer) pushed to `origin/docs/readme`.

## Out of Scope
- Fixing `ukf_wrappers.py` against the new API (it is being deleted).
- Notebook edits, README, figure export.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_1_code_cleanup.md` (do not commit it separately — include it in the task commit).
- Never run destructive git commands.
