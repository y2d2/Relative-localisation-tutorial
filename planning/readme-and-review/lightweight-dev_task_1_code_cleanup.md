# lightweight-dev Output — Task: Code cleanup & metadata

**Task:** #1    **Session:** readme-and-review    **Date:** 2026-11-22

## Summary
Pre-delete grep showed that `ekf`, `EKFConfig`, `ukf_helpers` and `ukf_wrappers` were referenced only from the files being deleted, plus the `EKFConfig` definition in `src/config.py`. I removed the three dead modules and the two broken UPF test files with `git rm`, and removed `EKFConfig` from `src/config.py`. I renamed `src/_2D_UPF.py` to `src/upf_2d.py` with `git mv` and changed the import in `src/part4_functions.py`. All `from src.` imports are now flat. In `pyproject.toml` the description now says 4 parts, `manifpy` and `upf_rte` are added (with a comment that `install.sh` installs them from `libs/`), and `ipympl` is removed. Function behaviour is unchanged.

**Change beyond the plan:** to meet acceptance criterion 2, I also made the `from src.` imports flat in `src/part2_functions.py` (`from part1_functions import get_pose`) and `src/uwb.py` (`from config import UWBNoiseConfig`). I also deleted the commented-out line `# from src._2d_upf import UPFConnectedAgent_2D` in `src/part4_functions.py`. All three changes touch imports only.

## Files
| File | Action | Why |
|---|---|---|
| `src/ekf.py` | delete | Dead code, not used by any notebook |
| `src/ukf_helpers.py` | delete | Dead code, not used by any notebook |
| `src/ukf_wrappers.py` | delete | Dead code that no longer works with the current UKF API |
| `tests/test_2d_upf.py` | delete | Tested `ukf_wrappers`; was failing |
| `tests/test_2d_upf_connected_agent.py` | delete | Tested `ukf_wrappers`; was failing |
| `src/config.py` | modify | Removed the unused `EKFConfig` dataclass |
| `src/_2D_UPF.py` -> `src/upf_2d.py` | rename | Module name now follows Python naming conventions |
| `src/part4_functions.py` | modify | Flat imports, `upf_2d` import, dropped stale commented-out import |
| `src/part2_functions.py` | modify | Flat import (needed for criterion 2) |
| `src/uwb.py` | modify | Flat import (needed for criterion 2) |
| `pyproject.toml` | modify | Description and dependency fixes |
| `planning/readme-and-review/lightweight-dev_task_1_code_cleanup.md` | create | This report |

## Build / Tests
- `.venv/bin/python -m pytest -q`: PASS (2 passed, 0 failed; baseline was 7 failed, 2 passed)
- Import check from `notebooks/`: PASS (exit 0)
- Forbidden-pattern grep: PASS (no matches, grep exit 1)

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | `.venv/bin/python -m pytest -q` → all tests pass (expected 2 passed, 0 failed). | PASS | 2 passed |
| 2. | `command grep -rnE "from src\.\|_2D_UPF\|ukf_wrappers\|ukf_helpers\|EKFConfig\|import ekf\|from ekf" src tests notebooks --include=*.py --include=*.ipynb` → no matches. | PASS | Required the import-only edits to `part2_functions.py` and `uwb.py` |
| 3. | `cd notebooks && ../.venv/bin/python -c "import sys; sys.path.insert(0,'../src'); import part1_functions, part2_functions, part3_functions, part4_functions"` exits 0. | PASS | |
| 4. | `pyproject.toml` description says 4 parts, lists `manifpy` and `upf_rte`, no `ipympl`. | PASS | |
| 5. | One commit `OC: Remove dead EKF/UKF-wrapper code, rename UPF module, fix pyproject metadata` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer) pushed to `origin/docs/readme`. | PASS | This report is part of that commit |

## Blockers
None.

## Next Steps
- Notebooks may still show stale output that mentions removed modules. Notebook edits were out of scope for this task.
