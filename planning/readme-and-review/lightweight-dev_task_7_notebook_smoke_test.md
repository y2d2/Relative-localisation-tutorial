# lightweight-dev Output — Task: Notebook smoke test

**Task:** #7    **Session:** readme-and-review    **Date:** 2026-11-23

## Summary
Added `tests/test_notebooks_execute.py`, which executes each `notebooks/part_*.ipynb` top-to-bottom in memory with `nbclient` (kernel `python3`, working dir `notebooks/`, 1800 s cell timeout). It fails with the notebook name on `CellExecutionError`, asserts at least one `image/png` output per notebook, and never writes back to disk. A guard test asserts at least 4 notebooks are discovered. Everything in the module carries the `notebooks` marker. Registered the marker and added `nbclient`/`nbformat` to the `dev` extras in `pyproject.toml`. Documented the full and fast pytest commands in the README "Running the tests" section.

## Files
| File | Action | Why |
|---|---|---|
| `tests/test_notebooks_execute.py` | create | Parametrised notebook execution smoke test, plus a discovery guard |
| `pyproject.toml` | modify | Register the `notebooks` marker; add `nbclient`, `nbformat` to `dev` extras |
| `README.md` | modify | Document `pytest` (all) and `pytest -m "not notebooks"` (fast) |
| `planning/readme-and-review/lightweight-dev_task_7_notebook_smoke_test.md` | create | This result report |

## Build / Tests
- `.venv/bin/python -m pytest -q`: PASS (7 passed in 99.01 s; wall time about 100 s)
- `.venv/bin/python -m pytest -q -m "not notebooks"`: PASS (2 passed, 5 deselected in 26.54 s)
- Throwaway failure sanity script (scratchpad, not committed): PASS (raised `CellExecutionError` as expected)

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | `.venv/bin/python -m pytest -q` → all pass (expected: existing 2 + 4 notebook params + 1 guard = 7 passed); report wall time. | PASS | 7 passed; pytest 99.01 s, wall time about 100 s |
| 2. | `.venv/bin/python -m pytest -q -m "not notebooks"` → only the 2 existing tests run, pass. | PASS | 2 passed, 5 deselected |
| 3. | `git status` after the test run shows no modified `notebooks/*.ipynb` (proves no write-back). | PASS | Only README.md, pyproject.toml and the new test were changed |
| 4. | Sanity: verify failure handling by running the helper on an in-memory notebook with a cell `raise RuntimeError("x")` in a throwaway script ... it must raise/fail. Describe the result. | PASS | `execute_notebook()` on a two-cell in-memory notebook (`print('ok')`, `raise RuntimeError("x")`) raised `CellExecutionError`, and its message contains `RuntimeError`. In the test this path becomes `pytest.fail` with the notebook name. |
| 5. | Commit `OC: Add notebook execution smoke test` (trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`), including the result report, pushed. | PASS | Committed and pushed to `origin/docs/readme` |

## Blockers
None.

## Next Steps
- Optional: wire `pytest -m "not notebooks"` into CI as the fast job (out of scope here).
