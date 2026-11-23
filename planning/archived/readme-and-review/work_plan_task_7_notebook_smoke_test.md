# Task Plan: Notebook smoke test

**Purpose:** Add a pytest that executes all four notebooks so breaking `src/` changes are caught.

## Objective
A parametrised test `tests/test_notebooks_execute.py` runs each `notebooks/part_*.ipynb` top-to-bottom with `nbclient` (on an in-memory copy, never writing the notebook files) and fails if any cell raises. It carries a `notebooks` marker so it can be skipped with `-m "not notebooks"`.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme`. Use `.venv/bin/python`. In this shell `grep` is a wrapper function, so use `command grep`.
- `.venv` has `nbclient 0.11`, `nbformat`, `ipykernel`; kernel `python3` resolves to the venv.
- Notebooks do `sys.path.insert(0, '../src')` → must execute with working dir `notebooks/` (`NotebookClient(nb, timeout=..., kernel_name="python3", resources={"metadata": {"path": <notebooks dir>}})`).
- Measured run times: part_1 9 s, part_2 10 s, part_3 9 s, part_4 63 s. Use timeout 1800 s per cell.
- Plots use `plt.show()` (inline backend) — no display needed. Part 4 uses `seed=42` already.
- `pyproject.toml` `[tool.pytest.ini_options]` currently has `markers = ["manual_plot: ..."]`, `python_files = ["test_*.py", "*_ut.py"]`, `pythonpath = ["src"]`.
- AGENTS.md: integration tests are regular `test_*.py` files; tests deterministic & non-interactive. Docstrings: Google-style.
- `nbclient`/`nbformat` must be declared: add them to `[project.optional-dependencies] dev` in `pyproject.toml`.

## Tasks

### Task 7: Notebook smoke test
- **Files:** `tests/test_notebooks_execute.py` (create), `pyproject.toml` (modify), `README.md` (modify — "Running the tests" section)
- **Do:**
  1. Create the test: discover notebooks with `sorted(Path(__file__).resolve().parents[1].joinpath("notebooks").glob("part_*.ipynb"))`; `pytest.mark.parametrize` with `ids` = file stem; mark module `pytestmark = pytest.mark.notebooks`; read with `nbformat.read(path, as_version=4)`, execute with `NotebookClient(...).execute()`; on `CellExecutionError` fail with the notebook name in the message. Also assert at least one `image/png` output exists after execution. Never write the notebook back to disk.
  2. Add a guard asserting ≥ 4 notebooks are discovered (separate small test, also under the marker).
  3. `pyproject.toml`: add marker `"notebooks: executes the tutorial notebooks end-to-end (slow, ~90 s); skip with -m 'not notebooks'"`; add `"nbclient"`, `"nbformat"` to `dev` extras.
  4. README "Running the tests": add the commands `pytest` (all, incl. notebooks) and `pytest -m "not notebooks"` (fast), one sentence each.
- **Acceptance:**
  1. `.venv/bin/python -m pytest -q` → all pass (expected: existing 2 + 4 notebook params + 1 guard = 7 passed); report wall time.
  2. `.venv/bin/python -m pytest -q -m "not notebooks"` → only the 2 existing tests run, pass.
  3. `git status` after the test run shows no modified `notebooks/*.ipynb` (proves no write-back).
  4. Sanity: temporarily break nothing in the repo — instead, verify failure handling by running the helper on an in-memory notebook with a cell `raise RuntimeError("x")` in a throwaway script in the scratch/tmp dir (not committed); it must raise/fail. Describe the result.
  5. Commit `OC: Add notebook execution smoke test` (trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`), including the result report, pushed.

## Out of Scope
- CI configuration, changing notebooks or src.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_7_notebook_smoke_test.md`.
- Never run destructive git commands.
