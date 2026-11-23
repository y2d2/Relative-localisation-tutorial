# Task Plan: Execute notebooks + export figures

**Purpose:** Run all four notebooks headless (outputs saved in place) and export every figure as a PNG for the README.

## Objective
Each `notebooks/part_N.ipynb` is executed top-to-bottom without errors, its outputs (including plots) are saved in the notebook, and every `image/png` output is written to `docs/images/` via a reusable script.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme`. Tasks 1–2 done (dead code removed, Part 4 seeded with `seed=42`).
- Env: `.venv` (Python 3.11) with `nbconvert 7.17`, `nbclient`, `ipykernel`. Check that a kernel pointing at the venv is used: notebooks declare kernelspec `python3`; run nbconvert via `.venv/bin/jupyter`, and confirm with `.venv/bin/jupyter kernelspec list` that `python3` resolves to the venv python (if not, run `.venv/bin/python -m ipykernel install --sys-prefix --name python3`).
- Notebooks use `sys.path.insert(0, '../src')` → execute with working directory `notebooks/` (nbconvert's `--ExecutePreprocessor.cwd`, or run from inside `notebooks/`).
- Plots use `plt.show()` + inline backend → outputs are `image/png` in `display_data`. No interactive widgets.
- Part 4 UPF runs can be slow: use `--ExecutePreprocessor.timeout=1800`.
- Headless: set `MPLBACKEND` unset/default (inline is selected by ipykernel automatically). Do not set `Agg` (it would suppress inline output).

## Tasks

### Task 3: Execute notebooks + export figures
- **Files:**
  - `notebooks/part_1.ipynb` … `part_4.ipynb` (modify — executed outputs)
  - `scripts/export_notebook_figures.py` (create)
  - `docs/images/part_<N>_fig_<K>.png` (create)
- **Do:**
  1. Execute each notebook in place: `cd notebooks && ../.venv/bin/jupyter nbconvert --to notebook --execute --inplace --ExecutePreprocessor.timeout=1800 part_N.ipynb` (N = 1..4). If one fails, read the traceback; fix only if the cause is a trivial path/import issue from Task 1 (report it), otherwise stop and report the blocker.
  2. Create `scripts/export_notebook_figures.py` (standard library + `nbformat` only; Google-style module/function docstrings): CLI `python scripts/export_notebook_figures.py [--notebooks-dir notebooks] [--out-dir docs/images]`; for each `part_*.ipynb` (sorted), walk code cells in order, base64-decode every `image/png` output and write `part_<N>_fig_<K>.png` (K = 1-based running index per notebook). Print a summary `file -> cell index`. Overwrite existing files. Exit non-zero if a notebook has zero images.
  3. Run it: `.venv/bin/python scripts/export_notebook_figures.py`.
  4. Visually inspect a few exported PNGs (open them with your image-reading tool) to confirm they are real plots (not blank). Record in the result report, for every exported file: filename, source cell, and a one-line description of what the figure shows (title/axes/legend) — Task 4 uses this to write captions.
- **Acceptance:**
  1. All four nbconvert runs exit 0; every code cell in every notebook has `execution_count` set and no `error` outputs.
  2. Each notebook contains ≥ 1 `image/png` output.
  3. `docs/images/` contains ≥ 1 `part_N_fig_K.png` per part; each file > 10 KB and opens as a valid PNG.
  4. Re-running the export script produces the same file list (idempotent).
  5. Result report contains the filename → cell → description table.
  6. Commit `OC: Execute notebooks with outputs and export figures to docs/images` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed. Total size of `docs/images/` reported.

## Out of Scope
- Changing notebook parameters, src logic, README.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_3_execute_and_export.md` (include in the task commit).
- Report runtime per notebook.
- Never run destructive git commands.
