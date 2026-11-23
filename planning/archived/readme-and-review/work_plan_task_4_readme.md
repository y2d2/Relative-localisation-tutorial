# Task Plan: Write README

**Purpose:** Replace the title-only `README.md` with full project documentation including notebook figures.

## Objective
A newcomer can read `README.md` and understand what the tutorial teaches, how to install it, how to run the notebooks and tests, and see one or more figures per part.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme`. Tasks 1–3 done. Read `planning/readme-and-review/lightweight-dev_task_3_execute_and_export.md` for the exported figure list + descriptions (this is the one allowed extra read).
- Facts to use (verified in the code):
  - Interactive, open-source Python tutorial on relative localisation for multi-robot systems, developed at VUB within euROBIN; given as a hands-on session to eight participants on 28 August 2026; the material may still change.
  - Part 1 — Global localisation: how global pose noise (σ_x, σ_θ; e.g. MoCap vs UWB-level accuracy) affects the global pose estimate and the relative pose between two robots (`src/part1_functions.py`).
  - Part 2 — UWB anchors: trilateration with UWB range noise, Geometric Dilution of Precision (GDOP) heatmap vs anchor geometry, and anchor mis-calibration (`src/part2_functions.py`, `src/uwb.py`).
  - Part 3 — SLAM/VIO drift: integrating noisy odometry (σ_v, σ_ω) causes unbounded drift; effect on relative pose between two robots (`src/part3_functions.py`, `src/vio_sensor.py`).
  - Part 4 — One UWB beacon: fusing odometry with ranges to one anchor in an Unscented Particle Filter (UPF; `src/upf_2d.py`, `src/part4_functions.py`) to (a) track a robot from a known start, (b) estimate the robot's location from an unknown start, (c) estimate the anchor's position from the robot's trajectory — highlighting observability limits.
  - Setup: Linux, Python 3.11; system packages `libeigen3-dev python3.11-dev build-essential cmake ninja-build`; `pip install "pybind11[global]"`; clone with submodules; run `./install.sh`, which creates `.venv`, builds `manifpy` from the `libs/manif` submodule, installs `libs/arp_utility_code-0.1.0.tar.gz` and converts/installs `libs/UPF_RPE-0.0.1.tar.gz` as `upf_rte`, plus jupyter/pytest. Then `source .venv/bin/activate`, `jupyter lab notebooks/`.
  - Tests: `pytest` (config in `pyproject.toml`; `manual_plot` marker for interactive plot tests).
  - Figures regenerate with `python scripts/export_notebook_figures.py` after executing the notebooks.
  - Layout: `notebooks/`, `src/` (config dataclasses, trajectory, uwb, vio_sensor, relative_pose, plotting, partN_functions, upf_2d), `tests/`, `scripts/`, `docs/images/`, `libs/`, `install.sh`.
  - Licence: MIT (from `pyproject.toml`); author Yuri Durodie.

## Tasks

### Task 4: Write README
- **Files:** `README.md` (modify — full rewrite)
- **Do:** Write `README.md` with sections: title + one-paragraph intro; "Tutorial overview" (one subsection per part: 2–4 sentence description, key parameters to play with, 1–2 embedded figures via relative paths `docs/images/part_N_fig_K.png` with short italic captions based on the Task 3 descriptions — pick the most illustrative ones, max ~8 images total); "Requirements"; "Installation"; "Running the notebooks" (mention notebooks are committed with outputs; re-run to experiment); "Regenerating the figures"; "Running the tests"; "Project structure" (tree block); "Acknowledgements" (euROBIN, VUB, session on 28 Aug 2026); "License". Keep it concise (~150–250 lines), plain GitHub Markdown, no HTML except optional `<img width>` if a figure is very large.
- **Acceptance:**
  1. Every image path referenced in `README.md` exists on disk (check with a small script/grep).
  2. All four parts are described, and Part 4 says UPF (no "EKF" anywhere in the README).
  3. Install steps match `install.sh` (venv name `.venv`, Python 3.11, submodule step).
  4. No statement contradicts the code facts above; no invented grant numbers, URLs or results.
  5. Commit `OC: Write README with tutorial overview, setup and notebook figures` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed.

## Out of Scope
- Code/notebook changes, PR creation.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_4_readme.md` (include in the task commit).
- Never run destructive git commands.
