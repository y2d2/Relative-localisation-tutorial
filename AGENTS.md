# Repository Guidelines for Relative-localisation-tutorial

AGENTS FILE WAS LOADED

## Git Rule
- Detect the current branch. If it is `main` or `master` (protected branches), create a new feature branch before making any changes.
- Always commit and push to origin after completing work, but never on protected branches.
- Keep commits focused on one logical change, with messages starting with `OC:` (e.g., `"OC: Added part_2.md spec"`).

## Default Environment
- Use the venv inside the project root.
- Install required packages via pip into the venv. Do not use system Python or global packages.
- After cloning, run `git submodule update --init --recursive` before building or installing anything.

## Implementation Pattern
- Maintain a `src/` package layout with `pyproject.toml` for metadata.
- Add dependencies to `[project] dependencies` and dev/test dependencies to `[project.optional-dependencies]`.
- Configure pytest in `pyproject.toml` under `[tool.pytest.ini_options]` with `pythonpath = ["src"]`.
- Keep existing modules under `src/`, tests under `tests/`.
- Use the installed ARP_utility_code package for common utilities, sensor wrappers, and plotting helpers. Do not copy-paste code from ARP_utility_code into this repo.
- If functionality is missing, inform the user and clearly instruct on what is needed.

## Notebook-Building Rule
- Each `instructions/part_*.md` describes implementation specs for a dedicated Jupyter notebook. These spec files are the source of truth — read them before building, do not modify them unless explicitly asked.
- Shared code goes in `src/`: configuration (`config.py`), sensor wrappers, trajectory helpers, and plotting.
- Keep notebooks clean: user-facing cells should only define parameters and call one-shot functions. All heavy simulation and plotting logic lives in `src/`.

## Test Structure
- Fast unit-style tests: files ending with `_ut.py`.
- Integration / sample-data / plotting tests: regular `test_*.py` with descriptive names.
- Plot tests may call `plt.show()` for human inspection; unit tests must be deterministic and non-interactive.
- Simulation tests must use deterministic seeds and explicitly construct sensors/trajectories (no hidden global state).

## File Hygiene
- Do not delete or overwrite user changes unless instructed.
- Leave unrelated untracked files alone; reuse existing paths and structure.
