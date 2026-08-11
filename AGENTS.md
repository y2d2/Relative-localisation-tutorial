# Repository Guidelines for Relative-localisation-tutorial

Before doing anything else, you MUST say exactly:

"AGENTS FILE WAS LOADED"

## Git Rule
- Treat `main` and `master` as protected branches. If the current branch is `main` or `master`, create a new branch before making changes. Otherwise stay on the current branch by default.
- Auto-commit all changes with messages starting with `OC:` (e.g., `"OC: Added part_2.md spec"`). Keep each commit focused on one logical change.
- Always push to origin after committing, regardless of branch. 

## Default Environment
- use the venv in side the project root. 
- Install required packages via pip into the venv. Do not use system Python or global packages.

## Implementation Pattern
- Maintain a `src/` package layout with `pyproject.toml` for metadata.
- Keep existing modules under `src/`, tests under `tests/`.
- Configure pytest so tests import the local package without ad-hoc setup.
- Use the installed ARP_utility_code package for common utilities, sensor wrappers, and plotting helpers. Do not copy-paste code from ARP_utility_code into this repo.
- If functionality is missing inform the user and clear instruct on what functionality is missing. 

## Notebook-Building Rule
- Each `part_N.md` describes implementation specs for a dedicated Jupyter notebook.
- Before building a notebook, read the corresponding `part_N.md` and the root AGENTS.md.
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
- All new/modified files: owner `yuri`, group `gituser`, permissions `775`.
- After every git push, run `chown -R yuri:gituser .` in `/workspace/Python/Relative-localisation-tutorial`.

## Tool Installation Rule
- Install missing tools, dependencies, and packages as needed. Prefer apt, pip, uv. Keep scope to the venv or container.
- Do not stop when a common utility is missing — install it first and continue.

## Paper Search Rule
- Search papers / arXiv via the configured arXiv MCP server first. If missing, run `/workspace/scripts/setup-arxiv-mcp.sh`.
