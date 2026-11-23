# lightweight-dev Output — Task: Fix AGENTS.md spec reference

**Task:** #6    **Session:** readme-and-review    **Date:** 2026-11-23

## Summary
Replaced the single "Notebook-Building Rule" bullet in `AGENTS.md` that referenced non-existent `instructions/part_*.md` spec files with the bullet text specified in the plan (notebook <-> `src/partN_functions.py` mapping, notebook markdown + README "Tutorial overview" as the description of each part, fallback to `instructions/part_*.md` if added later). No other line changed; the "AGENTS FILE WAS LOADED" first line and the generic "instructions specify reuse" line were left untouched.

## Files
| File | Action | Why |
|---|---|---|
| `AGENTS.md` | modify | Replace the stale spec-file bullet (1 line removed, 1 line added). |
| `planning/readme-and-review/lightweight-dev_task_6_agents_md.md` | create | This result report. |

## Build / Tests
- `git diff --numstat` before commit: PASS — `1 1 AGENTS.md` only.
- Repo facts check (`notebooks/part_1..4.ipynb`, `src/part1..4_functions.py` exist): PASS.
- No code changed; test suite not applicable.

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | `git diff --stat` shows only `AGENTS.md` (+ the result report), 1 line removed / 1 line added in AGENTS.md. | PASS | numstat `1 1 AGENTS.md`; report is the only other file. |
| 2. | `command grep -c "These spec files are the source of truth" AGENTS.md` → 0. | PASS | Output `0`. |
| 3. | Commit `OC: Update AGENTS.md notebook rule (no instructions/ spec files)` (trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`), including the result report, pushed. | PASS | Committed with that message and trailer on `docs/readme` and pushed to origin (see git log). |

## Blockers
None

## Next Steps
