# Task Plan: Fix AGENTS.md spec reference

**Purpose:** Remove the reference to non-existent `instructions/part_*.md` specs from AGENTS.md.

## Objective
`AGENTS.md` (section "Notebook-Building Rule", line ~26) says each `instructions/part_*.md` is the source of truth for a notebook, but no `instructions/` folder exists. Replace that bullet with guidance that matches the repo, so agents don't search for missing files.

## Context
- Repo `/workspace/Python/Relative-localisation-tutorial`, branch `docs/readme`. The user explicitly asked for this edit.
- Current bullet (verbatim): "- Each `instructions/part_*.md` describes implementation specs for a dedicated Jupyter notebook. These spec files are the source of truth — read them before building, do not modify them unless explicitly asked."
- Repo facts: notebooks are `notebooks/part_1.ipynb` … `part_4.ipynb`; each has markdown cells explaining the exercise; `README.md` has a "Tutorial overview" with one subsection per part; shared logic is in `src/partN_functions.py`.
- Line ~49 "Reuse trajectory generators across scenarios when instructions specify reuse" is generic — leave it.
- The first line "When reading this document print "AGENTS FILE WAS LOADED"…" must stay unchanged.

## Tasks

### Task 6: Fix AGENTS.md spec reference
- **Files:** `AGENTS.md` (modify)
- **Do:** Replace only the bullet quoted above with: "- Each notebook `notebooks/part_N.ipynb` has a matching `src/partN_functions.py`. The notebook's markdown cells and the README "Tutorial overview" describe what each part teaches; keep them consistent when changing a part. (There are no separate spec files; if `instructions/part_*.md` files are added later, treat them as the source of truth.)" Change nothing else.
- **Acceptance:**
  1. `git diff --stat` shows only `AGENTS.md` (+ the result report), 1 line removed / 1 line added in AGENTS.md.
  2. `command grep -c "These spec files are the source of truth" AGENTS.md` → 0.
  3. Commit `OC: Update AGENTS.md notebook rule (no instructions/ spec files)` (trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`), including the result report, pushed.

## Out of Scope
- Any other AGENTS.md change.

## Notes
- Result file: `planning/readme-and-review/lightweight-dev_task_6_agents_md.md`.
- Never run destructive git commands.
