# lightweight-dev Output — Task: Write README

**Task:** #4    **Session:** readme-and-review    **Date:** 2026-11-22

## Summary
I rewrote `README.md` in full; it is now 204 lines. It has these sections, in order:
- A title and an intro.
- A "Tutorial overview" with one subsection per part. Each subsection has a description, the parameters to play with and the code modules.
- Requirements, Installation, Running the notebooks, Regenerating the figures, Running the tests and Project structure.
- Known limitations, Acknowledgements and License.

The README embeds 8 figures: part_1_fig_3, part_2_fig_3, part_2_fig_5, part_3_fig_1, part_3_fig_4 and part_4_fig_1/2/3. Each caption is in italics. The Part 4 captions name the scenario, (a), (b) or (c), and the setup, because those plots have no title.

The README quotes no numeric RMSE values and gives no Part 3 angle errors in degrees. Its "Known limitations" section lists three points:
- The Part 3 angle-error plots are labelled "deg" but show radians.
- The setup is tested only on Linux with Python 3.11.
- `manifpy` and `upf_rte` come from the local `libs/` folder, installed by `install.sh`, not from PyPI.

The clone URL comes from the repo's `origin` remote, converted to HTTPS. I made no code or notebook changes.

## Files
| File | Action | Why |
|---|---|---|
| `README.md` | modify (full rewrite) | Project documentation with figures. |
| `planning/readme-and-review/lightweight-dev_task_4_readme.md` | create | This report. |

## Build / Tests
- Image-path check (every `docs/images/*.png` reference in the README, tested with `test -f`): PASS, 8/8 exist.
- `grep -ci ekf README.md` returns 0: PASS.
- Check for numeric RMSE values or degree values: PASS, none found.
- Compared the install steps with `install.sh` and `pyproject.toml`: they match. The venv is `.venv` and created with `python3.11`. The script runs `git submodule update --init libs/manif`, builds manif, installs arp_utility_code from its tarball, converts UPF_RPE into `upf_rte`, and installs jupyter, pytest and ipykernel.
- pytest: not run, because no code changed.

## Acceptance Verification
| # | Criterion (verbatim) | Result | Notes |
|---|---|---|---|
| 1. | Every image path referenced in `README.md` exists on disk (check with a small script/grep). | PASS | 8/8 |
| 2. | All four parts are described, and Part 4 says UPF (no "EKF" anywhere in the README). | PASS | 0 EKF matches |
| 3. | Install steps match `install.sh` (venv name `.venv`, Python 3.11, submodule step). | PASS | |
| 4. | No statement contradicts the code facts above; no invented grant numbers, URLs or results. | PASS | The only URL is the clone URL, taken from `git remote`. The README quotes no result numbers. |
| 5. | Commit `OC: Write README with tutorial overview, setup and notebook figures` (with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer), pushed. | PASS | This report is included in that commit. |

## Blockers
None.

## Next Steps
- Fix the Part 3 angle-axis unit, either by changing the label or by converting the values with `np.rad2deg`. Then remove that point from "Known limitations".
- `tests/` has no `manual_plot` tests at the moment. The README mentions only that the marker is defined.
