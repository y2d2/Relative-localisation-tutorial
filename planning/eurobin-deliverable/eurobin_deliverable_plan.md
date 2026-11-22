# [Deliverable] euROBIN — Relative Localisation Tutorial

**Purpose:** Short (~100-word) euROBIN deliverable text describing the relative localisation tutorial delivered on 28/08/2026.

## Objective
Provide a concise, reusable description of the Relative Localisation Tutorial for euROBIN reporting. The text must state what the tutorial covers, that it was delivered as a hands-on session to eight participants on 28 August 2026, and that the material may still evolve. Success = the text below can be pasted into a euROBIN report without further edits. This is an interim version; it may be replaced by a longer or final deliverable later.

## Deliverable Text (final, ~100 words)

> **Relative Localisation Tutorial**
>
> Within euROBIN, VUB developed an interactive, open-source Python tutorial on relative localisation for multi-robot systems, delivered on 28 August 2026 to a group of eight participants. In four progressive Jupyter notebooks, participants explored how global pose noise propagates into relative pose estimates, how UWB anchor geometry (GDOP) and anchor mis-calibration affect localisation accuracy, how odometry/SLAM drift accumulates over time, and how fusing odometry with a single UWB beacon in an Unscented Particle Filter bounds drift while exposing observability limits. Simulations are reproducible and parameterised for hands-on experimentation. The material will be further refined in future iterations.

## Context
- The text reflects the repository state reviewed on 2026-11-03 (four notebooks `part_1`–`part_4`, shared `src/` package, `upf_rte` and `ARP_utility_code` libraries) — own analysis of the codebase.
- Delivery facts (date 28/08, eight participants, not necessarily final version) were provided by the user.

## Functional Tasks

### Task 1: Provide the deliverable text
- **Goal:** A ~100-word paragraph summarising the tutorial and its delivery.
- **Why this approach:**
  - **[Decision: Length]** Value = ~100 words because the user confirmed a short abstract is sufficient for now.
  - **[Decision: Part 4 method]** Value = describe as Unscented Particle Filter because I evaluated the code: Part 4 runs the UPF (`UPF_2D`); the EKF module is not used by any notebook.
  - **[Decision: Interim status]** Value = state that material will be refined because the user indicated this may not be the final version.
- **Acceptance criteria:** The text mentions the date, the eight participants, all four tutorial topics, and contains no claims contradicted by the code.

## Alternatives Considered
1. **~300-word descriptive version** — Rejected; user preferred ~100 words for now.
2. **Report with Status / Limitations / Future work sections** — Deferred until a final deliverable is required.

## Notes
- **[Assumption]** Year of delivery is 2026; partner named as VUB (user affiliation). Adjust if the deliverable needs a different partner name or grant reference.
- **[Risk]** Code-review findings not yet addressed (could be reflected in a future version):
  1. Part 4 notebook intro says "EKF" but the code uses the UPF; `ekf.py` is unused.
  2. `pyproject.toml` says "5 progressive parts"; there are 4.
  3. `README.md` contains only a title (no install/usage instructions).
  4. `upf_rte` is imported but not declared in project dependencies.
  5. Mixed `from src.x` / `from x` imports in Part 4 functions (path-fragile).
  6. Part 4 notebook calls pass no `seed` → non-reproducible runs.
  7. ~40 lines of setup duplicated across the three Part 4 functions.
  8. Duplicate `plot_anchor_positions` / `plot_gdop_heatmap` in `uwb.py` and `plotting.py`.
  9. Tests cover only the UPF; Parts 1–3 are untested.
  10. Typos (`perfec_poses`, `traj_configig`, "en", "viceversa", "fo", "usefull").
