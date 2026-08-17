# Part 5 — Relative Localization

## Goal

Demonstrate full relative localization between two mobile robots using UWB inter-robot range measurements as the primary sensor. Show how two agents sharing distance observations can achieve bounded error localization without relying on any absolute position references (anchors or beacons).

## Prerequisite

- Completion of Parts 1–4: understands odometry drift, anchor-based corrections, single-beacon non-observability, and covariance envelopes.

## Learning Objectives

- Model the relative localization problem: two robots with individual odometry plus a shared UWB range measurement between them.
- Show how inter-robot distance observation constrains both robots' error states — unlike absolute ranging, relative range only determines distance, not bearing, requiring motion for observability.
- Demonstrate bounded vs. unbounded relative position error depending on robot mobility patterns (e.g., non-collinear motion is required for full relative observability).

---

## Implementation Specs

### 1. Shared `../src` Components Required (new/extended from Part 4)

| Module | Extension Needed | Key Functions / Classes to Export |
|--------|------------------|-----------------------------------|
| **`config.py`** | Relative localization configs | `RELATIVE_SENSOR_CONFIG = {'sig_d': ..., 'NLOS_bias': ...}`, relative localization parameters (e.g., two-robot covariance configuration, shared NLOS params) |
| **`trajectories.py`** | Two-robot trajectory pairs with different mobility patterns | `build_relative_scenario(mobility='non_collinear')` — returns `scenario = {robot_a, robot_b, traj_a, traj_b, uwb_pair}` with trajectories that enable full relative observability; `build_degenerate_scenario()` for collinear baseline |
| **`sensors.py`** | Mutual UWB link setup | `create_mutual_uwb_link(robot_a, name_a, robot_b, name_b, sig_d)` — pairs two `InterRobotDistanceSensorSimulation` sensors on each robot with symmetric distance observations |
| **`plotters/plots_relative.py`** | Relative-specific visualization | `plot_relative_error(position_error_a_vs_b, times, title=None)`, `plot_individual_tracks_with_gt(robot_A_state_history, robot_B_state_history, gt_A, gt_B)`, `plot_covariance_coupling(P_matrix_trace)` for joint covariance analysis |

### 2. Which `arp_utility_code` Classes to Use

- **`Simulation.Robot.RobotSimulation`** — two robots, each with their own odometry sensors and one UWB sensor (TX/RX pair across the link)
- **`Simulation.Sensors_simulation.InterRobotDistanceSensorSimulation`** — core relative range sensor; `measure_distance_to()` on RobotSimulation computes true distance then feeds into both robot's sensor measurements
- **`UtilityCode.Trajectories.Trajectory`, `TrajectoryComparison`** — each robot has its own trajectory; comparison per-agent vs. GT
- **`SGal3_filters.SGal3_EKF` and/or `SGal3_filters.SGal3_UKF.position_filter()`** — two independent filters (one per robot), coupled only through the shared UWB measurement: when robot A's range to B is measured, both filters receive a correlated update
- **`manifpy_helper.SGal3Tangent`, `sgal3tangent_from_accel()`** — per-robot odometry integration → tangent increment for prediction

### 3. Notebook Structure

| Cell Number | Content | Code Pattern |
|-------------|---------|--------------|
| Setup / Imports | Standard seed, import boilerplate | Same pattern |
| Parameter Block | Define relative localization scenario | Variables: `ROBOT_A_SIG`, `ROBOT_B_SIG`, `SIG_UWB_RANGE`, `MOBILITY_PATTERN = 'non_collinear'`/`'collinear'` |
| Scenario Setup | Build two robots with individual odometry and a mutual UWB link | `scenario = build_relative_scenario(mobility_pattern)` — returns both robots, trajectories, and the UWB pair |
| Simulation Run | Step through both robots in lockstep; at each time step: predict from odometry → measure distance between A and B → update both filters with the same range | `results = run_relative_simulation(scenario)` — single-line simulation returning dict with all histories |
| Analysis (text/code) | Show how relative position error (between agents) converges or diverges; compare non-collinear vs. collinear scenarios | Compute `delta_error = ||hat{x}_A - hat{x}_B|| - ||x_A - x_B||` — the residual between measured distance and estimated inter-robot distance |
| Plot 1 — Individual tracks | Robot A track, Robot B track, ground truth for each, in a single overlay plot per robot (or side-by-side) | `plots_individual_tracks(...)` |
| Plot 2 — Relative position error over time | The key plot: inter-robot range error = measured distance minus filter-estimated distance, plus the per-agents relative state vector. Shows convergence to zero only under non-collinear mobility | `plot_relative_error(...)` |
| Plot 3 (optional) — NLOS robustness comparison | If desired: same scenario with NLOS applied via `NLOS_Manager` on the UWB measurements; shows how filter behaves with biased range data | Additive NLOS: `results_nlos = run_with_nlos(scenario, nlos_pattern='1')` from configured NLOS patterns in `NLOS_Manager` |
| Analysis & Summary (text cell) | Key insight: non-collinear motion enables full relative observability; collinear motion leaves bearing unobservable. Connection back to Part 4's single beacon limitation — relative ranging between two movers provides better geometric diversity than fixed anchors alone | Markdown + key takeaways for tutorial conclusion |

### 4. Data Flow

```
Two robot trajectories (A, B) with odometry noise
    → RobotSimulation A + RobotSimulation B on synced time grids
    → mutual UWB link: robot_A.measure_distance_to(robot_B) at each time step
    → two filter instances (EKF or UKF per robot):
        predict from odometry integration (per-robot tangent increment)
        update both with observed range z = ||p_A - p_B|| + noise
            for each filter: H_i = ∂z/∂x_i (gradient wrt that robot's state, via chain rule through range function)
    → compare individual tracks vs. GT; compute relative position error magnitude
```

### 5. Expected Output / Verification

- **Plot 1**: Both robots' estimated trajectories overlaid with their respective ground truths — two panels or four curves on one plot.
- **Plot 2**: Primary result: relative inter-robot distance error (measured - estimated) converges to near zero over time if and only if the mobility pattern provides sufficient geometric diversity (non-collinear). Collinear case shows persistent error in the bearing direction — connecting back to Part 4's observability lesson.
- **Plot 3** (optional): NLOS impact showing degraded estimates during NLOS periods in both scenarios.
- **Unit test** (`part_5_ut.py`): Verify that relative position error converges for non-collinear scenario and diverges/stays above threshold for collinear scenario; verify mutual UWB range measurements are symmetric (both robots see the same distance value).

### 6. Tutorial Conclusion

Summarize how each part built on the previous:
1. Part 1 established baseline convergence (ideal case)
2. Part 2 showed GDOP effects for anchor-based absolute positioning
3. Part 3 revealed unbounded odometry drift without corrections
4. Part 4 demonstrated partial observability with one beacon
5. Part 5 ties it all together: relative localization with two movers provides geometric diversity that overcomes the limits shown in Parts 2–4

---

## File Placement

```
part_5.md                                       ← this file (final part)
src/config.py                                   # RELATIVE_SENSOR_CONFIG
src/trajectories.py                             # build_relative_scenario(), build_degenerate_collinear_scenario()
src/sensors.py                                  # create_mutual_uwb_link()
src/plotters/plots_relative.py                  # plot_individual_tracks_with_gt(), plot_relative_error(), plot_covariance_coupling()
tutorials/notebooks/part_5.ipynb                ← built from this spec (final notebook)
tests/test_part_5_relative_ut.py               # convergence vs non-convergence assertions across mobility patterns
```
