# Part 4 — Drift and Observability Using One UWB Beacon

## Goal

Study how a single fixed UWB observation point affects the observability of a localization system. Demonstrate that one beacon provides only partial observability (distance from beacon is observed, but bearing/angle is not) and shows how drift evolves differently depending on robot trajectory relative to the beacon's location.

## Prerequisite

- Completion of Parts 1–3: understands odometry drift, anchor corrections, and covariance envelopes.

## Learning Objectives

- Define observability in the context of single-beacon localization and show which state degrees of freedom are observable vs. unobservable (distance to beacon is observed; rotational motion around the beacon is not).
- Quantify how trajectory geometry relative to the beacon determines whether the filter can converge or drifts indefinitely.
- Show that without multiple observation points, the bearing angle and along-track position remain unobservable — a fundamental limitation of single-range sensors.

---

## Implementation Specs

### 1. Shared `../src` Components Required (new/extended from Part 3)

| Module | Extension Needed | Key Functions / Classes to Export |
|--------|------------------|-----------------------------------|
| **`config.py`** | Single-beacon configs | `BEACON_POSITIONS = {'center': (0,0), 'corner': (-5,-5), 'perimeter': (10,0)}`; per-geometry observability annotations |
| **`trajectories.py`** | Trajectory generators that pass through/around different beacon locations | `build_beacon_scenario(beacon_pos, trajectory_type)` — produces trajectory + ground truth for comparison across geometries |
| **`sensors.py`** | Single UWB beacon distance sensor setup | `attach_single_beacon_sensor(beacon_pos, sigma_d)` — creates one `InterRobotDistanceSensorSimulation` fixed to a known anchor position instead of a mobile robot |
| **`plotters/plots.py`** | Observability visualization | `plot_observability_heatmap(state_dim, observable_regions)`, `plot_trajectory_beacon_view(beacon_pos, traj, ax=None)`, `plot_1d_distance_residuals(residuals, times)` |

### 2. Which `arp_utility_code` Classes to Use

- **`Simulation.Sensors_simulation.InterRobotDistanceSensorSimulation`** — but only one range measurement per time step (distance to the single beacon)
- **`UtilityCode.Trajectories.Trajectory`, `TrajectoryComparison`** — trajectory + error comparison, extended for partial observability analysis: compare each state dimension's error growth separately
- **`SGal3_filters.SGal3_UKF.position_filter()`** — UKF preferred for this part because nonlinearity of single-range-to-state mapping is more tractable to show conceptually. EKF acceptable as alternative with note on linearization limits.
- **`manifpy_helper.sgal3tangent_from_accel()`** — same odometry integration from IMU/VIC → tangent increment for prediction
- **`Simulation.Robot.RobotSimulation`** — one robot; ground truth also tracked

### 3. Notebook Structure

| Cell Number | Content | Code Pattern |
|-------------|---------|--------------|
| Setup / Imports | Standard seed, import boilerplate | Same pattern |
| Single-Beacon Physics Block (text+code) | Derive what a single-range measurement gives: `d = ||x - b||` where b is beacon position. Jacobian `∇h = (x-b)/||x-b||`. The rank is 1 — only the radial direction from beacon is observed; tangential directions are unobservable. Show this visually as an arrow along the range vector only. | One-shot: `J, observable_rank = compute_single_beacon_jacobian(robot_pos, beacon_pos)` |
| Parameter Block | Beacon positions, trajectory types (toward beacon, around beacon, away from), odometry drift params (lower than Part 3 since this is about observability not drift magnitude) | Variables: `BEACON_POS`, `TRAJ_AROUND_BEACON`, `TRAJ_TOWARD_BEACON` |
| Scenario Setup + Observability Analysis | Build trajectory that passes near beacon; analyze which state dimensions are observable at each time step from the Jacobian rank | `beacon_results = run_beacon_scenario(beacon_pos, traj_type)` — returns filter results + observability analysis per state dimension |
| Analyze Observation (text+code) | Show error evolution per state dimension: radial position converges, angular/along-track drifts. Discuss when the robot moves toward vs. away vs. perpendicular to beacon | Compute per-dimension error magnitudes; identify which decay (radial) and which grow (tangential) |
| Plot 1 — Trajectories with observable direction arrows | Show trajectory passing by the beacon, each time step annotated with a small arrow showing the observable direction (gradient of range function), i.e., radial only | `plots.plot_trajectory_beacon_view(beacon_pos, traj)` with arrow field overlay |
| Plot 2 — State dimension error growth | Per-dimension plot: x, y, z, vx, vy, vz position/velocity errors over time. Radial direction converges; orthogonal directions never settle | `plot_state_dimension_errors(error_by_dim, labels, times)`. Multiple geometries overlaid |
| Analysis & Transition (text cell) | Conclusion: one beacon cannot fully localize — we need either multiple anchors (Part 2/3) or relative observations between robots (Part 5) | Markdown discussion |

### 4. Data Flow

```
beacon_position + robot trajectory → InterRobotDistanceSensorSimulation(sig_d) per time step → single range measurement z_i = ||p_robot(t_i) - beacon_pos|| + noise
    → filter.predict() from odometry (always happens)
    → filter.update(z_range_measurement, hx=lambda x: ||x - beacon_pos||) — nonlinear update for UKF
    → analyze error per state dimension → identify which converge vs. which don't
```

### 5. Expected Output / Verification

- **Plot 1**: Trajectory path with small radial arrows at each time step showing only the range direction is observed (perpendicular directions = null space). Multiple geometries overlaid or sub-plotted.
- **Plot 2**: Per-state-dimension error magnitudes over time — clear separation between converging dimensions (radial to beacon) and growing/constant-error dimensions (tangential bearing).
- **Unit test** (`part_4_ut.py`): Verify that the state dimension aligned with `beacon_to_robot` vector has strictly lower post-update error than orthogonal dimensions; verify unobservability in at least one tangent direction.

### 6. Transition to Part 5

Explain that single-beacon localization cannot produce full-state convergence due to geometric non-observability — a natural motivation for moving from absolute ranging (beacons) to relative ranging (two mobile robots sharing UWB links). This is the focus of Part 5: true relative localization where robots observe each other's position.

---

## File Placement

```
part_4.md                                       ← this file
src/config.py                                   # BEACON_POSITIONS dicts, per-beacon observability annotations
src/trajectories.py                             # build_beacon_scenario(), generate_trajectories_for_beacon()
src/sensors.py                                  # attach_single_beacon_sensor(beacon_pos)
src/plotters/plots_observability.py             # compute_single_beacon_jacobian(), plot_trajectory_beacon_view(), plot_state_dimension_errors()
tutorials/notebooks/part_4.ipynb                ← built from this spec
tests/test_part_4_observability_ut.py           # radial < tangential error convergence assertion
```
