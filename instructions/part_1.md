# Part 1 — Global Localization (Perfect Situation)

## Goal

Show that the relative pose between two robots can be easily computed when both are tracked in a shared reference frame

## Learning Objectives

- Derive relative pose between two robots from their known global poses in a shared reference frame (ideal case).
- Analyze how error propagation from sensor noise in global poses affects the computed relative pose.

---

## Implementation Specs

### 0. Exercise

- Build a visualization that shows the relative pose of two robots in a shared reference frame from their known global poses.
  - Assume no noise; work in 2D. Plot both trajectories, and periodically draw an arrow between them showing the expected relative pose (position + orientation).
- Repeat the same exercise with noisy global poses and show how the computed relative pose changes.

### 1. Shared `../src` Components Required

| Module | Purpose | Key Functions / Classes to Export |
|--------|---------|-----------------------------------|
| **`config.py`** | Central constants shared across all parts | `DEFAULT_TIME`, `DEFAULT_DT`, `DEFAULT_SIG_*` placeholders, part-specific config classes |
| **`trajectories.py`** | Trajectory creation and management wrappers | `create_gt_trajectory()`, `build_two_robot_scenario()` (two separate trajectories for later parts) |
| **`sensors.py`** | Sensor setup helpers (noise-free to start) | `setup_noise_free_sensors(robot, sensors_config)`, `attach_uwb_pair_to_robots(rbt_a, rbt_b)` |
| **`plots.py`** | One-call visualization | `plot_global_trajectories(gt_traj, filter_trajs, labels=None, save=None)`, `plot_position_errors(errors, labels=None, title=None, save=None)` |

### 2. Which `arp_utility_code` Classes to Use

- `Simulation.Trajectories_Generator.trajectory_generator()` — generates the base ground-truth trajectory
- `Simulation.Robot.RobotSimulation` — creates two robot objects (robot A for the filter, robot B for relative distance generation)
- `Simulation.Sensors_simulation.OdometrySensorSimulation`, `InterRobotDistanceSensorSimulation(sig_d=0.0)` — noise-free sensor simulation
- `UtilityCode.Trajectories.TrajectoryComparison` — compare filter output against ground truth
- `SGal3_filters.SGal3_EKF.SGal3EKF` (primary) and/or `SGal3_filters.SGal3_UKF.SGal3UKF.position_filter()` (alternative)
- `Manifpy_helper.sgal3_from_se3_matrix()`, `sgal3tangent_from_accel()`

### 3. Notebook Structure

| Cell Number | Content | Code Pattern |
|-------------|---------|--------------|
| Setup / Imports | Standardize all imports via `config.py` paths for reproducibility | `from config import DEFAULT_TIME, DEFAULT_DT, DEFAULT_SIG_V, DEFAULT_SIG_UWB`; seed setting |
| Parameter Block | User defines trajectory time, dt, noise sigma (zero for ideal case), filter hyperparameters | Variables at top: `TIME = 100`, `DT = 0.1`, `SIG_V = 0.0`, `SIG_UWB = 0.0` |
| Build Scenario | Calls `src/trajectories.py` to create trajectories and `src/sensors.py` to attach sensors | `scenario = build_two_robot_scenario(TIME, DT, SIG_V, SIG_UWB)` — returns dict with `robot_a, robot_b, traj_A, traj_B, sensors` |
| Simulation Run | Single call: `results = run_simulation(scenario)` — steps through all timestamps | One line returning dict |
| Filter Execution | Single call: `filter_result = run_filter(results, config)` — feeds sensor measurements into EKF/UKF | One line filtering |
| Visualization | **One plotting call**: `plots.plot_global_trajectories(...)` and `plots.plot_position_errors(...)` | Zero per-cell boilerplate after params |
| Analysis (text cell) | Interpretation of results; transition commentary toward Parts 2–5 | Markdown only |

### 4. Data Flow

```
trajectory_generator()      → Trajectory (SGal3 manifold)
RobotSimulation.do_step()   → noisy sensor measurements (zero σ here)
filter.predict(tangent,Q)   → propagate nominal state + covariance
filter.update_position(R)   → inject UWB/odometry position
TrajectoryComparison        → GT error array comparison
plots.plot_global_trajectories() + plots.plot_position_errors()
```

### 5. Expected Output / Verification

- **Plot 1**: Ground-truth trajectory (true pose), robot A estimated pose, and robot B trajectory — all in one overlay plot. Ideal case: perfect overlap.
- **Plot 2**: Position error over time — should reach zero quickly, stay at machine precision.
- **Unit test** (`part_1_ut.py`): Verify position error norms are below a threshold (e.g., `max_error < 1e-6`).

### 6. Transition to Part 2

Include a text summary noting:
- In the perfect case, filter convergence was immediate and exact.
- With real sensors, uncertainty from noise propagates — what changes?
- This sets up GDOP analysis in Part 2 (UWB anchor geometry effects).

---

## File Placement

```
part_1.md                       ← this file
src/config.py                   # DEFAULT_TIME=100, DT=0.1, defaults placeholder
src/trajectories.py             # build_two_robot_scenario(), create_gt_trajectory()
src/sensors.py                  # setup_noise_free_sensors()
src/plots.py                    # plot_global_trajectories(), plot_position_errors()
tests/part_1_ut.py              # ideal case correctness unit test
tutorials/notebooks/part_1.ipynb ← built from this spec
```
