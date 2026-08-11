# Part 3 — SLAM Global Localization: Accumulation of Uncertainty Through Drift

## Goal

Demonstrate how odometry-based localization (simulated SLAM or pose-graph style tracking) accumulates error over time due to integration drift. Show that even with perfect UWB anchor measurements, unbounded odometry drift degrades overall accuracy unless corrected by absolute anchors.

## Prerequisite

- Completion of Parts 1 & 2: assumes familiarity with the full simulation-filter pipeline and GDOP concepts from anchor geometry.

## Learning Objectives

- Quantify odometry drift as an unbounded error source that grows with trajectory length (not just measurement noise).
- Show how SLAM-style global localization works: the robot uses odometry between anchor sightings to bridge gaps, creating a covariance envelope that grows between corrections and contracts at observation moments.
- Contrast pure odometry vs. odom+UWB-corrected tracking over long trajectories.

---

## Implementation Specs

### 1. Shared `../src` Components Required (new/extended from Part 2)

| Module | Extension Needed | Key Functions / Classes to Export |
|--------|------------------|-----------------------------------|
| **`config.py`** | Drift parameters | `DEFAULT_ODOM_DRIFT_PARAMS = {'sig_v_odom': ..., 'sig_w_odom': ...}`, anchor sight frequency parameter (`SECONDS_BETWEEN_SIGHTS`) |
| **`trajectories.py`** | Long-duration trajectory + drift scenario builder | `build_slam_scenario(drift_intensity, long_trajectory=True)` — returns dict with odometry-only path, odometry+anchor-corrected path |
| **`sensors.py`** | Odometry-injection helper that applies realistic sensor noise to odometry readings (no UWB yet) | `apply_odometry_noise(sig_w, sig_a, tau_ba, tau_bw)` — creates sensors with configurable drift parameters for long simulations |
| **`plotters/plots.py`** | Drift visualization helpers | `plot_drift_comparison(odom_only_traj, odom_anchor_traj, gt_traj)`, `plot_covariance_envelope(cov_trace_history, times)`, `plot_corrections_marked(correction_times, traj)` |

### 2. Which `arp_utility_code` Classes to Use

- **`Simulation.Sensors_simulation.ImuSimulation`, `VioSimulation`** — primary drift mechanisms (IMU integration error + VIO velocity error accumulate over time in odometry)
- **`Simulation.Robot.RobotSimulation`** — two robot copies: one with pure odometry sensors, one with odometry + occasional anchor-corrected UWB ranges
- **`UtilityCode.Trajectories.Trajectory`, `TrajectoryComparison`** — trajectory storage and comparison; extended for drift envelope visualization
- **`Simulation.Sensors_simulation.InterRobotDistanceSensorSimulation`** — but used here to represent intermittent "anchor fixes" (not continuous) at known times
- **`SGal3_filters` filters** — propagate continuously from IMU/VIO odometry, update only at anchor sighting times
- **`manifpy_helper.sgal3tangent_from_accel()`** — converts raw IMU accelerations to tangent increments for EKF/UKF prediction steps

### 3. Notebook Structure

| Cell Number | Content | Code Pattern |
|-------------|---------|--------------|
| Setup / Imports | Standard seed, import boilerplate | Same pattern as Part 1 |
| Parameter Block | Define odometry noise parameters and trajectory length | `ODOM_DRIFT = {'sig_v': 0.05, 'sig_w': 0.02, 'tau_ba': 1e-4, 'tau_bw': 1e-4}`, `LONG_TIME = 300` (longer than Part 1), `ANCHOR_SIGHT_INTERVAL = 30` |
| Scenario Setup | Build odometry-only and odom+anchor robot copies on same trajectory | Both robots step through identical ground truth; one gets anchor updates, one doesn't |
| Simulation Run | One call to run both paths: `odom_results, corrected_results = run_drift_comparison(...)` | Single-line simulation of parallel tracking |
| Analyze Drift (text + code) | Show how error grows quadratically/linearly with time for pure odometry; anchor corrections cause periodic re-centering | Compute RMS drift at each timestamp, plot as function of elapsed time |
| Plot 1 — Comparison overlay | Pure odometry track vs. corrected vs. ground truth — show divergence over time | `plots.plot_drift_comparison(...)` |
| Plot 2 — Covariance envelope or position error | Y-axis: position error magnitude. X-axis: time. Shows periodic drop at each anchor correction and continuous growth between corrections | `plots.plot_covariance_envelope(cov_trace, times)` with marker lines for correction events |
| Analysis & Transition (text cell) | Discussion of what "SLAM global localization" means in this context; how drift dominates when anchors become sparse → sets up Part 4 with a single beacon | Markdown commentary |

### 4. Data Flow

```
long trajectory (300+ seconds) + odom_drift_params
    → two parallel RobotSimulation chains:
        path A: pure OdometrySensor (IMU/VIO noise, no absolute corrections)
        path B: same odometry + anchor sightings at t = 0, INTERVAL, 2*INTERVAL, ...
            each sighting adds measurement to InterRobotDistanceSensor at known anchor positions
    → filter propagates continuously from odometry on both paths
    → filter updates (position injection) only at anchor sighting times on path B
    → error growth comparison over time → visualization as divergence curves + covariance envelope
```

### 5. Expected Output / Verification

- **Plot 1**: Overlaid trajectories showing pure odometry diverging away from ground truth while the odom+anchor path tracks much closer but with visible "sawtooth"-like corrections (periodic drops in error).
- **Plot 2**: Position RMSE or RMS error vs. time — linear/quadratic growth for odom-only, bounded/oscillating for corrected path. Error envelope visualization showing covariance trace between correction events.
- **Unit test** (`part_3_ut.py`): Verify that corrected track has lower max position error than pure odometry over the full trajectory duration; verify periodic error drops coincide with expected anchor sighting times.

### 6. Transition to Part 4

Highlight that multiple anchor sightings dramatically improve tracking, but in sparse-sensor environments (e.g., only UWB beacon infrastructure available), how does a single fixed observation point change the observability dynamics?

---

## File Placement

```
part_3.md                                       ← this file
src/config.py                                   # ODOMETRY_DRIFT_PARAMS, ANCHOR_SIGHT_INTERVAL configs
src/trajectories.py                             # build_slam_scenario(), set_long_trajectory()
src/sensors.py                                  # attach_odometry_only(), attach_intermittent_anchor_fixes()
src/plotters/plots_drift.py                     # plot_drift_comparison(), plot_covariance_envelope(), plot_corrections_marked()
tutorials/notebooks/part_3.ipynb                ← built from this spec
tests/test_part_3_drift_ut.py                   # corrected < odom-only error assertion over full trajectory
```
