# Part 2 — UWB Anchor Global Localization (GDOP & Incorrect Anchor Placement)

## Goal

Show how anchor geometry determines localization accuracy through GDOP (Geometric Dilution of Precision). Compare ideal anchor placement against degraded configurations to quantify the impact of UWB infrastructure layout.

## Prerequisite

- Completion of Part 1: assumes a working pipeline for trajectory generation, sensor noise injection, EKF/UKF filtering, and error plotting.

## Learning Objectives

- Define GDOP analytically from the UWB measurement Jacobian and understand its physical meaning (error amplification factor).
- Show how anchor geometry affects the covariance growth rate in a filter, not just theoretical GDOP.
- Demonstrate concrete placement errors: anchors at non-orthogonal angles, coplanar anchors, missing-range scenarios, and clustered anchors.

---

## Implementation Specs

### 1. Shared `../src` Components Required (new/extended from Part 1)

| Module | Extension Needed | Key Functions / Classes to Export |
|--------|------------------|-----------------------------------|
| **`config.py`** | GDOP anchor configs | `ANCHOR_CONFIGS = {ideal, degraded_clustered, coplanar, asymmetric}`; each config = list of `(x,y,z)` positions + `SIG_ANCHOR_*` defaults |
| **`trajectories.py`** | Anchor-aware trajectory setup helper | `build_anchor_global_scenario(anchor_config)` — returns scenario dict with robots, ground-truth pos, anchor array |
| **`sensors.py`** | Anchored UWB measurements | `set_up_anchored_uwb_sensors(anchors, position_function, sigma)` — converts fixed-anchor positions into measured ranges; also `compute_noisy_anchor_range(true_dist, sigma)` |
| **`plotters/plots.py`** | GDOP visualization | `plot_gdop_scatter(dilution_values, anchor_configs, title=None)`, `plot_anchor_geometry(anchors, robot_pos, ax=None)`, `gdop_from_jacobian(J)` |

### 2. Which `arp_utility_code` Classes to Use

- **`Simulation.Sensors_simulation.InterRobotDistanceSensorSimulation`** — adapt for anchor-to-robot ranges instead of inter-robot (noise model same: Gaussian on raw distance)
- **`UtilityCode.Trajectories.Trajectory`, `TrajectoryComparison`** — trajectory + error comparison as in part 1
- **`SGal3_filters.SGal3_EKF`** or **`SGal3_UKF.position_filter()`** — filter receiving anchor position measurements instead of full odometry; R matrix now maps per-anchor measurement noise to the filter update
- **`manifpy.SGal3`, `manifpy.SE3`** for computing anchor-to-robot range Jacobians: if robot pose = `[px,py,pz]` and anchor i at position `ai = [axi,ayi,azi]`, then `hi(x) = ||x - ai||`, gradient `∇hi = (x - ai) / ||x - ai||`. Stack all `∇hi` into `H` Jacobian for H-inversion → GDOP = trace(H^T R^-1 H)
- **`manifpy_helper`** conversions as in part 1

### 3. Notebook Structure

| Cell Number | Content | Code Pattern |
|-------------|---------|--------------|
| Setup / Imports | Standard import boilerplate | Same seed pattern; set `np.random.seed(PART_2_SEED)` |
| GDOP Definition Block (text+code) | Define GDOP formula inline, compute from measurement Jacobian: `H_i = ∇h_i(x_true)`, `GDOP = sqrt(trace((H^T R^{-1} H)^{-1}))`. Show it as a function of anchor geometry. | One-shot helper: `gdop_value, H_jacobian = compute_gdop(robot_pos, anchors, sig_a)` that returns scalar GDOP value + Jacobian |
| Parameter Block | Define anchor configs and noise | Variables: `ANCHOR_IDEAL`, `ANCHOR_CLUSTERED`, `ANCHOR_COPLANAR`; `SIG_ANCHOR_POS = 0.02` (anchor position measurement noise) |
| Compare Geometries | For each anchor config, run filter on same trajectory with same ground truth; collect GDOP values and post-filter covariance traces | Loop over configs → one call per config: `cfg_result = run_gdop_experiment(anchor_config)` |
| Plot 1 — Anchor geometry visualizations | Show where anchors are placed relative to the trajectory path (one subplot per config) | Single plotting line |
| Plot 2 — GDOP vs. trajectory position along path | Animate or sub-layout: GDOP value at time steps for each config overlayed in one plot (error amplification over space) | Single plotting line |
| Plot 3 — Filtering error comparison | Position + orientation errors overlay per geometry on same axes (ideal, clustered, coplanar) | `plots.plot_gdop_results(results_dict)` returning multi-panel figure |
| Analysis (text cell) | Interpret each GDOP result; relate GDOP scalar to observed filter covariance trace and RMS error | Markdown commentary → transition to Part 3 |

### 4. Data Flow

```
anchor_config (list of positions) + robot true pos
    → Jacobian H per time step (∇h_i from geometry)
    → GDOP = sqrt(trace((H^T R^-1 H)^{-1}))  [analytical]
    → same trajectory run with noisy anchor range measurements via InterRobotDistanceSensorSimulation(sig_d=SIG_ANCHOR_POS)
    → filter.predict() + filter.update_position(anchors_observed_positions)
    → TrajectoryComparison vs. GT
```

### 5. Expected Output / Verification

- **Plot 1**: Four scenes, one per anchor config (ideal/orthogonal square around trajectory path, coplanar anchors on a circle, asymmetric L-shaped, clustered nearby). Robot true pos shown as a red dot.
- **Plot 2**: GDOP value (y-axis) vs. robot position along the trajectory or vs. time step — each geometry has its own curve. Ideal config should have lowest, stable GDOP. Clustered/coplanar should spike.
- **Plot 3**: Overlay of estimated trajectories per config vs. ground truth, plus error bar traces showing covariance growth rate correlation with GDOP.
- **Unit test** (`part_2_ut.py`): Verify ideal config GDOP is strictly lower than degraded configs for the same robot position and noise levels.

### 6. Transition to Part 3

Explain that even with fixed, well-placed anchors in static/idealized settings, real-world localization must handle moving sensors and accumulated drift from odometry (SLAM pipeline) — what happens when odometry errors dominate?

---

## File Placement

```
part_2.md                                       ← this file
src/config.py                                   # ANCHOR_CONFIGS dict with all anchor placements + sig defaults
src/plotters/plots_gdop.py                      # compute_gdop(), plot_anchor_geometry(), plot_gdop_results()
tutorials/notebooks/part_2.ipynb                ← built from this spec
tests/test_part_2_gdop_ut.py                    # ideal GDOP < degraded GDOP assertion
```
