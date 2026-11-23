import numpy as np
import matplotlib.pyplot as plt

from manifpy import SE2

import sys

from part1_functions import get_pose
from config import TrajectoryConfig, UWBNoiseConfig
from trajectory import generate_trajectory, sgal3_to_se2_poses
from uwb import (
    simulate_uwb_ranges,
    estimate_poses_from_ranges,
    compute_gdop_at_point,
    generate_gdop_heatmap,
    plot_anchor_positions,
    plot_gdop_heatmap
)

def plot_robot_with_anchors(traj_config, uwb_noise_config, anchor_positions):
    """Generate robot trajectory and visualize with UWB anchors.
    
    Args:
        traj_config: TrajectoryConfig with duration, timestep, seed
        anchor_positions: (N, 2) array of anchor [x, y] positions
        
    Returns:
        poses_robot1: (N, 3) array of perfect robot poses [x, y, theta]
    """
    traj_robot1 = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )
    
    poses_robot1 = sgal3_to_se2_poses(traj_robot1.X_RRi)
    ranges_robot1 = simulate_uwb_ranges(
        poses_robot1,
        anchor_positions,
        uwb_noise_config
    )

    poses_estimated, residuals = estimate_poses_from_ranges(ranges_robot1, anchor_positions)
    error = poses_robot1 - poses_estimated
    RSE_pose = np.linalg.norm(error[:, :2], axis=1)

    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(poses_robot1[:, 0], poses_robot1[:, 1], 'b-', label='Robot 1', alpha=0.7)

    ax.plot(poses_estimated[:, 0], poses_estimated[:, 1], 'r:',
            label='Estimated positions', markersize=4, alpha=0.8)


    plot_anchor_positions(ax, anchor_positions, legend_label='UWB Anchors')
    
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Robot 1 Perfect Trajectory with UWB Anchors')
    
    plt.tight_layout()
    plt.show()

    fig, ax3 = plt.subplots(1, 1, figsize=(8  , 4))

    RMSE= np.mean(RSE_pose)
    ax3.plot(RSE_pose, 'r.', label=f'RSE of Position Estimate RMSE={RMSE:.2f} m', alpha=1.0)
    ax3.legend()
    plt.show()

    
    return poses_robot1

def run_gdop_analysis(traj_config, anchor_positions, uwb_noise_config):

    traj_robot1 = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )

    poses_robot1 = sgal3_to_se2_poses(traj_robot1.X_RRi)

    x_min = anchor_positions[:, 0].min() - 10
    x_max = anchor_positions[:, 0].max() + 10
    y_min = anchor_positions[:, 1].min() - 10
    y_max = anchor_positions[:, 1].max() + 10
    X, Y, gdop_grid = generate_gdop_heatmap(
        anchor_positions,
        x_range=(x_min, x_max),
        y_range=(y_min, y_max),
        resolution=100
    )


    gdop_along_trajectory = np.array([
        compute_gdop_at_point(poses_robot1[i, :2], anchor_positions)
        for i in range(len(poses_robot1))
    ])

    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    plot_gdop_heatmap(ax, X, Y, gdop_grid, anchor_positions=anchor_positions)
    ax.plot(poses_robot1[:, 0], poses_robot1[:, 1], 'w-',
            label='Robot trajectory', linewidth=2, alpha=0.8)
    ax.legend()
    ax.set_title('GDOP Heatmap with Robot Trajectory Overlay')

    plt.tight_layout()
    plt.show()

    fig, ax = plt.subplots(1, 2, figsize=(14, 5))

    t = np.arange(len(gdop_along_trajectory)) * uwb_noise_config.sig_range if hasattr(uwb_noise_config,
                                                                                        'sigma_range') else np.arange(
        len(gdop_along_trajectory)) * 0.1

    ax[0].plot(t, gdop_along_trajectory, 'b-', alpha=0.7)
    ax[0].set_xlabel('Time (s)')
    ax[0].set_ylabel('GDOP')
    ax[0].set_title('GDOP Along Trajectory')
    ax[0].grid(True, alpha=0.3)

    est_position_error = gdop_along_trajectory * uwb_noise_config.sig_range
    ax[1].plot(t, est_position_error, 'r-', alpha=0.7)
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Estimated position error (m)')
    ax[1].set_title('Position Error Estimate from GDOP + Range Noise')
    ax[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

def run_uwb_analysis(poses_robot1, anchor_positions, uwb_noise_config):
    """Run full UWB localization analysis including GDOP evaluation.
    
    Args:
        poses_robot1: (N, 3) array of perfect robot poses [x, y, theta]
        anchor_positions: (N, 2) array of anchor [x, y] positions
        uwb_noise_config: UWBNoiseConfig with sigma_range
        
    Returns:
        results: dict containing:
            - 'poses_estimated': estimated poses from trilateration
            - 'gdop_along_trajectory': GDOP values at each trajectory point
            - 'X, Y, gdop_grid': heatmap grid data
            - 'residuals': trilateration residuals
    """
    ranges_robot1 = simulate_uwb_ranges(
        poses_robot1,
        anchor_positions,
        uwb_noise_config
    )
    
    poses_estimated, residuals = estimate_poses_from_ranges(ranges_robot1, anchor_positions)

    

    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(poses_robot1[:, 0], poses_robot1[:, 1], 'b-', label='True trajectory', alpha=0.7)
    

    ax.plot(poses_estimated[:, 0], poses_estimated[:, 1], 'r:',
            label='Estimated positions', markersize=4, alpha=0.8)
    
    plot_anchor_positions(ax, anchor_positions, legend_label='UWB Anchors')
    
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Robot Trajectory and UWB Position Estimates')
    
    plt.tight_layout()
    plt.show()
    

    
    # results = {
    #     'poses_estimated': poses_estimated,
    #     'gdop_along_trajectory': gdop_along_trajectory,
    #     'X': X,
    #     'Y': Y,
    #     'gdop_grid': gdop_grid,
    #     'residuals': residuals
    # }
    #
    # return results


def analyze_anchor_miscalibration(traj_config, anchor_positions, uwb_noise_config, sigma_uwb_anchor):
    """Analyze the effect of anchor position mis-calibration on pose estimation.
    
    Args:
        poses_robot1: (N, 3) array of perfect robot poses [x, y, theta]
        ranges: (N, M) array of range measurements
        anchor_positions: (N, 2) array of true anchor positions
        sigma_uwb_anchor: standard deviation of the anchor position mis-calibration
        uwb_noise_config: UWBNoiseConfig for noise parameters
        
    Returns:
        None (displays plots and prints summary statistics)
    """
    traj_robot1 = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )

    poses_robot1 = sgal3_to_se2_poses(traj_robot1.X_RRi)
    anchor_offset = np.random.rand(*np.shape(anchor_positions)) * sigma_uwb_anchor
    anchor_positions_miscalibrated = anchor_positions + anchor_offset
    ranges = simulate_uwb_ranges(
        poses_robot1,
        anchor_positions,
        uwb_noise_config
    )


    poses_estimated, residuals = estimate_poses_from_ranges(ranges, anchor_positions)
    
    poses_estimated_miscal, residuals_miscal = estimate_poses_from_ranges(
        ranges,
        anchor_positions_miscalibrated
    )
    
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    
    ax[0].plot(poses_robot1[:, 0], poses_robot1[:, 1], 'k-', 
               label='True trajectory', alpha=0.7)
    ax[0].plot(poses_estimated[:, 0], poses_estimated[:, 1], 'g-', 
               label='Estimate (perfect anchors)', alpha=0.7)
    ax[0].plot(poses_estimated_miscal[:, 0], poses_estimated_miscal[:, 1], 'r--', 
               label='Estimate (mis-calibrated anchors)', alpha=0.7)
    
    plot_anchor_positions(ax[0], anchor_positions, legend_label='True Anchors')
    ax[0].scatter(anchor_positions_miscalibrated[:, 0], anchor_positions_miscalibrated[:, 1],
                  c='orange', marker='x', s=150, label='Mis-calibrated anchors')
    
    ax[0].set_xlabel('x (m)')
    ax[0].set_ylabel('y (m)')
    ax[0].axis('equal')
    ax[0].legend()
    ax[0].set_title('Position Estimates: Perfect vs Mis-calibrated Anchors')
    
    t = np.arange(len(poses_robot1)) * 0.1
    pos_error_perfect = np.sqrt((poses_estimated[:, 0] - poses_robot1[:, 0])**2 + 
                                (poses_estimated[:, 1] - poses_robot1[:, 1])**2)
    pos_error_miscal = np.sqrt((poses_estimated_miscal[:, 0] - poses_robot1[:, 0])**2 +
                               (poses_estimated_miscal[:, 1] - poses_robot1[:, 1])**2)
    
    ax[1].plot(t, pos_error_perfect, 'g-', label=f'Perfect anchors - MRSE: {np.mean(pos_error_perfect):.2f} m', alpha=1.0)
    ax[1].plot(t, pos_error_miscal, 'r--', label=f'Mis-calibrated anchors - MRSE: {np.mean(pos_error_miscal):.2f} m', alpha=1.0)
    
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Position error (m)')
    ax[1].legend()
    ax[1].set_title('Position Estimation Error Over Time')
    ax[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
