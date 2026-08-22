import numpy as np
import matplotlib.pyplot as plt

from manifpy import SE2

import sys
sys.path.insert(0, '/workspace/Python/Relative-localisation-tutorial/src')

from config import TrajectoryConfig, VIOConfig
from trajectory import generate_trajectory, sgal3_to_se2_poses
from vio_sensor import simulate_vio_odometry


def plot_robot_vio_trajectory(traj_config: TrajectoryConfig, vio_config: VIOConfig):
    """Generate and visualize a single robot's perfect vs VIO-trajectory with drift.
    
    Args:
        traj_config: TrajectoryConfig with duration, timestep, seed
        vio_config: VIOConfig with sig_v and sig_w noise parameters
        
    Returns:
        poses_perfect: (N, 3) array of perfect robot poses [x, y, theta]
        poses_noisy: (N, 3) array of VIO-estimated poses with drift
    """
    traj = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )
    
    poses_perfect = sgal3_to_se2_poses(traj.X_RRi)
    poses_noisy, _ = simulate_vio_odometry(traj, vio_config)
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(poses_perfect[:, 0], poses_perfect[:, 1], 'b-', label='Perfect', alpha=0.7)
    ax.plot(poses_noisy[:, 0], poses_noisy[:, 1], 'r--', label='VIO Estimate', alpha=0.7)
    
    arrow_len = 0.3
    step = max(1, len(poses_perfect) // 20)
    
    for i in range(0, len(poses_perfect), step):
        x_i, y_i, theta_i = poses_perfect[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='b', ec='b')
    
    for i in range(0, len(poses_noisy), step):
        x_i, y_i, theta_i = poses_noisy[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='r', ec='r')
    
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Robot 1: Perfect vs VIO Estimated Trajectory with Drift')
    
    plt.tight_layout()
    plt.show()
    
    return poses_perfect, poses_noisy


def plot_two_robots_vio(
    traj_config1: TrajectoryConfig,
    vio_config1: VIOConfig,
    traj_config2: TrajectoryConfig,
    vio_config2: VIOConfig
):
    """Generate and visualize two robots' perfect vs VIO-trajectories with drift.
    
    Args:
        traj_config1, vio_config1: Configs for robot 1
        traj_config2, vio_config2: Configs for robot 2
        
    Returns:
        poses1_perfect, poses1_noisy: Robot 1 perfect and noisy trajectories
        poses2_perfect, poses2_noisy: Robot 2 perfect and noisy trajectories
    """
    traj1 = generate_trajectory(
        duration=traj_config1.duration,
        timestep=traj_config1.timestep,
        seed=traj_config1.seed
    )
    
    traj2 = generate_trajectory(
        duration=traj_config2.duration,
        timestep=traj_config2.timestep,
        seed=traj_config2.seed
    )
    
    poses1_perfect = sgal3_to_se2_poses(traj1.X_RRi)
    poses1_noisy, _ = simulate_vio_odometry(traj1, vio_config1)
    
    poses2_perfect = sgal3_to_se2_poses(traj2.X_RRi)
    poses2_noisy, _ = simulate_vio_odometry(traj2, vio_config2)
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(poses1_perfect[:, 0], poses1_perfect[:, 1], 'b-', label='Robot 1 Perfect', alpha=0.7)
    ax.plot(poses2_perfect[:, 0], poses2_perfect[:, 1], 'g-', label='Robot 2 Perfect', alpha=0.7)
    ax.plot(poses1_noisy[:, 0], poses1_noisy[:, 1], 'b--', label='Robot 1 VIO', alpha=0.6)
    ax.plot(poses2_noisy[:, 0], poses2_noisy[:, 1], 'g--', label='Robot 2 VIO', alpha=0.6)
    
    arrow_len = 0.3
    step = max(1, len(poses1_perfect) // 20)
    
    for i in range(0, len(poses1_noisy), step):
        x_i, y_i, theta_i = poses1_noisy[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='b', ec='b', alpha=0.6)
    
    for i in range(0, len(poses2_noisy), step):
        x_i, y_i, theta_i = poses2_noisy[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='g', ec='g', alpha=0.6)
    
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Two Robots: Perfect vs VIO Estimated Trajectories with Drift')
    
    plt.tight_layout()
    plt.show()
    
    return poses1_perfect, poses1_noisy, poses2_perfect, poses2_noisy
