"""Part 3 - SLAM Global Localization: Functions for VIO odometry drift demonstration."""

import numpy as np
import matplotlib.pyplot as plt

from manifpy import SE2

from config import TrajectoryConfig, VIOConfig
from trajectory import generate_trajectory, sgal3_to_se2_poses
from vio_sensor import simulate_vio_odometry
from relative_pose import compute_relative_poses


def plot_single_robot_vio(traj_config: TrajectoryConfig, 
                          vio_config: VIOConfig,
                          arrow_len: float = 0.3) -> tuple[np.ndarray, np.ndarray]:
    """Plot single robot trajectory with perfect and VIO-estimated poses.
    
    Args:
        traj_config: Trajectory configuration
        vio_config: VIO sensor configuration (velocity/angular velocity noise)
        arrow_len: Length of orientation arrows
        
    Returns:
       poses_perfect: (N, 3) array of perfect ground truth poses [x, y, theta]
        poses_noisy: (N, 3) array of VIO-estimated noisy poses [x, y, theta]
    """
    traj = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )
    
    poses_perfect = sgal3_to_se2_poses(traj.X_RRi)
    poses_noisy, _ = simulate_vio_odometry(traj, vio_config)
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(poses_perfect[:, 0], poses_perfect[:, 1], 'b-', label='Robot Perfect', alpha=0.7)
    ax.plot(poses_noisy[:, 0], poses_noisy[:, 1], 'r--', label='Robot VIO Estimate', alpha=0.7)
    
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
    ax.set_title('Robot: Perfect vs VIO Estimated Trajectory with Drift')
    
    plt.tight_layout()
    plt.show()
    
    return poses_perfect, poses_noisy


def plot_two_robots_vio(
    traj_config_robot1: TrajectoryConfig,
    vio_config_robot1: VIOConfig,
    traj_config_robot2: TrajectoryConfig,
    vio_config_robot2: VIOConfig,
    arrow_len: float = 0.3
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Plot two robots with VIO odometry and compute relative pose estimation errors.
    
    Args:
        traj_config_robot1: Trajectory config for robot 1 (perfect reference)
        vio_config_robot1: VIO config for robot 1
        traj_config_robot2: Trajectory config for robot 2 (to be estimated)
        vio_config_robot2: VIO config for robot 2
        arrow_len: Length of orientation arrows
        
    Returns:
        poses_robot1_perfect, poses_robot1_noisy,
        poses_robot2_perfect, poses_robot2_noisy
    """
    # Robot 1 trajectory (perfect reference)
    traj_robot1 = generate_trajectory(
        duration=traj_config_robot1.duration,
        timestep=traj_config_robot1.timestep,
        seed=traj_config_robot1.seed
    )
    poses_robot1_perfect = sgal3_to_se2_poses(traj_robot1.X_RRi)
    poses_robot1_noisy, _ = simulate_vio_odometry(traj_robot1, vio_config_robot1)
    
    # Robot 2 trajectory (to be estimated from robot 1)
    traj_robot2 = generate_trajectory(
        duration=traj_config_robot2.duration,
        timestep=traj_config_robot2.timestep,
        seed=traj_config_robot2.seed
    )
    poses_robot2_perfect = sgal3_to_se2_poses(traj_robot2.X_RRi)
    poses_robot2_noisy, _ = simulate_vio_odometry(traj_robot2, vio_config_robot2)
    
    # Plot both robots: perfect vs VIO estimates
    fig1, ax1 = plt.subplots(1, 1, figsize=(8, 6))
    ax1.plot(poses_robot1_perfect[:, 0], poses_robot1_perfect[:, 1], 'b-', 
             label='Robot 1 Perfect', alpha=0.7)
    ax1.plot(poses_robot2_perfect[:, 0], poses_robot2_perfect[:, 1], 'g-', 
             label='Robot 2 Perfect', alpha=0.7)
    ax1.plot(poses_robot1_noisy[:, 0], poses_robot1_noisy[:, 1], 'b--', 
             label='Robot 1 VIO', alpha=0.6)
    ax1.plot(poses_robot2_noisy[:, 0], poses_robot2_noisy[:, 1], 'g--', 
             label='Robot 2 VIO', alpha=0.6)
    
    step = max(1, len(poses_robot1_perfect) // 20)
    
    for i in range(0, len(poses_robot1_noisy), step):
        x_i, y_i, theta_i = poses_robot1_noisy[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax1.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, 
                  fc='b', ec='b', alpha=0.6)
    
    for i in range(0, len(poses_robot2_noisy), step):
        x_i, y_i, theta_i = poses_robot2_noisy[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax1.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, 
                  fc='g', ec='g', alpha=0.6)
    
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.axis('equal')
    ax1.legend()
    ax1.set_title('Two Robots: Perfect vs VIO Estimated Trajectories with Drift')
    
    plt.tight_layout()
    plt.show()
    
    # Compute relative pose from VIO estimates
    relative_poses_vio = compute_relative_poses(poses_robot1_noisy, poses_robot2_noisy)
    
    # Estimate robot 2 global pose: T_OR2_est = T_OR1_perfect * T_R1R2_VIO
    se2_robot1_perfect = [SE2(p[0], p[1], p[2]) for p in poses_robot1_perfect]
    relative_poses_vio_se2 = [SE2(p[0], p[1], p[2]) for p in relative_poses_vio]
    
    poses_robot2_estimated = np.zeros_like(poses_robot1_perfect)
    for i in range(len(se2_robot1_perfect)):
        T_OR2_est = se2_robot1_perfect[i].compose(relative_poses_vio_se2[i])
        poses_robot2_estimated[i, 0] = T_OR2_est.x()
        poses_robot2_estimated[i, 1] = T_OR2_est.y()
        poses_robot2_estimated[i, 2] = T_OR2_est.angle()
    
    # Plot: Robot 2 estimation from robot 1 perfect + VIO relative pose
    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 6))
    ax2.plot(poses_robot1_perfect[:, 0], poses_robot1_perfect[:, 1], 'b-', 
             label='Robot 1 Perfect', alpha=0.7)
    ax2.plot(poses_robot2_perfect[:, 0], poses_robot2_perfect[:, 1], 'g-', 
             label='Robot 2 Ground Truth', alpha=0.7)
    ax2.plot(poses_robot2_estimated[:, 0], poses_robot2_estimated[:, 1], 'r--', 
             label='Robot 2 Estimated', alpha=0.7)
    
    step = max(1, len(poses_robot2_estimated) // 20)
    for i in range(0, len(poses_robot2_estimated), step):
        x_i, y_i, theta_i = poses_robot2_estimated[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax2.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, 
                  fc='r', ec='r')
    
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.axis('equal')
    ax2.legend()
    ax2.set_title('Robot 2 Estimated from Robot 1 Perfect + VIO Relative Pose')
    
    plt.tight_layout()
    plt.show()
    
    # RMSE Analysis
    timestep = traj_config_robot1.timestep
    t = np.arange(len(relative_poses_vio)) * timestep
    
    pos_error = np.sqrt((poses_robot2_estimated[:, 0] - poses_robot2_perfect[:, 0])**2 + 
                       (poses_robot2_estimated[:, 1] - poses_robot2_perfect[:, 1])**2)
    angle_error = poses_robot2_estimated[:, 2] - poses_robot2_perfect[:, 2]
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    
    fig3, ax3 = plt.subplots(2, 1, figsize=(14, 8))
    
    ax3[0].plot(t, pos_error, 'b-', alpha=0.8)
    ax3[0].set_xlabel('Time (s)')
    ax3[0].set_ylabel('RMSE Distance (m)')
    ax3[0].set_title('Error on Robot 2 Position')
    ax3[0].grid(True)
    
    ax3[1].plot(t, angle_error, 'r-', alpha=0.8)
    ax3[1].set_xlabel('Time (s)')
    ax3[1].set_ylabel('RMSE Angle (rad)')
    ax3[1].set_title('Error on Robot 2 Orientation')
    ax3[1].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    return poses_robot1_perfect, poses_robot1_noisy, poses_robot2_perfect, poses_robot2_noisy
