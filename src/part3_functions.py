"""Part 3 - SLAM Global Localization: Functions for VIO odometry drift demonstration."""

import numpy as np
import matplotlib.pyplot as plt

from manifpy import SE2

from config import TrajectoryConfig, VIOConfig
from trajectory import generate_trajectory, sgal3_to_se2_poses
from vio_sensor import simulate_vio_odometry
from relative_pose import compute_relative_poses
from manifpy import SGal3

def get_noisy_vio_trajectory(traj_config: TrajectoryConfig, vio_config: VIOConfig) -> tuple[np.ndarray, np.ndarray]:
    traj = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        start_x=traj_config.start_x,
        start_y=traj_config.start_y,
        start_theta=traj_config.start_theta,
        seed=traj_config.seed
    )
    poses_perfect = sgal3_to_se2_poses(traj.X_RRi)
    poses_noisy, _ = simulate_vio_odometry(traj, vio_config)
    return poses_perfect, poses_noisy

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
    poses_perfect, poses_noisy = get_noisy_vio_trajectory(traj_config, vio_config)
    
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

    timestep = traj_config.timestep
    t = np.arange(len(poses_perfect)) * timestep

    pos_error_1 = np.sqrt((poses_noisy[:, 0] - poses_perfect[:, 0]) ** 2 +
                          (poses_noisy[:, 1] - poses_perfect[:, 1]) ** 2
                          )
    RMSE_robot_1 = np.mean(pos_error_1)

    angle_error_1 = poses_noisy[:, 2] - poses_perfect[:, 2]
    angle_error_1 = np.abs((angle_error_1 + np.pi) % (2 * np.pi) - np.pi)
    RMSE_robot_a_1 = np.mean(angle_error_1)

    fig3, ax3 = plt.subplots(2, 1, figsize=(14, 8))

    ax3[0].plot(t, pos_error_1, 'r-', alpha=1.0, label=f"Estimated pose of robot 1 error, RMSE={RMSE_robot_1:.2f}m")

    ax3[0].set_xlabel('Time (s)')
    ax3[0].set_ylabel('RMSE Distance (m)')
    ax3[0].legend()
    ax3[0].grid(True)

    ax3[1].plot(t, angle_error_1, 'r-', alpha=1.,
                label=f"Estimated angle of robot 1 error, RMSE={np.rad2deg(RMSE_robot_a_1):.2f} deg")
    ax3[1].set_xlabel('Time (s)')
    ax3[1].set_ylabel('RMSE Angle (deg)')
    ax3[1].legend()
    ax3[1].grid(True)

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
    poses_robot1_perfect, poses_robot1_noisy = get_noisy_vio_trajectory(traj_config_robot1, vio_config_robot1)

    # Robot 2 trajectory (to be estimated from robot 1)
    poses_robot2_perfect, poses_robot2_noisy = get_noisy_vio_trajectory(traj_config_robot2, vio_config_robot2)

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

    # Plot both robots: perfect vs VIO estimates
    fig1, ax1 = plt.subplots(1, 1, figsize=(8, 6))
    ax1.plot(poses_robot1_perfect[:, 0], poses_robot1_perfect[:, 1], 'b-', 
             label='Robot 1 Perfect', alpha=0.7)
    ax1.plot(poses_robot2_perfect[:, 0], poses_robot2_perfect[:, 1], 'g-', 
             label='Robot 2 Perfect', alpha=0.7)
    ax1.plot(poses_robot1_noisy[:, 0], poses_robot1_noisy[:, 1], 'lightblue',
             label='Robot 1 VIO', alpha=0.6)
    ax1.plot(poses_robot2_noisy[:, 0], poses_robot2_noisy[:, 1], 'lightgreen',
             label='Robot 2 VIO', alpha=0.6)
    ax1.plot(poses_robot2_estimated[:, 0], poses_robot2_estimated[:, 1], 'r--',
             label='Robot 2 Estimated', alpha=0.7)
    
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.axis('equal')
    ax1.legend()
    ax1.set_title('Two Robots: Perfect vs VIO Estimated Trajectories with Drift')
    
    plt.tight_layout()
    plt.show()

    # RMSE Analysis
    timestep = traj_config_robot1.timestep
    t = np.arange(len(relative_poses_vio)) * timestep

    pos_error_1 = np.sqrt((poses_robot1_noisy[:,0]- poses_robot1_perfect[:,0])**2 +
                          (poses_robot1_noisy[:,1]- poses_robot1_perfect[:,1])**2
                          )
    RMSE_robot_1 = np.mean(pos_error_1)

    pos_error_2 = np.sqrt((poses_robot2_noisy[:,0]- poses_robot2_perfect[:,0])**2 +
                          (poses_robot2_noisy[:,1]- poses_robot2_perfect[:,1])**2)
    RMSE_robot_2 = np.mean(pos_error_2)
    
    pos_error = np.sqrt((poses_robot2_estimated[:, 0] - poses_robot2_perfect[:, 0])**2 + 
                       (poses_robot2_estimated[:, 1] - poses_robot2_perfect[:, 1])**2)
    RMSE = np.mean(pos_error)

    angle_error_1 = poses_robot1_noisy[:,2] - poses_robot1_perfect[:,2]
    angle_error_1 =  np.abs((angle_error_1 + np.pi) % (2 * np.pi) - np.pi)
    RMSE_robot_a_1 = np.mean(angle_error_1)
    angle_error_2 = poses_robot2_noisy[:, 2] - poses_robot2_perfect[:, 2]
    angle_error_2 = np.abs((angle_error_2 + np.pi) % (2 * np.pi) - np.pi)
    RMSE_robot_a_2 = np.mean(angle_error_2)
    angle_error = poses_robot2_estimated[:, 2] - poses_robot2_perfect[:, 2]
    angle_error = np.abs((angle_error + np.pi) % (2 * np.pi) - np.pi)
    RMSE_a_2 = np.mean(angle_error)
    
    fig3, ax3 = plt.subplots(2, 1, figsize=(14, 8))
    
    ax3[0].plot(t, pos_error_1, 'b-', alpha=1.0, label=f"Estimated pose of robot 1 error, RMSE={RMSE_robot_1:.2f}m")
    ax3[0].plot(t, pos_error_2, 'g-', alpha=1.0, label=f"Estimated pose of robot 2 error, RMSE={RMSE_robot_2:.2f}m")
    ax3[0].plot(t, pos_error, 'r-', alpha=1.0, label=f"Estimated relative pose error,  RMSE={RMSE:.2f}m")

    ax3[0].set_xlabel('Time (s)')
    ax3[0].set_ylabel('RMSE Distance (m)')
    ax3[0].set_title('Error on position')
    ax3[0].legend()
    ax3[0].grid(True)

    ax3[1].plot(t, angle_error_1, 'b-', alpha=1., label=f"Estimated angle of robot 1 error, RMSE={np.rad2deg(RMSE_robot_a_1):.2f} deg")
    ax3[1].plot(t, angle_error_2, 'g-', alpha=1., label=f"Estimated angle of robot 2 error, RMSE={np.rad2deg(RMSE_robot_a_2):.2f} deg")
    ax3[1].plot(t, angle_error, 'r-', alpha=1., label=f"Estimated relative angle error, RMSE={np.rad2deg(RMSE_a_2):.2f} deg")
    ax3[1].set_xlabel('Time (s)')
    ax3[1].set_ylabel('RMSE Angle (deg)')
    ax3[1].set_title('Error on orientation')
    ax3[0].legend()
    ax3[1].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    return poses_robot1_perfect, poses_robot1_noisy, poses_robot2_perfect, poses_robot2_noisy
