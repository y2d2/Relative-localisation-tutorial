import numpy as np
import matplotlib.pyplot as plt

from manifpy import SE2

import sys
sys.path.insert(0, '/workspace/Python/Relative-localisation-tutorial/src')

from config import TrajectoryConfig, NoiseConfig
from trajectory import generate_trajectory, sgal3_to_se2_poses, add_noise
from relative_pose import poses_to_se2, compute_relative_pose
import upf_rte.UtilityCode.Transformation_Matrix_Fucntions as TMF

from relative_pose import se2_to_poses


def calculate_rmse(ground_truth: np.ndarray, estimated: np.ndarray) -> tuple[float, float]:
    """Calculate RMSE for position and orientation between two pose arrays.
    
    Args:
        ground_truth: (N, 3) array with columns [x, y, theta]
        estimated: (N, 3) array with columns [x, y, theta]
    
    Returns:
        rmse_pos: RMSE for position (meters)
        rmse_angle: RMSE for orientation (radians)
    """
    pos_error = np.sqrt((estimated[:, 0] - ground_truth[:, 0]) ** 2 +
                        (estimated[:, 1] - ground_truth[:, 1]) ** 2)
    angle_error = estimated[:, 2] - ground_truth[:, 2]
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    
    rmse_pos = np.sqrt(np.mean(pos_error ** 2))
    rmse_angle = np.sqrt(np.mean(angle_error ** 2))
    
    return rmse_pos, rmse_angle


def plot_trajectory(ax, poses, heading =None,
                    color="k", alpha =1.0, style="-", size=1.5,
                    label="",
                    arrows_on = True, arrow_steps =20, arrow_len=0.3):
    if heading is None:
        heading = poses[:,2]
    ax.plot(poses[:, 0], poses[:, 1], style, color=color, alpha=alpha, label=label, linewidth=size)
    ax.plot(poses[0,0], poses[0,1], 'o', color=color, alpha=alpha, markersize= 10)
    ax.plot(poses[-1,0], poses[-1,1], 's', color=color, alpha=alpha, markersize= 10)
    if arrows_on:
        step = max(1, len(heading) // arrow_steps)
        for i in range(0, len(heading), step):
            x_i, y_i, theta_i = poses[i,0], poses[i,1], heading[i]
            dx = arrow_len * np.cos(theta_i)
            dy = arrow_len * np.sin(theta_i)
            ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc=color, ec=color, alpha=alpha)


def get_pose(traj_config: TrajectoryConfig):
    traj_robot = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )
    
    T_start = SE2(traj_config.start_x, traj_config.start_y, traj_config.start_theta)
    
    poses_local = sgal3_to_se2_poses(traj_robot.X_RRi)
    se2_local = poses_to_se2(poses_local)
    
    poses_robot_se2 = [T_start.compose(T) for T in se2_local]
    poses_robot = se2_to_poses(poses_robot_se2)
    
    return poses_robot

def get_noisy_pose(poses_robot, noise_config: NoiseConfig):
    poses_robot_noisy = add_noise(poses_robot,
                                  sigma_x=noise_config.sigma_x,
                                  sigma_y=noise_config.sigma_y,
                                  sigma_theta=noise_config.sigma_theta)
    return poses_robot_noisy

def plot_robot_trajectory(traj_config, noise_config, arrow_len=0.3):
    # todo: also plot the arrows for the ground truth.
    traj_robot1 = generate_trajectory(
        duration=traj_config.duration,
        timestep=traj_config.timestep,
        seed=traj_config.seed
    )
    poses_robot1 = get_pose(traj_config)
    poses_robot1_noisy = get_noisy_pose(poses_robot1, noise_config)

    fig, ax = plt.subplots(1, 1, figsize=(8, 6))

    plot_trajectory(ax, poses_robot1, color='b', alpha=0.7, style='-', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Ground Truth")
    plot_trajectory(ax, poses_robot1_noisy, color='r', alpha=0.7, style='--', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Measured pose")

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Robot: Perfect vs Noisy Trajectory')

    plt.tight_layout()
    plt.show()
    return poses_robot1, poses_robot1_noisy

def plot_2_robots(traj_config_1, noise_config_1, traj_config_2, noise_config_2, arrow_len=0.3):
    poses_robot1 = get_pose(traj_config_1)
    poses_robot2 = get_pose(traj_config_2)
    poses_robot1_noisy = get_noisy_pose(poses_robot1, noise_config_1)
    poses_robot2_noisy = get_noisy_pose(poses_robot2, noise_config_2)

    se2_robot1 = poses_to_se2(poses_robot1)
    se2_robot2 = poses_to_se2(poses_robot2)

    relative_poses = np.zeros_like(poses_robot1)
    for i in range(len(se2_robot1)):
        T_rel = compute_relative_pose(se2_robot1[i], se2_robot2[i])
        relative_poses[i, 0] = T_rel.x()
        relative_poses[i, 1] = T_rel.y()
        relative_poses[i, 2] = T_rel.angle()

    se2_robot1_noisy = poses_to_se2(poses_robot1_noisy)
    se2_robot2_noisy = poses_to_se2(poses_robot2_noisy)

    relative_poses_noisy = np.zeros_like(poses_robot1)
    for i in range(len(se2_robot1_noisy)):
        T_rel = compute_relative_pose(se2_robot1_noisy[i], se2_robot2_noisy[i])
        relative_poses_noisy[i, 0] = T_rel.x()
        relative_poses_noisy[i, 1] = T_rel.y()
        relative_poses_noisy[i, 2] = T_rel.angle()

    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    plot_trajectory(ax, poses_robot1, color='b', alpha=1.0, style='-', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 1 Ground Truth")
    plot_trajectory(ax, poses_robot2, color='g', alpha=1.0, style='-', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 2 Ground Truth")
    plot_trajectory(ax, poses_robot1_noisy, color='lightblue', alpha=1., style='--', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 1 Measured")
    plot_trajectory(ax, poses_robot2_noisy, color='lightgreen', alpha=1., style='--', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 2 Measured")

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.axis('equal')
    ax.legend(fontsize=8)
    ax.set_title('Noisy Global Trajectories')

    t = np.arange(len(relative_poses)) * traj_config_1.timestep

    arrow_step = max(1, len(poses_robot1) // 20)

    # Calculate Robot2 estimated location using Robot1 true pose + noisy relative pose
    relative_poses_noisy_se2 = [SE2(p[0], p[1], p[2]) for p in relative_poses_noisy]
    poses_robot2_estimated = np.zeros_like(poses_robot1)
    for i in range(len(se2_robot1)):
        T_OR2_est = se2_robot1[i].compose(relative_poses_noisy_se2[i])
        poses_robot2_estimated[i, 0] = T_OR2_est.x()
        poses_robot2_estimated[i, 1] = T_OR2_est.y()
        poses_robot2_estimated[i, 2] = T_OR2_est.angle()

    # Plot Robot2 estimation
    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 6))
    plot_trajectory(ax2, poses_robot1, color='b', alpha=1.0, style='-', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 1 Ground Truth")
    plot_trajectory(ax2, poses_robot2, color='g', alpha=1.0, style='-', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 2 Ground Truth")
    plot_trajectory(ax2, poses_robot2_estimated, color='r', alpha=1.0, style='--', size=1.5, arrows_on=True, arrow_steps=20, arrow_len=arrow_len, label="Robot 2 Estimated")

    for i in range(0, len(poses_robot2_estimated), arrow_step):
        x_i, y_i, theta_i = poses_robot2_estimated[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax2.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='r', ec='r')

    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.axis('equal')
    ax2.legend()
    ax2.set_title('Effect of uncertainty of global pose estimation on relative pose estimation')

    plt.tight_layout()
    plt.show()

    # Calculate RMSE metrics (per time instance)
    pos_error = np.sqrt((relative_poses_noisy[:, 0] - relative_poses[:, 0]) ** 2 +
                        (relative_poses_noisy[:, 1] - relative_poses[:, 1]) ** 2)
    angle_error = relative_poses_noisy[:, 2] - relative_poses[:, 2]

    # Calculate global pose RMSE
    pos_error_r1 = np.sqrt((poses_robot1_noisy[:, 0] - poses_robot1[:, 0]) ** 2 +
                           (poses_robot1_noisy[:, 1] - poses_robot1[:, 1]) ** 2)
    angle_error_r1 = poses_robot1_noisy[:, 2] - poses_robot1[:, 2]
    angle_error_r1 = (angle_error_r1 + np.pi) % (2 * np.pi) - np.pi
    pos_rmse_r1, angle_rmse_r1 = calculate_rmse(poses_robot1, poses_robot1_noisy)

    pos_error_r2 = np.sqrt((poses_robot2_noisy[:, 0] - poses_robot2[:, 0]) ** 2 +
                           (poses_robot2_noisy[:, 1] - poses_robot2[:, 1]) ** 2)
    angle_error_r2 = poses_robot2_noisy[:, 2] - poses_robot2[:, 2]
    angle_error_r2 = (angle_error_r2 + np.pi) % (2 * np.pi) - np.pi
    pos_rmse_r2, angle_rmse_r2 = calculate_rmse(poses_robot2, poses_robot2_noisy)

    # Plot RMSE over time with global pose errors and mean annotations
    fig3, ax3 = plt.subplots(3, 1, figsize=(14, 12))

    ax3[0].plot(t, pos_error, 'b-', alpha=0.8)
    ax3[0].set_xlabel('Time (s)')
    ax3[0].set_ylabel('RMSE Distance (m)')
    ax3[0].set_title('RMSE on Relative Pose Distance')
    ax3[0].grid(True)

    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    ax3[1].plot(t, angle_error, 'r-', alpha=0.8)
    ax3[1].set_xlabel('Time (s)')
    ax3[1].set_ylabel('RMSE Angle (rad)')
    ax3[1].set_title('RMSE on Relative Pose Angle')
    ax3[1].grid(True)

    ax3[2].plot(t, pos_error_r1, 'b-', alpha=0.8, label=f'Robot 1: RMSE_pos={pos_rmse_r1:.4f}m, RMSE_ang={angle_rmse_r1:.4f}rad')
    ax3[2].plot(t, pos_error_r2, 'g-', alpha=0.8, label=f'Robot 2: RMSE_pos={pos_rmse_r2:.4f}m, RMSE_ang={angle_rmse_r2:.4f}rad')
    ax3[2].set_xlabel('Time (s)')
    ax3[2].set_ylabel('Position Error (m)')
    ax3[2].set_title('Global Pose Estimation Error (Position) with RMSE Values')
    ax3[2].legend(fontsize=8)
    ax3[2].grid(True)

    plt.tight_layout()
    plt.show()

    fig4, ax4 = plt.subplots(2, 1, figsize=(14, 8))

    ax4[0].plot(t, pos_error_r1, 'b-', alpha=0.8)
    ax4[0].axhline(y=np.mean(pos_error_r1), color='b', linestyle='--', alpha=0.5,
                   label=f'Mean Robot 1: {np.mean(pos_error_r1):.4f}m')
    ax4[0].plot(t, pos_error_r2, 'g-', alpha=0.8)
    ax4[0].axhline(y=np.mean(pos_error_r2), color='g', linestyle='--', alpha=0.5,
                   label=f'Mean Robot 2: {np.mean(pos_error_r2):.4f}m')
    ax4[0].set_xlabel('Time (s)')
    ax4[0].set_ylabel('Position Error (m)')
    ax4[0].set_title('Global Pose Position Errors with Mean Values')
    ax4[0].legend(fontsize=8)
    ax4[0].grid(True)

    ax4[1].plot(t, angle_error_r1, 'r-', alpha=0.8)
    ax4[1].axhline(y=np.mean(angle_error_r1), color='b', linestyle='--', alpha=0.5,
                   label=f'Mean Robot 1: {np.mean(angle_error_r1):.4f}rad')
    ax4[1].plot(t, angle_error_r2, 'm-', alpha=0.8)
    ax4[1].axhline(y=np.mean(angle_error_r2), color='g', linestyle='--', alpha=0.5,
                   label=f'Mean Robot 2: {np.mean(angle_error_r2):.4f}rad')
    ax4[1].set_xlabel('Time (s)')
    ax4[1].set_ylabel('Angle Error (rad)')
    ax4[1].set_title('Global Pose Angle Errors with Mean Values')
    ax4[1].legend(fontsize=8)
    ax4[1].grid(True)

    plt.tight_layout()
    plt.show()
