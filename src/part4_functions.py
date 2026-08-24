"""UPF for single beacon localization - estimating robot pose relative to anchor."""

import numpy as np
import matplotlib.pyplot as plt

from arp_utility_code.Simulation.Trajectories import Trajectory
from arp_utility_code.UtilityCode import Transformation_Matrix_Fucntions as TMF

# from src._2d_upf import UPFConnectedAgent_2D
from upf_rte.DataLoggers.ConnectedAgent_DataLogger import UPFConnectedAgentDataLogger
from upf_rte.Simulation.RobotClass import NewRobot
from src.vio_sensor import simulate_vio_odometry
from src.part1_functions import get_pose
from trajectory import generate_trajectory, sgal3_to_se2_poses
from src.part3_functions import get_noisy_vio_trajectory
from _2D_UPF import UPF_2D
from src.part2_functions import simulate_uwb_ranges


def poses_to_transformation_matrices(poses):
    """Convert SE(2) poses [x, y, theta] to list of 4x4 transformation matrices.
    
    Args:
        poses: (N, 3) array with columns [x, y, theta]
        
    Returns:
        T_list: List of 4x4 transformation matrices
    """
    n = len(poses)
    T_list = []
    
    for i in range(n):
        x, y, theta = poses[i]
        T = TMF.transformation_matrix_from_rot_vect([0, 0, theta], [x, y, 0])
        T_list.append(T)
    
    return T_list


def poses_to_delta_transformations(poses):
    """Convert absolute poses to delta transformations.
    
    Args:
        poses: (N, 3) array with columns [x, y, theta]
        
    Returns:
        DT_list: List of 4x4 delta transformation matrices
    """
    n = len(poses)
    DT_list = []
    
    for i in range(1, n):
        x_curr, y_curr, theta_curr = poses[i]
        x_prev, y_prev, theta_prev = poses[i-1]
        
        R_prev = np.array([
            [np.cos(theta_prev), -np.sin(theta_prev)],
            [np.sin(theta_prev),  np.cos(theta_prev)]
        ])
        
        diff = np.array([x_curr - x_prev, y_curr - y_prev])
        diff_local = R_prev.T @ diff
        
        theta_diff = (theta_curr - theta_prev + np.pi) % (2 * np.pi) - np.pi
        
        #Transformation_Matrix_Fucntions expects 3D translation [x, y, z]
        DT = TMF.transformation_matrix_from_rot_vect([0, 0, theta_diff], [diff_local[0], diff_local[1], 0])
        DT_list.append(DT)
    
    return DT_list


def estimnate_robot_location_with_beacon(
        traj_config,
        vio_config,          # VIOConfig for odometry noise
        uwb_config,
        anchor_pos,  # (2,) beacon position in world frame
        n_azimuth = 4,
        n_heading = 4,
        seed =None):
    if seed is not None:
        np.random.seed(seed)

    perfec_poses, noisy_poses = get_noisy_vio_trajectory(traj_config, vio_config)
    uwb_ranges = simulate_uwb_ranges(perfec_poses, np.array([anchor_pos]), uwb_config)

    x_ha_0 = np.array([anchor_pos[0], anchor_pos[1], 0, 0])
    upf = UPF_2D(x_ha_0=x_ha_0)
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.set_initalisation_parameters(sigma_uwb= 2*uwb_config.sig_range, n_azimuth=n_azimuth, n_heading=n_heading,
                                      dz=0, sigma_dz=0.0001)
    upf.set_regeneration_parameters(regeneration= False)
    upf.min_likelihood = 0.9
    q_j = np.diag([
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (1e-8),
        (vio_config.sig_w * traj_config.timestep) ** 2
    ])
    q_i = np.diag([(1e-8 * traj_config.timestep) ** 2] * 4)
    n_steps = len(perfec_poses)
    t_seq = [traj_config.timestep * i for i in range(n_steps)]

    list_n_particles = []
    
    # Convert perfect poses to transformation matrices for Vicon tracking (ground truth)
    perfec_T_list = poses_to_transformation_matrices(perfec_poses)
    
    # Convert noisy poses to delta transformations for VIO SLAM
    DT_noisy_list = poses_to_delta_transformations(noisy_poses)
    # Prepend zero delta to match lengths: N poses need N deltas (first is no movement)
    zero_dt = TMF.transformation_matrix_from_rot_vect([0, 0, 0], [0, 0, 0])
    DT_noisy_list_full = [zero_dt] + DT_noisy_list
    
    Q_vio = np.diag([
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_w * traj_config.timestep) ** 2
    ])
    
    # Initialize Connected Agent (robot) with VIO SLAM and Vicon tracking
    ca = NewRobot()
    ca.from_experimental_data(perfec_T_list, DT_noisy_list_full, Q_vio, int(1/traj_config.timestep))
    
    # Initialize Host Agent (anchor) - stationary beacon at origin of its frame
    ha = NewRobot(x_start=np.array([anchor_pos[0], anchor_pos[1], 0]), h_start=0, 
                  simulation_time_step=traj_config.timestep)
    ha.set_vicon_tracking(perfec_T_list)
    
    # For anchor, VIO SLAM is just zero movement with small uncertainty
    DT_anchor = [TMF.transformation_matrix_from_rot_vect([0, 0, 0], [0, 0, 0]) for _ in range(len(perfec_T_list))]
    ha.set_vio_slam(DT_anchor, q_i)

    upf_logger = UPFConnectedAgentDataLogger(ha, ca, upf)
    for i in range(n_steps):
        upf.ha.set_x_ha(x_ha_0)
        upf.ha.p_x_ha = q_i
        noisy_pose = np.zeros(4)
        noisy_pose[:2] = noisy_poses[i][:2]
        noisy_pose[3] = noisy_poses[i][2]
        upf.run_2d_model(noisy_pose, q_j, uwb_ranges[i][0])
        list_n_particles.append(len(upf.particles))
        upf_logger.log_data(i)
        print(len(upf.particles))

    plot_upf_estimations(upf_logger, perfec_poses, noisy_poses, anchor_pos, traj_config)


def plot_upf_estimations(
    upf_logger,
    perfect_poses: np.ndarray,
    noisy_poses: np.ndarray,
    anchor_pos: np.ndarray,
    traj_config
) -> None:
    """Plot UPF estimation results with particle evolution.
    
    Args:
        upf_logger: UPFConnectedAgentDataLogger from estimnate_robot_location_with_beacon
        perfect_poses: (N, 3) array of ground truth robot poses [x, y, theta]
        noisy_poses: (N, 3) array of VIO-noisy robot poses [x, y, theta]
        anchor_pos: (2,) beacon ground truth position [x, y]
        traj_config: TrajectoryConfig for timestep
    """
    fig = plt.figure(figsize=(14, 10))
    
    n_steps = len(perfect_poses)
    t = np.arange(n_steps) * traj_config.timestep
    
    # Subplot 1: Trajectory with particle cloud
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(perfect_poses[:, 0], perfect_poses[:, 1], 'b-', label='Perfect trajectory', alpha=0.7, linewidth=2)
    ax1.plot(noisy_poses[:, 0], noisy_poses[:, 1], 'r--', label='VIO noisy', alpha=0.6, linewidth=1.5)
    
    step = max(1, n_steps // 20)
    arrow_len = 0.3
    for i in range(0, len(perfect_poses), step):
        x_i, y_i, theta_i = perfect_poses[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax1.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='b', ec='b', alpha=0.7)
    
    for i in range(0, len(noisy_poses), step):
        x_i, y_i, theta_i = noisy_poses[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax1.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1, fc='r', ec='r', alpha=0.6)
    
    ax1.scatter(anchor_pos[0], anchor_pos[1], c='green', marker='^', s=200, 
                label='Beacon (ground truth)', zorder=5)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.axis('equal')
    ax1.legend()
    ax1.set_title('Robot Trajectory and Beacon Position')
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Particle count evolution
    ax2 = fig.add_subplot(2, 2, 2)
    n_particles_list = upf_logger.number_of_particles
    ax2.plot(t[:len(n_particles_list)], n_particles_list, 'b-', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Number of particles')
    ax2.set_title('Particle Count Evolution')
    ax2.grid(True, alpha=0.3)
    
    # Subplot 3: Position error over time
    ax3 = fig.add_subplot(2, 2, 3)
    
    vio_pos_error = np.sqrt((noisy_poses[:, 0] - perfect_poses[:, 0])**2 + 
                            (noisy_poses[:, 1] - perfect_poses[:, 1])**2)
    ax3.plot(t, vio_pos_error, 'r--', label='VIO error', alpha=0.7, linewidth=1.5)
    
    best_particle_log = upf_logger.get_best_particle_log()
    if best_particle_log is not None:
        est_pos_error = np.sqrt((best_particle_log.connected_agent_trajectory[:, 0] - perfect_poses[:, 0])**2 + 
                                (best_particle_log.connected_agent_trajectory[:, 1] - perfect_poses[:, 1])**2)
        ax3.plot(t[:len(est_pos_error)], est_pos_error, 'b-', label='UPF error', alpha=0.8, linewidth=2)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position error (m)')
    ax3.legend()
    ax3.set_title(' Position Estimation Error')
    ax3.grid(True, alpha=0.3)
    
    # Subplot 4: Beacon uncertainty from particles
    ax4 = fig.add_subplot(2, 2, 4)
    
    from matplotlib.patches import Ellipse
    
    beacon_states = np.array([p.kf.x[:2] for p in upf_logger.upf_connected_agent.particles])
    mean_beacon = np.mean(beacon_states, axis=0)
    cov_beacon = np.cov(beacon_states.T)
    
    ax4.scatter(anchor_pos[0], anchor_pos[1], c='green', marker='^', s=200, 
                label='Ground truth', zorder=5, alpha=0.8)
    ax4.scatter(mean_beacon[0], mean_beacon[1], c='orange', marker='o', s=100,
               label='UPF estimate (mean)', zorder=5, alpha=0.8)
    
    vals, vecs = np.linalg.eigh(cov_beacon[:2, :2])
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    width = 2 * 2.0 * np.sqrt(vals[0])
    height = 2 * 2.0 * np.sqrt(vals[1])
    
    ell = Ellipse(xy=mean_beacon[:2], width=width, height=height,
                  angle=angle, ec='orange', fc='none', lw=2, alpha=0.8)
    ax4.add_patch(ell)
    
    ax4.set_xlabel('x (m)')
    ax4.set_ylabel('y (m)')
    ax4.axis('equal')
    ax4.legend()
    ax4.set_title('Beacon Position Estimation')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

