"""UPF for single beacon localization - estimating robot pose relative to anchor."""

import numpy as np

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

    # oc-todo analayse the UPFConnectedAgentDataLogger. I want to plot the evolution of the estimations (all living and dead particles)
    #       and the real and noisy poses of the robot and the location of the beacon.
    #       There should be plots close to what I try to achieve.
#
# def estimate_robot_relative_to_beacon_with_upf(
#     trajectory,          # Trajectory object from ARP_utility_code
#     anchor_pos,          # (2,) beacon position in world frame
#     vio_config,          # VIOConfig for odometry noise
#     uwb_config,          # UWBNoiseConfig for range measurement noise
#     n_azimuth=4,
#     n_altitude=1,        # Will be made odd internally if even
#     n_heading=4,
#     seed=None
# ):
#     """Estimate robot pose relative to single UWB beacon using UPF.
#
#     Args:
#         trajectory: Ground truth Trajectory from ARP_utility_code
#         anchor_pos: (2,) anchor position in world frame
#         vio_config: VIOConfig for odometry noise
#         uwb_config: UWBNoiseConfig for range measurement noise
#         n_azimuth: Number of azimuth particles (default 4)
#         n_altitude: Number of altitude particles (default 1, made odd internally)
#         n_heading: Number of heading particles (default 4)
#         seed: Random seed
#
#     Returns:
#         robot_states: list of (3,) [r, theta, h] spherical coords relative to anchor
#         covariances: list of (3, 3) covariance matrices
#         t_seq: list of timestamps
#     """
#     if seed is not None:
#         np.random.seed(seed)
#
#     sig_v = vio_config.sig_v
#
#     # Extract noisy odometry from trajectory
#     poses_noisy, _ = simulate_vio_odometry(trajectory, vio_config)
#
#     n_steps = len(trajectory.t)
#     t_seq = list(trajectory.t)
#
#     # Initialize UPF with beacon as host (stationary)
#     x_ha_0 = np.array([anchor_pos[0], anchor_pos[1], 0, 0])
#     upf = UPFConnectedAgent_2D(x_ha_0=x_ha_0)
#     upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
#
#     # Compute initial range from first pose to beacon
#     r_init = np.linalg.norm(poses_noisy[0, :2] - anchor_pos)
#
#     # Create particles: n_altitude × n_azimuth × n_heading = 1×4×4 = 16 particles
#     upf.split_sphere_in_equal_areas(
#         r=r_init,
#         sigma_uwb=uwb_config.sigma_range,
#         n_altitude=n_altitude,
#         n_azimuth=n_azimuth,
#         n_heading=n_heading
#     )
#
#     # Process noise for robot odometry
#     sig_w = vio_config.sig_w
#     dt = vio_config.timestep if hasattr(vio_config, 'timestep') else 0.05
#     q_j = np.diag([
#         (sig_v * dt) ** 2,
#         (sig_v * dt) ** 2,
#         (sig_w * dt) ** 2,
#         (sig_w * dt) ** 2
#     ])
#
#     # Host (beacon) motion - very small covariance since beacon is fixed
#     dt_i = np.zeros(4)
#     q_i = np.diag([(1e-6 * dt) ** 2] * 4)
#
#     robot_states = []
#     covariances = []
#
#     for i in range(n_steps):
#         if i > 0:
#             # Compute odometry delta from noisy poses
#             dx = poses_noisy[i, 0] - poses_noisy[i-1, 0]
#             dy = poses_noisy[i, 1] - poses_noisy[i-1, 1]
#             dtheta = (poses_noisy[i, 2] - poses_noisy[i-1, 2] + np.pi) % (2 * np.pi) - np.pi
#
#             dt_j = np.array([dx, dy, 0.0, dtheta])
#
#             # UWB range measurement with noise
#             range_true = np.linalg.norm(poses_noisy[i, :2] - anchor_pos)
#             range_meas = range_true + np.random.randn() * uwb_config.sigma_range
#
#             # Run particle filter update
#             upf.run_model(dt_j, q_j, dt_i, q_i, range_meas)
#
#         # Store state and covariance (3D: [r, θ, h])
#         robot_states.append(upf.t_si_sj.copy())
#         covariances.append(upf.P_t_si_sj.copy())
#
#     return robot_states, covariances, t_seq
#
#
# def run_single_beacon_upf(trajectory, anchor_pos, vio_config, uwb_config, seed=None):
#     """Estimate robot pose relative to single UWB beacon using UPF.
#
#     Args:
#         trajectory: Ground truth Trajectory from ARP_utility_code
#         anchor_pos: (2,) anchor position in world frame
#         vio_config: VIOConfig for odometry noise
#         uwb_config: UWBNoiseConfig for range measurement noise
#         seed: Random seed
#
#     Returns:
#         robot_states: list of (3,) [r, theta, h] spherical coords relative to anchor
#         covariances: list of (3, 3) covariance matrices
#         t_seq: list of timestamps
#     """
#     if seed is not None:
#         np.random.seed(seed)
#
#     sig_v = vio_config.sig_v
#
#     n_steps = len(trajectory.t)
#     t_seq = list(trajectory.t)
#
#     ukf = TargetTrackingUKF_2D(x_ha_0=np.zeros(4))
#     ukf.set_ukf_properties()
#
#     state_0 = trajectory.X_RRi[0]
#     robot_pos_0 = state_0.translation()[:2]
#     rel_pos_0 = robot_pos_0 - anchor_pos
#     r_init = np.linalg.norm(rel_pos_0)
#     theta_init = np.arctan2(rel_pos_0[1], rel_pos_0[0])
#     h_init = np.arctan2(state_0.rotation()[1, 0], state_0.rotation()[0, 0])
#
#     t_j_ukf = np.array([r_init, theta_init, h_init])
#     sigma_t = np.array([10.0, 1.0, 1.0])
#
#     ukf.set_initial_state(t_j_ukf, sigma_t)
#
#     robot_states = []
#     covariances = []
#
#     q_odom = np.eye(4) * (sig_v ** 2)
#
#     for i in range(n_steps):
#         if i > 0:
#             state_curr = trajectory.X_RRi[i]
#             state_prev = trajectory.X_RRi[i-1]
#
#             dx = state_curr.translation()[0] - state_prev.translation()[0]
#             dy = state_curr.translation()[1] - state_prev.translation()[1]
#             dtheta = (state_curr.coeffs()[5] - state_prev.coeffs()[5] + np.pi) % (2 * np.pi) - np.pi
#
#             dt_j = np.array([dx, dy, 0.0, dtheta])
#
#             robot_pos_curr = state_curr.translation()[:2]
#             range_true = np.linalg.norm(robot_pos_curr - anchor_pos)
#             range_meas = range_true + np.random.randn() * uwb_config.sigma_range
#
#             sig_d = uwb_config.sigma_range
#
#             ukf.run_filter(dt_j, q_odom, np.zeros(4), q_odom,
#                           range_meas, sig_d, update_bool=True)
#
#         robot_states.append(ukf.t_si_sj.copy())
#         covariances.append(ukf.P_t_si_sj.copy())
#
#     return robot_states, covariances, t_seq
