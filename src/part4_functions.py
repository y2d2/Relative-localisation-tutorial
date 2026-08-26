"""UPF for single beacon localization - estimating robot pose relative to anchor."""

import numpy as np
import matplotlib.pyplot as plt

from arp_utility_code.Simulation.Trajectories import Trajectory
from arp_utility_code.UtilityCode import Transformation_Matrix_Fucntions as TMF

# from src._2d_upf import UPFConnectedAgent_2D
from upf_rte.DataLoggers.ConnectedAgent_DataLogger import UPFConnectedAgentDataLogger, UKFDatalogger
from upf_rte.Simulation.RobotClass import NewRobot
from src.vio_sensor import simulate_vio_odometry
from src.part1_functions import get_pose, plot_trajectory
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

def track_robot_location_with_beacon( traj_config,
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
    upf = UPF_2D(x_ha_0=x_ha_0, drift_correction_bool=False)
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.set_initalisation_parameters(sigma_uwb= uwb_config.sig_range, n_azimuth=n_azimuth, n_heading=n_heading,
                                      dz=0, sigma_dz=0.0001)
    upf.set_regeneration_parameters(regeneration= False)
    upf.min_likelihood = 0.9


    n_steps = len(perfec_poses)

    list_n_particles = []

    
    # Convert noisy poses to delta transformations for VIO SLAM
    ca = NewRobot()
    perfec_T_list = poses_to_transformation_matrices(perfec_poses)
    T_noisy_list = poses_to_transformation_matrices(noisy_poses)
    DT_noisy_list = [TMF.inv_transformation_matrix(T_noisy_list[i]) @ T_noisy_list[i+1] for i in range(len(T_noisy_list)-1) ]
    DT_noisy_list.insert(0, np.eye(4))
    q_j = np.diag([
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        1e-16,
        (vio_config.sig_w * traj_config.timestep) ** 2
    ])
    ca.from_experimental_data(perfec_T_list, DT_noisy_list, q_j, int(1 / traj_config.timestep))
    T_oj = TMF.transformation_matrix_from_rot_vect([0, 0, ca.h_real[0]], ca.x_real[0])
    upf.create_partilce_from_known_initial_position(T_oj)

    # Initialize Host Agent (anchor) - stationary beacon at origin of its frame

    ha = NewRobot()
    q_i = np.diag([(1e-8 * traj_config.timestep) ** 2] * 4)
    anchor_T = TMF.transformation_matrix_from_rot_vect([0, 0, 0], [*anchor_pos, 0])
    anchor_T_list = [anchor_T] * n_steps
    ha.from_experimental_data(anchor_T_list, [np.eye(4)] * n_steps ,q_i, int(1/traj_config.timestep))

    
    # For anchor, VIO SLAM is just zero movement with small uncertainty

    upf_logger = UPFConnectedAgentDataLogger(ha, ca, upf)
    for i in range(n_steps):
        upf.ha.set_x_ha(x_ha_0)
        upf.ha.p_x_ha = q_i
        dt = TMF.get_4D_t_from_matrix(DT_noisy_list[i])

        upf.run_2d_model(dt, q_j, uwb_ranges[i][0])
        list_n_particles.append(len(upf.particles))
        upf_logger.log_data(i)

    plot_upf_2d(upf_logger)
    print(len(upf.particles))




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
    upf = UPF_2D(x_ha_0=x_ha_0, drift_correction_bool=False)
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.set_initalisation_parameters(sigma_uwb= uwb_config.sig_range, n_azimuth=n_azimuth, n_heading=n_heading,
                                      dz=0, sigma_dz=0.0001)
    upf.set_regeneration_parameters(regeneration=False)
    upf.min_likelihood = 0.9

    n_steps = len(perfec_poses)

    list_n_particles = []

    ca = NewRobot()
    perfec_T_list = poses_to_transformation_matrices(perfec_poses)
    T_noisy_list = poses_to_transformation_matrices(noisy_poses)
    DT_noisy_list = [TMF.inv_transformation_matrix(T_noisy_list[i]) @ T_noisy_list[i+1] for i in range(len(T_noisy_list)-1) ]
    DT_noisy_list.insert(0, np.eye(4))
    q_j = np.diag([
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        1e-16,
        (vio_config.sig_w * traj_config.timestep) ** 2
    ])
    ca.from_experimental_data(perfec_T_list, DT_noisy_list, q_j, int(1 / traj_config.timestep))

    ha = NewRobot()
    q_i = np.diag([(1e-8 * traj_config.timestep) ** 2] * 4)
    anchor_T = TMF.transformation_matrix_from_rot_vect([0, 0, 0], [*anchor_pos, 0])
    anchor_T_list = [anchor_T] * n_steps
    ha.from_experimental_data(anchor_T_list, [np.eye(4)] * n_steps ,q_i, int(1/traj_config.timestep))

    upf_logger = UPFConnectedAgentDataLogger(ha, ca, upf)
    for i in range(n_steps):
        upf.ha.set_x_ha(x_ha_0)
        upf.ha.p_x_ha = q_i
        dt = TMF.get_4D_t_from_matrix(DT_noisy_list[i])

        upf.run_2d_model(dt, q_j, uwb_ranges[i][0])
        list_n_particles.append(len(upf.particles))
        upf_logger.log_data(i)

    plot_upf_2d(upf_logger, False)
    print(len(upf.particles))



def estimate_anchor_from_robot(
        traj_config,
        vio_config,          # VIOConfig for odometry noise
        uwb_config,
        anchor_pos,  # (2,) beacon position in world frame
        n_azimuth=4,
        n_heading=4,
        seed=None):
    if seed is not None:
        np.random.seed(seed)

    perfect_poses, noisy_poses = get_noisy_vio_trajectory(traj_config, vio_config)
    uwb_ranges = simulate_uwb_ranges(perfect_poses, np.array([anchor_pos]), uwb_config)

    n_steps = len(perfect_poses)

    list_n_particles = []

    ha = NewRobot()
    perfec_T_list = poses_to_transformation_matrices(perfect_poses)
    T_noisy_list = poses_to_transformation_matrices(noisy_poses)
    DT_noisy_list = [TMF.inv_transformation_matrix(T_noisy_list[i]) @ T_noisy_list[i+1] for i in range(len(T_noisy_list)-1)]
    DT_noisy_list.insert(0, np.eye(4))
    q_j = np.diag([
        (vio_config.sig_v * traj_config.timestep) ** 2,
        (vio_config.sig_v * traj_config.timestep) ** 2,
        1e-16,
        (vio_config.sig_w * traj_config.timestep) ** 2
    ])
    ha.from_experimental_data(perfec_T_list, DT_noisy_list, q_j, int(1 / traj_config.timestep))
    t_ha = TMF.get_4D_t_from_matrix(T_noisy_list[0])
    upf = UPF_2D(x_ha_0=t_ha, drift_correction_bool=True)
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.set_initalisation_parameters(sigma_uwb=uwb_config.sig_range, n_azimuth=n_azimuth, n_heading=n_heading,
                                     dz=0, sigma_dz=0.0001)
    upf.set_regeneration_parameters(regeneration=False)
    upf.min_likelihood = 0.9

    ca = NewRobot()
    q_i = np.diag([1e-16]*4)

    anchor_T = TMF.transformation_matrix_from_rot_vect([0, 0, 0], [*anchor_pos, 0])
    anchor_T_list = [anchor_T] * n_steps
    ca.from_experimental_data(anchor_T_list, [np.eye(4)] * n_steps, q_i, int(1/traj_config.timestep))
    # T_oj = TMF.transformation_matrix_from_rot_vect([0, 0, ca.h_real[0]], ca.x_real[0])
    # upf.create_partilce_from_known_initial_position(T_oj)

    upf_logger = UPFConnectedAgentDataLogger(ha, ca, upf)
    
    for i in range(n_steps):
        t_ha = TMF.get_4D_t_from_matrix(T_noisy_list[i])
        upf.ha.set_x_ha(t_ha)
        upf.ha.p_x_ha = q_i
        dt = np.zeros(4)
        upf.run_2d_model(dt, q_j, uwb_ranges[i][0])
        list_n_particles.append(len(upf.particles))
        upf_logger.log_data(i)

    plot_upf_2d(upf_logger)
    print(len(upf.particles))


def plot_upf_2d(upf_logger: UPFConnectedAgentDataLogger, SLAM_on = True):
    plt.figure()

    # plt.plot(upf_logger.connected_agent.x_slam[:,0], upf_logger.connected_agent.x_slam[:,1],  'r--', label='VIO noisy', alpha=0.6, linewidth=1.5)

    for particle_log  in upf_logger.particle_logs:
        log : UKFDatalogger =particle_log
        if log.ukf in upf_logger.upf_connected_agent.particles:
            plot_trajectory(plt, log.estimated_ca_position, log.ca_heading_estimation, color="g", alpha=1., style="-", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="")
        else:
            plot_trajectory(plt, log.estimated_ca_position, log.ca_heading_estimation, color="lightgreen", alpha=1., style="--", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="")
    plt.plot(upf_logger.host_agent.x_real[0,0],upf_logger.host_agent.x_real[0,1],"-", color="g", alpha=1., linewidth=1.5, label="Active estimations")
    plt.plot(upf_logger.host_agent.x_real[0,0],upf_logger.host_agent.x_real[0,1],"--", color="lightgreen", alpha=1.0, linewidth=1.5, label="Killed estimations")

    if SLAM_on:
        plot_trajectory(plt, upf_logger.connected_agent.x_slam, upf_logger.connected_agent.h_slam, color="r", alpha=1., style="--", size=1.5, arrows_on=True, arrow_steps=20, arrow_len=0.3, label="SLAM Estimation")
    plot_trajectory(plt, upf_logger.connected_agent.x_real, upf_logger.connected_agent.h_real, color="b", alpha=1., style="-", size=1.5, arrows_on=True, arrow_steps=20, arrow_len=0.3, label="True Trajectory")
    plot_trajectory(plt, upf_logger.host_agent.x_real, upf_logger.host_agent.h_real, color="k", alpha=1., style="-", size=1.5, arrows_on=True, arrow_steps=20, arrow_len=0.3, label="Anchor Trajectory")

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend(loc='upper left')
    plt.show()


def plot_upf_2d_reverse(upf_logger: UPFConnectedAgentDataLogger):
    plt.figure()

    # plt.plot(upf_logger.connected_agent.x_slam[:,0], upf_logger.connected_agent.x_slam[:,1],  'r--', label='VIO noisy', alpha=0.6, linewidth=1.5)

    for particle_log  in upf_logger.particle_logs:
        log : UKFDatalogger =particle_log
        if log.ukf in upf_logger.upf_connected_agent.particles:
            plot_trajectory(plt, log.estimated_ca_position, log.ca_heading_estimation, color="g", alpha=1., style="-", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="")
        else:
            plot_trajectory(plt, log.estimated_ca_position, log.ca_heading_estimation, color="lightgreen", alpha=1., style="--", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="")
    plt.plot(upf_logger.host_agent.x_real[0,0],upf_logger.host_agent.x_real[0,1],"-", color="g", alpha=1., linewidth=1.5, label="Active estimations")
    plt.plot(upf_logger.host_agent.x_real[0,0],upf_logger.host_agent.x_real[0,1],"--", color="lightgreen", alpha=1., linewidth=1.5, label="Killed estimations")

    # plot_trajectory(plt, upf_logger.connected_agent.x_slam, upf_logger.connected_agent.h_slam, color="r", alpha=1., style="--", size=1.5, arrows_on=True, arrow_steps=20, arrow_len=0.3, label="SLAM Estimation")
    plot_trajectory(plt, upf_logger.connected_agent.x_real, upf_logger.connected_agent.h_real, color="k", alpha=1., style="-", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="True Anchor location")
    plot_trajectory(plt, upf_logger.host_agent.x_real, upf_logger.host_agent.h_real, color="b", alpha=1., style="-", size=1.5, arrows_on=False, arrow_steps=20, arrow_len=0.3, label="Robot Trajectory")

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend(loc='upper left')
    plt.show()

