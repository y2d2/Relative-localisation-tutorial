import numpy as np
import matplotlib.pyplot as plt

from arp_utility_code.Simulation.Trajectories import Trajectory
from arp_utility_code.UtilityCode import Transformation_Matrix_Fucntions as TMF

# from src._2d_upf import UPFConnectedAgent_2D
from upf_rte.DataLoggers.ConnectedAgent_DataLogger import UPFConnectedAgentDataLogger, UKFDatalogger
from upf_rte.Simulation.RobotClass import NewRobot
from src.vio_sensor import simulate_vio_odometry
from src.part1_functions import get_pose
from trajectory import generate_trajectory, sgal3_to_se2_poses
from src.part3_functions import get_noisy_vio_trajectory
from _2D_UPF import UPF_2D
from src.part2_functions import simulate_uwb_ranges
from src.part4_functions import poses_to_transformation_matrices, plot_upf_2d

def uwb_ranges(perfect_poses_1, perfect_poses_2, uwb_conf):
    dif = [pose1 - pose2 for pose1, pose2 in zip(perfect_poses_1, perfect_poses_2)]
    d = [np.sqrt(np.sum(np.square(p))) for p in dif]
    noise = np.random.normal(0, uwb_conf.sig_range, np.array(d).shape)
    d = np.array(d) + noise
    return d

def active_relative_pose(traj_conf1, traj_conf2, vio_conf, uwb_conf, n_azimuth, n_heading):
    perfec_poses_1, noisy_poses_1 = get_noisy_vio_trajectory(traj_conf1, vio_conf)
    perfec_poses_2, noisy_poses_2 = get_noisy_vio_trajectory(traj_conf2, vio_conf)
    uwb_range = uwb_ranges(perfec_poses_1, perfec_poses_2, uwb_conf)

    n_steps = len(uwb_range)

    q_vio= np.diag([
        (vio_conf.sig_v * traj_conf1.timestep) ** 2,
        (vio_conf.sig_v * traj_conf1.timestep) ** 2,
        1e-16,
        (vio_conf.sig_w * traj_conf1.timestep) ** 2
    ])
    perfec_T_list_1 = poses_to_transformation_matrices(perfec_poses_1)
    T_noisy_list_1 = poses_to_transformation_matrices(noisy_poses_1)
    DT_noisy_list_1 = [TMF.inv_transformation_matrix(T_noisy_list_1[i]) @ T_noisy_list_1[i + 1] for i in
                     range(len(T_noisy_list_1) - 1)]
    DT_noisy_list_1.insert(0, np.eye(4))

    ha = NewRobot()
    ha.from_experimental_data(perfec_T_list_1, DT_noisy_list_1, q_vio, int(1 / traj_conf1.timestep))

    perfec_T_list_2 = poses_to_transformation_matrices(perfec_poses_2)
    T_noisy_list_2 = poses_to_transformation_matrices(noisy_poses_2)
    DT_noisy_list_2 = [TMF.inv_transformation_matrix(T_noisy_list_2[i]) @ T_noisy_list_2[i + 1] for i in
                       range(len(T_noisy_list_2) - 1)]
    DT_noisy_list_2.insert(0, np.eye(4))

    ca = NewRobot()
    ca.from_experimental_data(perfec_T_list_2, DT_noisy_list_2, q_vio, int(1 / traj_conf2.timestep))
    ha_t = np.append(ha.x_slam[0], np.array(ha.h_slam[0]))

    upf = UPF_2D(x_ha_0=ha_t, drift_correction_bool=False)
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.set_initalisation_parameters(sigma_uwb= uwb_conf.sig_range, n_azimuth=n_azimuth, n_heading=n_heading,
                                      dz=0, sigma_dz=0.0001)
    upf.set_regeneration_parameters(regeneration= False)
    upf.min_likelihood = 0.9

    upf_logger = UPFConnectedAgentDataLogger(ha, ca, upf)

    for i in range(n_steps):
        ha_t = np.append(ha.x_slam[i], np.array(ha.h_slam[i]))
        upf.ha.set_x_ha(ha_t)
        upf.ha.p_x_ha = q_vio
        dt = TMF.get_4D_t_from_matrix(DT_noisy_list_2[i])

        upf.run_2d_model(dt, q_vio, uwb_range[i])
        upf_logger.log_data(i)

    plot_upf_2d(upf_logger)
    print(len(upf.particles))