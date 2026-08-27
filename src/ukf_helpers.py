import numpy as np

from src._2d_upf import TargetTrackingUKF_2D


def estimate_beacon_with_upf(poses_noisy, ranges, anchor_pos, vio_cfg, uwb_cfg):
    """
    Estimate beacon position using UPF tracking.
    
    Args:
        poses_noisy: (N, 3) array of robot poses [x, y, theta]
        ranges: (N, 1) array of range measurements to beacon
        anchor_pos: (2,) true beacon position
        vio_cfg: VIOConfig for odometry noise
        uwb_cfg: UWBNoiseConfig for range noise
        
    Returns:
        beacon_states: (N, 3) array of UKF beacon estimates [x, y, h] in robot frame
        covariances: (N, 3, 3) array of covariance matrices
    """
    n_steps = len(poses_noisy)
    
    beacon_states = np.zeros((n_steps, 3))
    covariances = np.zeros((n_steps, 3, 3))
    
    robot_prev = poses_noisy[0]
    
    ukf = TargetTrackingUKF_2D(x_ha_0=np.zeros(4))
    ukf.set_ukf_properties()
    
    initial_beacon_rel = np.array([
        anchor_pos[0] - robot_prev[0],
        anchor_pos[1] - robot_prev[1],
        0.0
    ])
    
    r_init = np.sqrt(initial_beacon_rel[0]**2 + initial_beacon_rel[1]**2)
    theta_init = np.arctan2(initial_beacon_rel[1], initial_beacon_rel[0])
    h_init = 0.0
    
    t_j_ukf = np.array([r_init, theta_init, h_init])
    sigma_t = np.array([10.0, 0.5, 1.0])
    
    ukf.set_initial_state(t_j_ukf, sigma_t)
    
    sig_d = uwb_cfg.sig_range
    
    dt_odom = vio_cfg.timestep if hasattr(vio_cfg, 'timestep') else 0.05
    q_odom = np.eye(4) * (vio_cfg.sig_v ** 2)
    
    for i in range(n_steps):
        if i > 0:
            robot_curr = poses_noisy[i]
            
            dx = robot_curr[0] - robot_prev[0]
            dy = robot_curr[1] - robot_prev[1]
            dtheta = (robot_curr[2] - robot_prev[2] + np.pi) % (2 * np.pi) - np.pi
            
            dt_j = np.array([dx, dy, 0.0, dtheta])
            
            t_i = np.zeros(4)
            P_i = q_odom
            d_ij = ranges[i, 0] if ranges.shape[1] == 1 else ranges[i]
            
            ukf.run_filter(dt_j, q_odom, t_i, P_i, d_ij, sig_d, update_bool=True)
            
            robot_prev = robot_curr
        
        beacon_states[i] = ukf.t_si_sj
        covariances[i] = ukf.P_t_si_sj
    
    return beacon_states, covariances
