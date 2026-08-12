"""VIO sensor simulation for SLAM uncertainty accumulation."""

import numpy as np

from arp_utility_code.Simulation.Trajectories import Trajectory
from arp_utility_code.UtilityCode.Sensors import Vio


def simulate_vio_odometry(traj, vio_config):
    """Simulate VIO odometry along a trajectory with Gaussian velocity/angularvelocity noise."""
    from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix
    
    sig_v = vio_config.sig_v
    sig_w = vio_config.sig_w
    
    initial_state = traj.X_RRi[0]
    
    T_init = sgal3_to_se3_matrix(initial_state)
    v_init = np.zeros(3)
    
    sensor = Vio(T_init, v_init, 0.0)
    
    for a, v, w, t in traj.stream_body_frame_measurements():
        v_noise = v + np.random.randn(3) * sig_v
        w_noise = w + np.random.randn(3) * sig_w
        
        v_noise[2] = 0
        w_noise[0:2] = 0
        
        sensor.get_new_measurement(None, v_noise, w_noise, t)
    
    odometry_traj = sensor.trajectory
    
    n_steps = len(odometry_traj.t)
    poses_noisy = np.zeros((n_steps, 3))
    
    for i in range(n_steps):
        state = odometry_traj.X_RRi[i]
        pos = state.translation()
        poses_noisy[i, 0] = pos[0]
        poses_noisy[i, 1] = pos[1]
        
        R = state.rotation()
        poses_noisy[i, 2] = np.arctan2(R[1, 0], R[0, 0])
    
    return poses_noisy, odometry_traj
