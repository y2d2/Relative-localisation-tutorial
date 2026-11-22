import numpy as np
import pytest

from ukf_wrappers import TargetTrackingUKF_2D


def test_2d_initialization():
    ukf = TargetTrackingUKF_2D()
    ukf.set_ukf_properties()
    
    t_j = np.array([10.0, np.pi/4, 0.0])
    sigma_t = np.array([0.1, 0.01, 0.001])
    
    ukf.set_initial_state(t_j, sigma_t)
    
    assert abs(ukf.kf.x[2]) < 1e-10
    assert abs(ukf.kf.P[2, 2] - (1e-6) ** 2) < 1e-12


def test_2d_trajectory_stays_in_plane():
    ukf = TargetTrackingUKF_2D()
    ukf.set_ukf_properties(alpha=1, beta=2, kappa=-1)
    
    t_j = np.array([5.0, 0.0, 0.0])
    sigma_t = np.array([0.1, 0.01, 0.001])
    ukf.set_initial_state(t_j, sigma_t)
    
    dt_j = np.array([1.0, 0.0, 0.0, 0.1])
    q_j = np.diag([0.01 ** 2, 0.01 ** 2, 0.01 ** 2, 0.001 ** 2])
    t_i = np.array([0.0, 0.0, 0.0, 0.0])
    P_i = np.diag([0.1 ** 2, 0.01 ** 2, 0.01 ** 2, 0.001 ** 2])
    d_ij = 5.0
    sig_d = 0.1
    
    z_vals = []
    for _ in range(10):
        ukf.run_filter(dt_j, q_j, t_i, P_i, d_ij, sig_d)
        z_vals.append(ukf.t_si_sj[2])
    
    assert max(abs(z) for z in z_vals) < 1e-3


def test_2d_covariance_shape():
    ukf = TargetTrackingUKF_2D()
    ukf.set_ukf_properties()
    
    t_j = np.array([10.0, np.pi/4, 0.0])
    sigma_t = np.array([0.1, 0.01, 0.001])
    ukf.set_initial_state(t_j, sigma_t)
    
    assert ukf.P_t_si_sj.shape == (3, 3)
