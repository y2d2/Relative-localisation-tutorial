"""Tests for part 4 UPF estimator."""

import numpy as np

from config import VIOConfig, UWBNoiseConfig, TrajectoryConfig
from part4_functions import estimnate_robot_location_with_beacon, estimate_anchor_from_robot
from trajectory import generate_trajectory


def test_upf_estimation_runs():
    """Verify that the UPF estimation runs without errors and produces plots."""
    traj_conf = TrajectoryConfig(duration=50.0, timestep=0.1, seed=42)
    anchor_pos = np.array([5.0, 5.0])
    
    vio_cfg = VIOConfig(sig_v=0.1, sig_w=0.05)
    uwb_cfg = UWBNoiseConfig(sig_range=0.1)
    
    estimnate_robot_location_with_beacon(
        traj_conf, vio_cfg, uwb_cfg, anchor_pos,
        n_azimuth=4, n_heading=4
    )
    

def test_anchor_from_robot_runs():
    """Verify that the anchor estimation from robot runs without errors."""
    traj_conf = TrajectoryConfig(duration=50.0, timestep=0.1, seed=42)
    anchor_pos = np.array([5.0, 5.0])
    
    vio_cfg = VIOConfig(sig_v=0.1, sig_w=0.05)
    uwb_cfg = UWBNoiseConfig(sig_range=0.1)
    
    estimate_anchor_from_robot(
        traj_conf, vio_cfg, uwb_cfg, anchor_pos,
        n_azimuth=4, n_heading=4
    )
    


