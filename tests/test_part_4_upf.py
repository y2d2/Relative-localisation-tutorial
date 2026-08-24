"""Tests for part 4 UPF estimator."""

import numpy as np

from config import VIOConfig, UWBNoiseConfig, TrajectoryConfig
from part4_functions import estimnate_robot_location_with_beacon
from trajectory import generate_trajectory


def test_particle_count():
    """Verify particle count with configured parameters."""
    # Generate trajectory
    traj_conf = TrajectoryConfig(duration=2.0, timestep=0.05, seed=42)
    # trajectory = generate_trajectory(duration=1.0, timestep=0.05, seed=42)
    anchor_pos = np.array([5.0, 5.0])
    
    vio_cfg = VIOConfig(sig_v=0.1, sig_w=0.05)
    uwb_cfg = UWBNoiseConfig(sig_range=0.1)
    
    # Run with configured particle count
    states, covariances, t_seq = estimnate_robot_location_with_beacon(
        traj_conf, vio_cfg, uwb_cfg, anchor_pos,
        n_azimuth=4, n_heading=4
    )
    


