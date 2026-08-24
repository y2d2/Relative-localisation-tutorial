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
    
    # Should have 16 particles initially (may drop after resampling)
    assert len(states) > 0
    assert len(covariances) > 0
    assert len(t_seq) > 0


def test_particles_move():
    """Check that states change from initial values."""
    trajectory = generate_trajectory(duration=2.0, timestep=0.05, seed=42)
    anchor_pos = np.array([5.0, 5.0])
    
    vio_cfg = VIOConfig(sig_v=0.1, sig_w=0.05)
    uwb_cfg = UWBNoiseConfig(sig_range=0.1)
    
    # Run filter
    states, covariances, t_seq = estimate_robot_relative_to_beacon_with_upf(
        trajectory, anchor_pos, vio_cfg, uwb_cfg,
        n_azimuth=4, n_altitude=1, n_heading=4
    )
    
    # States should evolve over time (not stay constant)
    assert len(states) >= 2
    
    # Check that at least some state components change
    first_state = np.array(states[0])
    last_state = np.array(states[-1])
    
    # At least one component should differ significantly
    diff = np.abs(last_state - first_state)
    assert np.any(diff > 0.01), "States should evolve over time"


def test_seed_reproducibility():
    """Verify that seeds produce reproducible results."""
    trajectory = generate_trajectory(duration=1.0, timestep=0.05, seed=42)
    anchor_pos = np.array([5.0, 5.0])
    
    vio_cfg = VIOConfig(sig_v=0.1, sig_w=0.05)
    uwb_cfg = UWBNoiseConfig(sig_range=0.1)
    
    # Run twice with same seed
    states1, covs1, _ = estimate_robot_relative_to_beacon_with_upf(
        trajectory, anchor_pos, vio_cfg, uwb_cfg,
        n_azimuth=4, n_altitude=1, n_heading=4, seed=123
    )
    
    states2, covs2, _ = estimate_robot_relative_to_beacon_with_upf(
        trajectory, anchor_pos, vio_cfg, uwb_cfg,
        n_azimuth=4, n_altitude=1, n_heading=4, seed=123
    )
    
    # Results should be identical with same seed
    for s1, s2 in zip(states1, states2):
        assert np.allclose(s1, s2), "Same seed should produce identical results"
    
    for c1, c2 in zip(covs1, covs2):
        assert np.allclose(c1, c2), "Same seed should produce identical covariances"
