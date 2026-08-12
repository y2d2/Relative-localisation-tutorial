"""Trajectory generation and noise helpers."""

import numpy as np

from arp_utility_code.Simulation.Trajectories import Trajectory


def generate_trajectory(duration: float = 100.0, timestep: float = 0.1, seed: int | None = None):
    """Generate a 2D trajectory using ARP_utility_code's trajectory generator.
    
    Args:
        duration: Total simulation time in seconds
        timestep:Sampling timestep in seconds
        seed: Random seed for reproducibility (optional)
    
    Returns:
        Trajectory object with SGAL3 objects at each timestep
    """
    if seed is not None:
        np.random.seed(seed)
    
    from arp_utility_code.Simulation.Trajectories_Generator import trajectory_generator, random_starting_conditions
    
    T_OR, v_R0 = random_starting_conditions()
    
    traj = trajectory_generator(
        T_OR=T_OR,
        v_R0=v_R0,
        trajectory_time=duration,
        dt=timestep,
        l=10,
        b=10,
        z=2,
        dict_of_sensors={},
        plt_bool=False,
        control_type='2D'
    )
    
    return traj


def sgal3_to_se2_poses(sgal3_poses):
    """Convert list of SGAL3 objects to SE(2) pose array [x, y, theta].
    
    For planar motion, extracts x, y from translation and yaw from rotation matrix.
    
    Args:
        sgal3_poses: List of SGAL3 objects
    
    Returns:
        poses: (N, 3) array with columns [x, y, theta]
    """
    n = len(sgal3_poses)
    poses = np.zeros((n, 3))
    
    for i, state in enumerate(sgal3_poses):
        pos = state.translation()
        poses[i, 0] = pos[0]
        poses[i, 1] = pos[1]
        
        R = state.rotation()
        poses[i, 2] = np.arctan2(R[1, 0], R[0, 0])
    
    return poses


def add_noise(poses: np.ndarray, sigma_x: float = 0.1, sigma_y: float = 0.1, sigma_theta: float = 0.05):
    """Add Gaussian noise to SE(2) poses.
    
    Args:
        poses: (N, 3) array with columns [x, y, theta]
        sigma_x: Standard deviation for x position noise
        sigma_y: Standard deviation for y position noise
        sigma_theta: Standard deviation for orientation noise
    
    Returns:
        noisy_poses: (N, 3) array with noisy poses
    """
    noisy_poses = poses.copy()
    
    n = poses.shape[0]
    noisy_poses[:, 0] += np.random.normal(0, sigma_x, n)
    noisy_poses[:, 1] += np.random.normal(0, sigma_y, n)
    noisy_poses[:, 2] += np.random.normal(0, sigma_theta, n)
    
    return noisy_poses
