"""Trajectory creation and scenario building for Part 1 global localisation."""

import numpy as np
from arp_utility_code.Simulation.Trajectories_Generator import trajectory_generator

from src.config import GlobalLocalizationConfig


def create_gt_trajectory(
    config: GlobalLocalizationConfig,
    offset_X=0.0,
    offset_Y=0.0,
    seed=None,
):
    """Create a ground-truth trajectory using ARP velocity control in 2D.

    Parameters
    ----------
    config : GlobalLocalizationConfig
        Simulation parameters (time, dt, room dimensions).
    offset_X, offset_Y : float
        Initial X-Y offset applied to the trajectory's base pose so that
        two robots can follow *identical* motion but start at different
        global positions. This is what Part 1 exercises: computing the
        fixed relative transform between them.
    seed : int or None
        Random seed for the velocity control loop (for reproducibility).

    Returns
    -------
    Trajectory
        Ground-truth trajectory container from arp_utility_code.
    """
    import random
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    T_init = np.eye(4)
    T_init[:3, 3] += np.array([offset_X, offset_Y, 0.0])

    traj = trajectory_generator(
        T_OR=T_init,
        v_R0=np.zeros(3),
        trajectory_time=config.time,
        dt=config.dt,
        l=config.room_l,
        b=config.room_b,
        z=1.0,
        dict_of_sensors={},
        plt_bool=False,
        control_type="2D",
    )
    return traj


def build_two_robot_scenario(config: GlobalLocalizationConfig):
    """Build a two-robot scenario sharing the same ground-truth trajectory pattern.

    Robot-A and Robot-B start at different positions (offset by `rel_offset`) but
    follow identical motion profiles — their relative pose is constant throughout
    the simulation, which Part 1 verifies analytically via ^A T_B = (^O T_A)^-1 ^O T_B.

    Parameters
    ----------
    config : GlobalLocalizationConfig
        Simulation parameters.

    Returns
    -------
    dict
        ``{
            'gt_traj_a': Trajectory,       # Robot A ground-truth
            'gt_traj_b': Trajectory,       # Robot B ground-truth (offset start)
            'rel_offset': np.ndarray(3),    # constant relative position [x,y,z]_A frame
        }``
    """
    rel_offset = np.array([2.0, 1.5, 0.0])

    traj_a = create_gt_trajectory(config, offset_X=0.0, offset_Y=0.0, seed=42)
    traj_b = create_gt_trajectory(
        config, offset_X=rel_offset[0], offset_Y=rel_offset[1], seed=42
    )

    return {
        "gt_traj_a": traj_a,
        "gt_traj_b": traj_b,
        "rel_offset": rel_offset,
    }
