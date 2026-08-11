"""GTSensor setup helpers for Part 1 global localisation.

Since Part 1 deals with perfect GT knowledge via vicon-like tracking,
GT poses are read directly from Trajectory.X_OR states (the "vicon truth")
rather than through OdometrySensor/UWB chains. This module exists to document
the separation and provide the noisy-GT wrapper where needed for the
error-propagation analysis in Part 1.
"""

import numpy as np
from arp_utility_code.Simulation.Robot import RobotSimulation
from arp_utility_code.UtilityCode.Sensors import GTSensor
from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_from_se3_matrix


class NoisyGTSensor(GTSensor):
    """GTSensor that adds Gaussian noise to each global pose measurement.

    This simulates a vicon/GT tracker with non-zero positional and
    orientational uncertainty. In noise-free Part 1, sig_pos and
    sig_orient are both 0.0 so measurements are exact.
    """

    def __init__(self, T_OR, v_RR0, time, sig_pos=0.0, sig_orient=0.0):
        super().__init__(T_OR, v_RR0, time)
        self.sig_pos = float(sig_pos)
        self.sig_orient = float(sig_orient)

    def get_new_measurement(self, t, q, time):
        """Return noisy t, q instead of the original."""
        t_noisy = t + np.random.randn(3) * self.sig_pos
        q_noisy = q + np.random.randn(3) * self.sig_orient
        super().get_new_measurement(t_noisy, q_noisy, time)


def create_gt_sensor(robot: RobotSimulation):
    """Create a GTSensor at the robot's initial global pose.

    The sensor is instantiated but NOT auto-primed by RobotSimulation.do_step()
    since GTSensor is not an OdometrySensor subclass — its trajectory must
    be seeded manually before stepping.

    Parameters
    ----------
    robot : RobotSimulation
        Robot whose base_link GT pose we want to track.

    Returns
    -------
    GTSensor
        Initialized at the first pose of the robot's GT trajectory.
    """
    from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix

    T_init = sgal3_to_se3_matrix(robot.trajectory.X_OR)
    gt_sensor = GTSensor(
        sgal3_from_se3_matrix(T_init),
        np.zeros(3),
        time=robot.trajectory.t[0],
    )
    return gt_sensor


def create_noisy_gt_sensor(robot: RobotSimulation, sig_pos=0.01, sig_orient=0.05):
    """Create a noisy GTSensor at the robot's initial global pose.

    Parameters
    ----------
    robot : RobotSimulation
        Robot whose base_link GT pose we want to track.
    sig_pos : float
        Position noise std-dev (m) per axis.
    sig_orient : float
        Orientation noise std-dev (rad) per Euler component.

    Returns
    -------
    NoisyGTSensor
    """
    from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix

    T_init = sgal3_to_se3_matrix(robot.trajectory.X_OR)
    sensor = NoisyGTSensor(
        sgal3_from_se3_matrix(T_init),
        np.zeros(3),
        time=robot.trajectory.t[0],
        sig_pos=sig_pos,
        sig_orient=sig_orient,
    )
    return sensor
