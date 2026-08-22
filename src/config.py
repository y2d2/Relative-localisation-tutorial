"""Configuration parameters for localization notebooks."""

from dataclasses import dataclass


@dataclass
class TrajectoryConfig:
    """Parameters for trajectory generation."""
    duration: float = 100.0
    timestep: float = 0.1
    seed: int | None = None
    start_x: float = 0.0
    start_y: float = 0.0
    start_theta: float = 0.0


@dataclass
class NoiseConfig:
    """Parameters for noise addition (Gaussian)."""
    sigma_x: float = 0.1
    sigma_y: float = 0.1
    sigma_theta: float = 0.05


@dataclass
class UWBAnchorConfig:
    """Parameters for UWB anchor positions."""
    anchor_positions: list[tuple[float, float]]


@dataclass
class UWBNoiseConfig:
    """Parameters for UWB range and bearing measurement noise (Gaussian)."""
    sigma_range: float = 0.1
    sigma_bearing: float = 0.02


@dataclass
class VIOConfig:
    """Parameters for VIO/odometry sensor noise (Gaussian on velocity and angular velocity)."""
    sig_v: float = 0.1   # Velocity noise std (m/s)
    sig_w: float = 0.05  # Angular velocity noise std (rad/s)


@dataclass
class EKFConfig:
    """Parameters for SE(2) EKF."""
    sigma_range_init: float = 10.0  # Initial position uncertainty (m)
    dt_odom: float = 0.05           # Odometry update period (20 Hz)
    dt_uwb: float = 0.1             # UWB update period (10 Hz)
