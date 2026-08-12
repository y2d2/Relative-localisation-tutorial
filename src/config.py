"""Configuration parameters for localization notebooks."""

from dataclasses import dataclass


@dataclass
class TrajectoryConfig:
    """Parameters for trajectory generation."""
    duration: float = 100.0
    timestep: float = 0.1
    seed: int | None = None


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
    """Parameters for UWB range measurement noise (Gaussian)."""
    sigma_range: float = 0.1
