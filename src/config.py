"""Configuration parameters for part_1 notebook."""

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
