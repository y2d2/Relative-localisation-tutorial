"""Central constants shared across all parts of the relative localisation tutorial."""

from dataclasses import dataclass

import numpy as np

# Default simulation parameters
DEFAULT_TIME: float = 100.0
DEFAULT_DT: float = 0.1
DEFAULT_SIG_GT_POS: float = 0.0
DEFAULT_SIG_GT_ORIENT: float = 0.0
NUM_ROBOTS: int = 2


@dataclass
class GlobalLocalizationConfig:
    """Configuration for Part 1 global localisation."""

    time: float = DEFAULT_TIME
    dt: float = DEFAULT_DT
    sig_gt_pos: float = DEFAULT_SIG_GT_POS
    sig_gt_orient: float = DEFAULT_SIG_GT_ORIENT
    room_l: float = 10.0
    room_b: float = 10.0
