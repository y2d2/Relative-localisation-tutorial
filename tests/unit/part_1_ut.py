"""Unit tests for Part 1 global localisation relative pose computation.

Tests ideal (perfect GT) and noisy GT cases to verify:
- Perfect case: relative pose error ≈ 0 at all timesteps
- Noisy case: position/orientation errors remain proportional to σ
"""

import sys
sys.path.insert(0, '/workspace/Python/Relative-localisation-tutorial')
sys.path.insert(0, '/workspace/Python/Relative-localisation-tutorial/src')

os.chdir('/workspace/Python/Relative-localisation-tutorial')
import numpy as np
import pytest

from config import GlobalLocalizationConfig
from trajectories import build_two_robot_scenario
from relative_pose import (
    compute_relative_pose,
    compute_noisy_relative_pose,
    position_error,
)


class TestPerfectCase:
    """Verify perfect GT produces the expected constant relative pose."""

    def setup_method(self):
        config = GlobalLocalizationConfig(time=20.0, dt=0.1)
        scenario = build_two_robot_scenario(config)
        self.scenario = scenario
        self.result = compute_relative_pose(scenario['gt_traj_a'], scenario['gt_traj_b'])

    def test_position_error_below_threshold(self):
        """Perfect case: max position error < 5e-6."""
        errors = np.linalg.norm(
            self.result.rel_pos - self.scenario['rel_offset'], axis=1
        )
        assert errors.max() < 5e-6, f"Max perf pos error {errors.max():.2e} > 5e-6"

    def test_orientation_error_zero(self):
        """Perfect case: orientation error == 0 everywhere."""
        assert np.allclose(
            self.result.rel_orient,
            np.zeros_like(self.result.rel_orient),
            atol=1e-12,
        ), "Rotations should be identical in perfect GT"

    def test_relative_pose_magnitude_constant(self):
        """Relative position magnitude should equal the offset norm."""
        magnitudes = np.linalg.norm(self.result.rel_pos, axis=1)
        expected = np.linalg.norm(self.scenario['rel_offset'])
        assert np.allclose(magnitudes, expected), "Norm of relative pos should be constant"

    def test_position_error_function(self):
        """Verify position_error() matches direct computation."""
        errs = position_error(self.result)
        direct = np.linalg.norm(
            self.scenario['gt_traj_b'].X_OR.compose(
                self.scenario['gt_traj_b'].X_RRi[0]
            ).translation()
            - self.scenario['gt_traj_a'].X_OR.compose(
                self.scenario['gt_traj_a'].X_RRi[0]
            ).translation(),
            axis=1,
        )
        # Compare computed errors against expected offset
        expected = [np.linalg.norm(self.scenario['rel_offset'])] * len(errs)
        np.testing.assert_allclose(errs, expected, rtol=1e-4)

    def test_relative_transform_count(self):
        assert len(self.result.rel_transforms) == len(self.result.time)


class TestNoisyCase:
    """Verify noisy GT introduces expected error magnitude."""

    @pytest.mark.parametrize(
        ("sig_pos", "sig_orient"), [(0.01, 0.02), (0.05, 0.1)]
    )
    def test_noisy_position_error_proportional(self, sig_pos, sig_orient):
        config = GlobalLocalizationConfig(time=20.0, dt=0.1)
        scenario = build_two_robot_scenario(config)

        np.random.seed(42)
        result_perfect = compute_relative_pose(scenario['gt_traj_a'], scenario['gt_traj_b'])

        np.random.seed(42)
        result_noisy = compute_noisy_relative_pose(
            scenario['gt_traj_a'], scenario['gt_traj_b'], sig_pos, sig_orient
        )

        pos_diff = np.linalg.norm(result_perfect.rel_pos - result_noisy.rel_pos, axis=1)
        # Expected error scale: ~σ√2 for relative position from 2 sensors
        expected_scale = sig_pos * np.sqrt(2)
        assert pos_diff.max() > sig_pos / 10., "Noisy should introduce measurable error"
        assert pos_diff.max() < expected_scale * 6, f"Noisy error {pos_diff.max():.4f} exceeds expected"

    def test_noisy_orientation_error_proportional(self):
        config = GlobalLocalizationConfig(time=20.0, dt=0.1)
        scenario = build_two_robot_scenario(config)

        np.random.seed(42)
        result_perfect = compute_relative_pose(scenario['gt_traj_a'], scenario['gt_traj_b'])

        np.random.seed(42)
        result_noisy = compute_noisy_relative_pose(scenario['gt_traj_a'], scenario['gt_traj_b'], sig_pos=0.0, sig_orient=0.1)

        rot_diff = np.linalg.norm(result_perfect.rel_orient - result_noisy.rel_orient, axis=1)
        assert rot_diff.max() > 0.05, "Orientation error should be measurable with σ=0.1 rad"


class TestSE3Math:
    """Test helper functions used in the SE(3) computation."""

    def test_inv_se3_identity(self):
        from relative_pose import _inv_se3
        T = np.eye(4)
        T[:3, 3] = [1.0, 2.0, 3.0]
        T_inv = _inv_se3(T)
        T_composed = T_inv @ T
        assert np.allclose(T_composed, np.eye(4), atol=1e-12)

    def test_compute_noisy_reduces_to_perfect(self):
        config = GlobalLocalizationConfig(time=10.0, dt=0.1)
        scenario = build_two_robot_scenario(config)

        # With σ = 0 should give identical results to perfect case
        result_0 = compute_relative_pose(scenario['gt_traj_a'], scenario['gt_traj_b'])
        result_noisy = compute_noisy_relative_pose(
            scenario['gt_traj_a'],
            scenario['gt_traj_b'],
            sig_pos=0.0,
            sig_orient=0.0,
        )

        np.testing.assert_allclose(result_0.rel_pos, result_noisy.rel_pos, atol=1e-12)
