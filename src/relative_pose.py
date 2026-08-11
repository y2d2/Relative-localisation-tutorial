"""Direct relative pose computation from two GT-tracked robots.

For each timestep t:
    ^A T_B = (^O T_A)^{-1} \u00b7 ^O T_B

Both ground-truth poses are read directly from Trajectory.X_OR states — this is
equivalent to having a noise-free vicon/GT tracker on each robot.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class RelativePoseResult:
    """Container for computed relative pose between robots A and B."""
    time: np.ndarray         # (N,) shared timestamps
    rel_pos: np.ndarray      # (N, 3) relative position in A's frame
    rel_orient: np.ndarray   # (N, 3) rotation vector in A's frame
    rel_transforms: list[np.ndarray]  # (N,) list of (4,4) SE(3) matrices


def compute_relative_pose(traj_a, traj_b):
    """Compute the relative pose of B with respect to A at every timestep.

    Reads global poses directly from Trajectory.X_OR states — this is
    equivalent to having a noise-free vicon/GT tracker on each robot.

    Parameters
    ----------
    traj_a : Trajectory
        Ground-truth trajectory of Robot A.
    traj_b : Trajectory
        Ground-truth trajectory of Robot B.

    Returns
    -------
    RelativePoseResult
    """
    N = len(traj_a.X_RRi)
    rel_pos = np.zeros((N, 3))
    rel_orient = np.zeros((N, 3))
    rel_transforms = [np.eye(4)] * N

    for i in range(N):
        T_OA = _se3(traj_a, i)
        T_OB = _se3(traj_b, i)
        T_AB = _inv_se3(T_OA) @ T_OB
        rel_transforms[i] = T_AB
        rel_pos[i, :] = T_AB[:3, 3]

        R_rel = T_AB[:3, :3]
        cos_theta = (np.trace(R_rel) - 1.0) / 2.0
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        theta = np.arccos(cos_theta)

        if abs(theta) < 1e-15:
            rel_orient[i, :] = np.zeros(3)
        else:
            S = (R_rel - R_rel.T) / (2.0 * np.sin(theta))
            rel_orient[i, :] = np.array([S[2, 1], S[0, 2], S[1, 0]]) * theta

    return RelativePoseResult(
        time=np.array(traj_a.t),
        rel_pos=rel_pos,
        rel_orient=rel_orient,
        rel_transforms=rel_transforms,
    )


def compute_noisy_relative_pose(traj_a, traj_b, sig_pos=0.01, sig_orient=0.05):
    """Compute relative pose with Gaussian noise injected into each GT measurement.

    Simulates a noisy vicon/GT tracker: position and orientation readings
    are corrupted with independent Gaussian noise before computing the
    relative pose.

    Parameters
    ----------
    traj_a : Trajectory
        Ground-truth trajectory of Robot A (used for truth).
    traj_b : Trajectory
        Ground-truth trajectory of Robot B (used for truth).
    sig_pos : float
        Position noise std-dev (m) per axis.
    sig_orient : float
        Orientation noise std-dev (rad) per Euler component.

    Returns
    -------
    RelativePoseResult
    """
    N = len(traj_a.X_RRi)
    rel_pos = np.zeros((N, 3))
    rel_orient = np.zeros((N, 3))
    rel_transforms = [np.eye(4)] * N

    for i in range(N):
        T_OA = _se3(traj_a, i)
        T_OB = _se3(traj_b, i)

        if sig_pos > 0 or sig_orient > 0:
            T_OA = _inject_noise(T_OA, sig_pos, sig_orient)
            T_OB = _inject_noise(T_OB, sig_pos, sig_orient)

        T_AB = _inv_se3(T_OA) @ T_OB
        rel_transforms[i] = T_AB
        rel_pos[i, :] = T_AB[:3, 3]

        R_rel = T_AB[:3, :3]
        cos_theta = (np.trace(R_rel) - 1.0) / 2.0
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        theta = np.arccos(cos_theta)

        if abs(theta) < 1e-15:
            rel_orient[i, :] = np.zeros(3)
        else:
            S = (R_rel - R_rel.T) / (2.0 * np.sin(theta))
            rel_orient[i, :] = np.array([S[2, 1], S[0, 2], S[1, 0]]) * theta

    return RelativePoseResult(
        time=np.array(traj_a.t),
        rel_pos=rel_pos,
        rel_orient=rel_orient,
        rel_transforms=rel_transforms,
    )


def _se3(traj, index):
    """Extract SE(3) pose from a Trajectory at the given index."""
    from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix

    X_global = traj.X_OR.compose(traj.X_RRi[index])
    return sgal3_to_se3_matrix(X_global)


def _inject_noise(T, sig_pos=0.01, sig_orient=0.05):
    """Inject independent Gaussian noise into position and rotation of SE(3).

    Uses manifold-aware perturbation for the rotation component to stay on SO(3).
    """
    T_out = T.copy()
    pos = T[:3, 3].copy()
    T_out[:3, 3] = pos + np.random.randn(3) * sig_pos

    if sig_orient > 0:
        from scipy.spatial.transform import Rotation as R
        rand_vec = np.random.randn(3) * sig_orient
        r_new = R.from_matrix(T_out[:3, :3]) * R.from_rotvec(rand_vec)
        T_out[:3, :3] = r_new.as_matrix()

    return T_out


def _inv_se3(T):
    """Invert a homogeneous SE(3) matrix."""
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def position_error(result):
    """Compute relative position magnitude error from a RelativePoseResult."""
    return np.linalg.norm(result.rel_pos, axis=1)


def display_relative_pose(result, rel_offset=None, num_rows=8):
    """Pretty-print representative rows of a RelativePoseResult.

    Parameters
    ----------
    result : RelativePoseResult
        Output of compute_relative_pose() or compute_noisy_relative_pose().
    rel_offset : np.ndarray or None
        Expected relative position for an absolute-error column.
    num_rows : int
        Number of representative timesteps to show (evenly spaced).
    """
    n = len(result.time)
    step = max(1, n // num_rows)

    header = "Time (s) | Rel Pos [m]                | Rel Rot norm (rad)"
    if rel_offset is not None:
        header += "  | Abs-Pos err (m)"
    print(header)
    print('-' * max(len(header), 70))

    for i in range(0, n, step):
        t = result.time[i]
        pos = result.rel_pos[i]
        orient_norm = np.linalg.norm(result.rel_orient[i])
        line = f'{t:6.1f} | [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]  | {orient_norm:12.6f}'
        if rel_offset is not None:
            err = np.linalg.norm(pos - rel_offset)
            line += f'  | {err:9.2e}'
        print(line)
