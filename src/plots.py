"""One-call visualization helpers for Part 1 global localisation."""

import numpy as np
from matplotlib import pyplot as plt


def _positions_for_arrows(traj_a, traj_b=None):
    """Return (pos_a, pos_b) pairs suitable for arrow drawing on XY plot.

    Trajectories may carry ``X_OR`` / ``X_RRi`` (SE(3)-manifold style) or
    pre-computed position arrays via ``.positions``.  This function detects
    whichever is present and returns an ``(N, 3)`` array per robot.
    """
    if hasattr(traj_a, 'X_OR') and hasattr(traj_a, 'X_RRi'):
        from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix
        N = len(traj_a.X_RRi)
        pos_a = np.empty((N, 3))
        for i in range(N):
            X = traj_a.X_OR.compose(traj_a.X_RRi[i])
            pos_a[i] = sgal3_to_se3_matrix(X)[:3, 3]
        positions = (pos_a,)
    else:
        positions = (np.asarray(traj_a),)

    if traj_b is not None and hasattr(traj_b, 'X_OR'):
        from arp_utility_code.UtilityCode.Manifpy_helper import sgal3_to_se3_matrix
        N = len(traj_b.X_RRi)
        pos_b = np.empty((N, 3))
        for i in range(N):
            X = traj_b.X_OR.compose(traj_b.X_RRi[i])
            pos_b[i] = sgal3_to_se3_matrix(X)[:3, 3]
        positions += (pos_b,)
    elif traj_b is not None:
        positions += (np.asarray(traj_b),)

    if len(positions) == 2:
        return positions[0], positions[1]
    else:
        return positions[0], None


def plot_global_trajectory_with_relative_arrow(
    traj_a,
    traj_b=None,
    rel_result=None,
    labels=("Robot A", "Robot B"),
    n_arrows=8,
    arrow_color="green",
    title_3d="Global Trajectories (3D)",
    title_xy="XY Top-Down with Relative Pose Arrows",
    save=None,
):
    """One-call figure: GT trajectories + XY relative-pose arrows.

    This replaces inline Cell 6+7 notebook code (~45 lines → 1 line).

    Parameters
    ----------
    traj_a : Trajectory | np.ndarray(N,3)
        Robot A ground-truth trajectory or position array.
    traj_b : Trajectory | np.ndarray or None
        Robot B ground-truth trajectory (optional arrows use rel_result instead).
    rel_result : RelativePoseResult | None
        If provided, uses ``rel_result.rel_transforms`` to draw green relative-pose
        arrows in the XY panel from Robot A positions toward Robot B.
    labels : tuple[str, str]
        Legend labels for (A, B).
    n_arrows : int
        Number of representative arrow markers in the XY plot.
    save : str or None

    Returns
    -------
    matplotlib.figure.Figure  (has 3D + 2D subplots)
    """
    pos_a, pos_b = _positions_for_arrows(traj_a, traj_b)

    fig = plt.figure(figsize=(18, 6))
    fig.subplots_adjust(wspace=0.15)

    # ---- 3D side (subplot 1) ----
    ax3d = fig.add_subplot(121, projection="3d")
    ax3d.plot(pos_a[:, 0], pos_a[:, 1], pos_a[:, 2],
              color=arrow_color if arrow_color != "orange" else "blue",
              linewidth=2, label=labels[0])
    if pos_b is not None:
        ax3d.plot(pos_b[:, 0], pos_b[:, 1], pos_b[:, 2],
                  color="red", linestyle="--", linewidth=2, label=labels[1])
        # For arrows in 3D we only use rel_result when it is available
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
    ax3d.set_title(title_3d); ax3d.legend(loc="upper right")

    # ---- 2D XY side (subplot 2) ----
    ax2d = fig.add_subplot(122)
    ax2d.plot(pos_a[:, 0], pos_a[:, 1], color=arrow_color if arrow_color != "orange" else "blue",
              linewidth=2, label=labels[0])
    if pos_b is not None:
        ax2d.plot(pos_b[:, 0], pos_b[:, 1], color="red", linestyle="--",
                  linewidth=2, label=labels[1])

    # Draw arrows based on the chosen regime
    if rel_result is not None and hasattr(rel_result, "rel_transforms"):
        N = len(pos_a)
        step = max(1, N // n_arrows)
        for i in range(0, N, step):
            ax2d.arrow(pos_a[i, 0], pos_a[i, 1],
                       rel_result.rel_pos[i, 0], rel_result.rel_pos[i, 1],
                       head_width=0.3 if arrow_color == "green" else 0.25,
                       head_length=0.2 if arrow_color == "green" else 0.18,
                       fc=arrow_color, ec=arrow_color, alpha=0.65)
    ax2d.set_title(title_xy)
    ax2d.set_xlabel("X (m)"); ax2d.set_ylabel("Y (m)")
    ax2d.legend(); ax2d.set_aspect("equal")

    plt.tight_layout()
    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)
    return fig


def plot_perfect_vs_noisy_comparison(
    result_perfect,
    result_noisy,
    pos_a=None,
    pos_b=None,
    n_arrows=8,
    noisy_arrow_color="orange",
    labels=("Robot A", "Robot B"),
    save=None,
):
    """One-call figure: noisy vs perfect comparison with error curves.

    Replaces inline Cell 9 notebook code (~45 lines → 1 line).

    Left panel  — GT trajectories (if traj_b is provided) with noisy relative-pose arrows.
    Right panel — position and orientation error over time.

    Parameters
    ----------
    result_perfect : RelativePoseResult
    result_noisy   : RelativePoseResult
    pos_a, pos_b  : np.ndarray(N,3) or None (if None, trajectories not drawn on left)
    n_arrows       : number of noise arrows
    save           : optional output file path
    """
    N = len(result_perfect.time)
    step = max(1, N // n_arrows)
    pos_e = np.linalg.norm(result_perfect.rel_pos - result_noisy.rel_pos, axis=1)
    rot_e = np.linalg.norm(result_perfect.rel_orient - result_noisy.rel_orient, axis=1)

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(16, 5))

    # --- Left panel: trajectories + noisy arrows via overlay / offset scatter ---
    if pos_b is not None:
        ax_left.plot(pos_a[:, 0], pos_a[:, 1], "b-", linewidth=2, label=labels[0])
        ax_left.plot(pos_b[:, 0], pos_b[:, 1], "r--", linewidth=2, label=labels[1])

    # Draw arrows using noisy relative pose offsets (start from A GT positions)
    if pos_a is not None and result_noisy is not None:
        for i in range(0, N, step):
            ax_left.arrow(pos_a[i, 0], pos_a[i, 1],
                          result_noisy.rel_pos[i, 0], result_noisy.rel_pos[i, 1],
                          head_width=0.25, head_length=0.18,
                          fc=noisy_arrow_color, ec=noisy_arrow_color, alpha=0.6)

    ax_left.set_title("Noisy Relative Pose (arrows show noisy offset)")
    pos_b_exists = pos_b is not None
    if pos_b_exists:
        ax_left.legend(); ax_left.set_aspect("equal")
    ax_left.set_xlabel("X (m)"); ax_left.set_ylabel("Y (m)")

    # --- Right panel: error comparison curves ---
    t = result_perfect.time
    r = result_noisy.time if len(result_noisy.time) == len(t) else None
    tm = max(min(len(t), len(r)) if r is not None else len(t), 1)
    time_common = r if r is not None and len(r) == len(t) else t[:tm]

    ax_right.plot(time_common, pos_e[:tm], "b-", linewidth=1.5, label="Position error (m)")
    ax_right.plot(time_common, rot_e[:tm] * 100, "r-", linewidth=1.5,
                  label="Orientation err × 100 (rad)")
    ax_right.set_ylabel("Error magnitude"); ax_right.set_xlabel("Time (s)")
    ax_right.set_title("Perfect vs Noisy: Error Magnitude")
    ax_right.legend(); ax_right.grid(True, alpha=0.3)

    plt.tight_layout()
    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)
    return fig


def plot_global_trajectories(traj_a, traj_b=None, labels=None, save=None):
    """Plot GT trajectories of both robots on a 3D + 2D XY figure.

    Parameters
    ----------
    traj_a : any object with .X_RRi or .t attributes (Trajectory-like)
        Robot A's ground-truth trajectory.
    traj_b : Trajectory or None
        Robot B's ground-truth trajectory.
    labels : tuple[str, str] or None
        Labels for (A, B). Defaults to ("Robot A GT", "Robot B GT").
    save : str or None
        If given, saves the figure to this path.

    Returns
    -------
    matplotlib.figure.Figure
    """
    if labels is None:
        labels = ("Robot A GT", "Robot B GT")

    def _extract_positions(traj):
        pos = np.zeros((len(traj.X_RRi), 3))
        for i, X in enumerate(traj.X_RRi):
            pos[i, :] = X.translation()
        return pos

    pos_a = _extract_positions(traj_a)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), subplot_kw={"projection": "3d"} if False else {})

    # Fix: create proper 3D subplot
    plt.close(fig.number)
    fig, (ax_3d, ax_2d) = plt.subplots(1, 2, figsize=(16, 6))
    ax_3d = ax_3d if not hasattr(ax_3d, 'set_xlabel') else None

    # 3D plot
    import matplotlib.pyplot as mpl
    fig3d, ax3d = plt.subplots(figsize=(8, 6), subplot_kw={"projection": "3d"})
    ax3d.plot(pos_a[:, 0], pos_a[:, 1], pos_a[:, 2], color="blue", linewidth=2,
              label=f"{labels[0]}")
    if traj_b is not None:
        pos_b = _extract_positions(traj_b)
        ax3d.plot(pos_b[:, 0], pos_b[:, 1], pos_b[:, 2], color="red", linewidth=2, linestyle="--",
                  label=f"{labels[1]}")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("Global Trajectories (3D)")
    ax3d.legend()

    # 2D XY top-down overlay
    ax_2d.plot(pos_a[:, 0], pos_a[:, 1], color="blue", linewidth=2, label=f"{labels[0]}")
    if traj_b is not None:
        ax_2d.plot(pos_b[:, 0], pos_b[:, 1], color="red", linewidth=2, linestyle="--",
                   label=f"{labels[1]}")
    ax_2d.set_xlabel("X (m)")
    ax_2d.set_ylabel("Y (m)")
    ax_2d.set_title("Global Trajectories (XY Top-Down)")
    ax_2d.set_aspect("equal")
    ax_2d.legend()

    plt.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)

    return fig


def plot_position_errors(errors, labels=None, title=None, save=None):
    """Plot position error magnitude over time.

    Parameters
    ----------
    errors : np.ndarray
        (N,) array of position magnitudes or (N, 3) per-axis errors.
    labels : list[str] or None
        Labels for multiple error series.
    title : str or None
        Plot title.
    save : str or None
        If given, saves the figure.

    Returns
    -------
    matplotlib.figure.Figure
    """
    fig, ax = plt.subplots(figsize=(10, 5))

    if errors.ndim == 1:
        ax.plot(errors, color="blue", linewidth=1.5)
    else:
        for i in range(errors.shape[1]):
            lbl = labels[i] if labels and i < len(labels) else f"Axis {i}"
            ax.plot(errors[:, i], label=lbl, linewidth=1.5)

    title = title or "Position Error Magnitude Over Time"
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Error magnitude (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)

    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)

    return fig


def plot_relative_pose_errors(result, labels=None, title=None, save=None):
    """Plot relative position and orientation errors from a RelativePoseResult.

    Parameters
    ----------
    result : RelativePoseResult
        Output of relative_pose.compute_relative_pose() or compute_noisy_relative_pose().
    labels : list[str] or None
    title : str
    save : str or None
    """
    position_err = np.linalg.norm(result.rel_pos, axis=1)
    orient_err = np.linalg.norm(result.rel_orient, axis=1)

    fig, (ax_pos, ax_rot) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax_pos.plot(result.time, position_err, color="blue", linewidth=1.5)
    ax_pos.set_ylabel("Position error (m)")
    ax_pos.set_title(title or "Relative Pose Error Over Time")
    ax_pos.grid(True, alpha=0.3)

    ax_rot.plot(result.time, orient_err, color="red", linewidth=1.5)
    ax_rot.set_xlabel("Time (s)")
    ax_rot.set_ylabel("Orientation error (rad)")
    ax_rot.grid(True, alpha=0.3)

    plt.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)

    return fig
