"""One-call visualization helpers for Part 1 global localisation."""

import numpy as np
from matplotlib import pyplot as plt


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
