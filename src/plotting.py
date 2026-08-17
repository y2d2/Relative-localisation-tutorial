"""2D trajectory plotting utilities."""

import numpy as np


def plot_trajectory_2d(ax, poses, label="", color="k", marker="", linestyle="-", alpha=1):
    """Plot 2D trajectory with orientation arrows.
    
    Args:
        ax: Matplotlib axes object (2D)
        poses: (N, 3) array or list of SE2 objects with [x, y, theta]
        label: Legend label
        color: Line color
        marker: Point marker style
        linestyle: Line style
        alpha: Line transparency
    
    Returns:
        ax: Updated axes object
    """
    try:
        from manifpy import SE2
        if isinstance(poses, list) and len(poses) > 0 and isinstance(poses[0], SE2):
            n = len(poses)
            poses_array = np.zeros((n, 3))
            for i, T in enumerate(poses):
                poses_array[i, 0] = T.x()
                poses_array[i, 1] = T.y()
                poses_array[i, 2] = T.angle()
            poses = poses_array
    except ImportError:
        pass
    
    x = poses[:, 0]
    y = poses[:, 1]
    
    ax.plot(x, y, label=label, color=color, marker=marker, linestyle=linestyle, alpha=alpha)
    
    arrow_len = 0.3
    step = max(1, len(poses) // 20)
    for i in range(0, len(poses), step):
        x_i, y_i, theta_i = poses[i]
        dx = arrow_len * np.cos(theta_i)
        dy = arrow_len * np.sin(theta_i)
        ax.arrow(x_i, y_i, dx, dy, head_width=0.1, head_length=0.1,
                 fc=color, ec=color, alpha=alpha)
    
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    
    return ax


def plot_relative_pose(ax, T_ref, T_target, label="", color="r", scale=1.0):
    """Plot relative pose as vector from reference to target.
    
    Args:
        ax: Matplotlib axes object
        T_ref: SE2 or (x, y, theta) for reference robot
        T_target: SE2 or (x, y, theta) for target robot
        label: Legend label
        color: Vector color
        scale: Scale factor for vector length
    
    Returns:
        ax: Updated axes object
    """
    from manifpy import SE2
    
    if isinstance(T_ref, SE2):
        x1, y1 = T_ref.x(), T_ref.y()
    else:
        x1, y1 = T_ref[0], T_ref[1]
    
    if isinstance(T_target, SE2):
        x2, y2 = T_target.x(), T_target.y()
    else:
        x2, y2 = T_target[0], T_target[1]
    
    dx = (x2 - x1) * scale
    dy = (y2 - y1) * scale
    
    ax.arrow(x1, y1, dx, dy, head_width=0.1, head_length=0.1,
             fc=color, ec=color, label=label)
    
    return ax


def plot_anchor_positions(ax, anchor_positions: np.ndarray, 
                          legend_label: str = "Anchors",
                          color: str = "green",
                          marker: str = "^",
                          markersize: int = 12):
    """Plot UWB anchor positions on a 2D axes.
    
    Args:
        ax: Matplotlib axes object
        anchor_positions: (N, 2) array of anchor [x, y] positions
        legend_label: Label for legend
        color: Marker and line color
        
    Returns:
        ax: Updated axes object
    """
    import matplotlib.pyplot as plt
    
    x = anchor_positions[:, 0]
    y = anchor_positions[:, 1]
    
    ax.scatter(x, y, c=color, marker=marker, s=markersize**2, 
               label=legend_label, zorder=5)
    
    for i in range(len(anchor_positions)):
        ax.annotate(f"A{i+1}", (x[i], y[i]), textcoords="offset points", 
                    xytext=(8, 8), ha="center", fontsize=8, fontweight="bold")
    
    return ax


def plot_gdop_heatmap(ax, X: np.ndarray, Y: np.ndarray, gdop_grid: np.ndarray,
                      anchor_positions: np.ndarray | None = None,
                      alpha: float = 0.7):
    """Plot GDOP heatmap on a 2D axes.
    
    Args:
        ax: Matplotlib axes object
        X: Meshgrid X coordinates
        Y: Meshgrid Y coordinates
        gdop_grid: GDOP values
        anchor_positions: Optional (N, 2) array of anchor positions to overlay
        
    Returns:
        ax: Updated axes object
    """
    import matplotlib.pyplot as plt
    
    im = ax.pcolormesh(X, Y, gdop_grid, shading="auto", cmap="YlOrRd", 
                       alpha=alpha, vmin=0)
    
    if anchor_positions is not None:
        plot_anchor_positions(ax, anchor_positions)
    
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("GDOP")
    
    return ax


def plot_error_ellipse(ax, state, cov, color='r', n_std=2.0):
    """Plot 2D position error ellipse at given SE(2) pose.
    
    Args:
        ax: Matplotlib axes object
        state: (3,) array [x, y, theta] for pose center
        cov: (3, 3) covariance matrix (uses top-left 2×2 block)
        color: Ellipse edge color
        n_std: Number of standard deviations for ellipse radius
        
    Returns:
        ax: Updated axes object
    """
    from matplotlib.patches import Ellipse
    
    Sigma_xy = cov[:2, :2]
    vals, vecs = np.linalg.eigh(Sigma_xy)
    
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    width = 2 * n_std * np.sqrt(vals[0])
    height = 2 * n_std * np.sqrt(vals[1])
    
    ell = Ellipse(xy=state[:2], width=width, height=height,
                  angle=angle, ec=color, fc='none', lw=1.5)
    ax.add_patch(ell)
    
    return ax