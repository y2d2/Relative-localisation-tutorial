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
