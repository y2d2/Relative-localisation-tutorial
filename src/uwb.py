"""UWB range-based localization utilities."""

import numpy as np

from manifpy import SE2

from src.config import UWBNoiseConfig


def trilaterate_2d(ranges: np.ndarray, anchor_positions: np.ndarray) -> tuple[np.ndarray, float]:
    """Trilateration in 2D using least squares approach.
    
    Uses linearized least squares (Taylor series expansion) for robust position estimation.
    
    Args:
        ranges: (3,) array of distances to each anchor
        anchor_positions: (3, 2) array of anchor [x, y] positions
        
    Returns:
        pose: (3,) estimated robot pose [x, y, theta]
        error: residual error of the solution
    """
    if len(anchor_positions) != 3 or len(ranges) != 3:
        raise ValueError("Trilateration requires exactly 3 anchors and 3 ranges")
    
    x_a = anchor_positions[:, 0]
    y_a = anchor_positions[:, 1]
    
    d = ranges
    
    # Use first anchor as initial guess (midpoint of others)
    x_mid = (anchor_positions[1, 0] + anchor_positions[2, 0]) / 2
    y_mid = (anchor_positions[1, 1] + anchor_positions[2, 1]) / 2
    
    x_est = x_mid
    y_est = y_mid
    
    # Iterative least squares (Gauss-Newton)
    for _ in range(10):
        dx = x_est - x_a
        dy = y_est - y_a
        
        r_est = np.sqrt(dx**2 + dy**2)
        
        J = np.zeros((3, 2))
        J[:, 0] = dx / (r_est + 1e-8)
        J[:, 1] = dy / (r_est + 1e-8)
        
        residual = r_est - d
        
        try:
            ATA = J.T @ J
            ATb = -J.T @ residual
            
            delta = np.linalg.solve(ATA, ATb)
            
            x_est += delta[0]
            y_est += delta[1]
            
        except np.linalg.LinAlgError:
            break
    
    # Compute final residual
    dx_final = x_est - x_a
    dy_final = y_est - y_a
    r_final = np.sqrt(dx_final**2 + dy_final**2)
    error = np.sum((r_final - d)**2)
    
    return np.array([x_est, y_est, 0.0]), error


def compute_gdop(anchor_positions: np.ndarray) -> float:
    """Compute GDOP from anchor geometry.
    
    Args:
        anchor_positions: (N, 2) array of anchor [x, y] positions
        
    Returns:
        gdop: Geometric Dilution of Precision
    """
    n_anchors = len(anchor_positions)
    
    if n_anchors < 3:
        return np.inf
    
    robot_pos = np.mean(anchor_positions, axis=0)
    
    J = np.zeros((n_anchors, 2))
    for i in range(n_anchors):
        dx = robot_pos[0] - anchor_positions[i, 0]
        dy = robot_pos[1] - anchor_positions[i, 1]
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < 1e-6:
            return np.inf
        
        J[i, 0] = dx / dist
        J[i, 1] = dy / dist
    
    try:
        ATA = J.T @ J
        cov_matrix = np.linalg.inv(ATA)
        gdop = np.sqrt(np.trace(cov_matrix))
    except np.linalg.LinAlgError:
        gdop = np.inf
    
    return gdop


def compute_gdop_at_point(point: np.ndarray, anchor_positions: np.ndarray) -> float:
    """Compute GDOP at a specific point given anchor geometry.
    
    Args:
        point: (2,) test position [x, y]
        anchor_positions: (N, 2) array of anchor positions
        
    Returns:
        gdop: Geometric Dilution of Precision
    """
    n_anchors = len(anchor_positions)
    
    J = np.zeros((n_anchors, 2))
    for i in range(n_anchors):
        dx = point[0] - anchor_positions[i, 0]
        dy = point[1] - anchor_positions[i, 1]
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < 1e-6:
            return np.inf
        
        J[i, 0] = dx / dist
        J[i, 1] = dy / dist
    
    try:
        ATA = J.T @ J
        cov_matrix = np.linalg.inv(ATA)
        gdop = np.sqrt(np.trace(cov_matrix))
    except np.linalg.LinAlgError:
        gdop = np.inf
    
    return gdop


def generate_gdop_heatmap(anchor_positions: np.ndarray, 
                          x_range: tuple[float, float],
                          y_range: tuple[float, float],
                          resolution: int = 50) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate GDOP heatmap over a 2D grid.
    
    Args:
        anchor_positions: (N, 2) array of anchor positions
        x_range: (min_x, max_x)
        y_range: (min_y, max_y)
        resolution: grid points per dimension
        
    Returns:
        X: (resolution, resolution) meshgrid X coordinates
        Y: (resolution, resolution) meshgrid Y coordinates
        gdop_grid: (resolution, resolution) GDOP values
    """
    x_min, x_max = x_range
    y_min, y_max = y_range
    
    x = np.linspace(x_min, x_max, resolution)
    y = np.linspace(y_min, y_max, resolution)
    
    X, Y = np.meshgrid(x, y)
    gdop_grid = np.zeros_like(X)
    
    for i in range(resolution):
        for j in range(resolution):
            point = np.array([X[i, j], Y[i, j]])
            gdop_grid[i, j] = compute_gdop_at_point(point, anchor_positions)
    
    return X, Y, gdop_grid


def simulate_uwb_ranges(poses: np.ndarray, 
                        anchor_positions: np.ndarray,
                        noise_config: UWBNoiseConfig | None = None) -> np.ndarray:
    """Simulate UWB range measurements along a trajectory.
    
    Args:
        poses: (N, 3) array of robot poses [x, y, theta]
        anchor_positions: (M, 2) array of anchor positions
        noise_config: Optional UWBNoiseConfig for adding noise
        
    Returns:
        ranges: (N, M) array of range measurements
    """
    n_poses = len(poses)
    n_anchors = len(anchor_positions)
    
    ranges = np.zeros((n_poses, n_anchors))
    
    for i in range(n_poses):
        for j in range(n_anchors):
            dx = poses[i, 0] - anchor_positions[j, 0]
            dy = poses[i, 1] - anchor_positions[j, 1]
            ranges[i, j] = np.sqrt(dx**2 + dy**2)
    
    if noise_config is not None:
        noise = np.random.normal(0, noise_config.sigma_range, ranges.shape)
        ranges += noise
    
    return ranges


def estimate_poses_from_ranges(ranges: np.ndarray,
                               anchor_positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Estimate poses from UWB range measurements using trilateration.
    
    Args:
        ranges: (N, M) array of range measurements
        anchor_positions: (M, 2) array of anchor positions
        
    Returns:
        estimated_poses: (N, 3) array of [x, y, theta]
        residuals: (N,) array of trilateration residuals
    """
    n_poses = ranges.shape[0]
    estimated_poses = np.zeros((n_poses, 3))
    residuals = np.zeros(n_poses)
    
    for i in range(n_poses):
        estimated_poses[i], residuals[i] = trilaterate_2d(ranges[i], anchor_positions)
    
    return estimated_poses, residuals
