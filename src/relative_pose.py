"""SE(2) relative pose computation."""

import numpy as np

from manifpy import SE2


def poses_to_se2(poses: np.ndarray):
    """Convert SE(2) pose array [x, y, theta] to list of SE2 objects.
    
    Args:
        poses: (N, 3) array with columns [x, y, theta]
    
    Returns:
        List of SE2 objects
    """
    return [SE2(x, y, theta) for x, y, theta in poses]


def se2_to_poses(se2_list):
    """Convert list of SE2 objects to pose array [x, y, theta].
    
    Args:
        se2_list: List of SE2 objects
    
    Returns:
        (N, 3) array with columns [x, y, theta]
    """
    n = len(se2_list)
    poses = np.zeros((n, 3))
    
    for i, T in enumerate(se2_list):
        poses[i, 0] = T.x()
        poses[i, 1] = T.y()
        poses[i, 2] = T.angle()
    
    return poses


def compute_relative_pose(T_O_R1, T_O_R2):
    """Compute relative pose T_{R1,R2} = T_{O,R1}^{-1} * T_{O,R2}.
    
    Args:
        T_O_R1: SE2 object for robot 1 in global frame O
        T_O_R2: SE2 object for robot 2 in global frame O
    
    Returns:
        T_R1_to_R2: SE2 object for robot 2 relative to robot 1
    """
    return T_O_R1.inverse().compose(T_O_R2)


def compute_relative_poses(poses1: np.ndarray, poses2: np.ndarray):
    """Compute relative poses between two trajectories.
    
    Args:
        poses1: (N, 3) array for robot 1 [x, y, theta] in global frame
        poses2: (N, 3) array for robot 2 [x, y, theta] in global frame
    
    Returns:
        relative_poses: (N, 3) array with columns [x_rel, y_rel, theta_rel]
    """
    se2_1 = poses_to_se2(poses1)
    se2_2 = poses_to_se2(poses2)
    
    n = len(se2_1)
    relative_poses = np.zeros((n, 3))
    
    for i in range(n):
        T_rel = compute_relative_pose(se2_1[i], se2_2[i])
        relative_poses[i, 0] = T_rel.x()
        relative_poses[i, 1] = T_rel.y()
        relative_poses[i, 2] = T_rel.angle()
    
    return relative_poses
