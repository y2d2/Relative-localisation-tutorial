"""SE(2) discrete-time Extended Kalman Filter for UWB-aided localization."""

from dataclasses import dataclass
import numpy as np
from manifpy import SE2

from config import VIOConfig, UWBNoiseConfig, EKFConfig
from trajectory import sgal3_to_se2_poses


@dataclass
class EKFOdometryInput:
    """Odometry control input at a single timestep."""
    v: float    # Linear velocity (m/s)
    w: float    # Angular velocity (rad/s)


def SE2_ekf_predict(state, cov, ctrl, Q, dt):
    """Discrete-time SE(2) motion model prediction step.
    
    Args:
        state: (3,) array [x, y, theta] current robot pose
        cov: (3, 3) covariance matrix in tangent space
        ctrl: EKFOdometryInput with v, w
        Q: (3, 3) process noise covariance
        dt: timestep for prediction
    
    Returns:
        state_pred: (3,) predicted state
        cov_pred: (3, 3) Predicted covariance
    """
    x, y, theta = state[0], state[1], state[2]
    v, w = ctrl.v, ctrl.w
    
    # Motion model (discrete-time)
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + w * dt
    
    state_pred = np.array([x_new, y_new, theta_new])
    
    # Jacobian F = df/dx (linearization around current state)
    F = np.eye(3)
    F[0, 2] = -v * np.sin(theta) * dt
    F[1, 2] = v * np.cos(theta) * dt
    
    # Predict covariance
    cov_pred = F @ cov @ F.T + Q
    
    return state_pred, cov_pred


def SE2_ekf_update_range(state, cov, z_meas, anchor_pos, R):
    """Range-only measurement update step.
    
    Args:
        state: (3,) array [x, y, theta] current robot pose
        cov: (3, 3) covariance matrix
        z_meas: scalar range measurement
        anchor_pos: (2,) array [x_a, y_a] anchor position
        R: scalar range noise variance
    
    Returns:
        state_upd: (3,) updated state estimate
        cov_upd: (3, 3) updated covariance
    """
    dx = state[0] - anchor_pos[0]
    dy = state[1] - anchor_pos[1]
    d = np.sqrt(dx**2 + dy**2)
    
    # Measurement model: z = ||p_robot - p_anchor||
    H = np.array([[dx / d, dy / d, 0.0]])  # ∂z/∂x
    
    # Innovation
    y_innov = np.array([z_meas - d])
    
    # Innovation covariance
    S = H @ cov @ H.T + R
    
    # Kalman gain
    K = (cov @ H.T) / S
    
    # Update state and covariance
    state_upd = state + (K @ y_innov).flatten()
    cov_upd = (np.eye(3) - K @ H) @ cov
    
    return state_upd, cov_upd


def SE2_ekf_update_bearing(state, cov, z_meas, anchor_pos, R):
    """Bearing measurement update step.
    
    Args:
        state: (3,) array [x, y, theta] current robot pose
        cov: (3, 3) covariance matrix
        z_meas: scalar bearing measurement (rad)
        anchor_pos: (2,) array [x_a, y_a] anchor position
        R: scalar bearing noise variance
    
    Returns:
        state_upd: (3,) updated state estimate
        cov_upd: (3, 3) updated covariance
    """
    dx = anchor_pos[0] - state[0]
    dy = anchor_pos[1] - state[1]
    d = np.sqrt(dx**2 + dy**2)
    
    # Bearing measurement relative to robot heading
    bearing_meas = np.arctan2(dy, dx)  # absolute bearing to anchor
    z_pred = bearing_meas - state[2]   # relative to robot frame
    
    # Wrap prediction to [-pi, pi]
    z_pred = (z_pred + np.pi) % (2 * np.pi) - np.pi
    
    # Measurement model Jacobian H = ∂z/∂x
    # z = atan2(y_a - y_r, x_a - x_r) - theta_r
    H = np.array([[-dy / d**2, dx / d**2, -1.0]])  # ∂z/∂x
    
    # Innovation (wrap to [-pi, pi])
    y_innov = np.array([(z_meas - z_pred + np.pi) % (2 * np.pi) - np.pi])
    
    # Innovation covariance
    S = H @ cov @ H.T + R
    
    # Kalman gain
    K = (cov @ H.T) / S
    
    # Update state and covariance
    state_upd = state + (K @ y_innov).flatten()
    cov_upd = (np.eye(3) - K @ H) @ cov
    
    return state_upd, cov_upd


def simulate_ekf(trajectory, anchor_pos, ekf_cfg, noise_cfgs, seed=None):
    """Run EKF on trajectory with mixed odometry/UWB updates.
    
    Args:
        trajectory: Trajectory object from ARP_utility_code
        anchor_pos: (2,) array [x_a, y_a] single_anchor position
        ekf_cfg: EKFConfig instance
        noise_cfgs: tuple of (VIOConfig, UWBNoiseConfig)
        seed: random seed for reproducibility
    
    Returns:
        state_seq: list of (3,) estimated poses at each timestep
        cov_seq: list of (3, 3) covariance matrices
        t_seq: list of timestamps
    """
    if seed is not None:
        np.random.seed(seed)
    
    vio_cfg, uwb_cfg = noise_cfgs
    
    # Compute control input noise covariance Q
    sigma_v = vio_cfg.sig_v
    sigma_omega = vio_cfg.sig_w
    Q = np.diag([sigma_v**2, sigma_v**2, sigma_omega**2])
    
    R_range = uwb_cfg.sig_range ** 2
    
    # Extract ground truth trajectory as SE(2) poses and get measurements
    se2_poses_gt = [SE2(pose[0], pose[1], pose[2]) 
                    for pose in sgal3_to_se2_poses(trajectory.X_RRi)]
    
    n_steps = len(se2_poses_gt)
    
    # Stream body frame measurements from trajectory
    measurements = list(trajectory.stream_body_frame_measurements())
    
    # Initialize EKF state and covariance
    # Start at ground truth initial pose with high uncertainty
    state_init = np.array([se2_poses_gt[0].x(), se2_poses_gt[0].y(), 
                          se2_poses_gt[0].angle()])
    cov_init = ekf_cfg.sigma_range_init**2 * np.eye(3)  # Initialize with anchor range uncertainty
    
    # Store sequences
    state_seq = [state_init.copy()]
    cov_seq = [cov_init.copy()]
    t_seq = [trajectory.t[0]]
    
    # Current EKF state and covariance
    state_curr = state_init.copy()
    cov_curr = cov_init.copy()
    
    # Odometry update period in simulation steps
    odom_step_ratio = int(ekf_cfg.dt_uwb / ekf_cfg.dt_odom)
    
    for k in range(n_steps - 1):
        t_curr = trajectory.t[k]
        t_next = trajectory.t[k + 1]
        dt = t_next - t_curr
        
        # Extract control inputs (v, w) from measurements
        _, v_body, w_body, _ = measurements[k]
        
        # Add noise to control inputs (only for the first 2D components)
        v_noisy = np.sqrt(v_body[0]**2 + v_body[1]**2) + np.random.randn() * sigma_v
        w_noisy = w_body[2] + np.random.randn() * sigma_omega
        
        ctrl = EKFOdometryInput(v=v_noisy, w=w_noisy)
        
        # Prediction step (at odometry rate)
        state_pred, cov_pred = SE2_ekf_predict(state_curr, cov_curr, ctrl, Q, dt)
        
        # Store predicted state
        t_seq.append(t_next)
        
        # Check if this is a UWB measurement update timestep
        if k % odom_step_ratio == 0:
            # Generate noisy range measurement
            dx = state_pred[0] - anchor_pos[0]
            dy = state_pred[1] - anchor_pos[1]
            d_true = np.sqrt(dx**2 + dy**2)
            z_meas = d_true + np.random.randn() * uwb_cfg.sig_range
            
            # Update step
            state_curr, cov_curr = SE2_ekf_update_range(
                state_pred, cov_pred, z_meas, anchor_pos, R_range
            )
        
        else:
            # No UWB update: keep predicted state
            state_curr = state_pred.copy()
            cov_curr = cov_pred.copy()
        
        state_seq.append(state_curr.copy())
        cov_seq.append(cov_curr.copy())
    
    return state_seq, cov_seq, t_seq


def simulate_ekf_with_bearing(trajectory, anchor_pos, ekf_cfg, noise_cfgs, seed=None):
    """Run EKF with range and bearing fusion.
    
    Args:
        trajectory: Trajectory object
        anchor_pos: (2,) array [x_a, y_a] single_anchor position
        ekf_cfg: EKFConfig instance
        noise_cfgs: tuple of (VIOConfig, UWBNoiseConfig)
        seed: random seed for reproducibility
    
    Returns:
        state_seq: list of (3,) estimated poses
        cov_seq: list of (3, 3) covariance matrices
        t_seq: list of timestamps
    """
    if seed is not None:
        np.random.seed(seed)
    
    vio_cfg, uwb_cfg = noise_cfgs
    
    # Process noise
    sigma_v = vio_cfg.sig_v
    sigma_omega = vio_cfg.sig_w
    Q = np.diag([sigma_v**2, sigma_v**2, sigma_omega**2])
    
    R_range = uwb_cfg.sig_range ** 2
    R_bearing = uwb_cfg.sig_bearing ** 2
    
    # Extract ground truth trajectory as SE(2) poses and get measurements
    se2_poses_gt = [SE2(pose[0], pose[1], pose[2]) 
                    for pose in sgal3_to_se2_poses(trajectory.X_RRi)]
    
    n_steps = len(se2_poses_gt)
    
    # Stream body frame measurements from trajectory
    measurements = list(trajectory.stream_body_frame_measurements())
    
    # Initialize EKF state and covariance
    state_init = np.array([se2_poses_gt[0].x(), se2_poses_gt[0].y(), 
                          se2_poses_gt[0].angle()])
    cov_init = ekf_cfg.sigma_range_init**2 * np.eye(3)
    
    # Store sequences
    state_seq = [state_init.copy()]
    cov_seq = [cov_init.copy()]
    t_seq = [trajectory.t[0]]
    
    # Current EKF state and covariance
    state_curr = state_init.copy()
    cov_curr = cov_init.copy()
    
    # Odometry update period in simulation steps
    odom_step_ratio = int(ekf_cfg.dt_uwb / ekf_cfg.dt_odom)
    
    for k in range(n_steps - 1):
        t_curr = trajectory.t[k]
        t_next = trajectory.t[k + 1]
        dt = t_next - t_curr
        
        _, v_body, w_body, _ = measurements[k]
        
        # Add noise
        v_noisy = np.sqrt(v_body[0]**2 + v_body[1]**2) + np.random.randn() * sigma_v
        w_noisy = w_body[2] + np.random.randn() * sigma_omega
        
        ctrl = EKFOdometryInput(v=v_noisy, w=w_noisy)
        
        # Prediction step
        state_pred, cov_pred = SE2_ekf_predict(state_curr, cov_curr, ctrl, Q, dt)
        
        t_seq.append(t_next)
        
        if k % odom_step_ratio == 0:
            # Range measurement
            dx = state_pred[0] - anchor_pos[0]
            dy = state_pred[1] - anchor_pos[1]
            d_true = np.sqrt(dx**2 + dy**2)
            z_range = d_true + np.random.randn() * uwb_cfg.sig_range
            
            # Bearing measurement: vector from robot to anchor
            dx_ab = anchor_pos[0] - state_pred[0]
            dy_ab = anchor_pos[1] - state_pred[1]
            abs_bearing = np.arctan2(dy_ab, dx_ab)
            rel_bearing = (abs_bearing - state_pred[2] + np.pi) % (2 * np.pi) - np.pi
            z_bearing = rel_bearing + np.random.randn() * uwb_cfg.sig_bearing
            
            # Update with range
            state_curr, cov_curr = SE2_ekf_update_range(
                state_pred, cov_pred, z_range, anchor_pos, R_range
            )
            
            # Update with bearing (use updated state/cov)
            dx_anchor = anchor_pos[0] - state_curr[0]
            dy_anchor = anchor_pos[1] - state_curr[1]
            bearing_pred = (np.arctan2(dy_anchor, dx_anchor) - state_curr[2] + np.pi) % (2 * np.pi) - np.pi
            
            state_curr, cov_curr = SE2_ekf_update_bearing(
                state_curr, cov_curr, z_bearing, anchor_pos, R_bearing
            )
        else:
            state_curr = state_pred.copy()
            cov_curr = cov_pred.copy()
        
        state_seq.append(state_curr.copy())
        cov_seq.append(cov_curr.copy())
    
    return state_seq, cov_seq, t_seq
