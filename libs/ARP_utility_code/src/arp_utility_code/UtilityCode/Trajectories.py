from dataclasses import dataclass

import numpy as np
import quaternion
from manifpy import SGal3

from arp_utility_code.UtilityCode.Manifpy_helper import (
    as_se3_matrix as _as_se3_matrix,
    limit_angle as _limit_angle,
    rotation_matrix_from_rotvec as _rotation_matrix_from_rotvec,
    rotation_vector_from_matrix as _rotation_vector_from_matrix,
    se3_from_sgal3 as _se3_from_sgal3,
    sgal3_from_parts as _sgal3_from_parts,
    sgal3_from_se3_matrix as _sgal3_from_se3_matrix,
    sgal3_increment_from_accel as _sgal3_increment_from_accel,
    sgal3_to_matrix as _sgal3_to_matrix,
    sgal3tangent_from_accel as _sgal3tangent_from_accel,
    sgal3tangent_from_state as _sgal3tangent_from_state,
)


@dataclass
class TrajectoryComparisonResult:
    time: np.ndarray
    position_error_2d: np.ndarray
    position_error_3d: np.ndarray
    position_error_2d_axes: np.ndarray
    position_error_3d_axes: np.ndarray
    orientation_error_2d: np.ndarray
    orientation_error_3d: np.ndarray
    orientation_error_2d_yaw: np.ndarray
    orientation_error_3d_axes: np.ndarray
    position_rmse_2d: float
    position_rmse_3d: float
    position_rmse_2d_axes: np.ndarray
    position_rmse_3d_axes: np.ndarray
    orientation_rmse_2d: float
    orientation_rmse_3d: float
    orientation_rmse_3d_axes: np.ndarray


def _trajectory_global_states(trajectory):
    time = np.asarray(trajectory.t, dtype=float)
    states = [trajectory.X_OR.compose(X_RRi) for X_RRi in trajectory.X_RRi]
    if len(time) != len(states):
        raise ValueError("Trajectory time and state lists must have the same length.")
    if len(time) == 0:
        raise ValueError("Trajectory must contain at least one state.")
    if np.any(np.diff(time) < 0):
        raise ValueError("Trajectory timestamps must be sorted in ascending order.")
    return time, states


def _shared_ground_truth_times(ground_truth, calculated):
    gt_time, _ = _trajectory_global_states(ground_truth)
    calc_time, _ = _trajectory_global_states(calculated)
    start_time = max(gt_time[0], calc_time[0])
    end_time = min(gt_time[-1], calc_time[-1])
    if start_time > end_time:
        raise ValueError("Trajectories do not overlap in time.")
    time = gt_time[(start_time <= gt_time) & (gt_time <= end_time)]
    if len(time) == 0:
        raise ValueError("No ground-truth timestamps fall inside the shared time interval.")
    return time


def _interpolate_positions(trajectory, time):
    trajectory_time, states = _trajectory_global_states(trajectory)
    positions = np.asarray([state.translation() for state in states])
    return np.column_stack([np.interp(time, trajectory_time, positions[:, axis]) for axis in range(3)])


def _interpolate_rotations(trajectory, time):
    trajectory_time, states = _trajectory_global_states(trajectory)
    rotations = np.asarray([state.rotation() for state in states])
    interpolated_rotations = []
    for sample_time in time:
        right_index = np.searchsorted(trajectory_time, sample_time, side="right")
        if right_index == 0:
            interpolated_rotations.append(rotations[0])
            continue
        if right_index >= len(trajectory_time):
            interpolated_rotations.append(rotations[-1])
            continue

        left_index = right_index - 1
        left_time = trajectory_time[left_index]
        right_time = trajectory_time[right_index]
        alpha = (sample_time - left_time) / (right_time - left_time)
        q_left = quaternion.from_rotation_matrix(rotations[left_index])
        q_right = quaternion.from_rotation_matrix(rotations[right_index])
        q_interp = quaternion.slerp_evaluate(q_left, q_right, alpha)
        interpolated_rotations.append(quaternion.as_rotation_matrix(q_interp))
    return np.asarray(interpolated_rotations)


def _yaw_from_rotation_matrix(rotation):
    return np.arctan2(rotation[1, 0], rotation[0, 0])


def _rmse(error):
    error = np.asarray(error, dtype=float)
    return np.sqrt(np.mean(error**2, axis=0))


def _validate_dimensions(dimensions):
    if dimensions not in (2, 3):
        raise ValueError("dimensions must be 2 or 3.")


class TrajectoryComparison:
    def __init__(self, ground_truth, calculated):
        self.ground_truth = ground_truth
        self.calculated = calculated
        self._time = None
        self._position_error_cache = {}
        self._orientation_error_cache = {}
        self._result = None

    @property
    def time(self):
        if self._time is None:
            self._time = _shared_ground_truth_times(self.ground_truth, self.calculated)
        return self._time

    def position_error(self, dimensions=3, norm=True):
        _validate_dimensions(dimensions)

        key = (dimensions, norm)
        if key not in self._position_error_cache:
            gt_positions = _interpolate_positions(self.ground_truth, self.time)[:, :dimensions]
            calculated_positions = _interpolate_positions(self.calculated, self.time)[:, :dimensions]
            error = calculated_positions - gt_positions
            if norm:
                error = np.linalg.norm(error, axis=1)
            self._position_error_cache[key] = error
        return self._position_error_cache[key]

    def orientation_error(self, dimensions=3, norm=True):
        _validate_dimensions(dimensions)

        key = (dimensions, norm)
        if key not in self._orientation_error_cache:
            gt_rotations = _interpolate_rotations(self.ground_truth, self.time)
            calculated_rotations = _interpolate_rotations(self.calculated, self.time)
            if dimensions == 2:
                error = np.array(
                    [
                        _limit_angle(
                            _yaw_from_rotation_matrix(calculated_rotation) - _yaw_from_rotation_matrix(gt_rotation)
                        )
                        for gt_rotation, calculated_rotation in zip(gt_rotations, calculated_rotations)
                    ]
                )
                if norm:
                    error = np.abs(error)
            else:
                error = np.asarray(
                    [
                        _rotation_vector_from_matrix(gt_rotation.T @ calculated_rotation)
                        for gt_rotation, calculated_rotation in zip(gt_rotations, calculated_rotations)
                    ]
                )
                if norm:
                    error = np.linalg.norm(error, axis=1)
            self._orientation_error_cache[key] = error
        return self._orientation_error_cache[key]

    @property
    def result(self):
        if self._result is None:
            position_error_2d = self.position_error(dimensions=2, norm=True)
            position_error_3d = self.position_error(dimensions=3, norm=True)
            position_error_2d_axes = self.position_error(dimensions=2, norm=False)
            position_error_3d_axes = self.position_error(dimensions=3, norm=False)
            orientation_error_2d = self.orientation_error(dimensions=2, norm=True)
            orientation_error_3d = self.orientation_error(dimensions=3, norm=True)
            orientation_error_2d_yaw = self.orientation_error(dimensions=2, norm=False)
            orientation_error_3d_axes = self.orientation_error(dimensions=3, norm=False)

            self._result = TrajectoryComparisonResult(
                time=self.time,
                position_error_2d=position_error_2d,
                position_error_3d=position_error_3d,
                position_error_2d_axes=position_error_2d_axes,
                position_error_3d_axes=position_error_3d_axes,
                orientation_error_2d=orientation_error_2d,
                orientation_error_3d=orientation_error_3d,
                orientation_error_2d_yaw=orientation_error_2d_yaw,
                orientation_error_3d_axes=orientation_error_3d_axes,
                position_rmse_2d=float(_rmse(position_error_2d)),
                position_rmse_3d=float(_rmse(position_error_3d)),
                position_rmse_2d_axes=_rmse(position_error_2d_axes),
                position_rmse_3d_axes=_rmse(position_error_3d_axes),
                orientation_rmse_2d=float(_rmse(orientation_error_2d)),
                orientation_rmse_3d=float(_rmse(orientation_error_3d)),
                orientation_rmse_3d_axes=_rmse(orientation_error_3d_axes),
            )
        return self._result

    def _sampled_positions(self):
        gt_positions = _interpolate_positions(self.ground_truth, self.time)
        calculated_positions = _interpolate_positions(self.calculated, self.time)
        return gt_positions, calculated_positions

    def plot_trajectory_overlays(self, dimensions=3):
        import matplotlib.pyplot as plt

        _validate_dimensions(dimensions)
        gt_positions, calculated_positions = self._sampled_positions()
        figure = plt.figure(figsize=(12, 5), constrained_layout=True)

        if dimensions == 3:
            ax_3d = figure.add_subplot(1, 2, 1, projection="3d")
            ax_xy = figure.add_subplot(1, 2, 2)
            ax_3d.plot3D(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], label="Ground truth")
            ax_3d.plot3D(
                calculated_positions[:, 0],
                calculated_positions[:, 1],
                calculated_positions[:, 2],
                label="Calculated",
                linestyle="--",
            )
            ax_3d.set_title("3D trajectory")
            ax_3d.set_xlabel("x")
            ax_3d.set_ylabel("y")
            ax_3d.set_zlabel("z")
            ax_3d.legend()
        else:
            ax_xy = figure.add_subplot(1, 1, 1)

        ax_xy.plot(gt_positions[:, 0], gt_positions[:, 1], label="Ground truth")
        ax_xy.plot(calculated_positions[:, 0], calculated_positions[:, 1], label="Calculated", linestyle="--")
        ax_xy.set_title("XY trajectory")
        ax_xy.set_xlabel("x")
        ax_xy.set_ylabel("y")
        ax_xy.axis("equal")
        ax_xy.legend()

        return figure

    def plot_position_errors(self, dimensions=3):
        import matplotlib.pyplot as plt

        _validate_dimensions(dimensions)
        result = self.result
        figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
        axes[0].plot(result.time, result.position_error_3d_axes[:, 0], label="x")
        axes[0].plot(result.time, result.position_error_3d_axes[:, 1], label="y")
        if dimensions == 3:
            axes[0].plot(result.time, result.position_error_3d_axes[:, 2], label="z")
        axes[0].set_title("Position error per axis")
        axes[0].set_ylabel("error")
        axes[0].legend()

        if dimensions == 2:
            axes[1].plot(result.time, result.position_error_2d, label="2D norm")
        else:
            axes[1].plot(result.time, result.position_error_3d, label="3D norm")
        axes[1].set_title("Position error norm")
        axes[1].set_xlabel("time")
        axes[1].set_ylabel("error")
        axes[1].legend()

        return figure

    def plot_orientation_errors(self, dimensions=3):
        import matplotlib.pyplot as plt

        _validate_dimensions(dimensions)
        result = self.result
        if dimensions == 2:
            figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
            axes[0].plot(result.time, result.orientation_error_2d_yaw, label="yaw")
            axes[0].set_title("2D yaw error")
            axes[0].set_ylabel("rad")
            axes[0].legend()

            axes[1].plot(result.time, result.orientation_error_2d, label="2D norm")
            axes[1].set_title("Orientation error norm")
            axes[1].set_xlabel("time")
            axes[1].set_ylabel("rad")
            axes[1].legend()
        else:
            figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
            axes[0].plot(result.time, result.orientation_error_3d_axes[:, 0], label="x")
            axes[0].plot(result.time, result.orientation_error_3d_axes[:, 1], label="y")
            axes[0].plot(result.time, result.orientation_error_3d_axes[:, 2], label="z")
            axes[0].set_title("3D rotation-vector error")
            axes[0].set_ylabel("rad")
            axes[0].legend()

            axes[1].plot(result.time, result.orientation_error_3d, label="3D norm")
            axes[1].set_title("Orientation error norm")
            axes[1].set_xlabel("time")
            axes[1].set_ylabel("rad")
            axes[1].legend()

        return figure

    def plot_diagnostics(self, dimensions=3):
        _validate_dimensions(dimensions)
        return {
            "trajectory_overlays": self.plot_trajectory_overlays(dimensions=dimensions),
            "position_errors": self.plot_position_errors(dimensions=dimensions),
            "orientation_errors": self.plot_orientation_errors(dimensions=dimensions),
        }


class Trajectory():
    def __init__(self, T_OR, v_R0, time=0):
        self.X_OR = _sgal3_from_se3_matrix(_as_se3_matrix(T_OR), velocity=np.zeros(3), time=0.0)
        self.X_R0 = _sgal3_from_parts(velocity=v_R0, time=0.0)
        self.dX_0 = SGal3.Identity()
        self.X_RRi = [self.X_R0]
        self.dX_i = [self.dX_0]
        self.t = [time]

    def get_current_position(self):
        return self.X_OR.compose(self.X_RRi[-1]).translation()

    def get_global_postion_at_time(self, time):
        # Get the global position at a specific time by interpolating between the two closest time steps.
        if time < self.t[0] or time > self.t[-1]:
            print("Time is out of bounds.")
            return None
        for i in range(len(self.t)-1):
            if self.t[i] <= time <= self.t[i+1]:
                t1 = self.t[i]
                t2 = self.t[i+1]
                X1 = self.X_OR.compose(self.X_RRi[i])
                X2 = self.X_OR.compose(self.X_RRi[i+1])
                # Linear interpolation of the position
                alpha = (time - t1) / (t2 - t1)
                position = (1-alpha) * X1.translation() + alpha * X2.translation()
                return position
        return None

    def get_global_state_at_time(self, time) -> SGal3:
        X_RRi = self.get_local_state_at_time(time)
        if X_RRi is None:
            return None
        X_ORi = self.X_OR.compose(X_RRi)
        return  X_ORi


    def get_local_state_at_time(self, time) :
        if time < self.t[0] or time > self.t[-1]:
            print("Time is out of bounds.")

        for i in range(len(self.t)-1):
            if self.t[i] <= time <= self.t[i+1]:
                t1 = self.t[i]
                t2 = self.t[i+1]
                X1 = self.X_RRi[i]
                X2 = self.X_RRi[i+1]
                # Linear interpolation of the position
                alpha = (time - t1) / (t2 - t1)
                position = (1-alpha) * X1.translation() + alpha * X2.translation()
                velocity = (1-alpha) * X1.linearVelocity() + alpha * X2.linearVelocity()

                # Interpolation of angle:
                R1 = X1.rotation()
                R2 = X2.rotation()
                angle_vect_1 = _rotation_vector_from_matrix(R1)
                angle_vect_2 = _rotation_vector_from_matrix(R2)
                angle_vect = (1-alpha) * angle_vect_1 + alpha * angle_vect_2
                X_RRi = _sgal3_from_parts(position=position, velocity=velocity, rotation=_rotation_matrix_from_rotvec(angle_vect))
                return X_RRi
        return None

    def get_current_velocity(self):
        return self.X_OR.compose(self.X_RRi[-1]).linearVelocity()


    def get_current_orientation(self):
        X_ORi = self.X_OR.compose(self.X_RRi[-1])
        return _rotation_vector_from_matrix(X_ORi.rotation())

    def get_current_state(self):
        X_ORi = self.X_OR.compose(self.X_RRi[-1])
        position = X_ORi.translation()
        velocity = X_ORi.linearVelocity()
        orientation = _rotation_vector_from_matrix(X_ORi.rotation())
        return position, velocity, orientation, self.t[-1]

    def get_current_body_velocity(self):
        X_ORi = self.X_OR.compose(self.X_RRi[-1])
        return X_ORi.rotation().T @ X_ORi.linearVelocity()

    def set_current_global_pose(self, t, q, time):
        # t, and q are the translation, and orientation (quaternion) in the global reference frame O
        t = np.asarray(t, dtype=float)
        if hasattr(q, "w"):
            q_rot = q
        else:
            q_rot = quaternion.from_float_array(np.asarray(q, dtype=float))

        previous_X_ORi = self.X_OR.compose(self.X_RRi[-1])
        dt = time - self.t[-1]
        if dt > 0:
            velocity = (t - previous_X_ORi.translation()) / dt
        else:
            velocity = previous_X_ORi.linearVelocity()

        X_ORi = _sgal3_from_parts(
            position=t,
            velocity=velocity,
            rotation=quaternion.as_rotation_matrix(q_rot),
            time=time - self.t[0],
        )
        X_RRi = self.X_OR.inverse().compose(X_ORi)
        dX = self.X_RRi[-1].inverse().compose(X_RRi)

        if dt == 0:
            self.X_RRi[-1] = X_RRi
            self.dX_i[-1] = dX
        else:
            self.X_RRi.append(X_RRi)
            self.dX_i.append(dX)
            self.t.append(time)
        return X_RRi, dX

    def set_current_global_transformation(self, T, time):
        # T is the SE(3) transformation of the robot in the global reference frame O.
        T = _as_se3_matrix(T)

        previous_X_ORi = self.X_OR.compose(self.X_RRi[-1])
        dt = time - self.t[-1]
        if dt > 0:
            velocity = (T[:3, 3] - previous_X_ORi.translation()) / dt
        else:
            velocity = previous_X_ORi.linearVelocity()

        X_ORi = _sgal3_from_se3_matrix(T, velocity=velocity, time=time - self.t[0])
        X_RRi = self.X_OR.inverse().compose(X_ORi)
        dX = self.X_RRi[-1].inverse().compose(X_RRi)

        if dt == 0:
            self.X_RRi[-1] = X_RRi
            self.dX_i[-1] = dX
        else:
            self.X_RRi.append(X_RRi)
            self.dX_i.append(dX)
            self.t.append(time)
        return X_RRi, dX

    def set_current_local_state(self, state: SGal3, time):
        DXi = self.X_RRi[-1].inverse().compose(state)
        self.X_RRi.append(state)
        self.dX_i.append(DXi)
        self.t.append(time)

    def do_step(self, a, v, w, time):
        # a and w are expressed in the body frame of the robot.
        # a is acceleration, w is angular velocity.
        # If v_bool is true, then a is velocity instead of acceleration
        try:
            dt = time - self.t[-1]
        except IndexError:
            dt = 0
        if dt ==0:
            return a, v, w, time
        if a is not None:
            dX = _sgal3_increment_from_accel(a, w, dt)
            # Calculate v

        else:
            current_body_velocity = self.X_RRi[-1].rotation().T @ self.X_RRi[-1].linearVelocity()
            a = (np.asarray(v, dtype=float) - current_body_velocity) / dt
            dX = _sgal3_increment_from_accel(a, w, dt)
            # Calculate a

        self.t.append(time)
        self.dX_i.append(dX)
        X_RR_new = self.X_RRi[-1].compose(dX)
        if v is None:
            v = X_RR_new.rotation().T @ X_RR_new.linearVelocity()
        self.X_RRi.append(X_RR_new)
        return a, v, w, time

    def load_trajectory(self, T_ORi, time):
        # T_ORi is a list of SE(3) transformation of the trajectory as measured with a VICON or similar.
        first_T = _as_se3_matrix(T_ORi[0])
        second_T = _as_se3_matrix(T_ORi[1])
        self.X_OR = _sgal3_from_se3_matrix(first_T, velocity=np.zeros(3), time=0.0)
        v_OR0 = (second_T[:3, 3] - first_T[:3, 3]) / (time[1] - time[0])
        v_RR0 = first_T[:3, :3].T @ v_OR0

        self.X_R0 = _sgal3_from_parts(velocity=v_RR0, time=0.0)
        self.dX_0 = SGal3.Identity()

        # list of SE32 transformations and states.
        self.X_RRi = [self.X_R0]  # List of transformations from start of trajectory to each time step
        self.dX_i = [self.dX_0]
        self.t  = [time[0]]

        for T_OR, t in zip(T_ORi[1:], time[1:]):
            T_matrix = _as_se3_matrix(T_OR)
            self.set_current_global_pose(T_matrix[:3, 3], quaternion.from_rotation_matrix(T_matrix[:3, :3]), t)

    def export_SE3_trajectory(self):
        T_ORi = []
        for X_RRi in self.X_RRi:
            X_ORi = self.X_OR.compose(X_RRi)
            T_ORi.append(_se3_from_sgal3(X_ORi))
        return T_ORi, self.t

    def stream_body_frame_measurements(self):
        # Streams the measurments expressed in the body frame of the robot, as they would be measured by an IMU or VIO sensor.
        previous_t = None
        for dX, X, t in zip(self.dX_i, self.X_RRi, self.t):
            R = X.rotation()
            v = R.T @ X.linearVelocity()
            a = np.zeros(3)
            w = np.zeros(3)
            if previous_t is not None:
                a = dX.linearVelocity() / (t - previous_t)
                w = _rotation_vector_from_matrix(dX.rotation()) / (t - previous_t)
            previous_t = t
            yield a, v, w, t

    def get_steps(self):
        a_list = []
        w_list = []
        v_list = []
        t_list = []
        for a, v, w, t in self.stream_body_frame_measurements():
            a_list.append(a)
            w_list.append(w)
            v_list.append(v)
            t_list.append(t)
        return a_list, w_list, v_list, t_list


    def plot_trajectory(self,ax, label="", color="k", mark="",
                        linestyle="-",
                        alpha=1, i=-1, history=None):

        positions = np.asarray([self.X_OR.compose(X_RRi).translation() for X_RRi in self.X_RRi])
        x = positions[:, 0]
        y = positions[:, 1]
        z = positions[:, 2]
        if history is None or history > i:
            j=0
        else:
            j = i - history
            if j < 0:
                j = 0
        ax.plot3D(x[j:i], y[j:i], z[j:i], label=label,
                  color=color, marker=mark, linestyle=linestyle, alpha=alpha)
        stems = ax.stem([x[i]], [y[i]], [z[i]],
                        basefmt=f"^", linefmt=f":", bottom=0, markerfmt="")
        stems.stemlines.set_color(color)
        stems.baseline.set_color(color)
        stems.markerline.set_color(color)

    def plot_distance_to_origin(self, ax, label="", format_string="k-"):
        positions = np.asarray([self.X_OR.compose(X_RRi).translation() for X_RRi in self.X_RRi])
        x = positions[:, 0]
        y = positions[:, 1]
        z = positions[:, 2]
        distance = np.sqrt(x**2 + y**2 + z**2)
        ax.plot(self.t, distance, format_string, label=label)

    def save_trajectory(self, filename):
        np.savez(
            filename,
            X_OR=_sgal3_to_matrix(self.X_OR),
            X_RRi=[_sgal3_to_matrix(X) for X in self.X_RRi],
            dX_i=[_sgal3_to_matrix(dX) for dX in self.dX_i],
            t=self.t,
        )
