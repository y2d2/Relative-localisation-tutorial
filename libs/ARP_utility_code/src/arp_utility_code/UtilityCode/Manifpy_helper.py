from __future__ import annotations

import math

import numpy as np
import quaternion
from manifpy import SE3, SGal3, SGal3Tangent


def limit_angle(angle: float) -> float:
    return np.arctan2(np.sin(angle), np.cos(angle))


def rotation_matrix_from_rotvec(rotation_vector) -> np.ndarray:
    return quaternion.as_rotation_matrix(quaternion.from_rotation_vector(rotation_vector))


def rotation_vector_from_matrix(rotation) -> np.ndarray:
    q = quaternion.from_rotation_matrix(rotation)
    if isinstance(q, np.ndarray):
        q = q[0]
    rotation_vector = quaternion.as_rotation_vector(q)
    norm = np.linalg.norm(rotation_vector)
    if norm == 0:
        return np.zeros(3)
    return rotation_vector / norm * limit_angle(norm)


def rotation_matrix_to_coeffs(rotation) -> np.ndarray:
    q_wxyz = quaternion.as_float_array(quaternion.from_rotation_matrix(rotation))
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)


def quaternion_to_coeffs(q) -> np.ndarray:
    if hasattr(q, "w"):
        q_wxyz = quaternion.as_float_array(q)
    else:
        q_wxyz = np.asarray(q, dtype=float)
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)


def rotation_from_rpy(rpy) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def sgal3_from_parts(position=None, velocity=None, rotation=None, time=0.0) -> SGal3:
    coeffs = np.zeros(11)
    if position is not None:
        coeffs[:3] = np.asarray(position, dtype=float)
    coeffs[3:7] = rotation_matrix_to_coeffs(np.eye(3) if rotation is None else rotation)
    if velocity is not None:
        coeffs[7:10] = np.asarray(velocity, dtype=float)
    coeffs[10] = float(time)
    return SGal3(coeffs)


def sgal3_from_transform(translation, rotation, velocity=None, time=0.0) -> SGal3:
    return sgal3_from_parts(
        position=np.asarray(translation, dtype=float),
        velocity=np.zeros(3) if velocity is None else velocity,
        rotation=np.asarray(rotation, dtype=float),
        time=time,
    )


def sgal3_from_se3_matrix(transform, velocity=None, time=0.0) -> SGal3:
    transform = np.asarray(transform, dtype=float)
    return sgal3_from_transform(
        translation=transform[:3, 3],
        velocity=np.zeros(3) if velocity is None else velocity,
        rotation=transform[:3, :3],
        time=time,
    )


def sgal3_to_matrix(state: SGal3) -> np.ndarray:
    matrix = np.eye(5)
    matrix[:3, :3] = state.rotation()
    matrix[:3, 3] = state.linearVelocity()
    matrix[:3, 4] = state.translation()
    matrix[3, 4] = state.t()
    return matrix


def sgal3_to_se3_matrix(state: SGal3) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = state.rotation()
    matrix[:3, 3] = state.translation()
    return matrix


def se3_to_matrix(state: SE3) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = state.rotation()
    matrix[:3, 3] = state.translation()
    return matrix


def as_se3_matrix(state) -> np.ndarray:
    return se3_to_matrix(state) if isinstance(state, SE3) else np.asarray(state, dtype=float)


def se3_from_sgal3(state: SGal3) -> SE3:
    coeffs = np.zeros(7)
    coeffs[:3] = state.translation()
    coeffs[3:7] = rotation_matrix_to_coeffs(state.rotation())
    return SE3(coeffs)


def sgal3tangent_from_accel(a, w, dt) -> SGal3Tangent:
    tangent = np.zeros(10)
    tangent[:3] = np.asarray(a, dtype=float) * dt**2 / 2
    tangent[3:6] = np.asarray(a, dtype=float) * dt
    tangent[6:9] = np.asarray(w, dtype=float) * dt
    tangent[9] = dt
    return SGal3Tangent(tangent)


def sgal3_increment_from_accel(a, w, dt) -> SGal3:
    return sgal3tangent_from_accel(a, w, dt).exp()


def sgal3tangent_from_state(state) -> SGal3Tangent:
    return sgal3tangent_from_accel(state.velocity(), state.angular_velocity(), 1)
