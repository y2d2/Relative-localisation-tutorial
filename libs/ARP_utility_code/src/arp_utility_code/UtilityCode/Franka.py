from __future__ import annotations

from dataclasses import dataclass

import numpy as np


PANDA_JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)


@dataclass(frozen=True)
class FrankaPandaModel:
    joint_names: tuple[str, ...] = PANDA_JOINT_NAMES

    def link_transform(self, joint_positions, link_index: int) -> np.ndarray:
        if not 0 <= link_index <= 7:
            raise ValueError("link_index must be in the interval [0, 7].")
        q = np.asarray(joint_positions, dtype=float)
        if q.shape[0] < link_index:
            raise ValueError(f"Expected at least {link_index} joint positions.")

        transform = np.eye(4)
        for joint_index in range(link_index):
            transform = transform @ _PANDA_ORIGIN_TRANSFORMS[joint_index] @ _rotz(q[joint_index])
        return transform

    def link5_transform(self, joint_positions) -> np.ndarray:
        return self.link_transform(joint_positions, 5)


def panda_link_positions_over_time(
    time,
    joint_positions,
    link_index: int = 5,
    mount_transform: np.ndarray | None = None,
    model: FrankaPandaModel | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    time = np.asarray(time, dtype=float)
    joint_positions = np.asarray(joint_positions, dtype=float)
    if joint_positions.ndim != 2:
        raise ValueError("joint_positions must be a 2D array.")
    if time.shape[0] != joint_positions.shape[0]:
        raise ValueError("time and joint_positions must contain the same number of samples.")
    if mount_transform is None:
        mount_transform = np.eye(4)
    if model is None:
        model = FrankaPandaModel()

    positions = []
    for joint_row in joint_positions:
        transform = model.link_transform(joint_row, link_index) @ mount_transform
        positions.append(transform[:3, 3])
    return time, np.asarray(positions, dtype=float)


def transform_from_xyz_rpy(xyz=None, rpy=None) -> np.ndarray:
    if xyz is None:
        xyz = np.zeros(3)
    if rpy is None:
        rpy = np.zeros(3)
    transform = np.eye(4)
    transform[:3, :3] = _rpy_matrix(np.asarray(rpy, dtype=float))
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def _origin_transform(xyz, rpy) -> np.ndarray:
    return transform_from_xyz_rpy(np.asarray(xyz, dtype=float), np.asarray(rpy, dtype=float))


def _rotz(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return transform


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


_PANDA_ORIGIN_TRANSFORMS = (
    _origin_transform([0.0, 0.0, 0.333], [0.0, 0.0, 0.0]),
    _origin_transform([0.0, 0.0, 0.0], [-np.pi / 2.0, 0.0, 0.0]),
    _origin_transform([0.0, -0.316, 0.0], [np.pi / 2.0, 0.0, 0.0]),
    _origin_transform([0.0825, 0.0, 0.0], [np.pi / 2.0, 0.0, 0.0]),
    _origin_transform([-0.0825, 0.384, 0.0], [-np.pi / 2.0, 0.0, 0.0]),
    _origin_transform([0.0, 0.0, 0.0], [np.pi / 2.0, 0.0, 0.0]),
    _origin_transform([0.088, 0.0, 0.0], [np.pi / 2.0, 0.0, 0.0]),
)
