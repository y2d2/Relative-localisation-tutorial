from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from manifpy import SGal3

from arp_utility_code.Measurements.ROS_base import RosMeasurement
from arp_utility_code.UtilityCode.Franka import FrankaPandaModel, PANDA_JOINT_NAMES
from arp_utility_code.UtilityCode.Manifpy_helper import (
    rotation_vector_from_matrix,
    sgal3_from_se3_matrix,
)
from arp_utility_code.UtilityCode.Sensors import GTSensor, Vio


@dataclass(frozen=True)
class JointStateSample:
    time: float
    names: tuple[str, ...]
    positions: np.ndarray

    def positions_for(self, joint_names: tuple[str, ...] | list[str]) -> np.ndarray:
        index_by_name = {name: index for index, name in enumerate(self.names)}
        missing = [name for name in joint_names if name not in index_by_name]
        if missing:
            raise KeyError(f"Missing joint names: {missing!r}.")
        return np.array([self.positions[index_by_name[name]] for name in joint_names], dtype=float)


@dataclass(frozen=True)
class JointStateUwbPoseSample:
    time: float
    transform: np.ndarray


@dataclass(frozen=True)
class JointStateUwbIncrementSample:
    time: float
    transform: SGal3


class JointStateUwbOdometrySensor(Vio):
    def __init__(self, T_OR, v_RR0, time, increments=None):
        super().__init__(T_OR, v_RR0, time)
        self.increments = [] if increments is None else list(increments)


class RosJointStateMeasurement(RosMeasurement):
    def __init__(self, samples=None, topic: str | None = None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.topic = topic

    @classmethod
    def from_rosbag(cls, bag_path: str | Path, topic: str) -> "RosJointStateMeasurement":
        samples = []
        for msg, fallback_time in cls.messages_from_any_rosbag(bag_path, topic):
            time = cls._message_time_or_fallback(msg, fallback_time)
            samples.append(cls.sample_from_msg(msg, time=time))
        return cls(samples=samples, topic=topic)

    @staticmethod
    def sample_from_msg(msg, time: float | None = None) -> JointStateSample:
        if time is None:
            time = RosMeasurement.message_time(msg)
        return JointStateSample(
            time=float(time),
            names=tuple(str(name) for name in msg.name),
            positions=np.asarray(msg.position, dtype=float),
        )

    def joint_matrix(self, joint_names: tuple[str, ...] | list[str]) -> tuple[np.ndarray, np.ndarray]:
        if not self.samples:
            return np.array([], dtype=float), np.empty((0, len(joint_names)), dtype=float)
        time = np.array([sample.time for sample in self.samples], dtype=float)
        positions = np.vstack([sample.positions_for(joint_names) for sample in self.samples])
        return time, positions

    def uwb_pose_samples(
        self,
        T_world_franka,
        link_index: int = 5,
        T_link_uwb=None,
        joint_names: tuple[str, ...] | list[str] = PANDA_JOINT_NAMES,
        model: FrankaPandaModel | None = None,
    ) -> list[JointStateUwbPoseSample]:
        T_world_franka = np.asarray(T_world_franka, dtype=float)
        if T_world_franka.shape != (4, 4):
            raise ValueError("T_world_franka must be a 4x4 transform.")
        if T_link_uwb is None:
            T_link_uwb = np.eye(4)
        else:
            T_link_uwb = np.asarray(T_link_uwb, dtype=float)
            if T_link_uwb.shape != (4, 4):
                raise ValueError("T_link_uwb must be a 4x4 transform.")
        if model is None:
            model = FrankaPandaModel()

        pose_samples = []
        for sample in self.samples:
            joint_positions = sample.positions_for(joint_names)
            transform = T_world_franka @ model.link_transform(joint_positions, link_index) @ T_link_uwb
            pose_samples.append(JointStateUwbPoseSample(time=sample.time, transform=transform))
        return pose_samples

    def to_gt_sensor(
        self,
        T_world_franka,
        link_index: int = 5,
        T_link_uwb=None,
        joint_names: tuple[str, ...] | list[str] = PANDA_JOINT_NAMES,
        model: FrankaPandaModel | None = None,
    ) -> GTSensor:
        pose_samples = self.uwb_pose_samples(
            T_world_franka=T_world_franka,
            link_index=link_index,
            T_link_uwb=T_link_uwb,
            joint_names=joint_names,
            model=model,
        )
        if not pose_samples:
            return GTSensor(np.eye(4), np.zeros(3), time=0.0)

        first = pose_samples[0]
        gt_sensor = GTSensor(first.transform, self._initial_body_velocity(pose_samples), time=first.time)
        for sample in pose_samples:
            gt_sensor.get_new_transformation_measurement(sample.transform, time=sample.time)
        return gt_sensor

    def to_odometry_sensor(
        self,
        T_world_franka,
        link_index: int = 5,
        T_link_uwb=None,
        joint_names: tuple[str, ...] | list[str] = PANDA_JOINT_NAMES,
        model: FrankaPandaModel | None = None,
    ) -> JointStateUwbOdometrySensor:
        pose_samples = self.uwb_pose_samples(
            T_world_franka=T_world_franka,
            link_index=link_index,
            T_link_uwb=T_link_uwb,
            joint_names=joint_names,
            model=model,
        )
        if not pose_samples:
            return JointStateUwbOdometrySensor(np.eye(4), np.zeros(3), time=0.0)

        first = pose_samples[0]
        odometry_sensor = JointStateUwbOdometrySensor(
            first.transform,
            self._initial_body_velocity(pose_samples),
            time=first.time,
            increments=[JointStateUwbIncrementSample(time=first.time, transform=sgal3_from_se3_matrix(np.eye(4)))],
        )
        if len(pose_samples) == 1:
            return odometry_sensor

        previous = first
        for sample in pose_samples[1:]:
            dt = sample.time - previous.time
            relative_transform = np.linalg.inv(previous.transform) @ sample.transform
            relative_sgal3 = sgal3_from_se3_matrix(relative_transform)
            odometry_sensor.increments.append(
                JointStateUwbIncrementSample(time=sample.time, transform=relative_sgal3)
            )
            if dt > 0:
                v_world = (sample.transform[:3, 3] - previous.transform[:3, 3]) / dt
                v_body = sample.transform[:3, :3].T @ v_world
                w_body = rotation_vector_from_matrix(relative_transform[:3, :3]) / dt
                odometry_sensor.get_new_measurement(a=None, v=v_body, w=w_body, time=sample.time)
            previous = sample
        return odometry_sensor

    @staticmethod
    def _message_time_or_fallback(msg, fallback_time: float) -> float:
        try:
            time = RosMeasurement.message_time(msg)
        except AttributeError:
            return fallback_time
        return fallback_time if time == 0.0 else time

    @staticmethod
    def _initial_body_velocity(pose_samples: list[JointStateUwbPoseSample]) -> np.ndarray:
        if len(pose_samples) < 2:
            return np.zeros(3)
        first, second = pose_samples[:2]
        dt = second.time - first.time
        if dt <= 0:
            return np.zeros(3)
        v_world = (second.transform[:3, 3] - first.transform[:3, 3]) / dt
        return first.transform[:3, :3].T @ v_world
