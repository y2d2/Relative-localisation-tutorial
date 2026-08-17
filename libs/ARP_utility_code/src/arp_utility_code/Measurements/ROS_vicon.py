from dataclasses import dataclass

import numpy as np
import quaternion

from arp_utility_code.Measurements.ROS_base import RosMeasurement
from arp_utility_code.UtilityCode.Sensors import GTSensor


@dataclass
class ViconPoseSample:
    time: float
    position: np.ndarray
    quaternion_wxyz: np.ndarray


class ViconMeasurement(RosMeasurement):
    def __init__(self, samples=None):
        self.samples = [] if samples is None else list(samples)
        self.samples.sort(key=lambda sample: sample.time)

    @classmethod
    def from_rosbag(cls, bag_path, pose_topic):
        samples = []
        for msg in cls.messages_from_topic(bag_path, pose_topic):
            position, quaternion_wxyz = cls.pose_from_msg(msg)
            samples.append(
                ViconPoseSample(
                    time=cls.message_time(msg),
                    position=position,
                    quaternion_wxyz=quaternion_wxyz,
                )
            )
        return cls(samples=samples)

    def to_gt_sensor(self, T_cal=None):
        if T_cal is None:
            T_cal = np.eye(4)
        else:
            T_cal = np.asarray(T_cal, dtype=float)
        if not self.samples:
            return GTSensor(np.eye(4), np.zeros(3), time=0.0)

        first = self.samples[0]
        T_OR = self._sample_to_transform(first) @ T_cal
        v_R0 = self._initial_body_velocity(T_cal=T_cal)

        gt_sensor = GTSensor(T_OR, v_R0, time=first.time)
        for sample in self.samples:
            gt_sensor.get_new_transformation_measurement(
                self._sample_to_transform(sample) @ T_cal,
                time=sample.time,
            )
        return gt_sensor

    @staticmethod
    def _sample_to_transform(sample):
        T = np.eye(4)
        T[:3, :3] = quaternion.as_rotation_matrix(
            quaternion.from_float_array(sample.quaternion_wxyz)
        )
        T[:3, 3] = sample.position
        return T

    def _initial_body_velocity(self, T_cal=None):
        if len(self.samples) < 2:
            return np.zeros(3)
        if T_cal is None:
            T_cal = np.eye(4)

        first = self.samples[0]
        second = self.samples[1]
        dt = second.time - first.time
        if dt <= 0:
            return np.zeros(3)

        first_T = self._sample_to_transform(first) @ T_cal
        second_T = self._sample_to_transform(second) @ T_cal
        v_global = (second_T[:3, 3] - first_T[:3, 3]) / dt
        return first_T[:3, :3].T @ v_global
