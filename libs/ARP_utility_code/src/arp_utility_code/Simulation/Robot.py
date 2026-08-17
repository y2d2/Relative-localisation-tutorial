from __future__ import annotations

from pathlib import Path

import numpy as np
from manifpy import SGal3

from arp_utility_code.UtilityCode.Manifpy_helper import (
    rotation_vector_from_matrix,
    sgal3_to_se3_matrix,
)
from arp_utility_code.UtilityCode.Robot import Robot, RobotSensor
from arp_utility_code.UtilityCode.Sensors import InterRobotDistanceSensor, OdometrySensor
from arp_utility_code.UtilityCode.Trajectories import Trajectory


class RobotSimulation(Robot):
    """Robot frame graph with a simulated ground-truth base trajectory."""

    def __init__(self, T_OR=None, v_R0=None, time: float = 0.0, base_link: str = "base_link"):
        super().__init__(base_link=base_link)
        if T_OR is None:
            T_OR = np.eye(4)
        if v_R0 is None:
            v_R0 = np.zeros(3)
        self.trajectory = Trajectory(T_OR, np.asarray(v_R0, dtype=float), time=time)
        self.sensor_truth: dict[str, Trajectory] = {}

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str | Path,
        T_OR=None,
        v_R0=None,
        time: float = 0.0,
        base_link: str = "base_link",
    ) -> "RobotSimulation":
        robot = cls(T_OR=T_OR, v_R0=v_R0, time=time, base_link=base_link)
        robot.load_urdf(urdf_path)
        return robot

    def add_sensor(
        self,
        name: str,
        sensor: OdometrySensor | InterRobotDistanceSensor,
        link_name: str | None = None,
    ) -> RobotSensor:
        robot_sensor = super().add_sensor(name, sensor, link_name=link_name)
        self._reset_sensor_truth(name)
        return robot_sensor

    def set_trajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory
        self._reset_all_sensor_truth()

    def replay_trajectory(self, trajectory: Trajectory) -> None:
        self.set_trajectory(trajectory)

    def do_step(self, a, v, w, time):
        measurement = self.trajectory.do_step(a, v, w, time)
        for name in self.sensors:
            self._append_sensor_truth(name)
            self._feed_latest_odometry_measurement(name)
        return measurement

    def get_sensor_trajectory(self, name: str) -> Trajectory:
        return self.sensor_truth[name]

    def get_sensor_global_transform(self, name: str, index: int = -1) -> SGal3:
        return self._sensor_global_transform(name, index=index)

    def measure_distance_to(
        self,
        other: "RobotSimulation",
        sensor_name: str,
        other_sensor_name: str,
        time: float | None = None,
    ) -> float:
        sensor = self.sensors[sensor_name].sensor
        if not isinstance(sensor, InterRobotDistanceSensor):
            raise TypeError(f"Sensor {sensor_name!r} is not an inter-robot distance sensor.")

        p_self = self.get_sensor_global_transform(sensor_name).translation()
        p_other = other.get_sensor_global_transform(other_sensor_name).translation()
        distance = float(np.linalg.norm(p_self - p_other))
        measurement_time = self.trajectory.t[-1] if time is None else time
        sensor.get_new_measurement(distance, measurement_time)
        return distance

    def _reset_all_sensor_truth(self) -> None:
        self.sensor_truth.clear()
        for name in self.sensors:
            self._reset_sensor_truth(name)

    def _reset_sensor_truth(self, name: str) -> None:
        first_transform = self._sensor_global_transform(name, index=0)
        truth = Trajectory(sgal3_to_se3_matrix(first_transform), np.zeros(3), time=self.trajectory.t[0])
        self.sensor_truth[name] = truth

        for index, time in enumerate(self.trajectory.t[1:], start=1):
            transform = self._sensor_global_transform(name, index=index)
            truth.set_current_global_transformation(sgal3_to_se3_matrix(transform), time)

        sensor = self.sensors[name].sensor
        if isinstance(sensor, OdometrySensor):
            first_transform = self._sensor_global_transform(name, index=0)
            first_velocity = first_transform.rotation().T @ first_transform.linearVelocity()
            sensor.trajectory = Trajectory(
                sgal3_to_se3_matrix(first_transform),
                first_velocity,
                self.trajectory.t[0],
            )
            for index in range(len(self.trajectory.t)):
                sensor.get_new_measurement(*self._sensor_body_measurement(name, index))

    def _append_sensor_truth(self, name: str) -> None:
        if name not in self.sensor_truth:
            self._reset_sensor_truth(name)
            return

        transform = self._sensor_global_transform(name)
        self.sensor_truth[name].set_current_global_transformation(
            sgal3_to_se3_matrix(transform),
            self.trajectory.t[-1],
        )

    def _feed_latest_odometry_measurement(self, name: str) -> None:
        sensor = self.sensors[name].sensor
        if not isinstance(sensor, OdometrySensor):
            return

        a, v, w, time = self._latest_sensor_body_measurement(name)
        sensor.get_new_measurement(a, v, w, time)

    def _latest_sensor_body_measurement(self, name: str):
        return self._sensor_body_measurement(name, len(self.trajectory.t) - 1)

    def _sensor_body_measurement(self, name: str, index: int):
        time = self.trajectory.t[index]
        current_transform = self._sensor_global_transform(name, index=index)
        v = current_transform.rotation().T @ current_transform.linearVelocity()

        if index == 0:
            return np.zeros(3), v, np.zeros(3), time

        previous_time = self.trajectory.t[index - 1]
        dt = time - previous_time
        if dt == 0:
            return np.zeros(3), v, np.zeros(3), time

        previous_transform = self._sensor_global_transform(name, index=index - 1)
        relative_motion = previous_transform.inverse().compose(current_transform)
        a = relative_motion.linearVelocity() / dt
        w = rotation_vector_from_matrix(relative_motion.rotation()) / dt
        return a, v, w, time

    def _sensor_global_transform(self, name: str, index: int = -1) -> SGal3:
        sensor_transform = self.get_sensor_transform(name)
        base_transform = self.trajectory.X_OR.compose(self.trajectory.X_RRi[index])
        return base_transform.compose(sensor_transform)
