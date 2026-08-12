from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from arp_utility_code.Measurements.ROS_base import RosMeasurement
from arp_utility_code.UtilityCode.Sensors import GTSensor, Imu


POINTCLOUD2_MSG_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_MSG_TYPE = "sensor_msgs/msg/Imu"

_POINT_FIELD_DTYPES = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}


@dataclass
class RosImuSample:
    time: float
    orientation_wxyz: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    orientation_covariance: np.ndarray | None = None
    angular_velocity_covariance: np.ndarray | None = None
    linear_acceleration_covariance: np.ndarray | None = None


@dataclass
class RosLidarPointCloudSample:
    time: float
    frame_id: str
    points: np.ndarray
    intensity: np.ndarray | None = None
    ring: np.ndarray | None = None
    point_times: np.ndarray | None = None


@dataclass
class LidarSlamPoseSample:
    time: float
    transform: np.ndarray


@dataclass
class RosLidarSlamSensors:
    imu: Imu
    lidar_gt: GTSensor


class RosLidarSlamMeasurement(RosMeasurement):
    def __init__(self, lidar_samples=None, imu_samples=None, pose_samples=None):
        self.lidar_samples = [] if lidar_samples is None else list(lidar_samples)
        self.imu_samples = [] if imu_samples is None else list(imu_samples)
        self.pose_samples = [] if pose_samples is None else list(pose_samples)
        self.lidar_samples.sort(key=lambda sample: sample.time)
        self.imu_samples.sort(key=lambda sample: sample.time)
        self.pose_samples.sort(key=lambda sample: sample.time)

    @classmethod
    def from_rosbag(
        cls,
        bag_path,
        lidar_topic=None,
        imu_topic=None,
        max_lidar_frames=None,
        data_percentage=None,
        run_slam=True,
        slam_config=None,
    ):
        bag_path = Path(bag_path).expanduser().resolve()
        connections = cls._connections_from_rosbag(bag_path)
        lidar_topic = cls._select_topic(
            connections,
            requested=lidar_topic,
            suffix_candidates=(
                "/sensors/lidar3d_0/points",
                "/sensors/lidar_0/points",
                "/sensors/lidar0/points",
            ),
            msgtype=POINTCLOUD2_MSG_TYPE,
        )
        imu_topic = cls._select_topic(
            connections,
            requested=imu_topic,
            suffix_candidates=("/sensors/imu_0/data", "/sensors/imu_0"),
            msgtype=IMU_MSG_TYPE,
            required=imu_topic is not None,
        )

        lidar_samples = []
        lidar_limit = cls._row_limit(
            cls._connection_message_count(connections, lidar_topic),
            max_rows=max_lidar_frames,
            data_percentage=data_percentage,
        )
        for index, msg in enumerate(cls.messages_from_topic(bag_path, lidar_topic)):
            if index >= lidar_limit:
                break
            lidar_samples.append(cls._pointcloud2_to_sample(msg))

        if imu_topic is None:
            imu_samples = []
        else:
            imu_samples = [
                cls._imu_to_sample(msg)
                for msg in cls.messages_from_topic(bag_path, imu_topic)
            ]

        measurement = cls(lidar_samples=lidar_samples, imu_samples=imu_samples)
        if run_slam:
            measurement.run_kiss_icp(config=slam_config)
        return measurement

    @classmethod
    def _connections_from_rosbag(cls, bag_path):
        Reader, _, _, _ = cls._load_rosbag_reader()
        with Reader(bag_path) as reader:
            return [
                SimpleNamespace(
                    topic=connection.topic,
                    msgtype=connection.msgtype,
                    msgcount=getattr(connection, "msgcount", None),
                )
                for connection in reader.connections
            ]

    @staticmethod
    def _select_topic(connections, requested, suffix_candidates, msgtype, required=True):
        if requested is not None:
            for connection in connections:
                if connection.topic == requested:
                    if connection.msgtype != msgtype:
                        raise ValueError(
                            f"Topic {requested!r} has type {connection.msgtype!r}, expected {msgtype!r}."
                        )
                    return requested
            raise ValueError(f"Topic {requested!r} was not found in the ROS bag.")

        matches = [
            connection.topic
            for connection in connections
            if connection.msgtype == msgtype
            and any(connection.topic.endswith(suffix) for suffix in suffix_candidates)
        ]
        if not matches:
            if not required:
                return None
            raise ValueError(f"No topic with type {msgtype!r} matched {suffix_candidates!r}.")
        if len(matches) > 1:
            raise ValueError(f"Multiple {msgtype!r} topics matched: {matches}. Pass the topic explicitly.")
        return matches[0]

    @staticmethod
    def _connection_message_count(connections, topic):
        for connection in connections:
            if connection.topic == topic:
                return connection.msgcount
        return None

    @staticmethod
    def _row_limit(total_rows, max_rows=None, data_percentage=None):
        if total_rows is None:
            total_rows = max_rows if max_rows is not None else np.inf
        if data_percentage is None:
            percentage_limit = total_rows
        else:
            if not 0 < data_percentage <= 100:
                raise ValueError("data_percentage must be in the interval (0, 100].")
            percentage_limit = int(np.ceil(total_rows * data_percentage / 100.0))

        if max_rows is None:
            return percentage_limit
        return min(max_rows, percentage_limit)

    @classmethod
    def _pointcloud2_to_sample(cls, msg):
        field_values = {
            field.name: cls._pointcloud2_field_array(msg, field)
            for field in msg.fields
            if field.name in {"x", "y", "z", "intensity", "ring", "time"}
        }
        missing = {"x", "y", "z"} - field_values.keys()
        if missing:
            raise ValueError(f"PointCloud2 message is missing required fields: {sorted(missing)}")

        points = np.column_stack(
            [
                field_values["x"].astype(float),
                field_values["y"].astype(float),
                field_values["z"].astype(float),
            ]
        )
        finite_mask = np.all(np.isfinite(points), axis=1)
        points = points[finite_mask]

        return RosLidarPointCloudSample(
            time=cls.message_time(msg),
            frame_id=msg.header.frame_id,
            points=points,
            intensity=cls._optional_masked_field(field_values, "intensity", finite_mask),
            ring=cls._optional_masked_field(field_values, "ring", finite_mask),
            point_times=cls._optional_masked_field(field_values, "time", finite_mask),
        )

    @staticmethod
    def _optional_masked_field(field_values, name, mask):
        if name not in field_values:
            return None
        return field_values[name][mask]

    @staticmethod
    def _pointcloud2_field_array(msg, field):
        if field.count != 1:
            raise ValueError(f"PointCloud2 field {field.name!r} has unsupported count {field.count}.")
        if field.datatype not in _POINT_FIELD_DTYPES:
            raise ValueError(f"PointCloud2 field {field.name!r} has unsupported datatype {field.datatype}.")

        byte_order = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(_POINT_FIELD_DTYPES[field.datatype]).newbyteorder(byte_order)
        data = bytes(msg.data)
        values = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=data,
            offset=field.offset,
            strides=(msg.row_step, msg.point_step),
        )
        return values.reshape(-1).copy()

    @classmethod
    def _imu_to_sample(cls, msg):
        return RosImuSample(
            time=cls.message_time(msg),
            orientation_wxyz=np.array(
                [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                dtype=float,
            ),
            angular_velocity=np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                dtype=float,
            ),
            linear_acceleration=np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                dtype=float,
            ),
            orientation_covariance=cls._covariance_matrix(msg.orientation_covariance),
            angular_velocity_covariance=cls._covariance_matrix(msg.angular_velocity_covariance),
            linear_acceleration_covariance=cls._covariance_matrix(msg.linear_acceleration_covariance),
        )

    @staticmethod
    def _covariance_matrix(values):
        if values is None:
            return None
        return np.asarray(values, dtype=float).reshape(3, 3)

    def run_kiss_icp(self, config=None):
        try:
            from kiss_icp.config import load_config
            from kiss_icp.kiss_icp import KissICP
        except ImportError as exc:
            raise ImportError(
                "LiDAR SLAM pose extraction requires the optional 'kiss-icp' package."
            ) from exc

        kiss_config = load_config(config)
        odometry = KissICP(config=kiss_config)
        pose_samples = []
        for sample in self.lidar_samples:
            timestamps = sample.point_times
            if timestamps is None:
                timestamps = np.zeros(sample.points.shape[0], dtype=float)
            _, _ = odometry.register_frame(sample.points, timestamps)
            pose_samples.append(
                LidarSlamPoseSample(
                    time=sample.time,
                    transform=np.asarray(odometry.last_pose, dtype=float).copy(),
                )
            )
        self.pose_samples = pose_samples
        return self

    def to_imu_sensor(self, T_OR=None, v_R0=None, R_cal=None):
        if T_OR is None:
            T_OR = np.eye(4)
        if v_R0 is None:
            v_R0 = np.zeros(3)
        if R_cal is None:
            R_cal = np.eye(3)
        else:
            R_cal = np.asarray(R_cal, dtype=float)

        start_time = self.imu_samples[0].time if self.imu_samples else 0.0
        imu = Imu(T_OR, v_R0, time=start_time)
        for sample in self.imu_samples:
            imu.get_new_measurement(
                a=R_cal @ sample.linear_acceleration,
                v=None,
                w=R_cal @ sample.angular_velocity,
                time=sample.time,
            )
        return imu

    def to_gt_sensor(self, T_cal=None):
        if T_cal is None:
            T_cal = np.eye(4)
        else:
            T_cal = np.asarray(T_cal, dtype=float)
        if not self.pose_samples:
            return GTSensor(np.eye(4), np.zeros(3), time=0.0)

        first = self.pose_samples[0]
        T_OR = first.transform @ T_cal
        v_R0 = self._initial_body_velocity(T_cal=T_cal)
        gt_sensor = GTSensor(T_OR, v_R0, time=first.time)
        for sample in self.pose_samples:
            gt_sensor.get_new_transformation_measurement(sample.transform @ T_cal, time=sample.time)
        return gt_sensor

    def _initial_body_velocity(self, T_cal=None):
        if len(self.pose_samples) < 2:
            return np.zeros(3)
        if T_cal is None:
            T_cal = np.eye(4)

        first = self.pose_samples[0]
        second = self.pose_samples[1]
        dt = second.time - first.time
        if dt <= 0:
            return np.zeros(3)

        first_T = first.transform @ T_cal
        second_T = second.transform @ T_cal
        v_global = (second_T[:3, 3] - first_T[:3, 3]) / dt
        return first_T[:3, :3].T @ v_global

    def to_sensors(self, T_OR=None, v_R0=None, R_cal=None, T_cal=None):
        return RosLidarSlamSensors(
            imu=self.to_imu_sensor(T_OR=T_OR, v_R0=v_R0, R_cal=R_cal),
            lidar_gt=self.to_gt_sensor(T_cal=T_cal),
        )
