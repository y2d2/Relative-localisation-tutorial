import csv
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from arp_utility_code.Measurements.ROS_base import RosMeasurement
from arp_utility_code.UtilityCode.Sensors import Imu, InterRobotDistanceSensor


ESP_PACKET_MSG_TYPE = "ros2_esp_bridge/msg/EspPacketStamped"
ESP_PACKET_MSG_DEF = "std_msgs/Header header\nstring raw_packet\n"
ESP_IMU_SAMPLE_FIELD_COUNT = 15
ESP_RANGE_FIELD_COUNT = 4
ESP_CSV_FILE_RE = re.compile(r"udp_data_(\d+)\.csv$")
ESP_UWB_DISTANCE_CORRECTION_M = -0.15


@dataclass
class EspImuSample:
    time_ms: int
    temperature: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    q0: float
    q1: float
    q2: float
    q3: float
    ros_time: float | None = None


@dataclass
class EspRangeMeasurement:
    rid: int
    dist_m: float
    fp_rssi: int
    rx_rssi: int
    ros_time: float | None = None
    sid: int | None = None


@dataclass
class EspPacket:
    packet_id: int
    packet_time: int
    imu_sample_count: int
    imu_dropped_samples: int
    imu_time_ms: int
    imu_point_count: int
    imu_samples: list[EspImuSample] = field(default_factory=list)
    valid_ranges: int = 0
    ranges: list[EspRangeMeasurement] = field(default_factory=list)


@dataclass
class EspSensors:
    imu: Imu
    uwb_by_id: dict[int, InterRobotDistanceSensor]
    uwb_by_pair: dict[tuple[int | None, int], InterRobotDistanceSensor]


class EspPacketParser:
    @staticmethod
    def _normalize_row(row):
        normalized = list(row)
        while normalized and normalized[-1] == "":
            normalized.pop()
        return normalized

    @classmethod
    def parse_row(cls, row):
        normalized = cls._normalize_row(row)
        if len(normalized) < 6:
            raise ValueError(f"Row too short to parse packet: {normalized}")

        packet_id = int(normalized[0])
        packet_time = int(normalized[1])
        imu_sample_count = int(normalized[2])
        imu_dropped_samples = int(normalized[3])
        imu_time_ms = int(normalized[4])
        imu_point_count = int(normalized[5])

        cursor = 6
        imu_samples = []
        for _ in range(imu_point_count):
            next_cursor = cursor + ESP_IMU_SAMPLE_FIELD_COUNT
            if next_cursor > len(normalized):
                raise ValueError(
                    f"Not enough fields for {imu_point_count} IMU samples: {normalized}"
                )
            fields = normalized[cursor:next_cursor]
            imu_samples.append(
                EspImuSample(
                    time_ms=int(fields[0]),
                    temperature=float(fields[1]),
                    ax=float(fields[2]),
                    ay=float(fields[3]),
                    az=float(fields[4]),
                    gx=float(fields[5]),
                    gy=float(fields[6]),
                    gz=float(fields[7]),
                    mx=float(fields[8]),
                    my=float(fields[9]),
                    mz=float(fields[10]),
                    q0=float(fields[11]),
                    q1=float(fields[12]),
                    q2=float(fields[13]),
                    q3=float(fields[14]),
                )
            )
            cursor = next_cursor

        if cursor >= len(normalized):
            raise ValueError(f"Missing valid_ranges field: {normalized}")

        valid_ranges = int(normalized[cursor])
        cursor += 1

        ranges = []
        for _ in range(valid_ranges):
            next_cursor = cursor + ESP_RANGE_FIELD_COUNT
            if next_cursor > len(normalized):
                raise ValueError(
                    f"Not enough fields for {valid_ranges} ranges: {normalized}"
                )
            fields = normalized[cursor:next_cursor]
            ranges.append(
                EspRangeMeasurement(
                    rid=int(fields[0]),
                    dist_m=float(fields[1]),
                    fp_rssi=int(fields[2]),
                    rx_rssi=int(fields[3]),
                    sid=packet_id,
                )
            )
            cursor = next_cursor

        if cursor != len(normalized):
            raise ValueError(f"Unexpected trailing fields: {normalized[cursor:]}")

        return EspPacket(
            packet_id=packet_id,
            packet_time=packet_time,
            imu_sample_count=imu_sample_count,
            imu_dropped_samples=imu_dropped_samples,
            imu_time_ms=imu_time_ms,
            imu_point_count=imu_point_count,
            imu_samples=imu_samples,
            valid_ranges=valid_ranges,
            ranges=ranges,
        )

    @classmethod
    def parse_packet_string(cls, raw_packet):
        return cls.parse_row(raw_packet.split(";"))


class EspMeasurement(RosMeasurement):
    def __init__(self, imu_samples=None, range_measurements=None):
        self.imu_samples = [] if imu_samples is None else list(imu_samples)
        self.range_measurements = (
            [] if range_measurements is None else list(range_measurements)
        )

    @classmethod
    def from_rosbag(cls, bag_path, packet_topic):
        custom_types = {ESP_PACKET_MSG_TYPE: ESP_PACKET_MSG_DEF}
        imu_samples = []
        range_measurements = []

        for msg in cls.messages_from_topic(
            bag_path,
            packet_topic,
            msgtype=ESP_PACKET_MSG_TYPE,
            custom_types=custom_types,
        ):
            ros_time = cls.message_time(msg)
            try:
                packet = EspPacketParser.parse_packet_string(msg.raw_packet)
            except ValueError:
                continue

            for sample in packet.imu_samples:
                sample.ros_time = ros_time - (packet.imu_time_ms - sample.time_ms) / 1000.0
                imu_samples.append(sample)

            for measurement in packet.ranges:
                measurement.ros_time = ros_time
                range_measurements.append(measurement)

        imu_samples.sort(key=lambda sample: (sample.ros_time, sample.time_ms))
        range_measurements.sort(key=lambda measurement: (measurement.ros_time, measurement.rid))
        return cls(imu_samples=imu_samples, range_measurements=range_measurements)

    @classmethod
    def from_rosbag_any(cls, bag_path, packet_topic):
        imu_samples = []
        range_measurements = []

        for msg, fallback_time in cls.messages_from_any_rosbag(bag_path, packet_topic):
            ros_time = cls._message_time_or_fallback(msg, fallback_time)
            try:
                packet = EspPacketParser.parse_packet_string(msg.raw_packet)
            except ValueError:
                continue

            for sample in packet.imu_samples:
                sample.ros_time = ros_time - (packet.imu_time_ms - sample.time_ms) / 1000.0
                imu_samples.append(sample)

            for measurement in packet.ranges:
                measurement.ros_time = ros_time
                range_measurements.append(measurement)

        imu_samples.sort(key=lambda sample: (sample.ros_time, sample.time_ms))
        range_measurements.sort(key=lambda measurement: (measurement.ros_time, measurement.rid))
        return cls(imu_samples=imu_samples, range_measurements=range_measurements)

    @classmethod
    def from_csv_folder(
        cls,
        folder_path,
        max_rows=None,
        data_percentage=None,
        strict=False,
    ):
        rows = []
        for file_path in _sorted_esp_csv_files(folder_path):
            with open(file_path, newline="", encoding="utf-8") as csvfile:
                reader = csv.reader(csvfile)
                next(reader, None)
                rows.extend(row for row in reader if row)

        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        imu_samples = []
        range_measurements = []
        first_packet_time_s = None

        for row in rows[:row_limit]:
            try:
                packet = EspPacketParser.parse_row(row)
            except ValueError:
                if strict:
                    raise
                continue

            packet_time_s = packet.packet_time / 1000.0
            if first_packet_time_s is None:
                first_packet_time_s = packet_time_s
            relative_packet_time_s = packet_time_s - first_packet_time_s

            for sample in packet.imu_samples:
                sample.ros_time = (
                    relative_packet_time_s
                    - (packet.imu_time_ms - sample.time_ms) / 1000.0
                )
                imu_samples.append(sample)

            for measurement in packet.ranges:
                measurement.ros_time = relative_packet_time_s
                range_measurements.append(measurement)

        imu_samples.sort(key=lambda sample: (sample.ros_time, sample.time_ms))
        range_measurements.sort(key=lambda measurement: (measurement.ros_time, measurement.rid))
        return cls(imu_samples=imu_samples, range_measurements=range_measurements)

    def to_imu_sensor(self, T_OR=None, v_R0=None, R_cal=None):
        if T_OR is None:
            T_OR = np.eye(4)
        if v_R0 is None:
            v_R0 = np.zeros(3)
        if R_cal is None:
            R_cal = np.eye(3)
        else:
            R_cal = np.asarray(R_cal, dtype=float)

        start_time = self.imu_samples[0].ros_time if self.imu_samples else 0.0
        imu = Imu(T_OR, v_R0, time=start_time)
        for sample in self.imu_samples:
            a_body = R_cal @ np.array([sample.ax, sample.ay, sample.az], dtype=float) * 1e-3
            w_body = R_cal @ np.array([sample.gx, sample.gy, sample.gz], dtype=float)
            imu.get_new_measurement(
                a=a_body,
                v=None,
                w=w_body,
                time=sample.ros_time,
            )
        return imu

    def to_uwb_sensors(self):
        sensors = {}
        for measurement in self.range_measurements:
            sensor = sensors.setdefault(measurement.rid, InterRobotDistanceSensor())
            sensor.get_new_measurement(measurement.dist_m, measurement.ros_time)
        return sensors

    def to_uwb_pair_sensors(self, directed=False):
        sensors = {}
        for measurement in self.range_measurements:
            pair = _range_pair_key(measurement, directed=directed)
            sensor = sensors.setdefault(pair, InterRobotDistanceSensor())
            sensor.get_new_measurement(measurement.dist_m, measurement.ros_time)
        return sensors

    def to_sensors(self, T_OR=None, v_R0=None, R_cal=None):
        return EspSensors(
            imu=self.to_imu_sensor(T_OR=T_OR, v_R0=v_R0, R_cal=R_cal),
            uwb_by_id=self.to_uwb_sensors(),
            uwb_by_pair=self.to_uwb_pair_sensors(),
        )

    @staticmethod
    def _message_time_or_fallback(msg, fallback_time):
        try:
            time = RosMeasurement.message_time(msg)
        except AttributeError:
            return fallback_time
        return fallback_time if time == 0.0 else time


def _row_limit(total_rows, max_rows=None, data_percentage=None):
    if data_percentage is None:
        percentage_limit = total_rows
    else:
        if not 0 < data_percentage <= 100:
            raise ValueError("data_percentage must be in the interval (0, 100].")
        percentage_limit = int(np.ceil(total_rows * data_percentage / 100.0))

    if max_rows is None:
        return percentage_limit
    return min(max_rows, percentage_limit)


def _sorted_esp_csv_files(folder_path):
    folder_path = Path(folder_path)
    files = []
    for file_path in folder_path.glob("*.csv"):
        match = ESP_CSV_FILE_RE.match(file_path.name)
        if match:
            files.append((int(match.group(1)), file_path))
    return [file_path for _, file_path in sorted(files)]


def _range_pair_key(measurement, directed=False):
    if measurement.sid is None:
        return (None, measurement.rid)
    if directed:
        return (measurement.sid, measurement.rid)
    return tuple(sorted((measurement.sid, measurement.rid)))
