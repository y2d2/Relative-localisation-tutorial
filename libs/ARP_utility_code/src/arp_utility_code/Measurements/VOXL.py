import csv
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from arp_utility_code.UtilityCode.Sensors import (
    GTSensor,
    Imu,
    InterRobotDistanceSensor,
    Vio,
)


_UDP_FILE_RE = re.compile(r"udp_data_(\d+)\.csv$")


@dataclass
class VoxlImuSample:
    time: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temperature: float | None = None
    batch_id: int | None = None


@dataclass
class VoxlVioSample:
    time: float
    position: np.ndarray
    euler_rpy: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    gravity: np.ndarray
    features: int | None = None
    quality: int | None = None
    state: int | None = None
    error_code: int | None = None


@dataclass
class VoxlUwbRangeSample:
    time: float
    id: int
    rid: int
    dist_m: float
    fp_rssi: int
    rx_rssi: int


@dataclass
class VoxlGroundTruthSample:
    time: float
    id: int
    position: np.ndarray
    covariance: np.ndarray


@dataclass
class VoxlSensors:
    imu: Imu | None = None
    vio: Vio | None = None
    uwb_by_id: dict[int, InterRobotDistanceSensor] = field(default_factory=dict)
    uwb_gt: GTSensor | None = None


class VoxlImuMeasurement:
    def __init__(self, samples=None, tag_id=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.tag_id = tag_id

    @classmethod
    def from_csv(
        cls,
        path,
        tag_id,
        time_offset_s=0.0,
        gyro_axis_map=None,
        accel_axis_map=None,
        max_rows=None,
        data_percentage=None,
    ):
        accel_axis_map = _axis_map_or_identity(accel_axis_map)
        gyro_axis_map = _axis_map_or_identity(gyro_axis_map)
        rows = []
        with open(path, newline="", encoding="utf-8") as csvfile:
            for row in csv.DictReader(csvfile):
                if not _has_required_values(
                    row,
                    [
                        "timestamp(ns)",
                        "AX(m/s2)",
                        "AY(m/s2)",
                        "AZ(m/s2)",
                        "GX(rad/s)",
                        "GY(rad/s)",
                        "GZ(rad/s)",
                    ],
                ):
                    continue
                rows.append(row)

        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        samples = []
        for row in rows[:row_limit]:
            accel = _apply_axis_map(
                np.array(
                    [
                        float(row["AX(m/s2)"]),
                        float(row["AY(m/s2)"]),
                        float(row["AZ(m/s2)"]),
                    ]
                ),
                accel_axis_map,
            )
            gyro = _apply_axis_map(
                np.array(
                    [
                        float(row["GX(rad/s)"]),
                        float(row["GY(rad/s)"]),
                        float(row["GZ(rad/s)"]),
                    ]
                ),
                gyro_axis_map,
            )
            samples.append(
                VoxlImuSample(
                    time=float(row["timestamp(ns)"]) * 1e-9 + time_offset_s,
                    ax=accel[0],
                    ay=accel[1],
                    az=accel[2],
                    gx=gyro[0],
                    gy=gyro[1],
                    gz=gyro[2],
                    temperature=_optional_float(row, "T(C)"),
                    batch_id=_optional_int(row, "batch_id"),
                )
            )
        return cls(samples=samples, tag_id=tag_id)

    def to_imu_sensor(self, T_OR=None, v_R0=None, R_cal=None):
        if T_OR is None:
            T_OR = np.eye(4)
        if v_R0 is None:
            v_R0 = np.zeros(3)
        if R_cal is None:
            R_cal = np.eye(3)
        else:
            R_cal = np.asarray(R_cal, dtype=float)

        start_time = self.samples[0].time if self.samples else 0.0
        imu = Imu(T_OR, v_R0, time=start_time)
        for sample in self.samples:
            accel = R_cal @ np.array([sample.ax, sample.ay, sample.az], dtype=float)
            gyro = R_cal @ np.array([sample.gx, sample.gy, sample.gz], dtype=float)
            imu.get_new_measurement(a=accel, v=None, w=gyro, time=sample.time)
        return imu


class VoxlVioMeasurement:
    def __init__(self, samples=None, tag_id=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.tag_id = tag_id

    @classmethod
    def from_csv(
        cls,
        path,
        tag_id,
        time_offset_s=0.0,
        T_align=None,
        max_rows=None,
        data_percentage=None,
    ):
        if T_align is None:
            T_align = np.eye(4)
        else:
            T_align = np.asarray(T_align, dtype=float)

        rows = []
        with open(path, newline="", encoding="utf-8") as csvfile:
            for row in csv.DictReader(csvfile):
                if not _has_required_values(
                    row,
                    [
                        "timestamp(ns)",
                        "T_imu_wrt_vio_x(m)",
                        "T_imu_wrt_vio_y(m)",
                        "T_imu_wrt_vio_z(m)",
                        "roll(rad)",
                        "pitch(rad)",
                        "yaw(rad)",
                        "vel_imu_wrt_vio_x(m/s)",
                        "vel_imu_wrt_vio_y(m/s)",
                        "vel_imu_wrt_vio_z(m/s)",
                        "angular_vel_x(rad/s)",
                        "angular_vel_y(rad/s)",
                        "angular_vel_z(rad/s)",
                        "gravity_vector_x(m/s2)",
                        "gravity_vector_y(m/s2)",
                        "gravity_vector_z(m/s2)",
                    ],
                ):
                    continue
                rows.append(row)

        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        samples = []
        for row in rows[:row_limit]:
            T = _vio_row_to_transform(row)
            T = T_align @ T
            samples.append(
                VoxlVioSample(
                    time=float(row["timestamp(ns)"]) * 1e-9 + time_offset_s,
                    position=T[:3, 3].copy(),
                    euler_rpy=Rotation.from_matrix(T[:3, :3]).as_euler("xyz"),
                    velocity=np.array(
                        [
                            float(row["vel_imu_wrt_vio_x(m/s)"]),
                            float(row["vel_imu_wrt_vio_y(m/s)"]),
                            float(row["vel_imu_wrt_vio_z(m/s)"]),
                        ]
                    ),
                    angular_velocity=np.array(
                        [
                            float(row["angular_vel_x(rad/s)"]),
                            float(row["angular_vel_y(rad/s)"]),
                            float(row["angular_vel_z(rad/s)"]),
                        ]
                    ),
                    gravity=np.array(
                        [
                            float(row["gravity_vector_x(m/s2)"]),
                            float(row["gravity_vector_y(m/s2)"]),
                            float(row["gravity_vector_z(m/s2)"]),
                        ]
                    ),
                    features=_optional_int(row, "features"),
                    quality=_optional_int(row, "quality"),
                    state=_optional_int(row, "state"),
                    error_code=_optional_int(row, "error_code"),
                )
            )
        return cls(samples=samples, tag_id=tag_id)

    def to_vio_sensor(self, T_OR=None, v_R0=None):
        if not self.samples:
            if T_OR is None:
                T_OR = np.eye(4)
            if v_R0 is None:
                v_R0 = np.zeros(3)
            return Vio(T_OR, v_R0, time=0.0)

        first = self.samples[0]
        if T_OR is None:
            T_OR = _sample_to_transform(first)
        if v_R0 is None:
            v_R0 = _initial_body_velocity(self.samples)

        vio = Vio(T_OR, v_R0, time=first.time)
        for sample in self.samples:
            vio.trajectory.set_current_global_transformation(_sample_to_transform(sample), sample.time)
        return vio


class VoxlUwbRangeMeasurement:
    def __init__(self, range_samples=None, packet_imu_samples=None):
        self.range_samples = (
            [] if range_samples is None else sorted(range_samples, key=lambda sample: sample.time)
        )
        self.packet_imu_samples = (
            []
            if packet_imu_samples is None
            else sorted(packet_imu_samples, key=lambda sample: sample.time)
        )

    @classmethod
    def from_esp_folder(cls, folder_path, max_rows=None, data_percentage=None):
        rows = []
        for file_path in _sorted_udp_files(folder_path):
            with open(file_path, newline="", encoding="utf-8") as csvfile:
                reader = csv.reader(csvfile)
                next(reader, None)
                rows.extend(row for row in reader if row)

        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        range_samples = []
        imu_samples = []
        for row in rows[:row_limit]:
            packet_imu, packet_ranges = cls.parse_esp_row(row)
            imu_samples.append(packet_imu)
            range_samples.extend(packet_ranges)
        return cls(range_samples=range_samples, packet_imu_samples=imu_samples)

    @staticmethod
    def parse_esp_row(row):
        normalized = _strip_trailing_empty_fields(row)
        if len(normalized) < 13:
            raise ValueError(f"Row too short to parse VOXL ESP row: {normalized}")

        sid = int(float(normalized[0]))
        time = float(normalized[1]) * 1e-3
        valid_ranges = int(float(normalized[12]))
        expected_len = 13 + valid_ranges * 4
        if len(normalized) < expected_len:
            raise ValueError(f"Not enough fields for {valid_ranges} ranges: {normalized}")

        imu = VoxlImuSample(
            time=time,
            ax=float(normalized[9]),
            ay=float(normalized[10]),
            az=float(normalized[11]),
            gx=float(normalized[6]),
            gy=float(normalized[7]),
            gz=float(normalized[8]),
            temperature=float(normalized[5]),
        )
        ranges = []
        for i in range(valid_ranges):
            cursor = 13 + i * 4
            ranges.append(
                VoxlUwbRangeSample(
                    time=time,
                    id=sid,
                    rid=int(float(normalized[cursor])),
                    dist_m=float(normalized[cursor + 1]),
                    fp_rssi=int(float(normalized[cursor + 2])),
                    rx_rssi=int(float(normalized[cursor + 3])),
                )
            )
        return imu, ranges

    def to_uwb_sensors(self, tag_id=None):
        sensors = {}
        for sample in self.range_samples:
            if tag_id is None:
                key = sample.rid
            elif sample.id == tag_id:
                key = sample.rid
            elif sample.rid == tag_id:
                key = sample.id
            else:
                continue
            sensor = sensors.setdefault(key, InterRobotDistanceSensor())
            sensor.get_new_measurement(sample.dist_m, sample.time)
        return sensors

    def ranges_for_pair(self, sid, rid):
        return [
            sample
            for sample in self.range_samples
            if (sample.id == sid and sample.rid == rid) or (sample.id == rid and sample.rid == sid)
        ]


class VoxlUwbGroundTruthMeasurement:
    def __init__(self, samples=None, tag_id=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.tag_id = tag_id

    @classmethod
    def from_ranges(
        cls,
        range_measurement,
        anchor_positions,
        tag_id,
        time_horizon_s=0.1,
        max_std_m=0.1,
        min_anchor_count=5,
        fixed_z=None,
        output_frequency_hz=None,
    ):
        anchor_positions = {
            int(anchor_id): np.asarray(position, dtype=float)
            for anchor_id, position in anchor_positions.items()
        }
        filtered = []
        for anchor_id in anchor_positions:
            if anchor_id == tag_id:
                continue
            filtered.extend(
                _filter_ranges_for_pair(
                    range_measurement.ranges_for_pair(tag_id, anchor_id),
                    time_horizon_s=time_horizon_s,
                    max_std_m=max_std_m,
                )
            )
        filtered.sort(key=lambda sample: sample.time)

        current_ranges = {
            anchor_id: np.array([anchor_id, np.nan, np.nan, np.nan], dtype=float)
            for anchor_id in anchor_positions
        }
        samples = []
        current_time = None
        previous_position = np.array([np.nan, np.nan, np.nan])
        min_dt = 0.0 if output_frequency_hz is None else 1.0 / output_frequency_hz

        for sample in filtered:
            if current_time is None:
                current_time = sample.time
            if current_time + min_dt < sample.time:
                current_time = sample.time
                position, covariance = _calculate_position(
                    current_ranges.values(),
                    anchor_positions,
                    current_time,
                    previous_position,
                    min_anchor_count=min_anchor_count,
                    fixed_z=fixed_z,
                )
                previous_position = position
                samples.append(
                    VoxlGroundTruthSample(
                        time=current_time,
                        id=tag_id,
                        position=position,
                        covariance=covariance,
                    )
                )

            anchor_id = sample.rid if sample.rid != tag_id else sample.id
            if anchor_id in current_ranges:
                current_ranges[anchor_id] = np.array(
                    [anchor_id, sample.time, sample.dist_m, sample.fp_rssi],
                    dtype=float,
                )

        return cls(samples=samples, tag_id=tag_id)

    def to_gt_sensor(self):
        finite_samples = [sample for sample in self.samples if np.all(np.isfinite(sample.position))]
        if not finite_samples:
            return GTSensor(np.eye(4), np.zeros(3), time=0.0)

        first = finite_samples[0]
        T_OR = np.eye(4)
        T_OR[:3, 3] = first.position
        v_R0 = _initial_position_velocity(finite_samples)
        gt_sensor = GTSensor(T_OR, v_R0, time=first.time)
        q_identity = np.array([1.0, 0.0, 0.0, 0.0])
        for sample in finite_samples:
            gt_sensor.get_new_measurement(sample.position, q_identity, sample.time)
        return gt_sensor


class VoxlExperiment:
    def __init__(self, imu=None, vio=None, uwb=None, uwb_gt=None):
        self.imu = imu
        self.vio = vio
        self.uwb = uwb
        self.uwb_gt = uwb_gt

    @classmethod
    def from_folder(
        cls,
        base_path,
        tag_id,
        anchor_positions,
        vio_time_offset_s=0.0,
        vio_T_align=None,
        imu_time_offset_s=None,
        time_horizon_s=0.1,
        max_std_m=0.1,
        min_anchor_count=5,
        fixed_z=None,
        imu_max_rows=None,
        vio_max_rows=None,
        esp_max_rows=None,
        data_percentage=None,
        gt_output_frequency_hz=None,
    ):
        base_path = Path(base_path)
        vio_folder = base_path / "Long Experiment noon VIO"
        uwb_folder = base_path / "Long Experiment noon"
        if imu_time_offset_s is None:
            imu_time_offset_s = vio_time_offset_s

        imu = VoxlImuMeasurement.from_csv(
            vio_folder / "data_imu.csv",
            tag_id=tag_id,
            time_offset_s=imu_time_offset_s,
            max_rows=imu_max_rows,
            data_percentage=data_percentage,
        )
        vio = VoxlVioMeasurement.from_csv(
            vio_folder / "data.csv",
            tag_id=tag_id,
            time_offset_s=vio_time_offset_s,
            T_align=vio_T_align,
            max_rows=vio_max_rows,
            data_percentage=data_percentage,
        )
        uwb = VoxlUwbRangeMeasurement.from_esp_folder(
            uwb_folder,
            max_rows=esp_max_rows,
            data_percentage=data_percentage,
        )
        uwb_gt = VoxlUwbGroundTruthMeasurement.from_ranges(
            uwb,
            anchor_positions=anchor_positions,
            tag_id=tag_id,
            time_horizon_s=time_horizon_s,
            max_std_m=max_std_m,
            min_anchor_count=min_anchor_count,
            fixed_z=fixed_z,
            output_frequency_hz=gt_output_frequency_hz,
        )
        return cls(imu=imu, vio=vio, uwb=uwb, uwb_gt=uwb_gt)

    def to_sensors(self):
        return VoxlSensors(
            imu=None if self.imu is None else self.imu.to_imu_sensor(),
            vio=None if self.vio is None else self.vio.to_vio_sensor(),
            uwb_by_id={} if self.uwb is None else self.uwb.to_uwb_sensors(),
            uwb_gt=None if self.uwb_gt is None else self.uwb_gt.to_gt_sensor(),
        )


def _axis_map_or_identity(axis_map):
    if axis_map is None:
        return ((0, 1.0), (1, 1.0), (2, 1.0))
    return axis_map


def _apply_axis_map(values, axis_map):
    return np.array([sign * values[index] for index, sign in axis_map], dtype=float)


def _optional_float(row, key):
    value = row.get(key)
    if value in (None, ""):
        return None
    return float(value)


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


def _has_required_values(row, keys):
    return all(row.get(key) not in (None, "") for key in keys)


def _optional_int(row, key):
    value = row.get(key)
    if value in (None, ""):
        return None
    return int(float(value))


def _vio_row_to_transform(row):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler(
        "xyz",
        [
            float(row["roll(rad)"]),
            float(row["pitch(rad)"]),
            float(row["yaw(rad)"]),
        ],
    ).as_matrix()
    T[:3, 3] = np.array(
        [
            float(row["T_imu_wrt_vio_x(m)"]),
            float(row["T_imu_wrt_vio_y(m)"]),
            float(row["T_imu_wrt_vio_z(m)"]),
        ]
    )
    return T


def _sample_to_transform(sample):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", sample.euler_rpy).as_matrix()
    T[:3, 3] = sample.position
    return T


def _initial_body_velocity(samples):
    if len(samples) < 2:
        return np.zeros(3)
    first = samples[0]
    second = samples[1]
    dt = second.time - first.time
    if dt <= 0:
        return np.zeros(3)
    T_first = _sample_to_transform(first)
    v_global = (second.position - first.position) / dt
    return T_first[:3, :3].T @ v_global


def _initial_position_velocity(samples):
    if len(samples) < 2:
        return np.zeros(3)
    first = samples[0]
    second = samples[1]
    dt = second.time - first.time
    if dt <= 0:
        return np.zeros(3)
    return (second.position - first.position) / dt


def _sorted_udp_files(folder_path):
    folder_path = Path(folder_path)
    files = []
    for file_path in folder_path.glob("*.csv"):
        match = _UDP_FILE_RE.match(file_path.name)
        if match:
            files.append((int(match.group(1)), file_path))
    return [file_path for _, file_path in sorted(files)]


def _strip_trailing_empty_fields(row):
    normalized = list(row)
    while normalized and normalized[-1] == "":
        normalized.pop()
    return normalized


def _filter_ranges_for_pair(samples, time_horizon_s, max_std_m):
    filtered = []
    distances = []
    times = []
    for sample in sorted(samples, key=lambda item: item.time):
        times.append(sample.time)
        distances.append(sample.dist_m)
        std_dist = np.std(distances)
        if std_dist > max_std_m:
            times.pop()
            distances.pop()
        elif len(distances) >= int(time_horizon_s * 10 * 0.9):
            filtered.append(sample)
        while times and sample.time - times[0] > time_horizon_s:
            times.pop(0)
            distances.pop(0)
    return filtered


def _calculate_position(
    current_ranges,
    anchor_positions,
    current_time,
    initial_guess,
    min_anchor_count,
    fixed_z,
):
    useful_ranges = [
        range_row
        for range_row in current_ranges
        if np.isfinite(range_row[1]) and range_row[1] > current_time - 1.0
    ]
    if len(useful_ranges) < min_anchor_count:
        return np.array([np.nan, np.nan, np.nan]), np.full((3, 3), np.nan)
    if fixed_z is None:
        return _calculate_3d_nls(useful_ranges, anchor_positions, initial_guess)
    return _calculate_2d_nls(useful_ranges, anchor_positions, initial_guess, fixed_z)


def _calculate_2d_nls(useful_ranges, anchor_positions, initial_guess, fixed_z):
    anchors, distances = _anchors_and_distances(useful_ranges, anchor_positions)
    if np.any(np.isnan(initial_guess)):
        initial_guess = np.mean(anchors, axis=0)
    initial_guess = np.clip(np.array([initial_guess[0], initial_guess[1]]), [-20, -20], [40, 40])

    def residuals(pos):
        augmented_pos = np.array([pos[0], pos[1], fixed_z])
        return np.linalg.norm(anchors - augmented_pos, axis=1) - distances

    result = least_squares(
        residuals,
        x0=initial_guess,
        bounds=([-20, -20], [40, 40]),
        method="trf",
        loss="soft_l1",
        f_scale=0.5,
    )
    covariance_2d = _covariance_from_result(result, rank=2)
    covariance = np.zeros((3, 3))
    covariance[:2, :2] = covariance_2d
    return np.array([result.x[0], result.x[1], fixed_z]), covariance


def _calculate_3d_nls(useful_ranges, anchor_positions, initial_guess):
    anchors, distances = _anchors_and_distances(useful_ranges, anchor_positions)
    if np.any(np.isnan(initial_guess)):
        initial_guess = np.mean(anchors, axis=0)

    def residuals(pos):
        return np.linalg.norm(anchors - pos, axis=1) - distances

    result = least_squares(
        residuals,
        x0=initial_guess,
        method="trf",
        loss="soft_l1",
        f_scale=0.5,
    )
    return result.x, _covariance_from_result(result, rank=3)


def _anchors_and_distances(useful_ranges, anchor_positions):
    anchors = []
    distances = []
    for range_row in useful_ranges:
        anchor_id = int(range_row[0])
        anchors.append(anchor_positions[anchor_id])
        distances.append(range_row[2])
    return np.array(anchors), np.array(distances)


def _covariance_from_result(result, rank):
    jtj = result.jac.T @ result.jac
    if np.linalg.matrix_rank(jtj) != rank:
        return np.full((rank, rank), np.nan)
    sigma = np.std(result.fun)
    return sigma**2 * np.linalg.inv(jtj)
