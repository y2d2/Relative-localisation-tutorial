import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import quaternion

from arp_utility_code.UtilityCode.Sensors import GTSensor, Imu, InterRobotDistanceSensor


@dataclass
class MiluvImuSample:
    time: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


@dataclass
class MiluvUwbRangeSample:
    time: float
    from_id: int
    to_id: int
    dist_m: float
    bias: float | None = None
    range_raw: float | None = None
    gt_range: float | None = None


@dataclass
class MiluvGroundTruthSample:
    time: float
    position: np.ndarray
    quaternion_wxyz: np.ndarray


@dataclass
class MiluvSensors:
    imu: Imu | None = None
    uwb_by_id: dict[int, InterRobotDistanceSensor] = field(default_factory=dict)
    ground_truth: GTSensor | None = None


@dataclass
class MiluvConfig:
    experiments: dict[str, dict[str, str]] = field(default_factory=dict)
    anchors: dict[str, dict[int, np.ndarray]] = field(default_factory=dict)
    tag_moment_arms: dict[str, dict[int, np.ndarray]] = field(default_factory=dict)

    @classmethod
    def from_folder(cls, base_path):
        config_path = _find_config_path(base_path)
        if config_path is None:
            return cls()

        return cls(
            experiments=_read_experiments_csv(config_path / "experiments.csv"),
            anchors=_read_vector_yaml(config_path / "uwb" / "anchors.yaml"),
            tag_moment_arms=_read_vector_yaml(config_path / "uwb" / "tags.yaml"),
        )

    def robots_for_experiment(self, exp_name):
        experiment = self.experiments.get(exp_name)
        if experiment is None:
            return []
        num_robots = int(experiment["num_robots"])
        return [f"ifo00{i}" for i in range(1, num_robots + 1)]


class MiluvImuMeasurement:
    def __init__(self, samples=None, robot_id=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.robot_id = robot_id

    @classmethod
    def from_csv(cls, path, robot_id=None, max_rows=None, data_percentage=None):
        rows = _read_rows(path)
        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        samples = []
        for row in rows[:row_limit]:
            if not _has_required_values(
                row,
                [
                    "timestamp",
                    "linear_acceleration.x",
                    "linear_acceleration.y",
                    "linear_acceleration.z",
                    "angular_velocity.x",
                    "angular_velocity.y",
                    "angular_velocity.z",
                ],
            ):
                continue
            samples.append(
                MiluvImuSample(
                    time=_timestamp_to_seconds(row["timestamp"]),
                    ax=float(row["linear_acceleration.x"]),
                    ay=float(row["linear_acceleration.y"]),
                    az=float(row["linear_acceleration.z"]),
                    gx=float(row["angular_velocity.x"]),
                    gy=float(row["angular_velocity.y"]),
                    gz=float(row["angular_velocity.z"]),
                )
            )
        return cls(samples=samples, robot_id=robot_id)

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


class MiluvUwbRangeMeasurement:
    def __init__(self, samples=None, robot_id=None, robot_tag_ids=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.robot_id = robot_id
        self.robot_tag_ids = [] if robot_tag_ids is None else [int(tag_id) for tag_id in robot_tag_ids]

    @classmethod
    def from_csv(
        cls,
        path,
        robot_id=None,
        robot_tag_ids=None,
        max_rows=None,
        data_percentage=None,
    ):
        rows = _read_rows(path)
        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        samples = []
        for row in rows[:row_limit]:
            if not _has_required_values(row, ["timestamp", "from_id", "to_id"]):
                continue
            distance_key = _first_present(row, ["range", "dist_m", "distance", "range_raw", "gt_range"])
            if distance_key is None or row.get(distance_key) in (None, ""):
                continue
            samples.append(
                MiluvUwbRangeSample(
                    time=_timestamp_to_seconds(row["timestamp"]),
                    from_id=int(float(row["from_id"])),
                    to_id=int(float(row["to_id"])),
                    dist_m=float(row[distance_key]),
                    bias=_optional_float(row, "bias"),
                    range_raw=_optional_float(row, "range_raw"),
                    gt_range=_optional_float(row, "gt_range"),
                )
            )
        return cls(samples=samples, robot_id=robot_id, robot_tag_ids=robot_tag_ids)

    def to_uwb_sensors(self, robot_tag_ids=None):
        if robot_tag_ids is None:
            robot_tag_ids = self.robot_tag_ids
        robot_tag_ids = {int(tag_id) for tag_id in robot_tag_ids}

        sensors = {}
        for sample in self.samples:
            peer_id = _peer_id_for_sample(sample, robot_tag_ids)
            if peer_id is None:
                continue
            sensor = sensors.setdefault(peer_id, InterRobotDistanceSensor())
            sensor.get_new_measurement(sample.dist_m, sample.time)
        return sensors

    def ranges_for_pair(self, from_id, to_id):
        from_id = int(from_id)
        to_id = int(to_id)
        return [
            sample
            for sample in self.samples
            if (sample.from_id == from_id and sample.to_id == to_id)
            or (sample.from_id == to_id and sample.to_id == from_id)
        ]


class MiluvGroundTruthMeasurement:
    def __init__(self, samples=None, robot_id=None):
        self.samples = [] if samples is None else sorted(samples, key=lambda sample: sample.time)
        self.robot_id = robot_id

    @classmethod
    def from_csv(cls, path, robot_id=None, max_rows=None, data_percentage=None):
        rows = _read_rows(path)
        row_limit = _row_limit(len(rows), max_rows=max_rows, data_percentage=data_percentage)
        samples = []
        for row in rows[:row_limit]:
            if not _has_required_values(
                row,
                [
                    "timestamp",
                    "pose.position.x",
                    "pose.position.y",
                    "pose.position.z",
                    "pose.orientation.x",
                    "pose.orientation.y",
                    "pose.orientation.z",
                    "pose.orientation.w",
                ],
            ):
                continue
            samples.append(
                MiluvGroundTruthSample(
                    time=_timestamp_to_seconds(row["timestamp"]),
                    position=np.array(
                        [
                            float(row["pose.position.x"]),
                            float(row["pose.position.y"]),
                            float(row["pose.position.z"]),
                        ],
                        dtype=float,
                    ),
                    quaternion_wxyz=np.array(
                        [
                            float(row["pose.orientation.w"]),
                            float(row["pose.orientation.x"]),
                            float(row["pose.orientation.y"]),
                            float(row["pose.orientation.z"]),
                        ],
                        dtype=float,
                    ),
                )
            )
        return cls(samples=samples, robot_id=robot_id)

    def to_gt_sensor(self):
        finite_samples = [
            sample
            for sample in self.samples
            if np.all(np.isfinite(sample.position)) and np.all(np.isfinite(sample.quaternion_wxyz))
        ]
        if not finite_samples:
            return GTSensor(np.eye(4), np.zeros(3), time=0.0)

        first = finite_samples[0]
        T_OR = _sample_to_transform(first)
        gt_sensor = GTSensor(T_OR, _initial_position_velocity(finite_samples), time=first.time)
        for sample in finite_samples:
            gt_sensor.get_new_measurement(sample.position, sample.quaternion_wxyz, sample.time)
        return gt_sensor


class MiluvRobotMeasurement:
    def __init__(self, imu=None, ranges=None, ground_truth=None, robot_id=None):
        self.imu = imu
        self.ranges = ranges
        self.ground_truth = ground_truth
        self.robot_id = robot_id

    @classmethod
    def from_folder(
        cls,
        robot_path,
        robot_id=None,
        robot_tag_ids=None,
        imu_sensor="imu_px4",
        imu_max_rows=None,
        range_max_rows=None,
        gt_max_rows=None,
        data_percentage=None,
    ):
        robot_path = Path(robot_path)
        if robot_id is None:
            robot_id = robot_path.name

        imu = MiluvImuMeasurement.from_csv(
            robot_path / f"{imu_sensor}.csv",
            robot_id=robot_id,
            max_rows=imu_max_rows,
            data_percentage=data_percentage,
        )
        ranges = MiluvUwbRangeMeasurement.from_csv(
            robot_path / "uwb_range.csv",
            robot_id=robot_id,
            robot_tag_ids=robot_tag_ids,
            max_rows=range_max_rows,
            data_percentage=data_percentage,
        )
        ground_truth = MiluvGroundTruthMeasurement.from_csv(
            robot_path / "mocap.csv",
            robot_id=robot_id,
            max_rows=gt_max_rows,
            data_percentage=data_percentage,
        )
        return cls(imu=imu, ranges=ranges, ground_truth=ground_truth, robot_id=robot_id)

    def to_sensors(self):
        return MiluvSensors(
            imu=None if self.imu is None else self.imu.to_imu_sensor(),
            uwb_by_id={} if self.ranges is None else self.ranges.to_uwb_sensors(),
            ground_truth=None if self.ground_truth is None else self.ground_truth.to_gt_sensor(),
        )


class MiluvExperiment:
    def __init__(self, robots=None, config=None, exp_name=None, exp_path=None):
        self.robots = {} if robots is None else dict(robots)
        self.config = MiluvConfig() if config is None else config
        self.exp_name = exp_name
        self.exp_path = None if exp_path is None else Path(exp_path)

    @classmethod
    def from_folder(
        cls,
        base_path,
        exp_name,
        robots=None,
        imu_sensor="imu_px4",
        imu_max_rows=None,
        range_max_rows=None,
        gt_max_rows=None,
        data_percentage=None,
    ):
        config = MiluvConfig.from_folder(base_path)
        exp_path = _find_experiment_path(base_path, exp_name)
        robot_ids = _resolve_robot_ids(exp_path, config, exp_name, robots)

        robot_measurements = {}
        for robot_id in robot_ids:
            robot_measurements[robot_id] = MiluvRobotMeasurement.from_folder(
                exp_path / robot_id,
                robot_id=robot_id,
                robot_tag_ids=config.tag_moment_arms.get(robot_id, {}).keys(),
                imu_sensor=imu_sensor,
                imu_max_rows=imu_max_rows,
                range_max_rows=range_max_rows,
                gt_max_rows=gt_max_rows,
                data_percentage=data_percentage,
            )
        return cls(
            robots=robot_measurements,
            config=config,
            exp_name=exp_name,
            exp_path=exp_path,
        )

    def to_sensors(self):
        return {robot_id: robot.to_sensors() for robot_id, robot in self.robots.items()}


def _read_rows(path):
    path = Path(path)
    with open(path, newline="", encoding="utf-8") as csvfile:
        return list(csv.DictReader(csvfile))


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


def _optional_float(row, key):
    value = row.get(key)
    if value in (None, ""):
        return None
    return float(value)


def _first_present(row, keys):
    for key in keys:
        if key in row:
            return key
    return None


def _timestamp_to_seconds(value):
    timestamp = float(value)
    if abs(timestamp) > 1e12:
        return timestamp * 1e-9
    return timestamp


def _peer_id_for_sample(sample, robot_tag_ids):
    if not robot_tag_ids:
        return sample.to_id
    if sample.from_id in robot_tag_ids:
        return sample.to_id
    if sample.to_id in robot_tag_ids:
        return sample.from_id
    return None


def _initial_position_velocity(samples):
    if len(samples) < 2:
        return np.zeros(3)
    first = samples[0]
    second = samples[1]
    dt = second.time - first.time
    if dt <= 0:
        return np.zeros(3)
    return (second.position - first.position) / dt


def _sample_to_transform(sample):
    T = np.eye(4)
    q = quaternion.from_float_array(sample.quaternion_wxyz)
    T[:3, :3] = quaternion.as_rotation_matrix(q)
    T[:3, 3] = sample.position
    return T


def _find_experiment_path(base_path, exp_name):
    base_path = Path(base_path).expanduser().resolve()
    candidates = [
        base_path / "data" / exp_name,
        base_path / exp_name,
        base_path,
    ]
    for candidate in candidates:
        if candidate.exists() and candidate.is_dir() and any(candidate.glob("ifo*/mocap.csv")):
            return candidate
    raise FileNotFoundError(
        f"Could not find MILUV experiment '{exp_name}' under {base_path}."
    )


def _find_config_path(base_path):
    base_path = Path(base_path).expanduser().resolve()
    candidates = [
        base_path / "config",
        base_path.parent / "config",
    ]
    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate
    return None


def _resolve_robot_ids(exp_path, config, exp_name, robots):
    if robots is not None:
        return [robots] if isinstance(robots, str) else list(robots)

    robot_ids = config.robots_for_experiment(exp_name)
    if robot_ids:
        return robot_ids

    return sorted(path.name for path in exp_path.glob("ifo*") if path.is_dir())


def _read_experiments_csv(path):
    path = Path(path)
    if not path.exists():
        return {}
    experiments = {}
    with open(path, newline="", encoding="utf-8") as csvfile:
        for row in csv.DictReader(csvfile):
            experiment = row.get("experiment")
            if experiment:
                experiments[experiment] = row
    return experiments


def _read_vector_yaml(path):
    path = Path(path)
    if not path.exists():
        return {}
    try:
        import yaml
    except ImportError as exc:
        raise ImportError("MILUV config loading requires the optional 'PyYAML' package.") from exc

    with open(path, encoding="utf-8") as yaml_file:
        raw_data = yaml.safe_load(yaml_file) or {}

    return {
        str(group): {
            int(item_id): np.asarray(_parse_vector(value), dtype=float)
            for item_id, value in values.items()
        }
        for group, values in raw_data.items()
    }


def _parse_vector(value):
    if isinstance(value, str):
        return [float(part.strip()) for part in value.strip("[]").split(",")]
    return value
