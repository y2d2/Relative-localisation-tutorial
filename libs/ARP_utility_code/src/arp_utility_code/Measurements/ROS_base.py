from pathlib import Path

import numpy as np


class RosMeasurement:
    @staticmethod
    def _load_rosbag_reader():
        try:
            from rosbags.rosbag2 import Reader
            from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        except ImportError as exc:
            raise ImportError(
                "ROS bag measurements require the optional 'rosbags' package."
            ) from exc
        return Reader, Stores, get_types_from_msg, get_typestore

    @staticmethod
    def _load_any_reader():
        try:
            from rosbags.highlevel import AnyReader
        except ImportError as exc:
            raise ImportError(
                "ROS bag measurements require the optional 'rosbags' package."
            ) from exc
        return AnyReader

    @classmethod
    def messages_from_topic(cls, bag_path, topic, msgtype=None, custom_types=None):
        Reader, Stores, get_types_from_msg, get_typestore = cls._load_rosbag_reader()
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        for type_name, msg_def in (custom_types or {}).items():
            typestore.register(get_types_from_msg(msg_def, type_name))

        bag_path = Path(bag_path).expanduser().resolve()
        with Reader(bag_path) as reader:
            for connection, _, rawdata in reader.messages():
                if connection.topic != topic:
                    continue
                decode_type = msgtype or connection.msgtype
                yield typestore.deserialize_cdr(rawdata, decode_type)

    @classmethod
    def messages_from_any_rosbag(cls, bag_path, topic):
        AnyReader = cls._load_any_reader()
        bag_path = Path(bag_path).expanduser().resolve()
        paths = [bag_path]
        with AnyReader(paths) as reader:
            connections = [connection for connection in reader.connections if connection.topic == topic]
            if not connections:
                raise ValueError(f"Topic {topic!r} was not found in the ROS bag.")
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                yield reader.deserialize(rawdata, connection.msgtype), timestamp * 1e-9

    @staticmethod
    def message_time(msg):
        stamp = msg.header.stamp
        sec = getattr(stamp, "sec", getattr(stamp, "secs", 0))
        nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0))
        return sec + nanosec * 1e-9

    @staticmethod
    def pose_from_msg(msg):
        pose = msg.pose
        if hasattr(pose, "pose"):
            pose = pose.pose
        t = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ],
            dtype=float,
        )
        q = np.array(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=float,
        )
        return t, q

    @staticmethod
    def sort_by_time(samples):
        return sorted(samples, key=lambda item: item[0])
