from __future__ import annotations

import ast
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from manifpy import SGal3

from arp_utility_code.UtilityCode.Sensors import InterRobotDistanceSensor, OdometrySensor
from arp_utility_code.UtilityCode.Manifpy_helper import (
    rotation_from_rpy,
    sgal3_from_transform,
    sgal3_to_se3_matrix,
)

_XACRO_NAMESPACE = "http://ros.org/wiki/xacro"
_EXPRESSION_RE = re.compile(r"\$\{([^}]+)\}")


@dataclass(frozen=True)
class RobotFrame:
    name: str
    parent: str | None
    transform_from_parent: SGal3
    source: str = "urdf"


@dataclass
class RobotSensor:
    name: str
    link_name: str
    sensor: OdometrySensor | InterRobotDistanceSensor
    transform_from_base: SGal3


def _local_name(tag: str) -> str:
    return tag.rsplit("}", maxsplit=1)[-1]


class _UnknownName(dict):
    def __missing__(self, key):
        self[key] = 0.0
        return 0.0


def _safe_eval(expression: str, names: dict[str, float]) -> float:
    tree = ast.parse(expression, mode="eval")
    allowed_binary = {
        ast.Add: lambda left, right: left + right,
        ast.Sub: lambda left, right: left - right,
        ast.Mult: lambda left, right: left * right,
        ast.Div: lambda left, right: left / right,
        ast.Pow: lambda left, right: left**right,
    }
    allowed_unary = {
        ast.UAdd: lambda value: value,
        ast.USub: lambda value: -value,
    }

    def evaluate(node):
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name):
            return float(names[node.id])
        if isinstance(node, ast.BinOp) and type(node.op) in allowed_binary:
            return allowed_binary[type(node.op)](evaluate(node.left), evaluate(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in allowed_unary:
            return allowed_unary[type(node.op)](evaluate(node.operand))
        raise ValueError(f"Unsupported URDF expression: {expression!r}")

    return float(evaluate(tree))


def _resolve_value(value: str | None, names: dict[str, float]) -> float:
    if value is None:
        return 0.0
    value = value.strip()
    if not value:
        return 0.0
    match = _EXPRESSION_RE.fullmatch(value)
    if match:
        return _safe_eval(match.group(1), names)
    try:
        return float(value)
    except ValueError:
        return _safe_eval(_EXPRESSION_RE.sub(lambda m: str(_safe_eval(m.group(1), names)), value), names)


def _resolve_vector(value: str | None, names: dict[str, float], length: int = 3) -> np.ndarray:
    if value is None:
        return np.zeros(length)
    value = _EXPRESSION_RE.sub(lambda m: str(_safe_eval(m.group(1), names)), value)
    parts = value.split()
    if len(parts) != length:
        raise ValueError(f"Expected {length} values, got {value!r}.")
    return np.array([_resolve_value(part, names) for part in parts], dtype=float)


def _origin_to_sgal3(origin: ET.Element | None, names: dict[str, float]) -> SGal3:
    if origin is None:
        return SGal3.Identity()
    translation = _resolve_vector(origin.attrib.get("xyz"), names)
    rpy = _resolve_vector(origin.attrib.get("rpy"), names)
    return sgal3_from_transform(translation, rotation_from_rpy(rpy))


class Robot:
    """Fixed-frame robot graph backed by SGal3 transforms.

    Graph shape:
        base_link -> fixed URDF joints -> sensor/link frames

    Sensors from this src base can be attached to any frame with `add_sensor`.
    Their base transform is then available through `sensor_transforms` and
    `get_sensor_transform`.
    """

    def __init__(self, base_link: str = "base_link"):
        self.base_link = base_link
        self.frames: dict[str, RobotFrame] = {
            base_link: RobotFrame(base_link, None, SGal3.Identity(), source="base")
        }
        self.sensors: dict[str, RobotSensor] = {}
        self._transform_cache: dict[str, SGal3] = {base_link: SGal3.Identity()}

    @classmethod
    def from_urdf(cls, urdf_path: str | Path, base_link: str = "base_link") -> "Robot":
        robot = cls(base_link=base_link)
        robot.load_urdf(urdf_path)
        return robot

    @property
    def sensor_transforms(self) -> dict[str, SGal3]:
        return {name: sensor.transform_from_base for name, sensor in self.sensors.items()}

    def add_frame(
        self,
        name: str,
        parent: str,
        transform_from_parent: SGal3 | None = None,
        source: str = "manual",
    ) -> None:
        if name in self.frames and name != self.base_link:
            raise ValueError(f"Frame {name!r} already exists.")
        if name == self.base_link:
            return
        self.frames[name] = RobotFrame(
            name=name,
            parent=parent,
            transform_from_parent=SGal3.Identity() if transform_from_parent is None else transform_from_parent,
            source=source,
        )
        self._transform_cache.clear()
        self._transform_cache[self.base_link] = SGal3.Identity()

    def add_sensor(
        self,
        name: str,
        sensor: OdometrySensor | InterRobotDistanceSensor,
        link_name: str | None = None,
    ) -> RobotSensor:
        link_name = name if link_name is None else link_name
        transform = self.get_base_transform(link_name)
        robot_sensor = RobotSensor(
            name=name,
            link_name=link_name,
            sensor=sensor,
            transform_from_base=transform,
        )
        self.sensors[name] = robot_sensor
        return robot_sensor

    def get_base_transform(self, link_name: str) -> SGal3:
        if link_name in self._transform_cache:
            return self._transform_cache[link_name]
        if link_name not in self.frames:
            raise KeyError(f"Unknown robot frame {link_name!r}.")
        frame = self.frames[link_name]
        if frame.parent is None:
            transform = frame.transform_from_parent
        else:
            transform = self.get_base_transform(frame.parent).compose(frame.transform_from_parent)
        self._transform_cache[link_name] = transform
        return transform

    def get_transform(self, parent_link: str, child_link: str) -> SGal3:
        parent_transform = self.get_base_transform(parent_link)
        child_transform = self.get_base_transform(child_link)
        return parent_transform.inverse().compose(child_transform)

    def get_transform_matrix(self, parent_link: str, child_link: str) -> np.ndarray:
        return sgal3_to_se3_matrix(self.get_transform(parent_link, child_link))

    def get_sensor_transform(self, name: str) -> SGal3:
        return self.sensors[name].transform_from_base

    def load_urdf(self, urdf_path: str | Path) -> None:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        names = self._load_xacro_properties(root)
        for joint in root.findall("joint"):
            if joint.attrib.get("type", "fixed") != "fixed":
                continue
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_link = parent.attrib["link"]
            child_link = child.attrib["link"]
            self.add_frame(
                name=child_link,
                parent=parent_link,
                transform_from_parent=_origin_to_sgal3(joint.find("origin"), names),
                source=joint.attrib.get("name", "urdf_joint"),
            )
        self._load_xacro_sensor_frames(root, names)

    def _load_xacro_properties(self, root: ET.Element) -> dict[str, float]:
        names = _UnknownName({"pi": np.pi, "cm2m": 0.01})
        for element in root.iter():
            if element.tag == f"{{{_XACRO_NAMESPACE}}}property":
                name = element.attrib.get("name")
                if name:
                    names[name] = _resolve_value(element.attrib.get("value"), names)
        return names

    def _load_xacro_sensor_frames(self, root: ET.Element, names: dict[str, float]) -> None:
        for element in root.iter():
            if not element.tag.startswith(f"{{{_XACRO_NAMESPACE}}}"):
                continue
            tag = _local_name(element.tag)
            if tag in {"include", "property", "arg"}:
                continue
            origin = element.find("origin")
            if origin is None:
                continue
            parent = element.attrib.get("parent_link") or element.attrib.get("parent")
            name = element.attrib.get("name") or element.attrib.get("link") or tag
            if parent is None or not name or name in self.frames:
                continue
            self.add_frame(
                name=name,
                parent=parent,
                transform_from_parent=_origin_to_sgal3(origin, names),
                source=f"xacro:{tag}",
            )
