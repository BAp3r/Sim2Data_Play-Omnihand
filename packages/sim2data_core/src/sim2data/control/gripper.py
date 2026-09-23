"""A one-scalar OmniHand-as-gripper commissioning interface.

The scalar is only an interface convenience.  Each side expands it to its own
active joints, clamps to the URDF limits, and leaves mimic joints to the
articulation.  The values here are synthetic commissioning assumptions and
must not be used as hardware calibration.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointTarget:
    name: str
    open_rad: float
    close_rad: float
    lower_rad: float
    upper_rad: float

    def at(self, amount: float) -> float:
        x = min(1.0, max(0.0, float(amount)))
        value = self.open_rad + x * (self.close_rad - self.open_rad)
        return min(self.upper_rad, max(self.lower_rad, value))


@dataclass(frozen=True)
class GripperMap:
    side: str
    joints: tuple[JointTarget, ...]
    synthetic_drive: Mapping[str, float]

    def expand(self, amount: float) -> dict[str, float]:
        """Expand ``amount`` in [0,1] to independent active joint targets."""
        if not 0.0 <= float(amount) <= 1.0:
            raise ValueError("gripper amount must be within [0, 1]")
        return {joint.name: joint.at(amount) for joint in self.joints}


def _active_joints(urdf: Path, prefix: str) -> list[ET.Element]:
    root = ET.parse(urdf).getroot()
    joints = []
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed" or joint.find("mimic") is not None or not joint.get("name", "").startswith(prefix):
            continue
        limit = joint.find("limit")
        if limit is not None and float(limit.get("lower", "0")) != float(limit.get("upper", "0")):
            joints.append(joint)
    return joints


def load_gripper_map(side: str, urdf: str | Path, config: str | Path) -> GripperMap:
    """Build a side-specific map from a commissioning JSON profile.

    The profile names each active joint explicitly; a mismatch is rejected so
    a left map cannot silently be copied to the right hand.
    """
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    raw = json.loads(Path(config).read_text(encoding="utf-8"))
    spec = raw["gripper_commissioning"][side]
    prefix = f"{side}_hand__{'l' if side == 'left' else 'R'}_"
    parsed = _active_joints(Path(urdf), prefix)
    by_name = {j.get("name"): j for j in parsed}
    targets: list[JointTarget] = []
    for item in spec["active_joints"]:
        name = item["name"]
        joint = by_name.get(name)
        if joint is None:
            raise ValueError(f"{side} active joint missing or mimic: {name}")
        limit = joint.find("limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        targets.append(JointTarget(name, float(item["open_rad"]), float(item["close_rad"]), lower, upper))
    if {j.get("name") for j in parsed} != {j.name for j in targets}:
        raise ValueError(f"{side} map does not cover exactly all independent hand joints")
    return GripperMap(side, tuple(targets), spec["synthetic_drive"])
