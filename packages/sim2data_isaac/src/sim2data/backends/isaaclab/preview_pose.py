"""Resolve explicit static preview poses; never a robot controller."""
import math
import xml.etree.ElementTree as ET


def resolve_preview_positions(root: ET.Element, requested: dict[str, float]) -> dict[str, float]:
    """Use nearest-to-zero bounded defaults, preserving and checking mimic.

    User overrides must name independent movable joints and remain in limits.
    Defaults are visualization choices, not calibrated home positions.
    """
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    if set(requested) - joints.keys():
        raise ValueError("Unknown preview joint names")
    result = {}

    def resolve(name, chain=()):
        if name in chain:
            raise ValueError("Cyclic preview mimic")
        if name in result:
            return result[name]
        if name not in joints:
            raise ValueError("Missing mimic source")
        joint = joints[name]
        kind = joint.attrib["type"]
        mimic = joint.find("mimic")
        limit = joint.find("limit")
        lower, upper = -math.inf, math.inf
        if kind in ("revolute", "prismatic"):
            if limit is None:
                raise ValueError(f"Missing preview limits: {name}")
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"Invalid preview limits: {name}")
        if name in requested and (kind == "fixed" or mimic is not None):
            raise ValueError(f"Cannot override fixed/mimic joint: {name}")
        if mimic is not None:
            q = float(mimic.get("multiplier", "1")) * resolve(mimic.attrib["joint"], chain+(name,))
            q += float(mimic.get("offset", "0"))
        else:
            q = float(requested.get(name, max(lower, min(upper, 0.0))))
        if not math.isfinite(q) or not lower - 1e-9 <= q <= upper + 1e-9:
            raise ValueError(f"Preview joint outside limits: {name}={q}")
        result[name] = q
        return q

    for name in joints:
        resolve(name)
    return result
