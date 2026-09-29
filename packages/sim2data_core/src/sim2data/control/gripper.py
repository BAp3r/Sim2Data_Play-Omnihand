"""The synthetic scalar interface for the two O10 hand gestures.

The scalar is only an interface convenience.  A map contains all ten current
URDF active hand joints, including channels whose value is fixed for the
selected legacy gesture.  URDF mimic joints are deliberately excluded from
the command map and are left for the articulation to derive.  The values here
are synthetic commissioning assumptions and must not be used as hardware
calibration.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping
import xml.etree.ElementTree as ET


_ACTIVE_SUFFIXES = (
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "index_abad_joint",
    "index_pip_joint",
    "middle_pip_joint",
    "ring_abad_joint",
    "ring_pip_joint",
    "pinky_abad_joint",
    "pinky_pip_joint",
)
_CLOSING_SUFFIXES = {
    "left": ("thumb_mcp_joint", "index_pip_joint", "middle_pip_joint"),
    "right": ("thumb_mcp_joint", "index_pip_joint"),
}


def _prefix(side: str) -> str:
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    return f"{side}_hand__{'l' if side == 'left' else 'R'}_"


def allowed_closing_joint_names(side: str) -> tuple[str, ...]:
    """Return the current-URDF joints allowed to close for this legacy gesture."""
    prefix = _prefix(side)
    return tuple(prefix + suffix for suffix in _CLOSING_SUFFIXES[side])


def _number(value: object, *, field: str, joint: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{joint} {field} must be numeric") from exc
    if not (result == result and abs(result) != float("inf")):
        raise ValueError(f"{joint} {field} must be finite")
    return result


def _named_values(
    values: Mapping[str, object] | list[object] | tuple[object, ...],
    names: tuple[str, ...],
    *,
    label: str,
) -> dict[str, float]:
    if isinstance(values, Mapping):
        if set(values) != set(names):
            raise ValueError(f"{label} joint names do not match the active profile")
        raw_values = {name: values[name] for name in names}
    else:
        if len(values) != len(names):
            raise ValueError(f"{label} count does not match the active profile")
        raw_values = dict(zip(names, values))
    return {name: _number(raw_values[name], field=label, joint=name) for name in names}


def validate_gesture_targets(
    side: str,
    profile_active: list[Mapping[str, object]] | tuple[Mapping[str, object], ...],
    open_values: Mapping[str, object] | list[object] | tuple[object, ...],
    close_values: Mapping[str, object] | list[object] | tuple[object, ...],
) -> tuple[str, ...]:
    """Validate that only this side's legacy gesture channels are free to close.

    ``profile_active`` must describe all ten independent active channels, in
    stable command order. ``open_values`` and ``close_values`` may be name
    mappings or vectors in that order. The returned tuple is the only set a
    planner or optimizer may use as finger-closure variables.
    """
    prefix = _prefix(side)
    names = tuple(str(item["name"]) for item in profile_active)
    expected_names = tuple(prefix + suffix for suffix in _ACTIVE_SUFFIXES)
    if len(names) != len(set(names)) or set(names) != set(expected_names):
        missing = sorted(set(expected_names) - set(names))
        extra = sorted(set(names) - set(expected_names))
        raise ValueError(
            f"{side} active profile must cover the ten current O10 channels; "
            f"missing={missing}, extra={extra}"
        )
    opened = _named_values(open_values, names, label="open")
    closed = _named_values(close_values, names, label="close")
    expected_closing = set(allowed_closing_joint_names(side))
    changed = {name for name in names if abs(closed[name] - opened[name]) > 1e-9}
    if changed != expected_closing:
        raise ValueError(
            f"{side} closure channels must be {sorted(expected_closing)}; got {sorted(changed)}"
        )
    return tuple(name for name in names if name in expected_closing)


@dataclass(frozen=True)
class JointTarget:
    name: str
    open_rad: float
    close_rad: float
    lower_rad: float
    upper_rad: float
    # The legacy repository names and signs are retained as an audit trail.
    # They are optional for backwards compatibility with small plan fixtures,
    # but production commissioning profiles provide all four fields.
    legacy_name: str | None = None
    legacy_sign: float = 1.0
    legacy_offset_rad: float = 0.0
    allow_close: bool = True
    legacy_open_rad: float | None = None
    legacy_close_rad: float | None = None
    gesture_open_adjustment_rad: float = 0.0

    def at(self, amount: float) -> float:
        x = min(1.0, max(0.0, float(amount)))
        # Fixed gesture channels are commanded at their open value for every
        # scalar amount.  This is what keeps thumb roll/abad and the unused
        # fingers from becoming optimizer variables.
        value = self.open_rad if not self.allow_close else self.open_rad + x * (
            self.close_rad - self.open_rad
        )
        return min(self.upper_rad, max(self.lower_rad, value))

    @property
    def is_fixed(self) -> bool:
        """Whether this active channel is held throughout the gesture."""
        return not self.allow_close


@dataclass(frozen=True)
class GripperMap:
    side: str
    joints: tuple[JointTarget, ...]
    synthetic_drive: Mapping[str, float]

    @property
    def command_joint_names(self) -> tuple[str, ...]:
        """All current URDF active joints that may receive a command.

        This is intentionally ten names for the O10 source URDF.  Mimic and
        passive joints never enter this tuple.
        """
        return tuple(joint.name for joint in self.joints)

    @property
    def closing_joint_names(self) -> tuple[str, ...]:
        """Active channels allowed to move from open to closed."""
        return tuple(joint.name for joint in self.joints if joint.allow_close)

    @property
    def fixed_joint_names(self) -> tuple[str, ...]:
        """Active channels held at the open gesture value."""
        return tuple(joint.name for joint in self.joints if not joint.allow_close)

    # Short aliases make the channel sets convenient to pass to optimizers.
    @property
    def closing_joints(self) -> tuple[str, ...]:
        return self.closing_joint_names

    @property
    def fixed_joints(self) -> tuple[str, ...]:
        return self.fixed_joint_names

    @property
    def closing_targets(self) -> tuple[JointTarget, ...]:
        return tuple(joint for joint in self.joints if joint.allow_close)

    @property
    def fixed_targets(self) -> tuple[JointTarget, ...]:
        return tuple(joint for joint in self.joints if not joint.allow_close)

    @property
    def mapping_table(self) -> dict[str, dict[str, float | str | bool | None]]:
        """Return the explicit current-to-legacy mapping audit table."""
        return {
            joint.name: {
                "legacy_name": joint.legacy_name,
                "legacy_sign": joint.legacy_sign,
                "legacy_offset_rad": joint.legacy_offset_rad,
                "legacy_open_rad": joint.legacy_open_rad,
                "legacy_close_rad": joint.legacy_close_rad,
                "gesture_open_adjustment_rad": joint.gesture_open_adjustment_rad,
                "allow_close": joint.allow_close,
                "open_rad": joint.open_rad,
                "close_rad": joint.close_rad,
                "lower_rad": joint.lower_rad,
                "upper_rad": joint.upper_rad,
            }
            for joint in self.joints
        }

    def expand(self, amount: float) -> dict[str, float]:
        """Expand ``amount`` in [0,1] to all current active joint targets."""
        if not 0.0 <= float(amount) <= 1.0:
            raise ValueError("gripper amount must be within [0, 1]")
        return {joint.name: joint.at(amount) for joint in self.joints}

    def open_targets(self) -> dict[str, float]:
        """Return the open gesture for direct post-initialize hand command."""
        return self.expand(0.0)

    def closed_targets(self) -> dict[str, float]:
        """Return the closed gesture, with fixed channels held at open."""
        return self.expand(1.0)

    def prepare_targets(self) -> dict[str, float]:
        """Return the direct open target used after articulation initialization.

        There is intentionally no interpolation from a URDF zero pose here.
        Arm reset and this hand command are separate operations in the runtime.
        """
        return self.open_targets()

    def reset_targets(self) -> dict[str, float]:
        """Alias for the direct open hand reset target."""
        return self.open_targets()

    def prepare(self, amount: float = 0.0) -> dict[str, float]:
        """Return a gesture target without a URDF-zero interpolation step."""
        return self.expand(amount)

    def reset(self) -> dict[str, float]:
        """Return the direct open target after articulation initialization."""
        return self.open_targets()


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
    _prefix(side)  # validate side before reading a potentially large URDF
    raw = json.loads(Path(config).read_text(encoding="utf-8"))
    spec = raw["gripper_commissioning"][side]
    prefix = _prefix(side)
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
        open_rad = _number(item["open_rad"], field="open_rad", joint=name)
        close_rad = _number(item["close_rad"], field="close_rad", joint=name)
        if not lower <= open_rad <= upper:
            raise ValueError(f"{side} {name} open endpoint {open_rad} outside URDF limits [{lower}, {upper}]")
        if not lower <= close_rad <= upper:
            raise ValueError(f"{side} {name} closed endpoint {close_rad} outside URDF limits [{lower}, {upper}]")

        legacy_name = item.get("legacy_name", item.get("legacy_joint"))
        has_mapping = legacy_name is not None or any(
            key in item for key in ("legacy_sign", "mapping_sign", "legacy_offset_rad", "mapping_offset_rad")
        )
        legacy_sign = _number(
            item.get("legacy_sign", item.get("mapping_sign", 1.0)),
            field="legacy_sign",
            joint=name,
        )
        legacy_offset = _number(
            item.get("legacy_offset_rad", item.get("mapping_offset_rad", 0.0)),
            field="legacy_offset_rad",
            joint=name,
        )
        legacy_open = (
            _number(item["legacy_open_rad"], field="legacy_open_rad", joint=name)
            if "legacy_open_rad" in item
            else None
        )
        legacy_close = (
            _number(item["legacy_close_rad"], field="legacy_close_rad", joint=name)
            if "legacy_close_rad" in item
            else None
        )
        open_adjustment = _number(
            item.get("gesture_open_adjustment_rad", 0.0),
            field="gesture_open_adjustment_rad", joint=name,
        )
        if has_mapping:
            if not legacy_name:
                raise ValueError(f"{side} {name} mapping is missing legacy_name")
            if legacy_sign not in (-1.0, 1.0):
                raise ValueError(f"{side} {name} legacy_sign must be -1 or +1")
            # If source endpoints are recorded, verify that the declared sign
            # and offset really produce the current-URDF endpoints.  This
            # prevents silently copying legacy angles into a mirrored URDF.
            for current_key, legacy_key in (("open_rad", "legacy_open_rad"), ("close_rad", "legacy_close_rad")):
                if legacy_key in item:
                    expected = legacy_sign * _number(item[legacy_key], field=legacy_key, joint=name) + legacy_offset
                    if current_key == "open_rad":
                        expected += open_adjustment
                    actual = open_rad if current_key == "open_rad" else close_rad
                    if abs(expected - actual) > 1e-8:
                        raise ValueError(
                            f"{side} {name} {current_key} does not match explicit legacy mapping "
                            f"(sign/offset plus gesture adjustment = {expected}, got {actual})"
                        )

        allow_close = bool(item.get("allow_close", item.get("close_enabled", abs(close_rad - open_rad) > 1e-12)))
        if not allow_close and abs(close_rad - open_rad) > 1e-12:
            raise ValueError(f"{side} fixed channel {name} must have equal open/closed endpoints")
        targets.append(
            JointTarget(
                name,
                open_rad,
                close_rad,
                lower,
                upper,
                str(legacy_name) if legacy_name is not None else None,
                legacy_sign,
                legacy_offset,
                allow_close,
                legacy_open,
                legacy_close,
                open_adjustment,
            )
        )
    if {j.get("name") for j in parsed} != {j.name for j in targets}:
        raise ValueError(f"{side} map does not cover exactly all independent hand joints")
    validate_gesture_targets(
        side,
        spec["active_joints"],
        [target.open_rad for target in targets],
        [target.close_rad for target in targets],
    )
    expected_closing = set(allowed_closing_joint_names(side))
    declared_closing = {target.name for target in targets if target.allow_close}
    if declared_closing != expected_closing:
        raise ValueError(
            f"{side} allow_close declarations must be {sorted(expected_closing)}; "
            f"got {sorted(declared_closing)}"
        )
    return GripperMap(side, tuple(targets), spec["synthetic_drive"])
