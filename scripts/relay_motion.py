"""Time sampling on already screened waypoint edges, without changing the path."""
import math
import numbers
from collections.abc import Mapping


_MISSING = object()
_MOTION_TIMING_DEFAULTS = {"right_lower_s": 4.0, "right_release_s": 2.0}


def resolve_motion_timing(config=_MISSING):
    """Resolve the approved synthetic timing-only inputs for the right arm.

    The returned mapping is fresh for every call. Only right lower and release
    durations are configurable; all values are seconds in the inclusive range
    [1, 10].
    """
    if config is _MISSING:
        config = {}
    if not isinstance(config, Mapping):
        raise ValueError("motion_timing must be an object")
    unknown = set(config) - set(_MOTION_TIMING_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown motion_timing keys: {sorted(map(str, unknown))}")
    resolved = dict(_MOTION_TIMING_DEFAULTS)
    for key, value in config.items():
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ValueError(f"motion_timing.{key} must be a finite number of seconds")
        try:
            seconds = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"motion_timing.{key} must be a finite number of seconds") from exc
        if not math.isfinite(seconds) or not 1.0 <= seconds <= 10.0:
            raise ValueError(f"motion_timing.{key} must be between 1 and 10 seconds")
        resolved[key] = seconds
    return resolved


def sample_waypoints(waypoints, progress, stop_indices=()):
    """Use equal-duration legs with zero velocity at explicitly stopped corners.

    No spline shortcuts are taken: every output lies on an original screened
    joint-space edge. With no stops this matches the existing global smoothstep.
    This is synthetic timing, not a robot velocity/acceleration calibration.
    """
    if len(waypoints)<2 or not math.isfinite(progress):
        raise ValueError("invalid waypoint path/progress")
    ends=[0,*stop_indices,len(waypoints)-1]
    if any(type(i) is not int for i in ends) or any(a>=b for a,b in zip(ends,ends[1:])):
        raise ValueError("stop indices must be ordered interior waypoint indices")
    t=min(1.,max(0.,progress))*(len(ends)-1)
    leg=min(int(t),len(ends)-2);local=t-leg
    blend=local*local*(3-2*local)
    x=ends[leg]+blend*(ends[leg+1]-ends[leg]);j=min(int(x),len(waypoints)-2)
    fraction=x-j
    return [a+fraction*(b-a) for a,b in zip(waypoints[j],waypoints[j+1],strict=True)]
