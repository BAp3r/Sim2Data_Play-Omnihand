"""Time sampling on already screened waypoint edges, without changing the path."""
import math


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
