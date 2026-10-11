"""Bounded, reproducible initial-pose variation for synthetic relay trials."""
import math
import random


def initial_pose(seed, reference_center):
    if type(seed) is not int:
        raise ValueError('seed must be an integer')
    if len(reference_center) != 3 or not all(math.isfinite(x) for x in reference_center):
        raise ValueError('finite reference center required')
    rng = random.Random(seed)
    dx, dy = (rng.uniform(-.005, .005) for _ in range(2))
    yaw = rng.uniform(-math.pi/60, math.pi/60)
    return dict(seed=seed, synthetic=True, kind='initial_box_xy_yaw',
                xy_half_range_m=.005, yaw_half_range_rad=math.pi/60,
                offset_xy_m=[dx,dy], yaw_rad=yaw,
                box_center_m=[reference_center[0]+dx,reference_center[1]+dy,reference_center[2]],
                quaternion_wxyz=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)],
                reset_only=True, mass_friction_drives_cameras_unchanged=True)
