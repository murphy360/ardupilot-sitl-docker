"""GUIDED's targets: ``SET_POSITION_TARGET_GLOBAL_INT``, as ArduSub takes it.

* a position (the type mask does not ignore it): a point at a depth. GLOBAL_INT's
  depth is the home altitude less ``alt``, GLOBAL_RELATIVE_ALT_INT's is ``-alt``. A
  position with a velocity of zero, or alone, is the same point; sent again it is not
  restarted;
* GLOBAL_TERRAIN_ALT_INT: ``alt`` is a height above the bed the rangefinder senses
  (``terrain.py``), followed while the point is flown. With a velocity it is dropped:
  ArduSub converts a position with a velocity through the origin, which a terrain
  altitude cannot be, and stops there. Alone it is dropped too when nothing is sensed
  and the target before it was not in the terrain frame (ArduSub's waypoint
  controller cannot convert its origin then);
* a velocity alone: a course and speed (between the steering and the top speed) at
  the depth held. It lasts ``VELOCITY_TIMEOUT_S`` (``vehicle.py``) without another.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .physics import clamp
from .terrain import is_terrain

FRAME_GLOBAL_INT = 5
FRAME_RELATIVE_ALT_INT = 6
POSITION_IGNORED = 0b111
VELOCITY_IGNORED = 0b111000


@dataclass
class Target:
    """A GUIDED target: a point at a depth, or a course and speed."""

    kind: str  # "point" or "velocity"
    depth: float
    north: float = 0.0
    east: float = 0.0
    course: float = 0.0
    speed: float = 0.0
    at: float = 0.0  # vehicle time it arrived
    height: float | None = None  # m above the bed: a terrain-frame point


def position_target(
    v,
    frame: int,
    mask: int,
    lat: float,
    lon: float,
    alt: float,
    vel: tuple[float, float],
) -> tuple[bool, str]:
    """``SET_POSITION_TARGET_GLOBAL_INT`` (degrees, metres, m/s north and east) for
    vehicle ``v`` in GUIDED: (True, "") or (False, why)."""
    m = v.model
    if is_terrain(frame) and (mask & POSITION_IGNORED) != POSITION_IGNORED:
        return _terrain_target(v, mask, lat, lon, alt)
    if (mask & POSITION_IGNORED) != POSITION_IGNORED:
        depth = _depth_of(v, frame, alt)
        if depth is None:
            return False, f"frame {frame} is not supported"
        depth = clamp(depth, 0.0, m.depth_limit_m)
        n, e = v.frame.to_ne(lat, lon)
        if _same_point(v, n, e, depth, None):
            return True, ""  # the same point again: a held target is not restarted
        v.target = Target("point", depth, n, e, at=v.t)
        v.pilot.guidance.goto((n, e))
        return True, ""
    if (mask & VELOCITY_IGNORED) != VELOCITY_IGNORED:
        vn, ve = vel
        depth = v.target.depth if v.target else v.hull.depth
        speed = math.hypot(vn, ve)
        course = math.atan2(ve, vn) if speed > 1e-3 else v.hull.heading
        speed = clamp(speed, m.steer_speed_mps, m.max_speed_mps)
        v.target = Target("velocity", depth, course=course, speed=speed, at=v.t)
        return True, ""
    return False, "no position or velocity in the target"


def _terrain_target(v, mask, lat, lon, height) -> tuple[bool, str]:
    """A terrain-frame position: with a velocity ArduSub converts it through the
    origin, which a terrain altitude cannot be, and drops it. Alone it is flown
    at ``height`` above the bed; it is dropped too when there is no terrain and
    the target before it was not a terrain one (ArduSub's waypoint controller
    cannot convert its origin)."""
    if mask & VELOCITY_IGNORED == 0:
        return False, "terrain target with a velocity"
    old, bed = v.target, v.rangefinder.bed()
    was_terrain = old is not None and old.height is not None
    if bed is None and not was_terrain:
        return False, "no terrain data"
    n, e = v.frame.to_ne(lat, lon)
    if _same_point(v, n, e, None, height):
        return True, ""
    depth = old.depth if bed is None else bed - height
    depth = clamp(depth, 0.0, v.model.depth_limit_m)
    v.target = Target("point", depth, n, e, at=v.t, height=height)
    v.pilot.guidance.goto((n, e))
    return True, ""


def _same_point(v, n, e, depth, height) -> bool:
    """The point target in force is this one: at ``depth``, or at ``height``
    above the bed."""
    old = v.target
    if old is None or old.kind != "point" or (old.height is None) != (height is None):
        return False
    if math.hypot(old.north - n, old.east - e) >= 0.05:
        return False
    if height is None:
        return abs(old.depth - depth) < 0.05
    return abs(old.height - height) < 0.05


def _depth_of(v, frame: int, alt: float) -> float | None:
    if frame == FRAME_GLOBAL_INT:
        return v.home_alt - alt
    if frame == FRAME_RELATIVE_ALT_INT:
        return -alt
    return None
