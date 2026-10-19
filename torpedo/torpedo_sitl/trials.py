"""The trials the acceptance lines are measured on: a hull flown in-process.

Each trial builds a hull and its pilot, flies a shape and returns a :class:`Trace`.
They run at hundreds of times real time, with no MAVLink, so a unit test can fly one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .autopilot import Autopilot, fly
from .control import Control, drag_thrust
from .guidance import Point
from .model import Model
from .physics import Hull

DT = 0.05  # s: the pilot's step
DEPTH = 10.0  # m: the depth the trials fly at
LANE_M = 300.0
SPACING_M = 40.0


@dataclass
class Sample:
    """The hull at one step."""

    t: float
    north: float
    east: float
    depth: float
    heading: float
    speed: float  # over the ground
    index: int  # the route's current line


@dataclass
class Trace:
    """What a trial recorded."""

    samples: list[Sample] = field(default_factory=list)
    switches: list[tuple[float, int]] = field(default_factory=list)  # (t, point)
    done_t: float | None = None  # when a route or point was reached and circled


def fly_route(
    model: Model, points: list[Point], speed: float, tail_s: float = 0.0
) -> Trace:
    """Fly ``points`` as a route from its first point, heading along its first line,
    at ``speed`` and ``DEPTH``, until the last point is reached, then ``tail_s`` more."""
    a, b = points[0], points[1]
    heading = math.atan2(b[1] - a[1], b[0] - a[0])
    hull = Hull(model, a[0], a[1], DEPTH, heading, 0.0, speed)
    pilot = Autopilot(model)
    pilot.speed, pilot.depth = speed, DEPTH
    pilot.guidance.route(points)
    trace, t, ended = Trace(), 0.0, None
    limit = 3 * sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))
    while t < limit / speed + tail_s + 60.0:
        act = pilot.step(DT, hull)
        hull.step(DT, act.thrust_n, act.rudder_rad, act.elevator_rad)
        t += DT
        for p in pilot.guidance.pop_reached():
            trace.switches.append((t, p))
        g = pilot.guidance
        idx = g.index if g.mode == "route" else len(points) - 1
        speed_now = hull.ground_speed
        sample = Sample(
            t, hull.north, hull.east, hull.depth, hull.heading, speed_now, idx
        )
        trace.samples.append(sample)
        if g.done and ended is None:
            ended, trace.done_t = t, t
        if ended is not None and t - ended >= tail_s:
            break
    return trace


def turn90(model: Model, speed: float) -> tuple[list[Point], Trace]:
    """A lane, a 90 degree turn and a second lane, ``LANE_M`` each."""
    pts = [(0.0, 0.0), (LANE_M, 0.0), (LANE_M, LANE_M)]
    return pts, fly_route(model, pts, speed)


def lawnmower(model: Model, speed: float, lanes: int = 3) -> tuple[list[Point], Trace]:
    """Lanes ``SPACING_M`` apart, each ``LANE_M`` long, joined by short turn lines:
    the route points are lane ends in the order flown."""
    pts: list[Point] = []
    for k in range(lanes):
        row = k * SPACING_M
        ends = (
            [(0.0, row), (LANE_M, row)] if k % 2 == 0 else [(LANE_M, row), (0.0, row)]
        )
        pts += ends
    return pts, fly_route(model, pts, speed)


def steady_turn(model: Model, speed: float | None = None, seconds: float = 400.0):
    """(radius, speed): the steady turn at full rudder and ``speed`` (cruise by
    default), speed and depth held by the loops. The radius is the speed over the yaw
    rate."""
    speed = model.cruise_speed_mps if speed is None else speed
    hull = Hull(model, depth=DEPTH, speed=speed)
    control = Control(model)
    sign = 1.0
    for _ in range(int(seconds / 0.02)):
        thrust = control.thrust(0.02, speed, hull.u, True)
        pitch = control.pitch_demand(DEPTH, hull.depth, hull.velocity_ned()[2])
        elev = control.elevator(pitch, hull.pitch, hull.q, hull.u)
        hull.step(0.02, thrust, sign * model.fin.max_rad, elev)
    return hull.speed_through_water / abs(hull.r), hull.speed_through_water


def _start(model: Model, pos: Point, heading: float, speed: float):
    hull = Hull(model, pos[0], pos[1], DEPTH, heading, 0.0, speed)
    pilot = Autopilot(model)
    pilot.speed, pilot.depth = speed, DEPTH
    return hull, pilot


def circle(
    model: Model,
    centre: Point,
    radius: float,
    speed: float,
    start: Point,
    heading: float,
    seconds: float,
    clockwise: bool = True,
) -> Trace:
    """Fly a circle about ``centre`` from ``start`` (heading ``heading``) for
    ``seconds``. The guidance raises a radius below the least."""
    hull, pilot = _start(model, start, heading, speed)
    pilot.guidance.circle(centre, radius, clockwise)
    trace = Trace()
    for t, h in fly(hull, pilot, seconds):
        trace.samples.append(
            Sample(t, h.north, h.east, h.depth, h.heading, h.ground_speed, 0)
        )
    return trace


def open_loop_thrust(model: Model, speed: float) -> float:
    """The thrust that holds ``speed`` on a straight line, N."""
    return drag_thrust(model, speed)
