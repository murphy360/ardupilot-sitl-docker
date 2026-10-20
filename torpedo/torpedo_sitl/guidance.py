"""Where a torpedo goes: lines, corners, points and circles.

Guidance turns a position into a course. A torpedo cannot stop and cannot back up, so
every shape ends in a circle: a route that has ended, a point that is reached, a hold.

* A line (from A to B): line of sight. The course is the line's bearing plus
  ``atan(-xte / lookahead)``, with ``xte`` the cross-track error (positive to the
  right of the line) and ``lookahead = LOS_LA * turn_radius_m``.
* A corner: the next line starts when the distance to B along the line is no more than
  ``R * tan(turn / 2)``, limited to [0, 2R], with ``R = TURN_K * turn_radius_m`` and
  ``turn`` the change of course at B. B is then reached. The last point of a route is
  reached within the waypoint radius.
* A point: the course is straight at it. Within the waypoint radius it is reached and
  the hull circles it.
* A circle (centre, radius, direction): the course is the tangent plus
  ``atan(CIRCLE_K * (d - radius) / radius)``, ``d`` the distance from the centre.
  From outside, the hull joins on a tangent. The radius is at least
  ``LOITER_MIN * turn_radius_m``; a smaller ask is raised to it.

Tuning (the values tests hold the acceptance lines with): ``LOS_LA`` 2.0 and
``TURN_K`` 1.4 are the defaults the parameters ``TPDO_LOS_LA`` and ``TPDO_TURN_K``
start at, ``LOITER_MIN`` 1.5 for ``TPDO_LOIT_MIN``.

Positions are (north, east) in m, bearings clockwise from north in rad.
"""

from __future__ import annotations

import math

from .model import Model
from .physics import clamp, wrap

LOS_LA = 2.0
TURN_K = 1.4
LOITER_MIN = 1.5
CIRCLE_K = 1.0
CIRCLE_I = 0.05  # 1/s: the radius error the circle learns of a steady bias
JOIN_RATIO = 1.5  # outside this many radii the hull joins on a tangent line

Point = tuple[float, float]


def bearing(a: Point, b: Point) -> float:
    """The bearing from ``a`` to ``b``, rad."""
    return math.atan2(b[1] - a[1], b[0] - a[0])


class Guidance:
    """Course guidance for one hull."""

    def __init__(
        self,
        model: Model,
        wp_radius: float | None = None,
        los_la: float = LOS_LA,
        turn_k: float = TURN_K,
        loiter_min: float = LOITER_MIN,
    ):
        self.turn_radius = model.turn_radius_m
        self.wp_radius = self.turn_radius if wp_radius is None else wp_radius
        self.los_la = los_la
        self.turn_k = turn_k
        self.loiter_min = loiter_min
        self.mode = "idle"  # idle, route, point, circle
        self.points: list[Point] = []
        self.index = 0  # the line flown is points[index] to points[index + 1]
        self.reached: list[int] = []  # indices of the route's points just reached
        self.centre: Point = (0.0, 0.0)
        self.radius = 0.0
        self.clockwise = True
        self.done = False  # a route or a point reached: now circling
        self.curvature = 0.0  # 1/m: how tightly the course bends now (a circle's)
        self.bias = 0.0  # m: the circle's learned radius bias (an integral)

    # -- what to fly ----------------------------------------------------------------

    def min_radius(self) -> float:
        """The least radius of a circle, m."""
        return self.loiter_min * self.turn_radius

    def route(self, points: list[Point]) -> None:
        """Fly the lines through ``points`` (two at least); the first is where the
        first line starts."""
        if len(points) < 2:
            raise ValueError("a route needs two points")
        self.points, self.index, self.mode = list(points), 0, "route"
        self.reached, self.done = [], False

    def goto(self, point: Point) -> None:
        """Fly straight at ``point``, then circle it."""
        self.centre, self.mode, self.done = point, "point", False
        self.reached = []

    def circle(self, centre: Point, radius: float, clockwise: bool = True) -> None:
        """Circle ``centre`` at ``radius`` (raised to the least)."""
        self.centre, self.clockwise = centre, clockwise
        self.radius = max(radius, self.min_radius())
        self.mode = "circle"
        self.bias = 0.0

    def pop_reached(self) -> list[int]:
        """The route points reached since the last call."""
        out, self.reached = self.reached, []
        return out

    # -- the course -------------------------------------------------------------------

    def course(self, pos: Point, dt: float = 0.0) -> float:
        """The course to steer from ``pos``, rad, ``dt`` s after the last call. It
        also advances the route."""
        self.curvature = 0.0
        if self.mode == "route":
            return self._route_course(pos)
        if self.mode == "point":
            return self._point_course(pos)
        if self.mode == "circle":
            return self._circle_course(pos, dt)
        raise RuntimeError("guidance has nothing to fly")

    def line_error(self, pos: Point, a: Point, b: Point) -> tuple[float, float]:
        """(along, xte): the distance along the line from ``a``, and the distance
        to the right of it."""
        chi = bearing(a, b)
        dn, de = pos[0] - a[0], pos[1] - a[1]
        along = dn * math.cos(chi) + de * math.sin(chi)
        xte = -dn * math.sin(chi) + de * math.cos(chi)
        return along, xte

    def corner_distance(self) -> float:
        """How far before the end of the current line the next one starts, m."""
        i = self.index
        if i + 2 >= len(self.points):
            return self.wp_radius
        turn = wrap(
            bearing(self.points[i + 1], self.points[i + 2])
            - bearing(self.points[i], self.points[i + 1])
        )
        r = self.turn_k * self.turn_radius
        half = abs(turn) / 2
        reach = 2 * r if half >= math.pi / 2 - 1e-6 else r * math.tan(half)
        return clamp(reach, 0.0, 2 * r)

    def _route_course(self, pos: Point) -> float:
        while True:
            a, b = self.points[self.index], self.points[self.index + 1]
            along, xte = self.line_error(pos, a, b)
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length - along > self.corner_distance():
                break
            self.reached.append(self.index + 1)
            if self.index + 2 >= len(self.points):
                self.done = True
                self.circle(b, self.min_radius(), self.clockwise)
                return self._circle_course(pos, 0.0)
            self.index += 1
        look = self.los_la * self.turn_radius
        return wrap(bearing(a, b) - math.atan(xte / look))

    def _point_course(self, pos: Point) -> float:
        if (
            math.hypot(self.centre[0] - pos[0], self.centre[1] - pos[1])
            <= self.wp_radius
        ):
            self.done = True
            self.reached.append(0)
            self.circle(self.centre, self.min_radius(), self.clockwise)
            return self._circle_course(pos, 0.0)
        return bearing(pos, self.centre)

    def _circle_course(self, pos: Point, dt: float) -> float:
        dn, de = pos[0] - self.centre[0], pos[1] - self.centre[1]
        d = math.hypot(dn, de)
        sign = 1.0 if self.clockwise else -1.0
        if d > JOIN_RATIO * self.radius:
            to_centre = bearing(pos, self.centre)
            return wrap(to_centre - sign * math.asin(self.radius / d))
        self.curvature = sign / self.radius
        err = d - self.radius
        if abs(err) < self.radius:  # learn a steady bias, as a current or the fins'
            self.bias = clamp(
                self.bias + CIRCLE_I * err * dt, -self.radius, self.radius
            )
        out = math.atan2(de, dn)  # bearing of the hull from the centre
        tangent = out + sign * math.pi / 2
        return wrap(
            tangent + sign * math.atan(CIRCLE_K * (err + self.bias) / self.radius)
        )
