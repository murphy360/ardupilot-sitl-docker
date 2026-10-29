"""Where a torpedo goes: lines, corners, points and circles.

Guidance turns a position into a course over the ground. A torpedo cannot stop and
cannot back up, so every shape ends in a circle: a route that has ended, a point that
is reached, a hold. Given the hull's velocity, guidance also says how fast its course
itself turns (``rate``), and the heading loop feeds that forward.

* A line (from A to B): line of sight. The course is the line's bearing plus
  ``atan(-xte / lookahead)``, with ``xte`` the cross-track error (positive to the
  right of the line) and ``lookahead = LOS_LA * turn_radius_m``.
* A corner: the next line starts when the distance to B along the line is no more than
  ``R * tan(turn / 2)``, limited to [0, 2R], with ``R = TURN_K * turn_radius_m`` (``TURN_K`` taken as 1 when less) and
  ``turn`` the change of course at B. B is then reached. A corner of 5 to 120 degrees
  is rounded on the arc of radius ``R`` that touches both lines (a fillet), flown as a
  circle with its turn rate fed forward, until the hull is as far along the next line
  as the arc reaches; line of sight then holds the line. A sharper corner is turned by
  line of sight alone. The last point of a route is reached within the waypoint
  radius.
* A point: the course is straight at it. Within the waypoint radius it is reached and
  the hull circles it.
* A circle (centre, radius, direction), and a corner's arc: the course is the tangent
  plus ``atan((d - radius) / lookahead)``, ``d`` the distance from the centre and the
  look-ahead half the radius but never less than ``turn_radius_m``. The turn rate fed
  forward is the course's own: the hull's rate round the centre, and how fast it
  closes on the circle. From outside ``JOIN_RATIO`` radii, the hull joins on a tangent.
  The radius is at least ``LOITER_MIN * turn_radius_m``; a smaller ask is raised to it.

No arc is tighter than ``turn_radius_m``, the least radius the hull's model file
gives: ``TURN_K`` is at least 1 by default, so the fins keep a margin to correct with.

Tuning (the values tests hold the acceptance lines with): ``LOS_LA`` 2.0 and
``TURN_K`` 1.2 are the defaults the parameters ``TPDO_LOS_LA`` and ``TPDO_TURN_K``
start at, ``LOITER_MIN`` 1.5 for ``TPDO_LOIT_MIN``.

Positions are (north, east) in m, bearings clockwise from north in rad.
"""

from __future__ import annotations

import math

from .model import Model
from .physics import clamp, wrap

LOS_LA = 2.0
TURN_K = 1.2
LOITER_MIN = 1.5
CIRCLE_K = 2.0  # a circle's look-ahead is its radius over this
FILLET_MIN = math.radians(5.0)  # a corner this slight is not rounded
FILLET_MAX = math.radians(120.0)  # nor one this sharp: line of sight turns it
CIRCLE_I = 0.05  # 1/s: the radius error the circle learns of a steady bias
CIRCLE_BAND = 0.1  # the bias is learnt only this close to the ring, in radii
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
        self.rate: float | None = None  # rad/s: the course's own turn rate, when known
        self.arc = None  # the corner being rounded: (centre, radius, sign, its end)
        self.vel: Point | None = None  # the hull's velocity over the ground, m/s

    # -- what to fly ----------------------------------------------------------------

    def arc_radius(self) -> float:
        """The radius a corner is rounded on, m: ``turn_k`` turn radii, and never
        less than the hull's turn radius (its model's least)."""
        return max(self.turn_k, 1.0) * self.turn_radius

    def min_radius(self) -> float:
        """The least radius of a circle, m."""
        return self.loiter_min * self.turn_radius

    def route(self, points: list[Point]) -> None:
        """Fly the lines through ``points`` (two at least); the first is where the
        first line starts."""
        if len(points) < 2:
            raise ValueError("a route needs two points")
        self.points, self.index, self.mode = list(points), 0, "route"
        self.arc = None
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

    def reframe(self, move) -> None:
        """The vehicle moved its flat frame: every point kept goes through ``move``
        (old north/east to new). Radii, bearings and distances along a line stay."""
        self.points = [move(p) for p in self.points]
        self.centre = move(self.centre)
        if self.arc is not None:
            centre, radius, sign, out = self.arc
            self.arc = move(centre), radius, sign, out

    def pop_reached(self) -> list[int]:
        """The route points reached since the last call."""
        out, self.reached = self.reached, []
        return out

    # -- the course -------------------------------------------------------------------

    def course(self, pos: Point, dt: float = 0.0, vel: Point | None = None) -> float:
        """The course over the ground to steer from ``pos``, rad, ``dt`` s after the
        last call. It also advances the route. With ``vel`` (north, east m/s over the
        ground) it sets ``rate``, the course's own turn rate on a circle or an arc."""
        self.curvature = 0.0
        self.rate = None
        self.vel = vel
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
        r = self.arc_radius()
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
                self.arc = None
                self.circle(b, self.min_radius(), self.clockwise)
                return self._circle_course(pos, 0.0)
            self.arc = self._fillet(self.index)
            self.index += 1
        if self.arc is not None:
            centre, radius, sign, out = self.arc
            if along < out:
                return self._round(pos, centre, radius, sign)
            self.arc = None
        look = self.los_la * self.turn_radius
        return wrap(bearing(a, b) - math.atan(xte / look))

    def _fillet(self, i: int):
        """The arc that rounds the corner at point ``i + 1``: (centre, radius, sign,
        how far along the next line it ends), or None for a corner too slight or too
        sharp to round (it is flown by line of sight alone)."""
        a, b, c = self.points[i], self.points[i + 1], self.points[i + 2]
        chi_in, chi_out = bearing(a, b), bearing(b, c)
        turn = wrap(chi_out - chi_in)
        if not FILLET_MIN < abs(turn) <= FILLET_MAX:
            return None
        r = self.arc_radius()
        sign = 1.0 if turn > 0 else -1.0
        reach = r * math.tan(abs(turn) / 2)
        start = (b[0] - reach * math.cos(chi_in), b[1] - reach * math.sin(chi_in))
        side = chi_in + sign * math.pi / 2  # towards the inside of the turn
        centre = (start[0] + r * math.cos(side), start[1] + r * math.sin(side))
        return centre, r, sign, reach

    def _round(
        self, pos: Point, centre: Point, radius: float, sign: float, bias=0.0
    ) -> float:
        """The course that holds ``pos`` on a circle (``sign`` 1 clockwise), with the
        course's own turn rate fed forward (``rate``)."""
        dn, de = pos[0] - centre[0], pos[1] - centre[1]
        d = math.hypot(dn, de)
        self.curvature = sign / radius
        look = max(radius / CIRCLE_K, self.turn_radius)  # m: the look-ahead
        x = (d - radius + bias) / look
        if self.vel is not None and d > 1e-3:
            vn, ve = self.vel
            about = (dn * ve - de * vn) / (d * d)  # rad/s round the centre
            closing = (dn * vn + de * ve) / d  # m/s away from the centre
            self.rate = about + sign * closing / look / (1.0 + x * x)
        out = math.atan2(de, dn)  # bearing of the hull from the centre
        return wrap(out + sign * (math.pi / 2 + math.atan(x)))

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
        err = d - self.radius
        if abs(err) < CIRCLE_BAND * self.radius:  # learn a steady bias
            self.bias = clamp(
                self.bias + CIRCLE_I * err * dt, -self.radius, self.radius
            )
        return self._round(pos, self.centre, self.radius, sign, self.bias)
