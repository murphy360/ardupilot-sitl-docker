"""AUTO: the mission, flown as a torpedo flies it.

AUTO starts at the mission's current item (from item 1 when it had ended) and walks the
items. A run of ``NAV_WAYPOINT`` items is one route for the guidance, so a corner is cut
by the corner rule and the next line starts early; the first line starts where AUTO
began (or at the last point reached). Each waypoint reached sends
``MISSION_ITEM_REACHED``.

* ``NAV_WAYPOINT``: a point at the item's depth (its frame's: GLOBAL_RELATIVE_ALT is
  ``-z``, GLOBAL is the home altitude less ``z``, TERRAIN_ALT a height above the 50 m
  floor). ``param2`` is its acceptance radius when set;
* ``DO_CHANGE_SPEED``: the speed, when the item before it is reached;
* ``NAV_LOITER_UNLIM``: the circle about the point at the least loiter radius, for
  good (the return route's end);
* ``NAV_LOITER_TURNS``: ``param1`` turns of the circle, ``param3`` its radius in
  metres (negative: counter-clockwise). A radius under the least loiter radius is
  raised to it, with a ``STATUSTEXT``. Joined on a tangent. Reached after the turns;
* ``DO_LAND_START``: a marker, passed over. Any other item is skipped, with a
  ``STATUSTEXT``;
* the end of the mission: the vehicle circles the last point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control import Actuators
from .mission import (
    DO_CHANGE_SPEED,
    DO_LAND_START,
    NAV_LOITER_TURNS,
    NAV_LOITER_UNLIM,
    NAV_WAYPOINT,
    Item,
)
from .physics import wrap

SEABED_M = 50.0
GLOBAL, GLOBAL_INT = 0, 5
RELATIVE_ALT, RELATIVE_ALT_INT = 3, 6
TERRAIN_ALT, TERRAIN_ALT_INT = 10, 11
JOIN_RATIO = 1.5  # inside this many radii the hull has joined the circle
INFO = 6


@dataclass
class Entry:
    """A waypoint of the run being flown."""

    seq: int
    point: tuple[float, float]
    depth: float
    radius: float
    after: list[int]  # the DO items that follow it, run when it is reached


class AutoFlight:
    """The AUTO mode of one vehicle."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.entries: list[Entry] = []
        self.last_point: tuple[float, float] | None = None
        self.last_depth = 0.0
        self.loiter: dict | None = None  # the loiter being flown
        self.depth = 0.0

    # -- the items -------------------------------------------------------------------

    def depth_of(self, item: Item) -> float:
        """The depth an item asks for, m (positive down, never above the surface)."""
        z, v = item.z, self.v
        if item.frame in (RELATIVE_ALT, RELATIVE_ALT_INT):
            depth = -z
        elif item.frame in (TERRAIN_ALT, TERRAIN_ALT_INT):
            depth = SEABED_M - z
        else:
            depth = v.home_alt - z
        return min(max(depth, 0.0), v.model.depth_limit_m)

    def point_of(self, item: Item) -> tuple[float, float]:
        """An item's place in the flat frame."""
        return self.v.frame.to_ne(item.x / 1e7, item.y / 1e7)

    def start(self) -> bool:
        """Begin AUTO; False when there is nothing to fly."""
        mission = self.v.mission
        if not mission.has_commands:
            return False
        if mission.complete or mission.current < 1:
            mission.set_current(1)
        self.entries, self.loiter = [], None
        self.last_point = None
        self.last_depth = self.v.hull.depth
        self.depth = self.v.hull.depth
        self._begin(mission.current)
        return True

    restart = start

    def _begin(self, seq: int) -> None:
        """Walk the items from ``seq`` until one that takes time starts."""
        mission = self.v.mission
        while True:
            item = mission.item(seq)
            if item is None:
                self._end()
                return
            mission.current = seq
            mission.changed = True
            if item.command == NAV_WAYPOINT:
                self._run(seq)
                return
            if item.command in (NAV_LOITER_UNLIM, NAV_LOITER_TURNS):
                self._start_loiter(item)
                return
            self._do(item)
            seq += 1

    def _do(self, item: Item) -> None:
        if item.command == DO_CHANGE_SPEED:
            if item.param2 > 0:
                self.v.change_speed(item.param2)
        elif item.command != DO_LAND_START:
            self.v.say(f"Skipping command {item.command} (item {item.seq})", INFO)

    def _run(self, seq: int) -> None:
        """A run of waypoints from ``seq`` is one route."""
        v, mission = self.v, self.v.mission
        self.entries = []
        n = seq
        while (item := mission.item(n)) is not None:
            if item.command == NAV_WAYPOINT:
                radius = item.param2 if item.param2 > 0 else 0.0
                self.entries.append(
                    Entry(n, self.point_of(item), self.depth_of(item), radius, [])
                )
            elif item.command in (NAV_LOITER_UNLIM, NAV_LOITER_TURNS):
                break
            elif self.entries:
                self.entries[-1].after.append(n)
            n += 1
        start = self.last_point or v.position
        points = [start] + [e.point for e in self.entries]
        if len(points) < 2 or math.dist(points[0], points[1]) < 1e-6:
            points[0] = (points[0][0] - 1e-3, points[0][1])  # a line needs a length
        v.pilot.guidance.route(points)
        self._radius(self.entries[-1].radius)

    def _radius(self, radius: float) -> None:
        self.v.wp_radius_override = radius if radius > 0 else None

    def _end(self) -> None:
        """Past the last item: circle the last point."""
        v, mission = self.v, self.v.mission
        mission.complete = True
        mission.current = max(mission.count - 1, 0)
        mission.changed = True
        centre = self.last_point or v.position
        v.pilot.guidance.circle(centre, v.pilot.guidance.min_radius(), True)
        self.depth, self.entries, self.loiter = self.last_depth, [], None

    def _start_loiter(self, item: Item) -> None:
        v, g = self.v, self.v.pilot.guidance
        radius, least = abs(item.param3), g.min_radius()
        if item.command == NAV_LOITER_TURNS and 0 < radius < least:
            v.say(f"Loiter radius raised to {least:.0f} m (item {item.seq})", INFO)
        self.depth = self.depth_of(item)
        centre = self.point_of(item)
        g.circle(
            centre,
            max(radius, least) if item.command == NAV_LOITER_TURNS else least,
            item.param3 >= 0,
        )
        turns = item.param1 if item.param1 > 0 else 1.0
        self.loiter = {
            "item": item,
            "centre": centre,
            "turns": turns if item.command == NAV_LOITER_TURNS else math.inf,
            "angle": 0.0,
            "last": None,
        }
        self._radius(0.0)

    # -- the step --------------------------------------------------------------------

    def actuators(self, dt: float) -> Actuators:
        """The actuators for one step of AUTO."""
        v, g = self.v, self.v.pilot.guidance
        if self.loiter is None and self.entries and g.mode == "route":
            self.depth = self.entries[min(g.index, len(self.entries) - 1)].depth
        act = v._fly_guidance(dt, v.speed, self.depth)
        for k in g.pop_reached():
            self._reached(k)
        if self.loiter is not None:
            self._turns()
        return act

    def _reached(self, k: int) -> None:
        """Point ``k`` of the route (1 is the first waypoint) was reached."""
        if not 1 <= k <= len(self.entries):
            return
        v, entry = self.v, self.entries[k - 1]
        v.reached.append(entry.seq)
        self.last_point, self.last_depth = entry.point, entry.depth
        for seq in entry.after:
            self._do(v.mission.item(seq))
        if k == len(self.entries):
            self._begin(entry.after[-1] + 1 if entry.after else entry.seq + 1)
        else:
            v.mission.current = self.entries[k].seq
            v.mission.changed = True

    def _turns(self) -> None:
        """Count the turns of a ``NAV_LOITER_TURNS``; it is reached after them."""
        lo, v = self.loiter, self.v
        n, e = v.position
        d = math.hypot(n - lo["centre"][0], e - lo["centre"][1])
        g = v.pilot.guidance
        if d > JOIN_RATIO * g.radius:
            return
        ang = math.atan2(e - lo["centre"][1], n - lo["centre"][0])
        if lo["last"] is not None:
            sign = 1.0 if g.clockwise else -1.0
            lo["angle"] += sign * wrap(ang - lo["last"])
        lo["last"] = ang
        if lo["angle"] >= 2 * math.pi * lo["turns"]:
            item = lo["item"]
            v.reached.append(item.seq)
            self.last_point, self.last_depth = lo["centre"], self.depth
            self.loiter = None
            self._begin(item.seq + 1)
