"""The measures of the acceptance lines, taken from a trial's trace.

Positions are (north, east) in m. A lane is ``(a, b)``, its two ends in the order
flown. ``run_out`` is measured as Proteus measures it (``run_out_m``): each sample
belongs to the lane nearest to it, and a lane's run-out is the furthest any of its
samples reaches along the lane past its end.
"""

from __future__ import annotations

import math
import statistics

from .guidance import Point, bearing
from .physics import wrap
from .trials import Sample, Trace

XTE_M = 2.0  # m: a hull is on its line within this
SETTLE_M = 60.0  # m: tools/speed_hold.py skips this much of a leg's start
END_M = 30.0  # m: and this much of its end
DEADBAND_M = 0.25  # m: a cross-track error this small is on the line


def lane_frame(pos: Point, lane: tuple[Point, Point]) -> tuple[float, float]:
    """(along, xte) of ``pos`` on ``lane``: metres from its start, and to the right."""
    a, b = lane
    chi = bearing(a, b)
    dn, de = pos[0] - a[0], pos[1] - a[1]
    return (
        dn * math.cos(chi) + de * math.sin(chi),
        -dn * math.sin(chi) + de * math.cos(chi),
    )


def lane_length(lane: tuple[Point, Point]) -> float:
    """The length of ``lane``, m."""
    return math.dist(*lane)


def segment_distance(pos: Point, lane: tuple[Point, Point]) -> float:
    """The distance from ``pos`` to the segment ``lane``, m."""
    along, xte = lane_frame(pos, lane)
    length = lane_length(lane)
    if 0.0 <= along <= length:
        return abs(xte)
    return math.dist(pos, lane[0] if along < 0 else lane[1])


def nearest_lane(pos: Point, lanes: list[tuple[Point, Point]]) -> int:
    """The index of the lane nearest to ``pos``."""
    return min(range(len(lanes)), key=lambda k: segment_distance(pos, lanes[k]))


def run_out(trace: Trace, lanes: list[tuple[Point, Point]]) -> list[float]:
    """Per lane: how far the turn reaches along the lane past its end, m."""
    out = [0.0] * len(lanes)
    for s in trace.samples:
        pos = (s.north, s.east)
        k = nearest_lane(pos, lanes)
        along, _ = lane_frame(pos, lanes[k])
        out[k] = max(out[k], along - lane_length(lanes[k]))
    return out


def lane_end_time(trace: Trace, point: int) -> float | None:
    """When route point ``point`` was reached (the corner), or None."""
    for t, p in trace.switches:
        if p == point:
            return t
    return None


def straight(trace: Trace, points: list[Point], k: int) -> list[tuple[Sample, float]]:
    """The settled part of line ``k`` of the route: [(sample, xte)] from the line's
    first crossing until the next line starts (or the trace ends)."""
    lane = (points[k], points[k + 1])
    out = []
    for s in trace.samples:
        if s.index != k:
            continue
        out.append((s, lane_frame((s.north, s.east), lane)[1]))
    crossing = next(
        (
            i
            for i, (_, x) in enumerate(out)
            if abs(x) <= DEADBAND_M or x * out[0][1] < 0
        ),
        None,
    )
    return [] if crossing is None else out[crossing:]


def sign_changes(xtes: list[float]) -> int:
    """How often the cross-track error changes sign, ignoring the dead band."""
    last, n = 0, 0
    for x in xtes:
        if abs(x) <= DEADBAND_M:
            continue
        sign = 1 if x > 0 else -1
        if last and sign != last:
            n += 1
        last = sign
    return n


def settle_time(trace: Trace, points: list[Point], k: int) -> float:
    """Seconds from the corner that starts line ``k`` until the cross-track error on
    it stays within ``XTE_M``."""
    start = lane_end_time(trace, k)
    lane = (points[k], points[k + 1])
    t_out = start
    for s in trace.samples:
        if s.index != k or s.t < start:
            continue
        if abs(lane_frame((s.north, s.east), lane)[1]) > XTE_M:
            t_out = s.t
    return t_out - start


def rms(values: list[float]) -> float:
    """The root mean square."""
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def speed_heading_rms(
    trace: Trace, points: list[Point], k: int, speed: float
) -> tuple[float, float]:
    """(speed error m/s, heading error deg), rms, on line ``k`` from ``SETTLE_M`` m
    after its start to ``END_M`` m before its end: the stretch tools/speed_hold.py
    measures (#1849)."""
    lane = (points[k], points[k + 1])
    chi = bearing(*lane)
    length = lane_length(lane)
    part = []
    for s in trace.samples:
        if s.index != k:
            continue
        along, _ = lane_frame((s.north, s.east), lane)
        if SETTLE_M <= along <= length - END_M:
            part.append(s)
    return (
        rms([s.speed - speed for s in part]),
        rms([math.degrees(wrap(s.heading - chi)) for s in part]),
    )


def median_radius_error(samples: list[Sample], centre: Point, radius: float) -> float:
    """The median of ``|d - radius|`` over ``samples``, m."""
    errs = [abs(math.dist((s.north, s.east), centre) - radius) for s in samples]
    return statistics.median(errs) if errs else math.inf
