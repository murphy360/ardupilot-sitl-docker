"""Path following (issue #8): circles without a standing error or a weave, joined
without a large overshoot; corners rounded on an arc and settled quickly; a cross
current crabbed out. Measured as the vehicle interface measures a run: from the first
sample within 10 m of a ring, the mean and the median of the radius error."""

import math
import statistics

import pytest

from torpedo_sitl import guidance, measures, trials
from torpedo_sitl import vehicle as veh
from torpedo_sitl.autopilot import Autopilot, fly
from torpedo_sitl.control import Command, Control
from torpedo_sitl.mission import DO_CHANGE_SPEED, NAV_LOITER_TURNS, NAV_LOITER_UNLIM
from torpedo_sitl.physics import Hull

from .support import home_item, make_vehicle, plain_item, run, waypoint

RING_M = 30.0
JOIN_M = 10.0  # a sample this close to the ring has joined it


def _errors(samples, centre, radius):
    """d - r of each sample from the first one within ``JOIN_M`` of the ring."""
    d = [math.dist((s.north, s.east), centre) - radius for s in samples]
    first = next(i for i, e in enumerate(d) if abs(e) <= JOIN_M)
    return d[first:]


def _ring(model, start, heading, speed=2.0, seconds=None):
    turn = 2 * math.pi * RING_M / speed
    seconds = seconds or 2 * turn + 150.0
    trace = trials.circle(model, (0.0, 0.0), RING_M, speed, start, heading, seconds)
    return trace, turn


@pytest.mark.parametrize(
    "start, heading, over",
    [((-100.0, 0.0), 0.0, 2.0), ((0.0, -120.0), 90.0, 2.0), ((0.0, 0.0), 0.0, 4.0)],
    ids=["outside", "abeam", "centre"],
)
def test_a_ring_is_joined_and_held(hull_model, start, heading, over):
    """A 30 m ring at 2 m/s from outside, from abeam and from the centre: from the
    first sample within 10 m of the ring, the mean radius error is under 1 m, the
    median within 0.5 m (no bias), and once on the ring the hull never crosses it by
    more than 2 m (no overshoot on joining, no weave). From the centre the hull
    leaves straight out, and a hull that turns no tighter than half the ring cannot
    turn onto it without running past it: 4 m there."""
    trace, _ = _ring(hull_model, start, heading)
    errs = _errors(trace.samples, (0.0, 0.0), RING_M)
    assert statistics.mean(abs(e) for e in errs) < 1.0
    assert abs(statistics.median(errs)) < 0.5
    on = next(i for i, e in enumerate(errs) if abs(e) < 0.5)
    assert max(abs(e) for e in errs[on:]) < over


def test_a_ring_has_no_standing_error(hull_model):
    """Over the second turn the radius error stays within 0.5 m: the rudder has learnt
    the steady turn and the heading loop holds the course over the ground."""
    trace, turn = _ring(hull_model, (-100.0, 0.0), 0.0)
    end = trace.samples[-1].t
    late = [s for s in trace.samples if s.t >= end - turn]
    errs = [abs(math.hypot(s.north, s.east) - RING_M) for s in late]
    assert max(errs) < 0.5


def test_the_ring_is_flown_at_the_speed_asked(hull_model):
    """On the ring the ground speed is within 3 % of the speed asked."""
    trace, turn = _ring(hull_model, (-100.0, 0.0), 0.0)
    end = trace.samples[-1].t
    speeds = [s.speed for s in trace.samples if s.t >= end - turn]
    assert statistics.mean(speeds) == pytest.approx(2.0, rel=0.03)


@pytest.mark.parametrize("speed", [1.5, 2.3])
def test_a_corner_is_rounded_and_settles(hull_model, speed):
    """A 90 degree corner: no run-out past the lane's end, within 2 m of the next lane
    15 s after the corner, and the turn never carries the hull more than 1 m past
    the next lane."""
    points, trace = trials.turn90(hull_model, speed)
    lanes = [(points[0], points[1]), (points[1], points[2])]
    assert measures.run_out(trace, lanes)[0] < 0.5
    assert measures.settle_time(trace, points, 1) <= 15.0
    after = [
        measures.lane_frame((s.north, s.east), lanes[1])[1]
        for s in trace.samples
        if s.index == 1
    ]
    assert min(after) > -1.0 and max(after) < 1.0 + abs(after[0])


def test_a_corner_arc_is_never_tighter_than_the_hull_turns(hull_model):
    """The arc a corner is rounded on has the hull's turn radius at least, whatever
    TPDO_TURN_K asks."""
    for turn_k in (0.5, 1.0, 1.2, 2.0):
        g = guidance.Guidance(hull_model, turn_k=turn_k)
        g.route([(0.0, 0.0), (300.0, 0.0), (300.0, 300.0)])
        centre, radius, sign, reach = g._fillet(0)
        assert radius >= hull_model.turn_radius_m
        assert sign == 1.0  # a right turn, clockwise
        assert reach == pytest.approx(radius)  # 90 degrees: R tan 45
        assert math.dist(centre, (300.0 - radius, radius)) < 1e-6


def test_a_slight_or_a_sharp_corner_is_not_rounded(hull_model):
    g = guidance.Guidance(hull_model)
    g.route([(0.0, 0.0), (300.0, 0.0), (600.0, 10.0), (300.0, 10.0)])
    assert g._fillet(0) is None  # 2 degrees
    assert g._fillet(1) is None  # a U-turn


def test_a_cross_current_is_crabbed_out(hull_model):
    """A lane flown across a 0.3 m/s current: the hull crabs into it and holds the
    line within 0.5 m (line of sight alone would hold it several metres downstream)."""
    hull = Hull(hull_model, 0.0, 0.0, trials.DEPTH, 0.0, 0.0, 1.5)
    pilot = Autopilot(hull_model)
    pilot.speed, pilot.depth = 1.5, trials.DEPTH
    pilot.guidance.route([(0.0, 0.0), (2000.0, 0.0)])
    xte = []
    for t, h in fly(hull, pilot, 600.0, current=(0.0, 0.3)):
        if t > 300.0:
            xte.append(h.east)
    assert max(abs(x) for x in xte) < 0.5
    assert abs(math.remainder(hull.heading, 2 * math.pi)) > math.radians(5)  # it crabs


def test_a_heading_is_held_as_a_heading(hull_model):
    """A command that is a heading (SURFACE's) is not corrected for the current."""
    hull = Hull(hull_model, 0.0, 0.0, trials.DEPTH, 0.0, 0.0, 1.5)
    control = Control(hull_model)
    for _ in range(int(120 / 0.05)):
        act = control.update(0.05, hull, Command(0.0, 1.5, 0.0, over_ground=False))
        hull.step(0.05, act.thrust_n, act.rudder_rad, act.elevator_rad, (0.0, 0.3))
    assert abs(math.remainder(hull.heading, 2 * math.pi)) < math.radians(1)


def test_what_the_rudder_learnt_on_a_circle_fades_off_it(hull_model):
    hull = Hull(hull_model, -100.0, 0.0, trials.DEPTH, 0.0, 0.0, 1.5)
    pilot = Autopilot(hull_model)
    pilot.speed, pilot.depth = 1.5, trials.DEPTH
    pilot.guidance.circle((0.0, 0.0), RING_M)
    for _ in fly(hull, pilot, 300.0):
        pass
    assert abs(pilot.control.rate_i) > 0.01
    pilot.guidance.route([(hull.north, hull.east), (hull.north + 1000.0, hull.east)])
    for _ in fly(hull, pilot, 20.0):
        pass
    assert abs(pilot.control.rate_i) < 0.01


# -- in AUTO, as the vehicle interface flies it ----------------------------------------


def _loiter(model, start, heading, radius, speed, seconds):
    """A NAV_LOITER_TURNS about (0, 0) in AUTO; the hull's track (north, east)."""
    v, _ = make_vehicle(model, FS_PILOT_INPUT=0, FS_GCS_ENABLE=0)
    run(v, 6.0)
    h = v.hull
    h.north, h.east, h.heading = start[0], start[1], math.radians(heading)
    h.depth = v.world.depth = 4.0
    h.u = 1.5
    v.world.north, v.world.east = start
    items = [
        home_item(v),
        plain_item(1, DO_CHANGE_SPEED, param1=1, param2=speed),
        waypoint(v, 2, 0, 0, 4.0, command=NAV_LOITER_TURNS, param1=255, param3=radius),
        waypoint(v, 3, 0, 0, 4.0, command=NAV_LOITER_UNLIM),
    ]
    v.new_mission(items)
    assert v.arm(True)[0] and v.set_mode(veh.AUTO)[0]
    track = []
    for _ in range(int(seconds / 0.05)):
        v.step(0.05)
        track.append((v.hull.north, v.hull.east))
    return v, track


def test_an_orbit_in_auto_meets_the_line(hull_model):
    """NAV_LOITER_TURNS of 30 m at 2 m/s joined from 110 m out: from the first point
    within 10 m of the ring, the mean radius error is 2 m or less and the median
    within 2 m (the vehicle interface's orbit line)."""
    _, track = _loiter(hull_model, (-110.0, 0.0), 90.0, RING_M, 2.0, 250.0)
    d = [math.dist(p, (0.0, 0.0)) - RING_M for p in track]
    first = next(i for i, e in enumerate(d) if abs(e) <= JOIN_M)
    assert statistics.mean(abs(e) for e in d[first:]) <= 2.0
    assert abs(statistics.median(d[first:])) <= 2.0


def test_a_hold_at_the_least_radius_meets_the_line(hull_model):
    """A hold (NAV_LOITER_TURNS at the least loiter radius, 1.5 m/s) joined from far:
    after its first half turn the hull stays within the ring plus 10 m, and the median
    distance is within 2 m of the ring (the vehicle interface's hold line)."""
    v, track = _loiter(hull_model, (-150.0, -40.0), 0.0, 1.0, 1.5, 400.0)
    ring = v.pilot.guidance.radius
    angle, last, half = 0.0, None, None
    for i, (n, e) in enumerate(track):
        a = math.atan2(e, n)
        if last is not None:
            angle += math.remainder(a - last, 2 * math.pi)
        last = a
        if abs(angle) >= math.pi:
            half = i
            break
    d = [math.dist(p, (0.0, 0.0)) for p in track[half:]]
    assert max(d) <= ring + 10.0
    assert abs(statistics.median(d) - ring) <= 2.0
