"""The hull's physics: top speed, turning, buoyancy, no reverse, the current."""

import math

import pytest

from torpedo_sitl import physics
from torpedo_sitl.control import drag_thrust


def _hull(m, **kw):
    return physics.Hull(m, depth=kw.pop("depth", 20.0), **kw)


def _run(hull, seconds, thrust, rudder=0.0, elevator=0.0, dt=0.05, current=(0, 0)):
    for _ in range(round(seconds / dt)):
        hull.step(dt, thrust, rudder, elevator, current)


def test_top_speed(hull_model):
    """Thrust equal to the drag at the top speed settles within 5 % of it."""
    top = hull_model.max_speed_mps
    hull = _hull(hull_model)
    _run(hull, 900.0, drag_thrust(hull_model, top))
    assert hull.u == pytest.approx(top, rel=0.05)


def test_turn_radius(hull_model):
    """Full rudder at cruise: the steady radius, taken from the track, is within 20 %
    of the model's ``turn_radius_m``."""
    cruise = hull_model.cruise_speed_mps
    hull = _hull(hull_model, speed=cruise)
    thrust = drag_thrust(hull_model, cruise)
    _run(hull, 150.0, thrust, rudder=hull_model.fin.max_rad)
    points = []
    for _ in range(3):  # three points a third of a lap or so apart
        _run(hull, 8.0, thrust, rudder=hull_model.fin.max_rad)
        points.append((hull.north, hull.east))
    (x1, y1), (x2, y2), (x3, y3) = points
    a, b, c = (
        math.dist(points[0], points[1]),
        math.dist(points[1], points[2]),
        math.dist(points[0], points[2]),
    )
    area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2
    radius = a * b * c / (4 * area)
    assert radius == pytest.approx(hull_model.turn_radius_m, rel=0.20)


def test_turns_follow_the_rudder_sign(hull_model):
    """A positive rudder turns the bow to starboard (the heading grows), a negative
    one to port."""
    for sign in (1.0, -1.0):
        hull = _hull(hull_model, speed=1.5)
        _run(hull, 10.0, drag_thrust(hull_model, 1.5), rudder=sign * 0.2)
        assert sign * physics.wrap(hull.heading) > 0.1


def test_floats_up(hull_model):
    """No thrust, level: the hull rises, slowly."""
    hull = _hull(hull_model, depth=20.0)
    _run(hull, 60.0, 0.0)
    assert hull.depth < 20.0
    assert hull.depth > 15.0  # slowly: a few cm/s at the most
    assert hull.velocity_ned()[2] > -0.1


def test_no_reverse(hull_model):
    """Negative thrust is no thrust: the hull never goes astern."""
    hull = _hull(hull_model)
    _run(hull, 120.0, -hull_model.max_thrust_n)
    assert hull.u >= -1e-6
    assert hull.north >= -1e-6


def test_thrust_is_clamped_at_the_most(hull_model):
    a, b = _hull(hull_model), _hull(hull_model)
    _run(a, 60.0, hull_model.max_thrust_n)
    _run(b, 60.0, 100 * hull_model.max_thrust_n)
    assert a.u == pytest.approx(b.u)


def test_the_current_moves_the_water(hull_model):
    """In a current, with the hull's thrust off, it is carried at the current's speed
    over the ground, and with thrust the ground speed is the hull's plus the current."""
    hull = _hull(hull_model, speed=0.0)
    _run(hull, 5.0, 0.0, current=(0.4, -0.3))
    n, e, _ = hull.velocity_ned()
    assert (n, e) == pytest.approx((0.4, -0.3), abs=0.05)
    assert hull.speed_through_water < 0.1
    flying = _hull(hull_model, speed=1.5, heading=0.0)
    n, e, _ = flying.velocity_ned()
    flying.current_ne = (0.5, 0.0)
    assert flying.velocity_ned()[0] == pytest.approx(n + 0.5)


def test_fins_move_at_most_their_rate(hull_model):
    hull = _hull(hull_model)
    hull.step(0.1, 0.0, 10.0, -10.0)
    rate = hull_model.fin.rate_rad_s
    assert hull.rudder == pytest.approx(rate * 0.1)
    assert hull.elevator == pytest.approx(-rate * 0.1)
    _run(hull, 5.0, 0.0, rudder=10.0, elevator=-10.0)
    assert hull.rudder == pytest.approx(hull_model.fin.max_rad)
    assert hull.elevator == pytest.approx(-hull_model.fin.max_rad)


def test_a_long_step_is_the_same_as_the_short_ones(hull_model):
    a, b = _hull(hull_model, speed=1.0), _hull(hull_model, speed=1.0)
    a.step(1.0, 5.0, 0.1, 0.05)
    for _ in range(50):
        b.step(0.02, 5.0, 0.1, 0.05)
    assert (a.north, a.east, a.heading) == pytest.approx((b.north, b.east, b.heading))


def test_the_hull_does_not_rise_out_of_the_water(hull_model):
    hull = _hull(hull_model, depth=0.5, speed=1.5, pitch=0.4)
    _run(hull, 30.0, drag_thrust(hull_model, 1.5))
    assert hull.depth >= 0.0


def test_wrap_and_clamp():
    assert physics.wrap(3 * math.pi) == pytest.approx(math.pi)
    assert physics.wrap(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert physics.clamp(5, 0, 1) == 1
    assert physics.clamp(-5, 0, 1) == 0
    assert physics.clamp(0.5, 0, 1) == 0.5
