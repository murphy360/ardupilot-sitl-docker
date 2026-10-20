"""The owner's acceptance lines (#2170), flown on both test hulls in-process, and the
shapes guidance flies."""

import math

import pytest

from torpedo_sitl import check, guidance, trials
from torpedo_sitl.autopilot import Autopilot, fly
from torpedo_sitl.physics import Hull


def test_lane_end_overshoot(measured):
    """A 90 degree turn at 1.54 and 2.30 m/s, and two lanes 40 m apart at 1.5 m/s: the
    turn reaches no more than 20 m past the lane's end."""
    for key in ("run_out_90_slow", "run_out_90_fast", "run_out_uturn"):
        assert measured[key] <= 20.0, key


def test_no_s_track(measured):
    """On the settled part of each lane the cross-track error stays within 2 m and
    changes sign at most once."""
    assert measured["xte_max"] <= 2.0
    assert measured["sign_changes"] <= 1


def test_settles(measured):
    """From the corner, the cross-track error is within 2 m in 30 s or less."""
    assert measured["settle_s"] <= 30.0


def test_speed_and_heading(measured):
    """On each straight: speed within 0.1 m/s rms and heading within 2 degrees rms."""
    assert measured["speed_rms"] <= 0.1
    assert measured["heading_rms"] <= 2.0


def test_orbit_radius(measured):
    """A 30 m ring at 2 m/s: after joining, the median of |d - r| over one turn is
    2 m or less."""
    assert measured["orbit_median"] <= 2.0


def test_hold_ring(measured):
    """A hold circle entered at 1.9 m/s: after the first half turn the hull stays
    within the ring plus 10 m, and the median |d - r| is 2 m or less."""
    assert measured["hold_over"] <= 10.0
    assert measured["hold_median"] <= 2.0


def test_the_check_report_and_exit_code(hull_name, capsys):
    from .conftest import MODELS

    assert check.main([str(MODELS / f"{hull_name}.json")]) == 0
    out = capsys.readouterr().out
    assert hull_name in out
    assert "FAIL" not in out
    assert out.count("ok ") == len(check.LINES)


def test_the_check_fails_a_hull_that_breaches_a_line(hull_model, monkeypatch, capsys):
    from dataclasses import replace

    monkeypatch.setattr(check, "SETTLE_S", 0.0)
    monkeypatch.setattr(
        check, "LINES", tuple(replace(x, limit=0.0) for x in check.LINES)
    )
    rows, ok = check.report(hull_model)
    assert not ok
    assert any(r.startswith("FAIL") for r in rows)


def test_the_check_refuses_a_bad_file(tmp_path, capsys):
    bad = tmp_path / "bad.json"
    bad.write_text("{}")
    assert check.main([str(bad)]) == 2
    assert "schema" in capsys.readouterr().err
    assert check.main([str(tmp_path / "missing.json")]) == 2


# -- the shapes -----------------------------------------------------------------------


def test_a_line_is_flown_by_line_of_sight(hull_model):
    g = guidance.Guidance(hull_model)
    g.route([(0.0, 0.0), (500.0, 0.0)])
    assert g.course((0.0, 0.0)) == pytest.approx(0.0)
    right = g.course((100.0, 10.0))  # to the right of the line (east): turn left
    left = g.course((100.0, -10.0))
    assert right < 0.0 < left
    assert right == pytest.approx(-math.atan(10.0 / (2.0 * hull_model.turn_radius_m)))


def test_the_corner_starts_the_next_line_early(hull_model):
    g = guidance.Guidance(hull_model)
    g.route([(0.0, 0.0), (300.0, 0.0), (300.0, 300.0)])
    ahead = guidance.TURN_K * hull_model.turn_radius_m * math.tan(math.pi / 4)
    g.course((300.0 - ahead - 1.0, 0.0))
    assert g.index == 0 and not g.pop_reached()
    g.course((300.0 - ahead + 1.0, 0.0))
    assert g.index == 1
    assert g.pop_reached() == [1]
    assert g.pop_reached() == []


def test_the_route_ends_in_a_circle(hull_model):
    g = guidance.Guidance(hull_model)
    g.route([(0.0, 0.0), (100.0, 0.0)])
    g.course((100.0 - hull_model.turn_radius_m + 1, 0.0))
    assert g.done and g.mode == "circle"
    assert g.pop_reached() == [1]
    assert g.radius == pytest.approx(guidance.LOITER_MIN * hull_model.turn_radius_m)


def test_a_point_is_reached_within_the_waypoint_radius(hull_model):
    g = guidance.Guidance(hull_model)
    g.goto((300.0, 0.0))
    assert g.course((0.0, 0.0)) == pytest.approx(0.0)
    assert not g.done
    r = hull_model.turn_radius_m
    assert g.course((300.0 - r - 1.0, 0.0)) == pytest.approx(0.0)
    g.course((300.0 - r + 1.0, 0.0))
    assert g.done and g.mode == "circle"
    custom = guidance.Guidance(hull_model, wp_radius=3.0)
    custom.goto((300.0, 0.0))
    custom.course((300.0 - 5.0, 0.0))
    assert not custom.done


def test_a_small_circle_is_raised_to_the_least_radius(hull_model):
    g = guidance.Guidance(hull_model)
    least = guidance.LOITER_MIN * hull_model.turn_radius_m
    g.circle((0.0, 0.0), 1.0)
    assert g.radius == pytest.approx(least)
    g.circle((0.0, 0.0), 10 * least)
    assert g.radius == pytest.approx(10 * least)


def test_a_circle_joins_on_a_tangent_from_outside(hull_model):
    g = guidance.Guidance(hull_model)
    g.circle((0.0, 0.0), 30.0, clockwise=True)
    course = g.course((-200.0, 0.0))  # south of the centre, flying north
    # the line leaves the circle at a tangent point with the centre on the hull's right
    # (a clockwise circle): the course is a little west of north, and the centre
    # bears to the right of it
    to_centre = guidance.bearing((-200.0, 0.0), (0.0, 0.0))
    assert course < 0.0
    assert to_centre - course == pytest.approx(math.asin(30.0 / 200.0))
    assert g.curvature == 0.0  # still on the approach line
    g.course((-31.0, 0.0))
    assert g.curvature == pytest.approx(1 / 30.0)


def test_a_circle_runs_both_ways(hull_model):
    for clockwise in (True, False):
        trace = trials.circle(
            hull_model, (0.0, 0.0), 30.0, 1.5, (-100.0, 0.0), 0.0, 300.0, clockwise
        )
        end = trace.samples[-1]
        assert abs(math.hypot(end.north, end.east) - 30.0) < 3.0
        turned = sum(
            math.atan2(math.sin(b.heading - a.heading), math.cos(b.heading - a.heading))
            for a, b in zip(trace.samples[-400:], trace.samples[-399:])
        )
        assert (turned > 0) == clockwise


def test_guidance_with_nothing_to_fly_says_so(hull_model):
    with pytest.raises(RuntimeError):
        guidance.Guidance(hull_model).course((0.0, 0.0))
    with pytest.raises(ValueError):
        guidance.Guidance(hull_model).route([(0.0, 0.0)])


def test_the_hull_goes_to_a_point_and_circles_it(hull_model):
    hull = Hull(hull_model, 0.0, 0.0, trials.DEPTH, 0.0, 0.0, 1.5)
    pilot = Autopilot(hull_model)
    pilot.speed, pilot.depth = 1.5, trials.DEPTH
    pilot.guidance.goto((200.0, 100.0))
    for t, h in fly(hull, pilot, 400.0):
        pass
    assert pilot.guidance.done
    assert math.hypot(h.north - 200.0, h.east - 100.0) < 3 * pilot.guidance.radius
    assert abs(h.depth - trials.DEPTH) < 0.5


def test_the_depth_is_held_and_changed(hull_model):
    hull = Hull(hull_model, 0.0, 0.0, 10.0, 0.0, 0.0, 1.5)
    pilot = Autopilot(hull_model)
    pilot.speed, pilot.depth = 1.5, 20.0
    pilot.guidance.route([(0.0, 0.0), (5000.0, 0.0)])
    for t, h in fly(hull, pilot, 400.0):
        pass
    assert abs(h.depth - 20.0) < 0.3
    assert abs(h.pitch) < 0.1


def test_the_speed_is_held_under_thrust_limits(hull_model):
    hull = Hull(hull_model, 0.0, 0.0, 10.0, 0.0, 0.0, 0.5)
    pilot = Autopilot(hull_model)
    pilot.speed, pilot.depth = 1.8, 10.0
    pilot.guidance.route([(0.0, 0.0), (5000.0, 0.0)])
    for t, h in fly(hull, pilot, 300.0):
        pass
    assert h.u == pytest.approx(1.8, abs=0.05)
    pilot.speed = 5.0  # more than the hull can do: it gives its best
    for t, h in fly(hull, pilot, 300.0):
        pass
    assert h.u == pytest.approx(hull_model.max_speed_mps, rel=0.05)
