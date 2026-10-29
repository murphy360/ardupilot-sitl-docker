"""Routes of any length, across 180 degrees (murphy360/ardupilot-sitl-docker#12).

The flat frame moves to the hull every ``REBASE_M``; longitudes wrap into (-180, 180].
"""

import copy
import math

from pymavlink import mavutil

from torpedo_sitl import vehicle as veh
from torpedo_sitl.geo import Frame, haversine_m, wrap_lon
from torpedo_sitl.mission import Item

from .support import home_item, make_vehicle, run

M = mavutil.mavlink
CM = 0.01  # m


def _close_ll(a, b, frame, tol_m=CM):
    """Two (lat, lon) within ``tol_m`` of each other."""
    dn = math.radians(a[0] - b[0]) * frame.m_per_rad_lat
    de = math.radians(wrap_lon(a[1] - b[1])) * frame.m_per_rad_lon
    return math.hypot(dn, de) < tol_m


def _parked(model, **params):
    """A vehicle past its boot, at rest at 6 m with nothing to push it."""
    v, _ = make_vehicle(model, FS_PILOT_INPUT=0, FS_GCS_ENABLE=0, **params)
    run(v, 6.0)
    v.hull.depth = v.world.depth = 6.0
    v.hull.u = v.hull.v = v.hull.w = 0.0
    return v


def _put(v, north, east):
    """Put the hull (and the estimate) at (north, east) of the frame."""
    v.hull.north, v.hull.east = north, east
    v.world.north, v.world.east = north, east


def test_wrap_lon():
    assert wrap_lon(181.0) == -179.0
    assert wrap_lon(-181.0) == 179.0
    assert wrap_lon(180.0) == 180.0
    assert wrap_lon(-180.0) == 180.0
    assert wrap_lon(540.0) == 180.0
    assert wrap_lon(-71.1) == -71.1  # in range: returned to the bit


def test_to_ne_takes_the_short_way_across_180():
    f = Frame(29.5, 179.99)
    n, e = f.to_ne(29.5, -179.99)
    assert abs(n) < 1e-6
    assert 1930.0 < e < 1945.0  # 0.02 degrees east, not 359.98 west
    lat, lon = f.to_ll(0.0, 2000.0)
    assert abs(lat - 29.5) < 1e-12
    assert abs(lon - -179.9894) < 1e-3
    assert -180.0 < lon <= 180.0
    # and back the other way
    assert abs(Frame(29.5, -179.99).to_ne(29.5, 179.99)[1] + e) < 1e-6


def test_a_rebase_moves_nothing(hull_model):
    """10,001 m east of the origin, with a route, a GUIDED target, a hold centre and a
    loiter set: one step moves the origin to the hull, and nothing kept moves in
    latitude and longitude (to 1 cm); the course the guidance asks stays (0.1 deg)."""
    v = _parked(hull_model)
    _put(v, 0.0, 10_001.0)
    g = v.pilot.guidance
    g.route([(0.0, 10_001.0), (300.0, 10_400.0), (300.0, 11_000.0), (0.0, 11_500.0)])
    g.arc = ((150.0, 10_300.0), 11.5, 1.0, 7.0)
    v.target = veh.Target("point", 6.0, 250.0, 10_200.0)
    v._hold_centre = (-40.0, 10_050.0)
    v.auto.last_point = (10.0, 9_990.0)
    v.auto.loiter = {"centre": (500.0, 10_500.0), "turns": 1.0, "angle": 0.0}
    old = v.frame
    kept = {
        "truth": v.truth,
        "estimate": v.position,
        "target": (v.target.north, v.target.east),
        "hold": v._hold_centre,
        "last": v.auto.last_point,
        "loiter": v.auto.loiter["centre"],
        "centre": g.centre,
        "arc": g.arc[0],
        **{f"point{i}": p for i, p in enumerate(g.points)},
    }
    before = {k: old.to_ll(*p) for k, p in kept.items()}
    course = copy.deepcopy(g).course(v.position)

    v.step(0.05)

    new = v.frame
    assert new is not old
    assert _close_ll((new.lat, new.lon), before["truth"], new)
    assert v.truth == (0.0, 0.0)
    after = {
        "truth": v.truth,
        "estimate": v.position,
        "target": (v.target.north, v.target.east),
        "hold": v._hold_centre,
        "last": v.auto.last_point,
        "loiter": v.auto.loiter["centre"],
        "centre": g.centre,
        "arc": g.arc[0],
        **{f"point{i}": p for i, p in enumerate(g.points)},
    }
    for k, p in after.items():
        assert _close_ll(new.to_ll(*p), before[k], new), k
    assert v.world.frame is new
    again = copy.deepcopy(g).course(v.position)
    assert abs(math.degrees(math.remainder(again - course, 2 * math.pi))) < 0.1


def test_no_rebase_inside_ten_km(hull_model):
    v = _parked(hull_model)
    _put(v, 0.0, 9_999.0)
    old = v.frame
    v.step(0.05)
    assert v.frame is old
    assert abs(v.hull.east - 9_999.0) < 0.01


def test_home_stays_home(hull_model):
    """After three rebases the fence measures the true distance home, and home's
    mission item and HOME_POSITION are where it booted."""
    v = _parked(hull_model)
    home_ll = v.home_ll
    item0 = v.mission.item(0)
    frames = [v.frame]
    for _ in range(3):
        _put(v, 0.0, 10_001.0)
        v.step(0.05)
        frames.append(v.frame)
    assert len({id(f) for f in frames}) == 4  # three rebases
    assert v.home_ll == home_ll
    assert v.mission.item(0) == item0
    # The flat frame is on the WGS84 ellipsoid: east of here its radius is the prime
    # vertical, so the sphere the great circle is measured on has that radius (on the
    # mean radius the two differ by 0.2 per cent, the ellipsoid's own flattening).
    s = math.sin(math.radians(home_ll[0]))
    prime_vertical = 6378137.0 / math.sqrt(1.0 - 0.00669437999014 * s * s)
    truth = haversine_m(home_ll, v.latlon, prime_vertical)
    assert 30_000.0 < truth < 30_010.0
    assert abs(v.home_distance() - truth) / truth < 0.001
    v.params.set("FENCE_ENABLE", 1.0)
    v.params.set("FENCE_TYPE", 2.0)
    v.params.set("FENCE_RADIUS", 29_000.0)
    v.step(0.05)
    assert v.fence[:2] == (True, 3)  # 30 km out: beyond a 29 km fence
    v.params.set("FENCE_RADIUS", 31_000.0)
    v.step(0.05)
    assert v.fence[0] is False


def _item(seq, lat, lon, depth=6.0):
    return Item(
        seq, 16, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, round(lat * 1e7),
        round(lon * 1e7), -depth, 0.0, 0.0, 0.0, 0.0,
    )  # fmt: skip


def test_a_long_route_lands_where_it_should(hull_model):
    """AUTO along 25 km of the 29.4 N parallel, east across 180 degrees: the frame
    moves at least twice, the hull ends within 50 m of the last waypoint (great
    circle), and every longitude reported is in (-180, 180]."""
    start, end = (29.4, 179.88), (29.4, -179.86)
    v = _parked(hull_model, SIM_OPOS_LAT=start[0], SIM_OPOS_LNG=start[1])
    v.hull.u = v.model.max_speed_mps
    v.change_speed(v.model.max_speed_mps)
    route = [(29.4, 179.95), (29.4, -179.99), end]
    v.new_mission([home_item(v)] + [_item(i + 1, *p) for i, p in enumerate(route)])
    assert v.arm(True)[0] and v.set_mode(veh.AUTO)[0]
    frames, worst = {(v.frame.lat, v.frame.lon)}, 0.0
    lons = []
    for i in range(int(16_000 / 0.05)):
        v.step(0.05)
        frames.add((v.frame.lat, v.frame.lon))  # its origin: ids are reused
        if i % 200 == 0:
            lons += [v.latlon[1], v.truth_latlon[1], v.frame.lon]
            worst = max(worst, abs(v.hull.depth - 6.0))
        if v.mission.complete:
            break
    assert v.mission.complete, "the route was not finished"
    assert v.reached[:3] == [1, 2, 3]
    assert len(frames) >= 3  # at least two rebases
    assert haversine_m(v.truth_latlon, end) < 50.0
    assert all(-180.0 < lon <= 180.0 for lon in lons)
    assert any(lon < 0 for lon in lons) and any(lon > 0 for lon in lons)
    assert worst < 2.0
