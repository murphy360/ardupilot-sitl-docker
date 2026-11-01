"""The terrain frame: the bed the rangefinder senses, its failsafe and its checks.

The seabed is emulated as the SITL's is: the rangefinder reads ``50 - SIM_SONAR_POS_Z
- depth``, so a bed at depth ``b`` under the hull is ``SIM_SONAR_POS_Z = 50 - b``. The
tests move it under the hull as it goes.
"""

import pytest

from torpedo_sitl import terrain
from torpedo_sitl import vehicle as veh
from torpedo_sitl.mission import Item

from .support import M, home_item, make_vehicle, run

TERRAIN = M.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
POSITION_ONLY = 0b0000_1101_1111_1000
WITH_VELOCITY = 0b0000_1111_1100_0000
BED_M, HEIGHT_M, RISE_M, RISE_AT_M = 10.0, 4.0, 3.0, 120.0
# How far from its height a hull may be on a rise: 1 m (as ArduSub's 0.7 m measured),
# but the large test hull on 20 %: its elevators are at their stop for the first 3 s
# of the rise and it dips 1.03 m. It pitches no faster at any speed it can fly.
WITHIN_M = {("torpedo-test-large", 0.20): 1.1}
CUT = -1000.0  # SIM_SONAR_POS_Z that puts the reading past RNGFND1_MAX


def _item(v, seq, north, east=0.0, height=HEIGHT_M, command=16):
    lat, lon = v.frame.to_ll(north, east)
    return Item(seq, command, TERRAIN, int(lat * 1e7), int(lon * 1e7), height)


def _bed_under(v, store, bed):
    """Put the bed at ``bed(north)`` m under the hull."""
    store.values["SIM_SONAR_POS_Z"] = 50.0 - bed(v.truth[0])


def _flat(_north):
    return BED_M


def _terrain_auto(model, bed=_flat, points=((400.0, 0.0),), **params):
    """Armed in AUTO on a terrain-frame route, at its height over the bed."""
    v, store = make_vehicle(model, FS_PILOT_INPUT=0, **params)
    _bed_under(v, store, bed)
    run(v, 6.0)
    depth = bed(0.0) - HEIGHT_M
    v.hull.depth = v.world.depth = depth
    v.hull.u = model.cruise_speed_mps
    items = [_item(v, i + 1, n, e) for i, (n, e) in enumerate(points)]
    v.new_mission([home_item(v), *items])
    assert v.arm(True)[0]
    assert v.set_mode(veh.AUTO)[0]
    return v, store


def _fly(v, store, seconds, bed=_flat, watch=None):
    for _ in range(round(seconds / 0.05)):
        _bed_under(v, store, bed)
        v.step(0.05)
        if watch is not None:
            watch(v)


def _rise(grade):
    """A bed at ``BED_M`` that rises ``RISE_M`` at ``grade`` from ``RISE_AT_M`` north."""

    def bed(north):
        up = min(max(north - RISE_AT_M, 0.0) * grade, RISE_M)
        return BED_M - up

    return bed


# -- following --------------------------------------------------------------------


@pytest.mark.parametrize("grade", [0.03, 0.10, 0.20], ids=["gentle", "10%", "20%"])
def test_a_route_follows_a_rising_bed_at_its_height(hull_model, grade):
    bed = _rise(grade)
    v, store = _terrain_auto(hull_model, bed)
    heights = []

    def watch(v):
        if v.truth[0] > 40.0:  # settled on the line
            heights.append(bed(v.truth[0]) - v.hull.depth)

    end = RISE_AT_M + RISE_M / grade + 80.0
    _fly(v, store, end / hull_model.cruise_speed_mps + 20.0, bed, watch)
    assert v.truth[0] > end  # over the rise and well past it
    assert min(heights) >= 2.0
    within = WITHIN_M.get((hull_model.id, grade), 1.0)
    assert all(abs(h - HEIGHT_M) <= within for h in heights), (
        min(heights),
        max(heights),
    )
    assert not v.failsafe.active


def test_the_bed_is_followed_along_each_line_not_only_at_its_points(hull_model):
    """Two lines, the rise in the middle of the first: the depth follows it there."""
    bed = _rise(0.10)
    v, store = _terrain_auto(hull_model, bed, points=((300.0, 0.0), (300.0, 300.0)))
    seen = []
    _fly(v, store, 160.0, bed, lambda v: seen.append((v.truth[0], v.hull.depth)))
    before = [d for n, d in seen if 60.0 < n < RISE_AT_M]
    after = [d for n, d in seen if RISE_AT_M + 60.0 < n < 280.0]
    assert before and after
    assert sum(before) / len(before) == pytest.approx(BED_M - HEIGHT_M, abs=0.5)
    assert sum(after) / len(after) == pytest.approx(BED_M - RISE_M - HEIGHT_M, abs=0.5)


def test_the_depth_asked_stays_within_the_surface_and_the_depth_limit(hull_model):
    v, store = _terrain_auto(hull_model)
    deep = hull_model.depth_limit_m + 20.0
    store.values["SIM_SONAR_POS_Z"] = 50.0 - deep
    store.values["RNGFND1_MAX"] = 200.0
    run(v, 6.0)  # the filtered range settles on the new reading
    assert v.terrain.depth_for(HEIGHT_M, 0.0) == hull_model.depth_limit_m
    assert v.terrain.depth_for(deep + 10.0, 0.0) == 0.0


def test_rangefinder_health_is_ardusubs(hull_model):
    v, store = make_vehicle(hull_model, SIM_SONAR_POS_Z=40.0)
    run(v, 1.0)
    rf = v.rangefinder
    assert rf.fitted and rf.status == "good" and rf.healthy
    assert rf.bed() == pytest.approx(BED_M, abs=0.05)
    store.values["SIM_SONAR_POS_Z"] = 49.9  # 0.1 m: under RNGFND1_MIN
    run(v, 0.2)
    assert rf.status == "low" and not rf.healthy and rf.bed() is None
    store.values["SIM_SONAR_POS_Z"] = CUT
    run(v, 0.2)
    assert rf.status == "high" and not rf.healthy
    store.values["SIM_SONAR_POS_Z"] = 40.0
    v.step(0.05)
    assert rf.status == "good" and not rf.healthy  # three good readings in a row
    run(v, 0.15)
    assert rf.healthy
    store.values["WP_RFND_USE"] = 0.0
    assert rf.healthy and rf.bed() is None  # not used for terrain
    store.values["RNGFND1_TYPE"] = 0.0
    run(v, 0.1)
    assert not rf.fitted and rf.status == "none"


# -- the terrain failsafe ---------------------------------------------------------


def _texts(v, start):
    return v.texts[start:]


def _cut(model, action, seconds=16.0):
    """In AUTO over the bed, the reading goes past its maximum; returns
    (vehicle, store, texts said since, depths seen since, depth at the cut)."""
    v, store = _terrain_auto(model, FS_TERRAIN_ENAB=float(action))
    _fly(v, store, 20.0)
    at, mark, depths = v.hull.depth, len(v.texts), []
    _fly(
        v, store, seconds, lambda _n: 50.0 - CUT, lambda v: depths.append(v.hull.depth)
    )
    return v, store, _texts(v, mark), depths, at


def test_a_lost_rangefinder_recovers_then_disarms_by_default(hull_model):
    v, _, said, depths, at = _cut(hull_model, 0)
    assert said == [
        terrain.TRIGGERED,
        terrain.RECOVERING,
        terrain.TIMED_OUT,
        (6, "Disarming motors"),
    ]
    assert not v.armed and v.mode == veh.AUTO
    assert v.critical  # as ArduSub's heartbeat: the failsafe stays on in AUTO
    assert max(depths) <= at + 0.3  # it never went deeper


def test_a_lost_rangefinder_recovers_then_holds(hull_model):
    v, _, said, depths, at = _cut(hull_model, 1)
    assert said == [terrain.TRIGGERED, terrain.RECOVERING, terrain.TIMED_OUT]
    assert v.armed and v.mode == veh.ALT_HOLD and v.mode_reason == terrain.REASON
    assert not v.critical  # out of AUTO the failsafe clears
    assert max(depths) <= at + 0.3


def test_a_lost_rangefinder_recovers_then_surfaces(hull_model):
    v, store, said, depths, at = _cut(hull_model, 2)
    assert said == [terrain.TRIGGERED, terrain.RECOVERING, terrain.TIMED_OUT]
    assert v.mode in (veh.SURFACE, veh.ALT_HOLD) and v.armed
    _fly(v, store, 120.0, lambda _n: 50.0 - CUT)  # SURFACE rises at steering speed
    assert v.mode == veh.ALT_HOLD and v.hull.depth < 1.0  # held at the top
    assert max(depths) <= at + 0.3


def test_the_failsafe_waits_its_timeout_and_recovery_its_own(hull_model):
    v, store = _terrain_auto(hull_model, FS_TERRAIN_ENAB=2.0)
    _fly(v, store, 20.0)
    cut, said = v.t, {}

    def watch(v):
        for _, text in v.texts:
            said.setdefault(text, v.t - cut)

    v.texts.clear()
    _fly(v, store, 14.0, lambda _n: 50.0 - CUT, watch)
    trig = said[terrain.TRIGGERED[1]]
    assert terrain.TIMEOUT_S < trig <= terrain.TIMEOUT_S + 0.6
    assert said[terrain.RECOVERING[1]] == trig
    late = said[terrain.TIMED_OUT[1]] - trig
    assert terrain.RECOVER_TIMEOUT_S < late <= terrain.RECOVER_TIMEOUT_S + 0.2


def test_recovery_succeeds_after_healthy_range_and_the_mission_resumes(hull_model):
    v, store = _terrain_auto(hull_model, FS_TERRAIN_ENAB=2.0)
    _fly(v, store, 20.0)
    mark = len(v.texts)
    _fly(v, store, 2.0, lambda _n: 50.0 - CUT)
    assert v.terrain.recovering
    current = v.mission.current
    _fly(v, store, 2.5)
    assert _texts(v, mark) == [
        terrain.TRIGGERED,
        terrain.RECOVERING,
        terrain.RECOVERED,
    ]
    assert v.mode == veh.AUTO and not v.terrain.recovering and not v.critical
    assert v.mission.current == current
    _fly(v, store, 40.0)
    assert v.hull.depth == pytest.approx(BED_M - HEIGHT_M, abs=0.7)  # following again


def test_recovery_climbs_when_the_bed_is_too_close(hull_model):
    v, store = _terrain_auto(hull_model, FS_TERRAIN_ENAB=1.0)
    _fly(v, store, 20.0)
    at = v.hull.depth

    def too_close(_north):  # the bed rises to within RNGFND1_MIN under the hull
        return v.hull.depth + 0.1

    _fly(v, store, 9.0, too_close)  # within the recovery time
    assert v.terrain.recovering and v.hull.depth < at - 0.3


def test_no_rangefinder_acts_at_once(hull_model):
    v, store = make_vehicle(hull_model, RNGFND1_TYPE=0.0, FS_TERRAIN_ENAB=2.0)
    run(v, 6.0)
    v.hull.depth = v.world.depth = 6.0
    v.new_mission([home_item(v), _item(v, 1, 300.0)])
    v.arm(True)
    mark = len(v.texts)
    assert v.set_mode(veh.AUTO)[0]
    run(v, 0.1)
    assert _texts(v, mark) == [terrain.NO_DATA]
    assert v.mode == veh.SURFACE and v.armed


def test_a_level_leg_needs_no_rangefinder(hull_model):
    v, store = make_vehicle(hull_model, SIM_SONAR_POS_Z=CUT)
    run(v, 6.0)
    v.hull.depth = v.world.depth = 6.0
    lat, lon = v.frame.to_ll(300.0, 0.0)
    level = Item(1, 16, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, int(lat * 1e7),
                 int(lon * 1e7), -6.0)  # fmt: skip
    v.new_mission([home_item(v), level])
    v.arm(True)
    v.set_mode(veh.AUTO)
    run(v, 20.0)
    assert v.mode == veh.AUTO and not v.critical and not v.terrain.recovering


# -- arming -----------------------------------------------------------------------


def _with_terrain_mission(model, **params):
    v, store = make_vehicle(model, **params)
    run(v, 6.0)
    v.new_mission([home_item(v), _item(v, 1, 300.0)])
    return v, store


def test_arming_is_refused_with_a_terrain_mission_and_the_checks_on(hull_model):
    v, _ = _with_terrain_mission(hull_model, ARMING_SKIPCHK=0.0)
    v.texts.clear()
    assert v.arm(True) == (False, "")
    assert not v.armed
    assert v.texts == [(2, "Arm: terrain disabled")]
    v.texts.clear()
    run(v, 31.0)
    assert (2, "PreArm: terrain disabled") in v.texts  # the periodic check
    assert v.arm(True, force=True)[0]  # a forced arm skips the checks


def test_arming_with_ardusubs_own_check_mask_is_not_refused(hull_model):
    v, store = _with_terrain_mission(hull_model)
    assert int(store.get("ARMING_SKIPCHK")) & terrain.PARAMETERS_CHECK
    assert v.arm(True)[0]


def test_arming_without_terrain_items_is_not_refused(hull_model):
    v, store = make_vehicle(hull_model, ARMING_SKIPCHK=0.0)
    run(v, 6.0)
    assert v.arm(True)[0]


def test_with_terrain_enabled_it_waits_for_terrain_data(hull_model):
    v, _ = _with_terrain_mission(hull_model, ARMING_SKIPCHK=0.0, TERRAIN_ENABLE=1.0)
    v.texts.clear()
    assert not v.arm(True)[0]
    assert v.texts == [(2, "Arm: waiting for terrain data")]


# -- GUIDED -------------------------------------------------------------------------


def _guided(model, **params):
    v, store = make_vehicle(model, SIM_SONAR_POS_Z=50.0 - BED_M, **params)
    run(v, 6.0)
    v.hull.depth = v.world.depth = 2.0
    v.set_mode(veh.GUIDED)
    v.arm(True)
    return v, store


def test_a_terrain_target_with_a_velocity_is_dropped(hull_model):
    v, _ = _guided(hull_model)
    lat, lon = v.frame.to_ll(300.0, 0.0)
    ok, _ = v.position_target(TERRAIN, WITH_VELOCITY, lat, lon, HEIGHT_M, (1.0, 0.0))
    assert not ok and v.target is None


def test_a_terrain_target_with_a_position_only_is_flown_over_the_bed(hull_model):
    v, store = _guided(hull_model)
    lat, lon = v.frame.to_ll(300.0, 0.0)
    ok, _ = v.position_target(TERRAIN, POSITION_ONLY, lat, lon, HEIGHT_M, (0.0, 0.0))
    assert ok and v.target.height == HEIGHT_M
    bed = _rise(0.10)
    _fly(v, store, 150.0, bed)
    assert v.truth[0] > RISE_AT_M + RISE_M / 0.10 + 20.0
    assert bed(v.truth[0]) - v.hull.depth == pytest.approx(HEIGHT_M, abs=1.0)


def test_a_terrain_target_with_nothing_sensed_is_dropped(hull_model):
    v, _ = _guided(hull_model, RNGFND1_TYPE=0.0)
    lat, lon = v.frame.to_ll(300.0, 0.0)
    ok, _ = v.position_target(TERRAIN, POSITION_ONLY, lat, lon, HEIGHT_M, (0.0, 0.0))
    assert not ok and v.target is None


def test_a_lost_rangefinder_in_guided_says_so_and_flies_on(hull_model):
    """ArduSub starts AUTO's recovery from GUIDED, where nothing runs it."""
    v, store = _guided(hull_model)
    lat, lon = v.frame.to_ll(600.0, 0.0)
    v.position_target(TERRAIN, POSITION_ONLY, lat, lon, HEIGHT_M, (0.0, 0.0))
    _fly(v, store, 40.0)
    depth, mark = v.target.depth, len(v.texts)
    _fly(v, store, 15.0, lambda _n: 50.0 - CUT)
    assert v.texts[mark:] == [terrain.TRIGGERED, terrain.RECOVERING]
    assert v.mode == veh.GUIDED and v.armed and v.critical
    assert v.target.depth == pytest.approx(depth, abs=0.05)  # the depth last asked
