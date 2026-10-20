"""The simulated world: the current, the GPS, the DVL, the rangefinder, the depth sensor."""

import math

import pytest

from torpedo_sitl import captured
from torpedo_sitl import vehicle as veh
from torpedo_sitl.messages import BUILDERS

from .support import HOME, M, make_vehicle, place, ready, run

DVL = {
    "VISO_TYPE": 1.0,
    "SERIAL5_PROTOCOL": 2.0,
    "SIM_VICON_TMASK": 2.0,
    "EK3_SRC1_VELXY": 6.0,
}


def _moving(model, depth=8.0, **params):
    """An armed vehicle circling at depth in GUIDED (no target), after the estimate."""
    v, store = make_vehicle(model, FS_PILOT_INPUT=0, **params)
    run(v, 6.0)
    v.hull.depth, v.hull.u = depth, 1.5
    v.world.depth = depth
    assert v.set_mode(veh.GUIDED)[0] and v.arm(True)[0]
    return v, store


def _error(v):
    return math.dist(v.position, v.truth)


def test_the_estimate_is_the_truth_with_no_error_injected(hull_model):
    v, _ = _moving(hull_model)
    run(v, 120)
    assert _error(v) < 1e-6
    assert v.world.mode == "gps"


def test_the_current_moves_the_hull(hull_model):
    # a current from the south (180 deg) flows north
    v, _ = make_vehicle(
        hull_model, SIM_WIND_SPD=1.0, SIM_WIND_DIR=180.0, SIM_WIND_T=1.0
    )
    run(v, 60)
    assert v.truth[0] == pytest.approx(60.0, abs=3.0)
    assert abs(v.truth[1]) < 3.0


def test_the_current_is_the_same_at_every_depth(hull_model):
    v, _ = _moving(hull_model, depth=30.0, SIM_WIND_SPD=0.5, SIM_WIND_DIR=90.0)
    assert v.world.current == pytest.approx((0.0, -0.5), abs=1e-9)


def test_a_gps_offset_is_added_to_the_fix_and_the_estimate(hull_model):
    v, _ = _moving(hull_model, SIM_GPS1_GLTCH_X=0.0001, SIM_GPS1_GLTCH_Y=-0.0002)
    run(v, 1)
    north = math.radians(0.0001) * v.frame.m_per_rad_lat
    east = -math.radians(0.0002) * v.frame.m_per_rad_lon
    off = (v.position[0] - v.truth[0], v.position[1] - v.truth[1])
    assert off == pytest.approx((north, east), abs=1e-6)
    fix_lat, fix_lon = v.frame.to_ll(*v.world.gps_position(v.truth))
    est_lat, est_lon = v.latlon
    assert (fix_lat, fix_lon) == pytest.approx((est_lat, est_lon), abs=1e-9)


def test_dvl_drift_at_the_glitch_rate(hull_model):
    v, store = _moving(hull_model, SIM_GPS1_ENABLE=0.0, SIM_VICON_VGLI_X=0.1, **DVL)
    assert v.world.dvl_locked
    start = _error(v)
    run(v, 100)
    assert v.world.mode == "dvl"
    assert _error(v) - start == pytest.approx(10.0, rel=0.1)  # 0.1 m/s for 100 s
    assert v.position[0] - v.truth[0] == pytest.approx(10.0, rel=0.1)  # to the north
    assert abs(v.position[1] - v.truth[1]) < 1.0
    assert store.get("SIM_VICON_VGLI_X") == pytest.approx(0.1)


def test_dvl_position_error_and_noise(hull_model):
    v, _ = _moving(
        hull_model,
        SIM_GPS1_ENABLE=0.0,
        SIM_VICON_GLIT_X=3.0,
        SIM_VICON_V_SD=0.05,
        **DVL,
    )
    run(v, 60)
    assert v.position[0] - v.truth[0] == pytest.approx(3.0, abs=3.0)
    v2, _ = _moving(hull_model, SIM_GPS1_ENABLE=0.0, SIM_VICON_GLIT_X=3.0, **DVL)
    run(v2, 10)
    assert v2.position[0] - v2.truth[0] == pytest.approx(3.0, abs=0.2)


def test_with_no_gps_and_no_lock_the_position_holds(hull_model):
    v, _ = _moving(hull_model, SIM_GPS1_ENABLE=0.0)
    run(v, 1)
    held = v.position
    run(v, 60)
    assert v.world.mode == "none"
    assert v.position == held
    assert _error(v) > 20.0  # while the hull went on


def test_a_dvl_without_a_lock_holds_too(hull_model):
    v, _ = _moving(hull_model, SIM_GPS1_ENABLE=0.0, **DVL)
    v.params.values["SIM_VICON_TMASK"] = 0.0
    run(v, 30)
    assert v.world.mode == "none"


def test_a_fix_snaps_the_estimate_back_to_the_truth(hull_model):
    v, store = _moving(hull_model, SIM_GPS1_ENABLE=0.0, SIM_VICON_VGLI_X=0.2, **DVL)
    run(v, 100)
    assert _error(v) > 15.0
    v.texts.clear()
    store.values["SIM_GPS1_ENABLE"] = 1.0
    run(v, 0.2)
    assert _error(v) < 0.5
    assert [t for _, t in v.texts] == [t for _, t in captured.gps_back_texts()]


def test_gps_lost_says_what_ardusub_says(hull_model):
    v, store = _moving(hull_model)
    v.texts.clear()
    store.values["SIM_GPS1_ENABLE"] = 0.0
    run(v, 0.2)
    assert [t for _, t in v.texts] == [t for _, t in captured.gps_lost_texts()]


def test_the_older_gps_switch(hull_model):
    v, store = _moving(hull_model)
    store.values["SIM_GPS_DISABLE"] = 1.0
    run(v, 0.2)
    assert v.world.mode == "none"


def test_the_vehicle_steers_by_its_estimate(hull_model):
    """A DVL's bias drags the estimate; the vehicle flies to the point by it, and the
    truth ends up off by the bias's drift."""
    v, _ = _moving(hull_model, SIM_GPS1_ENABLE=0.0, SIM_VICON_VGLI_Y=0.1, **DVL)
    lat, lon = v.frame.to_ll(150.0, 0.0)
    v.position_target(
        veh.FRAME_RELATIVE_ALT_INT, 0b0000_1101_1111_1000, lat, lon, -8.0, (0, 0)
    )
    run(v, 130)
    assert v.pilot.guidance.done  # it reached the point by its estimate
    assert math.dist(v.position, (150.0, 0.0)) < 3 * hull_model.turn_radius_m
    assert _error(v) > 8.0  # but the hull is not where the estimate says


def test_ekf_flags_are_the_captured_ones_of_each_state(hull_model):
    v, store = _moving(hull_model)
    assert v.world.ekf_flags(True) == captured.state("gps")["ekf_flags"]
    store.values["SIM_GPS1_ENABLE"] = 0.0
    run(v, 0.2)
    assert v.world.ekf_flags(True) == captured.state("none")["ekf_flags"]
    for k, val in DVL.items():
        store.values[k] = val
    run(v, 0.2)
    assert v.world.ekf_flags(True) == captured.state("dvl")["ekf_flags"]
    assert v.world.ekf_flags(False) == 1  # attitude alone before the estimate is valid


def test_sensor_words_follow_the_state(hull_model):
    v, store = _moving(hull_model)
    base = captured.state("gps")
    present, enabled, health = v.world.sensor_words()
    assert present == base["sensors_present"] and health == base["sensors_health"]
    store.values["SIM_GPS1_ENABLE"] = 0.0
    run(v, 0.2)
    assert v.world.sensor_words()[2] == captured.state("none")["sensors_health"]


def test_the_rangefinder_reads_the_flat_floor_less_the_offset_and_depth(hull_model):
    v, store = _moving(hull_model, depth=12.0, RNGFND1_TYPE=1.0, SIM_SONAR_POS_Z=10.0)
    assert v.world.range_m(v.hull.depth) == pytest.approx(50 - 10 - 12)


def test_rangefinder_messages_only_when_fitted_and_in_range(hull_model):
    class Sent:
        def __init__(self):
            self.out = []

        def __getattr__(self, name):
            return lambda *a, **k: self.out.append((name, a))

    v, store = _moving(hull_model, depth=30.0, RNGFND1_TYPE=1.0, RNGFND1_MAX=30.0)
    mav = Sent()
    BUILDERS["RANGEFINDER"](v, mav)
    BUILDERS["DISTANCE_SENSOR"](v, mav)
    assert [n for n, _ in mav.out] == ["rangefinder_send", "distance_sensor_send"]
    assert mav.out[0][1][0] == pytest.approx(20.0)
    assert (
        mav.out[1][1][3] == 2000 and mav.out[1][1][6] == M.MAV_SENSOR_ROTATION_PITCH_270
    )
    mav.out.clear()
    store.values["SIM_SONAR_POS_Z"] = -20.0  # 50 + 20 - 30 = 40 m: past the maximum
    BUILDERS["RANGEFINDER"](v, mav)
    BUILDERS["DISTANCE_SENSOR"](v, mav)
    assert mav.out == []
    store.values["SIM_SONAR_POS_Z"] = 0.0
    store.values["RNGFND1_TYPE"] = 0.0  # not fitted
    BUILDERS["RANGEFINDER"](v, mav)
    assert mav.out == []
    assert not v.world.sensor_words()[0] & captured.LASER_POSITION


def test_a_lost_depth_sensor(hull_model):
    v, store = _moving(hull_model, depth=8.0)
    v.texts.clear()
    store.values["SIM_BARO_DISABLE"] = 1.0
    run(v, 0.2)
    assert captured.depth_sensor_lost() in v.texts
    assert v.mode == veh.MANUAL and v.mode_reason == "bad_depth"
    frozen = v.depth
    v.hull.depth += 5.0  # the hull goes on; the reading does not
    run(v, 1)
    assert v.depth == frozen
    health = v.world.sensor_words()[2]
    assert not health & captured.ABSOLUTE_PRESSURE
    store.values["SIM_BARO_DISABLE"] = 0.0
    run(v, 0.2)
    assert v.depth == pytest.approx(v.hull.depth)
    assert v.world.sensor_words()[2] & captured.ABSOLUTE_PRESSURE


def test_a_lost_depth_sensor_over_the_link(front_seat):
    fs = front_seat()
    c = fs.client("SERIAL2")
    c.interval("SYS_STATUS", 20)
    ok = c.wait("SYS_STATUS").onboard_control_sensors_health
    c.m.mav.param_set_send(
        c.target, 1, b"SIM_BARO_DISABLE", 1.0, M.MAV_PARAM_TYPE_REAL32
    )
    gone = c.wait(
        "SYS_STATUS",
        cond=lambda s: not s.onboard_control_sensors_health
        & captured.ABSOLUTE_PRESSURE,
    )
    assert ok & captured.ABSOLUTE_PRESSURE and gone


def test_world_over_the_link_estimate_and_truth_differ(front_seat):
    fs = front_seat(speedup=30.0)
    c = fs.client("SERIAL2")
    ready(c)
    c.m.mav.param_set_send(
        c.target, 1, b"SIM_GPS1_GLTCH_X", 0.0002, M.MAV_PARAM_TYPE_REAL32
    )
    c.interval("SIM_STATE", 10)
    c.interval("GPS_RAW_INT", 10)
    time_end = 10
    n, _, _ = place(c, fs.frame)
    sim = c.fresh("SIM_STATE")
    gps = c.fresh("GPS_RAW_INT")
    est_lat = c.fresh("GLOBAL_POSITION_INT").lat / 1e7
    assert est_lat - sim.lat_int / 1e7 == pytest.approx(0.0002, abs=2e-6)
    assert gps.lat / 1e7 - sim.lat_int / 1e7 == pytest.approx(0.0002, abs=2e-6)
    assert time_end and n == pytest.approx(0.0002 * 111000, abs=3.0) and HOME
