"""The contract with the captured ArduSub: its parameters, its words, its states.

``torpedo/tools/capture_ardusub.py`` captured these from the ArduSub SITL image. The
front seat answers with them, so a client that reads a default, a failsafe's name or an
EKF's flags is told what ArduSub would tell it. ``torpedo/tools/capture_terrain.py``
captured a terrain-frame mission flown on the rangefinder: its arming refusal, its
terrain failsafe for each action, a recovery and its GUIDED targets.
"""

import json
import re

import pytest

from torpedo_sitl import captured, params, terrain
from torpedo_sitl import vehicle as veh

from .support import M, PLUGIN_NAMES, make_vehicle, run
from .test_terrain import CUT, _cut, _fly, _terrain_auto, _with_terrain_mission


def test_the_capture_is_complete():
    d = captured.data()
    assert len(d["params"]) > 1000
    assert {"gps", "dvl", "none"} <= set(d["states"])
    for key in (
        "boot",
        "arm",
        "disarm",
        "gcs_failsafe",
        "battery",
        "depth_sensor_lost",
    ):
        assert d["statustext"][key]
    assert set(d["statustext"]["gcs_failsafe"]) == {"1", "3", "4"}


def test_the_three_ekf_states_differ_as_they_should():
    gps, dvl, none = (captured.state(n) for n in ("gps", "dvl", "none"))
    abs_pos = M.EKF_POS_HORIZ_ABS
    assert gps["ekf_flags"] & abs_pos
    assert not dvl["ekf_flags"] & abs_pos and dvl["ekf_flags"] & M.EKF_VELOCITY_HORIZ
    assert none["ekf_flags"] & M.EKF_CONST_POS_MODE and not none["ekf_flags"] & abs_pos
    assert gps["gps_fix_type"] >= 3 and none["gps_fix_type"] < 3
    assert captured.depth_lost_clear_mask() & captured.ABSOLUTE_PRESSURE
    assert captured.gps_lost_clear_mask()


def test_every_name_the_plugin_reads_is_answered(hull_model):
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    missing = [n for n in PLUGIN_NAMES if n not in init]
    assert not missing


def test_every_captured_parameter_is_answered_with_its_captured_default(hull_model):
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    # only what the hull and the environment decide differs
    different = {
        n
        for n, value in captured.params().items()
        if n not in init or init[n] != pytest.approx(value, rel=1e-6, abs=1e-9)
    }
    assert different <= params.MODEL_DERIVED, different - params.MODEL_DERIVED
    assert list(captured.params())[:50] == list(init)[:50]  # ArduSub's own order


def test_the_plugin_names_the_capture_has_keep_its_default(hull_model):
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    had = [
        n
        for n in PLUGIN_NAMES
        if n in captured.params() and n not in params.MODEL_DERIVED
    ]
    assert had
    for name in had:
        assert init[name] == pytest.approx(
            captured.params()[name], rel=1e-6, abs=1e-9
        ), name


def test_names_the_capture_lacks_keep_a_documented_default(hull_model):
    lacking = [n for n in PLUGIN_NAMES if n not in captured.params()]
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    assert lacking and all(n in init for n in lacking)


def test_the_simulators_parameters_are_there(hull_model):
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    for name in (
        "SIM_GPS1_ENABLE", "SIM_GPS_DISABLE", "SIM_GPS1_GLTCH_X", "SIM_GPS1_GLTCH_Y",
        "SIM_VICON_TMASK", "SIM_VICON_VGLI_X", "SIM_VICON_VGLI_Y", "SIM_VICON_GLIT_X",
        "SIM_VICON_GLIT_Y", "SIM_VICON_V_SD", "SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_T",
        "SIM_SONAR_POS_Z", "SIM_BARO_DISABLE", "SIM_OPOS_LAT",
    ):  # fmt: skip
        assert name in init, name


def test_arming_and_disarming_say_what_ardusub_says(hull_model):
    v, _ = make_vehicle(hull_model)
    run(v, 6)
    v.texts.clear()
    v.arm(True)
    v.arm(True)  # already armed: nothing more to say
    v.arm(False)
    assert [t for _, t in v.texts] == [
        captured.arm_text()[1],
        captured.disarm_text()[1],
    ]


def test_refused_modes_say_what_ardusub_says(hull_model):
    v, _ = make_vehicle(hull_model)
    assert v.set_mode(veh.GUIDED) == (False, "refused")  # before the position
    assert v.texts[-1] == captured.guided_refused()
    run(v, 6)
    assert v.set_mode(veh.AUTO) == (False, "refused")  # no mission
    assert v.texts[-1] == captured.auto_refused()
    assert v.set_mode(veh.GUIDED)[0]


def test_the_gcs_failsafe_texts_are_the_captured_ones_for_each_action(hull_model):
    for action in (1, 3, 4):
        v, _ = make_vehicle(hull_model, FS_GCS_ENABLE=float(action), FS_GCS_TIMEOUT=2.0)
        run(v, 6)
        v.hull.depth = 6.0
        v.world.depth = 6.0
        v.set_mode(veh.ALT_HOLD)
        v.arm(True)
        v.heartbeat(255)
        v.texts.clear()
        run(v, 4)
        assert captured.gcs_lost(action) in v.texts


def test_battery_lines_keep_the_captured_shape(hull_model):
    for level in ("low", "critical"):
        sev, template = captured.battery_template(level)
        line = template.format(n=1, volts=12.5, mah=10)
        assert re.fullmatch(rf"Battery 1 is {level} 12\.50V used 10 mAh", line)
        assert sev == 4


def test_the_capture_script_writes_this_file():
    from pathlib import Path

    here = Path(__file__).resolve().parents[1]
    script = (here / "tools" / "capture_ardusub.py").read_text()
    assert "ardusub_surface.json" in script
    assert json.loads(captured.PATH.read_text()) == captured.data()
    script = (here / "tools" / "capture_terrain.py").read_text()
    assert "ardusub_terrain.json" in script
    assert json.loads(captured.TERRAIN_PATH.read_text()) == captured.terrain()


# -- the terrain frame (``tools/capture_terrain.py``) ---------------------------------


def _pairs(lines):
    return [(int(s), t) for s, t in lines]


def test_the_terrain_parameters_keep_ardusubs_defaults(hull_model):
    init = params.defaults(hull_model, 1, 36.6, -121.9, 0.0, 90.0, 5000.0)
    for name in ("FS_TERRAIN_ENAB", "WP_RFND_USE", "RNGFND1_MIN", "RNGFND1_MAX"):
        assert init[name] == pytest.approx(captured.params()[name]), name
    assert init["FS_TERRAIN_ENAB"] == 0 and init["WP_RFND_USE"] == 1  # disarm; used
    assert init["ARMING_SKIPCHK"] == captured.terrain()["skipchk_default"]


def test_terrain_arming_says_what_ardusub_says(hull_model):
    cap = captured.terrain()
    assert cap["arm_refused"]["ack"] == M.MAV_RESULT_FAILED
    assert cap["arm_default"]["ack"] == M.MAV_RESULT_ACCEPTED
    skip = cap["skipchk_default"] & ~terrain.PARAMETERS_CHECK
    v, _ = _with_terrain_mission(hull_model, ARMING_SKIPCHK=float(skip))
    v.texts.clear()
    assert not v.arm(True)[0]
    assert v.texts == _pairs(cap["arm_refused"]["texts"])
    v.texts.clear()
    run(v, 31.0)
    assert set(_pairs(cap["prearm"])) <= set(v.texts)
    v.params.values["ARMING_SKIPCHK"] = float(cap["skipchk_default"])
    assert v.arm(True)[0]


@pytest.mark.parametrize("action", [0, 1, 2])
def test_the_terrain_failsafe_says_and_does_what_ardusub_does(hull_model, action):
    cap = captured.terrain()["failsafe"][str(action)]
    v, store, said, depths, at = _cut(hull_model, action)
    assert said == _pairs(cap["texts"])
    assert v.armed == cap["armed"]
    assert v.critical == (cap["system_status"] == M.MAV_STATE_CRITICAL)
    # ArduSub climbs out of range high as well (its descent speed, as a climb rate)
    assert cap["depth_max"] <= cap["depth_at_cut"] + 0.1
    assert max(depths) <= at + 0.3
    if action == terrain.HOLD:  # ArduSub's POSHOLD; the torpedo's hold is ALT_HOLD
        assert cap["custom_mode"] == 16 and v.mode == veh.ALT_HOLD
    elif action == terrain.SURFACE_ACT:  # surfaced, ArduSub holds at the top
        assert cap["custom_mode"] == veh.ALT_HOLD and v.mode == veh.SURFACE
        _fly(v, store, 120.0, lambda _n: 50.0 - CUT)
        assert v.mode == veh.ALT_HOLD
    else:
        assert cap["custom_mode"] == v.mode == veh.AUTO


def test_terrain_recovery_says_what_ardusub_says(hull_model):
    cap = captured.terrain()["recovered"]
    v, store = _terrain_auto(hull_model, FS_TERRAIN_ENAB=2.0)
    _fly(v, store, 20.0)
    mark = len(v.texts)
    _fly(v, store, 2.0, lambda _n: 50.0 - CUT)
    _fly(v, store, 2.5)
    # ArduSub then says the mission item it resumes ("Mission: 1 WP"); the front
    # seat says no "Mission:" lines for any item
    assert v.texts[mark:] == _pairs(cap["texts"])[:3]
    assert cap["custom_mode"] == v.mode == veh.AUTO


def test_terrain_guided_targets_are_taken_as_ardusub_takes_them(hull_model):
    cap = captured.terrain()["guided"]
    assert abs(cap["with_velocity"]["moved_north_m"]) < 1.0  # dropped
    assert cap["position"]["moved_north_m"] > 5.0  # flown
    v, _ = make_vehicle(hull_model, SIM_SONAR_POS_Z=40.0)
    run(v, 6.0)
    v.set_mode(veh.GUIDED)
    lat, lon = v.frame.to_ll(300.0, 0.0)
    frame = M.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
    assert not v.position_target(frame, 0b110111000000, lat, lon, 4.0, (1.0, 0.0))[0]
    assert v.position_target(frame, 0b110111111000, lat, lon, 4.0, (0.0, 0.0))[0]
