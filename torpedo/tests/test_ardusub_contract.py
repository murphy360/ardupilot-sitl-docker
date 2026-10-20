"""The contract with the captured ArduSub: its parameters, its words, its states.

``torpedo/tools/capture_ardusub.py`` captured these from the ArduSub SITL image. The
front seat answers with them, so a client that reads a default, a failsafe's name or an
EKF's flags is told what ArduSub would tell it.
"""

import json
import re

import pytest

from torpedo_sitl import captured, params
from torpedo_sitl import vehicle as veh

from .support import M, PLUGIN_NAMES, make_vehicle, run


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
