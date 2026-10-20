"""The parts of the front seat that need no socket: the battery, the parameter store,
the frame, the command line."""

import json
import math

import pytest

from torpedo_sitl import cli, geo
from torpedo_sitl.battery import FULL_V, Battery
from torpedo_sitl.params import ParamStore, defaults, model_number, parse_lines

from .support import MODELS

PLUGIN_NAMES = """
SYSID_THISMAV MAV_SYSID ARMING_CHECK ARMING_SKIPCHK WP_RADIUS_M WPNAV_RADIUS
WPNAV_RADIUS_M WP_RADIUS FS_GCS_ENABLE FS_GCS_TIMEOUT SYSID_MYGCS MAV_GCS_SYSID
MAV_GCS_SYSID_HI SYSID_ENFORCE MAV_OPTIONS PDEAD_ACTION PDEAD_HOLD_S SCR_ENABLE
FENCE_ENABLE FENCE_TYPE FENCE_ACTION FENCE_ALT_MIN FENCE_RADIUS FENCE_MARGIN
BATT_MONITOR BATT_CAPACITY BATT_LOW_MAH BATT_LOW_VOLT BATT_FS_LOW_ACT BATT_CRT_MAH
BATT_CRT_VOLT BATT_FS_CRT_ACT FS_LEAK_ENABLE LEAK1_PIN FS_PILOT_INPUT FS_PILOT_TIMEOUT
BARO_EXT_BUS RNGFND1_TYPE RNGFND1_MAX_CM RNGFND1_MIN_CM RNGFND1_MAX RNGFND1_MIN
EK3_SRC1_VELXY WPNAV_SPEED_UP PILOT_SPEED_UP EK3_POS_I_GATE EK3_HGT_I_GATE
EK3_GLITCH_RAD EK3_ALT_M_NSE SIM_OPOS_LAT SIM_OPOS_LNG SIM_OPOS_ALT SIM_OPOS_HDG
TPDO_MODEL TPDO_LOS_LA TPDO_TURN_K TPDO_LOIT_MIN
""".split()


def test_battery_lasts_the_endurance_at_cruise():
    b = Battery(10000.0, 10.0, 1.5)
    hours = 0
    while b.remaining > 0.5 and hours < 20 * 3600:
        b.step(60.0, 1.5)  # a minute at cruise: the hotel load and the propulsion
        hours += 60
    assert hours == pytest.approx(0.5 * 10 * 3600, rel=0.02)
    assert b.voltage == pytest.approx((FULL_V + 13.6) / 2, abs=0.05)


def test_battery_draw_grows_with_the_speed_cubed():
    b = Battery(10000.0, 10.0, 1.5)
    b.step(1.0, 0.0)
    idle = b.current_ma
    b.step(1.0, 1.5)
    cruise = b.current_ma
    b.step(1.0, 3.0)
    assert idle == pytest.approx(0.3 * cruise)
    assert b.current_ma - idle == pytest.approx(8 * (cruise - idle))
    assert b.remaining_s > 0


def test_the_model_number():
    assert model_number("torpedo-100") == 100
    assert model_number("torpedo-6000") == 6000
    assert model_number("torpedo-test-small") == 0


def test_parameter_lines():
    text = "A_ONE 1.5\n# c\nB_TWO,2;C_THREE=3\nbad line here\nTOO_LONG_A_NAME_FOR_IT 1\nX nan"
    assert parse_lines(text) == [("A_ONE", 1.5), ("B_TWO", 2.0), ("C_THREE", 3.0)]


def test_store_saves_only_what_is_set_and_clears_at_start(tmp_path, hull_model):
    path = tmp_path / "p.parm"
    init = defaults(hull_model, 1, 1.0, 2.0, 0.0, 90.0, 5000.0)
    store = ParamStore(init, path)
    store.load([("SYSID_THISMAV", 9.0), ("EXTRA", 1.0)])
    assert store.get("SYSID_THISMAV") == 9.0 and store.get("EXTRA") == 1.0
    assert not path.exists()  # start-up lines are not saved
    assert store.set("FS_GCS_ENABLE", 3.0) and not store.set("NOPE", 1.0)
    assert not store.set("TPDO_MODEL", 5.0)
    assert path.read_text().split() == ["FS_GCS_ENABLE", "3"]
    store.values["FS_GCS_ENABLE"] = 0.0
    store.reload()  # a reboot: the saved value comes back
    assert store.get("FS_GCS_ENABLE") == 3.0
    again = ParamStore(init, path)  # a new process: cleared
    assert not path.exists() and again.get("FS_GCS_ENABLE") == 0.0


def test_every_name_the_plugin_reads_is_there(hull_model):
    init = defaults(hull_model, 1, 1.0, 2.0, 0.0, 90.0, 5000.0)
    assert not [n for n in PLUGIN_NAMES if n not in init]
    assert all(len(n) <= 16 for n in init)
    assert init["WP_RADIUS_M"] == hull_model.turn_radius_m
    assert init["FENCE_ALT_MIN"] == -hull_model.depth_limit_m


def test_frame_round_trip():
    f = geo.Frame(36.6, -121.9)
    n, e = f.to_ne(*f.to_ll(1234.5, -678.9))
    assert (n, e) == pytest.approx((1234.5, -678.9), abs=1e-6)
    lat, lon = f.to_ll(111_000.0, 0.0)
    assert lat - 36.6 == pytest.approx(111000.0 / 110970.3, abs=1e-4)
    assert math.isclose(lon, -121.9)


def test_no_model_exits_2_and_says_why(capsys):
    assert cli.main([], env={}) == 2
    assert "TORPEDO_MODEL" in capsys.readouterr().err


def test_a_bad_model_exits_2(tmp_path, capsys):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema": 2}))
    assert cli.main([], env={"TORPEDO_MODEL": str(bad)}) == 2
    assert "schema" in capsys.readouterr().err
    assert cli.main(["--model", str(tmp_path / "none.json")], env={}) == 2


def test_the_environment_is_the_sitl_images(tmp_path):
    env = {
        "INSTANCE": "3",
        "LAT": "10.5",
        "LON": "-20.25",
        "ALT": "2",
        "DIR": "45",
        "SPEEDUP": "20",
        "SYSID": "6",
        "SITL_PARAMS": "A 1",
        "BATT_CAPACITY": "7000",
    }
    args = cli.argparse.Namespace(param_file=["x.parm"])
    cfg = cli.config_from_env(env, args)
    assert (cfg.instance, cfg.lat, cfg.lon, cfg.alt, cfg.heading) == (
        3,
        10.5,
        -20.25,
        2,
        45,
    )
    assert (cfg.speedup, cfg.sysid, cfg.batt_mah) == (20, 6, 7000)
    assert cfg.sitl_params == "A 1" and cfg.param_files == ["x.parm"]
    assert (
        cfg.ports["SERIAL0"] == 5790 and (MODELS / "torpedo-test-small.json").exists()
    )
