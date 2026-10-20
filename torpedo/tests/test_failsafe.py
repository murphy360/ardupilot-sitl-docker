"""The failsafes and the Proteus-dead script, on a vehicle stepped by hand."""

import time

from torpedo_sitl import captured
from torpedo_sitl import vehicle as veh
from torpedo_sitl.mission import DO_LAND_START, NAV_LOITER_UNLIM

from .support import home_item, make_vehicle, plain_item, run, waypoint

GCS = 255


def _vehicle(model, mode=veh.ALT_HOLD, depth=8.0, **params):
    """An armed vehicle at depth that has heard a GCS heartbeat."""
    params = {"FS_PILOT_INPUT": 0, "FS_GCS_TIMEOUT": 2.0, "FS_GCS_ENABLE": 0.0} | params
    v, _ = make_vehicle(model, **params)
    run(v, 6.0)
    v.hull.depth, v.hull.u = depth, 1.5
    v.world.depth = depth
    assert v.set_mode(mode)[0] and v.arm(True)[0]
    v.heartbeat(int(v.params.get("MAV_GCS_SYSID")))
    v.texts.clear()
    return v


def _silence(v, seconds):
    run(v, seconds)


def _said(v):
    return [t for _, t in v.texts]


def test_no_failsafe_before_a_gcs_has_been_heard(hull_model):
    v, _ = make_vehicle(hull_model, FS_GCS_ENABLE=3.0, FS_PILOT_INPUT=0)
    run(v, 6)
    v.set_mode(veh.ALT_HOLD)
    v.arm(True)
    run(v, 60)
    assert not v.critical and v.armed


def test_gcs_warn_only(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=1.0)
    _silence(v, 4)
    assert captured.gcs_lost(1)[1] in _said(v)
    assert v.critical and v.armed and v.mode == veh.ALT_HOLD
    v.heartbeat(GCS)
    run(v, 0.1)
    assert captured.gcs_cleared()[1] in _said(v)
    assert not v.critical


def test_gcs_failsafe_says_it_once(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=1.0)
    _silence(v, 20)
    assert _said(v).count(captured.gcs_lost(1)[1]) == 1


def test_gcs_disarm(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=2.0)
    _silence(v, 4)
    assert not v.armed and v.critical
    assert captured.disarm_text()[1] in _said(v)


def test_gcs_depth_hold_is_alt_hold_for_that_reason(hull_model):
    v = _vehicle(hull_model, mode=veh.GUIDED, FS_GCS_ENABLE=3.0)
    _silence(v, 4)
    assert v.mode == veh.ALT_HOLD and v.mode_reason == "gcs_failsafe"
    assert v.hull.u > 0.3  # a torpedo circles at depth, it does not stop


def test_gcs_surface(hull_model):
    v = _vehicle(hull_model, mode=veh.GUIDED, FS_GCS_ENABLE=4.0)
    _silence(v, 4)
    assert v.mode == veh.SURFACE and v.mode_reason == "gcs_failsafe"


def test_gcs_off_does_nothing(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=0.0)
    _silence(v, 30)
    assert not v.critical and v.armed


def test_gcs_failsafe_is_not_raised_when_disarmed(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=1.0)
    v.arm(False)
    _silence(v, 30)
    assert not v.critical


def test_only_the_gcs_system_counts(hull_model):
    v = _vehicle(hull_model, FS_GCS_ENABLE=1.0, MAV_GCS_SYSID=7.0)
    for _ in range(40):
        v.heartbeat(8)  # another system's heartbeat keeps nothing alive
        run(v, 0.1)
    assert v.critical
    v2 = _vehicle(
        hull_model, FS_GCS_ENABLE=1.0, MAV_GCS_SYSID=7.0, MAV_GCS_SYSID_HI=9.0
    )
    for _ in range(40):
        v2.heartbeat(8)  # inside the range
        run(v2, 0.1)
    assert not v2.critical


# -- the Proteus-dead script ---------------------------------------------------------


def test_pdead_loaded_is_said_at_boot(hull_model):
    v, _ = make_vehicle(hull_model)
    run(v, 1)
    assert "PDEAD: loaded" in _said(v)


def test_pdead_hold_depth_waits_then_surfaces(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=1.0,
        PDEAD_HOLD_S=20.0,
    )
    _silence(v, 8)
    assert v.mode == veh.ALT_HOLD
    assert "PDEAD: Proteus silent, holding at depth" in _said(v)
    _silence(v, 20)
    assert v.mode == veh.SURFACE
    assert any(
        t.startswith("PDEAD: silent") and t.endswith("surfacing") for t in _said(v)
    )


def test_pdead_hold_without_a_limit_holds_for_good(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=1.0,
        PDEAD_HOLD_S=0.0,
    )
    _silence(v, 300)
    assert v.mode == veh.ALT_HOLD


def test_pdead_proteus_back_ends_the_episode(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=1.0,
        PDEAD_HOLD_S=20.0,
    )
    _silence(v, 8)
    v.heartbeat(GCS)
    run(v, 1)
    assert any(t.startswith("PDEAD: Proteus back after") for t in _said(v))
    _silence(v, 3)  # a second silence is a new episode: it waits again
    run(v, 30)
    assert v.mode in (veh.ALT_HOLD, veh.SURFACE)


def test_pdead_yields_to_someone_else(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=1.0,
        PDEAD_HOLD_S=10.0,
    )
    _silence(v, 6)
    assert v.set_mode(veh.GUIDED)[0]  # an operator takes the vehicle
    _silence(v, 30)
    assert v.mode == veh.GUIDED


def test_pdead_off_leaves_it_to_the_failsafe(hull_model):
    v = _vehicle(hull_model, mode=veh.GUIDED, FS_GCS_ENABLE=3.0, PDEAD_ACTION=0.0)
    _silence(v, 300)
    assert v.mode == veh.ALT_HOLD
    assert not any(t.startswith("PDEAD: Proteus silent") for t in _said(v))


def _with_route(model, marker=True, **params):
    v = _vehicle(
        model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=2.0,
        PDEAD_HOLD_S=15.0,
        **params,
    )
    items = [
        home_item(v),
        waypoint(v, 1, 0, 400),
        waypoint(v, 2, 0, 400, command=NAV_LOITER_UNLIM),
    ]
    if marker:
        items += [
            plain_item(3, DO_LAND_START),
            waypoint(v, 4, 100, 100),
            waypoint(v, 5, 100, 0),
        ]
    v.new_mission(items)
    return v


def test_pdead_return_route_flies_from_the_marker(hull_model):
    v = _with_route(hull_model)
    _silence(v, 8)
    assert v.mode == veh.AUTO
    assert v.mission.current in (3, 4, 5)  # the marker, or past it
    assert any(t.startswith("PDEAD: flying the return route") for t in _said(v))
    # the loiter before the marker is never flown: the way home is
    assert 2 not in v.reached


def test_pdead_return_route_ends_in_the_surface(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=2.0,
        PDEAD_HOLD_S=0.0,
    )
    v.new_mission([home_item(v), waypoint(v, 1, 0, 60), waypoint(v, 2, 60, 60)])
    _silence(v, 6)
    assert v.mode == veh.AUTO
    run(v, 300)
    assert v.mode in (veh.SURFACE, veh.ALT_HOLD)
    assert "PDEAD: route flown, surfacing" in _said(v)


def test_pdead_falls_back_to_the_timed_hold_when_auto_is_refused(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=2.0,
        PDEAD_HOLD_S=15.0,
    )  # no mission
    _silence(v, 8)
    assert v.mode == veh.ALT_HOLD
    assert "PDEAD: no route flown, holding at depth" in _said(v)
    _silence(v, 20)
    assert v.mode == veh.SURFACE


def test_pdead_marker_with_nothing_after_it_is_no_route(hull_model):
    v = _vehicle(
        hull_model,
        mode=veh.GUIDED,
        FS_GCS_ENABLE=3.0,
        PDEAD_ACTION=2.0,
        PDEAD_HOLD_S=0.0,
    )
    v.new_mission([home_item(v), waypoint(v, 1, 0, 200), plain_item(2, DO_LAND_START)])
    _silence(v, 8)
    assert v.mode == veh.ALT_HOLD
    assert "PDEAD: no route flown, holding at depth" in _said(v)


def test_scripting_reads_enabled(hull_model):
    v, store = make_vehicle(hull_model)
    assert store.get("SCR_ENABLE") == 1 and store.get("PDEAD_ACTION") == 0
    assert store.get("PDEAD_HOLD_S") == 120


# -- the battery ---------------------------------------------------------------------


def _battery(model, **params):
    """An armed vehicle whose battery limits are set once it is under way (the monitor
    runs whether or not it is armed)."""
    v = _vehicle(model)
    v.params.values.update(params)
    return v


def test_battery_low_by_volts_after_the_timer(hull_model):
    v = _battery(
        hull_model, BATT_LOW_VOLT=40.0, BATT_FS_LOW_ACT=1.0, BATT_LOW_TIMER=10.0
    )
    run(v, 8)
    assert not _said(v)  # the timer has not run out
    run(v, 4)
    sev, template = captured.battery_template("low")
    line = [t for t in _said(v) if t.startswith("Battery 1 is low")]
    assert len(line) == 1 and line[0].startswith(template.split("{")[0])
    assert v.armed and v.critical
    run(v, 40)  # said again every 30 s
    assert len([t for t in _said(v) if t.startswith("Battery 1 is low")]) == 2


def test_battery_critical_by_charge_left_disarms(hull_model):
    v = _battery(hull_model, BATT_CRT_MAH=100000.0, BATT_FS_CRT_ACT=2.0)
    run(v, 1)
    assert any(t.startswith("Battery 1 is critical") for t in _said(v))
    assert not v.armed


def test_battery_low_surface(hull_model):
    v = _battery(hull_model, BATT_LOW_MAH=100000.0, BATT_FS_LOW_ACT=3.0)
    run(v, 1)
    assert v.mode == veh.SURFACE and v.mode_reason == "battery"


def test_battery_with_no_action_only_says(hull_model):
    v = _battery(hull_model, BATT_LOW_MAH=100000.0, BATT_FS_LOW_ACT=0.0)
    run(v, 1)
    assert any(t.startswith("Battery 1 is low") for t in _said(v))
    assert v.armed and v.mode == veh.ALT_HOLD and not v.critical


def test_battery_critical_follows_low(hull_model):
    v = _battery(
        hull_model, BATT_LOW_MAH=100000.0, BATT_FS_LOW_ACT=1.0,
        BATT_CRT_MAH=100000.0, BATT_FS_CRT_ACT=1.0,
    )  # fmt: skip
    run(v, 1)
    assert any(t.startswith("Battery 1 is critical") for t in _said(v))


# -- the pilot's input ---------------------------------------------------------------


def test_pilot_input_warning_shows_nothing_and_disarm_disarms(hull_model):
    v = _vehicle(hull_model, mode=veh.MANUAL, FS_PILOT_INPUT=1.0, FS_PILOT_TIMEOUT=2.0)
    for _ in range(40):
        v.manual_control(300, 500, 0)
        run(v, 0.1)
    assert v.armed and not v.texts
    run(v, 4)  # silent: ArduSub showed nothing for a warning
    assert v.armed and not v.critical and not v.texts
    v2 = _vehicle(hull_model, mode=veh.MANUAL, FS_PILOT_INPUT=2.0, FS_PILOT_TIMEOUT=2.0)
    for _ in range(40):
        v2.manual_control(300, 500, 0)
        run(v2, 0.1)
    assert v2.armed  # input keeps it
    run(v2, 4)
    assert not v2.armed


def test_the_capture_saw_no_pilot_input_failsafe():
    for action in (1, 2):
        on = captured.pilot_input(action)
        assert on["texts"] == [] and on["system_status"] == 4


def test_pilot_input_is_for_manual_and_stabilize_only(hull_model):
    v = _vehicle(
        hull_model, mode=veh.ALT_HOLD, FS_PILOT_INPUT=2.0, FS_PILOT_TIMEOUT=2.0
    )
    run(v, 10)
    assert v.armed and not v.critical


# -- the fence -----------------------------------------------------------------------


def test_fence_breach_is_shown_not_acted_on(hull_model):
    v = _vehicle(hull_model, FENCE_ENABLE=1.0, FENCE_TYPE=2.0, FENCE_RADIUS=5.0)
    run(v, 60)
    breached, kind, count, _ = v.fence
    assert breached and kind == 3 and count == 1
    assert v.armed and v.mode == veh.ALT_HOLD


def test_fence_minimum_altitude_is_the_depth_limit(hull_model):
    v = _vehicle(hull_model, FENCE_ENABLE=1.0, FENCE_TYPE=8.0, FENCE_ALT_MIN=-5.0)
    run(v, 1)
    assert v.fence[0] and v.fence[1] == 1


def test_fence_off_by_default(hull_model):
    v = _vehicle(hull_model)
    run(v, 30)
    assert not v.fence[0]


def test_gcs_failsafe_over_the_link(front_seat):
    """The words, the mode and the heartbeat's CRITICAL, as ArduSub shows them."""
    from pymavlink import mavutil

    M = mavutil.mavlink
    fs = front_seat(speedup=10.0)
    c = fs.client(system=255)  # MAV_GCS_SYSID defaults to 255
    for name, value in (
        ("FS_GCS_ENABLE", 3.0),
        ("FS_GCS_TIMEOUT", 2.0),
        ("FS_PILOT_INPUT", 0.0),
    ):
        c.m.mav.param_set_send(
            c.target, 1, name.encode(), value, M.MAV_PARAM_TYPE_REAL32
        )
        c.wait("PARAM_VALUE", cond=lambda p, n=name: p.param_id == n)
    assert c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.MANUAL) == M.MAV_RESULT_ACCEPTED
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    for _ in range(10):  # the GCS is heard, then it goes quiet
        c.beat()
        time.sleep(0.05)
    lost = c.wait("STATUSTEXT", cond=lambda t: "heartbeat lost" in t.text, timeout=10)
    assert (
        lost.text == captured.gcs_lost(3)[1]
        and lost.severity == captured.gcs_lost(3)[0]
    )
    hb = c.heartbeat(lambda h: h.system_status == M.MAV_STATE_CRITICAL)
    assert hb.custom_mode == veh.ALT_HOLD  # the failsafe's depth hold
    for _ in range(10):
        c.beat()
        time.sleep(0.05)
    back = c.wait("STATUSTEXT", cond=lambda t: "Cleared" in t.text)
    assert back.text == captured.gcs_cleared()[1]
    c.heartbeat(lambda h: h.system_status == M.MAV_STATE_ACTIVE)
