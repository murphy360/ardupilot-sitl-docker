"""The mission protocol and AUTO."""

import math
import time

import pytest

from torpedo_sitl import captured
from torpedo_sitl import vehicle as veh
from torpedo_sitl.mission import (
    DO_CHANGE_SPEED,
    DO_LAND_START,
    NAV_LOITER_TURNS,
    NAV_LOITER_UNLIM,
    NAV_WAYPOINT,
)

from .support import (
    M,
    download_mission,
    home_item,
    make_vehicle,
    plain_item,
    run,
    upload_mission,
    waypoint,
)

DEG = math.pi / 180


def _auto(model, items, depth=6.0, speed=1.5, **params):
    """A vehicle armed at depth (item 0 is added to ``items``); the test sets AUTO."""
    v, _ = make_vehicle(model, FS_PILOT_INPUT=0, **params)
    run(v, 6.0)
    v.hull.depth, v.hull.u = depth, speed
    v.world.depth = depth
    v.new_mission([home_item(v), *items])
    assert v.arm(True)[0]
    return v


def _fly(v, seconds, until=None):
    for _ in range(int(seconds / 0.05)):
        v.step(0.05)
        if until is not None and until(v):
            return True
    return until is None


def _line(v, *points, depth=6.0):
    return [waypoint(v, i + 1, n, e, depth) for i, (n, e) in enumerate(points)]


# -- the protocol ------------------------------------------------------------------


def test_upload_and_read_back(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    items = [
        home_item(v),
        waypoint(v, 1, 100, 0),
        plain_item(2, DO_CHANGE_SPEED, param1=1, param2=2.0),
        waypoint(v, 3, 100, 100, 8.0, radius=4.0),
        waypoint(v, 4, 0, 100, 8.0, command=NAV_LOITER_UNLIM),
        plain_item(5, DO_LAND_START),
    ]
    ack = upload_mission(c, items)
    assert ack.type == M.MAV_MISSION_ACCEPTED
    back = download_mission(c)
    assert len(back) == 6
    for sent, got in zip(items[1:], back[1:]):
        assert (got.seq, got.command, got.frame, got.x, got.y) == (
            sent.seq, sent.command, sent.frame, sent.x, sent.y,
        )  # fmt: skip
        assert got.param1 == sent.param1 and got.param2 == sent.param2
        assert got.z == pytest.approx(sent.z)
    assert back[0].command == NAV_WAYPOINT and back[0].frame == M.MAV_FRAME_GLOBAL
    assert abs(back[0].x / 1e7 - 36.6) < 1e-6  # item 0 is rewritten as home


def test_the_old_item_message_is_taken_too(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    items = [home_item(v), waypoint(v, 1, 50, 50)]
    ack = upload_mission(c, items, kind="MISSION_ITEM")
    assert ack.type == M.MAV_MISSION_ACCEPTED
    assert abs(download_mission(c)[1].x - items[1].x) < 100  # the old message's float32


def test_a_duplicate_item_is_answered_invalid_sequence_and_the_upload_goes_on(
    front_seat,
):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    items = [home_item(v), waypoint(v, 1, 50, 0), waypoint(v, 2, 100, 0)]
    c.m.mav.mission_count_send(c.target, 1, 3, 0)
    assert c.wait("MISSION_REQUEST_INT").seq == 0
    from .support import send_item

    send_item(c, items[0])
    send_item(c, items[0])  # the same item twice
    # the first copy was taken and the next item asked for; the second is refused
    assert c.wait("MISSION_REQUEST_INT").seq == 1
    ack = c.wait("MISSION_ACK")
    assert ack.type == M.MAV_MISSION_INVALID_SEQUENCE
    send_item(c, items[1])
    assert c.wait("MISSION_REQUEST_INT").seq == 2
    send_item(c, items[2])
    assert c.wait("MISSION_ACK").type == M.MAV_MISSION_ACCEPTED
    assert len(download_mission(c)) == 3


def test_fence_and_rally_uploads_are_unsupported(front_seat):
    fs = front_seat()
    c = fs.client()
    for kind in (M.MAV_MISSION_TYPE_FENCE, M.MAV_MISSION_TYPE_RALLY):
        c.m.mav.mission_count_send(c.target, 1, 2, kind)
        ack = c.wait("MISSION_ACK")
        assert ack.type == M.MAV_MISSION_UNSUPPORTED and ack.mission_type == kind


def test_clear_all(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    upload_mission(c, [home_item(v), waypoint(v, 1, 50, 0)])
    c.m.mav.mission_clear_all_send(c.target, 1, 0)
    assert c.wait("MISSION_ACK").type == M.MAV_MISSION_ACCEPTED
    assert len(download_mission(c)) == 1


def test_auto_is_refused_with_no_mission(front_seat):
    fs = front_seat()
    c = fs.client()
    c.interval("GLOBAL_POSITION_INT", 10)
    c.wait("GLOBAL_POSITION_INT", cond=lambda g: g.lat != 0)
    assert c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.AUTO) == M.MAV_RESULT_FAILED
    text = c.wait("STATUSTEXT", cond=lambda t: "failed" in t.text)
    assert text.text == captured.auto_refused()[1] and text.severity == 4


def test_do_set_mission_current_and_mission_current(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    upload_mission(c, [home_item(v), waypoint(v, 1, 50, 0), waypoint(v, 2, 80, 0)])
    assert c.command(M.MAV_CMD_DO_SET_MISSION_CURRENT, 2) == M.MAV_RESULT_ACCEPTED
    c.interval("MISSION_CURRENT", 10)
    cur = c.wait("MISSION_CURRENT", cond=lambda m: m.seq == 2)
    assert cur.total == 3
    assert c.command(M.MAV_CMD_DO_SET_MISSION_CURRENT, 9) == M.MAV_RESULT_FAILED


def test_the_mission_survives_a_reboot(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    upload_mission(c, [home_item(v), waypoint(v, 1, 50, 0)])
    assert c.command(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1) == M.MAV_RESULT_ACCEPTED
    time.sleep(0.5)
    again = fs.client()
    assert len(download_mission(again)) == 2


def test_auto_flies_an_uploaded_route_over_the_link(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    upload_mission(
        c, [home_item(v), waypoint(v, 1, 0, 60, 5.0), waypoint(v, 2, 60, 60, 5.0)]
    )
    c.interval("GLOBAL_POSITION_INT", 10)
    c.wait("GLOBAL_POSITION_INT", cond=lambda g: g.lat != 0)
    assert c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.AUTO) == M.MAV_RESULT_ACCEPTED
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    first = c.wait("MISSION_ITEM_REACHED", timeout=60)
    second = c.wait("MISSION_ITEM_REACHED", timeout=60)
    assert (first.seq, second.seq) == (1, 2)
    cur = c.wait("MISSION_CURRENT")
    assert cur.total == 3


def test_a_mission_written_in_auto_restarts_the_command(front_seat):
    fs = front_seat()
    c = fs.client()
    v, _ = make_vehicle(fs.model)
    upload_mission(c, [home_item(v), waypoint(v, 1, 200, 0)])
    c.interval("GLOBAL_POSITION_INT", 10)
    c.wait("GLOBAL_POSITION_INT", cond=lambda g: g.lat != 0)
    c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.AUTO)
    c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    upload_mission(c, [home_item(v), waypoint(v, 1, 0, 200)])
    seen = c.wait("STATUSTEXT", cond=lambda t: "restarted" in t.text)
    assert seen.severity == 2


# -- AUTO on a vehicle stepped by hand -----------------------------------------------


def test_waypoints_are_reached_in_order_and_the_end_is_circled(hull_model):
    v = _auto(hull_model, [])
    v.new_mission([home_item(v), *_line(v, (0, 100), (100, 100), (100, 0))])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert _fly(v, 400, lambda v: v.mission.complete)
    assert v.reached[:3] == [1, 2, 3]
    assert v.mission.state == 4
    stop = v.position
    far = 0.0
    for _ in range(1200):
        v.step(0.05)
        far = max(far, math.dist(v.position, stop))
    assert far < 6 * hull_model.turn_radius_m  # circling the last point, not leaving it
    assert abs(v.hull.depth - 6.0) < 1.5


def test_the_corner_is_cut_and_the_lane_is_flown(hull_model):
    """A 90 degree corner: the turn reaches no more than 20 m past the lane's end."""
    v = _auto(hull_model, [])
    v.new_mission([home_item(v), *_line(v, (0, 200), (200, 200))])
    v.set_mode(veh.AUTO)
    v.auto.start()
    far = 0.0
    for _ in range(int(300 / 0.05)):
        v.step(0.05)
        far = max(far, v.hull.east - 200.0)
        if v.mission.complete:
            break
    assert far < 20.0


def test_a_depth_per_waypoint(hull_model):
    v = _auto(hull_model, [])
    items = [
        waypoint(v, 1, 0, 150, 4.0),
        waypoint(v, 2, 150, 150, 9.0),
    ]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert _fly(v, 400, lambda v: 2 in v.reached)
    run(v, 40)
    assert abs(v.hull.depth - 9.0) < 1.5


def test_do_change_speed_sets_the_speed_of_the_next_leg(hull_model):
    v = _auto(hull_model, [])
    items = [
        waypoint(v, 1, 0, 80, 6.0),
        plain_item(2, DO_CHANGE_SPEED, param1=1, param2=2.0),
        waypoint(v, 3, 300, 80, 6.0),
    ]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert _fly(v, 200, lambda v: 1 in v.reached)
    run(v, 40)
    assert v.speed == 2.0 and 1.8 < v.hull.u < 2.2


def test_loiter_unlim_circles_for_good_and_the_marker_is_passed(hull_model):
    v = _auto(hull_model, [])
    items = [
        waypoint(v, 1, 0, 100, 6.0),
        waypoint(v, 2, 0, 100, 6.0, command=NAV_LOITER_UNLIM),
        plain_item(3, DO_LAND_START),
        waypoint(v, 4, 300, 0, 6.0),
    ]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert _fly(v, 200, lambda v: 1 in v.reached)
    run(v, 60)
    assert v.mission.current == 2 and not v.mission.complete
    far = 0.0
    for _ in range(3000):
        v.step(0.05)
        far = max(far, math.dist(v.truth, (0.0, 100.0)))
    assert far < 4 * hull_model.turn_radius_m  # never goes on to item 4
    assert 4 not in v.reached


def test_loiter_turns_raised_joined_and_reached(hull_model):
    v = _auto(hull_model, [])
    least = 1.5 * hull_model.turn_radius_m
    items = [
        waypoint(v, 1, 0, 150, 6.0, command=NAV_LOITER_TURNS, param1=2, param3=3.0),
        waypoint(v, 2, 150, 150, 6.0),
    ]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert any("raised" in t for _, t in v.texts)
    assert _fly(v, 600, lambda v: 1 in v.reached)
    turns = v.t
    assert 2 not in v.reached
    assert _fly(v, 400, lambda v: 2 in v.reached)
    assert least > 3.0 and v.t > turns


def test_loiter_turns_counter_clockwise_with_a_negative_radius(hull_model):
    v = _auto(hull_model, [])
    r = 2.5 * hull_model.turn_radius_m
    items = [waypoint(v, 1, 0, 120, 6.0, command=NAV_LOITER_TURNS, param1=1, param3=-r)]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert not v.pilot.guidance.clockwise
    assert _fly(v, 600, lambda v: 1 in v.reached)


def test_unknown_items_are_skipped_and_said(hull_model):
    v = _auto(hull_model, [])
    items = [plain_item(1, M.MAV_CMD_NAV_TAKEOFF), waypoint(v, 2, 0, 80, 6.0)]
    v.new_mission([home_item(v), *items])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert any("Skipping" in t for _, t in v.texts)
    assert _fly(v, 200, lambda v: 2 in v.reached)


def test_auto_resumes_at_the_current_item_and_restarts_when_it_is_set(hull_model):
    v = _auto(hull_model, [])
    v.new_mission([home_item(v), *_line(v, (0, 80), (80, 80), (160, 80))])
    v.set_mode(veh.AUTO)
    v.auto.start()
    assert v.set_current_item(3)
    run(v, 1)
    assert v.mission.current == 3
    assert _fly(v, 300, lambda v: 3 in v.reached)
    assert 1 not in v.reached and 2 not in v.reached


def test_a_mission_without_commands_is_not_flown(hull_model):
    v, _ = make_vehicle(hull_model)
    run(v, 6)
    v.new_mission([home_item(v)])
    assert v.set_mode(veh.AUTO) == (False, "refused")
