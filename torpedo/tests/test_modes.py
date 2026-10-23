"""The modes and the targets: what each does on a torpedo."""

import math
import time

from torpedo_sitl import vehicle as veh

from .support import HOME, M, make_vehicle, place, ready, run

DEG = math.pi / 180


def test_modes_by_both_message_forms(front_seat):
    fs = front_seat()
    c = fs.client()
    ready(c)
    for number in (veh.ALT_HOLD, veh.GUIDED, veh.MANUAL, veh.STABILIZE):
        c.set_mode(number)  # SET_MODE
        c.mode_is(number)
    for number in (veh.GUIDED, veh.ALT_HOLD, veh.MANUAL, veh.STABILIZE, veh.GUIDED):
        res = c.command(M.MAV_CMD_DO_SET_MODE, 1, number)  # MAV_CMD_DO_SET_MODE
        assert res == M.MAV_RESULT_ACCEPTED
        c.mode_is(number)
    # SURFACE is accepted; at the surface it switches itself to ALT_HOLD, as ArduSub
    assert c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.SURFACE) == M.MAV_RESULT_ACCEPTED
    c.mode_is(veh.ALT_HOLD)


def test_unsupported_modes_are_refused(front_seat):
    fs = front_seat()
    c = fs.client()
    for number in (1, 5, 6, 7, 16, 17, 18, 20):
        res = c.command(M.MAV_CMD_DO_SET_MODE, 1, number)
        assert res == M.MAV_RESULT_UNSUPPORTED
    c.set_mode(3)  # AUTO belongs to the missions (Part C2)
    c.set_mode(1)
    c.set_mode(veh.ALT_HOLD)
    assert c.mode_is(veh.ALT_HOLD).custom_mode == veh.ALT_HOLD


def test_arm_and_disarm(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    hb = c.heartbeat(lambda h: h.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
    assert hb.system_status == M.MAV_STATE_ACTIVE
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0) == M.MAV_RESULT_ACCEPTED
    c.heartbeat(lambda h: not h.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)


def test_guided_is_refused_until_the_position_estimate_is_valid(front_seat):
    """ArduSub refuses the mode (it does not refuse the arming): its own words."""
    fs = front_seat(speedup=1.0)
    c = fs.client()
    res = c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.GUIDED)
    assert res == M.MAV_RESULT_FAILED
    text = c.wait("STATUSTEXT", cond=lambda t: t.text.startswith("Mode change"))
    assert text.text == "Mode change failed: Guided requires position"
    assert text.severity == M.MAV_SEVERITY_WARNING
    assert not c.heartbeat().custom_mode == veh.GUIDED
    ready(c)  # five seconds of vehicle time later it has one
    assert c.command(M.MAV_CMD_DO_SET_MODE, 1, veh.GUIDED) == M.MAV_RESULT_ACCEPTED
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    assert c.wait("STATUSTEXT", cond=lambda t: t.text == "Arming motors")


def test_arming_in_alt_hold_needs_no_position(front_seat):
    fs = front_seat(speedup=1.0)
    c = fs.client()
    c.set_mode(veh.ALT_HOLD)
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED


def _go(c, fs, frame, mask, depth_alt, ahead=80.0, vx=0.0):
    """Arm in GUIDED and send a target ``ahead`` m east of home."""
    ready(c)
    c.set_mode(veh.GUIDED)
    c.mode_is(veh.GUIDED)
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    lat, lon = fs.frame.to_ll(0.0, ahead)
    c.position_target(frame, mask, lat, lon, depth_alt, vx)
    return (0.0, ahead)


def _within(c, fs, goal, radius, depth, seconds=40.0):
    """Wait for the hull to be within ``radius`` of ``goal`` at about ``depth``."""
    end = time.time() + seconds
    while time.time() < end:
        n, e, d = place(c, fs.frame)
        if math.hypot(n - goal[0], e - goal[1]) <= radius:
            assert abs(d - depth) < 1.5
            return
    raise AssertionError("never arrived")


def test_target_position_and_zero_velocity_relative_frame(front_seat):
    """The plugin's ``guided_posvel`` mask, in GLOBAL_RELATIVE_ALT_INT (depth = -alt)."""
    fs = front_seat()
    c = fs.client()
    mask = 0b0000_1111_1100_0000  # position, velocity not ignored, the rest ignored
    goal = _go(c, fs, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, -6.0)
    _within(c, fs, goal, fs.model.turn_radius_m + 2, 6.0)


def test_target_position_only_in_msl(front_seat):
    fs = front_seat()
    c = fs.client()
    mask = 0b0000_1101_1111_1000
    goal = _go(c, fs, M.MAV_FRAME_GLOBAL_INT, mask, -5.0)  # home altitude 0: depth 5
    _within(c, fs, goal, fs.model.turn_radius_m + 2, 5.0)


def test_target_in_the_terrain_frame_is_a_height_above_the_seabed(front_seat):
    fs = front_seat()
    c = fs.client()
    mask = 0b0000_1101_1111_1000
    goal = _go(c, fs, M.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, mask, 45.0)  # 50 m floor
    _within(c, fs, goal, fs.model.turn_radius_m + 2, 5.0)


def test_velocity_target_holds_course_and_depth(front_seat):
    fs = front_seat(speedup=10.0)
    c = fs.client()
    ready(c)
    c.set_mode(veh.GUIDED)
    c.mode_is(veh.GUIDED)
    c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    mask = 0b0000_1111_1100_0111  # velocity only
    start = time.time()
    while time.time() - start < 4.0:  # 40 s of vehicle time, a target every second
        c.position_target(M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, 0, 0, 0, 0.0, 2.0)
        time.sleep(0.1)
    g = c.fresh("GLOBAL_POSITION_INT")
    assert abs(g.vy / 100.0 - 2.0) < 0.4 and abs(g.vx / 100.0) < 0.4
    assert abs(g.hdg / 100.0 - 90.0) < 10


def test_manual_control_drives_the_thrust(front_seat):
    fs = front_seat(speedup=10.0)
    c = fs.client()
    ready(c)
    c.set_mode(veh.MANUAL)
    c.mode_is(veh.MANUAL)
    c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    start = time.time()
    while time.time() - start < 3.0:  # 30 s of vehicle time, an input every second
        c.m.mav.manual_control_send(c.target, 800, 0, 500, 0, 0)
        time.sleep(0.1)
    g = c.fresh("GLOBAL_POSITION_INT")
    assert math.hypot(g.vx, g.vy) / 100.0 > 1.0


# -- the modes, on a vehicle stepped by hand -------------------------------------------


def _armed(model, mode, depth=0.0, speed=0.0, **params):
    v, _ = make_vehicle(model, **params)
    run(v, 6.0)  # past the estimate delay
    v.hull.depth, v.hull.u = depth, speed
    assert v.set_mode(mode)[0]
    assert v.arm(True)[0]
    return v


def test_manual_without_input_for_a_second_is_thrust_zero(hull_model):
    v = _armed(hull_model, veh.MANUAL)
    for _ in range(400):  # twenty seconds of input
        v.manual_control(1000, 500, 0)
        v.step(0.05)
    assert v.hull.u > 1.0 and v.actuators.thrust_n > 0
    run(v, 2)  # no input for a second: the thrust is off and the fins neutral
    assert v.actuators.thrust_n == 0.0 and v.actuators.rudder_rad == 0.0


def test_manual_has_no_reverse_and_no_input_means_neutral(hull_model):
    v = _armed(hull_model, veh.MANUAL, depth=5.0)
    v.manual_control(-800, 500, 0)
    run(v, 5)
    assert v.hull.u < 0.1  # thrust below 0 is 0


def test_stabilize_holds_the_pitch_asked(hull_model):
    v = _armed(hull_model, veh.STABILIZE, depth=20.0, speed=1.5)
    for _ in range(400):  # z above 500 asks a nose-up pitch
        v.manual_control(600, 800, 0)
        v.step(0.05)
    assert 0.05 < v.hull.pitch <= hull_model.max_pitch_rad + 0.05


def test_alt_hold_at_the_surface_has_the_motor_off(hull_model):
    v = _armed(hull_model, veh.ALT_HOLD, depth=0.0)
    run(v, 30)
    assert v.hull.u < 0.05 and v.hull.depth < 0.5
    assert math.hypot(*v.position) < 1.0


def test_alt_hold_at_depth_circles_where_it_entered(hull_model):
    v = _armed(hull_model, veh.ALT_HOLD, depth=10.0, speed=1.5)
    centre = v.position
    r = 1.5 * hull_model.turn_radius_m
    far = 0.0
    for _ in range(int(120 / 0.05)):
        v.step(0.05)
        far = max(far, math.dist(v.position, centre))
    assert abs(v.hull.depth - 10.0) < 1.5
    assert far < 3 * r + 5  # a circle about the entry point: it never hovers or leaves
    assert v.hull.u > 0.5  # and it never stops


def test_surface_rises_on_its_heading_and_then_holds_with_the_motor_off(hull_model):
    v = _armed(hull_model, veh.SURFACE, depth=12.0, speed=1.5)
    heading = v.hull.heading
    for _ in range(int(240 / 0.05)):
        v.step(0.05)
        if v.mode != veh.SURFACE:
            break
    assert v.mode == veh.ALT_HOLD and v.hull.depth < veh.SURFACED_M + 0.2
    assert abs(math.remainder(v.hull.heading - heading, 2 * math.pi)) < 15 * DEG
    run(v, 5)
    assert v.actuators.thrust_n == 0.0  # the motor is off
    for _ in range(int(300 / 0.05)):  # and it stays off: the hull floats (#8)
        v.step(0.05)
        assert v.actuators.thrust_n == 0.0
    assert v.hull.depth < veh.SURFACED_M
    assert v.hull.speed_through_water < 0.3  # it has glided to a stop


def test_guided_with_no_target_circles_where_it_entered(hull_model):
    v = _armed(hull_model, veh.GUIDED, depth=8.0, speed=1.5)
    centre = v.position
    far = 0.0
    for _ in range(int(90 / 0.05)):
        v.step(0.05)
        far = max(far, math.dist(v.position, centre))
    assert far < 3 * 1.5 * hull_model.turn_radius_m + 5
    assert abs(v.hull.depth - 8.0) < 1.5


def test_a_held_point_is_circled_and_not_restarted(hull_model):
    v = _armed(hull_model, veh.GUIDED, depth=6.0, speed=1.5)
    lat, lon = v.frame.to_ll(60.0, 0.0)
    mask = 0b0000_1101_1111_1000
    args = (veh.FRAME_RELATIVE_ALT_INT, mask, lat, lon, -6.0, (0.0, 0.0))
    assert v.position_target(*args)[0]
    for i in range(int(200 / 0.05)):
        v.step(0.05)
        if i % 20 == 0:
            assert v.position_target(*args)[0]  # sent again and again, as the plugin
    assert v.pilot.guidance.done  # reached, then circling
    far = max(
        (math.dist(v.position, (60.0, 0.0)), v.step(0.05))[0] for _ in range(1000)
    )
    assert far < 3 * 1.5 * hull_model.turn_radius_m + 5


def test_velocity_target_times_out_into_a_circle(hull_model):
    v = _armed(hull_model, veh.GUIDED, depth=6.0, speed=1.5)
    mask = 0b0000_1111_1100_0111
    args = (veh.FRAME_RELATIVE_ALT_INT, mask, 0.0, 0.0, 0.0, (2.0, 0.0))
    for _ in range(40):
        v.position_target(*args)
        run(v, 1.0)
    assert abs(v.hull.velocity_ned()[0] - 2.0) < 0.4  # north at 2 m/s
    assert v.target is not None
    run(v, 4)  # three seconds without a new velocity: it circles where it is
    assert v.target is None
    stop = v.position
    far = 0.0
    for _ in range(int(60 / 0.05)):
        v.step(0.05)
        far = max(far, math.dist(v.position, stop))
    assert far < 6 * hull_model.turn_radius_m + 10


def test_change_speed_sets_the_speed_of_a_point_and_is_clamped(hull_model):
    v = _armed(hull_model, veh.GUIDED, depth=6.0, speed=1.0)
    lat, lon = v.frame.to_ll(400.0, 0.0)
    mask = 0b0000_1101_1111_1000
    v.position_target(veh.FRAME_RELATIVE_ALT_INT, mask, lat, lon, -6.0, (0.0, 0.0))
    v.change_speed(2.0)
    run(v, 60)
    assert 1.8 < v.hull.u < 2.2
    v.change_speed(50.0)
    assert v.speed == hull_model.max_speed_mps
    v.change_speed(0.1)
    assert v.speed == hull_model.steer_speed_mps


def test_a_target_outside_guided_is_ignored(hull_model):
    v = _armed(hull_model, veh.ALT_HOLD, depth=6.0, speed=1.5)
    lat, lon = v.frame.to_ll(60.0, 0.0)
    ok, _ = v.position_target(veh.FRAME_RELATIVE_ALT_INT, 0, lat, lon, -6.0, (0, 0))
    assert not ok and v.target is None


def test_the_depth_is_never_above_the_surface_or_below_the_limit(hull_model):
    v = _armed(hull_model, veh.GUIDED, depth=6.0, speed=1.5)
    lat, lon = v.frame.to_ll(60.0, 0.0)
    v.position_target(veh.FRAME_RELATIVE_ALT_INT, 0, lat, lon, 500.0, (0, 0))
    assert v.target.depth == 0.0  # alt +500 is 500 m above the surface
    v.position_target(veh.FRAME_RELATIVE_ALT_INT, 0, lat, lon, -5000.0, (0, 0))
    assert v.target.depth == hull_model.depth_limit_m


def test_it_cannot_hover(hull_model):
    """In every mode that holds, with a target reached or none, it keeps moving."""
    for mode in (veh.ALT_HOLD, veh.GUIDED):
        v = _armed(hull_model, mode, depth=9.0, speed=1.5)
        run(v, 60)
        assert v.hull.u >= hull_model.steer_speed_mps * 0.6
    assert HOME  # the start is a real place
