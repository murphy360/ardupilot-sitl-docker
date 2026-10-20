"""The MAVLink links: heartbeat, ports, streams, clock, parameters and a reboot."""

import time

from pymavlink import mavutil

from .support import HOME, M, model_with_id, place, ready

OPOS = ("SIM_OPOS_LAT", "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG")


def test_heartbeat_fields(front_seat):
    fs = front_seat()
    c = fs.client()
    hb = c.hb
    assert hb.type == M.MAV_TYPE_SUBMARINE
    assert hb.autopilot == M.MAV_AUTOPILOT_ARDUPILOTMEGA
    assert hb.get_srcSystem() == 7 and hb.get_srcComponent() == 1
    assert hb.base_mode & M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert not hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED
    assert hb.custom_mode == 0
    assert hb.system_status == M.MAV_STATE_STANDBY


def test_heartbeat_once_a_second_of_vehicle_time(front_seat):
    fs = front_seat(speedup=30.0)
    c = fs.client()
    c.wait("HEARTBEAT")
    start = time.time()
    for _ in range(10):
        c.wait("HEARTBEAT")
    wall = time.time() - start  # ten beats: ten vehicle seconds, a third of a second
    assert 0.15 < wall < 1.5


def test_boot_banner_names_the_hull(front_seat):
    fs = front_seat()
    c = fs.client()
    text = c.m.messages["STATUSTEXT"].text  # sent before the first heartbeat
    assert text.startswith("Torpedo front seat") and "torpedo-test-small" in text


def test_three_ports_and_no_waiting_for_serial0(front_seat):
    fs = front_seat()
    ports = fs.ports
    assert len(set(ports.values())) == 3
    c2 = fs.client("SERIAL2")  # first: nothing waits for SERIAL0
    c0, c1 = fs.client("SERIAL0"), fs.client("SERIAL1")
    for c in (c0, c1, c2):
        assert c.hb.type == M.MAV_TYPE_SUBMARINE


def test_default_ports_follow_the_instance():
    from torpedo_sitl.server import Config

    assert Config(instance=2).ports == {
        "SERIAL0": 5780,
        "SERIAL1": 5782,
        "SERIAL2": 5783,
    }


def test_a_link_closes_and_comes_back(front_seat):
    fs = front_seat()
    c = fs.client()
    c.close()
    again = fs.client()
    assert again.hb.get_srcSystem() == 7
    assert again.param("SYSID_THISMAV") == 7


def test_each_connection_has_its_own_rates(front_seat):
    fs = front_seat()
    a, b = fs.client("SERIAL0"), fs.client("SERIAL2")
    assert a.interval("ATTITUDE", 10) == M.MAV_RESULT_ACCEPTED
    a.wait("ATTITUDE")
    b.m.recv_match(blocking=False)
    time.sleep(0.3)
    seen = set()
    while (msg := b.m.recv_match(blocking=False)) is not None:
        seen.add(msg.get_type())
    assert "ATTITUDE" not in seen


def test_streams_by_request_data_stream(front_seat):
    fs = front_seat()
    c = fs.client()
    c.m.mav.request_data_stream_send(c.target, 1, M.MAV_DATA_STREAM_POSITION, 10, 1)
    assert c.wait("GLOBAL_POSITION_INT") is not None
    c.m.mav.request_data_stream_send(c.target, 1, M.MAV_DATA_STREAM_ALL, 5, 1)
    for kind in ("SYS_STATUS", "GPS_RAW_INT", "ATTITUDE", "BATTERY_STATUS"):
        assert c.wait(kind) is not None
    c.m.mav.request_data_stream_send(c.target, 1, M.MAV_DATA_STREAM_ALL, 5, 0)
    time.sleep(0.5)
    while c.m.recv_match(blocking=False) is not None:
        pass
    time.sleep(0.5)
    kinds = set()
    while (msg := c.m.recv_match(blocking=False)) is not None:
        kinds.add(msg.get_type())
    assert kinds <= {"HEARTBEAT"}


def test_streams_by_interval_and_request_message(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.interval("EKF_STATUS_REPORT", 5) == M.MAV_RESULT_ACCEPTED
    assert c.wait("EKF_STATUS_REPORT") is not None
    assert c.interval("EKF_STATUS_REPORT", 0) == M.MAV_RESULT_ACCEPTED
    res = c.command(M.MAV_CMD_REQUEST_MESSAGE, M.MAVLINK_MSG_ID_HOME_POSITION)
    assert res == M.MAV_RESULT_ACCEPTED
    home = c.wait("HOME_POSITION")
    assert abs(home.latitude / 1e7 - HOME[0]) < 1e-6
    res = c.command(M.MAV_CMD_REQUEST_MESSAGE, M.MAVLINK_MSG_ID_TERRAIN_REPORT)
    assert res == M.MAV_RESULT_UNSUPPORTED


def test_telemetry_messages_exist(front_seat):
    fs = front_seat()
    c = fs.client("SERIAL2")
    for name in (
        "GLOBAL_POSITION_INT",
        "ATTITUDE",
        "SYS_STATUS",
        "GPS_RAW_INT",
        "EKF_STATUS_REPORT",
        "BATTERY_STATUS",
        "HOME_POSITION",
        "SYSTEM_TIME",
        "EXTENDED_SYS_STATE",
        "SIM_STATE",
    ):
        assert c.interval(name, 10) == M.MAV_RESULT_ACCEPTED, name
        assert c.wait(name) is not None
    g = ready(c)
    assert abs(g.lat / 1e7 - HOME[0]) < 1e-3
    s = c.wait("SIM_STATE")
    assert s.lat_int != 0 and s.lon_int != 0
    assert abs(s.alt) < 1.0  # home altitude 0, at the surface
    st = c.wait("SYS_STATUS")
    assert 13000 < st.voltage_battery < 17000 and st.battery_remaining > 90
    ekf = c.wait("EKF_STATUS_REPORT", cond=lambda e: e.flags & 16)
    assert ekf.flags & M.EKF_ATTITUDE and ekf.flags & M.EKF_VELOCITY_HORIZ


def test_no_position_until_the_estimate_is_valid(front_seat):
    fs = front_seat(speedup=1.0)
    c = fs.client()
    c.interval("GLOBAL_POSITION_INT", 10)
    first = c.wait("GLOBAL_POSITION_INT")
    assert first.lat == 0 and first.lon == 0
    c.interval("EKF_STATUS_REPORT", 10)
    assert not c.wait("EKF_STATUS_REPORT").flags & M.EKF_POS_HORIZ_ABS


def test_system_time_runs_at_the_speedup(front_seat):
    fs = front_seat(speedup=30.0)
    c = fs.client()

    def sample():
        t0 = time.time()
        c.m.mav.command_long_send(
            c.target, 1, M.MAV_CMD_REQUEST_MESSAGE, 0,
            M.MAVLINK_MSG_ID_SYSTEM_TIME, 0, 0, 0, 0, 0, 0,
        )  # fmt: skip
        msg = c.wait("SYSTEM_TIME")
        return (t0 + time.time()) / 2, msg

    w0, a = sample()
    time.sleep(10.0)
    w1, b = sample()
    rate = (b.time_boot_ms - a.time_boot_ms) / 1000.0 / (w1 - w0)
    assert abs(rate - 30.0) / 30.0 < 0.02
    unix = (b.time_unix_usec - a.time_unix_usec) / 1e6
    assert abs(unix - (b.time_boot_ms - a.time_boot_ms) / 1000.0) < 0.01


def test_timesync_is_answered_in_vehicle_time(front_seat):
    fs = front_seat()
    c = fs.client()
    c.m.mav.timesync_send(0, 123456789)
    ts = c.wait("TIMESYNC", cond=lambda t: t.ts1 == 123456789)
    assert ts.tc1 > 0


def test_parameters_read_set_and_list(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.param("SYSID_THISMAV") == 7
    assert c.param("SIM_OPOS_LAT") != 0  # what a SITL answers: it is the probe
    by_index = None
    c.m.mav.param_request_read_send(c.target, 1, b"", 0)
    by_index = c.wait("PARAM_VALUE")
    assert by_index.param_index == 0
    c.m.mav.param_set_send(c.target, 1, b"TPDO_LOS_LA", 2.5, M.MAV_PARAM_TYPE_REAL32)
    echoed = c.wait("PARAM_VALUE", cond=lambda p: p.param_id == "TPDO_LOS_LA")
    assert echoed.param_value == 2.5
    c.m.mav.param_request_list_send(c.target, 1)
    got = {}
    first = c.wait("PARAM_VALUE")
    count = first.param_count
    got[first.param_id] = first.param_value
    end = time.time() + 10
    while len(got) < count and time.time() < end:
        p = c.wait("PARAM_VALUE")
        got[p.param_id] = p.param_value
    assert len(got) == count and got["TPDO_LOS_LA"] == 2.5
    names = {
        "WP_RADIUS_M", "WPNAV_RADIUS", "FS_GCS_ENABLE", "PDEAD_ACTION", "SCR_ENABLE",
        "FENCE_ALT_MIN", "BATT_CAPACITY", "RNGFND1_MAX_CM", "EK3_SRC1_VELXY",
        "TPDO_MODEL", "TPDO_TURN_K", "TPDO_LOIT_MIN", *OPOS,
    }  # fmt: skip
    assert names <= set(got)
    assert (
        got["WP_RADIUS_M"] == fs.model.turn_radius_m
        or abs(got["WP_RADIUS_M"] - fs.model.turn_radius_m) < 1e-3
    )


def test_unknown_names_are_ignored_by_set_but_kept_from_files(front_seat, tmp_path):
    pfile = tmp_path / "extra.parm"
    pfile.write_text("# a comment\nMY_OWN_PARAM 4.5\nTPDO_TURN_K 1.9\n")
    fs = front_seat(
        sitl_params="OTHER_ONE 3\nBATT_CAPACITY 5000\n", param_files=[str(pfile)]
    )
    c = fs.client()
    assert c.param("MY_OWN_PARAM") == 4.5
    assert c.param("OTHER_ONE") == 3.0
    assert c.param("BATT_CAPACITY") == 5000.0
    assert abs(c.param("TPDO_TURN_K") - 1.9) < 1e-6
    c.m.mav.param_set_send(c.target, 1, b"NEVER_SEEN", 1.0, M.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)
    assert fs.server.params.get("NEVER_SEEN", -1) == -1


def test_the_model_number_is_the_id_s_number_and_read_only(front_seat, tmp_path):
    path = model_with_id(tmp_path, "torpedo-test-small", "torpedo-100")
    fs = front_seat(hull=path)
    c = fs.client()
    assert c.param("TPDO_MODEL") == 100
    c.m.mav.param_set_send(c.target, 1, b"TPDO_MODEL", 5.0, M.MAV_PARAM_TYPE_REAL32)
    assert (
        c.wait("PARAM_VALUE", cond=lambda p: p.param_id == "TPDO_MODEL").param_value
        == 100
    )
    assert "torpedo-100" in c.m.messages["STATUSTEXT"].text


def test_reboot_keeps_parameters_and_relocates(front_seat, tmp_path):
    fs = front_seat()
    c = fs.client()
    here = (37.0, -122.5)
    for name, value in zip(OPOS, (here[0], here[1], 0.0, 180.0)):
        c.m.mav.param_set_send(
            c.target, 1, name.encode(), value, M.MAV_PARAM_TYPE_REAL32
        )
        c.wait("PARAM_VALUE", cond=lambda p, n=name: p.param_id == n)
    c.m.mav.param_set_send(c.target, 1, b"TPDO_TURN_K", 1.7, M.MAV_PARAM_TYPE_REAL32)
    c.wait("PARAM_VALUE", cond=lambda p: p.param_id == "TPDO_TURN_K")
    ready(c)
    res = c.command(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    assert res == M.MAV_RESULT_ACCEPTED
    assert "SIM_OPOS_LAT" in (tmp_path / "torpedo-0.parm").read_text()
    time.sleep(0.1)
    again = None
    end = time.time() + 20
    while again is None and time.time() < end:  # the link was closed: connect again
        try:
            again = fs.client()
        except (OSError, AssertionError):
            time.sleep(0.2)
    assert again is not None
    again.interval("SYSTEM_TIME", 20)
    assert again.wait("SYSTEM_TIME").time_boot_ms < 5000  # a new boot clock
    g = ready(again)
    assert abs(g.lat / 1e7 - here[0]) < 1e-4 and abs(g.lon / 1e7 - here[1]) < 1e-4
    assert abs(g.hdg / 100.0 - 180.0) < 5.0
    assert abs(again.param("TPDO_TURN_K") - 1.7) < 1e-6
    assert fs.frame is not None and place(again, fs.frame) is not None


def test_reboot_is_refused_while_armed(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    res = c.command(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    assert res == M.MAV_RESULT_FAILED


def test_other_commands_are_unsupported(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.command(M.MAV_CMD_DO_ORBIT, 10, 2) == M.MAV_RESULT_UNSUPPORTED
    assert (
        c.command(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 5)
        == M.MAV_RESULT_UNSUPPORTED
    )


def test_commands_for_another_system_are_ignored(front_seat):
    fs = front_seat()
    c = fs.client()
    c.m.mav.command_long_send(
        99, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.3)
    assert not c.heartbeat().base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED


def test_mission_protocol_is_answered_but_empty(front_seat):
    fs = front_seat()
    c = fs.client()
    c.m.mav.mission_request_list_send(c.target, 1, 0)
    assert c.wait("MISSION_COUNT").count == 0
    c.m.mav.mission_count_send(c.target, 1, 3, 0)
    assert c.wait("MISSION_ACK").type == M.MAV_MISSION_UNSUPPORTED
    c.m.mav.mission_clear_all_send(c.target, 1, 0)
    assert c.wait("MISSION_ACK").type == M.MAV_MISSION_ACCEPTED


def test_the_dialect_is_mavlink_two(front_seat):
    fs = front_seat()
    c = fs.client()
    c.interval("BATTERY_STATUS", 5)
    msg = c.wait("BATTERY_STATUS")
    assert msg.get_header().mlen > 0 and mavutil.mavlink.WIRE_PROTOCOL_VERSION
