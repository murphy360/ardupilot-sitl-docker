"""Check that a SITL can be relocated at runtime over MAVLink.

Runs inside a SITL container (uses the image's pymavlink) and talks to the
simulator on tcp:127.0.0.1:5760, so it must be the only client on that port.

usage: sitl_relocate.py check LAT LON HDG
           Assert the vehicle and home are at LAT/LON with heading HDG.
       sitl_relocate.py relocate LAT LON ALT HDG [--arm]
           PARAM_SET SIM_OPOS_*, send MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, reconnect
           and assert the vehicle came back at the new location. With --arm,
           also arm (and take off, for Copter) and report the timings.

Prints one JSON line with the measurements; exits non-zero on failure.
Heading is checked against SIMSTATE (the simulator's true yaw): the EKF's
ATTITUDE yaw is reported too but takes ~a minute to converge after boot.
"""
import json
import math
import sys
import time

from pymavlink import mavutil

URL = "tcp:127.0.0.1:5760"
M = mavutil.mavlink


class Disconnected(Exception):
    """The simulator closed the TCP connection (e.g. it rebooted)."""


def _disconnected():
    raise Disconnected()


def connect(timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            m = mavutil.mavlink_connection(URL, source_system=250, autoreconnect=False)
            # pymavlink's defaults on a closed socket are to spin printing "EOF"
            # or to silently reconnect mid-read; make it an exception instead.
            m.handle_eof = _disconnected
            m.handle_disconnect = _disconnected
            if m.wait_heartbeat(timeout=5):
                m.mav.request_data_stream_send(m.target_system, m.target_component,
                                               M.MAV_DATA_STREAM_ALL, 10, 1)
                return m
            m.close()
        except (OSError, Disconnected):
            pass
        time.sleep(0.25)
    raise SystemExit("no heartbeat within %ds" % timeout)


def dist_m(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians(lat1))
    return 6371000 * math.hypot(dlat, dlon)


def hdg_err(a, b):
    return abs((a - b + 180) % 360 - 180)


def position(m, timeout=120):
    """Wait for a real GPS-derived position, then read home and heading."""
    deadline = time.time() + timeout
    pos = None
    while time.time() < deadline:
        p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if p and (p.lat or p.lon):
            pos = p
            break
    if pos is None:
        raise SystemExit("no GLOBAL_POSITION_INT fix within %ds" % timeout)
    home = None
    while time.time() < deadline and home is None:
        m.mav.command_long_send(m.target_system, m.target_component,
                                M.MAV_CMD_REQUEST_MESSAGE, 0,
                                M.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
        home = m.recv_match(type="HOME_POSITION", blocking=True, timeout=2)
    if home is None:
        raise SystemExit("no HOME_POSITION within %ds" % timeout)
    att = m.recv_match(type="ATTITUDE", blocking=True, timeout=5)
    # SIMSTATE is the simulator's true state; ATTITUDE is the EKF estimate,
    # whose yaw takes ~a minute to converge after boot.
    sim = m.recv_match(type="SIMSTATE", blocking=True, timeout=5)
    pos = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5) or pos
    return {
        "lat": pos.lat / 1e7, "lon": pos.lon / 1e7, "alt_amsl": pos.alt / 1e3,
        "home_lat": home.latitude / 1e7, "home_lon": home.longitude / 1e7,
        "home_alt_amsl": home.altitude / 1e3,
        "yaw_deg": round(math.degrees(sim.yaw) % 360, 1),
        "ekf_yaw_deg": round(math.degrees(att.yaw) % 360, 1),
    }


def assert_at(r, lat, lon, hdg, tol_m=50, tol_hdg=10):
    r["pos_err_m"] = round(dist_m(lat, lon, r["lat"], r["lon"]), 1)
    r["home_err_m"] = round(dist_m(lat, lon, r["home_lat"], r["home_lon"]), 1)
    r["hdg_err_deg"] = round(hdg_err(r["yaw_deg"], hdg), 1)
    r["ok"] = r["pos_err_m"] < tol_m and r["home_err_m"] < tol_m and r["hdg_err_deg"] < tol_hdg
    return r


def set_param(m, name, value):
    for _ in range(10):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(value), M.MAV_PARAM_TYPE_REAL32)
        pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        while pv and pv.param_id != name:
            pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if pv and abs(pv.param_value - value) < 1e-3 * max(1, abs(value)):
            return
    raise SystemExit("PARAM_SET %s=%s not acknowledged" % (name, value))


def arm(m, vehicle, timeout=300):
    mode = {"ArduCopter": "GUIDED", "ArduPlane": "MANUAL",
            "Rover": "MANUAL", "ArduSub": "MANUAL"}[vehicle]
    deadline = time.time() + timeout
    last_text = ""
    while time.time() < deadline:
        m.set_mode(m.mode_mapping()[mode])
        m.mav.command_long_send(m.target_system, m.target_component,
                                M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        end = time.time() + 1.0
        while time.time() < end:
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                last_text = msg.text
            elif msg.get_srcSystem() == m.target_system and \
                    msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
                return time.time()
    raise SystemExit("could not arm within %ds (last: %s)" % (timeout, last_text))


def takeoff(m, alt=10, timeout=60):
    m.mav.command_long_send(m.target_system, m.target_component,
                            M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    deadline = time.time() + timeout
    while time.time() < deadline:
        p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if p and p.relative_alt / 1e3 > alt * 0.8:
            return round(p.relative_alt / 1e3, 1)
    raise SystemExit("takeoff did not reach %sm" % alt)


def main():
    try:
        run()
    except Disconnected:
        raise SystemExit("simulator closed the connection unexpectedly")


def run():
    phase = sys.argv[1]
    m = connect()
    vehicle = {M.MAV_TYPE_QUADROTOR: "ArduCopter", M.MAV_TYPE_FIXED_WING: "ArduPlane",
               M.MAV_TYPE_GROUND_ROVER: "Rover", M.MAV_TYPE_SUBMARINE: "ArduSub"}
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    veh = vehicle.get(hb.type, "?")
    if phase == "check":
        lat, lon, hdg = map(float, sys.argv[2:5])
        r = assert_at(position(m), lat, lon, hdg)
    else:
        lat, lon, alt, hdg = map(float, sys.argv[2:6])
        do_arm = "--arm" in sys.argv[6:]
        # Proteus-style detection: only relocate if SIM_OPOS_LAT exists
        m.mav.param_request_read_send(m.target_system, m.target_component, b"SIM_OPOS_LAT", -1)
        if not m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5):
            raise SystemExit("SIM_OPOS_LAT not readable")
        for name, v in (("SIM_OPOS_LAT", lat), ("SIM_OPOS_LNG", lon),
                        ("SIM_OPOS_ALT", alt), ("SIM_OPOS_HDG", hdg)):
            set_param(m, name, v)
        t0 = time.time()
        m.mav.command_long_send(m.target_system, m.target_component,
                                M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        m.close()
        # Only count the reboot as done once we're talking to the new process:
        # its boot clock restarts from zero.
        while True:
            m = connect()
            try:
                a = m.recv_match(type="ATTITUDE", blocking=True, timeout=5)
            except Disconnected:
                a = None
            if a and a.time_boot_ms < (time.time() - t0) * 1000 + 5000:
                break
            m.close()
            time.sleep(0.25)
        t_hb = time.time()
        r = assert_at(position(m), lat, lon, hdg)
        t_fix = time.time()
        r["reboot_to_heartbeat_s"] = round(t_hb - t0, 1)
        r["reboot_to_position_s"] = round(t_fix - t0, 1)
        if do_arm:
            r["reboot_to_armed_s"] = round(arm(m, veh) - t0, 1)
            if veh == "ArduCopter":
                r["takeoff_rel_alt_m"] = takeoff(m)
    r["vehicle"] = veh
    r["ok"] = r["ok"] and veh != "?"
    r["phase"] = phase
    print(json.dumps(r))
    sys.exit(0 if r["ok"] else 1)


if __name__ == "__main__":
    main()
