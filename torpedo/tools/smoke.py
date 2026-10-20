"""Smoke test for the image at 30x: arm, fly GUIDED to a point 300 m away at 4 m depth
and see it within the waypoint radius in under 30 s of wall time; then upload a
three-point route, AUTO, and see ``MISSION_ITEM_REACHED`` for each point in under 30 s.

Runs inside the container (it uses the image's pymavlink) against SERIAL0:

    python3 smoke.py [tcp:127.0.0.1:5760]

Prints one JSON line; exits non-zero on failure.
"""

import json
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
URL = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760"
AHEAD_M = 300.0
DEPTH_M = 4.0
WALL_LIMIT_S = 30.0
ESTIMATE_TIMEOUT_S = 60.0


def metres(lat0, lon0, lat, lon):
    """(north, east) metres of a position from another, flat earth."""
    north = math.radians(lat - lat0) * 6371000.0
    east = math.radians(lon - lon0) * 6371000.0 * math.cos(math.radians(lat0))
    return north, east


def need(m, kind, timeout, cond=None):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=kind, blocking=True, timeout=1)
        if msg is not None and (cond is None or cond(msg)):
            return msg
    raise SystemExit(f"no {kind} within {timeout:.0f} s")


def command(m, cmd, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, 1, cmd, 0, *p)
    return need(m, "COMMAND_ACK", 10, lambda a: a.command == cmd).result


def upload(m, items):
    """Upload ``items`` ((lat, lon, depth) for items 1 and up; item 0 is home)."""
    rows = [(0.0, 0.0, 0.0)] + list(items)
    m.mav.mission_count_send(m.target_system, 1, len(rows), 0)
    while True:
        msg = need(m, "MISSION_REQUEST_INT", 10)
        lat, lon, depth = rows[msg.seq]
        m.mav.mission_item_int_send(
            m.target_system, 1, msg.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            M.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), -depth, 0,
        )  # fmt: skip
        if msg.seq == len(rows) - 1:
            break
    ack = need(m, "MISSION_ACK", 10)
    if ack.type != M.MAV_MISSION_ACCEPTED:
        raise SystemExit(f"mission refused: {ack.type}")


def route(m, lat0, lon0, radius):
    """Fly a three-point route in AUTO; return the wall seconds it took."""
    d = math.degrees(80.0 / 6371000.0)
    dlon = d / math.cos(math.radians(lat0))
    upload(
        m,
        [(lat0 + d, lon0, 4.0), (lat0 + d, lon0 + dlon, 4.0), (lat0, lon0 + dlon, 4.0)],
    )
    if command(m, M.MAV_CMD_DO_SET_MODE, 1, 3) != M.MAV_RESULT_ACCEPTED:
        raise SystemExit("AUTO refused")
    start = time.time()
    for seq in (1, 2, 3):
        left = WALL_LIMIT_S - (time.time() - start)
        need(m, "MISSION_ITEM_REACHED", max(left, 1), lambda r, s=seq: r.seq == s)
    return time.time() - start


def main():
    m = mavutil.mavlink_connection(URL, source_system=250, source_component=190)
    if m.wait_heartbeat(timeout=30) is None:
        raise SystemExit("no heartbeat")
    command(
        m, M.MAV_CMD_SET_MESSAGE_INTERVAL, M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1e5
    )
    here = need(m, "GLOBAL_POSITION_INT", ESTIMATE_TIMEOUT_S, lambda g: g.lat != 0)
    lat0, lon0 = here.lat / 1e7, here.lon / 1e7
    m.mav.param_request_read_send(m.target_system, 1, b"WP_RADIUS_M", -1)
    radius = need(m, "PARAM_VALUE", 10, lambda p: p.param_id == "WP_RADIUS_M")
    radius = radius.param_value
    if command(m, M.MAV_CMD_DO_SET_MODE, 1, 4) != M.MAV_RESULT_ACCEPTED:
        raise SystemExit("GUIDED refused")
    if command(m, M.MAV_CMD_COMPONENT_ARM_DISARM, 1) != M.MAV_RESULT_ACCEPTED:
        raise SystemExit("arming refused")
    lat = lat0 + math.degrees(AHEAD_M / 6371000.0)
    m.mav.set_position_target_global_int_send(
        0, m.target_system, 1, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000_1111_1100_0000,
        int(lat * 1e7), int(lon0 * 1e7), -DEPTH_M, 0, 0, 0, 0, 0, 0, 0, 0,
    )  # fmt: skip
    goal = metres(lat0, lon0, lat, lon0)
    start, best = time.time(), math.inf
    while time.time() - start < WALL_LIMIT_S:
        g = need(m, "GLOBAL_POSITION_INT", 5, lambda g: g.lat != 0)
        n, e = metres(lat0, lon0, g.lat / 1e7, g.lon / 1e7)
        best = min(best, math.hypot(goal[0] - n, goal[1] - e))
        if best <= radius:
            wall = time.time() - start
            auto = route(m, g.lat / 1e7, g.lon / 1e7, radius)
            print(
                json.dumps(
                    {
                        "reached": True,
                        "wall_s": round(wall, 1),
                        "route_wall_s": round(auto, 1),
                        "radius_m": radius,
                        "depth_m": -g.relative_alt / 1000.0,
                    }
                )
            )
            return
    print(
        json.dumps({"reached": False, "closest_m": round(best, 1), "radius_m": radius})
    )
    raise SystemExit(1)


main()
