"""Check that a motorboat holding a point stays on it: the hull damps sideslip.

Runs inside a rover SITL container started with MODEL=motorboat (it uses the
image's pymavlink) and talks to the simulator on tcp:127.0.0.1:5760, so it must
be the only client on that port.

usage: hull_sideslip.py [--minutes 15]

GUIDED, armed, a point 40 m to the north-east sent again every 15 s of simulated
time. ArduPilot's stock motorboat drags only along its hull, so each turn leaves
a sideways velocity that nothing damps: on station it grows to about 1 m/s in 15
simulated minutes. patches/0001-motorboat-hull-drag.patch adds lateral hull drag
(about 0.1 m/s). The sideslip is read from the autopilot's own estimate:
GLOBAL_POSITION_INT's velocity across ATTITUDE's heading. Fails above 0.3 m/s.
Prints one JSON line with the measurements; exits non-zero on failure.
(murphy360/Proteus#864, #2252; Proteus CI runs the same probe,
tools/sitl_hull_probe.py.)
"""
import argparse
import json
import math
import time

from pymavlink import mavutil

URL = "tcp:127.0.0.1:5760"
M = mavutil.mavlink
LIMIT = 0.3  # m/s
RESEND_S = 15.0
HOLD_M = 40.0
SPEED = 2.0
GUIDED = 15
POSITION_ONLY = 0b110111111000


def sideslip(vn, ve, yaw):
    return -vn * math.sin(yaw) + ve * math.cos(yaw)


def offset(lat, lon, dist, bearing):
    b = math.radians(bearing)
    return (
        lat + dist * math.cos(b) / 111_320.0,
        lon + dist * math.sin(b) / (111_320.0 * math.cos(math.radians(lat))),
    )


def distance(a, b):
    dn = (a[0] - b[0]) * 111_320.0
    de = (a[1] - b[1]) * 111_320.0 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def command(m, cmd, *params):
    p = list(params) + [0.0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, 1, cmd, 0, *p)


def drain(m):
    while m.recv_match(blocking=False) is not None:
        pass


def position(m, timeout=120.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is not None and msg.lat != 0:
            return msg
    raise SystemExit("no position from the autopilot")


def arm(m, timeout=300.0):
    end = time.time() + timeout
    while time.time() < end:
        m.mav.heartbeat_send(6, 8, 0, 0, 0)
        m.set_mode(GUIDED)
        command(m, M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        drain(m)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and hb.custom_mode == GUIDED and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
            return
        time.sleep(2)
    raise SystemExit("the boat would not arm in GUIDED")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=15.0)
    a = ap.parse_args()
    m = mavutil.mavlink_connection(URL, source_system=255)
    m.wait_heartbeat(timeout=120)
    for msg_id in (M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, M.MAVLINK_MSG_ID_ATTITUDE):
        command(m, M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 2e5)
    start = position(m)
    point = offset(start.lat / 1e7, start.lon / 1e7, HOLD_M, 45.0)
    arm(m)
    drain(m)
    t0 = position(m).time_boot_ms / 1000.0
    yaw, sent, beat, slips, offs = None, -RESEND_S, 0.0, [], []
    while True:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == "ATTITUDE":
            yaw = msg.yaw
            continue
        t = msg.time_boot_ms / 1000.0 - t0
        if t - sent >= RESEND_S:
            m.mav.set_position_target_global_int_send(
                0, m.target_system, 1, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                POSITION_ONLY, int(point[0] * 1e7), int(point[1] * 1e7), 0, *([0] * 8))
            command(m, M.MAV_CMD_DO_CHANGE_SPEED, 1, SPEED, -1)
            sent = t
        if time.time() - beat >= 1.0:
            m.mav.heartbeat_send(6, 8, 0, 0, 0)
            beat = time.time()
        if yaw is not None:
            slips.append(abs(sideslip(msg.vx / 100.0, msg.vy / 100.0, yaw)))
            offs.append(distance((msg.lat / 1e7, msg.lon / 1e7), point))
        if t >= a.minutes * 60.0:
            break
    worst = max(slips, default=0.0)
    ok = bool(slips) and worst <= LIMIT
    print(json.dumps({"max_sideslip": round(worst, 3), "end_off_m": round(offs[-1], 1) if offs else None,
                      "samples": len(slips), "limit": LIMIT, "ok": ok}), flush=True)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
