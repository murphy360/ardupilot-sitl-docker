"""The messages the front seat sends, built from the vehicle.

Each builder takes the vehicle and the connection's ``MAVLink`` object and sends one
message. ``BUILDERS`` names them by MAVLink message name; ``STREAMS`` gives ArduPilot's
stream groups (what ``REQUEST_DATA_STREAM`` selects) as lists of message names.
"""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import ardupilotmega as mav2

from . import captured
from .vehicle import MODE_NAMES, Vehicle

M = mav2
DEG7 = 1e7


def _quaternion(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return cy * cp, -sy * sp, cy * sp, sy * cp


def _velocity(v: Vehicle) -> tuple[float, float, float]:
    return v.hull.velocity_ned()


def heartbeat(v: Vehicle, mav) -> None:
    """HEARTBEAT: a submarine on ArduPilot's dialect."""
    base = M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if v.armed:
        base |= M.MAV_MODE_FLAG_SAFETY_ARMED
    state = M.MAV_STATE_CRITICAL if v.critical else M.MAV_STATE_STANDBY
    if v.armed and not v.critical:
        state = M.MAV_STATE_ACTIVE
    mav.heartbeat_send(
        M.MAV_TYPE_SUBMARINE, M.MAV_AUTOPILOT_ARDUPILOTMEGA, base, v.mode, state, 3
    )


def global_position_int(v: Vehicle, mav) -> None:
    """GLOBAL_POSITION_INT: the estimate (nothing placed until it is valid)."""
    n, e, d = _velocity(v)
    lat, lon = v.latlon
    placed = v.estimate_ok
    mav.global_position_int_send(
        v.boot_ms,
        int(lat * DEG7) if placed else 0,
        int(lon * DEG7) if placed else 0,
        int(v.altitude_msl * 1000),
        int(-v.depth * 1000),
        int(n * 100),
        int(e * 100),
        int(d * 100),
        int(math.degrees(v.hull.heading) % 360 * 100) % 36000,
    )


def attitude(v: Vehicle, mav) -> None:
    """ATTITUDE: roll stays 0."""
    h = v.hull
    mav.attitude_send(v.boot_ms, 0.0, h.pitch, h.heading, 0.0, h.q, h.r)


def sys_status(v: Vehicle, mav) -> None:
    """SYS_STATUS: the sensors (the captured words of the estimator's state) and the
    battery."""
    b = v.battery
    present, enabled, health = v.world.sensor_words()
    mav.sys_status_send(
        present,
        enabled,
        health,
        100,
        int(b.voltage * 1000),
        int(b.current_ma / 10),
        int(b.remaining * 100),
        0,
        0,
        0,
        0,
        0,
        0,
    )


def gps_raw_int(v: Vehicle, mav) -> None:
    """GPS_RAW_INT: the fix (the truth and the simulated GPS's offset), or none."""
    fix, sats, eph = v.world.gps_fix()
    n, e, _ = _velocity(v)
    cog = int(math.degrees(math.atan2(e, n)) % 360 * 100) % 36000
    lat, lon = v.frame.to_ll(*v.world.gps_position(v.truth)) if fix >= 2 else (0.0, 0.0)
    alt = int(v.truth_altitude_msl * 1000) if fix >= 2 else 0
    mav.gps_raw_int_send(
        v.unix_usec,
        fix,
        int(lat * DEG7),
        int(lon * DEG7),
        alt,
        eph,
        150,
        int(math.hypot(n, e) * 100),
        cog,
        sats,
        alt,
        1500,
        2000,
        200,
        0,
        0,
    )


def ekf_status_report(v: Vehicle, mav) -> None:
    """EKF_STATUS_REPORT: the captured flags of the estimator's state, once the
    position estimate is valid."""
    s = captured.state(v.world.mode)
    flags = v.world.ekf_flags(v.estimate_ok)
    mav.ekf_status_report_send(
        flags,
        s["ekf_velocity_variance"],
        s["ekf_pos_horiz_variance"],
        s["ekf_pos_vert_variance"],
        s["ekf_compass_variance"],
        0.0,
        0.0,
    )


def battery_status(v: Vehicle, mav) -> None:
    """BATTERY_STATUS: the one battery."""
    b = v.battery
    pct = int(b.remaining * 100)
    charge = M.MAV_BATTERY_CHARGE_STATE_OK
    if pct < 5:
        charge = M.MAV_BATTERY_CHARGE_STATE_CRITICAL
    elif pct < 10:
        charge = M.MAV_BATTERY_CHARGE_STATE_LOW
    volts = [int(b.voltage * 1000)] + [65535] * 9
    mav.battery_status_send(
        0,
        M.MAV_BATTERY_FUNCTION_ALL,
        M.MAV_BATTERY_TYPE_LIPO,
        32767,
        volts,
        int(b.current_ma / 10),
        int(b.used_mah),
        -1,
        pct,
        b.remaining_s,
        charge,
    )


def home_position(v: Vehicle, mav) -> None:
    """HOME_POSITION: where it started."""
    mav.home_position_send(
        int(v.frame.lat * DEG7),
        int(v.frame.lon * DEG7),
        int(v.home_alt * 1000),
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        0.0,
        int(v.unix_boot * 1e6),
    )


def system_time(v: Vehicle, mav) -> None:
    """SYSTEM_TIME: UTC, and the boot clock in vehicle time."""
    mav.system_time_send(v.unix_usec, v.boot_ms)


def extended_sys_state(v: Vehicle, mav) -> None:
    """EXTENDED_SYS_STATE: nothing to say for a sub."""
    mav.extended_sys_state_send(
        M.MAV_VTOL_STATE_UNDEFINED, M.MAV_LANDED_STATE_UNDEFINED
    )


def sim_state(v: Vehicle, mav) -> None:
    """SIM_STATE: the truth, for the world simulator and the referee."""
    h = v.hull
    lat, lon = v.truth_latlon
    n, e, d = _velocity(v)
    q = _quaternion(h.heading, h.pitch)
    mav.sim_state_send(
        *q,
        0.0,
        h.pitch,
        h.heading,
        0.0,
        0.0,
        0.0,
        0.0,
        h.q,
        h.r,
        lat,
        lon,
        v.truth_altitude_msl,
        0.0,
        0.0,
        n,
        e,
        d,
        int(lat * DEG7),
        int(lon * DEG7),
    )


def mission_current(v: Vehicle, mav) -> None:
    """MISSION_CURRENT: the item AUTO is on."""
    m = v.mission
    mav.mission_current_send(m.current, m.count, m.state, 0)


def fence_status(v: Vehicle, mav) -> None:
    """FENCE_STATUS: whether the fence is breached (ArduSub only logs it)."""
    breached, kind, count, at = v.fence
    mav.fence_status_send(int(breached), count, kind, at, 0)


def _range(v: Vehicle) -> float | None:
    """The rangefinder's reading, m, or None when it is not fitted or reads nothing."""
    if v.params.get("RNGFND1_TYPE") == 0:
        return None
    r = v.world.range_m(v.hull.depth)
    top = v.params.get("RNGFND1_MAX", v.params.get("RNGFND1_MAX_CM", 3000.0) / 100.0)
    return r if 0.0 <= r <= top else None


def distance_sensor(v: Vehicle, mav) -> None:
    """DISTANCE_SENSOR: the downward rangefinder (nothing past its maximum)."""
    r = _range(v)
    if r is None:
        return
    low = int(v.params.get("RNGFND1_MIN", 0.2) * 100)
    high = int(v.params.get("RNGFND1_MAX", 30.0) * 100)
    mav.distance_sensor_send(
        v.boot_ms, low, high, int(r * 100), M.MAV_DISTANCE_SENSOR_UNKNOWN, 0,
        M.MAV_SENSOR_ROTATION_PITCH_270, 255,
    )  # fmt: skip


def rangefinder(v: Vehicle, mav) -> None:
    """RANGEFINDER: the same reading, ArduPilot's own message."""
    r = _range(v)
    if r is not None:
        mav.rangefinder_send(r, 0.0)


def statustext(mav, severity: int, text: str) -> None:
    """STATUSTEXT, cut to the 50 characters one message holds."""
    mav.statustext_send(severity, text[:50].encode())


BUILDERS = {
    "HEARTBEAT": heartbeat,
    "GLOBAL_POSITION_INT": global_position_int,
    "ATTITUDE": attitude,
    "SYS_STATUS": sys_status,
    "GPS_RAW_INT": gps_raw_int,
    "EKF_STATUS_REPORT": ekf_status_report,
    "BATTERY_STATUS": battery_status,
    "HOME_POSITION": home_position,
    "SYSTEM_TIME": system_time,
    "EXTENDED_SYS_STATE": extended_sys_state,
    "SIM_STATE": sim_state,
    "MISSION_CURRENT": mission_current,
    "FENCE_STATUS": fence_status,
    "DISTANCE_SENSOR": distance_sensor,
    "RANGEFINDER": rangefinder,
}
# ArduPilot's stream groups: what REQUEST_DATA_STREAM selects
STREAMS = {
    M.MAV_DATA_STREAM_EXTENDED_STATUS: (
        "SYS_STATUS",
        "GPS_RAW_INT",
        "MISSION_CURRENT",
        "FENCE_STATUS",
    ),
    M.MAV_DATA_STREAM_POSITION: ("GLOBAL_POSITION_INT",),
    M.MAV_DATA_STREAM_EXTRA1: ("ATTITUDE",),
    M.MAV_DATA_STREAM_EXTRA3: (
        "SYSTEM_TIME",
        "BATTERY_STATUS",
        "EKF_STATUS_REPORT",
        "DISTANCE_SENSOR",
        "RANGEFINDER",
    ),
}
STREAMS[M.MAV_DATA_STREAM_ALL] = tuple(n for g in STREAMS.values() for n in g)


def mode_name(number: int) -> str:
    """The mode's name for text."""
    return MODE_NAMES.get(number, str(number))


def message_id(name: str) -> int:
    """The MAVLink id of a message name."""
    return getattr(M, f"MAVLINK_MSG_ID_{name}")


IDS = {message_id(n): n for n in BUILDERS}
