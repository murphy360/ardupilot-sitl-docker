"""What the front seat does with each message it is sent.

``handle(server, conn, msg)`` is the one entry. Commands (``COMMAND_LONG`` and
``COMMAND_INT``) are each answered with a ``COMMAND_ACK``. Messages for another system
are ignored. The mission protocol, the failsafes and the simulated world are Part C2;
here a mission upload is refused as unsupported and a read-back is an empty mission.
"""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import ardupilotmega as mav2

from .messages import BUILDERS, IDS, statustext

M = mav2
ACCEPTED, DENIED = M.MAV_RESULT_ACCEPTED, M.MAV_RESULT_DENIED
UNSUPPORTED, FAILED = M.MAV_RESULT_UNSUPPORTED, M.MAV_RESULT_FAILED
CMD_ARM = M.MAV_CMD_COMPONENT_ARM_DISARM
CMD_MODE = M.MAV_CMD_DO_SET_MODE
CMD_SPEED = M.MAV_CMD_DO_CHANGE_SPEED
CMD_REQUEST = M.MAV_CMD_REQUEST_MESSAGE
CMD_INTERVAL = M.MAV_CMD_SET_MESSAGE_INTERVAL
CMD_REBOOT = M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
FORCE_ARM = 21196.0  # param2 of ARM_DISARM that skips the pre-arm checks
REAL32 = M.MAV_PARAM_TYPE_REAL32


def _ack(conn, command: int, result: int) -> None:
    sysid, comp = conn.client
    conn.mav.command_ack_send(command, result, 0, 0, sysid, comp)


def _for_us(server, msg) -> bool:
    target = getattr(msg, "target_system", 0)
    comp = getattr(msg, "target_component", 0)
    return target in (0, server.cfg.sysid) and comp in (0, 1)


def _params_of(msg) -> list[float]:
    names = ["param1", "param2", "param3", "param4"]
    return [float(getattr(msg, n)) for n in names]


# -- commands -------------------------------------------------------------------------


def _arm(server, conn, p) -> int:
    ok, why = server.vehicle.arm(p[0] >= 0.5, force=p[1] == FORCE_ARM)
    if not ok:
        statustext(conn.mav, 4, why)
        return FAILED
    return ACCEPTED


def _mode(server, conn, p) -> int:
    if not int(p[0]) & M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return DENIED
    ok, _ = server.vehicle.set_mode(int(p[1]))
    return ACCEPTED if ok else UNSUPPORTED


def _speed(server, conn, p) -> int:
    if p[1] > 0:
        server.vehicle.change_speed(p[1])
    return ACCEPTED


def _request(server, conn, p) -> int:
    return ACCEPTED if int(p[0]) in IDS else UNSUPPORTED


def _interval(server, conn, p) -> int:
    name = IDS.get(int(p[0]))
    if name is None:
        return UNSUPPORTED
    now = server.vehicle.t
    if p[1] < 0:
        conn.set_interval(name, None, now)
    elif p[1] == 0:  # the default rate: none, except the heartbeat's
        conn.set_interval(name, 1.0 if name == "HEARTBEAT" else None, now)
    else:
        conn.set_interval(name, p[1] / 1e6, now)
    return ACCEPTED


def _reboot(server, conn, p) -> int:
    if int(p[0]) != 1:
        return UNSUPPORTED
    if server.vehicle.armed:
        statustext(conn.mav, 4, "PreArm: Disarm to reboot")
        return FAILED
    server.reboot_requested = True
    return ACCEPTED


COMMANDS = {
    CMD_ARM: _arm,
    CMD_MODE: _mode,
    CMD_SPEED: _speed,
    CMD_REQUEST: _request,
    CMD_INTERVAL: _interval,
    CMD_REBOOT: _reboot,
}


def _command(server, conn, msg) -> None:
    if not _for_us(server, msg):
        return
    fn = COMMANDS.get(msg.command)
    result = UNSUPPORTED if fn is None else fn(server, conn, _params_of(msg))
    _ack(conn, msg.command, result)
    if msg.command == CMD_REQUEST and result == ACCEPTED:  # the answer follows the ack
        BUILDERS[IDS[int(msg.param1)]](server.vehicle, conn.mav)


# -- the rest -------------------------------------------------------------------------


def _set_mode(server, conn, msg) -> None:
    if not _for_us(server, msg) or not msg.base_mode & 1:
        return
    ok, _ = server.vehicle.set_mode(int(msg.custom_mode))
    if not ok:
        statustext(conn.mav, 4, "Mode change failed")


def _manual_control(server, conn, msg) -> None:
    if msg.target in (0, server.cfg.sysid):
        server.vehicle.manual_control(msg.x, msg.z, msg.r)


def _position_target(server, conn, msg) -> None:
    if not _for_us(server, msg):
        return
    v = server.vehicle
    ok, why = v.position_target(
        msg.coordinate_frame,
        msg.type_mask,
        msg.lat_int / 1e7,
        msg.lon_int / 1e7,
        msg.alt,
        (msg.vx, msg.vy),
    )
    if not ok and why.startswith("frame"):
        statustext(conn.mav, 4, f"Target {why}")


def _timesync(server, conn, msg) -> None:
    if msg.tc1 == 0:
        # tc1 0 means a request: an answer is never 0
        conn.mav.timesync_send(max(1, int(server.vehicle.t * 1e9)), msg.ts1)


# -- parameters -----------------------------------------------------------------------


def _send_param(server, conn, name: str) -> None:
    p = server.params
    conn.mav.param_value_send(
        name.encode(), float(p.get(name)), REAL32, len(p.values), p.index(name)
    )


def _param_name(msg) -> str:
    raw = msg.param_id
    text = raw.decode(errors="ignore") if isinstance(raw, (bytes, bytearray)) else raw
    return text.rstrip("\x00")


def _param_read(server, conn, msg) -> None:
    if not _for_us(server, msg):
        return
    p = server.params
    name = _param_name(msg) if msg.param_index < 0 else None
    if name is None and 0 <= msg.param_index < len(p.values):
        name = p.names()[msg.param_index]
    if name in p.values:
        _send_param(server, conn, name)


def _param_list(server, conn, msg) -> None:
    if _for_us(server, msg):
        for name in server.params.names():
            _send_param(server, conn, name)


def _param_set(server, conn, msg) -> None:
    if not _for_us(server, msg):
        return
    name = _param_name(msg)
    value = float(msg.param_value)
    if math.isfinite(value):
        server.params.set(name, value)
    if name in server.params.values:
        _send_param(server, conn, name)


# -- streams and the mission protocol (a mission is Part C2) ----------------------------


def _data_stream(server, conn, msg) -> None:
    if _for_us(server, msg):
        rate = msg.req_message_rate if msg.start_stop else 0
        conn.set_stream(msg.req_stream_id, float(rate), server.vehicle.t)


def _mission_list(server, conn, msg) -> None:
    if _for_us(server, msg):
        conn.mav.mission_count_send(conn.client[0], conn.client[1], 0, msg.mission_type)


def _mission_refuse(server, conn, msg) -> None:
    if _for_us(server, msg):
        kind = getattr(msg, "mission_type", 0)
        conn.mav.mission_ack_send(
            conn.client[0], conn.client[1], M.MAV_MISSION_UNSUPPORTED, kind
        )


def _mission_clear(server, conn, msg) -> None:
    if _for_us(server, msg):
        kind = getattr(msg, "mission_type", 0)
        conn.mav.mission_ack_send(
            conn.client[0], conn.client[1], M.MAV_MISSION_ACCEPTED, kind
        )


HANDLERS = {
    "COMMAND_LONG": _command,
    "COMMAND_INT": _command,
    "SET_MODE": _set_mode,
    "MANUAL_CONTROL": _manual_control,
    "SET_POSITION_TARGET_GLOBAL_INT": _position_target,
    "TIMESYNC": _timesync,
    "PARAM_REQUEST_READ": _param_read,
    "PARAM_REQUEST_LIST": _param_list,
    "PARAM_SET": _param_set,
    "REQUEST_DATA_STREAM": _data_stream,
    "MISSION_REQUEST_LIST": _mission_list,
    "MISSION_COUNT": _mission_refuse,
    "MISSION_CLEAR_ALL": _mission_clear,
}


def handle(server, conn, msg) -> None:
    """Act on one message from ``conn``."""
    fn = HANDLERS.get(msg.get_type())
    if fn is not None:
        fn(server, conn, msg)
