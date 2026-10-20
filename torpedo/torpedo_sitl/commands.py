"""What the front seat does with each message it is sent.

``handle(server, conn, msg)`` is the one entry. Commands (``COMMAND_LONG`` and
``COMMAND_INT``) are each answered with a ``COMMAND_ACK``. Messages for another system
are ignored. The mission protocol (missions, AUTO, the failsafes) live in ``mission.py``, ``auto.py`` and
``failsafe.py``; the messages of the mission protocol go to ``mission.Protocol``.
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
CMD_CURRENT = M.MAV_CMD_DO_SET_MISSION_CURRENT
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
    ok, why = server.vehicle.set_mode(int(p[1]))
    if ok:
        return ACCEPTED
    return UNSUPPORTED if why == "unsupported" else FAILED


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


def _current(server, conn, p) -> int:
    return ACCEPTED if server.vehicle.set_current_item(int(p[0])) else FAILED


COMMANDS = {
    CMD_CURRENT: _current,
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
    ok, why = server.vehicle.set_mode(int(msg.custom_mode))
    if not ok and why == "unsupported":
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


def _heartbeat(server, conn, msg) -> None:
    server.vehicle.heartbeat(msg.get_srcSystem())


def _mission(name: str):
    """A handler that gives the message to ``mission.Protocol.<name>`` if it is ours."""

    def handle(server, conn, msg) -> None:
        if _for_us(server, msg):
            getattr(server.protocol, name)(server, conn, msg)

    return handle


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
    "HEARTBEAT": _heartbeat,
    "MISSION_COUNT": _mission("count"),
    "MISSION_ITEM_INT": _mission("item"),
    "MISSION_ITEM": _mission("item"),
    "MISSION_REQUEST_LIST": _mission("request_list"),
    "MISSION_REQUEST_INT": _mission("request"),
    "MISSION_REQUEST": _mission("request"),
    "MISSION_CLEAR_ALL": _mission("clear"),
    "MISSION_SET_CURRENT": _mission("set_current"),
    "MISSION_ACK": _mission("ack"),
}


def handle(server, conn, msg) -> None:
    """Act on one message from ``conn``."""
    fn = HANDLERS.get(msg.get_type())
    if fn is not None:
        fn(server, conn, msg)
