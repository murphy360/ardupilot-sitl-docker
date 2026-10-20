"""Helpers for the link tests: a front seat in a thread and a pymavlink client."""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

from pymavlink import mavutil

from torpedo_sitl.geo import Frame
from torpedo_sitl.model import load
from torpedo_sitl.server import Config, Server

M = mavutil.mavlink
PLUGIN_NAMES = """
SYSID_THISMAV MAV_SYSID ARMING_CHECK ARMING_SKIPCHK WP_RADIUS_M WPNAV_RADIUS
WPNAV_RADIUS_M WP_RADIUS FS_GCS_ENABLE FS_GCS_TIMEOUT SYSID_MYGCS MAV_GCS_SYSID
MAV_GCS_SYSID_HI SYSID_ENFORCE MAV_OPTIONS PDEAD_ACTION PDEAD_HOLD_S SCR_ENABLE
FENCE_ENABLE FENCE_TYPE FENCE_ACTION FENCE_ALT_MIN FENCE_RADIUS FENCE_MARGIN
BATT_MONITOR BATT_CAPACITY BATT_LOW_MAH BATT_LOW_VOLT BATT_FS_LOW_ACT BATT_CRT_MAH
BATT_CRT_VOLT BATT_FS_CRT_ACT FS_LEAK_ENABLE LEAK1_PIN FS_PILOT_INPUT FS_PILOT_TIMEOUT
BARO_EXT_BUS RNGFND1_TYPE RNGFND1_MAX_CM RNGFND1_MIN_CM RNGFND1_MAX RNGFND1_MIN
EK3_SRC1_VELXY WPNAV_SPEED_UP PILOT_SPEED_UP EK3_POS_I_GATE EK3_HGT_I_GATE
EK3_GLITCH_RAD EK3_ALT_M_NSE SIM_OPOS_LAT SIM_OPOS_LNG SIM_OPOS_ALT SIM_OPOS_HDG
TPDO_MODEL TPDO_LOS_LA TPDO_TURN_K TPDO_LOIT_MIN
""".split()
MODELS = Path(__file__).resolve().parent / "models"
HOME = (36.6, -121.9)
WAIT_S = 20.0


class FrontSeat:
    """A running front seat: its server, its ports and its thread."""

    def __init__(self, tmp_path, hull="torpedo-test-small", **cfg):
        self.tmp_path = tmp_path
        self.model = load(hull if isinstance(hull, Path) else MODELS / f"{hull}.json")
        values = dict(
            speedup=30.0,
            lat=HOME[0],
            lon=HOME[1],
            alt=0.0,
            heading=90.0,
            sysid=7,
            any_port=True,
            param_dir=str(tmp_path),
            host="127.0.0.1",
            reboot_delay_s=0.2,
        )
        values.update(cfg)
        self.cfg = Config(**values)
        self.server = Server(self.model, self.cfg)
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._serve)
        self.thread.start()
        self.frame = Frame(*HOME)
        self.clients = []

    def _serve(self):
        try:
            self.server.serve(self.stop)
        except Exception as exc:  # shown by close(): a dead server is a failed test
            self.error = exc
            raise

    @property
    def ports(self):
        return self.server.bound

    def client(self, port_name="SERIAL0", system=250):
        c = Client(self.ports[port_name], system)
        self.clients.append(c)
        return c

    def close(self):
        for c in self.clients:
            c.close()
        self.stop.set()
        self.thread.join(10)
        assert self.error is None, f"the front seat died: {self.error!r}"
        assert not self.server.errors, self.server.errors


class Client:
    """A ground station: sends, and reads the messages that come back."""

    def __init__(self, port, system=250):
        self.m = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{port}", source_system=system, source_component=190
        )
        self.texts = []  # what was said before the first heartbeat
        self.hb = None
        end = time.time() + WAIT_S
        while self.hb is None and time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                self.texts.append(msg.text)
            elif msg is not None and msg.get_type() == "HEARTBEAT":
                self.hb = msg
        assert self.hb is not None, "no heartbeat"
        self.target = self.hb.get_srcSystem()

    def close(self):
        self.m.close()

    def wait(self, kind, timeout=WAIT_S, cond=None):
        """The next ``kind`` message (that passes ``cond``)."""
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type=kind, blocking=True, timeout=0.5)
            if msg is not None and (cond is None or cond(msg)):
                return msg
        raise AssertionError(f"no {kind} within {timeout} s")

    def wait_any(self, kinds, timeout=WAIT_S):
        """The next message of any of ``kinds``."""
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type=list(kinds), blocking=True, timeout=0.5)
            if msg is not None:
                return msg
        raise AssertionError(f"none of {kinds} within {timeout} s")

    def fresh(self, kind, settle=0.3):
        """The latest ``kind`` message: what has queued is read first."""
        end = time.time() + settle
        while time.time() < end:
            self.m.recv_match(blocking=True, timeout=0.05)
        return self.m.messages[kind]

    def command(self, command, *params, timeout=WAIT_S):
        """Send a COMMAND_LONG and return its COMMAND_ACK's result."""
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.target, 1, command, 0, *p)
        ack = self.wait("COMMAND_ACK", timeout, lambda a: a.command == command)
        return ack.result

    def interval(self, name, hz):
        """Ask for ``name`` at ``hz`` (0 or less stops it)."""
        msg_id = getattr(M, f"MAVLINK_MSG_ID_{name}")
        us = 1e6 / hz if hz > 0 else -1
        return self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, us)

    def set_mode(self, number):
        self.m.mav.set_mode_send(
            self.target, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number
        )

    def beat(self):
        """A heartbeat from this ground station (the GCS failsafe's clock)."""
        self.m.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 4)

    def heartbeat(self, cond=None):
        return self.wait("HEARTBEAT", cond=cond)

    def mode_is(self, number, timeout=WAIT_S):
        return self.heartbeat(lambda h: h.custom_mode == number)

    def param(self, name):
        self.m.mav.param_request_read_send(self.target, 1, name.encode(), -1)
        return self.wait("PARAM_VALUE", cond=lambda p: p.param_id == name).param_value

    def position_target(self, frame, mask, lat, lon, alt, vx=0.0, vy=0.0):
        self.m.mav.set_position_target_global_int_send(
            0,
            self.target,
            1,
            frame,
            mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            vx,
            vy,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def ready(client):
    """Ask for the position stream and wait until the estimate is placed."""
    client.interval("GLOBAL_POSITION_INT", 10)
    return client.wait("GLOBAL_POSITION_INT", cond=lambda g: g.lat != 0)


def place(client, frame):
    """The latest GLOBAL_POSITION_INT as (north, east, depth) from home."""
    g = client.fresh("GLOBAL_POSITION_INT")
    n, e = frame.to_ne(g.lat / 1e7, g.lon / 1e7)
    return n, e, -g.relative_alt / 1000.0


def model_with_id(tmp_path, source, new_id):
    """A copy of a fixture hull under another id."""
    doc = json.loads((MODELS / f"{source}.json").read_text())
    doc["id"] = new_id
    path = tmp_path / f"{new_id}.json"
    path.write_text(json.dumps(doc))
    return path


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def make_vehicle(model, **params):
    """A vehicle on its own, stepped by hand (no sockets): returns (vehicle, store)."""
    from torpedo_sitl.params import ParamStore, defaults
    from torpedo_sitl.vehicle import Vehicle

    store = ParamStore(defaults(model, 1, HOME[0], HOME[1], 0.0, 90.0, 10000.0))
    store.load(list(params.items()))
    return Vehicle(model, store, 1_700_000_000.0), store


def run(vehicle, seconds, dt=0.05):
    """Step a vehicle ``seconds`` of vehicle time."""
    for _ in range(round(seconds / dt)):
        vehicle.step(dt)


def waypoint(vehicle, seq, north, east, depth=6.0, radius=0.0, command=16, **p):
    """A mission item ``north`` m north and ``east`` m east of home at ``depth``."""
    from torpedo_sitl.mission import Item

    lat, lon = vehicle.frame.to_ll(north, east)
    return Item(
        seq, command, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, int(lat * 1e7),
        int(lon * 1e7), -depth, p.get("param1", 0.0), radius or p.get("param2", 0.0),
        p.get("param3", 0.0), p.get("param4", 0.0),
    )  # fmt: skip


def plain_item(seq, command, **p):
    """An item with no place (a DO command or a marker)."""
    from torpedo_sitl.mission import Item

    return Item(seq, command, M.MAV_FRAME_GLOBAL_RELATIVE_ALT, 0, 0, 0.0, **p)


def home_item(vehicle):
    """Item 0, as a client would send it."""
    return waypoint(vehicle, 0, 0.0, 0.0, 0.0)


def send_item(client, it, kind="MISSION_ITEM_INT"):
    """One mission item as the client sends it."""
    args = (
        client.target, 1, it.seq, it.frame, it.command, 0, it.autocontinue,
        it.param1, it.param2, it.param3, it.param4,
    )  # fmt: skip
    if kind == "MISSION_ITEM":
        client.m.mav.mission_item_send(*args, it.x / 1e7, it.y / 1e7, it.z, 0)
    else:
        client.m.mav.mission_item_int_send(*args, it.x, it.y, it.z, 0)


def upload_mission(client, items, duplicate=None, kind="MISSION_ITEM_INT"):
    """Upload ``items`` (item 0 first) and return the final MISSION_ACK. ``duplicate``:
    a seq whose item is sent twice, the way a late request is answered twice."""
    client.m.mav.mission_count_send(client.target, 1, len(items), 0)
    while True:
        msg = client.wait_any(("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"))
        if msg.get_type() == "MISSION_ACK":
            if msg.type == M.MAV_MISSION_INVALID_SEQUENCE:
                continue  # a duplicate's refusal: the upload stays open
            return msg
        send_item(client, items[msg.seq], kind)
        if duplicate == msg.seq:
            send_item(client, items[msg.seq], kind)
            duplicate = None


def download_mission(client):
    """The mission the vehicle holds, as a list of MISSION_ITEM_INT messages."""
    client.m.mav.mission_request_list_send(client.target, 1, 0)
    count = client.wait("MISSION_COUNT").count
    out = []
    for seq in range(count):
        client.m.mav.mission_request_int_send(client.target, 1, seq, 0)
        out.append(client.wait("MISSION_ITEM_INT", cond=lambda m, s=seq: m.seq == s))
    return out
