"""The mission: its items, its current item, and the MAVLink mission protocol.

Item 0 is home, which ArduPilot rewrites with the real home when a mission is
uploaded; the route is items 1 and up. The protocol is ``MAV_MISSION_TYPE_MISSION``'s:

* upload: ``MISSION_COUNT``, then ``MISSION_REQUEST_INT`` for each item and
  ``MISSION_ITEM_INT`` (or ``MISSION_ITEM``) back, ended by ``MISSION_ACK``. An item
  that arrives twice (a late request answered twice) is answered ``INVALID_SEQUENCE``
  and the upload stays open, as ArduSub does (the plugin's #22);
* read-back: ``MISSION_REQUEST_LIST`` then ``MISSION_REQUEST_INT``;
* ``MISSION_CLEAR_ALL`` and ``MISSION_SET_CURRENT``.

Fence and rally uploads are answered ``MAV_MISSION_UNSUPPORTED``: the sub kind sends
none. ``MISSION_CURRENT`` and ``MISSION_ITEM_REACHED`` are sent by the vehicle's
events (server.py) and the stream.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from pymavlink.dialects.v20 import ardupilotmega as mav2

M = mav2
MISSION = M.MAV_MISSION_TYPE_MISSION
NAV_WAYPOINT = M.MAV_CMD_NAV_WAYPOINT
NAV_LOITER_UNLIM = M.MAV_CMD_NAV_LOITER_UNLIM
NAV_LOITER_TURNS = M.MAV_CMD_NAV_LOITER_TURNS
DO_CHANGE_SPEED = M.MAV_CMD_DO_CHANGE_SPEED
DO_LAND_START = M.MAV_CMD_DO_LAND_START
NAV_COMMANDS = (NAV_WAYPOINT, NAV_LOITER_UNLIM, NAV_LOITER_TURNS)
FRAME_GLOBAL = M.MAV_FRAME_GLOBAL
INVALID_SEQUENCE = M.MAV_MISSION_INVALID_SEQUENCE
MAX_ITEMS = 700


@dataclass
class Item:
    """One mission item, as MISSION_ITEM_INT carries it."""

    seq: int
    command: int
    frame: int = M.MAV_FRAME_GLOBAL_RELATIVE_ALT
    x: int = 0  # latitude, degrees E7
    y: int = 0  # longitude, degrees E7
    z: float = 0.0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    autocontinue: int = 1

    @property
    def is_nav(self) -> bool:
        """A navigation command (it has a place to reach)."""
        return self.command in NAV_COMMANDS


class Mission:
    """The items, the current one and whether the mission has ended."""

    def __init__(self, home: Item):
        self.home = home
        self.items: list[Item] = [home]
        self.current = 1
        self.complete = False
        self.changed = True  # MISSION_CURRENT is to be sent

    @property
    def count(self) -> int:
        """The items, home included."""
        return len(self.items)

    @property
    def has_commands(self) -> bool:
        """Anything to fly besides home."""
        return len(self.items) > 1

    def replace(self, items: list[Item]) -> None:
        """A finished upload: home is rewritten and the mission starts again."""
        self.items = [self.home] + [replace(i, seq=n) for n, i in enumerate(items) if n]
        self.current = 1 if len(self.items) > 1 else 0
        self.complete = False
        self.changed = True

    def clear(self) -> None:
        """MISSION_CLEAR_ALL."""
        self.items = [self.home]
        self.current, self.complete, self.changed = 0, False, True

    def set_current(self, seq: int) -> bool:
        """Make ``seq`` the current item; False when there is no such item."""
        if not 0 <= seq < self.count:
            return False
        self.current, self.complete, self.changed = seq, False, True
        return True

    def item(self, seq: int) -> Item | None:
        """The item at ``seq``, or None."""
        return self.items[seq] if 0 <= seq < self.count else None

    @property
    def state(self) -> int:
        """MISSION_STATE: 1 no mission, 2 not started or active, 4 complete."""
        if not self.has_commands:
            return 1
        return 4 if self.complete else 3


def item_of(msg) -> Item:
    """An Item from MISSION_ITEM_INT or MISSION_ITEM (floats: degrees to E7)."""
    x, y = msg.x, msg.y
    if msg.get_type() == "MISSION_ITEM":
        x, y = int(round(x * 1e7)), int(round(y * 1e7))
    return Item(
        msg.seq,
        msg.command,
        msg.frame,
        int(x),
        int(y),
        float(msg.z),
        float(msg.param1),
        float(msg.param2),
        float(msg.param3),
        float(msg.param4),
        int(msg.autocontinue),
    )


class Upload:
    """The one upload in progress."""

    def __init__(self, count: int, client: tuple[int, int]):
        self.count, self.client = count, client
        self.items: list[Item] = []


class Protocol:
    """The mission protocol of one vehicle (state: the upload in progress)."""

    def __init__(self):
        self.upload: Upload | None = None

    # -- upload ---------------------------------------------------------------------

    def count(self, server, conn, msg) -> None:
        """MISSION_COUNT: start an upload (or refuse a fence or rally one)."""
        sysid, comp = conn.client
        if msg.mission_type != MISSION:
            conn.mav.mission_ack_send(
                sysid, comp, M.MAV_MISSION_UNSUPPORTED, msg.mission_type
            )
            return
        if msg.count > MAX_ITEMS:
            conn.mav.mission_ack_send(sysid, comp, M.MAV_MISSION_NO_SPACE, MISSION)
            return
        if msg.count == 0:
            server.vehicle.mission.clear()
            self.upload = None
            conn.mav.mission_ack_send(sysid, comp, M.MAV_MISSION_ACCEPTED, MISSION)
            return
        self.upload = Upload(msg.count, conn.client)
        conn.mav.mission_request_int_send(sysid, comp, 0, MISSION)

    def item(self, server, conn, msg) -> None:
        """MISSION_ITEM_INT / MISSION_ITEM: the next item of the upload."""
        up, (sysid, comp) = self.upload, conn.client
        if up is None:
            return
        if msg.seq != len(
            up.items
        ):  # a duplicate (or one out of turn): the upload stays
            conn.mav.mission_ack_send(sysid, comp, INVALID_SEQUENCE, MISSION)
            return
        up.items.append(item_of(msg))
        if len(up.items) < up.count:
            conn.mav.mission_request_int_send(sysid, comp, len(up.items), MISSION)
            return
        self.upload = None
        server.vehicle.new_mission(up.items)
        conn.mav.mission_ack_send(sysid, comp, M.MAV_MISSION_ACCEPTED, MISSION)

    # -- download -------------------------------------------------------------------

    def request_list(self, server, conn, msg) -> None:
        """MISSION_REQUEST_LIST: how many items there are."""
        if msg.mission_type != MISSION:
            conn.mav.mission_count_send(*conn.client, 0, msg.mission_type)
            return
        conn.mav.mission_count_send(*conn.client, server.vehicle.mission.count, MISSION)

    def request(self, server, conn, msg) -> None:
        """MISSION_REQUEST_INT / MISSION_REQUEST: one item."""
        sysid, comp = conn.client
        item = server.vehicle.mission.item(msg.seq)
        if msg.mission_type != MISSION or item is None:
            conn.mav.mission_ack_send(
                sysid, comp, M.MAV_MISSION_INVALID_SEQUENCE, MISSION
            )
            return
        cur = int(server.vehicle.mission.current == item.seq)
        args = (sysid, comp, item.seq, item.frame, item.command, cur, item.autocontinue)
        params = (item.param1, item.param2, item.param3, item.param4)
        if msg.get_type() == "MISSION_REQUEST":
            conn.mav.mission_item_send(
                *args, *params, item.x / 1e7, item.y / 1e7, item.z, MISSION
            )
        else:
            conn.mav.mission_item_int_send(
                *args, *params, item.x, item.y, item.z, MISSION
            )

    def clear(self, server, conn, msg) -> None:
        """MISSION_CLEAR_ALL."""
        sysid, comp = conn.client
        if msg.mission_type == MISSION:
            self.upload = None
            server.vehicle.mission.clear()
            server.vehicle.mission_cleared()
        conn.mav.mission_ack_send(sysid, comp, M.MAV_MISSION_ACCEPTED, msg.mission_type)

    def set_current(self, server, conn, msg) -> None:
        """MISSION_SET_CURRENT."""
        server.vehicle.set_current_item(msg.seq)

    def ack(self, server, conn, msg) -> None:
        """MISSION_ACK from a client closing a download: nothing to do."""
