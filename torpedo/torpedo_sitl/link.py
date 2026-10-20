"""One MAVLink connection: its socket, its parser and its own stream rates.

Every connection has its own rates (SERIAL0, 1 and 2 are asked for different things),
and a connection may close and come back at any time. A connection sends a HEARTBEAT
once a second of vehicle time from the moment it opens; every other message only when
asked for, by ``REQUEST_DATA_STREAM``, ``SET_MESSAGE_INTERVAL`` or ``REQUEST_MESSAGE``.
"""

from __future__ import annotations

import socket

from pymavlink.dialects.v20 import ardupilotmega as mav2

from .messages import BUILDERS, STREAMS
from .vehicle import Vehicle

HEARTBEAT_S = 1.0


class Connection:
    """A client of one of the three ports."""

    def __init__(self, sock: socket.socket, sysid: int, port_name: str):
        self.sock = sock
        sock.setblocking(False)
        self.port_name = port_name
        self.out = bytearray()
        self.mav = mav2.MAVLink(self, srcSystem=sysid, srcComponent=1)
        self.mav.robust_parsing = True
        self.client = (0, 0)  # the last system and component heard from
        self.intervals: dict[str, float] = {"HEARTBEAT": HEARTBEAT_S}
        self.next_due: dict[str, float] = {"HEARTBEAT": 0.0}
        self.closed = False
        self.greeted = False

    # the file object the MAVLink object writes to
    def write(self, data: bytes) -> None:
        """Queue bytes for the socket."""
        self.out += data

    def read(self) -> list:
        """The messages that arrived; closes the connection when the peer has."""
        try:
            data = self.sock.recv(65536)
        except BlockingIOError:
            return []
        except OSError:
            data = b""
        if not data:
            self.closed = True
            return []
        out = []
        for msg in self.mav.parse_buffer(data) or []:
            if msg.get_type() != "BAD_DATA":
                self.client = (msg.get_srcSystem(), msg.get_srcComponent())
                out.append(msg)
        return out

    def flush(self) -> None:
        """Send what is queued; closes the connection when the socket has failed."""
        while self.out and not self.closed:
            try:
                sent = self.sock.send(self.out)
            except BlockingIOError:
                return
            except OSError:
                self.closed = True
                return
            del self.out[:sent]

    def close(self) -> None:
        """Close the socket."""
        self.closed = True
        try:
            self.sock.close()
        except OSError:
            pass

    # -- rates ------------------------------------------------------------------------

    def set_interval(self, name: str, seconds: float | None, now: float) -> None:
        """Send ``name`` every ``seconds`` of vehicle time; None stops it."""
        if seconds is None or seconds <= 0:
            self.intervals.pop(name, None)
            self.next_due.pop(name, None)
            return
        self.intervals[name] = seconds
        self.next_due.setdefault(name, now)

    def set_stream(self, stream: int, hz: float, now: float) -> None:
        """``REQUEST_DATA_STREAM``: a group at ``hz`` (0 stops it)."""
        for name in STREAMS.get(stream, ()):
            self.set_interval(name, 1.0 / hz if hz > 0 else None, now)

    def emit(self, vehicle: Vehicle) -> None:
        """Send every message that is due at the vehicle's time."""
        now = vehicle.t
        for name, every in list(self.intervals.items()):
            if now >= self.next_due[name]:
                BUILDERS[name](vehicle, self.mav)
                nxt = self.next_due[name] + every
                self.next_due[name] = nxt if nxt > now - every else now
