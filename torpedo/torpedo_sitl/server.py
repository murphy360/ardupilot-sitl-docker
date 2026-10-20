"""The front seat as a process: three MAVLink ports, one vehicle and its clock.

Ports (TCP, MAVLink 2) are ``5760 + 10 * INSTANCE`` (SERIAL0, Proteus), ``+2``
(SERIAL1, QGroundControl) and ``+3`` (SERIAL2, the referee and the world simulator).
They open at once: unlike the SITL, nothing waits for SERIAL0. Vehicle time runs
``SPEEDUP`` times wall time. Behind it (a loaded host) the front seat steps as fast as
it can and falls behind, as the SITL does, without trying to catch up later.
``PREFLIGHT_REBOOT_SHUTDOWN`` closes every link and starts again at ``SIM_OPOS_*`` with
the saved parameters.
"""

from __future__ import annotations

import selectors
import socket
import sys
import threading
import traceback
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import commands
from .link import Connection
from .messages import BUILDERS, statustext
from .mission import Protocol
from .model import Model
from .params import ParamStore, defaults, parse_lines
from .vehicle import Vehicle

DT = 0.05  # s of vehicle time per step
WORK_S = 0.05  # wall s of stepping at most between two looks at the sockets
LAG_S = 0.5  # wall s of lag the clock puts up with before it falls behind for good
PORT_BASE = 5760
BANNER = "Torpedo front seat 0.1, hull {}"


@dataclass
class Config:
    """How to run: the SITL images' environment names, in lower case."""

    instance: int = 0
    sysid: int = 1
    speedup: float = 1.0
    lat: float = 42.3898
    lon: float = -71.1476
    alt: float = 0.0
    heading: float = 270.0
    batt_mah: float = 10000.0
    sitl_params: str = ""
    param_files: list[str] = field(default_factory=list)
    host: str = "0.0.0.0"
    base_port: int | None = None  # the SERIAL0 port; 5760 + 10 * instance when None
    any_port: bool = False  # let the system choose all three ports (tests)
    param_dir: str = "/tmp"
    reboot_delay_s: float = 1.0

    @property
    def ports(self) -> dict[str, int]:
        """The three ports by name."""
        if self.any_port:
            return {"SERIAL0": 0, "SERIAL1": 0, "SERIAL2": 0}
        base = self.base_port
        if base is None:
            base = PORT_BASE + 10 * self.instance
        return {"SERIAL0": base, "SERIAL1": base + 2, "SERIAL2": base + 3}


class Server:
    """One front seat."""

    def __init__(self, model: Model, cfg: Config):
        self.model, self.cfg = model, cfg
        path = Path(cfg.param_dir) / f"torpedo-{cfg.instance}.parm"
        initial = defaults(
            model, cfg.sysid, cfg.lat, cfg.lon, cfg.alt, cfg.heading, cfg.batt_mah
        )
        self.params = ParamStore(initial, path)
        self.params.load(parse_lines(cfg.sitl_params))
        for name in cfg.param_files:
            self.params.load(parse_lines(Path(name).read_text()))
        self.sel = selectors.DefaultSelector()
        self.conns: list[Connection] = []
        self.reboot_requested = False
        self.reboot_until: float | None = None
        self.errors: list[str] = []  # a message that failed to be handled
        self.bound: dict[str, int] = {}
        self.protocol = Protocol()
        self.vehicle: Vehicle | None = None
        self.vehicle = self._boot()
        self._open()

    def _boot(self) -> Vehicle:
        """Start a vehicle. The mission survives a reboot, as ArduPilot's does."""
        self.boot_wall = time.monotonic()
        kept = self.vehicle.mission.items if self.vehicle is not None else []
        self.protocol = Protocol()
        vehicle = Vehicle(self.model, self.params, time.time())
        if len(kept) > 1:
            vehicle.mission.replace(kept)
        return vehicle

    def _open(self) -> None:
        for name, port in self.cfg.ports.items():
            s = socket.socket()
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.cfg.host, port))
            s.listen(8)
            s.setblocking(False)
            self.sel.register(s, selectors.EVENT_READ, name)
            self.bound[name] = s.getsockname()[1]

    # -- the clock --------------------------------------------------------------------

    def _advance(self, now: float) -> None:
        """Step the vehicle to where its clock says it should be."""
        v, speedup = self.vehicle, self.cfg.speedup
        due = (now - self.boot_wall) * speedup - v.t
        if due > LAG_S * speedup:  # fall behind for good rather than chase
            self.boot_wall += (due - LAG_S * speedup) / speedup
        deadline = time.monotonic() + WORK_S
        while (now - self.boot_wall) * speedup - v.t >= DT:
            v.step(DT)
            if time.monotonic() > deadline:
                break

    # -- the loop ---------------------------------------------------------------------

    def serve(self, stop: threading.Event) -> None:
        """Run until ``stop`` is set."""
        try:
            while not stop.is_set():
                self.tick()
        finally:
            self.close()

    def tick(self) -> None:
        """One pass: the clock, the sockets, the messages due."""
        now = time.monotonic()
        if self.reboot_until is not None:
            if now < self.reboot_until:
                time.sleep(0.01)
                return
            self.reboot_until = None
            self.params.reload()
            self.vehicle = self._boot()
        else:
            self._advance(now)
        self._io(self._wait())
        self._emit()
        if self.reboot_requested:
            self._reboot()

    def _wait(self) -> float:
        v, speedup = self.vehicle, self.cfg.speedup
        ahead = v.t + DT - (time.monotonic() - self.boot_wall) * speedup
        return min(max(ahead / speedup, 0.0), 0.02)

    def _io(self, timeout: float) -> None:
        for key, _ in self.sel.select(timeout):
            if isinstance(key.data, str):
                if self.reboot_until is None:
                    self._accept(key.fileobj, key.data)
            else:
                self._read(key.data)
        self.conns = [c for c in self.conns if self._alive(c)]

    def _alive(self, conn: Connection) -> bool:
        if conn.closed:
            try:
                self.sel.unregister(conn.sock)
            except (KeyError, ValueError, OSError):
                pass
            conn.close()
            return False
        return True

    def _accept(self, listener, name: str) -> None:
        try:
            sock, _ = listener.accept()
        except BlockingIOError:
            return
        conn = Connection(sock, self.cfg.sysid, name)
        self.conns.append(conn)
        self.sel.register(sock, selectors.EVENT_READ, conn)

    def _read(self, conn: Connection) -> None:
        for msg in conn.read():
            try:
                commands.handle(self, conn, msg)
            except Exception:  # one bad message must not stop the vehicle
                self.errors.append(traceback.format_exc())
                print(self.errors[-1], file=sys.stderr, flush=True)

    def _emit(self) -> None:
        v = self.vehicle
        texts, v.texts = v.texts, []
        reached, v.reached = v.reached, []
        changed, v.mission.changed = v.mission.changed, False
        for conn in self.conns:
            if not conn.greeted:
                conn.greeted = True
                text = BANNER.format(self.model.id)
                statustext(conn.mav, 6, text)
            for severity, text in texts:
                statustext(conn.mav, severity, text)
            for seq in reached:
                conn.mav.mission_item_reached_send(seq)
            if changed:
                BUILDERS["MISSION_CURRENT"](v, conn.mav)
            conn.emit(v)
            conn.flush()

    def _reboot(self) -> None:
        """Close every link and start again once the delay is over."""
        self.reboot_requested = False
        for conn in self.conns:
            conn.flush()
            conn.close()
            self._alive(conn)
        self.conns = []
        self.reboot_until = time.monotonic() + self.cfg.reboot_delay_s

    def close(self) -> None:
        """Close every socket."""
        for conn in self.conns:
            conn.close()
        for key in list(self.sel.get_map().values()):
            try:
                self.sel.unregister(key.fileobj)
                key.fileobj.close()
            except (KeyError, ValueError, OSError):
                pass
        self.sel.close()
