"""ArduSub's failsafes and the Proteus-dead script, on the vehicle.

* GCS: no ``HEARTBEAT`` from ``MAV_GCS_SYSID`` (the range up to ``MAV_GCS_SYSID_HI``
  when it is set) for ``FS_GCS_TIMEOUT`` s of vehicle time while armed, after one was
  heard. ``FS_GCS_ENABLE``: 0 none, 1 warn, 2 disarm, 3 ALT_HOLD (it circles at
  depth), 4 SURFACE. The heartbeat says CRITICAL until the GCS is back. The words are
  the captured ones (``captured.py``);
* battery: ``BATT_LOW_*`` and ``BATT_CRT_*`` (volts below the limit for
  ``BATT_LOW_TIMER`` s, or the charge left below the mAh limit), the action by
  ``BATT_FS_LOW_ACT`` / ``BATT_FS_CRT_ACT`` (0 none, 1 warn, 2 disarm, 3 surface);
* pilot input: ``FS_PILOT_INPUT`` (1 warn, 2 disarm) after ``FS_PILOT_TIMEOUT`` s with
  no ``MANUAL_CONTROL`` in MANUAL or STABILIZE;
* fence and leak: parameters only. A fence breach is shown in ``FENCE_STATUS``; ArduSub
  only logs it;
* Proteus dead: ``deploy/autopilot/ardusub/proteus_dead.lua`` of the Proteus
  repository, in Python: the same states, timings and texts (see :class:`DeadScript`).
"""

from __future__ import annotations

import math

from . import captured

ALT_HOLD, AUTO, SURFACE = 2, 3, 9
LOW, CRITICAL = 1, 2
WARN, DISARM, SURFACE_ACT = 1, 2, 3
GCS_FAILSAFE = "gcs_failsafe"  # ModeReason::GCS_FAILSAFE
PDEAD_PERIOD_S = 0.2  # the script runs about five times a second
BATT_REPEAT_S = 30.0  # a battery failsafe says it again this often
DO_LAND_START = 189
INFO, WARNING = 6, 4


class Failsafes:
    """The failsafes of one vehicle. ``step`` runs every vehicle step."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.gcs_seen: float | None = None  # vehicle time of the last GCS heartbeat
        self.gcs_active = False
        self.battery_level = 0
        self.battery_since = [None, None]  # when the low / critical voltage began
        self.battery_said = -math.inf
        self.battery_active = False
        self.pilot_active = False
        self.armed_at = 0.0
        self.dead = DeadScript(vehicle)

    def p(self, name: str, default: float = 0.0) -> float:
        """A parameter of the vehicle."""
        return self.v.params.get(name, default)

    @property
    def active(self) -> bool:
        """Any failsafe that shows is on (the heartbeat says CRITICAL)."""
        return self.gcs_active or self.battery_active

    def armed(self) -> None:
        """The vehicle has just armed: the pilot-input clock starts."""
        self.armed_at = self.v.t

    # -- the GCS ------------------------------------------------------------------

    def heartbeat(self, sysid: int) -> None:
        """A HEARTBEAT from system ``sysid`` was received."""
        low = int(self.p("MAV_GCS_SYSID", 255.0))
        high = int(self.p("MAV_GCS_SYSID_HI"))
        if low <= sysid <= max(low, high):
            self.gcs_seen = self.v.t

    def _gcs(self) -> None:
        action = int(self.p("FS_GCS_ENABLE"))
        if action == 0 or self.gcs_seen is None:
            self.gcs_active = False
            return
        silent = self.v.t - self.gcs_seen
        if silent <= self.p("FS_GCS_TIMEOUT", 5.0):
            if self.gcs_active:
                self.gcs_active = False
                self.v.announce(captured.gcs_cleared())
            return
        if self.gcs_active or not self.v.armed:
            return
        self.gcs_active = True
        self.v.announce(captured.gcs_lost(action))
        if action == DISARM:
            self.v.arm(False)
        elif action == 3:
            self.v.set_mode(ALT_HOLD, GCS_FAILSAFE)
        elif action == 4:
            self.v.set_mode(SURFACE, GCS_FAILSAFE)

    # -- the battery --------------------------------------------------------------

    def _limit(self, level: int) -> bool:
        """Whether the battery is beyond the limit of ``level`` (volts for the timer,
        or the charge left)."""
        volt, mah = (
            ("BATT_LOW_VOLT", "BATT_LOW_MAH")
            if level == LOW
            else ("BATT_CRT_VOLT", "BATT_CRT_MAH")
        )
        b = self.v.battery
        left = b.capacity_mah - b.used_mah
        if self.p(mah) > 0 and left < self.p(mah):
            return True
        if self.p(volt) > 0 and b.voltage < self.p(volt):
            since = self.battery_since[level - 1]
            if since is None:
                self.battery_since[level - 1] = since = self.v.t
            return self.v.t - since >= self.p("BATT_LOW_TIMER", 10.0)
        self.battery_since[level - 1] = None
        return False

    def _battery(self) -> None:
        level = CRITICAL if self._limit(CRITICAL) else LOW if self._limit(LOW) else 0
        if level == 0:
            self.battery_level = 0
            self.battery_active = False
            return
        v = self.v
        if level > self.battery_level or v.t - self.battery_said >= BATT_REPEAT_S:
            sev, template = captured.battery_template(
                "low" if level == LOW else "critical"
            )
            b = v.battery
            v.say(template.format(n=1, volts=b.voltage, mah=int(b.used_mah)), sev)
            self.battery_said = v.t
        if level > self.battery_level:
            self.battery_level = level
            act = int(self.p("BATT_FS_LOW_ACT" if level == LOW else "BATT_FS_CRT_ACT"))
            self.battery_active = act != 0
            if act == DISARM:
                v.arm(False)
            elif act == SURFACE_ACT:
                v.set_mode(SURFACE, "battery")

    # -- the pilot's input --------------------------------------------------------

    def _pilot(self) -> None:
        """No MANUAL_CONTROL for ``FS_PILOT_TIMEOUT`` s in MANUAL or STABILIZE. A
        warning shows nothing (ArduSub 4.7.1 showed nothing, ``captured.pilot_input``);
        action 2 disarms."""
        action = int(self.p("FS_PILOT_INPUT"))
        v = self.v
        manual = v.armed and v.mode in (0, 19)
        if action == 0 or not manual:
            self.pilot_active = False
            return
        last = max(v.manual_at, self.armed_at)
        silent = v.t - last > self.p("FS_PILOT_TIMEOUT", 3.0)
        if silent and not self.pilot_active and action == DISARM:
            v.arm(False)
        self.pilot_active = silent

    # -- the step -----------------------------------------------------------------

    def step(self, dt: float) -> None:
        """Run the checks for this step."""
        self._gcs()
        self._battery()
        self._pilot()
        self.dead.step(dt)


class DeadScript:
    """``proteus_dead.lua`` in Python: what the autopilot does when Proteus is silent.

    It acts only on ArduSub's own GCS failsafe: armed, the GCS silent for
    ``FS_GCS_TIMEOUT``, the vehicle in ALT_HOLD put there by that failsafe
    (``FS_GCS_ENABLE`` 3). Once per silence, and it never fights anybody: a mode that
    someone else chose ends its part, and a heartbeat back ends the episode.

    * ``PDEAD_ACTION`` 1: the failsafe's depth hold is a submerged wait of
      ``PDEAD_HOLD_S`` s (0: no limit), then SURFACE;
    * ``PDEAD_ACTION`` 2: the depth hold is left at once for AUTO, from the return
      route's ``DO_LAND_START`` marker (or item 1). If AUTO is refused it waits as
      action 1 does;
    * ``PDEAD_ACTION`` 0: off.
    """

    def __init__(self, vehicle):
        self.v = vehicle
        self.phase: str | None = None  # None | hold | route | surfaced | yielded
        self.since = 0.0
        self.next_run = 0.0
        self.said_loaded = False

    def _say(self, level: int, text: str) -> None:
        self.v.say(f"PDEAD: {text}"[:50], level)

    def _surface(self, why: str) -> None:
        if self.v.set_mode(SURFACE, "pdead")[0]:
            self.phase = "surfaced"
            self._say(WARNING, f"{why}, surfacing")

    def route_start(self) -> int | None:
        """Where the return route starts: its ``DO_LAND_START`` marker when Proteus
        keeps a route the vehicle was flying in front of it, else item 1. A marker with
        nothing after it is a route with no way home: None."""
        mission = self.v.mission
        n = mission.count
        for i in range(1, n):
            item = mission.item(i)
            if item is not None and item.command == DO_LAND_START:
                return None if i == n - 1 else i
        return 1

    def step(self, dt: float) -> None:
        """Run the script's update when it is due."""
        v = self.v
        if not self.said_loaded and int(v.params.get("SCR_ENABLE")) == 1:
            self.said_loaded = True
            self._say(INFO, "loaded")
        if v.t < self.next_run:
            return
        self.next_run = v.t + PDEAD_PERIOD_S
        self._update()

    def _update(self) -> None:
        v, fs = self.v, self.v.failsafe
        action = int(v.params.get("PDEAD_ACTION"))
        if action < 1 or not v.armed:
            self.phase = None
            return
        if fs.gcs_seen is None:
            return  # never heard Proteus: nothing to lose
        silent = v.t - fs.gcs_seen
        if silent < v.params.get("FS_GCS_TIMEOUT", 5.0):
            if self.phase is not None:
                self._say(INFO, f"Proteus back after {v.t - self.since:.0f} s")
                self.phase = None
            return
        mode = v.mode
        if self.phase is None:
            if mode != ALT_HOLD or v.mode_reason != GCS_FAILSAFE:
                return  # wait for ArduSub's own failsafe: its depth hold
            self._begin(action)
        elif self.phase == "hold":
            hold = v.params.get("PDEAD_HOLD_S", 120.0)
            if mode != ALT_HOLD:
                self.phase = "yielded"
            elif hold > 0 and v.t - self.since >= hold:
                self._surface(f"silent {v.t - self.since:.0f} s")
        elif self.phase == "route":
            if mode != AUTO:
                self.phase = "yielded"
            elif v.mission.complete:
                self._surface("route flown")

    def _begin(self, action: int) -> None:
        v = self.v
        self.since = v.t
        self.phase = "hold"
        if action != 2:
            self._say(WARNING, "Proteus silent, holding at depth")
            return
        start = self.route_start()
        mission = v.mission
        if (
            mission.count > 2
            and start
            and v.set_current_item(start)
            and v.set_mode(AUTO, "pdead")[0]
        ):
            self.phase = "route"
            self._say(WARNING, f"flying the return route, {mission.count} items")
        else:
            self._say(WARNING, "no route flown, holding at depth")
