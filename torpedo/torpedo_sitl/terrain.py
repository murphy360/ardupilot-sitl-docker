"""The terrain frame: following the bed the rangefinder senses, and its failsafe.

ArduSub 4.7 flies a terrain-frame item (``MAV_FRAME_GLOBAL_TERRAIN_ALT`` / ``_INT``)
on its downward rangefinder (``rangefinder.py``): the depth asked is the bed's depth,
the terrain offset, less the item's height, all along each line. This module is that
behaviour on a torpedo, in ArduSub's words:

* following: :meth:`Terrain.depth_for` is the depth for a height above the bed,
  between the surface and the hull's ``depth_limit_m``, with the rate it moves at
  (fed forward to the depth loop, as ArduSub's position controller shapes its terrain
  offset). Each call records whether there was terrain to follow
  (``failsafe_terrain_set_status``): a failure begins the clock and 0.1 s of success
  clears it;
* the failsafe (``Sub::failsafe_terrain_check``, 3.3 Hz): in AUTO or GUIDED after
  ``TIMEOUT_S`` of failures, ``Failsafe terrain triggered``. With a rangefinder fitted
  recovery starts (``Attempting auto failsafe recovery``), else the action at once.
  ArduSub starts AUTO's recovery from GUIDED too, which then runs nowhere: in GUIDED
  the words are said, the target is flown on at the depth last asked, and nothing
  more happens;
* recovery (``ModeAuto::auto_terrain_recover_run``, AUTO only): the mission stops; it
  climbs at ``WP_SPD_UP`` when the reading is under ``RNGFND1_MIN``, asks
  ``WP_SPD_DN`` when it is past ``RNGFND1_MAX`` (ArduSub gives it as a climb rate, so
  it climbs then too: it never goes deeper), and holds its depth otherwise. A torpedo
  cannot hover: it holds by circling where recovery began. 1.5 s of healthy range:
  ``Terrain failsafe recovery successful!`` and the mission resumes from the current
  item. After ``RECOVER_TIMEOUT_S``: ``Terrain failsafe recovery timeout!`` and the
  ``FS_TERRAIN_ENAB`` action: 0 disarm (ArduSub's default), 1 hold (ArduSub asks
  POSHOLD, then ALT_HOLD; the torpedo has ALT_HOLD, its circle at depth), 2 SURFACE;
* a terrain leg that begins after one that was not (ArduSub's waypoint controller
  converts its origin then) with no terrain: ``Terrain data (rangefinder) not
  available`` and the failsafe's event at once;
* arming (``AP_Arming::terrain_checks``): with terrain-frame items in the stored
  mission and the parameter checks on (``ARMING_SKIPCHK`` without bit 5; ArduSub's
  default skips them), arming is refused: ``Arm: terrain disabled`` when the command
  is refused, ``PreArm: terrain disabled`` from the periodic check (``TERRAIN_ENABLE``
  0). With ``TERRAIN_ENABLE`` 1 the front seat has no terrain database, so the words
  are ``waiting for terrain data``. A forced arm skips the check, as ArduSub's does.
"""

from __future__ import annotations

from .control import Actuators
from .mission import LOCATED
from .physics import clamp

CRITICAL, WARNING, INFO = 2, 4, 6
TRIGGERED = (CRITICAL, "Failsafe terrain triggered")
RECOVERING = (WARNING, "Attempting auto failsafe recovery")
RECOVERED = (INFO, "Terrain failsafe recovery successful!")
TIMED_OUT = (CRITICAL, "Terrain failsafe recovery timeout!")
NO_RANGEFINDER = (CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!")
NO_DATA = (WARNING, "Terrain data (rangefinder) not available")
TERRAIN_FRAMES = (10, 11)  # MAV_FRAME_GLOBAL_TERRAIN_ALT, _INT
TIMEOUT_S = 1.0  # FS_TERRAIN_TIMEOUT_MS
RECOVER_TIMEOUT_S = 10.0  # FS_TERRAIN_RECOVER_TIMEOUT_MS
RECOVER_HEALTHY_S = 1.5  # healthy range for this long ends recovery
CLEAR_S = 0.1  # this long without a failure clears the failure clock
CHECK_S = 0.3  # the check runs in ArduSub's 3.3 Hz loop
PREARM_S, PREARM_SOON_S = 30.0, 4.0  # the pre-arm check's display periods
PARAMETERS_CHECK = 1 << 5  # AP_Arming::Check::PARAMETERS
DISARM, HOLD, SURFACE_ACT = 0, 1, 2
ALT_HOLD, AUTO, GUIDED, SURFACE = 2, 3, 4, 9
REASON = "terrain_failsafe"  # ModeReason::TERRAIN_FAILSAFE


def is_terrain(frame: int) -> bool:
    """A terrain-frame altitude: a height above the bed."""
    return frame in TERRAIN_FRAMES


class Terrain:
    """Terrain following, the terrain failsafe and its arming check for a vehicle."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.first_fail: float | None = None
        self.last_fail: float | None = None
        self.active = False  # failsafe.terrain: the heartbeat says CRITICAL
        self.recovering = False
        self.depth_rate = 0.0  # m/s: how fast the depth asked moves
        self._since = 0.0  # when recovery began
        self._healthy_at: float | None = None
        self._depth = 0.0  # the depth recovery holds
        self._circling = False  # recovery's circle is set up
        self._check_due = 0.0
        self._prearm_due = 0.0
        self._shown = 0.0  # when the pre-arm check last showed its failures
        self._passed = False
        self._soon = False

    def _p(self, name: str, default: float = 0.0) -> float:
        return self.v.params.get(name, default)

    # -- following ----------------------------------------------------------------

    def depth_for(self, height: float, fallback: float) -> float:
        """The depth that is ``height`` m above the bed (``fallback``, the depth
        asked until now, while there is no terrain). Records the status and sets
        ``depth_rate``: the bed's rate, while the depth asked is not held at the
        surface or the depth limit."""
        rf = self.v.rangefinder
        bed = rf.bed()
        self.status(bed is not None)
        depth = fallback if bed is None else bed - height
        held = clamp(depth, 0.0, self.v.model.depth_limit_m)
        self.depth_rate = rf.bed_rate if bed is not None and held == depth else 0.0
        return held

    def level(self) -> None:
        """A leg that is not in the terrain frame: nothing to follow."""
        self.depth_rate = 0.0
        self.status(True)

    def leg_start(self) -> None:
        """A terrain leg begins after one that was not: it needs terrain now."""
        if self.v.rangefinder.bed() is None:
            self.v.announce(NO_DATA)
            self.on_event()

    def status(self, ok: bool) -> None:
        """``failsafe_terrain_set_status``: the time of the first and the latest
        failure; cleared after ``CLEAR_S`` of successes."""
        t = self.v.t
        if not ok:
            self.last_fail = t
            if self.first_fail is None:
                self.first_fail = t
        elif self.last_fail is not None and t - self.last_fail > CLEAR_S:
            self.first_fail = self.last_fail = None

    # -- the failsafe -------------------------------------------------------------

    def step(self, dt: float) -> None:
        """The failsafe's check (3.3 Hz) and the pre-arm check (1 Hz)."""
        self._check_due -= dt
        if self._check_due <= 1e-9:
            self._check_due += CHECK_S
            self._check()
        self._prearm_due -= dt
        if self._prearm_due <= 1e-9:
            self._prearm_due += 1.0
            self._prearm()

    def _check(self) -> None:
        v = self.v
        timeout = (
            self.first_fail is not None and self.last_fail - self.first_fail > TIMEOUT_S
        )
        trigger = v.mode in (AUTO, GUIDED) and timeout
        if trigger == self.active:
            return
        if trigger:
            v.announce(TRIGGERED)
            self.on_event()
        else:
            self.active = False

    def on_event(self) -> None:
        """``failsafe_terrain_on_event``: recover when a rangefinder is fitted."""
        self.active = True
        if self.v.rangefinder.fitted:  # it reads, in range or out: it can recover
            self._recover_start()
        else:
            self.act()

    def _recover_start(self) -> None:
        """``ModeAuto::auto_terrain_recover_start``. Said in GUIDED too, as ArduSub
        says it; only AUTO runs recovery."""
        v = self.v
        self.recovering = True
        self._since, self._healthy_at = v.t, None
        self._depth, self._circling = v.depth, False
        v.announce(RECOVERING)

    def act(self) -> None:
        """``failsafe_terrain_act``: the ``FS_TERRAIN_ENAB`` action."""
        v, action = self.v, int(self._p("FS_TERRAIN_ENAB"))
        if action == HOLD:
            v.set_mode(ALT_HOLD, REASON)
        elif action == SURFACE_ACT:
            v.set_mode(SURFACE, REASON)
        else:
            v.arm(False)

    def recovery_actuators(self, dt: float) -> Actuators:
        """One step of recovery in AUTO: climb, or hold, circling."""
        v, rf = self.v, self.v.rangefinder
        rate = 0.0  # m/s up
        if rf.status == "low":
            rate, self._healthy_at = self._p("WP_SPD_UP", 2.5), None
        elif rf.status == "high":
            rate, self._healthy_at = abs(self._p("WP_SPD_DN", 1.5)), None
        elif rf.status == "good":
            if rf.healthy and self._healthy_at is None:
                self._healthy_at, self._depth = v.t, v.depth
            if rf.healthy and v.t - self._healthy_at > RECOVER_HEALTHY_S:
                v.announce(RECOVERED)
                self.first_fail = self.last_fail = None
                self.active = self.recovering = False
                v.auto.start()  # the mission resumes at its current item
                return v.auto.actuators(dt)
        else:
            v.announce(NO_RANGEFINDER)
            self.act()
            return Actuators(0.0, 0.0, 0.0)
        if v.t - self._since > RECOVER_TIMEOUT_S:
            v.announce(TIMED_OUT)
            self.act()
        if not self._circling:  # where recovery began: a torpedo holds by circling
            g, self._circling = v.pilot.guidance, True
            g.circle(v.position, g.min_radius(), True)
        self._depth = max(self._depth - rate * dt, 0.0)
        return v._fly_guidance(dt, v.model.steer_speed_mps, self._depth)

    # -- arming -------------------------------------------------------------------

    def mission_needs_terrain(self) -> bool:
        """The stored mission has a terrain-frame item with a place."""
        m = self.v.mission
        return any(it.command in LOCATED and is_terrain(it.frame) for it in m.items[1:])

    def arming_failure(self) -> str | None:
        """Why ``AP_Arming::terrain_checks`` refuses to arm, or None."""
        if int(self._p("ARMING_SKIPCHK")) & PARAMETERS_CHECK:
            return None
        if not self.mission_needs_terrain():
            return None
        if self._p("TERRAIN_ENABLE") < 0.5:
            return "terrain disabled"
        return "waiting for terrain data"

    def _prearm(self) -> None:
        """``AP_Arming::update``: while disarmed, the failures are shown every 30 s,
        and 4 s after the checks start failing."""
        v = self.v
        t = v.t
        show = (self._soon and t - self._shown > PREARM_SOON_S) or (
            t - self._shown > PREARM_S
        )
        if show:
            self._soon, self._shown = False, t
        if v.armed:
            return
        why = self.arming_failure()
        if show and why:
            v.announce((CRITICAL, f"PreArm: {why}"))
        if why and self._passed:
            self._soon = True
        self._passed = why is None
