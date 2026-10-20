"""The vehicle: the hull, its pilot, its modes and what it knows of itself.

This is ArduSub's face on a torpedo's body. The modes carry ArduSub's numbers and what
each does on a torpedo is written beside it:

* MANUAL (19): ``MANUAL_CONTROL`` x is the thrust (0 to 1000, below 0 is 0), r the
  rudder and z the elevator (500 neutral). One second without input and the thrust is 0
  and the fins neutral;
* STABILIZE (0): as MANUAL, but z asks a pitch (up to the hull's steepest) that the
  elevators hold;
* ALT_HOLD (2): holds the depth it has. Shallower than ``SURFACED_M`` the motor is off
  and it floats. Deeper, it circles the point where it entered at the least loiter
  radius and the steering speed: a torpedo cannot hover;
* GUIDED (4): a position target (point guidance, then the circle), or a velocity target
  (course and speed at the depth it holds; three seconds without a new one and it
  circles where it is). With no target it circles where it entered;
* SURFACE (9): the steering speed on its heading, pitched up to rise. At the surface it
  switches itself to ALT_HOLD with the motor off, as ArduSub does.

AUTO (3) belongs to the mission protocol (Part C2). The position estimate is the truth
here; it is valid ``ESTIMATE_DELAY_S`` of vehicle time after boot (a GPS fix at boot).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .autopilot import Autopilot
from .battery import Battery
from .control import Actuators, Command
from .geo import Frame
from .model import Model
from .params import ParamStore
from .physics import Hull, clamp

STABILIZE, ALT_HOLD, AUTO, GUIDED, SURFACE, MANUAL = 0, 2, 3, 4, 9, 19
MODE_NAMES = {
    STABILIZE: "STABILIZE",
    ALT_HOLD: "ALT_HOLD",
    AUTO: "AUTO",
    GUIDED: "GUIDED",
    SURFACE: "SURFACE",
    MANUAL: "MANUAL",
}
SUPPORTED = (STABILIZE, ALT_HOLD, GUIDED, SURFACE, MANUAL)
SURFACED_M = 0.5  # shallower than this the sub is at the surface
ESTIMATE_DELAY_S = 5.0  # a GPS fix at boot: the position estimate is valid this late
MANUAL_TIMEOUT_S = 1.0  # no MANUAL_CONTROL for this long: thrust 0, fins neutral
VELOCITY_TIMEOUT_S = 3.0  # no new velocity target for this long: circle where it is
SEABED_M = 50.0  # the SITL's flat floor, for terrain-frame targets
PREARM_NO_POSITION = "PreArm: Need Position Estimate"

FRAME_GLOBAL_INT = 5
FRAME_RELATIVE_ALT_INT = 6
FRAME_TERRAIN_ALT_INT = 11
POSITION_IGNORED = 0b111
VELOCITY_IGNORED = 0b111000


@dataclass
class Target:
    """A GUIDED target: a point at a depth, or a course and speed."""

    kind: str  # "point" or "velocity"
    depth: float
    north: float = 0.0
    east: float = 0.0
    course: float = 0.0
    speed: float = 0.0
    at: float = 0.0  # vehicle time it arrived


class Vehicle:
    """One torpedo front seat's vehicle."""

    def __init__(self, model: Model, params: ParamStore, unix_boot: float):
        self.model = model
        self.params = params
        self.unix_boot = unix_boot  # wall (UNIX) time at boot, s
        p = params.get
        self.frame = Frame(p("SIM_OPOS_LAT"), p("SIM_OPOS_LNG"))
        self.home_alt = p("SIM_OPOS_ALT")
        self.hull = Hull(model, heading=math.radians(p("SIM_OPOS_HDG")))
        self.pilot = Autopilot(model)
        self.battery = Battery(
            p("BATT_CAPACITY", 10000.0), model.endurance_h, model.cruise_speed_mps
        )
        self.t = 0.0  # vehicle time since boot, s
        self.armed = False
        self.critical = False  # a failsafe is active (Part C2)
        self.mode = STABILIZE
        self.speed = model.cruise_speed_mps
        self.target: Target | None = None
        self.manual = (0.0, 0.0, 500.0, 0.0)
        self.manual_at = -math.inf
        self.texts: list[tuple[int, str]] = []  # STATUSTEXT to send: (severity, text)
        self._hold_centre = (0.0, 0.0)
        self._hold_depth = 0.0
        self._hold_heading = self.hull.heading
        self._held = False  # the hold circle is set up
        self.current_ne = (0.0, 0.0)
        self.actuators = Actuators(
            0.0, 0.0, 0.0
        )  # what the last step asked of the hull

    # -- what it knows ----------------------------------------------------------------

    @property
    def boot_ms(self) -> int:
        """Vehicle time since boot, ms."""
        return int(self.t * 1000.0) & 0xFFFFFFFF

    @property
    def unix_usec(self) -> int:
        """UTC now as the vehicle sees it, microseconds: boot time plus vehicle time."""
        return int((self.unix_boot + self.t) * 1e6)

    @property
    def estimate_ok(self) -> bool:
        """The position estimate is valid."""
        return self.t >= ESTIMATE_DELAY_S

    @property
    def position(self) -> tuple[float, float]:
        """(north, east) metres from home."""
        return (self.hull.north, self.hull.east)

    @property
    def latlon(self) -> tuple[float, float]:
        """(lat, lon) degrees."""
        return self.frame.to_ll(*self.position)

    @property
    def altitude_msl(self) -> float:
        """Altitude above mean sea level, m (home altitude less the depth)."""
        return self.home_alt - self.hull.depth

    def say(self, text: str, severity: int = 6) -> None:
        """Queue a STATUSTEXT for every link."""
        self.texts.append((severity, text))

    # -- commands ---------------------------------------------------------------------

    def set_mode(self, number: int) -> tuple[bool, str]:
        """Change mode; (True, "") or (False, why)."""
        if number == self.mode:
            return True, ""
        if number not in SUPPORTED:
            return False, f"mode {number} is not supported"
        self.mode = number
        self.target = None
        self._held = False
        self.pilot.control.reset()
        self._hold_depth = self.hull.depth
        self._hold_heading = self.hull.heading
        self._hold_centre = self.position
        return True, ""

    def arm(self, arm: bool, force: bool = False) -> tuple[bool, str]:
        """Arm or disarm; (True, "") or (False, the PreArm text)."""
        if arm and not self.armed and not force:
            if self.mode in (GUIDED, AUTO) and not self.estimate_ok:
                return False, PREARM_NO_POSITION
        self.armed = arm
        return True, ""

    def change_speed(self, mps: float) -> None:
        """``DO_CHANGE_SPEED``: the speed of the target in force, between the steering
        speed and the top speed."""
        m = self.model
        self.speed = clamp(mps, m.steer_speed_mps, m.max_speed_mps)
        if self.target is not None and self.target.kind == "velocity":
            self.target.speed = self.speed

    def manual_control(self, x: float, z: float, r: float) -> None:
        """``MANUAL_CONTROL``: x thrust (-1000 to 1000), z elevator (0 to 1000), r
        rudder (-1000 to 1000)."""
        self.manual = (x, 0.0, z, r)
        self.manual_at = self.t

    def position_target(
        self,
        frame: int,
        mask: int,
        lat: float,
        lon: float,
        alt: float,
        vel: tuple[float, float],
    ) -> tuple[bool, str]:
        """``SET_POSITION_TARGET_GLOBAL_INT`` (degrees, metres, m/s north and east).
        Taken in GUIDED only. A position holds until another comes; a velocity for
        ``VELOCITY_TIMEOUT_S``."""
        if self.mode != GUIDED:
            return False, "not in GUIDED"
        m = self.model
        if (mask & POSITION_IGNORED) != POSITION_IGNORED:
            depth = self._depth_of(frame, alt)
            if depth is None:
                return False, f"frame {frame} is not supported"
            depth = clamp(depth, 0.0, m.depth_limit_m)
            n, e = self.frame.to_ne(lat, lon)
            old = self.target
            if (
                old is not None
                and old.kind == "point"
                and math.hypot(old.north - n, old.east - e) < 0.05
                and abs(old.depth - depth) < 0.05
            ):
                return True, ""  # the same point again: a held target is not restarted
            self.target = Target("point", depth, n, e, at=self.t)
            self.pilot.guidance.goto((n, e))
            return True, ""
        if (mask & VELOCITY_IGNORED) != VELOCITY_IGNORED:
            vn, ve = vel
            depth = self.target.depth if self.target else self.hull.depth
            speed = math.hypot(vn, ve)
            course = math.atan2(ve, vn) if speed > 1e-3 else self.hull.heading
            speed = clamp(speed, m.steer_speed_mps, m.max_speed_mps)
            self.target = Target(
                "velocity", depth, course=course, speed=speed, at=self.t
            )
            return True, ""
        return False, "no position or velocity in the target"

    def _depth_of(self, frame: int, alt: float) -> float | None:
        if frame == FRAME_GLOBAL_INT:
            return self.home_alt - alt
        if frame == FRAME_RELATIVE_ALT_INT:
            return -alt
        if frame == FRAME_TERRAIN_ALT_INT:
            return SEABED_M - alt
        return None

    # -- the step ---------------------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance ``dt`` s of vehicle time."""
        self.t += dt
        self._tune()
        act = self._actuators(dt) if self.armed else Actuators(0.0, 0.0, 0.0)
        self.actuators = act
        h = self.hull
        h.step(dt, act.thrust_n, act.rudder_rad, act.elevator_rad, self.current_ne)
        self.battery.step(dt, h.speed_through_water)
        if self.mode == SURFACE and h.depth < SURFACED_M:
            self.set_mode(ALT_HOLD)  # at the surface ArduSub holds, motor off

    def _tune(self) -> None:
        g, p = self.pilot.guidance, self.params.get
        g.wp_radius = p("WP_RADIUS_M", self.model.turn_radius_m)
        g.los_la = p("TPDO_LOS_LA")
        g.turn_k = p("TPDO_TURN_K")
        g.loiter_min = p("TPDO_LOIT_MIN")

    def _actuators(self, dt: float) -> Actuators:
        mode = self.mode
        if mode in (MANUAL, STABILIZE):
            return self._manual(dt, mode == STABILIZE)
        if mode == SURFACE:
            return self._course(dt, self._hold_heading, self.model.steer_speed_mps, 0.0)
        if mode == GUIDED:
            return self._guided(dt)
        return self._hold(dt)

    def _manual(self, dt: float, stabilize: bool) -> Actuators:
        m, h = self.model, self.hull
        if self.t - self.manual_at > MANUAL_TIMEOUT_S:
            return Actuators(0.0, 0.0, 0.0)
        x, _, z, r = self.manual
        thrust = clamp(x, 0.0, 1000.0) / 1000.0 * m.max_thrust_n
        rudder = clamp(r, -1000.0, 1000.0) / 1000.0 * m.fin.max_rad
        up = clamp((z - 500.0) / 500.0, -1.0, 1.0)
        if not stabilize:
            return Actuators(thrust, rudder, up * m.fin.max_rad)
        elev = self.pilot.control.elevator(up * m.max_pitch_rad, h.pitch, h.q, h.u)
        return Actuators(thrust, rudder, elev)

    def _course(
        self, dt: float, course: float, speed: float, depth: float
    ) -> Actuators:
        cmd = Command(course, speed, depth)
        return self.pilot.control.update(dt, self.hull, cmd)

    def _circle_here(self) -> None:
        """Set up the hold circle about the point where the mode was entered."""
        g = self.pilot.guidance
        g.circle(self._hold_centre, g.min_radius(), True)
        self._held = True

    def _fly_guidance(self, dt: float, speed: float, depth: float) -> Actuators:
        g, h = self.pilot.guidance, self.hull
        course = g.course(self.position, dt)
        cmd = Command(course, speed, depth, g.curvature * h.u)
        return self.pilot.control.update(dt, h, cmd)

    def _hold(self, dt: float) -> Actuators:
        """ALT_HOLD, and a GUIDED with nothing to fly: the depth it has, in a circle."""
        if self.hull.depth < SURFACED_M and self._hold_depth < SURFACED_M:
            return Actuators(0.0, 0.0, 0.0)  # at the surface: the motor is off
        if not self._held:
            self._circle_here()
        return self._fly_guidance(dt, self.model.steer_speed_mps, self._hold_depth)

    def _guided(self, dt: float) -> Actuators:
        tgt = self.target
        if tgt is not None and tgt.kind == "velocity":
            if self.t - tgt.at > VELOCITY_TIMEOUT_S:
                self._hold_centre, self._hold_depth = self.position, tgt.depth
                self.target, self._held = None, False
                return self._hold(dt)
            return self._course(dt, tgt.course, tgt.speed, tgt.depth)
        if tgt is not None:
            return self._fly_guidance(dt, self.speed, tgt.depth)
        return self._hold(dt)
