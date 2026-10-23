"""A torpedo's three loops: speed, heading and depth.

The laws are those of a fin mixer in front of a torpedo, without a position controller
that assumes a hovering vehicle in front of them:

* speed: the thrust that balances the drag at the wanted speed, plus a PI on the speed
  error. The integral is bounded at a quarter of the full thrust and frozen in a turn.
  While the hull turns or changes depth the thrust is never less than what holds the
  steering speed (a torpedo steers on its fins, and fins need flow);
* heading: a PD on the heading error and the yaw rate, to a rudder angle, in units of
  the hull's own turn rate. Full rudder turns the hull at ``v / R`` rad/s (``R`` is
  ``turn_radius_m``). The loop asks a turn rate that is that rate times the heading
  error over ``HEADING_SPAN``, sets the rudder fraction that gives that rate, and adds
  ``RATE_K`` times the shortfall in the rate. A circle feeds its own turn rate
  forward. The hull's turn rate is the unit, so one tuning flies hulls of any size.
  The course asked is a course over the ground: the heading held is that course less
  the hull's crab (``crab``: the sideslip of a turn, the set of a current), so a hull
  in a turn or a cross current makes good the course it was given. On a circle, where
  a turn rate is fed forward, the rate shortfall is integrated too (``RATE_I``): the
  rudder learns what the steady turn takes, and the circle has no standing error. Off
  the circle what it learnt fades in ``RATE_I_FADE_S``;
* depth: the depth error gives a pitch demand (limited to the hull's steepest pitch),
  with a rate term and a small integral that trims the net buoyancy out. The elevators
  hold the pitch in the same way as the rudder holds the heading, a hull turning in
  pitch at about the same ``v / R``. A pitch demand can also be given directly.

Tuned values (the acceptance lines of ``check`` hold with them on both test hulls):
``HEADING_SPAN`` 1.0 rad, ``PITCH_SPAN`` 0.4 rad, ``RATE_K`` 1.0, ``RATE_I`` 0.5 per s,
``DEPTH_P`` 0.06 rad per m, ``DEPTH_D`` 0.3, ``DEPTH_I`` 0.003.
"""

from __future__ import annotations

from dataclasses import dataclass

from .model import Model
from .physics import Hull, clamp, wrap

HEADING_SPAN = 1.0  # rad of heading error at which the turn rate asked is the most
PITCH_SPAN = 0.4  # rad of pitch error at which the pitch rate asked is the most
RATE_K = 1.0  # fins per unit of rate shortfall, over the rate the fins hold
RATE_I = 0.5  # fins per unit of rate shortfall per s, integrated (the rudder's)
RATE_I_FADE_S = 3.0  # s: off a circle, the rudder's integral fades this fast
RATE_I_MAX = 0.5  # the most the rudder's integral holds, a fraction of the full fin
DEPTH_P = 0.06  # rad of pitch per m of depth error
DEPTH_D = 0.3  # rad of pitch per m/s of rate of descent
DEPTH_I = 0.003  # rad of pitch per m of depth error per s
TRIM_BAND = 2.0  # m: the depth integral works only this close to the depth
TRIM_MAX = 0.1  # rad: the most the depth integral trims the pitch demand
SPEED_P = 3.0  # N per m/s short, over the drag's N per (m/s)^2
SPEED_I = 0.3  # N per m/s per s, likewise
SPEED_I_MAX = 0.25  # the integral's bound, over the full thrust
TURNING = 0.03  # rad/s of yaw rate and rad of heading error: it is turning
DEEP_RATE = 0.1  # m/s of descent or climb: it is changing depth
CRAB_SPEED = 0.3  # m/s over the ground: slower than this the course is the heading
CRAB_MAX = 0.6  # rad: the most crab the heading loop allows for


@dataclass(frozen=True)
class Command:
    """What the guidance asks of the loops: a course (rad), a speed through the water
    (m/s) and a depth (m, positive down)."""

    course: float
    speed: float
    depth: float
    turn_rate: float = 0.0  # rad/s: the yaw rate the course itself turns at (a circle)
    over_ground: bool = True  # the course is over the ground; False: it is a heading


@dataclass(frozen=True)
class Actuators:
    """What the loops ask of the hull."""

    thrust_n: float
    rudder_rad: float
    elevator_rad: float


class Control:
    """The loops of one hull."""

    def __init__(self, model: Model):
        self.model = model
        self.drag = -model.hydro.xUabsU
        self.integral = 0.0
        self.trim = 0.0
        self.rate_i = 0.0  # the rudder's integral of the rate shortfall

    def reset(self) -> None:
        """Forget the integrals."""
        self.integral = 0.0
        self.trim = 0.0
        self.rate_i = 0.0

    # -- the loops ------------------------------------------------------------------

    def speed_demand(self, wanted: float) -> float:
        """``wanted`` limited to what the hull can do: steering speed to top speed."""
        m = self.model
        return clamp(wanted, m.steer_speed_mps, m.max_speed_mps)

    def thrust(self, dt: float, wanted: float, speed: float, busy: bool) -> float:
        """The thrust for ``wanted`` m/s, with the hull at ``speed``. ``busy``: it is
        turning or changing depth."""
        m = self.model
        wanted = self.speed_demand(wanted)
        err = wanted - speed
        if not busy:
            bound = SPEED_I_MAX * m.max_thrust_n
            step = SPEED_I * self.drag * err * dt
            self.integral = clamp(self.integral + step, -bound, bound)
        out = self.drag * wanted * abs(wanted) + SPEED_P * self.drag * err
        out += self.integral
        floor = self.drag * m.steer_speed_mps**2 if busy else 0.0
        return clamp(max(out, floor), 0.0, m.max_thrust_n)

    def _fin(
        self, err: float, span: float, rate: float, speed: float, ahead: float = 0.0
    ) -> tuple[float, float, float]:
        """(fin fraction, rate asked, rate shortfall) that close ``err`` (rad) with
        the hull at ``rate`` (rad/s), the rates over the full fin's rate. ``ahead``
        is the rate the target itself moves at, fed forward."""
        m = self.model
        top = clamp(
            abs(speed), m.steer_speed_mps, max(m.steer_speed_mps, m.max_speed_mps)
        )
        full = top / m.turn_radius_m  # rad/s: what full fin turns the hull at
        want = clamp(err / span + ahead / full, -1.0, 1.0)
        short = want - rate / full
        return want + RATE_K * short, want, short

    def rudder(
        self,
        course: float,
        heading: float,
        yaw_rate: float,
        speed: float,
        ahead=0.0,
        dt: float = 0.0,
    ):
        """The rudder angle that turns the hull to ``course``, a course that itself
        turns at ``ahead`` rad/s. The rate shortfall is integrated (``RATE_I``) while
        neither the rate asked nor the rudder is at its limit: a steady turn then
        turns at the rate asked, whatever rudder the hull needs for it."""
        frac, want, short = self._fin(
            wrap(course - heading), HEADING_SPAN, yaw_rate, speed, ahead
        )
        if ahead == 0.0:  # no circle: what it learnt fades
            self.rate_i *= max(0.0, 1.0 - dt / RATE_I_FADE_S)
        elif dt > 0.0 and abs(want) < 1.0 and abs(frac + self.rate_i) < 1.0:
            self.rate_i = clamp(
                self.rate_i + RATE_I * short * dt, -RATE_I_MAX, RATE_I_MAX
            )
        return self.model.fin.max_rad * clamp(frac + self.rate_i, -1.0, 1.0)

    def pitch_demand(
        self, depth_cmd: float, depth: float, rate: float, dt: float = 0.0
    ) -> float:
        """The pitch that brings the hull to ``depth_cmd`` m: nose down to go deeper.
        A small integral trims the net buoyancy out (frozen while the demand is at its
        limit)."""
        lim = self.model.max_pitch_rad
        err = depth_cmd - depth
        p = -DEPTH_P * err + DEPTH_D * rate - self.trim
        if abs(p) < lim and abs(err) < TRIM_BAND:
            self.trim = clamp(self.trim + DEPTH_I * err * dt, -TRIM_MAX, TRIM_MAX)
        return clamp(p, -lim, lim)

    def elevator(self, pitch_cmd: float, pitch: float, pitch_rate: float, speed: float):
        """The elevator angle that holds ``pitch_cmd``."""
        frac, _, _ = self._fin(pitch_cmd - pitch, PITCH_SPAN, pitch_rate, speed)
        return self.model.fin.max_rad * clamp(frac, -1.0, 1.0)

    # -- all three --------------------------------------------------------------------

    def update(self, dt: float, hull: Hull, cmd: Command) -> Actuators:
        """The thrust and fin angles for ``cmd``, the hull as it is now."""
        speed = hull.u
        _, _, rate = hull.velocity_ned()
        heading = cmd.course - crab(hull) if cmd.over_ground else cmd.course
        err = abs(wrap(heading - hull.heading))
        busy = abs(hull.r) > TURNING or err > TURNING or abs(rate) > DEEP_RATE
        thrust = self.thrust(dt, cmd.speed, speed, busy)
        rudder = self.rudder(heading, hull.heading, hull.r, speed, cmd.turn_rate, dt)
        pitch_cmd = self.pitch_demand(cmd.depth, hull.depth, rate, dt)
        elev = self.elevator(pitch_cmd, hull.pitch, hull.q, speed)
        return Actuators(thrust, rudder, elev)


def crab(hull: Hull) -> float:
    """The angle from the hull's heading to its course over the ground, rad: the
    sideslip of a turn and the set of a current. The heading loop steers the heading
    this much short of the course asked, so the course over the ground is the one
    asked. Below ``CRAB_SPEED`` over the ground it is 0; it is at most ``CRAB_MAX``."""
    if hull.ground_speed < CRAB_SPEED:
        return 0.0
    return clamp(wrap(hull.course - hull.heading), -CRAB_MAX, CRAB_MAX)


def drag_thrust(model: Model, speed: float) -> float:
    """The thrust that balances the drag at ``speed``, N."""
    return -model.hydro.xUabsU * speed * abs(speed)
