"""A torpedo's rigid-body physics: five degrees of freedom, no roll.

The hull has four fins in a cross and no roll control, so roll stays 0. The state is
the position (north, east, depth, depth positive down), the heading and pitch, and the
velocities along the body's axes (x forward, y to starboard, z down): surge ``u``, sway
``v``, heave ``w``, pitch rate ``q`` and yaw rate ``r``. ``u``, ``v`` and ``w`` are
relative to the water; the current moves the hull over the ground (``ground_ne``).

The terms are the ones gz-sim's Hydrodynamics system computes for a vehicle: the rigid
body and the added mass (diagonal) with their Coriolis and Munk terms, and quadratic
drag in every degree of freedom. The fins add lift, a pair per plane, from their
angle of attack. Net buoyancy lifts the hull; its centre of gravity under its centre of
buoyancy rights the pitch. Thrust is along the hull and never negative.

Heading and pitch angles are in rad, heading clockwise from north. Steps are
semi-implicit Euler at ``DT_MAX``; a larger step is divided, and the quadratic drag is
applied implicitly so a large hull is stable at any step.
"""

from __future__ import annotations

import math

from .model import Model

G = 9.81
DT_MAX = 0.02  # s
PITCH_LIMIT = 1.2  # rad: the pitch is never allowed past this (cos stays positive)
MIN_FLOW = 0.05  # m/s: the least axial flow the fins' angle of attack is taken at
TAU = 2.0 * math.pi


def wrap(angle: float) -> float:
    """``angle`` in (-pi, pi]."""
    a = math.fmod(angle + math.pi, TAU)
    if a <= 0.0:
        a += TAU
    return a - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    """``x`` limited to [lo, hi]."""
    return lo if x < lo else hi if x > hi else x


def _drag_step(x: float, accel: float, coeff: float, h: float) -> float:
    """``x`` after ``h`` s of ``accel`` and the quadratic damping ``-coeff x |x|``
    (``coeff`` is per unit of inertia), the damping taken implicitly."""
    return (x + h * accel) / (1.0 + h * coeff * abs(x))


class Hull:
    """A torpedo in the water. ``step`` moves it."""

    def __init__(
        self,
        model: Model,
        north: float = 0.0,
        east: float = 0.0,
        depth: float = 0.0,
        heading: float = 0.0,
        pitch: float = 0.0,
        speed: float = 0.0,
    ):
        self.model = model
        self.north, self.east, self.depth = north, east, depth
        self.heading, self.pitch = wrap(heading), pitch
        self.u, self.v, self.w, self.q, self.r = speed, 0.0, 0.0, 0.0, 0.0
        self.rudder, self.elevator = 0.0, 0.0  # the fins' angles now
        self.current_ne = (0.0, 0.0)
        h = model.hydro
        self._mu = model.mass_kg - h.xDotU
        self._mv = model.mass_kg - h.yDotV
        self._mw = model.mass_kg - h.zDotW
        self._iq = model.iyy - h.mDotQ
        self._ir = model.izz - h.nDotR
        self.buoyancy_n = (model.volume_m3 * model.water_kg_m3 - model.mass_kg) * G

    # -- derived state --------------------------------------------------------------

    def velocity_ned(self) -> tuple[float, float, float]:
        """(north, east, down) velocity over the ground, m/s."""
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        ch, sh = math.cos(self.heading), math.sin(self.heading)
        u, v, w = self.u, self.v, self.w
        n = u * cp * ch - v * sh + w * sp * ch + self.current_ne[0]
        e = u * cp * sh + v * ch + w * sp * sh + self.current_ne[1]
        d = -u * sp + w * cp
        return n, e, d

    @property
    def ground_speed(self) -> float:
        """Speed over the ground in the horizontal plane, m/s."""
        n, e, _ = self.velocity_ned()
        return math.hypot(n, e)

    @property
    def course(self) -> float:
        """The direction of travel over the ground, rad clockwise from north."""
        n, e, _ = self.velocity_ned()
        return math.atan2(e, n)

    @property
    def speed_through_water(self) -> float:
        """The hull's speed relative to the water, m/s."""
        return math.sqrt(self.u**2 + self.v**2 + self.w**2)

    # -- the step ---------------------------------------------------------------------

    def step(
        self,
        dt: float,
        thrust_n: float,
        rudder_rad: float,
        elevator_rad: float,
        current_ne: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """Advance ``dt`` s with the thrust (clamped to [0, max_thrust_n]) and the
        fin angles asked of the rudder pair and the elevator pair, in a water that
        moves at ``current_ne`` (m/s north, east) over the ground."""
        self.current_ne = current_ne
        n = max(1, math.ceil(dt / DT_MAX - 1e-9))
        h = dt / n
        thrust = clamp(thrust_n, 0.0, self.model.max_thrust_n)
        for _ in range(n):
            self._step(h, thrust, rudder_rad, elevator_rad)

    def _move_fins(self, h: float, rudder: float, elevator: float) -> None:
        f = self.model.fin
        reach, move = f.max_rad, f.rate_rad_s * h
        self.rudder += clamp(clamp(rudder, -reach, reach) - self.rudder, -move, move)
        self.elevator += clamp(
            clamp(elevator, -reach, reach) - self.elevator, -move, move
        )

    def _fin_loads(self) -> tuple[float, float, float, float]:
        """(sway force, heave force, yaw moment, pitch moment) of the two fin pairs.

        A pair's lift is ``2 * 0.5 rho A CLa alpha u |u|``. A positive rudder turns the
        bow to starboard, a positive elevator lifts the bow. Each fin sees the water's
        flow across it: the hull's sway and yaw (pitch and heave) at the fin's post.
        """
        f = self.model.fin
        ua = max(abs(self.u), MIN_FLOW)
        beta_y = math.atan2(self.v + f.x_m * self.r, ua)
        beta_z = math.atan2(self.w - f.x_m * self.q, ua)
        reach = f.max_rad
        a_r = clamp(self.rudder + beta_y, -reach, reach)
        a_e = clamp(self.elevator - beta_z, -reach, reach)
        k = (
            0.5
            * self.model.water_kg_m3
            * f.area_m2
            * f.lift_slope
            * self.u
            * abs(self.u)
        )
        fy, fz = -k * a_r, k * a_e
        return fy, fz, f.x_m * fy, -f.x_m * fz

    def _step(self, h: float, thrust: float, rudder: float, elevator: float) -> None:
        m, hy = self.model, self.model.hydro
        self._move_fins(h, rudder, elevator)
        fy, fz, mz, my_fin = self._fin_loads()
        sp, cp = math.sin(self.pitch), math.cos(self.pitch)
        u, v, w, q, r = self.u, self.v, self.w, self.q, self.r
        mu, mv, mw = self._mu, self._mv, self._mw
        fx = thrust + self.buoyancy_n * sp
        fz = fz - self.buoyancy_n * cp
        my = my_fin - m.mass_kg * G * m.zg_m * sp
        self.u = _drag_step(u, (fx - q * mw * w + r * mv * v) / mu, -hy.xUabsU / mu, h)
        self.v = _drag_step(v, (fy - r * mu * u) / mv, -hy.yVabsV / mv, h)
        self.w = _drag_step(w, (fz + q * mu * u) / mw, -hy.zWabsW / mw, h)
        self.q = _drag_step(
            q, (my + (mw - mu) * u * w) / self._iq, -hy.mQabsQ / self._iq, h
        )
        self.r = _drag_step(
            r, (mz + (mu - mv) * u * v) / self._ir, -hy.nRabsR / self._ir, h
        )
        n, e, d = self.velocity_ned()
        self.north += h * n
        self.east += h * e
        self.depth += h * d
        self.pitch = clamp(self.pitch + h * self.q, -PITCH_LIMIT, PITCH_LIMIT)
        self.heading = wrap(self.heading + h * self.r / math.cos(self.pitch))
        if self.depth < 0.0:  # at the surface: it floats there, it does not fly
            self.depth = 0.0
