"""The loops and the guidance together: a hull's pilot.

``Autopilot.step(dt, hull)`` returns the actuators for one step: guidance gives the
course from the hull's position, the loops hold it, the speed and the depth.
"""

from __future__ import annotations

import math

from .control import Actuators, Command, Control
from .guidance import Guidance
from .model import Model
from .physics import Hull


class Autopilot:
    """One hull's pilot. Give it a shape to fly (``guidance``), a speed and a depth."""

    def __init__(self, model: Model, **tuning):
        self.model = model
        self.control = Control(model)
        self.guidance = Guidance(model, **tuning)
        self.speed = model.cruise_speed_mps
        self.depth = 0.0
        self.last = Command(0.0, self.speed, 0.0)

    def step(self, dt: float, hull: Hull) -> Actuators:
        """The actuators for the next ``dt`` s."""
        course = self.guidance.course((hull.north, hull.east), dt)
        rate = self.guidance.curvature * hull.u
        self.last = Command(course, self.speed, self.depth, rate)
        return self.control.update(dt, hull, self.last)


def fly(hull: Hull, pilot: Autopilot, seconds: float, dt: float = 0.05, current=(0, 0)):
    """Fly ``hull`` under ``pilot`` for ``seconds``; yields (t, hull) every step."""
    steps = math.ceil(seconds / dt)
    for i in range(steps):
        act = pilot.step(dt, hull)
        hull.step(dt, act.thrust_n, act.rudder_rad, act.elevator_rad, current)
        yield (i + 1) * dt, hull
