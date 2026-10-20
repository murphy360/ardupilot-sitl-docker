"""The battery: a charge that lasts ``endurance_h`` at cruise.

The power drawn is 30 % hotel load plus propulsion in proportion to the speed cubed,
scaled so that cruise draws exactly the capacity over the endurance. The voltage falls
linearly from 16.8 V full to 13.6 V empty.
"""

from __future__ import annotations

HOTEL = 0.3
FULL_V, EMPTY_V = 16.8, 13.6


class Battery:
    """One battery. ``capacity_mah`` is the full charge."""

    def __init__(self, capacity_mah: float, endurance_h: float, cruise_mps: float):
        self.capacity_mah = capacity_mah
        self.cruise = cruise_mps
        self.cruise_ma = capacity_mah / endurance_h  # draw at cruise, mA
        self.used_mah = 0.0
        self.current_ma = self.cruise_ma * HOTEL

    def step(self, dt: float, speed_mps: float) -> None:
        """Draw for ``dt`` s at ``speed_mps`` through the water."""
        ratio = min(max(speed_mps, 0.0) / self.cruise, 3.0)
        self.current_ma = self.cruise_ma * (HOTEL + (1.0 - HOTEL) * ratio**3)
        self.used_mah = min(
            self.capacity_mah, self.used_mah + self.current_ma * dt / 3600
        )

    @property
    def remaining(self) -> float:
        """The charge left, 0 to 1."""
        return max(0.0, 1.0 - self.used_mah / self.capacity_mah)

    @property
    def voltage(self) -> float:
        """The terminal voltage, V."""
        return EMPTY_V + (FULL_V - EMPTY_V) * self.remaining

    @property
    def remaining_s(self) -> int:
        """Seconds left at the present draw."""
        if self.current_ma <= 0:
            return 0
        left = self.capacity_mah - self.used_mah
        return int(left / self.current_ma * 3600)
