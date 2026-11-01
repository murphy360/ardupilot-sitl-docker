"""The downward rangefinder as ArduSub reads it, and the seabed it senses.

ArduSub reads its downward rangefinder at 20 Hz (``Sub::read_rangefinder``):

* fitted: ``RNGFND1_TYPE`` is set (ArduSub: a rangefinder facing down);
* its status: ``Good`` within ``RNGFND1_MIN`` and ``RNGFND1_MAX``, ``OutOfRangeLow``
  under the minimum, ``OutOfRangeHigh`` past the maximum;
* healthy: the status is ``Good`` and has been for ``HEALTH_COUNT`` readings in a row;
* the filtered range: a low-pass at ``FILTER_HZ`` (``RANGEFINDER_WPNAV_FILT_HZ``) of
  the healthy readings, started afresh after ``TIMEOUT_S`` without one;
* the terrain offset (``rangefinder_terrain_offset_cm``): the depth of the bed, the
  estimate's depth plus the filtered range, kept from the last healthy reading.

The front seat's own addition is the bed's slope: ``bed_rate``, how fast the bed
under the hull rises or falls (m/s, down), from the depth plus each healthy reading
(the unfiltered bed, so the hull's own climbs and dives do not show in it) smoothed
over ``SLOPE_TAU_S``. The depth loop feeds it forward (``terrain.py``).

Waypoint navigation follows the terrain on it when ``WP_RFND_USE`` is 1 (AC_WPNav):
:meth:`Rangefinder.bed` is the bed's depth while it is healthy, else None. The reading
is the world's (``world.py``: ``50 - SIM_SONAR_POS_Z - depth``).
"""

from __future__ import annotations

import math

RATE_S = 0.05  # ArduSub reads it at 20 Hz
HEALTH_COUNT = 3  # RANGEFINDER_HEALTH_MAX: good readings in a row for healthy
TIMEOUT_S = 1.0  # RANGEFINDER_TIMEOUT_MS: the filter starts afresh after this
FILTER_HZ = 0.25  # RANGEFINDER_WPNAV_FILT_HZ
SLOPE_TAU_S = 0.25  # s: the bed's rate is smoothed this much
GOOD, LOW, HIGH, NONE = "good", "low", "high", "none"


class Rangefinder:
    """The downward rangefinder of one vehicle. ``step`` runs every vehicle step."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.status = NONE
        self.reading = math.nan  # m, the last reading
        self.good_count = 0
        self.healthy = False
        self.filtered = math.nan  # m, the filtered range
        self.last_healthy = -math.inf  # vehicle time of the last healthy reading
        self.bed_depth: float | None = None  # m: the terrain offset, as a depth
        self.bed_rate = 0.0  # m/s (down): how fast the bed under the hull moves
        self._raw_bed: float | None = None  # m: the last unfiltered bed
        self._due = 0.0

    def _p(self, name: str, default: float = 0.0) -> float:
        return self.v.params.get(name, default)

    @property
    def fitted(self) -> bool:
        """A downward rangefinder is fitted (``RNGFND1_TYPE`` is set)."""
        return self._p("RNGFND1_TYPE") != 0

    @property
    def used(self) -> bool:
        """Waypoint navigation follows the terrain on it (``WP_RFND_USE``)."""
        return self.fitted and self._p("WP_RFND_USE", 1.0) >= 0.5

    def bed(self) -> float | None:
        """The depth of the bed it senses, m, while it is healthy and used; else
        None (``AC_WPNav::get_terrain_U_m`` has no terrain)."""
        return self.bed_depth if self.used and self.healthy else None

    def step(self, dt: float) -> None:
        """Read it when a reading is due (20 Hz of vehicle time)."""
        self._due -= dt
        if self._due > 1e-9:
            return
        self._due += RATE_S
        self._read()

    def _read(self) -> None:
        v = self.v
        if not self.fitted:
            self.status, self.good_count, self.healthy = NONE, 0, False
            return
        r = v.world.range_m(v.hull.depth)
        self.reading = r
        if r < self._p("RNGFND1_MIN", 0.2):
            self.status = LOW
        elif r > self._p("RNGFND1_MAX", 30.0):
            self.status = HIGH
        else:
            self.status = GOOD
        self.good_count = self.good_count + 1 if self.status == GOOD else 0
        self.healthy = self.good_count >= HEALTH_COUNT
        if not self.healthy:
            self.bed_rate, self._raw_bed = 0.0, None
            return
        if v.t - self.last_healthy > TIMEOUT_S:
            self.filtered = r  # not used for a second: start afresh
        else:
            rc = 1.0 / (2.0 * math.pi * FILTER_HZ)
            self.filtered += (r - self.filtered) * RATE_S / (RATE_S + rc)
        self.last_healthy = v.t
        self.bed_depth = v.depth + self.filtered
        raw = v.depth + r
        if self._raw_bed is not None:
            slope = (raw - self._raw_bed) / RATE_S
            self.bed_rate += (slope - self.bed_rate) * RATE_S / (RATE_S + SLOPE_TAU_S)
        self._raw_bed = raw
