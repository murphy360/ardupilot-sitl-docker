"""Local north/east metres and WGS84 latitude/longitude, about a start point.

The stand-in flies in a flat frame (north, east) about its start position; the world
sees latitude and longitude. The two are joined with the WGS84 radii of curvature at
the start latitude, which is exact enough over the few kilometres a run covers.
"""

from __future__ import annotations

import math

A = 6378137.0  # WGS84 equatorial radius, m
E2 = 0.00669437999014  # first eccentricity squared


class Frame:
    """The flat frame about ``(lat, lon)`` degrees."""

    def __init__(self, lat: float, lon: float):
        self.lat, self.lon = lat, lon
        s = math.sin(math.radians(lat))
        w = math.sqrt(1.0 - E2 * s * s)
        self.m_per_rad_lat = A * (1.0 - E2) / (w**3)  # meridional radius
        self.m_per_rad_lon = A / w * math.cos(math.radians(lat))

    def to_ne(self, lat: float, lon: float) -> tuple[float, float]:
        """(north, east) metres of a position."""
        return (
            math.radians(lat - self.lat) * self.m_per_rad_lat,
            math.radians(lon - self.lon) * self.m_per_rad_lon,
        )

    def to_ll(self, north: float, east: float) -> tuple[float, float]:
        """(lat, lon) degrees of a north/east offset."""
        return (
            self.lat + math.degrees(north / self.m_per_rad_lat),
            self.lon + math.degrees(east / self.m_per_rad_lon),
        )
