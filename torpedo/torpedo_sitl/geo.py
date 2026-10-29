"""Local north/east metres and WGS84 latitude/longitude, about an origin.

The stand-in flies in a flat frame (north, east) about an origin; the world sees
latitude and longitude. The two are joined with the WGS84 radii of curvature at the
origin's latitude. That is exact enough over a few kilometres, so the vehicle moves
the origin to the hull whenever the hull is ``REBASE_M`` (``vehicle.py``) from it:
the frame stays within 10 km of the hull, and a route may be of any length.

Longitudes are wrapped into (-180, 180]: a frame near 180 degrees takes the short way
across it, and every longitude it gives back is a valid one.
"""

from __future__ import annotations

import math

A = 6378137.0  # WGS84 equatorial radius, m
E2 = 0.00669437999014  # first eccentricity squared
MEAN_RADIUS_M = 6371008.8  # the mean Earth radius, for the great-circle distance


def wrap_lon(lon: float) -> float:
    """``lon`` degrees in (-180, 180]; a value already in range comes back unchanged."""
    if -180.0 < lon <= 180.0:
        return lon
    w = math.fmod(lon + 180.0, 360.0)
    if w <= 0.0:
        w += 360.0
    return w - 180.0


def haversine_m(
    a: tuple[float, float], b: tuple[float, float], radius: float = MEAN_RADIUS_M
) -> float:
    """The great-circle distance between two (lat, lon) degrees, m, on a sphere of
    ``radius`` m."""
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp = p2 - p1
    dl = math.radians(wrap_lon(b[1] - a[1]))
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(h)))


class Frame:
    """The flat frame about ``(lat, lon)`` degrees."""

    def __init__(self, lat: float, lon: float):
        self.lat, self.lon = lat, wrap_lon(lon)
        s = math.sin(math.radians(lat))
        w = math.sqrt(1.0 - E2 * s * s)
        self.m_per_rad_lat = A * (1.0 - E2) / (w**3)  # meridional radius
        self.m_per_rad_lon = A / w * math.cos(math.radians(lat))

    def to_ne(self, lat: float, lon: float) -> tuple[float, float]:
        """(north, east) metres of a position, the short way round in longitude."""
        return (
            math.radians(lat - self.lat) * self.m_per_rad_lat,
            math.radians(wrap_lon(lon - self.lon)) * self.m_per_rad_lon,
        )

    def to_ll(self, north: float, east: float) -> tuple[float, float]:
        """(lat, lon) degrees of a north/east offset; the longitude in (-180, 180]."""
        return (
            self.lat + math.degrees(north / self.m_per_rad_lat),
            wrap_lon(self.lon + math.degrees(east / self.m_per_rad_lon)),
        )
