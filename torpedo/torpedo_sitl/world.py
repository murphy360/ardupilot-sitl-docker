"""The simulated world: the current, the GPS, the DVL, the rangefinder, the depth sensor.

A real autopilot has none of these: they are ``SIM_*`` parameters of a simulator, set
from outside (the start-up file the container is given, or ``PARAM_SET`` from a world
simulator), and they live in this stand-in alone. Whatever error the world puts on the
estimate is made here; nothing a vehicle's operator loads carries it.

* ``SIM_WIND_SPD`` / ``SIM_WIND_DIR`` (the direction it comes *from*): the water
  current, the same at every depth (``SIM_WIND_T`` is read, and ignored);
* ``SIM_GPS1_ENABLE`` (and the older ``SIM_GPS_DISABLE``): the GPS fix.
  ``SIM_GPS1_GLTCH_X/Y``: the fix's offset in degrees of latitude and longitude;
* a DVL (fitted when ``VISO_TYPE`` is set and ``EK3_SRC1_VELXY`` is 6): locked when
  ``SIM_VICON_TMASK`` has bit 1 or 3 (2 or 8). With the GPS off, the estimate
  dead-reckons on the true velocity plus ``SIM_VICON_VGLI_X/Y`` (m/s north, east) and
  noise ``SIM_VICON_V_SD`` (m/s, one sigma), and carries ``SIM_VICON_GLIT_X/Y`` (m) as a
  steady position error. With no GPS and no lock it holds its last position (EKF3's
  ``const_pos_mode``). A fix snaps the estimate back to the GPS;
* ``SIM_SONAR_POS_Z``: the rangefinder reads ``50 - SIM_SONAR_POS_Z - depth``, the
  SITL's flat 50 m floor;
* ``SIM_BARO_DISABLE``: the depth sensor fails; the estimate's depth freezes.
"""

from __future__ import annotations

import math
import random

from . import captured
from .geo import Frame
from .params import ParamStore

SEABED_M = 50.0  # the SITL's flat floor
GPS, DVL, NONE = "gps", "dvl", "none"


def flow_vector(speed: float, from_deg: float) -> tuple[float, float]:
    """(north, east) m/s of a flow of ``speed`` coming from ``from_deg``."""
    toward = math.radians(from_deg + 180.0)
    return speed * math.cos(toward), speed * math.sin(toward)


class World:
    """The estimate and the environment of one vehicle."""

    def __init__(self, params: ParamStore, frame: Frame, seed: int = 1):
        self.params, self.frame = params, frame
        self.rng = random.Random(seed)
        self.north = self.east = 0.0  # the estimator's own integration
        self.depth = 0.0  # the estimate's depth
        self.mode = GPS
        self.baro_lost = False
        self.events: list[tuple[int, str]] = []  # STATUSTEXT: (severity, text)

    # -- the parameters -----------------------------------------------------------

    def _p(self, name: str, default: float = 0.0) -> float:
        return self.params.get(name, default)

    @property
    def gps_on(self) -> bool:
        """The GPS has a fix."""
        return (
            self._p("SIM_GPS1_ENABLE", 1.0) >= 0.5 and self._p("SIM_GPS_DISABLE") < 0.5
        )

    @property
    def dvl_locked(self) -> bool:
        """A DVL is fitted and has a lock on the bottom."""
        fitted = self._p("VISO_TYPE") != 0 and self._p("EK3_SRC1_VELXY") == 6
        return fitted and int(self._p("SIM_VICON_TMASK")) & 0b1010 != 0

    @property
    def current(self) -> tuple[float, float]:
        """The water's velocity over the ground, (north, east) m/s."""
        return flow_vector(self._p("SIM_WIND_SPD"), self._p("SIM_WIND_DIR"))

    @property
    def gps_offset(self) -> tuple[float, float]:
        """The GPS fix's offset from the truth, (north, east) m."""
        return (
            math.radians(self._p("SIM_GPS1_GLTCH_X")) * self.frame.m_per_rad_lat,
            math.radians(self._p("SIM_GPS1_GLTCH_Y")) * self.frame.m_per_rad_lon,
        )

    @property
    def sonar_offset(self) -> float:
        """``SIM_SONAR_POS_Z``, m."""
        return self._p("SIM_SONAR_POS_Z")

    def range_m(self, depth: float) -> float:
        """What the downward rangefinder reads at ``depth`` (the truth), m."""
        return SEABED_M - self.sonar_offset - depth

    def state_name(self) -> str:
        """Which of the three captured EKF states the estimator is in."""
        return self.mode

    # -- the step -----------------------------------------------------------------

    def step(
        self, dt: float, truth_ne: tuple[float, float], velocity_ne, depth: float
    ) -> None:
        """Advance the estimate ``dt`` s; ``truth_ne`` is where the hull is,
        ``velocity_ne`` its velocity over the ground, ``depth`` its true depth."""
        self._sensors(depth)
        gps, locked = self.gps_on, self.dvl_locked
        mode = GPS if gps else DVL if locked else NONE
        if mode != self.mode:
            self.events += self._said(self.mode, mode)
            self.mode = mode
        if mode == GPS:
            off = self.gps_offset
            self.north, self.east = truth_ne[0] + off[0], truth_ne[1] + off[1]
        elif mode == DVL:
            sd = self._p("SIM_VICON_V_SD")
            vn = velocity_ne[0] + self._p("SIM_VICON_VGLI_X") + self.rng.gauss(0.0, sd)
            ve = velocity_ne[1] + self._p("SIM_VICON_VGLI_Y") + self.rng.gauss(0.0, sd)
            self.north += vn * dt
            self.east += ve * dt
        # no GPS and no lock: the position holds

    def _sensors(self, depth: float) -> None:
        lost = self._p("SIM_BARO_DISABLE") >= 0.5
        if lost and not self.baro_lost:
            sev, text = captured.depth_sensor_lost()
            self.events.append((sev, text))
        self.baro_lost = lost
        if not lost:
            self.depth = depth

    @staticmethod
    def _said(old: str, new: str) -> list[tuple[int, str]]:
        if old == GPS:
            return captured.gps_lost_texts()
        if new == GPS:
            return captured.gps_back_texts()
        return []

    # -- what the estimate says -----------------------------------------------------

    @property
    def estimate(self) -> tuple[float, float]:
        """The estimated position, (north, east) m from home."""
        if self.mode == DVL:
            return self.north + self._p("SIM_VICON_GLIT_X"), self.east + self._p(
                "SIM_VICON_GLIT_Y"
            )
        return self.north, self.east

    def gps_position(self, truth_ne: tuple[float, float]) -> tuple[float, float]:
        """Where the GPS says the hull is: the truth and the offset."""
        off = self.gps_offset
        return truth_ne[0] + off[0], truth_ne[1] + off[1]

    def ekf_flags(self, ready: bool) -> int:
        """EKF_STATUS_REPORT flags: the captured ones of the estimator's state; attitude
        alone until ``ready`` (the position estimate is valid)."""
        return captured.state(self.mode)["ekf_flags"] if ready else 1

    def sensor_words(self) -> tuple[int, int, int]:
        """SYS_STATUS present, enabled and health: the captured ones of the state, with
        the rangefinder only when one is fitted and the depth sensor lost when it is."""
        s = captured.state(self.mode)
        present, enabled, health = (
            s["sensors_present"],
            s["sensors_enabled"],
            s["sensors_health"],
        )
        if self._p("RNGFND1_TYPE") == 0:
            mask = ~captured.LASER_POSITION
            present, enabled, health = present & mask, enabled & mask, health & mask
        if self.baro_lost:
            health &= ~captured.depth_lost_clear_mask()
        return present, enabled, health

    def gps_fix(self) -> tuple[int, int, int]:
        """(fix type, satellites, eph in cm) of the GPS."""
        s = captured.state(GPS if self.gps_on else NONE)
        return s["gps_fix_type"], s["gps_satellites"], s["gps_eph_cm"]
