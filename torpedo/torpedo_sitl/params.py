"""The parameter store: names, values, start-up lines and the saved file.

Defaults are written at start: first the parameters captured from the real ArduSub
image (``captured.py``), in ArduSub's index order; then the documented ArduSub defaults
for the few names an older or newer ArduSub lacks (the plugin reads them by every name
it has been through); then what the hull and the environment decide (identity, the
arrival radius, the depth limit, the start position, ``TPDO_*``). ``PARAM_SET`` changes a value and saves it to
``/tmp/torpedo-<INSTANCE>.parm``; the file is cleared when the process starts (as the
SITL's ``-w`` wipes its saved parameters) and read back after a reboot. Start-up lines
come from the ``SITL_PARAMS`` environment variable and ``--param-file`` files, one
``NAME VALUE`` per line (or separated by commas or semicolons in the variable). A name
nobody knows is kept and answered, like a known one.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

from . import captured
from .guidance import LOITER_MIN, LOS_LA, TURN_K
from .model import Model

READ_ONLY = frozenset({"TPDO_MODEL"})
MAX_NAME = 16


def model_number(model_id: str) -> float:
    """The number a generic id ends in (100 for ``torpedo-100``), else 0."""
    found = re.search(r"(\d+)\s*$", model_id)
    return float(found.group(1)) if found else 0.0


def defaults(
    model: Model,
    sysid: int,
    lat: float,
    lon: float,
    alt: float,
    heading: float,
    batt_mah: float,
) -> dict[str, float]:
    """Every parameter and its start value: the captured ArduSub ones, the documented
    ones the capture lacks, and what the hull and the environment decide."""
    out = captured.params()
    for name, value in documented(
        model, sysid, lat, lon, alt, heading, batt_mah
    ).items():
        if name in MODEL_DERIVED or name not in out:
            out[name] = value
    return out


MODEL_DERIVED = frozenset(
    {
        "SYSID_THISMAV", "MAV_SYSID", "WP_RADIUS_M", "WPNAV_RADIUS", "WPNAV_RADIUS_M",
        "WP_RADIUS", "FENCE_ALT_MIN", "BATT_CAPACITY", "SCR_ENABLE", "SIM_OPOS_LAT",
        "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG", "TPDO_MODEL", "TPDO_LOS_LA",
        "TPDO_TURN_K", "TPDO_LOIT_MIN",
    }
)  # fmt: skip


def documented(
    model: Model,
    sysid: int,
    lat: float,
    lon: float,
    alt: float,
    heading: float,
    batt_mah: float,
) -> dict[str, float]:
    """The parameters the plugin reads for a sub, the stand-in's own and the
    simulator's, with ArduSub 4.7's documented defaults: what the capture lacks."""
    r = model.turn_radius_m
    return {
        # identity and arrival
        "SYSID_THISMAV": float(sysid),
        "MAV_SYSID": float(sysid),
        "ARMING_CHECK": 1.0,
        "ARMING_SKIPCHK": 0.0,
        "WP_RADIUS_M": r,
        "WPNAV_RADIUS": r * 100.0,
        "WPNAV_RADIUS_M": r,
        "WP_RADIUS": r,
        # failsafe
        "FS_GCS_ENABLE": 0.0,
        "FS_GCS_TIMEOUT": 5.0,
        "SYSID_MYGCS": 255.0,
        "MAV_GCS_SYSID": 255.0,
        "MAV_GCS_SYSID_HI": 255.0,
        "SYSID_ENFORCE": 0.0,
        "MAV_OPTIONS": 0.0,
        "PDEAD_ACTION": 0.0,
        "PDEAD_HOLD_S": 120.0,  # the script's own default
        "SCR_ENABLE": 1.0,
        # sub
        "FENCE_ENABLE": 0.0,
        "FENCE_TYPE": 3.0,
        "FENCE_ACTION": 0.0,
        "FENCE_ALT_MIN": -model.depth_limit_m,
        "FENCE_RADIUS": 300.0,
        "FENCE_MARGIN": 2.0,
        "BATT_MONITOR": 4.0,
        "BATT_CAPACITY": batt_mah,
        "BATT_LOW_MAH": 0.0,
        "BATT_LOW_VOLT": 0.0,
        "BATT_FS_LOW_ACT": 0.0,
        "BATT_CRT_MAH": 0.0,
        "BATT_CRT_VOLT": 0.0,
        "BATT_FS_CRT_ACT": 0.0,
        "FS_LEAK_ENABLE": 1.0,
        "LEAK1_PIN": -1.0,
        "FS_PILOT_INPUT": 1.0,
        "FS_PILOT_TIMEOUT": 3.0,
        "BARO_EXT_BUS": 1.0,
        "RNGFND1_TYPE": 0.0,
        "RNGFND1_MAX_CM": 700.0,
        "RNGFND1_MIN_CM": 20.0,
        "RNGFND1_MAX": 7.0,
        "RNGFND1_MIN": 0.2,
        "EK3_SRC1_VELXY": 3.0,
        "WPNAV_SPEED_UP": 50.0,
        "PILOT_SPEED_UP": 500.0,
        "EK3_POS_I_GATE": 500.0,
        "EK3_HGT_I_GATE": 500.0,
        "EK3_GLITCH_RAD": 25.0,
        "EK3_ALT_M_NSE": 3.0,
        # the simulator
        "SIM_OPOS_LAT": lat,
        "SIM_OPOS_LNG": lon,
        "SIM_OPOS_ALT": alt,
        "SIM_OPOS_HDG": heading,
        # its own
        "TPDO_MODEL": model_number(model.id),
        "TPDO_LOS_LA": LOS_LA,
        "TPDO_TURN_K": TURN_K,
        "TPDO_LOIT_MIN": LOITER_MIN,
        # the older name of the GPS switch
        "SIM_GPS_DISABLE": 0.0,
    }


def parse_lines(text: str) -> list[tuple[str, float]]:
    """The ``NAME VALUE`` pairs of a parameter file or of ``SITL_PARAMS``. Lines
    may also be separated by commas or semicolons; ``#`` starts a comment; a line that
    is not a name and a number is skipped."""
    out = []
    for raw in re.split(r"[\n;]", text):
        line = raw.split("#", 1)[0].strip()
        parts = re.split(r"[\s,=]+", line)
        if len(parts) != 2 or not parts[0] or len(parts[0]) > MAX_NAME:
            continue
        try:
            value = float(parts[1])
        except ValueError:
            continue
        if math.isfinite(value):
            out.append((parts[0], value))
    return out


class ParamStore:
    """The parameters of one front seat."""

    def __init__(self, initial: dict[str, float], path: Path | None = None):
        self.values: dict[str, float] = dict(initial)
        self.path = path
        self.saved: dict[str, float] = {}
        if path is not None and path.exists():
            path.unlink()  # cleared at start

    def get(self, name: str, default: float = 0.0) -> float:
        """The value of ``name``."""
        return self.values.get(name, default)

    def names(self) -> list[str]:
        """Every name, in index order."""
        return list(self.values)

    def index(self, name: str) -> int:
        """The index of ``name``, -1 when unknown."""
        for i, n in enumerate(self.values):
            if n == name:
                return i
        return -1

    def load(self, pairs: list[tuple[str, float]]) -> None:
        """Start-up lines: they set the value and are not saved."""
        for name, value in pairs:
            if name not in READ_ONLY:
                self.values[name] = value

    def set(self, name: str, value: float) -> bool:
        """``PARAM_SET``: change a known parameter and save it. False when the name
        is unknown or read only (the caller answers the current value)."""
        if name not in self.values or name in READ_ONLY:
            return False
        self.values[name] = value
        self.saved[name] = value
        self._write()
        return True

    def reload(self) -> None:
        """A reboot: the saved values win over the start-up ones."""
        if self.path is not None and self.path.exists():
            for name, value in parse_lines(self.path.read_text()):
                if name in self.values:
                    self.values[name] = value

    def _write(self) -> None:
        if self.path is None:
            return
        self.path.write_text("".join(f"{n} {v:.9g}\n" for n, v in self.saved.items()))
