"""A hull's model file, schema 1: what the physics and the control need of a hull.

``load(path)`` reads one file and returns a frozen :class:`Model`. A missing key, a
value that is not a number, or a schema other than 1 raises :class:`ModelError` naming
the key. Extra keys are ignored: a file may carry notes for people.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

SCHEMA = 1
HYDRO_KEYS = (
    "xUabsU yVabsV zWabsW kPabsP mQabsQ nRabsR xDotU yDotV zDotW kDotP mDotQ nDotR"
).split()
FIN_KEYS = "area_m2 span_m chord_m x_m lift_slope max_rad rate_rad_s".split()
NUMBER_KEYS = (
    "length_m diameter_m mass_kg volume_m3 water_kg_m3 ixx iyy izz zg_m max_thrust_n "
    "steer_speed_mps max_pitch_rad max_speed_mps cruise_speed_mps turn_radius_m "
    "depth_limit_m endurance_h"
).split()


class ModelError(ValueError):
    """The model file is not a schema-1 file; the message names the key."""


@dataclass(frozen=True)
class Hydro:
    """The hydrodynamic coefficients, by Fossen's names (gz-sim's, too): drag
    ``xUabsU`` (N per (m/s)^2), added masses ``xDotU`` (kg) and so on. All negative."""

    xUabsU: float
    yVabsV: float
    zWabsW: float
    kPabsP: float
    mQabsQ: float
    nRabsR: float
    xDotU: float
    yDotV: float
    zDotW: float
    kDotP: float
    mDotQ: float
    nDotR: float


@dataclass(frozen=True)
class Fin:
    """One fin of a cross of four (a pair steers one plane)."""

    area_m2: float
    span_m: float
    chord_m: float
    x_m: float  # behind the centre of buoyancy: negative
    lift_slope: float  # per radian
    max_rad: float
    rate_rad_s: float


@dataclass(frozen=True)
class Model:
    """A hull. Lengths in m, masses in kg, angles in rad."""

    id: str
    length_m: float
    diameter_m: float
    mass_kg: float
    volume_m3: float
    water_kg_m3: float
    ixx: float
    iyy: float
    izz: float
    zg_m: float
    hydro: Hydro
    fin: Fin
    max_thrust_n: float
    steer_speed_mps: float
    max_pitch_rad: float
    max_speed_mps: float
    cruise_speed_mps: float
    turn_radius_m: float
    depth_limit_m: float
    endurance_h: float


def _number(doc: dict, key: str, where: str = "") -> float:
    if key not in doc:
        raise ModelError(f"missing key {where}{key}")
    value = doc[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ModelError(f"{where}{key} is not a number")
    if not math.isfinite(value):
        raise ModelError(f"{where}{key} is not finite")
    return float(value)


def _section(doc: dict, name: str, keys: list[str]) -> dict:
    if not isinstance(doc.get(name), dict):
        raise ModelError(f"missing key {name}")
    return {k: _number(doc[name], k, f"{name}.") for k in keys}


def from_dict(doc: dict) -> Model:
    """The model of a decoded schema-1 document."""
    if not isinstance(doc, dict):
        raise ModelError("the file is not an object")
    if doc.get("schema") != SCHEMA:
        raise ModelError(f"schema is {doc.get('schema')!r}, not {SCHEMA}")
    if not isinstance(doc.get("id"), str) or not doc["id"]:
        raise ModelError("missing key id")
    numbers = {k: _number(doc, k) for k in NUMBER_KEYS}
    hydro = Hydro(**_section(doc, "hydro", HYDRO_KEYS))
    fin = Fin(**_section(doc, "fin", FIN_KEYS))
    return Model(id=doc["id"], hydro=hydro, fin=fin, **numbers)


def load(path: str | Path) -> Model:
    """The model in the schema-1 JSON file at ``path``."""
    try:
        doc = json.loads(Path(path).read_text())
    except json.JSONDecodeError as e:
        raise ModelError(f"{path} is not JSON: {e}") from e
    return from_dict(doc)
