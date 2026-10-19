"""Writes the two generic hulls the tests fly: ``torpedo-test-small.json`` and
``torpedo-test-large.json`` (schema 1), from slender-body textbook estimates.

Run it from the torpedo directory: ``PYTHONPATH=. python3 tests/models/make_fixtures.py``. The
hulls are invented here, not copies of any real vehicle's model. The coefficients are
the textbook estimates for a slender body of length L and diameter d in water of
1000 kg/m3, with A = pi d^2 / 4:

* drag: ``xUabsU = -0.5 rho 0.2 A``; the cross-flow ``yVabsV = zWabsW = -0.5 rho 1.1
  L d``; the rate damping ``mQabsQ = nRabsR = -0.5 rho 1.1 d L^4 / 32``;
  ``kPabsP = -0.01``. The last four (sway, heave, pitch and yaw damping) are taken
  ``APPENDAGE`` times: a bare cylinder is lightly damped, its Munk moment beats fins
  of this size and the hull cannot be flown at all (a real hull's fins, sensors and
  boundary layer damp it more, as the published figures for a small torpedo show);
* added mass: ``xDotU = -0.1 m``; ``yDotV = zDotW = -rho A L``; ``mDotQ = nDotR =
  -rho A L^3 / 12``; ``kDotP = -0.001``;
* the hull weighs 0.5 % less than the water it displaces, its inertia is a rod's, its
  centre of gravity is 0.1 d below its centre of buoyancy;
* four fins, each ``0.25 d^2``, lift slope 3.0, at most 0.24 rad at 2 rad/s, at
  ``-0.42 L``.

The turning radius is measured with ``torpedo_sitl.trials.steady_turn`` (full rudder at
cruise speed) and written, so the fixture agrees with the physics.
"""

import json
import math
from pathlib import Path

from torpedo_sitl import model, trials

HERE = Path(__file__).resolve().parent
RHO = 1000.0
APPENDAGE = 8.0  # the cross-flow and rate damping, over the bare cylinder's
HULLS = {
    "torpedo-test-small": (2.0, 0.20, 50.0),
    "torpedo-test-large": (3.5, 0.35, 300.0),
}


def build(name: str, length: float, d: float, mass: float) -> dict:
    """The schema-1 document of one hull, its turning radius not yet measured."""
    area = math.pi * d * d / 4
    drag = -0.5 * RHO * 0.2 * area
    side = -0.5 * RHO * 1.1 * length * d * APPENDAGE
    rate = -0.5 * RHO * 1.1 * d * length**4 / 32 * APPENDAGE
    return {
        "schema": 1,
        "id": name,
        "source": "Written by tests/models/make_fixtures.py. Do not edit.",
        "length_m": length,
        "diameter_m": d,
        "mass_kg": mass,
        "volume_m3": round(mass * 1.005 / RHO, 6),
        "water_kg_m3": RHO,
        "ixx": round(mass * d * d / 8, 4),
        "iyy": round(mass * length**2 / 12, 4),
        "izz": round(mass * length**2 / 12, 4),
        "zg_m": round(0.1 * d, 6),
        "hydro": {
            "xUabsU": round(drag, 4),
            "yVabsV": round(side, 4),
            "zWabsW": round(side, 4),
            "kPabsP": -0.01,
            "mQabsQ": round(rate, 4),
            "nRabsR": round(rate, 4),
            "xDotU": round(-0.1 * mass, 4),
            "yDotV": round(-RHO * area * length, 4),
            "zDotW": round(-RHO * area * length, 4),
            "kDotP": -0.001,
            "mDotQ": round(-RHO * area * length**3 / 12, 4),
            "nDotR": round(-RHO * area * length**3 / 12, 4),
        },
        "fin": {
            "area_m2": round(0.25 * d * d, 6),
            "span_m": round(0.5 * d, 4),
            "chord_m": round(0.5 * d, 4),
            "x_m": round(-0.42 * length, 4),
            "lift_slope": 3.0,
            "max_rad": 0.24,
            "rate_rad_s": 2.0,
        },
        "max_thrust_n": round(-drag * 2.2**2 * 1.3, 2),
        "steer_speed_mps": 1.0,
        "max_pitch_rad": 0.45,
        "max_speed_mps": 2.2,
        "cruise_speed_mps": 1.5,
        "turn_radius_m": 1.0,
        "depth_limit_m": 100,
        "endurance_h": 10,
    }


def main() -> None:
    """Write both files."""
    for name, (length, d, mass) in HULLS.items():
        doc = build(name, length, d, mass)
        radius, _ = trials.steady_turn(model.from_dict(doc))
        doc["turn_radius_m"] = round(radius, 1)
        path = HERE / f"{name}.json"
        path.write_text(json.dumps(doc, indent=2) + "\n")
        print(f"wrote {path.name}: turn radius {doc['turn_radius_m']} m")


if __name__ == "__main__":
    main()
