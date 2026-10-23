"""``torpedo-check <model.json>``: fly the acceptance scenarios on a hull in-process.

It prints one line per measure against its line and exits 1 if any line is breached.
The unit tests call :func:`measure` on the test hulls; a deployment calls the console
script on the model file it mounts. The scenarios (each at hundreds of times real
time, no MAVLink):

* a 300 m lane, a 90 degree turn and a second lane, at 1.54 and 2.30 m/s;
* three 300 m lanes 40 m apart at 1.5 m/s (two 90 degree turns between lanes);
* a ring of 30 m at 2 m/s, joined from 100 m out: the median radius error over a turn
  once on it, and the mean from the first point within 10 m of it (as the vehicle
  interface measures an orbit);
* a hold ring of 18 m, entered at 1.9 m/s;
* the radius of a full-rudder turn at cruise speed, against the file's.

A ring smaller than the hull's least loiter radius is raised to it (the guidance does).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass

from . import measures, trials
from .control import Control
from .guidance import LOITER_MIN
from .model import Model, ModelError, load

RUN_OUT_M = 20.0
XTE_M = measures.XTE_M
SIGN_CHANGES = 1
SETTLE_S = 30.0
SPEED_RMS = 0.1
HEADING_RMS_DEG = 2.0
ORBIT_M = 2.0
ORBIT_RING_M = 30.0
ORBIT_SPEED = 2.0
HOLD_RING_M = 18.0
HOLD_SPEED = 1.9
HOLD_OVER_M = 10.0
JOIN_M = 10.0  # m: the vehicle interface measures a ring from this close to it
RADIUS_TOL = 0.20
SPEEDS = (1.54, 2.30)  # 3 knots, and a small torpedo's top speed
UTURN_SPEED = 1.5


@dataclass(frozen=True)
class Line:
    """One acceptance line: a measure that must be at most ``limit``."""

    key: str
    limit: float
    unit: str
    what: str


LINES = (
    Line(
        "run_out_90_slow", RUN_OUT_M, "m", "90 degree turn at 1.54 m/s, past lane end"
    ),
    Line(
        "run_out_90_fast", RUN_OUT_M, "m", "90 degree turn at 2.30 m/s, past lane end"
    ),
    Line("run_out_uturn", RUN_OUT_M, "m", "lanes 40 m apart at 1.5 m/s, past lane end"),
    Line("xte_max", XTE_M, "m", "cross-track error on a lane's settled part"),
    Line("sign_changes", SIGN_CHANGES, "", "cross-track sign changes (no S-track)"),
    Line("settle_s", SETTLE_S, "s", "corner until within 2 m of the next lane"),
    Line("speed_rms", SPEED_RMS, "m/s", "speed error on a straight, rms"),
    Line("heading_rms", HEADING_RMS_DEG, "deg", "heading error on a straight, rms"),
    Line("orbit_median", ORBIT_M, "m", "ring of 30 m at 2 m/s, median radius error"),
    Line("orbit_mean", ORBIT_M, "m", "the ring from 10 m off it, mean radius error"),
    Line("hold_over", HOLD_OVER_M, "m", "hold ring at 1.9 m/s, farthest past the ring"),
    Line(
        "hold_median", ORBIT_M, "m", "hold ring, median radius error after a half turn"
    ),
    Line(
        "radius_error",
        RADIUS_TOL,
        "",
        "full-rudder turn radius, fraction off the file's",
    ),
)


def _lanes(points: list, step: int) -> list:
    """The lines of a route as lanes: every ``step``-th, from the first."""
    return [(points[i], points[i + 1]) for i in range(0, len(points) - 1, step)]


def _straights(trace, points, speed, ks):
    """(xte worst, sign changes, speed rms, heading rms) over the lines ``ks``."""
    xte = signs = spd = hdg = 0.0
    for k in ks:
        part = [x for _, x in measures.straight(trace, points, k)]
        xte = max(xte, max((abs(x) for x in part), default=math.inf))
        signs = max(signs, measures.sign_changes(part))
        s_err, h_err = measures.speed_heading_rms(trace, points, k, speed)
        spd, hdg = max(spd, s_err), max(hdg, h_err)
    return xte, signs, spd, hdg


def _single_turns(model: Model, out: dict) -> None:
    """A lane, a 90 degree turn and a lane, at two speeds."""
    worst = dict.fromkeys(
        ("xte_max", "sign_changes", "settle_s", "speed_rms", "heading_rms"), 0.0
    )
    for key, speed in zip(("run_out_90_slow", "run_out_90_fast"), SPEEDS):
        points, trace = trials.turn90(model, speed)
        out[key] = measures.run_out(trace, _lanes(points, 1))[0]
        speed = Control(model).speed_demand(speed)  # the hull cannot do more
        xte, signs, spd, hdg = _straights(trace, points, speed, (0, 1))
        settle = measures.settle_time(trace, points, 1)
        got = dict(zip(worst, (xte, signs, settle, spd, hdg)))
        worst = {k: max(worst[k], got[k]) for k in worst}
    out |= worst


def _uturn(model: Model, out: dict) -> None:
    """Three lanes 40 m apart: the turn reaches no farther than the line, and the
    lanes are not flown in an S."""
    points, trace = trials.lawnmower(model, UTURN_SPEED)
    out["run_out_uturn"] = max(measures.run_out(trace, _lanes(points, 2))[:-1])
    for k in (0, 2, 4):
        part = [x for _, x in measures.straight(trace, points, k)]
        out["sign_changes"] = max(out["sign_changes"], measures.sign_changes(part))


def _ring(model: Model, asked: float) -> float:
    """``asked`` raised to the hull's least loiter radius."""
    return max(asked, LOITER_MIN * model.turn_radius_m)


def _circles_trial(model: Model, out: dict) -> None:
    """The orbit and the hold."""
    ring = _ring(model, ORBIT_RING_M)
    start, heading = (-100.0, 0.0), 0.0
    trace = trials.circle(model, (0.0, 0.0), ring, ORBIT_SPEED, start, heading, 600.0)
    joined = next(
        s.t for s in trace.samples if abs(math.hypot(s.north, s.east) - ring) <= XTE_M
    )
    turn = 2 * math.pi * ring / ORBIT_SPEED
    lap = [s for s in trace.samples if joined <= s.t <= joined + turn]
    out["orbit_median"] = measures.median_radius_error(lap, (0.0, 0.0), ring)
    near = next(
        s.t for s in trace.samples if abs(math.hypot(s.north, s.east) - ring) <= JOIN_M
    )
    lap = [s for s in trace.samples if near <= s.t <= near + turn]
    out["orbit_mean"] = statistics.mean(
        abs(math.hypot(s.north, s.east) - ring) for s in lap
    )
    ring = _ring(model, HOLD_RING_M)
    trace = trials.circle(
        model, (0.0, 0.0), ring, HOLD_SPEED, (-100.0, -30.0), 0.0, 400.0
    )
    entered = next(
        s.t
        for s in trace.samples
        if math.hypot(s.north, s.east) <= ring + model.turn_radius_m
    )
    after = [s for s in trace.samples if s.t >= entered + math.pi * ring / HOLD_SPEED]
    out["hold_over"] = max(math.hypot(s.north, s.east) for s in after) - ring
    out["hold_median"] = measures.median_radius_error(after, (0.0, 0.0), ring)


def measure(model: Model) -> dict[str, float]:
    """Every measure of :data:`LINES` for ``model``, by key."""
    out: dict[str, float] = {}
    _single_turns(model, out)
    _uturn(model, out)
    _circles_trial(model, out)
    radius, _ = trials.steady_turn(model)
    out["radius_m"] = radius
    out["radius_error"] = abs(radius - model.turn_radius_m) / model.turn_radius_m
    return out


def report(model: Model) -> tuple[list[str], bool]:
    """(one text line per measure, whether every line holds) for ``model``."""
    values = measure(model)
    rows, ok = [], True
    for line in LINES:
        value = values[line.key]
        good = value <= line.limit
        ok = ok and good
        rows.append(
            f"{'ok  ' if good else 'FAIL'} {line.key:16} {value:8.2f}"
            f" <= {line.limit:g} {line.unit:4} {line.what}"
        )
    rows.append(
        f"     full-rudder radius {values['radius_m']:.1f} m,"
        f" the file says {model.turn_radius_m:g} m"
    )
    return rows, ok


def main(argv: list[str] | None = None) -> int:
    """The console script. Exit 0 when every line holds, 1 when one does not, 2 when
    the file cannot be read."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("model", help="a schema-1 model file")
    args = ap.parse_args(argv)
    try:
        model = load(args.model)
    except (ModelError, OSError) as e:
        print(f"torpedo-check: {e}", file=sys.stderr)
        return 2
    rows, ok = report(model)
    print(f"hull {model.id}")
    print("\n".join(rows))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
