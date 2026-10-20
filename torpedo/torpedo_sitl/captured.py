"""What the real ArduSub says, captured once from its SITL image.

``ardusub_surface.json`` is written by ``tools/capture_ardusub.py`` from the
ArduSub SITL image: every parameter's name and default, the ``STATUSTEXT`` lines for
arming, a refused mode change, the GCS failsafe and its clearing, the battery
failsafes, a lost depth sensor and a lost GPS, and the ``EKF_STATUS_REPORT`` flags
and ``SYS_STATUS`` sensor bits in three states (a GPS fix; the GPS off with a DVL's
velocity; neither). The front seat uses them, so its words and numbers are ArduSub's own
and a client that reads them (a failsafe's name, an EKF's health) is not told
something ArduSub would not say.
"""

from __future__ import annotations

import json
import re
from functools import lru_cache
from pathlib import Path

PATH = Path(__file__).with_name("ardusub_surface.json")
LASER_POSITION = 0x100  # SYS_STATUS: the downward rangefinder
ABSOLUTE_PRESSURE = 0x8  # SYS_STATUS: ArduSub's external depth sensor


@lru_cache(maxsize=1)
def data() -> dict:
    """The captured document."""
    return json.loads(PATH.read_text())


def params() -> dict[str, float]:
    """Every ArduSub parameter and its default, in index order."""
    return dict(data()["params"])


def state(name: str) -> dict:
    """The EKF and sensor words of a state: ``gps``, ``dvl`` or ``none``."""
    return data()["states"][name]


def _lines(key, *path) -> list[tuple[int, str]]:
    node = data()["statustext"][key]
    for step in path:
        node = node[step]
    return [(int(s), t) for s, t in node]


def _first(lines: list[tuple[int, str]], what: str) -> tuple[int, str]:
    if not lines:
        raise KeyError(f"nothing captured for {what}")
    return lines[0]


def arm_text() -> tuple[int, str]:
    """The line ArduSub says when it arms: (severity, text)."""
    return _first(_lines("arm"), "arming")


def disarm_text() -> tuple[int, str]:
    """The line when it disarms."""
    return _first(_lines("disarm"), "disarming")


def guided_refused() -> tuple[int, str]:
    """The line when GUIDED is refused for want of a position."""
    for attempt in data()["statustext"]["early"]["attempts"]:
        for sev, text in attempt["texts"]:
            if text.startswith("Mode change failed"):
                return int(sev), text
    raise KeyError("nothing captured for a refused GUIDED")


def auto_refused() -> tuple[int, str]:
    """The line when AUTO is refused (no mission)."""
    return _first(_lines("auto_without_mission"), "a refused AUTO")


def gcs_lost(action: int) -> tuple[int, str]:
    """The line when the GCS heartbeat is lost, for an ``FS_GCS_ENABLE`` action."""
    node = data()["statustext"]["gcs_failsafe"][
        str(action if action in (1, 3, 4) else 1)
    ]
    return _first([(int(s), t) for s, t in node["on"]["texts"]], "a lost GCS")


def gcs_cleared() -> tuple[int, str]:
    """The line when the GCS heartbeat is back."""
    node = data()["statustext"]["gcs_failsafe"]["1"]["cleared"]["texts"]
    return _first([(int(s), t) for s, t in node], "a cleared GCS failsafe")


def battery_template(level: str) -> tuple[int, str]:
    """The battery failsafe's line (``low`` or ``critical``) as a format string with
    ``{n}``, ``{volts:.2f}`` and ``{mah}``."""
    sev, text = _first(_lines("battery", level), f"a {level} battery")
    return sev, re.sub(
        r"(\d+(?:\.\d+)?)V used (\d+) mAh", "{volts:.2f}V used {mah} mAh", text
    )


def depth_sensor_lost() -> tuple[int, str]:
    """The line when the depth sensor is lost."""
    node = data()["statustext"]["depth_sensor_lost"]["texts"]
    return _first([(int(s), t) for s, t in node], "a lost depth sensor")


def pilot_input(action: int) -> dict:
    """What ArduSub 4.7.1 showed when manual control stopped arriving in MANUAL, for an
    ``FS_PILOT_INPUT`` action: no line, no CRITICAL, no disarm (``texts`` is empty).
    The front seat follows the parameter's documented meaning (1 warn, 2 disarm)
    instead, and says nothing for a warning, as ArduSub did."""
    return data()["statustext"]["pilot_input"][str(action)]["on"]


def gps_lost_texts() -> list[tuple[int, str]]:
    """The lines when the GPS goes off, in order."""
    return [(int(s), t) for s, t in data()["states"]["none"]["texts"]]


def gps_back_texts() -> list[tuple[int, str]]:
    """The lines when the GPS is back, in order."""
    return _lines("gps_back")


def depth_lost_clear_mask() -> int:
    """The SYS_STATUS health bits a lost depth sensor clears."""
    gps = state("gps")["sensors_health"]
    lost = data()["statustext"]["depth_sensor_lost"]["sensors_health"]
    return gps & ~lost


def gps_lost_clear_mask() -> int:
    """The SYS_STATUS health bits a lost GPS clears."""
    return state("gps")["sensors_health"] & ~state("none")["sensors_health"]
