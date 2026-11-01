"""Capture what the real ArduSub does with a terrain-frame mission on its rangefinder.

Run on a host with Docker; it starts the ArduSub SITL image once (frame vectored, at
2x, under its own container name) and removes it:

    python3 torpedo/tools/capture_terrain.py [--image IMAGE] [--out FILE]

and writes ``torpedo/torpedo_sitl/ardusub_terrain.json``. The seabed is the SITL's
flat floor raised by ``SIM_SONAR_POS_Z`` (a bed at ``50 - SIM_SONAR_POS_Z`` m); the
mission is two waypoints 4 m above it (``MAV_FRAME_GLOBAL_TERRAIN_ALT_INT``), the
first near (ArduSub's waypoint controller moves the depth along each line).

* ``arm_refused``: the arming command's answer and lines with the parameter checks on
  (``ARMING_SKIPCHK`` without its parameters bit) and a terrain mission stored;
  ``prearm``: the lines the periodic pre-arm check says meanwhile;
  ``arm_default``: the answer with ArduSub's own ``ARMING_SKIPCHK``;
* ``failsafe``: for each ``FS_TERRAIN_ENAB`` action (0, 1, 2), the lines once the
  rangefinder reads past ``RNGFND1_MAX`` in AUTO, the mode and armed state after,
  and the shallowest and deepest depth from the cut to the end;
* ``recovered``: the lines when the reading comes back within the recovery time;
* ``guided``: how far the vehicle went toward a terrain-frame GUIDED target sent
  with a velocity, and toward one sent with a position only.

With ``--inside`` the script is the half that runs in the container; the host half
copies it and ``capture_ardusub.py`` (whose ``Probe`` it uses) in and runs it.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE = "ghcr.io/murphy360/ardupilot-sitl-docker-sub:latest"
OUT = os.path.join(HERE, "..", "torpedo_sitl", "ardusub_terrain.json")
SPEEDUP = 2
HOME = (42.3898, -71.1476)
BED_M = 10.0  # the seabed's depth for the capture
HEIGHT_M = 4.0  # the height the mission asks
SETTLE_S = 60.0  # vehicle time for AUTO to reach its height before a cut
PARAMETERS_CHECK = 1 << 5  # AP_Arming::Check::PARAMETERS
TERRAIN_ALT_INT = 11
START = (
    "printf 'SIM_OPOS_LAT {lat}\\nSIM_OPOS_LNG {lon}\\nSIM_OPOS_ALT 0\\n"
    "SIM_OPOS_HDG 0\\nSIM_SONAR_POS_Z {pos_z}\\n' > /tmp/opos.parm && exec "
    "/ardupilot/Tools/autotest/sim_vehicle.py --vehicle ArduSub -I0 "
    "--add-param-file=/tmp/opos.parm -w --frame vectored --no-rebuild --no-mavproxy "
    "--speedup {speedup} --sysid 1"
)


# ---------------------------------------------------------------- the host half
def run(*cmd, check=True):
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


def main_host(args) -> None:
    name = f"torpedo-capture-terrain-{os.getpid()}"
    script = START.format(lat=HOME[0], lon=HOME[1], pos_z=50.0 - BED_M, speedup=SPEEDUP)
    run("docker", "run", "-d", "--name", name, "--entrypoint", "/bin/sh", args.image,
        "-c", script)  # fmt: skip
    try:
        for f in ("capture_ardusub.py", "capture_terrain.py"):
            run("docker", "cp", os.path.join(HERE, f), f"{name}:/tmp/{f}")
        out = run("docker", "exec", name, "python3", "/tmp/capture_terrain.py",
                  "--inside")  # fmt: skip
        sys.stderr.write(out.stderr)
        doc = json.loads(out.stdout)
    finally:
        run("docker", "rm", "-f", name, check=False)
    doc["image"] = args.image
    with open(args.out, "w") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    print(f"wrote {args.out}")


# ------------------------------------------------------------ the container half
def ll(north: float, east: float = 0.0) -> tuple[int, int]:
    """Latitude and longitude (1e7) of a point ``north`` m, ``east`` m from home."""
    lat = HOME[0] + north / 111_320.0
    lon = HOME[1] + east / (111_320.0 * math.cos(math.radians(HOME[0])))
    return int(lat * 1e7), int(lon * 1e7)


def upload(p, items) -> int:
    """Upload ``items`` ((frame, command, north, alt), item 0 first): the ack type."""
    p.m.mav.mission_count_send(1, 1, len(items), 0)
    end = time.time() + 30
    while time.time() < end:
        msg = p.m.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            return int(msg.type)
        frame, command, north, alt = items[msg.seq]
        lat, lon = ll(north)
        p.m.mav.mission_item_int_send(
            1, 1, msg.seq, frame, command, 0, 1, 0, 0, 0, 0, lat, lon, alt, 0
        )
    return -1


def mission(p) -> int:
    M = p.M
    wp = M.MAV_CMD_NAV_WAYPOINT
    return upload(
        p,
        [
            (M.MAV_FRAME_GLOBAL, wp, 0.0, 0.0),
            (TERRAIN_ALT_INT, wp, 30.0, HEIGHT_M),  # short: down to its height soon
            (TERRAIN_ALT_INT, wp, 800.0, HEIGHT_M),
        ],
    )


def depth(p) -> float | None:
    g = p.last.get("GLOBAL_POSITION_INT")
    return None if g is None else -g.relative_alt / 1000.0


def north(p) -> float | None:
    g = p.last.get("GLOBAL_POSITION_INT")
    return None if g is None else (g.lat / 1e7 - HOME[0]) * 111_320.0


def watch(p, seconds: float) -> list[float]:
    """Pump ``seconds`` of wall time; the depths seen."""
    seen, end = [], time.time() + seconds
    while time.time() < end:
        p.pump(0.2)
        d = depth(p)
        if d is not None:
            seen.append(d)
    return seen


def wait_text(p, mark: int, text: str, seconds: float) -> bool:
    end = time.time() + seconds
    while time.time() < end:
        p.pump(0.1)
        if any(t == text for _, t in p.texts[mark:]):
            return True
    return False


def start_auto(p) -> None:
    """At the surface over the bed: arm, AUTO from item 1, and let it settle."""
    p.param("SIM_SONAR_POS_Z", 50.0 - BED_M)
    p.mode(2)
    p.pump(1.0)
    p.command(p.M.MAV_CMD_DO_SET_MISSION_CURRENT, 1)
    p.arm(True)
    p.mode(3)
    watch(p, SETTLE_S / SPEEDUP)  # down to its height over the bed


def reset(p) -> None:
    p.param("SIM_SONAR_POS_Z", 50.0 - BED_M)
    p.arm(True)  # a disarmed sub floats up slowly: SURFACE takes it up
    p.mode(9)
    watch(p, 30.0 / SPEEDUP)
    p.arm(False)
    p.pump(1.0)


def failsafe(p, action: int) -> dict:
    p.param("FS_TERRAIN_ENAB", action)
    start_auto(p)
    cut_at = depth(p)
    mark = p.mark()
    p.param("SIM_SONAR_POS_Z", -1000.0)  # the reading goes past RNGFND1_MAX
    seen = watch(p, 16.0 / SPEEDUP)
    hb = p.last["HEARTBEAT"]
    out = {
        "texts": p.since(mark),
        "custom_mode": int(hb.custom_mode),
        "armed": bool(hb.base_mode & p.M.MAV_MODE_FLAG_SAFETY_ARMED),
        "system_status": int(hb.system_status),
        "depth_at_cut": None if cut_at is None else round(cut_at, 2),
        "depth_min": round(min(seen), 2) if seen else None,
        "depth_max": round(max(seen), 2) if seen else None,
    }
    reset(p)
    return out


def recovered(p) -> dict:
    p.param("FS_TERRAIN_ENAB", 2)
    start_auto(p)
    mark = p.mark()
    p.param("SIM_SONAR_POS_Z", -1000.0)
    wait_text(p, mark, "Attempting auto failsafe recovery", 6.0 / SPEEDUP)
    p.param("SIM_SONAR_POS_Z", 50.0 - BED_M)
    wait_text(p, mark, "Terrain failsafe recovery successful!", 12.0 / SPEEDUP)
    p.pump(2.0 / SPEEDUP)
    hb = p.last["HEARTBEAT"]
    out = {"texts": p.since(mark), "custom_mode": int(hb.custom_mode)}
    reset(p)
    return out


def guided(p) -> dict:
    """GUIDED terrain targets 300 m north: with a velocity, then a position alone."""
    out = {}
    p.param("SIM_SONAR_POS_Z", 50.0 - BED_M)
    p.mode(4)
    p.arm(True)
    p.pump(1.0)
    lat, lon = ll(300.0)
    for key, mask in (("with_velocity", 0b110111000000), ("position", 0b110111111000)):
        start = north(p)
        end = time.time() + 30.0 / SPEEDUP
        while time.time() < end:
            p.m.mav.set_position_target_global_int_send(
                0, 1, 1, TERRAIN_ALT_INT, mask, lat, lon, HEIGHT_M,
                1.0, 0.0, 0.0, 0, 0, 0, 0, 0,
            )  # fmt: skip
            p.pump(0.5)
        out[key] = {"moved_north_m": round((north(p) or 0.0) - (start or 0.0), 1)}
        out[key]["depth"] = round(depth(p) or 0.0, 2)
    p.mode(9)
    watch(p, 20.0 / SPEEDUP)
    p.arm(False)
    return out


def inside() -> None:
    sys.path.insert(0, HERE)
    from capture_ardusub import Probe

    p = Probe()
    p.streams()
    p.m.mav.request_data_stream_send(1, 1, p.M.MAV_DATA_STREAM_POSITION, 5, 1)
    p.settled(lambda f: f & 16)
    skip = int(p.params().get("ARMING_SKIPCHK", 0))
    out = {"skipchk_default": skip, "mission_ack": mission(p)}
    p.param("ARMING_SKIPCHK", skip & ~PARAMETERS_CHECK)
    mark = p.mark()
    p.mode(2)
    res = p.arm(True)
    out["arm_refused"] = {"ack": res, "texts": p.since(mark)}
    if res == p.M.MAV_RESULT_ACCEPTED:
        p.arm(False)
    mark = p.mark()
    p.pump(35.0 / SPEEDUP)
    out["prearm"] = p.since(mark)
    p.param("ARMING_SKIPCHK", skip)
    mark = p.mark()
    out["arm_default"] = {"ack": p.arm(True), "texts": p.since(mark)}
    p.arm(False)
    p.pump(1.0)
    out["failsafe"] = {str(a): failsafe(p, a) for a in (0, 1, 2)}
    out["recovered"] = recovered(p)
    out["guided"] = guided(p)
    json.dump(out, sys.stdout)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image", default=IMAGE)
    ap.add_argument("--out", default=os.path.normpath(OUT))
    ap.add_argument("--inside", action="store_true")
    args = ap.parse_args()
    if args.inside:
        inside()
    else:
        main_host(args)


if __name__ == "__main__":
    main()
