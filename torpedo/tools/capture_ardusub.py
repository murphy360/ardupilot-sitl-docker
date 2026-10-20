"""Capture what the real ArduSub says, for the torpedo front seat to say the same.

Run on a host with Docker (nothing else is needed); it starts the ArduSub SITL image
twice, in turn, each time at 5x and with its own container name, and removes it:

    python3 torpedo/tools/capture_ardusub.py [--image IMAGE] [--out FILE]

and writes ``torpedo/torpedo_sitl/ardusub_surface.json``:

* ``params``: every parameter's name and default (``PARAM_REQUEST_LIST``);
* ``statustext``: the lines for arming and disarming, a pre-arm refusal, the GCS
  failsafe and its clearing (for each ``FS_GCS_ENABLE`` action), the battery failsafes
  and a lost depth sensor;
* ``states``: ``EKF_STATUS_REPORT`` flags and variances, the ``SYS_STATUS`` sensor
  bits and ``GPS_RAW_INT`` in three states: a GPS fix; the GPS off with the vision
  odometry's velocity (a DVL); neither.

The vehicle stays on the surface, disarmed or armed and motionless: nothing is flown.
With ``--inside KIND`` the script is the half that runs in the container (it uses the
image's pymavlink); the host half copies it in and runs it.
"""

import argparse
import json
import os
import subprocess
import sys
import time

IMAGE = "ghcr.io/murphy360/ardupilot-sitl-docker-sub:latest"
OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "torpedo_sitl",
    "ardusub_surface.json",
)
SPEEDUP = 5
GCS_SYSID = 255
START = (
    "printf 'SIM_OPOS_LAT 42.3898\\nSIM_OPOS_LNG -71.1476\\nSIM_OPOS_ALT 0\\n"
    "SIM_OPOS_HDG 270\\n{extra}' > /tmp/opos.parm && exec "
    "/ardupilot/Tools/autotest/sim_vehicle.py --vehicle ArduSub -I0 "
    "--add-param-file=/tmp/opos.parm -w --frame vectored --no-rebuild --no-mavproxy "
    "--speedup {speedup} --sysid 1 {args}"
)
KINDS = {
    "plain": {"extra": "", "args": ""},
    "vision": {
        "extra": "VISO_TYPE 1\\nSERIAL5_PROTOCOL 2\\nSIM_VICON_TMASK 2\\n"
        "EK3_SRC1_VELXY 6\\n",
        "args": "--sitl-instance-args=--serial5=sim:vicon:",
    },
}


# ---------------------------------------------------------------- the host half
def run(*cmd, check=True):
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


def capture_kind(image: str, kind: str) -> dict:
    name = f"torpedo-capture-{kind}-{os.getpid()}"
    script = START.format(speedup=SPEEDUP, **KINDS[kind])
    run(
        "docker",
        "run",
        "-d",
        "--name",
        name,
        "--entrypoint",
        "/bin/sh",
        image,
        "-c",
        script,
    )
    try:
        run("docker", "cp", os.path.abspath(__file__), f"{name}:/tmp/capture.py")
        out = run(
            "docker", "exec", name, "python3", "/tmp/capture.py", "--inside", kind
        )
        sys.stderr.write(out.stderr)
        return json.loads(out.stdout)
    finally:
        run("docker", "rm", "-f", name, check=False)


def main_host(args) -> None:
    plain = capture_kind(args.image, "plain")
    vision = capture_kind(args.image, "vision")
    plain["states"]["dvl"] = vision["states"]["dvl"]
    plain["statustext"]["depth_sensor_lost"] = plain["statustext"].get(
        "depth_sensor_lost", []
    )
    plain["image"] = args.image
    with open(args.out, "w") as f:
        json.dump(plain, f, indent=1)
        f.write("\n")
    print(f"wrote {args.out}: {len(plain['params'])} parameters")


# ------------------------------------------------------------ the container half
class Probe:
    """A ground station on SERIAL0 that keeps everything the vehicle says."""

    def __init__(self):
        from pymavlink import mavutil

        self.M = mavutil.mavlink
        self.m = mavutil.mavlink_connection(
            "tcp:127.0.0.1:5760", source_system=GCS_SYSID, source_component=190
        )
        if self.m.wait_heartbeat(timeout=60) is None:
            raise SystemExit("no heartbeat")
        self.texts = []  # (severity, text)
        self.last = {}
        self.pv = {}  # PARAM_VALUE by index: (name, value)
        self.pv_count = None
        self.gcs_beat = False
        self.last_beat = 0.0
        self.manual_beat = False  # send MANUAL_CONTROL while pumping
        self.last_manual = 0.0

    def pump(self, seconds: float) -> None:
        """Read for ``seconds`` of wall time; send GCS heartbeats if asked to."""
        end = time.time() + seconds
        while time.time() < end:
            if self.gcs_beat and time.time() - self.last_beat > 0.2:
                self.last_beat = time.time()
                self.m.mav.heartbeat_send(self.M.MAV_TYPE_GCS, 8, 0, 0, 4)
            if self.manual_beat and time.time() - self.last_manual > 0.05:
                self.last_manual = time.time()
                self.m.mav.manual_control_send(1, 0, 0, 500, 0, 0)
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                self.texts.append((msg.severity, msg.text.rstrip("\x00")))
            elif kind == "PARAM_VALUE":
                self.pv[msg.param_index] = (msg.param_id, msg.param_value)
                self.pv_count = msg.param_count
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == 1:
                self.last["HEARTBEAT"] = msg
            elif kind != "HEARTBEAT":
                self.last[kind] = msg

    def mark(self) -> int:
        return len(self.texts)

    def since(self, mark: int) -> list:
        return [list(t) for t in self.texts[mark:]]

    def command(self, cmd: int, *params) -> int:
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(1, 1, cmd, 0, *p)
        end = time.time() + 10
        while time.time() < end:
            self.pump(0.1)
            ack = self.last.get("COMMAND_ACK")
            if ack is not None and ack.command == cmd:
                del self.last["COMMAND_ACK"]
                return ack.result
        return -1

    def mode(self, number: int) -> int:
        return self.command(self.M.MAV_CMD_DO_SET_MODE, 1, number)

    def arm(self, on: bool) -> int:
        return self.command(self.M.MAV_CMD_COMPONENT_ARM_DISARM, 1 if on else 0)

    def param(self, name: str, value: float) -> None:
        self.m.mav.param_set_send(
            1, 1, name.encode(), float(value), self.M.MAV_PARAM_TYPE_REAL32
        )
        self.pump(0.5)

    def streams(self) -> None:
        for stream in (
            self.M.MAV_DATA_STREAM_EXTENDED_STATUS,
            self.M.MAV_DATA_STREAM_EXTRA3,
        ):
            self.m.mav.request_data_stream_send(1, 1, stream, 5, 1)

    def params(self) -> dict:
        """Every parameter by index, in order."""
        self.pv = {}
        self.m.mav.param_request_list_send(1, 1)
        end = time.time() + 90
        while time.time() < end:
            self.pump(0.5)
            if self.pv_count is not None and len(self.pv) >= self.pv_count:
                break
        for index in range(self.pv_count or 0):  # ask again for what was missed
            if index not in self.pv:
                self.m.mav.param_request_read_send(1, 1, b"", index)
                self.pump(0.3)
        return {name: value for _, (name, value) in sorted(self.pv.items())}

    def snapshot(self) -> dict:
        """The EKF, the sensors and the GPS as last reported."""
        ekf, st, gps = (
            self.last.get(k) for k in ("EKF_STATUS_REPORT", "SYS_STATUS", "GPS_RAW_INT")
        )
        return {
            "ekf_flags": int(ekf.flags) if ekf else None,
            "ekf_velocity_variance": round(float(ekf.velocity_variance), 3),
            "ekf_pos_horiz_variance": round(float(ekf.pos_horiz_variance), 3),
            "ekf_pos_vert_variance": round(float(ekf.pos_vert_variance), 3),
            "ekf_compass_variance": round(float(ekf.compass_variance), 3),
            "sensors_present": int(st.onboard_control_sensors_present),
            "sensors_enabled": int(st.onboard_control_sensors_enabled),
            "sensors_health": int(st.onboard_control_sensors_health),
            "gps_fix_type": int(gps.fix_type),
            "gps_satellites": int(gps.satellites_visible),
            "gps_eph_cm": int(gps.eph),
        }

    def settled(self, want=None, quiet: float = 5.0, limit: float = 120.0) -> dict:
        """The snapshot once the EKF flags satisfy ``want`` (any, by default) and have
        not changed for ``quiet`` s of wall time."""
        end, stable_since, flags = time.time() + limit, time.time(), None
        while time.time() < end:
            self.pump(0.5)
            now = self.last.get("EKF_STATUS_REPORT")
            now = None if now is None else int(now.flags)
            if now != flags:
                flags, stable_since = now, time.time()
            elif flags is not None and time.time() - stable_since >= quiet:
                if want is None or want(flags):
                    break
        return self.snapshot()


def early_probe(p: Probe) -> dict:
    """Try to change mode and arm in the first seconds, before the EKF has a position:
    the refusals and their words."""
    out, M = [], p.M
    start = time.time()
    while time.time() - start < 40:
        mark = p.mark()
        res_mode = p.mode(4)
        res_arm = p.arm(True)
        out.append(
            {
                "guided": res_mode,
                "arm": res_arm,
                "texts": p.since(mark),
                "wall_s": round(time.time() - start, 1),
            }
        )
        if res_arm == M.MAV_RESULT_ACCEPTED:
            p.arm(False)
            break
        p.pump(1.0)
    refused = [o for o in out if o["arm"] != M.MAV_RESULT_ACCEPTED and o["texts"]]
    return {"attempts": out[:6], "refusals": refused[:3]}


def gcs_failsafe(p: Probe, action: int) -> dict:
    """Arm, keep a GCS heartbeat going, stop it, start it again."""
    M = p.M
    p.param("FS_GCS_ENABLE", action)
    p.param("FS_GCS_TIMEOUT", 2)
    p.mode(19)  # MANUAL: a mode the failsafe can be seen to change
    p.arm(True)
    p.gcs_beat = True
    p.pump(2.0)
    mark = p.mark()
    p.gcs_beat = False
    p.pump(4.0)  # silence: well past the timeout at 5x
    hb = p.last["HEARTBEAT"]
    on = {
        "texts": p.since(mark),
        "custom_mode": int(hb.custom_mode),
        "system_status": int(hb.system_status),
        "armed": bool(hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED),
    }
    mark = p.mark()
    p.gcs_beat = True
    p.pump(3.0)
    cleared = {"texts": p.since(mark)}
    cleared["system_status"] = int(p.last["HEARTBEAT"].system_status)
    p.gcs_beat = False
    if on["armed"]:
        p.mode(2)
        p.arm(False)
    p.pump(1.0)
    return {"on": on, "cleared": cleared}


def pilot_input(p: Probe) -> dict:
    """Arm in MANUAL, send MANUAL_CONTROL, stop, send it again; for each action."""
    out = {}
    p.param("FS_PILOT_TIMEOUT", 1)
    for action in (1, 2):
        p.param("FS_PILOT_INPUT", action)
        p.mode(19)
        p.arm(True)
        p.manual_beat = True
        p.pump(1.5)
        mark = p.mark()
        p.manual_beat = False
        p.pump(3.0)  # 15 s of vehicle time without input
        hb = p.last["HEARTBEAT"]
        on = {
            "texts": p.since(mark),
            "system_status": int(hb.system_status),
            "armed": bool(hb.base_mode & p.M.MAV_MODE_FLAG_SAFETY_ARMED),
            "custom_mode": int(hb.custom_mode),
        }
        mark = p.mark()
        p.manual_beat = True
        p.pump(2.0)
        cleared = {
            "texts": p.since(mark),
            "system_status": int(p.last["HEARTBEAT"].system_status),
        }
        p.manual_beat = False
        out[str(action)] = {"on": on, "cleared": cleared}
        p.mode(2)
        p.arm(False)
        p.pump(1.0)
    p.param("FS_PILOT_INPUT", 0)
    return out


def battery(p: Probe) -> dict:
    out = {}
    p.mode(2)
    p.arm(True)
    for key, volt, act in (
        ("low", "BATT_LOW_VOLT", "BATT_FS_LOW_ACT"),
        ("critical", "BATT_CRT_VOLT", "BATT_FS_CRT_ACT"),
    ):
        mark = p.mark()
        p.param(act, 1)
        p.param(volt, 40)
        p.pump(8.0)
        out[key] = p.since(mark)
        p.param(volt, 0)
        p.param(act, 0)
        p.pump(1.0)
    p.arm(False)
    return out


def depth_sensor(p: Probe) -> dict:
    p.mode(2)
    p.arm(True)
    mark = p.mark()
    p.param("SIM_BARO_DISABLE", 1)
    p.pump(6.0)
    texts = p.since(mark)
    hb = p.last["HEARTBEAT"]
    snap = p.snapshot()
    p.param("SIM_BARO_DISABLE", 0)
    p.pump(3.0)
    p.arm(False)
    p.pump(1.0)
    mark = p.mark()
    p.param("SIM_BARO_DISABLE", 1)
    p.pump(2.0)
    refusal = {"arm": p.arm(True), "texts": p.since(mark)}
    p.arm(False)
    p.param("SIM_BARO_DISABLE", 0)
    p.pump(3.0)
    return {
        "arm_refused": refusal,
        "texts": texts,
        "custom_mode": int(hb.custom_mode),
        "sensors_health": snap["sensors_health"],
    }


def inside(kind: str) -> None:
    p = Probe()
    p.streams()
    p.pump(1.0)
    boot = p.since(0)
    out = {"statustext": {"boot": boot}, "states": {}}
    if kind == "plain":
        out["statustext"]["early"] = early_probe(p)
        out["params"] = p.params()
        out["states"]["gps"] = p.settled(lambda f: f & 16)  # absolute position
        mark = p.mark()
        p.mode(2)
        p.arm(True)
        p.pump(1.0)
        out["statustext"]["arm"] = p.since(mark)
        mark = p.mark()
        p.arm(False)
        p.pump(1.0)
        out["statustext"]["disarm"] = p.since(mark)
        mark = p.mark()
        res = p.mode(3)  # AUTO with no mission
        p.pump(1.0)
        out["statustext"]["auto_without_mission"] = p.since(mark)
        out["auto_without_mission_ack"] = res
        out["statustext"]["gcs_failsafe"] = {
            str(a): gcs_failsafe(p, a) for a in (1, 3, 4)
        }
        p.param("FS_GCS_ENABLE", 0)
        out["statustext"]["pilot_input"] = pilot_input(p)
        out["statustext"]["battery"] = battery(p)
        out["statustext"]["depth_sensor_lost"] = depth_sensor(p)
        p.mode(4)  # GUIDED, while there is a position
        mark = p.mark()
        p.param("SIM_GPS1_ENABLE", 0)
        out["states"]["none"] = p.settled(lambda f: not f & 16)
        out["states"]["none"]["texts"] = p.since(mark)
        mark = p.mark()
        out["statustext"]["arm_in_guided_without_position"] = {
            "arm": p.arm(True),
            "texts": p.since(mark),
        }
        p.arm(False)
        mark = p.mark()
        p.param("SIM_GPS1_ENABLE", 1)
        p.settled(lambda f: f & 16)
        out["statustext"]["gps_back"] = p.since(mark)
    else:
        p.settled(lambda f: f & 16)
        mark = p.mark()
        p.param("SIM_GPS1_ENABLE", 0)
        out["states"]["dvl"] = p.settled(lambda f: not f & 16)
        out["states"]["dvl"]["texts"] = p.since(mark)
    json.dump(out, sys.stdout)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image", default=IMAGE)
    ap.add_argument("--out", default=os.path.normpath(OUT))
    ap.add_argument("--inside", choices=list(KINDS))
    args = ap.parse_args()
    if args.inside:
        inside(args.inside)
    else:
        main_host(args)


if __name__ == "__main__":
    main()
