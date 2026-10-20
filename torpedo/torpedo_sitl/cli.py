"""``torpedo-sitl``: start the front seat from the SITL images' environment.

The environment names are the SITL images' own: ``INSTANCE``, ``LAT``, ``LON``,
``ALT``, ``DIR``, ``SPEEDUP``, ``SYSID``, ``SITL_PARAMS`` and ``BATT_CAPACITY``, plus
``TORPEDO_MODEL``, the path of a schema-1 model file. With no model it exits 2 and
says why.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading

from .model import ModelError, load
from .server import Config, Server

NO_MODEL = (
    "no hull: set TORPEDO_MODEL to the path of a model file (schema 1) or give --model"
)


def config_from_env(env, args: argparse.Namespace) -> Config:
    """The configuration the environment and the arguments make."""
    return Config(
        instance=int(env.get("INSTANCE", "0")),
        sysid=int(env.get("SYSID", "1")),
        speedup=float(env.get("SPEEDUP", "1")),
        lat=float(env.get("LAT", "42.3898")),
        lon=float(env.get("LON", "-71.1476")),
        alt=float(env.get("ALT", "0")),
        heading=float(env.get("DIR", "270")),
        batt_mah=float(env.get("BATT_CAPACITY", "10000")),
        sitl_params=env.get("SITL_PARAMS", ""),
        param_files=args.param_file,
    )


def main(argv: list[str] | None = None, env=None) -> int:
    """Run until SIGTERM or SIGINT; 2 when there is no usable model."""
    env = os.environ if env is None else env
    ap = argparse.ArgumentParser(prog="torpedo-sitl", description=__doc__)
    ap.add_argument("--model", default=env.get("TORPEDO_MODEL"))
    ap.add_argument("--param-file", action="append", default=[])
    args = ap.parse_args(argv)
    if not args.model:
        print(NO_MODEL, file=sys.stderr)
        return 2
    try:
        model = load(args.model)
    except (OSError, ModelError) as exc:
        print(f"cannot use the model {args.model}: {exc}", file=sys.stderr)
        return 2
    server = Server(model, config_from_env(env, args))
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    ports = ", ".join(f"{n} {p}" for n, p in server.bound.items())
    print(f"Torpedo front seat, hull {model.id}: {ports}", flush=True)
    server.serve(stop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
