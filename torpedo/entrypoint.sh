#!/bin/sh
# Start the torpedo front seat. The SITL images' environment names are read by
# torpedo-sitl itself; this checks the one thing it cannot do without.
set -eu

if [ -z "${TORPEDO_MODEL:-}" ]; then
    echo "no hull: set TORPEDO_MODEL to the path of a model file (schema 1)," \
         "mounted into the container" >&2
    exit 2
fi
if [ ! -f "$TORPEDO_MODEL" ]; then
    echo "no hull: TORPEDO_MODEL ($TORPEDO_MODEL) is not a file in the container" >&2
    exit 2
fi

exec torpedo-sitl "$@"
