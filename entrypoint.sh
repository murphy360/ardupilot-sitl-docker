#!/bin/sh
# Start the SITL with its start position coming from SIM_OPOS_* parameters
# rather than sim_vehicle.py's --custom-location. --custom-location becomes
# --home on the vehicle binary, which overrides SIM_OPOS_* on every boot,
# including MAVLink reboots (the binary re-execs itself with the same argv
# minus -w). With no --home, the start position is read from SIM_OPOS_*, so:
#   - container start: -w wipes saved params, SIM_OPOS_* come from the env
#     below via the defaults file -> vehicle starts at LAT/LON/ALT/DIR;
#   - MAVLink PARAM_SET SIM_OPOS_* + MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: the
#     saved values win over the defaults file -> vehicle restarts there.
set -eu

parm_file="/tmp/opos-${INSTANCE}.parm"
cat > "$parm_file" <<EOF
SIM_OPOS_LAT ${LAT}
SIM_OPOS_LNG ${LON}
SIM_OPOS_ALT ${ALT}
SIM_OPOS_HDG ${DIR}
EOF

exec /ardupilot/Tools/autotest/sim_vehicle.py \
    --vehicle "${VEHICLE}" \
    -I"${INSTANCE}" \
    -w \
    --frame "${MODEL}" \
    --no-rebuild \
    --no-mavproxy \
    --speedup "${SPEEDUP}" \
    --add-param-file="$parm_file"
