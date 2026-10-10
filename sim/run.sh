#!/bin/bash
# Start Graey's software-in-the-loop on this computer (Docker). See sim/README.md.
#
#   sim/run.sh                 the lab pool, 2 m deep
#   WATER_DEPTH=12 sim/run.sh  harbor depth
#
# Then: QGroundControl connects by itself (UDP 14550); GUI at http://localhost:8090;
# sonar map (simulated Ping360 and pipeline) at http://localhost:8095/map.
# PIPE_N / PIPE_E / PIPE_HDG move the simulated pipeline (metres from home, degrees).
# SIM_WORLD=pool: Ruth's ~0.9 m pool (use WATER_DEPTH=0.9), walls POOL_L x POOL_W m with
# a corner at POOL_N0, POOL_E0 from home, and the pipeline hung without legs at
# PIPE_DEPTH (default half the water depth).
# Stop with:  docker rm -f graey-sitl
set -e
cd "$(dirname "$0")/.."
: "${HOME_LAT:?set HOME_LAT/HOME_LON to the pool (see sim/README.md)}"
: "${HOME_LON:?}"
docker rm -f graey-sitl >/dev/null 2>&1 || true
docker run -d --name graey-sitl \
    -v "$(pwd)":/root/robotx_ws/src/robotx_graey_2026 \
    -p 8090:8090 -p 8081:8081 -p 8095:8095 -p 14553:14553/udp \
    -e HOME_LAT="$HOME_LAT" -e HOME_LON="$HOME_LON" -e HOME_HDG="${HOME_HDG:-0}" \
    -e WATER_DEPTH="${WATER_DEPTH:-2}" -e DVL_NOISE="${DVL_NOISE:-0}" \
    -e PIPE_N="${PIPE_N:-3}" -e PIPE_E="${PIPE_E:-2}" -e PIPE_HDG="${PIPE_HDG:-30}" \
    -e SIM_WORLD="${SIM_WORLD:-harbor}" -e PIPE_DEPTH="${PIPE_DEPTH:-}" -e PIPE_SEED="${PIPE_SEED:-7}" \
    -e POOL_L="${POOL_L:-11}" -e POOL_W="${POOL_W:-7}" -e POOL_N0="${POOL_N0:--3}" -e POOL_E0="${POOL_E0:--2}" \
    graey-sitl bash sim/inside.sh >/dev/null
echo "graey-sitl starting. Logs: docker exec graey-sitl tail -f /tmp/sim/sitl.log"
