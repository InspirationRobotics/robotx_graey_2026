#!/bin/bash
# The survey run (Ruth, Nov 10). On Graey's host, inside tmux so it outlives the WiFi:
#
#   tools/survey.sh [--dry] [--skip-path] [--alt 4.0] [-- -p name:=value ...]
#
# Starts the sonar map and the cameras if they aren't running (the GUI's launcher, as task2.sh does), then
# tools/survey_run.py: it waits for SE on the RadioMaster; each press reads the points uploaded from QGC and
# runs the whole survey once (the path and the start buoy at the surface by GPS, then turn to the end buoy,
# dive, transit along it for the plotted distance, surface), recording the sonar and both cameras from the
# start buoy until it is back up. Then it waits for SE again. Ctrl-C stops it.
#   --dry        nothing moves (no waypoint runs, pipeline_mission dry run)
#   --skip-path  pool test: the transit starts where the sub is
up() { curl -s -m 2 -o /dev/null "http://127.0.0.1:$1/"; }
launcher() { curl -s -m 20 -X POST -o /dev/null "http://127.0.0.1:8096/$1"; }
wait_up() {     # port, seconds
  for ((i = 0; i < $2; i++)); do
    up "$1" && return 0
    sleep 1
  done
  return 1
}
if ! up 8095; then
  echo "Starting the sonar map..."
  if ! launcher "start?tool=map"; then
    echo "The GUI's launcher (port 8096) isn't answering. Start the sonar map from the Sonar tab, then run this again."
    exit 1
  fi
  if ! wait_up 8095 30; then
    echo "The sonar map didn't start in 30 s - look at the GUI's Sonar tab."
    exit 1
  fi
fi
echo "Sonar map: running."
if ! up 8080; then
  echo "Starting the cameras..."
  launcher "camera/start"
  wait_up 8080 30
fi
if up 8080; then
  echo "Cameras: running."
else
  echo "Warning: the cameras didn't start - no camera recordings (the GUI's Camera tab shows why)."
fi
args=""
if [[ $# -gt 0 ]]; then
  args=$(printf ' %q' "$@")       # quoted for the shell inside the container
fi
docker exec -it graey bash -c "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=.:\$PYTHONPATH python3 tools/survey_run.py$args"
