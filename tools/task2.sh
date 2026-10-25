#!/bin/bash
# Task 2, the pipeline mission, one word per part (Ruth, Oct 24). On Graey's host:
#
#   tools/task2.sh search|follow|survey|repair|full [dry] [-p name:=value ...]
#
#   search   dive, find the pipe, go over it and line up; hold 5 s, surface
#   follow   the pilot leaves the sub over the pipe: line up, follow it; hand back in POSHOLD
#   survey   dive, find the pipe, follow it to the buoy end, then the whole pipe logging
#            each light; 5 s over RED; hand back in POSHOLD
#   repair   the survey, centring the magnet on the RED light until it turns GREEN
#   full     the competition run: the repair survey, then surface
#   dry      rehearse: nothing is sent to the sub (the pilot turns it when asked)
#
# It starts the sonar map and the cameras if they aren't running (through the GUI's launcher,
# tools/gui_sonar_launcher.py), and records the cameras to ~/camera_rec/task2_<part>_<time>.mjpg
# for the whole run. The mission records the sonar itself (sonar_data/rec_...).
# The numbers it uses are the SETTINGS at the top of
# robotx_graey_2026/api/navigation/pipeline_mission.py. A -p after the word changes one for
# this run only: tools/task2.sh search -p pool_alt_m:=3.5

PARTS="search follow survey repair full"
part=$1
if [[ -z $part || " $PARTS " != *" $part "* ]]; then
  sed -n '4,13p' "$0" | sed 's/^# \{0,1\}//'
  exit 1
fi
shift
dry=false
if [[ $1 == dry ]]; then
  dry=true
  shift
fi

up() { curl -s -m 2 -o /dev/null "http://127.0.0.1:$1/"; }
launcher() { curl -s -m 20 -X POST -o /dev/null "http://127.0.0.1:8096/$1"; }
wait_up() {     # port, what, seconds
  for ((i = 0; i < $3; i++)); do
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
  if ! wait_up 8095 "sonar map" 30; then
    echo "The sonar map didn't start in 30 s - look at the GUI's Sonar tab."
    exit 1
  fi
fi
echo "Sonar map: running."

if ! up 8080; then
  echo "Starting the cameras..."
  launcher "camera/start"
  wait_up 8080 "cameras" 30
fi
if up 8080; then
  echo "Cameras: running."
elif [[ $part == search || $part == follow ]]; then
  echo "Warning: the cameras didn't start - no video this run (GUI Camera tab shows why)."
else
  echo "The cameras didn't start, and $part needs them for the lights - look at the GUI's Camera tab."
  exit 1
fi

if [[ $dry == false ]]; then
  echo "task2 $part: the sub will ARM and MOVE on its own."
  echo "Pilot: POSHOLD, settled. Switch modes on the controller to take over at any time."
  read -r -p "Type y to start: " ok
  if [[ $ok != y ]]; then
    echo "Not started."
    exit 0
  fi
fi

if up 8080; then
  name=task2_$part
  [[ $dry == true ]] && name=${name}_dry
  launcher "record/start?name=$name" && echo "Recording the cameras to ~/camera_rec/${name}_<date>_<time>.mjpg"
  trap 'launcher record/stop; echo "Camera recording stopped."' EXIT
fi

extra=""
if [[ $# -gt 0 ]]; then
  extra=$(printf ' %q' "$@")       # quoted for the shell inside the container
fi
docker exec -it graey bash -c "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=.:\$PYTHONPATH python3 -c 'from robotx_graey_2026.api.navigation.pipeline_mission import main; main()' --ros-args -p part:=$part -p dry_run:=$dry$extra"
