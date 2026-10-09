#!/bin/bash
# Runs inside the graey-sitl container (started by sim/run.sh). Brings up, in order:
#   ArduSub 4.5.7 SITL with Graey's parameters      (the Cube)
#   MAVProxy with Graey's port layout               (start_mavproxy.sh's job)
#   sim_sensors.py                                  (stands in for vn100_node + dvl_node)
#   nav_ekf_bridge, gui_node, pos_server            (Graey's real Jetson code, unchanged)
#   pose_relay + sonar_map on a simulated Ping360    (sim/sim_sonar.py, a simulated pipeline)
# Logs go to /tmp/sim/*.log inside the container.
set -e
REPO=/root/robotx_ws/src/robotx_graey_2026
LOGS=/tmp/sim; mkdir -p $LOGS
source /opt/ros/humble/setup.bash
export PYTHONPATH=$REPO:$PYTHONPATH        # keep ROS's own paths

# Graey's parameters, then the simulator-only changes on top
cat $REPO/params/graey_4.5.7_autonomous_validated.parm $REPO/sim/sim_overrides.parm > $LOGS/graey_sim.parm

cd $LOGS
/opt/ardupilot/build/sitl/bin/ardusub --model vectored_6dof --speedup 1 -I0 \
    --home "$HOME_LAT,$HOME_LON,0,$HOME_HDG" \
    --defaults /opt/ardupilot/Tools/autotest/default_params/sub.parm,$LOGS/graey_sim.parm \
    > $LOGS/sitl.log 2>&1 &
sleep 3

# Graey's MAVProxy layout (scripts/start_mavproxy.sh), plus 14557/14558 for the
# simulator's own helpers and QGroundControl on this computer.
mavproxy.py --master=tcp:127.0.0.1:5760 --daemon --non-interactive --state-basedir=$LOGS \
    --out=udpin:0.0.0.0:14551 --out=udpin:0.0.0.0:14552 --out=udpin:0.0.0.0:14553 \
    --out=udpin:0.0.0.0:14554 --out=udpin:0.0.0.0:14555 --out=udpin:0.0.0.0:14556 \
    --out=udpin:0.0.0.0:14557 --out=udpin:0.0.0.0:14558 --out=udpin:0.0.0.0:14559 \
    --out=udp:host.docker.internal:14550 \
    > $LOGS/mavproxy.log 2>&1 &
sleep 3

node() { python3 -c "from $1 import main; main()" > $LOGS/$2.log 2>&1 & }
python3 $REPO/sim/sim_sensors.py --water-depth "$WATER_DEPTH" --dvl-noise "$DVL_NOISE" > $LOGS/sim_sensors.log 2>&1 &
node robotx_graey_2026.api.navigation.nav_ekf_bridge nav_ekf_bridge
node robotx_graey_2026.api.gui.gui_node gui_node
node robotx_graey_2026.api.navigation.pos_server pos_server
python3 $REPO/sim/set_origin.py --lat "$HOME_LAT" --lon "$HOME_LON" > $LOGS/set_origin.log 2>&1 &

# The sonar side, as on Graey: the relay passes VN-100 + DVL out of ROS (it quits
# when its input closes, so give it one that stays open), and the map tool runs
# on the simulated Ping360 and the simulated pipeline.
tail -f /dev/null | python3 $REPO/tools/pose_relay.py > $LOGS/pose_relay.log 2>&1 &
python3 $REPO/sim/sim_sonar_map.py > $LOGS/sonar_map.log 2>&1 &

echo "graey-sitl up. QGroundControl: UDP 14550.  GUI: http://localhost:8090"
wait
