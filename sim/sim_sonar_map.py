#!/usr/bin/env python3
"""tools/sonar_map.py, unchanged, with sim_sonar.SimSonar in place of the Ping360.

Its pose comes the same way as on Graey: sim_sensors -> ROS topics ->
tools/pose_relay.py (started by inside.sh) -> UDP 14660. Map: http://localhost:8095/map
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
sys.path.insert(0, os.path.dirname(__file__))
import sonar_map                     # noqa: E402
from sim_sonar import SimSonar       # noqa: E402

sonar_map.Sonar = SimSonar
sys.argv = ["sonar_map.py", "--no-relay", "--port", "8095", "--range", "4",
            "--save-dir", "/tmp/sim/sonar_run"] + sys.argv[1:]
sonar_map.main()
