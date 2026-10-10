#!/usr/bin/env python3
"""tools/sonar_map.py, unchanged, with sim_sonar.SimSonar in place of the Ping360.

Sim only: the Map tab also gets the REAL pipeline (dashed) and light boxes
(orange squares), put into the map's frame from where the simulator says the
sub really is, so what the sonar found can be checked against the truth.

Its pose comes the same way as on Graey: sim_sensors -> ROS topics ->
tools/pose_relay.py (started by inside.sh) -> UDP 14660. Map: http://localhost:8095/map
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
sys.path.insert(0, os.path.dirname(__file__))
import sonar_map                     # noqa: E402
from sim_sonar import SimSonar       # noqa: E402

from robotx_graey_2026.api.sonar import webview      # noqa: E402

made = []


class TrackedSimSonar(SimSonar):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        made.append(self)


def with_truth(snapshot):
    """The real pipeline in map coordinates: the map's own pose and the
    simulator's true pose at the same moment give the map's frame."""
    truth = made[0].truth_ned() if made else None
    if truth is not None and snapshot.get("pose"):
        n, e, yaw = truth
        x, y, h = snapshot["pose"]
        yaw0 = yaw - math.radians(h)                   # the map's +y, as a compass yaw
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        n0, e0 = n - (y * c - x * s_), e - (y * s_ + x * c)

        def to_map(nn, ee):
            dn, de = nn - n0, ee - e0
            return [round(-dn * s_ + de * c, 3), round(dn * c + de * s_, 3)]
        snapshot["truth"] = {"pipe": [to_map(p[0], p[1]) for p in made[0].centre_line],
                             "boxes": [to_map(b[0], b[1]) for b in made[0].truth_boxes]}
    set_map(snapshot)


set_map = webview.set_map
webview.set_map = with_truth
sonar_map.Sonar = TrackedSimSonar
sys.argv = ["sonar_map.py", "--no-relay", "--port", "8095", "--range", "4",
            "--save-dir", "/tmp/sim/sonar_run"] + sys.argv[1:]
sonar_map.main()
