#!/usr/bin/env python3
"""GUIDED test: drive forward N metres along the current heading, then hold there.

    python3 tools/guided_goto.py [forward_m]

Switch the vehicle to GUIDED and ARM it first, and keep the e-stop in reach.
Streams the target forever by design - switch to POSHOLD in QGroundControl
BEFORE Ctrl-C, or ArduSub lurches ~1 m forward when the setpoint stream stops.
Needs the workspace sourced.
"""
import math
import sys
import time

from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.mavlink import Link

fwd = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0

link = Link('udpout:127.0.0.1:14553', 190)
pose = {'pos': None, 'yaw': None}


def handle(kind, m):
    if kind == 'LOCAL_POSITION_NED':
        pose['pos'] = (m.x, m.y, m.z)
    elif kind == 'ATTITUDE':
        pose['yaw'] = m.yaw


t0 = time.time()
while (pose['pos'] is None or pose['yaw'] is None) and time.time() - t0 < 5:
    link.heartbeat()                            # MAVProxy will not route to us first
    link.drain(handle)
    time.sleep(0.1)

if pose['pos'] is None or pose['yaw'] is None:
    sys.exit('no position/attitude - is MAVProxy running?')

n, e, d = pose['pos']
yaw = pose['yaw']
tn, te = body_to_world(n, e, yaw, fwd, 0.0)
print(f'now ({n:.2f},{e:.2f}) hdg {math.degrees(yaw):.0f} deg '
      f'-> target NED ({tn:.2f},{te:.2f},{d:.2f})  streaming until stopped')

while True:                                     # POSHOLD first, THEN Ctrl-C
    link.goto_ned(tn, te, d, yaw)
    time.sleep(0.2)
