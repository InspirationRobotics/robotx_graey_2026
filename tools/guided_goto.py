#!/usr/bin/env python3
"""GUIDED test: drive forward N meters along the current heading, then hold.
usage: python3 guided_goto.py [forward_m] [seconds]
Switch the vehicle to GUIDED (armed) FIRST. Keep e-stop / MANUAL ready."""
import sys
import time
import math
from pymavlink import mavutil

fwd = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

m = mavutil.mavlink_connection('udpout:127.0.0.1:14553',
                               source_system=1, source_component=190)
for _ in range(10):
    m.mav.heartbeat_send(6, 8, 0, 0, 0)
    time.sleep(0.1)

pos = None
yaw = None
t0 = time.time()
while (pos is None or yaw is None) and time.time() - t0 < 5:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == 'LOCAL_POSITION_NED':
        pos = (msg.x, msg.y, msg.z)
    if msg.get_type() == 'ATTITUDE':
        yaw = msg.yaw

if pos is None or yaw is None:
    print('no position/attitude - is the stack (MAVProxy + nodes) running?')
    sys.exit(1)

tn = pos[0] + fwd * math.cos(yaw)
te = pos[1] + fwd * math.sin(yaw)
td = pos[2]
print(f'now ({pos[0]:.2f},{pos[1]:.2f}) hdg {math.degrees(yaw):.0f} deg '
      f'-> target NED ({tn:.2f},{te:.2f},{td:.2f})')

mask = 0b110111111000   # position only
end = time.time() + secs
while True:
    m.mav.set_position_target_local_ned_send(
        0, 1, 1, 1, mask, tn, te, td, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.2)
print('done streaming target')

