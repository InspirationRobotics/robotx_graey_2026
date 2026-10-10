#!/usr/bin/env python3
"""Log where the simulator says the sub is next to where Graey's EKF thinks it
is, so navigation errors can be told apart from control errors.

    python3 sim/truth_vs_ekf.py [seconds]
"""
import math
import sys
import time

from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil

link = Link("udpout:127.0.0.1:14558", 188)
for mid in (108, 32, 30):           # SIM_STATE, LOCAL_POSITION_NED, ATTITUDE
    link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 2e5)
s, t0, last, origin = {}, time.time(), 0, None
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60
print("   t   truth N    E    D  hdg  |  EKF N    E    D  hdg  |  err m  hdg err", flush=True)
while time.time() - t0 < secs:
    link.drain(lambda k, m: s.__setitem__(k, m))
    if all(k in s for k in ("SIM_STATE", "LOCAL_POSITION_NED", "ATTITUDE")) and time.time() - last > 2:
        st, lp, at = s["SIM_STATE"], s["LOCAL_POSITION_NED"], s["ATTITUDE"]
        lat, lon = st.lat_int / 1e7, st.lon_int / 1e7     # the float fields are too coarse
        if origin is None:
            origin = (lat, lon, st.alt, lp.x, lp.y, lp.z)
        n = math.radians(lat - origin[0]) * 6378137 + origin[3]
        e = math.radians(lon - origin[1]) * 6378137 * math.cos(math.radians(origin[0])) + origin[4]
        d = origin[2] - st.alt + origin[5]
        th, eh = math.degrees(st.yaw) % 360, math.degrees(at.yaw) % 360
        err = math.dist((n, e, d), (lp.x, lp.y, lp.z))
        print(f"{time.time() - t0:4.0f}  {n:7.2f} {e:5.2f} {d:5.2f} {th:4.0f}  | {lp.x:6.2f} {lp.y:5.2f} {lp.z:5.2f} {eh:4.0f}  |  {err:5.2f}  {((eh - th + 180) % 360) - 180:6.1f}", flush=True)
        last = time.time()
    time.sleep(0.02)
