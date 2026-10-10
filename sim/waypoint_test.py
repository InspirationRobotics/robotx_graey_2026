#!/usr/bin/env python3
"""Surface -> dive -> one GPS waypoint -> surface, in GUIDED mode.

    python3 sim/waypoint_test.py --lat 37.1234567 --lon -122.1234567 [--depth 0.5]

Starts with the sub on the surface. Arms, dives to --depth, drives to the
waypoint at that depth, surfaces there and disarms. The waypoint is turned
into metres from the EKF's origin (Graey has no GPS - the EKF dead-reckons
from the VN-100 and DVL), so it needs the origin set, which sim/inside.sh does.

Uses the mission port, 14553, like guided_goto.py. Stops and disarms if the
mode is changed away from GUIDED - that is the pilot taking over.
"""
import argparse
import math
import time

from robotx_graey_2026.api.pixhawk.mavlink import MODE_GUIDED, Link, mavutil

RESEND_S = 20.0         # every send restarts ArduSub's position controller, including
                        # the integrator that holds a buoyant sub down; at 3 s (as in
                        # mission_base.py) the simulated sub never left the surface
REACHED_M = 0.3
DEPTH_OK_M = 0.1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--lat", type=float, required=True)
    p.add_argument("--lon", type=float, required=True)
    p.add_argument("--depth", type=float, default=0.5, help="metres below the surface")
    p.add_argument("--mavlink", default="udpout:127.0.0.1:14553")
    p.add_argument("--timeout", type=float, default=300.0)
    a = p.parse_args()

    link = Link(a.mavlink, 190)
    s = {}

    def handle(kind, m):
        if kind == "LOCAL_POSITION_NED":
            s["pos"] = (m.x, m.y, m.z)
        elif kind == "ATTITUDE":
            s["yaw"] = m.yaw
        elif kind == "GPS_GLOBAL_ORIGIN":
            s["origin"] = (m.latitude / 1e7, m.longitude / 1e7)
        elif kind == "HEARTBEAT" and m.get_srcComponent() == 1:
            s["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = m.custom_mode

    def pump(secs=0.1):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            link.drain(handle)
            time.sleep(0.02)

    print("waiting for position and the EKF origin...", flush=True)
    t0 = time.monotonic()
    while not ("pos" in s and "origin" in s and "yaw" in s):
        link.heartbeat()
        link.command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 49)   # GPS_GLOBAL_ORIGIN
        pump(1.0)
        if time.monotonic() - t0 > 60:
            raise SystemExit("no position or origin after 60 s - is the nav bridge running?")

    lat0, lon0 = s["origin"]
    n_wp = math.radians(a.lat - lat0) * 6378137.0
    e_wp = math.radians(a.lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
    x0, y0, z_surf = s["pos"]
    z_dive = z_surf + a.depth
    print(f"waypoint is {n_wp:.2f} m north, {e_wp:.2f} m east of the origin; "
          f"sub at {x0:.2f}, {y0:.2f}, surface z={z_surf:.2f}", flush=True)

    def wait_for(goal, label, n, e, z, yaw):
        last = shown = 0.0
        while not goal():
            if time.monotonic() - t0 > a.timeout:
                raise SystemExit(f"timed out during: {label}")
            if s.get("mode") not in (None, MODE_GUIDED) and s.get("armed"):
                link.disarm()
                raise SystemExit("mode changed away from GUIDED - pilot took over; disarmed")
            if time.monotonic() - last > RESEND_S:
                link.goto_ned(n, e, z, yaw)         # once, then only as insurance
                last = time.monotonic()
            if time.monotonic() - shown > 2.0:
                link.heartbeat()
                x, y, zz = s["pos"]
                print(f"  {label}: at {x:6.2f} {y:6.2f}  depth {zz - z_surf:5.2f} m", flush=True)
                shown = time.monotonic()
            pump(0.1)

    t0 = time.monotonic()
    print("GUIDED + arm", flush=True)
    while not (s.get("armed") and s.get("mode") == MODE_GUIDED):
        link.heartbeat()
        link.set_mode(MODE_GUIDED)
        link.arm()
        pump(1.0)
        if time.monotonic() - t0 > 30:
            raise SystemExit("would not arm in GUIDED - check QGroundControl's messages")

    yaw = s["yaw"]
    wait_for(lambda: abs(s["pos"][2] - z_dive) < DEPTH_OK_M, f"dive to {a.depth} m",
             x0, y0, z_dive, yaw)
    yaw = math.atan2(e_wp - s["pos"][1], n_wp - s["pos"][0])
    wait_for(lambda: math.hypot(s["pos"][0] - n_wp, s["pos"][1] - e_wp) < REACHED_M,
             "to the waypoint", n_wp, e_wp, z_dive, yaw)
    wait_for(lambda: s["pos"][2] < z_surf + DEPTH_OK_M
             and math.hypot(s["pos"][0] - n_wp, s["pos"][1] - e_wp) < 2 * REACHED_M,
             "surface", n_wp, e_wp, z_surf, yaw)
    link.disarm()
    x, y, z = s["pos"]
    print(f"done: surfaced {math.hypot(x - n_wp, y - e_wp):.2f} m from the waypoint, disarmed", flush=True)


if __name__ == "__main__":
    main()
