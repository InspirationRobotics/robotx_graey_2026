#!/usr/bin/env python3
"""Fly the simulated sub along the simulated pipeline with the sonar map running,
then grade the map: how close did the crumbs land to where the pipe really is?

    python3 sim/flyover_test.py [--clearance 1.0] [--speed 0.15]

The route comes from the simulator's own copy of the pipeline (this is a test
of the map, not of finding the pipe). The sub dives beside home, the map is
Started there, and the sub flies the pipe's centre line at one depth, CLEARANCE
above its highest point, facing along each section so the scan slice cuts
across it. GUIDED targets are sent once each (see waypoint_test.py).
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from sim_sonar import PIPE_R, build_world                                 # noqa: E402
from robotx_graey_2026.api.pixhawk.mavlink import MODE_GUIDED, Link, mavutil  # noqa: E402

MAP = "http://127.0.0.1:8095"
RESEND_S, REACHED_M = 20.0, 0.3


def post(path, body):
    return urllib.request.urlopen(urllib.request.Request(MAP + path, data=body.encode()),
                                  timeout=3).read().decode()


def dist_to_line(p, line):
    best = 1e9
    for a, b in zip(line, line[1:]):
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
        best = min(best, np.linalg.norm(p - (a + t * ab)))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clearance", type=float, default=1.0, help="m, sonar above the pipe's top")
    ap.add_argument("--speed", type=float, default=0.15, help="m/s along the pipe")
    ap.add_argument("--threshold", type=int, default=120)
    a = ap.parse_args()

    depth = float(os.environ.get("WATER_DEPTH", "12"))
    _, centre = build_world(depth, float(os.environ.get("PIPE_N", "3")),
                            float(os.environ.get("PIPE_E", "2")), float(os.environ.get("PIPE_HDG", "30")))
    z_fly = centre[:, 2].min() - PIPE_R - a.clearance
    link = Link("udpout:127.0.0.1:14553", 184)
    s = {}

    def handle(kind, m):
        if kind == "LOCAL_POSITION_NED":
            s["pos"] = np.array([m.x, m.y, m.z])
        elif kind == "ATTITUDE":
            s["yaw"] = m.yaw
        elif kind == "HEARTBEAT" and m.get_srcComponent() == 1:
            s["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = m.custom_mode

    def pump(secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            link.drain(handle)
            time.sleep(0.02)

    def go(n, e, z, yaw, label, tol=REACHED_M, timeout=240):
        t, last, shown = time.monotonic(), 0.0, 0.0
        while not (np.linalg.norm(s["pos"][:2] - (n, e)) < tol and abs(s["pos"][2] - z) < 0.15):
            if time.monotonic() - t > timeout:
                raise SystemExit(f"timed out: {label}")
            if time.monotonic() - last > RESEND_S:
                link.goto_ned(n, e, z, yaw)
                last = time.monotonic()
            if time.monotonic() - shown > 5:
                link.heartbeat()
                p = s["pos"]
                print(f"  {label}: at {p[0]:6.2f} {p[1]:6.2f} depth {p[2]:5.2f}", flush=True)
                shown = time.monotonic()
            pump(0.1)

    while not all(k in s for k in ("pos", "yaw", "mode")):
        link.heartbeat()
        pump(0.5)
    while not (s.get("armed") and s.get("mode") == MODE_GUIDED):
        link.heartbeat()
        link.set_mode(MODE_GUIDED)
        link.arm()
        pump(1.0)
    link.command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, a.speed, -1)

    first = centre[1] - centre[0]
    hdg = math.atan2(first[1], first[0])
    start = centre[0, :2] - 1.0 * first[:2] / np.linalg.norm(first[:2])
    print(f"diving to {z_fly:.2f} m (pipe tops out at {centre[:, 2].min() - PIPE_R:.2f} m)", flush=True)
    go(s["pos"][0], s["pos"][1], z_fly, hdg, "dive", tol=1.0)
    go(start[0], start[1], z_fly, hdg, "to the start of the pipe")

    # Start the map here: crumbs come back in its frame, (0,0) = DVL here, +y = heading here
    for cmd in (("/range", "4"), ("/sector", "180 0"), ("/threshold", str(a.threshold))):
        post(*cmd)
    pump(0.5)
    post("/mapcmd", "start")
    pump(1.0)
    n0, e0, yaw0 = s["pos"][0], s["pos"][1], s["yaw"]

    for i, (p, q) in enumerate(zip(centre, centre[1:]), 1):
        d = q - p
        go(q[0], q[1], z_fly, math.atan2(d[1], d[0]), f"section {i}")
    pump(8.0)                                           # let the last sweeps land

    m = json.loads(urllib.request.urlopen(MAP + "/mapdata", timeout=3).read())
    c = np.array(m["crumbs"]) if m["crumbs"] else np.zeros((0, 4))
    # map (x right, y forward at Start) -> NED
    n = n0 + c[:, 1] * math.cos(yaw0) - c[:, 0] * math.sin(yaw0)
    e = e0 + c[:, 1] * math.sin(yaw0) + c[:, 0] * math.cos(yaw0)
    off = np.array([dist_to_line(np.array(pt), centre[:, :2]) for pt in zip(n, e)])
    print(f"\nmap: {len(c)} crumbs over {m['sweep']} sweeps", flush=True)
    if len(c):
        print(f"  within 0.3 m of the pipe's centre line: {np.mean(off < 0.3):.0%}   "
              f"within 0.5 m: {np.mean(off < 0.5):.0%}   median {np.median(off):.2f} m", flush=True)
    go(s["pos"][0], s["pos"][1], 0.2, s["yaw"], "surface", tol=1.0)
    link.disarm()
    print("surfaced, disarmed", flush=True)


if __name__ == "__main__":
    main()
