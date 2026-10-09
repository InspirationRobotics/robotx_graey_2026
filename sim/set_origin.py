#!/usr/bin/env python3
"""Tell the simulated Cube's EKF where on Earth its (0, 0) is.

Graey has no GPS: the EKF dead-reckons from the VN-100 and DVL and only knows
metres from where it started. Given an origin, it also reports latitude and
longitude, which is what QGroundControl needs to draw the sub on its map - so
the sub shows up on the satellite picture of the pool even while submerged.
Sent until the EKF confirms it (GPS_GLOBAL_ORIGIN), then exits.
"""
import argparse
import time

from robotx_graey_2026.api.pixhawk.mavlink import Link


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--lat", type=float, required=True)
    p.add_argument("--lon", type=float, required=True)
    p.add_argument("--mavlink", default="udpout:127.0.0.1:14558")
    a = p.parse_args()
    link = Link(a.mavlink, 198)
    got = {}
    for _ in range(120):
        link.heartbeat()
        link.send("set_gps_global_origin_send", 1, int(a.lat * 1e7), int(a.lon * 1e7), 0)
        link.drain(lambda kind, m: got.update(origin=m) if kind == "GPS_GLOBAL_ORIGIN" else None)
        if "origin" in got:
            o = got["origin"]
            print(f"EKF origin set: {o.latitude / 1e7:.7f}, {o.longitude / 1e7:.7f}", flush=True)
            return
        time.sleep(1.0)
    print("EKF never confirmed the origin", flush=True)


if __name__ == "__main__":
    main()
