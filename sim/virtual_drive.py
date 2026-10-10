#!/usr/bin/env python3
"""Drive a virtual sub along a fixed path, ping with sonar_physics, build the map
exactly as tools/sonar_map.py does (CrumbMap), and draw it over the real shapes.

    python3 sim/virtual_drive.py pool       --out pool.png
    python3 sim/virtual_drive.py pipe-along --out along.png
    python3 sim/virtual_drive.py pipe-turn  --out turn.png

No SITL: the sub moves perfectly along the path, level, at the real ping pace
(0.03 s + the sound's round trip per ping, as sim_sonar.py). For seeing what
the sonar makes of a scene; the live sim uses the same sonar_physics.ping().
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import sonar_physics as P                                                   # noqa: E402
from sim_sonar import N_BINS, PIPE_R, build_world, pipeline_world, pool_world  # noqa: E402
from robotx_graey_2026.api.sonar import settings as S                      # noqa: E402
from robotx_graey_2026.api.sonar.crumbs import CrumbMap                    # noqa: E402
from robotx_graey_2026.api.sonar.outlines import find                      # noqa: E402
from robotx_graey_2026.api.sonar.pose import Pose                          # noqa: E402

POOL = dict(length=10.0, width=5.0, depth=1.8)       # placeholder until Ruth's numbers


def yaw_matrix(hdg):
    h = math.radians(hdg)
    return np.array([[math.cos(h), -math.sin(h), 0], [math.sin(h), math.cos(h), 0], [0, 0, 1.0]])


def legs_path(points, depth, speed, turn_dps):
    """Straight legs between (n, e) points at speed; turn on the spot between them."""
    out, t = [], 0.0
    hdg = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
    for a, b in zip(points, points[1:]):
        want = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        turn = (want - hdg + 180) % 360 - 180
        steps = int(abs(turn) / turn_dps * 10) + 1
        for k in range(steps):                                  # 10 Hz pose samples
            out.append((t, a[0], a[1], depth, hdg + turn * k / steps)); t += 0.1
        hdg = want
        dist = math.dist(a, b)
        for k in range(int(dist / speed * 10)):
            f = k * speed * 0.1 / dist
            out.append((t, a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), depth, hdg)); t += 0.1
    return np.array(out)


def turn_path(n, e, depth, rate_dps, degrees):
    ts = np.arange(0, degrees / rate_dps, 0.1)
    return np.array([(t, n, e, depth, rate_dps * t) for t in ts])


def at(path, t):
    i = min(np.searchsorted(path[:, 0], t), len(path) - 1)
    return path[i]


def run(world, path, range_m, floor_depth, sector=(180, 360), step=2.0, threshold=100,
        floor_cut=0.5, seed=1):
    rng = np.random.default_rng(seed)
    cm = CrumbMap(threshold=threshold, floor_cut_m=floor_cut)
    t, end, track = 0.0, path[-1, 0], []
    while t < end:
        cm.new_sweep()
        a = sector[0]
        while a <= sector[1] + 1e-9 and t < end:
            t += 0.03 + 2 * range_m / 1500.0
            _, n, e, d, hdg = at(path, t)
            r = yaw_matrix(hdg)
            sonar = np.array([n, e, d]) + r @ (S.SONAR_FWD_M, S.SONAR_RIGHT_M, 0)
            row = P.ping(world, sonar, r, a % 360, range_m, N_BINS, rng)
            pose = Pose(t, e, n, hdg % 360, True, 0.0, 0.0, floor_depth - d)   # map: x east, y north
            cm.add_ping(a % 360, row, range_m / N_BINS, pose, step, range_m, 0.0)
            track.append((e, n))
            a += step
    return cm, np.array(track)


def draw(cm, track, truth_lines, out, title, view):
    (e0, e1), (n0, n1) = view
    W = 900
    s = W / max(e1 - e0, n1 - n0)
    H = int((n1 - n0) * s) + 70
    img = np.full((H, int((e1 - e0) * s), 3), 22, np.uint8)
    px = lambda e, n: (int((e - e0) * s), int(H - 70 - (n - n0) * s))
    for g in np.arange(math.ceil(e0), e1, 1.0):
        cv2.line(img, px(g, n0), px(g, n1), (45, 45, 45), 1)
    for g in np.arange(math.ceil(n0), n1, 1.0):
        cv2.line(img, px(e0, g), px(e1, g), (45, 45, 45), 1)
    for a, b in truth_lines:
        cv2.line(img, px(a[1], a[0]), px(b[1], b[0]), (255, 255, 255), 2)
    for (e, n), (e2, n2) in zip(track[::5], track[5::5]):
        cv2.line(img, px(e, n), px(e2, n2), (230, 180, 60), 1)
    c = cm.crumbs()
    for x, y, b, seen in zip(c["x"], c["y"], c["brightness"], c["seen"]):
        f = min(1.0, max(0.0, (b - cm.threshold) / max(1, 255 - cm.threshold)))
        col = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[int(120 - 120 * f), 220, 255]]]),
                                                 cv2.COLOR_HSV2BGR)[0, 0])
        cv2.circle(img, px(x, y), 3, col if not seen else tuple(v // 3 for v in col), -1)
    gs = find(c["x"], c["y"], c["up"])
    if len(c["x"]):
        from robotx_graey_2026.api.sonar.outlines import dist_to_polyline
        pts = np.column_stack([c["y"], c["x"]])                       # (n, e)
        line = np.array([truth_lines[0][0]] + [b for _, b in truth_lines])
        off = dist_to_polyline(pts, line)
        far = find(c["x"], c["y"], c["up"], link_m=1.5, min_crumbs=4)
        bunch = max(far, key=lambda g: g["n"]) if far else None
        where = ""
        if bunch:
            m = pts[bunch["members"]].mean(axis=0)
            where = f", biggest 1.5 m bunch {bunch['n']} crumbs, its middle {dist_to_polyline(m[None], line)[0]:.2f} m from the pipe"
        print(f"{len(off)} crumbs, {np.mean(off < 0.5):.0%} within 0.5 m of the real shape{where}")
    else:
        print("0 crumbs")
    cv2.putText(img, f"{title}, floor cut {cm.floor_cut_m:g} m: {len(c['x'])} crumbs, best "
                + (f"pipe {gs[0]['score']:.0%} ({gs[0]['length']:.1f} m)" if gs else "none"),
                (10, H - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, "white = real shape, dots = crumbs (blue weak -> red strong), orange = sub's track, grid 1 m",
                (10, H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.imwrite(out, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", choices=["pool", "pipe-along", "pipe-turn"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--range", type=float)
    ap.add_argument("--height", type=float, default=2.5, help="m, sonar above the pipe's top")
    ap.add_argument("--threshold", type=int, default=100)
    ap.add_argument("--floor-cut", type=float, default=0.5, help="m; 0 = off")
    ap.add_argument("--pipe", default="0,0,0", help="N,E,heading of the pipe's start (live sim: 3,2,30)")
    ap.add_argument("--at", help="N,E to turn at (pipe-turn); default above the pipe's middle")
    ap.add_argument("--depth", type=float, help="m below the surface to turn at, instead of --height")
    a = ap.parse_args()

    if a.scene == "pool":
        L, Wd, D = POOL["length"], POOL["width"], POOL["depth"]
        world = pool_world(L, Wd, D, 0.0, 0.0, 0.0)
        # back and forth 1.5 m off the left wall, then along the far half, like a hand drive
        path = legs_path([(1.0, 1.5), (9.0, 1.5), (9.0, 3.5), (1.0, 3.5), (1.0, 1.5), (9.0, 1.5)],
                         0.3, 0.2, 8.0)
        cm, track = run(world, path, a.range or 4.0, D, threshold=a.threshold, floor_cut=a.floor_cut)
        corners = [(0, 0), (L, 0), (L, Wd), (0, Wd), (0, 0)]
        draw(cm, track, [((p[0], p[1]), (q[0], q[1])) for p, q in zip(corners, corners[1:])],
             a.out, "pool, driven back and forth", ((-1, Wd + 1), (-1, L + 1)))
        return

    depth = 12.0
    pn, pe, ph = (float(v) for v in a.pipe.split(","))
    world = pipeline_world(depth, pn, pe, ph)
    _, centre = build_world(depth, pn, pe, ph)
    top = centre[:, 2].min() - PIPE_R
    z = a.depth if a.depth is not None else top - a.height
    if a.scene == "pipe-along":
        pts = [tuple(centre[0, :2] - (1.0, 0))] + [tuple(p[:2]) for p in centre[1:]]
        path = legs_path(pts, z, 0.15, 8.0)
        title = f"pipeline, flown along it {a.height} m above"
    else:
        mid = centre[:, :2].mean(axis=0) if not a.at else [float(v) for v in a.at.split(",")]
        path = turn_path(mid[0], mid[1], z, 4.0, 360.0)
        title = f"pipeline, turning on the spot {top - z:.1f} m above it"
    cm, track = run(world, path, a.range or 6.0, depth, threshold=a.threshold, floor_cut=a.floor_cut)
    lines = [((p[0], p[1]), (q[0], q[1])) for p, q in zip(centre, centre[1:])]
    lo = np.minimum(centre[:, :2].min(axis=0), track[:, ::-1].min(axis=0))
    hi = np.maximum(centre[:, :2].max(axis=0), track[:, ::-1].max(axis=0))
    draw(cm, track, lines, a.out, title, ((lo[1] - 2.5, hi[1] + 2.5), (lo[0] - 2.0, hi[0] + 2.0)))


if __name__ == "__main__":
    main()
