#!/usr/bin/env python3
"""A simulated Ping360 for the software-in-the-loop, and the world it hears.

SimSonar is a drop-in for api/sonar/sweep.Sonar: same constructor, same
sweep(), same Sweep back, ping times included. Each ping is sent from where the
simulator says the sonar really is (SIM_STATE: position and full attitude, the
sonar SONAR_FWD_M ahead of the DVL), in the real beam shape - 2 deg wide in the
scan plane, 25 deg fore-aft - and returns the nearest echo of each thing in it.

THE WORLD (NED metres from the sim's home, which is the active buoy):
  seafloor   flat, WATER_DEPTH down
  pipeline   from the handbook (3.5.5): five straight sections joined by 45 deg
             elbows, not all in one plane - three 0.95 m tee sections (two 450 mm
             pipes and a tee, with a 2" leg down to the floor) and two 1.0 m
             connectors, ~4.9 m of 3" pipe, 1.0-1.7 m above the floor
  light boxes three, ~0.15 m tall, sitting on the pipe
Placement: PIPE_N, PIPE_E (start of the pipe), PIPE_HDG (degrees) in the
environment, default 3 m north, 2 m east, heading 30.

Echo strengths are invented but ordered like the real ones: box > pipe > leg >
floor > background. They are for testing the code, not for tuning thresholds.
"""
import math
import os
import threading
import time

import numpy as np

from robotx_graey_2026.api.pixhawk.mavlink import Link
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar.sweep import Sweep

N_BINS = 540
FAN_DEG = S.BEAM_FORE_AFT_DEG / 2.0
SLICE_DEG = S.BEAM_IN_PLANE_DEG / 2.0
PIPE_R = 0.089 / 2
LEG_R = 0.060 / 2


def _line(a, b, step=0.02):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    return a + (b - a) * np.linspace(0, 1, n)[:, None]


def build_world(depth, n0, e0, hdg_deg):
    """Points (N, E, D) for each kind of object, plus the pipe's centre line."""
    # pipe path in its own frame: x along the start heading, y right, z UP from the floor
    p = [np.array([0.0, 0.0, 1.0])]
    for d, length in (((1, 0, 0), 0.95),                 # tee section 1
                      ((0.7071, 0, 0.7071), 1.0),        # elbow up 45, connector
                      ((1, 0, 0), 0.95),                 # elbow level, tee section 2
                      ((0.7071, 0.7071, 0), 1.0),        # elbow right 45, connector
                      ((0, 1, 0), 0.95)):                # elbow right 45, tee section 3
        p.append(p[-1] + length * np.array(d))
    tees = [(p[0] + p[1]) / 2, (p[2] + p[3]) / 2, (p[4] + p[5]) / 2]
    h = math.radians(hdg_deg)

    def to_ned(xyz):
        xyz = np.atleast_2d(xyz)
        n = n0 + xyz[:, 0] * math.cos(h) - xyz[:, 1] * math.sin(h)
        e = e0 + xyz[:, 0] * math.sin(h) + xyz[:, 1] * math.cos(h)
        return np.column_stack([n, e, depth - xyz[:, 2]])

    pipe = to_ned(np.vstack([_line(a, b) for a, b in zip(p, p[1:])]))
    legs = to_ned(np.vstack([_line(t, (t[0], t[1], 0.0)) for t in tees]))
    # light boxes: on top of the pipe at 20 %, 50 % and 85 % along it
    centre = to_ned(np.vstack([_line(a, b) for a, b in zip(p, p[1:])]))
    boxes = []
    for f in (0.20, 0.50, 0.85):
        c = centre[int(f * (len(centre) - 1))] + np.array([0, 0, -(PIPE_R + 0.075)])
        g = np.mgrid[-0.1:0.11:0.05, -0.075:0.08:0.05, -0.075:0.08:0.05].reshape(3, -1).T
        boxes.append(c + g)
    return {"pipe": pipe, "leg": legs, "box": np.vstack(boxes)}, to_ned(np.array(p))


class SimSonar:
    """Same interface as api.sonar.sweep.Sonar."""
    STRENGTH = {"box": 230, "pipe": 175, "leg": 150}
    RADIUS = {"box": 0.05, "pipe": PIPE_R, "leg": LEG_R}

    def __init__(self, device=None, udp=None, down_gradian=S.DOWN_GRADIAN, **_):
        self.down_gradian = down_gradian
        self.depth = float(os.environ.get("WATER_DEPTH", "12"))
        self.home = (float(os.environ["HOME_LAT"]), float(os.environ["HOME_LON"]))
        self.world, self.centre_line = build_world(
            self.depth, float(os.environ.get("PIPE_N", "3")), float(os.environ.get("PIPE_E", "2")),
            float(os.environ.get("PIPE_HDG", "30")))
        self.metres_per_bin = None
        self._state = None
        self._rng = np.random.default_rng(1)
        self._link = Link(os.environ.get("SIM_SONAR_MAVLINK", "udpout:127.0.0.1:14559"), 185)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        while True:
            self._link.drain(lambda k, m: setattr(self, "_state", m) if k == "SIM_STATE" else None)
            time.sleep(0.005)

    def _pose(self):
        m = self._state
        while m is None:
            time.sleep(0.05)
            m = self._state
        lat0, lon0 = self.home
        n = math.radians(m.lat_int / 1e7 - lat0) * 6378137.0
        e = math.radians(m.lon_int / 1e7 - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        w, x, y, z = m.q1, m.q2, m.q3, m.q4
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        sonar = np.array([n, e, -m.alt]) + r @ np.array([S.SONAR_FWD_M, S.SONAR_RIGHT_M, 0.0])
        return sonar, r

    def _ping(self, a_deg, max_range):
        sonar, r = self._pose()
        mpb = max_range / N_BINS
        row = self._rng.integers(0, 25, N_BINS).astype(float)
        for kind, pts in self.world.items():
            v = (pts - sonar) @ r                       # into the body frame (forward, right, down)
            across = np.hypot(v[:, 1], v[:, 2])
            rng = np.linalg.norm(v, axis=1)
            bearing = np.degrees(np.arctan2(-v[:, 2], v[:, 1]))
            spread = SLICE_DEG + np.degrees(self.RADIUS[kind] / np.maximum(rng, 0.1))
            hit = ((np.abs((bearing - a_deg + 180) % 360 - 180) <= spread)
                   & (np.abs(np.degrees(np.arctan2(v[:, 0], across))) <= FAN_DEG)
                   & (rng < max_range))
            if hit.any():
                k = np.argmin(np.where(hit, rng, np.inf))
                c = int(rng[k] / mpb)
                fade = 1.0 - 0.3 * abs(math.degrees(math.atan2(v[k, 0], across[k]))) / FAN_DEG
                row[c:c + max(3, int(0.06 / mpb))] = np.maximum(
                    row[c:c + max(3, int(0.06 / mpb))], self.STRENGTH[kind] * fade)
        # the seafloor: where the fan meets the plane, nearest to farthest
        a = math.radians(a_deg)
        dists = []
        for fa in np.radians(np.linspace(-FAN_DEG, FAN_DEG, 7)):
            d_body = np.array([math.sin(fa), math.cos(fa) * math.cos(a), -math.cos(fa) * math.sin(a)])
            down = (r @ d_body)[2]
            if down > 1e-3:
                dists.append((self.depth - sonar[2]) / down)
        if dists and min(dists) < max_range:
            c0, c1 = int(min(dists) / mpb), int(min(max(dists), max_range) / mpb)
            row[c0:c1 + 1] = np.maximum(row[c0:c1 + 1], 115)
        row[: int(S.MIN_RANGE_M / mpb)] *= 0.3          # near-field ringing, as the real one
        return np.clip(row, 0, 255).astype(np.uint8)

    def sweep(self, start_deg, end_deg, step_deg, max_range_m, heading_deg=None, on_ping=None):
        self.metres_per_bin = max_range_m / N_BINS
        angles, a = [], float(start_deg)
        while a <= end_deg + 1e-9:
            angles.append(a % 360.0)
            a += step_deg
        rows, times = [], []
        for ang in angles:
            time.sleep(0.03 + 2 * max_range_m / 1500.0)    # the real head's pace, roughly
            rows.append(self._ping(ang, max_range_m))
            times.append(time.monotonic())
            if on_ping is not None:
                on_ping(Sweep(np.vstack(rows), angles[:len(rows)], self.metres_per_bin,
                              heading_deg, times))
        return Sweep(np.vstack(rows), angles, self.metres_per_bin, heading_deg, times)
