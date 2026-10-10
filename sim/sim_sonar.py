#!/usr/bin/env python3
"""A simulated Ping360 for the software-in-the-loop, and the world it hears.

SimSonar is a drop-in for api/sonar/sweep.Sonar: same constructor, same
sweep(), same Sweep back, ping times included. Each ping is sent from where the
simulator says the sonar really is (SIM_STATE: position and full attitude, the
sonar SONAR_FWD_M ahead of the DVL), in the real beam shape - 2 deg wide in the
scan plane, 25 deg fore-aft - and returns the nearest echo of each thing in it.

THE WORLD (NED metres from the sim's home, which is the active buoy):
  seafloor   flat, WATER_DEPTH down
  pipeline   handbook 3.5.5, figure and parts list, to scale. Three straight
             TEE SECTIONS, all parallel: 450 mm pipe, tee, 450 mm pipe (~1.0 m).
             Between them two 1000 mm CONNECTORS on 45 deg elbows, the first
             jogging left, the second back right, all level - as in the figure.
             3" pipe (88.9 mm OD). ~5.2 m of pipe, ~4.6 m end to end, 0.74 m jog.
  legs       one under each tee: 200 mm 3" stub, 3"-2" reducer, 1500 mm 2" pipe
             (60.3 mm OD) into a base frame on the floor. That puts the pipe's
             centre ~1.86 m above the floor (handbook: "1-2 m").
  bases      2" pipe squares, ~0.78 m a side with a cross through the middle
             (scaled off the figure), lying on the floor
  light boxes FOUR, at random places along the pipe (Ruth, from the updated
             handbook), on top, ~0.15 m tall. PIPE_SEED picks the places.
Fitting sizes are typical 3" Sch 40 socket fittings, not measured: a tee adds
~49 mm from its centre to each socket bottom, a 45 deg elbow ~22 mm.
Placement: PIPE_N, PIPE_E (the open end of the first tee section), PIPE_HDG
(degrees, along the tee sections) in the environment; default 3 m north, 2 m
east, heading 30.

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
PIPE_R = 0.0889 / 2                         # 3" Sch 40, 3.500" OD
LEG_R = 0.0603 / 2                          # 2" Sch 40, 2.375" OD
TEE_M, ELBOW_M = 0.049, 0.022               # centre to socket bottom (typical)
PIPE_450, PIPE_1000, STUB_200, LEG_1500 = 0.45, 1.0, 0.2, 1.5
REDUCER_M = 0.04                            # 3"-2" coupler, laying length (typical)
BASE_SIDE_M = 0.78
N_BOXES = 4
BASE_TEE_M = 0.038                          # 2" base cross, centre to socket bottom (typical)


def _line(a, b, step=0.02):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    return a + (b - a) * np.linspace(0, 1, n)[:, None]


def build_world(depth, n0, e0, hdg_deg):
    """Points (N, E, D) for each kind of object, plus the pipe's centre line
    (its open ends and elbow corners)."""
    # in the pipeline's own frame: x along the tee sections, y right, z UP from the floor
    # pipe centre height: base cross on the floor, its socket, the leg, reducer, stub, tee
    h = LEG_R + BASE_TEE_M + LEG_1500 + REDUCER_M + STUB_200 + TEE_M
    end_run = PIPE_450 + 2 * TEE_M + PIPE_450     # open end -> tee -> into the elbow
    tee_section = ELBOW_M + end_run + ELBOW_M     # elbow corner to elbow corner
    connector = ELBOW_M + PIPE_1000 + ELBOW_M
    d45 = math.sqrt(0.5)
    p = [np.array([0.0, 0.0, h])]
    for d, length in (((1, 0, 0), end_run + ELBOW_M),          # tee section 1
                      ((d45, -d45, 0), connector),             # 45 deg left
                      ((1, 0, 0), tee_section),                # tee section 2
                      ((d45, d45, 0), connector),              # 45 deg back right
                      ((1, 0, 0), ELBOW_M + end_run)):         # tee section 3
        p.append(p[-1] + length * np.array(d, float))
    tees = [p[0] + (PIPE_450 + TEE_M) * np.array([1.0, 0, 0]),
            (p[2] + p[3]) / 2,
            p[5] - (PIPE_450 + TEE_M) * np.array([1.0, 0, 0])]
    hd = math.radians(hdg_deg)

    def to_ned(xyz):
        xyz = np.atleast_2d(xyz)
        n = n0 + xyz[:, 0] * math.cos(hd) - xyz[:, 1] * math.sin(hd)
        e = e0 + xyz[:, 0] * math.sin(hd) + xyz[:, 1] * math.cos(hd)
        return np.column_stack([n, e, depth - xyz[:, 2]])

    pipe = np.vstack([_line(a, b) for a, b in zip(p, p[1:])])
    legs = np.vstack([_line(t, (t[0], t[1], LEG_R)) for t in tees])
    half = BASE_SIDE_M / 2
    bases = []
    for t in tees:
        c = np.array([t[0], t[1], LEG_R])
        corners = [c + (sx * half, sy * half, 0) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]
        bases += [_line(a, b) for a, b in zip(corners, corners[1:])]
        bases += [_line(c - (half, 0, 0), c + (half, 0, 0)), _line(c - (0, half, 0), c + (0, half, 0))]
    # light boxes: four, at random distances along the pipe, at least 0.4 m apart
    rng = np.random.default_rng(int(os.environ.get("PIPE_SEED", "7")))
    along = np.cumsum([0.0] + [float(np.linalg.norm(b - a)) for a, b in zip(p, p[1:])])
    while True:
        at = np.sort(rng.uniform(0.15, along[-1] - 0.15, N_BOXES))
        if np.all(np.diff(at) > 0.4):
            break
    boxes = []
    g = np.mgrid[-0.1:0.11:0.05, -0.075:0.08:0.05, 0:0.151:0.05].reshape(3, -1).T
    for d in at:
        k = min(int(np.searchsorted(along, d)) - 1, len(p) - 2)
        k = max(k, 0)
        c = p[k] + (p[k + 1] - p[k]) * (d - along[k]) / (along[k + 1] - along[k])
        boxes.append(c + np.array([0, 0, PIPE_R]) + g)
    return ({"pipe": to_ned(pipe), "leg": to_ned(legs), "base": to_ned(np.vstack(bases)),
             "box": to_ned(np.vstack(boxes))}, to_ned(np.array(p)))


class SimSonar:
    """Same interface as api.sonar.sweep.Sonar."""
    STRENGTH = {"box": 230, "pipe": 175, "leg": 150, "base": 150}
    RADIUS = {"box": 0.05, "pipe": PIPE_R, "leg": LEG_R, "base": LEG_R}

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
