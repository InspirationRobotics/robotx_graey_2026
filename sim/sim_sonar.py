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

Echoes: sonar_physics.py traces rays through the real beam shape, with shadows,
and surfaces that echo by material and angle (Oct 9). Its scale is set so a 3"
pipe 2.25 m away reads ~125, as Ruth's real one did; the rest is to be checked
against her real pool run.
"""
import math
import os
import threading
import time

import numpy as np

from robotx_graey_2026.api.pixhawk.mavlink import Link
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar.sweep import Sweep

from sonar_physics import World
from sonar_physics import ping as physics_ping

N_BINS = 540
PIPE_R = 0.0889 / 2                         # 3" Sch 40, 3.500" OD
LEG_R = 0.0603 / 2                          # 2" Sch 40, 2.375" OD
TEE_M, ELBOW_M = 0.049, 0.022               # centre to socket bottom (typical)
PIPE_450, PIPE_1000, STUB_200, LEG_1500 = 0.45, 1.0, 0.2, 1.5
REDUCER_M = 0.04                            # 3"-2" coupler, laying length (typical)
BASE_SIDE_M = 0.78
N_BOXES = 4
BASE_TEE_M = 0.038                          # 2" base cross, centre to socket bottom (typical)
BOX_L, BOX_W, BOX_H = 0.20, 0.15, 0.15      # light box (Ruth: ~0.15 m tall)


def _line(a, b, step=0.02):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    return a + (b - a) * np.linspace(0, 1, n)[:, None]


def _geometry(depth, n0, e0, hdg_deg):
    """The pipeline's shape, in NED: elbow corners, tees, and the light boxes'
    centres and along-pipe directions."""
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
    # light boxes: four, at random distances along the pipe, at least 0.4 m apart
    rng = np.random.default_rng(int(os.environ.get("PIPE_SEED", "7")))
    along = np.cumsum([0.0] + [float(np.linalg.norm(b - a)) for a, b in zip(p, p[1:])])
    while True:
        at = np.sort(rng.uniform(0.15, along[-1] - 0.15, N_BOXES))
        if np.all(np.diff(at) > 0.4):
            break
    boxes = []
    for d in at:
        k = max(min(int(np.searchsorted(along, d)) - 1, len(p) - 2), 0)
        u = (p[k + 1] - p[k]) / np.linalg.norm(p[k + 1] - p[k])
        boxes.append((p[k] + u * (d - along[k]) + np.array([0, 0, PIPE_R + BOX_H / 2]), u))
    hd = math.radians(hdg_deg)

    def to_ned(xyz):
        xyz = np.atleast_2d(xyz)
        n = n0 + xyz[:, 0] * math.cos(hd) - xyz[:, 1] * math.sin(hd)
        e = e0 + xyz[:, 0] * math.sin(hd) + xyz[:, 1] * math.cos(hd)
        return np.column_stack([n, e, depth - xyz[:, 2]])

    def dir_ned(v):
        return np.array([v[0] * math.cos(hd) - v[1] * math.sin(hd),
                         v[0] * math.sin(hd) + v[1] * math.cos(hd), -v[2]])

    return {"corners": to_ned(np.array(p)), "tees": to_ned(np.array(tees)), "h": h,
            "boxes": [(to_ned(c)[0], dir_ned(u)) for c, u in boxes], "to_ned": to_ned}


def build_world(depth, n0, e0, hdg_deg):
    """Points (N, E, D) for each kind of object, plus the pipe's centre line
    (its open ends and elbow corners). For checking crumbs against."""
    g = _geometry(depth, n0, e0, hdg_deg)
    p, tees = g["corners"], g["tees"]
    floor_d = depth - LEG_R
    pipe = np.vstack([_line(a, b) for a, b in zip(p, p[1:])])
    legs = np.vstack([_line(t, (t[0], t[1], floor_d)) for t in tees])
    bases = []
    for a, b in _base_pipes(tees, floor_d):
        bases.append(_line(a, b))
    box_pts = []
    gr = np.mgrid[-0.1:0.11:0.05, -0.075:0.08:0.05, -0.075:0.08:0.05].reshape(3, -1).T
    for c, u in g["boxes"]:
        across = np.cross(u, (0, 0, 1.0))
        box_pts.append(c + gr @ np.vstack([u, across, (0, 0, 1.0)]))
    return ({"pipe": pipe, "leg": legs, "base": np.vstack(bases), "box": np.vstack(box_pts)}, p)


def _base_pipes(tees, floor_d):
    """Each base frame: a square of 2" pipe with a cross through the middle."""
    half, out = BASE_SIDE_M / 2, []
    for t in tees:
        c = np.array([t[0], t[1], floor_d])
        corners = [c + (sx * half, sy * half, 0) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]
        out += list(zip(corners, corners[1:]))
        out += [(c - (half, 0, 0), c + (half, 0, 0)), (c - (0, half, 0), c + (0, half, 0))]
    return out


def pipeline_world(depth, n0, e0, hdg_deg):
    """The same pipeline as solid shapes for sonar_physics, on a mud seafloor
    under a flat water surface."""
    g = _geometry(depth, n0, e0, hdg_deg)
    w = World()
    w.plane((0, 0, depth), (0, 0, 1), "mud")
    w.plane((0, 0, 0), (0, 0, 1), "surface")
    p, tees, floor_d = g["corners"], g["tees"], depth - LEG_R
    for a, b in zip(p, p[1:]):
        w.capsule(a, b, PIPE_R, "pvc")
    for t in tees:
        w.capsule(t, (t[0], t[1], floor_d), LEG_R, "pvc")
    for a, b in _base_pipes(tees, floor_d):
        w.capsule(a, b, LEG_R, "pvc")
    for c, u in g["boxes"]:
        across = np.cross(u, (0, 0, 1.0))
        w.box(c, np.vstack([u, across, (0, 0, 1.0)]), (BOX_L / 2, BOX_W / 2, BOX_H / 2), "plastic")
    return w


def pool_world(length, width, depth, n0, e0, hdg_deg):
    """A rectangular concrete pool: one corner at (n0, e0), the long side along
    hdg_deg, the other side to its right."""
    h = math.radians(hdg_deg)
    u = np.array([math.cos(h), math.sin(h), 0.0])         # along the long side
    v = np.array([-math.sin(h), math.cos(h), 0.0])        # across, to the right
    c0 = np.array([n0, e0, 0.0])
    w = World()
    w.plane((0, 0, depth), (0, 0, 1), "concrete")
    w.plane((0, 0, 0), (0, 0, 1), "surface")
    w.plane(c0, u, "concrete")
    w.plane(c0 + length * u, u, "concrete")
    w.plane(c0, v, "concrete")
    w.plane(c0 + width * v, v, "concrete")
    return w


class SimSonar:
    """Same interface as api.sonar.sweep.Sonar."""

    def __init__(self, device=None, udp=None, down_gradian=S.DOWN_GRADIAN, **_):
        self.down_gradian = down_gradian
        self.depth = float(os.environ.get("WATER_DEPTH", "12"))
        self.home = (float(os.environ["HOME_LAT"]), float(os.environ["HOME_LON"]))
        self.world, self.centre_line = build_world(
            self.depth, float(os.environ.get("PIPE_N", "3")), float(os.environ.get("PIPE_E", "2")),
            float(os.environ.get("PIPE_HDG", "30")))
        place = (self.depth, float(os.environ.get("PIPE_N", "3")),
                 float(os.environ.get("PIPE_E", "2")), float(os.environ.get("PIPE_HDG", "30")))
        self.solid = pipeline_world(*place)
        self.truth_boxes = [c for c, _ in _geometry(*place)["boxes"]]
        self.metres_per_bin = None
        self.settings = {"simulated": True}
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

    def truth_ned(self):
        """Where the sub really is: north, east from home, yaw (rad); None before data."""
        m = self._state
        if m is None:
            return None
        lat0, lon0 = self.home
        n = math.radians(m.lat_int / 1e7 - lat0) * 6378137.0
        e = math.radians(m.lon_int / 1e7 - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        return n, e, m.yaw

    def _ping(self, a_deg, max_range):
        sonar, r = self._pose()
        return physics_ping(self.solid, sonar, r, a_deg, max_range, N_BINS, self._rng)

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
