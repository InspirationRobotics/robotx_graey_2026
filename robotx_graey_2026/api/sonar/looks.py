"""Combining looks: where several pings agree, that is where the echo really was.

A ping says how far away an echo is and its angle in the scan plane - both
sharp - but not where inside the beam's 25 deg fore-aft it came from. So one
echo only says "somewhere on this short arc": up to 0.22 x range either side of
the crumb that crumbs.py puts in the middle (in the sim, crumbs 7 m out were a
median 0.65 m off the pipe, all of it along the sub's heading, sideways ~1 cm).

Each echo votes along its whole arc instead, most in the middle where the beam
is loudest (half as much at 12.5 deg). As the sub turns or moves it hears the
same thing from other places and headings; their arcs all pass through the
real spot and cross nowhere else, so the votes pile up there (Ruth's choice,
Oct 9: "combine looks").

The votes go on a grid of CELL_M squares. A cell counts as agreed when at least
MIN_LOOKS different sweeps voted in it and its votes are at least SHARE of the
most in any cell within AROUND cells, which keeps the ridge of the pipe and drops
the arcs' lonely ends. Tuned in the sim (far scan, close scan, flying along):
agreed crumbs within 0.3 m of the pipe 79 / 100 / 99 %, vs 77 / 85 / 94 % for
one-look crumbs. No fading yet: agreed crumbs stay until Reset. Each agreed cell becomes one crumb:
its middle, the height its votes say, and how many looks agreed.

Height: every vote carries the height (up from the DVL) of its spot on the arc,
so a cell also learns how high the thing in it is. The floor cut applies to
each vote's own height.
"""
import math

import numpy as np

from . import settings as S

CELL_M = 0.1
HALF_M = 40.0           # the grid covers +-this around (0, 0)
MIN_LOOKS = 3           # sim, Oct 9: 2 let through cells where just two arcs overlapped
SHARE = 0.5
AROUND = 4              # cells each way (0.4 m) that a cell's votes are compared with
FAN_SAMPLES = 31
FAN_SPAN_DEG = 15.0     # votes out to here; the beam is at half power at 12.5


class LookMap:
    def __init__(self, cell_m=CELL_M, half_m=HALF_M, min_looks=MIN_LOOKS):
        self.cell, self.half, self.min_looks = cell_m, half_m, min_looks
        n = int(round(2 * half_m / cell_m))
        self.votes = np.zeros((n, n), np.float32)
        self.up_sum = np.zeros((n, n), np.float32)
        self.looks = np.zeros((n, n), np.int16)
        self.last = np.full((n, n), -1, np.int32)
        phi = np.radians(np.linspace(-FAN_SPAN_DEG, FAN_SPAN_DEG, FAN_SAMPLES))
        w = 10 ** (-0.3 * (np.degrees(phi) / (S.BEAM_FORE_AFT_DEG / 2.0)) ** 2)
        self._phi, self._w = phi, (w / w.sum()).astype(np.float32)

    def clear(self):
        for a in (self.votes, self.up_sum, self.looks):
            a[:] = 0
        self.last[:] = -1

    def add(self, sonar_xyz, body_to_map, angle_deg, ranges, look_id, alt_m=-1.0, floor_cut_m=0.0):
        """Votes for the echoes of one ping. sonar_xyz: (x, y, up) of the sonar on
        the map; body_to_map: crumbs.body_to_map; ranges: metres of each echo."""
        a = math.radians(angle_deg)
        # every direction across the fan, in the sub's frame (forward, right, down) -> map
        d_body = np.column_stack([np.sin(self._phi), np.cos(self._phi) * math.cos(a),
                                  -np.cos(self._phi) * math.sin(a)])
        n_, e_, dn = body_to_map @ d_body.T
        sx, sy, sup = sonar_xyz
        for r in np.atleast_1d(ranges):
            x, y, up, w = sx + r * e_, sy + r * n_, sup - r * dn, self._w
            if floor_cut_m > 0 and alt_m > 0:
                keep = alt_m + up >= floor_cut_m
                x, y, up, w = x[keep], y[keep], up[keep], w[keep]
            i = np.floor((x + self.half) / self.cell).astype(int)
            j = np.floor((y + self.half) / self.cell).astype(int)
            ok = (i >= 0) & (j >= 0) & (i < self.votes.shape[0]) & (j < self.votes.shape[1])
            i, j, up, w = i[ok], j[ok], up[ok], w[ok]
            np.add.at(self.votes, (i, j), w)
            np.add.at(self.up_sum, (i, j), w * up)
            new = self.last[i, j] != look_id
            self.looks[i[new], j[new]] += 1
            self.last[i, j] = look_id

    def agreed(self, min_looks=None, share=SHARE, around=AROUND):
        """The agreed cells as crumbs: x, y, up, votes, looks (arrays)."""
        cand = self.looks >= (self.min_looks if min_looks is None else min_looks)
        if not cand.any():
            return {k: np.zeros(0) for k in ("x", "y", "up", "votes", "looks")}
        ii, jj = np.nonzero(cand)
        v = self.votes
        # best votes in the cells around each candidate, `around` cells each way
        pad = np.pad(v, around)
        k = 2 * around + 1
        best = np.max([pad[ii + di, jj + dj] for di in range(k) for dj in range(k)], axis=0)
        keep = v[ii, jj] >= share * best
        ii, jj = ii[keep], jj[keep]
        return {"x": (ii + 0.5) * self.cell - self.half, "y": (jj + 0.5) * self.cell - self.half,
                "up": self.up_sum[ii, jj] / np.maximum(v[ii, jj], 1e-9),
                "votes": v[ii, jj].astype(float), "looks": self.looks[ii, jj].astype(float)}
