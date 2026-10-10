"""What a Ping360 ping hears, by tracing rays through its beam.

The beam is 2 deg wide in the scan plane and 25 deg fore-aft (half power at the
edges, fading beyond). One ping = ~700 rays spread over that shape, each
weighted by how loud the beam is in its direction. Every ray stops at the first
thing it hits - so things behind a nearer thing are in its shadow - and sends
back an echo that depends on what the surface is and the angle it is hit at:

  mud seafloor   rough: echoes from any angle, louder the more head-on (Lambert)
  concrete       rough, plus a strong mirror-like flash when hit head-on
  PVC, plastic   smooth: mostly the head-on flash, little from a slant
  water surface  a mirror: only head-on

The echoes are added up by range, blurred by the pulse length, given speckle
(the grainy look of real sonar), and turned into the 0-255 the Ping360 reports
on a log scale. GAIN_DB and RANGE_DB set that scale; they are guesses to be
tuned against Ruth's real pool runs, where the pipe peaked ~125 at 2.25 m. ALL the
material numbers are placeholders until the next pool session's recordings
(sim/POOL_CALIBRATION.md) - don't read the sim's detection ranges as real.

World pieces (NED metres): planes (floor, walls, the surface), capsules (pipes:
a segment and a radius) and boxes (centre, three axes, half sizes).
"""
import math

import numpy as np

IN_PLANE_HALF_DEG = 1.0         # half-power half-widths (the 2 x 25 deg beam)
FORE_AFT_HALF_DEG = 12.5
STEP_DEG = 0.5                  # ray spacing
SPAN_IN, SPAN_FA = 2.0, 20.0    # rays out to these angles, past half power

GAIN_DB = 49.0                  # set so a 3" pipe 2.25 m below reads ~125, as Ruth's real one did
RANGE_DB = 45.0                 # dB from silence to 255
NOISE = 12.0                    # mean background level, 0-255

#               diffuse, flash, flash sharpness. Intensities relative to a perfect
#               mirror; the flash is what comes straight back from a surface hit
#               near head-on (cos ** sharpness), the diffuse part from any angle (cos ** 2)
MATERIALS = {"mud": (0.01, 0.05, 4),            # soft, rough; ~-13 dB head-on, ~-20 dB slanted
             "concrete": (0.02, 0.40, 8),       # hard, a bit rough
             "pvc": (0.06, 0.06, 4),            # PLACEHOLDER: echoes from any side alike.
                                                # A mirror-like pipe (0.005, 0.14, 30) went silent
                                                # seen end-on; Ruth doubts that - set from pool data
             "plastic": (0.01, 0.30, 12),       # light box housing
             "surface": (0.0, 1.0, 60)}         # water to air: a near-perfect mirror

_d = np.radians(np.arange(-SPAN_IN, SPAN_IN + 1e-9, STEP_DEG))
_f = np.radians(np.arange(-SPAN_FA, SPAN_FA + 1e-9, STEP_DEG))
_DD, _FF = (a.ravel() for a in np.meshgrid(_d, _f))
_W = 10 ** (-0.3 * ((np.degrees(_DD) / IN_PLANE_HALF_DEG) ** 2
                    + (np.degrees(_FF) / FORE_AFT_HALF_DEG) ** 2))
_W = _W / _W.sum()               # a beam full of one surface sums to 1


class World:
    def __init__(self):
        self.planes, self.capsules, self.boxes = [], [], []

    def plane(self, point, normal, material):
        n = np.asarray(normal, float)
        self.planes.append((np.asarray(point, float), n / np.linalg.norm(n), material))

    def capsule(self, a, b, radius, material):
        self.capsules.append((np.asarray(a, float), np.asarray(b, float), float(radius), material))

    def box(self, centre, axes, half, material):
        self.boxes.append((np.asarray(centre, float), np.asarray(axes, float), np.asarray(half, float), material))


def _hit_planes(o, d, world, t, nrm, mat):
    for p, n, m in world.planes:
        dn = d @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = ((p - o) @ n) / dn
        ok = (np.abs(dn) > 1e-9) & (tt > 1e-6) & (tt < t)
        t[ok], nrm[ok], mat[ok] = tt[ok], n, m


def _hit_capsules(o, d, world, t, nrm, mat):
    for a, b, r, m in world.capsules:
        ba, oa = b - a, o - a
        baba, bard, baoa = ba @ ba, d @ ba, ba @ oa
        rdoa, oaoa = d @ oa, oa @ oa
        A = baba - bard * bard
        B = baba * rdoa - baoa * bard
        C = baba * oaoa - baoa * baoa - r * r * baba
        h = B * B - A * C
        with np.errstate(invalid="ignore", divide="ignore"):
            tt = (-B - np.sqrt(h)) / A
            y = baoa + tt * bard
            body = (h >= 0) & (A > 1e-12) & (y > 0) & (y < baba)
        tt_body = np.where(body, tt, np.inf)
        for end in (a, b):                          # the rounded ends

            oc = o - end
            bb = d @ oc
            cc = oc @ oc - r * r
            hh = bb * bb - cc
            with np.errstate(invalid="ignore"):
                te = -bb - np.sqrt(hh)
            te = np.where((hh >= 0) & (te > 1e-6), te, np.inf)
            tt_body = np.minimum(tt_body, te)
        ok = (tt_body > 1e-6) & (tt_body < t)
        if ok.any():
            pt = o + d[ok] * tt_body[ok, None]
            s = np.clip(((pt - a) @ ba) / baba, 0, 1)
            nn = pt - (a + s[:, None] * ba)
            nrm[ok] = nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-9)
            t[ok], mat[ok] = tt_body[ok], m


def _hit_boxes(o, d, world, t, nrm, mat):
    for c, axes, half, m in world.boxes:
        lo_ = (o - c) @ axes.T                     # ray in the box's own frame
        ld = d @ axes.T
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half - lo_) / ld
            t2 = (half - lo_) / ld
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        ok = (tmax >= tmin) & (tmin > 1e-6) & (tmin < t)
        if ok.any():
            face = np.argmax(np.minimum(t1, t2)[ok], axis=1)
            sign = -np.sign(ld[ok, face])
            nrm[ok] = axes[face] * sign[:, None]
            t[ok], mat[ok] = tmin[ok], m


def ping(world, origin, body_to_world, angle_deg, max_range, n_bins, rng, hits=None):
    """One ping: 0-255 levels in n_bins out to max_range. angle_deg is the sub's
    own scan angle (0 right, 90 up, 270 down); body frame is forward, right, down.
    hits: a list to append (point, bin, loudness) of every ray that hit, for
    checking where an echo really came from."""
    a = math.radians(angle_deg) + _DD
    d_body = np.column_stack([np.sin(_FF), np.cos(_FF) * np.cos(a), -np.cos(_FF) * np.sin(a)])
    d = d_body @ np.asarray(body_to_world).T
    n = len(d)
    t = np.full(n, np.inf)
    nrm = np.zeros((n, 3))
    mat = np.full(n, "", dtype=object)
    _hit_planes(origin, d, world, t, nrm, mat)
    _hit_capsules(origin, d, world, t, nrm, mat)
    _hit_boxes(origin, d, world, t, nrm, mat)

    mpb = max_range / n_bins
    energy = np.zeros(n_bins)
    hit = np.isfinite(t) & (t < max_range)
    if hit.any():
        cos_i = np.abs(np.einsum("ij,ij->i", d[hit], nrm[hit]))
        diffuse, flash, sharp = (np.array([MATERIALS[m][k] for m in mat[hit]]) for k in range(3))
        back = diffuse * cos_i ** 2 + flash * cos_i ** sharp
        np.add.at(energy, (t[hit] / mpb).astype(int), _W[hit] * back)
        if hits is not None:
            pts = origin + d[hit] * t[hit, None]
            hits.extend(zip(pts, (t[hit] / mpb).astype(int), _W[hit] * back))
    # pulse length blur, then speckle
    sigma = max(1.5, 0.02 / mpb)
    k = np.exp(-0.5 * (np.arange(-4 * sigma, 4 * sigma + 1) / sigma) ** 2)
    energy = np.convolve(energy, k / k.sum(), "same")
    energy *= rng.exponential(1.0, n_bins)
    level = 255.0 * (10 * np.log10(np.maximum(energy, 1e-30)) + GAIN_DB) / RANGE_DB
    level = np.maximum(level, rng.exponential(NOISE, n_bins))
    return np.clip(level, 0, 255).astype(np.uint8)
