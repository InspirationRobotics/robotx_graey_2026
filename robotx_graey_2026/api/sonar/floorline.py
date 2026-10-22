"""The floor filter (Ruth's approach, Oct 18-19): find the floor line in each sweep, then keep
only echoes in a height band ABOVE that line that stand out from their own beam.

Tested offline on all of Ruth's labelled sweeps (Oct 17 pool, Oct 18 lake), sure pipe marks kept /
junk dots per sweep: pool 90-100% / 0.4-0.5, lake 82-91% / 0.2-4 (the old fixed cut-off: 3-39 junk).

1. Floor line, one sweep, any shape. Per downward beam the floor candidates are its long echoes (peaks
   of a 0.2 m running mean, at least half the beam's best), so a thin pipe, rope or fish (water
   behind it) is not floor. Anchored straight below - on the DVL's floor if an echo agrees with it,
   else the strongest echoes there - and traced outward beam by beam. Points far from their
   neighbours (a rock top, the pipe) and steep stretches (> 50 deg: walls) are dropped.
2. Band: height above the floor line within (lo, hi) m, and never closer than NEAR_M below the sonar
   (its own ringing). Ruth: 0.75-2.25 m at the competition; 0.15-0.6 for the pool pipe.
3. Local contrast: echo / max(median of its beam +-0.4 m, leaving out +-0.06 m, NOISE_FLOOR) >= CONTRAST.
   Brightness changes with distance and water; standing out from the beam does not (Ruth, Oct 18).
4. One dot per connected patch (beam x range), at its strongest point.
No gain change and no votes across sweeps (Ruth, Oct 18)."""
import math

import numpy as np

NEAR_M = 0.45           # nothing closer to the sonar than this: the sonar's own ringing
RUN_M = 0.2             # a floor echo stays strong this far along the beam
DVL_BELOW_SONAR = 0.2   # the floor shows at DVL altitude + ~0.15-0.3 (pool and lake)
CONTRAST = 3.0
NOISE_FLOOR = 30.0
BG_HALF_M, GAP_HALF_M = 0.4, 0.06


def floor_line(sm, angles_deg, mpb, alt_m=float("nan")):
    """Floor points (side, up) m in the sonar's scan plane, sorted by side, or an empty (0, 2) array.
    sm: the smoothed sweep (pings x bins). alt_m: the DVL's altitude, or nan."""
    a = np.radians(np.asarray(angles_deg, float))
    L = max(2, int(RUN_M / mpb))
    none = np.zeros((0, 2))
    # the sonar's own ringing: strong at the SAME range in every direction. Measured on the
    # near-sideways beams (they can't see the floor that close); look past it
    ring = []
    for i in range(len(a)):
        if abs(math.sin(a[i])) < 0.35:
            b0 = int(0.3 / mpb)
            seg = sm[i][b0:int(1.2 / mpb)]
            if len(seg) and seg.max() >= 60:
                ring.append((b0 + int(np.flatnonzero(seg >= 0.5 * seg.max())[-1])) * mpb)
    near = max(NEAR_M, (float(np.median(ring)) + 0.1) if len(ring) >= 3 else NEAR_M)
    n0 = int(near / mpb)
    cand = {}
    for i in range(len(a)):
        if math.sin(a[i]) > -0.2:                       # only beams pointing more than ~12 deg down
            continue
        row = sm[i]
        if row.shape[0] <= n0 + L:
            continue
        run = np.convolve(row, np.ones(L) / L, "valid")[n0:]
        top = run.max()
        if top < 40:
            continue
        pk = [b for b in range(1, len(run) - 1)
              if run[b] >= run[b - 1] and run[b] >= run[b + 1] and run[b] >= 0.5 * top]
        cs = []
        for b in sorted(pk, key=lambda b: -run[b])[:4]:
            seg = row[n0 + b:n0 + b + L]
            r = (n0 + b + int(np.flatnonzero(seg >= 0.5 * seg.max())[0])) * mpb
            cs.append((-r * math.sin(a[i]), r * math.cos(a[i]), r * math.sin(a[i])))   # depth, side, up
        if cs:
            cand[i] = cs
    if not cand:
        return none
    down = sorted(cand, key=lambda i: abs(math.degrees(a[i]) % 360.0 - 270.0))
    d_ref = float(np.median([cand[i][0][0] for i in down[:5]]))
    if math.isfinite(alt_m) and alt_m > 0.1:            # the DVL, if a sonar echo agrees with it
        d_dvl = alt_m + DVL_BELOW_SONAR
        if any(abs(c[0] - d_dvl) <= 0.25 for i in down[:5] for c in cand[i]):
            d_ref = d_dvl
    chosen = {}
    for i in down:                                      # outward from straight down
        nb = [j for j in chosen if abs(j - i) <= 3]
        if nb:
            j = min(nb, key=lambda j: abs(j - i))
            ref = chosen[j][0]
            c = min(cand[i], key=lambda c: abs(c[0] - ref))
            tol = 0.15 + 0.4 * abs(c[1] - chosen[j][1])
        else:
            ref = d_ref
            c = min(cand[i], key=lambda c: abs(c[0] - ref))
            tol = 0.25 + 0.35 * abs(c[1])
        if abs(c[0] - ref) <= tol:
            chosen[i] = c
    if len(chosen) < 3:
        return none
    P = np.array([(i, c[1], c[2]) for i, c in sorted(chosen.items())])
    keep = np.ones(len(P), bool)
    for j in range(len(P)):                             # away from its neighbours: a rock top, the pipe
        if abs(P[j, 2] - np.median(P[max(0, j - 4):j + 5, 2])) > 0.12 + 0.05 * abs(P[j, 1]):
            keep[j] = False
    F = P[keep][:, 1:]
    F = F[np.argsort(F[:, 0])]
    if len(F) >= 2:                                     # steep stretches are wall, not floor
        d = np.diff(F, axis=0)
        st = np.abs(d[:, 1]) > math.tan(math.radians(50)) * np.maximum(np.abs(d[:, 0]), 1e-3)
        steep = np.zeros(len(F), bool)
        steep[:-1] |= st
        steep[1:] |= st
        if (~steep).sum() >= 2:
            F = F[~steep]
    return F


def contrast(sm, mpb):
    """Each sample over its beam's local background (median +-0.4 m, leaving out +-0.06 m)."""
    W, G = max(1, int(BG_HALF_M / mpb)), int(GAP_HALF_M / mpb)
    out = np.zeros_like(sm, dtype=float)
    for i, row in enumerate(sm):
        win = np.lib.stride_tricks.sliding_window_view(np.pad(row, W, mode="edge"), 2 * W + 1).copy()
        win[:, W - G:W + G + 1] = np.nan
        out[i] = row / np.maximum(np.nanmedian(win, axis=1), NOISE_FLOOR)
    return out


def pipe_dots(sm, angles_deg, mpb, band, alt_m=float("nan"), near_m=0.0, far_m=0.0):
    """The filter on one smoothed sweep. Returns (dots, floor): dots = [(ping index, bin)], one per
    patch; floor = floor_line's points. No floor line (fewer than 5 points): no dots."""
    sm = np.asarray(sm, float)
    F = floor_line(sm, angles_deg, mpb, alt_m)
    if len(F) < 5:
        return [], F
    a = np.radians(np.asarray(angles_deg, float))[:, None]
    r = (np.arange(sm.shape[1]) + 0.5) * mpb
    side, up = np.cos(a) * r[None, :], np.sin(a) * r[None, :]
    hgt = up - np.interp(side, F[:, 0], F[:, 1], left=np.nan, right=np.nan)
    blind = max(NEAR_M, near_m)
    sc = sm.copy()
    sc[:, :int(blind / mpb)] = 0
    rc = contrast(sc, mpb)
    with np.errstate(invalid="ignore"):
        m = (rc >= CONTRAST) & (hgt >= band[0]) & (hgt <= band[1]) & (up <= -NEAR_M)
    m[:, :int(blind / mpb)] = False
    if far_m > 0:
        m[:, int(far_m / mpb) + 1:] = False
    import cv2
    n, lab = cv2.connectedComponents(m.astype(np.uint8), connectivity=8)
    dots = []
    for j in range(1, n):
        s = np.where(lab == j, rc, -1.0)
        i, b = np.unravel_index(np.argmax(s), s.shape)
        dots.append((int(i), int(b)))
    return dots, F
