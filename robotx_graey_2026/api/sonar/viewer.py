"""Radar display of the vertical scan plane, plus the numbers and the state.

Ping Viewer shows what is out there. It cannot show what your code decided
about it. This shows both, and the state it is in, so when something odd
happens you can tell straight away whether it is a perception problem or a
decision problem.

Nothing is stretched to fill space. A thing 3 m away is drawn at the 3 m ring,
and an arc that was never swept stays empty rather than being filled in with a
guess.

EVERY sample is painted, not just the bright ones. An earlier version drew only
samples above a threshold, and worse, it used a DIFFERENT threshold from the one
the detector uses - so a blob could be circled in a part of the picture that had
been left black, which is impossible to interpret. Weak returns now show as the
cold end of the colour map, exactly as they do in Ping Viewer.

The whole sweep is warped from polar to cartesian in one operation instead of
drawing an arc per sample. That is what makes it continuous rather than speckled,
and it is also far faster.

Angles follow the package convention: 0 right, 90 up, 180 left, 270 down.
"""
import math

import cv2
import numpy as np

from . import detect as D
from . import settings as S

BG = (18, 17, 16)
FACE = (78, 74, 70)         # unswept water: grey, never a ramp colour
INK = (238, 238, 238)
DIM = (135, 135, 135)
GRID = (62, 60, 58)
HIT = (70, 200, 255)
BEST = (90, 255, 150)
BLIND = (150, 170, 255)
FLOORC = (120, 110, 200)

SKY = (198, 126, 30)        # BGR. Ping Viewer's background blue.
MARK = (12, 12, 12)         # near-black, for rings and detection circles

# Ping Viewer's own ramp, near enough: mid blue for water, then cyan, green,
# yellow, orange, dark red for the hardest returns. Built by hand rather than
# taken from an OpenCV map because every stock map either starts near black
# (INFERNO, TURBO) or ends somewhere that reads as "weak" on a bright field.
_STOPS = [(0, (168, 96, 20)), (40, (176, 140, 24)), (90, (168, 190, 40)),
          (140, (80, 200, 70)), (190, (60, 232, 226)), (225, (40, 150, 240)),
          (255, (30, 30, 190))]


def _ramp():
    lut = np.zeros((256, 3), np.uint8)
    for (v0, c0), (v1, c1) in zip(_STOPS, _STOPS[1:]):
        n = v1 - v0
        for ch in range(3):
            lut[v0:v1 + 1, ch] = np.linspace(c0[ch], c1[ch], n + 1)
    return lut


_LUT = _ramp()

# Rows in the polar buffer before warping. 1440 is a quarter degree per row,
# so a 2 degree beam step spans 8 rows and the warp has room to blend between
# pings instead of stamping hard wedges.
_POLAR_ROWS = 1440


def render(perception, memory=None, state="", radar_size=520, panel_w=580):
    radar = _radar(perception, radar_size)
    panel = _panel(perception, memory, state, panel_w, radar_size)
    return np.hstack([radar, panel])


def _to_screen(centre, angle_deg, r_px):
    a = math.radians(angle_deg)
    return (int(centre + r_px * math.cos(a)), int(centre - r_px * math.sin(a)))


def _radar(p, size):
    canvas = np.full((size, size, 3), SKY, dtype=np.uint8)
    c, margin = size // 2, 34
    sweep = p.sweep
    if sweep is None or sweep.image.size == 0:
        return canvas

    max_range = sweep.col_to_range_m(sweep.image.shape[1])
    radius = int(size / 2 - margin)
    ppm = radius / max_range
    # Deliberately NOT the cold end of the ramp. Water the head has not reached
    # yet has to look different from water it swept and found empty, or a
    # half-finished sweep is indistinguishable from a clear picture.
    cv2.circle(canvas, (c, c), radius, FACE, -1)

    _paint_returns(canvas, sweep, c, radius)

    # range rings
    ring = 1.0 if max_range <= 6 else 2.0
    r = ring
    while r <= max_range + 1e-6:
        cv2.circle(canvas, (c, c), int(r * ppm), MARK, 1)
        # just inside the ring and left of centre, clear of the "up" marker
        cv2.putText(canvas, f"{r:.0f}m", (c - 46, c - int(r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, MARK, 1, cv2.LINE_AA)
        r += ring

    for ang, txt in ((0, "right"), (90, "up"), (180, "left"), (270, "down")):
        x, y = _to_screen(c, ang, size / 2 - margin + 16)
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)
        cv2.putText(canvas, txt, (x - tw // 2, y + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, INK, 1, cv2.LINE_AA)

    # Where the transducer is pointing right now. On a partial sweep this is the
    # live edge, so it sits at the boundary between painted and unpainted water
    # and shows the head moving.
    hx, hy = _to_screen(c, sweep.angles_deg[-1], radius)
    cv2.line(canvas, (c, c), (hx, hy), (255, 255, 255), 1, cv2.LINE_AA)

    # the detected floor, drawn as the arc it was measured at
    if p.floor is not None:
        pts = []
        for ang in range(180, 361, 3):
            fr = p.floor.range_at(ang % 360)
            if fr is None or fr > max_range:
                continue
            pts.append(_to_screen(c, ang % 360, fr * ppm))
        for a, b in zip(pts, pts[1:]):
            cv2.line(canvas, a, b, FLOORC, 1)

    # blind zone
    _dashed_circle(canvas, (c, c), int(S.MIN_RANGE_M * ppm), BLIND)

    # Black on the colour ramp. Nothing in the ramp is dark, so a black ring
    # reads as an annotation rather than as another return - which a bright ring
    # did not, when it sat on top of a bright patch.
    for i, d in enumerate(p.candidates):
        x, y = _to_screen(c, d.angle_deg, d.range_m * ppm)
        cv2.circle(canvas, (x, y), 12, MARK, 2, cv2.LINE_AA)
        if d is p.best:
            cv2.circle(canvas, (x, y), 16, MARK, 1, cv2.LINE_AA)
        cv2.putText(canvas, str(i), (x + 15, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, MARK, 2, cv2.LINE_AA)

    cv2.line(canvas, (c, c - 6), (c, c + 6), MARK, 1)
    cv2.line(canvas, (c - 6, c), (c + 6, c), MARK, 1)
    return canvas


def _paint_returns(canvas, sweep, centre, radius):
    """Warp the whole sweep from polar to cartesian in one operation.

    sweep.image IS already a polar image - rows are scan angles, columns are
    range bins - so the only work is resampling it onto a full circle at a fixed
    angular pitch and handing it to OpenCV.

    The resample is a LINEAR resize rather than stamping each ping across the
    rows it covers. Stamping gives hard-edged wedges, which is what made the
    display look blocky; resizing blends between neighbouring pings the way Ping
    Viewer does. It invents nothing the beam did not already smear together - the
    beam is 2 degrees wide and the step is 2 degrees, so adjacent pings genuinely
    do overlap.

    OpenCV measures its polar angle clockwise from the +x axis, because image y
    runs downward. Our angles run anticlockwise from the same axis, so the rows
    are laid down against -angle. Getting this backwards mirrors the picture,
    which is easy to miss on a symmetric scene, so sonar_check.py pins it down.
    """
    n_bins = sweep.image.shape[1]
    angs = np.asarray(sweep.angles_deg, dtype=float)
    step = sweep.step_deg
    half = step / 2.0

    # How much of the circle this sweep covers, unwrapped so an arc crossing 360
    # stays monotonic.
    span = float((angs[-1] - angs[0]) % 360.0) if len(angs) > 1 else 0.0
    arc = span + step
    rows = max(2, int(round(arc / 360.0 * _POLAR_ROWS)))

    strip = cv2.resize(sweep.image, (n_bins, rows), interpolation=cv2.INTER_LINEAR)
    # Index 0 of the flipped strip is the HIGHEST angle, which is the LOWEST row
    # once angles are negated.
    strip = np.flipud(strip)
    start = int(round(((-(angs[0] + span + half)) % 360.0) * _POLAR_ROWS / 360.0))
    idx = (np.arange(rows) + start) % _POLAR_ROWS

    polar = np.zeros((_POLAR_ROWS, n_bins), dtype=np.uint8)
    swept = np.zeros(_POLAR_ROWS, dtype=bool)
    polar[idx] = strip
    swept[idx] = True

    flags = cv2.WARP_INVERSE_MAP + cv2.WARP_POLAR_LINEAR
    side = 2 * radius
    disc = cv2.warpPolar(_LUT[polar], (side, side), (radius, radius), radius, flags)
    seen = cv2.warpPolar(np.repeat(swept.astype(np.uint8)[:, None] * 255, n_bins, 1),
                         (side, side), (radius, radius), radius, flags)

    # Only inside the circle, and only where the sonar actually looked.
    inside = np.zeros((side, side), np.uint8)
    cv2.circle(inside, (radius, radius), radius, 255, -1)
    mask = (seen > 0) & (inside > 0)

    y0, x0 = centre - radius, centre - radius
    patch = canvas[y0:y0 + side, x0:x0 + side]
    patch[mask] = disc[mask]


def _dashed_circle(img, centre, radius, colour, dashes=36):
    if radius < 3:
        return
    for k in range(0, dashes, 2):
        a0 = (360.0 / dashes) * k
        cv2.ellipse(img, centre, (radius, radius), 0, a0, a0 + 360.0 / dashes, colour, 1)


def _panel(p, memory, state, width, height):
    panel = np.full((height, width, 3), BG, dtype=np.uint8)
    y = 30

    cv2.putText(panel, state or "-", (14, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.72, BEST if p.found else INK, 2, cv2.LINE_AA)
    y += 26
    from .driver import explain
    cv2.putText(panel, explain(p), (14, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.40, DIM, 1, cv2.LINE_AA)
    y += 30

    floor_txt = "floor: not found" if p.floor is None else (
        f"floor: {p.floor.depth_m:.2f} m below  (agreement {p.floor.confidence:.0%})")
    cv2.putText(panel, floor_txt, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                INK if p.floor else DIM, 1, cv2.LINE_AA)
    y += 30

    cv2.line(panel, (14, y - 12), (width - 14, y - 12), GRID, 1)
    labels = ["#", "range", "offset", "height", "span", "bright", "solid", "score"]
    xs = [14, 44, 110, 186, 262, 326, 400, 470]
    for x, lab in zip(xs, labels):
        cv2.putText(panel, lab, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, DIM, 1, cv2.LINE_AA)
    y += 22

    if not p.candidates:
        cv2.putText(panel, "no candidates", (14, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, DIM, 1, cv2.LINE_AA)
        y += 34
    else:
        for i, d in enumerate(p.candidates[:5]):
            colour = BEST if d is p.best else INK
            cells = [str(i), f"{d.range_m:.2f}m", f"{d.offset_m:+.2f}m",
                     "-" if d.height_m is None else f"{d.height_m:.2f}m",
                     f"{d.span_deg:.0f}d",
                     f"{d.brightness:.0f}", f"{d.solidity:.2f}",
                     "-" if d.score is None else f"{d.score:.2f}"]
            for x, cell in zip(xs, cells):
                cv2.putText(panel, cell, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                            colour, 1, cv2.LINE_AA)
            if d.scores:
                parts = "  ".join(f"{k.replace('_m','').replace('_deg','')} {v:.2f}"
                                  for k, v in d.scores.items())
                cv2.putText(panel, parts, (44, y + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.30, DIM, 1, cv2.LINE_AA)
            y += 30

    if p.best is not None:
        y += 8
        cv2.putText(panel, f"orientation: {p.best.orientation}", (14, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, BEST, 1, cv2.LINE_AA)
        y += 24

    if memory is not None:
        slope = memory.offset_slope()
        bend = memory.bend()
        lines = [
            f"misses: {memory.misses}   centred: {memory.centred}",
            "drift: -" if slope is None else f"drift: {slope:+.3f} m/sweep",
            f"bend: {bend or '-'}",
        ]
        for line in lines:
            cv2.putText(panel, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        BEST if bend and "bend" in line else DIM, 1, cv2.LINE_AA)
            y += 20

    _legend(panel, 14, height - 74, width)
    return panel


def _legend(panel, x, y, width):
    """Annotations are black on the radar. They are drawn light here because
    this panel is dark - what matters is the shape, not the colour."""
    cv2.line(panel, (14, y - 14), (width - 14, y - 14), GRID, 1)
    cv2.circle(panel, (x + 8, y + 2), 5, DIM, 1, cv2.LINE_AA)
    cv2.circle(panel, (x + 8, y + 2), 8, DIM, 1, cv2.LINE_AA)
    cv2.putText(panel, "best match", (x + 24, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.circle(panel, (x + 132, y + 2), 7, DIM, 1, cv2.LINE_AA)
    cv2.putText(panel, "other", (x + 148, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.line(panel, (x + 210, y + 2), (x + 232, y + 2), INK, 1)
    cv2.putText(panel, "head", (x + 238, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.ellipse(panel, (x + 300, y + 2), (7, 7), 0, 0, 120, DIM, 1)
    cv2.ellipse(panel, (x + 300, y + 2), (7, 7), 0, 180, 300, DIM, 1)
    cv2.putText(panel, "blind zone", (x + 316, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.putText(panel, "every sample drawn; blue = weak, red = strong; grey = never swept",
                (x, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.32, DIM, 1, cv2.LINE_AA)
