"""Radar display of the vertical scan plane, plus the numbers and the state.

Ping Viewer shows what is out there. It cannot show what your code decided
about it. This shows both, and the state it is in, so when something odd
happens you can tell straight away whether it is a perception problem or a
decision problem.

Nothing is stretched to fill space. A thing 3 m away is drawn at the 3 m ring,
an arc that was never swept stays empty, and each return is drawn one beam step
wide - not inflated into a round dot, which would make objects look bigger than
they are.

Angles follow the package convention: 0 right, 90 up, 180 left, 270 down.
"""
import math

import cv2
import numpy as np

from . import detect as D
from . import settings as S

BG = (18, 17, 16)
FACE = (30, 28, 26)
INK = (238, 238, 238)
DIM = (135, 135, 135)
GRID = (62, 60, 58)
HIT = (70, 200, 255)
BEST = (90, 255, 150)
BLIND = (150, 170, 255)
FLOORC = (120, 110, 200)

_LUT = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1),
                         cv2.COLORMAP_INFERNO).reshape(-1, 3)


def render(perception, memory=None, state="", radar_size=520, panel_w=580):
    radar = _radar(perception, radar_size)
    panel = _panel(perception, memory, state, panel_w, radar_size)
    return np.hstack([radar, panel])


def _to_screen(centre, angle_deg, r_px):
    a = math.radians(angle_deg)
    return (int(centre + r_px * math.cos(a)), int(centre - r_px * math.sin(a)))


def _radar(p, size):
    canvas = np.full((size, size, 3), BG, dtype=np.uint8)
    c, margin = size // 2, 34
    sweep = p.sweep
    if sweep is None or sweep.image.size == 0:
        return canvas

    max_range = sweep.col_to_range_m(sweep.image.shape[1])
    ppm = (size / 2.0 - margin) / max_range
    cv2.circle(canvas, (c, c), int(size / 2 - margin), FACE, -1)

    # returns, one arc per sample, each one beam step wide
    floor_noise = max(40, int(sweep.image.mean() + 2.0 * sweep.image.std()))
    rows, cols = np.where(sweep.image >= floor_noise)
    step = max(sweep.step_deg, 0.9)
    for row, col in zip(rows, cols):
        r_px = sweep.col_to_range_m(col) * ppm
        if r_px < 2 or r_px > size / 2 - margin:
            continue
        ang = sweep.row_to_angle_deg(row)
        shade = tuple(int(v) for v in _LUT[int(sweep.image[row, col])])
        # screen angles run clockwise from east, ours run anticlockwise
        a0 = -ang - step / 2.0
        cv2.ellipse(canvas, (c, c), (int(r_px), int(r_px)), 0, a0, a0 + step, shade, 2)

    # range rings
    ring = 1.0 if max_range <= 6 else 2.0
    r = ring
    while r <= max_range + 1e-6:
        cv2.circle(canvas, (c, c), int(r * ppm), GRID, 1)
        # just inside the ring and left of centre, clear of the "up" marker
        cv2.putText(canvas, f"{r:.0f}m", (c - 46, c - int(r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, DIM, 1, cv2.LINE_AA)
        r += ring

    for ang, txt in ((0, "right"), (90, "up"), (180, "left"), (270, "down")):
        x, y = _to_screen(c, ang, size / 2 - margin + 16)
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)
        cv2.putText(canvas, txt, (x - tw // 2, y + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, DIM, 1, cv2.LINE_AA)

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

    for i, d in enumerate(p.candidates):
        colour = BEST if d is p.best else HIT
        x, y = _to_screen(c, d.angle_deg, d.range_m * ppm)
        cv2.circle(canvas, (x, y), 12, colour, 2)
        cv2.putText(canvas, str(i), (x + 15, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)

    cv2.line(canvas, (c, c - 6), (c, c + 6), INK, 1)
    cv2.line(canvas, (c - 6, c), (c + 6, c), INK, 1)
    return canvas


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
    cv2.line(panel, (14, y - 14), (width - 14, y - 14), GRID, 1)
    cv2.circle(panel, (x + 8, y + 2), 7, BEST, 2)
    cv2.putText(panel, "best match", (x + 24, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.circle(panel, (x + 132, y + 2), 7, HIT, 2)
    cv2.putText(panel, "other", (x + 148, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.line(panel, (x + 210, y + 2), (x + 232, y + 2), FLOORC, 2)
    cv2.putText(panel, "floor", (x + 238, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.ellipse(panel, (x + 300, y + 2), (7, 7), 0, 0, 120, BLIND, 1)
    cv2.ellipse(panel, (x + 300, y + 2), (7, 7), 0, 180, 300, BLIND, 1)
    cv2.putText(panel, "blind zone", (x + 316, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, DIM, 1, cv2.LINE_AA)
    cv2.putText(panel, "each arc = one beam step", (x, y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, DIM, 1, cv2.LINE_AA)
