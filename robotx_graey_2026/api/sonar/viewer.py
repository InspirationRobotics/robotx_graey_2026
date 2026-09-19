"""Radar display of the vertical scan plane, plus the numbers and the state.

Ping Viewer shows what is out there. It cannot show what your code decided
about it. This shows both, and the state it is in, so when something odd
happens you can tell straight away whether it is a perception problem or a
decision problem.

Nothing is stretched to fill space. A thing 3 m away is drawn at the 3 m ring.

RINGS. This file knows nothing about pipelines, or about any other object. It
reads the score already sitting on each candidate and shades that candidate's
ring by it: dark means a sure match, pale means it barely qualifies. A candidate
that was never scored - because no target was given - gets no ring at all, so an
empty radar with a full table underneath means "you have not said what you are
looking for", not "nothing is out there". What counts as a match is set entirely
by the target you pass to perceive(); see library.resolve().

PERSISTENCE. A Radar object keeps the most recent measurement at every angle
until the head comes round and measures that angle again, the way Ping Viewer
does. A sweep takes many seconds on this hardware, so wiping the picture at the
end of each one leaves you staring at a mostly empty disc. Water the head has
never reached is grey; water it swept and found empty is blue. Those two must
never look alike.

WHY THE FLOOR WAS HARD TO SEE. The obvious suspect was that warpPolar squeezes
~540 range bins into far fewer pixels with NEAREST sampling and skips bins. That
was measured against a real Graey pool sweep and ruled out: far-field peaks came
through at 130 of 132, no ray lost more than 25, and only one return in the
whole sweep was a single bin thick. Real returns are several bins wide, because
the pulse and beam smear them. Do not "fix" that again.

What hid it, confirmed by rendering the same real sweep through the old and new
viewers side by side, was two things together. The colour ramp: floor and wall
returns sit around 120-140, which the old ramp coloured teal on a blue
background, so they barely separated from water. And size: a 4 m sweep drawn in
226 px of radius. The ramp below puts 130 in yellow, as Ping Viewer effectively
does, and the radar is now 860 px. The radial squeeze is still a max-pool, as a
safety margin for longer ranges where returns thin out, not because it was the
cause.

Angles follow the package convention: 0 right, 90 up, 180 left, 270 down.
"""
import math

import cv2
import numpy as np

from . import settings as S

# ------------------------------------------------------------------- look
SIZE = 860                  # radar edge, px. Big enough to read a floor at 4 m.
PANEL_W = 720

BG = (22, 20, 18)
SKY = (198, 126, 30)        # BGR. Ping Viewer's background blue.
FACE = (84, 80, 76)         # water the head has never reached
INK = (240, 240, 240)
DIM = (150, 150, 150)
GRID = (70, 66, 62)
MARK = (10, 10, 10)         # fixed furniture: rings, floor line, blind zone
HEAD = (255, 255, 255)

# Detection rings are shaded by how well the candidate matched the target:
# dark means sure, pale means it only just counts. Both ends are greys the
# colour ramp never produces, so a ring always reads as annotation rather than
# as something the sonar heard.
CONF_SURE = (10, 10, 10)
CONF_WEAK = (220, 220, 220)

FONT = cv2.FONT_HERSHEY_DUPLEX   # heavier strokes than SIMPLEX; survives JPEG

# Ping Viewer's ramp, set against a real Graey sweep rather than guessed: in the
# pool the median sample was 36 and the top tenth started at 120. So 36 sits in
# plain blue, and 120 is already yellow - which is where a floor or wall lands,
# and why it stands out in Ping Viewer.  (value, RGB)
_STOPS = [(0, (18, 60, 140)), (36, (30, 110, 195)), (70, (40, 180, 205)),
          (100, (60, 205, 95)), (130, (235, 228, 50)), (180, (242, 140, 30)),
          (255, (185, 25, 25))]


def _ramp():
    lut = np.zeros((256, 3), np.uint8)
    for (v0, c0), (v1, c1) in zip(_STOPS, _STOPS[1:]):
        for ch in range(3):
            # stops are RGB, OpenCV wants BGR
            lut[v0:v1 + 1, 2 - ch] = np.linspace(c0[ch], c1[ch], v1 - v0 + 1)
    return lut


_LUT = _ramp()

# Rows in the polar buffer. A quarter degree each, so a 2 degree ping covers 8.
_POLAR_ROWS = 1440


class Radar:
    """The picture, kept between frames.

    update() writes each ping's samples across the rows its beam covers. Nothing
    is ever cleared except when the range setting changes, because then the old
    samples are at a different scale and would be drawn at the wrong distance.
    """

    def __init__(self):
        self.polar = None
        self.swept = None
        self.metres_per_bin = None
        self.head_deg = None

    def update(self, sweep):
        if sweep is None or sweep.image.size == 0:
            return
        n_bins = sweep.image.shape[1]
        if (self.polar is None or self.polar.shape[1] != n_bins
                or self.metres_per_bin != sweep.metres_per_bin):
            self.polar = np.zeros((_POLAR_ROWS, n_bins), np.uint8)
            self.swept = np.zeros(_POLAR_ROWS, bool)
            self.metres_per_bin = sweep.metres_per_bin

        # OpenCV measures polar angle clockwise, this package anticlockwise, so
        # rows are laid down against -angle. sonar_check.py pins this down.
        half = max(sweep.step_deg, S.BEAM_IN_PLANE_DEG) / 2.0
        for row, ang in enumerate(sweep.angles_deg):
            lo = int(math.floor((-ang - half) * _POLAR_ROWS / 360.0))
            hi = int(math.ceil((-ang + half) * _POLAR_ROWS / 360.0))
            idx = np.arange(lo, hi) % _POLAR_ROWS
            self.polar[idx] = sweep.image[row]
            self.swept[idx] = True
        self.head_deg = sweep.angles_deg[-1]

    @property
    def max_range_m(self):
        return self.polar.shape[1] * self.metres_per_bin


def render(perception, memory=None, state="", radar=None, size=SIZE,
           panel_w=PANEL_W, target=""):
    """Radar beside the numbers.

    Pass a Radar to keep the picture between calls. Without one, the radar shows
    only this perception's sweep - which is what the checks and one-off renders
    want.

    target is only ever printed. What the rings do is decided entirely by the
    scores already sitting on the candidates, so this cannot show one target
    while the detector used another.
    """
    return np.hstack([_radar(perception, size, radar),
                      _panel(perception, memory, state, panel_w, size, target)])


def confidence_colour(score):
    """Score -> ring colour. 1.0 is nearly black, 0.0 is nearly white."""
    s = max(0.0, min(1.0, float(score)))
    return tuple(int(round(w + (c - w) * s))
                 for w, c in zip(CONF_WEAK, CONF_SURE))


def _text(img, s, org, scale, colour, thick=1, halo=None):
    if halo is not None:
        cv2.putText(img, s, org, FONT, scale, halo, thick + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, colour, thick, cv2.LINE_AA)


def _to_screen(centre, angle_deg, r_px):
    a = math.radians(angle_deg)
    return (int(round(centre + r_px * math.cos(a))),
            int(round(centre - r_px * math.sin(a))))


def _radar(p, size, radar=None):
    canvas = np.full((size, size, 3), SKY, np.uint8)
    if radar is None:
        radar = Radar()
        radar.update(p.sweep)
    if radar.polar is None:
        return canvas

    c = size // 2
    margin = int(size * 0.065)
    radius = c - margin
    max_range = radar.max_range_m
    ppm = radius / max_range

    cv2.circle(canvas, (c, c), radius, FACE, -1, cv2.LINE_AA)
    _paint(canvas, radar, c, radius)

    ring = 0.5 if max_range <= 2.5 else (1.0 if max_range <= 6 else 2.0)
    r = ring
    while r <= max_range + 1e-6:
        cv2.circle(canvas, (c, c), int(r * ppm), MARK, 1, cv2.LINE_AA)
        label = f"{r:g} m"
        _text(canvas, label, (c + 6, c - int(r * ppm) + 18), 0.5, MARK, 1, halo=INK)
        r += ring

    for ang, txt in ((0, "right"), (90, "up"), (180, "left"), (270, "down")):
        x, y = _to_screen(c, ang, radius + margin * 0.55)
        (tw, th), _ = cv2.getTextSize(txt, FONT, 0.65, 1)
        # "right" and "left" sit close enough to the edge that centring them on
        # the anchor runs the word off the canvas, so keep them on it.
        _text(canvas, txt, (max(4, min(x - tw // 2, size - tw - 4)), y + th // 2),
              0.65, INK, 1)

    if p.floor is not None:
        pts = []
        for ang in range(180, 361, 2):
            fr = p.floor.range_at(ang % 360)
            if fr is not None and fr <= max_range:
                pts.append(_to_screen(c, ang % 360, fr * ppm))
        for a, b in zip(pts, pts[1:]):
            cv2.line(canvas, a, b, MARK, 1, cv2.LINE_AA)

    _dashed_circle(canvas, (c, c), int(S.MIN_RANGE_M * ppm), MARK)

    if radar.head_deg is not None:
        cv2.line(canvas, (c, c), _to_screen(c, radar.head_deg, radius), HEAD, 2, cv2.LINE_AA)

    # Rings, shaded by confidence. A candidate with score None was never judged,
    # because no target was given - so there is nothing to be confident about
    # and nothing is drawn. An empty radar here means "you have not said what
    # you are looking for", not "nothing is there"; the table still lists every
    # blob and its numbers.
    #
    # One pixel wide so a ring does not hide what it is pointing at.
    for i, d in enumerate(p.candidates):
        if d.score is None:
            continue
        shade = confidence_colour(d.score)
        x, y = _to_screen(c, d.angle_deg, d.range_m * ppm)
        cv2.circle(canvas, (x, y), 14, shade, 1, cv2.LINE_AA)
        if d is p.best:
            cv2.circle(canvas, (x, y), 19, shade, 1, cv2.LINE_AA)
        _text(canvas, f"{i}  {d.score:.2f}", (x + 16, y - 12), 0.5, shade, 1,
              halo=INK)

    cv2.drawMarker(canvas, (c, c), MARK, cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
    return canvas


def _paint(canvas, radar, centre, radius):
    """Warp the kept picture from polar to cartesian and lay it on the canvas."""
    polar = radar.polar
    n_bins = polar.shape[1]

    # Blend neighbouring pings a little. The beam is 2 degrees wide, so this
    # removes the hard wedge edges without blurring anything it actually resolved.
    k = 5
    wrapped = np.concatenate([polar[-k:], polar, polar[:k]])
    polar = cv2.blur(wrapped, (1, k))[k:-k]

    # Squeeze range bins down to pixels by keeping the BRIGHTEST in each group.
    if n_bins > radius:
        kx = int(math.ceil(n_bins / radius))
        polar = cv2.dilate(polar, np.ones((1, kx), np.uint8))
        polar = cv2.resize(polar, (radius, _POLAR_ROWS), interpolation=cv2.INTER_AREA)
    cols = polar.shape[1]

    side = 2 * radius
    flags = cv2.WARP_INVERSE_MAP + cv2.WARP_POLAR_LINEAR + cv2.INTER_LINEAR
    intensity = cv2.warpPolar(polar, (side, side), (radius, radius), radius, flags)
    swept = np.repeat(radar.swept.astype(np.uint8)[:, None] * 255, cols, 1)
    seen = cv2.warpPolar(swept, (side, side), (radius, radius), radius,
                         cv2.WARP_INVERSE_MAP + cv2.WARP_POLAR_LINEAR)

    inside = np.zeros((side, side), np.uint8)
    cv2.circle(inside, (radius, radius), radius, 255, -1)
    mask = (seen > 0) & (inside > 0)

    y0 = x0 = centre - radius
    patch = canvas[y0:y0 + side, x0:x0 + side]
    # Ramp applied AFTER the warp, so every pixel is a true ramp colour rather
    # than a blend of two colours the ramp never produces.
    patch[mask] = _LUT[intensity][mask]


def _dashed_circle(img, centre, radius, colour, dashes=40):
    if radius < 3:
        return
    for k in range(0, dashes, 2):
        a0 = (360.0 / dashes) * k
        cv2.ellipse(img, centre, (radius, radius), 0, a0, a0 + 360.0 / dashes,
                    colour, 1, cv2.LINE_AA)


def _panel(p, memory, state, width, height, target=""):
    panel = np.full((height, width, 3), BG, np.uint8)
    L = 22
    y = 48

    _text(panel, state or "-", (L, y), 1.0, INK, 1)
    y += 38

    # What is being hunted, always on screen. Rings missing with a target set is
    # a detector problem; rings missing with no target is just this line.
    if target:
        _text(panel, f"target: {target}", (L, y), 0.6, (120, 240, 150))
    else:
        _text(panel, "target: none - measuring everything, judging nothing",
              (L, y), 0.6, DIM)
    y += 32

    from .driver import explain
    _text(panel, explain(p), (L, y), 0.55, DIM)
    y += 32
    floor_txt = "floor: not found" if p.floor is None else (
        f"floor: {p.floor.depth_m:.2f} m below   agreement {p.floor.confidence:.0%}")
    _text(panel, floor_txt, (L, y), 0.6, INK if p.floor else DIM)
    y += 40

    cv2.line(panel, (L, y - 22), (width - L, y - 22), GRID, 1)
    labels = ["#", "range", "offset", "height", "span", "bright", "solid", "score"]
    xs = [L, 58, 150, 255, 360, 445, 535, 625]
    for x, lab in zip(xs, labels):
        _text(panel, lab, (x, y), 0.52, DIM)
    y += 34

    if not p.candidates:
        _text(panel, "no detections", (L, y), 0.6, DIM)
        y += 40
    else:
        for i, d in enumerate(p.candidates[:8]):
            cells = [str(i), f"{d.range_m:.2f}m", f"{d.offset_m:+.2f}m",
                     "-" if d.height_m is None else f"{d.height_m:.2f}m",
                     f"{d.span_deg:.0f}°".replace("°", " deg"),
                     f"{d.brightness:.0f}", f"{d.solidity:.2f}",
                     "-" if d.score is None else f"{d.score:.2f}"]
            colour = INK if d is not p.best else (120, 240, 150)
            for x, cell in zip(xs, cells):
                _text(panel, cell, (x, y), 0.55, colour)
            y += 32
        if len(p.candidates) > 8:
            _text(panel, f"+ {len(p.candidates) - 8} more", (L, y), 0.5, DIM)
            y += 32

    if p.best is not None:
        y += 6
        _text(panel, f"orientation: {p.best.orientation}", (L, y), 0.6, (120, 240, 150))
        y += 34

    if memory is not None:
        slope = memory.offset_slope()
        bend = memory.bend()
        for line in (f"misses {memory.misses}    centred {memory.centred}",
                     "drift -" if slope is None else f"drift {slope:+.3f} m per sweep",
                     f"bend {bend or '-'}"):
            _text(panel, line, (L, y), 0.55, DIM)
            y += 28

    _legend(panel, L, height - 250, width)
    return panel


def _legend(panel, x, y, width):
    """Swatches are drawn on the radar's own colours, so each symbol looks here
    exactly as it does on the radar."""
    cv2.line(panel, (x, y - 20), (width - x, y - 20), GRID, 1)

    # echo strength ramp
    bar_w, bar_h = width - 2 * x - 150, 18
    bar = np.repeat(_LUT[np.linspace(0, 255, bar_w).astype(np.uint8)][None, :, :], bar_h, 0)
    panel[y:y + bar_h, x:x + bar_w] = bar
    _text(panel, "weak echo", (x, y + bar_h + 22), 0.5, DIM)
    (tw, _), _ = cv2.getTextSize("strong", FONT, 0.5, 1)
    _text(panel, "strong", (x + bar_w - tw, y + bar_h + 22), 0.5, DIM)
    cv2.rectangle(panel, (x + bar_w + 24, y), (x + bar_w + 24 + bar_h, y + bar_h), FACE, -1)
    _text(panel, "not swept", (x + bar_w + 50, y + 15), 0.5, DIM)

    # ring shade ramp: how well a candidate matched the target, which is a
    # different question from how loud its echo was, so it gets its own bar
    y += 62
    ring_bar = np.zeros((bar_h, bar_w, 3), np.uint8)
    for col_x in range(bar_w):
        ring_bar[:, col_x] = confidence_colour(col_x / max(bar_w - 1, 1))
    panel[y:y + bar_h, x:x + bar_w] = ring_bar
    _text(panel, "ring: weak match", (x, y + bar_h + 22), 0.5, DIM)
    (tw, _), _ = cv2.getTextSize("sure", FONT, 0.5, 1)
    _text(panel, "sure", (x + bar_w - tw, y + bar_h + 22), 0.5, DIM)
    _text(panel, "none = no target", (x + bar_w + 24, y + 15), 0.5, DIM)

    # symbols, each on a patch of radar blue
    y2 = y + 82
    items = [("best match", "double"), ("sonar head", "head"),
             ("blind zone", "dash")]
    col = (width - 2 * x) // 2
    for n, (label, kind) in enumerate(items):
        cx = x + (n % 2) * col
        cy = y2 + (n // 2) * 44
        panel[cy - 16:cy + 16, cx:cx + 36] = _LUT[36]
        mid = (cx + 18, cy)
        if kind == "double":
            cv2.circle(panel, mid, 8, CONF_SURE, 1, cv2.LINE_AA)
            cv2.circle(panel, mid, 13, CONF_SURE, 1, cv2.LINE_AA)
        elif kind == "head":
            cv2.line(panel, (cx + 4, cy + 10), (cx + 32, cy - 10), HEAD, 2, cv2.LINE_AA)
        else:
            _dashed_circle(panel, mid, 11, MARK, dashes=16)
        _text(panel, label, (cx + 50, cy + 7), 0.55, INK)
