"""Pieces 3, 4 and 5: find lumps, measure them, judge them, read them.

Three jobs kept as separate functions:

  find_blobs()  turns a sweep into candidate shapes. Knows nothing about
                pipelines.
  measure()     turns a shape into numbers. Also knows nothing about pipelines.
  score()       compares those numbers to a description of the target. Even
                this one does not know what a pipeline is: the description
                arrives as an argument, and there is no default. Nothing in
                this package names an object it expects to find.

perceive() runs the whole chain and reports WHICH way it failed, because
"cannot see the bottom" and "can see the bottom but nothing above it" are
different problems and the driver should react differently to each.

No ROS, no hardware.
"""
import math

import cv2
import numpy as np

from . import library as L
from . import settings as S
from .floor import find_floor

# what perceive() can report
OK = "ok"
NO_FLOOR = "no_floor"
NOTHING_ABOVE_FLOOR = "nothing_above_floor"
NOTHING_SCORED = "nothing_scored"

# orientation labels
ALONG = "along"        # pipe runs fore-aft: the sub is pointed along it
SLANTED = "slanted"
ACROSS = "across"      # pipe lies across the view: the sub is perpendicular

FEATURES = ("height_m", "span_deg", "brightness", "solidity", "thickness_m",
            "width_m")


class Detection:
    def __init__(self, range_m, angle_deg, offset_m, height_m, span_deg,
                 brightness, solidity, contour, thickness_m=None):
        self.range_m = range_m
        self.angle_deg = angle_deg
        self.offset_m = offset_m        # positive to starboard
        self.height_m = height_m        # above the floor
        self.span_deg = span_deg        # angular extent in the scan plane
        self.brightness = brightness
        self.solidity = solidity
        self.thickness_m = thickness_m  # radial depth of the return, metres
        self.contour = contour
        self.score = None       # 0-1, geometric mean of the per-feature scores
        self.scores = {}        # per feature, 0-1
        self.unscored = ()      # features the profile wanted and we could not
                                # supply - height_m when there is no floor

    @property
    def confidence(self):
        """The score as a percentage, for reading rather than arithmetic.

        None when nothing was scored, which is not the same as 0. 0 means "this
        matched nothing"; None means "you never said what to match against".
        """
        return None if self.score is None else int(round(self.score * 100))

    @property
    def width_m(self):
        """Angular extent converted to metres at the measured range.

        span_deg on its own says nothing about size - 20 degrees is a pipe up
        close and a wall far away. This is the same measurement in units you can
        compare against a tape.
        """
        return self.range_m * math.radians(self.span_deg)

    @property
    def features(self):
        f = {"span_deg": self.span_deg, "brightness": self.brightness,
             "solidity": self.solidity, "width_m": self.width_m}
        if self.height_m is not None:
            f["height_m"] = self.height_m
        if self.thickness_m is not None:
            f["thickness_m"] = self.thickness_m
        return f

    @property
    def orientation(self):
        """Which way the pipe runs relative to the sub, read off the span.

        A pipe running fore-aft only crosses the scan plane at one point, so it
        comes back as a narrow blob. A pipe lying across the view sits IN the
        plane along its whole length, so it spreads across many angles.

        Caveat worth remembering: span is not an absolute angle measurement. It
        also depends on range and on how much pipe happens to be in view. Use
        it comparatively - yaw and find where it peaks - rather than trusting a
        single reading to tell you an exact bearing.
        """
        if self.span_deg <= S.SPAN_ALONG_DEG:
            return ALONG
        if self.span_deg >= S.SPAN_ACROSS_DEG:
            return ACROSS
        return SLANTED

    def __repr__(self):
        s = "unscored" if self.score is None else f"{self.score:.2f}"
        return (f"Detection({self.range_m:.2f} m, offset {self.offset_m:+.2f} m, "
                f"{self.height_m:.2f} m up, span {self.span_deg:.0f} deg, {s})")


class Perception:
    """What one sweep told us, including how it failed if it did."""

    def __init__(self, sweep, floor, candidates, best, reason):
        self.sweep = sweep
        self.floor = floor
        self.candidates = candidates
        self.best = best
        self.reason = reason

    @property
    def found(self):
        return self.best is not None


def find_blobs(sweep, floor, tuning=None):
    """Sweep plus floor -> contours of everything that isn't the floor."""
    t = dict(S.DETECT)
    if tuning:
        t.update(tuning)

    if sweep.image.size == 0:
        return [], np.zeros_like(sweep.image)

    gray = sweep.image.copy()

    # blank the near field - always bright, always the sonar's own ringing
    blind_cols = int(S.MIN_RANGE_M / sweep.metres_per_bin)
    gray[:, :blind_cols] = 0

    # blank the floor and everything past it, row by row, since the floor sits
    # at a different range in every direction
    if floor is not None:
        for row in range(gray.shape[0]):
            fr = floor.range_at(sweep.row_to_angle_deg(row))
            if fr is None:
                continue
            cut = int(max(0, (fr - t["floor_margin_m"]) / sweep.metres_per_bin))
            gray[row, cut:] = 0

    blurred = cv2.blur(gray, (t["blur"], t["blur"]))
    _, thresh = cv2.threshold(blurred, t["threshold"], 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (t["close"], t["close"]))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= t["min_blob_px"]]
    return contours, closed


def measure(contour, sweep, floor=None):
    """Contour -> a Detection with real units.

    Works without a floor. Height above the floor is then None, since there is
    nothing to measure it against - but range, bearing, offset, span and
    brightness are all still real. That is enough to answer "where is that
    thing", which is a useful question on its own and the first one worth
    asking of new hardware.
    """
    x, y, w, h = cv2.boundingRect(contour)      # x = range bins, y = angle rows

    range_m = sweep.col_to_range_m(x + w / 2.0)
    angle_deg = sweep.row_to_angle_deg(y + h / 2.0)
    offset_m, height_rel_sonar = sweep.offset_and_height(angle_deg, range_m)

    height_m = floor.height_above(angle_deg, range_m) if floor is not None else None

    span_deg = h * sweep.step_deg

    area = cv2.contourArea(contour)
    hull = cv2.contourArea(cv2.convexHull(contour))
    solidity = (area / hull) if hull > 0 else 0.0

    mask = np.zeros(sweep.image.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    brightness = float(cv2.mean(sweep.image, mask=mask)[0])

    # Radial depth of the return. Nearly pose-invariant: a pipe is its own
    # diameter thick however you look at it, while a grazing seabed return
    # smears over many bins. That makes it worth more for identity than span,
    # which changes completely with viewing angle.
    thickness_m = w * sweep.metres_per_bin

    return Detection(range_m, angle_deg, offset_m, height_m, span_deg,
                     brightness, solidity, contour, thickness_m)


def _score_one(value, spec):
    """1.0 at the ideal, tapering straight to 0 at either edge.

    A straight taper is a choice, not a law. It is easy to reason about when a
    score comes back low and you want to know why.
    """
    lo, hi, ideal = spec["min"], spec["max"], spec["ideal"]
    if value < lo or value > hi:
        return 0.0
    # Note the strict comparisons. An earlier version used <= and >=, which
    # scored a value sitting exactly ON a bound as zero - so a perfectly solid
    # blob (solidity 1.0, against a max of 1.0) was rated worst possible. For a
    # feature whose max is a physical ceiling that is exactly backwards.
    if value < ideal:
        return (value - lo) / (ideal - lo) if ideal > lo else 1.0
    if value > ideal:
        return (hi - value) / (hi - ideal) if hi > ideal else 1.0
    return 1.0


def score(detection, profile):
    """Measured features against a target description -> per-feature and total.

    Total is the GEOMETRIC MEAN of the per-feature scores: multiply them, then
    take the n-th root. Detection.confidence is the same number as 0-100.

    NOT THE AVERAGE, because averaging lets a good brightness paper over a
    completely wrong shape. A candidate has to be plausible on every feature the
    profile mentions, and one zero has to kill it whatever the others say. A
    geometric mean does that - anything times zero is zero - while an average
    would still hand back 0.75 for three good features and one total failure.

    NOT THE PLAIN PRODUCT, which is what this used to be, because a product
    punishes you for asking more questions. Four features at 0.9 each multiply
    out to 0.66, so adding a feature you are perfectly happy about drags the
    number down, and the same object scores differently depending on how many
    features the profile happens to name. The n-th root cancels that: 0.9 on
    every feature comes out as 0.9 whether there are two features or ten, which
    is the only way a percentage means anything.

    Features the profile names but the detection cannot supply are SKIPPED, not
    scored zero. The usual case is height_m in water too shallow to find the
    bottom, and failing a candidate for that would be blaming it for the pool.
    But skipping quietly makes the score easier, so the caller records what was
    dropped in Detection.unscored and the viewer says so on screen.
    """
    feats = detection.features
    per = {name: _score_one(feats[name], spec)
           for name, spec in profile.items() if name in feats}
    if not per:
        return per, 0.0
    total = 1.0
    for v in per.values():
        total *= v
    return per, total ** (1.0 / len(per))


def perceive(sweep, target=None, tuning=None, floor=None, min_score=None,
             require_floor=True, library_path=None):
    """The whole chain. Returns a Perception, including why it found nothing.

    target says WHAT to look for, and nothing in this file decides that. It is
    whatever library.resolve() accepts: the name of an object you measured, a
    "height_m=1.5,brightness=140" string, a profile dict, or None.

    target=None is discovery mode: measure everything, judge nothing. Every
    candidate comes back with its numbers and score left as None, and the viewer
    then draws no rings, because there is nothing to be confident ABOUT. That is
    how you learn what a target's numbers really are - put the thing in the
    water and read them off the viewer.

    floor= lets you assert a floor instead of measuring one, for the shallow
    pool where the bottom is inside the blind zone.

    require_floor=False skips the floor entirely and reports every blob with
    its range and bearing. No height, so no height filtering and no height
    scoring - but it answers "where is that thing", which is what you want the
    first time you point new hardware at real water.
    """
    profile = L.resolve(target, **({"path": library_path} if library_path else {}))

    t = dict(S.DETECT)
    if tuning:
        t.update(tuning)
    if min_score is None:
        min_score = S.MIN_SCORE

    if floor is None and require_floor:
        floor = find_floor(sweep)
        if floor is None:
            return Perception(sweep, None, [], None, NO_FLOOR)

    contours, _ = find_blobs(sweep, floor, t)

    lo, hi = t["height_band_m"]
    candidates = []
    for c in contours:
        d = measure(c, sweep, floor)
        if d is None:
            continue
        if d.height_m is not None and not (lo <= d.height_m <= hi):
            continue                       # on the floor, or up near the surface
        candidates.append(d)

    if not candidates:
        return Perception(sweep, floor, [], None, NOTHING_ABOVE_FLOOR)

    return rescore(Perception(sweep, floor, candidates, None, OK), profile,
                   min_score=min_score)


def rescore(perception, target=None, min_score=None, library_path=None):
    """Judge an already-measured Perception against a different target.

    Measuring is the expensive half and it does not depend on the target at all:
    a sweep is fifteen to twenty-five seconds on this hardware, and scoring what
    came back is microseconds. So changing what you are looking for does not
    need a new sweep - it needs this. That is what lets the target box on the
    radar page answer the moment you press enter rather than at the end of the
    next sweep.

    Modifies the Perception in place and returns it.
    """
    profile = L.resolve(target, **({"path": library_path} if library_path else {}))
    if min_score is None:
        min_score = S.MIN_SCORE

    candidates = perception.candidates
    if not profile:
        for d in candidates:
            d.scores, d.score, d.unscored = {}, None, ()
        candidates.sort(key=lambda d: d.range_m)
        perception.best = None
        if perception.reason == NOTHING_SCORED:
            perception.reason = OK
        return perception

    for d in candidates:
        d.scores, d.score = score(d, profile)
        d.unscored = tuple(n for n in profile if n not in d.scores)
    candidates.sort(key=lambda d: d.score, reverse=True)
    perception.best = (candidates[0] if candidates
                       and candidates[0].score >= min_score else None)
    if perception.reason in (OK, NOTHING_SCORED):
        perception.reason = OK if perception.best else NOTHING_SCORED
    return perception
