"""Piece 2: find the seafloor.

This runs before anything else, because the floor is the reference everything
is measured against. You are not looking for "something at 1.5 m depth" - you
are looking for "something about a metre above the bottom", and the bottom
moves as the sub moves.

How it works: in every downward direction, the FARTHEST return past the blind
zone is the bottom. Each implies a floor depth; if they agree, that agreement
is the floor, and if they don't, there is no flat bottom down there and we say
so rather than guessing.

Farthest, not brightest. An earlier version took the strongest return, which
broke the moment a pipeline passed underneath - the pipe is brighter than a
grazing bottom echo, so it got mistaken for the floor, and then masked away as
floor. Nothing lies below the seabed, so "last thing the sound hits" is a
sounder rule than "loudest".

The caveat that comes with it: multipath. In shallow water sound bounces
between surface and bottom, and a second-bounce ghost arrives at roughly twice
the true range. Cross-angle agreement catches some of that, but in a shallow
hard-walled pool expect this to struggle.

No ROS, no hardware.
"""
import math

import numpy as np

from . import settings as S

MIN_DOWNWARD = 0.25      # only rays angled at least this far below horizontal
AGREE_TOLERANCE_M = 0.35  # implied depths within this of the median count
MIN_AGREEING = 0.45       # at least this fraction must agree to call it a floor


class Floor:
    """Where the bottom is, as seen from the sonar.

    depth_m         metres below the sonar, positive
    confidence      0-1, how much of the sweep agreed
    """

    def __init__(self, depth_m, confidence, per_angle=None):
        self.depth_m = depth_m
        self.confidence = confidence
        self.per_angle = per_angle or {}

    def range_at(self, angle_deg):
        """How far the bottom should be in a given direction, or None if that
        direction never reaches it."""
        s = math.sin(math.radians(angle_deg))
        if s > -MIN_DOWNWARD:
            return None
        return self.depth_m / -s

    def height_above(self, angle_deg, range_m):
        """Height of a return above the floor. Negative means below it."""
        return range_m * math.sin(math.radians(angle_deg)) + self.depth_m

    def __repr__(self):
        return f"Floor({self.depth_m:.2f} m below, confidence {self.confidence:.2f})"

    @staticmethod
    def from_known_depth(depth_m):
        """Assert a floor rather than measuring one.

        For the shallow pool, where the bottom sits inside the 0.75 m blind
        zone and genuinely cannot be seen. Lets you exercise everything
        downstream without pretending the measurement worked.
        """
        return Floor(depth_m, confidence=0.0)


def find_floor(sweep, min_range_m=None):
    """Sweep -> Floor, or None if there is no consistent bottom in view.

    Returning None is a real answer, not a failure to be papered over. "I
    cannot see the bottom" and "I can see the bottom but nothing above it" are
    different situations and the driver should treat them differently.
    """
    if sweep.image.size == 0:
        return None
    if min_range_m is None:
        min_range_m = S.MIN_RANGE_M

    blind_cols = int(min_range_m / sweep.metres_per_bin)
    implied = []
    per_angle = {}

    for row in range(sweep.image.shape[0]):
        angle = sweep.row_to_angle_deg(row)
        s = math.sin(math.radians(angle))
        if s > -MIN_DOWNWARD:
            continue                       # not pointed down enough to hit floor

        strip = sweep.image[row, blind_cols:].astype(np.float32)
        if strip.size < 3:
            continue
        # smooth a little so a lone noise spike at long range cannot pose as
        # the farthest return
        k = np.ones(3, dtype=np.float32) / 3.0
        smooth = np.convolve(strip, k, mode="same")
        hot = np.flatnonzero(smooth >= S.DETECT["threshold"])
        if hot.size == 0:
            continue                       # nothing bright enough to be a floor
        col = int(hot[-1]) + blind_cols

        rng = sweep.col_to_range_m(col)
        depth = rng * -s                   # implied metres below the sonar
        implied.append(depth)
        per_angle[angle] = rng

    if len(implied) < 3:
        return None

    median = float(np.median(implied))
    agreeing = [d for d in implied if abs(d - median) <= AGREE_TOLERANCE_M]
    fraction = len(agreeing) / len(implied)

    if fraction < MIN_AGREEING:
        return None                        # returns disagree: no flat floor here

    return Floor(float(np.median(agreeing)), fraction, per_angle)
