"""A fake sonar, so the whole chain runs with no hardware and no water.

Give it a scene - some pipe segments and a floor depth, in world coordinates -
and a sub position and heading, and it produces a Sweep exactly as the real
sonar would. Everything downstream cannot tell the difference.

This is what lets you develop the detector, the memory and the state machine on
a laptop, and it is also how the behaviour gets tested: put a pipeline
somewhere, fly a simulated sub at it, and watch what the driver decides.

World frame: x east, y north, z up. Heading is degrees clockwise from north.
Sub frame:   x starboard, y forward, z up - which is what the scan plane uses.
"""
import math

import numpy as np

from . import settings as S
from .sweep import Sweep


class Scene:
    """Pipe segments and a flat floor, in world coordinates."""

    def __init__(self, segments, floor_z=-4.0, pipe_value=225, floor_value=120):
        self.segments = [(np.array(a, float), np.array(b, float)) for a, b in segments]
        self.floor_z = floor_z
        self.pipe_value = pipe_value
        self.floor_value = floor_value

    @staticmethod
    def zigzag(start=(0.0, 0.0), z=-2.5, floor_z=-4.0,
               seg_len=1.1, first_heading_deg=0.0, turns=(45, -45, 45, -45)):
        """A pipeline like the build guide describes: five straight sections
        joined by 45 degree elbows, roughly 1 m each, a few metres end to end.

        Kept flat here for clarity. The real one explicitly is not - the guide
        says segments need not be planar with each other - so treat this as the
        easy case, not the realistic one.
        """
        pts = [np.array([start[0], start[1], z])]
        heading = first_heading_deg
        headings = [heading] + [heading := heading + t for t in turns]
        for h in headings:
            r = math.radians(h)
            last = pts[-1]
            pts.append(last + np.array([math.sin(r) * seg_len,
                                        math.cos(r) * seg_len, 0.0]))
        return Scene(list(zip(pts, pts[1:])), floor_z=floor_z)


def _to_sub_frame(point, sub_pos, heading_deg):
    """World point -> (starboard, forward, up) relative to the sub."""
    d = np.asarray(point, float) - np.asarray(sub_pos, float)
    h = math.radians(heading_deg)
    return np.array([d[0] * math.cos(h) - d[1] * math.sin(h),
                     d[0] * math.sin(h) + d[1] * math.cos(h),
                     d[2]])


def simulate_sweep(scene, sub_pos, heading_deg, start_deg, end_deg, step_deg,
                   max_range_m, n_bins=600, noise=8, seed=None):
    """Produce a Sweep as the real sonar would see this scene."""
    rng = np.random.default_rng(seed)
    angles = []
    a = float(start_deg)
    while a <= end_deg + 1e-9:
        angles.append(a % 360.0)
        a += step_deg

    m_per_bin = max_range_m / n_bins
    img = rng.integers(0, max(1, noise), size=(len(angles), n_bins)).astype(np.float32)
    index = {round(ang, 3): i for i, ang in enumerate(angles)}

    def paint(angle_deg, rng_m, value):
        if not (S.MIN_RANGE_M * 0.4) < rng_m < max_range_m:
            return
        # snap to the nearest swept angle, and only if it really was swept
        best, bestd = None, 1e9
        for ang, row in index.items():
            d = abs(((angle_deg - ang + 180) % 360) - 180)
            if d < bestd:
                best, bestd = row, d
        if best is None or bestd > step_deg:
            return
        col = int(rng_m / m_per_bin)
        half = max(1, int((S.BEAM_IN_PLANE_DEG / step_deg) / 2))
        for dr in range(-half, half + 1):
            for dc in (-2, -1, 0, 1, 2):
                r2, c2 = best + dr, col + dc
                if 0 <= r2 < img.shape[0] and 0 <= c2 < n_bins:
                    img[r2, c2] = max(img[r2, c2], value)

    # the floor: every downward ray eventually meets it
    depth = sub_pos[2] - scene.floor_z
    for ang in angles:
        s = math.sin(math.radians(ang))
        if s > -0.2:
            continue
        r = depth / -s
        graze = min(1.0, abs(s))
        paint(ang, r, scene.floor_value * (0.45 + 0.55 * graze))

    # the pipe: sample each segment, keep whatever falls inside the beam
    for p0, p1 in scene.segments:
        n = max(60, int(np.linalg.norm(p1 - p0) / 0.01))
        for t in np.linspace(0, 1, n):
            local = _to_sub_frame(p0 + t * (p1 - p0), sub_pos, heading_deg)
            r = float(np.linalg.norm(local))
            if r < 0.15:
                continue
            fore_aft = math.degrees(math.asin(max(-1.0, min(1.0, local[1] / r))))
            if abs(fore_aft) > S.BEAM_FORE_AFT_DEG / 2:
                continue
            ang = math.degrees(math.atan2(local[2], local[0])) % 360.0
            paint(ang, r, scene.pipe_value)

    return Sweep(np.clip(img, 0, 255).astype(np.uint8), angles, m_per_bin, heading_deg)


class FakeSonar:
    """Drop-in stand-in for Sonar, backed by a Scene.

    Same sweep_for_state() interface, so a mission script can run against
    either one by swapping which object it is handed.
    """

    def __init__(self, scene, sub_pos=(0.0, 0.0, 0.0), heading_deg=0.0, seed=1):
        self.scene = scene
        self.sub_pos = list(sub_pos)
        self.heading_deg = heading_deg
        self.seed = seed

    def sweep_for_state(self, state, heading_deg=None):
        cfg = S.SWEEP[state]
        if heading_deg is not None:
            self.heading_deg = heading_deg
        return simulate_sweep(self.scene, self.sub_pos, self.heading_deg,
                              cfg["start_deg"], cfg["end_deg"], cfg["step_deg"],
                              cfg["max_range_m"], seed=self.seed)

    # the sim's stand-in for the vehicle actually moving
    def apply(self, action):
        from .driver import FORWARD, STRAFE, YAW_BY, YAW_TO
        h = math.radians(self.heading_deg)
        if action.kind == YAW_TO:
            self.heading_deg = action.value % 360.0
        elif action.kind == YAW_BY:
            self.heading_deg = (self.heading_deg + action.value) % 360.0
        elif action.kind == FORWARD:
            self.sub_pos[0] += math.sin(h) * action.value
            self.sub_pos[1] += math.cos(h) * action.value
        elif action.kind == STRAFE:
            # starboard is 90 degrees clockwise of the heading
            self.sub_pos[0] += math.cos(h) * action.value
            self.sub_pos[1] += -math.sin(h) * action.value
        return self.heading_deg
