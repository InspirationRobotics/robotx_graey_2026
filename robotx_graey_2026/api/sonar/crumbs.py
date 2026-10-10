"""Breadcrumbs: every echo, put on the top-down map where it really was.

The sonar is mounted on its side, so each ping looks out along one direction in
the slice across the sub (0 = right, 90 = up, 180 = left, 270 = down). An echo
at range r on bearing a is r*cos(a) to the right of the sonar and r*sin(a)
above it. Turned by the sub's heading and added to its position at THAT ping's
moment, the right/left part becomes a point on the map. The up/down part is
kept (fading needs it) but not drawn, so things at different depths overlap.

The beam is 2 deg wide in the slice but 25 deg fore-aft, so an echo at r metres
could have come from anywhere along ~0.44*r of the sub's heading. The crumb is
put in the middle of that.

What counts as an echo: the ping smoothed over 5 bins, then every stretch at or
above the threshold, past the blind zone, gives one crumb at its brightest
point. Up to MAX_PER_PING per ping, brightest first.

FADING. When a newer sweep looks at the spot a crumb is in again, the crumb
fades: one newer sweep = half faded, two = gone. "Looks at" means the spot is
in the MIDDLE of one of that sweep's pings - within half a step of its bearing,
within FADE_FAN_DEG fore-aft, and between the blind zone and the range. Only
the middle, because that is where the ping would put a new crumb: if the thing
is still there, the old crumb is replaced by an almost identical one. The
first version used the whole 25 deg fan, and a crumb at the fan's edge was
faded while the echo that replaced it went in the middle, up to ~0.6 m away 3 m
down; a sub turning on the spot wiped out 9 crumbs in 10 that way (sim, Oct 9). It does not matter
whether that sweep found something there: nothing there now is news too. A
crumb's own sweep never fades it, and a sweep fades a crumb at most once.

A ping with no pose, or one the DVL can't vouch for (valid=False), adds nothing
and fades nothing, because we would not know where it was looking.

Nor does a ping taken while the sub turns faster than MAX_TURN_DPS (Ruth's
call). Turning swings the 25 deg curtain along whatever is beside the sub, and
the echo comes from the nearest bit anywhere in it while the crumb goes in the
middle, so turns smear crumbs sideways. Fading is skipped too: a fast-turn ping
that saw something would otherwise wipe the old crumb with nothing to replace it.

Map frame and pose are pose.py's: x right, y forward at start, heading
clockwise from +y. The pose is the DVL's; the sonar sits SONAR_FWD_M ahead of
it and SONAR_RIGHT_M to its right (settings.py), and every crumb is measured
from the sonar.

TILT. The scan angles are the sub's own (DOWN_GRADIAN is how the sonar is
bolted on, measured with the sub upright), so each echo is turned into the map
with the VN-100's roll and pitch as well as the heading. A sub rolled 9 deg
would otherwise put a wall 2 m away ~0.3 m off in height and a little off in
distance, and its "straight down" ~0.25 m to one side. "up" is height relative
to the DVL, measured against gravity.
"""
import math

import numpy as np

from . import settings as S

THRESHOLD = 100         # smoothed echo strength, 0-255, that makes a crumb. A guess:
                        # the pool pipe peaked ~125 out to 2.25 m.
SMOOTH_BINS = 5
MAX_PER_PING = 5
FADE_SWEEPS = 2         # newer sweeps that must look at a crumb to wipe it out
FADE_FAN_DEG = 3.0      # ... within this many deg fore-aft of the beam's middle
MAX_TURN_DPS = 10.0     # turning faster than this, deg/s: ping ignored

_FIELDS = ("x", "y", "up", "brightness", "range_m", "sweep", "seen", "faded_by")


def body_to_map(pose):
    """3x3 matrix turning a vector in the sub's frame (forward, right, down)
    into the map's (+y, +x, down), from the pose's heading, pitch and roll."""
    r, p, y = (math.radians(v) for v in (pose.roll_deg, pose.pitch_deg, pose.heading_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class CrumbMap:
    def __init__(self, threshold=THRESHOLD, max_per_ping=MAX_PER_PING,
                 sonar_fwd_m=S.SONAR_FWD_M, sonar_right_m=S.SONAR_RIGHT_M,
                 max_turn_dps=MAX_TURN_DPS):
        self.threshold = threshold
        self.max_per_ping = max_per_ping
        self.max_turn_dps = max_turn_dps
        self.sonar_fwd_m = sonar_fwd_m
        self.sonar_right_m = sonar_right_m
        self.clear()

    def clear(self):
        self._c = {f: np.zeros(0) for f in _FIELDS}
        self.sweep_id = 0
        self.turning = False        # was the last ping ignored for turning?
        self.skipped_turning = 0

    def new_sweep(self):
        """Call at the start of every sweep, including one cut short."""
        self.sweep_id += 1

    def add_ping(self, angle_deg, row, metres_per_bin, pose, step_deg, max_range_m,
                 turn_dps=None):
        """Fade what this ping looks at, then add what it found. Returns the
        number of crumbs added. turn_dps is DeadReckoner.turn_rate at this ping;
        None (not known) does not block it."""
        if pose is None or not pose.valid:
            return 0
        self.turning = bool(turn_dps is not None and abs(turn_dps) > self.max_turn_dps)
        if self.turning:
            self.skipped_turning += 1
            return 0
        self._fade(angle_deg, pose, step_deg, max_range_m)
        return self._add(angle_deg, row, metres_per_bin, pose)

    def add_sweep(self, sweep, pose_at, max_range_m, turn_rate=None):
        """A whole Sweep at once (replay). pose_at(t) and turn_rate(t) are the
        DeadReckoner's."""
        self.new_sweep()
        for i, ang in enumerate(sweep.angles_deg):
            t = sweep.ping_times[i] if sweep.ping_times else None
            pose = pose_at(t) if t is not None else None
            turn = turn_rate(t) if turn_rate and t is not None else None
            self.add_ping(ang, sweep.image[i], sweep.metres_per_bin, pose,
                          sweep.step_deg, max_range_m, turn)

    def crumbs(self):
        """Copies of every live crumb's fields. seen = 0 fresh, 1 half faded."""
        return {f: v.copy() for f, v in self._c.items()}

    def __len__(self):
        return len(self._c["x"])

    def sonar_at(self, pose):
        """Where the sonar is when the DVL is at pose: (x, y, up), and the
        body-to-map matrix for that pose."""
        m = body_to_map(pose)
        n, e, d = m @ (self.sonar_fwd_m, self.sonar_right_m, 0.0)
        return pose.x + e, pose.y + n, -d, m

    def sonar_xy(self, pose):
        """Where the sonar is on the map when the DVL is at pose."""
        x, y, _, _ = self.sonar_at(pose)
        return x, y

    # ---- inside -----------------------------------------------------------
    def _fade(self, a_deg, pose, step_deg, max_range_m):
        c = self._c
        if not len(c["x"]):
            return
        older = (c["sweep"] < self.sweep_id) & (c["faded_by"] < self.sweep_id)
        if not older.any():
            return
        # every crumb, as seen from where the sonar is now, in the sub's frame
        sx, sy, sup, m = self.sonar_at(pose)
        fwd, right, down = m.T @ np.vstack([c["y"] - sy, c["x"] - sx, sup - c["up"]])
        up = -down
        across = np.hypot(right, up)
        bearing = np.degrees(np.arctan2(up, right))
        half = max(step_deg, S.BEAM_IN_PLANE_DEG) / 2.0
        in_slice = np.abs((bearing - a_deg + 180.0) % 360.0 - 180.0) <= half
        in_fan = np.abs(np.degrees(np.arctan2(fwd, across))) <= FADE_FAN_DEG
        dist = np.hypot(across, fwd)
        in_range = (dist >= S.MIN_RANGE_M) & (dist <= max_range_m)

        hit = older & in_slice & in_fan & in_range
        if not hit.any():
            return
        c["seen"][hit] += 1
        c["faded_by"][hit] = self.sweep_id
        keep = c["seen"] < FADE_SWEEPS
        if not keep.all():
            for f in _FIELDS:
                c[f] = c[f][keep]

    def _add(self, a_deg, row, metres_per_bin, pose):
        sm = np.convolve(np.asarray(row, float), np.ones(SMOOTH_BINS) / SMOOTH_BINS, "same")
        sm[:int(math.ceil(S.MIN_RANGE_M / metres_per_bin))] = 0
        above = sm >= self.threshold
        if not above.any():
            return 0
        edges = np.flatnonzero(np.diff(np.r_[0, above.astype(np.int8), 0]))
        peaks = []
        for s, e in zip(edges[0::2], edges[1::2]):
            top = np.flatnonzero(sm[s:e] == sm[s:e].max())
            peaks.append(s + int(top[len(top) // 2]))     # middle of a flat top, not its near edge
        peaks = sorted(peaks, key=lambda b: -sm[b])[:self.max_per_ping]

        r = np.array(peaks) * metres_per_bin
        a = math.radians(a_deg)
        sx, sy, sup, m = self.sonar_at(pose)
        # the echo in the sub's frame: in the scan plane, so forward = 0
        n, e, d = m @ np.vstack([np.zeros_like(r), r * math.cos(a), -r * math.sin(a)])
        new = {"x": sx + e,
               "y": sy + n,
               "up": sup - d,
               "brightness": sm[peaks],
               "range_m": r,
               "sweep": np.full(len(r), self.sweep_id),
               "seen": np.zeros(len(r)),
               "faded_by": np.zeros(len(r))}
        for f in _FIELDS:
            self._c[f] = np.concatenate([self._c[f], new[f]])
        return len(r)
