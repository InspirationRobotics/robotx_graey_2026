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

FADING (Map tab checkbox, on by default). When a newer sweep looks at the spot
a crumb is in again - anywhere in one of its pings' beams (FADE_DEG fore-aft,
within half a step of its bearing, between the blind zone and the range) - the
crumb fades: one newer sweep = half faded, two = gone. It does not matter
whether that sweep found something there: nothing there now is news too. A
crumb's own sweep never fades it, and a sweep fades a crumb at most once. Off,
crumbs stay until Reset.

(A "combine looks" mode - crumbs only where 3+ sweeps' pings agreed, looks.py -
was tried Oct 9-10 and removed: on the real pool runs it made arcs and blobs
and Ruth found it confusing.)

A ping with no pose, or one the DVL can't vouch for (valid=False), adds nothing
and fades nothing, because we would not know where it was looking.

THE FLOOR. Looking down, every ping hears the seafloor, so every sweep would
lay a line of floor crumbs across the map and bury what sits on it (sim, Oct 9).
The DVL measures how high it is above the bottom, so along each ping the map
knows how high above the floor each range is (along the beam's steeper edge,
which reaches the floor first); echoes less than floor_cut_m above it make no
crumbs. The handbook puts the pipeline 1-2 m up, so 0.5 m keeps it.
Off by default since Oct 10: in the pool it hid the pipe or did nothing, and
Ruth found the box confusing, so the Map page no longer has it; the height
band does that job by hand. pipeline_mission still sets it for its harbor
scans, looking down at a seafloor from a few metres up.
This takes the floor as flat out to the echo. With no altitude (no bottom lock,
or the DVL not heard lately) nothing is cut. floor_cut_m = 0 turns it off.

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

HEIGHT BAND (Ruth, Oct 10). In the 0.9 m pool, pings aimed up hear the pipe's
mirror image in the water surface, drawn in the air above it (77% of crumbs in
one run), and pings aimed down hear the floor. A narrower sector cuts those out
but can miss the pipe. The band keeps the sector and drops echoes whose height
from the sonar (against gravity, along the middle of the beam) is outside
(lo, hi) instead - before the strongest MAX_PER_PING are picked, so junk
outside the band can't crowd the pipe out. It moves with the sub: fine while
the depth is held.
"""
import math

import numpy as np

from . import settings as S

THRESHOLD = 100         # smoothed echo strength, 0-255, that makes a crumb. A guess:
                        # the pool pipe peaked ~125 out to 2.25 m.
SMOOTH_BINS = 5
MAX_PER_PING = 5
FADE_SWEEPS = 2         # newer sweeps that must look at a crumb to wipe it out
FADE_DEG = S.BEAM_FORE_AFT_DEG / 2.0    # ... anywhere in the beam, fore-aft
MAX_TURN_DPS = 10.0     # turning faster than this, deg/s: ping ignored
FLOOR_CUT_M = 0.0       # echoes less than this above the floor make no crumbs; 0 = off (missions set it)

_FIELDS = ("x", "y", "up", "brightness", "range_m", "sweep", "seen", "faded_by")

# LITTLE DOTS ONLY (Ruth, Oct 11). In the pool a wall is the brightest thing, and
# it is tall: in one sweep (the scan plane is up-down) it is heard at the same
# distance to the side from the floor to the surface, plus its reflections. The
# pipe is a small dot, smeared over neighbouring angles at the same range. On
# the Oct 11 run (sector 140-200) the echoes 2-2.5 m out (the wall) spanned
# 2.3 m of height (median) and those 1-1.5 m out (the pipe) 1.0 m; calling
# anything over TALL_M a wall dropped 90% of the wall's dots and kept 72% of
# the pipe's. Needs a sector of ~60 deg or more so a wall can show its height.
TALL_M = 1.5            # height an echo column must span to count as a wall
TALL_BIN_M = 0.1        # columns: this wide in distance to the side, +-1 neighbour
TALL_REL = 0.6          # ... counting echoes at least this share of the column's strongest
TALL_MIN_RANGE_M = S.MIN_RANGE_M + 0.1      # the blind zone's ringing edge is not a wall
LITTLE_DOTS_NOTE = "little dots only: long echoes are left off the map"


def smooth(image):
    """The 5-bin smoothing every crumb decision uses, for a whole sweep (pings x bins)."""
    k = np.ones(SMOOTH_BINS) / SMOOTH_BINS
    return np.array([np.convolve(np.asarray(r, float), k, "same") for r in image])


def tall_mask(sm, angles_deg, metres_per_bin, threshold):
    """Which samples of a smoothed sweep are part of a tall, wall-like column.

    In the scan plane each sample sits at (distance to the side, height). Group
    samples into TALL_BIN_M columns by distance to the side; a column is tall
    when its echoes (at least threshold and TALL_REL of its strongest) span more
    than TALL_M of height. Returns a bool array the shape of sm: the tall columns' echoes."""
    sm = np.asarray(sm, float)
    a = np.radians(np.asarray(angles_deg, float))[:, None]
    r = (np.arange(sm.shape[1]) + 0.5) * metres_per_bin
    side = -np.cos(a) * r                     # + left, - right, so walls either side stay apart
    up = np.sin(a) * r
    strong = (sm >= threshold) & (r[None, :] >= TALL_MIN_RANGE_M)
    out = np.zeros(sm.shape, bool)
    if not strong.any():
        return out
    col = np.round(side / TALL_BIN_M).astype(int)
    for c in np.unique(col[strong]):
        near = strong & (np.abs(col - c) <= 1)
        top = sm[near].max()
        lit = near & (sm >= TALL_REL * top)
        if up[lit].max() - up[lit].min() > TALL_M:
            out |= strong & (col == c)
    return out


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
                 max_turn_dps=MAX_TURN_DPS, floor_cut_m=FLOOR_CUT_M, fade=True, band=None,
                 compact=False):
        self.threshold = threshold
        self.floor_cut_m = floor_cut_m
        self.max_per_ping = max_per_ping
        self.max_turn_dps = max_turn_dps
        self.sonar_fwd_m = sonar_fwd_m
        self.sonar_right_m = sonar_right_m
        self.fade = fade            # False: crumbs stay until Reset (a faint pipe builds up)
        self.band = band            # (lo, hi) m from the sonar's height, or None = every height
        self.compact = compact      # True: little dots only - wall-like tall echoes make no crumbs
        self.clear()

    def clear(self):
        self._c = {f: np.zeros(0) for f in _FIELDS}
        self.sweep_id = 0
        self.turning = False        # was the last ping ignored for turning?
        self.skipped_turning = 0
        self._pending = []          # little dots only: this sweep's pings, judged when it ends

    def new_sweep(self):
        """Call at the start of every sweep, including one cut short."""
        self.end_sweep()
        self.sweep_id += 1

    def end_sweep(self):
        """Little dots only: a ping can't tell a wall from a dot by itself, so its
        echoes wait until the sweep is over; then the tall ones are dropped and
        the rest become crumbs. Called by new_sweep; call it after the last sweep
        too (a replay's end). Returns the number of crumbs added."""
        pend, self._pending = self._pending, []
        if not pend:
            return 0
        n = len(pend[0][1])
        pend = [p for p in pend if len(p[1]) == n and p[2] == pend[0][2]]   # one sweep's geometry
        tall = tall_mask([p[1] for p in pend], [p[0] for p in pend], pend[0][2], self.threshold)
        added = 0
        for k, (a_deg, raw, mpb, pose, sm, peaks) in enumerate(pend):
            keep = [b for b in peaks if not tall[k, b]][:self.max_per_ping]
            added += self._place(a_deg, keep, sm, mpb, pose)
        return added

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
        if self.fade:
            self._fade(angle_deg, pose, step_deg, max_range_m)
        if self.compact:
            raw, sm, peaks = self._peaks(angle_deg, row, metres_per_bin, pose)
            self._pending.append((angle_deg, raw, metres_per_bin, pose, sm, peaks))
            return 0
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

    def for_page(self):
        """What the Map tab gets: crumbs [x, y, brightness, seen, up], and the
        same crumbs as arrays for outlines.py (raw = their x, y, up)."""
        c = self.crumbs()
        crumbs = np.column_stack([np.round(c["x"], 2), np.round(c["y"], 2), np.round(c["brightness"]),
                                  c["seen"], np.round(c["up"], 2)]).tolist()
        return crumbs, dict(c, raw=(c["x"], c["y"], c["up"]))

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
        in_fan = np.abs(np.degrees(np.arctan2(fwd, across))) <= FADE_DEG
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
        _, sm, peaks = self._peaks(a_deg, row, metres_per_bin, pose)
        return self._place(a_deg, peaks[:self.max_per_ping], sm, metres_per_bin, pose)

    def _peaks(self, a_deg, row, metres_per_bin, pose):
        """The ping smoothed (raw, for the wall test), then with the blind zone,
        floor cut and band applied (sm), and its echoes' bins, strongest first."""
        raw = np.convolve(np.asarray(row, float), np.ones(SMOOTH_BINS) / SMOOTH_BINS, "same")
        sm = raw.copy()
        sm[:int(math.ceil(S.MIN_RANGE_M / metres_per_bin))] = 0
        a = math.radians(a_deg)
        sx, sy, sup, m = self.sonar_at(pose)
        # metres down per metre of range, along the steeper edge of the 2 deg beam:
        # that edge meets the floor first, ~0.35 m sooner 12 m out at 45 deg
        edge = math.radians(S.BEAM_IN_PLANE_DEG / 2.0)
        down = max((m @ (0.0, math.cos(a + k), -math.sin(a + k)))[2] for k in (-edge, edge))
        if self.floor_cut_m > 0 and pose.alt_m > 0 and down > 1e-3:
            # height above the floor at range r: alt + sup - r * down
            near_floor = (pose.alt_m + sup - self.floor_cut_m) / down
            sm[max(0, int(near_floor / metres_per_bin)):] = 0
        if self.band is not None:
            # height above the sonar at range r is r * rise, the same sum that places the crumb
            rise = -(m @ (0.0, math.cos(a), -math.sin(a)))[2]
            h = np.arange(len(sm)) * metres_per_bin * rise
            sm[(h < self.band[0]) | (h > self.band[1])] = 0
        above = sm >= self.threshold
        if not above.any():
            return raw, sm, []
        edges = np.flatnonzero(np.diff(np.r_[0, above.astype(np.int8), 0]))
        peaks = []
        for s, e in zip(edges[0::2], edges[1::2]):
            top = np.flatnonzero(sm[s:e] == sm[s:e].max())
            peaks.append(s + int(top[len(top) // 2]))     # middle of a flat top, not its near edge
        return raw, sm, sorted(peaks, key=lambda b: -sm[b])

    def _place(self, a_deg, peaks, sm, metres_per_bin, pose):
        """Crumbs at these bins of one ping."""
        if not len(peaks):
            return 0
        a = math.radians(a_deg)
        sx, sy, sup, m = self.sonar_at(pose)
        r = np.array(peaks) * metres_per_bin
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
