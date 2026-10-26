#!/usr/bin/env python3
"""Task 2: find the pipeline with the sonar map, then follow it and mark the light boxes.

Ruth's plan (Oct 2026). It starts where the buoy code leaves the sub: at the
active buoy, on the surface. Built in steps, each checked in the simulator:

  DIVE        to far_depth (3 m; the harbor is ~12 m deep)
  FAR SCAN    turn once on the spot with the sonar map running. From up here
              the pipe gives only scattered crumbs - each sweep crosses it in
              about one ping, and the 25 deg fan is ~3.5 m wide 8 m down - so
              this only finds WHERE: the biggest bunch of crumbs, joined at
              far_link_m instead of the map's 0.5 m
  TO SPOT     over that bunch, still at far_depth
  DESCEND     down until the sonar says the highest thing below (pipe or light
              box) is close_height away (Ruth: depth near the pipe is measured
              from the pipe). Down in 0.5 m steps while nothing is seen yet.
              Never below the floor limit: DVL altitude pipe_max_height +
              close_height (the pipe's highest possible top, Ruth's rule)
  CLOSE SCAN  the same turn, map Reset, shorter range: crumbs dense enough to
              join into outlines (api/sonar/outlines.py)
  PICK        the most pipe-like outline; none scoring min_score -> surface.
              Its near end is the one nearest the buoy (the handbook's report
              order starts at the end nearest the active buoy)
  (next)      go above the near end, follow it, mark the boxes

Every scan turns slower than the map's 10 deg/s limit, so no ping is thrown away.

POOL PLAN (plan:=pool; Ruth, Oct 10; for the lake too, Oct 14). With the pipe
about level with the sonar and the sonar looking to one side (pool_side; left =
sector 140 -> 210, as Ruth sweeps), it tests the real scanning and the approach.
The scan uses Ruth's map settings: threshold 100, little dots only, fade off, and
a height band around sonar level (pool_band), so the floor and the sonar's own
ringing don't become the pick. At threshold 100 the pipe is heard to only about
1.5-2 m (Oct 3 measurements; the sim is tuned to them), so START THE RUN WITHIN
ABOUT 1.5 M OF THE PIPE:
  DIVE        to pool_depth (0.3 m)
  POOL SCAN   one slow turn, the map at pool_range (6 m) looking sideways
  PICK        the most pipe-like outline, and its crumb nearest the sonar
  FACE        turn so that point is straight to the pool_side, where the sonar looks
  APPROACH    strafe sideways toward it at approach_speed - a target that moves
              (stream_posvel), since GUIDED alone runs at WPNAV_SPEED (0.45 m/s on
              Graey) and overshot 0.2 m in the sim - re-aiming as new crumbs land,
              until the sonar is approach_range (1.2 m) from it: in the pool the
              pipe vanished into the 0.75 m blind zone and its ringing (to about
              1.05 m) whenever the sub got closer than that
  HOLD        hold_s seconds there, then surface where it is

pool_max_pick_m (2.5) ignores shapes farther from the sonar than that: walls. For a
pool test the scan need not be a full circle - the sonar looks one way, so with the
pipe ~1.5 m to that side, scan_turn_deg:=90 sweeps along 2-3 m of it, and a sub
beside the pool wall never looks at the wall (Oct 14).

SPOT CHECK (spot_check, Ruth, Oct 16). Her labels of every Oct 15 sweep: from
beside the pipe it showed in 9 of 94 sweeps, from over it in 22 of 28, and best
with the sub facing along it. So after the scan the top spot_count shapes scoring
spot_min_score become spots: the sonar goes over each, facing along the shape,
and the sub turns -spot_turn_deg/2, +spot_turn_deg/2 and back, slowly, with the
map looking straight down. Following starts from the spot whose sweeps showed
pipe dots within spot_radius_m most often (at least spot_min_frac of them).

SEARCH (scan_mode:=search, Ruth, Oct 23). A stop every search_step_deg, sweeps_per_stop
sweeps still at each; a stop is promising when dots within blob_m of each other came
from blob_sweeps of its sweeps. One stop only ever gives a bunch: each sweep is one
slice across the sub, so at one heading every sweep cuts the pipe at the same point.
So a promising stop gets a closer look - more stops look_step_deg apart, out to
look_span_deg either side - each cutting the pipe somewhere else, so the dots draw
a line; a shape scoring look_min_score near the bunch passes. Then, looking straight
down (Oct 25), the sonar goes over the shape's densest spot facing along it (step 1),
checks the pipe is under it (sliding across if not), and walks straight that way
(step 2): the pipe drifting sideways says how far off step 1 was. Once it is
align_turn_m off the sonar's middle, or after align_walk_m, it turns by that much,
the sonar over the pipe where it has got to, and follows. A turn on the spot can't
do step 2: its slices cross the pipe within ~0.2 m of each other (Oct 23). No pass in
the whole turn: surface.

LOST (follow_searches, Ruth, Oct 23). No pipe for follow_lost_s mid-follow: back over
where it was last seen and the same search from there, leaving out the part already
followed, then follow on. follow_searches searches at one spot (within SAME_SPOT_M)
that find nothing or lose it there again: its end, as before.

ALTITUDE (pool_alt_m, Ruth, Oct 24). Depth from the bottom, not from the start: the dive
goes down until the DVL reads pool_alt_m to the bottom (none if it already does), and
pool_depth and pool_max_depth are not used - the only depth limit is ALT_MAX_DEPTH_M below
the start, for a DVL fault. While searching it never gets closer than SEARCH_MIN_ALT_M to
the bottom (it rises), so it stays above a pipe standing up to 2 m. A search after the
pipe is lost mid-follow goes back up to pool_alt_m first.

DOWN TO THE PIPE (Ruth, Oct 24). Over the pipe before the align walk, it goes down until
the pipe (its height from the search) is WALK_PIPE_BELOW_M under the sonar: from the
search's height the walk's echoes were too weak and the floor out of its range (lake,
Oct 24). A walk that fails goes back up to the search's depth. The walk itself waits for
ALIGN_SWEEPS finished sweeps on the way, however long a sweep takes (lake: ~6 s).

start_with:=follow skips the scan: the pilot leaves the sub over the pipe, facing
along it, and following starts there (after the align walk).

REPAIR (repair_centre, Ruth, Oct 22). The magnet sits on a clear board in the down
camera's view (magnet_px), so the RED light is centred on the magnet IN THE PICTURE:
line up (GUIDED, centre_speed), step down centre_step_m, line up again, until the
light turns GREEN; then up watch_rise_m and watch the colours for watch_s. Lined up
in the picture, the light is on the magnet's line of sight, which is directly under
the magnet once the magnet reaches it, so no camera calibration is needed. In the
survey it replaces the hold over the RED light; start_with:=centre tries it alone
(plan:=pool, the pilot leaves the sub over the light), then holds 5 s and surfaces.

stop_after:=scan|pick|face|approach|over|align|spots ends the run after that stage (hold, then
surface), so each stage can be tried alone. In a dry run the scan waits for the
PILOT to turn the sub through scan_turn_deg (read from the Cube's attitude), so
a dry run while someone drives tests the real pick.

THE MAP. tools/sonar_map.py runs beside this (on Graey, on the host; in the sim,
sim/sim_sonar_map.py) and is driven through its web page's own controls, the
same as pressing the buttons: range, sector, threshold, Reset/Start. Its frame
is (0,0) = where the sub was at Start, +y = the heading then; the EKF's
position and heading at that moment turn map points into NED. Crumb heights
("up") are relative to the DVL at the ping, so with the depth held, the pipe's
top is the scan depth minus the highest crumb's up.

DEPTHS. Before and after the pipe (the dive, the far scan, surfacing) depth is
from the surface, taken as the depth reading when the mission starts, so a
depth sensor that is off by a constant does no harm (the sim's drifted 2 m).
Near the pipe it is from the pipe, as above.

SAFETY: as every MissionBase mission - dry_run defaults True and then nothing
is sent; changing flight mode hands the sub to the pilot and this exits. No
target deeper than max_depth is ever sent.
"""
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np
import rclpy
from std_msgs.msg import Float32

from robotx_graey_2026.api.sonar import settings as sonar_settings

from robotx_graey_2026.api.navigation import mission_base
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S
from robotx_graey_2026.api.sonar import outlines
from robotx_graey_2026.api.vision import led_boxes

# =====================================================================================
#  SETTINGS - the numbers worth changing (lake / competition). Ruth, Oct 24.
#
#  Run a part of the mission with ONE word, on Graey (tools/task2.sh):
#      task2 search      task2 follow      task2 survey      task2 repair      task2 full
#  and add 'dry' to rehearse it without moving the sub:   task2 survey dry
#
#  Every run uses the numbers below. To try a different number for ONE run, add
#  -p name:=value after the word (decimals keep their dot):  task2 search -p pool_alt_m:=3.5
#  'pool_...' settings are the SEARCH view, 'follow_...' the WALK AND FOLLOW view.
# =====================================================================================
SETTINGS = {
    # ---- depth ----------------------------------------------------------------------
    'pool_alt_m':          4.0,        # dive until the bottom is this far below the sub (m)
                                       #   (no dive if it already is)
    'follow_pipe_below_m': 0.7,        # while following, keep the pipe this far below the sonar (m)
    'follow_pipe_tol_m':   0.15,       # ... give or take this much before changing depth (m)

    # ---- search: the sonar looks down, both sides ----------------------------------------
    'pool_range':          7.0,        # how far the sonar listens (m); 4 m above the bottom, the bottom
                                       #   at the sweep's edge (45 deg) is ~6 m away
    'pool_sector':         '225 315',  # the sonar's sweep (deg): 270 is straight down, so this is
                                       #   45 deg either side of straight down
    'pool_far_m':          7.0,        # ignore echoes farther from the sonar than this (m, along the beam)
    'scan_turn_deg':       180.0,      # total turn (deg); looking both ways, 180 covers every direction
    'search_step_deg':     30.0,       # turn between stops (deg, rounded to divide the turn evenly):
                                       #   30 = 6 stops over 180 (Ruth, Oct 25; was 25 = 8)
    'sweeps_per_stop':     2,          # sonar sweeps held still at each stop
    'floor_filter':        '0.75 2.25',  # keep only echoes this high above the bottom (m, low high)
    'look_min_score':      0.2,        # how pipe-like the closer look's shape must be to pass (0-1)
    'stop_tol_deg':        10.0,       # a stop starts looking once the heading is this close to it for 2 s
                                       #   (Oct 25 pool: the sub swung +-12 deg in its heading hold; was 3,
                                       #   which also waited for the turning to stop)

    # ---- walk and follow: the sonar looks straight down ----------------------------------
    'follow_range':        6.0,        # how far the sonar listens (m); must reach past the bottom
    'follow_speed':        0.08,       # top speed along the pipe (m/s)
    'align_walk_m':        0.6,        # the line-up walk: at most this far straight along the pipe (m) ...
    'align_turn_m':        0.4,        # ... turning as soon as the pipe is this far off the sonar's middle (m)
    'follow_searches':     2,          # pipe lost: search again this many times at one spot, then it's the end
    'follow_max_m':        40.0,       # stop after following this far (m)
    'follow_time_s':       1200.0,     # ... or after this long (s)

    # ---- survey and repair ---------------------------------------------------------------
    'light_detector':      'contrast', # how the down camera finds lights: 'contrast' = a round patch much
                                       #   brighter than the water around it, colour from its glow (Oct 25:
                                       #   no false GREEN on the Oct 24 lake video, pool LEDs as before);
                                       #   'hsv' = the old fixed colour test (82% of the lake picture 'green');
                                       #   'yolo' = a trained model (light_model; contrast if it won't load)
    'light_model':         '/root/robotx_ws/src/robotx_graey_2026/models/lights.pt',   # the yolo model
    'end_model':           '/root/robotx_ws/src/robotx_graey_2026/models/pipe_end.pt',
                                       # a yolo model with a pipe_end class, for the pipe's ends only (the
                                       #   lights stay on light_detector): '' = none, ends from the sonar
                                       #   (also the sonar if the file isn't there or won't load)
    'end_conf':            0.25,       # ... the confidence a pipe end needs (pipe_end.pt, Oct 25 held-back
                                       #   frames: 0.25 found all 28 ends, 1 false box in 139 frames; 0.4 found 13)
    'light_above_floor_m': 1.6,        # the light box's top above the bottom (m): sets the repair's first step down
    'centre_max_down_m':   0.65,       # repair: never more than this below where centring began (m)

    # ---- time -----------------------------------------------------------------------------
    'phase_timeout':       1200.0,     # give up on any one step after this long (s)
}

# What each word runs. These switch parts of the mission on and off - not numbers to tune.
PARTS = {
    'search': ('dive, find the pipe, go over it and line up; hold 5 s, surface',
               {'stop_after': 'align'}),
    'follow': ('the pilot leaves the sub over the pipe: line up, follow it; hand back in POSHOLD',
               {'start_with': 'follow'}),
    'survey': ('dive, find the pipe, follow it to the buoy end, then the whole pipe logging each '
               'light; 5 s over RED; hand back in POSHOLD',
               {'follow_mode': 'survey', 'repair_hold_s': 5.0}),
    'repair': ('the survey, centring the magnet on the RED light until it turns GREEN; hand back in POSHOLD',
               {'follow_mode': 'survey', 'repair_centre': True}),
    'full':   ('the competition run: the repair survey, then surface',
               {'follow_mode': 'survey', 'repair_centre': True, 'follow_end': 'surface'}),
}

# Lake and competition: how the code is set up there. Not for changing.
FIXED = {'plan': 'pool', 'scan_mode': 'search', 'pool_band': 'none', 'pool_compact': False,
         'survey_buoy': 'start', 'pool_max_pick_m': 0.0, 'follow_near_m': 0.45}

# The simulator never left the surface with unchanged targets re-sent every 3 s
# (sim/README.md). Here a target changes when the plan does, and is otherwise
# only re-sent as insurance.
mission_base.RESEND_S = 20.0
SETTLE_S = 6.0          # after a scan: the last sweep lands and the outlines refresh
DESCEND_SECTOR = '240 300'   # +-30 deg either side of straight down, so sweeps come fast
BELOW_RADIUS_M = 1.0    # echoes this close to straight below count as "below"
MODE_POSHOLD = 16       # ArduSub custom mode number (mode.h)


SIDE_WINDOW_DEG = 60.0  # pool plan: the pick's nearest point this close to straight out the sonar's side
BEHIND_M = 0.5          # ... and its middle no farther behind where the scan started
POOL_FULL_M = 0.8       # pool scan: a shape this long gets full length marks (none below 0.3 m)
CENTRE_GAIN = 0.3       # repair centring: each picture's guess of the lined-up spot counts this much
GREEN_DONE = 0.05       # repair: the light has turned GREEN once green fills this share of the picture
RED_SEEN = 0.02         # ... and is still the RED light while red fills at least this share
WATCH_MIN_SHARE = 0.02  # watching the colours: less of any colour than this is 'off'
C_GREEN_FRAMES = 3      # light_detector:=contrast: GREEN once this many pictures running say so
END_SEEN_S = 3.0        # light_detector:=yolo: a pipe end the camera saw this recently ...
END_MID_X = 0.6         # ... this near the picture's middle across (-1 to 1) ...
END_BEHIND_Y = 0.3      # ... and no further back in it than this (-1 top/ahead to 1 bottom/behind): the end is here
LOOK_NEAR_M = 0.5       # search: the closer look's shape passes this close to the promising bunch
DENSE_M = 0.15          # ... its densest spot: the dot with the most others this close
PATH_M = 0.35           # lost search: dots this close to where the sonar went are the part already followed
SAME_SPOT_M = 0.5       # ... the pipe lost this close to where it was lost before: the same spot
ALIGN_GATE_M = 0.3      # align walk: dots this far either side of where the pipe was last seen count -
ALIGN_FIRST_M = 0.6     # ... this far until it is first seen: the side-looking search places the pipe up to
                        # ~0.2 x range off along the sub (the beam is 25 deg thick; lake sim, Oct 24)
ALIGN_UP_M = 0.12       # ... and this far above or below its height (a light box on it is ~0.15 higher)
ALIGN_SWEEPS = 5        # ... walked slowly enough for about this many sweeps
ALIGN_MIN_SPAN_M = 0.25 # ... and the dots fitted spread at least this far along it
ALIGN_EDGE_M = 0.55     # ... the pipe this far off the middle (1 m down, the +-30 deg view is +-0.58): turn now
BUOY_END_M = 3.0        # survey, on the way to the buoy: the pipe lost this close to the buoy is that end (no search)
DZ_SPEED = 0.05         # follow depth (follow_pipe_below_m): the held depth moves no faster than this, m/s,
DZ_MAX_M = 1.5          # ... never more than this from where following began,
DZ_MIN_ALT_M = 0.5      # ... nor this close to the floor (DVL)
ALT_DZ_SPEED = 0.1      # pool_alt_m's dive, the search's rise and the step down to the pipe: m/s
SEARCH_MIN_ALT_M = 2.5  # pool_alt_m: while searching, never closer to the bottom than this (DVL)
ALT_DIVE_WAIT_S = 20.0  # ... no DVL altitude on the dive this long: search from where it is
ALT_MAX_DEPTH_M = 6.0   # ... and never deeper than this below the start: a DVL reading wrong (Ruth, Oct 25)
WALK_PIPE_BELOW_M = 1.0 # over the pipe before the walk: down until it is this far under the sonar
OVER_CHECK_SWEEPS = 2   # over the picked spot: look straight down for this many sweeps ...
OVER_CHECK_M = 1.0      # ... for the pipe this far either side of the sonar ...
OVER_SLIDE_M = 0.15     # ... and slide over it when it is at least this far to one side
BOX_H_M = 0.15          # a light box on the pipe: its top this far above the pipe's
MAGNET_BELOW_M = 0.15   # the magnet's bottom under the down camera (and the sonar/DVL level; Ruth's ruler, Oct 22)


def pool_candidates(found, sonar_xy, side, max_pick, ignore_flat):
    """The shapes the pool plan may pick, best first, as (score, shape):
    - within max_pick m of the sonar (0 = any distance);
    - toward the sonar's side of where the scan started: the map is (0,0) at the
      sub when the scan began, +y its heading then. The shape's point nearest the
      sonar (where the approach goes) must lie within SIDE_WINDOW_DEG of straight
      left (side -1) or right (+1) from there, and its middle no more than
      BEHIND_M behind the start, so a wall behind the sub isn't picked (Oct 14 dry
      runs) while a long pipe reaching far ahead still is (sim). side 0 skips this:
      any side, any direction (Ruth, Oct 15: "look everywhere", sonar looking down);
    - length counted fully from POOL_FULL_M, since one spot hears only ~1 m of pipe;
    - scored without the 'flat' part when a height band already caps the heights,
      because then a pipe's smear and a wall look the same height-wise (Oct 14).
    Returns (candidates, ignored-for-distance, ignored-for-side)."""
    out, far, wrong_side = [], 0, 0
    for g in found:
        near = min(g['spine'], key=lambda q: math.hypot(q[0] - sonar_xy[0], q[1] - sonar_xy[1]))
        if max_pick > 0 and math.hypot(near[0] - sonar_xy[0], near[1] - sonar_xy[1]) > max_pick:
            far += 1
            continue
        bearing = math.degrees(math.atan2(near[0], near[1]))  # 0 ahead, -90 left, +90 right
        mid_y = float(np.mean(np.asarray(g['spine'], float)[:, 1]))
        if side and (abs((bearing - 90.0 * side + 180.0) % 360.0 - 180.0) > SIDE_WINDOW_DEG
                     or mid_y < -BEHIND_M):
            wrong_side += 1
            continue
        score = g['score']
        if g.get('parts'):
            parts = dict(g['parts'])
            # from one spot the sonar hears only ~1 m of a pipe 1.2-1.5 m away, so a
            # pool shape is long enough from POOL_FULL_M (the map's own rule wants 1.5 m)
            parts['length'] = float(np.clip((g['length'] - 0.3) / (POOL_FULL_M - 0.3), 0, 1))
            if ignore_flat:
                parts.pop('flat', None)
            score = float(np.prod(list(parts.values())))
        out.append((score, g))
    out.sort(key=lambda sg: -sg[0])
    return out, far, wrong_side


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PipelineMission(MissionBase):
    def __init__(self):
        super().__init__('pipeline_mission', component=197)
        self.phase = None
        self.scan = None                    # 'far' or 'close'
        self.map_origin = None              # (n, e, yaw) of map (0,0) and its +y
        self.hold = None                    # (n, e, down, yaw) while scanning
        self.turned = 0.0
        self.stop_i, self.stop_yaw0, self.still_t, self.look_from = 0, 0.0, None, 0
        self.last_yaw = None
        self.spot = None                    # (n, e, pipe top depth) from the far scan
        self.pipe = None                    # the outline picked, in NED
        self.map_down_since = None
        self.z_surf = 0.0
        self.surface_at = None              # pool plan: (n, e, yaw) to surface at
        self.pipe_spine = None              # pool plan: the picked outline, map frame
        self.aim = None                     # pool plan: the point approached, map frame
        self.recording = False              # a scan recording this mission started and hasn't stopped
        self.approach_yaw = None
        self.retarget_t = 0.0
        self.carrot = None                  # pool plan: the moving target, NED
        self.spots, self.spot_i = [], 0     # spot check: the likely shapes to look at again
        self.spot_active = None             # ... the one being gone to / looked at
        self.srch = None                    # a search under way: scan_mode:=search, or the pipe lost mid-follow
        self.look = None                    # ... its closer look at a promising stop
        self.lost_spots = []                # where the pipe was lost mid-follow, and how often searched there
        self.aligned = False                # the align walk is done for this approach
        self.align_rev = False              # ... and has been tried the other way too
        self.a_up = None                    # the pipe's height (map 'up') as the search saw it
        self.a_up_z = None                  # ... and the depth it was seen from
        self.low_for_walk = False           # gone down over the pipe for the walk (no search guard)
        self.back_z = None                  # search back: the depth to go back to, if any
        self.dive_wait_t, self.dive_logged = None, False   # pool_alt_m's dive
        self.guard_on = False               # pool_alt_m: rising, too close to the bottom while searching
        self.wd = None                      # down to the pipe: {'want', 'z0'}
        self.over_checked = False           # looked straight down once over the picked spot
        self.cam_ends_only = False          # the camera runs for pipe ends only (no lights counted yet)
        self.end_said = None                # the pipe end the log last mentioned
        self.f_path = []                    # where the sonar went while following, NED
        self.last_t = 0.0
        self.step_z = None                  # DESCEND's current target
        self.altitude = None                # (metres above the bottom, time); -1 = no lock
        self.create_subscription(Float32, '/graey/dvl/altitude', self.on_altitude, 10)

    def on_altitude(self, msg):
        self.altitude = (msg.data, self.now())
        if msg.data > 0:                    # the last 10 s of good readings, for the follow's band
            self.alt_hist = [(t, a) for t, a in getattr(self, 'alt_hist', []) if self.now() - t <= 10.0]
            self.alt_hist.append((self.now(), float(msg.data)))

    def floor_clearance(self):
        """Metres from the DVL to the bottom, or None if the DVL has no lock now."""
        if self.altitude is None or self.altitude[0] <= 0 or self.now() - self.altitude[1] > 2.0:
            return None
        return self.altitude[0]

    def alt_now(self, span=3.0):
        """The DVL's altitude, the median of the last span s (it jumps +-0.3 m over a sloping
        bottom, lake Oct 24), or None without a lock now."""
        if self.floor_clearance() is None:
            return None
        h = [a for t, a in getattr(self, 'alt_hist', []) if self.now() - t <= span]
        return float(np.median(h)) if h else None

    def ramp(self, z, want, rate):
        """z moved toward want at no more than rate m/s since the last call."""
        now = self.now()
        dt, self.ramp_t = min(now - getattr(self, 'ramp_t', now), 0.5), now
        return z + max(-rate * dt, min(rate * dt, want - z))

    def a_up_now(self):
        """The search's (or the walk's) pipe height from the sonar, moved by how far the sub has
        gone down since it was measured: dots are placed relative to the sub at their ping."""
        if self.a_up is None:
            return None
        z = self.cur[2] if self.cur else self.hold[2]
        return self.a_up + (z - (self.a_up_z if self.a_up_z is not None else z))

    def reached_depth(self):
        """pool_alt_m's dive: down (ALT_DZ_SPEED, over where it started) until the DVL reads
        pool_alt_m to the bottom - none if it already does. MissionBase's DIVE sends self.depth."""
        if self.pool_alt <= 0:
            return super().reached_depth()
        if self.dry:
            if not self.dive_logged:
                self.dive_logged = True
                self.get_logger().info(f'    [dry] dive: down until the DVL reads {self.pool_alt:.2f} m to the bottom')
            return super().reached_depth()
        if self.cur is None:
            return False
        alt, below = self.alt_now(), self.cur[2] - self.z_surf
        if alt is None:
            if self.dive_wait_t is None:
                self.dive_wait_t = self.now()
                self.get_logger().warn(f'dive: no DVL altitude - holding at {below:.2f} m down '
                                       f'(searching from here after {ALT_DIVE_WAIT_S:.0f} s without it)')
            self.depth = max(0.0, min(self.depth, below))           # stop going down
            if self.now() - self.dive_wait_t > ALT_DIVE_WAIT_S:
                self.get_logger().warn(f'dive: still no DVL altitude - searching from {below:.2f} m down')
                self.depth = below
                return True
            return False
        self.dive_wait_t = None
        want = min(max(0.0, below + alt - self.pool_alt), self.max_depth)
        if not self.dive_logged:
            self.dive_logged = True
            self.get_logger().info(f'dive: the DVL reads {alt:.2f} m to the bottom - ' + (
                f'down to {self.pool_alt:.2f} m above it, about {want:.2f} m below the start' if want > 0.05
                else f'already within {self.pool_alt:.2f} m of it: searching from here'))
        self.depth = self.ramp(self.depth, want, ALT_DZ_SPEED)
        if abs(self.depth - want) < 0.05 and abs(below - self.depth) < 0.1:
            if want >= self.max_depth - 0.01 and alt > self.pool_alt + 0.15:
                self.get_logger().warn(f'dive: at the depth limit ({self.max_depth:.1f} m below the start), still '
                                       f'{alt:.2f} m off the bottom - searching from here')
            else:
                self.get_logger().info(f'dive: {alt:.2f} m above the bottom at {below:.2f} m down')
            return True
        return False

    def declare_extra_parameters(self):
        # part:=search|follow|survey|repair|full (tools/task2.sh): SETTINGS, FIXED and the part's
        # switches become the defaults; a -p on the command line still wins. No part: the defaults below
        self.part = self.declare_parameter('part', '').value.strip().lower()
        if self.part and self.part not in PARTS:
            raise ValueError(f"part must be one of {', '.join(PARTS)}, not {self.part!r}")
        preset = {**SETTINGS, **FIXED, **PARTS[self.part][1]} if self.part else {}
        p = lambda name, default: self.declare_parameter(name, preset.get(name, default))
        p('map_url', 'http://127.0.0.1:8095')
        p('far_depth', 3.0)                 # Ruth: 3 m at the buoy
        p('far_range', 15.0)                # m, the map's range on the far scan
        p('far_link_m', 1.5)                # far crumbs this close count as one bunch
        p('close_height', 2.5)              # m, sonar above the pipe's top (Ruth: within 3 m)
        p('pipe_max_height', 2.05)          # m, floor to the top of a light box: handbook
                                            # "1-2 m", 3.5.5 parts list ~1.9 m, box 0.15 m
        p('close_range', 6.0)
        p('max_depth', 10.0)                # m below the surface; no target ever goes deeper
        p('scan_sector', '180 0')           # bottom half: left, down, right
        p('far_threshold', 90)              # the pipe 7 m down is faint (sim: ~100)
        p('floor_cut', 1.0)                 # m: the handbook puts the pipe 1-2 m up, so no
                                            # crumbs from the metre above the floor
        p('scan_threshold', 100)              # the map's own default
        p('scan_turn_deg', 360.0)
        p('scan_rate_dps', 3.0)             # Ruth, Oct 15 (the sub turns ~1 deg/s slower than asked)
        p('min_score', 0.5)
        p('phase_timeout', 360.0)
        p('plan', 'harbor')                 # or 'pool': one sideways scan, then approach
        p('pool_depth', 0.3)                # m below the surface (the pool is 0.9 m)
        p('pool_max_depth', 0.6)            # m; no pool target ever goes deeper
        p('pool_alt_m', 0.0)                # > 0 (Ruth, Oct 24): dive until the DVL reads this far to the
                                            # bottom; then pool_depth and pool_max_depth are not used
        p('pool_range', 6.0)
        p('pool_sector', '140 210')         # Ruth, Oct 14: the left side, a little up and down
        p('pool_side', 'left')              # where the sonar looks: 'left' or 'right'
        p('pool_pick_any', False)           # pick a shape on any side, not only pool_side's
                                            # (Ruth, Oct 15: look everywhere); face still puts it there
        p('pool_band', '-0.3 0.4')          # m from the sonar; the pipe is about level with it.
                                            # 'off' = every height
        p('floor_filter', '0.75 2.25')      # m ABOVE THE FLOOR LINE the sonar finds each sweep (floorline.py):
                                            # only echoes standing out from their beam in this band make
                                            # dots, in the scan and while following. Ruth: the comp pipe is
                                            # ~1-2 m off the floor; the pool pipe needs '0.15 0.6'. 'off' =
                                            # the old threshold + band + little dots. Its floor needs
                                            # downward beams (> 12 deg down) in the sector
        p('pool_compact', True)             # little dots only: drops the pool wall (Ruth, Oct 14:
                                            # the tall columns 1.2-1.8 m left were the wall)
        p('pool_near_m', 0.0)               # no crumbs closer than this, m (0 = the blind zone
                                            # only); 1.05 skips the ringing if the pipe is farther
        p('pool_far_m', 2.5)                # no crumbs farther than this (0 = no limit): in the pool
                                            # the wrong dots were mostly 3-5.6 m out (Oct 14)
        p('scan_mode', 'turn')              # 'turn': one slow turn; 'stops': stop and look
                                            # (stops shelved Oct 15: the sub slid at each heading jump);
                                            # 'search': stops, and a closer look where something shows (Oct 23)
        p('scan_time_s', 300.0)             # pool: give up searching (turns + picks) after this
        p('retry_turn_deg', 180.0)          # pool: nothing picked -> turn this much more and look again
        p('scan_stops', 20)                 # stops: this many headings round scan_turn_deg
        p('sweeps_per_stop', 3)             # stops, search: full sweeps taken still at each (1-5)
        p('search_step_deg', 25.0)          # search: a stop at least every this many degrees round scan_turn_deg
        p('blob_m', 0.15)                   # ... promising: dots this close together, from at least
        p('blob_sweeps', 2)                 # this many of the stop's sweeps
        p('look_span_deg', 20.0)            # ... then a closer look: stops out to this far either side,
        p('look_step_deg', 10.0)            # this far apart
        p('look_min_score', 0.2)            # ... passed: a shape this pipe-like near the promising bunch
        p('stop_tol_deg', 3.0)              # a stop looks once the heading is within this for 2 s (and, at
                                            # 3 or less, turning slower than 2 deg/s)
        p('record_scan', True)              # record the scan (Map tab Record), a Mark at each stop
        p('pool_max_pick_m', 2.5)           # pick only shapes this close to the sonar: start
                                            # ~1.5 m from the pipe, so walls farther out are ignored
        p('pool_floor_cut', 0.25)           # the pipe hangs ~0.45 m off the pool floor
        p('approach_range', 1.2)            # m, sonar to pipe at the stop: past the 0.75 m blind
                                            # zone AND its ringing (to ~1.05 m), where the pipe hid
        p('approach_speed', 0.15)           # m/s
        p('hold_s', 15.0)
        p('stop_after', '')                 # '' = the whole plan; or scan, pick, face, approach, over
        # ---- after the pick: ride over the pipe and follow it (Ruth, Oct 15) ----
        p('pool_after_pick', 'follow')      # 'follow': over the pipe, along it; 'approach': stop beside it
        p('follow_sector', '240 300')       # sonar looks straight down +-30 deg: one cross-section per sweep
        p('follow_range', 2.5)              # m; short range, quicker pings
        p('follow_band', '0.12 0.65')       # m ABOVE THE FLOOR (DVL altitude): pipe top ~0.3, box top ~0.45
        p('follow_near_m', 0.3)             # look this close while following (0 = the 0.75 m blind zone):
                                            # the pool pipe sat 0.5-0.7 m under the sonar (Oct 16)
        p('follow_band_from', 'floor')      # 'sonar': follow_band is m from the sonar, up + (Ruth, Oct 16:
                                            # the pool pipe sits 0.6-0.85 m under the sonar wherever the floor is)
        p('sonar_above_dvl_m', 0.3)         # the floor shows at DVL altitude + this (every pool session)
        p('follow_speed', 0.08)             # m/s at most
        p('follow_step_m', 0.3)             # ... and at most this far per sonar sweep
        p('follow_yaw_dps', 5.0)            # turn no faster (fast turns made GUIDED slide, Oct 15)
        p('follow_fit_m', 1.2)              # line fit: pipe dots this far behind the sonar
        p('follow_gate_m', 0.3)             # once seen, only echoes this far (sideways) from the pipe's
                                            # line count as the pipe (0 = off): not the wall (Oct 17)
        p('follow_gate_up_m', 0.2)          # ... and this far above or below the pipe's height
        p('follow_threshold', 80)           # echo strength that makes a dot while following. 100 (the
                                            # scan's) lost a tilted segment after an elbow: replayed, the map
                                            # made 0 dots there at 100 and 2-6 a sweep at 80 (pool, Oct 17)
        p('follow_start_gate_m', 0.25)      # before the first sighting: only this far to the side
                                            # (line the sub up with the pipe straight below)
        p('follow_lost_s', 30.0)            # no pipe under the sonar this long: lost or ended
        p('follow_searches', 2)             # lost mid-follow: search for it from there (Ruth, Oct 23); this many
                                            # searches at one spot that don't get past it: its end (0 = no search)
        p('align_walk_m', 0.6)              # over the pipe before following: walk at most this far straight along
                                            # it looking down, then turn by how far it drifted sideways (Ruth,
                                            # Oct 23; 0 = off) ...
        p('align_turn_m', 0.4)              # ... turning as soon as the pipe is this far off the sonar's middle
                                            # (Ruth, Oct 25), before it can leave the +-30 deg view
        p('follow_pipe_below_m', 0.7)       # while following, change depth to keep the pipe this far under the
        p('follow_pipe_tol_m', 0.15)        # sonar, give or take this (Ruth, Oct 24: the comp pipe isn't level;
                                            # too high loses it, too low bumps it). 0 = hold the depth
        p('follow_max_m', 8.0)              # stop after following this far
        p('follow_time_s', 300.0)           # ... or after this long (under phase_timeout)
        p('follow_both_ways', True)         # at one end, follow the rest of the pipe the other way
        p('follow_turn', 'return')          # 'return': back to where following began, turn 180, back up
                                            # follow_backup_m, follow the other part (Ruth, Oct 16);
                                            # 'spot': turn round at the end and follow it all back
        p('follow_backup_m', 0.85)          # the sonar is 0.42 m ahead of the centre, so turning 180
                                            # moves it 0.84 m: back up that much and none is missed
        p('follow_return_speed', 0.15)      # m/s on the way back (blind, DVL only)
        p('camera_boxes', True)             # count LED boxes in the bottom camera while following (log only)
        p('camera_url', 'http://127.0.0.1:8080/stream')   # tools/cam_pair.py; DOWN is the left half
        p('camera_fwd_m', 0.22)             # camera ahead of the sub's centre (sonar 0.42, camera 0.2 behind it)
        p('boxes_expected', 4)              # the handbook's light boxes
        p('follow_toward', -1.0)            # deg: start along the pipe toward this heading; -1 = its longer part
        p('follow_end', 'poshold')          # 'poshold': hand over armed in POSHOLD when the follow ends (Ruth,
                                            # Oct 16: GUIDED drifted into the wall at the end); 'surface': disarm
        # ---- after the scan: look again from over the likely spots (Ruth, Oct 16) ----
        p('spot_check', True)               # pool_after_pick:=follow only
        p('spot_count', 3)                  # at most this many spots, best scoring first
        p('spot_min_score', 0.2)            # worth a look (a pick without the check needs min_score)
        p('spot_turn_deg', 30.0)            # turn half each way over a spot, then back
        p('spot_rate_dps', 2.0)
        p('spot_radius_m', 0.3)             # pipe dots this close to the spot count for it
        p('spot_min_frac', 0.3)             # follow only from a spot that showed the pipe in this share of sweeps
        p('start_with', 'scan')             # 'follow': no scan - the pilot leaves the sub over the pipe,
                                            # facing along it, and it follows from there (Ruth, Oct 16)
        # ---- survey the pipeline's lights (Ruth, Oct 17; handbook Task 2) ----
        p('follow_mode', 'follow')          # 'survey': along the pipe to the end nearest the green buoy,
                                            # turn round, then follow it to the far end noting each light
                                            # GREEN or RED in order, stopping over the RED one on the way;
                                            # 'mark_buoy': save where the sub is now as the green buoy, then
                                            # quit - sends nothing to the vehicle
        p('survey_buoy', 'start')           # the origin is the green buoy, no compass or GPS (Ruth, Oct 17):
                                            # 'start' = where this mission started (begin it at the buoy);
                                            # 'marked' = where follow_mode:=mark_buoy saved it; 'here' = the
                                            # sub is already at the buoy's end of the pipe, facing along it
        p('survey_buoy_file', 'sonar_data/green_buoy.json')   # mark_buoy writes it, 'marked' reads it
        p('repair_hold_s', 0.0)             # > 0: go back over the RED light and hold this long (stands in
                                            # for the magnet repair); 0: just note it and go on (Ruth, Oct 17:
                                            # the magnet isn't on the sub yet)
        p('repair_centre', False)           # True: at the RED light, centre it on the magnet in the down camera
                                            # and step down until it turns GREEN, then watch (Ruth, Oct 22)
        p('magnet_px', f'{led_boxes.MAGNET_PX[0]:.0f} {led_boxes.MAGNET_PX[1]:.0f}')   # the magnet in the
                                            # 640x360 down picture: lined up there, the magnet is over the light
        p('centre_tol_px', 20.0)            # lined up: the light's middle this close to the magnet's (the median
        p('centre_settle_s', 1.5)           # over this long), then one step down. Oct 22: the light switched
                                            # with its middle 16-34 px from the magnet's
        p('centre_step_m', 0.1)             # each step down (Ruth, Oct 24) ...
        p('centre_first_step_m', 0.4)       # ... but this much while the magnet is still more than this + 0.1 m
                                            # above the light (from the sonar's pipe height)
        p('centre_max_down_m', 0.6)         # never more than this below where centring began ...
        p('centre_min_alt_m', 0.3)          # ... nor closer than this to the floor (DVL)
        p('centre_speed', 0.03)             # m/s while centring
        p('centre_time_s', 300.0)           # no GREEN by then: give up (start_with:=centre surfaces; the survey
                                            # goes on along the pipe at its own depth)
        p('light_above_floor_m', 0.45)      # the light's top above the floor and the down camera's focal
        p('cam_f_px', 330.0)                # length (full-size px, rough: Oct 22, from where the magnet shows
                                            # 15 cm under the lens) turn pixels into metres; only the speed of
                                            # the correction depends on them
        p('watch_rise_m', 0.05)             # once GREEN: rise this much and watch the light's colours this long
        p('watch_s', 40.0)                  # (handbook: GREEN 5 s, then 30 s of flashes; the colour-sequence
                                            # reader comes later - Ruth, Oct 22)
        p('light_same_m', 0.3)              # sightings this close together are one light
        p('light_min_share', 0.10)          # a new light must once be seen whole (clear of the picture's
                                            # edge) or fill this share of it: glimpses at the edge (5-8%)
                                            # sat in the wrong place and were counted twice (Oct 16-17)
        p('light_detector', 'hsv')          # 'hsv': a light is any pixel past the fixed HSV thresholds below;
                                            # 'contrast' (Oct 25): a round patch much brighter than the water
                                            # around it, its colour from its glow (led_boxes.contrast_lights)
        p('light_model', '')                # light_detector:=yolo: the trained model (.pt, ultralytics) ...
        p('light_conf', 0.4)                # ... and the confidence a box needs
        p('end_model', '')                  # a model with a pipe_end class used for the pipe's ends only, the
                                            # lights staying on light_detector (Ruth, Oct 25); '' = none
        p('end_conf', 0.25)                 # ... and the confidence a pipe end needs (either model)
        p('light_min_frac', 0.02)           # smallest blob that counts: 0.02 in the pool (camera close,
                                            # the glow breaks into pieces); ~0.0005 at the lake, where a
                                            # ring ~1 m below is 0.05-6% of the picture (Oct 18)
        p('light_v_min', led_boxes.V_MIN)   # HSV (Ruth, Oct 18): a light's pixel is at least this bright
        p('light_s_min', led_boxes.S_MIN)   # ... and this strongly coloured (0-255); hue ranges in led_boxes.py
        p('survey_backup_m', 0.6)           # at the buoy end, after turning 180: back up this far, so the
                                            # camera (0.2 m behind the sonar) starts at the end
        p('survey_dir', 'sonar_data')       # the survey report (JSON) is written here

    def read_extra_parameters(self):
        g = lambda n: self.get_parameter(n).value
        if self.part:
            self.get_logger().info(f'task2 {self.part}: {PARTS[self.part][0]}')
        self.map_url = g('map_url')
        self.depth = g('far_depth')         # MissionBase dives to self.depth
        self.gate_f = 0.0                   # no gate: hold where we dived
        self.far_range, self.far_link = g('far_range'), g('far_link_m')
        self.close_height, self.close_range = g('close_height'), g('close_range')
        self.pipe_max = g('pipe_max_height')
        self.max_depth = g('max_depth')
        self.scan_sector = g('scan_sector')
        self.scan_threshold = g('scan_threshold')
        self.far_threshold = g('far_threshold')
        self.floor_cut = g('floor_cut')
        self.scan_turn = math.radians(g('scan_turn_deg'))
        self.scan_rate = math.radians(g('scan_rate_dps'))
        self.min_score = g('min_score')
        self.timeout = g('phase_timeout')   # per phase: each one restarts the clock
        self.plan = g('plan')
        self.approach_range, self.approach_speed = g('approach_range'), g('approach_speed')
        self.hold_s = g('hold_s')
        self.stop_after = g('stop_after').strip().lower()
        if self.stop_after not in ('', 'scan', 'pick', 'face', 'approach', 'over', 'align', 'spots'):
            raise ValueError(f"stop_after must be scan, pick, face, approach, over, align or spots, not {self.stop_after!r}")
        self.after_pick = g('pool_after_pick').strip().lower()
        if self.after_pick not in ('follow', 'approach'):
            raise ValueError(f"pool_after_pick must be follow or approach, not {self.after_pick!r}")
        self.follow_sector, self.follow_range = g('follow_sector'), float(g('follow_range'))
        self.follow_band = tuple(float(v) for v in g('follow_band').split())
        self.follow_near = float(g('follow_near_m'))
        self.follow_band_from = g('follow_band_from').strip().lower()
        if self.follow_band_from not in ('floor', 'sonar'):
            raise ValueError(f"follow_band_from must be floor or sonar, not {self.follow_band_from!r}")
        self.sonar_above_dvl = float(g('sonar_above_dvl_m'))
        self.follow_speed, self.follow_step = float(g('follow_speed')), float(g('follow_step_m'))
        self.follow_yaw = math.radians(g('follow_yaw_dps'))
        self.follow_fit, self.follow_lost = float(g('follow_fit_m')), float(g('follow_lost_s'))
        self.follow_gate, self.follow_gate_up = float(g('follow_gate_m')), float(g('follow_gate_up_m'))
        self.follow_start_gate = float(g('follow_start_gate_m'))
        self.follow_searches, self.align_walk = max(0, int(g('follow_searches'))), max(0.0, float(g('align_walk_m')))
        self.align_turn = max(0.1, float(g('align_turn_m')))
        self.follow_below, self.follow_below_tol = max(0.0, float(g('follow_pipe_below_m'))), float(g('follow_pipe_tol_m'))
        self.follow_threshold = int(g('follow_threshold'))
        self.f_up, self.f_up_z = None, None
        self.follow_max, self.follow_time = float(g('follow_max_m')), float(g('follow_time_s'))
        self.follow_both, self.follow_toward = bool(g('follow_both_ways')), float(g('follow_toward'))
        self.follow_turn = g('follow_turn').strip().lower()
        self.follow_end = g('follow_end').strip().lower()
        if self.follow_end not in ('poshold', 'surface'):
            raise ValueError(f"follow_end must be poshold or surface, not {self.follow_end!r}")
        if self.follow_turn not in ('return', 'spot'):
            raise ValueError(f"follow_turn must be return or spot, not {self.follow_turn!r}")
        self.follow_backup, self.return_speed = float(g('follow_backup_m')), float(g('follow_return_speed'))
        self.camera_boxes, self.camera_url = bool(g('camera_boxes')), g('camera_url')
        self.camera_fwd, self.boxes_expected = float(g('camera_fwd_m')), int(g('boxes_expected'))
        self.camera = None
        self.spot_check, self.spot_count = bool(g('spot_check')), max(1, int(g('spot_count')))
        self.spot_min_score, self.spot_turn = float(g('spot_min_score')), float(g('spot_turn_deg'))
        self.spot_rate = math.radians(max(0.5, float(g('spot_rate_dps'))))
        self.spot_radius, self.spot_min_frac = float(g('spot_radius_m')), float(g('spot_min_frac'))
        self.start_with = g('start_with').strip().lower()
        if self.start_with not in ('scan', 'follow', 'centre'):
            raise ValueError(f"start_with must be scan, follow or centre, not {self.start_with!r}")
        mode = g('follow_mode').strip().lower()
        if mode not in ('follow', 'survey', 'mark_buoy'):
            raise ValueError(f"follow_mode must be follow, survey or mark_buoy, not {mode!r}")
        self.survey, self.mark_buoy = mode == 'survey', mode == 'mark_buoy'
        self.survey_buoy = g('survey_buoy').strip().lower()
        if self.survey_buoy not in ('start', 'marked', 'here'):
            raise ValueError(f"survey_buoy must be start, marked or here, not {self.survey_buoy!r}")
        self.buoy_file = g('survey_buoy_file')
        self.marked_buoy = None
        if self.survey and self.survey_buoy == 'marked':      # check it now, not once armed and over the pipe
            try:
                with open(self.buoy_file) as f:
                    j = json.load(f)
                self.marked_buoy = (float(j['n']), float(j['e']))
            except (OSError, ValueError, KeyError) as ex:
                raise ValueError(f'survey_buoy:=marked but no green buoy saved in {self.buoy_file} ({ex}): '
                                 f'run follow_mode:=mark_buoy with the sub at the buoy first')
        self.repair_hold, self.light_same = float(g('repair_hold_s')), float(g('light_same_m'))
        self.repair_centre = bool(g('repair_centre'))
        mx = str(g('magnet_px')).split()
        if len(mx) != 2:
            raise ValueError(f"magnet_px must be 'x y' pixels, not {g('magnet_px')!r}")
        self.magnet = (float(mx[0]), float(mx[1]))
        self.centre_tol, self.centre_settle = float(g('centre_tol_px')), float(g('centre_settle_s'))
        self.centre_step_m, self.centre_max_down = float(g('centre_step_m')), float(g('centre_max_down_m'))
        self.centre_first_step = float(g('centre_first_step_m'))
        self.centre_min_alt, self.centre_speed = float(g('centre_min_alt_m')), float(g('centre_speed'))
        self.centre_time, self.light_above_floor = float(g('centre_time_s')), float(g('light_above_floor_m'))
        self.cam_f = float(g('cam_f_px'))
        self.watch_rise, self.watch_s = float(g('watch_rise_m')), float(g('watch_s'))
        self.light_min_share = float(g('light_min_share'))
        self.light_min_frac = float(g('light_min_frac'))
        self.light_detector = g('light_detector').strip().lower()
        if self.light_detector not in ('hsv', 'contrast', 'yolo'):
            raise ValueError(f"light_detector must be hsv, contrast or yolo, not {self.light_detector!r}")
        self.light_model, self.light_conf = g('light_model').strip(), float(g('light_conf'))
        self.end_model, self.end_conf = g('end_model').strip(), float(g('end_conf'))
        self.yolo = None                    # the loaded model (light_detector:=yolo, or end_model), once needed
        self.yolo_failed = False
        led_boxes.V_MIN, led_boxes.S_MIN = int(g('light_v_min')), int(g('light_s_min'))
        self.survey_backup = float(g('survey_backup_m'))
        self.survey_dir = g('survey_dir')
        self.s_leg = None                   # survey: 'to buoy', then 'survey'
        # set again by survey_begin; here too, as start_with:=centre's camera notes sightings without a
        # survey (Oct 25 pool: a red sighting stopped the camera thread on a missing s_red)
        self.s_red, self.s_red_new, self.s_repair_mono, self.s_track = None, False, None, []
        self.side = 1.0 if g('pool_side').strip().lower() == 'right' else -1.0
        self.pool_pick_any = bool(g('pool_pick_any'))
        ff = g('floor_filter').strip().lower()
        self.floor_filter = None if ff in ('', 'off', 'none') else tuple(float(v) for v in ff.split()[:2])
        if self.floor_filter is not None and not 0 <= self.floor_filter[0] < self.floor_filter[1] <= 10:
            raise ValueError(f"floor_filter must be 'lo hi' m above the floor (0-10) or off, not {ff!r}")
        band = g('pool_band').strip().lower()
        self.pool_band = 'off' if band in ('', 'off', 'none') else band
        self.pool_compact = bool(g('pool_compact'))
        self.pool_near = float(g('pool_near_m'))
        self.pool_far = float(g('pool_far_m'))
        self.scan_mode = g('scan_mode').strip().lower()
        self.scan_time = float(g('scan_time_s'))
        self.retry_turn = math.radians(g('retry_turn_deg'))
        if self.scan_mode not in ('turn', 'stops', 'search'):
            raise ValueError(f"scan_mode must be turn, stops or search, not {self.scan_mode!r}")
        self.scan_stops = max(1, int(g('scan_stops')))
        self.sweeps_per_stop = min(5, max(1, int(g('sweeps_per_stop'))))
        self.search_step = math.radians(max(5.0, float(g('search_step_deg'))))
        self.blob_m, self.blob_sweeps = float(g('blob_m')), max(1, int(g('blob_sweeps')))
        self.look_span = math.radians(max(0.0, float(g('look_span_deg'))))
        self.look_step = math.radians(max(2.0, float(g('look_step_deg'))))
        self.look_min_score = float(g('look_min_score'))
        self.stop_tol = math.radians(max(1.0, float(g('stop_tol_deg'))))
        self.record_scan = bool(g('record_scan'))
        self.pool_max_pick = float(g('pool_max_pick_m'))
        self.pool_alt = max(0.0, float(g('pool_alt_m'))) if self.plan == 'pool' else 0.0
        if self.plan == 'pool':
            self.depth = g('pool_depth')
            self.max_depth = g('pool_max_depth')
        if self.pool_alt > 0:
            self.depth, self.max_depth = 0.0, ALT_MAX_DEPTH_M     # the dive sets the depth
            self.guard_alt = min(SEARCH_MIN_ALT_M, self.pool_alt - 0.5)
            if self.guard_alt < SEARCH_MIN_ALT_M:
                self.get_logger().warn(f'pool_alt_m {self.pool_alt:.2f}: the search keeps only {self.guard_alt:.2f} m '
                                       f'off the bottom (not {SEARCH_MIN_ALT_M:.1f})')
        self.scans = {
            'far': (self.far_range, self.scan_sector, self.far_threshold, self.floor_cut),
            'close': (self.close_range, self.scan_sector, self.scan_threshold, self.floor_cut),
            'pool': (g('pool_range'), g('pool_sector'), self.scan_threshold, g('pool_floor_cut'))}

    # ---------- depth from the surface ----------
    def enter(self, s):
        if s is S.ARM and self.mark_buoy:
            # follow_mode:=mark_buoy: the sub is at the green buoy - save where, and quit before
            # anything (mode, arm) is sent to the vehicle (Ruth, Oct 17: the buoy is the origin)
            mark = {'n': round(self.start_x, 3), 'e': round(self.start_y, 3),
                    'yaw_deg': round(math.degrees(self.start_yaw) % 360, 1), 'saved': time.strftime('%Y-%m-%d %H:%M:%S')}
            try:
                os.makedirs(os.path.dirname(self.buoy_file) or '.', exist_ok=True)
                with open(self.buoy_file, 'w') as f:
                    json.dump(mark, f)
                self.get_logger().info(f'green buoy saved: the sub is here, NED ({mark["n"]:.2f}, {mark["e"]:.2f}), '
                                       f'in {self.buoy_file} - use -p survey_buoy:=marked')
            except OSError as ex:
                self.get_logger().error(f'green buoy NOT saved to {self.buoy_file}: {ex}')
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)
        if s is S.ARM:
            self.z_surf = self.cur[2] if self.cur else 0.0
            self.get_logger().info(f'surface reads {self.z_surf:.2f} m; depths are from there')
        if s is S.SURFACE and self.plan == 'pool' and self.cur:
            self.surface_at = (self.cur[0], self.cur[1], self.cur_yaw or 0.0)   # up where it is
            self.post_point(None)
        if s is S.SURFACE:
            self.stop_camera()                  # the box summary, however the run ends
        if s is S.SURFACE and self.recording:    # an early end (timeout, no map) left it running
            try:
                self.map_post('/record', 'stop')
                self.recording = False
            except OSError:
                pass
        super().enter(s)

    def goto(self, forward, right, down, label):
        """MissionBase's dive, gate and surface targets, from the surface."""
        if self.state == S.SURFACE and self.surface_at is not None:
            n, e, yaw = self.surface_at
            self.goto_ned(n, e, self.z_surf + down, yaw, label)
            return
        super().goto(forward, right, self.z_surf + down, label)

    def below_surface(self, z):
        return z - self.z_surf

    # Every depth here is the Cube's own z. The MissionBase on Graey's branch
    # (tether_toggle_beichen) adds its start_z to each target, so it is taken off
    # first; the older MissionBase has no start_z.
    def goto_ned(self, n, e, down, yaw, label):
        down = min(down, self.z_surf + self.max_depth)
        super().goto_ned(n, e, down - getattr(self, 'start_z', 0.0), yaw, label)

    def stream_posvel(self, n, e, down, vn, ve, yaw_rate, label):
        down = min(down, self.z_surf + self.max_depth)
        super().stream_posvel(n, e, down - getattr(self, 'start_z', 0.0), vn, ve, yaw_rate, label)

    # ---------- the sonar map, through its web page ----------
    def map_post(self, path, body):
        req = urllib.request.Request(self.map_url + path, data=str(body).encode())
        return urllib.request.urlopen(req, timeout=1.0).read().decode()

    def map_get(self):
        return json.loads(urllib.request.urlopen(self.map_url + '/mapdata', timeout=1.0).read())

    def mark(self, msg):
        """A Mark in the recording, if one runs; a dropped one is let go (the step goes on)."""
        if self.recording:
            try:
                self.map_post('/mark', msg)
            except OSError:
                pass

    def post_point(self, xy):
        """Show where the mission is heading on the Map tab (None clears it)."""
        try:
            self.map_post('/missionpoint', 'none' if xy is None else f'{xy[0]:.3f} {xy[1]:.3f}')
        except OSError:
            pass

    def map_to_ned(self, x, y):
        n0, e0, yaw0 = self.map_origin
        return (n0 + y * math.cos(yaw0) - x * math.sin(yaw0),
                e0 + y * math.sin(yaw0) + x * math.cos(yaw0))

    # ---------- phases ----------
    def to_phase(self, name, note=''):
        self.get_logger().info(f'    phase: {name}' + (f' - {note}' if note else ''))   # note: only printed
        self.phase = name
        self.state_t0 = self.now()

    def hold_here(self, down, label):
        self.target = None                  # a new hold is always sent
        self.hold = (self.cur[0], self.cur[1], down, self.cur_yaw) if self.cur else (
            self.start_x, self.start_y, down, self.start_yaw)
        self.goto_ned(*self.hold, label)

    def start_scan(self, which):
        self.scan = which
        self.to_phase(f'{which} scan: map setup')

    def do_to_marker(self):
        """The map can drop a request now and then; ten seconds of nothing ends it."""
        try:
            self.search()
            self.map_down_since = None
        except OSError as ex:
            if self.map_down_since is None:
                self.map_down_since = self.now()
            self.get_logger().warn(f'sonar map at {self.map_url}: {ex}')
            if self.now() - self.map_down_since > 10.0:
                self.get_logger().error('no sonar map for 10 s - surfacing')
                self.enter(S.SURFACE)

    def search(self):
        if self.phase is None:
            self.hold_here(self.z_surf + self.depth, 'hold')
            if self.start_with == 'centre':
                self.centre_begin('start_with:=centre - no scan, no follow: centring the light under the camera')
                return
            if self.plan == 'pool' and self.start_with == 'follow':
                self.start_follow_here()
                return
            self.start_scan('pool' if self.plan == 'pool' else 'far')
        ph = self.phase.split(': ')[-1]
        if self.pool_alt > 0:
            self.alt_guard(ph)

        if ph == 'map setup':
            # range first: changing it restarts the sweep
            rng, sector, threshold, cut = self.scans[self.scan]
            self.map_post('/range', rng)
            self.map_post('/sector', sector)
            self.map_post('/threshold', threshold)
            self.map_post('/floorcut', cut)
            pool = self.scan == 'pool'
            self.map_post('/compact', 'on' if pool and self.pool_compact else 'off')
            self.map_post('/near', str(self.pool_near) if pool else 'off')
            self.map_post('/far', str(self.pool_far) if pool else 'off')
            if pool:        # keep every crumb of the turn: with fade on the turn wiped 97% of them (sim)
                self.map_post('/fade', 'off')
            self.map_post('/band', self.pool_band if pool else 'off')
            self.post_floor_filter()
            for cmd in ('reset', 'start', 'resume'):    # whatever state it was in
                self.map_post('/mapcmd', cmd)
            if self.record_scan and not self.recording:
                self.map_post('/record', f'start {self.scan} scan {self.scan_mode}' +
                              (f' {self.scan_stops}x{self.sweeps_per_stop}' if self.scan_mode == 'stops' else ''))
                self.recording = True
            elif self.recording:            # a search mid-follow, or back to one: the same recording
                self.map_post('/mark', 'search: the side view again' +
                              (' (the pipe was lost)' if self.srch and self.srch['kind'] == 'lost' else ''))
            self.to_phase(f'{self.scan} scan: map start')

        elif ph == 'map start':
            self.goto_ned(*self.hold, 'hold')
            m = self.map_get()
            if m['state'] == 'running' and m['pose'] and math.hypot(*m['pose'][:2]) < 0.05:
                if self.dry or self.cur is None:
                    self.map_origin = (self.hold[0], self.hold[1], self.hold[3])
                else:
                    self.map_origin = (self.cur[0], self.cur[1], self.cur_yaw)
                self.turned, self.last_yaw = 0.0, self.cur_yaw
                self.scan_t0, self.turn_goal, self.turn_no = self.now(), self.scan_turn, 1
                if self.scan == 'pool' and (self.scan_mode == 'search' or self.srch is not None):
                    self.search_start()
                elif self.scan_mode == 'stops':
                    self.stop_i, self.stop_yaw0 = 0, (self.cur_yaw if self.cur_yaw is not None else self.hold[3])
                    self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())
                else:
                    self.to_phase(f'{self.scan} scan: turn')

        elif ph == 'stop turn':
            # stop i looks at heading yaw0 + i * scan_turn / scan_stops (the first one where it is)
            want = self.stop_want()
            n, e, down, _ = self.hold
            self.goto_ned(n, e, down, want, self.stop_label())
            if self.srch is not None:
                self.stop_collect(self.map_get(), keep=False)   # dots that land while turning belong to no stop
            if self.dry and self.now() - self.state_t0 < 0.3:
                self.get_logger().info(f'    dry run: turn the sub to heading {math.degrees(want) % 360:.0f} deg '
                                       f'({self.stop_label()}) and hold it still')
            err = None if self.cur_yaw is None else abs(wrap_pi(self.cur_yaw - want))
            # wider than 3 deg (Oct 25 pool: the heading hold swung +-12 deg with the gyro past 30 deg/s):
            # the heading alone counts - with the turning check, a stop waited up to 3 minutes
            still = (err is not None and err < self.stop_tol
                     and (self.stop_tol > math.radians(3) or abs(self.yaw_rate or 0.0) < math.radians(2)))
            if not still:
                self.still_t = None
            elif self.still_t is None:
                self.still_t = self.now()
            elif self.now() - self.still_t >= 2.0:
                self.look_from, self.stop_dots = self.map_get().get('sweep', 0), []
                if self.record_scan:
                    self.map_post('/mark', f'{self.stop_label()} at heading '
                                           f'{math.degrees(self.cur_yaw) % 360:.0f}: looking {self.sweeps_per_stop} sweeps')
                self.to_phase(f'{self.scan} scan: stop look', self.stop_label())

        elif ph == 'stop look':
            # full sweeps while still: the one in progress started before the stop, so it doesn't count
            n, e, down, _ = self.hold
            self.goto_ned(n, e, down, self.stop_want(), self.stop_label())
            m = self.map_get()
            if self.srch is not None:
                self.stop_collect(m)
            if m.get('sweep', 0) >= self.look_from + self.sweeps_per_stop + 1:
                self.still_t = None
                if self.srch is not None:
                    self.search_after_stop(m)
                    return
                self.stop_i += 1
                if self.stop_i >= self.scan_stops:
                    self.hold_here(self.hold[2], 'hold')
                    self.to_phase(f'{self.scan} scan: settle')
                else:
                    self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())

        elif ph == 'turn':
            n, e, down, _ = self.hold
            self.stream_posvel(n, e, down, 0.0, 0.0, self.scan_rate, f'{self.scan} scan turn')
            if self.cur_yaw is not None and self.last_yaw is not None:
                before = abs(self.turned)
                self.turned += wrap_pi(self.cur_yaw - self.last_yaw)
                if int(abs(self.turned) // math.radians(45)) > int(before // math.radians(45)):
                    self.get_logger().info(f'    turned {math.degrees(abs(self.turned)):.0f} of '
                                           f'{math.degrees(self.turn_goal):.0f} deg'
                                           + (' (dry run: the pilot turns the sub)' if self.dry else ''))
            self.last_yaw = self.cur_yaw
            # a dry run sends nothing, so it waits for the pilot to turn the sub; with
            # no attitude at all (no Cube) it stops waiting after 5 s
            no_yaw = self.dry and self.cur_yaw is None and self.now() - self.state_t0 > 5
            out_of_time = self.scan == 'pool' and self.now() - self.scan_t0 >= self.scan_time
            if out_of_time:
                self.get_logger().warn(f'    search time {self.scan_time:.0f} s used up mid-turn - last look')
            if abs(self.turned) >= self.turn_goal or no_yaw or out_of_time:
                self.hold_here(self.hold[2], 'hold')
                self.to_phase(f'{self.scan} scan: settle')

        elif ph == 'settle':
            self.goto_ned(*self.hold, 'hold')
            if self.now() - self.state_t0 >= SETTLE_S:
                if self.stop_after == 'scan':
                    self.stop_here('scan')
                    return
                {'far': self.locate, 'close': self.pick, 'pool': self.pick_target}[self.scan](self.map_get())

        elif ph == 'face':
            if self.stop_after == 'pick':
                self.stop_here('pick')
                return
            self.goto_ned(self.hold[0], self.hold[1], self.hold[2], self.approach_yaw, 'face the pipe')
            if self.dry or (self.cur_yaw is not None
                            and abs(wrap_pi(self.cur_yaw - self.approach_yaw)) < math.radians(5)):
                if self.stop_after == 'face':
                    self.stop_here('face')
                    return
                self.carrot = (self.cur[0], self.cur[1]) if self.cur else self.hold[:2]
                self.last_t = self.now()
                self.to_phase('approach')

        elif ph == 'approach':
            self.approach(self.map_get())

        elif ph == 'to pipe':
            if self.stop_after == 'pick':
                self.stop_here('pick')
                return
            self.go_over()

        elif ph == 'spot anchor':
            self.spot_anchor()

        elif ph == 'spot look':
            self.spot_look(self.map_get())

        elif ph == 'search back':
            self.search_back_step()

        elif ph == 'over check':
            self.over_check_step(self.map_get())

        elif ph == 'down to pipe':
            self.walk_down_step()

        elif ph == 'anchor':
            self.anchor_step()

        elif ph == 'align walk':
            self.align_walk_step(self.map_get())

        elif ph == 'follow start':
            self.follow_anchor()

        elif ph == 'follow':
            self.follow_step_once(self.map_get())

        elif ph == 'follow return':
            self.follow_return()

        elif ph == 'follow turn':
            self.follow_turn_round()

        elif ph == 'follow backup':
            self.follow_back_up()

        elif ph == 'survey turn':
            self.survey_turn_first()

        elif ph == 'repair go':
            self.repair_go()

        elif ph == 'repair hold':
            self.repair_hold_step()

        elif ph == 'centre':
            self.centre_step()

        elif ph == 'watch':
            self.watch_step()

        elif ph == 'poshold':
            if self.now() - self.poshold_t > 1.0:     # a dropped command: ask again
                self.link.set_mode(MODE_POSHOLD)
                self.poshold_t = self.now()

        elif ph == 'stopped':
            self.goto_ned(*self.hold, 'hold')
            if self.now() - self.state_t0 >= 5.0:
                self.enter(S.SURFACE)

        elif ph == 'at pipe':
            self.goto_ned(*self.hold, 'hold at the pipe')
            if self.now() - self.state_t0 >= self.hold_s:
                self.enter(S.SURFACE)

        elif ph == 'to spot':
            n, e, _ = self.spot
            self.goto_ned(n, e, self.hold[2], self.hold[3], 'over the bunch')
            if self.cur and math.hypot(self.cur[0] - n, self.cur[1] - e) < 0.3 and self.speed() < 0.05:
                self.hold_here(self.hold[2], 'hold')
                self.step_z = self.hold[2]
                self.map_post('/sector', DESCEND_SECTOR)        # a narrow look straight down
                self.map_post('/mapcmd', 'reset')
                self.to_phase('descend')

        elif ph == 'descend':
            n, e, floor_z = self.spot
            limit = floor_z - self.pipe_max - self.close_height
            alt = self.floor_clearance()
            if alt is not None and self.cur:
                limit = min(limit, self.cur[2] + alt - self.pipe_max - self.close_height)
            below = self.pipe_below(self.map_get())
            if below is not None:
                want = min(self.cur[2] + below - self.close_height, limit)
                if abs(below - self.close_height) < 0.15:
                    self.get_logger().info(f'pipe top {below:.2f} m below at '
                                           f'{self.below_surface(self.cur[2]):.2f} m down - holding here')
                    self.hold_here(self.cur[2], 'hold')
                    self.start_scan('close')
                    return
                self.step_z = want
            elif self.cur and abs(self.cur[2] - self.step_z) < 0.1:
                if self.step_z >= limit - 0.05:
                    self.get_logger().warn(f'floor limit {self.below_surface(limit):.1f} m down, '
                                           f'no pipe seen below - scanning from here')
                    self.hold_here(self.cur[2], 'hold')
                    self.start_scan('close')
                    return
                self.step_z = min(self.step_z + 0.5, limit)       # nothing seen yet: a bit lower
            self.goto_ned(n, e, self.step_z, self.hold[3], 'descend')

    def stop_here(self, stage):
        """stop_after: hold where the sub is for 5 s, then surface."""
        self.get_logger().info(f'stop_after:={stage} - holding 5 s, then surfacing')
        if self.cur and not self.dry:
            self.hold_here(self.cur[2], 'hold')
        self.to_phase('stopped')

    def pipe_below(self, m):
        """How far below the sonar the highest raised echo near straight down is
        (pipe or light box top; the map has already cut the floor), or None."""
        if not m.get('crumbs') or not m.get('sonar'):
            return None
        c = np.array(m['crumbs']).reshape(-1, 5)
        sx, sy = m['sonar']
        near = np.hypot(c[:, 0] - sx, c[:, 1] - sy) < BELOW_RADIUS_M
        if near.sum() < 2:
            return None
        return float(-np.max(c[near, 4]))

    def locate(self, m):
        """Far scan: the biggest bunch of crumbs is where the pipe is - a rough
        "where" is all this needs."""
        c = np.array(m.get('crumbs') or [], float).reshape(-1, 5)[:, :2]
        groups = outlines.find(c[:, 0], c[:, 1], None, link_m=self.far_link, min_crumbs=4)
        if not groups:
            self.get_logger().warn(f'far scan: {len(c)} crumbs, no bunch of 4 - surfacing')
            self.enter(S.SURFACE)
            return
        g = max(groups, key=lambda gr: gr['n'])
        pts = c[g['members']]
        n, e = self.map_to_ned(float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        alt = 9.0 if self.dry else self.floor_clearance()
        if alt is None:
            self.get_logger().warn('far scan: no DVL altitude, so no safe depth to go down to - surfacing')
            self.enter(S.SURFACE)
            return
        floor = self.hold[2] + alt
        self.spot = (n, e, floor)
        self.get_logger().info(
            f'far scan: {len(c)} crumbs, biggest bunch {g["n"]} at NED ({n:.1f}, {e:.1f}); floor '
            f'{self.below_surface(floor):.1f} m down, so the pipe top is at most '
            f'{self.below_surface(floor) - self.pipe_max:.1f} m down')
        self.to_phase('to spot')

    def pick(self, m):
        """Close scan: the most pipe-like outline, ends in NED, near end first."""
        found = m.get('outlines') or []
        for i, g in enumerate(found[:5]):
            self.get_logger().info(
                f'    outline {i}: pipe {g["score"]:.0%} {g["parts"]} {g["n"]} crumbs, '
                f'{g["length"]:.1f} m long, {g["width"]:.2f} m wide, top {g["top"]} m')
        best = found[0] if found else None
        if best is None or best['score'] < self.min_score:
            self.get_logger().warn(f'close scan: no outline scores {self.min_score:.0%} - surfacing')
            self.enter(S.SURFACE)
            return
        spine = [self.map_to_ned(x, y) for x, y in best['spine']]
        if math.dist(spine[-1], (self.start_x, self.start_y)) < math.dist(spine[0], (self.start_x, self.start_y)):
            spine.reverse()                         # near end (nearest the buoy) first
        self.pipe = {'spine': spine, 'top': self.hold[2] - best['top'], 'score': best['score']}
        alt = self.floor_clearance()
        if alt is not None:
            self.pipe['floor'] = self.hold[2] + alt
        (n1, e1), (n2, e2) = spine[0], spine[1]
        self.get_logger().info(
            f'picked: pipe {best["score"]:.0%}, near end NED ({n1:.2f}, {e1:.2f}), first section '
            f'heading {math.degrees(math.atan2(e2 - e1, n2 - n1)) % 360:.0f} deg, top '
            f'{self.pipe["top"]:.2f} m deep, spine ' + ' '.join(f'({n:.1f},{e:.1f})' for n, e in spine))
        self.enter(S.SURFACE)                       # step 2 ends here

    # ---------- pool plan: approach the nearest part of the pipe ----------
    def pick_target(self, m):
        if self.srch is not None:           # scan_mode:=search: the closer look is done
            self.look_judge(m)
            return
        found = m.get('outlines') or []
        sonar = m.get('sonar') or [0.0, 0.0]
        cands, far, wrong = pool_candidates(found, sonar, 0.0 if self.pool_pick_any else self.side,
                                            self.pool_max_pick,
                                            ignore_flat=self.pool_band != 'off')
        for i, (score, g) in enumerate(cands[:5]):
            self.get_logger().info(f'    outline {i}: pipe {score:.0%} {g["parts"]} {g["n"]} crumbs, '
                                   f'{g["length"]:.1f} m long, {g["width"]:.2f} m wide')
        if far or wrong:
            self.get_logger().info(f'    ignored {far} shape(s) more than {self.pool_max_pick:.1f} m away and '
                                   f'{wrong} on the other side of where the scan started')
        if self.after_pick == 'follow' and self.spot_check:
            self.check_spots(cands, m)
            return
        best = cands[0][1] if cands else None
        if best is None or cands[0][0] < self.min_score:
            self.look_again(f'no outline scores {self.min_score:.0%}')
            return
        self.pipe_spine = np.array(best['spine'], float)
        if not self.retarget(m):
            self.look_again('no crumbs along the picked outline')
            return
        tn, te = self.map_to_ned(*self.aim)
        here = self.cur if self.cur else self.hold     # a dry run on a sub with no position yet
        if self.after_pick == 'follow':
            self.start_over(best, tn, te, here, cands[0][0])
            return
        # turn so the pipe is on pool_side: to the right means heading 90 deg left of it
        self.approach_yaw = math.atan2(te - here[1], tn - here[0]) - self.side * math.pi / 2
        self.get_logger().info(
            f'picked: pipe {cands[0][0]:.0%}, {best["length"]:.1f} m; nearest part at NED '
            f'({tn:.2f}, {te:.2f}), {math.hypot(tn - here[0], te - here[1]):.2f} m away - '
            f'strafing to it, heading {math.degrees(self.approach_yaw) % 360:.0f} deg')
        self.retarget_t = self.now()
        self.to_phase('face')

    # ---------- pool plan: ride over the pipe and follow it (Ruth, Oct 15) ----------
    # The Ping360's slice runs across the sub, so with the sub lined up along the pipe
    # every sweep cuts the pipe once: one dot where it crosses. A line through the
    # last ~1 m of those dots gives the pipe's direction (bends included) and how far
    # to the side of the sonar it lies. The sonar is SONAR_FWD_M ahead of the sub's
    # centre; the bottom camera is ~0.2 m behind the sonar (Ruth).
    def pipe_dir(self, sp, tn, te, here):
        """Along spine sp (NED) at (tn, te): the direction toward its longer part (or
        follow_toward), and the metres of it before and after that point."""
        k = int(np.argmin(np.hypot(sp[:, 0] - tn, sp[:, 1] - te)))
        seg = np.hypot(*np.diff(sp, axis=0).T) if len(sp) > 1 else np.zeros(0)
        before, after = float(seg[:k].sum()), float(seg[k:].sum())
        if len(sp) < 2:
            d = np.array([math.cos(here[3] if len(here) > 3 and here[3] is not None else 0.0), 0.0])
        elif after >= before:
            d = sp[min(k + 1, len(sp) - 1)] - sp[k] if k + 1 < len(sp) else sp[k] - sp[k - 1]
        else:
            d = sp[k - 1] - sp[k]
        d = d / max(np.linalg.norm(d), 1e-6)
        if self.follow_toward >= 0:
            want = math.radians(self.follow_toward)
            if d @ np.array([math.cos(want), math.sin(want)]) < 0:
                d = -d
        return d, before, after

    def start_over(self, best, tn, te, here, score):
        """The picked pipe's direction at the aim point, toward its longer part, and
        the sonar's target: right above that point."""
        sp = np.array([self.map_to_ned(x, y) for x, y in best['spine']], float)
        d, before, after = self.pipe_dir(sp, tn, te, here)
        self.f_dir, self.over_pt, self.aligned, self.a_up, self.align_rev = d, (tn, te), False, None, False
        self.carrot = (here[0], here[1])
        self.last_t = self.now()
        self.get_logger().info(
            f'picked: pipe {score:.0%}, {best["length"]:.1f} m; nearest part at NED ({tn:.2f}, {te:.2f}), '
            f'{math.hypot(tn - here[0], te - here[1]):.2f} m away - going over it, then following it '
            f'toward heading {math.degrees(math.atan2(d[1], d[0])) % 360:.0f} deg '
            f'({max(before, after):.1f} m of it that way)')
        self.to_phase('to pipe')

    def yaw_rate_to(self, yaw_d):
        h = self.cur_yaw if self.cur_yaw is not None else yaw_d
        return max(-self.follow_yaw, min(self.follow_yaw, 0.8 * wrap_pi(yaw_d - h))), wrap_pi(yaw_d - h)

    def go_over(self):
        """Walk a carrot to where the SONAR is over the pipe, turning (slowly) to face along it."""
        d, (tn, te) = self.f_dir, self.over_pt
        yaw_d = math.atan2(d[1], d[0])
        ahead = sonar_settings.SONAR_FWD_M
        gn, ge = tn - ahead * math.cos(yaw_d), te - ahead * math.sin(yaw_d)
        dt, self.last_t = min(self.now() - self.last_t, 0.5), self.now()
        cn, ce = self.carrot
        left = math.hypot(gn - cn, ge - ce)
        step = min(self.follow_speed * 1.5 * dt, left)
        ux, uy = ((gn - cn) / left, (ge - ce) / left) if left > 1e-6 else (0.0, 0.0)
        self.carrot = (cn + ux * step, ce + uy * step)
        v = self.follow_speed * 1.5 if left > step else 0.0
        rate, err = self.yaw_rate_to(yaw_d)
        self.stream_posvel(self.carrot[0], self.carrot[1], self.hold[2], ux * v, uy * v, rate, 'over the pipe')
        if self.dry:
            if self.now() - self.state_t0 < 0.3:
                self.get_logger().info(f'    dry run: put the sonar over NED ({tn:.2f}, {te:.2f}), '
                                       f'heading {math.degrees(yaw_d) % 360:.0f} deg')
            if self.now() - self.state_t0 > 5:
                self.arrived_over()
            return
        h = self.cur_yaw if self.cur_yaw is not None else yaw_d
        sn, se = self.cur[0] + ahead * math.cos(h), self.cur[1] + ahead * math.sin(h)
        if (math.hypot(tn - sn, te - se) < 0.15 and abs(err) < math.radians(8)
                and left < 0.02 and self.speed() < 0.05):
            self.get_logger().info(f'    over the {"spot" if self.spot_active else "pipe"}: sonar '
                                   f'{math.hypot(tn - sn, te - se):.2f} m from the picked point, '
                                   f'heading {math.degrees(h) % 360:.0f} deg')
            self.arrived_over()

    def arrived_over(self):
        if self.spot_active is not None:
            self.look_view(f'spot {self.spot_i + 1}')
            self.to_phase('spot anchor')
            return
        if self.stop_after == 'over' and not self.aligned:
            self.stop_here('over')
            return
        if self.align_walk > 0 and not self.aligned:
            if not self.over_checked and not self.align_rev:
                self.over_check_begin()             # is the pipe really under the sonar? slide over it
                return
            if not self.walk_down_begin():          # down to the pipe first, if it is far below
                self.align_begin()                  # step 2, then following
            return
        if self.stop_after == 'align':
            self.stop_here('align')
            return
        if self.srch is not None:
            if self.srch['kind'] == 'lost':
                self.follow_resume()
                return
            self.srch = None                        # the search is over: following from here
        self.follow_map()

    def over_check_begin(self):
        """Over the picked spot, looking straight down (Ruth, Oct 25): hold for OVER_CHECK_SWEEPS sweeps. The
        pipe at its height to one side: slide across over it first (the side-looking search places it up
        to ~0.5 m off). Its height measured straight down replaces the search's."""
        here = self.cur if self.cur else self.carrot
        self.oc = {'known': set(), 'pts': [], 'sweep': None, 'sweep_t': None, 'n_sw': 0, 'period': 6.0,
                   't0': self.now(), 'at': (here[0], here[1])}
        self.to_phase('over check')

    def over_check_step(self, m):
        O, now = self.oc, self.now()
        n, e = O['at']
        self.stream_posvel(n, e, self.hold[2], 0.0, 0.0, 0.0, 'looking down')
        sw = m.get('sweep', 0)
        if sw != O['sweep']:
            if O['sweep_t'] is not None and 0.5 < now - O['sweep_t'] < 60:
                O['period'] = 0.6 * O['period'] + 0.4 * (now - O['sweep_t'])
            O['n_sw'] += O['sweep'] is not None
            O['sweep'], O['sweep_t'] = sw, now
        here = self.cur if self.cur else (n, e)
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        sonar = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
        d = self.f_dir / max(np.linalg.norm(self.f_dir), 1e-6)
        right = np.array([-d[1], d[0]])
        up = self.a_up_now()
        for x, y, z in (np.array(m['crumbs'], float)[:, [0, 1, 4]] if m.get('crumbs') else []):
            key = (round(x, 2), round(y, 2), round(z, 2))
            if key in O['known']:
                continue
            O['known'].add(key)
            rel = np.array(self.map_to_ned(x, y)) - sonar
            along, across = float(rel @ d), float(rel @ right)
            if abs(along) <= 0.6 and abs(across) <= OVER_CHECK_M and (up is None or abs(z - up) <= 0.3):
                O['pts'].append((across, z))
        if not (O['n_sw'] >= OVER_CHECK_SWEEPS or now - O['t0'] > 3 * O['period'] + 10
                or (self.dry and now - O['t0'] > 5)):
            return
        self.over_checked = True
        if len(O['pts']) >= 2:
            lat, zm = float(np.median([p[0] for p in O['pts']])), float(np.median([p[1] for p in O['pts']]))
            self.a_up, self.a_up_z = zm, (self.cur[2] if self.cur else self.hold[2])
            if abs(lat) >= OVER_SLIDE_M:
                msg = (f'looking down: the pipe {abs(lat):.2f} m {"right" if lat > 0 else "left"} of the sonar, '
                       f'{-zm:.2f} m under it ({len(O["pts"])} dots) - sliding over it')
                self.over_pt = (float(sonar[0] + lat * right[0]), float(sonar[1] + lat * right[1]))
                self.get_logger().info(msg)
                self.mark(msg)
                self.carrot, self.last_t = (here[0], here[1]), now
                self.to_phase('to pipe')
                return
            self.get_logger().info(f'    looking down: the pipe under the sonar ({lat:+.2f} m), {-zm:.2f} m under it '
                                   f'({len(O["pts"])} dots)')
        else:
            self.get_logger().info(f'    looking down: no pipe under the sonar in {O["n_sw"]} sweeps - the walk decides')
        if not self.walk_down_begin():
            self.align_begin()

    def alt_guard(self, ph):
        """pool_alt_m, while searching (the stops, the way back to them, over to the pipe before
        going down): closer to the bottom than guard_alt, the held depth rises until it isn't."""
        searching = (self.phase.startswith('pool scan') or ph in ('search back', 'over check')
                     or (ph == 'to pipe' and not self.low_for_walk))
        alt = self.alt_now(1.5) if searching and self.hold is not None else None
        if alt is None or alt >= self.guard_alt + (0.1 if self.guard_on else 0.0):
            if self.guard_on:
                self.guard_on = False
                self.get_logger().info(f'    search guard: {alt if alt is not None else float("nan"):.2f} m '
                                       f'above the bottom again')
            self.guard_t = self.now()
            return
        if not self.guard_on:
            self.guard_on = True
            self.get_logger().warn(f'search guard: only {alt:.2f} m above the bottom (at least '
                                   f'{self.guard_alt:.2f} while searching) - going up')
        dt, self.guard_t = min(self.now() - getattr(self, 'guard_t', self.now()), 0.5), self.now()
        n, e, z, yaw = self.hold
        self.hold = (n, e, max(self.z_surf, z - ALT_DZ_SPEED * dt), yaw)

    def walk_down_begin(self):
        """Over the pipe, before the walk: if it lies more than WALK_PIPE_BELOW_M (+0.2) under the
        sonar, go down until it doesn't - straight down over where the sub is. False: no need."""
        up = self.a_up_now()
        if up is None or -up <= WALK_PIPE_BELOW_M + 0.2:
            return False
        here = self.cur if self.cur else self.hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        z = here[2] if len(here) > 2 else self.hold[2]
        want = min(z + (-up - WALK_PIPE_BELOW_M), self.z_surf + self.max_depth)
        self.hold = (self.carrot[0], self.carrot[1], z, h)
        self.wd = {'want': want, 'z0': z}
        msg = (f'over the pipe: it is {-up:.2f} m under the sonar - down {want - z:.2f} m, to '
               f'{WALK_PIPE_BELOW_M:.1f} m above it, before the walk')
        self.get_logger().info(msg)
        self.mark(msg)
        self.to_phase('down to pipe')
        return True

    def walk_down_step(self):
        n, e, z, yaw = self.hold
        want = self.wd['want']
        alt = self.floor_clearance()
        stop = None
        if self.dry:
            z = want
            if self.now() - self.state_t0 > 3:
                stop = 'dry run'
        else:
            nz = self.ramp(z, want, ALT_DZ_SPEED)
            if nz > z and alt is not None and alt - (nz - z) < DZ_MIN_ALT_M:
                stop = f'{alt:.2f} m above the bottom'
            else:
                z = nz
            if self.cur and abs(z - want) < 0.01 and abs(self.cur[2] - want) < 0.1:
                stop = 'there'
        self.hold = (n, e, z, yaw)
        self.stream_posvel(n, e, z, 0.0, 0.0, 0.0, 'down to the pipe')
        if stop is None:
            return
        zc = self.cur[2] if self.cur else z
        self.a_up, self.a_up_z = self.a_up_now(), zc
        self.hold = (n, e, zc, yaw)
        self.low_for_walk = True
        self.get_logger().info(f'    down to the pipe ({stop}): {self.below_surface(zc):.2f} m down, the pipe '
                               f'~{-self.a_up:.2f} m under the sonar')
        self.align_begin()

    # ---- spot check: look again from over the likely spots (Ruth, Oct 16) ----
    def spot_from(self, g, score, m):
        """A likely shape as a spot: its dot nearest the sonar, in NED, and the shape's
        direction there. None when no dots lie along it."""
        if not m.get('crumbs') or not m.get('sonar'):
            return None
        c = np.array(m['crumbs'], float)[:, :2]
        spine = np.array(g['spine'], float)
        on = outlines.dist_to_polyline(c, spine) < 0.4
        if not on.any():
            return None
        sx, sy = m['sonar']
        x, y = c[on][np.argmin(np.hypot(c[on, 0] - sx, c[on, 1] - sy))]
        tn, te = self.map_to_ned(x, y)
        sp = np.array([self.map_to_ned(a, b) for a, b in spine], float)
        d, _, _ = self.pipe_dir(sp, tn, te, self.cur if self.cur else self.hold)
        return {'pt': (tn, te), 'd': d, 'score': score, 'length': g['length']}

    def check_spots(self, cands, m):
        """The scan's likely shapes, best first, become spots to look at again."""
        spots = []
        for score, g in cands:
            if score < self.spot_min_score or len(spots) >= self.spot_count:
                break
            s = self.spot_from(g, score, m)
            if s and all(math.dist(s['pt'], o['pt']) > 0.4 for o in spots):
                spots.append(s)
        if not spots:
            self.look_again(f'no outline scores {self.spot_min_score:.0%}')
            return
        self.spots, self.spot_i = spots, 0
        self.get_logger().info(f'likely spots to look at again: ' + ', '.join(
            f'{k + 1}) {s["score"]:.0%} {s["length"]:.1f} m at NED ({s["pt"][0]:.2f}, {s["pt"][1]:.2f})'
            for k, s in enumerate(spots)))
        self.go_spot(spots[0])

    def go_spot(self, s, look=True):
        """Put the sonar over spot s, facing along its shape; then look at it (or follow from it)."""
        here = self.cur if self.cur else self.hold
        self.f_dir, self.over_pt, self.aligned, self.a_up, self.align_rev = s['d'], s['pt'], False, None, False
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.spot_active = s if look else None
        self.get_logger().info(
            f'{"spot " + str(self.spot_i + 1) + ": going over" if look else "following from the best spot: going over"} '
            f'NED ({s["pt"][0]:.2f}, {s["pt"][1]:.2f}), {math.hypot(s["pt"][0] - here[0], s["pt"][1] - here[1]):.2f} m '
            f'away, heading {math.degrees(math.atan2(s["d"][1], s["d"][0])) % 360:.0f} deg')
        self.to_phase('to pipe')

    def spot_anchor(self):
        """Hold still until the reset map runs from (0, 0), tie it to NED, then turn and look."""
        n, e, down, h = self.f_hold
        self.stream_posvel(n, e, down, 0.0, 0.0, 0.0, 'over the spot')
        m = self.map_get()
        if m['state'] == 'running' and m['pose'] and math.hypot(*m['pose'][:2]) < 0.05:
            if self.dry or self.cur is None:
                self.map_origin = (n, e, h)
            else:
                self.map_origin = (self.cur[0], self.cur[1], self.cur_yaw)
            half = math.radians(self.spot_turn) / 2
            self.sl = {'targets': [h - half, h + half, h], 'k': 0, 'k_t': self.now(), 'sweep0': m.get('sweep', 0),
                       'known': set(), 'hits': set(), 'dots': 0}
            self.get_logger().info(f'    spot {self.spot_i + 1}: turning {self.spot_turn / 2:.0f} deg each way, '
                                   f'looking straight down' + (' (dry run: the pilot turns the sub)' if self.dry else ''))
            self.to_phase('spot look')

    def spot_look(self, m):
        """Turn slowly through the targets while counting the sweeps with pipe dots near the spot."""
        n, e, down, _ = self.f_hold
        L, s = self.sl, self.spot_active
        want = L['targets'][L['k']]
        err = wrap_pi(want - (self.cur_yaw if self.cur_yaw is not None else want))
        rate = max(-self.spot_rate, min(self.spot_rate, 0.8 * err))
        self.stream_posvel(n, e, down, 0.0, 0.0, rate, 'look at the spot')
        for x, y, z in (np.array(m['crumbs'], float)[:, [0, 1, 4]] if m.get('crumbs') else []):
            key = (round(x, 2), round(y, 2), round(z, 2))
            if key in L['known']:
                continue
            L['known'].add(key)
            dn, de = self.map_to_ned(x, y)
            if math.hypot(dn - s['pt'][0], de - s['pt'][1]) <= self.spot_radius:
                L['dots'] += 1
                L['hits'].add(m.get('sweep', 0))
        t = self.now() - L['k_t']
        if (abs(err) < math.radians(2) or t > 2 * math.radians(self.spot_turn) / self.spot_rate + 10
                or (self.dry and t > 8)):
            L['k'], L['k_t'] = L['k'] + 1, self.now()
            if L['k'] >= len(L['targets']):
                self.spot_done(m)

    def spot_done(self, m):
        L, s = self.sl, self.spot_active
        sweeps = max(1, m.get('sweep', 0) - L['sweep0'])
        s['frac'], s['sweeps'], s['dots'] = min(1.0, len(L['hits']) / sweeps), sweeps, L['dots']
        msg = (f'spot {self.spot_i + 1}: pipe dots within {self.spot_radius:.1f} m of it in {len(L["hits"])} of '
               f'{sweeps} sweeps ({s["frac"]:.0%}), {L["dots"]} dots')
        self.get_logger().info(msg)
        if self.recording:
            try:
                self.map_post('/mark', msg)
            except OSError:
                pass
        self.spot_i += 1
        if self.spot_i < len(self.spots):
            self.go_spot(self.spots[self.spot_i])
            return
        best = max(self.spots, key=lambda x: (x['frac'], x['dots']))
        k = self.spots.index(best) + 1
        if best['frac'] < self.spot_min_frac:
            self.get_logger().warn(f'spot check: no spot showed the pipe in {self.spot_min_frac:.0%} of its sweeps '
                                   f'(best: spot {k}, {best["frac"]:.0%}) - surfacing')
            self.enter(S.SURFACE)
            return
        self.get_logger().info(f'spot check: spot {k} showed the pipe most ({best["frac"]:.0%} of its sweeps)')
        if self.stop_after == 'spots':
            self.stop_here('spots')
            return
        self.go_spot(best, look=False)

    # ---- search: stops, a closer look where something shows, line up, follow (Ruth, Oct 23) ----
    def stop_want(self):
        """The heading of the stop being turned to or looked from."""
        if self.srch is None:
            return self.stop_yaw0 + self.stop_i * self.scan_turn / self.scan_stops
        if self.look is not None:
            return self.look['heads'][self.look['k']]
        return self.stop_yaw0 + self.stop_i * self.srch['step']

    def stop_label(self):
        if self.srch is None:
            return f'stop {self.stop_i + 1}/{self.scan_stops}'
        if self.look is not None:
            return f'closer look {self.look["k"] + 1}/{len(self.look["heads"])}'
        return f'stop {self.stop_i + 1}/{self.srch["n"]}'

    def search_start(self):
        """The map runs: the search's stops from the heading the sub has now - or, back from a
        closer look that didn't pass, on from the stop after the one it looked from."""
        if self.srch is None:
            self.srch = {'kind': 'approach', 't0': self.now(), 'resume': None}
        sr = self.srch
        if sr.get('goal') is None:          # where the sub turns: come back here if a look fails
            sr['goal'], sr['yaw'] = (self.hold[0], self.hold[1]), self.hold[3]
        if sr.get('z') is None:             # ... and the depth it searches at
            sr['z'] = self.hold[2]
        if sr['resume'] is not None:
            self.stop_i, sr['resume'] = sr['resume'], None
        else:
            turn = self.scan_turn if sr['kind'] == 'approach' else 2 * math.pi
            sr['n'] = max(1, math.ceil(turn / self.search_step - 1e-6))
            sr['step'] = turn / sr['n']
            sr['yaw0'] = self.cur_yaw if self.cur_yaw is not None else self.hold[3]
            self.stop_i = 0
        self.stop_yaw0 = sr['yaw0']
        self.look, self.s_known, self.stop_dots, self.still_t = None, set(), [], None
        self.get_logger().info(f'    search: {sr["n"] - self.stop_i} stop(s), {math.degrees(sr["step"]):.0f} deg apart, '
                               f'{self.sweeps_per_stop} sweeps at each'
                               + ('' if sr['kind'] == 'approach' else ', leaving out the part already followed'))
        self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())

    def stop_collect(self, m, keep=True):
        """Dots new on the map, tagged with the sweep they came in (keep=False: only noted - they
        came while the sub turned)."""
        sw = m.get('sweep', 0)
        for x, y, z in (np.array(m['crumbs'], float)[:, [0, 1, 4]] if m.get('crumbs') else []):
            key = (round(x, 2), round(y, 2), round(z, 2))
            if key in self.s_known:
                continue
            self.s_known.add(key)
            if keep:
                self.stop_dots.append((x, y, z, sw))

    def near_path(self, xy):
        """Which of these map points lie within PATH_M of where the sonar went while following."""
        xy = np.asarray(xy, float).reshape(-1, 2)
        if not self.f_path or not len(xy):
            return np.zeros(len(xy), bool)
        P = np.array(self.f_path, float)
        ned = np.array([self.map_to_ned(x, y) for x, y in xy], float)
        return np.min(np.hypot(ned[:, None, 0] - P[None, :, 0], ned[:, None, 1] - P[None, :, 1]), axis=1) <= PATH_M

    def stop_blob(self, m):
        """This stop's tightest bunch of dots - ((x, y), dots, sweeps, up) - if it came from at least
        blob_sweeps of the stop's sweeps; within pool_max_pick_m, and not on the part followed."""
        if not self.stop_dots:
            return None
        D = np.array(self.stop_dots, float)
        sx, sy = m.get('sonar') or (0.0, 0.0)
        ok = np.ones(len(D), bool) if self.pool_max_pick <= 0 else np.hypot(D[:, 0] - sx, D[:, 1] - sy) <= self.pool_max_pick
        if self.srch['kind'] == 'lost':
            ok &= ~self.near_path(D[:, :2])
        D, best = D[ok], None
        for i in range(len(D)):
            near = np.hypot(D[:, 0] - D[i, 0], D[:, 1] - D[i, 1]) <= self.blob_m
            k, n = len(set(D[near, 3])), int(near.sum())
            if best is None or (k, n) > (best[2], best[1]):
                best = ((float(np.mean(D[near, 0])), float(np.mean(D[near, 1]))), n, k, float(np.median(D[near, 2])))
        return best if best is not None and best[2] >= self.blob_sweeps else None

    def search_after_stop(self, m):
        """A stop's sweeps are in: a closer look if it's promising, else the next stop; the closer
        look's last stop done, the verdict (after the outlines catch up)."""
        if self.look is None:
            blob, label, h = self.stop_blob(m), self.stop_label(), self.stop_want()
            if blob is None:
                self.get_logger().info(f'    {label}: {len(self.stop_dots)} new dot(s), none in one spot from '
                                       f'{self.blob_sweeps} sweeps')
                self.next_stop()
                return
            (bx, by), n, k, up = blob
            steps = max(1, int(round(self.look_span / self.look_step)))
            heads = [h + i * self.look_step for i in range(-steps, steps + 1) if i != 0]
            self.look = {'heads': heads, 'k': 0, 'blob': (bx, by), 'up': up}
            sx, sy = m.get('sonar') or (0.0, 0.0)
            msg = (f'{label}: {n} dots in one spot from {k} sweeps, {math.hypot(bx - sx, by - sy):.1f} m from the sonar '
                   f'- a closer look: {len(heads)} more stops {math.degrees(self.look_step):.0f} deg apart')
            self.get_logger().info('    ' + msg)
            self.mark(msg)
            self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())
            return
        self.look['k'] += 1
        if self.look['k'] < len(self.look['heads']):
            self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())
            return
        self.hold = (self.hold[0], self.hold[1], self.hold[2], self.cur_yaw if self.cur_yaw is not None else self.hold[3])
        self.to_phase(f'{self.scan} scan: settle')

    def next_stop(self):
        self.look = None
        self.stop_i += 1
        if self.stop_i >= self.srch['n']:
            self.search_failed(f'nothing passed in {self.srch["n"]} stops')
            return
        self.to_phase(f'{self.scan} scan: stop turn', self.stop_label())

    def look_judge(self, m):
        """The closer look is in: the most pipe-like shape near the promising bunch passes - then over
        its densest spot, facing along it (step 1). Otherwise on with the stops."""
        L, sr = self.look, self.srch
        cands, _, _ = pool_candidates(m.get('outlines') or [], m.get('sonar') or [0.0, 0.0], 0.0,
                                      self.pool_max_pick, ignore_flat=self.pool_band != 'off')
        blob, best = np.array([L['blob']], float), None
        for score, g in cands:
            sp = np.array(g['spine'], float)
            if float(outlines.dist_to_polyline(blob, sp)[0]) > LOOK_NEAR_M:
                continue
            if sr['kind'] == 'lost' and self.near_path(sp).mean() > 0.5:
                continue                                # the part already followed
            best = (score, g)
            break
        spot = self.dense_spot(best[1], m) if best is not None and best[0] >= self.look_min_score else None
        if spot is None:
            why = (f'the best shape near it is {best[0]:.0%} {best[1]["parts"]}, {best[1]["n"]} dots, '
                   f'{best[1]["length"]:.2f} m' if best else 'no shape near it')
            msg = f'closer look: {why} - not passed ({self.look_min_score:.0%} needed)'
            self.get_logger().info('    ' + msg + '; on with the stops')
            self.mark(msg)
            self.next_stop()
            return
        score, g = best
        (tn, te), d, up, k = spot
        here = self.cur if self.cur else self.hold
        msg = (f'closer look passed: pipe {score:.0%}, {g["n"]} dots, {g["length"]:.2f} m; densest spot ({k} dots within '
               f'{DENSE_M:.2f} m) at NED ({tn:.2f}, {te:.2f}), {math.hypot(tn - here[0], te - here[1]):.2f} m away - '
               f'over it heading {math.degrees(math.atan2(d[1], d[0])) % 360:.0f} deg'
               + (f', then a {self.align_walk:.1f} m walk along it' if self.align_walk > 0 else ''))
        self.get_logger().info(msg)
        self.mark(msg)
        sr['resume'] = self.stop_i + 1          # if the walk says it isn't the pipe after all
        self.look = None
        self.f_dir, self.over_pt, self.spot_active, self.aligned, self.a_up = d, (tn, te), None, False, up
        self.a_up_z = here[2] if len(here) > 2 else self.hold[2]
        self.align_rev, self.over_checked = False, False
        # looking straight down from here on (Ruth, Oct 25): faster sweeps under the sub, and the check
        # once over it sees the pipe the way it echoes best
        self.anchor_then('over the pipe: sonar straight down', self.go_over_start)

    def go_over_start(self, m):
        here = self.cur if self.cur else self.hold
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.to_phase('to pipe')

    def dense_spot(self, g, m):
        """Along shape g: the dot with the most others within DENSE_M (NED), the shape's direction
        there - toward its longer part, or in a lost search away from where it was lost - the dots'
        height there, and how many."""
        if not m.get('crumbs'):
            return None
        c = np.array(m['crumbs'], float)
        spine = np.array(g['spine'], float)
        on = outlines.dist_to_polyline(c[:, :2], spine) < 0.4
        if self.srch['kind'] == 'lost':
            on &= ~self.near_path(c[:, :2])
        if on.sum() < 3:
            return None
        xy, up = c[on, :2], c[on, 4]
        close = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1]) <= DENSE_M
        count = close.sum(axis=1)
        sx, sy = m.get('sonar') or (0.0, 0.0)
        k = max(range(len(xy)), key=lambda i: (count[i], -math.hypot(xy[i, 0] - sx, xy[i, 1] - sy)))
        tn, te = self.map_to_ned(*xy[k])
        sp = np.array([self.map_to_ned(a, b) for a, b in spine], float)
        d, _, _ = self.pipe_dir(sp, tn, te, self.cur if self.cur else self.hold)
        if self.srch['kind'] == 'lost':
            away = np.array([tn, te]) - self.srch['P']
            if float(d @ (away if np.linalg.norm(away) > 0.2 else self.srch['dir'])) < 0:
                d = -d
        return (tn, te), d, float(np.median(up[close[k]])), int(count[k])

    def search_failed(self, why):
        sr = self.srch
        if sr['kind'] == 'approach':
            self.get_logger().warn(f'pool scan: {why} - surfacing')
            self.srch = None
            self.enter(S.SURFACE)
            return
        self.get_logger().warn(f'search {sr["entry"]["n"]} where the pipe was lost: {why}')
        self.lost_retry()

    def search_go_back(self, resume_i):
        """Back to where the search turns, then on with its stops from resume_i (None: all again)."""
        sr = self.srch
        if resume_i is not None and resume_i >= sr['n']:
            self.search_failed(f'nothing passed in {sr["n"]} stops')
            return
        sr['resume'] = resume_i
        if self.low_for_walk:               # gone down for the walk: back up to the search's depth
            self.low_for_walk, self.back_z = False, sr.get('z')
        here = self.cur if self.cur else self.hold
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.to_phase('search back')

    def search_back_step(self):
        sr = self.srch
        if self.back_z is not None:         # the depth on the way, up or down at ALT_DZ_SPEED
            n, e, z, yaw = self.hold
            self.hold = (n, e, self.back_z if self.dry else self.ramp(z, self.back_z, ALT_DZ_SPEED), yaw)
            if self.dry or (self.cur and abs(self.hold[2] - self.back_z) < 0.01 and abs(self.cur[2] - self.back_z) < 0.1):
                self.back_z = None
        left, _ = self.walk_to(sr['goal'], self.return_speed, sr['yaw'], 'to the search spot')
        if self.back_z is None and ((left < 0.05 and self.speed() < 0.05) or (self.dry and self.now() - self.state_t0 > 5)):
            self.hold = (sr['goal'][0], sr['goal'][1], self.hold[2], sr['yaw'])
            self.goto_ned(*self.hold, 'hold')
            self.scan = 'pool'
            self.to_phase('pool scan: map setup')

    def lost_search_begin(self, now, here):
        """The pipe has gone mid-follow: search for it from where it was last seen - unless
        follow_searches searches there already didn't get past it. True if a search began."""
        if self.follow_searches <= 0 or self.f_line is None:
            return False
        pt = np.array(self.f_prev_loss if self.f_stuck_t is not None and self.f_prev_loss is not None
                      else self.f_line, float)
        entry = next((s for s in self.lost_spots if float(np.linalg.norm(s['pt'] - pt)) <= SAME_SPOT_M), None)
        if entry is None:
            entry = {'pt': pt, 'n': 0}
            self.lost_spots.append(entry)
        if entry['n'] >= self.follow_searches:
            return False
        entry['n'] += 1
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        self.srch = {'kind': 'lost', 'P': pt, 'dir': self.f_dir.copy(), 't0': now, 'entry': entry, 'resume': None,
                     'goal': (pt[0] - ahead * math.cos(h), pt[1] - ahead * math.sin(h)), 'yaw': h}
        self.low_for_walk = False
        alt = self.alt_now() if self.pool_alt > 0 else None
        if alt is not None:                 # pool_alt_m: search from that high again (only ever up)
            z = here[2] if len(here) > 2 else self.hold[2]
            self.srch['z'] = self.back_z = max(self.z_surf, z - max(0.0, self.pool_alt - alt))
        msg = (f'pipe lost {self.f_dist:.1f} m along - search {entry["n"]} of {self.follow_searches} there: back over '
               f'where it was last seen, NED ({pt[0]:.2f}, {pt[1]:.2f}), and a side scan')
        self.get_logger().info(msg)
        self.mark(msg)
        self.carrot, self.last_t = (here[0], here[1]), now
        self.to_phase('search back')
        return True

    def lost_retry(self):
        """A search where the pipe was lost found nothing: another, or (follow_searches done) back to
        following, where the pipe still missing ends it the follow's own way."""
        sr = self.srch
        if sr['entry']['n'] < self.follow_searches:
            sr['entry']['n'] += 1
            self.get_logger().info(f'    search {sr["entry"]["n"]} of {self.follow_searches} at the same spot')
            self.search_go_back(None)
            return
        self.get_logger().warn(f'searched {sr["entry"]["n"]} times where the pipe was lost - its end')
        self.f_t0 += self.now() - sr['t0']
        self.srch, self.look = None, None
        self.anchor_then('back to following', self.lost_give_up)

    def lost_give_up(self, m):
        self.f_known, self.f_hold_pt = set(), None
        self.to_phase('follow')

    def anchor_then(self, what, then):
        """The map's follow view (look_view resets it), then still until it runs from (0, 0); tied to
        NED, then(m)."""
        self.look_view(what)
        self.anchor_next = then
        self.to_phase('anchor')

    def anchor_step(self):
        n, e, down, h = self.f_hold
        self.stream_posvel(n, e, down, 0.0, 0.0, 0.0, 'hold')
        m = self.map_get()
        if m['state'] == 'running' and m['pose'] and math.hypot(*m['pose'][:2]) < 0.05:
            self.map_origin = (n, e, h) if self.dry or self.cur is None else (self.cur[0], self.cur[1], self.cur_yaw)
            self.anchor_next(m)

    def align_begin(self):
        """Step 2 (Ruth, Oct 23; turning as it goes, Oct 25): over the pipe facing the way step 1 says it
        runs, walk STRAIGHT that way looking straight down. The pipe's dots make a line across the walk:
        if step 1 was off, the pipe drifts away from the sonar's middle. As soon as it is align_turn_m off
        - before it can leave the +-30 deg view - or the walk has gone align_walk_m, turn by that line's
        angle and put the sonar back over the pipe where it is NOW (not back where the walk began), then
        follow from there."""
        here = self.cur if self.cur else self.hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        d = self.f_dir / max(np.linalg.norm(self.f_dir), 1e-6)
        self.al = {'d': d.copy(), 'pts': [], 'known': set(), 'up': self.a_up_now(), 'fit': None,
                   's0': np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)]),
                   'sweep': None, 'sweep_t': None, 'period': 6.0, 'n_sw': 0}
        self.get_logger().info(f'align: walking straight along heading {math.degrees(math.atan2(d[1], d[0])) % 360:.0f} '
                               f'deg looking down, at most {self.align_walk:.2f} m - turning once the pipe is '
                               f'{self.align_turn:.2f} m off the sonar\'s middle')
        self.anchor_then('align walk', self.align_go)

    def align_go(self, m):
        here = self.cur if self.cur else self.hold
        self.start_end_watch()                  # light_detector:=yolo: the camera can see the pipe's end
        self.al['t0'] = self.now()
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.to_phase('align walk')

    def align_line(self, loose=False):
        """The pipe across the walk, from its dots at the pipe's height (the search's, else the walk's
        own): (slope, offset, dots) with across = offset + slope * along, or None. A line needs 3 dots
        from 3 sweeps spread ALIGN_MIN_SPAN_M along the walk; loose (the pipe about to leave the view)
        takes 2 sweeps over 0.1 m."""
        A = self.al
        Q0 = np.array(A['pts'], float).reshape(-1, 4)
        span, sweeps = (0.1, 2) if loose else (ALIGN_MIN_SPAN_M, 3)
        for up0 in ([A['up']] if A['up'] is not None else []) + ([float(np.median(Q0[:, 2]))] if len(Q0) else []):
            Q = Q0[np.abs(Q0[:, 2] - up0) <= ALIGN_UP_M]
            if len(Q) >= 3 and len(set(Q[:, 3])) >= sweeps and np.ptp(Q[:, 0]) >= span:
                b, a0 = np.polyfit(Q[:, 0], Q[:, 1], 1)
                keep = np.abs(Q[:, 1] - (a0 + b * Q[:, 0])) <= 0.08
                if 3 <= keep.sum() < len(Q) and np.ptp(Q[keep, 0]) >= span:
                    Q = Q[keep]
                    b, a0 = np.polyfit(Q[:, 0], Q[:, 1], 1)
                return float(b), float(a0), Q
        return None

    def align_walk_step(self, m):
        A, now = self.al, self.now()
        sw = m.get('sweep', 0)
        if sw != A['sweep']:
            if A['sweep_t'] is not None and 0.5 < now - A['sweep_t'] < 60:
                A['period'] = 0.6 * A['period'] + 0.4 * (now - A['sweep_t'])
            A['n_sw'] += A['sweep'] is not None         # sweeps finished since the walk began
            A['sweep'], A['sweep_t'] = sw, now
        d = A['d']
        right = np.array([-d[1], d[0]])
        up0 = A['up']
        for x, y, z in (np.array(m['crumbs'], float)[:, [0, 1, 4]] if m.get('crumbs') else []):
            key = (round(x, 2), round(y, 2), round(z, 2))
            if key in A['known']:
                continue
            A['known'].add(key)
            rel = np.array(self.map_to_ned(x, y)) - A['s0']
            along, across = float(rel @ d), float(rel @ right)
            # near the line found so far, else near the walk's own line (the pipe may start up to
            # ALIGN_FIRST_M off it: the search places it ~0.2 x range off along the sub, lake sim Oct 24)
            ref = A['fit'][1] + A['fit'][0] * along if A['fit'] else 0.0
            if (abs(across - ref) <= (ALIGN_GATE_M if A['fit'] else ALIGN_FIRST_M)
                    and -0.3 <= along <= self.align_walk + 0.5
                    and (up0 is None or abs(z - up0) <= ALIGN_UP_M)):
                A['pts'].append((along, across, z, sw))
        fit = self.align_line()
        A['fit'] = fit[:2] if fit else None
        # straight on, no sliding sideways, slowly enough for ~ALIGN_SWEEPS sweeps on the way, and no
        # further than the sweeps finished so far allow (lake, Oct 24: 3 s assumed, ~6 s each)
        v = min(self.follow_speed, self.align_walk / (ALIGN_SWEEPS * A['period']))
        h = math.atan2(d[1], d[0])
        fwd = sonar_settings.SONAR_FWD_M * np.array([math.cos(h), math.sin(h)])
        s_c = float((np.array(self.carrot) + fwd - A['s0']) @ d)
        allow = self.align_walk * min(1.0, (A['n_sw'] + 1) / (ALIGN_SWEEPS + 1))
        goal = A['s0'] + min(self.align_walk, s_c + 0.15, allow) * d - fwd
        self.walk_to((float(goal[0]), float(goal[1])), v, h, 'align walk')
        here = self.cur if self.cur else self.carrot
        s = float((np.array(here[:2]) + fwd - A['s0']) @ d)
        if self.dry and now - A['t0'] < 0.3:
            self.get_logger().info(f'    dry run: drive the sub up to {self.align_walk:.2f} m straight forward, slowly')
        # the camera sees the pipe's end ahead (the model, Ruth Oct 25): the rest of it lies the other way
        if not self.align_rev and self.pipe_end_seen(ahead=True) is not None:
            self.align_rev = True
            pt = (A['s0'] + s * d + (fit[1] + fit[0] * s) * right) if fit else A['s0']
            msg = 'align: the camera sees the pipe\'s end ahead - walking the other way from here'
            self.get_logger().info(msg)
            self.mark(msg)
            self.f_dir, self.over_pt = -d, (float(pt[0]), float(pt[1]))
            self.carrot, self.last_t = (here[0], here[1]), now
            self.to_phase('to pipe')
            return
        # where the pipe is across the walk under the sonar now: the line's, else the latest dots'
        if fit:
            off = fit[1] + fit[0] * s
        else:
            last = [p[1] for p in A['pts'] if p[3] == max(q[3] for q in A['pts'])] if A['pts'] else []
            off = float(np.median(last)) if last else None
        if fit and abs(off) >= self.align_turn:
            self.align_turn_now(s, fit, f'the pipe {abs(off):.2f} m {"right" if off > 0 else "left"} of the '
                                        f'sonar\'s middle after {s:.2f} m')
            return
        if off is not None and abs(off) >= ALIGN_EDGE_M:
            # about to leave the view with no line yet: the best line there is, or the walk failed
            loose = self.align_line(loose=True)
            if loose:
                self.align_turn_now(s, loose, f'the pipe {abs(off):.2f} m off the middle after {s:.2f} m (few dots)')
            else:
                self.align_end(s)
            return
        if ((s >= self.align_walk - 0.03 and self.speed() < 0.05 and A['n_sw'] >= ALIGN_SWEEPS)
                or now - A['t0'] > 2 * (ALIGN_SWEEPS + 2) * A['period'] + 20
                or (self.dry and now - A['t0'] > 8)):
            self.align_end(s)

    def align_end(self, s):
        """The walk is over without the pipe reaching align_turn_m: a small turn by the line if there
        is one; the pipe only near where it began: walk the other way once; else not the pipe."""
        A = self.al
        fit = self.align_line()
        if fit:
            off = fit[1] + fit[0] * s
            self.align_turn_now(s, fit, f'walked {s:.2f} m, the pipe {abs(off):.2f} m '
                                        f'{"right" if off > 0 else "left"} of the sonar\'s middle')
            return
        Q = np.array(A['pts'], float).reshape(-1, 4)
        if not self.align_rev and len(Q) and float(np.max(Q[:, 0])) < ALIGN_MIN_SPAN_M:
            # the pipe only near where the walk began: step 1 pointed off its end (from a pipe's end
            # it can't tell which way the rest lies, sim Oct 24) - walk the other way from there once
            self.align_rev = True
            self.get_logger().info(f'align: the pipe showed only within {float(np.max(Q[:, 0])):.2f} m of the start '
                                   f'that way - walking the other way from there')
            self.f_dir, self.over_pt = -A['d'], (float(A['s0'][0]), float(A['s0'][1]))
            here = self.cur if self.cur else self.carrot
            self.carrot, self.last_t = (here[0], here[1]), self.now()
            self.to_phase('to pipe')
            return
        self.align_failed(f'the pipe showed in {len(set(Q[:, 3])) if len(Q) else 0} sweeps of the walk, over '
                          f'{float(np.ptp(Q[:, 0])) if len(Q) else 0.0:.2f} m')

    def align_turn_now(self, s, fit, why):
        """Turn by the line's angle, with the sonar over the pipe where the walk has got to - then follow."""
        b, a0, Q = fit
        A = self.al
        d = A['d']
        right = np.array([-d[1], d[0]])
        err = math.atan(b)
        nd = d * math.cos(err) + right * math.sin(err)
        # over the pipe where the walk has got to - but no further than where it was last seen: a walk
        # past the end of a short stretch put the sonar over nothing (sim, Oct 25)
        s_pipe = min(s, float(np.max(Q[:, 0])) + 0.1)
        pt = A['s0'] + s_pipe * d + (a0 + b * s_pipe) * right
        here = self.cur if self.cur else self.carrot
        where = 'here' if s_pipe >= s - 0.15 else f'where it was last seen, {s - s_pipe:.2f} m back'
        msg = (f'align: {why} ({len(Q)} dots, {len(set(Q[:, 3]))} sweeps) - {math.degrees(err):+.0f} deg off; '
               f'turning to heading {math.degrees(math.atan2(nd[1], nd[0])) % 360:.0f} deg, the sonar over the pipe {where}')
        self.get_logger().info(msg)
        self.mark(msg)
        self.f_dir, self.over_pt, self.aligned = nd, (float(pt[0]), float(pt[1])), True
        self.a_up, self.a_up_z = float(np.median(Q[:, 2])), (here[2] if len(here) > 2 else self.hold[2])
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.to_phase('to pipe')

    def align_failed(self, why):
        if self.srch is not None:
            self.get_logger().warn(f'align: {why} - not the pipe after all; back to the search')
            self.search_go_back(self.srch['resume'] if self.srch['resume'] is not None else self.stop_i + 1)
            return
        self.get_logger().warn(f'align: {why} - following from here the way it faces')
        self.aligned = True
        self.follow_map()

    def follow_resume(self):
        """A lost search found the pipe and the walk lined the sub up: follow on from here, the
        distance, the clock (less the search) and the survey carrying on. The map is already in the
        follow view, tied to NED at the walk's start."""
        now, sr = self.now(), self.srch
        here = self.cur if self.cur else self.hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        self.f_t0 += now - sr['t0']
        self.srch, self.look = None, None
        self.f_seen, self.f_leg_t0, self.f_line = now, now, np.array(self.over_pt, float)
        if self.a_up is not None:
            self.f_up, self.f_up_z = self.a_up_now(), (here[2] if len(here) > 2 else self.hold[2])
        self.f_lost_now, self.f_prev_loss, self.f_stuck_t, self.f_hold_pt, self.f_creep0 = False, None, None, None, None
        self.f_prev = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
        self.f_known = set()
        msg = (f'back on the pipe after {sr["entry"]["n"]} search(es) where it was lost - following on, heading '
               f'{math.degrees(math.atan2(self.f_dir[1], self.f_dir[0])) % 360:.0f} deg')
        self.get_logger().info(msg)
        self.mark(msg)
        self.to_phase('follow')

    def follow_band_now(self):
        """follow_band as the map wants it: m from the sonar, up +. Measured from the
        floor (DVL altitude), or as it is with follow_band_from:=sonar."""
        if self.follow_band_from == 'sonar':
            return (round(self.follow_band[0], 2), round(self.follow_band[1], 2))
        # the median of the last 10 s of DVL altitude, kept for up to a minute when the DVL
        # drops its lock: at the surface it lost and regained the floor, so the band jumped
        # and the follow started with none (pool, Oct 16)
        vals = [a for t, a in getattr(self, 'alt_hist', []) if self.now() - t <= 10.0]
        if vals:
            self.alt_good = (float(np.median(vals)), self.now())
        good = getattr(self, 'alt_good', None)
        if good is None or self.now() - good[1] > 60.0:
            return None
        floor_up = -(good[0] + self.sonar_above_dvl)
        return (round(floor_up + self.follow_band[0], 2), round(floor_up + self.follow_band[1], 2))

    def start_follow_here(self):
        """start_with:=follow: the sub is over the pipe, facing along it - follow from here."""
        h = self.cur_yaw if self.cur_yaw is not None else self.hold[3]
        self.f_dir = np.array([math.cos(h), math.sin(h)])
        if self.record_scan:
            try:
                self.map_post('/record', 'start follow from where the pilot left the sub')
                self.recording = True
            except OSError:
                pass
        self.get_logger().info(f'start_with:=follow - no scan; following from here, heading '
                               f'{math.degrees(h) % 360:.0f} deg')
        if self.align_walk > 0:             # the pilot's line-up, checked by the align walk first
            self.aligned, self.a_up, self.align_rev = False, None, False
            self.align_begin()
            return
        self.follow_map()

    def follow_map(self):
        if self.survey:
            self.survey_begin()
            if self.s_turn_first:               # the buoy is behind: turn first, then follow
                here = self.cur if self.cur else self.hold
                h = self.cur_yaw if self.cur_yaw is not None else self.hold[3]
                ahead = sonar_settings.SONAR_FWD_M
                self.s_turn_at, self.s_turned = (here[0], here[1]), False
                self.s_turn_sonar = (here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h))
                self.carrot, self.last_t = self.s_turn_at, self.now()
                self.get_logger().info('    turning 180 on the spot toward the buoy first, then following')
                self.to_phase('survey turn')
                return
        self.look_view('follow start')
        self.to_phase('follow start')

    def survey_turn_first(self):
        """The buoy is behind the sub: turn on the spot before following. Turning while
        following made the pipe come and go under the swinging sonar for 30 s, which read
        as the buoy end after 1.0 m (pool, Oct 17). Then back up until the sonar is where it
        was before the turn: the turn swings it 0.84 m, off the pipe when the sub is at its
        end (lake sim, Oct 24). The map is reset once there."""
        d = self.f_dir
        yaw = math.atan2(d[1], d[0])
        if not self.s_turned:
            _, err = self.walk_to(self.s_turn_at, self.return_speed, yaw, 'turning toward the buoy')
            if abs(err) < math.radians(5) or (self.dry and self.now() - self.state_t0 > 5):
                self.s_turned = True
                here = self.cur if self.cur else self.s_turn_at
                self.carrot, self.last_t = (here[0], here[1]), self.now()
                self.get_logger().info('    turned toward the buoy - backing up so the sonar is where it was')
            return
        ahead = sonar_settings.SONAR_FWD_M
        goal = (self.s_turn_sonar[0] - ahead * d[0], self.s_turn_sonar[1] - ahead * d[1])
        left, _ = self.walk_to(goal, self.follow_speed, yaw, 'backing up')
        if (left < 0.06 and self.speed() < 0.05) or (self.dry and self.now() - self.state_t0 > 10):
            self.get_logger().info('    turned toward the buoy and backed up - following from here')
            self.look_view('follow start')
            self.to_phase('follow start')

    def post_floor_filter(self):
        ff = self.floor_filter
        self.map_post('/floorband', f'{ff[0]} {ff[1]}' if ff else 'off')

    def look_view(self, what):
        """The map's follow settings (straight down, the band from the floor), then a reset
        so the map holds only what this look sees; f_hold = where to hold meanwhile."""
        self.map_post('/range', self.follow_range)
        self.map_post('/sector', self.follow_sector)
        self.map_post('/threshold', self.follow_threshold)
        self.map_post('/floorcut', 0.0)
        self.map_post('/compact', 'on' if self.pool_compact else 'off')
        self.map_post('/near', str(self.follow_near) if self.follow_near > 0 else 'off')
        self.map_post('/far', 'off')
        self.map_post('/fade', 'off')
        band = self.follow_band_now()
        self.map_post('/band', f'{band[0]} {band[1]}' if band else 'off')
        self.f_band, self.f_band_t = band, self.now()
        self.post_floor_filter()
        for cmd in ('reset', 'start', 'resume'):
            self.map_post('/mapcmd', cmd)
        if self.recording:
            try:
                where = 'from the sonar' if self.follow_band_from == 'sonar' else 'above the floor'
                ff = self.floor_filter
                self.map_post('/mark', f'{what}: sonar straight down, ' + (
                    f'floor filter {ff[0]} to {ff[1]} m above the floor line' if ff else 'band '
                    + (f'{self.follow_band[0]} to {self.follow_band[1]} m {where}' if band else 'off (no DVL)')))
            except OSError:
                pass
        if band is None:
            self.get_logger().warn('    no DVL altitude yet: no height band until the DVL is heard '
                                   '(following waits up to 6 s for it)')
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        here = self.cur if self.cur else self.hold
        self.f_hold = (here[0], here[1], self.hold[2], h)

    def follow_anchor(self):
        """Hold still until the reset map runs from (0, 0), then tie its frame to NED."""
        n, e, down, h = self.f_hold
        self.stream_posvel(n, e, down, 0.0, 0.0, 0.0, 'over the pipe')
        if self.f_band is None and self.follow_band_from == 'floor' and self.now() - self.state_t0 < 6.0:
            # no band yet: a just-started mission has not heard the DVL. Wait up to 6 s for it,
            # then set the band and clear the map, so the first sweeps can't take the floor (Oct 16)
            band = self.follow_band_now()
            if band is None:
                return
            self.map_post('/band', f'{band[0]} {band[1]}')
            self.map_post('/mapcmd', 'reset')
            self.f_band, self.f_band_t = band, self.now()
            self.get_logger().info(f'    DVL heard: band {band[0]} to {band[1]} m from the sonar')
            return
        m = self.map_get()
        if m['state'] == 'running' and m['pose'] and math.hypot(*m['pose'][:2]) < 0.05:
            if self.dry or self.cur is None:
                self.map_origin = (n, e, h)
            else:
                self.map_origin = (self.cur[0], self.cur[1], self.cur_yaw)
            here = self.cur if self.cur else self.f_hold
            ahead = sonar_settings.SONAR_FWD_M
            self.f_prev = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
            self.f_t0 = self.f_sweep_t = self.f_log_t = self.now()
            self.f_seen = None                  # nothing moves until the pipe shows under the sonar
            self.f_dist, self.f_logged, self.f_period, self.f_sweep = 0.0, 0.0, 8.0, m.get('sweep', 0)
            self.f_lat, self.f_known, self.f_box, self.f_line = 0.0, set(), None, None
            self.f_up = None                    # the pipe's height from the sonar, once seen ...
            self.f_up_z, self.f_z0, self.f_dz_on, self.f_dz_t = None, self.hold[2], False, self.now()   # ... and the depth then
            self.f_lost_now, self.f_prev_loss, self.f_stuck_t = False, None, None
            self.f_reversed, self.f_hold_pt, self.f_leg_t0, self.f_creep0 = False, None, self.now(), None
            self.f_start, self.f_dir0, self.f_path = (here[0], here[1]), self.f_dir.copy(), []
            if self.aligned and self.align_walk > 0 and getattr(self, 'over_pt', None) is not None:
                # the walk has just seen the pipe here: missing from the start is a lost pipe (search for
                # it), not one that never showed (sim, Oct 25: following began past a stretch's end)
                self.f_line, self.f_seen = np.array(self.over_pt, float), self.now()
            if not self.survey or self.s_leg == 'survey':     # the survey notes lights from the buoy end on
                self.start_camera()
            else:
                self.start_end_watch()                         # ... but can see the pipe's end before that
            self.get_logger().info(f'    following: heading {math.degrees(math.atan2(self.f_dir[1], self.f_dir[0])) % 360:.0f} deg, '
                                   f'at most {self.follow_speed:.2f} m/s and {self.follow_step:.2f} m per sweep')
            self.to_phase('follow')

    def follow_step_once(self, m):
        now = self.now()
        here = self.cur if self.cur else self.f_hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        sonar = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
        if not self.f_path or float(np.linalg.norm(sonar - self.f_path[-1])) >= 0.1:
            self.f_path.append(sonar.copy())                # a lost search leaves this part out
        if self.survey and self.s_leg == 'survey':
            if not self.s_track or float((sonar - self.s_track[-1]) @ self.f_dir) > 0.05:
                self.s_track.append(sonar.copy())          # the way along the pipe: the lights' order
            if self.s_red_new:                             # the camera has just passed the RED light
                self.s_red_new = False
                if self.repair_hold > 0 or self.repair_centre:     # neither: note it, go on (Oct 17)
                    self.repair_begin()
                    return
        # sonar sweep period, for the speed limit
        sw = m.get('sweep', 0)
        if sw != self.f_sweep:
            dt = now - self.f_sweep_t
            if 0.5 < dt < 60:
                self.f_period = 0.6 * self.f_period + 0.4 * dt
            self.f_sweep, self.f_sweep_t = sw, now
        # the height band follows the floor
        band = self.follow_band_now()
        if band and (self.f_band is None or max(abs(band[0] - self.f_band[0]), abs(band[1] - self.f_band[1])) > 0.05) \
                and now - self.f_band_t > 2.0:
            try:
                self.map_post('/band', f'{band[0]} {band[1]}')
                self.f_band, self.f_band_t = band, now
            except OSError:
                pass
        d = self.f_dir
        right = np.array([-d[1], d[0]])
        if m.get('crumbs'):
            raw = np.array(m['crumbs'], float)
            c = np.array([self.map_to_ned(x, y) for x, y in raw[:, :2]])
            up = raw[:, 4]
            keys = [(round(x, 2), round(y, 2), round(z, 2)) for x, y, z in raw[:, [0, 1, 4]]]
            fresh = np.array([k not in self.f_known for k in keys])     # dots the latest sweeps added
            self.f_known.update(keys)
            rel = c - sonar
            along, across = rel @ d, rel @ right
            # only echoes where the pipe should be count: near the line through the last point
            # seen on it (f_line) and at its height (f_up). With the pipe missing from a sweep the
            # wall or the sloping floor was taken for it and pulled the sub off the pipe (pool, Oct
            # 17); now a missing pipe stays missing and the sub goes back to where it was last seen
            if self.f_line is not None and self.follow_gate > 0:
                near = np.abs(across - float((self.f_line - sonar) @ right)) <= self.follow_gate
            else:
                # not seen yet: only almost straight below counts - the first sighting is what the
                # gate then holds on to, and 0.5 m to the side it was the wall (pool, Oct 17)
                near = np.abs(across) <= self.follow_start_gate
            if self.f_up is not None and self.follow_gate_up > 0:
                near &= np.abs(up - self.f_up_now()) <= self.follow_gate_up
            corridor = near & (along >= -self.follow_fit) & (along <= 0.4)
            if corridor.sum() >= 4 and np.ptp(along[corridor]) >= 0.3:
                pts = c[corridor]
                _, _, vt = np.linalg.svd(pts - pts.mean(axis=0))
                nd = vt[0] if vt[0] @ d >= 0 else -vt[0]
                if nd @ d > math.cos(math.radians(35)):          # one fit never swings it far
                    d = 0.6 * d + 0.4 * nd
                    d = d / np.linalg.norm(d)
                    self.f_dir, right = d, np.array([-d[1], d[0]])
            # only FRESH dots say the pipe is under the sonar now; old ones linger behind it
            under = fresh & (np.abs(along) <= 0.2) & (np.abs(across) <= 0.6) & near
            if under.any():
                pipe_up = float(np.median(up[under]))
                self.f_up = pipe_up if self.f_up is None else 0.7 * self.f_up_now() + 0.3 * pipe_up
                self.f_up_z = self.cur[2] if self.cur else self.hold[2]
                self.f_lat = float(np.median((c[under] - sonar) @ right))
                self.f_line = sonar + self.f_lat * right        # a point ON the pipe, fixed in NED
                self.f_lost_now = False
                self.f_seen = now
                # a box sits on the pipe (~0.15 m tall): a fresh dot clearly above the pipe's line
                level = corridor & ~fresh
                if level.sum() >= 5:
                    pipe_up = float(np.median(up[level]))
                    hi = under & (up >= pipe_up + 0.10)
                    if hi.any():
                        k = int(np.argmax(np.where(hi, up, -9)))
                        bn, be = c[k]
                        if self.f_box is None or math.hypot(bn - self.f_box[0], be - self.f_box[1]) > 0.4:
                            self.f_box = (bn, be)
                            self.get_logger().info(f'    possible box: echo {up[k] - pipe_up:.2f} m above the pipe at '
                                                   f'NED ({bn:.2f}, {be:.2f}), {self.f_dist:.1f} m along')
                            if self.recording:
                                try:
                                    self.map_post('/mark', f'possible box {up[k] - pipe_up:.2f} m above the pipe')
                                except OSError:
                                    pass
        # done?
        self.f_dist += max(0.0, float((sonar - self.f_prev) @ d))
        self.f_prev = sonar
        why = None
        seen = self.f_seen if self.f_seen is not None else self.f_leg_t0
        lost = self.f_seen is not None and (now - seen > self.follow_lost
                                            or (self.f_stuck_t is not None and now - self.f_stuck_t > self.follow_lost))
        # the camera sees the pipe's end here (the model, Ruth Oct 25): gone from under the sonar with its end
        # in view, it is the end - no waiting follow_lost_s, no search
        at_end = (self.f_line is not None and (self.f_seen is None or now - self.f_seen > self.f_period)
                  and self.pipe_end_seen() is not None)
        if at_end:
            if self.end_said is None or float(np.linalg.norm(self.f_line - self.end_said)) > 0.5:
                self.end_said = self.f_line.copy()
                msg = f'follow: the pipe has gone from under the sonar and the camera sees its end - the end, {self.f_dist:.1f} m along'
                self.get_logger().info(msg)
                self.mark(msg)
            lost = True
        # on the way to the buoy the pipe ends near it: that end is expected there, so no search
        to_buoy_end = (self.survey and self.s_leg == 'to buoy' and self.f_line is not None
                       and float(np.linalg.norm(np.asarray(self.f_line) - np.asarray(self.s_buoy))) <= BUOY_END_M)
        if lost and not to_buoy_end and not at_end and self.lost_search_begin(now, here):
            return                                         # a bend, or a gap: look for it from there
        if self.survey and self.s_leg == 'to buoy' and lost:
            self.survey_turn(now, here)                    # at the buoy end: the survey starts here
            return
        if ((now - seen > self.follow_lost or at_end) and self.follow_both and not self.f_reversed and not self.survey
                and self.f_dist < self.follow_max and now - self.f_t0 < self.follow_time):
            if self.follow_turn == 'return':
                self.get_logger().info(f'    end of the pipe this way after {self.f_dist:.1f} m - going back to '
                                       f'where following began, then turning round')
                if self.recording:
                    try:
                        self.map_post('/mark', f'end after {self.f_dist:.1f} m: back to the start')
                    except OSError:
                        pass
                self.f_reversed = True
                self.carrot, self.last_t = (here[0], here[1]), now
                self.to_phase('follow return')
                return
            self.get_logger().info(f'    end of the pipe this way after {self.f_dist:.1f} m - turning round '
                                   f'to follow it the other way')
            if self.recording:
                try:
                    self.map_post('/mark', f'turning round after {self.f_dist:.1f} m')
                except OSError:
                    pass
            self.f_dir, self.f_reversed = -self.f_dir, True
            self.f_seen, self.f_leg_t0, self.f_lat, self.f_creep0, self.f_hold_pt = None, now, 0.0, None, None
            self.f_line = None
            self.f_lost_now, self.f_prev_loss, self.f_stuck_t = False, None, None
            return
        if at_end:
            why = 'the camera saw the pipe\'s end'
        elif now - seen > self.follow_lost:
            why = (f'no pipe under the sonar for {self.follow_lost:.0f} s (it ended, or it was lost)'
                   if self.f_seen is not None else f'the pipe never showed under the sonar in {self.follow_lost:.0f} s')
        elif self.f_stuck_t is not None and now - self.f_stuck_t > self.follow_lost:
            why = 'lost the pipe twice at the same spot: its end'
        elif self.f_dist >= self.follow_max:
            why = f'followed {self.follow_max:.1f} m'
        elif now - self.f_t0 > self.follow_time:
            why = f'{self.follow_time:.0f} s of following'
        if why:
            poshold = self.follow_end == 'poshold' and not self.dry
            self.get_logger().info(f'follow done: {why}; {self.f_dist:.1f} m along the pipe - '
                                   + ('switching to POSHOLD, take over with the remote' if poshold else 'holding, then surfacing'))
            self.stop_camera(why)
            if poshold:
                self.end_in_poshold()
                return
            if self.cur and not self.dry:
                self.hold_here(self.cur[2], 'hold')
            self.to_phase('stopped')
            return
        # steer: along the line at v, sliding toward it; heading along it, slowly
        v = min(self.follow_speed, self.follow_step / max(self.f_period, 1.0))
        yaw_d = math.atan2(d[1], d[0])
        rate, err = self.yaw_rate_to(yaw_d)
        waiting = self.f_seen is None or now - self.f_seen > 1.0 * self.f_period or self.f_stuck_t is not None
        if waiting and not self.f_lost_now and self.f_line is not None:
            self.f_lost_now = True
            self.note_loss(now)
        since = now - (self.f_seen if self.f_seen is not None else self.f_leg_t0)
        creep = False
        if abs(err) > math.radians(20) and self.f_seen is None:
            self.f_leg_t0 = now                 # the lost clock starts once it has turned round
        if (self.f_reversed and self.f_seen is None and abs(err) <= math.radians(20)
                and since > 2 * self.f_period):
            # just turned round at an end and the pipe isn't under the sonar yet:
            # creep back along the line, at most 1 m, to find it again. (Not at a gap on
            # the way: creeping on past a real end only overshoots it - sim, Oct 16.)
            if self.f_creep0 is None:
                self.f_creep0 = sonar.copy()
            creep = float(np.linalg.norm(sonar - self.f_creep0)) < 1.0
        if creep:
            v *= 0.5
            self.f_hold_pt = None
            goal = sonar + d * v
            tn, te = goal[0] - ahead * math.cos(h), goal[1] - ahead * math.sin(h)
        elif waiting and self.f_line is not None and abs(err) <= math.radians(20):
            # pipe missing: back, slowly, until the sonar is over where it was last seen, and
            # wait there - not wherever the sub drifted to (Ruth, Oct 16)
            v = 0.0
            self.f_hold_pt = None
            back = (self.f_prev_loss if self.f_stuck_t is not None else self.f_line) - sonar
            dist = float(np.linalg.norm(back))
            goal = sonar + (back / dist * min(dist, self.follow_speed) if dist > 1e-6 else 0.0)
            tn, te = goal[0] - ahead * math.cos(h), goal[1] - ahead * math.sin(h)
        elif waiting or abs(err) > math.radians(20):
            # waiting for the first sighting, or still turning (round): stay on one spot
            v = 0.0
            if self.f_hold_pt is None:
                self.f_hold_pt = (here[0], here[1])
            tn, te = self.f_hold_pt
        else:
            self.f_hold_pt, self.f_creep0 = None, None
            # sideways: to the pipe as last seen, a fixed point - not f_lat from wherever the sonar
            # is now, which slid the sub on past the pipe between 5 s sweeps (pool, Oct 16)
            lat = float((self.f_line - sonar) @ right) if self.f_line is not None else 0.0
            # ... and no faster than follow_speed: GUIDED goes sideways at WPNAV speed (~0.45 m/s),
            # which lunged past the pipe between 5-8 s sweeps (pool, Oct 16)
            lat = max(-self.follow_speed, min(self.follow_speed, lat))
            goal = sonar + lat * right + d * v                       # where the sonar should be in ~1 s
            tn, te = goal[0] - ahead * math.cos(h), goal[1] - ahead * math.sin(h)
        self.follow_depth(now)
        self.stream_posvel(tn, te, self.hold[2], v * d[0], v * d[1], rate, 'follow')
        if self.f_dist - self.f_logged >= 0.5 or now - self.f_log_t >= (2.0 if self.dry else 10.0):
            self.f_logged, self.f_log_t = self.f_dist, now
            self.get_logger().info(
                f'    follow: sonar at NED ({sonar[0]:.2f}, {sonar[1]:.2f}), {self.f_dist:.1f} m along, pipe heading {math.degrees(yaw_d) % 360:.0f} deg '
                f'(sub {math.degrees(h) % 360:.0f}), pipe {abs(self.f_lat):.2f} m '
                f'{"right" if self.f_lat >= 0 else "left"} of the sonar, {v:.2f} m/s, sweep {self.f_period:.1f} s'
                + (f', last seen {now - seen:.0f} s ago' if now - seen > 3 else '')
                + (' (dry run: drive it there)' if self.dry else ''))

    def f_up_now(self):
        """The pipe's height from the sonar (up +) as last measured, moved by how far the sub has gone
        down since: dots are placed relative to the sub at their ping."""
        if self.f_up is None:
            return None
        z = self.cur[2] if self.cur else self.hold[2]
        return self.f_up + (z - (self.f_up_z if self.f_up_z is not None else z))

    def follow_depth(self, now):
        """Keep the pipe follow_pipe_below_m under the sonar (Ruth, Oct 24). Once more than
        follow_pipe_tol_m off, the held depth moves toward it at DZ_SPEED until within a third of
        that (so it doesn't hunt at the edge); never more than DZ_MAX_M from where following began,
        above the mission's surface, or within DZ_MIN_ALT_M of the floor. Only while the pipe is
        being seen: a missing pipe leaves the depth alone."""
        dt, self.f_dz_t = min(now - self.f_dz_t, 0.5), now
        if (self.follow_below <= 0 or self.f_up is None or self.f_seen is None
                or now - self.f_seen > 1.5 * self.f_period):
            self.f_dz_on = False
            return
        err = -self.f_up_now() - self.follow_below           # + : too far above the pipe, go down
        if not self.f_dz_on and abs(err) <= self.follow_below_tol:
            return
        n, e, z, yaw = self.hold
        if not self.f_dz_on:
            self.f_dz_on = True
            self.get_logger().info(f'    follow: the pipe {-self.f_up_now():.2f} m under the sonar - going '
                                   f'{"down" if err > 0 else "up"} to {self.follow_below:.2f}')
        if abs(err) <= self.follow_below_tol / 3:
            self.f_dz_on = False
            self.get_logger().info(f'    follow: the pipe {-self.f_up_now():.2f} m under the sonar, at '
                                   f'{self.below_surface(z):.2f} m down')
            return
        step = max(-DZ_SPEED * dt, min(DZ_SPEED * dt, err))
        alt = self.floor_clearance()
        if step > 0 and alt is not None and alt - step < DZ_MIN_ALT_M:
            step = 0.0
        lo, hi = max(self.f_z0 - DZ_MAX_M, self.z_surf + 0.1), self.f_z0 + DZ_MAX_M     # depth limits, down +
        step = min(0.0, max(step, lo - z)) if step < 0 else max(0.0, min(step, hi - z))
        self.hold = (n, e, z + step, yaw)

    def note_loss(self, now):
        """The pipe just went missing. Missing again within 0.3 m of where it went missing last
        time (it came back there, went on a little, lost it again) is its end (Ruth, Oct 16)."""
        pt = self.f_line.copy()
        if self.f_prev_loss is not None and float(np.linalg.norm(pt - self.f_prev_loss)) <= 0.3:
            if self.f_stuck_t is None:
                self.f_stuck_t = now
                self.get_logger().info(f'    lost the pipe again at the same spot - its end? holding over it '
                                       f'{self.follow_lost:.0f} s')
        else:
            self.f_prev_loss = pt
            self.get_logger().info('    pipe missing - going back over where it was last seen')

    def on_pilot_takeover(self):
        """The pilot took the sub mid-survey: still log and save what it saw so far."""
        if self.survey:
            self.stop_camera('the pilot took over')

    def end_in_poshold(self):
        """Hand the sub over armed and holding: MissionBase sees the mode leave GUIDED and
        ends the mission without disarming, as when the pilot takes over."""
        if self.recording:
            try:
                self.map_post('/record', 'stop')
                self.recording = False
            except OSError:
                pass
        self.link.set_mode(MODE_POSHOLD)
        self.poshold_t = self.now()
        self.to_phase('poshold')

    # ---- the other part, Ruth's way: back to the start, turn 180, back up, follow ----
    def walk_to(self, goal, speed, yaw_d, label, down=None):
        """Carrot walk toward goal (n, e) at speed, turning toward yaw_d, at depth down (the hold's if
        None). Returns metres left."""
        now = self.now()
        dt, self.last_t = min(now - self.last_t, 0.5), now
        cn, ce = self.carrot
        left = math.hypot(goal[0] - cn, goal[1] - ce)
        step = min(speed * dt, left)
        ux, uy = ((goal[0] - cn) / left, (goal[1] - ce) / left) if left > 1e-6 else (0.0, 0.0)
        self.carrot = (cn + ux * step, ce + uy * step)
        v = speed if left > step else 0.0
        rate, err = self.yaw_rate_to(yaw_d)
        self.stream_posvel(self.carrot[0], self.carrot[1], self.hold[2] if down is None else down,
                           ux * v, uy * v, rate, label)
        here = self.cur if self.cur else self.carrot
        return math.hypot(goal[0] - here[0], goal[1] - here[1]), err

    def follow_return(self):
        """Back to where following began, keeping the heading (no turning on the way)."""
        d = self.f_dir0
        left, _ = self.walk_to(self.f_start, self.return_speed, math.atan2(d[1], d[0]), 'back to the start')
        if (left < 0.1 and self.speed() < 0.05) or (self.dry and self.now() - self.state_t0 > 5):
            self.get_logger().info('    back where following began - turning round')
            self.carrot, self.last_t = self.f_start, self.now()
            self.to_phase('follow turn')

    def follow_turn_round(self):
        """Turn 180 deg on the spot, slowly."""
        d = -self.f_dir0
        _, err = self.walk_to(self.f_start, self.return_speed, math.atan2(d[1], d[0]), 'turning round')
        if abs(err) < math.radians(5) or (self.dry and self.now() - self.state_t0 > 5):
            here = self.cur if self.cur else self.f_start
            back = self.survey_backup if self.survey else self.follow_backup
            if self.survey:
                # the camera starts once turned, as it backs up over the end: started before the
                # turn, the spinning picture split one light into two (sim, Oct 17)
                self.start_camera()
            self.f_backup_goal = (self.f_start[0] + back * self.f_dir0[0],
                                  self.f_start[1] + back * self.f_dir0[1])
            self.get_logger().info(f'    turned round - backing up {back:.2f} m so the '
                                   + ('camera starts at the end' if self.survey else 'sonar starts where it began'))
            self.carrot, self.last_t = (here[0], here[1]), self.now()
            self.to_phase('follow backup')

    def follow_back_up(self):
        d = -self.f_dir0
        left, _ = self.walk_to(self.f_backup_goal, self.follow_speed, math.atan2(d[1], d[0]), 'backing up')
        if (left < 0.06 and self.speed() < 0.05) or (self.dry and self.now() - self.state_t0 > 5):
            now = self.now()
            here = self.cur if self.cur else self.f_backup_goal
            h = self.cur_yaw if self.cur_yaw is not None else math.atan2(d[1], d[0])
            ahead = sonar_settings.SONAR_FWD_M
            self.f_dir = d.copy()
            self.f_seen, self.f_leg_t0, self.f_lat, self.f_creep0, self.f_hold_pt = None, now, 0.0, None, None
            self.f_line = None
            self.f_lost_now, self.f_prev_loss, self.f_stuck_t = False, None, None
            self.f_prev = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
            if self.survey:                     # the survey's own distance and clock; f_reversed: if the
                self.f_dist, self.f_t0, self.f_reversed = 0.0, now, True   # pipe isn't under the sonar, creep to it
            self.get_logger().info(f'    following the other part: heading {math.degrees(math.atan2(d[1], d[0])) % 360:.0f} deg')
            if self.recording:
                try:
                    self.map_post('/mark', 'following the other part')
                except OSError:
                    pass
            self.to_phase('follow')

    # ---- light boxes in the bottom camera (log only) ----
    def camera_at(self):
        if not self.cur or self.cur_yaw is None:
            return None
        return (self.cur[0] + self.camera_fwd * math.cos(self.cur_yaw),
                self.cur[1] + self.camera_fwd * math.sin(self.cur_yaw))

    def on_box(self, b):
        """The camera thread: a sighting of a colour has just ended."""
        n = len(self.camera.tracker.boxes) if self.camera else 0
        if self.survey:
            self.get_logger().info(f'    camera: {"NEW " if b["new"] else "again: "}{led_boxes.describe(b)}'
                                   .replace('box', 'light') + f' - {n} sighting(s) so far')
            if b['new'] and b['colour'] == 'red' and self.s_red is None and b['n'] is not None:
                self.s_red = (b['n'], b['e'])
                self.s_red_new = True                      # the mission thread goes back to it
        else:
            self.get_logger().info(f'    camera: {"NEW " if b["new"] else "again: "}{led_boxes.describe(b)} '
                                   f'- {n} of {self.boxes_expected} boxes so far')
        if self.recording and b['new']:
            try:
                self.map_post('/mark', f'camera {"light" if self.survey else "box"} {b["number"]}: '
                              + max(b['colours'], key=b['colours'].get))
            except OSError:
                pass

    def yolo_lights(self):
        """The YOLO model, loaded once: light_detector:=yolo's light_model (lights and pipe ends), else
        end_model (pipe ends only). If it can't be loaded (no file, no ultralytics): lights are judged
        against the water (contrast) and the pipe's ends come from the sonar, as without one."""
        path = self.light_model if self.light_detector == 'yolo' else self.end_model
        if self.yolo is None and path and not self.yolo_failed:
            try:
                self.yolo = led_boxes.YoloLights(path, self.light_conf, self.end_conf)
                self.get_logger().info(f'    camera: the model {path} - '
                                       + ('lights and pipe ends' if self.light_detector == 'yolo' else 'pipe ends only'))
            except Exception as ex:
                self.yolo_failed = True
                self.get_logger().error(f'    camera: the model {path!r} would not load ({ex}) - '
                                        f'lights against the water, pipe ends from the sonar')
                if self.light_detector == 'yolo':
                    self.light_detector = 'contrast'
        return self.yolo

    def start_end_watch(self):
        """A YOLO model (light_detector:=yolo or end_model): the camera on for pipe ends only (no lights
        counted) - on the line-up walk and on the survey's way to the buoy end, where the light list
        mustn't start yet (Ruth, Oct 25)."""
        if self.camera is None and self.yolo_lights() is not None:
            self.camera = led_boxes.CameraWatcher(self.camera_url, self.camera_at,
                                                  lambda t: self.get_logger().info('    ' + t), None,
                                                  tracker=led_boxes.NoTracker(), yolo=self.yolo).start()
            self.cam_ends_only = True
            self.get_logger().info('    camera: watching for the pipe\'s end (the model)')

    def start_camera(self, force=False):
        if self.camera is not None and self.cam_ends_only and (force or self.camera_boxes or self.survey):
            # the pipe-end watch already runs: from here it counts the lights too
            self.camera.tracker = (led_boxes.LightTracker(same_m=self.light_same, new_frac=self.light_min_share,
                                                          min_frac=self.light_min_frac, detector=self.light_detector,
                                                          yolo=self.yolo)
                                   if self.survey else led_boxes.BoxTracker())
            self.camera.on_box, self.cam_ends_only = self.on_box, False
            self.get_logger().info('    camera: counting the lights from here')
            return
        if (force or self.camera_boxes or self.survey) and self.camera is None:
            self.yolo_lights()
            tracker = (led_boxes.LightTracker(same_m=self.light_same, new_frac=self.light_min_share,
                                              min_frac=self.light_min_frac, detector=self.light_detector,
                                              yolo=self.yolo)
                       if self.survey else None)
            self.get_logger().info('    camera: lights judged against the water (contrast_lights)'
                                   if self.light_detector == 'contrast' else
                                   '    camera: lights from the trained model' if self.light_detector == 'yolo' else
                                   f'    camera: HSV - V >= {led_boxes.V_MIN}, S >= {led_boxes.S_MIN}, hues '
                                   f'{led_boxes.SURVEY_HUES if self.survey else led_boxes.HUES}')
            self.camera = led_boxes.CameraWatcher(self.camera_url, self.camera_at,
                                                  lambda t: self.get_logger().info('    ' + t), self.on_box,
                                                  tracker=tracker, yolo=self.yolo).start()

    def stop_camera(self, why='the mission ended early'):
        if self.camera is not None and self.cam_ends_only:      # only watching for ends: nothing to report
            self.camera.close()
            self.camera, self.cam_ends_only = None, False
        if self.camera is None:
            if self.survey and self.s_leg == 'to buoy':
                self.get_logger().warn(f'survey: no report - it never reached the buoy end ({why})')
                self.s_leg = None
            return
        boxes = self.camera.close()
        self.camera = None
        if self.survey:
            self.survey_report(boxes, why)
            return
        self.get_logger().info(f'camera: {len(boxes)} of {self.boxes_expected} light boxes seen'
                               + ''.join(f'\n      {led_boxes.describe(b)}' for b in boxes))

    # ---- the survey (Ruth, Oct 17): to the buoy end, then every light in order ----
    def survey_begin(self):
        """Following begins: first along the pipe toward the green buoy, to the end nearest
        it - the lights are noted from that end on (handbook: the report starts with the
        part of the pipeline closest to the active buoy)."""
        self.s_red, self.s_red_new, self.s_repair_mono, self.s_track = None, False, None, []
        self.s_turn_first = False
        if self.survey_buoy == 'here':
            self.s_leg = 'survey'
            self.get_logger().info('survey: starting at the buoy end - noting the lights from here')
            return
        # the origin is the green buoy (Ruth, Oct 17): where the mission started, or where it was marked
        bn, be = self.marked_buoy if self.survey_buoy == 'marked' else (self.start_x, self.start_y)
        self.s_buoy = (bn, be)
        here = self.cur if self.cur else self.hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        to_buoy = np.array([bn - here[0] - ahead * math.cos(h), be - here[1] - ahead * math.sin(h)])
        where = 'where the mission started' if self.survey_buoy == 'start' else 'where it was marked'
        if np.linalg.norm(to_buoy) < 0.5:
            # begun over the pipe right at the buoy: the pipe's nearest part to it is here
            self.s_leg = 'survey'
            self.get_logger().info(f'survey: the green buoy ({where}) is right here - noting the lights '
                                   f'from here, along the pipe the way the sub faces')
            return
        if float(to_buoy @ self.f_dir) < 0:
            self.f_dir = -self.f_dir                       # the way along the pipe that gets closer to it
            self.s_turn_first = True
        self.s_leg = 'to buoy'
        self.get_logger().info(f'survey: the green buoy ({where}) is {np.linalg.norm(to_buoy):.1f} m away - '
                               f'first along the pipe the way that gets closer to it, heading '
                               f'{math.degrees(math.atan2(self.f_dir[1], self.f_dir[0])) % 360:.0f} deg, to its end')

    def survey_turn(self, now, here):
        """The end nearest the buoy: turn 180 on the spot and back up (follow turn, follow
        backup - the way the other part is followed), then survey from here. Turning while
        following made the pipe come and go under the swinging sonar, which read as 'lost
        twice at the same spot' and ended the survey after 1 m (sim, Oct 17)."""
        self.get_logger().info(f'survey: at the buoy end after {self.f_dist:.1f} m - turning round; '
                               f'noting the lights from here to the far end')
        if self.recording:
            try:
                self.map_post('/mark', 'survey: at the buoy end, turning round')
            except OSError:
                pass
        self.f_start, self.f_dir0 = (here[0], here[1]), self.f_dir.copy()
        self.s_leg, self.s_track = 'survey', []
        self.carrot, self.last_t = (here[0], here[1]), now
        self.to_phase('follow turn')

    def repair_begin(self):
        """The camera has passed the RED light: go back until the camera is over it, and hold."""
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        rn, re = self.s_red
        self.s_repair_goal = (rn - self.camera_fwd * math.cos(h), re - self.camera_fwd * math.sin(h))
        self.s_repair_yaw = h
        here = self.cur if self.cur else self.s_repair_goal
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        self.get_logger().info(f'survey: RED light at NED ({rn:.2f}, {re:.2f}) - going back over it')
        self.to_phase('repair go')

    def repair_go(self):
        left, _ = self.walk_to(self.s_repair_goal, self.follow_speed, self.s_repair_yaw, 'to the red light')
        if (left < 0.06 and self.speed() < 0.05) or (self.dry and self.now() - self.state_t0 > 5):
            self.s_repair_mono = time.monotonic()          # the camera's clock: sightings after this
            if self.repair_centre:
                self.centre_begin('survey: over the RED light - centring it on the magnet')
                return
            self.get_logger().info(f'survey: over the RED light - holding {self.repair_hold:.0f} s '
                                   f'(stands in for the magnet repair)')
            if self.recording:
                try:
                    self.map_post('/mark', 'survey: holding over the red light')
                except OSError:
                    pass
            self.to_phase('repair hold')

    def repair_hold_step(self):
        self.walk_to(self.s_repair_goal, self.follow_speed, self.s_repair_yaw, 'over the red light')
        if self.now() - self.state_t0 < self.repair_hold:
            return
        self.repair_done()

    def repair_done(self):
        """On along the pipe from where the repair (or its stand-in hold) ended."""
        now = self.now()
        here = self.cur if self.cur else self.s_repair_goal
        h = self.cur_yaw if self.cur_yaw is not None else self.s_repair_yaw
        ahead = sonar_settings.SONAR_FWD_M
        # on along the pipe from here: count as seen just now, so the hold isn't taken for the end
        self.f_seen, self.f_leg_t0, self.f_hold_pt, self.f_creep0 = now, now, None, None
        self.f_lost_now, self.f_prev_loss, self.f_stuck_t = False, None, None
        self.f_prev = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
        self.get_logger().info('survey: repair done - on along the pipe to the far end')
        self.to_phase('follow')

    # ---- the repair: centre the RED light on the magnet, step down to it (Ruth, Oct 22) ----
    def centre_begin(self, why):
        """The magnet is in the down camera's view, so lining the light up with the magnet's spot in the
        picture puts the light on the magnet's line of sight; that is directly under the magnet once the
        magnet reaches it. So: line up, step down, line up again, until the light turns GREEN."""
        self.start_camera(force=True)
        here = self.cur if self.cur else self.hold[:3]
        self.c_yaw = self.cur_yaw if self.cur_yaw is not None else self.hold[3]
        self.c_top = self.c_down = here[2]                # Cube z where centring began / asked for now
        self.c_goal = (here[0], here[1])
        self.c_frame, self.c_frame_t, self.c_seen, self.c_log, self.c_lost_said = None, self.now(), self.now(), 0.0, False
        self.c_errs, self.c_ok_since = [], self.now()   # recent offsets (px); since the last step (or the start)
        self.c_green_n = 0                              # contrast: pictures running the light has read green
        self.c_last, self.c_first = None, True          # the light's last place in the picture; no guess yet
        self.c_dist = 0.5                               # the light's distance below the camera, m (until a picture)
        self.carrot, self.last_t = (here[0], here[1]), self.now()
        lb = self.light_below(self.floor_clearance())
        self.get_logger().info(f'{why}: the light to ({self.magnet[0]:.0f}, {self.magnet[1]:.0f}) px, '
                               f'{self.centre_step_m * 100:.0f} cm steps down ({self.centre_first_step * 100:.0f} cm while far), '
                               f'at most {self.centre_max_down:.2f} m'
                               + (f'; the light ~{lb:.2f} m below the camera' if lb is not None else ''))
        self.to_phase('centre')

    def centre_step(self):
        now = self.now()
        cam = self.camera.latest if self.camera else None
        if cam is not None and cam[0] != self.c_frame:
            self.c_frame, self.c_frame_t = cam[0], now
            fr = self.light_fractions(cam[1])
            if fr['green'] >= GREEN_DONE and fr['green'] > fr['red']:
                self.get_logger().info(f'repair: the light turned GREEN ('
                                       + (f'{C_GREEN_FRAMES} pictures running' if self.light_detector != 'hsv'
                                          else f'{fr["green"]:.0%} of the picture') + ') at '
                                       f'{self.below_surface(self.cur[2]) if self.cur else float("nan"):.2f} m down')
                self.watch_begin()
                return
            spot = self.light_where(cam[1]) if fr['red'] >= RED_SEEN else None
            if spot is None and fr['red'] >= RED_SEEN and self.c_last is not None and \
                    math.hypot(self.c_last[0] - self.magnet[0], self.c_last[1] - self.magnet[1]) <= self.centre_tol:
                spot = (self.c_last, 0.0)       # its core just went behind the magnet (about as big as the ring)
            if spot is not None:
                (u, v), _ = spot
                self.c_last = (u, v)
                self.c_seen, self.c_lost_said = now, False
                du, dv = u - self.magnet[0], v - self.magnet[1]
                alt = self.floor_clearance()
                lb = self.light_below(alt)
                dist = self.c_dist = min(max(lb if lb is not None else 0.4, 0.1), 1.5)
                # picture top = front, right = right: a light right of the magnet means go right, one below
                # it (behind) means go back. Each picture says where the sub should be (a fixed point, as
                # the light is); the guesses are smoothed and walked to. Stepping part of the way from
                # wherever the sub was instead chased its own lag round in circles (sim, Oct 22)
                right, fwd = du * dist / self.cam_f, -dv * dist / self.cam_f
                h = self.cur_yaw if self.cur_yaw is not None else self.c_yaw
                n0, e0 = (self.cur[0], self.cur[1]) if self.cur else self.c_goal
                est = (n0 + fwd * math.cos(h) - right * math.sin(h), e0 + fwd * math.sin(h) + right * math.cos(h))
                g, self.c_first = (1.0 if self.c_first else CENTRE_GAIN), False
                self.c_goal = (self.c_goal[0] + g * (est[0] - self.c_goal[0]), self.c_goal[1] + g * (est[1] - self.c_goal[1]))
                # lined up: the median offset over the last centre_settle_s, as the picture jitters
                # with the sub's rocking (sim: +-50 px picture to picture)
                self.c_errs = [(t, e) for t, e in self.c_errs if now - t <= self.centre_settle] + [(now, math.hypot(du, dv))]
                if (now - self.c_ok_since >= self.centre_settle
                        and float(np.median([e for _, e in self.c_errs])) <= self.centre_tol):
                    self.c_errs, self.c_ok_since = [], now
                    self.centre_lower(alt)
                if now - self.c_log >= 2.0:
                    self.c_log = now
                    self.get_logger().info(f'    centre: light {du:+.0f} px right, {dv:+.0f} px below the magnet '
                                           f'(~{fwd * 100:+.0f} cm ahead, {right * 100:+.0f} cm right), '
                                           f'{self.below_surface(self.c_down):.2f} m down, floor '
                                           + (f'{alt:.2f} m' if alt is not None else 'unknown'))
            else:
                if now - self.c_seen > 3.0 and not self.c_lost_said:
                    self.c_lost_said = True
                    self.get_logger().warn('    centre: no red light in the picture for 3 s - going on to where it was')
                if now - self.c_log >= 2.0:
                    self.c_log = now
                    self.get_logger().info(f'    centre: no light core (red {fr["red"]:.0%}, green {fr["green"]:.0%})')
        elif now - self.c_frame_t > 2.0 and now - self.c_log >= 2.0:
            self.c_log = now
            self.get_logger().warn(f'    centre: no new camera picture for {now - self.c_frame_t:.0f} s '
                                   f'({self.camera.frames if self.camera else 0} so far)')
        if now - self.state_t0 > self.centre_time:
            self.get_logger().warn(f'repair: no GREEN after {self.centre_time:.0f} s - giving up')
            self.centre_end()
            return
        # slower close up, where the same move shifts the light further across the picture (Ruth, Oct 24)
        speed = self.centre_speed * min(1.0, max(0.4, self.c_dist / 0.5))
        self.walk_to(self.c_goal, speed, self.c_yaw, 'centre the light on the magnet', down=self.c_down)

    def light_fractions(self, rgb):
        """The repair's {'red', 'green'}: shares of the picture (hsv), or (contrast) 1.0 for the colour of
        the biggest light judged against the water - green only once C_GREEN_FRAMES pictures running say
        so, and any other light (a red LED close up reads yellow-orange, Oct 24) still the RED light."""
        if self.light_detector == 'hsv':
            return led_boxes.red_green_fractions(rgb)
        colour, _ = self.biggest_light(rgb)
        self.c_green_n = self.c_green_n + 1 if colour == 'green' else 0
        return {'green': 1.0 if self.c_green_n >= C_GREEN_FRAMES else 0.0,
                'red': 1.0 if colour not in (None, 'green') else 0.0}

    def lights_now(self, rgb):
        """The lights in the newest picture: the trained model's on the full picture (light_detector:=yolo;
        the camera thread's finds when fresh), else judged against the water on the small one."""
        cam = self.camera
        if self.light_detector == 'yolo' and self.yolo is not None and cam is not None:
            if cam.latest_dets is not None and time.monotonic() - cam.latest_dets[0] < 0.5:
                dets = cam.latest_dets[1]
            elif cam.latest_full is not None:
                dets = self.yolo(cam.latest_full)
            else:
                return []
            return [d for d in dets if d['colour'] in ('red', 'green')]
        return led_boxes.contrast_lights(rgb)

    def biggest_light(self, rgb):
        """(colour, share) of the biggest light in the newest picture (lights_now)."""
        return led_boxes.biggest_colour(self.lights_now(rgb))

    def light_where(self, rgb):
        """Where the repair's light is: ((x, y) in 640x360 pixels, share). hsv: its bright core
        (light_spot); contrast/yolo: the biggest light those judged - in the sunlit pool the brightest
        patch was the white pipe beside the box, not the LED ring (Oct 25)."""
        if self.light_detector == 'hsv':
            return led_boxes.light_spot(rgb)
        b = led_boxes.biggest_light(self.lights_now(rgb))
        return (((b['x'] + 1) * 320, (b['y'] + 1) * 180), b['share']) if b else None

    def pipe_end_seen(self, ahead=None):
        """light_detector:=yolo: the pipe's end in the down camera within END_SEEN_S, near the middle across
        (END_MID_X) - ahead (top half of the picture) or under the camera with ahead=None/False, only ahead
        with ahead=True - or None. The picture's top is the sub's front."""
        cam = self.camera
        if self.yolo is None or cam is None or cam.end_seen is None:
            return None
        t, d = cam.end_seen
        if time.monotonic() - t > END_SEEN_S or d['conf'] < self.end_conf or abs(d['x']) > END_MID_X:
            return None
        if d['y'] > (-0.2 if ahead else END_BEHIND_Y):
            return None
        return d

    def light_below(self, alt):
        """How far the light's top is below the down camera now: from the sonar's pipe height when
        following found one (the camera is at about the sonar's level), else from the DVL altitude
        less light_above_floor_m; None if neither."""
        if self.f_up is not None:
            return -self.f_up_now() - BOX_H_M
        return (alt - self.light_above_floor) if alt is not None else None

    def centre_lower(self, alt):
        """Lined up long enough: one step down - centre_first_step_m while the magnet is still that and
        0.1 m more above the light (Ruth, Oct 24: less time far off), else centre_step_m - but never
        past centre_max_down_m or near the floor."""
        lb = self.light_below(alt)
        gap = None if lb is None else lb - MAGNET_BELOW_M        # the magnet to the light's top
        step = self.centre_first_step if gap is not None and gap >= self.centre_first_step + 0.1 else self.centre_step_m
        lower = min(self.c_down + step, self.c_top + self.centre_max_down)
        if alt is not None and alt - (lower - self.c_down) < self.centre_min_alt:
            lower = self.c_down
        if lower > self.c_down + 1e-3:
            moved, self.c_down = lower - self.c_down, lower
            self.get_logger().info(f'    centre: lined up - down {moved * 100:.0f} cm to {self.below_surface(lower):.2f} m'
                                   + (f' (the magnet was ~{gap:.2f} m above the light)' if gap is not None else ''))
        else:
            self.get_logger().info('    centre: lined up, but at the lowest it may go - holding')

    def watch_begin(self):
        """GREEN: rise a few cm and watch the light's colours (the sequence reader comes later)."""
        here = self.cur if self.cur else (self.c_goal[0], self.c_goal[1], self.c_down)
        self.w_hold = (here[0], here[1], max(here[2] - self.watch_rise, self.z_surf))
        self.w_frame, self.w_colour, self.w_seq = None, None, []
        self.get_logger().info(f'    watch: up {self.watch_rise * 100:.0f} cm, watching the colours for {self.watch_s:.0f} s')
        self.to_phase('watch')

    def watch_step(self):
        self.goto_ned(*self.w_hold, self.c_yaw, 'watch the light')
        cam = self.camera.latest if self.camera else None
        if cam is not None and cam[0] != self.w_frame:
            self.w_frame = cam[0]
            if self.light_detector != 'hsv':
                c, share = self.biggest_light(cam[1])
                c = c or 'off'
            else:
                fr = led_boxes.colour_fractions(cam[1])
                c, share = max(fr.items(), key=lambda kv: kv[1])
                c = c if share >= WATCH_MIN_SHARE else 'off'
            if c != self.w_colour:
                self.w_colour = c
                t = self.now() - self.state_t0
                self.w_seq.append((round(t, 1), c))
                self.get_logger().info(f'    watch: {t:5.1f} s {c.upper()}' + (f' ({share:.0%})' if c != 'off' else ''))
        if self.now() - self.state_t0 >= self.watch_s:
            self.get_logger().info('repair: colours seen ' + ' '.join(f'{t}s:{c}' for t, c in self.w_seq))
            self.centre_end()

    def centre_end(self):
        if self.start_with == 'centre':
            self.stop_camera('centring done')
            self.stop_here('centre')
        else:
            self.repair_done()

    def survey_report(self, boxes, why):
        """The lights in order from the buoy end: each sighting placed on the way the sub
        went (s_track), sightings within light_same_m of each other are one light, and its
        colour is the one it showed most. Anything seen at the RED light after the repair
        began (it turns GREEN, then flashes) is not a light of the survey."""
        self.s_leg = None
        track = np.array(self.s_track) if len(self.s_track) >= 2 else None

        def along(n, e):
            if track is None:
                return 0.0
            p, best, s = np.array([n, e]), None, 0.0
            for k in range(len(track) - 1):
                a, b = track[k], track[k + 1]
                ab = b - a
                L = float(np.linalg.norm(ab))
                if L < 1e-6:
                    continue
                lo = -math.inf if k == 0 else 0.0          # before the way's start or past its end:
                hi = math.inf if k == len(track) - 2 else 1.0   # on along the first or last piece
                t = min(max(float((p - a) @ ab) / (L * L), lo), hi)
                dist = float(np.linalg.norm(a + t * ab - p))
                if best is None or dist < best[0]:
                    best = (dist, s + t * L)
                s += L
            return best[1] if best else 0.0

        seen = []
        for b in boxes:
            if b['n'] is None:
                continue
            if (self.s_red is not None and self.s_repair_mono is not None and b['first_t'] >= self.s_repair_mono
                    and math.hypot(b['n'] - self.s_red[0], b['e'] - self.s_red[1]) <= 0.4):
                continue                                   # the repaired light changing colour
            seen.append({'colour': b['colour'], 'share': max(b['colours'].values()), 'n': b['n'], 'e': b['e'],
                          'sightings': b['sightings']})
        lights = []
        for s in sorted(seen, key=lambda s: -s['share']):
            near = [L for L in lights if math.hypot(L['n'] - s['n'], L['e'] - s['e']) <= self.light_same]
            if near:
                near[0]['sightings'] += s['sightings']
                near[0]['shares'][s['colour']] = max(near[0]['shares'].get(s['colour'], 0.0), s['share'])
            else:
                lights.append({'n': s['n'], 'e': s['e'], 'sightings': s['sightings'], 'shares': {s['colour']: s['share']}})
        for L in lights:
            L['colour'] = max(L['shares'], key=L['shares'].get).upper()
            L['along_m'] = round(along(L['n'], L['e']), 2)
        lights.sort(key=lambda L: L['along_m'])
        seq = [L['colour'] for L in lights]
        # pipe surveyed: the way's length (f_dist counts again what it re-covers after backing up)
        surveyed = float(np.linalg.norm(np.diff(track, axis=0), axis=1).sum()) if track is not None else 0.0
        self.get_logger().info(
            f'SURVEY REPORT, from the buoy end: {", ".join(seq) if seq else "no lights seen"} '
            f'({len(seq)} light(s), {surveyed:.1f} m of pipe surveyed; ended: {why})'
            + ''.join(f'\n      {k + 1}. {L["colour"]:5s} {L["along_m"]:.2f} m along, NED ({L["n"]:.2f}, {L["e"]:.2f}), '
                      f'{L["sightings"]} sighting(s), ' + ', '.join(f'{c} {v:.0%}' for c, v in L['shares'].items())
                      for k, L in enumerate(lights)))
        report = {'order_from': 'buoy end', 'sequence': seq, 'ended': why,
                  'surveyed_m': round(surveyed, 2), 'repair_held': self.s_repair_mono is not None,
                  'red_light_ned': list(self.s_red) if self.s_red else None,
                  'lights': [{'colour': L['colour'], 'along_m': L['along_m'], 'ned': [round(L['n'], 3), round(L['e'], 3)],
                              'sightings': L['sightings'], 'peak_share': {c: round(v, 3) for c, v in L['shares'].items()}}
                             for L in lights]}
        try:
            os.makedirs(self.survey_dir, exist_ok=True)
            path = os.path.join(self.survey_dir, time.strftime('survey_%Y%m%d_%H%M%S.json'))
            with open(path, 'w') as f:
                json.dump(report, f, indent=1)
            self.get_logger().info(f'    survey report written to {path}')
        except OSError as ex:
            self.get_logger().warn(f'    survey report not written: {ex}')
        if self.recording:
            try:
                self.map_post('/mark', 'survey report: ' + (', '.join(seq) if seq else 'no lights'))
            except OSError:
                pass

    def look_again(self, why):
        """Nothing picked: turn retry_turn_deg more and look again (the map keeps every
        dot so far), until scan_time_s is used up (Ruth, Oct 15)."""
        used = self.now() - getattr(self, 'scan_t0', self.now())
        if self.scan_mode != 'turn' or used >= self.scan_time:
            self.get_logger().warn(f'pool scan: {why}; searched {used:.0f} s - surfacing')
            self.enter(S.SURFACE)
            return
        self.turn_no += 1
        self.get_logger().info(f'pool scan: {why} after {used:.0f} s - turning another '
                               f'{math.degrees(self.retry_turn):.0f} deg (turn {self.turn_no})'
                               + (' (dry run: the pilot turns the sub)' if self.dry else ''))
        self.turned, self.last_yaw, self.turn_goal = 0.0, self.cur_yaw, self.retry_turn
        self.to_phase('pool scan: turn')            # still holding the spot the scan began at

    def retarget(self, m):
        """Aim at the crumb along the picked outline nearest the sonar. New crumbs
        keep landing on the way; the aim moves only if they move it 0.15 m."""
        if not m.get('crumbs') or not m.get('sonar'):
            return self.aim is not None
        c = np.array(m['crumbs'], float)[:, :2]
        on = outlines.dist_to_polyline(c, self.pipe_spine) < 0.4
        if not on.any():
            return self.aim is not None
        sx, sy = m['sonar']
        best = c[on][np.argmin(np.hypot(c[on, 0] - sx, c[on, 1] - sy))]
        if self.aim is None or math.dist(best, self.aim) > 0.15:
            self.aim = best
            self.post_point(best)
        return True

    def approach(self, m):
        if self.now() - self.retarget_t > 2.0:
            self.retarget(m)
            self.retarget_t = self.now()
        tn, te = self.map_to_ned(*self.aim)
        yaw = self.approach_yaw
        fwd, right = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
        ahead = sonar_settings.SONAR_FWD_M
        # stand where the SONAR is approach_range from the aim, with the aim straight to its pool_side
        gn = tn - self.side * self.approach_range * right[0] - ahead * fwd[0]
        ge = te - self.side * self.approach_range * right[1] - ahead * fwd[1]
        # the carrot walks toward that spot at approach_speed; the sub follows it
        dt, self.last_t = min(self.now() - self.last_t, 0.5), self.now()
        cn, ce = self.carrot
        left = math.hypot(gn - cn, ge - ce)
        step = min(self.approach_speed * dt, left)
        ux, uy = ((gn - cn) / left, (ge - ce) / left) if left > 1e-6 else (0.0, 0.0)
        self.carrot = (cn + ux * step, ce + uy * step)
        v = self.approach_speed if left > step else 0.0
        self.stream_posvel(self.carrot[0], self.carrot[1], self.hold[2], ux * v, uy * v, 0.0, 'approach')
        if self.dry:
            if self.now() - self.state_t0 > 5:
                self.to_phase('at pipe')
            return
        h = self.cur_yaw if self.cur_yaw is not None else yaw
        sn, se = self.cur[0] + ahead * math.cos(h), self.cur[1] + ahead * math.sin(h)
        gap = math.hypot(tn - sn, te - se)
        # stop AT approach_range, not anywhere inside it: starting closer than that (the
        # sim did, 0.87 m) the spot to stand on is farther out, so the sub backs away
        if abs(gap - self.approach_range) <= 0.05 or (left < 0.02 and self.speed() < 0.05):
            self.get_logger().info(f'at the pipe: sonar {gap:.2f} m from it (aim {self.approach_range} m)'
                                   f' - holding {self.hold_s:.0f} s')
            self.hold_here(self.cur[2], 'hold')
            self.to_phase('at pipe')

    def do_maneuver(self):
        self.enter(S.SURFACE)


def main():
    rclpy.init()
    rclpy.spin(PipelineMission())


if __name__ == '__main__':
    main()
