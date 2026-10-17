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

start_with:=follow skips the scan: the pilot leaves the sub over the pipe, facing
along it, and following starts there.

stop_after:=scan|pick|face|approach|over|spots ends the run after that stage (hold, then
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
import urllib.request

import numpy as np
import rclpy
from std_msgs.msg import Float32

from robotx_graey_2026.api.sonar import settings as sonar_settings

from robotx_graey_2026.api.navigation import mission_base
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S
from robotx_graey_2026.api.sonar import outlines
from robotx_graey_2026.api.vision import led_boxes

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

    def declare_extra_parameters(self):
        p = self.declare_parameter
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
        p('pool_range', 6.0)
        p('pool_sector', '140 210')         # Ruth, Oct 14: the left side, a little up and down
        p('pool_side', 'left')              # where the sonar looks: 'left' or 'right'
        p('pool_pick_any', False)           # pick a shape on any side, not only pool_side's
                                            # (Ruth, Oct 15: look everywhere); face still puts it there
        p('pool_band', '-0.3 0.4')          # m from the sonar; the pipe is about level with it.
                                            # 'off' = every height
        p('pool_compact', True)             # little dots only: drops the pool wall (Ruth, Oct 14:
                                            # the tall columns 1.2-1.8 m left were the wall)
        p('pool_near_m', 0.0)               # no crumbs closer than this, m (0 = the blind zone
                                            # only); 1.05 skips the ringing if the pipe is farther
        p('pool_far_m', 2.5)                # no crumbs farther than this (0 = no limit): in the pool
                                            # the wrong dots were mostly 3-5.6 m out (Oct 14)
        p('scan_mode', 'turn')              # 'turn': one slow turn; 'stops': stop and look
                                            # (stops shelved Oct 15: the sub slid at each heading jump)
        p('scan_time_s', 300.0)             # pool: give up searching (turns + picks) after this
        p('retry_turn_deg', 180.0)          # pool: nothing picked -> turn this much more and look again
        p('scan_stops', 20)                 # stops: this many headings round scan_turn_deg
        p('sweeps_per_stop', 3)             # stops: full sweeps taken still at each (1-5)
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
        p('follow_lost_s', 30.0)            # no pipe under the sonar this long: lost or ended
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

    def read_extra_parameters(self):
        g = lambda n: self.get_parameter(n).value
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
        if self.stop_after not in ('', 'scan', 'pick', 'face', 'approach', 'over', 'spots'):
            raise ValueError(f"stop_after must be scan, pick, face, approach, over or spots, not {self.stop_after!r}")
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
        if self.start_with not in ('scan', 'follow'):
            raise ValueError(f"start_with must be scan or follow, not {self.start_with!r}")
        self.side = 1.0 if g('pool_side').strip().lower() == 'right' else -1.0
        self.pool_pick_any = bool(g('pool_pick_any'))
        band = g('pool_band').strip().lower()
        self.pool_band = 'off' if band in ('', 'off', 'none') else band
        self.pool_compact = bool(g('pool_compact'))
        self.pool_near = float(g('pool_near_m'))
        self.pool_far = float(g('pool_far_m'))
        self.scan_mode = g('scan_mode').strip().lower()
        self.scan_time = float(g('scan_time_s'))
        self.retry_turn = math.radians(g('retry_turn_deg'))
        if self.scan_mode not in ('turn', 'stops'):
            raise ValueError(f"scan_mode must be turn or stops, not {self.scan_mode!r}")
        self.scan_stops = max(1, int(g('scan_stops')))
        self.sweeps_per_stop = min(5, max(1, int(g('sweeps_per_stop'))))
        self.record_scan = bool(g('record_scan'))
        self.pool_max_pick = float(g('pool_max_pick_m'))
        if self.plan == 'pool':
            self.depth = g('pool_depth')
            self.max_depth = g('pool_max_depth')
        self.scans = {
            'far': (self.far_range, self.scan_sector, self.far_threshold, self.floor_cut),
            'close': (self.close_range, self.scan_sector, self.scan_threshold, self.floor_cut),
            'pool': (g('pool_range'), g('pool_sector'), self.scan_threshold, g('pool_floor_cut'))}

    # ---------- depth from the surface ----------
    def enter(self, s):
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
    def to_phase(self, name):
        self.get_logger().info(f'    phase: {name}')
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
            if self.plan == 'pool' and self.start_with == 'follow':
                self.start_follow_here()
                return
            self.start_scan('pool' if self.plan == 'pool' else 'far')
        ph = self.phase.split(': ')[-1]

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
            for cmd in ('reset', 'start', 'resume'):    # whatever state it was in
                self.map_post('/mapcmd', cmd)
            if self.record_scan:
                self.map_post('/record', f'start {self.scan} scan {self.scan_mode}' +
                              (f' {self.scan_stops}x{self.sweeps_per_stop}' if self.scan_mode == 'stops' else ''))
                self.recording = True
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
                if self.scan_mode == 'stops':
                    self.stop_i, self.stop_yaw0 = 0, (self.cur_yaw if self.cur_yaw is not None else self.hold[3])
                    self.to_phase(f'{self.scan} scan: stop turn')
                else:
                    self.to_phase(f'{self.scan} scan: turn')

        elif ph == 'stop turn':
            # stop i looks at heading yaw0 + i * scan_turn / scan_stops (the first one where it is)
            step = self.scan_turn / self.scan_stops
            want = self.stop_yaw0 + self.stop_i * step
            n, e, down, _ = self.hold
            self.goto_ned(n, e, down, want, f'stop {self.stop_i + 1}')
            if self.dry and self.now() - self.state_t0 < 0.3:
                self.get_logger().info(f'    dry run: turn the sub to heading {math.degrees(want) % 360:.0f} deg '
                                       f'(stop {self.stop_i + 1} of {self.scan_stops}) and hold it still')
            err = None if self.cur_yaw is None else abs(wrap_pi(self.cur_yaw - want))
            still = err is not None and err < math.radians(3) and abs(self.yaw_rate or 0.0) < math.radians(2)
            if not still:
                self.still_t = None
            elif self.still_t is None:
                self.still_t = self.now()
            elif self.now() - self.still_t >= 2.0:
                self.look_from = self.map_get().get('sweep', 0)
                if self.record_scan:
                    self.map_post('/mark', f'stop {self.stop_i + 1}/{self.scan_stops} at heading '
                                           f'{math.degrees(self.cur_yaw) % 360:.0f}: looking {self.sweeps_per_stop} sweeps')
                self.to_phase(f'{self.scan} scan: stop look')

        elif ph == 'stop look':
            # full sweeps while still: the one in progress started before the stop, so it doesn't count
            n, e, down, _ = self.hold
            want = self.stop_yaw0 + self.stop_i * self.scan_turn / self.scan_stops
            self.goto_ned(n, e, down, want, f'stop {self.stop_i + 1}')
            if self.map_get().get('sweep', 0) >= self.look_from + self.sweeps_per_stop + 1:
                self.stop_i += 1
                self.still_t = None
                if self.stop_i >= self.scan_stops:
                    self.hold_here(self.hold[2], 'hold')
                    self.to_phase(f'{self.scan} scan: settle')
                else:
                    self.to_phase(f'{self.scan} scan: stop turn')

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
        self.f_dir, self.over_pt = d, (tn, te)
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
        if self.stop_after == 'over':
            self.stop_here('over')
            return
        self.follow_map()

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
        self.f_dir, self.over_pt = s['d'], s['pt']
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
        self.follow_map()

    def follow_map(self):
        self.look_view('follow start')
        self.to_phase('follow start')

    def look_view(self, what):
        """The map's follow settings (straight down, the band from the floor), then a reset
        so the map holds only what this look sees; f_hold = where to hold meanwhile."""
        _, sector, threshold, _ = self.scans['pool']
        self.map_post('/range', self.follow_range)
        self.map_post('/sector', self.follow_sector)
        self.map_post('/threshold', threshold)
        self.map_post('/floorcut', 0.0)
        self.map_post('/compact', 'on' if self.pool_compact else 'off')
        self.map_post('/near', str(self.follow_near) if self.follow_near > 0 else 'off')
        self.map_post('/far', 'off')
        self.map_post('/fade', 'off')
        band = self.follow_band_now()
        self.map_post('/band', f'{band[0]} {band[1]}' if band else 'off')
        self.f_band, self.f_band_t = band, self.now()
        for cmd in ('reset', 'start', 'resume'):
            self.map_post('/mapcmd', cmd)
        if self.recording:
            try:
                where = 'from the sonar' if self.follow_band_from == 'sonar' else 'above the floor'
                self.map_post('/mark', f'{what}: sonar straight down, band '
                              + (f'{self.follow_band[0]} to {self.follow_band[1]} m {where}' if band else 'off (no DVL)'))
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
            self.f_lost_now, self.f_prev_loss, self.f_stuck_t = False, None, None
            self.f_reversed, self.f_hold_pt, self.f_leg_t0, self.f_creep0 = False, None, self.now(), None
            self.f_start, self.f_dir0 = (here[0], here[1]), self.f_dir.copy()
            self.start_camera()
            self.get_logger().info(f'    following: heading {math.degrees(math.atan2(self.f_dir[1], self.f_dir[0])) % 360:.0f} deg, '
                                   f'at most {self.follow_speed:.2f} m/s and {self.follow_step:.2f} m per sweep')
            self.to_phase('follow')

    def follow_step_once(self, m):
        now = self.now()
        here = self.cur if self.cur else self.f_hold
        h = self.cur_yaw if self.cur_yaw is not None else math.atan2(self.f_dir[1], self.f_dir[0])
        ahead = sonar_settings.SONAR_FWD_M
        sonar = np.array([here[0] + ahead * math.cos(h), here[1] + ahead * math.sin(h)])
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
            corridor = (np.abs(across) <= 0.8) & (along >= -self.follow_fit) & (along <= 0.4)
            if corridor.sum() >= 4 and np.ptp(along[corridor]) >= 0.3:
                pts = c[corridor]
                _, _, vt = np.linalg.svd(pts - pts.mean(axis=0))
                nd = vt[0] if vt[0] @ d >= 0 else -vt[0]
                if nd @ d > math.cos(math.radians(35)):          # one fit never swings it far
                    d = 0.6 * d + 0.4 * nd
                    d = d / np.linalg.norm(d)
                    self.f_dir, right = d, np.array([-d[1], d[0]])
            # only FRESH dots say the pipe is under the sonar now; old ones linger behind it
            under = fresh & (np.abs(along) <= 0.2) & (np.abs(across) <= 0.6)
            if under.any():
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
        if (now - seen > self.follow_lost and self.follow_both and not self.f_reversed
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
        if now - seen > self.follow_lost:
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
            self.stop_camera()
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
        self.stream_posvel(tn, te, self.hold[2], v * d[0], v * d[1], rate, 'follow')
        if self.f_dist - self.f_logged >= 0.5 or now - self.f_log_t >= (2.0 if self.dry else 10.0):
            self.f_logged, self.f_log_t = self.f_dist, now
            self.get_logger().info(
                f'    follow: sonar at NED ({sonar[0]:.2f}, {sonar[1]:.2f}), {self.f_dist:.1f} m along, pipe heading {math.degrees(yaw_d) % 360:.0f} deg '
                f'(sub {math.degrees(h) % 360:.0f}), pipe {abs(self.f_lat):.2f} m '
                f'{"right" if self.f_lat >= 0 else "left"} of the sonar, {v:.2f} m/s, sweep {self.f_period:.1f} s'
                + (f', last seen {now - seen:.0f} s ago' if now - seen > 3 else '')
                + (' (dry run: drive it there)' if self.dry else ''))

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
    def walk_to(self, goal, speed, yaw_d, label):
        """Carrot walk toward goal (n, e) at speed, turning toward yaw_d. Returns metres left."""
        now = self.now()
        dt, self.last_t = min(now - self.last_t, 0.5), now
        cn, ce = self.carrot
        left = math.hypot(goal[0] - cn, goal[1] - ce)
        step = min(speed * dt, left)
        ux, uy = ((goal[0] - cn) / left, (goal[1] - ce) / left) if left > 1e-6 else (0.0, 0.0)
        self.carrot = (cn + ux * step, ce + uy * step)
        v = speed if left > step else 0.0
        rate, err = self.yaw_rate_to(yaw_d)
        self.stream_posvel(self.carrot[0], self.carrot[1], self.hold[2], ux * v, uy * v, rate, label)
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
            self.f_backup_goal = (self.f_start[0] + self.follow_backup * self.f_dir0[0],
                                  self.f_start[1] + self.follow_backup * self.f_dir0[1])
            self.get_logger().info(f'    turned round - backing up {self.follow_backup:.2f} m so the sonar '
                                   f'starts where it began')
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
        n = len(self.camera.tracker.boxes) if self.camera else 0
        self.get_logger().info(f'    camera: {"NEW " if b["new"] else "again: "}{led_boxes.describe(b)} '
                               f'- {n} of {self.boxes_expected} boxes so far')
        if self.recording and b['new']:
            try:
                self.map_post('/mark', f'camera box {b["number"]}: ' + max(b['colours'], key=b['colours'].get))
            except OSError:
                pass

    def start_camera(self):
        if self.camera_boxes and self.camera is None:
            self.camera = led_boxes.CameraWatcher(self.camera_url, self.camera_at,
                                                  lambda t: self.get_logger().info('    ' + t), self.on_box).start()

    def stop_camera(self):
        if self.camera is None:
            return
        boxes = self.camera.close()
        self.get_logger().info(f'camera: {len(boxes)} of {self.boxes_expected} light boxes seen'
                               + ''.join(f'\n      {led_boxes.describe(b)}' for b in boxes))
        self.camera = None

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
