#!/usr/bin/env python3
"""Task 2: find the pipeline with the sonar map, then follow it and mark the light boxes.

Ruth's plan (Oct 2026). It starts where the buoy code leaves the sub: at the
active buoy, on the surface. Built in steps, each checked in the simulator:

  step 1  DIVE  to scan_depth (3 m)
          SCAN  turn slowly on the spot with the sonar map running, until the
                sub has turned scan_turn_deg. Slower than the map's 10 deg/s
                limit, so no ping is thrown away
          PICK  the most pipe-like outline on the map (api/sonar/outlines.py);
                none scoring min_score -> surface
  (next)  go above the near end, follow it, mark the boxes

THE MAP. tools/sonar_map.py runs beside this (on Graey, on the host; in the sim,
sim/sim_sonar_map.py) and is driven through its web page's own controls, the
same as pressing the buttons: range, sector, threshold, Reset/Start. Its frame
is (0,0) = where the sub was at Start, +y = the heading then. This captures the
EKF's position and heading at that moment to turn map points into NED.

SAFETY: as every MissionBase mission - dry_run defaults True and then nothing
is sent; changing flight mode hands the sub to the pilot and this exits.
"""
import json
import math
import urllib.request

import rclpy

from robotx_graey_2026.api.navigation import mission_base
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S

# The simulator never left the surface with unchanged targets re-sent every 3 s
# (sim/README.md). Here a target changes when the plan does, and is otherwise
# only re-sent as insurance.
mission_base.RESEND_S = 20.0


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PipelineMission(MissionBase):
    def __init__(self):
        super().__init__('pipeline_mission', component=197)
        self.phase = None
        self.map_origin = None              # (n, e, yaw) of map (0,0) and its +y
        self.turned = 0.0
        self.last_yaw = None
        self.pipe = None                    # the outline picked
        self.map_down_since = None

    def declare_extra_parameters(self):
        p = self.declare_parameter
        p('map_url', 'http://127.0.0.1:8095')
        p('scan_depth', 3.0)                # Ruth: 3 m at the buoy, harbor ~12 m deep
        p('scan_range', 15.0)               # m, the map's range while scanning
        p('scan_sector', '180 0')           # bottom half: left, down, right
        p('scan_threshold', 120)
        p('scan_turn_deg', 360.0)
        p('scan_rate_dps', 4.0)
        p('min_score', 0.5)
        p('phase_timeout', 240.0)

    def read_extra_parameters(self):
        g = lambda n: self.get_parameter(n).value
        self.map_url = g('map_url')
        self.depth = g('scan_depth')        # MissionBase dives to self.depth
        self.gate_f = 0.0                   # no gate: hold where we dived
        self.scan_range = g('scan_range')
        self.scan_sector = g('scan_sector')
        self.scan_threshold = g('scan_threshold')
        self.scan_turn = math.radians(g('scan_turn_deg'))
        self.scan_rate = math.radians(g('scan_rate_dps'))
        self.min_score = g('min_score')
        self.timeout = g('phase_timeout')   # per phase: each one restarts the clock

    # ---------- the sonar map, through its web page ----------
    def map_post(self, path, body):
        req = urllib.request.Request(self.map_url + path, data=str(body).encode())
        return urllib.request.urlopen(req, timeout=1.0).read().decode()

    def map_get(self):
        return json.loads(urllib.request.urlopen(self.map_url + '/mapdata', timeout=1.0).read())

    def map_to_ned(self, x, y):
        n0, e0, yaw0 = self.map_origin
        return (n0 + y * math.cos(yaw0) - x * math.sin(yaw0),
                e0 + y * math.sin(yaw0) + x * math.cos(yaw0))

    # ---------- phases ----------
    def to_phase(self, name):
        self.get_logger().info(f'    phase: {name}')
        self.phase = name
        self.state_t0 = self.now()

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
            self.to_phase('map setup')

        if self.phase == 'map setup':
            # range first: changing it restarts the sweep
            self.map_post('/range', self.scan_range)
            self.map_post('/sector', self.scan_sector)
            self.map_post('/threshold', self.scan_threshold)
            for cmd in ('reset', 'start', 'resume'):    # whatever state it was in
                self.map_post('/mapcmd', cmd)
            self.to_phase('map start')

        elif self.phase == 'map start':
            self.goto(0.0, 0.0, self.depth, 'hold')
            m = self.map_get()
            if m['state'] == 'running' and m['pose'] and math.hypot(*m['pose'][:2]) < 0.05:
                if self.dry or self.cur is None:
                    self.map_origin = (0.0, 0.0, self.start_yaw)
                else:
                    self.map_origin = (self.cur[0], self.cur[1], self.cur_yaw)
                self.hold_ne = self.map_origin[:2]
                self.turned, self.last_yaw = 0.0, self.cur_yaw
                self.to_phase('scan')

        elif self.phase == 'scan':
            n, e = self.hold_ne
            self.stream_posvel(n, e, self.depth, 0.0, 0.0, self.scan_rate, 'scan turn')
            if self.cur_yaw is not None and self.last_yaw is not None:
                self.turned += wrap_pi(self.cur_yaw - self.last_yaw)
            self.last_yaw = self.cur_yaw
            if abs(self.turned) >= self.scan_turn or (self.dry and self.now() - self.state_t0 > 5):
                self.target = None              # next goto is "changed", so it is sent
                self.hold_yaw = self.cur_yaw or 0.0
                self.to_phase('pick')

        elif self.phase == 'pick':
            n, e = self.hold_ne
            self.goto_ned(n, e, self.depth, self.hold_yaw, 'hold')
            if self.now() - self.state_t0 < 6.0:    # the last sweep lands, outlines refresh
                return
            m = self.map_get()
            outlines = m.get('outlines') or []
            for i, g in enumerate(outlines[:5]):
                (n1, e1), (n2, e2) = (self.map_to_ned(*g['spine'][k]) for k in (0, -1))
                self.get_logger().info(
                    f'    outline {i}: pipe {g["score"]:.0%} ({g["parts"]}) {g["n"]} crumbs, '
                    f'{g["length"]:.1f} m long, {g["width"]:.2f} m wide, top {g["top"]} m, '
                    f'NED ends ({n1:.1f},{e1:.1f}) ({n2:.1f},{e2:.1f})')
            best = outlines[0] if outlines else None
            if best is None or best['score'] < self.min_score:
                self.get_logger().warn(f'no outline scores {self.min_score:.0%} - surfacing')
                self.enter(S.SURFACE)
                return
            self.pipe = best
            self.get_logger().info(f'picked the {best["score"]:.0%} outline')
            self.enter(S.SURFACE)                    # step 1 ends here

    def do_maneuver(self):
        self.enter(S.SURFACE)


def main():
    rclpy.init()
    rclpy.spin(PipelineMission())


if __name__ == '__main__':
    main()
