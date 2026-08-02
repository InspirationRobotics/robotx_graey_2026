#!/usr/bin/env python3
"""Prequalification mission v2 - CV-assisted.

Dive, transit toward the planner's marker guess, and when pole_tracker reports a
sighting, convert it to a WORLD position and adopt that as the real marker.
Orbit at orbit_radius facing the pole the whole way (a full 360 of yaw), then
return and surface.

The CV fix only CORRECTS THE TARGET; GUIDED still does the driving, and the fix
freezes once the orbit starts - so a detection dropout mid-orbit is harmless. If
the pole is never seen it flies the planner's coordinates unchanged.
"""
import math

from geometry_msgs.msg import PointStamped

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S


class PrequalCV(MissionBase):
    def __init__(self):
        super().__init__('prequal_mission_cv')
        self.pole_ned = None                        # (n, e) world fix, None until seen
        self.pole_locked = False                    # frozen when the orbit begins
        self.a0 = None                              # orbit start angle, frozen on entry
        self.create_subscription(PointStamped, '/graey/pole', self.on_pole, 10)

    def declare_extra_parameters(self):
        self.declare_parameter('orbit_radius', 0.75)
        self.declare_parameter('orbit_points', 12)
        self.declare_parameter('use_cv', True)

    def read_extra_parameters(self):
        self.orbit_r = self.get_parameter('orbit_radius').value
        self.npts = self.get_parameter('orbit_points').value
        self.use_cv = self.get_parameter('use_cv').value

    def marker_ned(self):
        return self.pole_ned or self.fr_to_ned(self.marker_f, self.marker_r)

    def on_pole(self, msg):
        """Body-frame sighting -> world NED, adopted as the marker until locked."""
        if (not self.use_cv or self.pole_locked or self.cur is None
                or self.cur_yaw is None
                or self.state not in (S.TO_GATE, S.TO_MARKER)):
            return
        c, s = math.cos(self.cur_yaw), math.sin(self.cur_yaw)
        f, r = msg.point.x, msg.point.y
        first = self.pole_ned is None
        self.pole_ned = (self.cur[0] + f * c - r * s, self.cur[1] + f * s + r * c)
        if first:
            self.get_logger().info(
                f'*** CV LOCK: pole at NED ({self.pole_ned[0]:.2f},'
                f'{self.pole_ned[1]:.2f}) [{f:.2f} m fwd, {r:+.2f} m right] ***')

    def do_to_marker(self):                         # approach, stopping one radius short
        mn, me = self.marker_ned()
        dn, de = mn - self.start_x, me - self.start_y
        d = math.hypot(dn, de)
        if d < 1e-3:
            self.enter(S.MANEUVER)
            return
        cn = self.cur[0] if self.cur else self.start_x
        ce = self.cur[1] if self.cur else self.start_y
        reach = max(0.0, d - self.orbit_r)
        self.goto_ned(self.start_x + dn / d * reach, self.start_y + de / d * reach,
                      self.depth, math.atan2(me - ce, mn - cn), 'approach marker')
        if self.reached():
            self.pole_locked = True
            self.a0 = None
            self.step = 0
            src = 'CV' if self.pole_ned else 'planner'
            self.get_logger().info(
                f'orbiting {src} marker at NED ({mn:.2f},{me:.2f}) r={self.orbit_r}')
            self.enter(S.MANEUVER)

    def do_maneuver(self):                          # full circle, always facing the pole
        mn, me = self.marker_ned()
        if self.a0 is None:                         # freeze once, or waypoint 1 is
            if self.cur:                            # always "wherever I am right now"
                self.a0 = math.atan2(self.cur[1] - me, self.cur[0] - mn)
            else:
                self.a0 = 0.0
            self.get_logger().info(f'orbit start angle {math.degrees(self.a0):.0f} deg')
        a = self.a0 + 2 * math.pi * self.step / self.npts
        n = mn + self.orbit_r * math.cos(a)
        e = me + self.orbit_r * math.sin(a)
        self.goto_ned(n, e, self.depth, math.atan2(me - e, mn - n),
                      f'orbit {self.step + 1}/{self.npts}')
        if self.reached():
            self.step += 1
            self.state_t0 = self.now()
            if self.step > self.npts:               # one extra point closes the circle
                self.enter(S.RETURN_GATE)


def main():
    run(PrequalCV)
