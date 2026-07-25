#!/usr/bin/env python3
"""Prequalification mission v2 - CV-assisted.

Flow: dive -> transit toward the planner's marker guess -> when pole_tracker
reports a stable lock, convert the sighting to a WORLD position and adopt it as
the real marker -> orbit it at orbit_radius facing it the whole way (full 360
of yaw) -> return to start -> surface -> disarm.

The CV fix only CORRECTS THE TARGET; GUIDED still does the driving, so a
detection dropout mid-orbit is harmless (we keep circling the last good fix).
If the pole is never seen, it flies the planner's coordinates as before.

SAFETY: dry_run defaults True (prints only, never arms/moves).
"""
import math
import os
import time
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from pymavlink import mavutil

# position + yaw, ignore vel/accel/yaw-rate
TYPE_MASK_POS = 0b100111111000
FRAME_LOCAL_NED = 1


class S(Enum):
    WAIT_NAV = 0
    ARM = 1
    DIVE = 2
    TO_GATE = 3
    TO_MARKER = 4
    ORBIT = 5
    RETURN_GATE = 6
    RETURN_HOME = 7
    SURFACE = 8
    DONE = 9


class PrequalCV(Node):
    def __init__(self):
        super().__init__('prequal_mission_cv')
        p = self.declare_parameter
        p('mavlink', 'udpout:127.0.0.1:14553')
        p('dry_run', True)
        p('depth', 1.5)
        p('gate_forward', 4.0)
        p('marker_forward', 13.0)
        p('marker_right', 0.0)
        p('orbit_radius', 0.75)
        p('orbit_points', 12)
        p('use_cv', True)
        p('reach_thresh', 0.25)
        p('state_timeout', 60.0)
        p('sim_reach_time', 2.0)

        g = self.get_parameter
        self.dry = g('dry_run').value
        self.depth = g('depth').value
        self.gate_f = g('gate_forward').value
        self.marker_f = g('marker_forward').value
        self.marker_r = g('marker_right').value
        self.orbit_r = g('orbit_radius').value
        self.npts = g('orbit_points').value
        self.use_cv = g('use_cv').value
        self.thresh = g('reach_thresh').value
        self.timeout = g('state_timeout').value
        self.sim_reach = g('sim_reach_time').value

        conn = g('mavlink').value
        self.get_logger().info(f'opening MAVLink {conn} (dry_run={self.dry}, use_cv={self.use_cv})')
        self.mav = mavutil.mavlink_connection(conn, source_system=1, source_component=198)

        self.state = S.WAIT_NAV
        self.state_t0 = self.now()
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.cur = None
        self.cur_yaw = None
        self.target = None
        self.last_send_t = 0.0
        self.orbit_i = 0

        self.pole_ned = None          # (n, e) world position of the pole
        self.pole_locked = False      # frozen once the orbit begins
        self.pole_seen = False
        self.pole_body = None         # (fwd, right) latest sighting

        self.create_subscription(Bool, '/graey/pole/visible', self.on_vis, 10)
        self.create_subscription(PointStamped, '/graey/pole', self.on_pole, 10)
        self.auto_pub = self.create_publisher(Bool, '/graey/autonomy_active', 10)
        self.create_timer(0.1, self.pump_mavlink)
        self.create_timer(0.25, self.tick)

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, s):
        self.get_logger().info(f'--- {self.state.name} -> {s.name} ---')
        self.state = s
        self.state_t0 = self.now()

    def fr_to_ned(self, forward, right):
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        return (self.start_x + forward * c - right * s,
                self.start_y + forward * s + right * c)

    def marker_ned(self):
        if self.pole_ned is not None:
            return self.pole_ned
        return self.fr_to_ned(self.marker_f, self.marker_r)

    def goto_ned(self, n, e, down, yaw, label):
        tgt = (n, e, down, yaw)
        changed = (self.target is None or
                   abs(tgt[0] - self.target[0]) > 0.01 or
                   abs(tgt[1] - self.target[1]) > 0.01 or
                   abs(tgt[2] - self.target[2]) > 0.01 or
                   abs(tgt[3] - self.target[3]) > 0.02)
        self.target = tgt
        if self.dry:
            if changed:
                self.get_logger().info(
                    f'    [dry] {label}: NED=({n:.2f},{e:.2f},{down:.2f}) yaw={math.degrees(yaw):.0f}')
            return
        if not changed and self.now() - self.last_send_t < 3.0:
            return
        self.last_send_t = self.now()
        self.mav.mav.set_position_target_local_ned_send(
            0, 1, 1, FRAME_LOCAL_NED, TYPE_MASK_POS,
            n, e, down, 0, 0, 0, 0, 0, 0, yaw, 0)

    def goto(self, forward, right, down, label):
        n, e = self.fr_to_ned(forward, right)
        self.goto_ned(n, e, down, self.start_yaw, label)

    def reached(self):
        if self.dry:
            return self.now() - self.state_t0 > self.sim_reach
        if self.cur is None or self.target is None:
            return False
        dn = self.cur[0] - self.target[0]
        de = self.cur[1] - self.target[1]
        dd = self.cur[2] - self.target[2]
        return math.sqrt(dn*dn + de*de + dd*dd) < self.thresh

    def set_mode_guided(self):
        if self.dry:
            self.get_logger().info('    [dry] set mode GUIDED')
            return
        self.mav.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)

    def arm(self):
        if self.dry:
            self.get_logger().info('    [dry] ARM')
            return
        self.mav.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        if self.dry:
            self.get_logger().info('    [dry] DISARM')
            return
        for _ in range(10):
            self.mav.mav.command_long_send(
                1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
            time.sleep(0.1)

    # ---------- inputs ----------
    def on_vis(self, msg):
        self.pole_seen = msg.data

    def on_pole(self, msg):
        self.pole_body = (msg.point.x, msg.point.y)
        if not self.use_cv or self.pole_locked or self.cur is None or self.cur_yaw is None:
            return
        if self.state not in (S.TO_MARKER, S.TO_GATE):
            return
        c, s = math.cos(self.cur_yaw), math.sin(self.cur_yaw)
        f, r = msg.point.x, msg.point.y
        n = self.cur[0] + f * c - r * s
        e = self.cur[1] + f * s + r * c
        first = self.pole_ned is None
        self.pole_ned = (n, e)
        if first:
            self.get_logger().info(
                f'*** CV LOCK: pole at NED ({n:.2f},{e:.2f}) '
                f'[{f:.2f} m fwd, {r:+.2f} m right] - adopting as marker ***')

    def pump_mavlink(self):
        self.mav.mav.heartbeat_send(6, 8, 0, 0, 0)
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == 'LOCAL_POSITION_NED':
                self.cur = (m.x, m.y, m.z)
            elif t == 'ATTITUDE':
                self.cur_yaw = m.yaw

    # ---------- state machine ----------
    def tick(self):
        if self.state != S.DONE:
            self.auto_pub.publish(Bool(data=True))

        if self.state not in (S.WAIT_NAV, S.DONE):
            if self.now() - self.state_t0 > self.timeout:
                self.get_logger().warn(f'{self.state.name} timed out -> SURFACE')
                self.enter(S.SURFACE)

        if self.state == S.WAIT_NAV:
            if self.dry or (self.cur is not None and self.cur_yaw is not None):
                self.start_x = self.cur[0] if self.cur else 0.0
                self.start_y = self.cur[1] if self.cur else 0.0
                self.start_yaw = self.cur_yaw if self.cur_yaw is not None else 0.0
                self.get_logger().info(
                    f'nav ready, start=({self.start_x:.2f},{self.start_y:.2f}) '
                    f'yaw={math.degrees(self.start_yaw):.0f}')
                self.enter(S.ARM)

        elif self.state == S.ARM:
            self.set_mode_guided()
            self.arm()
            self.goto(0.0, 0.0, self.depth, 'initial hold')
            self.enter(S.DIVE)

        elif self.state == S.DIVE:
            self.goto(0.0, 0.0, self.depth, 'dive')
            if self.reached():
                self.enter(S.TO_GATE)

        elif self.state == S.TO_GATE:
            self.goto(self.gate_f, 0.0, self.depth, 'through gate')
            if self.reached():
                self.enter(S.TO_MARKER)

        elif self.state == S.TO_MARKER:
            # approach the (possibly CV-corrected) marker, stopping orbit_radius short
            mn, me = self.marker_ned()
            dn, de = mn - self.start_x, me - self.start_y
            d = math.hypot(dn, de)
            if d < 1e-3:
                self.enter(S.ORBIT)
                return
            ux, uy = dn / d, de / d
            tn = self.start_x + ux * max(0.0, d - self.orbit_r)
            te = self.start_y + uy * max(0.0, d - self.orbit_r)
            yaw = math.atan2(me - (self.cur[1] if self.cur else self.start_y),
                             mn - (self.cur[0] if self.cur else self.start_x))
            self.goto_ned(tn, te, self.depth, yaw, 'approach marker')
            if self.reached():
                self.pole_locked = True
                if hasattr(self, '_a0'):
                    del self._a0
                self.orbit_i = 0
                mn, me = self.marker_ned()
                src = 'CV' if self.pole_ned is not None else 'planner'
                self.get_logger().info(f'orbiting {src} marker at NED ({mn:.2f},{me:.2f}) r={self.orbit_r}')
                self.enter(S.ORBIT)

        elif self.state == S.ORBIT:
            mn, me = self.marker_ned()
            if self.orbit_i == 0 and not hasattr(self, '_a0'):
                if self.cur:
                    self._a0 = math.atan2(self.cur[1] - me, self.cur[0] - mn)
                else:
                    self._a0 = 0.0
                self.get_logger().info(f'orbit start angle {math.degrees(self._a0):.0f} deg')
            a = self._a0 + 2 * math.pi * self.orbit_i / self.npts
            n = mn + self.orbit_r * math.cos(a)
            e = me + self.orbit_r * math.sin(a)
            yaw = math.atan2(me - e, mn - n)          # face the pole
            self.goto_ned(n, e, self.depth, yaw, f'orbit {self.orbit_i+1}/{self.npts}')
            if self.reached():
                self.orbit_i += 1
                self.state_t0 = self.now()
                if self.orbit_i > self.npts:
                    self.enter(S.RETURN_GATE)

        elif self.state == S.RETURN_GATE:
            self.goto(self.gate_f, 0.0, self.depth, 'back through gate')
            if self.reached():
                self.enter(S.RETURN_HOME)

        elif self.state == S.RETURN_HOME:
            self.goto(0.0, 0.0, self.depth, 'return home')
            if self.reached():
                self.enter(S.SURFACE)

        elif self.state == S.SURFACE:
            self.goto(0.0, 0.0, 0.0, 'surface')
            if self.reached():
                self.enter(S.DONE)

        elif self.state == S.DONE:
            self.auto_pub.publish(Bool(data=False))
            self.disarm()
            self.get_logger().info('MISSION COMPLETE - disarmed')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PrequalCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
