#!/usr/bin/env python3
"""Shared machinery for the prequalification missions.

Both missions fly the same skeleton - wait for nav, arm, dive, out through the
gate, do something at the marker, come back, surface, disarm - and differ only
in how they approach the marker and what they do there. Those two steps are the
hooks do_to_marker() and do_maneuver(); everything else lives here.

Waypoints are (forward, right) offsets in a mission frame aligned to the heading
captured at mission start, then rotated into local NED.

SAFETY: dry_run defaults True -> prints the plan and state transitions only;
never arms, never changes mode, never sends setpoints. Set dry_run:=false only
in the water.
"""
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.mavlink import Link, MODE_GUIDED

RESEND_S = 3.0                                  # unchanged targets re-sent no faster than
                                                # this: streaming restarts ArduSub's
                                                # trajectory planner and the sub crawls


class S(Enum):
    WAIT_NAV = 0
    ARM = 1
    DIVE = 2
    TO_GATE = 3
    TO_MARKER = 4
    MANEUVER = 5                                # U-turn in v1, orbit in the CV mission
    RETURN_GATE = 6
    RETURN_HOME = 7
    SURFACE = 8
    DONE = 9


class MissionBase(Node):
    def __init__(self, name, component=198):
        super().__init__(name)
        p = self.declare_parameter
        p('mavlink', 'udpout:127.0.0.1:14553')
        p('dry_run', True)
        p('depth', 1.5)
        p('gate_forward', 4.0)
        p('marker_forward', 13.0)
        p('marker_right', 0.0)
        p('reach_thresh', 0.25)                 # 0.4 exceeded the dive delta and self-completed
        p('state_timeout', 60.0)
        p('sim_reach_time', 2.0)
        self.declare_extra_parameters()

        g = self.get_parameter
        self.dry = g('dry_run').value
        self.depth = g('depth').value
        self.gate_f = g('gate_forward').value
        self.marker_f = g('marker_forward').value
        self.marker_r = g('marker_right').value
        self.thresh = g('reach_thresh').value
        self.timeout = g('state_timeout').value
        self.sim_reach = g('sim_reach_time').value
        self.read_extra_parameters()

        self.link = Link(g('mavlink').value, component, self.get_logger())
        self.get_logger().info(f'dry_run={self.dry}')

        self.state = S.WAIT_NAV
        self.state_t0 = self.now()
        self.start_x = self.start_y = self.start_yaw = 0.0
        self.cur = None
        self.cur_yaw = None
        self.target = None
        self.last_send_t = 0.0
        self.step = 0                           # waypoint index within MANEUVER

        self.auto_pub = self.create_publisher(Bool, '/graey/autonomy_active', 10)
        self.create_timer(0.1, self.pump_mavlink)
        self.create_timer(0.25, self.tick)

    # ---------- subclass hooks ----------
    def declare_extra_parameters(self):
        pass

    def read_extra_parameters(self):
        pass

    def do_to_marker(self):
        raise NotImplementedError

    def do_maneuver(self):
        raise NotImplementedError

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, s):
        self.get_logger().info(f'--- {self.state.name} -> {s.name} ---')
        self.state = s
        self.state_t0 = self.now()

    def fr_to_ned(self, forward, right):
        return body_to_world(self.start_x, self.start_y, self.start_yaw, forward, right)

    def goto_ned(self, n, e, down, yaw, label):
        tgt = (n, e, down, yaw)
        changed = (self.target is None
                   or any(abs(a - b) > 0.01 for a, b in zip(tgt[:3], self.target[:3]))
                   or abs(yaw - self.target[3]) > 0.02)
        self.target = tgt
        if self.dry:
            if changed:
                self.get_logger().info(
                    f'    [dry] {label}: NED=({n:.2f},{e:.2f},{down:.2f}) '
                    f'yaw={math.degrees(yaw):.0f}')
            return
        if not changed and self.now() - self.last_send_t < RESEND_S:
            return
        self.last_send_t = self.now()
        self.link.goto_ned(n, e, down, yaw)

    def goto(self, forward, right, down, label):
        n, e = self.fr_to_ned(forward, right)
        self.goto_ned(n, e, down, self.start_yaw, label)

    def reached(self):
        if self.dry:
            return self.now() - self.state_t0 > self.sim_reach
        if self.cur is None or self.target is None:
            return False
        return math.dist(self.cur, self.target[:3]) < self.thresh

    def pump_mavlink(self):
        self.link.heartbeat()

        def handle(kind, m):
            if kind == 'LOCAL_POSITION_NED':
                self.cur = (m.x, m.y, m.z)
            elif kind == 'ATTITUDE':
                self.cur_yaw = m.yaw
        self.link.drain(handle)

    # ---------- state machine ----------
    def tick(self):
        if self.state != S.DONE:
            self.auto_pub.publish(Bool(data=True))

        if (self.state not in (S.WAIT_NAV, S.DONE)
                and self.now() - self.state_t0 > self.timeout):
            self.get_logger().warn(f'{self.state.name} timed out -> SURFACE')
            self.enter(S.SURFACE)

        if self.state == S.WAIT_NAV:
            if self.dry or (self.cur is not None and self.cur_yaw is not None):
                if self.cur:
                    self.start_x, self.start_y = self.cur[0], self.cur[1]
                self.start_yaw = self.cur_yaw if self.cur_yaw is not None else 0.0
                self.get_logger().info(
                    f'nav ready, start=({self.start_x:.2f},{self.start_y:.2f}) '
                    f'yaw={math.degrees(self.start_yaw):.0f}')
                self.enter(S.ARM)

        elif self.state == S.ARM:
            if self.dry:
                self.get_logger().info('    [dry] set mode GUIDED + ARM')
            else:
                self.link.set_mode(MODE_GUIDED)
                self.link.arm()
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
            self.do_to_marker()

        elif self.state == S.MANEUVER:
            self.do_maneuver()

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
            if self.dry:
                self.get_logger().info('    [dry] DISARM')
            else:
                self.link.disarm()
            self.get_logger().info('MISSION COMPLETE - disarmed')
            rclpy.shutdown()
