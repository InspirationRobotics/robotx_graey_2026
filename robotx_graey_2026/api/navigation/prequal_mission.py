#!/usr/bin/env python3
"""Prequalification mission state machine.

Course: start 3 m behind the gate, submerge, pass through the gate, U-TURN
(180 deg semicircle) around the far side of the marker, return through the gate,
surface. NOT a full orbit - a semicircle around the marker.

Drives ArduSub in GUIDED with SET_POSITION_TARGET_LOCAL_NED. Waypoints are
(forward, right) offsets in a mission frame aligned to the heading captured at
mission start, then rotated into local NED.

SAFETY: dry_run defaults True -> only prints the plan and state transitions;
never arms, never changes mode, never sends setpoints. Set dry_run:=false only
in the water.
"""
import math
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pymavlink import mavutil


class S(Enum):
    WAIT_NAV = 0
    ARM = 1
    DIVE = 2
    TO_GATE = 3
    TO_MARKER = 4
    UTURN = 5
    RETURN_GATE = 6
    RETURN_HOME = 7
    SURFACE = 8
    DONE = 9


TYPE_MASK_POS = 0b100111111000
FRAME_LOCAL_NED = 1


class PrequalMission(Node):
    def __init__(self):
        super().__init__('prequal_mission')
        p = self.declare_parameter
        p('mavlink', 'udpout:127.0.0.1:14553')
        p('dry_run', True)
        p('depth', 1.5)
        p('gate_forward', 4.0)
        p('marker_forward', 13.0)
        p('uturn_radius', 1.5)
        p('uturn_points', 7)
        p('reach_thresh', 0.4)
        p('state_timeout', 60.0)
        p('sim_reach_time', 2.0)

        g = self.get_parameter
        self.dry = g('dry_run').value
        self.depth = g('depth').value
        self.gate_f = g('gate_forward').value
        self.marker_f = g('marker_forward').value
        self.r = g('uturn_radius').value
        self.npts = g('uturn_points').value
        self.thresh = g('reach_thresh').value
        self.timeout = g('state_timeout').value
        self.sim_reach = g('sim_reach_time').value

        conn = g('mavlink').value
        self.get_logger().info(f'opening MAVLink {conn} (dry_run={self.dry})')
        self.mav = mavutil.mavlink_connection(conn, source_system=1, source_component=198)

        self.state = S.WAIT_NAV
        self.state_t0 = self.now()
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.cur = None
        self.target = None
        self.uturn_i = 0
        self.auto_pub = self.create_publisher(Bool, '/graey/autonomy_active', 10)
        self.create_timer(0.1, self.pump_mavlink)
        self.create_timer(0.25, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, s):
        self.get_logger().info(f'--- {self.state.name} -> {s.name} ---')
        self.state = s
        self.state_t0 = self.now()

    def fr_to_ned(self, forward, right):
        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)
        n = self.start_x + forward * c - right * s
        e = self.start_y + forward * s + right * c
        return n, e

    def goto(self, forward, right, down, label):
        n, e = self.fr_to_ned(forward, right)
        self.target = (n, e, down)
        if self.dry:
            self.get_logger().info(
                f'    [dry] GOTO {label}: fwd={forward:.1f} rgt={right:.1f} '
                f'dn={down:.1f}  NED=({n:.1f},{e:.1f},{down:.1f})')
            return
        self.mav.mav.set_position_target_local_ned_send(
            0, 1, 1, FRAME_LOCAL_NED, TYPE_MASK_POS,
            n, e, down, 0, 0, 0, 0, 0, 0, self.start_yaw, 0)

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
        self.mav.set_mode('GUIDED')

    def arm(self):
        if self.dry:
            self.get_logger().info('    [dry] ARM')
            return
        self.mav.mav.command_long_send(1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    def pump_mavlink(self):
        while True:
            m = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if m is None:
                break
            self.cur = (m.x, m.y, m.z)

    def tick(self):
        if self.state != S.DONE:
            self.auto_pub.publish(Bool(data=True))

        if self.state not in (S.WAIT_NAV, S.DONE):
            if self.now() - self.state_t0 > self.timeout:
                self.get_logger().warn(f'{self.state.name} timed out -> SURFACE')
                self.enter(S.SURFACE)

        if self.state == S.WAIT_NAV:
            if self.dry or self.cur is not None:
                self.start_x = self.cur[0] if self.cur else 0.0
                self.start_y = self.cur[1] if self.cur else 0.0
                self.get_logger().info(
                    f'nav ready, start=({self.start_x:.1f},{self.start_y:.1f})')
                self.enter(S.ARM)

        elif self.state == S.ARM:
            self.set_mode_guided()
            self.arm()
            self.enter(S.DIVE)

        elif self.state == S.DIVE:
            self.goto(0.0, 0.0, self.depth, 'dive in place')
            if self.reached():
                self.enter(S.TO_GATE)

        elif self.state == S.TO_GATE:
            self.goto(self.gate_f, 0.0, self.depth, 'through gate')
            if self.reached():
                self.enter(S.TO_MARKER)

        elif self.state == S.TO_MARKER:
            self.goto(self.marker_f, -self.r, self.depth, 'marker enter')
            if self.reached():
                self.uturn_i = 0
                self.enter(S.UTURN)

        elif self.state == S.UTURN:
            a = -math.pi/2 + math.pi * self.uturn_i / (self.npts - 1)
            fwd = self.marker_f + self.r * math.cos(a)
            rgt = self.r * math.sin(a)
            self.goto(fwd, rgt, self.depth, f'uturn {self.uturn_i+1}/{self.npts}')
            if self.reached():
                self.uturn_i += 1
                self.state_t0 = self.now()
                if self.uturn_i >= self.npts:
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
            self.get_logger().info('MISSION COMPLETE')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PrequalMission()
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
