#!/usr/bin/env python3
"""Fuse VN-100 attitude + DVL velocity into ArduSub's EKF as one ODOMETRY stream.

Supersedes dvl_ekf_bridge. Sends MAVLink ODOMETRY at a fixed 20 Hz carrying:
  - attitude quaternion  (from /graey/vn100/imu)    -> EKF yaw source
  - body velocity        (from /graey/dvl/velocity) -> EKF velocity source
  - integrated position  (dead-reckoned when DVL has bottom lock)

Attitude is sent every tick as long as the VN-100 is publishing, independent of
the DVL. On a dry bench (no DVL / no bottom lock) position stays at the origin
and velocity is zero, but heading still tracks - that is the yaw bench test.

Params:
  mavlink        udpout:127.0.0.1:14551
  yaw_offset_deg extra yaw applied to the reported heading (frame alignment)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
from pymavlink import mavutil

mavutil.set_dialect('ardupilotmega')

FRAME_LOCAL_FRD = 20
FRAME_BODY_FRD = 12
EST_VIO = 3


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def rotate_body_to_world(q, v):
    qc = (q[0], -q[1], -q[2], -q[3])
    vw = quat_mul(quat_mul(q, (0.0, v[0], v[1], v[2])), qc)
    return (vw[1], vw[2], vw[3])


def yaw_offset_quat(q, deg):
    if deg == 0.0:
        return q
    h = math.radians(deg) * 0.5
    return quat_mul((math.cos(h), 0.0, 0.0, math.sin(h)), q)


class NavEKFBridge(Node):
    def __init__(self):
        super().__init__('nav_ekf_bridge')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14551')
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.yaw_off = self.get_parameter('yaw_offset_deg').value

        conn = self.get_parameter('mavlink').value
        self.get_logger().info(f'opening MAVLink {conn}')
        self.mav = mavutil.mavlink_connection(conn, source_system=1, source_component=197)

        self.q = (1.0, 0.0, 0.0, 0.0)
        self.rates = (0.0, 0.0, 0.0)
        self.vb = (0.0, 0.0, 0.0)
        self.have_att = False
        self.valid = False
        self.pos = [0.0, 0.0, 0.0]
        self.last_t = None
        self.sent = 0

        self.create_subscription(Imu, '/graey/vn100/imu', self.on_imu, 20)
        self.create_subscription(Bool, '/graey/dvl/valid', self.on_valid, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/graey/dvl/velocity',
                                 self.on_vel, 20)
        self.create_timer(0.05, self.send_odom)     # 20 Hz
        self.create_timer(2.0, self.report)

    def report(self):
        self.get_logger().info(
            f'att={self.have_att} dvl_valid={self.valid} sent={self.sent} '
            f'pos=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f})')

    def on_imu(self, m):
        q = (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
        self.q = yaw_offset_quat(q, self.yaw_off)
        self.rates = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        self.have_att = True

    def on_valid(self, msg):
        self.valid = msg.data

    def on_vel(self, msg):
        self.vb = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is not None and self.valid:
            dt = t - self.last_t
            if 0.0 < dt < 1.0:
                vw = rotate_body_to_world(self.q, self.vb)
                self.pos[0] += vw[0] * dt
                self.pos[1] += vw[1] * dt
                self.pos[2] += vw[2] * dt
        self.last_t = t

    def send_odom(self):
        if not self.have_att:
            return
        v = self.vb if self.valid else (0.0, 0.0, 0.0)
        nan = float('nan')
        cov = [nan] + [0.0] * 20
        self.mav.mav.odometry_send(
            0, FRAME_LOCAL_FRD, FRAME_BODY_FRD,
            self.pos[0], self.pos[1], self.pos[2],
            list(self.q),
            v[0], v[1], v[2],
            self.rates[0], self.rates[1], self.rates[2],
            cov, cov, 0, EST_VIO)
        self.sent += 1


def main():
    rclpy.init()
    node = NavEKFBridge()
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
