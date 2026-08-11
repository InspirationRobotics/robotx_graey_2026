#!/usr/bin/env python3
"""Fuse VN-100 attitude + DVL velocity into ArduSub's EKF as one ODOMETRY stream.

Sends MAVLink ODOMETRY at a fixed 20 Hz carrying:
  - attitude quaternion  (from /graey/vn100/imu)    -> EKF yaw source
  - body velocity        (from /graey/dvl/velocity) -> EKF velocity source
  - integrated position  (dead-reckoned when the DVL has bottom lock)

Attitude is sent every tick as long as the VN-100 is publishing, independent of
the DVL. On a dry bench (no DVL / no bottom lock) position stays at the origin
and velocity is zero, but heading still tracks - that is the yaw bench test.

self.pos is the only accumulated state in the navigation chain, so THIS NODE MUST
NOT DIE. Restarting it snaps the dead-reckoned position back to the origin and
teleports the EKF's external-nav source by however far it had travelled. That is
why Link reconnects on ECONNREFUSED rather than letting the exception out.
"""
import math

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link


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
        self.declare_parameter('yaw_offset_deg', 0.0)   # frame alignment, not a calibration
        self.yaw_off = self.get_parameter('yaw_offset_deg').value
        self.link = Link(self.get_parameter('mavlink').value, 197, self.get_logger())

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
        self.create_timer(0.05, self.send_odom)         # 20 Hz
        self.create_timer(2.0, self.report)

    def report(self):
        self.get_logger().debug(
            f'att={self.have_att} dvl_valid={self.valid} sent={self.sent} '
            f'pos=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f})')

    def on_imu(self, m):
        self.q = yaw_offset_quat((m.orientation.w, m.orientation.x,
                                  m.orientation.y, m.orientation.z), self.yaw_off)
        self.rates = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        self.have_att = True

    def on_valid(self, msg):
        self.valid = msg.data

    def on_vel(self, msg):
        self.vb = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                   msg.twist.twist.linear.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is not None and self.valid:
            dt = t - self.last_t
            if 0.0 < dt < 1.0:                          # ignore stalls and clock jumps
                vw = rotate_body_to_world(self.q, self.vb)
                for i in range(3):
                    self.pos[i] += vw[i] * dt
        self.last_t = t

    def send_odom(self):
        if not self.have_att:
            return
        v = self.vb if self.valid else (0.0, 0.0, 0.0)
        self.link.odometry(self.pos, self.q, v, self.rates)
        self.sent += 1


def main():
    run(NavEKFBridge)
