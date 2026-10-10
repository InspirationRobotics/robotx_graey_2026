#!/usr/bin/env python3
"""Fuse VN-100 attitude + DVL velocity into ArduSub's EKF as one ODOMETRY stream.

Sends MAVLink ODOMETRY at a fixed 20 Hz carrying:
  - attitude quaternion  (from /graey/vn100/imu)    -> EKF yaw source
  - body velocity        (from /graey/dvl/velocity) -> EKF velocity source
  - integrated position  (dead-reckoned when the DVL has bottom lock)

Combined odometry is withheld when either DVL or VN-100 is invalid/stale.
Missing bottom lock is not a zero-velocity measurement. This also withholds
external yaw; independent heading-only transport requires separate validation.

self.pos is the only accumulated state in the navigation chain, so THIS NODE MUST
NOT DIE. Restarting it snaps the dead-reckoned position back to the origin and
teleports the EKF's external-nav source by however far it had travelled. That is
why Link reconnects on ECONNREFUSED rather than letting the exception out.
"""
import math
import json
import time
import uuid

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
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
        self.declare_parameter('yaw_offset_deg', 0.0, ParameterDescriptor(
            read_only=True,
            description='Startup-only frame alignment; restart requires frame revalidation.'))
        self.declare_parameter('allow_alignment', False)
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
        self.imu_received = self.dvl_received = -math.inf
        self.imu_stamp = -1
        self.sensor_stamp = None
        self.reset_counter = 0
        self.instance = str(uuid.uuid4())
        self.aligned_id = ''
        self.status_pub = self.create_publisher(String, '/graey/navigation/bridge_status', 10)

        self.create_subscription(Imu, '/graey/vn100/imu', self.on_imu, 20)
        self.create_subscription(String, '/graey/dvl/sample', self.on_sample, 20)
        self.create_subscription(String, '/graey/navigation/bridge_request', self.align, 10)
        self.create_timer(0.05, self.send_odom)         # 20 Hz
        self.create_timer(2.0, self.report)
        self.create_timer(.2, self.publish_status)

    def report(self):
        self.get_logger().debug(
            f'att={self.have_att} dvl_valid={self.valid} sent={self.sent} '
            f'pos=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f})')

    def on_imu(self, m):
        stamp = m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        if stamp <= self.imu_stamp:
            return
        self.imu_stamp = stamp
        q = (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
        rates = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        if not all(math.isfinite(v) for v in q + rates) or not .9 < sum(v*v for v in q) < 1.1:
            self.have_att = False
            return
        self.imu_received = time.monotonic()
        self.q = yaw_offset_quat((m.orientation.w, m.orientation.x,
                                  m.orientation.y, m.orientation.z), self.yaw_off)
        self.rates = rates
        self.have_att = True

    def on_sample(self, msg):
        try:
            d = json.loads(msg.data)
            t = d['stamp_ns']*1e-9
            if self.last_t is not None and t <= self.last_t:
                return
            sensor_stamp = d.get('sensor_time')
            if sensor_stamp is not None and sensor_stamp == self.sensor_stamp:
                return
            self.sensor_stamp = sensor_stamp
            self.vb = tuple(float(v) for v in d['velocity'])
            self.valid = bool(d['valid']) and len(self.vb) == 3 and all(math.isfinite(v) for v in self.vb)
            self.dvl_received = time.monotonic()
            if self.last_t is not None and self.valid and self.fresh():
                dt = t-self.last_t
                if 0 < dt < .5:
                    delta = rotate_body_to_world(self.q, self.vb)
                    self.pos = [a+b*dt for a, b in zip(self.pos, delta)]
            self.last_t = t
        except (ValueError, TypeError, KeyError):
            self.valid = False

    def fresh(self):
        now = time.monotonic()
        return (self.have_att and self.valid and 0 <= now-self.imu_received < .5
                and 0 <= now-self.dvl_received < .5)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(instance=self.instance,
            healthy=self.fresh(), aligned=bool(self.aligned_id), request_id=self.aligned_id,
            reset_counter=self.reset_counter))))

    def align(self, msg):
        if not self.get_parameter('allow_alignment').value or not self.fresh():
            return
        try:
            d = json.loads(msg.data)
            if not 0 < d['expires_at']-time.monotonic() <= .6 or d['request_id'] == self.aligned_id:
                return
            position = list(map(float, d['cube_ned']))
            if len(position) != 3 or not all(math.isfinite(v) for v in position):
                return
            # One frame alignment at transition, never continuous EKF feedback.
            self.pos = position
            self.reset_counter = (self.reset_counter+1) % 256
            self.aligned_id = d['request_id']
            self.publish_status()
        except (ValueError, TypeError, KeyError):
            return

    def send_odom(self):
        if not self.fresh():
            return
        self.link.odometry(self.pos, self.q, self.vb, self.rates,
                           reset_counter=self.reset_counter)
        self.sent += 1


def main():
    run(NavEKFBridge, ownership='nav_ekf_bridge')
