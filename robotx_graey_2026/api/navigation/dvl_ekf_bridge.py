#!/usr/bin/env python3
"""Feed DVL velocity into ArduSub's EKF.

Subscribes to /graey/dvl/velocity and /graey/dvl/valid, integrates velocity
into body-frame position deltas, and sends VISION_POSITION_DELTA to the Cube
through MAVProxy's local UDP port (14551).

Mounting correction: params swap_xy / flip_x / flip_y / flip_z map the DVL's
axes onto the vehicle body frame (x forward, y right, z down).
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovarianceStamped
import os
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

mavutil.set_dialect('ardupilotmega')


class DVLEKFBridge(Node):
    def __init__(self):
        super().__init__('dvl_ekf_bridge')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14551')
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('flip_z', False)
        self.declare_parameter('min_confidence', 20.0)

        self.swap_xy = self.get_parameter('swap_xy').value
        self.flip_x = self.get_parameter('flip_x').value
        self.flip_y = self.get_parameter('flip_y').value
        self.flip_z = self.get_parameter('flip_z').value
        self.min_conf = self.get_parameter('min_confidence').value

        conn = self.get_parameter('mavlink').value
        self.get_logger().info(f'opening MAVLink {conn}')
        self.mav = mavutil.mavlink_connection(conn, source_system=1, source_component=197)

        self.valid = False
        self.last_stamp = None
        self.sent = 0

        self.create_subscription(Bool, '/graey/dvl/valid', self.on_valid, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/graey/dvl/velocity',
                                 self.on_velocity, 10)
        self.create_timer(5.0, self.report)

    def on_valid(self, msg):
        self.valid = msg.data

    def report(self):
        self.get_logger().info(f'valid={self.valid} sent={self.sent} v={getattr(self,"last_vel",None)}')

    def on_velocity(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0 or dt > 1.0:
            return

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        if self.swap_xy:
            vx, vy = vy, vx
        if self.flip_x:
            vx = -vx
        if self.flip_y:
            vy = -vy
        if self.flip_z:
            vz = -vz

        if not self.valid:
            return                     # no bottom lock - do not feed the EKF
        conf = 100.0
        self.last_vel = (vx, vy, vz)

        self.mav.mav.vision_position_delta_send(
            0,                       # time_usec (0 = autopilot timestamps it)
            int(dt * 1e6),           # time_delta_usec
            [0.0, 0.0, 0.0],         # angle_delta - DVL measures no rotation
            [vx * dt, vy * dt, vz * dt],
            conf)
        self.sent += 1


def main():
    rclpy.init()
    node = DVLEKFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
