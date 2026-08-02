#!/usr/bin/env python3
"""Water Linked DVL A50 -> ROS 2.

Reads the JSON velocity stream (TCP 16171) and publishes:
  /graey/dvl/velocity   geometry_msgs/TwistWithCovarianceStamped  (m/s, DVL frame)
  /graey/dvl/valid      std_msgs/Bool     velocity_valid from the DVL
  /graey/dvl/altitude   std_msgs/Float32  metres above the bottom (-1 = no lock)
Auto-reconnects if the socket drops.
"""
import json
import socket

from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import TwistWithCovarianceStamped

from robotx_graey_2026.api.node_util import run


class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node')
        self.declare_parameter('ip', '192.168.2.10')
        self.declare_parameter('port', 16171)
        self.declare_parameter('scale', 1.0)        # measured 1.0 wet; RoboSub's 1.15385 does not apply

        self.ip = self.get_parameter('ip').value
        self.port = self.get_parameter('port').value
        self.scale = self.get_parameter('scale').value

        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, '/graey/dvl/velocity', 10)
        self.pub_valid = self.create_publisher(Bool, '/graey/dvl/valid', 10)
        self.pub_alt = self.create_publisher(Float32, '/graey/dvl/altitude', 10)

        self.sock = None
        self.buf = b''
        self.create_timer(0.02, self.spin_once)

    def connect(self):
        try:
            self.sock = socket.create_connection((self.ip, self.port), timeout=5)
            self.sock.settimeout(0.1)
            self.buf = b''
            self.get_logger().info(f'connected to DVL {self.ip}:{self.port}')
        except OSError as e:
            self.get_logger().warn(f'DVL connect failed: {e}')
            self.sock = None

    def spin_once(self):
        if self.sock is None:
            self.connect()
            return
        try:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError('socket closed')
            self.buf += chunk
        except socket.timeout:
            return
        except OSError as e:
            self.get_logger().warn(f'DVL read failed, reconnecting: {e}')
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            return

        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            if line.strip():
                self.handle_report(line)

    def handle_report(self, line):
        try:
            r = json.loads(line)
        except ValueError:
            return
        if 'vx' not in r:
            return                                  # dead-reckoning report, not velocity

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dvl'
        msg.twist.twist.linear.x = float(r['vx']) * self.scale
        msg.twist.twist.linear.y = float(r['vy']) * self.scale
        msg.twist.twist.linear.z = float(r['vz']) * self.scale
        cov = r.get('covariance')
        if cov:
            for i in range(3):
                for j in range(3):
                    msg.twist.covariance[i * 6 + j] = float(cov[i][j])
        self.pub_vel.publish(msg)
        self.pub_valid.publish(Bool(data=bool(r.get('velocity_valid', False))))
        self.pub_alt.publish(Float32(data=float(r.get('altitude', -1.0))))


def main():
    run(DVLNode)
