#!/usr/bin/env python3
"""VectorNav VN-100 -> ROS 2.

Parses $VNYBA (yaw, pitch, roll, gravity-free body accel, angular rates) at
115200 and publishes:
  /graey/vn100/imu      sensor_msgs/Imu
  /graey/vn100/heading  std_msgs/Float32   degrees, 0-360

The unit is mounted UPSIDE DOWN on Graey (raw roll reads ~180), so flip_180
rotates the reading into the vehicle body frame.
"""
import math

from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import serial

from robotx_graey_2026.api.node_util import run

DEFAULT_PORT = '/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9DQE-if00-port0'


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class VN100Node(Node):
    def __init__(self):
        super().__init__('vn100_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('flip_180', True)    # unit is mounted upside down
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.flip = self.get_parameter('flip_180').value
        self.yaw_off = self.get_parameter('yaw_offset_deg').value

        self.pub_imu = self.create_publisher(Imu, '/graey/vn100/imu', 10)
        self.pub_hdg = self.create_publisher(Float32, '/graey/vn100/heading', 10)
        self.ser = None
        self.buf = b''
        self.last_hdg = None
        self.create_timer(0.01, self.tick)
        self.create_timer(2.0, self.report)

    def report(self):
        self.get_logger().debug(f'heading={self.last_hdg}')

    def tick(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                self.get_logger().info(f'VN-100 open on {self.port}')
            except OSError as e:
                self.get_logger().warn(f'VN-100 open failed: {e}')
                return
        try:
            self.buf += self.ser.read(512)
        except OSError as e:
            self.get_logger().warn(f'VN-100 read failed: {e}')
            self.ser = None
            return
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.parse(line.strip())

    def parse(self, line):
        txt = line.decode('ascii', 'ignore')
        if not txt.startswith('$VNYBA'):
            return
        f = txt.split('*')[0].split(',')             # drop the checksum, then split fields
        if len(f) < 10:
            return
        try:
            yaw, pitch, roll = float(f[1]), float(f[2]), float(f[3])
            ax, ay, az = float(f[4]), float(f[5]), float(f[6])
            gx, gy, gz = float(f[7]), float(f[8]), float(f[9])
        except ValueError:
            return

        if self.flip:
            roll = (roll + 180.0 + 180.0) % 360.0 - 180.0   # +180 wrapped to [-180,180]
            pitch = -pitch
            ay, az = -ay, -az
            gy, gz = -gy, -gz
        hdg = (yaw + self.yaw_off) % 360.0
        self.last_hdg = round(hdg, 2)

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'vn100'
        q = euler_to_quat(math.radians(roll), math.radians(pitch), math.radians(hdg))
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
        self.pub_imu.publish(m)
        self.pub_hdg.publish(Float32(data=float(hdg)))


def main():
    run(VN100Node)
