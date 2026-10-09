#!/usr/bin/env python3
"""Stand-ins for vn100_node and dvl_node in the simulator.

ArduSub SITL knows exactly where the simulated sub is and how it is moving, and
reports it in SIM_STATE. This publishes that on the same topics, in the same
units and frames, as Graey's real VN-100 and DVL nodes:

  /graey/vn100/imu       sensor_msgs/Imu       attitude, body FRD -> NED
  /graey/vn100/heading   std_msgs/Float32      degrees, 0-360
  /graey/dvl/velocity    TwistWithCovariance   m/s in the body frame (forward, right, down)
  /graey/dvl/valid       std_msgs/Bool         bottom lock
  /graey/dvl/altitude    std_msgs/Float32      metres above the bottom, -1 = no lock

so nav_ekf_bridge, the EKF's external-nav input, and the sonar map all run
exactly as they do on the sub. Optional noise (--dvl-noise, m/s) makes the dead
reckoning drift the way the real one does.
"""
import argparse
import math
import random

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32

from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil

SIM_STATE_ID = 108


def rotate_to_body(q, v):
    """NED vector into the body frame: the inverse of the (w, x, y, z) attitude."""
    w, x, y, z = q
    # rotation matrix body->world, transposed
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    return tuple(sum(r[i][j] * v[i] for i in range(3)) for j in range(3))


class SimSensors(Node):
    def __init__(self, args):
        super().__init__("sim_sensors")
        self.args = args
        self.link = Link(args.mavlink, 199, self.get_logger())
        self.pub_imu = self.create_publisher(Imu, "/graey/vn100/imu", 10)
        self.pub_hdg = self.create_publisher(Float32, "/graey/vn100/heading", 10)
        self.pub_vel = self.create_publisher(TwistWithCovarianceStamped, "/graey/dvl/velocity", 10)
        self.pub_valid = self.create_publisher(Bool, "/graey/dvl/valid", 10)
        self.pub_alt = self.create_publisher(Float32, "/graey/dvl/altitude", 10)
        self.state = None
        self.create_timer(0.005, self.pump)
        self.create_timer(1.0 / args.imu_hz, self.publish_imu)
        self.create_timer(1.0 / args.dvl_hz, self.publish_dvl)
        self.create_timer(5.0, self.ask_for_sim_state)
        self.ask_for_sim_state()

    def ask_for_sim_state(self):
        if self.state is None:
            self.link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                              SIM_STATE_ID, 1e6 / self.args.imu_hz, 0, 0, 0, 0, 0)

    def pump(self):
        def handle(kind, m):
            if kind == "SIM_STATE":
                if self.state is None:
                    self.get_logger().info("simulator truth arriving")
                self.state = m
        self.link.drain(handle)

    def publish_imu(self):
        m = self.state
        if m is None:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vn100"
        msg.orientation.w, msg.orientation.x = m.q1, m.q2
        msg.orientation.y, msg.orientation.z = m.q3, m.q4
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = m.xgyro, m.ygyro, m.zgyro
        msg.linear_acceleration.x, msg.linear_acceleration.y = m.xacc, m.yacc
        msg.linear_acceleration.z = m.zacc
        self.pub_imu.publish(msg)
        self.pub_hdg.publish(Float32(data=float(math.degrees(m.yaw) % 360.0)))

    def publish_dvl(self):
        m = self.state
        if m is None:
            return
        # SITL puts the water surface at the home altitude, which inside.sh sets
        # to 0. The real DVL sits below the waterline and keeps bottom lock with
        # the sub on the surface, so lock only depends on height above the bottom.
        depth = -m.alt
        altitude = self.args.water_depth - depth
        valid = 0.1 < altitude < 50.0
        vb = rotate_to_body((m.q1, m.q2, m.q3, m.q4), (m.vn, m.ve, m.vd))
        n = self.args.dvl_noise
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dvl"
        msg.twist.twist.linear.x = vb[0] + random.gauss(0, n)
        msg.twist.twist.linear.y = vb[1] + random.gauss(0, n)
        msg.twist.twist.linear.z = vb[2] + random.gauss(0, n)
        self.pub_vel.publish(msg)
        self.pub_valid.publish(Bool(data=valid))
        self.pub_alt.publish(Float32(data=float(altitude if valid else -1.0)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mavlink", default="udpout:127.0.0.1:14557")
    p.add_argument("--water-depth", type=float, default=12.0, help="metres, surface to bottom")
    p.add_argument("--imu-hz", type=float, default=50.0)
    p.add_argument("--dvl-hz", type=float, default=8.0)
    p.add_argument("--dvl-noise", type=float, default=0.0, help="m/s, per axis")
    args, _ = p.parse_known_args()
    rclpy.init()
    rclpy.spin(SimSensors(args))


if __name__ == "__main__":
    main()
