"""Synthetic VN-100, DVL and depth topics derived from ArduSub SITL telemetry.

This is test-only glue. It is not launched by core.launch.py and must never be
pointed at the real vehicle. The feedback is intentionally deterministic: SITL
attitude/local velocity stand in for sensor truth so the production
nav_ekf_bridge and navigation_supervisor can be exercised end to end.
"""
import json
import math
import time

from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil


def qrotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    return (vx*(1-2*(y*y+z*z))+vy*2*(x*y-z*w)+vz*2*(x*z+y*w),
            vx*2*(x*y+z*w)+vy*(1-2*(x*x+z*z))+vz*2*(y*z-x*w),
            vx*2*(x*z-y*w)+vy*2*(y*z+x*w)+vz*(1-2*(x*x+y*y)))


class SitlSensorSim(Node):
    def __init__(self):
        super().__init__('sitl_sensor_sim')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14558')
        self.link = Link(self.get_parameter('mavlink').value, 201, self.get_logger())
        self.imu_pub = self.create_publisher(Imu, '/graey/vn100/imu', 20)
        self.dvl_pub = self.create_publisher(String, '/graey/dvl/sample', 20)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, '/graey/dvl/velocity', 20)
        self.valid_pub = self.create_publisher(Bool, '/graey/dvl/valid', 10)
        self.depth_pub = self.create_publisher(Float32, '/graey/sitl/depth_m', 10)
        self.latest = {}
        self.truth_previous = None
        self.truth_velocity = (0.0, 0.0, 0.0)
        self.last_ground_depth = None
        self.last_publish = 0.0
        self.last_request = 0.0
        self.create_timer(.01, self.tick)

    def tick(self):
        now = time.monotonic()
        if now-self.last_request > 2:
            for msg_id in (32, 33, 164):
                self.link.command(511, msg_id, 50000)
            self.last_request = now
        self.link.drain(self.consume)
        attitude, local = self.latest.get('attitude'), self.latest.get('local')
        if (not attitude or not 0 <= now-attitude['received'] < .5
                or now-self.last_publish < .05):
            return
        if local and not 0 <= now-local['received'] < .5:
            local = None
        self.last_publish = now
        q = attitude['q']
        world_velocity = self.truth_velocity
        if local:
            depth_velocity = local['velocity'][2]
            world_velocity = (world_velocity[0], world_velocity[1], depth_velocity)
        body_velocity = qrotate((q[0], -q[1], -q[2], -q[3]), world_velocity)
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec*1_000_000_000+stamp.nanosec
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'vn100_sitl'
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = attitude['rates']
        self.imu_pub.publish(imu)
        sample = dict(stamp_ns=stamp_ns, velocity=body_velocity, valid=True,
                      sensor_time=int(now*1_000_000))
        self.dvl_pub.publish(String(data=json.dumps(sample, allow_nan=False)))
        self.valid_pub.publish(Bool(data=True))
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = 'dvl_sitl'
        twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z = body_velocity
        self.vel_pub.publish(twist)
        # Local NED Z is the SITL water-depth proxy; production still uses the
        # Cube's configured calibrated pressure message.
        if local:
            self.depth_pub.publish(Float32(data=local['position'][2]))

    def consume(self, kind, msg):
        if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
            return
        if kind == 'SIMSTATE':
            q = self.euler_to_quat(msg.roll, msg.pitch, msg.yaw)
            if all(math.isfinite(x) for x in q):
                self.latest['attitude'] = dict(q=q, rates=(0.0, 0.0, 0.0), received=time.monotonic())
                now = time.monotonic()
                lat, lon = msg.lat/1e7, msg.lng/1e7
                previous = self.truth_previous
                if previous:
                    dt = now-previous[0]
                    if .02 <= dt <= 1.0:
                        vn = (lat-previous[1])*111_320/dt
                        ve = ((lon-previous[2]+180)%360-180)*111_320*math.cos(math.radians(lat))/dt
                        self.truth_velocity = (vn, ve, self.truth_velocity[2])
                self.truth_previous = (now, lat, lon)
        elif kind == 'LOCAL_POSITION_NED':
            values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            if all(math.isfinite(x) for x in values):
                self.latest['local'] = dict(position=values[:3], velocity=values[3:], received=time.monotonic())

    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)


def main():
    run(SitlSensorSim)
