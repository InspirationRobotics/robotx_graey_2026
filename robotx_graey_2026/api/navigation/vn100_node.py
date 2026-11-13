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
import json
import time

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
import serial

from robotx_graey_2026.api.node_util import run
from .vn_protocol import relative_heading, checked, transact
from .vn_calibration import VNCalibration
from .maintenance_safety import require_maintenance, calibration_reason

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
        self.declare_parameter('port', DEFAULT_PORT,ParameterDescriptor(read_only=True,description='Serial owner endpoint; restart required'))
        self.declare_parameter('baud',115200,ParameterDescriptor(read_only=True,description='Serial wire rate; restart required'))
        # Cached frame settings must not accept runtime writes that would only
        # change parameter readback, leaving the published transform unchanged.
        for name, default in (('flip_180', True), ('yaw_offset_deg', 0.0)):
            # rclpy fills in descriptor name/type; each parameter needs its own.
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True,
                description='Startup-only frame setting; restart with a parameter override.'))
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.flip = self.get_parameter('flip_180').value
        self.yaw_off = self.get_parameter('yaw_offset_deg').value

        self.pub_imu = self.create_publisher(Imu, '/graey/vn100/imu', 10)
        self.pub_hdg = self.create_publisher(Float32, '/graey/vn100/heading', 10)
        # Navigation retains the full-rate typed feeds above.  The GUI only
        # needs a bounded latest-value view, so avoid making it process both
        # high-rate topics independently.
        self.pub_gui = self.create_publisher(String, '/graey/vn100/gui', 1)
        self.gui_latest = None
        self.gui_published = -float('inf')
        self.ser = None
        self.calibration = VNCalibration()
        self.calibration_pub = self.create_publisher(String, '/graey/vn100/calibration', 10)
        self.create_service(Trigger, '/graey/vn100/calibrate_start', self.calibrate_start)
        self.create_service(Trigger, '/graey/vn100/calibrate_cancel', self.calibrate_cancel)
        self.buf = b''
        self.last_hdg = None
        self.create_timer(0.01, self.tick)
        self.create_timer(0.1, self.publish_gui)
        self.create_timer(2.0, self.report)
        self.create_timer(.5, self.calibration_tick)

    def calibrate_start(self, request, response):
        try:
            require_maintenance(check=calibration_reason)
            if self.ser is None:
                raise ValueError('VN serial unavailable')
            self.calibration.start(self.ser)
            response.success, response.message = True, 'Rotate now; keep disarmed and ESC power off. VN odometry input is withheld during calibration.'
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def calibrate_cancel(self, request, response):
        try:
            if self.ser is not None:
                self.calibration.stop(self.ser, 'Operator stopped calibration; repeat before accepting it', failed=True)
            response.success, response.message = True, self.calibration.reason
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def calibration_tick(self):
        if self.calibration.running:
            try:
                require_maintenance(check=calibration_reason)
                if self.ser is None:
                    raise OSError('VN serial lost during calibration')
                if time.monotonic()-self.calibration.started >= self.calibration.seconds:
                    self.calibration.stop(self.ser)
            except (OSError, ValueError) as error:
                try:
                    self.calibration.stop(self.ser, str(error), failed=True)
                except (OSError, AttributeError):
                    self.calibration.phase = 'failed'
                    self.calibration.reason = 'VN calibration freeze failed; inspect sensor before use'
        self.calibration_pub.publish(String(data=json.dumps(self.calibration.status())))

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.calibration.stop(self.ser, 'Sensor node stopped during calibration', failed=True)
                self.ser.close()
        finally:
            super().destroy_node()

    def report(self):
        self.get_logger().debug(f'heading={self.last_hdg}')

    def publish_gui(self):
        latest = self.gui_latest
        if latest is None or latest['received_monotonic'] <= self.gui_published:
            return
        self.pub_gui.publish(String(data=json.dumps(latest, separators=(',', ':'))))
        self.gui_published = latest['received_monotonic']

    def tick(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                relative_heading(self.ser)
                # Reconnection must not silently resume a possibly interrupted
                # HSI run left active in the sensor after a process/link failure.
                control = transact(self.ser, 'VNRRG,44')
                if len(control) != 3:
                    raise OSError('Unexpected VN calibration control reply')
                expected = ['0','3',control[2]]
                if control != expected:
                    transact(self.ser, 'VNWRG,44,'+','.join(expected))
                self.buf = b''
                self.get_logger().info(f'VN-100 open on {self.port}')
            except OSError as e:
                if self.ser is not None:
                    self.ser.close()
                    self.ser = None
                self.get_logger().warn(f'VN-100 open failed: {e}')
                return
        try:
            self.buf += self.ser.read(512)
        except OSError as e:
            self.get_logger().warn(f'VN-100 read failed: {e}')
            try:
                self.ser.close()
            except OSError:
                pass
            self.ser = None
            self.buf = b''
            return
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.parse(line.strip())

    def parse(self, line):
        txt = line.decode('ascii', 'ignore')
        if not txt.startswith('$VNYBA'):
            return
        try:
            f = checked(line)
        except ValueError:
            return
        if len(f) < 10:
            return
        try:
            yaw, pitch, roll = float(f[1]), float(f[2]), float(f[3])
            ax, ay, az = float(f[4]), float(f[5]), float(f[6])
            gx, gy, gz = float(f[7]), float(f[8]), float(f[9])
        except ValueError:
            return
        if not all(math.isfinite(v) for v in (yaw, pitch, roll, ax, ay, az, gx, gy, gz)):
            return
        if self.calibration.running:
            self.calibration.sample((yaw, pitch, roll))
            return
        if self.calibration.phase == 'failed':
            return  # Do not publish attitude from a failed/incomplete calibration.

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
        self.gui_latest = dict(
            received_monotonic=time.monotonic(),
            heading=float(hdg),
            orientation=list(q),
            angular_velocity=[gx, gy, gz],
            linear_acceleration=[ax, ay, az],
            frame=m.header.frame_id,
        )


def main():
    run(VN100Node, ownership='vn100_node')
