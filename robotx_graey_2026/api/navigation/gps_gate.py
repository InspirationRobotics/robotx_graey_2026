"""Exclusive Jetson GPS admission transport. Disabled unless commissioned.

Consumes only physical instance-0 GPS_RAW_INT from Cube 1/1. GPS2_RAW and
GLOBAL_POSITION_INT cannot feed back into this gate. No parameter/mode writes.
"""
import json
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterDescriptor
from robotx_graey_2026.api.steady_node import SteadyNode
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link
from .gps_admission import GPSAdmission, gps_input_args
from .continuous_profile import PROFILE


class GPSGate(SteadyNode):
    def __init__(self):
        super().__init__('gps_gate')
        for name, value in dict(enabled=False, mavlink='udpout:127.0.0.1:14558').items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.enabled = self.get_parameter('enabled').value
        self.link = Link(self.get_parameter('mavlink').value, 200, self.get_logger())
        self.gate = GPSAdmission()
        self.params = {}
        self.vn = None
        self.vn_stamp = self.dvl_stamp = 0
        self.dvl_sensor = None
        self.last_forwarded = None
        self.forwarded_fix = None
        self.forwarded_message = None
        self.pub = self.create_publisher(String, '/graey/navigation/gps_admission', 10)
        self.create_subscription(Imu, '/graey/vn100/imu', self.imu, 20)
        self.create_subscription(String, '/graey/dvl/sample', self.dvl, 20)
        self.create_timer(.02, self.tick)
        self.create_timer(1., self.request)

    def imu(self, m):
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if stamp <= self.vn_stamp:
            return
        self.vn_stamp = stamp
        q = m.orientation
        norm = sum(v*v for v in (q.w, q.x, q.y, q.z))
        self.vn = time.monotonic() if math.isfinite(norm) and .9 < norm < 1.1 else None

    def dvl(self, m):
        now = time.monotonic()
        try:
            d = json.loads(m.data)
            stamp, sensor = d['stamp_ns'], d.get('sensor_time')
            if stamp <= self.dvl_stamp or (sensor is not None and sensor == self.dvl_sensor):
                return
            self.dvl_stamp, self.dvl_sensor = stamp, sensor
            velocity = tuple(float(v) for v in d['velocity'])
            self.gate.quality.motion(now, velocity, d['valid'] is True and len(velocity) == 3)
        except (ValueError, TypeError, KeyError):
            self.gate.quality.motion(now, (), False)

    def request(self):
        self.link.heartbeat()
        stamp = self.gate.clock.request(time.monotonic_ns())
        self.link.send('timesync_send', 0, stamp)
        self.link.command(511, 24, 100000)
        for name in PROFILE:
            self.link.send('param_request_read_send', 1, 1, name.encode(), -1)

    def configured(self, now):
        return all(name in self.params and self.params[name][1] == value
                   and 0 <= now-self.params[name][0] < 3.
                   for name, value in PROFILE.items())

    def consume(self, kind, m):
        if (m.get_srcSystem(), m.get_srcComponent()) != (1, 1):
            return
        now = time.monotonic()
        if kind == 'TIMESYNC' and m.tc1:
            self.gate.clock.reply(m.ts1, m.tc1, time.monotonic_ns())
        elif kind == 'PARAM_VALUE':
            self.params[m.param_id.rstrip('\x00')] = (now, m.param_value)
        elif kind == 'GPS_RAW_INT':
            fix = dict(cube_fix_time_us=m.time_usec, lat=m.lat/1e7,
                       lon=m.lon/1e7, alt=m.alt/1000, fix=m.fix_type,
                       accuracy=getattr(m, 'h_acc', 0)/1000, received=now)
            fix.update(speed_mps=m.vel/100 if m.vel < 65535 else None,
                       course_deg=m.cog/100 if m.cog < 36000 else None,
                       speed_accuracy_mps=getattr(m, 'vel_acc', 0)/1000 or None)
            admitted = self.gate.check(now, fix, self.vn is not None and 0 <= now-self.vn < .5)
            if admitted and self.enabled and self.configured(now):
                self.link.send('gps_input_send', *gps_input_args(m))
                self.last_forwarded, self.forwarded_fix = now, fix
                self.forwarded_message = m

    def tick(self):
        self.link.drain(self.consume)
        now = time.monotonic()
        status = self.gate.status(now)
        if not status['qualified'] and self.forwarded_message is not None:
            if self.enabled and self.configured(now):
                # Withdraw the fix promptly. Same timestamp/coordinates, NO_FIX;
                # this is not a synthetic valid measurement or a zero velocity.
                self.link.send('gps_input_send', *gps_input_args(self.forwarded_message, valid=False))
            self.forwarded_message = None
            self.last_forwarded = None
        status.update(enabled=self.enabled, configuration_verified=self.configured(now),
            forwarded_recently=self.last_forwarded is not None and 0 <= now-self.last_forwarded < .35,
            fix=self.forwarded_fix, published_monotonic=now,
            fusion_confirmed=False)
        self.pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    run(GPSGate, ownership='gps_gate')
