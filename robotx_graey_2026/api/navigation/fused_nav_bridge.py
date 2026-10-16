"""Single sensor-ingress and ODOMETRY owner for continuous navigation.

Does not send GPS_INPUT, source-selection, origin-reset or vehicle-control commands.
Readbacks gate odometry transmission; startup defaults cannot affect Cube fusion.
"""
import json
import math
import time
import uuid
import os
import statistics
from collections import deque
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParametersAtomically
from robotx_graey_2026.api.steady_node import SteadyNode
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link
from .bridge_estimator import BridgeEstimator, yaw
from .odometry_profile import PROFILE
from .maintenance_safety import require_maintenance


class FusedNavBridge(SteadyNode):
    def __init__(self):
        super().__init__('nav_ekf_bridge')
        defaults = dict(enabled=False, mavlink='udpout:127.0.0.1:14551',
                        surface_pressure_hpa=0., water_density=1000.,
                        sitl_depth_topic='',
                        auto_north_alignment=False,
                        require_disarmed_start=False,
                        heading_reference_validated=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = {k: self.get_parameter(k).value for k in defaults}
        if self.settings['sitl_depth_topic'] and os.environ.get('GRAEY_RUNTIME_SCOPE') != 'sitl':
            raise ValueError('Synthetic depth is restricted to isolated SITL')
        self.est = BridgeEstimator(self.settings['heading_reference_validated'],
                                   self.settings['auto_north_alignment'])
        self.link = Link(self.settings['mavlink'], 197, self.get_logger())
        self.instance = str(uuid.uuid4())
        self.stamps, self.params = {}, {}
        self.last_sent = None
        self.boot_ms = None
        self.start_checked = not self.settings['require_disarmed_start']
        self.output_permitted = self.settings['enabled']
        self.reset_counter = 0
        self.pressures = deque(maxlen=80)
        self.vn_calibration = None
        self.pressure_sample = None
        self.pub = self.create_publisher(String, '/graey/navigation/bridge_status', 10)
        self.create_subscription(Imu, '/graey/vn100/imu', self.imu, 20)
        self.create_subscription(String, '/graey/dvl/sample', self.dvl, 20)
        self.create_subscription(String, '/graey/vn100/calibration', self.calibration_status, 10)
        self.create_service(Trigger, '/graey/navigation/pause_odometry', self.pause_odometry)
        self.create_service(SetBool, '/graey/navigation/prepare', self.prepare)
        self.create_service(SetParametersAtomically, '/graey/navigation/align_heading', self.align_heading)
        if self.settings['sitl_depth_topic']:
            self.create_subscription(Float32, self.settings['sitl_depth_topic'],
                lambda m: self.est.pressure_depth(time.monotonic(), m.data), 20)
        # Service TIMESYNC faster than its 50 ms RTT acceptance bound. Keep
        # ODOMETRY/status at 20 Hz separately below.
        self.create_timer(.01, self.tick)
        self.create_timer(1., self.request)

    def calibration_status(self, msg):
        try:
            self.vn_calibration = (time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            self.vn_calibration = None

    def align_heading(self, request, response):
        try:
            require_maintenance()
            if not self.output_permitted:
                raise ValueError('Run Prepare before capturing north')
            if (len(request.parameters)!=1 or request.parameters[0].name!='true_heading_deg'
                    or request.parameters[0].value.type!=3):
                raise ValueError('Expected one double true_heading_deg')
            response.result.reason=self.est.align_known_heading(time.monotonic(),request.parameters[0].value.double_value)
            self.get_logger().info('NORTH_REFERENCE '+json.dumps(self.est.north_alignment.status()))
            response.result.successful=True
        except (ValueError,OSError) as error:
            response.result.successful=False
            response.result.reason=str(error)
        return response

    def pause_odometry(self, request, response):
        try:
            require_maintenance()
            self.output_permitted = False
            self.est.fault = 'Navigation maintenance; Prepare required before reuse'
            response.success, response.message = True, 'Odometry paused for maintenance'
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def prepare(self, request, response):
        try:
            require_maintenance()
            now = time.monotonic()
            if request.data is not True:
                raise ValueError('Operator must confirm floating at the water surface')
            if not self.vn_calibration or not 0 <= now-self.vn_calibration[0] < 1.5:
                raise ValueError('Fresh VN calibration state unavailable')
            if self.vn_calibration[1].get('phase') not in ('idle','collected'):
                raise ValueError('VN calibration incomplete, failed or still running')
            rows = [r for r in self.pressures if 0 <= now-r[0] < 3.]
            if len(rows) < 15 or rows[-1][0]-rows[0][0] < 1.5 or now-rows[-1][0] > .35:
                raise ValueError('Wait for at least 1.5 seconds of fresh pressure samples')
            if not self.configured(now):
                raise ValueError('Cube external-navigation profile unavailable or mismatched')
            base = statistics.median(r[1] for r in rows)
            if max(r[1] for r in rows)-min(r[1] for r in rows) > 8.:
                raise ValueError('Pressure changing too much; hold stationary at the surface and retry')
            clock, origin = self.est.gps.clock, self.est.origin
            self.output_permitted = False
            self.est = BridgeEstimator(auto_north_alignment=True)
            self.est.gps.clock = clock
            if origin is not None:
                self.est.set_origin(origin)
            self.settings['surface_pressure_hpa'] = base
            self.reset_counter = (self.reset_counter+1)%256
            self.last_sent = None
            self.start_checked = True
            self.output_permitted = True  # Health/north/profile gates still withhold output.
            response.success, response.message = True, f'Surface baseline {base:.3f} hPa; waiting for GPS/DVL north alignment'
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def advancing(self, key, stamp):
        if stamp <= 0 or stamp <= self.stamps.get(key, -1):
            return False
        self.stamps[key] = stamp
        return True

    def imu(self, m):
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        clock = self.steady_clock if self.settings.get('sitl_depth_topic') else self.get_clock()
        if not 0 <= clock.now().nanoseconds-stamp < 500000000:
            self.est.imu_at = None
            return
        if self.advancing('imu', stamp):
            q, r = m.orientation, m.angular_velocity
            self.est.imu(time.monotonic(), (q.w, q.x, q.y, q.z), (r.x, r.y, r.z))

    def dvl(self, m):
        try:
            d = json.loads(m.data)
            clock = self.steady_clock if self.settings.get('sitl_depth_topic') else self.get_clock()
            if not 0 <= clock.now().nanoseconds-d['stamp_ns'] < 500000000:
                self.est.dvl(time.monotonic(), (), False)
                return
            if not self.advancing('dvl', d['stamp_ns']):
                return
            sensor = d.get('sensor_time')
            if sensor is not None and sensor <= self.stamps.get('dvl_sensor', -math.inf):
                return
            if sensor is not None:
                self.stamps['dvl_sensor'] = sensor
            self.est.dvl(time.monotonic(), tuple(float(v) for v in d['velocity']), d['valid'] is True)
        except (KeyError, TypeError, ValueError):
            self.est.dvl(time.monotonic(), (), False)

    def request(self):
        self.link.heartbeat()
        self.link.send('timesync_send', 0, self.est.gps.clock.request(time.monotonic_ns()))
        for mid in (24, 137, 49):
            self.link.command(511, mid, 100000)
        for name in PROFILE:
            self.link.send('param_request_read_send', 1, 1, name.encode(), -1)

    def configured(self, now):
        return all(k in self.params and self.params[k][1] == v and 0 <= now-self.params[k][0] < 3.
                   for k, v in PROFILE.items())

    def consume(self, kind, m):
        if (m.get_srcSystem(), m.get_srcComponent()) != (1, 1):
            return
        now = time.monotonic()
        if kind == 'HEARTBEAT' and not self.start_checked:
            self.start_checked = True
            if m.base_mode & 128:
                self.est.fault = 'Bridge started while armed; disarm and restart preparation'
        elif kind == 'TIMESYNC' and m.tc1:
            self.est.gps.clock.reply(m.ts1, m.tc1, time.monotonic_ns())
        elif kind == 'PARAM_VALUE':
            self.params[m.param_id.rstrip('\x00')] = (now, m.param_value)
        elif kind == 'GPS_GLOBAL_ORIGIN':
            self.est.set_origin((m.latitude/1e7, m.longitude/1e7, m.altitude/1000))
        elif kind == 'SCALED_PRESSURE2':
            if self.settings.get('sitl_depth_topic'):
                return
            if self.boot_ms is not None and m.time_boot_ms+1000 < self.boot_ms:
                self.est.fault = 'Cube restarted; revalidate navigation frame'
                self.params.clear()
            self.boot_ms = m.time_boot_ms
            if not self.advancing('pressure', m.time_boot_ms):
                return
            age = self.est.gps.clock.age(m.time_boot_ms*1000, now)
            if age is None or age > .35:
                self.est.pressure_depth(now, math.nan)
                return
            self.pressure_sample = dict(hpa=m.press_abs if math.isfinite(m.press_abs) else None,
                                        received=now, cube_boot_ms=m.time_boot_ms)
            if math.isfinite(m.press_abs):
                self.pressures.append((now, m.press_abs))
            base, rho = self.settings['surface_pressure_hpa'], self.settings['water_density']
            depth = (m.press_abs-base)*100/(rho*9.80665) if base > 0 and rho > 0 else math.nan
            self.est.pressure_depth(now, depth)
        elif kind == 'GPS_RAW_INT':
            fix = dict(cube_fix_time_us=m.time_usec, received=now,
                       lat=m.lat/1e7, lon=m.lon/1e7, alt=m.alt/1000,
                       fix=m.fix_type, accuracy=getattr(m, 'h_acc', 0)/1000,
                       speed_mps=m.vel/100 if m.vel < 65535 else None,
                       course_deg=m.cog/100 if m.cog < 36000 else None,
                       speed_accuracy_mps=getattr(m, 'vel_acc', 0)/1000 or None)
            previous = self.est.gps.last_fix
            self.est.observe_gps(now, fix)
            if previous != m.time_usec:
                self.get_logger().info('GPS_DECISION '+json.dumps(self.est.event, allow_nan=False))

    def tick(self):
        self.link.drain(self.consume)
        now = time.monotonic()
        status = self.est.status(now)
        configured = self.configured(now)
        if self.start_checked and self.output_permitted and configured and status['healthy'] and (self.last_sent is None or now-self.last_sent >= .05):
            self.link.odometry(self.est.position, self.est.orientation(), self.est.velocity(),
                               self.est.rates, reset_counter=self.reset_counter, covariance=self.est.covariance(), quality=80)
            self.last_sent = now
        status.update(instance=self.instance, fusion_policy='bridge_odometry',
                      clock_mapping=dict(offset=self.est.gps.clock.offset,
                          uncertainty=self.est.gps.clock.uncertainty, updated=self.est.gps.clock.updated),
                      pressure_sample=getattr(self, 'pressure_sample', None),
                      profile_mismatches={k: dict(expected=v, actual=self.params.get(k, (None, None))[1])
                          for k, v in PROFILE.items() if k not in self.params or self.params[k][1] != v},
                      configuration_verified=configured, enabled=self.output_permitted,
                      surface_pressure_hpa=self.settings['surface_pressure_hpa'],
                      startup_output_enabled=self.settings['enabled'],
                      transmitting=self.last_sent is not None and 0 <= now-self.last_sent < .15,
                      published_monotonic=now, reset_counter=self.reset_counter,
                      inputs=dict(vn=self.est.fresh(self.est.imu_at, now),
                                  dvl=self.est.motion_valid and self.est.fresh(self.est.motion_at, now)),
                      heading_vn=dict(received=self.est.imu_at, yaw=yaw(self.est.orientation()) % 360,
                                      rate=math.degrees(self.est.rates[2])),
                      heading_dvl=dict(received=self.est.motion_at, velocity=self.est.velocity(),
                                       valid=self.est.motion_valid))
        if now-getattr(self, 'last_published', -math.inf) >= .05:
            self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
            self.last_published = now


def main():
    run(FusedNavBridge, ownership='nav_ekf_bridge')
