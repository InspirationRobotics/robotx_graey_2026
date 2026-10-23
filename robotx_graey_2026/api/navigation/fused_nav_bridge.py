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
from .automatic_recovery import AutomaticRecovery
from .land_north import LandNorth
from .track_north import TrackNorth
from .relative_estimator import RelativeEstimator


class FusedNavBridge(SteadyNode):
    def __init__(self):
        super().__init__('nav_ekf_bridge')
        defaults = dict(enabled=False, mavlink='udpout:127.0.0.1:14551',
                        surface_pressure_hpa=0., water_density=1000.,
                        pressure_message='SCALED_PRESSURE3',
                        sitl_depth_topic='',
                        auto_north_alignment=False,
                        fixed_local_navigation=True,
                        require_disarmed_start=False,
                        heading_reference_validated=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = {k: self.get_parameter(k).value for k in defaults}
        if self.settings['pressure_message'] not in ('SCALED_PRESSURE','SCALED_PRESSURE2','SCALED_PRESSURE3'):
            raise ValueError('Select a verified MAVLink pressure sensor message')
        if self.settings['sitl_depth_topic'] and os.environ.get('GRAEY_RUNTIME_SCOPE') != 'sitl':
            raise ValueError('Synthetic depth is restricted to isolated SITL')
        self.est = BridgeEstimator(self.settings['heading_reference_validated'],
                                   self.settings['auto_north_alignment'])
        if self.est.north_alignment is not None:
            self.est.north_alignment = TrackNorth()
        if self.settings['fixed_local_navigation']:
            self.est=RelativeEstimator()
        self.land_north = LandNorth()
        self.raw_gps_health=None
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
        self.auto_recovery=AutomaticRecovery()
        self.control_state={}
        self.control_received=None
        self.pressure_sample = None
        self.pub = self.create_publisher(String, '/graey/navigation/bridge_status', 10)
        self.create_subscription(Imu, '/graey/vn100/imu', self.imu, 20)
        self.create_subscription(String, '/graey/dvl/sample', self.dvl, 20)
        self.create_subscription(String, '/graey/vn100/calibration', self.calibration_status, 10)
        self.create_subscription(String, '/graey/control_watchdog/status', self.control_status, 10)
        self.create_service(Trigger, '/graey/navigation/pause_odometry', self.pause_odometry)
        self.create_service(SetBool, '/graey/navigation/prepare', self.prepare)
        self.create_service(Trigger, '/graey/navigation/recover_dvl', self.recover_dvl)
        self.create_service(SetParametersAtomically, '/graey/navigation/align_heading', self.align_heading)
        self.create_service(SetParametersAtomically, '/graey/navigation/align_land', self.align_land)
        self.create_service(Trigger, '/graey/navigation/forget_land', self.forget_land)
        self.create_service(Trigger, '/graey/navigation/begin_motion_north', self.begin_motion_north)
        self.create_subscription(String, '/graey/navigation/north_maneuver', self.motion_north_status, 10)
        self.create_subscription(String, '/graey/waypoint/status', self.mission_alignment_status, 10)
        if self.settings['sitl_depth_topic']:
            self.create_subscription(Float32, self.settings['sitl_depth_topic'],
                self.sitl_depth, 20)
        # Service TIMESYNC faster than its 50 ms RTT acceptance bound. Keep
        # ODOMETRY/status at 20 Hz separately below.
        self.create_timer(.01, self.tick)
        self.create_timer(1., self.request)
        self.create_timer(.2, self.request_parameters)
        self.create_timer(.2, self.sync_clock)

    def sitl_depth(self, msg):
        # The isolated depth fixture must also exercise Prepare's pressure
        # history requirement, rather than bypassing the production service.
        now=time.monotonic()
        if not math.isfinite(msg.data) or not -.25 <= msg.data <= 100:
            self.est.pressure_depth(now,math.nan)
            return
        pressure=1013.25+msg.data*self.settings['water_density']*9.80665/100
        self.pressures.append((now,pressure))
        base=self.settings['surface_pressure_hpa']
        self.est.pressure_depth(now,(pressure-base)*100/(self.settings['water_density']*9.80665) if base>0 else math.nan)

    def calibration_status(self, msg):
        try:
            self.vn_calibration = (time.monotonic(), json.loads(msg.data))
            if self.vn_calibration[1].get('phase') not in ('idle','collected'):
                self.land_north.invalidate()
        except (ValueError, TypeError):
            self.vn_calibration = None

    def control_status(self, msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.control_state,self.control_received=data,time.monotonic()
        except (ValueError,TypeError):
            pass

    def begin_motion_north(self, request, response):
        try:
            if isinstance(self.est,RelativeEstimator):
                raise ValueError('North alignment is part of the GUIDED waypoint mission')
            require_maintenance(lambda c: '' if c.get('online') is True and c.get('vehicle_fresh') is True and c.get('mode')=='MANUAL' and c.get('armed') is True and c.get('sa_active') is False and c.get('esc_commanded_on') is True and c.get('mission_claimed') is False and c.get('autonomous') is False and not c.get('inhibit') else 'Enable propulsion in MANUAL with no mission before alignment')
            now=time.monotonic()
            if not self.output_permitted or not self.configured(now) or self.est.fault or self.est.heading_validated:
                raise ValueError('Prepare fresh unaligned navigation with the bridge-only profile first')
            if (not self.est.gps_eligible(now) or not self.est.gps.status(now)['qualified']
                    or not self.est.motion_valid or not self.est.fresh(self.est.motion_at,now)
                    or not self.est.fresh(self.est.imu_at,now)):
                raise ValueError('Qualified surface GPS and fresh valid DVL/VN required')
            self.est.north_alignment=TrackNorth()
            self.est.north_alignment.begin()
            self.motion_north_started=now
            if hasattr(self,'motion_north_received'):del self.motion_north_received
            response.success,response.message=True,'GPS coordinate alignment armed; explicit maneuver launch follows'
        except (ValueError,OSError) as error:
            response.success,response.message=False,str(error)
        return response

    def mission_alignment_status(self, msg):
        if not isinstance(self.est,RelativeEstimator):return
        try:
            m=json.loads(msg.data);now=time.monotonic()
            if not 0<=now-m.get('published_monotonic',-math.inf)<.5 or not m.get('run_id'):return
            c=self.control_state
            if (self.control_received is None or not 0<=now-self.control_received<1.
                    or not c.get('mission_claimed') or c.get('mode')!='GUIDED' or not c.get('armed')
                    or c.get('sa_active') or not c.get('esc_commanded_on')):
                if self.est.north_alignment.active:self.est.north_alignment.stop(False)
                return
            plan=m.get('plan') or {}
            self.est.target_gps=(plan['lat'],plan['lon'])
            if getattr(self,'segmented_course_run',None)!=m['run_id']:
                self.segmented_course_run=m['run_id']
                self.est.pending_course_step=None
                self.est.last_segment_correction=None
                self.est.course_heading.reset_window('New segmented mission; collect fresh correction evidence')
            if m.get('stage')=='TRANSIT_CHECK':
                status=self.est.status(now)
                v=self.est.velocity()
                # Never turn a queued correction into a stale-motion update.
                if (status.get('healthy') and self.est.motion_valid and self.est.fresh(self.est.motion_at,now)
                        and len(v)==3 and math.sqrt(sum(x*x for x in v))<=.08
                        and abs(math.degrees(self.est.rates[2]))<=3.
                        and (status.get('gps_admission') or {}).get('qualified')):
                    self.est.apply_segment_correction(now,(m['run_id'],m.get('segment_checks',0)))
            a=self.est.north_alignment
            if m.get('stage')=='ALIGN_MOVE' and getattr(self,'motion_north_run',None)!=m['run_id']:
                self.motion_north_run=m['run_id'];a.begin()
            if getattr(self,'motion_north_run',None)!=m['run_id']:return
            self.motion_north_received=now
            if m.get('stage')=='ALIGN_ACCEPT' and a.active and not a.approved:a.stop(True)
            elif m.get('stage') in ('ABORTED','COMPLETE') and a.active:a.stop(False)
        except (ValueError,KeyError,TypeError):return

    def motion_north_status(self, msg):
        try:
            m=json.loads(msg.data);now=time.monotonic()
            a=self.est.north_alignment
            if not isinstance(a,TrackNorth) or not a.active:return
            stamp=m.get('published_monotonic',-math.inf)
            if not getattr(self,'motion_north_started',math.inf)<=stamp<=now or now-stamp>.5:return
            self.motion_north_received=now
            if m.get('stage') in ('ABORTED','COMPLETE'):
                a.stop(m['stage']=='COMPLETE')
                self.get_logger().info('GPS_TRACK_NORTH '+json.dumps(dict(status=a.status(),candidate=a.candidate)))
        except (ValueError,TypeError):
            pass

    def forget_land(self, request, response):
        try:
            require_maintenance()
            if self.output_permitted:
                raise ValueError('Pause navigation before changing north reference')
            self.land_north.invalidate()
            response.success,response.message=True,'Land reference cleared; use selected surface alignment'
        except (ValueError,OSError) as error:
            response.success,response.message=False,str(error)
        return response

    def align_land(self, request, response):
        try:
            if isinstance(self.est,RelativeEstimator):
                raise ValueError('North is learned inside the GUIDED mission; Cube local frame stays fixed')
            require_maintenance()
            if self.output_permitted:
                raise ValueError('Pause navigation before capturing land north')
            now=time.monotonic()
            if (not self.vn_calibration or not 0<=now-self.vn_calibration[0]<1.5
                    or self.vn_calibration[1].get('phase') not in ('idle','collected')):
                raise ValueError('Fresh completed VN calibration state required')
            if (len(request.parameters)!=1 or request.parameters[0].name!='true_heading_deg'
                    or request.parameters[0].value.type!=3):
                raise ValueError('Expected one double true_heading_deg')
            reference=self.land_north.capture(now,request.parameters[0].value.double_value)
            self.get_logger().info('LAND_NORTH_REFERENCE '+json.dumps(reference))
            response.result.successful=True
            response.result.reason='Land north captured; keep VN powered. Prepare at water surface before use.'
        except (ValueError,OSError) as error:
            response.result.successful=False
            response.result.reason=str(error)
        return response

    def align_heading(self, request, response):
        try:
            if isinstance(self.est,RelativeEstimator):
                raise ValueError('North is learned inside the GUIDED mission; Cube local frame stays fixed')
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

    def recover_dvl(self, request, response):
        try:
            require_maintenance()
            if not self.configured(time.monotonic()):
                raise ValueError('Cube external-navigation profile unavailable or mismatched')
            message = self.est.recover_dvl(time.monotonic())
            self.reset_counter = (self.reset_counter+1)%256
            self.last_sent = None
            self.output_permitted = True
            self.get_logger().info('DVL_RECOVERY '+message)
            response.success, response.message = True, message
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
            retained_north = (self.est.north_alignment if self.est.heading_validated
                and not self.est.heading_continuity_lost and self.est.fresh(self.est.imu_at,now) else None)
            retained_offset = self.est.north_offset
            retained_bias = self.est.heading_bias
            self.output_permitted = False
            self.est = BridgeEstimator(auto_north_alignment=True)
            self.est.north_alignment = TrackNorth()
            if self.settings.get('fixed_local_navigation'):
                self.est=RelativeEstimator()
                retained_north=None
            if retained_north is not None:
                self.est.north_alignment = retained_north
                self.est.north_offset = retained_offset
                self.est.heading_bias = retained_bias
                self.est.heading_validated = True
            reference=self.land_north.current(now) if hasattr(self,'land_north') else None
            if reference and not isinstance(self.est,RelativeEstimator):
                self.est.north_offset=reference['offset_deg']
                self.est.heading_validated=True
                self.est.north_alignment.offset=reference['offset_deg']
                self.est.north_alignment.method=reference['method']
                self.est.north_alignment.reference=dict(reference)
                self.est.north_alignment.reason='Land north retained with continuous VN; awaiting surface navigation'
            self.auto_recovery=AutomaticRecovery()
            self.est.gps.clock = clock
            if origin is not None:
                self.est.set_origin(origin)
            self.settings['surface_pressure_hpa'] = base
            self.reset_counter = (self.reset_counter+1)%256
            self.last_sent = None
            self.start_checked = True
            self.output_permitted = True  # Health/north/profile gates still withhold output.
            response.success, response.message = True, f'Surface baseline {base:.3f} hPa; '+('land north retained' if reference else 'waiting for GPS/DVL north alignment')
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
            if hasattr(self,'land_north'):self.land_north.invalidate()
            return
        if self.advancing('imu', stamp):
            q, r = m.orientation, m.angular_velocity
            self.est.imu(time.monotonic(), (q.w, q.x, q.y, q.z), (r.x, r.y, r.z))
            if hasattr(self,'land_north'):
                if self.est.imu_at is None:
                    self.land_north.invalidate()
                else:
                    previous=bool(self.land_north.rows)
                    self.land_north.observe(time.monotonic(),self.est.raw_q,self.est.rates)

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
        self.sync_clock()
        pressure_id={'SCALED_PRESSURE':29,'SCALED_PRESSURE2':137,'SCALED_PRESSURE3':143}[self.settings.get('pressure_message','SCALED_PRESSURE3')]
        for mid in (24,pressure_id,49):
            self.link.command(511, mid, 100000)

    def request_parameters(self):
        names=list(PROFILE)
        names.append('COMPASS_LEARN')  # Reject an interrupted legacy calibration; never start one.
        # Spread reads across the bounded Cube reply queue. Four every 200 ms
        # refresh the full profile in 1.4 s without competing in large bursts
        # with stream requests. Keep the three-second verification age gate.
        if getattr(self,'parameter_schema',None)!=tuple(names):
            self.parameter_schema=tuple(names);self.parameter_cursor=0
        first=self.parameter_cursor;last=min(first+4,len(names))
        for name in names[first:last]:
            self.link.send('param_request_read_send',1,1,name.encode(),-1)
        self.parameter_cursor=0 if last==len(names) else last

    def sync_clock(self):
        now=time.monotonic_ns()
        pending=self.est.gps.clock.pending
        if pending is None or now-pending > 50000000:
            self.link.send('timesync_send',0,self.est.gps.clock.request(now))

    def configured(self, now):
        return all(k in self.params and self.params[k][1] == v and 0 <= now-self.params[k][0] < 3.
                   for k, v in dict(PROFILE, COMPASS_LEARN=0).items())

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
        elif kind == self.settings.get('pressure_message','SCALED_PRESSURE3'):
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
            self.pressure_sample = dict(source=kind,hpa=m.press_abs if math.isfinite(m.press_abs) else None,
                                        received=now, cube_boot_ms=m.time_boot_ms)
            if math.isfinite(m.press_abs):
                self.pressures.append((now, m.press_abs))
            base, rho = self.settings['surface_pressure_hpa'], self.settings['water_density']
            depth = (m.press_abs-base)*100/(rho*9.80665) if base > 0 and rho > 0 else math.nan
            self.est.pressure_depth(now, depth)
        elif kind == 'GPS_RAW_INT':
            self.raw_gps_health=(now,m.fix_type>=3)
            fix = dict(cube_fix_time_us=m.time_usec, received=now,
                       lat=m.lat/1e7, lon=m.lon/1e7, alt=m.alt/1000,
                       fix=m.fix_type, accuracy=getattr(m, 'h_acc', 0)/1000,
                       speed_mps=m.vel/100 if m.vel < 65535 else None,
                       course_deg=m.cog/100 if m.cog < 36000 else None,
                       speed_accuracy_mps=getattr(m, 'vel_acc', 0)/1000 or None)
            previous = self.est.gps.last_fix
            self.est.observe_gps(now, fix)
            clock=self.est.gps.clock
            self.est.event.update(clock_offset=clock.offset,clock_uncertainty=clock.uncertainty,
                clock_age_s=now-clock.updated if clock.updated is not None else None,
                signed_fix_age_s=now-(m.time_usec/1e6+clock.offset) if clock.offset is not None else None)
            if previous != m.time_usec:
                self.get_logger().info('GPS_DECISION '+json.dumps(self.est.event, allow_nan=False))

    def tick(self):
        self.link.drain(self.consume)
        now = time.monotonic()
        a=self.est.north_alignment
        if (isinstance(a,TrackNorth) and a.active
                and now-getattr(self,'motion_north_received',getattr(self,'motion_north_started',now))
                > (.7 if hasattr(self,'motion_north_received') else 10.)):
            a.stop(False)
        if self.output_permitted and self.configured(now):
            if self.auto_recovery.step(now,self.est,self.control_state,self.control_received):
                self.reset_counter=(self.reset_counter+1)%256
                self.last_sent=None
                self.get_logger().warning('AUTOMATIC_DVL_RECOVERY '+self.auto_recovery.reason)
        status = self.est.status(now)
        configured = self.configured(now)
        if self.start_checked and self.output_permitted and not self.auto_recovery.pending and configured and status['healthy'] and (self.last_sent is None or now-self.last_sent >= .05):
            self.link.odometry(self.est.position, self.est.orientation(), self.est.velocity(),
                               self.est.rates, reset_counter=self.reset_counter, covariance=self.est.covariance(),
                               quality=40 if status.get('dvl_continuity',{}).get('prediction_active') else 80)
            self.last_sent = now
        status.update(instance=self.instance, fusion_policy='bridge_odometry',
                      land_north_reference=self.land_north.current(now) if hasattr(self,'land_north') else None,
                      automatic_recovery=self.auto_recovery.status(now,self.control_state,self.control_received),
                      clock_mapping=dict(offset=self.est.gps.clock.offset,
                          uncertainty=self.est.gps.clock.uncertainty, updated=self.est.gps.clock.updated),
                      pressure_sample=getattr(self, 'pressure_sample', None),
                      profile_mismatches={k: dict(expected=v, actual=self.params.get(k, (None, None))[1])
                          for k, v in dict(PROFILE,COMPASS_LEARN=0).items() if k not in self.params or self.params[k][1] != v},
                      configuration_verified=configured, enabled=self.output_permitted,
                      surface_pressure_hpa=self.settings['surface_pressure_hpa'],
                      startup_output_enabled=self.settings['enabled'],
                      transmitting=self.last_sent is not None and 0 <= now-self.last_sent < .15,
                      published_monotonic=now, reset_counter=self.reset_counter,
                      inputs=dict(vn=self.est.fresh(self.est.imu_at, now),
                                  dvl=self.est.motion_valid and self.est.fresh(self.est.motion_at, now)),
                      heading_vn=dict(received=self.est.imu_at, yaw=yaw(self.est.orientation()) % 360,
                                      raw_yaw=yaw(self.est.raw_q) % 360,
                                      rate=math.degrees(self.est.rates[2])),
                      heading_dvl=dict(received=self.est.motion_at, velocity=self.est.velocity(),
                                       raw_velocity=self.est.raw_velocity,
                                       valid=self.est.motion_valid))
        if now-getattr(self, 'last_published', -math.inf) >= .05:
            self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
            self.last_published = now


def main():
    run(FusedNavBridge, ownership='nav_ekf_bridge')
