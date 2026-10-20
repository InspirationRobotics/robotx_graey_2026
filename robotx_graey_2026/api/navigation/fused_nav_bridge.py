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
from .cube_north import CubeNorth, COMPASS_PARAMS


class FusedNavBridge(SteadyNode):
    def __init__(self):
        super().__init__('nav_ekf_bridge')
        defaults = dict(enabled=False, mavlink='udpout:127.0.0.1:14551',
                        surface_pressure_hpa=0., water_density=1000.,
                        pressure_message='SCALED_PRESSURE3',
                        sitl_depth_topic='',
                        auto_north_alignment=False,
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
        self.land_north = LandNorth()
        self.cube_north = CubeNorth()
        self.compass_health=None
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
        for name in ('compass_start','compass_capture','compass_cancel'):
            self.create_service(Trigger,'/graey/navigation/'+name,lambda request,response,name=name:self.compass_action(name,response))
        if self.settings['sitl_depth_topic']:
            self.create_subscription(Float32, self.settings['sitl_depth_topic'],
                self.sitl_depth, 20)
        # Service TIMESYNC faster than its 50 ms RTT acceptance bound. Keep
        # ODOMETRY/status at 20 Hz separately below.
        self.create_timer(.01, self.tick)
        self.create_timer(1., self.request)
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
                if hasattr(self,'cube_north'):self.cube_north.invalidate_vn()
        except (ValueError, TypeError):
            self.vn_calibration = None

    def control_status(self, msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.control_state,self.control_received=data,time.monotonic()
        except (ValueError,TypeError):
            pass

    def compass_action(self,name,response):
        try:
            require_maintenance()
            if self.output_permitted:
                raise ValueError('Pause navigation before compass reference maintenance')
            now=time.monotonic()
            if name=='compass_start':
                if (not self.vn_calibration or not 0<=now-self.vn_calibration[0]<1.5
                        or self.vn_calibration[1].get('phase') not in ('idle','collected')):
                    raise ValueError('Fresh completed VN calibration status required')
                health=getattr(self,'compass_health',None);gps=getattr(self,'raw_gps_health',None)
                if not (health and 0<=now-health[0]<.5 and health[1] and gps and 0<=now-gps[0]<.5 and gps[1]):
                    raise ValueError('Fresh healthy enabled Cube compass and GPS fix required before learning')
                changes=self.cube_north.start(now,self.params)
                self.land_north.invalidate()
            elif name=='compass_capture':
                if not self.est.fresh(self.est.imu_at,now):
                    self.cube_north.invalidate_vn()
                    raise ValueError('VN continuity required to retain captured north')
                reference,changes=self.cube_north.capture(now,self.params)
                self.land_north.reference=dict(reference)
            else:
                changes=self.cube_north.cancel(now,self.params)
                self.land_north.invalidate()
            self.get_logger().warning('COMPASS_REFERENCE_ACTION '+json.dumps(dict(action=name,changes=changes)))
            for key,value in changes.items():
                self.link.send('param_set_send',1,1,key.encode(),float(value),9)
            response.success,response.message=True,self.cube_north.reason
        except (ValueError,OSError) as error:
            response.success,response.message=False,str(error)
        return response

    def finish_compass_reference(self,now):
        """Capture is passive while armed; source writes wait for maintenance."""
        if (not hasattr(self,'cube_north') or self.output_permitted
                or self.cube_north.phase not in ('captured','restoring')
                or now-getattr(self,'compass_restore_attempt',-math.inf)<1.):return
        c=self.control_state
        if not (self.control_received is not None and 0<=now-self.control_received<.7
                and c.get('vehicle_fresh') is True and c.get('armed') is False
                and c.get('sa_active') is True and c.get('esc_commanded_on') is False
                and not c.get('mission_claimed') and not c.get('autonomous')):return
        self.compass_restore_attempt=now
        try:
            require_maintenance()
            if self.cube_north.phase=='captured':
                if not self.est.fresh(self.est.imu_at,now):
                    self.cube_north.invalidate_vn()
                    self.land_north.invalidate()
                    return
                reference,changes=self.cube_north.capture(now,self.params)
                self.land_north.reference=dict(reference)
            else:
                changes={**{f'EK3_SRC{i}_YAW':6 for i in (1,2,3)},'COMPASS_LEARN':0}
            for key,value in changes.items():
                self.link.send('param_set_send',1,1,key.encode(),float(value),9)
        except (ValueError,OSError):
            # No write, no disarm, no propulsion action. Preserve passive capture.
            return

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
            if hasattr(self,'cube_north') and self.cube_north.phase in ('learning','captured','restoring'):
                raise ValueError('Finish or cancel compass learning and verify ExternalNav restoration before Prepare')
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
            reference=self.land_north.current(now) if hasattr(self,'land_north') else None
            if reference:
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
            if hasattr(self,'cube_north'):self.cube_north.invalidate_vn()
            return
        if self.advancing('imu', stamp):
            q, r = m.orientation, m.angular_velocity
            self.est.imu(time.monotonic(), (q.w, q.x, q.y, q.z), (r.x, r.y, r.z))
            if hasattr(self,'land_north'):
                if self.est.imu_at is None:
                    self.land_north.invalidate()
                    if hasattr(self,'cube_north'):self.cube_north.invalidate_vn()
                else:
                    previous=bool(self.land_north.rows)
                    self.land_north.observe(time.monotonic(),self.est.raw_q,self.est.rates)
                    if previous and len(self.land_north.rows)==1 and hasattr(self,'cube_north'):self.cube_north.invalidate_vn()

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
        for name in PROFILE:
            self.link.send('param_request_read_send', 1, 1, name.encode(), -1)
        if (not self.output_permitted or (hasattr(self,'cube_north') and self.cube_north.phase in ('learning','captured','restoring'))):
            for mid in (1,30):self.link.command(511,mid,100000)
            for name in COMPASS_PARAMS:
                if name not in PROFILE:self.link.send('param_request_read_send',1,1,name.encode(),-1)

    def sync_clock(self):
        now=time.monotonic_ns()
        pending=self.est.gps.clock.pending
        if pending is None or now-pending > 50000000:
            self.link.send('timesync_send',0,self.est.gps.clock.request(now))

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
        elif kind=='SYS_STATUS':
            self.compass_health=(now,bool(m.onboard_control_sensors_present&4 and m.onboard_control_sensors_enabled&4 and m.onboard_control_sensors_health&4))
        elif kind=='ATTITUDE' and hasattr(self,'cube_north'):
            if not self.advancing('compass_attitude',m.time_boot_ms):return
            health=getattr(self,'compass_health',None);gps=getattr(self,'raw_gps_health',None)
            q=self.est.raw_q
            good=bool(health and now-health[0]<.5 and health[1] and gps and now-gps[0]<.5 and gps[1]
                      and self.est.imu_at is not None and 0<=now-self.est.imu_at<.15
                      and max(abs(math.degrees(r)) for r in self.est.rates)<=10
                      and 1-2*(q[1]*q[1]+q[2]*q[2])>=math.cos(math.radians(30))
                      and abs(m.roll)<=math.radians(30) and abs(m.pitch)<=math.radians(30))
            self.cube_north.observe(now,self.params,math.degrees(m.yaw),yaw(q),good)
        elif kind=='STATUSTEXT' and hasattr(self,'cube_north'):
            self.cube_north.message(now,str(m.text))
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
            if previous != m.time_usec:
                self.get_logger().info('GPS_DECISION '+json.dumps(self.est.event, allow_nan=False))

    def tick(self):
        self.link.drain(self.consume)
        now = time.monotonic()
        self.finish_compass_reference(now)
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
                      compass_north=self.cube_north.status() if hasattr(self,'cube_north') else None,
                      compass_parameters={k:dict(value=self.params[k][1],fresh=0<=now-self.params[k][0]<3) for k in COMPASS_PARAMS if k in self.params},
                      automatic_recovery=self.auto_recovery.status(now,self.control_state,self.control_received),
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
