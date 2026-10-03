"""ROS/MAVLink adapter for navigation supervision. Observation is the default."""
import json
import math
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link
from robotx_graey_2026.api.navigation.mission_reference import MissionReference
from robotx_graey_2026.api.navigation.supervisor_logic import Observation, Supervisor

PROFILE = {'EK3_SRC1_POSXY': 6, 'EK3_SRC1_VELXY': 6, 'EK3_SRC1_POSZ': 1,
           'EK3_SRC1_VELZ': 0, 'EK3_SRC1_YAW': 6, 'EK3_SRC2_POSXY': 3,
           'EK3_SRC2_VELXY': 6, 'EK3_SRC2_POSZ': 1, 'EK3_SRC2_VELZ': 0,
           'EK3_SRC2_YAW': 6, 'EK3_SRC_OPTIONS': 0}


class NavigationSupervisor(Node):
    def __init__(self):
        super().__init__('navigation_supervisor')
        defaults = dict(mavlink='udpout:127.0.0.1:14557', active=False,
                        vehicle_validation_complete=False, surface_pressure_hpa=0.,
                        pressure_message='SCALED_PRESSURE2', water_density=1025.,
                        reference_file='~/robotx_ws/logs/navigation_reference.json',
                        surface_enter_m=.15, surface_exit_m=.4, depth_dwell_s=2.,
                        gps_dwell_s=3., dropout_grace_s=3.)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.settings = {k: self.get_parameter(k).value for k in defaults}
        self.logic = Supervisor(active=self.settings['active'],
            surface_enter=self.settings['surface_enter_m'], surface_exit=self.settings['surface_exit_m'],
            depth_dwell=self.settings['depth_dwell_s'], gps_dwell=self.settings['gps_dwell_s'],
            dropout_grace=self.settings['dropout_grace_s'])
        self.link = Link(self.settings['mavlink'], 196, self.get_logger())
        self.samples, self.stamps, self.params = {}, {}, {}
        self.epoch = str(uuid.uuid4())
        self.boot_ms = None
        self.cube_origin = None
        self.intent_seq = -1
        self.reference = None
        self.bridge_instance = None
        try:
            self.reference = MissionReference.load_history(Path(self.settings['reference_file']).expanduser())
        except (OSError, ValueError, TypeError, KeyError):
            pass
        self.alignment_id = ''
        self.last_gps = None
        self.last_gps_time = None
        self.baseline = None
        self.pub = self.create_publisher(String, '/graey/navigation/status', 10)
        self.align_pub = self.create_publisher(String, '/graey/navigation/bridge_request', 10)
        self.create_subscription(String, '/graey/navigation/intent', self.intent, 10)
        self.create_subscription(String, '/graey/navigation/bridge_status', self.bridge_status, 10)
        self.create_subscription(Bool, '/graey/dvl/valid', lambda m: self.put('dvl_valid', m.data), 10)
        self.create_subscription(TwistWithCovarianceStamped, '/graey/dvl/velocity', self.dvl, 20)
        self.create_subscription(Imu, '/graey/vn100/imu', self.vn, 20)
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.link.heartbeat)
        self.create_timer(5., self.request_telemetry)
        self.request_telemetry()

    def put(self, key, value):
        self.samples[key] = (time.monotonic(), value)

    def fresh(self, key, age=1.):
        item = self.samples.get(key)
        return item[1] if item and 0 <= time.monotonic()-item[0] < age else None

    def advancing(self, key, stamp):
        if stamp <= 0 or stamp <= self.stamps.get(key, -1):
            return False
        self.stamps[key] = stamp
        return True

    def dvl(self, msg):
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        if self.advancing('dvl', stamp):
            v = msg.twist.twist.linear
            self.put('dvl', all(math.isfinite(x) for x in (v.x, v.y, v.z)))

    def vn(self, msg):
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        q = msg.orientation
        norm = sum(x*x for x in (q.w, q.x, q.y, q.z))
        if self.advancing('vn', stamp):
            self.put('vn', math.isfinite(norm) and .9 < norm < 1.1)

    def intent(self, msg):
        try:
            d = json.loads(msg.data)
            if d['phase'] not in ('SURFACE', 'DIVE', 'UNDERWATER') or not d['run_id']:
                return
            previous = self.fresh('intent', 3600)
            if previous and previous['run_id'] == d['run_id'] and d['sequence'] <= self.intent_seq:
                return
            self.intent_seq = int(d['sequence'])
            self.put('intent', d)
        except (ValueError, TypeError, KeyError):
            return

    def bridge_status(self, msg):
        try:
            d = json.loads(msg.data)
            if self.bridge_instance is not None and d.get('instance') != self.bridge_instance:
                self.logic.fault = 'Navigation bridge restarted; external frame must be revalidated'
            self.bridge_instance = d.get('instance')
            self.put('bridge', d)
            if d.get('aligned'):
                self.alignment_id = d.get('request_id', '')
        except (ValueError, TypeError):
            return

    def request_telemetry(self):
        for mid in (24, 32, 49, 193, 137, 251):
            self.link.command(511, mid, 200000)
        for name in PROFILE:
            self.link.send('param_request_read_send', 1, 1, name.encode(), -1)

    def consume(self, kind, m):
        if m.get_srcSystem() != 1 or m.get_srcComponent() != 1:
            return
        now = time.monotonic()
        if kind == 'HEARTBEAT':
            self.put('heartbeat', True)
        elif kind == 'GPS_GLOBAL_ORIGIN':
            origin = (m.latitude, m.longitude, m.altitude)
            if self.cube_origin is not None and origin != self.cube_origin:
                self.epoch = str(uuid.uuid4())
                self.logic.fault = 'Cube origin changed; mission reference invalidated'
            self.cube_origin = origin
        elif kind == 'LOCAL_POSITION_NED':
            if self.boot_ms is not None and m.time_boot_ms + 1000 < self.boot_ms:
                self.samples.clear(); self.stamps.clear(); self.params.clear()
                self.epoch = str(uuid.uuid4())
                self.cube_origin = None
                self.logic.fault = 'Cube restarted; validate reference and restart supervisor'
            self.boot_ms = m.time_boot_ms
            if self.advancing('local', m.time_boot_ms):
                self.put('pose', dict(ned=[m.x, m.y, m.z], received=now)
                         if all(math.isfinite(v) for v in (m.x, m.y, m.z)) else None)
        elif kind == 'EKF_STATUS_REPORT':
            required = 1 | 2 | 32
            healthy = (m.flags & required == required and bool(m.flags & (8 | 16))
                       and not m.flags & (128 | 16384 | 32768))
            healthy = healthy and all(math.isfinite(v) and 0 <= v < 1 for v in
                                      (m.velocity_variance, m.pos_horiz_variance, m.pos_vert_variance))
            self.put('ekf', bool(healthy))
        elif kind == 'GPS_RAW_INT':
            if m.time_usec < self.stamps.get('gps', -1):
                self.stamps['gps'] = m.time_usec
                self.put('gps_good', False)
                self.last_gps = None
                return
            if m.fix_type < 3:
                self.put('gps_good', False)
            if not self.advancing('gps', m.time_usec):
                return
            d = dict(lat=m.lat/1e7, lon=m.lon/1e7, alt=m.alt/1000,
                     accuracy=getattr(m, 'h_acc', 0)/1000, fix=m.fix_type,
                     receiver_time_us=m.time_usec, received=now)
            good = (d['fix'] >= 3 and 0 < d['accuracy'] <= 3 and
                    -90 <= d['lat'] <= 90 and -180 <= d['lon'] <= 180)
            if self.last_gps is not None:
                dt = now-self.last_gps_time
                dn = (d['lat']-self.last_gps['lat'])*111320
                de = ((d['lon']-self.last_gps['lon']+180)%360-180)*111320*math.cos(math.radians(d['lat']))
                if dt < 2 and math.hypot(dn, de) > 3*dt + d['accuracy'] + self.last_gps['accuracy']:
                    good = False
            self.last_gps, self.last_gps_time = d, now
            self.put('gps', d); self.put('gps_good', good)
        elif kind == self.settings['pressure_message']:
            if self.advancing('pressure', m.time_boot_ms):
                base = self.settings['surface_pressure_hpa']
                rho = self.settings['water_density']
                depth = (m.press_abs-base)*100/(rho*9.80665) if base > 0 and rho > 0 else float('nan')
                self.put('depth', depth)
        elif kind == 'NAMED_VALUE_FLOAT' and m.name.rstrip('\x00') == 'NAV_SRC':
            if self.advancing('source', m.time_boot_ms) and m.value in (1., 2.):
                self.put('source', int(m.value))
        elif kind == 'COMMAND_ACK' and m.command == 42007:
            if getattr(m, 'target_component', 196) not in (0, 196):
                return
            if m.result != 5:  # IN_PROGRESS is neither success nor failure.
                self.put('ack', 'accepted' if m.result == 0 else 'rejected')
        elif kind == 'PARAM_VALUE':
            self.params[m.param_id.rstrip('\x00')] = m.param_value

    def tick(self):
        self.link.drain(self.consume)
        now = time.monotonic()
        intent = self.fresh('intent') or {}
        depth = self.fresh('depth')
        pose = self.fresh('pose', .5)
        checks = [('Cube link', self.fresh('heartbeat', 3)), ('EKF estimate', self.fresh('ekf', 2)),
                  ('Cube position', pose), ('DVL velocity', self.fresh('dvl', .5)),
                  ('DVL validity', self.fresh('dvl_valid', .5)), ('VN-100', self.fresh('vn', .5))]
        checks.append(('Navigation bridge', (self.fresh('bridge', .5) or {}).get('healthy')))
        failed = [name for name, ok in checks if not ok]
        ref_ready = bool(self.reference and self.reference.vehicle_epoch == self.epoch
                         and self.reference.run_id == intent.get('run_id'))
        o = Observation(healthy=not failed, reason='Unavailable: '+', '.join(failed),
            depth=depth if depth is not None else float('nan'),
            depth_valid=depth is not None and math.isfinite(depth),
            intent=intent.get('phase', ''), intent_fresh=bool(intent), run_id=intent.get('run_id', ''),
            gps_good=self.fresh('gps_good') is True,
            gps_updated=self.samples.get('gps', (-math.inf,))[0],
            source=self.fresh('source') or 0, source_updated=self.samples.get('source', (-math.inf,))[0],
            ack=self.fresh('ack', 5) or '', ack_updated=self.samples.get('ack', (-math.inf,))[0],
            alignment_id=self.alignment_id, reference_ready=ref_ready, reference_saved=ref_ready,
            configuration_verified=bool(self.settings['vehicle_validation_complete']
                and all(self.params.get(k) == v for k, v in PROFILE.items())))
        if self.baseline and pose and self.logic.pending and self.logic.pending[0] == 1:
            elapsed = now-self.baseline[0]
            o.continuity_ok = math.dist(pose['ned'][:2], self.baseline[1][:2]) <= .5+2*elapsed
        for action, value in self.logic.step(now, o):
            if action == 'capture_reference':
                try:
                    candidate = MissionReference.capture(run_id=o.run_id, vehicle_epoch=self.epoch,
                        gps=self.fresh('gps'), pose=pose, now=now,
                        captured_utc=datetime.now(timezone.utc).isoformat(),
                        surface_confirmed=self.logic.surface, gps_qualified=True, ekf_healthy=o.healthy)
                    candidate.save(Path(self.settings['reference_file']).expanduser())
                    self.reference = candidate
                except (OSError, ValueError, TypeError, KeyError) as error:
                    self.logic.fault = 'Dive reference could not be saved: '+str(error)
                    break
            elif action == 'align_external_position':
                self.baseline = (now, pose['ned'])
                self.align_pub.publish(String(data=json.dumps(dict(request_id=value, cube_ned=pose['ned'],
                    expires_at=now+.5))))
            elif action == 'select_source':
                self.link.command(42007, value)
        status = self.logic.snapshot(o)
        status['reference'] = self.reference.snapshot() if self.reference else None
        status['mission_ned'] = self.reference.from_cube(pose['ned'], vehicle_epoch=self.epoch) if ref_ready and pose else None
        status['reference_current'] = ref_ready
        status['depth_m'] = o.depth if o.depth_valid else None
        status['gps_qualified_input'] = o.gps_good
        status['configuration_verified'] = o.configuration_verified
        self.pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    run(NavigationSupervisor)
