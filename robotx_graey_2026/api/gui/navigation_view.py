"""Read-only GUI navigation cache. Display origin never changes vehicle state."""
import copy
import math
import threading
import time
from collections import deque


def offset(lat, lon, altitude, origin):
    """WGS84 ECEF -> tangent NED, relative to a display-only MSL reference.

    Both heights are MSL; this is a local display approximation, not navigation
    geodesy. Never compare this Down with Cube local Z or water depth.
    """
    def ecef(a, b, h):
        a, b = math.radians(a), math.radians(b)
        n = 6378137 / math.sqrt(1 - 0.00669437999014 * math.sin(a) ** 2)
        return ((n+h)*math.cos(a)*math.cos(b),
                (n+h)*math.cos(a)*math.sin(b),
                (n*(1-0.00669437999014)+h)*math.sin(a))
    base = ecef(*origin)
    x, y, z = [v-b for v, b in zip(ecef(lat, lon, altitude), base)]
    a, b = map(math.radians, origin[:2])
    return [-math.sin(a)*math.cos(b)*x-math.sin(a)*math.sin(b)*y+math.cos(a)*z,
            -math.sin(b)*x+math.cos(b)*y,
            -math.cos(a)*math.cos(b)*x-math.cos(a)*math.sin(b)*y-math.sin(a)*z]


class NavigationView:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.samples = {}
        self.origin = None
        self.trails = {k: deque(maxlen=600) for k in ('global', 'gps')}
        self.boot_ms = None
        self.session = 0
        self.reset_state = 'idle'
        self.reset_deadline = None
        self.reset_gps_time = None
        self.origin_revision = 0

    def reset_display(self):
        """Clear visualization and await a newer GPS fix; no vehicle commands."""
        with self.lock:
            self._expire_reset()
            if self.reset_state == 'waiting':
                return {'ok': True, 'state': 'waiting'}
            previous = self.samples.get('gps', {}).get('data', {})
            self.reset_gps_time = previous.get('receiver_time_us') or None
            self.reset_deadline = self.clock() + 15
            self.reset_state = 'waiting'
            self.origin = None
            self.session += 1
            for trail in self.trails.values():
                trail.clear()
            for name in ('gps', 'global'):
                if name in self.samples:
                    self.samples[name]['data']['ned'] = None
            return {'ok': True, 'state': 'waiting'}

    def _expire_reset(self):
        if self.reset_state == 'waiting' and self.clock() >= self.reset_deadline:
            self.reset_state = 'timeout'

    def put(self, name, data):
        # Never serialize NaN/Infinity into the browser's JSON parser.
        def clean(v):
            if isinstance(v, float) and not math.isfinite(v): return None
            if isinstance(v, dict): return {k: clean(x) for k, x in v.items()}
            if isinstance(v, (list, tuple)): return [clean(x) for x in v]
            return v
        with self.lock:
            self.samples[name] = {'data': clean(data), 'received': self.clock()}

    def consume(self, kind, m):
        if m.get_srcSystem() != 1 or m.get_srcComponent() != 1:
            return
        with self.lock:
            if kind == 'GLOBAL_POSITION_INT':
                if self.boot_ms is not None and m.time_boot_ms + 1000 < self.boot_ms:
                    self.samples.clear()
                    for trail in self.trails.values(): trail.clear()
                    self.origin = None
                    self.session += 1
                    # A new Cube boot invalidates the old GPS timestamp baseline.
                    if self.reset_state in ('waiting', 'timeout'):
                        self.reset_gps_time = None
                    else:
                        self.reset_state = 'idle'
                self.boot_ms = m.time_boot_ms
                self._geo('global', m.lat/1e7, m.lon/1e7, m.alt/1000,
                          {'velocity': [m.vx/100, m.vy/100, m.vz/100],
                           'heading': None if m.hdg == 65535 else m.hdg/100})
            elif kind == 'GPS_RAW_INT':
                previous = self.samples.get('gps', {}).get('data', {})
                if (m.time_usec and previous.get('receiver_time_us') == m.time_usec
                        and previous.get('fix') == m.fix_type):
                    return  # Repeated cached telemetry is not a fresh GPS measurement.
                self._geo('gps', m.lat/1e7, m.lon/1e7, m.alt/1000,
                          {'fix': m.fix_type, 'satellites': None if m.satellites_visible == 255 else m.satellites_visible,
                           'hdop': None if m.eph == 65535 else m.eph/100,
                           'accuracy': getattr(m, 'h_acc', 0)/1000 or None,
                           'vertical_accuracy': getattr(m, 'v_acc', 0)/1000 or None,
                           'receiver_time_us': m.time_usec})
            elif kind == 'LOCAL_POSITION_NED':
                self.put('local', {'ned': [m.x, m.y, m.z], 'velocity': [m.vx, m.vy, m.vz]})
            elif kind == 'ATTITUDE':
                self.put('attitude', {'rpy': [math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw)%360]})
            elif kind == 'HEARTBEAT':
                self.put('heartbeat', {'armed': bool(m.base_mode & 128), 'mode_id': m.custom_mode})
            elif kind == 'EKF_STATUS_REPORT':
                self.put('ekf', {'flags': m.flags, 'velocity_variance': m.velocity_variance,
                                 'horizontal_variance': m.pos_horiz_variance,
                                 'vertical_variance': m.pos_vert_variance})
            elif kind == 'GPS_GLOBAL_ORIGIN':
                self.put('cube_origin', {'lat': m.latitude/1e7, 'lon': m.longitude/1e7, 'alt': m.altitude/1000})

    def _geo(self, name, lat, lon, alt, extra):
        self._expire_reset()
        # (0,0) is a real location but also ArduPilot's uninitialized sentinel.
        valid = (-90 <= lat <= 90 and -180 <= lon <= 180
                 and (lat != 0 or lon != 0) and math.isfinite(alt)
                 and (name != 'gps' or extra['fix'] >= 3))
        if self.reset_state == 'waiting' and name == 'gps':
            gps_time = extra.get('receiver_time_us', 0)
            if gps_time and self.reset_gps_time is None:
                # If no measurement existed at click time, the first timestamp
                # is only a baseline. It might be a cached MAVLink response.
                self.reset_gps_time = gps_time
            elif valid and gps_time and gps_time > self.reset_gps_time:
                self.origin = [lat, lon, alt]
                self.reset_state = 'complete'
                self.origin_revision += 1
                for cached in ('gps', 'global'):
                    sample = self.samples.get(cached, {}).get('data')
                    if sample and sample['valid']:
                        sample['ned'] = offset(sample['lat'], sample['lon'], sample['alt'], self.origin)
        if valid and self.origin is None and self.reset_state == 'idle':
            self.origin = [lat, lon, alt]
        ned = offset(lat, lon, alt, self.origin) if valid and self.origin else None
        self.put(name, dict(lat=lat, lon=lon, alt=alt, valid=valid, ned=ned, **extra))
        if ned is not None:
            now = self.clock()
            trail = self.trails[name]
            if not trail or now-trail[-1][3] >= .5:
                trail.append([*ned, now, lat, lon])

    def snapshot(self):
        with self.lock:
            self._expire_reset()
            now = self.clock()
            streams = {}
            for name, sample in self.samples.items():
                age = max(0, now-sample['received'])
                streams[name] = {'data': copy.deepcopy(sample['data']), 'age': age,
                                 'fresh': age < (3 if name in ('heartbeat', 'ekf') else 2)}
            supervisor = streams.get('supervisor', {})
            reference = copy.deepcopy(supervisor.get('data', {}).get('reference'))
            if reference:
                reference['display_ned'] = offset(reference['gps_lat'], reference['gps_lon'],
                    reference['gps_msl_alt'], self.origin) if self.origin else None
                reference['live'] = bool(supervisor.get('fresh') and
                    supervisor.get('data', {}).get('reference_current'))
            return {'streams': streams, 'origin': copy.deepcopy(self.origin), 'session': self.session,
                    'mission_reference': reference,
                    'origin_revision': self.origin_revision,
                    'display_reset': {'state': self.reset_state,
                        'remaining_s': max(0, self.reset_deadline-now) if self.reset_state == 'waiting' else 0},
                    'trails': {k: [[*p[:3], max(0, now-p[3]), *p[4:]] for p in v] for k, v in self.trails.items()}}
