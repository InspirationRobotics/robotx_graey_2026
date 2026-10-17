"""Conservative GPS admission checks; never modifies receiver measurements.

DVL path length bounds displacement without requiring heading/frame alignment.
This detects inconsistent movement, not a constant geographic GPS bias.
"""
import math
from collections import deque


class GPSQuality:
    def __init__(self, max_accuracy=3., motion_margin=2., window=10.):
        if not all(math.isfinite(v) and v > 0 for v in (max_accuracy, motion_margin, window)):
            raise ValueError('GPS quality thresholds must be finite and positive')
        self.max_accuracy = max_accuracy
        self.motion_margin = motion_margin
        self.window = window
        self.motion_time = None
        self.speed = 0.
        self.distance = 0.
        self.anchors = deque()
        self.last_valid_motion_time = None
        self.stationary_since = None
        self.stationary_anchor = None

    def motion(self, now, velocity, valid):
        if not valid or not all(math.isfinite(v) for v in velocity):
            self.motion_time = None
            self.stationary_since = None
            return False
        speed = math.sqrt(sum(v*v for v in velocity))
        continuous = self.motion_time is not None and 0 < now-self.motion_time < .5
        if continuous:
            # Use the larger endpoint speed to tolerate sampled acceleration.
            self.distance += max(speed, self.speed)*(now-self.motion_time)
        else:
            # A dropout must not erase trusted GPS anchors and admit a new
            # geographic jump. Bound unmeasured path conservatively; long gaps
            # still require estimator position revalidation independently.
            if self.last_valid_motion_time is not None:
                gap=max(0.,now-self.last_valid_motion_time)
                self.distance += max(speed,self.speed)*gap+.5*gap*gap
            self.stationary_since = None
        self.motion_time, self.speed = now, speed
        self.last_valid_motion_time=now
        if speed<=.03:
            self.stationary_since=now if self.stationary_since is None else self.stationary_since
        else:
            self.stationary_since=None
            self.stationary_anchor=None
        return continuous

    def check(self, now, fix):
        if not all(math.isfinite(fix[k]) for k in ('lat', 'lon', 'accuracy')):
            return False, 'Nonfinite GPS measurement'
        if fix['fix'] < 3:
            return False, 'GPS has no 3D fix'
        if not 0 < fix['accuracy'] <= self.max_accuracy:
            return False, 'GPS horizontal accuracy unavailable or excessive'
        if not (-90 <= fix['lat'] <= 90 and -180 <= fix['lon'] <= 180):
            return False, 'GPS coordinates out of range'
        if self.motion_time is None or not 0 <= now-self.motion_time < .5:
            return False, 'Fresh valid DVL motion unavailable'
        gps_speed,sigma=fix.get('speed_mps'),fix.get('speed_accuracy_mps')
        if (isinstance(gps_speed,(int,float)) and isinstance(sigma,(int,float))
                and math.isfinite(gps_speed) and math.isfinite(sigma) and 0<sigma<=.2
                and abs(gps_speed-self.speed)>max(.3,3*sigma)):
            return False,'GPS velocity disagrees with DVL motion'
        # Retain the last trusted anchor during prolonged rejection. Otherwise
        # a stationary GPS jump becomes "good" merely by outlasting the window.
        # Invalid/interrupted DVL motion preserves this history with a bounded
        # allowance for unmeasured travel.
        still=self.stationary_since is not None and now-self.stationary_since>=2.
        if still and self.stationary_anchor is None and self.anchors:
            self.stationary_anchor=next((a for a in self.anchors if a[0]>=self.stationary_since),self.anchors[-1])
        while len(self.anchors) > 1 and now-self.anchors[0][0] > self.window:
            self.anchors.popleft()
        checks=list(self.anchors)
        if still and self.stationary_anchor is not None:
            checks.append(self.stationary_anchor)
        for stamp, lat, lon, distance in checks:
            dn = (fix['lat']-lat)*111320
            de = ((fix['lon']-lon+180)%360-180)*111320*math.cos(math.radians(lat))
            bound = self.distance-distance + self.speed*(now-self.motion_time) + self.motion_margin
            if math.hypot(dn, de) > bound:
                # A rejected point must not become the next trusted baseline.
                return False, 'GPS displacement disagrees with DVL motion'
        if not self.anchors or now-self.anchors[-1][0] >= .5:
            self.anchors.append((now, fix['lat'], fix['lon'], self.distance))
        return True, 'GPS quality and DVL motion checks passed'
