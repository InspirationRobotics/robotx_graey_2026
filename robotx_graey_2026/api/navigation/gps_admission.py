"""GPS admission, not fusion. Cube-fix timing is explicitly an approximation.

TIMESYNC bounds transport/clock uncertainty, not receiver processing delay.
No host wall-clock or invented GPS week is used. Instances must be discarded
after a Cube reboot; backwards fix timestamps latch a fault until restart.
"""
from .gps_quality import GPSQuality


class CubeClock:
    def __init__(self):
        self.pending = None
        self.offset = None
        self.uncertainty = None
        self.updated = None

    def request(self, now_ns):
        self.pending = now_ns
        return now_ns

    def reply(self, echoed_ns, cube_ns, now_ns):
        if echoed_ns != self.pending or self.pending is None:
            return False
        self.pending = None
        rtt = (now_ns-echoed_ns)/1e9
        if not 0 <= rtt <= .05 or cube_ns <= 0:
            return False
        offset = (echoed_ns+now_ns)/2e9-cube_ns/1e9
        if self.offset is not None and abs(offset-self.offset) > .1:
            self.offset = self.updated = self.uncertainty = None
            return False
        self.offset, self.updated, self.uncertainty = offset, now_ns/1e9, rtt/2
        return True

    def age(self, fix_us, now):
        if self.offset is None or not 0 <= now-self.updated <= 3.:
            return None
        # Modest drift allowance; receiver latency is NOT included in this bound.
        uncertainty = self.uncertainty + (now-self.updated)*.0001
        age = now-(fix_us/1e6+self.offset)
        if age < -uncertainty:
            return None
        return max(0., age)+uncertainty


class GPSAdmission:
    def __init__(self, dwell=3., max_age=.35):
        self.clock = CubeClock()
        self.quality = GPSQuality()
        self.dwell, self.max_age = dwell, max_age
        self.last_fix = None
        self.good_since = None
        self.last_accepted = None
        self.last_good = None
        self.fault = ''
        self.reason = 'Waiting for qualified GPS'
        self.age_bound = None

    def invalidate(self, reason):
        self.good_since = self.last_good = self.last_accepted = None
        self.reason = reason

    def check(self, now, fix, imu_fresh, eligible=True, eligibility_reason='GPS ineligible'):
        stamp = fix['cube_fix_time_us']
        self.age_bound = self.clock.age(stamp, now)
        reason = ''
        if self.last_fix is not None and stamp < self.last_fix:
            self.fault = 'Cube GPS clock regressed; restart and revalidate'
        if self.fault:
            reason = self.fault
        elif not eligible:
            reason = eligibility_reason
        elif fix['fix'] < 3:
            reason = 'GPS has no 3D fix'
        elif self.last_fix is not None and stamp == self.last_fix:
            # Repeated telemetry is normal. Never forward it or extend validity.
            return False
        elif self.age_bound is None or self.age_bound > self.max_age:
            reason = 'Cube fix timing unavailable or stale'
        elif not imu_fresh:
            reason = 'Fresh valid VN orientation unavailable'
        else:
            good, reason = self.quality.check(now, fix)
            if good:
                reason = ''
        self.last_fix = stamp
        if reason:
            self.good_since = self.last_good = None
            self.last_accepted = None
            self.reason = reason
            return False
        if self.last_good is None or now-self.last_good > self.max_age:
            self.good_since = now
        self.last_good = now
        if now-self.good_since < self.dwell:
            self.reason = 'GPS qualifying'
            return False
        self.last_accepted = now
        self.reason = 'GPS admitted; Cube-fix timing approximation'
        return True

    def status(self, now):
        accepted = (not self.fault and self.last_good is not None
                    and self.last_accepted is not None
                    and 0 <= now-self.last_accepted <= self.max_age)
        return dict(qualified=bool(accepted), reason=self.fault or self.reason,
                    cube_fix_age_upper_s=self.age_bound,
                    timing_basis='Cube last-fix time; not receiver epoch',
                    receiver_delay_bound_s=None, fault=self.fault)
