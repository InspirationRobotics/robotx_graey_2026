"""Session-only north alignment from independently measured velocity vectors.

Call only with admitted GPS and time-matched VN/DVL history. No stationary
course, compass assumption, persistent yaw offset or vehicle command is used.
"""
import math


def wrap(degrees):
    return (degrees + 180.) % 360. - 180.


class NorthAlignment:
    def __init__(self):
        self.samples = []
        self.last_fix = None
        self.offset = None
        self.method = 'GPS/DVL velocity vectors'
        self.reference = None
        self.reason = 'Make a steady surface movement with qualified GPS and DVL'

    def reject(self, reason, reset=False):
        # A rejected measurement is not a request to erase all accepted windows.
        # Their finite lifetime is enforced when the next valid sample arrives.
        if reset:
            self.samples.clear()
        self.reason = reason
        return False

    def observe(self, now, fix, world_velocity, rates, q):
        if self.offset is not None:
            return True  # Frozen until this bridge instance ends.
        stamp = fix['cube_fix_time_us']
        if stamp == self.last_fix:
            return False
        if self.last_fix is not None and stamp < self.last_fix:
            self.last_fix = stamp
            return self.reject('GPS time regressed during north alignment', reset=True)
        self.last_fix = stamp
        speed, course, sigma = (fix.get(k) for k in
                               ('speed_mps', 'course_deg', 'speed_accuracy_mps'))
        values = (*world_velocity, *rates, *q, speed, course, sigma, now)
        if any(v is None or not math.isfinite(v) for v in values):
            return self.reject('Velocity or uncertainty unavailable for north alignment')
        horizontal = math.hypot(*world_velocity[:2])
        # Do not prompt the pilot to accelerate when velocity precision itself
        # is inadequate regardless of speed.
        if not 0 < sigma <= .2:
            return self.reject('GPS velocity uncertainty too large for north alignment')
        if not 0 <= course < 360 or min(speed, horizontal) < .5:
            return self.reject('North alignment needs at least 0.5 m/s surface motion')
        if math.degrees(math.atan2(sigma, speed)) > 10:
            return self.reject('GPS velocity uncertainty too large for north alignment')
        # Relative yaw does not affect the vertical projection of body Z.
        w, x, y, z = q
        if 1 - 2*(x*x+y*y) < math.cos(math.radians(15)) or abs(world_velocity[2]) > .15:
            return self.reject('Keep the sub level during north alignment')
        if max(abs(math.degrees(v)) for v in rates) > 5:
            return self.reject('Keep rotation below 5 deg/s during north alignment')
        if abs(speed-horizontal) > max(.15, 3*sigma):
            return self.reject('GPS and DVL speeds disagree during north alignment')
        if self.samples and not 0 < now-self.samples[-1][0] <= 4.:
            self.samples.clear()
        angle = wrap(course-math.degrees(math.atan2(world_velocity[1], world_velocity[0])))
        self.samples.append((now, angle, horizontal))
        while self.samples and now-self.samples[0][0] > 12.:
            self.samples.pop(0)
        mean = math.degrees(math.atan2(sum(math.sin(math.radians(s[1])) for s in self.samples),
                                      sum(math.cos(math.radians(s[1])) for s in self.samples)))
        if any(abs(wrap(s[1]-mean)) > 5 for s in self.samples):
            return self.reject('North alignment inconsistent; repeat steady surface movement', reset=True)
        # Invalid/missing intervals never count toward duration or distance.
        pairs=[(a,b) for a,b in zip(self.samples,self.samples[1:]) if 0 < b[0]-a[0] <= .5]
        span = sum(b[0]-a[0] for a,b in pairs)
        distance = sum((b[0]-a[0])*(a[2]+b[2])/2 for a,b in pairs)
        if span < 5 or len(self.samples) < 15 or distance < 2.5:
            self.reason = f'Collecting north alignment: {span:.1f}/5 s; {distance:.1f}/2.5 m'
            return False
        self.offset = wrap(mean)
        self.reason = 'North aligned from GPS/DVL velocity; session reference, not surveyed accuracy'
        return True

    def status(self):
        return dict(aligned=self.offset is not None, offset_deg=self.offset,
                    reason=self.reason, samples=len(self.samples),
                    method=self.method, reference=self.reference, persistence='bridge session only')
