"""GPS evidence for a temporary native-GPS compass-learning attempt.

This monitor cannot filter GPS inside Cube. A contaminated attempt is rejected,
and the bridge requests restoration in MANUAL rather than transferring its north.
Speed/uncertainty is guidance, not measured GSF yaw variance or completion.
"""
import math


class CompassGPSQuality:
    def __init__(self):
        self.started = None
        self.last_seen = self.last_advanced = self.stamp = None
        self.anchor = None
        self.bad_since = None
        self.interruptions = 0
        self.fault = ''
        self.good = False
        self.reason = 'Waiting for GPS samples'
        self.latest = {}

    def begin(self, now):
        self.__init__()
        self.started = now

    def bad(self, now, reason, serious=False):
        if self.bad_since is None:
            self.bad_since = now
            self.interruptions += 1
        self.good = False
        self.reason = reason
        if serious or now-self.bad_since >= 1.:
            self.fault = self.fault or reason

    def observe(self, now, fix):
        self.last_seen = now
        self.latest = {k: fix.get(k) for k in ('speed_mps', 'speed_accuracy_mps', 'accuracy')}
        stamp = fix.get('cube_fix_time_us')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            self.bad(now, 'GPS measurement timestamp unavailable')
            return
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp:
                self.bad(now, 'GPS measurement clock moved backwards', serious=True)
            self.check(now)
            return
        self.stamp, self.last_advanced = stamp, now
        lat, lon, speed, uncertainty, accuracy = (fix.get(k) for k in
            ('lat', 'lon', 'speed_mps', 'speed_accuracy_mps', 'accuracy'))
        values = (lat, lon, speed, uncertainty, accuracy)
        if isinstance(speed, (int, float)) and math.isfinite(speed) and speed > 5.:
            self.bad(now, 'GPS speed exceeds 5 m/s; learning evidence contaminated', serious=True)
            return
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or not -90 <= lat <= 90 or not -180 <= lon <= 180
                or fix.get('fix', 0) < 3 or speed < 0
                or not 0 < uncertainty <= .5 or not 0 < accuracy <= 3.):
            self.bad(now, 'GPS fix/position/velocity uncertainty unsuitable for learning')
            return
        if self.anchor is not None:
            t, a_lat, a_lon, a_accuracy = self.anchor
            dt = now-t
            distance = math.hypot((lat-a_lat)*111320.,
                ((lon-a_lon+180.) % 360.-180.)*111320.*math.cos(math.radians(a_lat)))
            if 0 < dt < 1.5 and distance > max(5., 3.*(accuracy+a_accuracy)+3.*dt):
                self.bad(now, 'GPS position jumped beyond the learning motion bound', serious=True)
                return
        self.anchor = (now, lat, lon, accuracy)
        self.bad_since = None
        self.good = True
        self.reason = ('GPS velocity direction weak; use controlled forward acceleration/deceleration and turns'
                       if speed < max(.2, 3.*uncertainty) else
                       'GPS motion available; firmware convergence still required')

    def check(self, now):
        # Duplicate packets do not refresh measurement freshness. A short loss
        # pauses capture; a full second of unsuitable evidence taints the attempt.
        origin = self.last_advanced if self.last_advanced is not None else self.started
        if origin is not None and now-origin > 1.5:
            self.bad(now, 'GPS measurement stopped advancing or became stale')
        if self.bad_since is not None and now-self.bad_since >= 1.:
            self.fault = self.fault or self.reason

    def capture_allowed(self, now):
        self.check(now)
        return (self.good and not self.fault and self.last_advanced is not None
                and 0 <= now-self.last_advanced <= 1.5)

    def status(self, now=None):
        return dict(reason=self.reason, suitable=self.good and not self.fault,
                    fault=self.fault, interruptions=self.interruptions,
                    measurement_age_s=(max(0., now-self.last_advanced)
                        if now is not None and self.last_advanced is not None else None),
                    **self.latest, gsf_yaw_confidence='Unavailable in this telemetry')
