"""Pre-dive consistency check. GPS course is motion, never a yaw correction."""
import math


def angle_error(a, b):
    return (a-b+180.) % 360.-180.


class DiveHeadingCheck:
    def __init__(self, reference_validated=False, vn_to_true=0., cube_to_true=0.,
                 min_speed=.5, tolerance=15., dwell=3.):
        if not all(math.isfinite(v) for v in (vn_to_true, cube_to_true, min_speed, tolerance, dwell)):
            raise ValueError('Heading settings must be finite')
        if min_speed <= 0 or not 0 < tolerance < 90 or dwell <= 0:
            raise ValueError('Invalid heading thresholds')
        self.reference_validated = reference_validated
        self.vn_to_true, self.cube_to_true = vn_to_true, cube_to_true
        self.min_speed, self.tolerance, self.dwell = min_speed, tolerance, dwell
        self.blocked = False
        self.since = self.last_fix = self.last_update = self.agreement = None
        self.count = 0

    def reset_window(self):
        self.since = self.last_fix = self.agreement = None
        self.count = 0

    def evaluate(self, now, *, surface, gps_good, gps, vn, cube, dvl):
        result = dict(state='Waiting', allowed=False, reason='', course_deg=None,
                      vn_error_deg=None, cube_error_deg=None, correction_applied=False)

        def finish(state, reason, allowed=False, reset=True):
            if reset:
                self.reset_window()
            result.update(state=state, reason=reason, allowed=allowed)
            return result

        def skip(reason):
            if self.blocked:
                return finish('Blocked', 'Previous GPS/heading disagreement; requires sustained agreement')
            return finish('Skipped', reason+'; using validated attitude', True)

        if self.last_update is not None and not 0 <= now-self.last_update <= .5:
            self.reset_window()
        self.last_update = now
        if not surface:
            return finish('Not applicable', 'Pre-dive check requires confirmed surface state')
        if not self.reference_validated:
            return finish('Blocked', 'Heading north references and forward axes not validated')
        for label, sample in [('VN attitude', vn), ('Cube attitude', cube), ('DVL motion', dvl)]:
            if not sample or not 0 <= now-sample.get('received', -math.inf) < .5:
                return finish('Waiting', label+' unavailable or stale')
        if not dvl.get('valid'):
            return finish('Waiting', 'DVL motion invalid')
        try:
            v = dvl['velocity']
            if len(v) != 3 or not all(math.isfinite(x) for x in (*v, vn['yaw'], cube['yaw'], vn['rate'], cube['rate'])):
                return finish('Waiting', 'Nonfinite heading or motion')
            vyaw = (vn['yaw']+self.vn_to_true) % 360
            cyaw = (cube['yaw']+self.cube_to_true) % 360
        except (KeyError, TypeError):
            return finish('Waiting', 'Heading or motion fields unavailable')
        if abs(angle_error(vyaw, cyaw)) > self.tolerance:
            return finish('Blocked', 'VN and Cube heading disagree')
        if not gps_good or not gps or not 0 <= now-gps.get('received', -math.inf) < .5:
            return skip('GPS course not qualified')
        course, speed, uncertainty = (gps.get(k) for k in ('course_deg', 'speed_mps', 'speed_accuracy_mps'))
        if any(x is None or not math.isfinite(x) for x in (course, speed, uncertainty)):
            return skip('GPS course/speed uncertainty unavailable')
        if not 0 <= course < 360 or speed < self.min_speed or uncertainty <= 0:
            return skip('GPS course unreliable at this speed')
        # Conservative speed-uncertainty screen, not a measured yaw accuracy.
        if math.degrees(math.atan2(uncertainty, speed)) > 10:
            return skip('GPS velocity uncertainty too large')
        if v[0] < self.min_speed or abs(v[1]) > v[0]*math.tan(math.radians(10)) or abs(v[2]) > .15:
            return skip('Motion is not straight forward and level')
        if max(abs(vn['rate']), abs(cube['rate'])) > 5:
            return skip('Vehicle is turning')
        if abs(math.hypot(v[0], v[1])-speed) > max(.2, 3*uncertainty):
            return skip('GPS and DVL speeds disagree')
        if max(x['received'] for x in (gps, vn, cube, dvl))-min(x['received'] for x in (gps, vn, cube, dvl)) > .25:
            return skip('Heading and motion samples are not time-aligned')
        fix = gps.get('receiver_time_us')
        if not isinstance(fix, (int, float)) or not math.isfinite(fix) or fix <= 0:
            return skip('GPS measurement timestamp unavailable')
        result.update(course_deg=course, vn_error_deg=angle_error(vyaw, course),
                      cube_error_deg=angle_error(cyaw, course))
        agree = max(abs(result['vn_error_deg']), abs(result['cube_error_deg'])) <= self.tolerance
        if self.last_fix is not None and fix < self.last_fix:
            return finish('Waiting', 'GPS timestamp moved backwards')
        if fix != self.last_fix:
            if self.agreement != agree or self.since is None:
                self.since, self.count, self.agreement = now, 0, agree
            self.last_fix = fix
            self.count += 1
        if self.since is None or now-self.since < self.dwell or self.count < 5:
            return finish('Waiting', 'Checking sustained straight-forward GPS course', reset=False)
        self.blocked = not agree
        return finish('Passed' if agree else 'Blocked',
                      'GPS course agrees with VN/Cube heading' if agree else
                      'GPS course disagrees with VN/Cube heading; hold dive', agree, reset=False)
