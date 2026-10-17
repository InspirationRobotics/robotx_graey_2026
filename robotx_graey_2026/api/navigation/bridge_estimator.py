"""Continuous external navigation with pressure-gated GPS corrections.

This is a conservative complementary estimator, not an inertial EKF. DVL/VN
propagate motion; accepted GPS bounds drift. Receipt-time history is approximate.
No Cube position is used as a measurement. Origin is immutable frame metadata.
"""
from collections import deque
import math
from .gps_admission import GPSAdmission
from .north_alignment import NorthAlignment


def wrap(deg):
    return (deg+180.) % 360.-180.


def yaw(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def yaw_rotate(q, degrees):
    a, b = math.cos(math.radians(degrees)/2), math.sin(math.radians(degrees)/2)
    w, x, y, z = q
    return (a*w-b*z, a*x-b*y, a*y+b*x, a*z+b*w)


def rotate(q, v, inverse=False):
    w, x, y, z = q
    if inverse:
        x, y, z = -x, -y, -z
    # Unit-quaternion vector rotation, without importing ROS.
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


class BridgeEstimator:
    def __init__(self, heading_validated=False, auto_north_alignment=False):
        self.gps = GPSAdmission()
        self.heading_validated = heading_validated
        self.north_alignment = NorthAlignment() if auto_north_alignment else None
        if auto_north_alignment:
            self.heading_validated = False
        self.north_offset = 0.
        self.origin = None
        self.initialized = False
        self.position = [0., 0., 0.]
        self.raw_q = (1., 0., 0., 0.)
        self.rates = (0., 0., 0.)
        self.raw_velocity = (0., 0., 0.)
        self.velocity_bias = [0., 0., 0.]
        self.heading_bias = 0.
        self.heading_continuity_lost = False
        self.stationary_since = None
        self.stationary_anchor = None
        self.last_stationary_fix_at = None
        self.imu_at = self.motion_at = self.depth_at = None
        self.motion_valid = False
        self.dvl_good_at = None
        self.dvl_gap_start = None
        self.dvl_recovery_count = 0
        self.dvl_dropouts = 0
        self.propagated_at = None
        self.coast_world = (0., 0., 0.)
        self.recovery_since = None
        self.recovery_velocity = None
        self.missing_distance_bound = 0.
        self.gap_bound = 0.
        self.depth = None
        self.surface = None
        self.shallow_since = self.deep_since = None
        self.history = deque(maxlen=100)
        self.horizontal_variance = 9.
        self.distance_travelled = 0.
        self.last_fix = None
        self.last_observed_gps = None
        self.pending_gps = None
        self.last_correction = None
        self.heading_since = None
        self.heading_count = 0
        self.fault = ''
        self.event = dict(decision='waiting', reason='Waiting for sensors')

    def set_origin(self, origin):
        origin = tuple(origin)
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin) or not (-90 < origin[0] < 90 and -180 <= origin[1] <= 180):
            return False
        # Horizontal origin defines our NE frame. Cube may update its geodetic
        # altitude datum; our independent pressure zero does not use that value.
        if self.origin is not None and origin[:2] != self.origin[:2]:
            self.fault = 'Cube geographic origin changed; restart and revalidate frame'
            return False
        self.origin = origin
        return True

    def imu(self, now, q, rates):
        if self.heading_validated and self.imu_at is not None and not self.fresh(self.imu_at, now):
            self.heading_continuity_lost = True
        norm = sum(v*v for v in q)
        if len(q) != 4 or len(rates) != 3 or not all(math.isfinite(v) for v in (*q, *rates)) or not .9 < norm < 1.1:
            self.imu_at = None
            self.heading_continuity_lost = True
            if self.initialized:
                self.fault = self.fault or 'Invalid VN attitude; revalidate navigation'
            return
        self.raw_q = tuple(v/math.sqrt(norm) for v in q)
        self.rates, self.imu_at = tuple(rates), now

    def pressure_depth(self, now, depth):
        continuous = self.depth_at is not None and 0 <= now-self.depth_at < .5
        if not math.isfinite(depth) or not -.25 <= depth <= 100.:
            self.depth_at = None
            self.shallow_since = self.deep_since = None
            self.gps.invalidate('Pressure depth invalid or uncalibrated')
            return
        if not continuous:
            self.shallow_since = self.deep_since = None
        self.depth, self.depth_at = depth, now
        self.shallow_since = (now if self.shallow_since is None else self.shallow_since) if depth <= .15 else None
        self.deep_since = (now if self.deep_since is None else self.deep_since) if depth >= .4 else None
        if self.shallow_since is not None and now-self.shallow_since >= 2.:
            self.surface = True
        if self.deep_since is not None and now-self.deep_since >= 2.:
            self.surface = False
        if depth > .15:
            self.gps.invalidate('Pressure no longer confirms exposed antenna; GPS suspended')
        self.position[2] = depth

    @staticmethod
    def fresh(stamp, now):
        return stamp is not None and 0 <= now-stamp < .5

    def gps_eligible(self, now):
        return (self.fresh(self.depth_at, now) and self.shallow_since is not None
                and self.depth <= .15 and now-self.shallow_since >= 2.)

    def dvl(self, now, velocity, valid):
        valid = (valid and len(velocity) == 3 and all(math.isfinite(v) for v in velocity)
                 and math.sqrt(sum(v*v for v in velocity)) <= 5.)
        dt = now-self.motion_at if self.motion_at is not None else None
        # Silence and explicit invalid reports use the same propagation path.
        if self.initialized and self.dvl_good_at is not None and now-self.dvl_good_at > .25:
            self.start_dvl_gap()
        if self.dvl_gap_start is not None:
            self.coast(now)
        self.gps.quality.motion(now, velocity, valid)
        if not valid or (dt is not None and not 0 < dt < .5):
            self.stationary_since = self.stationary_anchor = None
            self.history.clear()
            self.start_dvl_gap()
            self.coast(now)
        self.motion_valid = bool(valid)
        self.motion_at = now
        if not valid:
            self.pending_gps = None
            self.history.clear()
            self.dvl_recovery_count = 0
            self.recovery_since = self.recovery_velocity = None
            return
        recovering = self.dvl_gap_start is not None
        if recovering:
            if not self.fault:
                if (self.recovery_velocity is not None and
                        max(abs(a-b) for a,b in zip(velocity,self.recovery_velocity)) > .2):
                    self.dvl_recovery_count = 0
                    self.recovery_since = None
                if self.recovery_since is None:
                    self.recovery_since = now
                self.recovery_velocity = tuple(velocity)
                self.dvl_recovery_count += 1
                if self.dvl_recovery_count >= 3 and now-self.recovery_since >= .2:
                    self.dvl_gap_start = None
                    self.gap_bound = 0.
            dt = None  # Already propagated by coast; never integrate a gap twice.
        self.dvl_good_at = now
        self.raw_velocity = tuple(velocity)
        if self.initialized and not self.fresh(self.imu_at, now):
            self.fault = 'VN stale; position propagation stopped'
        still = (self.initialized and not self.fault and self.dvl_gap_start is None
                 and self.fresh(self.imu_at, now) and self.fresh(self.depth_at, now)
                 and math.sqrt(sum(v*v for v in velocity)) <= .03
                 and max(abs(math.degrees(v)) for v in self.rates) <= 2.)
        if still:
            self.stationary_since = now if self.stationary_since is None else self.stationary_since
            if now-self.stationary_since >= 2. and self.stationary_anchor is None:
                self.stationary_anchor = tuple(self.position[:2])
        else:
            self.stationary_since = self.stationary_anchor = None
        if self.fresh(self.imu_at, now) and dt is not None and 0 < dt < .5 and not self.fault:
            world = rotate(self.orientation(), self.velocity())
            for i in (0, 1):
                self.position[i] += world[i]*dt
            if self.initialized:
                self.distance_travelled += math.hypot(world[0], world[1])*dt
                self.horizontal_variance = min(10000., self.horizontal_variance+.02*dt+.01*dt*dt)
        self.propagated_at = now
        if self.dvl_gap_start is None or self.fault:
            self.coast_world = rotate(self.orientation(), self.velocity())
            self.history.append(dict(t=now, position=tuple(self.position), q=self.orientation(),
                                     raw_velocity=self.raw_velocity, rates=self.rates))
        else:
            self.history.clear()  # Predicted velocity cannot validate GPS/north.
        if self.pending_gps is not None:
            received, fix = self.pending_gps
            age = self.gps.clock.age(fix['cube_fix_time_us'],now)
            if (now-received > .15 or age is None or age > self.gps.max_age
                    or not self.gps_eligible(now) or self.fault
                    or not self.fresh(self.imu_at,now)):
                self.pending_gps = None
            else:
                self.apply_gps(now,fix)

    def start_dvl_gap(self):
        if self.initialized and self.dvl_gap_start is None:
            self.dvl_gap_start = self.dvl_good_at
            self.dvl_dropouts += 1
            self.dvl_recovery_count = 0
            self.recovery_since = self.recovery_velocity = None
            self.gap_bound = 0.
            self.stationary_since = self.stationary_anchor = None
            self.pending_gps = None
            self.history.clear()

    def coast(self, now):
        """Bounded constant earth-velocity prediction, never a DVL measurement.

        Provisional envelope: one second, 0.2 m/s velocity error and 1 m/s²
        unobserved acceleration. Position uncertainty survives reacquisition;
        good velocity cannot erase unobserved displacement. After one second
        stop extrapolating and withhold output until measured motion returns.
        """
        if self.dvl_gap_start is None or self.fault:
            return
        age = now-self.dvl_gap_start
        if age < 0:
            self.fault = 'Navigation clock regressed; revalidate navigation'
            return
        bound = .2*age+.5*age*age
        increase = max(0., bound-self.gap_bound)
        self.missing_distance_bound += increase
        # Conservative correlated error accumulation. Reacquiring velocity is
        # not a position measurement; only actual admitted position corrections
        # can reduce the resulting covariance.
        self.horizontal_variance = min(10000., (math.sqrt(self.horizontal_variance)+increase)**2)
        self.gap_bound = bound
        if self.horizontal_variance > 25.:
            self.fault = 'DVL continuity lost; position uncertainty requires revalidation'
            return
        if not self.fresh(self.imu_at,now) or not self.fresh(self.depth_at,now):
            return
        if self.propagated_at is not None and now >= self.propagated_at:
            end=min(now,self.dvl_gap_start+1.)
            dt = max(0.,end-self.propagated_at)
            for i in (0,1):
                self.position[i] += self.coast_world[i]*dt
            self.distance_travelled += math.hypot(*self.coast_world[:2])*dt
            self.propagated_at = max(self.propagated_at,end)

    def orientation(self):
        return yaw_rotate(self.raw_q, self.north_offset+self.heading_bias)

    def recover_dvl(self, now):
        """Explicit surfaced re-anchor; never invent displacement across a gap."""
        if self.fault != 'DVL continuity lost; position uncertainty requires revalidation':
            raise ValueError('Recovery is only available for a DVL continuity fault')
        if (not self.heading_validated or self.heading_continuity_lost
                or not self.fresh(self.imu_at, now)):
            raise ValueError('VN continuity or north reference lost; Prepare and align north again')
        if not self.gps_eligible(now) or self.origin is None:
            raise ValueError('Recovery requires calibrated floating surface and unchanged Cube origin')
        rows = [r for r in self.history if 0 <= now-r['t'] <= 2.]
        if (len(rows) < 10 or rows[-1]['t']-rows[0]['t'] < 1.5
                or now-rows[-1]['t'] > .25 or not self.motion_valid
                or not self.fresh(self.motion_at, now)):
            raise ValueError('Hold stationary with continuous DVL bottom lock for two seconds')
        if any(math.sqrt(sum(v*v for v in r['raw_velocity'])) > .05
               or max(abs(math.degrees(v)) for v in r['rates']) > 2. for r in rows):
            raise ValueError('Hold stationary during DVL recovery')
        clock = self.gps.clock
        self.gps = GPSAdmission()
        self.gps.clock = clock
        self.initialized = False
        self.stationary_since = self.stationary_anchor = None
        self.position[:2] = [0., 0.]
        self.horizontal_variance = 9.
        self.velocity_bias = [0., 0., 0.]
        self.history.clear()
        self.pending_gps = self.last_fix = self.last_correction = None
        self.heading_since, self.heading_count = None, 0
        self.dvl_gap_start, self.dvl_recovery_count = None, 0
        self.propagated_at = now
        self.recovery_since = self.recovery_velocity = None
        self.missing_distance_bound = self.gap_bound = 0.
        self.fault = ''
        return 'North alignment retained; waiting for fresh qualified GPS to re-anchor position'

    def align_known_heading(self, now, degrees):
        if (isinstance(degrees,bool) or not isinstance(degrees,(int,float))
                or not math.isfinite(degrees) or not 0 <= degrees < 360):
            raise ValueError('Enter a true heading from 0 inclusive to 360 exclusive')
        if self.initialized or self.heading_validated or self.north_alignment is None:
            raise ValueError('Run Prepare again before changing the north reference')
        if self.fault or not self.gps_eligible(now):
            raise ValueError(self.fault or 'Prepare at the floating surface first')
        rows=[r for r in self.history if 0 <= now-r['t'] <= 2.]
        if (len(rows)<10 or rows[-1]['t']-rows[0]['t']<1.5
                or now-rows[-1]['t']>.25 or not self.fresh(self.imu_at,now)
                or not self.motion_valid):
            raise ValueError('Hold stationary for two seconds with fresh VN and DVL bottom lock')
        for row in rows:
            w,x,y,z=row['q']
            if (math.sqrt(sum(v*v for v in row['raw_velocity']))>.05
                    or max(abs(math.degrees(v)) for v in row['rates'])>2.
                    or 1-2*(x*x+y*y)<math.cos(math.radians(15))):
                raise ValueError('Keep Graey level and stationary while capturing true heading')
            if abs(wrap(yaw(row['q'])-yaw(self.raw_q)))>3.:
                raise ValueError('VN heading changing; hold the nose still before capture')
        offset=wrap(degrees-yaw(self.raw_q))
        self.north_offset=offset
        self.heading_validated=True
        self.north_alignment.offset=offset
        self.north_alignment.method='Operator measured true heading'
        self.north_alignment.reference=dict(true_heading_deg=degrees,vn_heading_deg=yaw(self.raw_q),
                                            captured_monotonic=now)
        self.north_alignment.reason=f'True heading {degrees:.1f} deg captured; independently verify alignment'
        self.position[:2]=[0.,0.]
        self.pending_gps=None
        for row in self.history:
            row['position']=(0.,0.,row['position'][2])
            row['q']=yaw_rotate(row['q'],offset)
        return self.north_alignment.reason

    def velocity(self):
        if self.dvl_gap_start is not None:
            return rotate(self.orientation(), self.coast_world, inverse=True)
        if self.stationary_anchor is not None:
            return (0., 0., 0.)
        return tuple(a+b for a, b in zip(self.raw_velocity, self.velocity_bias))

    def at(self, stamp):
        # Interpolate position; bound attitude/velocity alignment separately.
        if not self.history:
            return None
        for a, b in zip(self.history, list(self.history)[1:]):
            if a['t'] <= stamp <= b['t'] and 0 < b['t']-a['t'] <= .25:
                f = (stamp-a['t'])/(b['t']-a['t'])
                return dict(a, position=tuple(x+(y-x)*f for x, y in zip(a['position'], b['position'])))
        last = self.history[-1]
        if 0 <= stamp-last['t'] <= .1:
            return last
        return None

    def observe_gps(self, now, fix):
        self.last_observed_gps = dict(fix)
        duplicate = fix['cube_fix_time_us'] == self.gps.last_fix and fix['fix'] >= 3
        self._previous_anchors = self.gps.quality.anchors.copy()
        eligible = self.gps_eligible(now) and not self.fault
        good = self.gps.check(now, fix, self.fresh(self.imu_at, now),
            eligible=eligible, eligibility_reason=self.fault or 'Pressure surface eligibility unavailable')
        self.event = dict(decision='rejected', reason=self.gps.reason, received=now,
                          cube_fix_time_us=fix['cube_fix_time_us'], raw_lat=fix['lat'], raw_lon=fix['lon'],
                          age_upper_s=self.gps.age_bound, depth_m=self.depth)
        if not good:
            if not duplicate:
                self.pending_gps = None
            if self.north_alignment and not self.heading_validated and not duplicate:
                self.north_alignment.reject(self.gps.reason)
            if not duplicate:
                self.heading_since, self.heading_count = None, 0
            return False
        self.pending_gps = None
        return self.apply_gps(now,fix)

    def apply_gps(self, now, fix):
        if self.origin is None:
            return self.reject('Cube geographic origin unavailable')
        if self.dvl_gap_start is not None:
            self.event.update(decision='deferred', reason='DVL prediction cannot validate GPS or north alignment')
            return False
        sample_time = fix['cube_fix_time_us']/1e6+self.gps.clock.offset
        sample = self.at(sample_time)
        if sample is None:
            # A fix may precede the next ~9Hz DVL publication. Wait for a
            # bracketing sample instead of extrapolating or resetting GPS dwell.
            if self.history and self.history[-1]['t'] < sample_time <= now:
                if self.pending_gps is None:
                    self.pending_gps = (now,dict(fix))
                reason='Waiting for next DVL sample to time-align GPS'
            else:
                self.pending_gps = None
                reason='GPS cannot be aligned to DVL/VN history'
            self.event.update(decision='deferred',reason=reason)
            if self.north_alignment and not self.heading_validated:
                self.north_alignment.reason=reason
            return False
        self.pending_gps = None
        if self.north_alignment and not self.heading_validated:
            aligned = self.north_alignment.observe(now, fix,
                rotate(sample['q'], sample['raw_velocity']), sample['rates'], sample['q'])
            if not aligned:
                self.event.update(decision='aligning', reason=self.north_alignment.reason)
                return False
            # No external output/position initialization is permitted before
            # alignment. Discard coordinates integrated in the relative frame;
            # the first admitted fix anchors north/east in the corrected frame.
            self.north_offset = self.north_alignment.offset
            self.heading_validated = True
            self.position[:2] = [0., 0.]
            for row in self.history:
                row['position'] = (0., 0., row['position'][2])
                row['q'] = yaw_rotate(row['q'], self.north_offset)
            sample = self.at(sample_time)
        lat, lon, _ = self.origin
        gps_ne = ((fix['lat']-lat)*111320.,
                  ((fix['lon']-lon+180)%360-180)*111320.*math.cos(math.radians(lat)))
        residual = tuple(x-y for x, y in zip(gps_ne, sample['position'][:2]))
        magnitude = math.hypot(*residual)
        if self.initialized:
            limit = min(5., max(2., 3*math.sqrt(self.horizontal_variance+fix['accuracy']**2)))
            if magnitude > limit:
                return self.reject('GPS disagrees with propagated position; no reset applied')
            if self.stationary_anchor is not None:
                self.last_fix = dict(fix)
                self.last_stationary_fix_at = now
                self.event.update(decision='withheld', reason='DVL stationary hold; GPS position correction withheld')
                return False
            gain = min(self.horizontal_variance/(self.horizontal_variance+fix['accuracy']**2),
                       .1/max(magnitude, .000001))
            delta = [gain*v for v in residual]
            self.horizontal_variance = max(.25, (1-gain)*self.horizontal_variance)
        else:
            delta = list(residual)
            self.horizontal_variance = max(.25, fix['accuracy']**2)
            self.initialized = True  # first fix anchors frame BEFORE first odometry
        for i in (0, 1):
            self.position[i] += delta[i]
        for row in self.history:
            row['position'] = (row['position'][0]+delta[0], row['position'][1]+delta[1], row['position'][2])
        self.correct_motion(now, fix, sample)
        self.last_fix, self.last_correction = dict(fix), now
        self.event.update(decision='accepted', reason='GPS corrected external odometry',
                          position_correction_m=delta, heading_bias_deg=self.heading_bias,
                          velocity_bias_mps=list(self.velocity_bias))
        return True

    def reject(self, reason):
        self.gps.quality.anchors = self._previous_anchors
        self.gps.invalidate(reason)
        self.heading_since, self.heading_count = None, 0
        self.event.update(decision='rejected', reason=reason)
        return False

    def correct_motion(self, now, fix, sample):
        speed, course, accuracy = (fix.get(k) for k in ('speed_mps', 'course_deg', 'speed_accuracy_mps'))
        if (not self.heading_validated or any(v is None or not math.isfinite(v) for v in (speed, course, accuracy))
                or not 0 <= course < 360 or speed < .5 or not 0 < accuracy <= .2
                or math.degrees(math.atan2(accuracy, speed)) > 10):
            self.heading_since, self.heading_count = None, 0
            return
        body = sample['raw_velocity']
        if (body[0] < .5 or abs(body[1]) > body[0]*math.tan(math.radians(10))
                or abs(body[2]) > .15 or abs(math.degrees(sample['rates'][2])) > 5
                or abs(math.hypot(*body[:2])-speed) > max(.2, 3*accuracy)):
            self.heading_since, self.heading_count = None, 0
            return
        error = wrap(course-yaw(sample['q']))
        if abs(error) > 15:
            self.heading_since, self.heading_count = None, 0
            return
        self.heading_since = now if self.heading_since is None else self.heading_since
        self.heading_count += 1
        if now-self.heading_since < 3 or self.heading_count < 5:
            return
        self.heading_bias = max(-15., min(15., self.heading_bias+max(-.5, min(.5, .1*error))))
        gps_world = (speed*math.cos(math.radians(course)), speed*math.sin(math.radians(course)), 0.)
        expected = rotate(sample['q'], gps_world, inverse=True)
        # Do not learn a lateral DVL bias from yaw error while correcting yaw.
        if abs(error) > 3:
            return
        for i in (0,):
            innovation = expected[i]-body[i]-self.velocity_bias[i]
            if abs(innovation) <= .5:
                self.velocity_bias[i] = max(-.15, min(.15, self.velocity_bias[i]+.05*innovation))

    def status(self, now):
        if self.initialized and self.dvl_good_at is not None and now-self.dvl_good_at > .25:
            self.start_dvl_gap()
        self.coast(now)
        if (not self.motion_valid or not self.fresh(self.motion_at,now)
                or not self.fresh(self.imu_at,now) or not self.fresh(self.depth_at,now) or self.fault):
            self.stationary_since = self.stationary_anchor = None
        if not self.gps_eligible(now):
            self.gps.invalidate('Pressure surface eligibility unavailable')
            self.heading_since, self.heading_count = None, 0
        if self.initialized and not self.fresh(self.imu_at, now):
            self.heading_continuity_lost = True
            self.fault = self.fault or 'VN freshness lost; revalidate navigation'
        recovering = self.dvl_gap_start is not None
        coasting = recovering and not self.fault and 0<=now-self.dvl_gap_start<1.
        healthy = (self.initialized and not self.fault
                   and (coasting or (not recovering and self.motion_valid and self.fresh(self.motion_at, now)))
                   and self.fresh(self.imu_at, now)
                   and self.fresh(self.depth_at, now))
        gps = self.gps.status(now)
        gps.update(fix=self.last_fix, applied_recently=self.last_correction is not None
                   and 0 <= now-self.last_correction <= .35,
                   stationary_validated_recently=self.stationary_anchor is not None
                   and self.last_stationary_fix_at is not None and 0 <= now-self.last_stationary_fix_at <= .35)
        if self.north_alignment and not self.heading_validated:
            healthy = False
        return dict(healthy=bool(healthy), navigation_valid=bool(healthy and self.horizontal_variance <= 25.),
                    fault=self.fault, initialized=self.initialized,
                    dvl_continuity=dict(recovering=self.dvl_gap_start is not None,
                        state='POSITION_UNCERTAIN' if self.fault else 'REACQUIRING' if recovering and self.dvl_recovery_count else 'COASTING' if coasting else 'WAITING_FOR_DVL' if recovering else 'TRACKING',
                        prediction_active=bool(coasting and healthy),
                        missing_distance_bound_m=self.missing_distance_bound,
                        dropouts=self.dvl_dropouts,valid_recovery_samples=self.dvl_recovery_count,
                        maximum_gap_s=1.,position_sigma_limit_m=5.,
                        position_sigma_m=math.sqrt(self.horizontal_variance),
                        requires_position_revalidation=bool(self.fault)),
                    depth_m=self.depth if self.fresh(self.depth_at, now) else None,
                    surfaced=self.surface if self.fresh(self.depth_at, now) else None,
                    gps_eligible=self.gps_eligible(now), gps_admission=gps,
                    gps_observation=self.last_observed_gps,
                    position_ned=list(self.position), distance_travelled_m=self.distance_travelled,
                    position_sigma_m=math.sqrt(self.horizontal_variance),
                    heading_bias_deg=self.heading_bias, velocity_bias_mps=list(self.velocity_bias),
                    heading_reference_validated=self.heading_validated,
                    heading_continuity_lost=self.heading_continuity_lost,
                    stationary_hold=dict(active=self.stationary_anchor is not None,
                        dwell_s=max(0.,now-self.stationary_since) if self.stationary_since is not None else 0.,
                        anchor_ne=self.stationary_anchor, speed_threshold_mps=.03,
                        reason='DVL stationary; GPS translation withheld' if self.stationary_anchor is not None else 'Waiting for two seconds of stationary valid DVL/VN'),
                    north_alignment=self.north_alignment.status() if self.north_alignment else None)

    def covariance(self):
        cov = [0.]*21
        cov[0] = cov[6] = self.horizontal_variance
        cov[11] = .01  # provisional 0.1 m depth sigma
        cov[15] = cov[18] = math.radians(5)**2
        cov[20] = math.radians(10)**2
        return cov
