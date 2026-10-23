"""Explicit-run GPS-coordinate / relative DVL track alignment.

Fits a rotation and translation, never equates course over ground with nose
heading. Only admitted GPS paired to estimator history enters this object.
The maneuver must finish braking before a candidate can become an offset.
"""
import math
from .north_alignment import wrap


class TrackNorth:
    def __init__(self, maximum_uncertainty_deg=25., maximum_gap_s=1.5):
        self.maximum_uncertainty_deg=maximum_uncertainty_deg
        self.maximum_gap_s=maximum_gap_s
        self.offset = self.reference = self.candidate = None
        self.method = 'GPS coordinate track / relative DVL displacement'
        self.rows = []
        self.active = self.approved = False
        self.last_fix = None
        self.reason = 'Prepare surface sensors, then explicitly start the 5 m alignment run'

    def begin(self):
        self.__init__(self.maximum_uncertainty_deg,self.maximum_gap_s)
        self.active = True
        self.reason = 'Recording GPS coordinates and time-matched relative DVL positions'

    def reject(self, reason, reset=False):
        self.reason = reason
        self.candidate = None
        self.approved = False
        if reset:
            self.rows.clear()
            self.candidate = None
        return False

    def stop(self, complete=False):
        if complete and self.active and self.candidate is not None:
            self.approved = True
            self.reason = 'Motion stopped; awaiting fresh GPS to apply accepted north offset'
        else:
            self.active = False
            self.candidate = None
            self.rows.clear()
            self.reason = 'Alignment stopped without accepting north; explicitly retry after inspection'

    def observe(self, now, fix, velocity, rates, q, position=None):
        if self.offset is not None:
            return True
        if not self.active:
            return False
        stamp = fix['cube_fix_time_us']
        if self.last_fix is not None and stamp <= self.last_fix:
            return self.reject('Duplicate or reversed GPS timestamp', reset=stamp < self.last_fix)
        self.last_fix = stamp
        if self.approved and self.candidate is not None:
            self.offset = self.candidate['offset_deg']
            self.reference = dict(self.candidate)
            self.reason = 'North accepted from GPS coordinate track after controlled stop'
            self.active = False
            return True
        if position is None or any(not math.isfinite(v) for v in (*position, *rates, *q)):
            return self.reject('Time-matched DVL/VN track unavailable', reset=True)
        if max(abs(math.degrees(r)) for r in rates) > 5 or 1-2*(q[1]**2+q[2]**2) < math.cos(math.radians(15)):
            return self.reject('Keep level and straight during track capture')
        if self.rows and now-self.rows[-1][0] > self.maximum_gap_s:
            return self.reject('GPS track continuity interrupted; retry run', reset=True)
        self.rows.append((now, fix['lat'], fix['lon'], *position[:2], fix['accuracy'], stamp))
        self.rows = self.rows[-600:]
        if len(self.rows) < 20:
            return self.reject('Collecting GPS coordinate track')
        lat, lon = self.rows[0][1:3]
        gps = [((r[1]-lat)*111320, ((r[2]-lon+180)%360-180)*111320*math.cos(math.radians(lat))) for r in self.rows]
        dvl = [(r[3], r[4]) for r in self.rows]
        def centered(rows):
            mean = [sum(r[i] for r in rows)/len(rows) for i in (0,1)]
            return [(r[0]-mean[0], r[1]-mean[1]) for r in rows]
        g, d = centered(gps), centered(dvl)
        travel = math.dist(dvl[0], dvl[-1])
        if travel < 4:
            return self.reject(f'Recording coordinate track: {travel:.1f}/4 m usable baseline')
        dot = sum(a*c+b*e for (a,b),(c,e) in zip(d,g))
        cross = sum(a*e-b*c for (a,b),(c,e) in zip(d,g))
        angle = math.atan2(cross, dot)
        cs, sn = math.cos(angle), math.sin(angle)
        energy = sum(a*a+b*b for a,b in d)
        scale = math.hypot(dot,cross)/max(energy,1e-9)
        residual = math.sqrt(sum((cs*a-sn*b-c)**2+(sn*a+cs*b-e)**2 for (a,b),(c,e) in zip(d,g))/len(d))
        # Reported accuracy is not reduced by sqrt(N): correlated GPS bias is
        # common. A 5 m run may legitimately be insufficient in poor reception.
        uncertainty = math.degrees(math.atan2(2*max(r[5] for r in self.rows),travel))
        # DVL sets metric distance; GPS contributes direction, never a scale
        # correction. Tolerate ordinary metre-level GNSS disagreement over 5 m,
        # while retaining explicit gross-fit and angular-quality limits.
        if not .5 <= scale <= 1.5 or residual > 2. or uncertainty > self.maximum_uncertainty_deg:
            self.candidate = None
            return self.reject(f'Track rejected: scale {scale:.2f}, residual {residual:.2f} m, heading uncertainty {uncertainty:.1f} deg')
        self.candidate = dict(offset_deg=wrap(math.degrees(angle)), baseline_m=travel,
            residual_m=residual, heading_uncertainty_deg=uncertainty, gps_dvl_scale=scale,
            samples=len(d), method=self.method, gps_track=[list(r) for r in self.rows])
        self.reason = 'Coordinate track candidate ready; waiting for maneuver stop verification'
        return False

    def status(self):
        reference = ({k:v for k,v in self.reference.items() if k!='gps_track'}
                     if self.reference else None)
        return dict(aligned=self.offset is not None, offset_deg=self.offset,
            reason=self.reason, samples=len(self.rows), method=self.method,
            reference=reference, candidate_ready=self.candidate is not None,
            active=self.active, persistence='bridge session only')
