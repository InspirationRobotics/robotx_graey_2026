"""Surface-only DVL position revalidation; no vehicle-control commands."""
import math
from .gps_admission import GPSAdmission
from .maintenance_safety import maintenance_reason

DVL_FAULT = 'DVL continuity lost; position uncertainty requires revalidation'


class AutomaticRecovery:
    def __init__(self):
        self.candidate = GPSAdmission()
        self.reason = 'Monitoring DVL continuity'
        self.state = 'idle'
        self.count = 0
        self.last_motion = None
        self.points = []
        self.last_fix = None
        self.pending = False
        self.complete = False

    @staticmethod
    def safe(now, control, received):
        return (received is not None and 0<=now-received<.5
                and not maintenance_reason(dict(control,online=True))
                and control.get('mode') == 'MANUAL')

    def reset_candidate(self):
        self.candidate = GPSAdmission()
        self.last_motion = self.last_fix = None
        self.points.clear()

    def step(self, now, est, control, received):
        safe = self.safe(now,control,received)
        if self.pending:
            if not safe:
                self.state='reanchoring'
                self.reason='Recovery paused until MANUAL, killed/disarmed and ESC off'
                return False
            if est.fault:
                self.pending=False
                self.complete=False
            elif est.initialized and est.status(now)['healthy'] and est.gps.status(now)['qualified']:
                self.pending=False
                self.complete=True
                self.state='recovered'
                self.reason='Position revalidated from surface GPS; north retained; mission not restarted'
            else:
                self.state='reanchoring'
                self.reason='DVL restored; waiting for normal GPS qualification to resume odometry'
            return False
        if est.fault != DVL_FAULT:
            if est.fault or not self.complete:
                self.complete=False
                self.state='blocked' if est.fault else 'idle'
                self.reason=est.fault or 'Monitoring DVL continuity'
            self.reset_candidate()
            return False
        self.complete=False
        self.state='waiting'
        if not safe:
            self.reason='Automatic recovery waits for MANUAL, killed/disarmed, ESC off and no mission'
            self.reset_candidate()
            return False
        try:
            if est.gps.fault:
                raise ValueError(est.gps.fault)
            # Reuse existing physical recovery checks without changing estimator.
            if not est.heading_validated or est.heading_continuity_lost or not est.fresh(est.imu_at,now):
                raise ValueError('North/VN continuity unavailable; Prepare required')
            if not est.gps_eligible(now) or est.origin is None:
                raise ValueError('Waiting for calibrated floating surface and unchanged origin')
            rows=[r for r in est.history if 0<=now-r['t']<=2.]
            if (len(rows)<10 or rows[-1]['t']-rows[0]['t']<1.5
                    or not est.motion_valid or not est.fresh(est.motion_at,now)):
                raise ValueError('Waiting for stable DVL bottom lock')
            if any(math.sqrt(sum(v*v for v in r['raw_velocity']))>.05
                   or max(abs(math.degrees(v)) for v in r['rates'])>2. for r in rows):
                raise ValueError('Waiting for stationary DVL/VN readings')
            fix=est.last_observed_gps or {}
            if not 0<=now-fix.get('received',-math.inf)<.35 or not 0<fix.get('accuracy',0)<=2:
                raise ValueError('Waiting for fresh GPS accuracy <=2 m')
            self.candidate.clock=est.gps.clock
            if est.motion_at!=self.last_motion:
                self.candidate.quality.motion(est.motion_at,est.raw_velocity,True)
                self.last_motion=est.motion_at
            stamp=fix.get('cube_fix_time_us')
            if stamp==self.last_fix:return False
            self.last_fix=stamp
            good=self.candidate.check(now,fix,True)
            if self.candidate.last_rejection and not good and self.candidate.reason!='GPS qualifying':
                self.points.clear()
            if self.candidate.reason in ('GPS qualifying','GPS admitted; Cube-fix timing approximation'):
                self.points.append((now,fix['lat'],fix['lon']))
                self.points=[p for p in self.points if now-p[0]<=5.]
            self.reason='Recovery GPS: '+self.candidate.reason
            if not good:return False
            if len(self.points)<10 or self.points[-1][0]-self.points[0][0]<3.:
                return False
            lat,lon=self.points[0][1:]
            spread=max(math.hypot((p[1]-lat)*111320,
                (p[2]-lon)*111320*math.cos(math.radians(lat))) for p in self.points)
            if spread>1.:
                raise ValueError('Recovery GPS moved by more than 1 m; waiting for stable fixes')
            est.recover_dvl(now)
            self.count+=1
            self.pending=True
            self.state='reanchoring'
            self.reason='North retained; requalifying GPS before restoring odometry'
            self.reset_candidate()
            return True
        except (ValueError,KeyError,TypeError) as error:
            self.reason=str(error)
            self.reset_candidate()
            return False

    def status(self, now, control, received):
        return dict(state=self.state,reason=self.reason,count=self.count,
                    completed=self.complete,maintenance_safe=self.safe(now,control,received))
