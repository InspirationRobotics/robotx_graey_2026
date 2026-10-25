"""Slow north-offset corrections from admitted GPS versus measured DVL motion.

Course is a velocity direction, not the vehicle's nose. Compare earth-frame GPS
motion with rotated body-frame DVL motion, including lateral/reverse travel.
Never use the requested waypoint/course as a heading measurement.
"""
import math
from collections import deque


def wrap(degrees):
    return (degrees+180.) % 360.-180.


def angle(v):
    return math.degrees(math.atan2(v[1],v[0]))


class CourseHeading:
    def __init__(self):
        self.motion_ne=[0.,0.]
        self.distance=0.
        self.last_motion=None
        self.velocity_rows=deque()
        self.anchor=None
        self.reason='Waiting for admitted surface GPS and measured DVL motion'
        self.method=None
        self.last_update=None
        self.last_error=None
        self.updates=0
        self.preserve_stationary_anchor=False

    def reset_window(self,reason):
        self.velocity_rows.clear();self.anchor=None;self.reason=reason

    def motion(self,now,world,rates,valid=True):
        previous=self.last_motion
        self.last_motion=now
        if (not valid or previous is None or not 0<now-previous<.5
                or not all(math.isfinite(v) for v in (*world,*rates))):
            self.reset_window('Measured DVL/VN continuity required for heading correction')
            return
        dt=now-previous
        for i in (0,1):self.motion_ne[i]+=world[i]*dt
        self.distance+=math.hypot(*world[:2])*dt
        if abs(math.degrees(rates[2]))>3.:
            self.reset_window('Turning; retain VN orientation, pause north-offset learning')

    def observe(self,now,fix,sample,world):
        """Called only after time alignment and GPS admission/position gates."""
        if self.last_update is not None and now-self.last_update<3.:
            self.reason='Correction settling; waiting for independent motion evidence'
            return 0.
        rates=sample['rates']
        if (self.preserve_stationary_anchor and all(math.isfinite(v) for v in (*world,*rates))
                and math.hypot(*world[:2])<.08 and abs(world[2])<=.15
                and abs(math.degrees(rates[2]))<=3. and max(abs(math.degrees(r)) for r in rates[:2])<=10.):
            self.velocity_rows.clear()
            self.reason='Low motion; retaining cumulative position baseline'
            return 0.
        if (not all(math.isfinite(v) for v in (*world,*rates))
                or abs(math.degrees(rates[2]))>3.
                or max(abs(math.degrees(r)) for r in rates[:2])>10. or abs(world[2])>.15
                or math.hypot(*world[:2])<.08):
            self.reset_window('Stationary, turning or vertical motion; no heading correction')
            return 0.
        speed,course,sacc=(fix.get(k) for k in ('speed_mps','course_deg','speed_accuracy_mps'))
        velocity_good=(all(isinstance(v,(float,int)) and math.isfinite(v) for v in (speed,course,sacc))
                       and speed>=.08 and 0<=course<360 and 0<sacc
                       and math.degrees(math.atan2(sacc,speed))<=12.
                       and abs(speed-math.hypot(*world[:2]))<=max(.06,3*sacc))
        if velocity_good:
            error=wrap(course-angle(world))
            if self.velocity_rows and now-self.velocity_rows[-1][0]>.8:self.velocity_rows.clear()
            self.velocity_rows.append((now,error))
            while self.velocity_rows and now-self.velocity_rows[0][0]>4.:self.velocity_rows.popleft()
            if len(self.velocity_rows)>=8 and now-self.velocity_rows[0][0]>=3.:
                mean=angle((sum(math.cos(math.radians(r[1])) for r in self.velocity_rows),
                            sum(math.sin(math.radians(r[1])) for r in self.velocity_rows)))
                if max(abs(wrap(r[1]-mean)) for r in self.velocity_rows)<=5.:
                    return self.correct(now,mean,'GPS Doppler velocity versus DVL motion')
                self.reason='GPS/DVL direction inconsistent across the window'
            else:self.reason='Collecting three seconds of accurate GPS/DVL velocity agreement'
        else:
            self.velocity_rows.clear()
            self.reason='GPS velocity too uncertain; waiting for a long enough position baseline'
        # Position-displacement alternative for slow vehicles whose Doppler
        # uncertainty is too large. Endpoint uncertainty is NOT reduced by a
        # naive sqrt(sample-count); correlated GPS bias remains possible.
        trajectory=sample.get('course_motion_ne')
        distance=sample.get('course_distance_m')
        accuracy=fix.get('accuracy')
        if trajectory is None or distance is None or not isinstance(accuracy,(int,float)) or not 0<accuracy<=3.:
            return 0.
        point=(fix['lat'],fix['lon'])
        if self.anchor is None:
            self.anchor=(now,point,tuple(trajectory),distance,accuracy)
            return 0.
        t,p,start,start_distance,start_accuracy=self.anchor
        dt=now-t
        if not 0<dt<=60.:
            self.anchor=None;return 0.
        north=(point[0]-p[0])*111320.
        east=((point[1]-p[1]+180)%360-180)*111320.*math.cos(math.radians(p[0]))
        gps=(north,east);dvl=tuple(trajectory[i]-start[i] for i in (0,1))
        baseline=math.hypot(*gps);travel=math.hypot(*dvl)
        uncertainty=math.hypot(accuracy,start_accuracy)
        if (dt>=5. and baseline>=2. and travel>=1.
                and math.degrees(math.atan2(uncertainty,baseline))<=12.
                and travel>=.95*max(.001,distance-start_distance)
                and abs(baseline-travel)<=max(.3,2*uncertainty)):
            return self.correct(now,wrap(angle(gps)-angle(dvl)),
                                'GPS position baseline versus integrated DVL motion')
        return 0.

    def correct(self,now,error,method):
        self.last_error=error
        if abs(error)>25.:
            self.reset_window('GPS/DVL heading disagreement exceeds 25 degrees; reference needs review')
            return 0.
        step=max(-.5,min(.5,.15*error))
        self.last_update=now;self.method=method;self.updates+=1
        self.reset_window('Applied bounded north-offset correction; collecting new evidence')
        return step

    def status(self):
        return dict(reason=self.reason,method=self.method,updates=self.updates,
                    last_update=self.last_update,last_error_deg=self.last_error,
                    velocity_samples=len(self.velocity_rows),maximum_step_deg=.5,
                    minimum_update_interval_s=3.,minimum_speed_mps=.08)
