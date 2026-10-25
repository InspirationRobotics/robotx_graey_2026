"""Independent sensor updates and non-latching health for navigation_new."""
import math
from .relative_estimator import RelativeEstimator
from .horizontal_ekf import HorizontalEKF
from .bridge_estimator import rotate, yaw, yaw_rotate
from .geographic_frame import rotation
from .gps_quality import GPSQuality


class IndependentGPSQuality(GPSQuality):
    """Admission must not depend on DVL being present. EKF checks innovation."""
    def check(self, now, fix):
        if not all(isinstance(fix.get(k), (int, float)) and math.isfinite(fix[k]) for k in ('lat','lon','accuracy')):
            return False, 'Nonfinite GPS measurement'
        if fix['fix'] < 3 or not 0 < fix['accuracy'] <= 1.5:
            return False, 'GPS fix or reported accuracy unsuitable'
        if not (-90 <= fix['lat'] <= 90 and -180 <= fix['lon'] <= 180):
            return False, 'GPS coordinates out of range'
        if self.anchors:
            t, lat, lon, _ = self.anchors[-1]
            distance=math.hypot((fix['lat']-lat)*111320,
                ((fix['lon']-lon+180)%360-180)*111320*math.cos(math.radians(lat)))
            if distance > 4.+2.*max(0.,now-t):
                return False, 'GPS jump rejected'
        self.anchors.append((now,fix['lat'],fix['lon'],0.))
        while len(self.anchors)>20: self.anchors.popleft()
        return True, 'GPS admitted independently of DVL'


class ContinuousEstimator(RelativeEstimator):
    def __init__(self):
        super().__init__()
        self.filter=HorizontalEKF()
        self.gps.quality=IndependentGPSQuality()
        self.gps.dwell=0.
        self.track_position=[0.,0.,0.]
        self.track_time=None
        self.last_sample=None
        self.started=False

    def sync(self, now):
        self.filter.predict(now)
        self.position[:2]=[float(v) for v in self.filter.x[:2]]
        self.horizontal_variance=float(max(self.filter.P[0,0],self.filter.P[1,1]))
        self.initialized=self.filter.started

    def imu(self, now, q, rates):
        if len(q)!=4 or len(rates)!=3 or not all(math.isfinite(v) for v in (*q,*rates)):
            return
        norm=math.sqrt(sum(v*v for v in q))
        if not .9<norm<1.1:return
        q=tuple(v/norm for v in q)
        if self.filter.vn(now,math.radians(yaw(q)),rates[2]):
            self.raw_q=q;self.rates=tuple(rates);self.imu_at=now
        self.sync(now)

    def orientation(self):
        return yaw_rotate(self.raw_q,math.degrees(self.filter.x[4])-yaw(self.raw_q))

    def velocity(self):
        return rotate(self.orientation(),(*self.filter.x[2:4],0.),inverse=True)

    def dvl(self, now, velocity, valid):
        self.sync(now)
        good=bool(valid and len(velocity)==3 and all(math.isfinite(v) for v in velocity) and math.sqrt(sum(v*v for v in velocity))<=5.)
        self.motion_valid=good
        if not good:return
        # Tilt-compensate with VN; preserve body yaw for the nonlinear update.
        level=rotate(yaw_rotate(self.orientation(),-math.degrees(self.filter.x[4])),velocity)
        if not self.filter.dvl(now,*level[:2]):return
        previous=self.motion_at
        self.motion_at=self.dvl_good_at=now;self.raw_velocity=tuple(velocity)
        self.dvl_gap_start=None
        self.sync(now)
        # Heading evidence uses a separate DVL-only track: GPS must not
        # manufacture the track later used to validate its own heading.
        world=rotate(self.orientation(),velocity)
        if previous is not None and 0<now-previous<.5:
            dt=now-previous
            for i in (0,1):self.track_position[i]+=world[i]*dt
            self.distance_travelled+=math.hypot(*world[:2])*dt
        self.course_heading.motion(now,world,self.rates,True)
        self.history.append(dict(t=now,position=tuple(self.position),track_position=tuple(self.track_position),
            q=self.orientation(),raw_velocity=self.raw_velocity,rates=self.rates,
            course_motion_ne=tuple(self.course_heading.motion_ne),course_distance_m=self.course_heading.distance))

    def at(self,stamp):
        result=super().at(stamp)
        if result is None:return None
        result=dict(result)
        for a,b in zip(self.history,list(self.history)[1:]):
            if a['t']<=stamp<=b['t'] and b['t']>a['t']:
                f=(stamp-a['t'])/(b['t']-a['t'])
                result['track_position']=tuple(x+f*(y-x) for x,y in zip(a['track_position'],b['track_position']))
                break
        return result

    def observe_gps(self, now, fix):
        self.last_observed_gps=dict(fix)
        self.sync(now)
        # VN and DVL health never prevent a GPS measurement update.
        if not self.gps.check(now,fix,True,eligible=self.gps_eligible(now)):
            self.event=dict(decision='rejected',reason=self.gps.reason)
            return False
        return self.apply_gps(now,fix)

    def apply_gps(self, now, fix):
        sample=self.at(fix['cube_fix_time_us']/1e6+self.gps.clock.offset)
        a=self.north_alignment
        if sample and self.fresh(self.imu_at,now) and self.fresh(self.motion_at,now) and a.active:
            a.observe(now,fix,rotate(sample['q'],sample['raw_velocity']),sample['rates'],sample['q'],position=sample['track_position'])
            if a.offset is not None and self.map.angle is None:
                self.map.align(fix['lat'],fix['lon'],sample['position'],a.offset)
                self.heading_validated=True
        if self.map.angle is not None:
            point=self.map.gps_to_local(fix['lat'],fix['lon'])
            # Delay compensation uses the predicted velocity, with latency
            # included in observation noise; not a claim of receiver-time sync.
            age=self.gps.clock.age(fix['cube_fix_time_us'],now) or 0.
            point=[point[i]+self.filter.x[2+i]*age for i in (0,1)]
            if not self.filter.gps(now,point,fix['accuracy']+age*.3):
                self.event=dict(decision='rejected',reason='GPS EKF innovation rejected')
                return False
            if sample and self.fresh(self.motion_at,now) and self.fresh(self.imu_at,now):
                transformed=dict(sample)
                transformed['course_motion_ne']=rotation(sample['course_motion_ne'],self.map.angle)
                world=rotation(rotate(sample['q'],sample['raw_velocity']),self.map.angle)
                step=self.course_heading.observe(now,fix,transformed,(*world,0.))
                distance=math.dist(self.position[:2],self.map.gps_to_local(*self.target_gps)) if self.target_gps else math.inf
                if distance>2. and step:
                    self.map.correct_angle(max(-.25,min(.25,step)),self.position)
        else:
            # An admitted fix supplies geographic evidence before registration;
            # it cannot yet correct the fixed-local position.
            self.filter.accepted['gps']=now
        self.last_fix=dict(fix);self.last_correction=now
        self.event=dict(decision='accepted',reason='GPS accepted')
        self.sync(now)
        return True

    def coast(self,now): self.sync(now)
    def start_dvl_gap(self): pass

    def status(self,now):
        self.sync(now)
        health=self.filter.health(now)
        health['gps_position_fused']=self.map.angle is not None and health['sensors']['gps']
        if self.map.angle is None and health['usable']:health['state']='ALIGNING'
        gps=self.gps.status(now)
        gps.update(fix=self.last_fix,applied_recently=self.last_correction is not None and now-self.last_correction<2.,stationary_validated_recently=False)
        valid=health['usable'] and not self.fault
        return dict(healthy=valid,navigation_valid=valid,local_navigation_ready=valid,
            estimator_health=health,initialized=self.initialized,fault=self.fault,
            coordinate_frame='fixed_local',map_transform=self.map.snapshot(),
            map_position_ne=self.map.to_map(self.position) if self.map.angle is not None else None,
            map_heading_deg=(yaw(self.orientation())+self.map.angle)%360 if self.map.angle is not None else None,
            depth_m=self.depth,surfaced=self.surface,gps_eligible=self.gps_eligible(now),gps_admission=gps,
            gps_observation=self.last_observed_gps,position_ned=list(self.position),position_sigma_m=health['position_sigma_m'],
            distance_travelled_m=self.distance_travelled,heading_bias_deg=0.,velocity_bias_mps=[0.,0.,0.],
            heading_reference_validated=self.heading_validated,heading_continuity_lost=False,
            course_heading=dict(self.course_heading.status(),application_policy='every admitted fix; rolling displacement evidence'),
            north_alignment=self.north_alignment.status(),stationary_hold=dict(active=False),
            dvl_continuity=dict(recovering=False,state='AIDED' if health['sensors']['dvl'] else 'PREDICTING',
                prediction_active=not health['sensors']['dvl'],requires_position_revalidation=False))

    def covariance(self):
        cov=super().covariance()
        cov[20]=float(self.filter.P[4,4])
        return cov

    def velocity_covariance(self):
        cov=[0.]*21
        cov[0]=cov[6]=float(max(self.filter.P[2,2],self.filter.P[3,3]))
        cov[11]=.25
        cov[15]=cov[18]=cov[20]=math.radians(5)**2
        return cov
