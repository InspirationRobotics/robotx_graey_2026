"""Independent sensor updates and non-latching health for navigation_new."""
import math
from .relative_estimator import RelativeEstimator
from .horizontal_ekf import HorizontalEKF
from .bridge_estimator import rotate, yaw, yaw_rotate
from .geographic_frame import rotation
from .gps_quality import GPSQuality
from .waypoint_logic import geo_to_ne
from .geographic_frame import GeographicFrame


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
        self.alignment_run_id=None
        self.alignment_collecting=False
        self.track_start_variance=0.
        self.yaw_offset_deg=0.
        self.heading_reset=None
        self.manual_north=False
        self.pending_manual_heading=None
        self.north_alignment.reason="Manual heading correction available; automatic alignment disabled"

    def align_known_heading(self, now, degrees):
        if isinstance(degrees,bool) or not isinstance(degrees,(int,float)) or not math.isfinite(degrees) or not 0<=degrees<360:
            raise ValueError('Enter a true heading from 0 inclusive to 360 exclusive')
        if not self.filter.started:
            self.pending_manual_heading=degrees
            return 'Known heading stored; awaiting the first VN sample'
        self.sync(now)
        geographic=self.map.to_geo(self.position) if self.map.angle is not None else None
        previous=yaw(self.orientation())
        offset=(degrees-previous+180)%360-180
        self.correct_heading(offset,now)
        self.alignment_collecting=False
        self.manual_north=self.heading_validated=True
        a=self.north_alignment
        a.active=a.approved=False;a.candidate=None
        a.offset=offset;a.method='Operator measured true heading'
        a.reference=dict(true_heading_deg=degrees,previous_heading_deg=previous,captured_monotonic=now)
        a.reason=f'Manual north aligned: bow heading {degrees:.1f} deg true'
        if geographic:self.map.align(*geographic,self.position,0.)
        return a.reason

    def correct_heading(self, degrees, now):
        """Apply accepted north to Cube-bound yaw and the matching local frame."""
        anchor=tuple(self.position)
        self.filter.rotate_frame(degrees,anchor)
        self.yaw_offset_deg=(self.yaw_offset_deg+degrees+180)%360-180
        def point(p):
            xy=rotation((p[0]-anchor[0],p[1]-anchor[1]),degrees)
            return (xy[0]+anchor[0],xy[1]+anchor[1],*p[2:])
        self.track_position=list(point(self.track_position))
        for row in self.history:
            row['position']=point(row['position'])
            row['track_position']=point(row['track_position'])
            row['q']=yaw_rotate(row['q'],degrees)
            row['course_motion_ne']=(*rotation(row['course_motion_ne'],degrees),*row['course_motion_ne'][2:])
        revision=(self.heading_reset or {}).get('revision',0)+1
        self.heading_reset=dict(revision=revision,offset_deg=degrees,anchor=list(anchor),
            yaw_offset_deg=self.yaw_offset_deg,at=now)
        self.course_heading.reset_window('North applied to Cube yaw; collect motion in corrected frame')
        self.sync(now)
        return point

    def begin_guided_alignment(self, run_id):
        """Automatic north alignment is disabled; retain the EKF reference."""
        return

    def finish_guided_alignment(self):
        return

    def sync(self, now):
        self.filter.predict(now)
        self.position[:2]=[float(v) for v in self.filter.x[:2]]
        self.horizontal_variance=float(max(self.filter.P[0,0],self.filter.P[1,1]))
        self.initialized=self.filter.started
        if self.alignment_collecting:
            # GPS updates are gated for this entire run. The EKF local pose is
            # therefore independent of the GPS observations fitting north.
            self.track_position[:2]=self.position[:2]
            row=dict(t=now,position=tuple(self.position),track_position=tuple(self.track_position),
                track_sigma_m=math.sqrt(max(0.,self.horizontal_variance-self.track_start_variance)),
                q=self.orientation(),raw_velocity=self.raw_velocity,rates=self.rates,
                course_motion_ne=tuple(self.course_heading.motion_ne),course_distance_m=self.course_heading.distance)
            if self.history and self.history[-1]['t']==now:self.history[-1]=row
            elif not self.history or now-self.history[-1]['t']>=.02:self.history.append(row)

    def imu(self, now, q, rates):
        if len(q)!=4 or len(rates)!=3 or not all(math.isfinite(v) for v in (*q,*rates)):
            return
        norm=math.sqrt(sum(v*v for v in q))
        if not .9<norm<1.1:return
        q=tuple(v/norm for v in q)
        if self.filter.vn(now,math.radians(yaw(q)+self.yaw_offset_deg),rates[2]):
            self.raw_q=q;self.rates=tuple(rates);self.imu_at=now
        self.sync(now)
        if self.pending_manual_heading is not None:
            heading=self.pending_manual_heading;self.pending_manual_heading=None
            self.align_known_heading(now,heading)

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
        if not self.alignment_collecting and previous is not None and 0<now-previous<.5:
            dt=now-previous
            for i in (0,1):self.track_position[i]+=world[i]*dt
            self.distance_travelled+=math.hypot(*world[:2])*dt
        self.course_heading.motion(now,world,self.rates,True)
        if not self.alignment_collecting:
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
                result['track_sigma_m']=max(a.get('track_sigma_m',0.),b.get('track_sigma_m',0.))
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
        if self.map.angle is None:
            age=self.gps.clock.age(fix['cube_fix_time_us'],now) or 0.
            local=sample['position'] if sample else tuple(self.position[i]-self.filter.x[2+i]*age for i in (0,1))
            self.map.align(fix['lat'],fix['lon'],local,0.)
        point=self.map.gps_to_local(fix['lat'],fix['lon'])
        # Delay compensation uses the predicted velocity, with latency
        # included in observation noise; not a claim of receiver-time sync.
        age=self.gps.clock.age(fix['cube_fix_time_us'],now) or 0.
        point=[point[i]+self.filter.x[2+i]*age for i in (0,1)]
        if not self.filter.gps(now,point,fix['accuracy']+age*.3):
            self.event=dict(decision='rejected',reason='GPS EKF innovation rejected')
            return False
        self.last_fix=dict(fix);self.last_correction=now
        self.event=dict(decision='accepted',reason='GPS position fused by EKF')
        self.sync(now)
        return True

    def coast(self,now): self.sync(now)
    def start_dvl_gap(self): pass

    def cube_position(self, position):
        """Register private EKF coordinates in Cube's geographic NED frame."""
        if self.origin is None or self.map.angle is None:return list(position)
        north,east=geo_to_ne(*self.map.to_geo(position),self.origin)
        return [north,east,*position[2:]]

    def status(self,now):
        self.sync(now)
        health=self.filter.health(now)
        health['gps_position_fused']=health['sensors']['gps']
        gps=self.gps.status(now)
        gps.update(fix=self.last_fix,applied_recently=self.last_correction is not None and now-self.last_correction<2.,stationary_validated_recently=False)
        valid=health['usable'] and not self.fault
        registered=self.origin is not None and self.map.angle is not None
        frame=self.map.snapshot()
        reset=self.heading_reset
        if registered:
            frame=GeographicFrame();frame.align(*self.origin[:2],(0.,0.),0.)
            frame=frame.snapshot()
            if reset:reset=dict(reset,anchor=self.cube_position(reset['anchor']))
        return dict(healthy=valid,navigation_valid=valid,local_navigation_ready=valid,
            manual_north=self.manual_north,
            heading_reset=reset,yaw_offset_deg=self.yaw_offset_deg,
            alignment_run_id=self.alignment_run_id,alignment_collecting=self.alignment_collecting,
            alignment_track_position=list(self.track_position),
            estimator_health=health,initialized=self.initialized,fault=self.fault,
            coordinate_frame='cube_ned' if registered else 'fixed_local',map_transform=frame,
            cube_origin_registered=registered,estimator_map_transform=self.map.snapshot(),
            estimator_position_ned=list(self.position),
            map_position_ne=self.cube_position(self.position)[:2] if registered else self.map.to_map(self.position) if self.map.angle is not None else None,
            map_heading_deg=(yaw(self.orientation())+(self.map.angle or 0.))%360 if self.filter.started else None,
            depth_m=self.depth,surfaced=self.surface,gps_eligible=self.gps_eligible(now),gps_admission=gps,
            gps_observation=self.last_observed_gps,position_ned=self.cube_position(self.position),position_sigma_m=health['position_sigma_m'],
            distance_travelled_m=self.distance_travelled,heading_bias_deg=0.,velocity_bias_mps=[0.,0.,0.],
            heading_reference_validated=self.heading_validated,heading_continuity_lost=False,
            course_heading=dict(self.course_heading.status(),application_policy='manual heading only; automatic heading correction disabled'),
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
