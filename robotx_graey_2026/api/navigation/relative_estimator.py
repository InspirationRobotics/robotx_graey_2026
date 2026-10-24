"""Continuous DVL/VN local odometry with a separately registered geographic map."""
import math
from .bridge_estimator import BridgeEstimator, rotate, yaw
from .geographic_frame import GeographicFrame, rotation
from .track_north import TrackNorth
from .waypoint_logic import geo_to_ne


class RelativeEstimator(BridgeEstimator):
    fixed_local=True
    def __init__(self):
        super().__init__()
        self.north_alignment=TrackNorth(maximum_uncertainty_deg=35.,maximum_gap_s=5.)
        self.map=GeographicFrame()
        self.target_gps=None
        self.recovery_angle=None
        self.pending_course_step=None
        self.last_segment_correction=None
        self.course_heading.preserve_stationary_anchor=True
        # Metre-level receiver error is ordinary; reject gross discontinuities.
        self.gps.quality.max_accuracy=1.5
        self.gps.quality.motion_margin=4.

    def apply_gps(self, now, fix):
        sample=self.at(fix['cube_fix_time_us']/1e6+self.gps.clock.offset)
        if sample is None or self.dvl_gap_start is not None:
            self.event.update(decision='deferred',reason='Waiting for time-matched DVL/VN')
            self.pending_gps=(now,dict(fix))
            return False
        self.pending_gps=None
        if self.recovery_angle is not None:
            self.map.align(fix['lat'],fix['lon'],sample['position'],self.recovery_angle)
            self.north_alignment.offset=self.recovery_angle
            self.heading_validated=True
            self.recovery_angle=None
        # First qualified fix enables local output, NOT a false north heading.
        self.initialized=True
        a=self.north_alignment
        if a.active and not self.heading_validated:
            a.observe(now,fix,rotate(sample['q'],sample['raw_velocity']),sample['rates'],sample['q'],position=sample['position'])
            if a.offset is not None:
                self.map.align(fix['lat'],fix['lon'],sample['position'],a.offset)
                self.heading_validated=True
        if self.map.angle is not None:
            residual=math.dist(self.map.to_map(sample['position']),
                geo_to_ne(fix['lat'],fix['lon'],self.map.origin))
            if residual>max(6.,4*fix['accuracy']):
                return self.reject('GPS jump exceeds geographic-map agreement bound')
            if self.stationary_anchor is None:
                self.map.correct_position(fix['lat'],fix['lon'],sample['position'])
                distance=math.inf
                if self.target_gps:
                    distance=math.dist(self.map.gps_to_local(*self.target_gps),self.position[:2])
                # Never shrink the evidence baseline below what GPS can resolve.
                # Near arrival keep VN-relative heading, not noisy tiny baselines.
                if distance>2.:
                    world=rotate(sample['q'],sample['raw_velocity'])
                    world=(*rotation(world,self.map.angle),world[2])
                    transformed=dict(sample)
                    transformed['course_motion_ne']=rotation(sample['course_motion_ne'],self.map.angle)
                    step=self.course_heading.observe(now,fix,transformed,world)
                    step=max(-min(.5,.1*distance),min(min(.5,.1*distance),step))
                    # Retain qualified track evidence while moving, but only
                    # apply a heading correction at an authorized stopped check.
                    if step:self.pending_course_step=(now,step)
                else:self.course_heading.reset_window('Near waypoint; geographic heading correction frozen')
            else:self.last_stationary_fix_at=now
        self.last_fix,self.last_correction=dict(fix),now
        self.horizontal_variance=max(.25,min(self.horizontal_variance,fix['accuracy']**2))
        self.event.update(decision='accepted',reason='GPS admitted for Jetson geographic map; Cube frame unchanged')
        return True

    def apply_segment_correction(self,now,token):
        if token==self.last_segment_correction or self.pending_course_step is None:return False
        stamp,step=self.pending_course_step
        if not 0<=now-stamp<=10.:
            self.pending_course_step=None;return False
        if self.map.angle is None or not self.target_gps:return False
        distance=math.dist(self.map.gps_to_local(*self.target_gps),self.position[:2])
        self.pending_course_step=None
        if distance<=2.:return False
        self.map.correct_angle(max(-min(.5,.1*distance),min(min(.5,.1*distance),step)),self.position)
        self.course_heading.reset_window('Applied stopped-segment correction; collecting new frame-consistent evidence')
        self.last_segment_correction=token
        return True

    def status(self, now):
        s=super().status(now)
        # Relative odometry can be healthy before a geographic bearing exists.
        local_ready=(self.initialized and not self.fault and not self.heading_continuity_lost and self.fresh(self.imu_at,now)
            and self.fresh(self.depth_at,now) and (self.motion_valid and self.fresh(self.motion_at,now)
            or s['dvl_continuity']['prediction_active']))
        s.update(healthy=bool(local_ready),navigation_valid=bool(local_ready and self.horizontal_variance<=25),
            coordinate_frame='fixed_local',local_navigation_ready=bool(local_ready),map_transform=self.map.snapshot(),
            map_position_ne=self.map.to_map(self.position) if self.map.angle is not None else None,
            map_heading_deg=(yaw(self.orientation())+self.map.angle)%360 if self.map.angle is not None else None)
        s['course_heading'].update(application_policy='stopped one-metre check only',
            pending_correction_deg=self.pending_course_step[1] if self.pending_course_step else None,
            last_applied_segment=self.last_segment_correction)
        return s

    def recover_dvl(self, now):
        # Base recovery changes local origin. Re-registering is mandatory; do not
        # preserve a transform that refers to a discarded local position.
        position=list(self.position);angle=self.map.angle
        result=super().recover_dvl(now)
        self.position[:2]=position[:2]
        self.gps.quality.max_accuracy=1.5;self.gps.quality.motion_margin=4.
        self.map=GeographicFrame();self.north_alignment=TrackNorth(maximum_uncertainty_deg=35.,maximum_gap_s=5.);self.heading_validated=False
        self.recovery_angle=angle
        self.pending_course_step=None;self.last_segment_correction=None
        self.course_heading.preserve_stationary_anchor=True
        return result
