"""Continuous surface mission. Health comes from Jetson, not sensor gates."""
import math
from .waypoint_logic import WaypointRun
from .geographic_frame import GeographicFrame
from .straight_alignment import straight_command


class GuidedSurfaceRun(WaypointRun):
    # Name retained for launch/API compatibility; runtime mode is STABILIZE.
    moving={'CHECK_SURFACE','TRANSIT','TRANSIT_CHECK','STOP'}
    arrival_radius=.5
    segment_length=0.

    def __init__(self,plan,run_id,now):
        super().__init__(plan,run_id,now)
        if plan.get('alignment_corridor_confirmed') is not True:
            raise ValueError('Confirm clear water for the initial 10 m track')
        self.align_start=None;self.align_heading=None;self.alignment_distance=0.
        self.alignment_started=None;self.transit_started=None
        self.sample_check_ready=False;self.segment_checks=0
        self.geographic_tracking=False

    def abort_actions(self,now,o):
        if self.takeover_requested and not self.pilot_takeover and o.get('mode') in (None,0):
            return self.command(now,'manual')
        return []

    def step(self,now,o):
        gap,self.last=now-self.last,now
        if self.stage=='COMPLETE':return []
        if self.stage=='ABORTED':return self.abort_actions(now,o)
        b=o.get('bridge') or {};health=b.get('estimator_health') or {}
        moving=self.stage in self.moving
        failure=o.get('failure') or ''
        if o.get('abort'):failure='Operator aborted'
        if moving and (o.get('armed') is False or o.get('mode') not in (None,0)):
            self.abort(now,'Pilot/failsafe took control',pilot=True);return []
        if moving and not 0<=gap<1.:failure='Mission command loop interrupted'
        if moving and health.get('all_sensors_lost') is True:
            failure='Jetson estimator lost GPS, DVL and VN'
        if b.get('fault'):failure=b['fault']
        if failure:
            self.abort(now,failure);return self.abort_actions(now,o)
        # Missing control telemetry cannot be replaced by a sensor vote. Do not
        # fabricate commands; retain mission and recover when telemetry returns.
        if not o.get('fresh'):
            self.reason='Waiting for control telemetry; mission retained'
            return self.command(now,'stop',(self.heading,),interval=.1) if self.heading is not None else []
        local=tuple(o['local'])
        if self.stage=='WAIT_SURFACE':
            self.reason='Waiting for navigation startup and an initialized estimate'
            if b.get('enabled') and b.get('configuration_verified') and health.get('usable'):
                self.start=local;self.start_gps=o.get('global');self.surface_down=local[2]
                self.heading=o['yaw']
                self.enter('GUIDED',now,'Requesting STABILIZE; vertical input neutral')
        elif self.stage=='GUIDED':
            if o.get('mode')==0:self.enter('ARM',now,'Waiting for arming')
            else:return self.command(now,'guided')
        elif self.stage=='ARM':
            if o.get('mode')!=0:
                self.abort(now,'Pilot changed mode',pilot=True);return []
            if o.get('armed'):
                self.align_start=local;self.align_heading=o['yaw']
                self.alignment_started=self.transit_started=now
                self.enter('TRANSIT',now,'10 m straight alignment, then continuous waypoint navigation')
            else:return self.command(now,'arm')
        elif self.stage in ('TRANSIT','TRANSIT_CHECK'):
            frame=GeographicFrame.restore(b.get('map_transform'))
            if frame.angle is None:
                along,cross,command=straight_command(self.align_start,local,o['velocity_ned'],self.align_heading)
                self.alignment_distance=along
                if along>=9.9 or self.stage=='TRANSIT_CHECK':
                    self.sample_check_ready=True
                    if self.stage!='TRANSIT_CHECK':self.enter('TRANSIT_CHECK',now,'10 m complete; accepting GPS north evidence')
                    self.reason='Waiting for usable GPS north evidence; waypoint retained'
                    return self.command(now,'stop',(self.align_heading,),interval=.1)
                return self.command(now,'align_velocity',command,interval=.1)
            if not self.geographic_tracking:
                self.start=local;self.geographic_tracking=True
                self.enter('TRANSIT',now,'Continuous waypoint navigation')
            target=frame.gps_to_local(self.plan['lat'],self.plan['lon'])
            self.target=(*target,self.surface_down)
            distance=math.dist(local[:2],target)
            if self.settled(now,distance<=self.arrival_radius and o['speed']<=.15):
                self.enter('STOP',now,'Estimated arrival within 0.5 m; stopping')
                return self.command(now,'stop',(o['yaw'],),interval=.1)
            if distance<=self.arrival_radius:return self.command(now,'stop',(o['yaw'],),interval=.1)
            return self.command(now,'target',(*self.target,self.heading),interval=.1)
        elif self.stage=='STOP':
            if self.settled(now,o['speed']<=.08):self.enter('DISARM',now,'Stopped at waypoint')
            return self.command(now,'stop',(o['yaw'],),interval=.1)
        elif self.stage=='DISARM':
            if o.get('armed'):return self.command(now,'disarm')
            self.enter('COMPLETE',now,'Estimated waypoint reached; stopped and disarmed')
            return [('manual',None)]
        return []

    def snapshot(self):
        s=super().snapshot()
        s.update(sample_check_ready=self.sample_check_ready,alignment_distance_m=self.alignment_distance,
            coordinate_frame='fixed_local',jetson_map_axes='x=north, y=east',arrival_radius_m=self.arrival_radius,
            segment_length_m=0.,segment_checks=0,segment_check_dwell_s=0.)
        return s
