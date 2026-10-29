"""Continuous surface mission. Health comes from Jetson, not sensor gates."""
import math
from .waypoint_logic import WaypointRun
from .geographic_frame import GeographicFrame
from .geographic_frame import rotation


class GuidedSurfaceRun(WaypointRun):
    # Jetson owns the waypoint; Cube GUIDED controls motion from its EKF.
    moving={'CHECK_SURFACE','TRANSIT','TRANSIT_CHECK','STOP'}
    arrival_radius=.5
    segment_length=0.

    def __init__(self,plan,run_id,now):
        super().__init__(plan,run_id,now)
        self.alignment_length=10.
        self.gps_start=None;self.gps_alignment=None;self.guided_target=None;self.stabilize_requested=False
        self.align_start=None;self.align_heading=None;self.alignment_distance=0.
        self.alignment_started=None;self.transit_started=None
        self.sample_check_ready=False;self.segment_checks=0
        self.geographic_tracking=False;self.alignment_complete=False
        self.alignment_stopped=False
        self.heading_reset_revision=0
        self.last_local=None

    def apply_heading_reset(self, reset):
        revision=(reset or {}).get('revision',0)
        if revision==self.heading_reset_revision:return
        self.heading_reset_revision=revision
        angle=reset['offset_deg'];anchor=reset['anchor']
        for name in ('start','align_start','guided_target','target'):
            p=getattr(self,name,None)
            if p is not None:
                xy=rotation((p[0]-anchor[0],p[1]-anchor[1]),angle)
                setattr(self,name,(xy[0]+anchor[0],xy[1]+anchor[1],*p[2:]))
        for name in ('heading','align_heading'):
            value=getattr(self,name,None)
            if value is not None:setattr(self,name,(value+math.radians(angle)+math.pi)%(2*math.pi)-math.pi)

    def abort_actions(self,now,o):
        if self.takeover_requested and not self.pilot_takeover:
            return self.command(now,'manual')
        return []

    def step(self,now,o):
        gap,self.last=now-self.last,now
        if self.stage=='COMPLETE':return []
        if self.stage=='ABORTED':return self.abort_actions(now,o)
        b=o.get('bridge') or {};health=b.get('estimator_health') or {}
        self.apply_heading_reset(b.get('heading_reset'))
        moving=self.stage in self.moving
        failure=o.get('failure') or ''
        if o.get('abort'):failure='Operator aborted'
        if moving and (o.get('armed') is False or o.get('mode') not in (None,4)):
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
            return self.command(now,'guided_hold',(*self.last_local[:2],self.surface_down),interval=.1) if self.last_local is not None and self.surface_down is not None and moving else []
        local=tuple(o['local']);self.last_local=local
        frame=GeographicFrame.restore(b.get('map_transform'))
        if self.stage=='WAIT_SURFACE':
            self.reason='Waiting for navigation startup and an initialized estimate'
            if b.get('enabled') and b.get('configuration_verified') and health.get('usable') and frame.angle is not None:
                self.start=local;self.start_gps=frame.to_geo(local);self.surface_down=local[2]
                self.heading=o['yaw'];self.gps_start=self.start_gps
                self.enter('GUIDED',now,'Requesting GUIDED for EKF waypoint navigation')
        elif self.stage=='GUIDED':
            if o.get('mode')==4:self.enter('ARM',now,'Waiting for arming')
            else:return self.command(now,'guided')
        elif self.stage=='ARM':
            if o.get('mode')!=4:
                self.abort(now,'Pilot changed mode',pilot=True);return []
            if o.get('armed'):
                self.align_start=local;self.align_heading=o['yaw']
                self.alignment_started=self.transit_started=now
                self.surface_down=local[2]
                self.gps_start=tuple(o['raw_gps']) if o.get('surface_gps_current') and o.get('raw_gps') else self.gps_start
                self.alignment_complete=True
                self.enter('TRANSIT',now,'EKF surface waypoint navigation')
            elif self.plan.get('qgc_goto'):
                self.abort(now,'QGC Go-to cancelled by disarm',pilot=True);return []
            else:return self.command(now,'arm')
        elif self.stage in ('TRANSIT','TRANSIT_CHECK'):
            if frame.angle is None:
                self.reason='Waiting for EKF geographic position; waypoint retained'
                return self.command(now,'guided_hold',(*local[:2],self.surface_down),interval=.1)
            if not self.geographic_tracking:
                self.start=local;self.geographic_tracking=True
                self.enter('TRANSIT',now,'Continuous waypoint navigation')
            target=frame.gps_to_local(self.plan['lat'],self.plan['lon'])
            self.target=(*target,self.surface_down)
            distance=math.dist(local[:2],target)
            if self.settled(now,distance<=self.arrival_radius and o['speed']<=.15):
                self.enter('STOP',now,'Estimated arrival within 0.5 m; stopping')
                return self.command(now,'guided_hold',(*local[:2],self.surface_down),interval=.1)
            if distance<=self.arrival_radius:return self.command(now,'guided_hold',(*local[:2],self.surface_down),interval=.1)
            return self.command(now,'guided_target',self.target,interval=.1)
        elif self.stage=='STOP':
            if self.settled(now,o['speed']<=.08):self.enter('DISARM',now,'Stopped at waypoint')
            return self.command(now,'guided_hold',(*local[:2],self.surface_down),interval=.1)
        elif self.stage=='DISARM':
            if o.get('armed'):return self.command(now,'disarm')
            self.enter('COMPLETE',now,'Estimated waypoint reached; stopped and disarmed')
            return [('manual',None)]
        return []

    def snapshot(self):
        s=super().snapshot()
        s.update(gps_alignment=self.gps_alignment,alignment_stopped=self.alignment_stopped,alignment_method='manual_only',sample_check_ready=self.sample_check_ready,alignment_distance_m=self.alignment_distance,
            coordinate_frame='fixed_local',jetson_map_axes='x=north, y=east',arrival_radius_m=self.arrival_radius,
            segment_length_m=0.,segment_checks=0,segment_check_dwell_s=0.)
        return s
