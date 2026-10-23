"""Surface mission in fixed Cube local XY; Jetson map is north/east metres."""
import math
from .waypoint_logic import WaypointRun, geo_to_ne
from .geographic_frame import GeographicFrame


class GuidedSurfaceRun(WaypointRun):
    moving={'CHECK_SURFACE','ALIGN_MOVE','ALIGN_CHECK','ALIGN_SETTLE','ALIGN_ACCEPT','TRANSIT','TRANSIT_CHECK','STOP'}
    arrival_radius=1.5
    segment_length=1.0

    def __init__(self,plan,run_id,now):
        super().__init__(plan,run_id,now)
        if plan.get('alignment_corridor_confirmed') is not True:
            raise ValueError('Confirm a clear initial 5 m corridor plus 2 m stopping clearance')
        self.plan['alignment_corridor_confirmed']=True
        self.align_start=None
        self.align_heading=None
        self.alignment_distance=0.
        self.alignment_started=None
        self.segment_start=None
        self.segment_distance=0.
        self.segment_checks=0
        self.check_started=None
        self.transit_started=None

    def segment_complete(self,local):
        if self.segment_start is None:self.segment_start=local
        self.segment_distance=math.dist(local[:2],self.segment_start[:2])
        return self.segment_distance>=self.segment_length

    def checked(self,now,o,b):
        # Two seconds stopped with current GPS; loss of stationarity restarts only
        # this dwell, not the accumulated GPS/DVL alignment track.
        good=(o['speed']<=.08 and abs((b.get('heading_vn') or {}).get('rate',math.inf))<=3.
              and o.get('surface_gps_current'))
        if not good:self.check_started=None;return False
        if self.check_started is None:self.check_started=now
        return now-self.check_started>=2.

    def step(self,now,o):
        gap,self.last=now-self.last,now
        if self.stage=='COMPLETE':return []
        if self.stage=='ABORTED':return self.abort_actions(now,o)
        s=o.get('supervisor') or {};b=o.get('bridge') or {}
        moving=self.stage in self.moving
        failure=o.get('failure') or ''
        if o.get('abort'):failure='Operator aborted'
        if moving and (o.get('armed') is False or o.get('mode') not in (None,4)):
            self.abort(now,'Pilot/failsafe left armed GUIDED; yielding',pilot=True);return []
        if moving and not 0<=gap<1.:failure='Mission freshness deadline missed'
        limit=240. if self.stage=='TRANSIT' else 12. if self.stage in ('GUIDED','ARM','DISARM') else 90.
        if now-self.since>limit:failure='Stage timed out: '+self.stage
        if self.transit_started is not None and now-self.transit_started>240.:
            failure='Segmented transit timed out'
        if self.reference_id and ((s.get('reference') or {}).get('reference_id')!=self.reference_id or s.get('run_id')!=self.run_id):
            failure='Mission reference changed'
        if s.get('state')=='Fault':failure=s.get('reason','Navigation fault')
        if moving and (not o.get('fresh') or not s.get('navigation_ready') or not s.get('reference_current')):
            hold=self.navigation_wait(now,o,'Waiting for fresh healthy navigation',dvl=True) if not failure else None
            if hold is not None:return hold
            failure=failure or 'Navigation unavailable beyond recovery hold'
        if moving and (o.get('dvl_continuity') or {}).get('recovering'):
            hold=self.navigation_wait(now,o,'DVL reacquiring; hold position',dvl=True) if not failure else None
            if hold is not None:return hold
            failure=failure or 'DVL recovery exceeded bounded hold'
        if self.surface_down is not None and o.get('fresh'):
            self.vertical_offset_error=o['local'][2]-o['depth']-self.surface_down
            if o['depth']>.08 or abs(self.vertical_offset_error)>.15:
                failure='Surface depth or vertical reference exceeded limit'
        if failure:
            self.abort(now,failure);return self.abort_actions(now,o)
        if not o.get('fresh') or s.get('run_id')!=self.run_id:return []
        if self.navigation_wait_since is not None:self.last_command=-math.inf;self.arrival=None
        self.navigation_wait_since=self.navigation_hold=None
        self.navigation_wait_reason=''
        local=tuple(o['local']);self.heading=o['yaw'] if self.heading is None else self.heading
        if moving:
            if self.last_position and math.dist(local,self.last_position[1])>.5+2*max(0,now-self.last_position[0]):
                self.abort(now,'Cube local position discontinuity');return self.abort_actions(now,o)
            self.last_position=(now,local)
            if o.get('surface_gps_current'):
                self.last_surface_gps=now;self.last_surface_gps_position=local
                self.surface_unconfirmed_distance=0.
            elif self.last_surface_gps is None or now-self.last_surface_gps>5.:
                self.abort(now,'Surface GPS unavailable for five seconds');return self.abort_actions(now,o)
        if self.stage=='WAIT_SURFACE':
            measurement=o.get('surface_measurement') or {}
            self.reason=measurement.get('reason','Waiting for five seconds of qualified surface measurements before GUIDED')
            if s.get('navigation_ready') and s.get('reference_current') and o.get('surface_gps_current') and measurement.get('ready'):
                self.reference_id=s['reference']['reference_id']
                self.start=local;self.start_gps=o['global'];self.surface_down=local[2]-o['depth']
                self.heading=o['yaw'];self.last_position=(now,local)
                self.enter('GUIDED',now,'Requesting GUIDED with fixed local odometry')
        elif self.stage=='GUIDED':
            if o.get('mode')==4:
                self.enter('ARM',now,'Waiting for deliberate mission arming')
                return self.command(now,'stop',(self.heading,),interval=.1)
            else:return self.command(now,'guided')
        elif self.stage=='ARM':
            if o.get('mode')!=4:
                self.abort(now,'GUIDED lost before arming',pilot=True);return []
            if o.get('armed'):
                self.last_surface_gps=now
                self.enter('CHECK_SURFACE',now,'Holding start while validating surface measurements')
                return self.command(now,'target',(*self.start,self.heading),interval=.1)
            else:return self.command(now,'arm')
        elif self.stage=='CHECK_SURFACE':
            if (o.get('surface_measurement') or {}).get('ready'):
                if (b.get('map_transform') or {}).get('ready'):
                    self.segment_start=local;self.transit_started=now
                    self.enter('TRANSIT',now,'Geographic reference retained; tracking waypoint')
                else:
                    self.align_start=local;self.align_heading=o['yaw'];self.heading=self.align_heading
                    self.alignment_started=now
                    self.segment_start=local
                    self.enter('ALIGN_MOVE',now,'GUIDED straight initial movement; measuring GPS/DVL north')
            return self.command(now,'target',(*self.start,self.heading),interval=.1)
        elif self.stage in ('ALIGN_MOVE','ALIGN_CHECK','ALIGN_SETTLE','ALIGN_ACCEPT'):
            self.alignment_distance=math.dist(local[:2],self.align_start[:2])
            if self.alignment_distance>7. or now-self.alignment_started>90.:
                self.abort(now,'Alignment travel/time limit exceeded');return self.abort_actions(now,o)
            if not o.get('surface_gps_current'):
                return self.command(now,'stop',(self.align_heading,),interval=.1)
            if self.stage=='ALIGN_MOVE':
                error=(o['yaw']-self.align_heading+math.pi)%(2*math.pi)-math.pi
                if abs(error)>math.radians(20):
                    self.abort(now,'Initial heading hold exceeded 20 degrees');return self.abort_actions(now,o)
                if self.alignment_distance>=4.5:
                    self.navigation_hold=local
                    self.enter('ALIGN_SETTLE',now,'Braking before applying geographic alignment')
                    return self.command(now,'stop',(self.align_heading,),interval=.1)
                if self.segment_complete(local):
                    self.check_started=None
                    self.enter('ALIGN_CHECK',now,'One metre: brake and check GPS/DVL; retain full alignment track')
                    return self.command(now,'stop',(self.align_heading,),interval=.1)
                return self.command(now,'align_velocity',(.2*math.cos(self.align_heading),.2*math.sin(self.align_heading),self.align_heading),interval=.1)
            if self.stage=='ALIGN_CHECK':
                if self.checked(now,o,b):
                    self.segment_start=local;self.segment_checks+=1
                    self.enter('ALIGN_MOVE',now,'Segment checked; continue collecting the cumulative north baseline')
                return self.command(now,'stop',(self.align_heading,),interval=.1)
            if self.stage=='ALIGN_SETTLE':
                rate=abs((b.get('heading_vn') or {}).get('rate',math.inf))
                if self.settled(now,o['speed']<=.08 and rate<=3.):
                    if not (b.get('north_alignment') or {}).get('candidate_ready'):
                        self.abort(now,'GPS/DVL track insufficient; no north guessed');return self.abort_actions(now,o)
                    self.enter('ALIGN_ACCEPT',now,'Stopped; waiting for geographic transform')
            elif (b.get('map_transform') or {}).get('ready'):
                self.start=local
                self.segment_start=local;self.transit_started=now
                self.enter('TRANSIT',now,'Tracking geographic waypoint in fixed local GUIDED frame')
            return self.command(now,'stop',(self.align_heading,),interval=.1)
        elif self.stage in ('TRANSIT','TRANSIT_CHECK'):
            frame=GeographicFrame.restore(b.get('map_transform'))
            if frame.angle is None:
                self.abort(now,'Geographic transform unavailable');return self.abort_actions(now,o)
            target=frame.gps_to_local(self.plan['lat'],self.plan['lon'])
            self.target=(*target,self.surface_down)
            distance=math.dist(local[:2],target)
            if distance>30.:
                self.abort(now,'Geographic target outside bounded mission area');return self.abort_actions(now,o)
            if self.settled(now,distance<=self.arrival_radius and o['speed']<=.15 and o.get('surface_gps_current')):
                self.enter('STOP',now,'Within estimated 1.5 m radius for one second; stopping')
                return self.command(now,'stop',(o['yaw'],),interval=.1)
            if distance<=self.arrival_radius:
                return self.command(now,'stop',(o['yaw'],),interval=.1)
            if self.stage=='TRANSIT_CHECK':
                if self.checked(now,o,b):
                    self.segment_start=local;self.segment_checks+=1
                    self.enter('TRANSIT',now,'Stopped data check complete; continuing next metre')
                return self.command(now,'stop',(self.heading,),interval=.1)
            if self.segment_complete(local):
                self.heading=o['yaw'];self.check_started=None
                self.enter('TRANSIT_CHECK',now,'One metre: brake, validate GPS/DVL and apply qualified correction')
                return self.command(now,'stop',(self.heading,),interval=.1)
            return self.command(now,'target',(*self.target,self.heading),interval=.1)
        elif self.stage=='STOP':
            if self.settled(now,o['speed']<=.08):self.enter('DISARM',now,'Stopped at waypoint; disarming')
            return self.command(now,'stop',(o['yaw'],),interval=.1)
        elif self.stage=='DISARM':
            if o.get('armed'):return self.command(now,'disarm')
            self.enter('COMPLETE',now,'Estimated waypoint reached; stopped and disarmed')
            return [('manual',None)]
        return []

    def snapshot(self):
        s=super().snapshot()
        s.update(alignment_distance_m=self.alignment_distance,coordinate_frame='fixed_local',
            jetson_map_axes='x=north, y=east',arrival_radius_m=self.arrival_radius,
            segment_length_m=self.segment_length,segment_distance_m=self.segment_distance,
            segment_checks=self.segment_checks,segment_check_dwell_s=2.)
        return s
