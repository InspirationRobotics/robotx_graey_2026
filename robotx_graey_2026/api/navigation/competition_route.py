"""Onboard route sequencing; browser and RF connectivity are not mission inputs."""
import math
from .route_plan import validate_route
from .guided_surface_mission import GuidedSurfaceRun
from .guided_orientation import GuidedOrientation
from .geographic_frame import GeographicFrame


class CompetitionRouteRun(GuidedSurfaceRun):
    pipeline_stages = {'RECORD', 'PIPELINE_ORIENT', 'PIPELINE_DIVE', 'PIPELINE', 'PIPELINE_SURFACE'}
    moving = GuidedSurfaceRun.moving | pipeline_stages

    def __init__(self, plan, run_id, now):
        self.route = validate_route(plan)
        self.points = [p for p in self.route['points'] if p['kind'] != 'pipeline_buoy']
        self.point_index = 0
        self.recording_requested = False
        self.recording_warning = ''
        self.pipeline_heading = None
        self.pipeline_hold = None
        self.depth_target = 0.
        self.depth_reference_z = None
        self.depth_source = 'Waiting for pressure reference'
        super().__init__(self.route, run_id, now)

    @property
    def phase(self):
        return 'UNDERWATER' if self.stage in self.pipeline_stages or self.depth_target > 0 else 'SURFACE_NAV'

    def next_point(self, now, o):
        self.point_index += 1
        if self.point_index == len(self.points):
            self.enter('STOP', now, 'Route complete; stopping')
            return
        point = self.points[self.point_index]
        self.plan.update(lat=point['lat'], lon=point['lon'])
        frame = GeographicFrame.restore(o['bridge']['map_transform'])
        target = frame.gps_to_local(point['lat'], point['lon'])
        self.orientation = GuidedOrientation((*o['local'][:2], self.surface_down), target)
        self.geographic_tracking = False
        self.enter('ORIENT', now, f'Orienting toward point {self.point_index+1}')

    def step(self, now, o):
        pressure = o.get('depth')
        if (o.get('fresh') and o.get('local') is not None and
                isinstance(pressure, (float, int)) and math.isfinite(pressure)):
            self.depth_reference_z = o['local'][2]-pressure
            self.depth_source = 'Pressure depth'
        if self.stage not in self.pipeline_stages:
            old = self.stage
            actions = super().step(now, o)
            if old in ('TRANSIT', 'TRANSIT_CHECK') and self.stage == 'STOP':
                if self.points[self.point_index]['kind'] == 'pipeline_start':
                    self.recording_requested = self.route.get('recording_mode','automatic')=='automatic'
                    end = self.points[self.point_index+1]
                    frame = GeographicFrame.restore(o['bridge']['map_transform'])
                    start = self.points[self.point_index]
                    self.pipeline_hold = frame.gps_to_local(start['lat'], start['lon'])
                    self.pipeline_end = frame.gps_to_local(end['lat'], end['lon'])
                    self.plan.update(lat=end['lat'], lon=end['lon'])
                    self.pipeline_heading = math.atan2(self.pipeline_end[1]-self.pipeline_hold[1],
                                                       self.pipeline_end[0]-self.pipeline_hold[0])
                    self.orientation = GuidedOrientation((*self.pipeline_hold, self.surface_down), self.pipeline_end)
                    self.recording_warning = ''
                    self.enter('PIPELINE_ORIENT', now, 'Facing pipeline end; '+('starting recorders in background' if self.recording_requested else 'recording controlled by operator'))
                else:
                    self.next_point(now, o)
            return actions
        gap, self.last = now-self.last, now
        failure = o.get('failure') or ('Operator aborted' if o.get('abort') else '')
        bridge = o.get('bridge') or {}
        self.apply_heading_reset(bridge.get('heading_reset'))
        if o.get('armed') is False or o.get('mode') not in (None, 4):
            self.abort(now, 'Pilot/failsafe took control', pilot=True)
            self.recording_requested = False
            return []
        if not 0 <= gap < 1.:
            failure = 'Mission command loop interrupted'
        if (bridge.get('estimator_health') or {}).get('all_sensors_lost'):
            failure = 'Jetson estimator lost GPS, DVL and VN'
        failure = failure or bridge.get('fault')
        if failure:
            self.abort(now, failure)
            self.recording_requested = False
            return self.abort_actions(now, o)
        if not o.get('fresh'):
            self.reason = 'Waiting for onboard control telemetry; route retained'
            return []
        local = tuple(o['local'])
        self.last_local = local
        recording = o.get('recording') or {}
        if self.stage == 'RECORD':
            self.orientation = GuidedOrientation((*self.pipeline_hold, self.surface_down), self.pipeline_end)
            self.enter('PIPELINE_ORIENT', now, 'Facing pipeline end; recording does not gate motion')
            return self.command(now, 'guided_hold', (*self.pipeline_hold, self.surface_down), .1)
        if (self.stage in ('PIPELINE_ORIENT', 'PIPELINE_DIVE', 'PIPELINE')
                and not recording.get('ready')):
            self.recording_warning = recording.get('reason', 'Recording unavailable')
        # Pressure depth defines 4 m below the water surface, not DVL altitude.
        depth = o.get('depth')
        if not isinstance(depth, (float, int)) or not math.isfinite(depth):
            if self.depth_reference_z is None:
                self.reason = 'No depth reference yet; holding last vertical target'
                return self.command(now, 'guided_hold', (*local[:2], self.surface_down), .1)
            depth = local[2]-self.depth_reference_z
            self.depth_source = 'Cube prediction from last pressure surface reference'
        surface_z = local[2]-depth
        if self.stage == 'PIPELINE_ORIENT':
            if self.orientation.ready(o['yaw']):
                self.depth_target = 4.
                self.enter('PIPELINE_DIVE', now, 'Descending to 4 m at pipeline start')
            else:
                return self.command(now, 'guided_orient', self.orientation.target(), .1)
        if self.stage == 'PIPELINE_DIVE':
            self.surface_down = surface_z+4.
            self.target = (*self.pipeline_hold, self.surface_down)
            if self.settled(now, abs(depth-4.) <= .15):
                self.enter('PIPELINE', now, 'Surveying start-to-end line at 4 m')
            return self.command(now, 'guided_orient', (*self.pipeline_hold, self.surface_down, self.pipeline_heading), .1)
        if self.stage == 'PIPELINE':
            self.surface_down = surface_z+4.
            self.target = (*self.pipeline_end, self.surface_down)
            arrived = math.dist(local[:2], self.pipeline_end) <= self.arrival_radius
            if self.settled(now, arrived and o['speed'] <= .15 and abs(depth-4.) <= .15):
                self.point_index += 1  # paired end consumed; buoy annotations never execute
                self.pipeline_hold = local[:2]
                self.enter('PIPELINE_SURFACE', now, 'Pipeline complete; surfacing')
            target = local[:2] if arrived else self.pipeline_end
            return self.command(now, 'guided_orient', (*target, self.surface_down, self.pipeline_heading), .1)
        if self.stage == 'PIPELINE_SURFACE':
            self.surface_down = surface_z
            self.target = (*self.pipeline_hold, self.surface_down)
            if self.settled(now, depth <= .15):
                self.depth_target = 0.
                self.recording_requested = False
                self.next_point(now, o)
            return self.command(now, 'guided_hold', (*self.pipeline_hold, self.surface_down), .1)
        return []

    def apply_heading_reset(self, reset):
        if reset and reset.get('revision', 0) != self.heading_reset_revision:
            from .geographic_frame import rotation
            anchor, angle = reset['anchor'], reset['offset_deg']
            for name in ('pipeline_hold', 'pipeline_end'):
                point = getattr(self, name, None)
                if point is not None:
                    xy = rotation((point[0]-anchor[0], point[1]-anchor[1]), angle)
                    setattr(self, name, (xy[0]+anchor[0], xy[1]+anchor[1]))
            if self.pipeline_heading is not None:
                self.pipeline_heading = (self.pipeline_heading+math.radians(angle)+math.pi)%(2*math.pi)-math.pi
        super().apply_heading_reset(reset)

    def snapshot(self):
        status = super().snapshot()
        status.update(route=self.route, route_point_index=self.point_index,
                      route_point_order=self.points[self.point_index]['display_order'] if self.point_index < len(self.points) else None,
                      recording_requested=self.recording_requested,
                      recording_warning=self.recording_warning,
                      depth_source=self.depth_source,
                      commanded_depth_m=self.depth_target)
        return status
