"""GPS routes started from the radio page: surface only, or dock -> pipeline dive -> dock.

Chris, 12 Nov 2026 at RobotX. Three missions on the radio page's Missions tab, all GUIDED from the
Start to a killed end, all started and stopped over the RFD900 (WiFi does not reach the water):

    dive     where Graey is now: dive_spin.py on its own (Chris drives it to the pipeline by hand)
    route    surface only: through the points in order, stop at the last one, disarm, kill
    full     the dock is where Graey is at Start. Through the points on the surface; the LAST point
             is the pipeline: the dive there (dive_spin.py), up, then back through the same points
             in reverse and to the dock, disarm, kill

    WAIT_SURFACE   until navigation is running (Start navigation: radio page, or the Sensors page)
    GUIDED, ARM    as dive_spin.py
    ORIENT         turn on the spot to face the next point
    TRANSIT        GUIDED position target at the surface depth; arrived = within ARRIVAL_M and slow
    DESCEND .. SURFACE   the dive, at the pipeline, holding the spot it arrived at
    DISARM         and the kill

The surface legs are GuidedSurfaceRun's (guided_surface_mission.py): face the point, then go, on
the Cube's EKF. Every point is turned into the local frame again each tick from its lat/lon, so a
heading reset moves the targets with the frame. Failures follow dive_spin.py: once the vehicle is
ours, anything but a pilot taking over is disarm and kill, and Graey floats where it is. At the
surface the radio reaches it, so Stop works there.
"""
import math

from .dive_spin import DIVE_STAGES, NOTE_NO_NAV, NOTE_SAFETY, DiveSpinRun, dive_target
from .geographic_frame import GeographicFrame
from .guided_orientation import GuidedOrientation
from .waypoint_logic import geo_to_ne

MAX_POINTS = 15             # what one radio COMMAND frame carries (status_codec.py)
MAX_LEG_M = 2000.           # a point further than this from the one before is a typo
ARRIVAL_M = 1.5
ARRIVED_SPEED = .15         # m/s
NAV_WAIT_S = 30.            # navigation not running by then: not started (the vehicle was never taken)
ORIENT_LIMIT_S = 30.        # facing the point not reached by then: go anyway, the transit steers


def route_points(points):
    """[(lat, lon), ...] checked: 1..MAX_POINTS real positions, no leg longer than MAX_LEG_M."""
    if not isinstance(points, (list, tuple)) or not 1 <= len(points) <= MAX_POINTS:
        raise ValueError('Route: 1 to %d points' % MAX_POINTS)
    out = []
    for i, p in enumerate(points):
        try:
            lat, lon = (float(v) for v in p)
            ok = math.isfinite(lat) and math.isfinite(lon) and -90 < lat < 90 and -180 <= lon <= 180
        except (TypeError, ValueError):
            ok = False
        if not ok or (lat, lon) == (0., 0.):
            raise ValueError('Route point %d is not a position' % (i + 1))
        if out and math.hypot(*geo_to_ne(lat, lon, out[-1])) > MAX_LEG_M:
            raise ValueError('Route point %d is more than %d m from the one before' % (i + 1, MAX_LEG_M))
        out.append((round(lat, 7), round(lon, 7)))
    return out


def mission_plan(mission):
    """The radio page's mission -> the plan waypoint_mission is started with. ValueError if not allowed.
    mission: {'kind': 'dive'|'route'|'full', 'dive': {...dive_spin.dive_target...}, 'points': [[lat, lon]]}"""
    kind = mission.get('kind') if isinstance(mission, dict) else None
    operator_kind=mission.get('operator_kind')
    if operator_kind is not None and operator_kind not in ('dive_here','pool_dive','surface_route','dock_pipeline_dock'):
        raise ValueError('Unknown operator mission kind')
    if kind == 'dive':
        return dict(kind='dive_spin',operator_kind=operator_kind or 'dive_here', mode='surface', map='local', dive=_dive(mission.get('dive')))
    if kind not in ('route', 'full'):
        raise ValueError('Unknown mission kind')
    points = route_points(mission.get('points'))
    return dict(kind='radio_route',operator_kind=operator_kind or ('dock_pipeline_dock' if kind=='full' else 'surface_route'), mode='surface', map='geographic', lat=points[0][0], lon=points[0][1],
                points=points, dive=_dive(mission.get('dive')) if kind == 'full' else None)


def _dive(value):
    if not isinstance(value, dict) or not set(value) <= {'depth_m', 'altitude_m', 'max_depth_m'}:
        raise ValueError('Dive: a pool depth, or a height above the floor and a max depth')
    return dive_target(**value)


class RadioRouteRun(DiveSpinRun):
    moving = DIVE_STAGES | {'ORIENT', 'TRANSIT'}

    def __init__(self, plan, run_id, now):
        points = route_points(plan['points'])
        full = plan.get('dive') is not None
        super().__init__(dict(plan, dive=plan['dive'] if full else dict(depth_m=1.)), run_id, now)
        self.plan = dict(plan, points=points, dive=self.dive if full else None)
        self.dive = self.plan['dive']   # None: the dive above never runs, it only fills the time limits
        self.points, self.full = points, full
        self.limits.update(WAIT_SURFACE=NAV_WAIT_S, ORIENT=ORIENT_LIMIT_S)
        self.legs = list(points)        # where it goes, in order; the full run adds the way back at Start
        self.leg = 0
        self.dived = False
        self.dock = None                # (lat, lon) at Start, the full run's last point
        self.orientation = None
        self.reset_revision = 0
        self.reason = 'Waiting for navigation'

    def act(self, now, o, local, yaw, late):
        bridge = o.get('bridge') or {}
        frame = self.frame(o)
        revision = (bridge.get('heading_reset') or {}).get('revision', 0)
        if revision != self.reset_revision:
            self.reset_revision, self.orientation = revision, None      # face the point again in the new frame
        if self.stage in ('ORIENT', 'TRANSIT') and bridge.get('fault'):
            return self.fail(now, o, 'Navigation fault: ' + str(bridge['fault']), NOTE_SAFETY)
        if self.stage == 'WAIT_SURFACE':
            health = bridge.get('estimator_health') or {}
            if (bridge.get('enabled') and bridge.get('configuration_verified') and health.get('usable')
                    and frame.angle is not None):
                self.enter('GUIDED', now, 'Taking GUIDED')
            elif late:
                return self.fail(now, o, 'Navigation is not running (press Start navigation first)', NOTE_NO_NAV)
            else:
                self.reason = 'Waiting for navigation (press Start navigation first)'
            return []
        if self.stage not in ('ORIENT', 'TRANSIT'):
            return super().act(now, o, local, yaw, late)
        if frame.angle is None:
            self.reason = 'Waiting for the EKF geographic position; holding'
            return self.command(now, 'guided_hold', (*local[:2], self.surface_down), interval=.1)
        target = frame.gps_to_local(*self.legs[self.leg])
        if self.stage == 'ORIENT':
            if self.orientation is None:
                self.orientation = GuidedOrientation((*local[:2], self.surface_down), target)
            if self.orientation.ready(yaw) or late:
                self.enter('TRANSIT', now, 'Going to point %d of %d' % (self.leg + 1, len(self.legs)))
            else:
                return self.command(now, 'guided_orient', self.orientation.target(), interval=.1)
        self.target = (*target, self.surface_down)
        near = math.dist(local[:2], target) <= ARRIVAL_M
        if self.settled(now, near and o['speed'] <= ARRIVED_SPEED):
            return self.arrived(now, o, local, yaw)
        if near:
            return self.command(now, 'guided_hold', (*local[:2], self.surface_down), interval=.1)
        return self.command(now, 'guided_target', self.target, interval=.1)

    @staticmethod
    def frame(o):
        """The bridge's lat/lon <-> Cube local map, as it is this tick."""
        return GeographicFrame.restore((o.get('bridge') or {}).get('map_transform'))

    def begin(self, now, o, local, yaw):
        """Armed in GUIDED at the dock: the way there, and for the full run the way back."""
        if self.full:
            self.dock = tuple(o['raw_gps']) if o.get('raw_gps') else self.frame(o).to_geo(local)
            self.legs = self.points + self.points[-2::-1] + [self.dock]
        return self.next_leg(now, 0)

    def next_leg(self, now, leg):
        self.leg, self.orientation = leg, None
        self.enter('ORIENT', now, 'Facing point %d of %d' % (leg + 1, len(self.legs)))
        return []

    def arrived(self, now, o, local, yaw):
        if self.full and not self.dived and self.leg == len(self.points) - 1:
            self.dived = True
            return self.start_dive(now, (*local[:2], self.surface_down), yaw)
        if self.leg + 1 < len(self.legs):
            return self.next_leg(now, self.leg + 1)
        self.enter('DISARM', now, 'Last point reached: disarming')
        return self.command(now, 'disarm')

    def surfaced(self, now, o, local, yaw):
        """Up from the pipeline dive: the way back."""
        return self.next_leg(now, self.leg + 1)

    def snapshot(self):
        s = super().snapshot()
        s.update(route_leg=0 if self.stage in ('WAIT_SURFACE', 'GUIDED', 'ARM') else self.leg + 1,
                 route_legs=len(self.legs), dock=self.dock, dived=self.dived)
        return s

