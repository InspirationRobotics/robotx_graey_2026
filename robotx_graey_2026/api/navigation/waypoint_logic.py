"""Bounded single-waypoint mission. Pure decisions; transport belongs to adapter."""
import math

R = 6378137.0


def geo_to_ne(lat, lon, origin):
    if not all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
               for v in (lat, lon, *origin[:2])):
        raise ValueError('Finite geographic coordinates required')
    if not -85 <= lat <= 85 or not -180 <= lon <= 180 or not -85 <= origin[0] <= 85:
        raise ValueError('Geographic coordinates outside supported range')
    return (math.radians(lat-origin[0])*R,
            math.radians((lon-origin[1]+180)%360-180)*R*math.cos(math.radians(origin[0])))


def ne_to_geo(north, east, origin):
    return (origin[0]+math.degrees(north/R),
            (origin[1]+math.degrees(east/(R*math.cos(math.radians(origin[0]))))+180)%360-180)


def validate_plan(value):
    if not isinstance(value, dict):
        raise ValueError('Expected waypoint object')
    lat, lon, depth = (value.get(k) for k in ('lat', 'lon', 'depth_m'))
    geo_to_ne(lat, lon, (0., 0.))
    if isinstance(depth, bool) or not isinstance(depth, (int, float)) or not math.isfinite(depth) or not .3 <= depth <= 1.0:
        raise ValueError('Dive depth must be 0.3 to 1.0 m')
    if value.get('map') not in ('pool', 'geographic'):
        raise ValueError('Unknown map')
    return dict(lat=float(lat), lon=float(lon), depth_m=float(depth), map=value['map'])


class WaypointRun:
    moving = {'DIVE', 'TRANSIT', 'SURFACE'}
    terminal = {'COMPLETE', 'ABORTED'}

    def __init__(self, plan, run_id, now):
        self.plan = validate_plan(plan)
        self.run_id, self.stage, self.since = run_id, 'WAIT_SURFACE', now
        self.reason = 'Qualifying surfaced navigation'
        self.origin = self.reference_id = self.start = self.target = self.start_gps = None
        self.heading = None
        self.surface_down = None
        self.last = now
        self.arrival = None
        self.last_command = -math.inf
        self.last_position = None
        self.pilot_takeover = False
        self.takeover_requested = False
        self.started_armed = None
        self.navigation_wait_since = None
        self.navigation_hold = None
        self.navigation_wait_reason = ''
        self.navigation_wait_limit = .7
        self.abort_context = None
        self.vertical_offset_error = None

    @property
    def phase(self):
        if self.stage in ('WAIT_SURFACE', 'RECOVER_GPS', 'COMPLETE'):
            return 'SURFACE'
        return 'DIVE' if self.stage in ('PREPARE', 'GUIDED', 'ARM', 'DIVE') else 'UNDERWATER'

    def enter(self, stage, now, reason):
        self.stage, self.since, self.reason = stage, now, reason
        self.arrival, self.last_command = None, -math.inf

    def abort(self, now, reason, pilot=False):
        self.pilot_takeover = pilot
        self.takeover_requested = self.stage in self.moving | {'GUIDED', 'ARM', 'DISARM'}
        self.enter('ABORTED', now, reason)

    def navigation_wait(self, now, o, reason, dvl=False):
        """Pause travel only with independent healthy pose/bridge evidence.

        This is not stale-sensor extrapolation. Capture one current Cube position
        and stop moving the target; never chase a changing last-known position.
        Missing adapter evidence is fail-closed. A valid frame is mandatory.
        """
        s=o.get('supervisor') or {}
        if (self.stage not in self.moving or o.get('navigation_hold_safe') is not True
                or s.get('fusion_policy')!='continuous'
                or s.get('run_id')!=self.run_id
                or s.get('reference_current') is not True
                or (s.get('reference') or {}).get('reference_id')!=self.reference_id):
            return None
        if self.last_position is not None:
            t,p=self.last_position
            if math.dist(o['local'],p) > .5+2.*max(0.,now-t):
                return None
        if self.navigation_wait_since is None:
            self.navigation_wait_since=now
            self.navigation_hold=tuple(o['local'])
            self.navigation_wait_limit=1.5 if dvl else .7
        if now-self.navigation_wait_since >= self.navigation_wait_limit:
            return None
        self.navigation_wait_reason=reason
        # Last mission setpoint may still command forward travel. Replace it
        # immediately once, then refresh at the ordinary position-target rate.
        if self.last_command != -math.inf and now-self.navigation_wait_since < .001:
            self.last_command=-math.inf
        return self.command(now,'target',(*self.navigation_hold,self.heading),interval=3.)

    def abort_actions(self, now, o):
        if self.takeover_requested and not self.pilot_takeover and o.get('mode') in (None, 4):
            return self.command(now, 'manual')
        return []

    def command(self, now, action, payload=None, interval=1.):
        if now-self.last_command < interval:
            return []
        self.last_command = now
        return [(action, payload)]

    def settled(self, now, condition):
        self.arrival = (now if self.arrival is None else self.arrival) if condition else None
        return self.arrival is not None and now-self.arrival >= 1.

    def step(self, now, o):
        gap, self.last = now-self.last, now
        if self.stage == 'COMPLETE':
            return []
        if self.stage == 'ABORTED':
            return self.abort_actions(now, o)
        s = o.get('supervisor') or {}
        if self.started_armed is None and o.get('fresh') and isinstance(o.get('armed'), bool):
            self.started_armed = o['armed']
        if self.started_armed and self.stage in ('WAIT_SURFACE', 'PREPARE', 'GUIDED', 'ARM') and o.get('armed') is False:
            self.abort(now, 'Pilot disarmed during mission preparation', pilot=True)
            return []
        if self.stage in ('WAIT_SURFACE', 'PREPARE') and o.get('armed') and o.get('mode') not in (0, 19):
            self.abort(now, 'Armed preparation requires MANUAL or STABILIZE', pilot=True)
            return []
        moving = self.stage in self.moving
        failure = o.get('failure') or ''
        if moving and not 0 <= gap < 1.:
            failure = 'Mission callback missed freshness deadline'
        if o.get('abort'):
            failure = 'Operator aborted; pilot takeover requested'
        if (moving and o.get('armed') is False) or (
                self.stage in self.moving | {'ARM', 'DISARM'} and o.get('mode') not in (None, 4)):
            # A heartbeat reveals the outcome, not whether a pilot, firmware
            # failsafe or another MAVLink sender caused it. Always yield.
            self.abort(now, 'Vehicle left GUIDED or disarmed; yielding control', pilot=True)
            return []
        if now-self.since > (10. if self.stage in ('GUIDED', 'ARM', 'DISARM') else 90.):
            failure = 'Mission stage timed out: '+self.stage
        # Geographic altitude metadata is independent of calibrated pressure
        # depth and the frozen local-down offset. Only NE origin defines targets.
        if self.origin is not None and tuple(o.get('origin') or ())[:2] != self.origin[:2]:
            failure = 'Cube origin unavailable or changed'
        # A fixed Cube/pressure zero difference is supported, but a changing
        # difference invalidates the frozen vertical target. Check before the
        # temporary navigation-permission hold, which must never mask this.
        if self.surface_down is not None and (moving or self.stage in ('GUIDED', 'ARM')) and o.get('fresh'):
            local, depth = o.get('local'), o.get('depth')
            if (local and len(local)==3 and isinstance(depth,(int,float))
                    and math.isfinite(depth) and math.isfinite(local[2])):
                self.vertical_offset_error = local[2]-depth-self.surface_down
                if depth > self.plan['depth_m']+.15:
                    failure = 'Pressure depth exceeded mission target by more than 0.15 m; pilot takeover'
                elif abs(self.vertical_offset_error) > .15:
                    failure = 'Cube/pressure vertical reference changed by more than 0.15 m; pilot takeover'
        # Stop advancing the target on the FIRST recoverable DVL gap, while
        # bounded odometry is still healthy. Never wait for health to expire.
        dvl=o.get('dvl_continuity') or {}
        if moving and dvl.get('recovering'):
            if not failure and s.get('state')!='Fault' and o.get('fresh'):
                hold=self.navigation_wait(now,o,'DVL reacquiring; horizontal target held',dvl=True)
                if hold is not None:return hold
            failure=failure or 'DVL recovery exceeded bounded healthy navigation; pilot takeover'
        if (moving or self.stage in ('GUIDED', 'ARM')) and (not s.get('navigation_ready') or not s.get('reference_current')
                       or not (s.get('fusion_policy') == 'continuous')
                       or (s.get('reference') or {}).get('reference_id') != self.reference_id):
            if not failure and s.get('state')!='Fault':
                hold=self.navigation_wait(now,o,'Navigation permission temporarily unavailable: '+str(s.get('reason')))
                if hold is not None:return hold
            failure = failure or 'Underwater navigation permission or reference lost: '+str(s.get('reason'))
        if s.get('state') == 'Fault':
            failure = 'Supervisor fault: '+str(s.get('reason'))
        if failure:
            self.abort(now, failure)
            return self.abort_actions(now, o)
        # Every stage uses fresh status for this run, including pre-arm waits.
        if not o.get('fresh') or s.get('run_id') != self.run_id:
            if moving or self.stage in ('GUIDED', 'ARM', 'DISARM'):
                hold=self.navigation_wait(now,o,'Waiting for fresh supervisor status for this mission')
                if hold is not None:return hold
                self.abort(now, 'Fresh navigation telemetry for this run unavailable')
                return self.abort_actions(now, o)
            return []
        if self.navigation_wait_since is not None:
            # Resume the original stage target immediately after validation.
            self.last_command=-math.inf
            self.arrival=None
        self.navigation_wait_since=self.navigation_hold=None
        self.navigation_wait_reason=''
        self.navigation_wait_limit=.7
        if self.stage == 'WAIT_SURFACE':
            measurement=o.get('surface_measurement') or {}
            if not measurement.get('ready'):
                self.reason=measurement.get('reason','Waiting for surface GPS/DVL measurements')
                return []
            if s.get('surfaced') is True and (s.get('fusion_policy') == 'continuous' and s.get('gps_qualified_input') is True) and s.get('navigation_ready'):
                self.enter('PREPARE', now, 'Capture surface start and validate continuous navigation')
        elif self.stage == 'PREPARE':
            ref = s.get('reference') or {}
            if s.get('navigation_ready') and s.get('reference_current') and (s.get('fusion_policy') == 'continuous'):
                try:
                    origin = tuple(o['origin'])
                    target = geo_to_ne(self.plan['lat'], self.plan['lon'], origin)
                    start = tuple(o['local'])
                    global_ne = geo_to_ne(*o['global'], origin)
                    if math.dist(global_ne, start[:2]) > .75:
                        raise ValueError('Cube global/local frames disagree')
                    if not .3 <= math.dist(start[:2], target) <= 20.:
                        raise ValueError('Waypoint must be 0.3 to 20 m from start')
                    # Calibrated pressure depth and Cube NED down need not have
                    # the same zero. Freeze their offset; never reset the EKF.
                    surface_down = start[2]-o['depth']
                    heading = float(o['yaw'])
                    target = (*target, surface_down+self.plan['depth_m'])
                    if not all(math.isfinite(v) for v in (*start, *target, heading)):
                        raise ValueError('Non-finite mission reference')
                    self.origin, self.reference_id = origin, ref['reference_id']
                    self.start, self.target = start, target
                    self.surface_down, self.heading = surface_down, heading
                    self.start_gps = tuple(o['global'])
                    self.last_position = (now, start)
                    self.enter('GUIDED', now, 'Waiting for GUIDED mode confirmation')
                except (KeyError, ValueError, TypeError) as error:
                    self.abort(now, str(error))
        elif self.stage in ('GUIDED', 'ARM'):
            if not s.get('navigation_ready') or not s.get('reference_current'):
                self.abort(now, 'Navigation permission lost before arming')
                return self.command(now, 'manual') if o.get('mode') == 4 else []
            if self.stage == 'GUIDED':
                if o.get('mode') == 4:
                    self.enter('ARM', now, 'Waiting for arm confirmation')
                else:
                    return self.command(now, 'guided')
            elif o.get('armed'):
                self.enter('DIVE', now, 'Descending at the surface start')
            elif o.get('mode') != 4:
                self.abort(now, 'GUIDED mode lost before arming', pilot=True)
            else:
                return self.command(now, 'arm')
        elif moving:
            local = o['local']
            if self.last_position:
                t, p = self.last_position
                if math.dist(local, p) > .5+2.*(now-t):
                    self.abort(now, 'Cube position discontinuity')
                    return self.command(now, 'manual')
            self.last_position = (now, tuple(local))
            target = (self.start[0], self.start[1], self.target[2]) if self.stage == 'DIVE' else (
                (*self.target[:2], self.surface_down) if self.stage == 'SURFACE' else self.target)
            depth = o['depth']
            reached = (math.dist(local[:2], target[:2]) <= .35 and o['speed'] <= .15
                       and (abs(depth-self.plan['depth_m']) <= .08 if self.stage != 'SURFACE' else -.1 <= depth <= .12))
            if self.settled(now, reached):
                next_stage = {'DIVE': 'TRANSIT', 'TRANSIT': 'SURFACE', 'SURFACE': 'DISARM'}[self.stage]
                self.enter(next_stage, now, {'TRANSIT': 'Traveling underwater to waypoint',
                    'SURFACE': 'Ascending at waypoint', 'DISARM': 'Surface reached; waiting for disarm'}[next_stage])
                return []
            return self.command(now, 'target', (*target, self.heading), interval=3.)
        elif self.stage == 'DISARM':
            if o.get('armed'):
                return self.command(now, 'disarm')
            self.enter('RECOVER_GPS', now, 'Disarmed; verifying surface GPS recovery')
            return [('manual', None)]
        elif self.stage == 'RECOVER_GPS':
            if o.get('armed'):
                self.abort(now, 'Vehicle rearmed during GPS recovery', pilot=True)
            elif s.get('surfaced') is True and (s.get('fusion_policy') == 'continuous' and s.get('gps_qualified_input') is True) and s.get('navigation_ready'):
                self.enter('COMPLETE', now, 'Surfaced at waypoint and disarmed; GPS recovered')
        return []

    def snapshot(self):
        return dict(run_id=self.run_id, stage=self.stage, reason=self.reason, plan=self.plan,
                    surface_start=self.start_gps, start_cube=self.start, target_cube=self.target,
                    surface_down=self.surface_down,
                    vertical_offset_error_m=self.vertical_offset_error,
                    navigation_pause=dict(active=self.navigation_wait_since is not None,
                        reason=self.navigation_wait_reason,hold_ned=self.navigation_hold),
                    abort_context=self.abort_context,
                    heading_rad=self.heading, complete=self.stage == 'COMPLETE')
