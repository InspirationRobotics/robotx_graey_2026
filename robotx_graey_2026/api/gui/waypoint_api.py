"""Reviewed, single-use waypoint plans. No HTTP request directly drives motors."""
import math
import time
import uuid
from robotx_graey_2026.api.navigation.waypoint_logic import validate_plan, geo_to_ne


class WaypointAPI:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.plans = {}

    def baseline(self, nav):
        streams = nav['streams']
        def fresh(name, limit=.7):
            item = streams.get(name, {})
            if item.get('age', math.inf) > limit:
                raise ValueError(name+' telemetry unavailable or stale')
            return item['data']
        hb = fresh('heartbeat', 2.)
        cube = nav.get('cube_estimate', {})
        s = fresh('supervisor')
        bridge=streams.get('bridge',{})
        relative=bridge.get('age',math.inf)<.5 and bridge.get('data',{}).get('coordinate_frame')=='fixed_local'
        if hb.get('armed') and hb.get('mode_id') not in (0, 19):
            raise ValueError('Armed start requires MANUAL or STABILIZE pilot mode')
        if not (cube.get('local_valid') if relative else cube.get('valid')) or (not relative and cube.get('heading_deg') is None):
            raise ValueError('Healthy Cube location and heading required')
        if not s.get('configuration_verified') or s.get('mode') != 'Active':
            raise ValueError('Supervisor/source profiles must be commissioned and active')
        if s.get('surfaced') is not True or s.get('state') == 'Fault':
            raise ValueError('Calibrated surface state without a supervisor fault required')
        local = fresh('local')['ned']
        origin = streams.get('cube_origin', {})
        if origin.get('age', math.inf) > 10:
            raise ValueError('Cube origin unavailable')
        o = origin['data']
        origin = (o['lat'], o['lon'], o['alt'])
        if relative:
            b=bridge['data'];fix=b.get('gps_observation') or {}
            if not (b.get('local_navigation_ready') and (b.get('gps_admission') or {}).get('qualified')):
                raise ValueError('Prepared local navigation and qualified GPS required')
            return dict(local=local,origin=origin,session=nav['session'],relative=True,start_gps=(fix['lat'],fix['lon']))
        if math.dist(geo_to_ne(cube['lat'], cube['lon'], origin), local[:2]) > .75:
            raise ValueError('Cube global and local coordinates disagree')
        return dict(local=local, origin=origin, session=nav['session'])

    def preview(self, plan, nav):
        plan = validate_plan(plan)
        base = self.baseline(nav)
        if base.get('relative') and plan.get('alignment_corridor_confirmed') is not True:
            raise ValueError('Confirm the initial 5 m movement corridor and stopping clearance')
        target = geo_to_ne(plan['lat'], plan['lon'], base.get('start_gps',base['origin']))
        distance = math.hypot(*target) if base.get('relative') else math.dist(target, base['local'][:2])
        if not .3 <= distance <= 20:
            raise ValueError('Waypoint must be 0.3 to 20 m away')
        now = self.clock()
        self.plans = {k:v for k,v in self.plans.items() if now-v['created'] < 300}
        key = uuid.uuid4().hex
        self.plans[key] = dict(plan=plan, base=base, created=now, run_id=None)
        return dict(ok=True, plan_id=key, plan=plan, distance_m=distance, expires_s=30)

    def consume(self, key, nav):
        item = self.plans.get(key)
        if not item:
            raise ValueError('Preview the selected point first')
        if item['run_id']:
            return item, True
        if self.clock()-item['created'] > 30:
            raise ValueError('Preview expired; select the point again')
        base = self.baseline(nav)
        if (base['origin'][:2] != item['base']['origin'][:2] or base['session'] != item['base']['session']
                or math.dist(base['local'], item['base']['local']) > .5):
            raise ValueError('Vehicle/frame moved since preview; select the point again')
        return item, False
