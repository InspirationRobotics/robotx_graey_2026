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
        b=fresh('bridge')
        if hb.get('armed') and hb.get('mode_id') not in (0,19):
            raise ValueError('Start in MANUAL or STABILIZE')
        if not b.get('enabled'):raise ValueError('Start navigation first')
        if not (b.get('estimator_health') or {}).get('usable'):
            raise ValueError('Navigation estimate initializing or all sensors lost')
        local=fresh('local')['ned']
        origin_item=streams.get('cube_origin',{}).get('data') or {}
        fix=(b.get('gps_admission') or {}).get('fix') or {}
        frame=b.get('map_transform') or {}
        anchor=frame.get('origin') or ([fix.get('lat'),fix.get('lon')] if fix.get('fix',0)>=3 else None)
        if not anchor or any(v is None or not math.isfinite(v) for v in anchor):
            raise ValueError('A geographic reference is needed to select a waypoint')
        origin=(origin_item.get('lat',anchor[0]),origin_item.get('lon',anchor[1]),origin_item.get('alt',0.))
        return dict(local=local,origin=origin,session=nav['session'],relative=True,start_gps=tuple(anchor))
    def preview(self, plan, nav):
        plan = validate_plan(plan)
        if plan['mode']!='surface':raise ValueError('Only surface missions are supported in navigation_new')
        base = self.baseline(nav)
        if base.get('relative') and plan.get('alignment_corridor_confirmed') is not True:
            raise ValueError('Confirm the initial 10 m movement corridor and stopping clearance')
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
        if (base['origin'][:2] != item['base']['origin'][:2] or base['session'] != item['base']['session']):
            raise ValueError('Vehicle/frame moved since preview; select the point again')
        return item, False
