"""Fixed local 5 m / 2 m / 5 m / surface sequence, without GPS admission."""
import math
from .guided_surface_mission import GuidedSurfaceRun

class SimpleSequenceRun(GuidedSurfaceRun):
    moving = {'FORWARD_1', 'DESCEND', 'FORWARD_2', 'SURFACE', 'STOP'}

    def __init__(self, plan, run_id, now):
        super().__init__(dict(lat=0., lon=0., map='geographic', mode='surface'), run_id, now)
        self.plan = dict(kind='simple_sequence', mode='surface', map='local', forward_m=5., depth_m=2.)
        self.enter('GUIDED', now, 'Starting 5 m / 2 m / 5 m / surface sequence')
        self.targets = {}

    @property
    def phase(self):
        return 'UNDERWATER' if self.stage in ('DESCEND', 'FORWARD_2', 'SURFACE') else 'SURFACE_NAV'

    def step(self, now, o):
        self.last = now
        if self.stage == 'COMPLETE': return []
        if self.stage == 'ABORTED': return self.abort_actions(now, o)
        if o.get('abort') or o.get('failure'):
            self.abort(now, o.get('failure') or 'Operator aborted')
            return self.abort_actions(now, o)
        if self.stage in self.moving and (o.get('armed') is False or o.get('mode') not in (None, 4)):
            self.abort(now, 'Pilot/failsafe took control', pilot=True)
            return []
        # Actual local pose is needed to express metres; no GPS/Prepare/sensor-vote gate.
        if not o.get('fresh'):
            self.reason = 'Waiting for Cube control telemetry'
            return []
        local = tuple(o['local'])
        if self.stage == 'GUIDED':
            if o.get('mode') == 4: self.enter('ARM', now, 'Arming')
            else: return self.command(now, 'guided')
        elif self.stage == 'ARM':
            if o.get('mode') != 4:
                self.abort(now, 'Pilot changed mode', pilot=True); return []
            if not o.get('armed'): return self.command(now, 'arm')
            self.heading = o['yaw']; self.start = local; self.surface_down = local[2]
            dx, dy = 5*math.cos(self.heading), 5*math.sin(self.heading)
            x, y, z = local
            self.targets = {'FORWARD_1': (x+dx,y+dy,z), 'DESCEND': (x+dx,y+dy,z+2),
                            'FORWARD_2': (x+2*dx,y+2*dy,z+2), 'SURFACE': (x+2*dx,y+2*dy,z)}
            self.enter('FORWARD_1', now, 'Forward 5 m')
        elif self.stage in self.targets:
            self.target = self.targets[self.stage]
            if self.settled(now, math.dist(local, self.target) <= .25 and o['speed'] <= .15):
                stages = ['FORWARD_1','DESCEND','FORWARD_2','SURFACE','STOP']
                self.enter(stages[stages.index(self.stage)+1], now, 'Next sequence leg')
            return self.command(now, 'guided_orient', (*self.target,self.heading), interval=.1)
        elif self.stage == 'STOP':
            self.enter('DISARM', now, 'Surfaced; stopping and disarming')
            return self.command(now, 'guided_orient', (*self.targets['SURFACE'],self.heading), interval=.1)
        elif self.stage == 'DISARM':
            if o.get('armed'): return self.command(now, 'disarm')
            self.enter('COMPLETE', now, 'Surfaced, stopped and disarmed')
            return [('manual', None)]
        return []
