"""Explicit maintenance-only compass learning; no propulsion commands."""
import math
from collections import deque
from .bridge_estimator import wrap

YAW_PARAMS=tuple(f'EK3_SRC{i}_YAW' for i in (1,2,3))
COMPASS_PARAMS=('COMPASS_ENABLE','COMPASS_USE','COMPASS_DEV_ID','COMPASS_LEARN')+YAW_PARAMS


class CubeNorth:
    def __init__(self):
        self.started=None
        self.finished=None
        self.phase='idle'
        self.reason='Compass learning not started'
        self.rows=deque(maxlen=400)
        self.reference=None
        self.vn_invalidated=False
        self.active_seen=False

    def invalidate_vn(self):
        if self.phase=='ready':
            self.phase='expired';self.reference=None
            self.reason='VN reference expired; establish an independent north reference again'
        if self.phase in ('learning','restoring'):
            self.vn_invalidated=True;self.rows.clear();self.finished=None;self.reference=None
            self.reason='VN continuity/calibration changed; Cancel and repeat compass reference workflow'

    @staticmethod
    def values(now,params):
        if not all(k in params and 0<=now-params[k][0]<3 for k in COMPASS_PARAMS):
            raise ValueError('Fresh Cube compass and yaw parameter readbacks required')
        return {k:params[k][1] for k in COMPASS_PARAMS}

    def start(self,now,params):
        p=self.values(now,params)
        if self.phase not in ('idle','ready','cancelled','expired'):
            raise ValueError('Finish or cancel the current compass learning session first')
        if p['COMPASS_ENABLE']!=1 or p['COMPASS_USE']!=1 or p['COMPASS_DEV_ID']<=0:
            raise ValueError('First identify, enable and verify the selected Cube compass and mounting; acquisition may require a reboot. No compass enabled automatically.')
        if p['COMPASS_LEARN']!=0 or any(p[k]!=6 for k in YAW_PARAMS):
            raise ValueError('Start requires learning off and all yaw sources ExternalNav (6)')
        self.started=now;self.finished=None;self.rows.clear();self.reference=None;self.vn_invalidated=False;self.active_seen=False
        self.phase='learning';self.reason='Wait for compass-yaw readback; manually drive/turn in MANUAL only. No automatic propulsion.'
        return dict({k:1 for k in YAW_PARAMS},COMPASS_LEARN=3)

    def message(self,now,text):
        if self.phase=='learning' and self.active_seen and self.started is not None and now>=self.started and 'compasslearn: finished' in text.lower():
            self.finished=now

    def observe(self,now,params,cube_heading,vn_heading,good):
        try:p=self.values(now,params)
        except ValueError:self.rows.clear();return
        if self.phase=='restoring':
            if p['COMPASS_LEARN']==0 and all(p[k]==6 for k in YAW_PARAMS):
                self.phase='ready' if self.reference else 'cancelled'
                self.reason='Cube north transferred to VN; ExternalNav yaw restored. Run floating-surface Prepare and health check.' if self.reference else 'Learning cancelled; ExternalNav yaw restored. North capture still required.'
            return
        if self.phase!='learning':return
        if self.vn_invalidated:return
        if now-self.started>600:
            self.rows.clear();self.reason='Compass learning timed out; kill/disarm and Cancel to restore ExternalNav. No north accepted.';return
        independent=(p['COMPASS_ENABLE']==1 and p['COMPASS_USE']==1 and all(p[k]==1 for k in YAW_PARAMS))
        if independent and p['COMPASS_LEARN']==3:self.active_seen=True
        valid=(self.finished is not None and p['COMPASS_LEARN']==0 and independent and good
               and all(isinstance(v,(int,float)) and math.isfinite(v) for v in (cube_heading,vn_heading)))
        if not valid:
            self.rows.clear()
            self.reason=('Waiting for confirmed compass-based Cube yaw, firmware CompassLearn: Finished, learning readback 0 and fresh compass/GPS/paired attitudes')
            return
        if self.rows and now-self.rows[-1][0]>.25:self.rows.clear()
        self.rows.append((now,wrap(cube_heading-vn_heading)))
        while len(self.rows)>1 and now-self.rows[0][0]>3:self.rows.popleft()
        self.reason='Learning finished. Hold gently steady; kill/disarm/ESC-off, then Capture Cube north.'

    def capture(self,now,params):
        p=self.values(now,params)
        if (self.phase!='learning' or self.finished is None or p['COMPASS_LEARN']!=0
                or any(p[k]!=1 for k in YAW_PARAMS) or len(self.rows)<20
                or self.rows[-1][0]-self.rows[0][0]<2 or now-self.rows[-1][0]>.2):
            raise ValueError('Need completed compass learning and two seconds of fresh independent Cube/VN agreement')
        mean=math.degrees(math.atan2(sum(math.sin(math.radians(r[1])) for r in self.rows),sum(math.cos(math.radians(r[1])) for r in self.rows)))
        if any(abs(wrap(r[1]-mean))>5 for r in self.rows):
            raise ValueError('Cube/VN reference disagreement changes by more than 5 degrees; no north transferred')
        self.reference=dict(offset_deg=wrap(mean),captured_monotonic=now,method='Cube compass north after confirmed Compass Learn')
        self.phase='restoring';self.reason='Restoring ExternalNav yaw; no mission permission until readback'
        return self.reference,dict({k:6 for k in YAW_PARAMS},COMPASS_LEARN=0)

    def cancel(self,now=None,params=None):
        if self.phase not in ('learning','restoring'):
            p=self.values(now,params) if now is not None and params is not None else {}
            if not (p and p['COMPASS_LEARN'] in (0,3) and all(p[k] in (1,6) for k in YAW_PARAMS)
                    and (p['COMPASS_LEARN']==3 or any(p[k]==1 for k in YAW_PARAMS))):
                raise ValueError('No active or interrupted compass learning profile to restore')
        self.reference=None;self.phase='restoring';self.reason='Cancelling learning and restoring ExternalNav yaw'
        return dict({k:6 for k in YAW_PARAMS},COMPASS_LEARN=0)

    def status(self):
        return dict(phase=self.phase,reason=self.reason,finished_seen=self.finished is not None,
                    active_readback_seen=self.active_seen,paired_samples=len(self.rows),reference=self.reference)
