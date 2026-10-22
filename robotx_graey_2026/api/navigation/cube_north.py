"""Explicit maintenance-only compass learning; no propulsion commands."""
import math
from collections import deque
from .bridge_estimator import wrap
from .odometry_profile import PROFILE
from .compass_gps_quality import CompassGPSQuality

YAW_PARAMS=tuple(f'EK3_SRC{i}_YAW' for i in (1,2,3))
CALIBRATION_PROFILE=dict(PROFILE)
for slot in (1,2,3):
    CALIBRATION_PROFILE.update({f'EK3_SRC{slot}_POSXY':3,f'EK3_SRC{slot}_VELXY':3,
                                f'EK3_SRC{slot}_POSZ':1,f'EK3_SRC{slot}_YAW':1})
CALIBRATION_CHANGES={k:v for k,v in CALIBRATION_PROFILE.items() if PROFILE[k]!=v}
RESTORE_PROFILE=dict(PROFILE,COMPASS_LEARN=0)
COMPASS_PARAMS=tuple(dict.fromkeys((*PROFILE,'COMPASS_ENABLE','COMPASS_USE','COMPASS_DEV_ID','COMPASS_LEARN','EK3_GSF_RUN_MASK','EK3_GSF_USE_MASK','EK3_IMU_MASK')))


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
        self.completion_message_at=None
        self.stage='idle'
        self.learning_complete=False
        self.profile_mismatches={}
        self.original_profile=None
        self.fault=''
        self.gps_quality=CompassGPSQuality()
        self.lane=None

    def gps(self,now,fix):
        if self.phase not in ('learning','captured'):return
        self.gps_quality.observe(now,fix)
        self.check_gps(now)

    def check_gps(self,now):
        if self.phase not in ('learning','captured'):return
        self.gps_quality.check(now)
        if self.gps_quality.fault:
            self.fault=self.fault or 'Learning rejected: '+self.gps_quality.fault
            self.reference=None;self.rows.clear();self.finished=None
            self.learning_complete=False;self.stage='gps_rejected';self.reason=self.fault

    def invalidate_vn(self):
        if self.phase=='ready':
            self.phase='expired';self.reference=None
            self.stage='expired'
            self.reason='VN reference expired; establish an independent north reference again'
        if self.phase in ('learning','captured','restoring'):
            self.vn_invalidated=True;self.rows.clear();self.finished=None;self.reference=None;self.completion_message_at=None
            self.stage='reference_invalid'
            self.learning_complete=False
            self.reason='VN continuity/calibration changed; Cancel and repeat compass reference workflow'

    @staticmethod
    def values(now,params):
        if not all(k in params and 0<=now-params[k][0]<3
                   and isinstance(params[k][1],(int,float)) and math.isfinite(params[k][1])
                   for k in COMPASS_PARAMS):
            raise ValueError('Fresh Cube compass and yaw parameter readbacks required')
        return {k:params[k][1] for k in COMPASS_PARAMS}

    def start(self,now,params):
        p=self.values(now,params)
        if self.phase not in ('idle','ready','cancelled','expired'):
            raise ValueError('Finish or cancel the current compass learning session first')
        if p['COMPASS_ENABLE']!=1 or p['COMPASS_USE']!=1 or p['COMPASS_DEV_ID']<=0:
            raise ValueError('First identify, enable and verify the selected Cube compass and mounting; acquisition may require a reboot. No compass enabled automatically.')
        if p['COMPASS_LEARN']!=0 or any(p[k]!=v for k,v in PROFILE.items()):
            raise ValueError('Start requires learning off and the complete verified bridge-only profile')
        imu_mask=int(p['EK3_IMU_MASK'])
        if not 0<imu_mask<=7 or p['EK3_IMU_MASK']!=imu_mask:
            raise ValueError('Verified Cube IMU lane configuration required before learning')
        required=(1 << imu_mask.bit_count())-1
        if int(p['EK3_GSF_RUN_MASK']) & required != required:
            raise ValueError(f'GSF learning must cover all active lanes: EK3_GSF_RUN_MASK needs {required}; configure and reboot Cube before learning')
        self.original_profile={k:p[k] for k in PROFILE}
        self.fault='';self.profile_mismatches={}
        self.gps_quality.begin(now)
        self.started=now;self.finished=None;self.rows.clear();self.reference=None;self.vn_invalidated=False;self.active_seen=False
        self.completion_message_at=None;self.stage='confirming_commands';self.learning_complete=False
        self.phase='learning';self.reason='Configuring temporary native GPS/barometer/compass profile for MANUAL learning; wait for readbacks. No propulsion command.'
        return dict(CALIBRATION_CHANGES,COMPASS_LEARN=3)

    def message(self,now,text):
        if 'ekf3 lane switch ' in text.lower():
            try:self.lane=int(text.lower().split('ekf3 lane switch ',1)[1].split()[0])
            except (ValueError,IndexError):pass
        if (self.phase=='learning' and not self.fault and not self.vn_invalidated and self.started is not None
                and now>=self.started and 'compasslearn: finished' in text.lower()):
            # STATUSTEXT and PARAM_VALUE can arrive in either order. Retain the
            # event, but do not accept completion without this session's mode-3 ACK.
            if self.completion_message_at is None:self.completion_message_at=now
            if self.active_seen:self.finished=self.completion_message_at

    def parameter(self,now,key,value):
        if (self.phase=='learning' and not self.fault and not self.vn_invalidated and self.started is not None
                and now>=self.started and key=='COMPASS_LEARN' and value==3):
            self.active_seen=True
            if self.completion_message_at is not None:self.finished=self.completion_message_at

    def observe(self,now,params,cube_heading,vn_heading,good):
        if self.phase not in ('learning','restoring'):return
        try:p=self.values(now,params)
        except ValueError:
            self.rows.clear();self.stage='waiting_readbacks'
            self.reason='Waiting for fresh compass and yaw parameter readbacks'
            return
        if self.phase=='restoring':
            self.profile_mismatches={k:dict(expected=v,actual=p[k]) for k,v in RESTORE_PROFILE.items() if p[k]!=v}
            if not self.profile_mismatches:
                self.phase='ready' if self.reference else 'cancelled'
                self.stage=self.phase
                self.reason='Cube north transferred to VN; ExternalNav yaw restored. Run floating-surface Prepare and health check.' if self.reference else 'Learning cancelled; ExternalNav yaw restored. North capture still required.'
            return
        if self.phase!='learning':return
        if self.fault:return
        imu_mask=int(p['EK3_IMU_MASK'])
        required=(1 << imu_mask.bit_count())-1 if 0<imu_mask<=7 else 0
        if not required or int(p['EK3_GSF_RUN_MASK']) & required != required:
            self.fault='GSF learning coverage lost for an active IMU lane'
            self.reference=None;self.rows.clear();self.stage='lane_coverage_lost';self.reason=self.fault
            return
        if self.vn_invalidated:return
        if now-self.started>600:
            self.rows.clear();self.stage='timed_out';self.reason='Learning has not completed within ten minutes. Cancel in MANUAL; no north accepted.';return
        self.profile_mismatches={k:dict(expected=v,actual=p[k]) for k,v in CALIBRATION_PROFILE.items() if p[k]!=v}
        independent=(p['COMPASS_ENABLE']==1 and p['COMPASS_USE']==1 and not self.profile_mismatches)
        if independent and p['COMPASS_LEARN']==3:self.active_seen=True
        if self.active_seen and self.completion_message_at is not None:self.finished=self.completion_message_at
        self.learning_complete=bool(self.finished is not None and p['COMPASS_LEARN']==0
                                    and params['COMPASS_LEARN'][0]>=self.finished)
        valid=(self.finished is not None and p['COMPASS_LEARN']==0
               and params['COMPASS_LEARN'][0]>=self.finished and independent and good
               and all(isinstance(v,(int,float)) and math.isfinite(v) for v in (cube_heading,vn_heading)))
        if not valid:
            self.rows.clear()
            if self.profile_mismatches:
                self.stage='configuring_profile';self.reason='Waiting for temporary native GPS/barometer/compass profile readbacks: '+', '.join(self.profile_mismatches)
            elif not self.active_seen:
                self.stage='confirming_commands';self.reason='Waiting for Cube to acknowledge learning mode 3'
            elif self.finished is None:
                self.stage='learning';self.reason='Learning is still running. Drive and turn in armed MANUAL with good GPS; no completion event received.'
            elif p['COMPASS_LEARN']!=0:
                self.stage='confirming_completion';self.reason='Completion event received; waiting for Cube learning-mode readback to become 0'
            elif not independent:
                self.stage='confirming_compass_yaw';self.reason='Waiting for enabled compass and compass-yaw source readbacks'
            else:
                self.stage='waiting_paired_attitudes';self.reason='Firmware learning complete; waiting for fresh healthy compass/GPS and level, steady paired Cube/VN attitudes'
            return
        if self.rows and now-self.rows[-1][0]>.25:self.rows.clear()
        self.rows.append((now,wrap(cube_heading-vn_heading)))
        while len(self.rows)>1 and now-self.rows[0][0]>3:self.rows.popleft()
        self.stage='capturing_north';self.reason='Firmware learning complete; checking two seconds of stable Cube/VN north agreement'
        try:
            self.reference=self.candidate(now,params)
        except ValueError as error:
            self.reason=str(error)
            return
        self.phase='captured'
        self.stage='captured'
        self.reason='North captured automatically; restoring ExternalNav yaw in MANUAL. No arm or propulsion command is sent.'

    def candidate(self,now,params):
        if self.fault:raise ValueError(self.fault)
        p=self.values(now,params)
        if (self.phase!='learning' or self.finished is None or p['COMPASS_LEARN']!=0
                or params['COMPASS_LEARN'][0]<self.finished
                or any(p[k]!=v for k,v in CALIBRATION_PROFILE.items()) or len(self.rows)<20
                or self.rows[-1][0]-self.rows[0][0]<2 or now-self.rows[-1][0]>.2):
            raise ValueError('Need completed compass learning and two seconds of fresh independent Cube/VN agreement')
        mean=math.degrees(math.atan2(sum(math.sin(math.radians(r[1])) for r in self.rows),sum(math.cos(math.radians(r[1])) for r in self.rows)))
        if any(abs(wrap(r[1]-mean))>5 for r in self.rows):
            raise ValueError('Cube/VN reference disagreement changes by more than 5 degrees; no north transferred')
        return dict(offset_deg=wrap(mean),captured_monotonic=now,method='Cube compass north after confirmed Compass Learn')

    def capture(self,now,params):
        if self.fault:raise ValueError(self.fault)
        p=self.values(now,params)
        if self.phase=='captured':
            if self.reference is None or p['COMPASS_LEARN']!=0 or any(p[k]!=v for k,v in CALIBRATION_PROFILE.items()):
                raise ValueError('Captured north source changed; inspect compass reference before restoration')
        else:
            self.reference=self.candidate(now,params)
        self.phase='restoring';self.stage='restoring';self.reason='Restoring ExternalNav yaw; no mission permission until readback'
        return self.reference,dict(RESTORE_PROFILE)

    def cancel(self,now=None,params=None):
        if self.phase not in ('learning','captured','restoring'):
            p=self.values(now,params) if now is not None and params is not None else {}
            if not (p and p['COMPASS_LEARN'] in (0,3)
                    and all(p[k] in (PROFILE[k],CALIBRATION_PROFILE[k]) for k in PROFILE)
                    and (p['COMPASS_LEARN']==3 or any(p[k]!=v for k,v in PROFILE.items()))):
                raise ValueError('No active or interrupted compass learning profile to restore')
        # A completed, verified north capture is valuable even if the operator
        # clicks Cancel while its yaw-restoration readbacks are still arriving.
        # Never erase it merely to stop learning or finish restoration.
        keep_reference=self.phase in ('captured','restoring') and self.reference is not None and not self.vn_invalidated
        if not keep_reference:self.reference=None
        self.phase='restoring';self.stage='restoring'
        self.reason=('North reference retained; restoring ExternalNav yaw' if keep_reference
                     else 'Learning stopped without a north reference; restoring ExternalNav yaw')
        return dict(RESTORE_PROFILE)

    def status(self,now=None):
        return dict(phase=self.phase,stage=self.stage,reason=self.reason,finished_seen=self.finished is not None,
                    temporary_gps_profile=self.phase in ('learning','captured'),
                    profile_mismatches=self.profile_mismatches,original_profile=self.original_profile,fault=self.fault,
                    gps_quality=self.gps_quality.status(now),active_lane=self.lane,
                    completion_message_received=self.completion_message_at is not None,
                    learning_complete=self.learning_complete,
                    elapsed_s=max(0.,now-self.started) if now is not None and self.started is not None else 0.,
                    paired_span_s=self.rows[-1][0]-self.rows[0][0] if len(self.rows)>1 else 0.,
                    active_readback_seen=self.active_seen,paired_samples=len(self.rows),reference=self.reference)
