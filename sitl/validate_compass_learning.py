#!/usr/bin/env python3
"""Isolated native-firmware Compass Learn test; no ROS, MAVProxy or QGC forwarding.

SIMSTATE supplies independent simulated VN yaw. The learner's completion must be
an actual firmware message, never a generated test event. Simulation-only manual
controls exercise GPS/IMU motion; this script has no live-vehicle mode.
"""
import json
import argparse
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time

from robotx_graey_2026.api.pixhawk.mavlink import mavutil
from robotx_graey_2026.api.navigation.cube_north import CubeNorth, COMPASS_PARAMS
from robotx_graey_2026.api.navigation.odometry_profile import PROFILE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gps-glitch',action='store_true',help='Require rejection and restoration after injected GPS position jump')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    evidence=root/'sitl/logs'/('compass-learning-'+time.strftime('%Y%m%d-%H%M%S'))
    evidence.mkdir(parents=True)
    stage=Path(tempfile.mkdtemp(prefix='graey-compass-',dir='/tmp'))
    binary=Path(os.environ.get('SITL_BIN',str(root.parent/'output/gps-sitl-2026-10-02/ardusub')))
    shutil.copy2(binary,stage/'ardusub');(stage/'ardusub').chmod(0o700)
    # Instance 40: TCP 6160. No address or endpoint is accepted from user input.
    probe=socket.socket();probe.bind(('127.0.0.1',6160));probe.close()
    defaults=dict(PROFILE,COMPASS_ENABLE=1,COMPASS_USE=1,COMPASS_USE2=0,COMPASS_USE3=0,
                  COMPASS_LEARN=0,EK3_IMU_MASK=7,EK3_GSF_RUN_MASK=7,EK3_GSF_USE_MASK=3,EK2_ENABLE=0,
                  ARMING_CHECK=0,LOG_DISARMED=1,LOG_BACKEND_TYPE=1,LOG_BITMASK=65535,
                  SIM_GPS_DISABLE=0,SYSID_THISMAV=1)
    (stage/'params.parm').write_text('\n'.join(f'{k} {v}' for k,v in defaults.items()))
    log=(evidence/'firmware.txt').open('w')
    proc=subprocess.Popen([str(stage/'ardusub'),'-M','vectored','-I','40','-s','3',
                           '-O','32.92357,-117.03855,0,0','--defaults',str(stage/'params.parm')],
                          cwd=stage,stdout=log,stderr=subprocess.STDOUT)
    result=dict(passed=False,stage=str(stage),messages=[],transitions=[],qgc_forwarding=False,
                ros_started=False,endpoint='tcp:127.0.0.1:6160')
    link=None
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            try:
                link=mavutil.mavlink_connection('tcp:127.0.0.1:6160',source_system=255,source_component=190)
                break
            except OSError:time.sleep(.2)
        if link is None:raise RuntimeError('Simulator TCP did not open')
        hb=link.wait_heartbeat(timeout=15)
        if hb is None or hb.get_srcSystem()!=1:raise RuntimeError('No isolated simulator heartbeat')
        params={}; learner=CubeNorth();truth=None;gps=False;armed=False;mode=None;speed=None
        start=None;restored=False;last_poll=last_control=last_report=last_heartbeat=last_rates=0.;phase=None;boot_ms=0;cursor=0
        injected=False
        deadline=time.monotonic()+130
        while time.monotonic()<deadline:
            now=time.monotonic()
            if now-last_heartbeat>.1:
                link.mav.heartbeat_send(6,8,0,0,0)
                last_heartbeat=now
            if now-last_poll>.5:
                batch=COMPASS_PARAMS[cursor:cursor+12]
                for key in batch:link.mav.param_request_read_send(1,1,key.encode(),-1)
                cursor=0 if cursor+12>=len(COMPASS_PARAMS) else cursor+12
                last_poll=now
            if now-last_rates>5:
                for mid in (24,30,164):link.mav.command_long_send(1,1,511,0,mid,50000,0,0,0,0,0)
                last_rates=now
            m=link.recv_match(blocking=True,timeout=.03)
            if m is not None and (m.get_srcSystem(),m.get_srcComponent())==(1,1):
                kind=m.get_type()
                if kind=='PARAM_VALUE':
                    key=m.param_id.rstrip('\x00');params[key]=(now,m.param_value)
                    learner.parameter(now,key,m.param_value)
                elif kind=='HEARTBEAT':armed=bool(m.base_mode&128);mode=m.custom_mode
                elif kind=='SIMSTATE':truth=(now,math.degrees(m.yaw))
                elif kind=='GPS_RAW_INT':
                    gps=m.fix_type>=3;speed=m.vel/100.
                    learner.gps(now,dict(cube_fix_time_us=m.time_usec,fix=m.fix_type,
                        lat=m.lat/1e7,lon=m.lon/1e7,speed_mps=speed,
                        speed_accuracy_mps=getattr(m,'vel_acc',0)/1000.,accuracy=getattr(m,'h_acc',0)/1000.))
                elif kind=='STATUSTEXT':
                    result['messages'].append(str(m.text));learner.message(now,str(m.text))
                elif kind=='ATTITUDE':
                    boot_ms=m.time_boot_ms
                    good=bool(learner.gps_quality.capture_allowed(now) and truth and now-truth[0]<.5 and gps and abs(m.roll)<.52
                              and abs(m.pitch)<.52 and abs(m.yawspeed)<math.radians(10))
                    learner.observe(now,params,math.degrees(m.yaw),truth[1] if truth else 0.,good)
            if start is None and gps and all(k in params and now-params[k][0]<3 for k in COMPASS_PARAMS):
                changes=learner.start(now,params);result['original_profile']=learner.original_profile
                for key,value in changes.items():link.mav.param_set_send(1,1,key.encode(),float(value),9)
                link.mav.set_mode_send(1,1,19)
                start=now;result['start_boot_ms']=boot_ms
            if now-last_report>10:
                print(json.dumps(dict(gps=gps,speed_mps=speed,armed=armed,mode=mode,boot_ms=boot_ms,
                    missing=[k for k in COMPASS_PARAMS if k not in params],
                    stale={k:round(now-params[k][0],1) for k in COMPASS_PARAMS if k in params and now-params[k][0]>=3},
                    status=learner.status(now))),flush=True)
                last_report=now
            if start is None:continue
            if args.gps_glitch and not injected and armed and now-start>5.:
                link.mav.param_set_send(1,1,b'SIM_GPS_GLITCH_X',50.,9)
                injected=True;result['gps_glitch_injected']=True
            learner.check_gps(now)
            if learner.fault:
                if not args.gps_glitch:raise RuntimeError(learner.fault)
                if learner.phase in ('learning','captured'):
                    for key,value in learner.cancel(now,params).items():
                        link.mav.param_set_send(1,1,key.encode(),float(value),9)
            if args.gps_glitch and learner.phase=='cancelled':
                assert injected and learner.fault and learner.reference is None
                assert all(params[k][1]==v for k,v in PROFILE.items())
                result.update(passed=True,rejected_reason=learner.fault,
                              restored_profile={k:params[k][1] for k in PROFILE})
                restored=True;break
            if learner.phase!=phase:
                phase=learner.phase;result['transitions'].append(learner.status(now))
                print(json.dumps(result['transitions'][-1]),flush=True)
            if mode==19 and not armed and learner.phase=='learning':
                link.mav.command_long_send(1,1,400,0,1,0,0,0,0,0,0)
            if now-last_control>.05:
                # Accelerations and gentle turns excite native GPS/IMU yaw estimation.
                elapsed=(now-start)*3
                moving=armed and not learner.fault and not learner.finished and learner.phase=='learning'
                forward=(850 if int(elapsed/8)%3 else 200) if moving else 0
                turn=(400 if int(elapsed/12)%2 else -400) if moving and elapsed>12 else 0
                link.mav.manual_control_send(1,forward,0,500,turn,0)
                last_control=now
            if learner.phase=='captured':
                reference,changes=learner.capture(now,params);result['reference']=reference
                for key,value in changes.items():link.mav.param_set_send(1,1,key.encode(),float(value),9)
            if learner.phase=='ready':
                if args.gps_glitch:raise RuntimeError('Contaminated attempt unexpectedly accepted north')
                assert all(params[k][1]==v for k,v in PROFILE.items())
                result.update(passed=True,firmware_completion=learner.finished is not None,
                              restored_profile={k:params[k][1] for k in PROFILE},
                              elapsed_wall_s=now-start,elapsed_sim_s=(boot_ms-result['start_boot_ms'])/1000)
                restored=True;break
        if not restored:
            result.update(parameters={k:v[1] for k,v in params.items()},gps_fix=gps,armed=armed,mode=mode)
            raise RuntimeError('Native learner did not complete: '+json.dumps(learner.status(time.monotonic())))
    except Exception as error:
        result['error']=str(error)
    finally:
        if link:
            link.mav.manual_control_send(1,0,0,500,0,0)
            link.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0)
            link.close()
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()
        if (stage/'logs').exists():shutil.copytree(stage/'logs',evidence/'native-logs',dirs_exist_ok=True)
        (evidence/'result.json').write_text(json.dumps(result,indent=2))
        print('Evidence: '+str(evidence),flush=True)
    if not result['passed']:raise SystemExit(result.get('error','Validation failed'))


if __name__=='__main__':main()
