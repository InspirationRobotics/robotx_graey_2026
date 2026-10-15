#!/usr/bin/env python3
"""Exercise the actual GUI API and production waypoint node against isolated SITL."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import urllib.request
import uuid


def api(body=None):
    req = urllib.request.Request('http://127.0.0.1:8090/api/waypoint',
        data=json.dumps(body).encode() if body else None,
        headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req, timeout=2) as response:
        return json.load(response)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--abort', action='store_true')
    parser.add_argument('--continuous', action='store_true', default=True)
    parser.add_argument('--auto-north', action='store_true', help='Manual surface movement with a 40-degree synthetic VN reference error')
    args=parser.parse_args()
    from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo
    root=Path(__file__).resolve().parents[1]
    folder=root/'sitl/logs'/('waypoint-'+uuid.uuid4().hex[:10]);folder.mkdir(parents=True)
    env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),
             WAYPOINT_INTERACTIVE='true',GRAEY_WAYPOINT_LOG_DIR=str(folder))
    env['CONTINUOUS_FUSION'] = str(args.continuous).lower()
    if args.auto_north:
        env.update(AUTO_NORTH_ALIGNMENT='true', RELATIVE_YAW_OFFSET_DEG='40.0')
    env.pop('QGC_HOST',None)
    result=dict(passed=False,continuous=args.continuous,stages=[],gps_outage_seen=False,underwater_cube_seen=False)
    stop=threading.Event()
    def monitor():
        previous=time.monotonic()
        wall=time.time()
        gaps=[]
        adjustments=[]
        while not stop.wait(.05):
            now=time.monotonic()
            current_wall=time.time()
            if now-previous>.3: gaps.append(dict(at=now,gap=now-previous))
            difference=(current_wall-wall)-(now-previous)
            if abs(difference)>.1: adjustments.append(dict(at=now,wall_step=difference))
            previous=now
            wall=current_wall
        result['independent_clock_gaps']=gaps
        result['wall_clock_adjustments']=adjustments
    observer=threading.Thread(target=monitor,daemon=True);observer.start()
    proc=None; plan=None; run_id=None; abort_sent=False
    manual_link = None
    alignment_stopped = False
    try:
        with (folder/'runner.log').open('w') as output:
            proc=subprocess.Popen(['bash','sitl/run.sh'],cwd=root,env=env,stdout=output,
                                  stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+210
            while time.monotonic()<deadline:
                if proc.poll() is not None: raise RuntimeError('SITL exited')
                try:
                    state=api();nav=state['navigation'];streams=nav['streams']
                    s=streams.get('supervisor',{}).get('data',{});c=nav['cube_estimate']
                    if args.auto_north and not alignment_stopped and streams.get('heartbeat',{}).get('fresh'):
                        from pymavlink import mavutil
                        if manual_link is None:
                            manual_link=mavutil.mavlink_connection('udpout:127.0.0.1:14559',source_system=255,source_component=202)
                        manual_link.mav.heartbeat_send(6,8,0,0,0)
                        alignment=s.get('north_alignment') or {}
                        if not alignment.get('aligned'):
                            # Owned isolated simulator only. MANUAL surface movement
                            # establishes alignment before any external odometry.
                            manual_link.mav.set_mode_send(1,1,19)
                            if not streams['heartbeat']['data'].get('armed'):
                                manual_link.mav.command_long_send(1,1,400,0,1,0,0,0,0,0,0)
                            manual_link.mav.manual_control_send(1,800,0,500,0,0)
                            result['alignment_progress']=alignment
                            time.sleep(.2)
                            continue
                        manual_link.mav.manual_control_send(1,0,0,500,0,0)
                        manual_link.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0)
                        if streams['heartbeat']['data'].get('armed'):
                            time.sleep(.2)
                            continue
                        result['north_alignment']=alignment
                        alignment_stopped=True
                    if run_id is None and c['valid'] and s.get('configuration_verified') and s.get('surfaced'):
                        lat,lon=ne_to_geo(1.5,1.,(c['lat'],c['lon']))
                        plan=api(dict(action='preview',plan=dict(lat=lat,lon=lon,depth_m=.5,map='geographic')))
                        started=api(dict(action='start',plan_id=plan['plan_id']));run_id=started['run_id']
                        duplicate=api(dict(action='start',plan_id=plan['plan_id']))
                        result['duplicate_idempotent']=duplicate.get('duplicate') and duplicate['run_id']==run_id
                    m=streams.get('waypoint',{}).get('data',{})
                    if m.get('run_id')==run_id and run_id:
                        if not result['stages'] or result['stages'][-1]!=m['stage']: result['stages'].append(m['stage'])
                        gps=streams.get('gps',{}).get('data',{})
                        if m['stage']=='TRANSIT':
                            result['gps_outage_seen'] |= gps.get('fix',99)<3
                            result['underwater_cube_seen'] |= bool(c['valid'] and c['heading_deg'] is not None and s.get('depth_m',0)>.4)
                            if args.abort and result['gps_outage_seen'] and not abort_sent:
                                api(dict(action='abort',run_id=run_id));abort_sent=True
                        if m['stage'] in ('COMPLETE','ABORTED'):
                            if args.abort and m['stage']=='ABORTED' and streams['heartbeat']['data']['mode_id']!=19:
                                continue
                            result['mission']=m;result['supervisor']=s
                            result['final_local']=streams.get('local',{}).get('data',{}).get('ned')
                            if result['final_local'] and m.get('target_cube'):
                                import math
                                result['final_target_error_m']=math.dist(result['final_local'][:2],m['target_cube'][:2])
                            outcome=(m['stage']=='ABORTED' and abort_sent and 'Operator aborted' in m['reason']) if args.abort else (
                                m['stage']=='COMPLETE' and not streams['heartbeat']['data']['armed'])
                            result['passed']=bool(outcome and result['gps_outage_seen']
                                and result['underwater_cube_seen'] and result.get('duplicate_idempotent')
                                and (not args.abort or streams['heartbeat']['data']['mode_id']==19))
                            if args.auto_north:
                                result['passed'] &= bool(alignment_stopped and abs(
                                    result.get('north_alignment',{}).get('offset_deg',999)+40.) < 5.)
                            break
                except (OSError,KeyError,ValueError) as error:
                    result['last_retry']=str(error)
                time.sleep(.2)
            else: result['error']='Validation deadline exceeded'
    except Exception as error: result['error']=repr(error)
    finally:
        if manual_link:
            manual_link.close()
        if proc:
            try: os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError: pass
            try: proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
        stop.set();observer.join(timeout=2)
        (folder/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(evidence=str(folder),**result)),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
