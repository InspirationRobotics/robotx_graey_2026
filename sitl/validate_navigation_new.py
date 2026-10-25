#!/usr/bin/env python3
"""Actual production GUI/ROS + ArduSub, localhost only; never forwards to QGC."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request
import uuid


def api(body=None):
    req=urllib.request.Request('http://127.0.0.1:8090/api/waypoint',
        data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=2) as response:return json.load(response)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--total-loss',action='store_true');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    folder=root/'sitl/logs'/('navigation-new-'+uuid.uuid4().hex[:10]);folder.mkdir(parents=True)
    env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),
        WAYPOINT_INTERACTIVE='true',SITL_CONTROL_FIXTURE='true',GRAEY_WAYPOINT_LOG_DIR=str(folder),
        ROS_DOMAIN_ID='73',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='sitl',RELATIVE_YAW_OFFSET_DEG='40.0')
    env.pop('QGC_HOST',None)
    os.environ.update(ROS_DOMAIN_ID='73',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='sitl')
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('navigation_new_test')
    faults=node.create_publisher(String,'/graey/sitl/faults',10)
    result=dict(passed=False,stages=[],faults=[],qgc_forwarding=False)
    proc=None;run=None;tracking_at=None;gps_off_at=None;gps_restored=False;total_started=False
    from pymavlink import mavutil
    simlink=mavutil.mavlink_connection('udpout:127.0.0.1:14559',source_system=255,source_component=202)
    try:
        with (folder/'runner.log').open('w') as log:
            proc=subprocess.Popen(['bash','sitl/run.sh'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+480
            with (folder/'samples.jsonl').open('w') as samples:
                while time.monotonic()<deadline:
                    if proc.poll() is not None:raise RuntimeError('SITL exited')
                    try:
                        state=api();streams=state['navigation']['streams']
                        b=streams.get('bridge',{}).get('data',{})
                        m=streams.get('waypoint',{}).get('data',{})
                        samples.write(json.dumps(dict(t=time.monotonic(),bridge=b,mission=m,cube=state['navigation'].get('cube_estimate')))+'\n');samples.flush()
                        simlink.mav.heartbeat_send(6,8,0,0,0)
                        if run is None and b.get('local_navigation_ready') and b.get('configuration_verified') and (b.get('gps_admission') or {}).get('qualified'):
                            fix=b['gps_observation']
                            # Target beyond the 10m initial corridor, with a lateral leg.
                            from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo
                            lat,lon=ne_to_geo(12.,2.,(fix['lat'],fix['lon']))
                            plan=dict(lat=lat,lon=lon,mode='surface',map='geographic',alignment_corridor_confirmed=True)
                            preview=api(dict(action='preview',plan=plan))
                            run=api(dict(action='start',plan_id=preview['plan_id']))['run_id']
                            result['run_id']=run
                        if m.get('run_id')==run and run:
                            if not result['stages'] or result['stages'][-1]!=m['stage']:
                                result['stages'].append(m['stage']);print(m['stage'],m.get('reason'),flush=True)
                            if (b.get('map_transform') or {}).get('ready') and tracking_at is None:tracking_at=time.monotonic()
                            if tracking_at and m['stage']=='TRANSIT':
                                elapsed=time.monotonic()-tracking_at
                                if args.total_loss:
                                    if elapsed>2 and gps_off_at is None:
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS1_ENABLE',0.,9)
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS_DISABLE',1.,9)
                                        gps_off_at=time.monotonic()
                                    if gps_off_at and not (b.get('estimator_health') or {}).get('sensors',{}).get('gps'):
                                        result['gps_loss_observed']=True
                                    if gps_off_at and not gps_restored and time.monotonic()-gps_off_at>5:
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS1_ENABLE',1.,9)
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS_DISABLE',0.,9)
                                        gps_restored=True
                                    if gps_restored and elapsed>14 and not total_started:
                                        faults.publish(String(data=json.dumps(dict(vn=20.,dvl=20.))))
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS1_ENABLE',0.,9)
                                        simlink.mav.param_set_send(1,1,b'SIM_GPS_DISABLE',1.,9)
                                        total_started=True
                                else:
                                    for name,start in [('dvl',2),('vn',12)]:
                                        if elapsed>start and name not in result['faults']:
                                            faults.publish(String(data=json.dumps({name:5.})))
                                            result['faults'].append(name)
                            if m['stage'] in ('COMPLETE','ABORTED'):
                                result['mission']=m;result['health']=b.get('estimator_health');result['north']=b.get('north_alignment')
                                result['passed']=(m['stage']=='ABORTED' and 'lost GPS, DVL and VN' in m['reason'] and result.get('gps_loss_observed') and total_started) if args.total_loss else (m['stage']=='COMPLETE' and set(result['faults'])=={'vn','dvl'})
                                break
                    except (OSError,KeyError,ValueError) as error:result['last_retry']=str(error)
                    time.sleep(.2)
                else:result['error']='480 second deadline exceeded'
    except Exception as error:result['error']=repr(error)
    finally:
        if proc:
            try:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=15)
            except ProcessLookupError:pass
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        simlink.close()
        node.destroy_node();rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        print(str(folder),json.dumps(result),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
