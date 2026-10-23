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
    parser.add_argument('--abort-during-hold',action='store_true',help='Abort the armed surface qualification hold through the GUI API')
    parser.add_argument('--dive', action='store_true', help='Explicit legacy underwater regression; default is surface travel')
    parser.add_argument('--gps-dropout-seconds',type=float,default=0.,help='Surface transit GPS dropout injection')
    parser.add_argument('--expect-gps-abort',action='store_true',help='Require bounded GPS outage to yield MANUAL')
    parser.add_argument('--dvl-dropout',action='store_true',help='Inject brief invalid DVL during underwater transit')
    parser.add_argument('--dvl-gap-seconds',type=float,default=.1,help='Duration of injected DVL interruption')
    parser.add_argument('--supervisor-interruption',action='store_true',help='Inject a brief false permission status while Cube/bridge stay healthy')
    parser.add_argument('--continuous', action='store_true', default=True)
    parser.add_argument('--auto-north', action='store_true', help='Mission-integrated GUIDED alignment with a 40-degree synthetic VN reference error')
    parser.add_argument('--target-north',type=float,default=1.5)
    parser.add_argument('--target-east',type=float,default=1.)
    args=parser.parse_args()
    if args.abort_during_hold:
        if args.dive:parser.error('Surface hold does not apply to a dive mission')
        args.abort=True
    from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo
    root=Path(__file__).resolve().parents[1]
    folder=root/'sitl/logs'/('waypoint-'+uuid.uuid4().hex[:10]);folder.mkdir(parents=True)
    env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),
             WAYPOINT_INTERACTIVE='true',SITL_CONTROL_FIXTURE='true',GRAEY_WAYPOINT_LOG_DIR=str(folder))
    env['CONTINUOUS_FUSION'] = str(args.continuous).lower()
    if args.auto_north:
        env.update(AUTO_NORTH_ALIGNMENT='false', RELATIVE_YAW_OFFSET_DEG='40.0')
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
    alignment_drive_started = None
    injector=None
    if args.dvl_dropout or args.supervisor_interruption:
        os.environ.update(ROS_DOMAIN_ID='73',ROS_LOCALHOST_ONLY='1')
        import rclpy
        from std_msgs.msg import String
        from rclpy.clock import Clock,ClockType
        rclpy.init();injector=rclpy.create_node('waypoint_short_dvl_dropout')
        dvl_pub=injector.create_publisher(String,'/graey/dvl/sample',10)
        status_pub=injector.create_publisher(String,'/graey/navigation/status',10)
        dvl_clock=Clock(clock_type=ClockType.STEADY_TIME)
    try:
        with (folder/'runner.log').open('w') as output:
            proc=subprocess.Popen(['bash','sitl/run.sh'],cwd=root,env=env,stdout=output,
                                  stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+210
            while time.monotonic()<deadline:
                if proc.poll() is not None: raise RuntimeError('SITL exited')
                try:
                    state=api();nav=state['navigation'];streams=nav['streams']
                    if state.get('capabilities',{}).get('simulation') is not True:
                        raise RuntimeError('Refusing GUI outside isolated SITL scope')
                    s=streams.get('supervisor',{}).get('data',{});c=nav['cube_estimate']
                    if run_id is None and (c['valid'] or c.get('local_valid')) and s.get('configuration_verified') and s.get('surfaced'):
                        fix=streams.get('gps',{}).get('data',{})
                        origin=(c['lat'],c['lon']) if c['valid'] else (fix['lat'],fix['lon'])
                        lat,lon=ne_to_geo(args.target_north,args.target_east,origin)
                        result['requested_target']=dict(lat=lat,lon=lon)
                        plan=api(dict(action='preview',plan=dict(lat=lat,lon=lon,depth_m=.5 if args.dive else 0.,mode='dive' if args.dive else 'surface',map='geographic',alignment_corridor_confirmed=True)))
                        started=api(dict(action='start',plan_id=plan['plan_id']));run_id=started['run_id']
                        duplicate=api(dict(action='start',plan_id=plan['plan_id']))
                        result['duplicate_idempotent']=duplicate.get('duplicate') and duplicate['run_id']==run_id
                    m=streams.get('waypoint',{}).get('data',{})
                    if m.get('run_id')==run_id and run_id:
                        assert abs(m['plan']['lat']-lat)<1e-9 and abs(m['plan']['lon']-lon)<1e-9,'Mission changed the selected geographic target'
                        if not result['stages'] or result['stages'][-1]!=m['stage']: result['stages'].append(m['stage'])
                        gps=streams.get('gps',{}).get('data',{})
                        if m['stage']=='CHECK_SURFACE':
                            result['armed_guided_surface_hold_seen']=bool(m.get('cube_armed') and m.get('cube_mode_id')==4)
                            if args.abort_during_hold and not abort_sent:
                                api(dict(action='abort',run_id=run_id));abort_sent=True
                        if m['stage']=='TRANSIT':
                            if not args.dive:
                                result['surface_cube_seen']=bool(c['valid'] and c['heading_deg'] is not None and abs(s.get('depth_m',999))<.15)
                            if args.gps_dropout_seconds and not result.get('gps_dropout_injected'):
                                from pymavlink import mavutil
                                if manual_link is None:
                                    manual_link=mavutil.mavlink_connection('udpout:127.0.0.1:14559',source_system=255,source_component=202)
                                manual_link.mav.heartbeat_send(6,8,0,0,0)
                                manual_link.mav.param_set_send(1,1,b'SIM_GPS_DISABLE',1.,9)
                                injected_until=time.monotonic()+args.gps_dropout_seconds
                                while time.monotonic()<injected_until:
                                    state_gap=api()['navigation']['streams']
                                    bgap=state_gap.get('bridge',{}).get('data',{})
                                    result['gps_outage_seen'] |= not (bgap.get('gps_admission') or {}).get('qualified',True)
                                    time.sleep(.1)
                                manual_link.mav.param_set_send(1,1,b'SIM_GPS_DISABLE',0.,9)
                                result['gps_dropout_injected']=True
                            if args.supervisor_interruption and not result.get('permission_interruption_injected'):
                                original=dict(s)
                                deadline_injection=time.monotonic()+.35
                                while time.monotonic()<deadline_injection:
                                    original.update(navigation_ready=False,reason='Injected short permission publication interruption',published_monotonic=time.monotonic())
                                    status_pub.publish(String(data=json.dumps(original)))
                                    time.sleep(.01)
                                    sample=api()['navigation']['streams'].get('waypoint',{}).get('data',{})
                                    result['navigation_pause_seen']=bool(result.get('navigation_pause_seen') or sample.get('navigation_pause',{}).get('active'))
                                result['permission_interruption_injected']=True
                            if args.dvl_dropout and not result.get('dvl_dropout_injected'):
                                gap_end=time.monotonic()+args.dvl_gap_seconds
                                while time.monotonic()<gap_end:
                                    dvl_pub.publish(String(data=json.dumps(dict(stamp_ns=dvl_clock.now().nanoseconds,velocity=[],valid=False,sensor_time=None))))
                                    time.sleep(.03)
                                result['dvl_dropout_injected']=True
                            result['gps_outage_seen'] |= gps.get('fix',99)<3
                            result['underwater_cube_seen'] |= bool(c['valid'] and c['heading_deg'] is not None and s.get('depth_m',0)>.4)
                            if args.abort and not args.abort_during_hold and (result['gps_outage_seen'] or not args.dive) and not abort_sent:
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
                            if args.expect_gps_abort:
                                outcome=(m['stage']=='ABORTED' and 'Surface GPS unavailable beyond' in m['reason']
                                         and streams['heartbeat']['data']['mode_id']==19)
                            surface_seen=(result.get('armed_guided_surface_hold_seen') if args.abort_during_hold else result.get('surface_cube_seen'))
                            result['passed']=bool(outcome and (result['gps_outage_seen'] and result['underwater_cube_seen'] if args.dive else surface_seen and not any(x in result['stages'] for x in ('DIVE','SURFACE')))
                                and (not args.gps_dropout_seconds or result.get('gps_dropout_injected') and result['gps_outage_seen']) and result.get('duplicate_idempotent')
                                and (not args.abort or streams['heartbeat']['data']['mode_id']==19))
                            if args.auto_north:
                                result['north_alignment']=streams.get('bridge',{}).get('data',{}).get('north_alignment',{})
                                result['passed'] &= bool('ALIGN_MOVE' in result['stages'] and abs(
                                    (result['north_alignment'].get('offset_deg') or 999)+40.) < 5.)
                            if args.dvl_dropout:
                                b=streams.get('bridge',{}).get('data',{})
                                result['dvl_continuity']=b.get('dvl_continuity')
                                result['passed'] &= bool(result.get('dvl_dropout_injected') and not b.get('fault') and (b.get('dvl_continuity') or {}).get('dropouts',0)>0)
                            if args.supervisor_interruption:
                                result['passed'] &= bool(result.get('permission_interruption_injected') and result.get('navigation_pause_seen'))
                            break
                except (OSError,KeyError,ValueError) as error:
                    result['last_retry']=str(error)
                time.sleep(.2)
            else: result['error']='Validation deadline exceeded'
    except Exception as error: result['error']=repr(error)
    finally:
        if injector:
            injector.destroy_node();rclpy.shutdown()
        if manual_link:
            manual_link.mav.manual_control_send(1,0,0,500,0,0)
            manual_link.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0)
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
