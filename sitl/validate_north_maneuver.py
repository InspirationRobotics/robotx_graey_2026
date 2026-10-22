"""GUI Prepare -> scripted surface move -> stationary check -> commission.

Isolated SITL ONLY. Publishes a synthetic maintenance/control lease based on
the simulator's observed armed state; does not test hardware kill/joystick
arbitration. No QGC output, physical connection, or direct vehicle endpoint.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.request
import urllib.error
import uuid


def api(body=None):
    req=urllib.request.Request('http://127.0.0.1:8090/api/calibration',
        data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=3) as f:return json.load(f)
    except urllib.error.HTTPError as error:
        raise RuntimeError(error.read().decode()) from error


def main():
    root=Path.cwd();folder=root/'sitl/logs'/('north-maneuver-'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
    env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),
        WAYPOINT_INTERACTIVE='true',AUTO_NORTH_ALIGNMENT='true',RELATIVE_YAW_OFFSET_DEG='40.0',
        GRAEY_WAYPOINT_LOG_DIR=str(folder))
    env.pop('QGC_HOST',None)
    result=dict(passed=False,hardware_interlocks_tested=False,qgc_forwarding=False)
    proc=node=None;stop=threading.Event();worker=None
    try:
        with (folder/'runner.log').open('w') as log:
            proc=subprocess.Popen(['bash','sitl/run.sh'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        os.environ.update(ROS_DOMAIN_ID='73',ROS_LOCALHOST_ONLY='1')
        import rclpy
        from rclpy.clock import Clock, ClockType
        from std_msgs.msg import String,Bool
        from robotx_graey_2026.api.pixhawk.mavlink import Link
        rclpy.init();node=rclpy.create_node('isolated_north_maneuver_fixture')
        pub=node.create_publisher(String,'/graey/control_watchdog/status',10)
        vn=node.create_publisher(String,'/graey/vn100/calibration',10)
        actual=dict(armed=False,mode='MANUAL',known=False,claim=False)
        def claim(msg):actual['claim']=msg.data
        node.create_subscription(Bool,'/graey/autonomy_active',claim,10)
        def publish():
            if not actual['known']:return
            pub.publish(String(data=json.dumps(dict(session='isolated-north-test',revision=1,
                vehicle_fresh=True,armed=actual['armed'],mode=actual['mode'],
                sa_active=not actual['armed'],esc_commanded_on=actual['armed'],
                mission_claimed=actual['claim'],autonomous=False,inhibit=''))))
            vn.publish(String(data=json.dumps(dict(phase='idle',reason='Isolated synthetic VN'))))
        # Match production scheduling: host UTC adjustments must not stall the
        # synthetic control watchdog and create a false takeover in this test.
        steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(.1,publish,clock=steady_clock)
        def spin():
            while not stop.is_set():rclpy.spin_once(node,timeout_sec=.05)
        worker=threading.Thread(target=spin);worker.start()
        link=None;prepared=started=False;seen=[];deadline=time.monotonic()+240
        last=None
        while time.monotonic()<deadline:
            if proc.poll() is not None:raise RuntimeError('Owned SITL exited')
            try:state=api()
            except Exception:time.sleep(.2);continue
            with urllib.request.urlopen('http://127.0.0.1:8090/api/waypoint',timeout=3) as response:
                if json.load(response).get('capabilities',{}).get('simulation') is not True:
                    raise RuntimeError('Refusing to operate a GUI outside isolated SITL scope')
            streams=state['navigation']['streams'];b=streams.get('bridge',{}).get('data',{})
            hb=streams.get('heartbeat',{})
            if hb.get('fresh'):
                h=hb['data'];actual.update(known=True,armed=h['armed'],mode={19:'MANUAL',0:'STABILIZE',4:'GUIDED'}.get(h['mode_id'],'OTHER'))
            else:actual['known']=False
            job=state['job'];last=state
            if job.get('phase')=='failed':raise RuntimeError(job['message'])
            if not prepared:
                if actual['known'] and not actual['armed'] and b.get('configuration_verified') and b.get('depth_m') is not None:
                    # Allow pressure/VN fixture history and DDS discovery.
                    time.sleep(2.)
                    api(dict(action='prepare',confirmed=True,surface_confirmed=True,alignment_mode='gps'))
                    prepared=True
            elif not started and b.get('gps_admission',{}).get('qualified'):
                if link is None:link=Link('udpout:127.0.0.1:14559',202,system=255)
                link.heartbeat();link.set_mode(19)
                if not actual['armed']:link.command(400,1)
                elif (state['controller'].get('armed') is True
                      and state['controller'].get('sa_active') is False
                      and state['controller'].get('esc_commanded_on') is True):
                    api(dict(action='align_auto',clearance_confirmed=True,max_distance_m=10.))
                    started=True
            motion=streams.get('north_maneuver',{}).get('data',{})
            stage=motion.get('stage')
            if stage and (not seen or seen[-1]!=stage):seen.append(stage)
            if stage=='ABORTED':raise RuntimeError(motion['reason'])
            if stage=='COMPLETE':
                result['maneuver']=motion
                link.command(400,0)
            if job.get('phase')=='ready':
                result.update(passed=True,stages=seen,north=b.get('north_alignment'),
                    heading_bias_deg=b.get('heading_bias_deg'),bridge_healthy=b.get('healthy'),
                    cube=state['navigation'].get('cube_estimate'),job=job)
                break
            time.sleep(.1)
        else:raise RuntimeError('Automatic alignment/commission deadline expired')
    except Exception as error:
        result.update(error=str(error),last_state=locals().get('last'),stages=locals().get('seen'))
    finally:
        stop.set()
        if worker:worker.join(timeout=2)
        if node:node.destroy_node();rclpy.shutdown()
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(dict(passed=result['passed'],evidence=str(folder),error=result.get('error'),stages=result.get('stages'))),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
