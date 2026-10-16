import os,subprocess,time,json,pathlib,signal,urllib.request,uuid
import rclpy
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
root=pathlib.Path.cwd()
folder=root/'sitl/logs'/('dvl-recovery-'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),WAYPOINT_INTERACTIVE='true')
env.pop('QGC_HOST',None)
result=dict(passed=False)
def read():
    return json.load(urllib.request.urlopen('http://127.0.0.1:8090/api/calibration',timeout=2))['navigation']['streams']['bridge']['data']
proc=None;node=None
try:
    out=(folder/'runner.log').open('w')
    proc=subprocess.Popen(['bash','sitl/run.sh'],env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+100
    while time.monotonic()<deadline:
        try:
            b=read()
            if b['healthy'] and b['transmitting']:break
        except Exception:pass
        if proc.poll() is not None:raise RuntimeError('Owned SITL exited')
        time.sleep(.2)
    else:raise RuntimeError('SITL did not initialize')
    os.environ.update(ROS_DOMAIN_ID='73',ROS_LOCALHOST_ONLY='1')
    rclpy.init();node=rclpy.create_node('dvl_recovery_test_injector')
    pub=node.create_publisher(String,'/graey/dvl/sample',10)
    clock=Clock(clock_type=ClockType.STEADY_TIME)
    time.sleep(1)
    before=b['dvl_continuity']['dropouts']
    pub.publish(String(data=json.dumps(dict(stamp_ns=clock.now().nanoseconds,velocity=[0.,0.,0.],valid=False,sensor_time=None))))
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        b=read()
        if b['dvl_continuity']['dropouts']>before and b['healthy'] and b['transmitting']:break
        time.sleep(.02)
    else:raise RuntimeError('Short invalid DVL did not recover')
    result.update(short_drop_recovered=True,short_status=b)
    # Continuous invalid reports prevent two consecutive valid recovery samples.
    deadline=time.monotonic()+.8
    while time.monotonic()<deadline:
        pub.publish(String(data=json.dumps(dict(stamp_ns=clock.now().nanoseconds,velocity=[0.,0.,0.],valid=False,sensor_time=None))))
        time.sleep(.005)
    time.sleep(.3);b=read()
    assert b['fault'] and not b['healthy'] and not b['transmitting']
    result.update(passed=True,long_loss_latched=True,long_status=b)
except Exception as e:
    result['error']=str(e)
finally:
    if node is not None:node.destroy_node();rclpy.shutdown()
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
    (folder/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(evidence=str(folder),passed=result['passed'],error=result.get('error'),short_drop_recovered=result.get('short_drop_recovered'),long_loss_latched=result.get('long_loss_latched'))),flush=True)
raise SystemExit(0 if result['passed'] else 1)
