import os,subprocess,time,json,pathlib,signal,urllib.request,uuid,argparse
import rclpy
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger
parser=argparse.ArgumentParser()
parser.add_argument('--explicit-recovery',action='store_true')
args=parser.parse_args()
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
        if b['dvl_continuity']['dropouts']>before and b['healthy'] and b['transmitting'] and not b['dvl_continuity']['recovering']:break
        time.sleep(.02)
    else:raise RuntimeError('Short invalid DVL did not recover')
    result.update(short_drop_recovered=True,short_status=b)
    # Exceed the prediction horizon without exhausting position confidence.
    deadline=time.monotonic()+1.3
    while time.monotonic()<deadline:
        pub.publish(String(data=json.dumps(dict(stamp_ns=clock.now().nanoseconds,velocity=[0.,0.,0.],valid=False,sensor_time=None))))
        time.sleep(.005)
    b=read()
    assert not b['fault'] and not b['healthy'] and not b['transmitting'], b
    result['prediction_withheld_without_latch']=True
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        b=read()
        if b['healthy'] and b['transmitting'] and not b['dvl_continuity']['recovering']:break
        time.sleep(.02)
    else:raise RuntimeError('Extended DVL outage failed automatic reacquisition')
    result['extended_loss_recovered']=True
    # A genuinely unobserved displacement must still stop navigation.
    deadline=time.monotonic()+4.5
    while time.monotonic()<deadline:
        pub.publish(String(data=json.dumps(dict(stamp_ns=clock.now().nanoseconds,velocity=[0.,0.,0.],valid=False,sensor_time=None))))
        time.sleep(.005)
    time.sleep(.3);b=read()
    assert b['fault'] and not b['healthy'] and not b['transmitting']
    result.update(passed=True,long_loss_latched=True,long_status=b)
    if args.explicit_recovery:
        # Isolated simulator fixture only: no physical kill node is launched.
        # Publish a maintenance lease only after verifying the simulator is
        # disarmed. This does not claim to test hardware kill enforcement.
        state=json.load(urllib.request.urlopen('http://127.0.0.1:8090/api/waypoint',timeout=2))
        assert state['capabilities']['simulation'] is True
        assert state['navigation']['streams']['heartbeat']['data']['armed'] is False
        safe=node.create_publisher(String,'/graey/control_watchdog/status',10)
        def lease():
            safe.publish(String(data=json.dumps(dict(session='isolated-recovery-test',revision=1,
                vehicle_fresh=True,armed=False,sa_active=True,esc_commanded_on=False,
                mission_claimed=False,autonomous=False))))
        node.create_timer(.1,lease)
        end=time.monotonic()+2.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        client=node.create_client(Trigger,'/graey/navigation/recover_dvl')
        assert client.wait_for_service(timeout_sec=3.)
        prior_reset=b['reset_counter'];prior_north=b['north_alignment'];prior_bias=b['heading_bias_deg']
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        assert future.done() and future.result().success, str(future.result())
        result['recovery_ack']=future.result().message
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05);b=read()
            if b['healthy'] and b['transmitting']:break
        else:raise RuntimeError('Explicit recovery did not reacquire GPS and resume odometry')
        assert b['reset_counter']==(prior_reset+1)%256
        assert b['north_alignment']==prior_north and b['heading_bias_deg']==prior_bias
        assert b['gps_admission']['qualified']
        result.update(explicit_recovery_passed=True,recovered_status=b)
except Exception as e:
    result['passed']=False
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
