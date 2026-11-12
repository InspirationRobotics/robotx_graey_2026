"""Bounded ROS Humble wiring smoke test, isolated domain and synthetic telemetry.

Uses the real node, shared ROS FCU transport, SQLite and a local TCP fixture.
FCU frames are synthetic; no Cube, MAVProxy, RFD or physical interface is opened.
Run from a sourced ROS Humble shell with the repository on PYTHONPATH.
"""
import json
import os
import socket
import sys
import struct
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

os.environ['ROS_DOMAIN_ID'] = '120'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['GRAEY_RUNTIME_SCOPE'] = 'competition-wifi-smoke-'+str(os.getpid())

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from robotx_graey_2026.api.comms import competition_link_node as target
from robotx_graey_2026.api.comms.competition_wifi import WifiLink
from robotx_graey_2026.api.pixhawk.mavlink import mavutil


def main():
    listener = socket.socket()
    listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(.2)
    port = listener.getsockname()[1]
    done = threading.Event()
    reports = []
    errors = []

    def server():
        conn = None
        try:
            while not done.is_set():
                try:conn,_=listener.accept();break
                except socket.timeout:pass
            if conn is None:return
            conn.settimeout(.2)
            buffer = bytearray()
            while not done.is_set():
                try:chunk=conn.recv(65536)
                except socket.timeout:continue
                if not chunk:break
                buffer.extend(chunk)
                while len(buffer)>=4:
                    n=struct.unpack_from('!I',buffer)[0]
                    assert 0<n<=1<<20
                    if len(buffer)<4+n:break
                    obj=json.loads(bytes(buffer[4:4+n]));del buffer[:4+n]
                    if 'ocs_hello' in obj or 'graey_sync' in obj:
                        conn.sendall(WifiLink.frame({'ocs_run':{'run_id':70000,'state':'RUNNING','graey_report_v1':True}}))
                    elif 'graey_report' in obj:
                        report=obj['graey_report']['report'];reports.append(report)
                        kind=next(k for k in ('pipeline_survey','resource_delivery') if k in report)
                        conn.sendall(WifiLink.frame({'ocs_receipt':dict(vehicle_seq=report['seq'],kind=kind,verdict='acknowledged')}))
                    elif 'heartbeat' in obj:reports.append(obj)
        except Exception as exc:errors.append(str(exc))
        finally:
            if conn is not None:conn.close()

    worker=threading.Thread(target=server,daemon=True);worker.start()
    observed=[]

    def exercise(node):
        fixture=Node('competition_smoke_inputs')
        fcu=fixture.create_publisher(String,'/graey/fcu/rx',200)
        encoder=mavutil.mavlink.MAVLink(None,srcSystem=3,srcComponent=1)
        def telemetry():
            stamp=int(time.monotonic()*1e6);boot=(stamp//1000)%0xFFFFFFFF
            messages=[encoder.ekf_status_report_encode(24,.1,.1,.1,.1,.1),
                encoder.local_position_ned_encode(boot,1.,2.,.2,.3,.4,0.),
                encoder.attitude_encode(boot,0.,0.,1.57079632679,0.,0.,0.),
                encoder.heartbeat_encode(12,3,129,4,4,3),
                encoder.global_position_int_encode(boot,12806000,1038557000,10000,0,30,40,0,9000),
                encoder.gps_raw_int_encode(stamp,3,12806000,1038557000,10000,100,100,50,9000,15,
                    5900,500,1000,0,0,0)]
            for message in messages:
                fcu.publish(String(data=json.dumps(dict(received_monotonic=time.monotonic(),frame=bytes(message.pack(encoder)).hex()))))
                encoder.seq=(encoder.seq+1)%256
        topics=('navigation/bridge_status','control_watchdog/status','comms/current_task',
                'comms/pipeline_survey','comms/resource_request')
        pubs={name:fixture.create_publisher(String,'/graey/'+name,10) for name in topics}
        fixture.create_subscription(String,'/graey/comms/status',lambda m:observed.append(json.loads(m.data)),10)
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(fixture)
        deadline=time.monotonic()+10
        recorded=False
        last_publish=-1.
        try:
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now-last_publish<.05:
                    executor.spin_once(timeout_sec=.005)
                    continue
                last_publish=now
                telemetry()
                def send(name,data):pubs[name].publish(String(data=data if isinstance(data,str) else json.dumps(data)))
                send('navigation/bridge_status',dict(published_monotonic=now,surfaced=True,navigation_valid=True,
                     coordinate_frame='cube_ned',depth_m=.2))
                send('control_watchdog/status',dict(vehicle_fresh=True,sa_active=False,esc_commanded_on=True,
                                                   armed=True,autonomous=True))
                send('comms/current_task','TASK_INFRA_SURVEY_REPAIR')
                if node.sender.run==70000 and not recorded and all(p.get_subscription_count() for p in pubs.values()):
                    send('comms/pipeline_survey',dict(observation_id='ros-survey',active_buoy_position=dict(latitude=1.2806,longitude=103.8557),segments=[1,2,1]))
                    send('comms/resource_request',dict(observation_id='ros-resource',resource_color=1,delivery_circle_color=3))
                    recorded=True
                executor.spin_once(timeout_sec=.005)
                if (observed and observed[-1]['broker_acknowledged']==2
                        and len([r for r in reports if 'heartbeat' in r])>=2):break
            assert not errors,errors
            assert recorded and observed and observed[-1]['broker_acknowledged']==2,observed[-1:] or errors
            heartbeats=[r['heartbeat'] for r in reports if 'heartbeat' in r]
            assert len(heartbeats)>=2 and all(h['depth_m']==.2 for h in heartbeats),heartbeats
            assert all(h['current_task']=='TASK_INFRA_SURVEY_REPAIR' for h in heartbeats),heartbeats
            print(json.dumps(dict(result='PASS',ros_domain=120,transport='local TCP fixture',
                                  heartbeat_count=len(heartbeats),observations_acknowledged=2,
                                  measured_depth_m=.2,hardware_opened=False),indent=2))
        finally:
            executor.remove_node(node);executor.remove_node(fixture);fixture.destroy_node();executor.shutdown()

    try:
        with tempfile.TemporaryDirectory() as folder,patch.object(rclpy,'spin',exercise):
            target.main(['--ros-args','-p','ocs_host:=127.0.0.1','-p',f'ocs_port:={port}',
                         '-p','outbox_path:='+str(Path(folder)/'outbox.sqlite3')])
    finally:
        done.set();worker.join(2);listener.close()


if __name__=='__main__':main()
