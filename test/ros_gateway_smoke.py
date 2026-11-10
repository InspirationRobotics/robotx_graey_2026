"""Real ROS + UDP fixture tests the migrated Link API and gateway admission.
No serial device, vehicle address, or production ROS domain is used.
"""
import os
import sys
import json
import socket
import threading
import time
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
os.environ.update(ROS_DOMAIN_ID='121',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='gateway-smoke')
from robotx_graey_2026.api.pixhawk.mavlink import Link,mavutil
from robotx_graey_2026.api.pixhawk import telemetry_gateway
import robotx_graey_2026.api.node_util as node_util
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def main():
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',0));udp.settimeout(.02)
    port=udp.getsockname()[1];done=threading.Event();heard=[]
    parser=mavutil.mavlink.MAVLink(None);encoder=mavutil.mavlink.MAVLink(None,srcSystem=1,srcComponent=1)
    def fcu():
        peer=None
        while not done.is_set():
            try:
                raw,peer=udp.recvfrom(4096)
                for msg in parser.parse_buffer(raw) or []:heard.append(msg)
            except socket.timeout:pass
            if peer:
                hb=encoder.heartbeat_encode(12,3,129,4,4,3)
                udp.sendto(bytes(hb.pack(encoder)),peer)
    worker=threading.Thread(target=fcu,daemon=True);worker.start()
    def exercise(cls,**kwargs):
        rclpy.init(args=['--ros-args','-p',f'mavlink:=udpout:127.0.0.1:{port}'])
        gateway=cls();fixture=Node('gateway_test_clients')
        descriptors=gateway.describe_parameters(['mavlink','stream_ids','stream_hz'])
        assert [d.name for d in descriptors]==['mavlink','stream_ids','stream_hz']
        assert all(d.read_only for d in descriptors)
        mission=Link('unused',198,system=255,node=fixture)
        gui=Link('unused',195,node=fixture)
        safety=Link('unused',194,node=fixture)
        nav=Link('unused',197,node=fixture)
        pub=fixture.create_publisher(String,'/graey/control_watchdog/status',10)
        executor=SingleThreadedExecutor();executor.add_node(gateway);executor.add_node(fixture)
        received=[]
        def spin(duration,permission=True):
            until=time.monotonic()+duration
            last_publish=-1.
            while time.monotonic()<until:
                if permission and time.monotonic()-last_publish>=.05:
                    last_publish=time.monotonic()
                    pub.publish(String(data=json.dumps(dict(vehicle_fresh=True,sa_active=False,
                        esc_commanded_on=True,inhibit=False,armed=True,published_monotonic=time.monotonic()))))
                executor.spin_once(timeout_sec=.005)
                mission.drain(lambda kind,msg:received.append((kind,msg.get_srcSystem())))
        try:
            spin(1.)
            assert ('HEARTBEAT',1) in received,received
            assert mission.mav.flightmode=='GUIDED',mission.mav.flightmode
            mission.goto_ned(1.,2.,.3,0.)
            nav.odometry((0,0,0),(1,0,0,0),(0,0,0),(0,0,0))
            spin(.25)
            assert any(m.get_msgId()==84 and m.get_srcComponent()==198 for m in heard)
            assert any(m.get_msgId()==331 and m.get_srcComponent()==197 for m in heard)
            before=len([m for m in heard if m.get_msgId()==84])
            gui.goto_ned(9.,9.,9.,0.);spin(.1)
            assert len([m for m in heard if m.get_msgId()==84])==before
            spin(.9,False)
            mission.goto_ned(9.,9.,9.,0.)
            safety.command(400,0)
            spin(.2,False)
            assert len([m for m in heard if m.get_msgId()==84])==before
            assert any(m.get_msgId()==76 and m.command==400 and m.param1==0 for m in heard)
            assert gateway.refused>=2
            print(json.dumps(dict(result='PASS',rx_frames=gateway.received,tx_frames=gateway.transmitted,
                refused=gateway.refused,flightmode=mission.mav.flightmode,odometry_preserved=True,
                reader_cannot_drive=True,stale_motion_blocked=True,emergency_disarm_preserved=True,
                hardware_opened=False),indent=2))
        finally:
            for link in (mission,gui,safety,nav):link.mav.close()
            executor.remove_node(gateway);executor.remove_node(fixture)
            gateway.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.shutdown()
    try:
        with patch.object(node_util,'run',exercise):telemetry_gateway.main()
    finally:done.set();worker.join(2);udp.close()


if __name__=='__main__':main()
