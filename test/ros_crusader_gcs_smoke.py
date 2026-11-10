"""Copied Crusader /req -> TCP -> actual GRAEY ROS parameter services."""
import http.client
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch
os.environ.update(ROS_DOMAIN_ID='126',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='crusader-gcs-smoke')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from http.server import ThreadingHTTPServer
    from robotx_graey_2026.api.navigation import dvl_node
    from robotx_graey_2026.api.gui.gcs_transport import BridgeServer
    from robotx_graey_2026.api.gui.laptop import graey_app
    with tempfile.TemporaryDirectory() as folder:
        os.environ['GRAEY_MISSION_DIR']=folder+'/missions';os.environ['GRAEY_RECORD_DIR']=folder+'/recordings'
        from robotx_graey_2026.api.gui.gui_node import GuiNode
        rclpy.init(args=['--ros-args','-p','port:=0','-p','web_dir:='+str(Path(__file__).parents[1]/'tools')])
        gui=GuiNode()
        with patch.object(dvl_node.DVLNode,'spin_once',lambda self:None):dvl=dvl_node.DVLNode()
        fixture=rclpy.create_node('runtime_control_fixture');pub=fixture.create_publisher(String,'/graey/control_watchdog/status',10)
        state=dict(session='synthetic',revision=1,vehicle_fresh=True,armed=False,mission_claimed=False,esc_commanded_on=False,sa_active=True,mode='MANUAL')
        fixture.create_timer(.1,lambda:pub.publish(String(data=json.dumps(dict(state,published_monotonic=time.monotonic())))))
        executor=SingleThreadedExecutor()
        for node in (gui,dvl,fixture):executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        bridge=BridgeServer(('127.0.0.1',0),gui.http_server.server_port);bridge_thread=threading.Thread(target=bridge.serve_forever,daemon=True);bridge_thread.start()
        common=graey_app.common;hub=common.Hub();link=common.BoatLink('127.0.0.1',bridge.server_address[1],hub.frame)
        laptop=ThreadingHTTPServer(('127.0.0.1',0),http.server.BaseHTTPRequestHandler)
        laptop.RequestHandlerClass=graey_app.make_handler(link,hub,{f'127.0.0.1:{laptop.server_port}',f'localhost:{laptop.server_port}'});laptop.daemon_threads=True
        laptop_thread=threading.Thread(target=laptop.serve_forever,daemon=True);laptop_thread.start()
        conn=http.client.HTTPConnection('127.0.0.1',laptop.server_port,timeout=15)
        def request(value):
            conn.request('POST','/req',json.dumps(value),{'Content-Type':'application/json'});response=conn.getresponse();return json.loads(response.read())
        try:
            deadline=time.monotonic()+8
            while time.monotonic()<deadline and link._sock is None:time.sleep(.1)
            time.sleep(1.5)
            result=request(dict(op='params_list',node='/dvl_node'));assert result['ok'],result
            rows={row['name']:row for row in result['data']['params']}
            assert rows['scale']['editable'] and rows['ip']['read_only'],rows
            result=request(dict(op='params_set',node='/dvl_node',values=dict(scale=1.25)))
            assert result['ok'] and dvl.scale==1.25,result
            state['armed']=True;time.sleep(.4)
            result=request(dict(op='params_set',node='/dvl_node',values=dict(scale=1.3)))
            assert not result['ok'] and dvl.scale==1.25,result
            print(json.dumps(dict(result='PASS',ros_domain=126,copied_crusader_http=True,copied_crusader_tcp=True,
                                  actual_ros_parameter_service=True,live_scale=1.25,armed_change_rejected=True,hardware_opened=False),indent=2))
        finally:
            conn.close()
            if link._sock:link._sock.close()
            laptop.shutdown();laptop.server_close();bridge.shutdown();bridge.server_close()
            executor.shutdown();thread.join(3)
            for node in (fixture,dvl,gui):node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':main()
