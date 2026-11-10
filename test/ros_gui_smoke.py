"""Actual GUI node and HTTP handlers on isolated ROS; no physical FCU sources."""
import http.client
import json
import os
import sys
import tempfile
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='124',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='gui-smoke')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    import rclpy
    with tempfile.TemporaryDirectory() as folder:
        os.environ['GRAEY_MISSION_DIR']=str(Path(folder)/'missions')
        os.environ['GRAEY_RECORD_DIR']=str(Path(folder)/'recordings')
        from robotx_graey_2026.api.gui.gui_node import GuiNode
        web=Path(__file__).resolve().parents[1]/'tools'
        rclpy.init(args=['--ros-args','-p','port:=0','-p','web_dir:='+str(web)])
        node=GuiNode()
        try:
            for _ in range(15):rclpy.spin_once(node,timeout_sec=.1)
            conn=http.client.HTTPConnection('127.0.0.1',node.http_server.server_port,timeout=5)
            checked=[]
            for path,kind in [('/missions','text/html'),('/missions.js','application/javascript'),('/sensors','text/html'),('/record','text/html'),('/api/recording','application/json'),('/api/missions','application/json')]:
                conn.request('GET',path);response=conn.getresponse();body=response.read()
                assert response.status==200,(path,response.status,body)
                assert response.getheader('Content-Type').startswith(kind),(path,response.getheaders())
                checked.append(path)
            body=json.dumps(dict(action='save',name='ros_gui_test',mission=dict(kind='waypoints',competition=False,points=[dict(kind='waypoint',lat=1.28,lon=103.85)])))
            headers={'Content-Type':'application/json','Origin':f'http://127.0.0.1:{node.http_server.server_port}'}
            conn.request('POST','/api/missions',body,headers);response=conn.getresponse();data=json.loads(response.read());assert response.status==200,data
            conn.request('GET','/api/missions?name=ros_gui_test');response=conn.getresponse();data=json.loads(response.read());assert data['mission']['kind']=='waypoints',data
            conn.close()
            print(json.dumps(dict(result='PASS',ros_domain=124,actual_gui_node=True,checked=checked,
                                  mission_library_round_trip=True,hardware_opened=False),indent=2))
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
