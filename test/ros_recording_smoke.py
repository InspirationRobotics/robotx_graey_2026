"""Real sensor rosbag recording/finalization on isolated synthetic ROS traffic."""
import json
import os
import sqlite3
import sys
import tempfile
import time
from pathlib import Path

os.environ.update(ROS_DOMAIN_ID='123',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='recording-smoke')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    import rclpy
    from std_msgs.msg import String
    from robotx_graey_2026.api.gui.recording_service import RecordingService
    rclpy.init();node=rclpy.create_node('synthetic_recording_fixture')
    publisher=node.create_publisher(String,'/graey/test_sensor',10)
    with tempfile.TemporaryDirectory() as folder:
        recorder=RecordingService(folder,persistence=lambda _:('/fixture','synthetic',True))
        try:
            recorder.update_sources([('/graey/test_sensor',['std_msgs/msg/String'])],dict(synthetic=True))
            recorder.submit(dict(action='start',name='ros_smoke',sensors=True,camera=False,sonar=False,topics=['/graey/test_sensor']))
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                publisher.publish(String(data='synthetic sensor sample'))
                rclpy.spin_once(node,timeout_sec=.05)
            assert recorder.bag.recording,recorder.snapshot()
            recorder.submit(dict(action='stop'))
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and (recorder.snapshot()['busy'] or recorder.bag.recording):
                rclpy.spin_once(node,timeout_sec=.05)
            assert not recorder.bag.recording,recorder.snapshot()
            metadata=list(Path(folder).glob('*/bag/metadata.yaml'));assert metadata,recorder.snapshot()
            databases=list(Path(folder).glob('*/bag/*.db3'));assert databases,recorder.snapshot()
            with sqlite3.connect(databases[0]) as db:count=db.execute('SELECT count(*) FROM messages').fetchone()[0]
            assert count>0,count
            print(json.dumps(dict(result='PASS',ros_domain=123,messages_recorded=count,
                                  metadata_finalized=True,camera_opened=False,sonar_opened=False,
                                  hardware_opened=False),indent=2))
        finally:recorder.close()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
